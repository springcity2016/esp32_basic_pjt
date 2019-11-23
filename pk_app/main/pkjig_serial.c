
#define __PK_SERIAL_C_


#include "pk_serial.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"


#include "driver/uart.h"
#include "driver/gpio.h"

#include "petkit_clog.h"
#include "TaskType.h"
#include "serial_comm_table.h"
#include "user_lib.h"
#include "pk_T3_driver.h"
#include "product_test.h"
#include "user_config.h"
#include "pkjig_serial.h"
#include "pkjig_comm_table.h"

#define CRTDBG_MAP_ALLOC           // 检查内存泄漏
#include <stdlib.h>
// #include "crtdbg.h"

jig_params_t *gst_jig_params = NULL;

#define TASK_COMM_TIME                 (10/portTICK_RATE_MS)        // 串口通信任务延时周期时间

#define MAXNUM_ONETIME (128)

#define COMMEVENT_QUEUE_MAX     (15)
static xQueueHandle xQueueCommEvent;                               // 接受到数据驱动
#define COMMSEND_QUEUE_MAX      (15)
static xQueueHandle xQueueCommSend;                                // 用于任务之间Send的数据通信
#define SERIAL_SENDTASK_MAX     (5)                                // 单次发送最大队列数据 5组


#define SERIALA_RECVBUFF_SIZE		(500)
static unsigned char mau08SerialARecvBuff[SERIALA_RECVBUFF_SIZE] = {0};  // 接收队列
static unsigned short mu16SerialARecvBuffFront = 0;                      // 接收队列前指针
static unsigned short mu16SerialARecvBuffBack = 0;                       // 接收队列后指针
static void PushRecvSerialBuff(unsigned char u08Value);
static unsigned char PopRecvSerialBuff(void);
static unsigned short GetRecvSerialBuffLength(void);

static void Serial_RecvData_Func(void);      // 从UART Fifo中读取数据
static short GetSerialActiveData(void);     // 返回一组有效的周期数据
static short SerialSendDatas(void);      // 通过Serial发送数据

static short AnalyzeRecvData(const unsigned char *pu08Data);        // 分析解析的数据

static unsigned char ga_uart_msg_seq[E_JIG_COMM_MAX_CMD] = {0};
static unsigned char set_UART_MSG_seq(int seq_index, unsigned char value);
static unsigned char get_UART_MSG_seq(int seq_index);

// static void _pk_driver_sec_cb(void *arg)

#define SERIALB_TXD_GPIO            (25)            // 串口数据输出 TXD
#define SERIALB_RXD_GPIO            (33)            // 串口数据输入 RXD
static int _pkjig_serialport_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, SERIALB_TXD_GPIO, SERIALB_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);     // 临时取消

    return 0;
}

int pkjig_serial_init(void)
{
    _pkjig_serialport_init();

    xQueueCommEvent = xQueueCreate(COMMEVENT_QUEUE_MAX, sizeof(CommFunc_t));
    if (xQueueCommEvent == NULL) {
       pkLOG_ERR("TaskComm xQueueCommEvent is NULL!\n"); 
    }
    xQueueCommSend = xQueueCreate(COMMSEND_QUEUE_MAX, sizeof(Datas_t));
    if (xQueueCommSend == NULL) {
        pkLOG_ERR("TaskComm xQueueCommSend is NULL!\n");
    }

    gst_jig_params = (jig_params_t *)malloc(sizeof(jig_params_t));
    if (gst_jig_params == NULL) {
        pkLOG_ERR(" %s Func, malloc failed! %d \n", __func__, __LINE__);
    }

    return 0;
}


int pkjig_serial_taskloop(void *pvParams)
{
    Datas_t mstCommEvent = { 0, NULL };
    while(1) 
    {
         // 1. 读取UART FIFO 的数据
        Serial_RecvData_Func();

         // 2. 从接收缓存队列里获取最早一帧的指令，并更新接收结构体
        if (GetSerialActiveData() == 0) { }

        // 3. xQueueReceive Datas
        memset(&mstCommEvent, 0, sizeof(Datas_t));
        if (xQueueReceive(xQueueCommEvent, &mstCommEvent, (portTickType)(100)/portTICK_RATE_MS))
        {
            // printf("pk serial task: min = %u, %u, %d \n", esp_get_minimum_free_heap_size(), esp_get_free_heap_size(), __LINE__);

            if (mstCommEvent.u16Value & COMM_EVENT_RECV_CMD) {
                // pkLOG_INFO("Event - Recv Data! %s, %d\n", __FILE__, __LINE__);
                AnalyzeRecvData(mstCommEvent.pu08Data);
                if (mstCommEvent.pu08Data) { free(mstCommEvent.pu08Data); mstCommEvent.pu08Data = NULL; }
            }
            if (mstCommEvent.u16Value & COMM_EVENT_SEND_CMD) {
                // 发送命令
                pkLOG_INFO("Event - Send Data! %d\n", __LINE__);
                Datas_t stCommSend = { 0, NULL };
                memset(&stCommSend, 0, sizeof(Datas_t));
                stCommSend.u16Value = mstCommEvent.pu08Data[COMM_LENGTH_INDEX];
                stCommSend.pu08Data = mstCommEvent.pu08Data;
                xQueueSendToBack(xQueueCommSend, &stCommSend, (portTickType)1/portTICK_RATE_MS);
            }
        }

        // 4. 发送指令命令
        if (SerialSendDatas() == 0) { }

        if (mstCommEvent.pu08Data) { free(mstCommEvent.pu08Data); mstCommEvent.pu08Data = NULL; }
        vTaskDelay(TASK_COMM_TIME);
    }

    pkLOG_INFO(" pk serial taskloop exit! %d \n", __LINE__);
    vTaskDelete(NULL);
    return 0;
}

int pkjig_esp_mcu_comm(unsigned char cmd, unsigned char node, const unsigned char *pcdata, unsigned short size, unsigned char bfInt)
{
    unsigned char *_pdata = NULL;
    unsigned short _crc = 0;
    unsigned char _u08length = 0;
    unsigned short _u16cnt = 0;
    Datas_t _stValue = { 0, NULL };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
       
    _u08length = JIGCOMM_HEADERS_SIZE + JIGCOMM_CRC_SIZE + size;
    _pdata = (unsigned char *)malloc(sizeof(unsigned char) * _u08length);
    if (_pdata == NULL) {
        pkLOG_ERR(" %s Func, malloc failed! %d \n", __func__, __LINE__);
        return -1;
    }
    _pdata[JIGCOMM_HEADER_0_INDEX] = JIGCOMM_HEADER_0_DATA;
    _pdata[JIGCOMM_HEADER_1_INDEX] = JIGCOMM_HEADER_1_DATA;
    _pdata[JIGCOMM_LEN_INDEX] = _u08length;
    _pdata[JIGCOMM_CMD_INDEX] = cmd;
    _pdata[JIGCOMM_SEQ_INDEX] = set_UART_MSG_seq(cmd, get_UART_MSG_seq(cmd) + 1);
    _pdata[JIGCOMM_ACK_INDEX] = JIGCOMM_ACK_SUCCESS;
    _pdata[JIGCOMM_NODE_INDEX] = node; 

    if ((pcdata == NULL) || (size == 0)) {
        memcpy(&_pdata[JIGCOMM_PAYLOAD_INDEX], pcdata, size);
    }

    _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
    memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));

    _stValue.u16Value = _u08length;
    _stValue.pu08Data = _pdata;

    if (bfInt) {
        if (pdFALSE == xQueueSendToBackFromISR(xQueueCommSend, &_stValue, &xHigherPriorityTaskWoken)) {
            if (_pdata) { free(_pdata); _pdata = NULL; }
            return -1;
        }
    } else {
        if(pdFALSE == xQueueSendToBack(xQueueCommSend, &_stValue, (portTickType)1 / (portTICK_RATE_MS * 10))) {
            pkLOG_ERR(" Start MCUESP Comm! QueueSendtoBack Error! %d \n", __LINE__);
            if (_pdata) { free(_pdata); _pdata = NULL; }
            return -1;
        }
    }
    return 0;
}

/*****对外通信接口******************************************************************/
int pkjig_set_out_data(unsigned short addr, unsigned short data)
{
    int _ret = 0;
    jig_io_payload_t _payload;
    memset(&_payload, 0, sizeof(_payload));
    _payload.addr = addr;
    _payload.data = data;

    _ret = pkjig_esp_mcu_comm(E_JIG_COMM_SET_IO_CMD, 0, (const unsigned char*)&_payload, sizeof(jig_io_payload_t), 0);

    return _ret;
}
int pkjig_get_in_data(unsigned short addr, unsigned short data)
{
    int _ret = 0;
    jig_io_payload_t _payload;
    memset(&_payload, 0, sizeof(_payload));
    _payload.addr = addr;
    _payload.data = data;
    _ret = pkjig_esp_mcu_comm(E_JIG_COMM_GET_IO_CMD, 0, (const unsigned char*)&_payload, sizeof(jig_io_payload_t), 0);
    return _ret;    
}
/*********************************************************************************/

static void PushRecvSerialBuff(unsigned char u08Value)
{
	if ((mu16SerialARecvBuffFront == (mu16SerialARecvBuffBack + 1))
	 || ((mu16SerialARecvBuffFront == 0) && (mu16SerialARecvBuffBack == (SERIALA_RECVBUFF_SIZE - 1)))) {
        // pkLOG_ERR("PushRecvSerialBuff is Error BuffFront is %d, BuffBack is %d, %s, %d!\n", mu16SerialARecvBuffFront, mu16SerialARecvBuffBack, __FILE__, __LINE__);
		return;
	}
	if (mu16SerialARecvBuffBack >= SERIALA_RECVBUFF_SIZE)		//越界则回到数组头位置
		mu16SerialARecvBuffBack = 0;    
	mau08SerialARecvBuff[mu16SerialARecvBuffBack++] = u08Value;
	if (mu16SerialARecvBuffBack >= SERIALA_RECVBUFF_SIZE)		//越界则回到数组头位置
		mu16SerialARecvBuffBack = 0;     
    //printf("PushData: Front: %d; Back: %d; Value = %02x, %s, %d\n", mu16SerialARecvBuffFront, mu16SerialARecvBuffBack, u08Value, __FILE__, __LINE__);
}
static unsigned char PopRecvSerialBuff(void)
{
	unsigned char u08Value = 0x00;

	if (mu16SerialARecvBuffFront >= SERIALA_RECVBUFF_SIZE)
		mu16SerialARecvBuffFront = 0;
	u08Value = mau08SerialARecvBuff[mu16SerialARecvBuffFront];
	mu16SerialARecvBuffFront++;
	if (mu16SerialARecvBuffFront >= SERIALA_RECVBUFF_SIZE)
		mu16SerialARecvBuffFront = 0;    

	return u08Value;
}
static unsigned short GetRecvSerialBuffLength(void)
{
	unsigned short u16Data = 0;
	if (mu16SerialARecvBuffBack >= mu16SerialARecvBuffFront) {
		u16Data = mu16SerialARecvBuffBack - mu16SerialARecvBuffFront;
	} else {
		u16Data = mu16SerialARecvBuffBack + SERIALA_RECVBUFF_SIZE - mu16SerialARecvBuffFront;
	}
	return u16Data;
}
static void Serial_RecvData_Func(void)
{
    unsigned char u08Data = 0;
    unsigned int u32FifoCnt = 0;
	/*从fifo中接收数据并分析然后发送到任务中去*/
	while (0 < (u32FifoCnt = READ_PERI_REG(UART_STATUS_REG(1)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)))
	{
		u08Data = READ_PERI_REG(UART_FIFO_REG(1)) & 0xFF;
        PushRecvSerialBuff(u08Data);
        // pkLOG_INFO(" 0x%02x ,", u08Data);
	}
}

static short GetSerialActiveData(void)     // 返回一组有效的周期数据
{
    unsigned char u08TmpData = 0x00;                  // 临时变量
    unsigned char u08Length = 0x00;                   // 计算有效数据包长度
    unsigned short u16CheckSum = 0;                   // CheckSum 计算
    unsigned short u16TmpFront = 0;
	unsigned short u16InitFront = mu16SerialARecvBuffFront;		// 用于存储 最初缓存队列的 Front
    unsigned char *pu08Data = NULL;                   // 用于存储一个完整的有效数据包
    Datas_t stRecvData = { 0, NULL };                     // 用于传输数据

    bool bfCheckSum = 0;                    // 用于CheckSum标志位打印
    unsigned char u08IndexCnt = 0x00;                 // 用于打印接受数据下标计数

    unsigned char u08Cnt = 0;                         // 用于判断数字头
    unsigned short u08FirstLength = 0;                 // 长度
    unsigned char u08MaxCnt = 0;                      // 循环次数    

    memset(&stRecvData, 0, sizeof(Datas_t));

    // 0. 判断接收Buff里面是否有数据
    u08FirstLength = GetRecvSerialBuffLength();
    if (u08FirstLength <= COMM_LENGTH_INDEX)
    {
        // pkLOG_DEBUG("SerialA Buff is NoEnough! BuffLength = %d, %s, %d \r\n", u08FirstLength, __FILE__, __LINE__);
        return 0;
    }
    /****************************************************************/
    if (u08FirstLength >= MAXNUM_ONETIME) { u08MaxCnt = MAXNUM_ONETIME; }
    else { u08MaxCnt = u08FirstLength; }
    // 1. 判断头是否符合标准 0x5A
    for (u08Cnt = 0; u08Cnt < u08MaxCnt; u08Cnt++)
    {
        u16InitFront = mu16SerialARecvBuffFront;        // 指针永远指向当前最新取得数据值
        u08TmpData = PopRecvSerialBuff();
        if (u08TmpData == COMM_HEADER0_DATA)
            break;      // 如果值一样，则退出循环
    }
    if (u08Cnt >= u08MaxCnt)
    {
        pkLOG_DEBUG("Header is not 0x5A, is %d, %s, %d \n", u08TmpData, __FILE__, __LINE__);
        return 0;
    }    
    u08FirstLength = GetRecvSerialBuffLength();
    if (u08FirstLength < COMM_LENGTH_INDEX)
    {
        pkLOG_DEBUG("Length is Lower! u08FIrstLength = %d, %s, %d\n", u08FirstLength, __FILE__, __LINE__);
        mu16SerialARecvBuffFront = u16InitFront;
        return 0;
    }
    /****************************************************************/

    // 2. 判断头是否符合标准 0xA5
    u08TmpData = PopRecvSerialBuff();
    if (u08TmpData != COMM_HEADER1_DATA)
    {
        pkLOG_ERR("RecvBuff Header1 is Error!  = %d, != %d!, %s, %d \n", u08TmpData, COMM_HEADER1_DATA, __FILE__, __LINE__);
        return 0;
    }    

    // 3. 判断数据长度是否为一帧有效数据
    u08Length = PopRecvSerialBuff();
//    pkLOG_INFO("u08Length = %d, BuffLength = %d, %s, %d\n", u08Length, u08FirstLength, __FILE__, __LINE__);
    if (u08Length > (GetRecvSerialBuffLength() + 3))
    {
        pkLOG_DEBUG("Buff Length is Lower than Protocol Data Length! %s, %d\n", __FILE__, __LINE__);
        mu16SerialARecvBuffFront = u16InitFront;
        return 0;
    }

    // 4. 将接受的数据编程一个有效数据包, 传输到外面进行数据分析
    pu08Data = (unsigned char*)malloc(sizeof(unsigned char) * u08Length);
    if (pu08Data == NULL) {
        pkLOG_ERR("%s Func mallock is Error!!\n", __func__);
        mu16SerialARecvBuffFront = u16InitFront;
        return 0;
    }
    memset(pu08Data, 0, sizeof(unsigned char) * u08Length);

    // 5. 将一帧数据复制到指针内存里面继续处理
//    pkLOG_INFO("u16InitFront = %d, u16Front = %d, u16Back = %d \n", u16InitFront, mu16SerialARecvBuffFront, mu16SerialARecvBuffBack);
    // pkLOG_INFO("BuffLength = %d, %s, %d\n", u08FirstLength, __FILE__, __LINE__);
    if ((u08Length - 1 + u16InitFront) >= SERIALA_RECVBUFF_SIZE)	//读取的时候循环到列表起始位置的情况
	{
		memcpy(pu08Data, (unsigned char *)(mau08SerialARecvBuff + u16InitFront), (sizeof(unsigned char) * (SERIALA_RECVBUFF_SIZE - u16InitFront)));
        memcpy(pu08Data + (SERIALA_RECVBUFF_SIZE - u16InitFront), (unsigned char*)mau08SerialARecvBuff, sizeof(unsigned char) * (u08Length - (SERIALA_RECVBUFF_SIZE - u16InitFront)));
		u16TmpFront = u08Length - (SERIALA_RECVBUFF_SIZE - u16InitFront);
	}
	else
	{
		memcpy(pu08Data, (unsigned char *)(mau08SerialARecvBuff + u16InitFront), sizeof(unsigned char) * u08Length);
		u16TmpFront = u08Length + u16InitFront;
		if (u16TmpFront >= SERIALA_RECVBUFF_SIZE)
		{
			u16TmpFront -= SERIALA_RECVBUFF_SIZE;
		}
	}
	mu16SerialARecvBuffFront = u16TmpFront;

    // 6. 进行CheckSum计算判定
    u08Length = pu08Data[COMM_LENGTH_INDEX];    // 此数据包长度
    u16CheckSum = petkit_crc16(pu08Data, (u08Length - COMM_CHECKSUM_SIZE), NULL);
    if (u16CheckSum != (((unsigned short)pu08Data[u08Length - 1] << 8) + pu08Data[u08Length - 2]))
    {
        pkLOG_ERR("%s Func - CheckSum is Error! u16CheckSum = %d \n", __func__, u16CheckSum);
        bfCheckSum = 1;
        // return 0;
    }

    pkLOG_DEBUG("RecvData: ");
    for (u08IndexCnt = 0; u08IndexCnt < u08Length; u08IndexCnt++)
    {
        pkLOG_DEBUG("%02x ", pu08Data[u08IndexCnt]);
    }
    pkLOG_DEBUG("\n");
    

    //printf("CheckSumH = %02x, CheckSumL = %02x \n", (uint8_t)(u16CheckSum >> 8), (uint8_t)(u16CheckSum & 0x00FF));
    if (bfCheckSum == 1)
    {
        free(pu08Data);
        return 0;
    }

    stRecvData.u16Value = COMM_EVENT_RECV_CMD;
    stRecvData.pu08Data = pu08Data;

//    pkLOG_INFO("System Free Heap Size = %d! %s, %d\n", system_get_free_heap_size(), __FILE__, __LINE__);
	xQueueSendToBack(xQueueCommEvent, &stRecvData, (portTickType)1 / (portTICK_RATE_MS * 10));

    return 1;
}


static short SerialSendDatas(void)
{
    unsigned int u32FIFOCnt = 0;
    unsigned int _u32Data = 0;
    unsigned short u16IndexCnt = 0;
    int s32TaskSum = 0;
    int s32Cnt = 0;
    Datas_t stComm = { 0, NULL};
    Datas_t *p_stvalue = NULL;

    s32TaskSum = uxQueueMessagesWaiting(xQueueCommSend);
    // if (s32TaskSum != 0) {
    //     pkLOG_INFO(" s32TaskSum = %d, %d \n", s32TaskSum, __LINE__);
    // }
    if (s32TaskSum >= SERIAL_SENDTASK_MAX) { s32TaskSum = SERIAL_SENDTASK_MAX; }
    else if (s32TaskSum <= 0) { s32TaskSum = 0; return 1; }

    for (s32Cnt = 0; s32Cnt < s32TaskSum; s32Cnt++)
    {
        memset(&stComm, 0, sizeof(Datas_t));
        if (pdFALSE == xQueueReceive(xQueueCommSend, &stComm, (portTickType)1/(portTICK_RATE_MS))) {
            continue;
        }
        if ((stComm.pu08Data == NULL) || (stComm.u16Value <= 0)) {
            pkLOG_INFO(" serial send datas - stComm.pu08Data is NULL! %d\n", __LINE__);
            if (stComm.pu08Data) { free(stComm.pu08Data); stComm.pu08Data = NULL;}
            continue;
        }

        u32FIFOCnt = READ_PERI_REG(UART_STATUS_REG(UART_NUM_1)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);
        _u32Data = ((u32FIFOCnt >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT);
        // pkLOG_INFO(" Cnt = %d, %p, %d, %d, %d, %d \n", s32Cnt, stComm.pu08Data, stComm.u16Value, u32FIFOCnt, _u32Data, __LINE__);
        if (_u32Data <= 126) //不使用缓冲，一直到缓冲空在发送后面的
        {
            u16IndexCnt = 0;
            // pkLOG_DEBUG(" esp -> mcu: ");
            while(u16IndexCnt < stComm.u16Value)
            {
                // pkLOG_DEBUG("%02x ", stComm.pu08Data[u16IndexCnt]);
                WRITE_PERI_REG(UART_FIFO_REG(UART_NUM_1) , stComm.pu08Data[u16IndexCnt]);
                u16IndexCnt++;
            }
            // pkLOG_DEBUG("\n");
            if (stComm.pu08Data) { free(stComm.pu08Data); stComm.pu08Data = NULL;}
        }
        else
        {
            // 处理
            if (pdFALSE == xQueueSendToFront(xQueueCommSend, &stComm, (portTickType)1/(portTICK_RATE_MS))) {
                if (stComm.pu08Data) { free(stComm.pu08Data); stComm.pu08Data = NULL;}
            }
        }
    }

    return 1;
}

static void _print_sensor_datas(sensor_sta_t *pdata) 
{
    /**********************************************************************/
    pkLOG_INFO("/********************************************************/ \n");
    pkLOG_INFO(" mot_adc: %d, whq_adc: %d, dc_adc: %d, io_det: %d, %d \n", pdata->mot_adc, pdata->whq_adc, pdata->dc_adc, pdata->io_det, __LINE__);
    pkLOG_INFO(" det_wh1: %d, ", (pdata->io_det >> DET_WH1) & 0x0001);
    pkLOG_INFO(" det_wh2: %d, ", (pdata->io_det >> DET_WH2) & 0x0001);
    pkLOG_INFO(" det_wh3: %d, ", (pdata->io_det >> DET_WH3) & 0x0001);
    pkLOG_INFO(" det_sand_act: %d, ", (pdata->io_det >> DET_SAND_ACT) & 0x0001);
    pkLOG_INFO(" det_ws_off: %d, ", (pdata->io_det >> DET_WS_OFF) & 0x0001);
    pkLOG_INFO(" det_sand_l: %d, ", (pdata->io_det >> DET_SAND_L) & 0x0001);
    pkLOG_INFO(" det_sand_r: %d, ", (pdata->io_det >> DET_SAND_R) & 0x0001);
    pkLOG_INFO(" det_wakeup: %d, ", (pdata->io_det >> DET_WAKEUP) & 0x0001);
    pkLOG_INFO(" \n");
    pkLOG_INFO("/********************************************************/ \n");
    /**********************************************************************/
}

static short AnalyzeRecvData(const unsigned char *pu08Data)
{
    short s16Return = 0;
    unsigned char u08Ack = 0x00;
    unsigned char u08Cmd = 0x00;
    unsigned char u08WRSyncNode = 0x00;
    unsigned char _u08Seq = 0x00;
    
    if (pu08Data == NULL) {
        pkLOG_ERR("Receive Data is NULL!\n");
        return -1;
    }

    u08Ack = pu08Data[JIGCOMM_ACK_INDEX];
    u08Cmd = pu08Data[JIGCOMM_CMD_INDEX];
    u08WRSyncNode = pu08Data[JIGCOMM_NODE_INDEX];
    _u08Seq = pu08Data[JIGCOMM_SEQ_INDEX];
    // pkLOG_INFO("ESP MCU Comm cmd:%d, ack:%d, seq:%d\n", u08Cmd,  u08Ack, _u08Seq);
    
    switch(u08Cmd) 
    {
    case E_JIG_COMM_SET_IO_CMD:
        pkLOG_INFO(" recv set_io cmd! %d \n", __LINE__);

        break;
    case E_JIG_COMM_GET_IO_CMD:
        pkLOG_INFO(" recv get_io cmd! %d \n", __LINE__);
        
        break;
    default:
        pkLOG_ERR(" %s Func, cmd is over range! cmd: %d, %d \n", __func__, u08Cmd, __LINE__);
        break;
    }

    return 0;
}


static unsigned char set_UART_MSG_seq(int seq_index, unsigned char value)
{
    ga_uart_msg_seq[seq_index] = value;
    return ga_uart_msg_seq[seq_index];
}
static unsigned char get_UART_MSG_seq(int seq_index)
{
    return ga_uart_msg_seq[seq_index];
}

#undef __PK_SERIAL_C_
