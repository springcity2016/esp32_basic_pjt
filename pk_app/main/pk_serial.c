
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

#include "pk_app.h"

#define TASK_COMM_TIME                 (10/portTICK_RATE_MS)        // 串口通信任务延时周期时间
#define MAXNUM_ONETIME (128)


extern pkapp_params_t gst_app_params;


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

static unsigned char ga_uart_msg_seq[COMM_CMD_MAX] = {0};
static unsigned char set_UART_MSG_seq(int seq_index, unsigned char value);
static unsigned char get_UART_MSG_seq(int seq_index);

// static void _pk_driver_sec_cb(void *arg)


int pk_serial_init(void)
{
    xQueueCommEvent = xQueueCreate(COMMEVENT_QUEUE_MAX, sizeof(CommFunc_t));
    if (xQueueCommEvent == NULL) {
       pkLOG_ERR("TaskComm xQueueCommEvent is NULL!\n"); 
    }
    xQueueCommSend = xQueueCreate(COMMSEND_QUEUE_MAX, sizeof(Datas_t));
    if (xQueueCommSend == NULL) {
        pkLOG_ERR("TaskComm xQueueCommSend is NULL!\n");
    }

    return 0;
}


int pk_serial_taskloop(void *pvParams)
{
    Datas_t mstCommEvent = { 0, NULL };
    while(1) 
    {
         // 1. 读取UART FIFO 的数据
        Serial_RecvData_Func();

         // 2. 从接收缓存队列里获取最早一帧的指令，并更新接收结构体
        if (GetSerialActiveData() == 0) {
            // printf("GetSerialActiveData is Error!\n");
        }

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
        if (SerialSendDatas() == 0) {
            // pkLOG_ERR("SerialSendDatas is Error!\n");
        }

        if (mstCommEvent.pu08Data) { free(mstCommEvent.pu08Data); mstCommEvent.pu08Data = NULL; }
        vTaskDelay(TASK_COMM_TIME);
    }

    pkLOG_INFO(" pk serial taskloop exit! %d \n", __LINE__);
    vTaskDelete(NULL);
    return 0;
}

int pk_esp_mcu_comm(unsigned char cmd, unsigned char wr_read, unsigned char node, const unsigned char *pcdata, unsigned short size, unsigned char bfInt)
{
    unsigned char *_pdata = NULL;
    unsigned short _crc = 0;
    unsigned char _u08length = 0;
    unsigned short _u16cnt = 0;
    Datas_t _stValue = { 0, NULL };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch(cmd) 
    {
    case COMM_CMD_HB:       // 0. 心跳信息
        pkLOG_INFO(" %s Func, Heart Beat Cmd! %d \n", __func__, __LINE__);
        if (wr_read == COMM_WRITE_CMD) {
            if ((pcdata == NULL) || (size == 0)) {
                pkLOG_ERR(" write hb cmd input params is NULL! %d \n", __LINE__);
                return -1;
            }
            _u08length = size + 9;
            _pdata = (unsigned char *)malloc(sizeof(unsigned char) * _u08length);
            if (_pdata == NULL) {
                pkLOG_ERR(" malloc error! %d \n", __LINE__);
                return -1;
            }
            _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;
            _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
            _pdata[COMM_LENGTH_INDEX] = _u08length;
            _pdata[COMM_CMD_INDEX] = COMM_CMD_HB;
            _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_HB, get_UART_MSG_seq(COMM_CMD_HB) + 1);
            _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));
            _pdata[COMM_DATASTART_INDEX] = pcdata[0];
            _u16cnt = COMM_DATASTART_INDEX + 1;
            _pdata[_u16cnt] = pcdata[0];
            _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
            memcpy(&_pdata[_u08length - 2], &_crc, sizeof(unsigned short));

        } else if (wr_read == COMM_READ_CMD) {
            _u08length = 9;
            _pdata = (unsigned char *)malloc(sizeof(unsigned char) * _u08length);
            _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;
            _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
            _pdata[COMM_LENGTH_INDEX] = _u08length;
            _pdata[COMM_CMD_INDEX] = COMM_CMD_HB;
            _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_HB, get_UART_MSG_seq(COMM_CMD_HB) + 1);
            _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
            _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
            memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        }
        break;
    case COMM_CMD_SENSOR:
        pkLOG_INFO(" %s Func, Sensor Cmd! %d \n", __func__, __LINE__);
        _u16cnt = COMM_DATASTART_INDEX;
        _u08length = 9 + size;
        // if (node == E_RPT_TYPE_TIMING) {
        //     _u08length++;
        // }
        _pdata = (unsigned char *)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_SENSOR;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_SENSOR, get_UART_MSG_seq(COMM_CMD_SENSOR) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
            if (node == E_RPT_TYPE_TIMING) {
                if (pcdata != NULL) {
                    _pdata[COMM_DATASTART_INDEX] = pcdata[0];
                }
            }        
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }

        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        break;
    case COMM_CMD_MOT_RUN_CFG:
        pkLOG_INFO(" %s Func, Motor Run Config Cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_MOT_RUN_CFG;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_MOT_RUN_CFG, get_UART_MSG_seq(COMM_CMD_MOT_RUN_CFG) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
            if ((pcdata != NULL) && (size != 0)) {
                _u16cnt = COMM_DATASTART_INDEX;
                memcpy(&_pdata[_u16cnt], pcdata, size);
            }
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }        
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        break; 
    case COMM_CMD_MOT_RUN_RT:         // 获取电机运行中的关键数据
        pkLOG_INFO(" %s Func, Motor Run realtime Cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_MOT_RUN_RT;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_MOT_RUN_RT, get_UART_MSG_seq(COMM_CMD_MOT_RUN_RT) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
            if ((pcdata != NULL) && (size != 0)) {
                _u16cnt = COMM_DATASTART_INDEX;
                memcpy(&_pdata[_u16cnt], pcdata, size);
            }
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }        
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        // unsigned char _u08cnt = 0;
        // pkLOG_INFO(" motor run real value: ");
        // for(_u08cnt = 0; _u08cnt < _u08length; _u08cnt++) {
        //     pkLOG_INFO(" 0x%02x ", _pdata[_u08length]);
        // }
        // pkLOG_INFO(" \n");
        break;
    case COMM_CMD_MOT_QT_CFG:         //
        pkLOG_INFO(" %s Func, Motor qita config Cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_MOT_QT_CFG;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_MOT_QT_CFG, get_UART_MSG_seq(COMM_CMD_MOT_QT_CFG) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
            if ((pcdata != NULL) && (size != 0)) {
                _u16cnt = COMM_DATASTART_INDEX;
                memcpy(&_pdata[_u16cnt], pcdata, size);
            }
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }        
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));        
        break;
    case COMM_CMD_MOT_QT_RT:          //
        pkLOG_INFO(" %s Func, Motor qita realtime Cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_MOT_QT_RT;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_MOT_QT_RT, get_UART_MSG_seq(COMM_CMD_MOT_QT_RT) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
            if ((pcdata != NULL) && (size != 0)) {
                _u16cnt = COMM_DATASTART_INDEX;
                memcpy(&_pdata[_u16cnt], pcdata, size);
            }
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }        
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        break;
    case COMM_CMD_WEIGHT:           // 称重
        pkLOG_INFO(" %s Func, Weight Cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;  
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_WEIGHT;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_WEIGHT, get_UART_MSG_seq(COMM_CMD_WEIGHT) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;        
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
            if ((pcdata != NULL) && (size != 0)) {
                _u16cnt = COMM_DATASTART_INDEX;
                memcpy(&_pdata[_u16cnt], pcdata, size);
            }
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        break;
    case COMM_CMD_ERR:               // 7. 错误代码
        pkLOG_INFO(" %s Func, error code cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;  
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_ERR;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_ERR, get_UART_MSG_seq(COMM_CMD_ERR) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        break;
    case COMM_CMD_VER:               // 8. 软件代码
        pkLOG_INFO(" %s Func, version cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;  
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_VER;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_VER, get_UART_MSG_seq(COMM_CMD_VER) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        break;
    case COMM_CMD_RST:               // 9. 软件复位
        pkLOG_INFO(" %s Func, reset mcu cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;  
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_RST;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_RST, get_UART_MSG_seq(COMM_CMD_RST) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));            
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));
        break;
    case COMM_CMD_WORKMODE:          // 10. 工作模式
        pkLOG_INFO(" %s Func, work mode cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;  
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_WORKMODE;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_WORKMODE, get_UART_MSG_seq(COMM_CMD_WORKMODE) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));
            if ((pcdata != NULL) && (size != 0)) {
                _u16cnt = COMM_DATASTART_INDEX;
                memcpy(&_pdata[_u16cnt], pcdata, size);
            }        
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));    
        break;
    case COMM_CMD_CFGDATA:           // 11. 配置底层工作数据
        pkLOG_INFO(" %s Func, config datas cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;  
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_CFGDATA;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_CFGDATA, get_UART_MSG_seq(COMM_CMD_CFGDATA) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));
            if ((pcdata != NULL) && (size != 0)) {
                _u16cnt = COMM_DATASTART_INDEX;
                memcpy(&_pdata[_u16cnt], pcdata, size);
            }        
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short));          
        break;
    case COMM_CMD_OUTCTRL:
        pkLOG_INFO(" %s Func, output control cmd! %d \n", __func__, __LINE__);
        _u08length = 9 + size;
        _pdata = (unsigned char*)malloc(sizeof(unsigned char) * _u08length);
        if (_pdata == NULL) {
            pkLOG_ERR(" malloc error! %d \n", __LINE__);
            return -1;
        }
        _pdata[COMM_HEADER0_INDEX] = COMM_HEADER0_DATA;  
        _pdata[COMM_HEADER1_INDEX] = COMM_HEADER1_DATA;
        _pdata[COMM_LENGTH_INDEX] = _u08length;
        _pdata[COMM_CMD_INDEX] = COMM_CMD_OUTCTRL;
        _pdata[COMM_SEQ_INDEX] = set_UART_MSG_seq(COMM_CMD_OUTCTRL, get_UART_MSG_seq(COMM_CMD_OUTCTRL) + 1);
        _pdata[COMM_ACK_INDEX] = COMM_ACK_SUCCESS;
        if (wr_read == COMM_WRITE_CMD) {
            _pdata[COMM_WRSYNNODE_INDEX] = node & (~(0x01 << COMM_W_R_POS));
            if ((pcdata != NULL) && (size != 0)) {
                _u16cnt = COMM_DATASTART_INDEX;
                memcpy(&_pdata[_u16cnt], pcdata, size);
            }        
        } else {
            _pdata[COMM_WRSYNNODE_INDEX] = node | (0x01 << COMM_W_R_POS);
        }
        _crc = petkit_crc16(_pdata, (_u08length - 2), NULL);
        memcpy(&_pdata[_u08length - 2], (unsigned char *)&_crc, sizeof(unsigned short)); 
        break;
    default:
        pkLOG_ERR(" %s Func, cmd is over range! cmd = %d, %d \n", __func__, cmd, __LINE__);
        break;
    }

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
// #define RECV_BUFFSIZE           (1024)
// static unsigned char recv_buff[RECV_BUFFSIZE] = { 0 };
static void Serial_RecvData_Func(void)
{
    // int _cnt = 0;
    // int _len = 0;
    // _len = uart_read_bytes(UART_NUM_0, recv_buff, RECV_BUFFSIZE, 10 / portTICK_PERIOD_MS);
    // printf("len : %d \n", _len);
    // if (_len > 0) {
    //     for (_cnt = 0; _cnt < _len; _cnt++) {
    //         printf(" 0x%02x ", recv_buff[_cnt]);
    //     }
    //     printf(" \n");
    // }
    unsigned char u08Data = 0;
    unsigned int u32FifoCnt = 0;
	/*从fifo中接收数据并分析然后发送到任务中去*/
	while (0 < (u32FifoCnt = READ_PERI_REG(UART_STATUS_REG(0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)))
	{
		u08Data = READ_PERI_REG(UART_FIFO_REG(0)) & 0xFF;
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
    if (pu08Data == NULL)
    {
        pkLOG_ERR("GetSerialActiveData[] Func mallock is Error!!\n");
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
        pkLOG_ERR("GetSerialActiveData - CheckSum is Error! u16CheckSum = %d \n", u16CheckSum);
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
            pkLOG_INFO("SerialSendDatasFunc - stComm.pu08Data is NULL!\n");
            if (stComm.pu08Data) { free(stComm.pu08Data); stComm.pu08Data = NULL;}
            continue;
        }

        u32FIFOCnt = READ_PERI_REG(UART_STATUS_REG(UART_NUM_0)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);
        _u32Data = ((u32FIFOCnt >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT);
        // pkLOG_INFO(" Cnt = %d, %p, %d, %d, %d, %d \n", s32Cnt, stComm.pu08Data, stComm.u16Value, u32FIFOCnt, _u32Data, __LINE__);
        if (_u32Data <= 126) //不使用缓冲，一直到缓冲空在发送后面的
        {
            u16IndexCnt = 0;
            // pkLOG_DEBUG(" esp -> mcu: ");
            while(u16IndexCnt < stComm.u16Value)
            {
                // pkLOG_DEBUG("%02x ", stComm.pu08Data[u16IndexCnt]);
                WRITE_PERI_REG(UART_FIFO_REG(UART_NUM_0) , stComm.pu08Data[u16IndexCnt]);
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
    unsigned char _u08lenth = 0x00;
    
    if (pu08Data == NULL) {
        pkLOG_ERR("Receive Data is NULL!\n");
        return -1;
    }

    u08Ack = pu08Data[COMM_ACK_INDEX];
    u08Cmd = pu08Data[COMM_CMD_INDEX];
    u08WRSyncNode = pu08Data[COMM_WRSYNNODE_INDEX];
    _u08Seq = pu08Data[COMM_SEQ_INDEX];
    _u08lenth = pu08Data[COMM_LENGTH_INDEX];

    // pkLOG_INFO("ESP MCU Comm cmd:%d, ack:%d, seq:%d\n", u08Cmd,  u08Ack, _u08Seq);
    
    switch(u08Cmd) 
    {
    case COMM_CMD_HB:       // HearBeat
        pkLOG_INFO(" recv: MCU -> ESP, Heart beat! %d \n", __LINE__);
        pkmcu_set_exphy_upsta(E_EXPHY_HB, _u08Seq, pk_os_runtime());

        if ((pdDETECT_WAIT == pk_get_detect_sta())
         & (pkdet_get_pcbstep() == E_PCBTEST_MCU_COMM_STEP)) {
            pk_send_itemtest_event(E_TEST_MCU_COMM_ITEM, E_DET_MCU_COMM_RECV, NULL, 0);
        }
        break;
    case COMM_CMD_SENSOR:   // 传感器命令
        // pkLOG_INFO(" recv: sensor cmd! %d \n", __LINE__);
        pkmcu_set_exphy_upsta(E_EXPHY_SENSOR, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_MOT_ADC, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_WHQ_ADC, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DC_ADC, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DET_WH1, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DET_WH2, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DET_WH3, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DET_SAND_ACT, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DET_WS_OFF, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DET_SAND_L, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DET_SAND_R, _u08Seq, pk_os_runtime());
        pkmcu_set_exphy_upsta(E_EXPHY_DET_WAKEUP, _u08Seq, pk_os_runtime());

        drv_physta_t _status;
        memset(&_status, 0, sizeof(drv_physta_t));
        get_phy_status(&_status);
        sensor_sta_t *pdata = ((sensor_sta_t *)&pu08Data[COMM_DATASTART_INDEX]);
        if (((_status.st_sensor_sta.io_det & (0x01 << DET_SAND_L)) != (pdata->io_det & (0x01 << DET_SAND_L)))
         || ((_status.st_sensor_sta.io_det & (0x01 << DET_SAND_R)) != (pdata->io_det & (0x01 << DET_SAND_R)))) {
            gst_app_params.mot_run_real.mot_pos_end = ((pdata->io_det & (0x01 << DET_SAND_L)) != 0) ? 1 : 0;
            gst_app_params.mot_run_real.mot_pos_start = ((pdata->io_det & (0x01 << DET_SAND_R)) != 0) ? 1 : 0;
            gst_app_params.mot_run_real.os_time = sys_get_os_runtime_sec();
            pkLOG_INFO(" position, end: %d, start: %d, os_time: %d, %d \n", gst_app_params.mot_run_real.mot_pos_end, gst_app_params.mot_run_real.mot_pos_start, sys_get_os_runtime_sec(), __LINE__);
            if (pk_get_detect_sta() == pdDETECT_FALSE) {
                sandmot_real_result_t *p_real_mot = NULL;
                p_real_mot = (sandmot_real_result_t *)malloc(sizeof(sandmot_real_result_t));
                if (p_real_mot == NULL) { break; }
                memset(p_real_mot, 0, sizeof(sandmot_real_result_t));
                memcpy(p_real_mot, &(gst_app_params.mot_run_real), sizeof(sandmot_real_result_t));
                pkapp_set_mot_step_sta(pkapp_get_now_step(), E_MOT_STEP_STA_RESULT, (const unsigned char *)p_real_mot, 0);
            }
        }
        set_exphy_sensordata((sensor_sta_t *)&pu08Data[COMM_DATASTART_INDEX]);

        if ((pdDETECT_WAIT == pk_get_detect_sta())
         & (pkdet_get_pcbstep() == E_PCBTEST_DC_STEP)) {
            pk_send_itemtest_event(E_TEST_DC_POWER_ITEM, E_DET_DC_RECV_STEP, NULL, 0);
        } else if (pk_get_detect_sta() == pdDETECT_AGEING) {    // 测试模式下的情况

            // pk_send_aging_event(E_AGING_EVT_SENSOR, 0, (sensor_sta_t *)&pu08Data[COMM_DATASTART_INDEX], 0);
        }

        break;
    case COMM_CMD_MOT_RUN_CFG:// 
        pkLOG_INFO(" recv: motor runconfig cmd! %d \n", __LINE__);
        pkmcu_set_exphy_upsta(E_EXPHY_MOTRUN_CFG, _u08Seq, pk_os_runtime());
        set_exphy_motrun_cfgdata((const sandmot_runcfg_t *)&pu08Data[COMM_DATASTART_INDEX]);
        break;
    case COMM_CMD_MOT_RUN_RT: // 获取电机运行中的关键数据
        pkLOG_INFO(" recv: motor run realtime cmd! %d \n", __LINE__);
        printf(" recv data task: min = %u, %u, %d \n", esp_get_minimum_free_heap_size(), esp_get_free_heap_size(), __LINE__);

        pkmcu_set_exphy_upsta(E_EXPHY_MOTRUN_REAL, _u08Seq, pk_os_runtime());
        set_exphy_motrun_realdata((const sandmot_real_t *)&pu08Data[COMM_DATASTART_INDEX]);

        gst_app_params.mot_run_real.os_time = sys_get_os_runtime_sec();
        memcpy(&(gst_app_params.mot_run_real.real_value), &pu08Data[COMM_DATASTART_INDEX], sizeof(sandmot_real_t));
        if (u08Ack == COMM_ACK_MOTOR_OK) {
            gst_app_params.mot_run_real.result = 1;
        } else {
            gst_app_params.mot_run_real.result = 0;
        }

        sandmot_real_result_t *p_real_mot = NULL;
        p_real_mot = (sandmot_real_result_t *)malloc(sizeof(sandmot_real_result_t));
        if (p_real_mot == NULL) { break; }
        memset(p_real_mot, 0, sizeof(sandmot_real_result_t));
        memcpy(p_real_mot, &(gst_app_params.mot_run_real), sizeof(sandmot_real_result_t));

        if (pk_get_detect_sta() == pdDETECT_AGEING) {    // 测试模式下的情况
            pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_WAIT, (char *)p_real_mot, 0);
        } else if (pk_get_detect_sta() == pdDETECT_FALSE) {
            pkapp_set_mot_step_sta(pkapp_get_now_step(), E_MOT_STEP_STA_RESULT, (const unsigned char *)p_real_mot, 0);
        }
        break;
    case COMM_CMD_MOT_QT_CFG: //
        pkLOG_INFO(" recv: motor qt config cmd! %d \n", __LINE__);
        pkmcu_set_exphy_upsta(E_EXPHY_MOTQT_CFG, _u08Seq, pk_os_runtime());
        set_exphy_motqt_cfgdata((const sandmot_qtcfg_t *)&pu08Data[COMM_DATASTART_INDEX]);
        break;
    case COMM_CMD_MOT_QT_RT:  //
        pkLOG_INFO(" recv: motor qt realtime cmd! %d \n", __LINE__);
        pkmcu_set_exphy_upsta(E_EXPHY_MOTQT_CFG, _u08Seq, pk_os_runtime());
        set_exphy_motqt_realdata((const sandmot_qtreal_t *)&pu08Data[COMM_DATASTART_INDEX]);
        break;     
    case COMM_CMD_WEIGHT:            // 6. 称重传感器
        // pkLOG_INFO(" recv: weight cmd! lenth: %d, %d \n", _u08lenth, __LINE__);
        pkmcu_set_exphy_upsta(E_EXPHY_WEIGHT, _u08Seq, pk_os_runtime());
        // set_exphy_weight_data(&pu08Data[COMM_DATASTART_INDEX]);
        pkdrv_set_exphy_weight_data(&pu08Data[COMM_DATASTART_INDEX]);
        unsigned char _index_cnt = COMM_DATASTART_INDEX;
        unsigned char _u08cnt = 0;

        if (pk_get_detect_sta() == pdDETECT_AGEING) {    // 测试模式下的情况
            weight_real_t *p_weight = NULL;
            p_weight = (weight_real_t *)malloc(sizeof(weight_real_t));
            if (p_weight) { 
                memset(p_weight, 0, sizeof(weight_real_t));
                p_weight->size = pu08Data[_index_cnt];
                p_weight->p_weight = (unsigned int *)malloc(sizeof(unsigned int) * (p_weight->size));
                if (p_weight->p_weight) {
                    memset(p_weight->p_weight, 0, sizeof(unsigned int) * (p_weight->size));
                    _index_cnt++;
                    memcpy(p_weight->p_weight, &pu08Data[_index_cnt], sizeof(unsigned int) * (p_weight->size));
                    for (_u08cnt = 0; _u08cnt < p_weight->size; _u08cnt++) {
                        pkLOG_INFO(" weight: %d \n", p_weight->p_weight[_u08cnt]);
                    }
                    pkLOG_INFO("\n");
                    
                    pk_send_aging_event(E_AGING_EVT_WEIGHT, E_DET_WEIGHT_STEP_RECV, (char *)p_weight, 0);
                } else {
                    if (p_weight) { free(p_weight); p_weight = NULL; }
                }
            }
        }
        break;
    case COMM_CMD_ERR:               // 7. 错误代码
        pkLOG_INFO(" recv: error code cmd! %d \n", __LINE__);
        pkmcu_set_exphy_upsta(E_EXPHY_WEIGHT, _u08Seq, pk_os_runtime());
        set_exphy_err_data((const errcode_t *)&pu08Data[COMM_DATASTART_INDEX]);
        break;    
    case COMM_CMD_VER:               // 8. 软件代码
        pkLOG_INFO(" recv: version cmd! %d \n", __LINE__);
        pkmcu_set_exphy_upsta(E_EXPHY_VERSION, _u08Seq, pk_os_runtime());
        set_exphy_ver_data((const version_t *)&pu08Data[COMM_DATASTART_INDEX]);
        break;
    case COMM_CMD_RST:               // 9. 软件复位
        pkLOG_INFO(" recv: reset cmd! %d \n", __LINE__);
        
        break;
    case COMM_CMD_WORKMODE:          // 10. 工作模式
        pkLOG_INFO(" recv: work cmd! %d \n", __LINE__);
        
        break;
    case COMM_CMD_CFGDATA:           // 11. 配置底层工作数据
        pkLOG_INFO(" recv: config datas cmd! %d \n", __LINE__);
        
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
