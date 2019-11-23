
#ifndef __PRODUCT_TEST_H_
#define __PRODUCT_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    E_TEST_BEEP_ITEM = 0,            // 测试 Beep
    E_TEST_OLED_ITEM,                // 测试 OLED
    E_TEST_KEY_ITEM,                 // 按键检测
    E_TEST_MCU_COMM_ITEM,            // MCU 通信检测
    E_TEST_DC_POWER_ITEM,            // 检测电源电压
    E_TEST_MOT_ITEM,                 // 测试电机
    
    /***********************************/
    E_TEST_BLE_ITEM,                 // 蓝牙通信检测
    E_TEST_WIFI_ITEM,                // wifi功能检测
    E_TEST_IR_SENSOR_ITEM,           // 红外对管传感器检测
    E_TEST_WHQ_ITEM,                 // 雾化器检测
    E_TEST_WEIGHT_ITEM,              // 称重传感器检测
    E_TEST_RTC_ITEM,                 // RTC检测
    E_TEST_IO_ITEM,                  // IO通用检测
    /***********************************/

    E_TEST_FORCE_QUIT_ITEM,          // 强制退出

    E_TEST_MAX_ITEM
}drv_test_item;

typedef struct _test_item_t
{
    unsigned char block;            // 是否阻塞
    unsigned char run_step;         // run step
    unsigned char out_test;         // 是否需要退出, 1:退出测试函数, 0:进入测试函数
    unsigned int os_time;           // 当前系统时间
    unsigned int out_delay_time;    // delay time
    unsigned int out_next_time;     // 退出测试时间
    unsigned char *pdata;           // 其他结构体
}test_item_t;

enum
{
    E_DET_WEIGHT_STEP_SEND = 0,     // step 发送
    E_DET_WEIGHT_STEP_RECV,         // step 接收

    E_DET_BEEP_STEP_OFF = 0,
    E_DET_BEEP_STEP_ON,

    E_DET_WHQ_STEP_OFF = 0,
    E_DET_WHQ_STEP_ON,
    E_DET_WHQ_STEP_READ,

    E_DET_RTC_STEP_WRITE = 0,       // 写数据
    E_DET_RTC_STEP_READ,            // 读数据

    E_DET_OLED_STEP_OFF = 0,
    E_DET_OLED_STEP_ON,

    E_DET_MCU_COMM_SEND = 0,
    E_DET_MCU_COMM_RECV,

    E_DET_WIFI_BTN = 0,
    E_DET_FUNC_BTN,

    E_DET_DC_SEND_STEP = 0,
    E_DET_DC_RECV_STEP,

    // E_DET_MOT_STOP_IDLE = 0,
    E_DET_MOT_CW_LOWSPEED = 0,      // Mot test steps
    E_DET_MOT_CW_HIGHSPEED,
    E_DET_MOT_STOP_MID,
    E_DET_MOT_CCW_LOWSPEED,
    E_DET_MOT_CCW_HGIHSPEED,
    E_DET_MOT_STOP_END,

    E_AGDET_MOT_CW_POS = 0,         // CW 运行到边
    E_AGDET_MOT_CCW_POS,            // CCW 运行到边
    E_AGDET_MOT_WAIT,               // 
    E_AGDET_MOT_CHECK,              // 

    E_DET_IR_DRIVE_STEP = 0,        // 红外对射传感器 steps
    E_DET_IR_CHECK_STEP,

};


typedef struct _beep_test_item_t
{
    unsigned char step;             // E_DET_BEEP_OFF / E_DET_BEEP_ON
}beep_test_item_t;
typedef struct _oled_test_item_t
{
    unsigned char step;             // E_DET_OLED_STEP_OFF / 
}oled_test_item_t;
typedef struct _key_test_item_t
{
    unsigned char mask_key;
}key_test_item_t;
typedef struct _mcucomm_test_item_t
{
    unsigned char step;
}mcucomm_test_item_t;
typedef struct _whq_test_item_t
{
    unsigned char step;         // E_DET_WHQ_STEP_OFF / E_DET_WHQ_STEP_ON
}whq_test_item_t;
typedef struct _rtc_test_item_t
{
    unsigned char step;         // E_DET_RTC_STEP_WRITE / E_DET_RTC_STEP_READ
}rtc_test_item_t;
typedef struct _dc_test_item_t
{
    unsigned char step;         // E_DET_DC_SEND_STEP / E_DET_DC_RECV_STEP

    unsigned short nmh_volt;    // 电压上限
    unsigned short nml_volt;    // 电压下限
}dc_test_item_t;
typedef struct _mot_test_item_t
{
    unsigned char step;         // E_DET_MOT_STOP_IDLE / E_DET_MOT_CW_LOWSPEED / E_DET_MOT_CW_HIGHSPEED / E_DET_MOT_STOP_MID / E_DET_MOT_CCW_LOWSPEED / E_DET_MOT_CCW_HIGHSPEED
}mot_test_item_t;
typedef struct _ir_test_item_t
{
    unsigned char step;         // E_DET_IR_DRIVE_STEP / E_DET_IR_CHECK_STEP

    unsigned char bf_fj_1st;    // 防夹接收信号 1st
    unsigned char bf_fj_2nd;    // 防夹接收信号 2nd
    unsigned char bf_pwh;       // 排污盒信号
    unsigned char bf_rkgs;      // 入口光栅信号
    unsigned char bf_mpgs;      // 码盘光栅
}ir_test_item_t;

#define PCBTEST_BEEPOLED_TIMEOUT        (60)    // PCB 板级测试 BEEP_OLED超时时间
#define PCBTEST_MCU_COMM_TIMEOUT        (50)    // PCB 板级测试 esp<->mcu 通信检测
#define PCBTEST_MCU_COMM_SEND_DELAY     (10)    // 每10Sec请求一次
#define PCBTEST_DC_TIMEOUT              (10)    // PCB 板级测试 DC 电源检测
#define PCBTEST_DC_SENDCMD_TIME         (5)
#define PCBTEST_MOT_PRG_TIMEOUT         (80)    // PCB 板级测试 MOT电机测试超时时间
#define PCBTEST_MOT_STOP_TIMEOUT        (5)     // 电机停止5Sec
#define PCBTEST_MOT_RUN_DEF_TIMEOUT     (10)    // 电机停止超时时间
#define PCBTEST_IR_TIMEOUT              (30)    // PCB 板级测试 IR测试
#define PCBTEST_IR_DRVSTEP_TIME         (10)    // 10Sec内驱动一次

enum
{
    E_PCBTEST_BEEP_OLED_STEP = 0,       // Beep & OLED 切换, 内部包含按键测试
    E_PCBTEST_MCU_COMM_STEP,            // MCU通信检测
    E_PCBTEST_DC_STEP,                  // DC电源测试
    E_PCBTEST_MOT_STEP,                 // 电机震方砖测试

    E_PCBTEST_BLE_STEP,
    E_PCBTEST_WIFI_STEP,

    E_PCBTEST_IR_STEP,                  // IR 传感器测试
    E_PCBTEST_WHQ_STEP,                 // 雾化器测试
    E_PCBTEST_WEIGHT_STEP,              // 称重模块测试
    E_PCBTEST_RTC_STEP,                 // RTC测试

    E_PCBTEST_IO_SUB_STEP,              // IO SubTest

    E_PCBTEST_MAX_STEP
};  // PCB板测步骤


typedef struct _detect_param_t
{
    unsigned char test_mode;
    unsigned char test_sta;             // 
    
    unsigned char pcbtest_step;             // 
    unsigned char pcbtest_step_start;       // 
    unsigned int pcbtest_step_next_time;    // 板级测试超时时间
    unsigned char pcbtest_step_force_out;   // 强制退出

    unsigned char test_item;

    beep_test_item_t st_beep_params;    // beep 控制变量
    oled_test_item_t st_oled_params;    // oled 
    key_test_item_t st_key_params;
    whq_test_item_t st_whq_params;      // whq 控制变量
    rtc_test_item_t st_rtc_params;      // rtc 测试模块
    mcucomm_test_item_t st_mcucomm_params;          // 
    dc_test_item_t st_dc_params;        // dc 电压判定
    mot_test_item_t st_mot_params;      // mot 电机运行电流测试
    ir_test_item_t st_ir_params;        // ir 红外光栅传感器

}detect_param_t;


typedef enum
{
    E_AGING_EVT_WEIGHT = 0,         // 称重
    E_AGING_EVT_BEEP,               // 凤鸣器
    E_AGING_EVT_OLED,               // OLED
    E_AGING_EVT_MOT,                // 电机，       // 左到位 + 右到位, 两个项目
    E_AGING_EVT_WHQ,                // 雾化器
    E_AGING_EVT_RTC,                // RTC测试
    
    E_AGING_EVT_QUIT,               // 退出老化测试模式

    E_AGING_EVT_SENSOR,             // 传感器接收

    E_AGING_EVT_MAX,                // 最大
}aging_event;

typedef struct _aging_params_t
{
    unsigned char event;             // aging test step

    beep_test_item_t st_beep_params;    // beep 控制变量
    whq_test_item_t st_whq_params;      // whq 控制变量
    rtc_test_item_t st_rtc_params;      // rtc 测试模块

}aging_params_t;

#define pdMANUL                             (0)
#define pdTOOL                              (1)
#define pdWAIT                              (2)

#define pdDETECT_FALSE                      (0)
#define pdDETECT_ENTER                      (1)		//测试进入
#define pdDETECT_WAIT                       (2)		//板测模式
#define pdDETECT_CHECK                      (3)		//整机测试模式
#define pdDETECT_REPAIR                     (4)		//维修模式
#define pdDETECT_SPOT                       (5)		//抽检模式
#define pdDETECT_MAC                        (6)		//MAC修改模式
#define pdDETECT_AGEING	                    (7)		//老化模式

#define PRG_TEST_ERR                        (-1)    // 测试项目出错
#define PRG_TEST_ING                        (0)     // 测试进行中
#define PRG_TEST_OK                         (1)     // 测试完成
#define PRG_TEST_TIMEOUT                    (2)     // 测试过程超时
#define PRG_TEST_QUIT                       (3)     // 测试过程强制退出

int pk_detect_taskinit(void);

void pk_detect_taskloop(void *pvParams);

int pkdet_enter_test_mode(void);    // 进入测试模式
int pk_test_check(void *params);    // return 0:未进入测试模式; 1:进入测试模式

int pk_test_timer_start(int force);
int pk_test_timer_stop(void);

/*****Test project***********************************/
int pk_detct_aging_test(void *pvParams);    // 老化测试

/****************************************************/

/*****Test Item**************************************/
int pkdet_item_beep_manager(void *params);
int pkdet_item_oled_manager(void *params);
int pkdet_item_mcucomm_manager(void *params, unsigned short value);
int pkdet_item_dc_manager(void *params, unsigned short value);
int pkdet_item_mot_manager(void *params, unsigned short value);

int pkdet_item_ir_manager(void *params, unsigned short value);
/****************************************************/

/*****测试变量相关函数**********************************/
detect_param_t *pk_get_detectvalue(void);
int pk_is_test_mode(void);
int pk_get_detect_sta(void);                // 获取测试状态
int pk_set_detect_sta(unsigned char sta);
int pkdet_get_pcbstep(void);
/****************************************************/

/*****设置Agingtest的时间******************************/

/****************************************************/

int pk_send_itemtest_event(drv_test_item event, int value, char *pdata, unsigned char bfInt);
int pk_send_aging_event(aging_event event, int value, char *pdata, unsigned char bfInt);

#ifdef __cplusplus
}
#endif


#endif


