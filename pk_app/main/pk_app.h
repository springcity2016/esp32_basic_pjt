
#ifndef __PK_APP_H_
#define __PK_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "serial_comm_table.h"


/*******step config********************************************/
#define SHAKE_DEF_TIMES         (3)
#define SHAKE_DEF_1ST_DIR       (E_CW)
#define SHAKE_DEF_TIMEOUT       (30)        // timeout

#define SHAKE_DEF_CW_SPEED      (DEV_PHY_MOT_DEF_SPEED)
#define SHAKE_DEF_CW_STEP       (DEV_PHY_MOT_DEF_STEP)
#define SHAKE_DEF_CW_STEP_MAX   (DEV_PHY_MOT_DEF_STEP_MAX)
#define SHAKE_DEF_CW_OVER_TIME  (DEV_PHY_MOT_DEF_OVER_TIME)

#define SHAKE_DEF_CCW_SPEED      (DEV_PHY_MOT_DEF_SPEED)
#define SHAKE_DEF_CCW_STEP       (DEV_PHY_MOT_DEF_STEP)
#define SHAKE_DEF_CCW_STEP_MAX   (DEV_PHY_MOT_DEF_STEP_MAX)
#define SHAKE_DEF_CCW_OVER_TIME  (DEV_PHY_MOT_DEF_OVER_TIME)

#define RUN_STEP_DEF_TIMEOUT    (60)    // step 超时

#define STILL_STEP_DEF_TIME     (300)   // 静止时间
/**************************************************************/


#define MOT_SHAKE_STEP_PARAMS_INIT  {   \
    .shake_times = SHAKE_DEF_TIMES, \
    .start_dir = SHAKE_DEF_1ST_DIR, \
    .shake_timeout = SHAKE_DEF_TIMEOUT, \
    .shake_cw_cfg = NEW_RUN_CFG(E_SANDMOT_STEP_T, E_SANDMOT_POS_NONE, E_CW, SHAKE_DEF_CW_SPEED, SHAKE_DEF_CW_STEP, SHAKE_DEF_CW_STEP_MAX, SHAKE_DEF_CW_OVER_TIME), \
    .shake_ccw_cfg = NEW_RUN_CFG(E_SANDMOT_STEP_T, E_SANDMOT_POS_NONE, E_CCW, SHAKE_DEF_CCW_SPEED, SHAKE_DEF_CCW_STEP, SHAKE_DEF_CCW_STEP_MAX, SHAKE_DEF_CCW_OVER_TIME), \
}
#define MOT_RUN_STEP_PARAMS_INIT    {   \
    .step_timeout = RUN_STEP_DEF_TIMEOUT, \
    .mot_run_cfg = NEW_RUN_CFG = ,  \
}
#define MOT_STILL_STEP_PARAMS_INIT  {   \
    .still_time = STILL_STEP_DEF_TIME,  \
}

// 所有progress由细化step组成， 细化step: 运行，抖动，静止
typedef enum
{
    E_MOT_PRG_NONE = 0,     // 无状态   // 
    E_MOT_PRG_INIT,         // 电机寻址 // 
    E_MOT_PRG_RESET,        // 复位     // 
    E_MOT_PRG_SHOVEL_SHIT,  // 铲屎     // 
    E_MOT_PRG_POUR_SAND,    // 倾沙     // 
    E_MOT_PRG_SUSPEND,      // 暂停     // 

    E_MOT_PRG_MAX           // 电机最大的 progresss
}mot_progress_e;

typedef enum
{
    E_MOT_INIT_EVT = 0,     // 筒位置初始化

    /***************************************************/    
    E_MOT_RUN_CTRL_EVT,     // 具体电机运行控制  
    E_MOT_SHAKE_EVT,        // 电机晃动
    E_MOT_STILL_EVT,        // 保持静止状态
    /***************************************************/    

    E_MOT_CHECK_EVT,        // 获取电机信号位置
    E_MOT_RESULT_EVT,       // 结果查询
    E_MOT_MAX_EVT,
}mot_evt_e;

typedef enum
{
    E_MOT_STEP_STA_NONE = 0,
    E_MOT_STEP_STA_START,           // step起始状态
    E_MOT_STEP_STA_CHECK,           // step运行状态
    E_MOT_STEP_STA_RESULT,          // step
    E_MOT_STEP_STA_ING,
    E_MOT_STEP_STA_FINISHED,        // step完成

    E_MOT_STEP_STA_TIMEOUT,         // timeout

    E_MOT_STEP_STA_FORCE_QUIT,      // 强制退出当前状态

    E_MOT_STEP_STA_MAX,
}mot_step_sta_e;

typedef struct _mot_run_step_t
{   
    unsigned int step_timeout;          // 超时时间
    sandmot_runcfg_t mot_run_cfg;       // 电机运行参数     
}mot_run_step_t;    // 

typedef struct _mot_shake_step_t
{
    unsigned char shake_times;          // 晃动次数
    unsigned char start_dir;            // 起始方向，     0:CW, 1:CCW
    unsigned int shake_timeout;         // 抖动超时时间

    sandmot_runcfg_t shake_cw_cfg;      // cw方向  运行参数
    sandmot_runcfg_t shake_ccw_cfg;     // CCW方向 运行参数
}mot_shake_step_t;       // 电机晃动 step 参数

typedef struct _mot_still_step_t
{
    unsigned int still_time;            // 静止时间
}mot_still_step_t;

typedef enum
{
    E_MOTEVT_STEP_0 = 0,           // 起始step
    E_MOTEVT_STEP_1,
    E_MOTEVT_STEP_2,

    E_MOTEVT_STEP_MAX,

    E_MOT_RECV_DATA = 0xFF          // 接受step
}mot_evt_step_e;

typedef enum
{
    E_SCRAM_NONE_EVT = 0,           // 不需要即停
    E_MOT_TIMEOUT_EVT,              // 电机运行超时 event
    E_CLAMP_EVT,                    // 防夹信号 event
    E_MOT_OVERCURT_EVT,             // 电机过流
    E_YIWAI_DAOBIAN_EVT,            // 意外到边
    E_JIBU_WARNING_EVT,             // 记步异常
    E_WEIGHT_ADD_EVT,               // 称数据增加
    E_YIDONG_SENSOR_EVT,            // 红外移动侦测
    E_DOOR_SENSOR_EVT,              // 门红外对管
    E_PAIFEIHE_EVT,                 // 排废盒微动
    E_NEITONG_LOCK_EVT,             // 内筒锁止信号
    E_USER_STOP_EVT,                // 用户停止信号

    E_SCRAM_MAX_EVT
}scram_evt_e;     // 电机急停判定

typedef struct _pkapp_params_t
{
    unsigned char sensor_sta_lvl;   // 传感器状态等级

    int mot_init_finish;// 电机初始化完成标志, 0:未完成, 1:已完成
    int mot_init_step;  // 电机初始化step

    int dev_state;          // 
    int dev_progress;       // 
    int dev_prg_step;       // progress 对应下的 具体细化step
    int dev_prg_step_sta;   // step 的 运行状态

    mot_still_step_t still_step_params;
    mot_shake_step_t shake_step_params;
    mot_run_step_t run_step_params;

    sandmot_runcfg_t esp_mot_runcfg;            // esp32端控制变量
    sandmot_runcfg_t latest_mot_runcfg;         // 最新一次电机配置数据
    sandmot_real_result_t mot_run_real;         // 底下电机运行数据
}pkapp_params_t;

typedef enum
{
    E_APP_TIMER_RUN_TIMEOUT_STEP = 0,
    E_APP_TIMER_RUN_CHECK_STEP,
    
    E_APP_TIMER_STILL_TIMEOUT_STEP,     // 
    E_APP_TIMER_STILL_CHECK_STEP,       // 

    E_APP_TIMER_SHAKE_TIMEOUT_STEP,     // 
    E_APP_TIMER_SHAKE_CHECK_STEP,       //

    E_APP_TIMER_MAX
}app_timer_e;


int pk_app_init(void);


int pk_app_taskloop(void *params);


int pkapp_timer_start(unsigned char force);
int pkapp_timer_stop(void);

int pkapp_get_now_step(void);
int pkapp_get_now_step_sta(void);
int pkapp_set_mot_event(mot_evt_e event, int value, const unsigned char *pcdata, int bfInt);
int pkapp_set_mot_result(int value, const unsigned char *pcdata, int bfInt);
int pkapp_set_mot_step_sta(int step, int sta, const unsigned char *pcdata, int bfInt);


int pkapp_compare_mot_runsta(unsigned char finish, unsigned char start_pos, unsigned char end_pos, sandmot_real_t *p_real_value, sandmot_runcfg_t *p_run_cfg);        // 判定电机有没有运行完成 // 0,运行中; 1,完成; -1,急停
int compare_run_configdata(const sandmot_runcfg_t *p_src, const sandmot_runcfg_t *p_dst);   // 0: 相同, -1不同




#ifdef __cplusplus
}
#endif


#endif



