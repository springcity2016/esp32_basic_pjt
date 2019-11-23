

#define __PK_APP_C_

#include "pk_app.h"

#include "string.h"
#include "stdlib.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "queue.h"
#include "petkit_clog.h"
#include "pk_serial.h"
#include "pk_T3_driver.h"
#include "user_lib.h"
#include "user_config.h"
#include "serial_comm_table.h"

pkapp_params_t gst_app_params;

static int _pet_check_in(void);
static int _pet_check_out(void);

static int _sensor_lvl_1st_check(void);         // -1:异常状态; 0:正常状态
static int _sensor_lvl_2nd3rd_check(void);      // -1:异常状态; 0:正常状态

static int _door_signal_check(unsigned int min_sec, unsigned int max_sec);  // 门光栅捕获信号[min, max]秒内
static int _yidong_signal_check(void);      // 移动侦测信号, 返回持续时间，0:侦测信号无变化
typedef enum
{
    E_WEIGHT_STA_NONE = 0,
    E_WEIGHT_STA_NORMAL,            // 1. 变化符合预期
    E_WEIGHT_STA_OVER,              // 2. 称数据变化超预期(超重/减重)
    E_WEIGHT_STA_NO_ACTIVE,         // 3:无有效值(长时间大幅抖动)
    E_WEIGHT_STA_NO_CHANGE,         // 4:数据无变化

    E_WEIGHT_STA_MAX
}weight_sig_sta;
static weight_sig_sta _weight_sig_check(void);

#define MOTEVENT_QUEUE_MAX     (15)
static xQueueHandle xQue_Mot_Event;                               // 接受到数据驱动
static xQueueHandle xQue_mot_result;            // 用于传输电机通信结果

static int mot_warning_check(void);
static int sandmot_work_manager(void *params);


static esp_timer_handle_t app_timer;
#define APP_TIMER_DEF_TIME_US                 (1000000)       // 单位us
#define MOT_STA_CHECK_DEF_TIME                (4)             // 单位:sec, 电机状态查询时间

static timer_tick_t mst_app_timer[E_APP_TIMER_MAX] = { 0 };

static int _mot_run_step_manager(int sta, void *params);
static int _mot_still_step_manager(int sta, void *params);
static int _mot_shake_step_manager(int sta, void *params);

int pk_app_init(void)
{
    memset(&gst_app_params, 0, sizeof(gst_app_params));

    gst_app_params.esp_mot_runcfg = cst_mot_runcfg_init;
    
    xQue_Mot_Event = xQueueCreate(MOTEVENT_QUEUE_MAX, sizeof(sysevent_t));
    if (xQue_Mot_Event == NULL) {
        pkLOG_ERR("app task motor event queue is NULL!\n");
    }
    xQue_mot_result = xQueueCreate(MOTEVENT_QUEUE_MAX, sizeof(sysevent_t));
    if (xQue_mot_result == NULL) {
        pkLOG_ERR("app task motor event queue is NULL!\n");
    }


    return 0;
}


int pk_app_taskloop(void *params)
{
    short taskloop_sta = 1;

    while(taskloop_sta) 
    {

        if (gst_app_params.mot_init_finish == 0) {      // 电机初始化未完成
            if (gst_app_params.mot_init_step == E_MOTEVT_STEP_0) {
                pkapp_set_mot_event(E_MOT_INIT_EVT, 0, NULL, 0);
            }
        } else {    // 电机初始化已完成
            _pet_check_in();        // 宠物进入检测
            _pet_check_out();       // 宠物出去检测
        }

        sandmot_work_manager(NULL);     // 沙筒动作管理

        vTaskDelay(100/portTICK_PERIOD_MS);
    }

    return 0;
}

static void _pkapp_timer_sec_cb(void *params) 
{
     // Run step 运行管理
    if (Is_timer_tick_up(&mst_app_timer[E_APP_TIMER_RUN_TIMEOUT_STEP], sys_get_os_runtime_sec())) {
        pkapp_set_mot_step_sta(E_MOT_RUN_CTRL_EVT, E_MOT_STEP_STA_TIMEOUT, NULL, 1);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_TIMEOUT_STEP]);
    }
    if (Is_timer_tick_up(&mst_app_timer[E_APP_TIMER_RUN_CHECK_STEP], sys_get_os_runtime_sec())) {
        pkapp_set_mot_step_sta(E_MOT_RUN_CTRL_EVT, E_MOT_STEP_STA_CHECK, NULL, 1);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_CHECK_STEP]);
    }
     // Still Step 运行管理
    if (Is_timer_tick_up(&mst_app_timer[E_APP_TIMER_STILL_TIMEOUT_STEP], sys_get_os_runtime_sec())) {
        pkapp_set_mot_step_sta(E_MOT_STILL_EVT, E_MOT_STEP_STA_TIMEOUT, NULL, 1);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_TIMEOUT_STEP]);
    }
    if (Is_timer_tick_up(&mst_app_timer[E_APP_TIMER_STILL_CHECK_STEP], sys_get_os_runtime_sec())) {
        pkapp_set_mot_step_sta(E_MOT_STILL_EVT, E_MOT_STEP_STA_CHECK, NULL, 1);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_CHECK_STEP]);
    }    
     // Shake Step 时间管理
    if (Is_timer_tick_up(&mst_app_timer[E_APP_TIMER_SHAKE_TIMEOUT_STEP], sys_get_os_runtime_sec())) {
        pkapp_set_mot_step_sta(E_MOT_SHAKE_EVT, E_MOT_STEP_STA_TIMEOUT, NULL, 1);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_TIMEOUT_STEP]);
    }
    if (Is_timer_tick_up(&mst_app_timer[E_APP_TIMER_SHAKE_CHECK_STEP], sys_get_os_runtime_sec())) {
        pkapp_set_mot_step_sta(E_MOT_SHAKE_EVT, E_MOT_STEP_STA_CHECK, NULL, 1);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_CHECK_STEP]);
    }         

}

int pkapp_timer_start(unsigned char force)
{
    int _ret = 0;
    if (force) {
        _ret = pkapp_timer_stop();   // stop timer;
        if (_ret) { return _ret; }
        esp_timer_create_args_t _timer_conf = {
            .callback = _pkapp_timer_sec_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "app_timer"
        };
        esp_err_t _err = esp_timer_create(&_timer_conf, &app_timer);
        if (_err) { return _err; }
        _ret = esp_timer_start_periodic(app_timer, APP_TIMER_DEF_TIME_US);
    }

    return _ret;
}
int pkapp_timer_stop(void)
{
    int _ret = 0;
    if (app_timer) {
        _ret = esp_timer_stop(app_timer);
        if (_ret) { return _ret; }
        _ret = esp_timer_delete(app_timer);
    } 
    return _ret;
}

int pkapp_get_now_step(void)
{
    return gst_app_params.dev_prg_step;
}
int pkapp_get_now_step_sta(void)
{
    return gst_app_params.dev_prg_step_sta;
}
int pkapp_set_mot_event(mot_evt_e event, int value, const unsigned char *pcdata, int bfInt)
{
    int _ret = 0;
    if (bfInt) {    // 中断方式
        _ret = sys_xQueSendToBackFromISR(xQue_Mot_Event, event, value, (char *)pcdata);
    } else {        // 非中断方式
        _ret = sys_xQueSendToBack(xQue_Mot_Event, event, value, (char *)pcdata);
    }
    return _ret;
}
int pkapp_set_mot_step_sta(int step, int sta, const unsigned char *pcdata, int bfInt)
{
    int _ret = 0;
    if (bfInt) {    // 中断方式
        _ret = sys_xQueSendToBackFromISR(xQue_Mot_Event, step, sta, (char *)pcdata);
    } else {        // 非中断方式
        _ret = sys_xQueSendToBack(xQue_Mot_Event, step, sta, (char *)pcdata);
    }
    return _ret;   
}

int pkapp_set_mot_result(int value, const unsigned char *pcdata, int bfInt)
{
    int _ret = 0;
    if (bfInt) {    // 中断方式
        _ret = sys_xQueSendToBackFromISR(xQue_mot_result, E_MOT_RESULT_EVT, value, (char *)pcdata);
    } else {        // 非中断方式
        _ret = sys_xQueSendToBack(xQue_mot_result, E_MOT_RESULT_EVT, value, (char *)pcdata);
    }
    return _ret;   
}

int pkapp_compare_mot_runsta(unsigned char finish, unsigned char start_pos, unsigned char end_pos, sandmot_real_t *p_real_value, sandmot_runcfg_t *p_run_cfg)        // 判定电机有没有运行完成 // 0,运行中; 1,完成; -1,急停
{
    int _ret = 0;

    if (finish) { return 1; }

    if ((p_real_value == NULL) || (p_run_cfg == NULL)) {
        return _ret;
    }
    if (p_run_cfg->type == E_SANDMOT_POS_T) {              // 
        if (p_run_cfg->pos == E_SANDMOT_POS_NONE) {         // 到边
            if (((start_pos == 0) || (end_pos == 0)) 
              && (p_real_value->u16_mot_stepnum >= p_run_cfg->u16_stepnum)) {         // 到边
                _ret = 1;
            } else if (p_real_value->u16_mot_stepnum >= p_run_cfg->u16_maxstepnum) {  // 大于step
                _ret = 1;
            } else if (p_real_value->u16_mot_runtime >= p_run_cfg->u16_overtime) {    // 运行时间超时
                _ret = 1;
            } 
        } else if (p_run_cfg->pos == E_SANDMOT_POS_START) { // 到start点
            if ((start_pos == 0) 
              && (p_real_value->u16_mot_stepnum >= p_run_cfg->u16_stepnum)) {         // 到边
                _ret = 1;
            } else if (p_real_value->u16_mot_stepnum >= p_run_cfg->u16_maxstepnum) {  // 大于step
                _ret = 1;
            } else if (p_real_value->u16_mot_runtime >= p_run_cfg->u16_overtime) {    // 运行时间超时
                _ret = 1;
            }
        } else if (p_run_cfg->pos == E_SANDMOT_POS_END) {
            if ((end_pos == 0) 
              && (p_real_value->u16_mot_stepnum >= p_run_cfg->u16_stepnum)) {         // 到边
                _ret = 1;
            } else if (p_real_value->u16_mot_stepnum >= p_run_cfg->u16_maxstepnum) {  // 大于step
                _ret = 1;
            } else if (p_real_value->u16_mot_runtime >= p_run_cfg->u16_overtime) {    // 运行时间超时
                _ret = 1;
            }
        } else if ((p_real_value->mot_sta != E_STOP)
                    && (p_real_value->mot_sta != p_run_cfg->mot_sta)) {
            _ret = -1;
        }
    } else if (p_run_cfg->type == E_SANDMOT_STEP_T) {      // 
        if (p_real_value->u16_mot_stepnum >= p_run_cfg->u16_stepnum) {  // 大于step
            _ret = 1;
        } else if (p_real_value->u16_mot_runtime >= p_run_cfg->u16_overtime) {    // 运行时间超时
            _ret = 1;
        } else if ((p_real_value->mot_sta != E_STOP)
                    && (p_real_value->mot_sta != p_run_cfg->mot_sta)) {
            _ret = -1;
        }
    } else if (p_run_cfg->type == E_SANDMOT_SCRAM_T) {     // 
        if (p_real_value->mot_sta == E_STOP) {
            _ret = 1;
        } else if ((p_real_value->mot_sta != E_STOP)
                    && (p_real_value->mot_sta != p_run_cfg->mot_sta)) {
            _ret = -1;
        }
    }
    return _ret;   
}

int compare_run_configdata(const sandmot_runcfg_t *p_src, const sandmot_runcfg_t *p_dst)   // 0:不同, 1:相同, -1:参数出错
{
    int _ret = 0;
    if ((p_src == NULL) || (p_dst == NULL)) {
        return -1;
    }
    if ((p_src->type == p_dst->type)
     && (p_src->pos == p_dst->pos)
     && (p_src->mot_sta == p_dst->mot_sta)
     && (p_src->mot_speed == p_dst->mot_speed)
     && (p_src->u16_maxstepnum == p_dst->u16_maxstepnum)
     && (p_src->u16_overtime == p_dst->u16_overtime)
     && (p_src->u16_stepnum == p_dst->u16_stepnum)) {
        _ret = 1;
    }
    return _ret;
}

static int _pet_check_in(void)
{
    int _ret = 0;
    int _func_ret = 0;
    int _door_sig_sta = 0;
    int _yidong_sig_time = 0;
    weight_sig_sta _weight_sig_sta = 0;

    _func_ret = _sensor_lvl_1st_check();
    if (_func_ret != 0) {
        // 异常状态/人工介入
    }
    _func_ret = _sensor_lvl_2nd3rd_check();
    if (_func_ret != 0) {
        // 仅记录,故障传感器信号不作为判定依据
    }
    _door_sig_sta = _door_signal_check(0, 0);
    _yidong_sig_time = _yidong_signal_check();
    _weight_sig_sta = _weight_sig_check();

    if (_weight_sig_sta == E_WEIGHT_STA_OVER) {                 // 重量超出范围
        if (_yidong_sig_time != 0) {     // 有变化
            // 用户铲沙 || 宠物出
            // 称异常
            // 宠物活动干扰
        } else {    // 无变化
            // 设备倾倒
            // 内筒干涉
            // 手动铲沙 || 宠物出
        }
    } else if (_weight_sig_sta == E_WEIGHT_STA_NO_CHANGE) {     // 称数据无变化
        if (_yidong_sig_time != 0) {     // 信号持续变化
            // 宠物在筒内,称异常
            // 宠物过轻
            // 内筒干涉
        } else {    // 无变化
            // 无事件
        }
    } else if (((_weight_sig_sta == E_WEIGHT_STA_NO_ACTIVE) || (_weight_sig_sta == E_WEIGHT_STA_NORMAL)) && (_yidong_sig_time != 0)) {     // 称数据无有效值(长时间大幅抖动)
        // B时间过短
            // 手动加沙 -> 内筒晃动确认 -> 进入确认
        // B时间过长
            // 进入确认
            // 多次手动加沙 -> 内筒晃动确认 -> 进入确认
    } else if ((_weight_sig_sta == E_WEIGHT_STA_NORMAL) && (_door_sig_sta == 1))  {        // 称数据变化符合预期(基数增加[min, max]范围)
        // 手动加沙 -> 内筒晃动确认 -> 进入确认
        // 进入确认
    } else if (_weight_sig_sta == E_WEIGHT_STA_NORMAL) {        // 仅仅是符合要求
        // 内筒干涉 -> 内筒晃动确认 -> 宠物进入,且移动侦测故障
        // 宠物进入,且移动侦测故障
    } else if /*(*/(_weight_sig_sta == E_WEIGHT_STA_NO_ACTIVE)
            /* || ((_weight_sig_check == E_WEIGHT_STA_NO_ACTIVE) && (_door_sig_sta == 1)))*/ {
        // 称异常
        // 内筒干涉
        // 宠物进入,且移动侦测故障
        // 用户铲沙,且移动侦测故障
    }
    
    return _ret;
}

static int _pet_check_out(void)
{
    int _ret = 0;
    int _func_ret = 0;
    int _door_sig_sta = 0;
    int _yidong_sig_time = 0;
    weight_sig_sta _weight_sig_sta = 0;

    _func_ret = _sensor_lvl_1st_check();
    if (_func_ret != 0) {
        // 异常状态/人工介入
    }
    _func_ret = _sensor_lvl_2nd3rd_check();
    if (_func_ret != 0) {
        // 仅记录,故障传感器信号不作为判定依据
    }

    _door_sig_sta = _door_signal_check(0 ,0);
    _yidong_sig_time = _yidong_signal_check();
    _weight_sig_sta = _weight_sig_check();
    if ((_door_sig_sta == 1) && (_yidong_sig_time == 0) && (_weight_sig_sta == E_WEIGHT_STA_NONE)) {    // 仅A条件满足
        // 外部干扰
    } else if (_weight_sig_sta == E_WEIGHT_STA_OVER) {
        // 设备倾倒
        // 宠物状态未知，且移动失败
        // 内筒干涉
    } else if (_weight_sig_sta == E_WEIGHT_STA_NO_ACTIVE) {
        if (_yidong_sig_time != 0) {
            // 宠物在筒内
        } else {
            // 称异常
            // 内筒干涉
            // 宠物进入,且移动侦测失效
            // 用户铲沙,且移动侦测失效
        }
    } else if (_weight_sig_sta == E_WEIGHT_STA_NO_CHANGE) {
        // 宠物睡着 -> 内筒晃动确认
        // 内筒干涉
        // 称异常
        // 宠物在筒,且移动侦测失效
    } else if (_weight_sig_sta == E_WEIGHT_STA_NORMAL) {
        if (_yidong_sig_time != 0) {
            // 出筒确认
        } else if (_door_sig_sta == 1) {
            // 出筒确认
            // 宠物出筒,且移动侦测失效
        } else {    // 仅仅符合这个条件
            // 内筒干涉
            // 宠物出筒,且移动侦测失效
            // 用户铲沙,且移动侦测失效
        }
    }


    return _ret;
}

static int _sensor_lvl_1st_check(void)         // -1:异常状态; 0:正常状态
{
    int _ret = 0;

    return _ret;
}

static int _sensor_lvl_2nd3rd_check(void)      // -1:异常状态; 0:正常状态
{
    int _ret = 0;

    return _ret;
}

static weight_sig_sta _weight_sig_check(void)
{
    weight_sig_sta _ret_sta = E_WEIGHT_STA_NORMAL;


    return _ret_sta;
}
static int _door_signal_check(unsigned int min_sec, unsigned int max_sec)  // 门光栅捕获信号[min, max]秒内
{
    int _ret = 0;


    return _ret;
}
static int _yidong_signal_check(void)      // 移动侦测信号, 返回持续时间，0:侦测信号无变化
{
    int _ret = 0;

    return _ret;
}

static int mot_warning_check(void)          // 沙筒异常判定,用于急停判定
{
    int _ret = 0;


    return _ret;
}


static int sandmot_work_manager(void *params)
{
    int _ret = 0;
    sysevent_t stvalue = { 0, 0, NULL };
    int cmd = 0;
    int value = 0;
    int dev_state = 0;
    int _func_ret = 0;

    int dev_progress = gst_app_params.dev_progress;
    int dev_prg_step = gst_app_params.dev_prg_step;
    int dev_prg_step_sta = gst_app_params.dev_prg_step_sta;

    if (xQueueReceive(xQue_Mot_Event, &stvalue, 1/(portTICK_PERIOD_MS * 10))) {
        cmd = stvalue.cmd;
        value = stvalue.value;      // value在此event函数中极为step
        
        switch (cmd) 
        {
        case E_MOT_RUN_CTRL_EVT:
            {
                _func_ret = _mot_run_step_manager(value, stvalue.msgptr);
                if (value == E_MOT_STEP_STA_RESULT) {   // 事件结果进行驱动

                }
                if (stvalue.msgptr) { free(stvalue.msgptr); stvalue.msgptr = NULL; }
            }
            break;
        case E_MOT_STILL_EVT:
            {
                _func_ret = _mot_still_step_manager(value, stvalue.msgptr);
                if (value == E_MOT_STEP_STA_RESULT) {   // 事件结果进行驱动

                }
                if (stvalue.msgptr) { free(stvalue.msgptr); stvalue.msgptr = NULL; }
            }
            break;
        case E_MOT_SHAKE_EVT:
            {
                _func_ret = _mot_shake_step_manager(value, stvalue.msgptr);
                if (value == E_MOT_STEP_STA_RESULT) {   // 事件结果进行驱动

                }
                if (stvalue.msgptr) { free(stvalue.msgptr); stvalue.msgptr = NULL; }
            }
            break;            
        default:
             break;
        }

    }


    return _ret;
}



static int _mot_run_step_manager(int sta, void *params)
{
    int _ret = 0;
    int value = sta;
    int _func_ret = 0;
    int _retry_cnt = 3;
    int _block = 1;         // 阻塞控制
    gst_app_params.dev_prg_step = E_MOT_RUN_CTRL_EVT;
    if (value == E_MOT_STEP_STA_START) {
        mot_run_step_t *p_data = (mot_run_step_t *)(params);
_retry_send:        // 阻塞控制保证控制参数实际到达
        _retry_cnt--;
        pkmcu_set_mot_run(p_data->mot_run_cfg, 0);
        memcpy(&(gst_app_params.esp_mot_runcfg), p_data, sizeof(mot_run_step_t));
        memcpy(&(gst_app_params.run_step_params), p_data, sizeof(mot_run_step_t));
        gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_START;
        if ((_block == 1) && (_retry_cnt >= 0)) {       // 阻塞控制
            _func_ret = pkmcu_get_exphydata(E_EXPHY_MOTRUN_CFG, E_INFO_BLK, 2, 0);      // 等待测试
            if (_func_ret == EXPHY_UPSTA_NEW) {
                // 判定config 程序是否和设置一致
                drv_physta_t _status;
                get_phy_status(&_status);
                _func_ret = compare_run_configdata(&(_status.st_motrun_cfg_sta), &(gst_app_params.esp_mot_runcfg));
                if (_func_ret == 1) {           // 相同

                } else if (_func_ret == 0) {    // 不同
                    goto _retry_send;
                }
            } else {    // 阻塞超时
                pkLOG_ERR(" Timeout! run step start step_sta! %d \n", __LINE__);
            }
        }
        set_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_TIMEOUT_STEP], sys_get_os_runtime_sec(), p_data->step_timeout, 0);
        set_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_CHECK_STEP], sys_get_os_runtime_sec(), MOT_STA_CHECK_DEF_TIME, 0);

    } else if (value == E_MOT_STEP_STA_CHECK) {
        pk_esp_mcu_comm(COMM_CMD_MOT_RUN_RT, COMM_READ_CMD, 0, NULL, 0, 0);
        set_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_CHECK_STEP], sys_get_os_runtime_sec(), MOT_STA_CHECK_DEF_TIME, 0);
    } else if (value == E_MOT_STEP_STA_RESULT) {
        sandmot_real_result_t *p_data = (sandmot_real_result_t *)(params);
        _ret = pkapp_compare_mot_runsta(p_data->result, p_data->mot_pos_start, p_data->mot_pos_end, &(p_data->real_value), &(gst_app_params.esp_mot_runcfg));        // 判定电机有没有运行完成
        if (_ret == 1) {       // 
            gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_FINISHED;
            disable_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_TIMEOUT_STEP]);
            disable_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_CHECK_STEP]);
        } else if (_ret == -1) {
            pkLOG_INFO(" motor direction is different! %d \n", __LINE__);
            gst_app_params.esp_mot_runcfg.type = E_SANDMOT_SCRAM_T;
            gst_app_params.esp_mot_runcfg.mot_sta = E_STOP;
            pkmcu_set_mot_run(gst_app_params.esp_mot_runcfg, 0);
        }
        // } else if (value == E_MOT_STEP_STA_ING) {
    } else if (value == E_MOT_STEP_STA_TIMEOUT) {   // 此阶段超时
        pkLOG_INFO(" run contorl step, timeout! %d \n", __LINE__);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_TIMEOUT_STEP]);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_RUN_CHECK_STEP]);
        // 判断当前电机状态
    }
    return _ret;
}
static int _mot_still_step_manager(int sta, void *params)
{
    int _ret = 0;
    int value = sta;
    int _func_ret = 0;
    int _retry_cnt = 3;
    int _block = 1;         // 阻塞控制
    gst_app_params.dev_prg_step = E_MOT_STILL_EVT;
    if (value == E_MOT_STEP_STA_START) {
        mot_still_step_t *p_data = (mot_still_step_t *)(params);
_retry_send:                // 阻塞控制保证控制参数实际到达
        _retry_cnt--;
        gst_app_params.esp_mot_runcfg.type = E_SANDMOT_SCRAM_T;
        gst_app_params.esp_mot_runcfg.mot_sta = E_STOP;
        pkmcu_set_mot_run(gst_app_params.esp_mot_runcfg, 0);
        memcpy(&(gst_app_params.still_step_params), p_data, sizeof(mot_still_step_t));
        gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_START;
        if ((_block == 1) && (_retry_cnt >= 0)) {
            _func_ret = pkmcu_get_exphydata(E_EXPHY_MOTRUN_CFG, E_INFO_BLK, 2, 0);      // 等待测试
            if (_func_ret == EXPHY_UPSTA_NEW) {
                // 判定config 程序是否和设置一致
                drv_physta_t _status;
                get_phy_status(&_status);
                _func_ret = compare_run_configdata(&(_status.st_motrun_cfg_sta), &(gst_app_params.esp_mot_runcfg));
                if (_func_ret == 1) {           // 相同

                } else if (_func_ret == 0) {    // 不同
                    goto _retry_send;
                }
            } else {    // 阻塞超时
                pkLOG_ERR(" Timeout! run step start step_sta! %d \n", __LINE__);
            }
        }
        set_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_TIMEOUT_STEP], sys_get_os_runtime_sec(), p_data->still_time, 0);
        set_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_CHECK_STEP], sys_get_os_runtime_sec(), MOT_STA_CHECK_DEF_TIME, 0);
    
    } else if (value == E_MOT_STEP_STA_CHECK) {
        pk_esp_mcu_comm(COMM_CMD_MOT_RUN_RT, COMM_READ_CMD, 0, NULL, 0, 0);
        set_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_CHECK_STEP], sys_get_os_runtime_sec(), MOT_STA_CHECK_DEF_TIME, 0);
    } else if (value == E_MOT_STEP_STA_RESULT) {
        sandmot_real_result_t *p_data = (sandmot_real_result_t *)(params);
        _ret = pkapp_compare_mot_runsta(p_data->result, p_data->mot_pos_start, p_data->mot_pos_end, &(p_data->real_value), &(gst_app_params.esp_mot_runcfg));        // 判定电机有没有运行完成
        if (_ret == 1) {
            gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_FINISHED;
            disable_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_TIMEOUT_STEP]);
            disable_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_CHECK_STEP]);
        } else if (_ret == -1) {
            pkLOG_INFO(" motor direction is different! %d \n", __LINE__);
            gst_app_params.esp_mot_runcfg.type = E_SANDMOT_SCRAM_T;
            gst_app_params.esp_mot_runcfg.mot_sta = E_STOP;
            pkmcu_set_mot_run(gst_app_params.esp_mot_runcfg, 0);
        }
    } else if (value == E_MOT_STEP_STA_TIMEOUT) {
        pkLOG_INFO(" still step, timeout! %d \n", __LINE__);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_TIMEOUT_STEP]);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_CHECK_STEP]);                        
    }

    return _ret;
}
static int _mot_shake_step_manager(int sta, void *params)
{
    int _ret = 0;
    int value = sta;
    int _func_ret = 0;
    int _retry_cnt = 3;
    int _block = 1;         // 阻塞控制
    gst_app_params.dev_prg_step = E_MOT_SHAKE_EVT;
    if (value == E_MOT_STEP_STA_START) {
        mot_shake_step_t *p_data = (mot_shake_step_t *)(params);
_retry_send:                // 阻塞控制保证控制参数实际到达
        _retry_cnt--;
        gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_START;
        memcpy(&(gst_app_params.shake_step_params), p_data, sizeof(mot_shake_step_t));
        if (p_data->shake_times != 0) {
            if (p_data->start_dir == E_CW) {
                pkmcu_set_mot_run(gst_app_params.shake_step_params.shake_cw_cfg, 0);
            } else if (p_data->start_dir == E_CCW) {
                pkmcu_set_mot_run(gst_app_params.shake_step_params.shake_ccw_cfg, 0);                                
            } else {
                pkLOG_ERR(" error! shake step, first dir is stop! %d \n", __LINE__);
            }

            if ((_block == 1) && (_retry_cnt >= 0)) {
                _func_ret = pkmcu_get_exphydata(E_EXPHY_MOTRUN_CFG, E_INFO_BLK, 2, 0);      // 等待测试
                if (_func_ret == EXPHY_UPSTA_NEW) {
                    // 判定config 程序是否和设置一致
                    drv_physta_t _status;
                    get_phy_status(&_status);
                    _func_ret = compare_run_configdata(&(_status.st_motrun_cfg_sta), &(gst_app_params.esp_mot_runcfg));
                    if (_func_ret == 1) {           // 相同

                    } else if (_func_ret == 0) {    // 不同
                        goto _retry_send;
                    }
                } else {    // 阻塞超时
                    pkLOG_ERR(" Timeout! run step start step_sta! %d \n", __LINE__);
                }
            }

            set_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_TIMEOUT_STEP], sys_get_os_runtime_sec(), p_data->shake_timeout, 0);
            set_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_CHECK_STEP], sys_get_os_runtime_sec(), MOT_STA_CHECK_DEF_TIME, 0);
        } else {
            gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_FINISHED;
        }

        // 此处可添加阻塞查询指令是否到达 - 待添加
    } else if (value == E_MOT_STEP_STA_CHECK) {
        pk_esp_mcu_comm(COMM_CMD_MOT_RUN_RT, COMM_READ_CMD, 0, NULL, 0, 0);
        set_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_CHECK_STEP], sys_get_os_runtime_sec(), MOT_STA_CHECK_DEF_TIME, 0);
    } else if (value == E_MOT_STEP_STA_RESULT) {
            sandmot_real_result_t *p_data = (sandmot_real_result_t *)(params);
            _ret = pkapp_compare_mot_runsta(p_data->result, p_data->mot_pos_start, p_data->mot_pos_end, &(p_data->real_value), &(gst_app_params.esp_mot_runcfg));        // 判定电机有没有运行完成
        if (_ret == 1) {
            if (gst_app_params.shake_step_params.shake_times <= 0) {
                gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_FINISHED;
                disable_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_TIMEOUT_STEP]);
                disable_timer_tick(&mst_app_timer[E_APP_TIMER_STILL_CHECK_STEP]);
            } else {
                gst_app_params.shake_step_params.shake_times--;
                if (gst_app_params.shake_step_params.start_dir == E_CW) {
                    gst_app_params.shake_step_params.start_dir = E_CCW;
                } else {
                    gst_app_params.shake_step_params.start_dir = E_CW;
                }
                pkapp_set_mot_step_sta(E_MOT_SHAKE_EVT, E_MOT_STEP_STA_ING, (const unsigned char *)(&(gst_app_params.shake_step_params)), 0);
            }
        } else if (_ret == -1) {
            pkLOG_INFO(" motor direction is different! %d \n", __LINE__);
            gst_app_params.esp_mot_runcfg.type = E_SANDMOT_SCRAM_T;
            gst_app_params.esp_mot_runcfg.mot_sta = E_STOP;
            pkmcu_set_mot_run(gst_app_params.esp_mot_runcfg, 0);
        }
    } else if (value == E_MOT_STEP_STA_ING) {
_retry_send_ing:
        _retry_cnt--;
        mot_shake_step_t *p_data = (mot_shake_step_t *)(params);
        gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_ING;
        memcpy(&(gst_app_params.shake_step_params), p_data, sizeof(mot_shake_step_t));
        if (p_data->shake_times != 0) {
            if (p_data->start_dir == E_CW) {
                pkmcu_set_mot_run(gst_app_params.shake_step_params.shake_cw_cfg, 0);
            } else if (p_data->start_dir == E_CCW) {
                pkmcu_set_mot_run(gst_app_params.shake_step_params.shake_ccw_cfg, 0);                                
            } else {
                pkLOG_ERR(" error! shake step, first dir is stop! %d \n", __LINE__);
            }

            if ((_block == 1) && (_retry_cnt >= 0)) {
                _func_ret = pkmcu_get_exphydata(E_EXPHY_MOTRUN_CFG, E_INFO_BLK, 2, 0);      // 等待测试
                if (_func_ret == EXPHY_UPSTA_NEW) {
                    // 判定config 程序是否和设置一致
                    drv_physta_t _status;
                    get_phy_status(&_status);
                    _func_ret = compare_run_configdata(&(_status.st_motrun_cfg_sta), &(gst_app_params.esp_mot_runcfg));
                    if (_func_ret == 1) {           // 相同

                    } else if (_func_ret == 0) {    // 不同
                        goto _retry_send_ing;
                    }
                } else {    // 阻塞超时
                    pkLOG_ERR(" Timeout! run step start step_sta! %d \n", __LINE__);
                }
            }

            set_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_TIMEOUT_STEP], sys_get_os_runtime_sec(), p_data->shake_timeout, 0);
            set_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_CHECK_STEP], sys_get_os_runtime_sec(), MOT_STA_CHECK_DEF_TIME, 0);
        } else {
                            gst_app_params.dev_prg_step_sta = E_MOT_STEP_STA_FINISHED;
        }
    } else if (value == E_MOT_STEP_STA_TIMEOUT) {
        pkLOG_INFO(" still step, timeout! %d \n", __LINE__);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_TIMEOUT_STEP]);
        disable_timer_tick(&mst_app_timer[E_APP_TIMER_SHAKE_CHECK_STEP]);                        
    }

    return _ret;
}

#undef __PK_APP_C_

