
#define __PRODUCT_TEST_C_

#include "product_test.h"

#include <string.h>

#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
// #include "freertos/semphr.h"
// #include "freertos/portmacro.h"

#include "esp_timer.h"

#include "pk_hal.h"
#include "petkit_clog.h"
#include "queue.h"
#include "pk_T3_driver.h"
#include "pk_serial.h"

#include "user_lib.h"
#include "user_time.h"
#include "user_config.h"

#include "serial_comm_table.h"
#include "pk_app.h"

detect_param_t *gst_detect_params = NULL;
extern pkapp_params_t gst_app_params;


#define FUNCEVENT_CMD_MAX                   (15)
xQueueHandle xQueAgingEvent;        // 老化测试所用的队列
xQueueHandle xQueItemTestEvent;     // 单项测试所用的队列

/*********************************************************************/
#define RTC_TEST_DEF_YEAR                   (2019)
#define RTC_TEST_DEF_MON                    (11)
#define RTC_TEST_DEF_DAY                    (11)
#define RTC_TEST_DEF_HOUR                   (18)
#define RTC_TEST_DEF_MIN                    (35)
#define RTC_TEST_DEF_SEC                    (0)
static void _print_timer_Info(ctimer_t time);
/*********************************************************************/

static esp_timer_handle_t test_timer;
#define TEST_TIMER_DEF_TIME_US                 (1000000)       // 单位us

static unsigned int gs_det_os_time = 0;
static timer_tick_t mst_aging_evt_timer[E_AGING_EVT_MAX] = { 0 };
static timer_tick_t mst_item_evt_timer[E_TEST_MAX_ITEM] = { 0 };

static aging_params_t mst_aging_params;

static int _clear_agingtask_params(void);
static int _clear_detecttask_params(void);

static int _pcbtest_mode_manager(void);             // 2. 板测模式
static int _zjtest_mode_manager(void);              // 3. 整机测试模式
static int _repair_mode_manager(void);              // 4. 维修模式
static int _spot_mode_manager(void);                // 5. 抽检模式
static int _mac_mode_manager(void);                 // 6. MAC修改模式
static int _agingtest_mode_manager(void);           // 7. 老化测试
static int _multi_test_mode_manager(void);

int pk_detect_taskinit(void)
{
    gst_detect_params = (detect_param_t *)malloc(sizeof(detect_param_t));
    if (gst_detect_params == NULL) {
        pkLOG_ERR(" %s Func, malloc failed! %d \n", __func__, __LINE__);
        return -1;
    }
    memset(gst_detect_params, 0, sizeof(detect_param_t));

    xQueAgingEvent = xQueueCreate(FUNCEVENT_CMD_MAX, sizeof(sysevent_t));
    if (xQueAgingEvent == 0) {
        pkLOG_ERR(" %s Func, create aging queue event failed! %d \n", __func__, __LINE__);
        return -1;
    }

    xQueItemTestEvent = xQueueCreate(FUNCEVENT_CMD_MAX, sizeof(sysevent_t));
    if (xQueItemTestEvent == 0) {
        pkLOG_ERR(" %s Func, create item test queue event failed! %d \n", __func__, __LINE__);
        return -1;
    }

    memset(mst_aging_evt_timer, 0, sizeof(mst_aging_evt_timer));
    memset(mst_item_evt_timer, 0, sizeof(mst_item_evt_timer));

    memset(&mst_aging_params, 0, sizeof(mst_aging_params));

    return 0;
}

static int _pcbtest_mode_manager(void)
{
    int _ret = 0;
    unsigned char _bf_step_timeout = 0;

    if (gst_detect_params->pcbtest_step == E_PCBTEST_BEEP_OLED_STEP) {
            // 1. 超时管理
        if (gst_detect_params->pcbtest_step_start == 0) {
            gst_detect_params->pcbtest_step_start = 1;
            gst_detect_params->pcbtest_step_next_time = gs_det_os_time + PCBTEST_BEEPOLED_TIMEOUT;
        } else {
            if (gs_det_os_time > gst_detect_params->pcbtest_step_next_time) {
                _bf_step_timeout = 1;
            }
        }
            // 2. 按键依次按下
        if ((gst_detect_params->st_key_params.mask_key & (0x01 << E_DET_FUNC_BTN))
         && (gst_detect_params->st_key_params.mask_key & (0x01 << E_DET_WIFI_BTN))) {
             gst_detect_params->pcbtest_step_force_out = 1;
        }
            // 3. 步骤显示内容控制
        if ((_bf_step_timeout == 1)                                 // 超时退出
         || (gst_detect_params->pcbtest_step_force_out == 1)) {     // 强制退出
            gst_detect_params->pcbtest_step_start = 0;
            gst_detect_params->pcbtest_step_force_out = 0;
            gst_detect_params->pcbtest_step = E_PCBTEST_MCU_COMM_STEP;
            gst_detect_params->st_key_params.mask_key = 0;
        } else {
            if ((mst_item_evt_timer[E_TEST_BEEP_ITEM].enable == 0) && (mst_item_evt_timer[E_TEST_OLED_ITEM].enable == 0)) {      // beep & OLED 测试下
                if ((gst_detect_params->st_beep_params.step == E_DET_BEEP_STEP_OFF)
                && (gst_detect_params->st_oled_params.step == E_DET_OLED_STEP_OFF)) {
                    pk_send_itemtest_event(E_TEST_BEEP_ITEM, 0, NULL, 0);   // 开BEEP
                    set_timer_tick(&mst_item_evt_timer[ E_TEST_BEEP_ITEM], gs_det_os_time, 2, 1);
                } else if ((gst_detect_params->st_beep_params.step == E_DET_BEEP_STEP_ON)
                && (gst_detect_params->st_oled_params.step == E_DET_OLED_STEP_OFF)) {
                    pk_send_itemtest_event(E_TEST_BEEP_ITEM, 0, NULL, 0);   // 关BEEP
                    pk_send_itemtest_event(E_TEST_OLED_ITEM, 0, NULL, 0);   // 开OLED
                    set_timer_tick(&mst_item_evt_timer[ E_TEST_OLED_ITEM], gs_det_os_time, 2, 1);
                } else if ((gst_detect_params->st_beep_params.step == E_DET_BEEP_STEP_OFF)
                && (gst_detect_params->st_oled_params.step == E_DET_OLED_STEP_ON)) {
                    pk_send_itemtest_event(E_TEST_BEEP_ITEM, 0, NULL, 0);   // 开BEEP
                    pk_send_itemtest_event(E_TEST_OLED_ITEM, 0, NULL, 0);   // 关OLED
                    set_timer_tick(&mst_item_evt_timer[ E_TEST_BEEP_ITEM], gs_det_os_time, 2, 1);
                } else if ((gst_detect_params->st_beep_params.step == E_DET_BEEP_STEP_ON)
                && (gst_detect_params->st_oled_params.step == E_DET_OLED_STEP_ON)) {
                    pk_send_itemtest_event(E_TEST_BEEP_ITEM, 0, NULL, 0);   // 开BEEP
                    pk_send_itemtest_event(E_TEST_OLED_ITEM, 0, NULL, 0);   // 关OLED
                    set_timer_tick(&mst_item_evt_timer[ E_TEST_BEEP_ITEM], gs_det_os_time, 2, 1);
                    set_timer_tick(&mst_item_evt_timer[ E_TEST_OLED_ITEM], gs_det_os_time, 2, 1);
                }
            }
        }
    } else if (gst_detect_params->pcbtest_step == E_PCBTEST_MCU_COMM_STEP) {     // 通信检测
            // 1. 超时管理
        if (gst_detect_params->pcbtest_step_start == 0) {
            gst_detect_params->pcbtest_step_start = 1;
            gst_detect_params->pcbtest_step_next_time = gs_det_os_time + PCBTEST_MCU_COMM_TIMEOUT;
        } else {
            if (gs_det_os_time > gst_detect_params->pcbtest_step_next_time) {
                _bf_step_timeout = 1;
            }
        }
            // 2. 退出情况
        if ((_bf_step_timeout == 1)                                 // 超时退出
         || (gst_detect_params->pcbtest_step_force_out == 1)) {     // 强制退出
            gst_detect_params->pcbtest_step_start = 0;
            gst_detect_params->pcbtest_step = E_PCBTEST_DC_STEP;
            disable_timer_tick(&mst_item_evt_timer[ E_TEST_MCU_COMM_ITEM]);
        } else {
            if (mst_item_evt_timer[E_TEST_MCU_COMM_ITEM].enable == 0) {
                pk_send_itemtest_event(E_TEST_MCU_COMM_ITEM, E_DET_MCU_COMM_SEND, NULL, 0);
                set_timer_tick(&mst_item_evt_timer[ E_TEST_MCU_COMM_ITEM], gs_det_os_time, PCBTEST_MCU_COMM_SEND_DELAY, 1);
            }
        }
    } else if (gst_detect_params->pcbtest_step == E_PCBTEST_DC_STEP) {      // 电源电压判定
            // 1. 超时管理
        if (gst_detect_params->pcbtest_step_start == 0) {
            gst_detect_params->pcbtest_step_start = 1;
            gst_detect_params->pcbtest_step_next_time = gs_det_os_time + PCBTEST_DC_TIMEOUT;
        } else {
            if (gs_det_os_time > gst_detect_params->pcbtest_step_next_time) {
                _bf_step_timeout = 1;
            }
        }
            // 2. 退出情况
        if ((_bf_step_timeout == 1)                                 // 超时退出
         || (gst_detect_params->pcbtest_step_force_out == 1)) {     // 强制退出
            gst_detect_params->pcbtest_step_start = 0;
            gst_detect_params->pcbtest_step = E_PCBTEST_MOT_STEP;
            disable_timer_tick(&mst_item_evt_timer[E_TEST_DC_POWER_ITEM]);
        } else {
            if (mst_item_evt_timer[E_TEST_DC_POWER_ITEM].enable == 0) {         
                pk_send_itemtest_event(E_TEST_DC_POWER_ITEM, E_DET_DC_SEND_STEP, NULL, 0);
                set_timer_tick(&mst_item_evt_timer[E_TEST_DC_POWER_ITEM], gs_det_os_time, PCBTEST_DC_SENDCMD_TIME, 1);
            }        
        }
    } else if (gst_detect_params->pcbtest_step == E_PCBTEST_MOT_STEP) {     // 电机运行
            // 1. 超时管理
        if (gst_detect_params->pcbtest_step_start == 0) {
            gst_detect_params->pcbtest_step_start = 1;
            gst_detect_params->pcbtest_step_next_time = sys_get_os_runtime_sec() + PCBTEST_MOT_PRG_TIMEOUT;
        } else {
            if (sys_get_os_runtime_sec() > gst_detect_params->pcbtest_step_next_time) {
                _bf_step_timeout = 1;
            }
        }
            // 2. 等待人为判定按键按下

            // 3. 步骤情况
        if ((_bf_step_timeout == 1)                                 // 超时退出
         && (gst_detect_params->pcbtest_step_force_out == 1)) {     // 强制退出
            gst_detect_params->pcbtest_step_start = 0;
            gst_detect_params->pcbtest_step_force_out = 0;
            
        } else {
               
        }
    } else if (gst_detect_params->pcbtest_step == E_PCBTEST_IR_STEP) {    // 红外光栅检测
            // 1. 超时管理
        if (gst_detect_params->pcbtest_step_start == 0) {
            gst_detect_params->pcbtest_step_start = 1;
            gst_detect_params->pcbtest_step_next_time = sys_get_os_runtime_sec() + PCBTEST_IR_TIMEOUT;
        } else {
            if (sys_get_os_runtime_sec() > gst_detect_params->pcbtest_step_next_time) {
                _bf_step_timeout = 1;
            }
        }
            // 
        if ((_bf_step_timeout == 1)                                 // 超时退出
         && (gst_detect_params->pcbtest_step_force_out == 1)) {     // 强制退出
            gst_detect_params->pcbtest_step_start = 0;
            gst_detect_params->pcbtest_step_force_out = 0;
            
        } else {
            if (mst_item_evt_timer[E_TEST_IR_SENSOR_ITEM].enable == 0) {         
                pk_send_itemtest_event(E_TEST_IR_SENSOR_ITEM, E_DET_IR_DRIVE_STEP, NULL, 0);
                set_timer_tick(&mst_item_evt_timer[ E_TEST_IR_SENSOR_ITEM], sys_get_os_runtime_sec(), PCBTEST_IR_DRVSTEP_TIME, 1);
            }
        }        
    } else if (gst_detect_params->pcbtest_step == E_PCBTEST_WHQ_STEP) {         // 雾化器测试


    } else if (gst_detect_params->pcbtest_step == E_PCBTEST_WEIGHT_STEP) {      // 称重模块测试



    } else if (gst_detect_params->pcbtest_step == E_PCBTEST_RTC_STEP) {         // RTC测试

        

    }
    
    return _ret;
}
static int _zjtest_mode_manager(void)              // 3. 整机测试模式
{
    int _ret = 0;

    return _ret;
}
static int _repair_mode_manager(void)              // 4. 维修模式
{
    int _ret = 0;

    return _ret;
}
static int _spot_mode_manager(void)                // 5. 抽检模式
{
    int _ret = 0;

    return _ret;
}
static int _mac_mode_manager(void)                 // 6. MAC修改模式
{
    int _ret = 0;

    
    return _ret;
}
static int _agingtest_mode_manager(void)           // 7. 老化测试
{
    int _ret = 0;


    return _ret;
}

static int _multi_test_mode_manager(void)
{
    int _ret = 0;
    int _test_sta = 0;

    _test_sta = gst_detect_params->test_sta;
    if (_test_sta == pdDETECT_WAIT) { _pcbtest_mode_manager(); }                // 2. 板测模式
    else if (_test_sta == pdDETECT_CHECK) { _zjtest_mode_manager(); }            // 3. 整机测试模式
    else if (_test_sta == pdDETECT_REPAIR) { _repair_mode_manager(); }           // 4. 维修模式
    else if (_test_sta == pdDETECT_SPOT) { _spot_mode_manager(); }               // 5. 抽检模式
    else if (_test_sta == pdDETECT_MAC) { _mac_mode_manager(); }                 // 6. MAC修改模式
    else if (_test_sta == pdDETECT_AGEING) { _agingtest_mode_manager(); }        // 7. 老化测试

    return _ret;
}

void pk_detect_taskloop(void *pvParams)
{
    int detect_block = 1;
    sysevent_t st_detect_evt = { 0, 0, NULL };
    int cmd = 0;
    int value = 0;
    int _func_ret = 0;

        // 0. 进入测试板测模式
    gst_detect_params->test_sta = pdDETECT_ENTER;
    pkLOG_INFO(" pk_detect enter test mode! %d \n", __LINE__);
    pkdet_enter_test_mode();
        // 1. 开启定时器
    pk_test_timer_start(1);
    pkLOG_INFO(" pk_detect timer start! %d \n", __LINE__);

    while(detect_block == 1) {

            // 0. 不同的测试模式下运行效果不同
        _multi_test_mode_manager();

            // 1. 根据队列信息进行item测试
        memset(&st_detect_evt, 0, sizeof(sysevent_t));
        if (xQueueReceive(xQueItemTestEvent, &st_detect_evt, (portTickType)(100)/portTICK_RATE_MS)) {
            cmd = st_detect_evt.cmd;
            printf("pk product task: min = %u, %u, %d \n", esp_get_minimum_free_heap_size(), esp_get_free_heap_size(), __LINE__);
            switch(cmd) 
            {
            case E_TEST_BEEP_ITEM:                // 测试 Beep
                pkLOG_INFO(" item test =======> beep! %d \n", __LINE__);
                if (gst_detect_params->st_beep_params.step == E_DET_BEEP_STEP_ON) {
                    gst_detect_params->st_beep_params.step = E_DET_BEEP_STEP_OFF;
                    pkLOG_INFO(" set beep 0! %d \n", __LINE__);
                    // pkhal_beep_ctrl(0);     // 改 成LED灯的显示
                } else if (gst_detect_params->st_beep_params.step == E_DET_BEEP_STEP_OFF) {
                    gst_detect_params->st_beep_params.step = E_DET_BEEP_STEP_ON;
                    // pkhal_beep_ctrl(1);
                    pkLOG_INFO(" set beep 1! %d \n", __LINE__);
                }
                pkdet_item_beep_manager(NULL);
                break;
            case E_TEST_OLED_ITEM:                // 测试 OLED
                pkLOG_INFO(" item test =======> oled! %d \n", __LINE__);
                    // 待开发
                pkLOG_INFO(" oled test wait user to develop! %d \n", __LINE__);
                break;
            case E_TEST_KEY_ITEM:                 // 按键检测
                pkLOG_INFO(" item test =======> key! %d \n", __LINE__);
                value = st_detect_evt.value;
                gst_detect_params->st_key_params.mask_key |= (0x01 << value);
                break;
            case E_TEST_MCU_COMM_ITEM:            // MCU 通信检测
                pkLOG_INFO(" item test =======> ! %d \n", __LINE__);
                value = st_detect_evt.value;
                if (value == E_DET_MCU_COMM_SEND) {
                    pkmcu_get_exphydata(E_EXPHY_HB, E_INFO_NOBLK, 0, 0);
                } else if (value == E_DET_MCU_COMM_RECV) {
                    // 判断Sensor值 或者 判断
                }
                pkdet_item_mcucomm_manager(NULL, value);
                break;
            case E_TEST_DC_POWER_ITEM:            // 检测电源电压
                pkLOG_INFO(" item test =======> oled! %d \n", __LINE__);
                value = st_detect_evt.value;
                _func_ret = pkdet_item_dc_manager(NULL, value);
                if (value == E_DET_DC_SEND_STEP) {
                    if (_func_ret == 0) {
                        pkmcu_get_exphydata(E_EXPHY_DC_ADC, E_INFO_NOBLK, 0, 0);                    
                    }
                }  else if (value == E_DET_DC_RECV_STEP) {
                    // DC 判断
                }
                break;
            case E_TEST_MOT_ITEM:                 // 测试电机
                pkLOG_INFO(" item test =======> mot! %d \n", __LINE__);
                value = st_detect_evt.value;
                pkdet_item_mot_manager(NULL, value);
                break;
            /***********************************/
            case E_TEST_BLE_ITEM:                 // 蓝牙通信检测
                pkLOG_INFO(" item test =======> ble! %d \n", __LINE__);

                break;
            case E_TEST_WIFI_ITEM:                // wifi功能检测
                pkLOG_INFO(" item test =======> wifi! %d \n", __LINE__);

                break;
            case E_TEST_IR_SENSOR_ITEM:           // 红外对管传感器检测
                pkLOG_INFO(" item test =======> sensor! %d \n", __LINE__);
                value = st_detect_evt.value;
                pkdet_item_ir_manager(NULL, value);
                break;
            case E_TEST_WHQ_ITEM:                 // 雾化器检测
                pkLOG_INFO(" item test =======> whq! %d \n", __LINE__);

                break;
            case E_TEST_WEIGHT_ITEM:              // 称重传感器检测
                pkLOG_INFO(" item test =======> weight! %d \n", __LINE__);

                break;
            case E_TEST_RTC_ITEM:                 // RTC检测
                pkLOG_INFO(" item test =======> rtc! %d \n", __LINE__);

                break;
            case E_TEST_IO_ITEM:                  // IO通用检测
                pkLOG_INFO(" item test =======> IO! %d \n", __LINE__);
            
                break;
            /***********************************/

            case E_TEST_FORCE_QUIT_ITEM:                     // 强制退出
                pkLOG_INFO(" item test =======> Quit! %d \n", __LINE__);
                
                break;
            default:

                break;
            }

        }
    }

}

static int _clear_detecttask_params(void)
{
    int _ret = 0;


    return _ret;
}

int pkdet_enter_test_mode(void)
{
    int _ret = 0;
    if (pdDETECT_ENTER == gst_detect_params->test_sta) {
        close_esp_devices();            // 1. 关闭所有外设
        sys_force_quittask(0xFF);   // 2. 退出无关任务
        sys_close_wifi(1);          // 3. 关闭wifi, ble
        sys_close_ble(1);
        gst_detect_params->st_rtc_params.step = E_DET_RTC_STEP_WRITE;        // 4. RTC 写入测试值
        ctime_set_timer_func(RTC_TEST_DEF_YEAR, RTC_TEST_DEF_MON, RTC_TEST_DEF_DAY, RTC_TEST_DEF_HOUR, RTC_TEST_DEF_MIN, RTC_TEST_DEF_SEC);
        gst_detect_params->test_sta = pdDETECT_WAIT;
        gst_detect_params->pcbtest_step = E_PCBTEST_BEEP_OLED_STEP;
    } else {
        pkLOG_INFO(" other test sta: %d, %d \n", gst_detect_params->test_sta, __LINE__);
    }
    return _ret;
}

int pk_test_check(void *params)
{
    int _ret = 0;
    int _level = 0;
    int _tick = 0;
    int _read_cnt = 0;

    pkhal_tpio_init();
    if (1) {
        _ret = 0;
        pkLOG_INFO(" force 1! %d \n", __LINE__);
        goto _exit_testcheck;
    } else {
        _read_cnt = 10;
        vTaskDelay(1000/portTICK_RATE_MS);
        while(_read_cnt--) {
            _level = pkhal_get_tpio_level();
            pkLOG_DEBUG("test gpio:%d\n", _level);
            if (0 == _level) {
                _tick++;
                if (_tick >= 5) {
                    _tick = 0;
                    _ret = 1;
                    goto _exit_testcheck;
                }
            } else {
                _tick = 0;
            }
            vTaskDelay(5/portTICK_RATE_MS);
        }
    }
_exit_testcheck:
    if (_ret == 1) {
        gst_detect_params->test_mode = pdTOOL;
    }
    return _ret;
}

// 产测定时器控制函数
static void _pk_product_test_timer_cb(void *arg)
{
    int _detect_sta = 0;
    gs_det_os_time++;
    // pkLOG_INFO(" gs_det ostime: %d, %d \n", gs_det_os_time, __LINE__);

    _detect_sta = pk_get_detect_sta();
    if (_detect_sta == pdDETECT_WAIT) {         // 板测模式下
            // 测试 Beep
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_BEEP_ITEM], gs_det_os_time)) {
            // pk_send_itemtest_event(E_TEST_BEEP_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_BEEP_ITEM]);
        }
            // 测试 OLED
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_OLED_ITEM], gs_det_os_time)) {
            // pk_send_itemtest_event(E_TEST_OLED_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_OLED_ITEM]);
        }
            // 按键检测
        // if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_KEY_ITEM], gs_det_os_time)) {
        //     // pk_send_itemtest_event(E_TEST_KEY_ITEM, 0, NULL, 1);
        //     disable_timer_tick(&mst_item_evt_timer[E_TEST_KEY_ITEM]);
        // } 
            // MCU 通信检测
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_MCU_COMM_ITEM], gs_det_os_time)) {
            // pk_send_itemtest_event(E_TEST_MCU_COMM_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_MCU_COMM_ITEM]);
        }
            // 检测电源电压
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_DC_POWER_ITEM], gs_det_os_time)) {
            // pk_send_itemtest_event(E_TEST_DC_POWER_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_DC_POWER_ITEM]);
        }  
            // 测试电机
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_MOT_ITEM], gs_det_os_time)) {
            pk_send_itemtest_event(E_TEST_MOT_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_MOT_ITEM]);
        }

        /***************************************************************************************/
            // 蓝牙通信检测
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_BLE_ITEM], gs_det_os_time)) {
            pk_send_itemtest_event(E_TEST_BLE_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_BLE_ITEM]);
        }
            // wifi功能检测
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_WIFI_ITEM], gs_det_os_time)) {
            pk_send_itemtest_event(E_TEST_WIFI_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_WIFI_ITEM]);
        }
            // 红外对管传感器检测
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_IR_SENSOR_ITEM], sys_get_os_runtime_sec())) {
            // pk_send_itemtest_event(E_TEST_IR_SENSOR_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_IR_SENSOR_ITEM]);
        }   
            // 雾化器检测
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_WHQ_ITEM], gs_det_os_time)) {
            pk_send_itemtest_event(E_TEST_WHQ_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_WHQ_ITEM]);
        }  
            // 称重传感器检测
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_WEIGHT_ITEM], gs_det_os_time)) {
            pk_send_itemtest_event(E_TEST_WEIGHT_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_WEIGHT_ITEM]);
        }  
            // RTC检测
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_RTC_ITEM], gs_det_os_time)) {
            pk_send_itemtest_event(E_TEST_RTC_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_RTC_ITEM]);
        }
            // IO通用检测
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_IO_ITEM], gs_det_os_time)) {
            pk_send_itemtest_event(E_TEST_IO_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_IO_ITEM]);
        }
        /***************************************************************************************/

            // 强制退出
        if (Is_timer_tick_up(&mst_item_evt_timer[E_TEST_FORCE_QUIT_ITEM], gs_det_os_time)) {
            pk_send_itemtest_event(E_TEST_FORCE_QUIT_ITEM, 0, NULL, 1);
            disable_timer_tick(&mst_item_evt_timer[E_TEST_FORCE_QUIT_ITEM]);
        }

    }
 
    if (_detect_sta == pdDETECT_AGEING)         // aging 模式下 
    {       
            // Weight  // 称重
        if (Is_timer_tick_up(&mst_aging_evt_timer[E_AGING_EVT_WEIGHT], gs_det_os_time)) {
            pk_send_aging_event(E_AGING_EVT_WEIGHT, E_DET_WEIGHT_STEP_SEND, NULL, 1);
        }
            // Beep    // 凤鸣器
        if (Is_timer_tick_up(&mst_aging_evt_timer[E_AGING_EVT_BEEP], gs_det_os_time)) {
            pk_send_aging_event(E_AGING_EVT_BEEP, 0, NULL, 1);
            disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_BEEP]);
        }
            // OLED
        if (Is_timer_tick_up(&mst_aging_evt_timer[E_AGING_EVT_OLED], gs_det_os_time)) {
            pk_send_aging_event(E_AGING_EVT_OLED, gs_det_os_time, NULL, 1);
            disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_OLED]);
        }
            // MOT     // 左到位 + 右到位, 两个项目
        if (Is_timer_tick_up(&mst_aging_evt_timer[E_AGING_EVT_MOT], gs_det_os_time)) {
            // pk_send_aging_event(E_AGING_EVT_MOT, gs_det_os_time, NULL, 1);
            pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_CHECK, NULL, 1);
            disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_MOT]);
        }
            // WHQ     // 雾化器
        if (Is_timer_tick_up(&mst_aging_evt_timer[E_AGING_EVT_WHQ], gs_det_os_time)) {
            pk_send_aging_event(E_AGING_EVT_WHQ, gs_det_os_time, NULL, 1);
            disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_WHQ]);
        }       
            // RTC     // 雾化器
        if (Is_timer_tick_up(&mst_aging_evt_timer[E_AGING_EVT_RTC], gs_det_os_time)) {
            pk_send_aging_event(E_AGING_EVT_RTC, gs_det_os_time, NULL, 1);
            disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_RTC]);
        }
            // QUIT    // 退出
        if (Is_timer_tick_up(&mst_aging_evt_timer[E_AGING_EVT_QUIT], gs_det_os_time)) {
            pk_send_aging_event(E_AGING_EVT_QUIT, gs_det_os_time, NULL, 1);
            disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_QUIT]);
        }
    }


}

int pk_test_timer_start(int force)
{
    int _ret = 0;
    if (force) {
        _ret = pk_test_timer_stop();   // stop timer;
        if (_ret) { return _ret; }
        esp_timer_create_args_t _timer_conf = {
            .callback = _pk_product_test_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "test_timer"
        };
        esp_err_t _err = esp_timer_create(&_timer_conf, &test_timer);
        if (_err) { return _err; }
        _ret = esp_timer_start_periodic(test_timer, TEST_TIMER_DEF_TIME_US);
    }
    return _ret;
}

int pk_test_timer_stop(void)
{
    int _ret = 0;
    if (test_timer) {
        _ret = esp_timer_stop(test_timer);
        if (_ret) { return _ret; }
        _ret = esp_timer_delete(test_timer);
    } 
    return _ret;
}

static void _printf_mot_runreal_value(sandmot_real_result_t *value) 
{
    pkLOG_INFO(" result: %d, sta: %d, speed: %d, current: %d, step_num: %d, run_time: %d, %d \n",
        value->result, value->real_value.mot_sta, value->real_value.mot_speed, value->real_value.u16_mot_curt, value->real_value.u16_mot_stepnum, value->real_value.u16_mot_runtime, __LINE__);
}

// 老化测试过程: 1. 称重, 每30Sec读取一次，
//             2. Beep, 每10Min想1Sec
//             3. OLED, 刷屏一段时间
//             4. 电机左转到位,
//             5. 电机又转到位,
//             6. 雾化器工作2Min，工作5Sec
//             7. 每60Sec读取一次RTC
//             8. Wifi功能暂且不动
int pk_detct_aging_test(void *pvParams)    // 老化测试
{
    int _ret = PRG_TEST_ING;
    sysevent_t st_aging_evt = { 0, 0, NULL };
    int cmd = 0;
    int value = 0;
    int block = 1;          // 阻塞
    unsigned char retry_cnt = 0;
    unsigned int _u32data = 0;
    static unsigned char last_mot_cmd = 0;

    pkLOG_INFO(" pkdet agingtest task start! %d \n", __LINE__);

    /******for test****************************/
    pk_set_detect_sta(pdDETECT_AGEING);
    set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_WEIGHT], gs_det_os_time, 10, 0);
    set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_BEEP], gs_det_os_time, 12, 0);
    set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_OLED], gs_det_os_time, 14, 0);
    // set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_MOT], gs_det_os_time, 16, 0);
    // pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_CCW_POS, NULL, 1);
    set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_WHQ], gs_det_os_time, 18, 0);
    set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_RTC], gs_det_os_time, 20, 0);
    // set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_QUIT], gs_det_os_time, 25, 0);
    /*****************************************/

    pk_test_timer_start(1);
    pkLOG_INFO(" pk_detect timer start! %d \n", __LINE__);

    while(block == 1) {
            // 0. 电机在运行过程中如果传感器到位了则需要进行判定

            // 1. 处理aging test项目
        memset(&st_aging_evt, 0, sizeof(sysevent_t));
        if (xQueueReceive(xQueAgingEvent, &st_aging_evt, (portTickType)(100)/portTICK_RATE_MS)) {
            cmd = st_aging_evt.cmd;
            value = st_aging_evt.value;
            switch(cmd) 
            {
            case E_AGING_EVT_WEIGHT:             // 称重
                pkLOG_INFO(" aging weight event! value:%d, %d \n", value, __LINE__);
                if (value == E_DET_WEIGHT_STEP_SEND) {
                    retry_cnt++;
                    if (retry_cnt < 3) {
                        disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_WEIGHT]);
                        // pkLOG_INFO("=====================> get weight value cmd, retry_cnt: %d, %d \n", retry_cnt, __LINE__);
                        // pkmcu_get_exphydata(E_EXPHY_WEIGHT, E_INFO_NOBLK, 1, 0);
                        pkmcu_get_weight(0);
                        set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_WEIGHT], gs_det_os_time, 5, 1);
                    } else {
                        retry_cnt = 0;
                        disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_WEIGHT]);
                        pkLOG_ERR(" aging test get weight cmd is error! timeout! %d \n ", __LINE__);
                        // get weight 超时错误 - 待设置
                    }
                } else if (value == E_DET_WEIGHT_STEP_RECV) {
                    disable_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_WEIGHT]);
                    retry_cnt = 0;
                    if (st_aging_evt.msgptr != NULL) {
                        if (((weight_real_t *)(st_aging_evt.msgptr))->p_weight) {
                            unsigned char _length = 0;
                            unsigned char _cnt = 0;
                            _length = ((weight_real_t *)(st_aging_evt.msgptr))->size;
                            // pkLOG_INFO(" weight data: ");
                            // for (_cnt = 0; _cnt < _length; _cnt++) {
                            //     pkLOG_INFO(" %d \n", ((weight_real_t *)(st_aging_evt.msgptr))->p_weight[_cnt]);
                            // }
                            // pkLOG_INFO("\n");
                        }
                        // memcpy((unsigned char *)&_u32data, st_aging_evt.msgptr, sizeof(unsigned int));
                    }
                    if (((weight_real_t *)(st_aging_evt.msgptr))->p_weight) { free(((weight_real_t *)(st_aging_evt.msgptr))->p_weight); ((weight_real_t *)(st_aging_evt.msgptr))->p_weight = NULL; }
                    if (st_aging_evt.msgptr) { free(st_aging_evt.msgptr); st_aging_evt.msgptr = NULL; }
                    set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_WEIGHT], gs_det_os_time, 60, 1);
                }
                break;
            case E_AGING_EVT_BEEP:               // 凤鸣器
                // pkLOG_INFO(" aging Beep event! value:%d, %d \n", value, __LINE__);
                if (mst_aging_params.st_beep_params.step == E_DET_BEEP_STEP_ON) {
                    mst_aging_params.st_beep_params.step = E_DET_BEEP_STEP_OFF;
                    // pkLOG_INFO(" set beep 0! %d \n", __LINE__);
                    // pkhal_beep_ctrl(0);
                    set_timer_tick(&mst_aging_evt_timer[ E_AGING_EVT_BEEP], gs_det_os_time, 600, 1);
                } else if (mst_aging_params.st_beep_params.step == E_DET_BEEP_STEP_OFF) {
                    mst_aging_params.st_beep_params.step = E_DET_BEEP_STEP_ON;
                    pkLOG_INFO(" set beep 1! %d \n", __LINE__);
                    // pkhal_beep_ctrl(1);
                    set_timer_tick(&mst_aging_evt_timer[ E_AGING_EVT_BEEP], gs_det_os_time, 1, 1);
                }
                break;
            case E_AGING_EVT_OLED:               // OLED
                // pkLOG_INFO(" aging OLED event! value:%d, %d \n", value, __LINE__);
                    // OLED 待开发
                // pkLOG_INFO(" whq wait user to develop! %d \n", __LINE__);
                break;
            case E_AGING_EVT_MOT:                // 电机，       // 左到位 + 右到位, 两个项目
                // pkLOG_INFO(" aging mot event! value:%d, %d \n", value, __LINE__);
                {
                    if (value == E_AGDET_MOT_WAIT) {
                        if (st_aging_evt.msgptr) {
                            sandmot_real_result_t *p_data = (sandmot_real_result_t *)(st_aging_evt.msgptr);
                            if (p_data->result == 1) {
                                if (last_mot_cmd == E_AGDET_MOT_CW_POS) {
                                    pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_CCW_POS, NULL, 1);
                                } else if (last_mot_cmd == E_AGDET_MOT_CCW_POS) {
                                    pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_CW_POS, NULL, 1);
                                }
                            } else {
                                if ((last_mot_cmd == E_AGDET_MOT_CW_POS)
                                 && (p_data->real_value.mot_sta == E_STOP)
                                 && (pkmcu_get_exphydata(E_EXPHY_DET_SAND_L, E_INFO_NOW, 0, 0) == 0)
                                 && (pkmcu_get_exphydata(E_EXPHY_DET_SAND_R, E_INFO_NOW, 0, 0) != 0)) {
                                    pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_CCW_POS, NULL, 1);
                                } else if ((last_mot_cmd == E_AGDET_MOT_CCW_POS)
                                 && (p_data->real_value.mot_sta == E_STOP)
                                 && (pkmcu_get_exphydata(E_EXPHY_DET_SAND_R, E_INFO_NOW, 0, 0) == 0)
                                 && (pkmcu_get_exphydata(E_EXPHY_DET_SAND_L, E_INFO_NOW, 0, 0) != 0)) {
                                    pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_CW_POS, NULL, 1);
                                }
                            }
                            pkLOG_INFO(" aging test mot! waiting datas! %d \n", __LINE__);
                            _printf_mot_runreal_value((sandmot_real_result_t *)(st_aging_evt.msgptr));
                            free(st_aging_evt.msgptr); st_aging_evt.msgptr = NULL;
                        }
                    } else if (value == E_AGDET_MOT_CW_POS) {
                        // if ((pkmcu_get_exphydata(E_EXPHY_DET_SAND_L, E_INFO_NOW, 0, 0) == 0)
                        // || (pkmcu_get_exphydata(E_EXPHY_SENSOR, E_INFO_NOW, 0, 0) == EXPHY_UPSTA_NEW)) {
                        //     pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_CCW_POS, NULL, 1);                         
                        // // } else if ((pkmcu_get_exphydata(E_EXPHY_DET_SAND_R, E_INFO_NOW, 0, 0) == 0) 
                        // //         || (pkmcu_get_exphydata(E_EXPHY_SENSOR, E_INFO_NOW, 0, 0) == EXPHY_UPSTA_NEW)) {
                        // //     last_mot_cmd = value;
                        // //     pkmcu_set_motrun_cfgdata(E_SANDMOT_POS_T, E_SANDMOT_POS_END, E_CW, 1500, 0, 4000, 0xFFFF, 0);
                        // //     set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_MOT], gs_det_os_time, 4, 0);                         
                        // } else {
                            last_mot_cmd = value;
                            // gst_app_params.esp_mot_runcfg.type = E_SANDMOT_POS_T;
                            // gst_app_params.esp_mot_runcfg.pos = E_SANDMOT_POS_END;
                            // gst_app_params.esp_mot_runcfg.mot_sta = E_CW;
                            pkLOG_INFO(" set cw end pos! %d \n", __LINE__);
                            // pkmcu_set_motrun_cfgdata(E_SANDMOT_POS_T, E_SANDMOT_POS_END, E_CW, 1500, 0, 4000, 0xFFFF, 0);
                            set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_MOT], gs_det_os_time, 4, 0);             
                        // }
                    } else if (value == E_AGDET_MOT_CCW_POS) {
                        // if ((pkmcu_get_exphydata(E_EXPHY_DET_SAND_R, E_INFO_NOW, 0, 0) == 0) 
                        // || (pkmcu_get_exphydata(E_EXPHY_SENSOR, E_INFO_NOW, 0, 0) == EXPHY_UPSTA_NEW)) {
                        //     pk_send_aging_event(E_AGING_EVT_MOT, E_AGDET_MOT_CW_POS, NULL, 1);                                             
                        // } else {
                            last_mot_cmd = value;
                            pkLOG_INFO(" set ccw start pos! %d \n", __LINE__);
                            // pkmcu_set_motrun_cfgdata(E_SANDMOT_POS_T, E_SANDMOT_POS_START, E_CCW, 1500, 0, 4000, 0xFFFF, 0);
                            set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_MOT], gs_det_os_time, 4, 0);
                        // }
                    } else if (value == E_AGDET_MOT_CHECK) {
                        pk_esp_mcu_comm(COMM_CMD_MOT_RUN_RT, COMM_READ_CMD, 0, NULL, 0, 0);
                        set_timer_tick(&mst_aging_evt_timer[E_AGING_EVT_MOT], gs_det_os_time, 4, 0);
                    }
                }
                break;
            case E_AGING_EVT_WHQ:                // 雾化器
                // pkLOG_INFO(" aing whq event! value:%d, %d \n", value, __LINE__);
                if (mst_aging_params.st_whq_params.step == E_DET_WHQ_STEP_OFF) {
                    mst_aging_params.st_whq_params.step = E_DET_WHQ_STEP_ON;
                    pkLOG_INFO(" set whq 1! %d \n", __LINE__);
                    pkhal_set_whqpower_sta(1);
                    set_timer_tick(&mst_aging_evt_timer[ E_AGING_EVT_WHQ], gs_det_os_time, 2, 1);
                } else if (mst_aging_params.st_whq_params.step == E_DET_WHQ_STEP_ON) {
                    int _level = 0;
                    mst_aging_params.st_whq_params.step = E_DET_WHQ_STEP_READ;
                    // pkLOG_INFO(" set whq 1! & get whq level! %d \n", __LINE__);
                    pkhal_set_whqpower_sta(1);
                    _level = pkhal_get_whqio_level();
                    pkLOG_INFO(" whq device, qushui signal: %d, %d \n", _level, __LINE__);
                    set_timer_tick(&mst_aging_evt_timer[ E_AGING_EVT_WHQ], gs_det_os_time, 2, 1);
                } else if (mst_aging_params.st_whq_params.step == E_DET_WHQ_STEP_READ) {
                    mst_aging_params.st_whq_params.step = E_DET_WHQ_STEP_OFF;
                    pkhal_set_whqpower_sta(0);
                    set_timer_tick(&mst_aging_evt_timer[ E_AGING_EVT_WHQ], gs_det_os_time, 120, 1);
                }
                break;
            case E_AGING_EVT_RTC:                // RTC测试
                pkLOG_INFO(" aging rtc event! value:%d, %d \n", value, __LINE__);
                ctimer_t read_time = { 0 };
                if (mst_aging_params.st_rtc_params.step == E_DET_RTC_STEP_WRITE) {
                    // pkLOG_INFO(" write rtc signal cmd! %d \n", __LINE__);
                    ctime_set_timer_func(RTC_TEST_DEF_YEAR, RTC_TEST_DEF_MON, RTC_TEST_DEF_DAY, RTC_TEST_DEF_HOUR, RTC_TEST_DEF_MIN, RTC_TEST_DEF_SEC);
                    set_timer_tick(&mst_aging_evt_timer[ E_AGING_EVT_RTC], gs_det_os_time, 120, 1);
                    mst_aging_params.st_rtc_params.step = E_DET_RTC_STEP_READ;
                } else if (mst_aging_params.st_rtc_params.step == E_DET_RTC_STEP_READ) {
                    // pkLOG_INFO(" read rtc signal cmd! %d \n", __LINE__);
                    ctime_read_timer_func(&read_time, 800);
                    _print_timer_Info(read_time);
                    set_timer_tick(&mst_aging_evt_timer[ E_AGING_EVT_RTC], gs_det_os_time, 120, 1);
                    mst_aging_params.st_rtc_params.step = E_DET_RTC_STEP_WRITE;
                }
                break;
            case E_AGING_EVT_QUIT:               // 退出老化测试模式
                pkLOG_INFO(" quit aging test task! value:%d, %d \n", value, __LINE__);
                block = 0;
                break;
            case E_AGING_EVT_SENSOR:
                // if ((((sensor_sta_t *)(st_aging_evt.msgptr))->io_det & (0x01 << DET_SAND_L)) == 0) {             // 到边
                //     if (last_mot_cmd == E_AGDET_MOT_CCW_POS) {                  // 即停之后，再向反方向运行
                //         pkLOG_INFO(" ccw to sand left, End pos! %d \n", __LINE__);
                //         // pkmcu_set_motrun_cfgdata(E_SANDMOT_SCRAM_T, E_SANDMOT_POS_NONE, E_STOP, 1500, 0, 4000, 0xFFFF, 0);
                //     }
                // } else if ((((sensor_sta_t *)(st_aging_evt.msgptr))->io_det & (0x01 << DET_SAND_R)) == 0) {      // 到边
                //     if (last_mot_cmd == E_AGDET_MOT_CW_POS) {
                //         pkLOG_INFO(" cw to sand right, Start pos! %d \n", __LINE__);
                //         // pkmcu_set_motrun_cfgdata(E_SANDMOT_SCRAM_T, E_SANDMOT_POS_NONE, E_STOP, 1500, 0, 4000, 0xFFFF, 0);                        
                //     }
                // }

                break;
            default:
                pkLOG_INFO(" cmd is over set! value:%d, %d \n", value, __LINE__);
                break;
            }

        }

    }


    pkLOG_INFO(" delete aging test params! %d \n", __LINE__);
    _clear_agingtask_params();
    vTaskDelete(NULL);

    return _ret;
}

static void _print_timer_Info(ctimer_t time)
{
    printf("**********************************\n");
    printf(" %d - %d - %d, %d:%d:%d \n", time.year, time.mon, time.day, time.hour, time.min, time.sec);
    printf("**********************************\n");
}


static int _clear_agingtask_params(void)
{
    int _ret = 0;


    return _ret;
}

/*****Test Item**************************************/
int pkdet_item_beep_manager(void *params)
{
    int _ret = 0;

    if (gst_detect_params->test_sta == pdDETECT_WAIT) {     // PCB 板级测试方案

    } else {
        // 其他测试模式下控制方式
    }

    return _ret;
}
int pkdet_item_oled_manager(void *params);
int pkdet_item_mcucomm_manager(void *params, unsigned short value)
{
    int _ret = 0;
    if (gst_detect_params->test_sta == pdDETECT_WAIT) {     // PCB 板级测试方案
        gst_detect_params->st_mcucomm_params.step = value;
        if (value == E_DET_MCU_COMM_RECV) {
            gst_detect_params->pcbtest_step_force_out = 1;  // 数据更新
        }
    } else {
        // 其他测试模式下控制方式
    } 

    return _ret;
}
int pkdet_item_dc_manager(void *params, unsigned short value)
{
    int _ret = 0;

    if (gst_detect_params->test_sta == pdDETECT_WAIT) {     // PCB 板级测试方案
        gst_detect_params->st_mcucomm_params.step = value;
        if (value == E_DET_MCU_COMM_RECV) {
            gst_detect_params->pcbtest_step_force_out = 1;  // 数据更新
            _ret = 1;
        } else if (value == E_DET_MCU_COMM_SEND) {
            exphy_upsta_t upsta = { 0, 0, 0 };
            pkmcu_get_exphy_upsta(E_EXPHY_DC_ADC, &upsta);
            // if (upsta.upsta == EXPHY_UPSTA_NEW)
            //  && (upsta.update_time + 100 < get_os_time) ) {
            //     _ret = 1;
            // }
        }
        if (_ret == 1) {
             // 判断adc值
            // drv_physta_t status;
            // get_phy_status(&status);
            // if (status.st_sensor_sta ) {

            // }
        }
    } else {
        // 其他测试模式下控制方式
    } 
    
    return _ret;
}
int pkdet_item_mot_manager(void *params, unsigned short value)
{
    int _ret = 0;

    if (gst_detect_params->test_sta == pdDETECT_WAIT) {
        gst_detect_params->st_mot_params.step = value;
        
    } else {

    }

    return _ret;
}
int pkdet_item_ir_manager(void *params, unsigned short value)
{
    int _ret = 0;

    if (gst_detect_params->test_sta == pdDETECT_WAIT) {
        gst_detect_params->st_mot_params.step = value;
        if (value == E_DET_IR_DRIVE_STEP) {
            pkhal_set_catin_IR(1);
            pkmcu_set_exphy_outpin(E_EXPHY_OUTPIN_IR, E_DODRV_MODE_ALLON, 0xFF, 0xFF, 0, 0);
        } else if (value == E_DET_IR_CHECK_STEP) {
            
        }
    } else {

    }
    
    return _ret;
}
/****************************************************/

/*****测试变量相关函数**********************************/
detect_param_t *pk_get_detectvalue(void)
{
    return gst_detect_params;
}
int pk_is_test_mode(void)
{
    if (gst_detect_params == NULL) {
        return 0;
    } else {
        if (gst_detect_params->test_sta != pdDETECT_FALSE) {
            return 1;
        }
    }
    return 0;
}
int pk_get_detect_sta(void)            // 获取测试状态
{
    int _ret = pdDETECT_FALSE;
    if (gst_detect_params != NULL) {
        _ret = gst_detect_params->test_sta;
    }
    return _ret;
}
int pkdet_get_pcbstep(void)
{
    int _ret = -1;
    if (gst_detect_params != NULL) {
        _ret = gst_detect_params->pcbtest_step;
    }
    return _ret;   
}
int pk_set_detect_sta(unsigned char sta)
{
    return (gst_detect_params->test_sta = sta);
}
/****************************************************/

int pk_send_itemtest_event(drv_test_item event, int value, char *pdata, unsigned char bfInt)
{
    int _ret = 0;
    if (bfInt) {
        _ret = sys_xQueSendToBackFromISR(xQueItemTestEvent, event, value, pdata);
    } else {
        _ret = sys_xQueSendToBack(xQueItemTestEvent, event, value, pdata);
    }
    return _ret; 
}

int pk_send_aging_event(aging_event event, int value, char *pdata, unsigned char bfInt)
{
    int _ret = 0;
    if (bfInt) {
        _ret = sys_xQueSendToBackFromISR(xQueAgingEvent, event, value, pdata);
    } else {
        _ret = sys_xQueSendToBack(xQueAgingEvent, event, value, pdata);
    }
    return _ret;
}

#undef __PRODUCT_TEST_C_
