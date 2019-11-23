

#define __PK_T3_DRIVER_C_

#include "pk_T3_driver.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>

#include "pk_serial.h"
#include "pkDevGPIO.h"
#include "pk_hal.h"
#include "petkit_clog.h"
#include "product_test.h"

static exphy_upsta_t gast_exphy_upsta[E_EXPHY_UPSTA_MAX] = { 0 };
static exphy_upsta_t gast_motreal_upsta[E_MOTREAL_MAX] = { 0 };

static drv_physta_t g_ephy_sta = { 0 };

static drv_weight_data_t gst_weight = { 0 };

volatile static int g_os_runtime = 0;

int pk_os_runtime(void)
{
    return g_os_runtime;
}

int pkmcu_init_exphy_upsta(void)
{
    memset(gast_exphy_upsta, 0, sizeof(gast_exphy_upsta));
    memset(&g_ephy_sta, 0, sizeof(g_ephy_sta));
    return 0;
}
int pkmcu_clean_all_exphy_upsta(void)
{
    int i = 0;
    for (i = 0; i < E_EXPHY_UPSTA_MAX; i++) {
        gast_exphy_upsta[i].upsta = EXPHY_UPSTA_OLD;
    }
    return 0;
}
int pkmcu_clean_exphy_upsta(exphy_upsta_node dev_node)
{
    gast_exphy_upsta[dev_node].upsta = EXPHY_UPSTA_OLD;
    return 0;    
}
int pkmcu_set_exphy_upsta(exphy_upsta_node dev_node, int seq, int time)
{
    gast_exphy_upsta[dev_node].upsta = EXPHY_UPSTA_NEW;
    gast_exphy_upsta[dev_node].seq = (short)seq;
    gast_exphy_upsta[dev_node].update_time = time;
    return 0;
}
int pkmcu_get_exphy_upsta(exphy_upsta_node dev_node, exphy_upsta_t *upsta)
{
    if (upsta) {
        memcpy (upsta, &(gast_exphy_upsta[dev_node]), sizeof(exphy_upsta_t));
        return 0;
    }
    return -1;
}
int pkmcu_init_motreal_upsta(void)
{
    memset(gast_motreal_upsta, 0, sizeof(gast_motreal_upsta));
    return 0;
}
int pkmcu_clean_allmot_realupsta(void)
{
    int _cnt = 0;
    for (_cnt = 0; _cnt < E_MOTREAL_MAX; _cnt++) {
        gast_motreal_upsta[_cnt].upsta = EXPHY_UPSTA_OLD;
    }
    return 0;
}
int pkmcu_clean_mot_realupsta(mot_drvreal_node dev_node)
{
    gast_motreal_upsta[dev_node].upsta = EXPHY_UPSTA_OLD;
    return 0;       
}
int pkmcu_set_mot_realupsta(mot_drvreal_node dev_node, int seq, int time)
{
    gast_motreal_upsta[dev_node].upsta = EXPHY_UPSTA_NEW;
    gast_motreal_upsta[dev_node].seq = (short)seq;
    gast_motreal_upsta[dev_node].update_time = time;
    return 0;
}
int pkmcu_get_mot_realupsta(mot_drvreal_node dev_node, exphy_upsta_t *upsta)
{
    if (upsta) {
        memcpy (upsta, &(gast_motreal_upsta[dev_node]), sizeof(exphy_upsta_t));
        return 0;
    }
    return -1;    
}

int get_phy_status(drv_physta_t* lastest_device_status)
{
    if (lastest_device_status) {
		memcpy(lastest_device_status, &g_ephy_sta, sizeof(drv_physta_t));
		return 0;        
    }
    return -1;
}
int set_exphy_sensordata(sensor_sta_t *params)
{
    if (params) {
        memcpy(&(g_ephy_sta.st_sensor_sta), params, sizeof(sensor_sta_t));
        return 0;
    }
    return -1;
}

int set_exphy_motrun_cfgdata(const sandmot_runcfg_t *pdata)
{
    if (pdata) {
        memcpy(&(g_ephy_sta.st_motrun_cfg_sta), pdata, sizeof(sandmot_runcfg_t));
        return 0;
    }
    return -1; 
}
int set_exphy_motrun_realdata(const sandmot_real_t *pdata)
{
    if (pdata) {
        memcpy(&(g_ephy_sta.st_motrun_real_sta), pdata, sizeof(sandmot_real_t));
        return 0;
    }
    return -1; 
}
int set_exphy_motqt_cfgdata(const sandmot_qtcfg_t *pdata)
{
    if (pdata) {
        memcpy(&(g_ephy_sta.st_motqt_cfg_sta), pdata, sizeof(sandmot_qtcfg_t));
        return 0;
    }
    return -1; 
}
int set_exphy_motqt_realdata(const sandmot_qtreal_t *pdata)
{
    if (pdata) {
        memcpy(&(g_ephy_sta.st_motqt_real_sta), pdata, sizeof(sandmot_qtreal_t));
        return 0;
    }
    return -1; 
}
int set_exphy_weight_data(const unsigned char *pdata)
{
    if (pdata) {
        memcpy(&(g_ephy_sta.weight_data), pdata, sizeof(unsigned int));
        return 0;
    }
    return -1;
}
int pkdrv_set_exphy_weight_data(const unsigned char *pdata)
{
    if (pdata) {
        unsigned char _cnt = 0;
        gst_weight.size = pdata[_cnt];
        _cnt++;    
        memcpy(&gst_weight.a_weight, &pdata[_cnt], sizeof(unsigned int) * gst_weight.size);
        return 0;
    }
    return -1;
}
int set_exphy_err_data(const errcode_t *pdata)
{
    if (pdata) {
        memcpy(&(g_ephy_sta.st_errcode), pdata, sizeof(errcode_t));
        return 0;
    }
    return -1;
}
int set_exphy_ver_data(const version_t *pdata)
{
    if (pdata) {
        memcpy(&(g_ephy_sta.st_version), pdata, sizeof(version_t));
        return 0;
    }
    return -1;  
}

int pkmcu_check_comm(int timeout, unsigned char block)                  // 检测esp<->mcu之间通信协议是否准确
{
    int _wait_time = timeout;
    int _ret = 0;

    if (_wait_time == 0) { _wait_time = MCU_COMM_CHECK_DEF_TIME; }
     // 1. 发送心跳指令，判断是否收到心跳指令
     // 2. 判断是否有新的Sensor指令报上来
    do {
        if (EXPHY_UPSTA_NEW == pkmcu_get_exphydata(E_EXPHY_HB, E_INFO_BLK, 1, 0)) {
            block = 0;
            _ret = 1;
        } else {
            if (_wait_time-- == 0) {
                return -1;
            }
        }
    } while(block);
    return _ret;
}

int pkmcu_set_motrun_cfgdata(unsigned char type, unsigned char pos, unsigned char sta, unsigned char speed, unsigned short step, unsigned short max_step, unsigned short over_time, unsigned char bfInt)       // 待猜测
{
    sandmot_runcfg_t stvalue;
    stvalue.type = type;
    stvalue.pos = pos;
    stvalue.mot_sta = sta;
    stvalue.mot_speed = speed;
    stvalue.u16_stepnum = step;
    stvalue.u16_maxstepnum = max_step;
    stvalue.u16_overtime = over_time;
    return pk_esp_mcu_comm(COMM_CMD_MOT_RUN_CFG, COMM_WRITE_CMD, 0, (const unsigned char *)&stvalue, sizeof(sandmot_runcfg_t), bfInt);
}
int pkmcu_set_mot_run(sandmot_runcfg_t data, unsigned char bfInt)
{
    return pk_esp_mcu_comm(COMM_CMD_MOT_RUN_CFG, COMM_WRITE_CMD, 0, (const unsigned char *)&data, sizeof(sandmot_runcfg_t), bfInt);
}
int pkmcu_set_motqt_cfgdata(unsigned char uptime, unsigned char downtime, unsigned short optime, unsigned short nm1st_time, unsigned short nm2nd_time, unsigned short opcurt, unsigned short nmh_curt, unsigned short nml_curt, unsigned char bfInt)
{
    sandmot_qtcfg_t stvalue;
    stvalue.u08_up_time = uptime;
    stvalue.u08_down_time = downtime;
    stvalue.u16_op_time = optime;
    stvalue.u16_1stnm_time = nm1st_time;
    stvalue.u16_2ndnm_time = nm2nd_time;
    stvalue.u16_op_curt = opcurt;
    stvalue.u16_nmh_curt = nmh_curt;
    stvalue.u16_nml_curt = nml_curt;
    return pk_esp_mcu_comm(COMM_CMD_MOT_QT_CFG, COMM_WRITE_CMD, 0, (const unsigned char *)&stvalue, sizeof(sandmot_qtcfg_t), bfInt);
}
int pkmcu_set_exphy_outpin(exphy_outpin_node node, unsigned char mode, unsigned char times, unsigned char on_time, unsigned char off_time, unsigned char bfInt) // 待测试
{
    out_pin_t _stvalue = { 0, 0, 0, 0 };
    _stvalue.mode = mode;
    _stvalue.times = times;
    _stvalue.on_time = on_time;
    _stvalue.off_time = off_time;
    return pk_esp_mcu_comm(COMM_CMD_OUTCTRL, COMM_WRITE_CMD, node, (const unsigned char *)&_stvalue, sizeof(out_pin_t), bfInt);
}
int pkmcu_get_errcode(unsigned char bfInt)        // 读取错误代码 - OK
{
    return pk_esp_mcu_comm(COMM_CMD_ERR, COMM_READ_CMD, 0, NULL, 0, bfInt);
}
int pkmcu_get_version(unsigned char bfInt)        // 读取MCU软件版本 - OK
{
    return pk_esp_mcu_comm(COMM_CMD_VER, COMM_READ_CMD, 0, NULL, 0, bfInt);
}
int pkmcu_reset_mcu(unsigned char bfInt)          // 复位MCU - OK
{
    return pk_esp_mcu_comm(COMM_CMD_RST, COMM_WRITE_CMD, 0, NULL, 0, bfInt);
}
int pkmcu_set_mcuworkmode(mcu_work_mode mode, unsigned char bfInt)  // 设置工作模式 - OK
{
    unsigned char au08data[1] = { 0 };
    au08data[0] = (unsigned char)mode;
    return pk_esp_mcu_comm(COMM_CMD_WORKMODE, COMM_WRITE_CMD, 0, (const unsigned char *)&au08data, sizeof(au08data), bfInt);
}
int pkmcu_set_configdata(const unsigned char *pu08data, unsigned char size, unsigned char bfInt)    // OK
{
    return pk_esp_mcu_comm(COMM_CMD_CFGDATA, COMM_WRITE_CMD, 0, pu08data, size, bfInt);
}
int pkmcu_set_weight_data(unsigned char bfen, unsigned int time, unsigned char bfInt)
{
    set_weight_t _stvalue = { 0, 0 };
    _stvalue.bfEnable = bfen;
    _stvalue.checktime = time;
    return pk_esp_mcu_comm(COMM_CMD_WEIGHT, COMM_WRITE_CMD, 0, (const unsigned char *)&_stvalue, sizeof(set_weight_t), bfInt);
}
int pkmcu_get_motreal_data(mot_drvreal_node node, eInfoMode mode, int timeout, unsigned char bfInt)    // 等待测试
{
    exphy_upsta_t upsta = { 0, 0, 0 };
    drv_physta_t status;
    int wait_time = 1;

    wait_time = timeout ? (timeout * 10 + 1) : 1;
    if (E_INFO_NOW == mode) {
        goto _get_sta;
    }

    pkmcu_clean_mot_realupsta(node);
    if ((node >= E_MOTREAL_STA) && (node <= E_MOTREAL_RUNTIME)) {
        pkmcu_clean_exphy_upsta(E_EXPHY_MOTRUN_REAL);
        pk_esp_mcu_comm(COMM_CMD_MOT_RUN_RT, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if ((node >= E_MOTREAL_UPTIME) && (node <= E_MOTREAL_DOWNTIME)) {
        pkmcu_clean_exphy_upsta(E_EXPHY_MOTQT_REAL);
        pk_esp_mcu_comm(COMM_CMD_MOT_QT_RT, COMM_READ_CMD, 0, NULL, 0, bfInt);
    }

    if (E_INFO_NOBLK == mode) {
        goto _get_sta;
    }

    while(wait_time--) {
        pkmcu_get_mot_realupsta(node, &upsta);
        if (EXPHY_UPSTA_NEW == upsta.upsta) {
_get_sta:
            get_phy_status(&status);
            switch (node)
            {
            case E_MOTREAL_STA:
                return status.st_motrun_real_sta.mot_sta;
                break;
            case E_MOTREAL_SPEED:
                return status.st_motrun_real_sta.mot_speed;
                break;
            case E_MOTREAL_CURT:
                return status.st_motrun_real_sta.u16_mot_curt;
                break;
            case E_MOTREAL_STEP:
                return status.st_motrun_real_sta.u16_mot_stepnum;
                break;
            case E_MOTREAL_RUNTIME:
                return status.st_motrun_real_sta.u16_mot_runtime;
                break;
            case E_MOTREAL_UPTIME:
                return status.st_motqt_real_sta.u08_up_time;
                break;
            case E_MOTREAL_DOWNTIME:
                return status.st_motqt_real_sta.u08_down_time;
                break;
            default:
                break;
            }
        } else {
            vTaskDelay((portTickType)100/portTICK_RATE_MS);
            continue;
        }
    }

    return -1;
}
int pkmcu_get_exphydata(exphy_upsta_node index, eInfoMode mode, int timeout, unsigned char bfInt)
{
    exphy_upsta_t upsta = { 0, 0, 0 };
    drv_physta_t status;

    int wait_time = 1;

    wait_time = timeout ? (timeout * 10 + 1) : 1;
    if (E_INFO_NOW == mode) {
        goto get_sta;
    }
    if (index == E_EXPHY_HB) {
        pkmcu_clean_exphy_upsta(E_EXPHY_HB);
        pk_esp_mcu_comm(COMM_CMD_HB, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if ((index >= E_EXPHY_SENSOR) && (index <= E_EXPHY_DET_WAKEUP)) {
        pkmcu_clean_exphy_upsta(E_EXPHY_SENSOR);
        pkmcu_clean_exphy_upsta(index);
        pk_esp_mcu_comm(COMM_CMD_SENSOR, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if (index == E_EXPHY_WEIGHT) {
        pkmcu_clean_exphy_upsta(E_EXPHY_WEIGHT);
        pk_esp_mcu_comm(COMM_CMD_WEIGHT, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if (index == E_EXPHY_MOTRUN_CFG) {
        pkmcu_clean_exphy_upsta(E_EXPHY_MOTRUN_CFG);
        pk_esp_mcu_comm(COMM_CMD_MOT_RUN_CFG, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if (index == E_EXPHY_MOTRUN_REAL) {
        pkmcu_clean_exphy_upsta(E_EXPHY_MOTRUN_REAL);
        pk_esp_mcu_comm(COMM_CMD_MOT_RUN_RT, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if (index == E_EXPHY_MOTQT_CFG) {
        pkmcu_clean_exphy_upsta(E_EXPHY_MOTQT_CFG);
        pk_esp_mcu_comm(COMM_CMD_MOT_QT_CFG, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if (index == E_EXPHY_MOTQT_REAL) {
        pkmcu_clean_exphy_upsta(E_EXPHY_MOTQT_REAL);
        pk_esp_mcu_comm(COMM_CMD_MOT_QT_RT, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if (index == E_EXPHY_VERSION) {
        pk_esp_mcu_comm(COMM_CMD_VER, COMM_READ_CMD, 0, NULL, 0, bfInt);
    } else if (index == E_EXPHY_ERRCODE) {
        pk_esp_mcu_comm(COMM_CMD_ERR, COMM_READ_CMD, 0, NULL, 0, bfInt);
    }

    if (E_INFO_NOBLK == mode) {
        goto get_sta;
    }

    while(wait_time--) {
        pkmcu_get_exphy_upsta(index, &upsta);
        if (EXPHY_UPSTA_NEW == upsta.upsta) {
get_sta: 
            get_phy_status(&status);
            switch (index) 
            {
            case E_EXPHY_HB:
                return upsta.upsta;
                break;
            case E_EXPHY_SENSOR:
                return upsta.upsta;
                break;
            case E_EXPHY_MOT_ADC:
                return status.st_sensor_sta.mot_adc;
                break;
            case E_EXPHY_WHQ_ADC:
                return status.st_sensor_sta.whq_adc;
                break;
            case E_EXPHY_DC_ADC:
                return status.st_sensor_sta.dc_adc;
                break;
            case E_EXPHY_DET_WH1:
                return ((status.st_sensor_sta.io_det >> DET_WH1) & 0x01);
                break;
            case E_EXPHY_DET_WH2:
                return ((status.st_sensor_sta.io_det >> DET_WH2) & 0x01);
                break;
            case E_EXPHY_DET_WH3:
                return ((status.st_sensor_sta.io_det >> DET_WH3) & 0x01);
                break;
            case E_EXPHY_DET_SAND_ACT:
                return ((status.st_sensor_sta.io_det >> DET_SAND_ACT) & 0x01);
                break;
            case E_EXPHY_DET_WS_OFF:
                return ((status.st_sensor_sta.io_det >> DET_WS_OFF) & 0x01);
                break;
            case E_EXPHY_DET_SAND_L:
                return ((status.st_sensor_sta.io_det >> DET_SAND_L) & 0x01);
                break;
            case E_EXPHY_DET_SAND_R:
                return ((status.st_sensor_sta.io_det >> DET_SAND_R) & 0x01);
                break;
            case E_EXPHY_DET_WAKEUP:
                return ((status.st_sensor_sta.io_det >> DET_WAKEUP) & 0x01);
                break;
            case E_EXPHY_WEIGHT:
                return status.weight_data;
                break;
            case E_EXPHY_MOTRUN_CFG:
            case E_EXPHY_MOTRUN_REAL:
            case E_EXPHY_MOTQT_CFG:
            case E_EXPHY_MOTQT_REAL:
                return upsta.upsta;
                break;
            case E_EXPHY_ERRCODE:
                return upsta.upsta;
                break;
            case E_EXPHY_VERSION:
                return upsta.upsta;
                break;
            default:
                break;
            }
        } else {
            vTaskDelay((portTickType)100/portTICK_RATE_MS);
            continue;
        }
    }

    return -1;
}

int pkmcu_get_weight(unsigned char bfInt)
{
    int _ret = 0;
    _ret = pk_esp_mcu_comm(COMM_CMD_WEIGHT, COMM_READ_CMD, 0, NULL, 0, bfInt);
    return _ret;
}
int pkmcu_set_weight(unsigned char enable, unsigned char size, unsigned int gather_time, unsigned int rpt_time, unsigned char bfInt)
{
    int _ret = 0;
    weight_cfg_t _stvalue = { 0, 0, 0, 0};
    _stvalue.bfenable = enable;
    _stvalue.size = size;
    _stvalue.gather_time = gather_time;
    _stvalue.rpt_time = rpt_time;

    _ret = pk_esp_mcu_comm(COMM_CMD_WEIGHT, COMM_WRITE_CMD, 0, (const unsigned char *)&_stvalue, sizeof(weight_cfg_t), bfInt);
    return _ret;
}

static void key_event_sendToQueue(pk_gpio_num_e pin, int event)
{
    if (DX_WIFI_KEY_GPIO == pin) {
        // sys_xQueSendToBackFromISR(xQueueFoodEvent, FOODEVENT_KEY, event, NULL);
        if (pk_get_detect_sta() == pdDETECT_WAIT) {
            pk_send_itemtest_event(E_TEST_KEY_ITEM, E_DET_WIFI_BTN, NULL, 1);
        }
    } else if (DX_FUNC_KEY_GPIO == pin) {
        // sys_xQueSendToBackFromISR(xQueueWifiEvent, WIFI_KEY, event, NULL);
        if (pk_get_detect_sta() == pdDETECT_WAIT) {
            pk_send_itemtest_event(E_TEST_KEY_ITEM, E_DET_FUNC_BTN, NULL, 1);
        }

    }
}

/***电机阻塞控制**********************************/
int pkmcu_block_ctrl(unsigned char block, unsigned char type, unsigned char sta, int timeout_sec)          // mcu阻塞控制 - 待开发
{
    int _ret = 0;
    pkLOG_INFO(" %s Func, wait user to develop! %d \n", __func__, __LINE__);
    return _ret;
}
/***********************************************/

void sys_init_phy_iod(void)
{
    pk_dev_init_gpio(key_event_sendToQueue);
}

void close_esp_devices(void)
{
    pkhal_beep_ctrl(0);                                     // 0. 关闭蜂鸣器
    pkhal_set_whqpower_sta(0);                              // 1. 关闭雾化器
    // pkmcu_block_ctrl(1, E_SANDMOT_POS_T, E_CW, 120);    // 1. 关闭运行电机 - 电机阻塞左到位
}

#undef __PK_T3_DRIVER_C_
