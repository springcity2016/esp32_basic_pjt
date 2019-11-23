
#ifndef __PK_T3_DRIVER_H_
#define __PK_T3_DRIVER_H_


#include "serial_comm_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MCU_COMM_CHECK_DEF_TIME             (10)    // esp<->mcu通信检查默认时间, 单位:1Sec

typedef enum {
    E_INFO_NOW = 0,
    E_INFO_BLK,
    E_INFO_NOBLK,
    E_INFO_MAX
}eInfoMode;

#define EXPHY_UPSTA_OLD                     (0)     //未更新
#define EXPHY_UPSTA_NEW                     (1)     //已更新

typedef struct _exphy_update_sta_t
{
    short seq;                              //sequence
    char  upsta;                            //EXPHY_UPSTA_OLD or EXPHY_UPSTA_NEW
    int   update_time;                      //设备数据更新时间
} exphy_upsta_t;

typedef enum
{
    E_EXPHY_HB = 0,
    E_EXPHY_SENSOR,
    E_EXPHY_MOT_ADC,
    E_EXPHY_WHQ_ADC,
    E_EXPHY_DC_ADC,
    E_EXPHY_DET_WH1,
    E_EXPHY_DET_WH2,
    E_EXPHY_DET_WH3,
    E_EXPHY_DET_SAND_ACT,
    E_EXPHY_DET_WS_OFF,
    E_EXPHY_DET_SAND_L,         // 起始位置
    E_EXPHY_DET_SAND_R,         // 倾沙位置
    E_EXPHY_DET_WAKEUP,

    E_EXPHY_MOTRUN_CFG,
    E_EXPHY_MOTRUN_REAL,
    E_EXPHY_MOTQT_CFG,
    E_EXPHY_MOTQT_REAL,

    E_EXPHY_WEIGHT,

    E_EXPHY_ERRCODE,
    E_EXPHY_VERSION,

    E_EXPHY_UPSTA_MAX,
}exphy_upsta_node;

typedef enum
{
	DET_WH1 = 0,		// 排污盒光栅1			   	// PA0
	DET_WH2,			// 排污盒光栅2				// PA1
	DET_WH3,			// 排污盒光栅3				// PA2
	DET_SAND_ACT,		// 沙筒旋转信号				// PA4
	DET_WS_OFF,			// 污水位置					// PB4
	DET_SAND_L,			// 沙筒到左位置				// PB5
	DET_SAND_R,			// 沙筒到右位置				// PB6
	DET_WAKEUP,			// WakeUp 管脚接口		  	// PA13

    DET_IR_HEAT,        // 红外释热
    DET_INT_DOOR,       // 进门光栅

	DET_MAX
}IO_DET;

typedef enum
{
    E_EXPHY_OUTPIN_IR = 0,

    E_EXPHY_OUTPIN_MAX,
}exphy_outpin_node;

typedef struct _set_weight_t
{
    unsigned char bfEnable;
    unsigned int checktime;
}set_weight_t;

typedef enum
{
    E_MOTREAL_STA = 0,
    E_MOTREAL_SPEED,
    E_MOTREAL_CURT,
    E_MOTREAL_STEP,
    E_MOTREAL_RUNTIME,

    E_MOTREAL_UPTIME,
    E_MOTREAL_DOWNTIME,

    E_MOTREAL_MAX
}mot_drvreal_node;

typedef enum
{
    E_NORMAL_MODE = 0,      
    E_LOWPWR_MOE,

    E_MAX_MODE
}mcu_work_mode;

int pk_os_runtime(void);

int pkmcu_init_exphy_upsta(void);
int pkmcu_clean_all_exphy_upsta(void);
int pkmcu_clean_exphy_upsta(exphy_upsta_node dev_node);
int pkmcu_set_exphy_upsta(exphy_upsta_node dev_node, int seq, int time);
int pkmcu_get_exphy_upsta(exphy_upsta_node dev_node, exphy_upsta_t *upsta);

int pkmcu_init_motreal_upsta(void);
int pkmcu_clean_allmot_realupsta(void);
int pkmcu_clean_mot_realupsta(mot_drvreal_node dev_node);
int pkmcu_set_mot_realupsta(mot_drvreal_node dev_node, int seq, int time);
int pkmcu_get_mot_realupsta(mot_drvreal_node dev_node, exphy_upsta_t *upsta);

int get_phy_status(drv_physta_t* lastest_device_status);
int set_exphy_sensordata(sensor_sta_t *params);
int set_exphy_motrun_cfgdata(const sandmot_runcfg_t *pdata);
int set_exphy_motrun_realdata(const sandmot_real_t *pdata);
int set_exphy_motqt_cfgdata(const sandmot_qtcfg_t *pdata);
int set_exphy_motqt_realdata(const sandmot_qtreal_t *pdata);
int set_exphy_weight_data(const unsigned char *pdata);
int pkdrv_set_exphy_weight_data(const unsigned char *pdata);
int set_exphy_err_data(const errcode_t *pdata);
int set_exphy_ver_data(const version_t *pdata);

int pkmcu_check_comm(int timeout, unsigned char block);                  // 检测esp<->mcu之间通信协议是否准确

int pkmcu_set_exphy_outpin(exphy_outpin_node node, unsigned char mode, unsigned char times, unsigned char on_time, unsigned char off_time, unsigned char bfInt); // 待测试
int pkmcu_get_errcode(unsigned char bfInt);        // 读取错误代码 - OK
int pkmcu_get_version(unsigned char bfInt);        // 读取MCU软件版本 - OK
int pkmcu_reset_mcu(unsigned char bfInt);          // 复位MCU - OK
int pkmcu_set_mcuworkmode(mcu_work_mode mode, unsigned char bfInt);  // 设置工作模式 - OK
int pkmcu_set_configdata(const unsigned char *pu08data, unsigned char size, unsigned char bfInt);       // 设置数据 - OK
int pkmcu_get_motreal_data(mot_drvreal_node node, eInfoMode mode, int timeout, unsigned char bfInt);    // 等待测试
int pkmcu_set_motrun_cfgdata(unsigned char type, unsigned char pos, unsigned char sta, unsigned char speed, unsigned short step, unsigned short max_step, unsigned short over_time, unsigned char bfInt);       // OK
int pkmcu_set_mot_run(sandmot_runcfg_t data, unsigned char bfInt);

int pkmcu_set_motqt_cfgdata(unsigned char uptime, unsigned char downtime, unsigned short optime, unsigned short nm1st_time, unsigned short nm2nd_time, unsigned short opcurt, unsigned short nmh_curt, unsigned short nml_curt, unsigned char bfInt);       // 待测试
int pkmcu_set_weight_data(unsigned char bfen, unsigned int time, unsigned char bfInt);                  // 等待测试
int pkmcu_get_exphydata(exphy_upsta_node index, eInfoMode mode, int timeout, unsigned char bfInt);      // 等待测试

int pkmcu_get_weight(unsigned char bfInt);
int pkmcu_set_weight(unsigned char enable, unsigned char size, unsigned int gather_time, unsigned int rpt_time, unsigned char bfInt);

/***电机阻塞控制**********************************/
int pkmcu_block_ctrl(unsigned char block, unsigned char type, unsigned char sta, int timeout_sec);          // mcu阻塞控制 - 待开发
/***********************************************/

void sys_init_phy_iod(void);

void close_esp_devices(void);


#ifdef __cplusplus
}
#endif


#endif

