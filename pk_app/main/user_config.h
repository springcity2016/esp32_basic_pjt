/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

//#include "esp_common.h"
#include "esp_system.h"
#include "esp_wifi.h"
// #include "iot_export.h"
//#include "upgrade.h"
//#include "feeder.h"

#define pdNULL_ERROR                        (-1)
/*****系统功能模块的开启*****/
#define FEEDER_PORTS_FUNC                   (1)
#define FEEDER_PORTS_TEST                   (0)

#define FEEDER_ERASE_ID_TEST                (0)
/*****测试模式下的宏定义*****/
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

#define OPEN_PHY                            (1)
#define CLOSE_PHY                           (0)

#define FEEDER_UART_DEBUG                   (0)
#define FEEDER_TASK_DEBUG                   (0)
/*****mac重复模式下的宏定义*****/
#define pdDEAL_OK                           (1)
#define pdDEAL_WAIT                         (2)

/***************系统消息显示定义*****************/
#define SYS_TEST_CLEAN_ALL_CFG              (0)	    //系统将清楚全部数据，谨慎使用
#define SYS_TEST_UPGRADE                    (0)		//系统反复测试宏定义，一定开启，系统将重复进行升级,初始化设备的异常
#define SYS_CLEAN_SN_DATA                   (0)

#define FUNCEVENT_CMD_MAX                   (15)    //消息队列长度

#define SYS_FUNC_FEEDREP_SAVE_EN            (1)     //系统是否支持喂食报告保存处理
#define SYS_FUNC_FEEDSCH_SAVE_EN            (1)     //系统是否支持喂食计划保存处理

#define SYS_FUNC_DETECT_AP_EN               (0)     //系统用于测试的ap模式
#define SYS_FUNC_DETECT_APBOARD_EXIST       (1)     //系统用于测试的外接ap板是否存在

#define SYS_FUNC_APP_LOG_IN_API             (0)     //系统是否在report接口中携带log信息
#define SYS_FUNC_DOOR_DELAY_EN              (1)     //系统开启关门的夹手优化功能
#define SYS_ST_PLD_ERR_FOOD_EN              (1)		//系统开启开盖时的喂粮允许（服务器必须配合对高等级的去除变为低等级）
/********************************************************************************/
#define APP_TIMEZONE_DIS_DOOR               "4.0"
/***************************系统实时api消息类型******************************/
#define API_FEED_LIST_GET_MODE_ALONE        (0)			//api接口一次拿完


#if 1		// 用D2接口

#define API_DEV_SERVERINFO                  "/feedermini/dev_serverinfo"                //获取服务信息
#define API_DEV_SYNCTIME                    "/feedermini/dev_synctime"                  //同步时间
#define API_DEV_FEED_GET                    "/feedermini/dev_feed_get"                  //获取喂食信息
#define API_DEV_FEED_CONSUME_EVENT          "/feedermini/dev_feed_consume_event"        //消费喂食计划改变事件
#define API_DEV_FEED_REPORT                 "/feedermini/dev_feed_report"               //上报喂食情况
#define API_DEV_STATE_REPORT                "/feedermini/dev_state_report"              //上报设备情况
#define API_DEV_FEED_START                  "/feedermini/dev_feed_start"                //喂食开始
#define API_DEV_OTA_CHECK                   "/feedermini/dev_ota_check"                 //检查是否有新固件可升级
#define API_DEV_OTA_START                   "/feedermini/dev_ota_start"                 //报告开始OTA升级
#define API_DEV_OTA_COMPLETE                "/feedermini/dev_ota_complete"              //报告OTA升级结束
#define API_DEV_OTA_HEARTBEAT               "/feedermini/dev_ota_heartbeat"             //报告OTA进度(保持心跳)
#define API_DEV_SIGNUP                      "/feedermini/dev_signup"                    //设备自己注册
#define API_DEV_SAVELOG                     "/feedermini/dev_r"                         //设备日志（暂未启用）
#define API_IOT_DEVINFO                     "/feedermini/dev_iot_device_info"           //获取ali IOT三元信息
#define API_DEV_DEVINFO                     "/feedermini/dev_device_info"               //获取设备设置信息
#define API_DEV_HEARTBEAT                   "/poll/feedermini/heartbeat"                //http心跳
#define API_DEV_CLEAN_WT                    "/feeder/dev_set_food_reset"

//向service请求的type，1和2比较特殊不在这里
#define API_TYPE_SERVERINFO                 "fm_dev_serverinfo"                         //3.获取服务信息
#define API_TYPE_SYNCTIME                   "fm_dev_synctime"                           //4.同步时间
#define API_TYPE_FEED_GET                   "fm_dev_feed_get"                           //5.获取喂食信息
#define API_TYPE_FEED_CONSUME_EVENT         "fm_dev_feed_consume_event"                 //6.消费喂食计划改变事件
#define API_TYPE_FEED_START                 "fm_dev_feed_start"                         //7.开始喂食上报
#define API_TYPE_FEED_REPORT                "fm_dev_feed_report"                        //8.上报喂食情况
#define API_TYPE_STATE_REPORT               "fm_dev_state_report"                       //9.上报喂食机硬件状态
#define API_TYPE_OTA_CHECK                  "fm_dev_ota_check"                          //10.检查是否有新固件升级
#define API_TYPE_OTA_START                  "fm_dev_ota_start"                          //11.报告开始OTA升级
#define API_TYPE_OTA_COMPLETE               "fm_dev_ota_complete"                       //12.报告OTA升级结束
#define API_TYPE_OTA_HEARTBEAT              "fm_dev_ota_heartbeat"                      //13.报告OTA进度(保持心跳)
#define API_TYPE_SAVELOG                    "fm_dev_savelog"                            //14.上传log
#define API_TYPE_DEVICE_INFO                "fm_dev_device_info"                        //15.获取设备配置信息
#define API_TYPE_SET_FOOD_RESET             "fm_dev_set_food_reset"                     //16.上报清空状态

#else 		// 用D1接口

#define API_DEV_SERVERINFO                  "/feeder/dev_serverinfo"                //获取服务信息
#define API_DEV_SYNCTIME                    "/feeder/dev_synctime"                  //同步时间
#define API_DEV_FEED_GET                    "/feeder/dev_feed_get"                  //获取喂食信息
#define API_DEV_FEED_CONSUME_EVENT          "/feeder/dev_feed_consume_event"        //消费喂食计划改变事件
#define API_DEV_FEED_REPORT                 "/feeder/dev_feed_report"               //上报喂食情况
#define API_DEV_STATE_REPORT                "/feeder/dev_state_report"              //上报设备情况
#define API_DEV_FEED_START                  "/feeder/dev_feed_start"                //喂食开始
#define API_DEV_OTA_CHECK                   "/feeder/dev_ota_check"                 //检查是否有新固件可升级
#define API_DEV_OTA_START                   "/feeder/dev_ota_start"                 //报告开始OTA升级
#define API_DEV_OTA_COMPLETE                "/feeder/dev_ota_complete"              //报告OTA升级结束
#define API_DEV_OTA_HEARTBEAT               "/feeder/dev_ota_heartbeat"             //报告OTA进度(保持心跳)
#define API_DEV_SIGNUP                      "/feeder/dev_signup"                    //设备自己注册
#define API_DEV_SAVELOG                     "/feeder/dev_r"                         //设备日志（暂未启用）
#define API_IOT_DEVINFO                     "/feeder/dev_iot_device_info"           //获取ali IOT三元信息
#define API_DEV_DEVINFO                     "/feeder/dev_device_info"               //获取设备设置信息
#define API_DEV_HEARTBEAT                   "/poll/heartbeat"                       //http心跳
#define API_DEV_CLEAN_WT                    "/feeder/dev_set_food_reset"

//向service请求的type，1和2比较特殊不在这里
#define API_TYPE_SERVERINFO                 "f_dev_serverinfo"                         //3.获取服务信息
#define API_TYPE_SYNCTIME                   "f_dev_synctime"                           //4.同步时间
#define API_TYPE_FEED_GET                   "f_dev_feed_get"                           //5.获取喂食信息
#define API_TYPE_FEED_CONSUME_EVENT         "f_dev_feed_consume_event"                 //6.消费喂食计划改变事件
#define API_TYPE_FEED_START                 "f_dev_feed_start"                         //7.开始喂食上报
#define API_TYPE_FEED_REPORT                "f_dev_feed_report"                        //8.上报喂食情况
#define API_TYPE_STATE_REPORT               "f_dev_state_report"                       //9.上报喂食机硬件状态
#define API_TYPE_OTA_CHECK                  "f_dev_ota_check"                          //10.检查是否有新固件升级
#define API_TYPE_OTA_START                  "f_dev_ota_start"                          //11.报告开始OTA升级
#define API_TYPE_OTA_COMPLETE               "f_dev_ota_complete"                       //12.报告OTA升级结束
#define API_TYPE_OTA_HEARTBEAT              "f_dev_ota_heartbeat"                      //13.报告OTA进度(保持心跳)
#define API_TYPE_SAVELOG                    "f_dev_savelog"                            //14.上传log
#define API_TYPE_DEVICE_INFO                "f_dev_device_info"                        //15.获取设备配置信息
#define API_TYPE_SET_FOOD_RESET             "f_dev_set_food_reset"                     //16.上报清空状态

#endif

/***************************系统实时app版本定义******************************/
#define HTTP_API_VERSION                    "6.7"
/***************************系统实时消息类型与时间戳保存******************************/
typedef enum
{
    E_IM_SCHEDULE = 0,
    E_IM_REALTIME,
    E_IM_CANCEL,
    E_IM_OTA,
    E_IM_UNLINK,
    E_IM_LINK,
    E_IM_MANUAL_SETTING,
    E_IM_LOGLEVEL,
    E_IM_REBOOT,
    E_IM_REPAIRDOOR,
    E_IM_CLEAN_WT,

    E_IM_MAX
} E_IM_type;

#define	IM_FEEDER_SCHEDULE 					"feed_schedule"
#define	IM_FEEDER_REALTIME 					"feed_realtime"
#define	IM_FEEDER_CANCEL 					"feed_realtime_cancel"
#define	IM_FEEDER_OTA 						"ota"
#define	IM_FEEDER_UNLINK					"unlink"
#define	IM_FEEDER_LINK						"link"
#define IM_FEEDER_MANUAL_SETTING			"manual_lock"
#define	IM_FEEDER_LOGLEVEL					"loglevel"
#define IM_FEEDER_REBOOT					"devreboot"
#define IM_FEEDER_REPAIRDOOR				"repair_door"
#define IM_FEEDER_CLEAN_WT					"food_action"

/***************************系统校准失败原因分析说明******************************/
#define DEF_WT_CLEAN_START					"start"					//启动校准
#define DEF_WT_CLEAN_OK						"ok"					//校准成功
#define DEF_WT_CLEAN_TIMER					"timer"					//定时校准等待
#define DEF_WT_CLEAN_CANCEL					"cancel"				//校准取消
#define DEF_WT_CLEAN_ERR_INDOING			"err-indoing"			//正在校准中不可再次校准
#define DEF_WT_CLEAN_ERR_ZERO				"err-zero"				//秤小于0读数不可校准
#define DEF_WT_CLEAN_ERR_CREATLIST			"err-creatlist"			//创建校准列表失败
#define DEF_WT_CLEAN_ERR_INFEEDING			"err-feeding"			//正在喂粮不可校准
#define DEF_WT_CLEAN_ERR_DOOR1				"err-dr1"				//关门态错误不可校准
#define DEF_WT_CLEAN_ERR_DOOR2				"err-dr2"				//开门态错误不可校准
#define DEF_WT_CLEAN_ERR_WT1				"err-wt1"				//关门态秤错误不可校准
#define DEF_WT_CLEAN_ERR_WT2				"err-wt2"				//开门态秤错误不可校准
#define DEF_WT_CLEAN_ERR_WL1				"err-wl1"				//关门态叶轮故障不可校准
#define DEF_WT_CLEAN_ERR_WL2				"err-wl2"				//开门态叶轮故障不可校准
#define DEF_WT_CLEAN_ERR_OPT				"err-opt"				//校准执行失败退出

typedef struct _sys_cmd_timestmp_t
{
	int	sys_feed_schedule;
	int sys_feed_realtime;
	int sys_feed_realtime_cancel;
	int sys_unlink;
	int sys_OTA;
	int sys_link;
	int sys_setting;
	int sys_repair_door;
} sys_cmd_timestmp_t;

/***************************系统实时喂食的允许延迟时间******************************/
#define DELAY_SCHEDULE_TIME								(10)			//10秒内的数据可以再次喂
#define DELAY_SCHEDULE_TIME_MAX                         (60)            //10秒内的数据可以再次喂
/* NOTICE !!!*/
/* You can change to other sector if you use other size spi flash. */
/* Refer to the documentation about OTA support and flash mapping*/
/*0x70000,4k sector(1000),sec=*/
/**定义存储serverinfo信息，采用安全模式写入**/
//#define PET_SERVER_SAVE     	0xF0
#define OFFSET_STR				0

/**定义设备系统信息，采用安全模式写入**/
//#define PET_DEVICE_SAVE    		0xF3
//#define PET_DEVICE_SAVE_BACK   	0xFD
#define OFFSET_PARAM			0			//LEN 512

/**定义计划喂食信息，采用安全模式写入**/
//#define PET_SCHDULE_SAVE    	0xF6

/**定义报告信息，采用安全模式写入，每个报告占用128字节，可容纳32个喂食报告，该定义放置在drive.h文件中**/
//#define PET_REPORT_SAVEA    	0xF9
//#define PET_REPORT_SAVEB    	0xFA
//#define OFFSET_REP_LEN			256			//LEN 512

//只剩下如下的空间地址
/**保留地址**/
#define SXXX1    	0xFB
/**保留地址**/
#define SXXX2   	0xFC


/*有关soft ap 设置参数*/
#define SOFT_AP_PWD_EN                      (0)

#if 1
#define SOFT_AP_HW                          "HW2"
#define SOFT_AP_DETECT                      "HW2_RSSI"
#define SOFT_AP_SSID                        "PETKIT_FEEDER_"
#define SOFT_AP_PASSWORD                    "petkit1234"
#define SOFT_AP_NULL                        "PETKIT_AP_NULL"
#define SOFT_AP_NULL_PSW                    SOFT_AP_PASSWORD
#else
#define SOFT_AP_HW                          "HW2"
#define SOFT_AP_DETECT                      "HW2_RSSI"
#define SOFT_AP_SSID                        "PETKIT_AP_"
#define SOFT_AP_PASSWORD                    "petkit1234"
#define SOFT_AP_NULL                        "PETKIT_AP_NULL"
#define SOFT_AP_NULL_PSW                    SOFT_AP_PASSWORD
#endif


#define WIFI_MANUAL_WAIT_TIME		        (5)			//UNIT MIN
#define WIFI_AP_NUMBER_MAX		            (2)	//max=5

#pragma pack(4)
typedef struct _dev_param_t
{
    int rgt;
    char username[12];
    char id[20];
    char srt[20];
    char sn[32];
    char mac[16];
    char host[100];
    char type[12];
    // iotx_device_info_t iotx_dev_info;
    int pimtimeindex;
    int timezone;
    int serverlen;              //存储serverinfo有多长
    char *serverinfo;           //存储运行时保存的指针位置
    int ota_flag;//十位用于标志是否ota，个位用于指示bank位置
    int ota_err_cnt;            //秤校准模式
    int taregram;               //皮重
    char apptimezone[8];            //存放dev  的  timezone 值
    int verifyvalue[5];
    int addgram;        //每次更新粮后的附加重量
    int averagegram;        //平均值
    char flag[16];      //防止mac重复将softap与stationap交换
    int listschlen;                 //存储list schdule有多长
    char *listschinfo;          //存储运行时list schdule保存的指针位置
    int otafailNo;          //ota失败原因标志
    unsigned int  led_on;
    unsigned int  led_off;
    unsigned int  verify_timestamp;
    int manual_max_gram;
    int flashsavecnt;
    unsigned int  func_lock_on;
    unsigned int  func_lock_off;
    char devloacle[64];         //存放dev 的 local 值
    int sync_timezone;
//  local_fd_sta_t rec_food;
    char rec_food[64];
    char iotx_server_info[32];
    unsigned int crc16;
} dev_param_t;

#define DEV_INFO_NEWZONE_VERIFY     (0xa5a55a5a)
typedef struct _dev_info_t
{
    // iotx_device_info_t iotx_dev_info;
    int pimtimeindex;
    int timezone;
    int serverlen;              //存储serverinfo有多长
    char *serverinfo;           //存储运行时保存的指针位置
    int ota_flag;//十位用于标志是否ota，个位用于指示bank位置
    int ota_err_cnt;            //秤校准模式
    int taregram;               //皮重
    char apptimezone[8];            //存放dev  的  timezone 值
    int verifyvalue[5];
    int addgram;        //每次更新粮后的附加重量
    int averagegram;        //平均值
    char flag[16];      //防止mac重复将softap与stationap交换
    int listschlen;                 //存储list schdule有多长
    char *listschinfo;          //存储运行时list schdule保存的指针位置
    int otafailNo;          //ota失败原因标志
    unsigned int led_on;
    unsigned int led_off;
    unsigned int verify_timestamp;
    int manual_max_gram;
    int flashsavecnt;
    unsigned int func_lock_on;
    unsigned int func_lock_off;
    char devloacle[64];         //存放dev 的 local 值
    int sync_timezone;
//  local_fd_sta_t rec_food;
    char rec_food[64];
    char iotx_server_info[32];
	int rgt;
} dev_info_t;

// typedef struct dev_info_pack_t
// {
//     int dev_info_verify;            //DEV_INFO_NEWZONE_VERIFY
//     int rgt;
//     dev_info_t dev_info;
//     unsigned int crc16;
// } dev_info_pack_t;
#pragma pack()

//维护向服务器上报设备状态失败
typedef struct _state_report_status_t
{
    int ret;   //向服务器上报设备状态是否成功,成功为0,失败为-1
    char* msg_p;    //记录msgptr
    short msg_value;//记录msg.value
    int try_report_UTC;
    int try_report_cnt;
    int err_cnt;
} state_report_status_t;

typedef struct _sys_param_t
{
	short cnt_restgram_percent;
	short cnt_restgram_mut;			//用于计算余粮错误变化值
	int cnt_pld_alarm;					//用于粮盖的打开计算累计值
	int detect_state;						//用于判定按键是否进入测试状态
	int detect_mode;						//用于判定测试模式
	int en_verify_alarm;					//用于判定设备是否想起秤校准
	int en_api_connect;					//用于判定是否api连接
	int led_ctl_off;
	unsigned char ledmask;
	short ledmaskcnt;
	int verify_mode_old;
	int wlerr;
	int fdrerr;
	int rdrerr;
	short rep_movertime;
	short rep_movervalue;
	short real_movervalue;
	short delta_movervalue;
	unsigned int clean_timestamp;

	char apihost[100];

	int cnt_restart;
	int cnt_pimcnt;

	int connect_mode;
	short pim_en;
	char pim_tcp;
	char http_HB;
	char PC_en;

	char pim_heartmax;		//pim心跳时间
	int pim_windowmax;		//pim心跳下标
	char pc_errcnt;	    //pim心跳错误统计
	char http_errpingcnt;	//http心跳错误统计
	short pim_overcase;		//pim超时统计
	short http_overcase;	//http超时统计
	short pim_servercase;	//pim服务断开统计
	int msg_httpexitcnt;	//http退出主循环
	int msg_noipexitcnt;	//pim退出主循环

	int http_pongokcnt;
	int pim_pongokcnt;

	short ota_wait_time;
	short repair_dr_wait_time;

	unsigned char sturct_check_err;

	char dev_lock_func;
	char statinmac[16];
	char ChipId[16];

	char dev_new_ssid;
	char dev_food_task_run;
	char dev_food_state;
	char dev_food_lock_state;
	int dev_food_close_delay;

	char dev_food_close_back;
	int report_UTC_timestamp;

	state_report_status_t state_report_status;
	char feed_report_save;    //向服务器上报喂食状态失败后，存储到flash标志，存储为1，清掉为0
	int feed_report_pc_retry;
	int feed_report_http_retry;
	int dr_cerr_ignore;
	int fix_mode;
	int repair_result;
	int pc_connect_UTC;
	int pc_disconnect_UTC;
	int http_connect_UTC;
	int http_disconnect_UTC;

	int pc_serverinfo_UTC;
	int get_feedplan_UTC;

	int dr3_err_UTC;
	int setnet_OK;

	short motor_init;
//	char bind_proc_sta;
//	char feed_proc_sta;
} sys_param_t;


typedef enum
{
	F_CHILD_LOCK = 0,           //存储儿童锁
	F_MAC_SAME,                 //系统mac重复
	F_BAT_MSG,                  //电池电量过低需要发送报告
	F_DOOR_TIMEZONE_DIS,        //按照时区关闭door的反弹处理
	F_SIGN_UP,                  //系统app绑定进行signup
	F_DETECT_OK = 5,                //系统测试完成成品测试
	F_MOT_SIG_FAIL,             //兼容信号出问题门打开后不继续工作的问题
	F_PIM_CONNECT_EN,           //-XX
	F_DEV_LOCATION,             //8设备定制机器标志
	F_DOOR_DELAY_DIS,       //9设备是否允许门优化控制
	F_API_HTTPS_DIS = 10,            //10强制一直关闭http的ssl功能
	F_SERVER_INFO,

	F_DOOR_STA,
	F_LASTEST_NONE,
	F_NOOWNER_HB,
	F_PC_DISABLE = 15,

	FMAX = 16
}dev_flag_enum;

/****************系统的状态 configure**************/
#define MASK_CLEAR		    0
#define MASK_SET			1

typedef unsigned int SYS_MASK_VALUE;

//--------------0
#define ST_UNBIND_MASK						(1<<0)                      //未绑定
#define ST_DISCON_MASK						(1<<1)                      //离线
#define ST_NOIP_MASK						(1<<2)                      //未获取到IP
#define ST_NOAPISERV_MASK					(1<<3)                      //未获取api服务器地址

//--------------1
#define ST_NOPMSERV_MASK					(1<<4)                      //未获取长连接服务器地址
#define ST_SETNET_MASK						(1<<5)                      //配网中
#define ST_COMM_ERR_MASK					(1<<6)                      //外部扩展芯片故障
#define ST_WIFI_ERR_MASK					(1<<7)                      //wifi芯片故障

//--------------2
#define ST_RAM_ERR_MASK						(1<<8)                      //内存异常
#define ST_MDOOR_CLS_ERR_MASK				(1<<9)                      //关门故障
#define ST_MDOOR_OPN_ERR_MASK				(1<<10)                     //开门故障
#define ST_WEIGHTER_ERR_MASK				(1<<11)                     //秤故障（弃用）

//--------------3
#define ST_WHEEL_ERR_MASK					(1<<12)                     //叶轮故障
#define ST_DC_ERR_MASK						(1<<13)                     //适配器故障
#define ST_BAT_ERR_MASK						(1<<14)                     //电池故障
#define ST_RTC_ERR_MASK						(1<<15)                     //RTC故障

//--------------4
#define ST_FLH_ERR_MASK						(1<<16)                     //flash故障
#define ST_PLD_ERR_MASK						(1<<17)                     //粮盖
#define ST_OTA_MASK							(1<<18)                     //OTA中
#define ST_FEEDING_MASK						(1<<19)                     //喂粮中

//--------------5
#define ST_HAVE_BAT_MASK                    (1<<20)                     //安装了电池
#define ST_BAT_USE_MASK						(1<<21)                     //使用电池
#define ST_RTC_LOW_MASK						(1<<22)                     //（弃用）
#define ST_OTA_SECTOR_ERR_MASK				(1<<23)                     //设备OTA分区错误
#define ST_BARREL_ERR_EMP_MASK				(1<<24)                     //余粮为空

//--------------6
#define ST_DR_CLOSE_CURVCER_MASK		    (1<<25)                     //关门卡粮
#define ST_BARREL_ERR_MUT_MASK				(1<<26)                     //（弃用）
#define ST_BARREL_ERR_PCENT_MASK			(1<<27)                     //（弃用）
#define ST_WEIGHTER_READ_ERR_MASK			(1<<28)                     //（弃用）

//--------------7
#define ST_MOVER_SIG_ERR_MASK				(1<<29)                     //（弃用）

#define ST_DOOR_OPT_MASK                    (1<<30)                     //开关门中

#define ST_ERR_PHY_MASK             (ST_COMM_ERR_MASK | ST_WIFI_ERR_MASK | ST_RAM_ERR_MASK | ST_FLH_ERR_MASK \
                                        | ST_RTC_ERR_MASK | ST_DC_ERR_MASK | ST_BAT_ERR_MASK | ST_WHEEL_ERR_MASK \
                                        | ST_MDOOR_OPN_ERR_MASK | ST_MDOOR_CLS_ERR_MASK | ST_DR_CLOSE_CURVCER_MASK \
                                        | ST_PLD_ERR_MASK | ST_WEIGHTER_READ_ERR_MASK)

#define ST_REPORT_PHY_MASK          (ST_ERR_PHY_MASK | ST_FEEDING_MASK | ST_BAT_USE_MASK | ST_OTA_MASK | ST_BARREL_ERR_EMP_MASK)
/****************gpio configure**************
input:
	gpio0		key1		滤芯重置按键
	gpio5		key2		预留输入，测试点
	gpio13		key3		设置wifi，切换单位
	gpio16		key4		预留输入
output:
	gpio4	led1			红灯（可能PWM）
	gpio12	ctrl_cpu	绿灯（可能PWM）

uart:
	gpio1		UART		TX
	gpio3		UART		RX

IIC:
	gpio2		IIC			IIC_SDA
	gpio14		IIC			IIC_SCL
******************************************/
#define SYS_CHECK_PIN			GPIO_Pin_5		//定义check的检测管脚
typedef enum
{
	T_RESET=0,
	T_GPIO_IN,				//输入设备
	T_GPIO_OUT,			//输出设备
	T_ADC,						//adc设备
	T_DAC,						//dac设备
	T_PWM,					//pwm信号设备
	T_DISP,						//显示屏设备
	T_TEMP,					//温度设备
	T_HUMI,					//湿度设备
	T_NFC,						//NFC设备
	T_WEIGHTER,					//秤设备
	T_MOTOR,				//电机设备
	T_RTC,						//时钟设备
	T_MAX
}phy_iod_type;
/********设备外设编号值定义********/
typedef enum
{
	PHY_FOOD_KEY=0,				//按键1//GPIO0 复位按键
	PHY_WIFI_KEY,					//按键3//wifi 按键
	PHY_WIFI_LED,					//指示灯1-系统状态灯
	PHY_CTL_CPU,			//外接设备控制1-从cpu电源-------5

	PHY_MAX
}phy_iod_no;

/*******定义log控制命令的类型说明**************/
#define LOG_SET_START		10000
#define LOG_SET_STOP		20000

#define	LOG_SET_PARAM_PUSH	10000	//10000,读取一次state信息
#define	LOG_SET_PIMCNT_DIS	10001	//10001, 断开pim连接
#define LOG_SET_SEND_DMSG 	10099	//10099，发送debugmsg
#define	LOG_SET_PARAM_DIS	10100	//10100, 最大的参数值

#define	LOG_SET_SEND_OTACHK	10600	//10600, 发送一次ota_check
#define	LOG_SET_LOCALE_CLN	10601	//10601, 清空一次locale值

#define	LOG_SET_DISAPISSL_DIS	10500	//10500, 关闭disssl功能
#define	LOG_SET_DISAPISSL_EN	10501	//10501, 开启disssl功能

#define	LOG_SET_DOOR_TIMEZONE_DIS	10610	//10610, 禁止APP TIMEZONE门延迟夹手控制
#define	LOG_SET_DOOR_TIMEZONE_EN	10611	//10611, 开启APP TIMEZONE门延迟夹手控制

#define	LOG_SET_DOOR_DELAY_DIS	10620	//10620, 禁止门延迟夹手控制
#define	LOG_SET_DOOR_DELAY_EN	10621	//10621, 开启门延迟夹手控制

#define	LOG_SET_DOOR_OPEN	10631	//10631，测试一次开门
#define	LOG_SET_DOOR_CLS	10632	//10632，测试一次关门
#define	LOG_SET_WHEEL_FRT	10633	//10633，测试一次叶轮
#define	LOG_SET_WHEEL_RES	10634	//10634，测试一次叶轮反转

#define	LOG_SET_PIM_DIS		10640	//10640，关闭pim连接
#define	LOG_SET_PIM_EN		10641	//10641，开启pim连接

#define	LOG_SET_LOG_DIS		10650	//10650, 关闭log收集
#define	LOG_SET_LOG_EN		10651	//10651, 开启log收集

#define	LOG_SET_AVOID_DIS	10660	//10660，关闭开启规避功能
#define	LOG_SET_AVOID_EN	10661	//10661, 开启sig异常功能

#define	LOG_SET_ADD_CLN		10671	//10671，清除addgram
#define	LOG_SET_TARE_CLN	10672	//10672，清除taregram
#define	LOG_SET_VERIFY		10673	//10673，设置verify值//[x]value  【x】verify序号，value-秤的读数

#define LOG_SET_MCU_SLEEP   10680   //10680，设置MCU低功耗模式
#define LOG_SET_MCU_WAKEUP  10681   //10681，唤醒MCU

#define LOG_SET_ESP_OTA     10690   //10690，尝试OTA升级
/*******定义pim断开的类型说明**************/
#define PIM_DISCNT_SERVER		(100)
#define PIM_DISCNT_CONNECT		(101)

/*******定义http-https切换的类型说明**************/
#define HTTPS_BLOCK_CONNECT_CNT_MAX			(40)
#define HTTPS_BLOCK_ERROR_CNT_MAX			(5)
#define HTTP_AND_HTTPS_ALTER_CNT_MAX		(5)

typedef enum
{
	COMM_IDLE=0,

	COMM_HEARTBEAT=100,
	COMM_DISCNNT=101,

	COMM_DEVINFO=110,
	COMM_CFGINFO=111,

	COMM_SMTBIND=150,
	COMM_APBIND=151,
	COMM_SETMODE=152,
	COMM_SAVEWIFI=153,

	COMM_DETECTSTATE=160,
	COMM_DETECTSETSN=161,
	COMM_DETECTCLEAN=162,
	COMM_DETECTCMD=163,
	COMM_DETECTINFO=164,
	COMM_DETECTMACALT=165,
    COMM_DETECT_SET_AGEING=166,
    COMM_DETECT_AGEING=167,

	COMM_SENDLOG=200,

	COMM_HTTP_DATA=210,
	COMM_MAX
}con_app_comm_type;
/****************************************
 * feedplan:包含了 latest and schedule的全部计划
 ****************************************/
typedef enum
{
	SERVER_NOIP=0,
	SERVER_PIMCNT,
	SERVER_PIMDISCNT,
	SERVER_SENDPIM,
	SERVER_SENDPING,

	SERVER_SERVERINFO,//5
	SERVER_SYNCTIME,
	SERVER_GETLIST,
	SERVER_FEEDREPORT,
	SERVER_FEEDCONSUME,

	SERVER_STATEREPORT,//10
	SERVER_FEEDSTART,
	SERVER_FEEDEND,

	SERVER_OTA_CHECK,
	SERVER_OTA_START,
	SERVER_OTA_COMPLETE,//15
	SERVER_OTA_HEARTBEAT,
	SERVER_OTA_REBOOT,

	SERVER_PIMSCHEDULE,
	SERVER_PIMREALTIME,

	SERVER_DETECT_CANCEL,//20
	SERVER_DETECT_CMD,
	SERVER_SIGUP,
	SERVER_DEVINFO,

	SERVER_IOT_DEVINFO,
	SERVER_SET_FOOD_RESET,//25

	SERVER_FOOD_ACTION,
	SERVER_SAVELOG,

	SERVER_HEARTBEAT,
	SERVER_HTTPSFAIL,
	SERVER_HTTPSTOP,

	SERVER_DISCONNECT,//30

	SERVER_MAX
}server_event_type;

typedef enum
{
	WIFI_KEY=0,
	WIFI_APPKEY,
	WIFI_DESTROY,
	WIFI_CLTCONNECT,
	WIFI_APCONNECT,
	WIFI_APSTAIONCHK,//5
	WIFI_STACONNECT,
	WIFI_STATIONCHK,
	WIFI_SMARTOVER,
	WIFI_DETECT_CANCEL,
	WIFI_WAKEUP,
	WIFI_SLEEP,

	WIFI_RESULT,		// 

	WIFI_MAX
}wifi_event_type;

typedef enum
{
	DETECT_WIFI_KEY=100,
	DETECT_FOOD_KEY,
	DETECT_PLD_ST,

	DETECT_MAX
}detect_event_type;

typedef enum
{
	FOOD_RESET_OK=0,
	FOOD_RESET_DOING,	//1
	FOOD_RESET_CANCEL,	//2
	FOOD_RESET_FAIL,	//3
	FOOD_RESET_OVERWT,	//4

	FOOD_RESET_MAX
}food_reset_repstate;
typedef enum
{
	FOOD_RST_FAIL_NULL=0,
	FOOD_RST_FAIL_WT,	//1
	FOOD_RST_FAIL_WL,	//2
	FOOD_RST_FAIL_DR,	//3
	FOOD_RST_FAIL_OTH,	//4

	FOOD_RST_FAIL_MAX
}food_reset_failcase;

typedef enum
{
    PK_API_TICK_OTA_E = 0,
    PK_API_TICK_SIGNUP_E,
    PK_API_TICK_IOTINFO_E,
    PK_API_TICK_DEVINFO_E,
    PK_API_TICK_SERVERINFO_E,
    PK_API_TICK_GETPLAN_E,
    PK_API_TICK_SYNCTIME_E,
    PK_API_TICK_STA_REPORT_E,
    PK_API_TICK_HTTP_HB_E,
    PK_API_TICK_PC_CONNECT_E,

    PK_API_TICK_MAX
} api_ticks_e;

typedef struct _NWM_tick_t
{
    int baseTick;
    int nextTick;
    int tick_left;
    int nextUTC;
} NWM_tick_t;

typedef struct _NWM_timer_ctrl_t
{
    int enable;
    NWM_tick_t api_ticks[PK_API_TICK_MAX];
} NWM_timer_ctrl_t;

/*******制定有关定时的结构体*******/
typedef struct _server_tick_t
{
    int enable;
	unsigned int nextTick_serverinfo;
	unsigned int cnt_serverinfo;
	unsigned int serverinfo_tick_left;

	unsigned int nextTick_feedplanlist;
	unsigned int cnt_feedplanlist;
	unsigned int feedplanlist_tick_left;

	unsigned int nextTick_synctimer;
	unsigned int cnt_synctimer;
	unsigned int synctimer_tick_left;

	unsigned int nextTick_pimping;
	unsigned int cnt_pimping;
	unsigned int pimping_tick_left;
	char pingpong;

	unsigned int nextTick_pimconnect;
	unsigned int cnt_pimconnect;
	unsigned int pimconnect_tick_left;

	unsigned int nextTick_foodreset;
	unsigned int cnt_foodreset;
	unsigned int foodreset_tick_left;

	unsigned int nextTick_sigup;
	unsigned int cnt_sigup;
	unsigned int sigup_tick_left;

	unsigned int nextTick_statereport;
	unsigned int cnt_statereport;
	unsigned int statereport_tick_left;

	unsigned int nextTick_devinfo;
        unsigned int cnt_devinfo;
        unsigned int devinfo_tick_left;

        unsigned int nextTick_repair;
        unsigned int cnt_repair;
        unsigned int repair_tick_left;

        unsigned int nextTick_OTAcheck;
        unsigned int cnt_OTAcheck;
        unsigned int OTAcheck_tick_left;
}server_tick_t;

typedef struct _server_opt_t {
	char cnt_pim;
	char cnt_api;

	int http_api_send_block_cnt;
	int http_api_send_ssl_enable;
	int http_api_send_fail_cnt;
} server_opt_t;

typedef struct
{
	int firmwareId;
	int software;
	int filesize;
	char firmware[8];
    char hardware;
	char *digest;
	char *url;
	char *module;
	struct upgrade_server_info *server;
	char sendpim;
}ota_param_t;

typedef struct
{
	char firmware[8];
	char hardware;
	char force;
	int software;
	char ota_flag;//upgrade,fail,success;
	char sendticks;
	int otafailNo;
	ota_param_t *ota;
	unsigned short MCU_version;

    char target_firmware[8];
    char target_hardware;
    int  target_software;
}version_param_t;


void sys_force_quittask(unsigned int mask_task);
void sys_close_wifi(unsigned char force);
void sys_close_ble(unsigned char force);

int sys_timer_start(int force);
int sys_timer_stop(void);
unsigned int sys_get_os_runtime_sec(void);
void sys_set_os_runtime_sec(unsigned int x_sec);

// //void sys_detect_set_timer(char *msg);
// //void sys_detect_tool_test(void);
// //void sys_detect_app_test(int MCU_comm);
// //int sys_detect_check(void);
// void get_task_state(void);
// void sys_init_variable(void);
// char sys_get_input_gpio(void);
// void sys_init_input_gpio(void);
// void sys_init_output_gpio(void);
// void sys_init_phy_iod(void);
// int GPIO_get_funcPIN_value_jitt(void);
// int  sys_check_bind_param(void);
// void sys_read_config_param(void);
// void sys_init_api_param(void);
// void system_init(void);
// void sys_write_state(SYS_MASK_VALUE mask,char opt);
// unsigned char sys_read_state(SYS_MASK_VALUE mask);
// char *system_parse_feedlist(char *pFramer);
// char *sys_urlencode_str(char *str,char *addstr);
// int is_asc_str(char value);
// void sys_urldecode_str(char *str);
// void sys_strdecode_str(char *str);
// char *sys_get_restart_reason(void);

// void api_timer_reset_pimping(int delay);
// void api_timer_calu_pimpong(void);

// void api_timer_reset_repair(int delay);
// void api_timer_reset_foodreset(int delay);

// int api_timer_judge_handle(void * parg);
// void stop_all_key_timer(void);

// void xTask_mutex_lock(void *mutex);
// int xTask_mutex_trylock(void *mutex, int time_ms);
// void xTask_mutex_unlock(void *mutex);

#endif

