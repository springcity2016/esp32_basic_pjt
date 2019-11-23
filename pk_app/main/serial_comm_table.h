
#ifndef __SERIAL_COMM_TABLE_H_
#define __SERIAL_COMM_TABLE_H_

#define COMM_EVENT_SEND_CMD             0x01        // 向外发送指令
#define COMM_EVENT_RECV_CMD             0x02        // 接受处理指令

#define COMM_W_R_POS                (7)
#define COMM_WRITE_CMD              (0x00)
#define COMM_READ_CMD               (0x01)

// 1. 数据格式协议用到的 Index
#define COMM_HEADER0_INDEX				(0)       // 数据包包头 0x5A
#define COMM_HEADER1_INDEX				(1)       // 数据包包头 0xA5
#define COMM_LENGTH_INDEX				(2)       // 数据包协议长度
#define COMM_CMD_INDEX					(3)       // 数据指令
#define COMM_SEQ_INDEX					(4)       // 数据序列号
#define COMM_ACK_INDEX					(5)       // 数据答复
#define COMM_WRSYNNODE_INDEX            (6)       // BIT7: W/R； BIT6: SYN； BIT5~0：数据节点
#define COMM_DATASTART_INDEX            (7)       // 数据位其实位

#define COMM_HEADERSNUM_MAX             (COMM_DATASTART_INDEX)       // 数据包包头总共有8个数据

#define COMM_CHECKSUM_H_LEN             (1)
#define COMM_CHECKSUM_L_LEN				(1)
#define COMM_CHECKSUM_SIZE              (2)       // CheckSum长度大小

// 2. 对应数据包用到的 数据Data
  // a. Header0, Header1 数据
#define COMM_HEADER0_DATA               (0x5A)
#define COMM_HEADER1_DATA               (0xA5)

enum
{
    COMM_CMD_HB = 0,            // 0. 心跳
    COMM_CMD_SENSOR,            // 1. 传感器
    COMM_CMD_MOT_RUN_CFG,       // 2. 电机运行设置命令
    COMM_CMD_MOT_RUN_RT,        // 3. 电机运行实施数据监测
    COMM_CMD_MOT_QT_CFG,        // 4. 电机运行其他指令
    COMM_CMD_MOT_QT_RT,         // 5. 电机其他指令

    COMM_CMD_WEIGHT,            // 6. 称重传感器

    COMM_CMD_ERR,               // 7. 错误代码
    COMM_CMD_VER,               // 8. 软件代码
    COMM_CMD_RST,               // 9. 软件复位
    COMM_CMD_WORKMODE,          // 10. 工作模式
    COMM_CMD_CFGDATA,           // 11. 配置底层工作数据

    COMM_CMD_OUTCTRL,           // 12. IO口控制

    COMM_CMD_MAX
};

  // b. Cmd 命令数据
#define COMM_ERRCMD_DATA                (0x06)       // 错误代码命令
#define COMM_VERCMD_DATA                (0x07)       // 软件版本命令
#define COMM_RESETCMD_DATA              (0x08)       // MCU复位命令

#define COMM_LEDDEVCMD_DATA             (0x0A)       // LED/Beep 控制命令
#define COMM_RTCCMD_DATA                (0x0B)       // RTC相关指令 - 读写内部时钟
#define COMM_SENSORTIMECMD_DATA         (0x0C)       // IO/ADC采集时间
#define COMM_INITCFGCMD_DATA            (0x0D)       // Config配置命令
#define COMM_FHCTRLCMD_DATA             (0x0E)       // Fan & Heater 控制
#define COMM_FHOPCCMD_DATA              (0x0F)       // Fan & Heater 过流设置
#define COMM_SETCHECKCMD_DATA           (0x11)       // 设定动物检测指令
#define COMM_STARTVOICECMD_DATA         (0x12)       // 播放音频文件
#define COMM_DELVOICECMD_DATA           (0x13)       // 删除音频文件
#define COMM_STOPVOICECMD_DATA          (0x14)       // 停止播放音频文件
#define COMM_WEIGHTCMD_DATA             (0x15)       // 秤重
#define COMM_SYSMODECMD_DATA            (0x16)       // 系统工作情况，0: NormalMode, 1: LowPowerMode
#define COMM_IOPORTCMD_DATA             (0x17)       // IO管脚控制端口命令设置
#define COMM_MCUHWWDGCMD_DATA           (0x18)       // MCU硬件看门够 -> 利用DC<->BAT电源切换

#define COMM_STARTUPAPROM_CMD               (0x20)          // 启动更新APROM内部程序
#define COMM_REQUPAPROM_CMD				    (0x21)			// 请求更新APRom程序
#define COMM_UPAPROMDATA_CMD				(0x22)			// 正在更新软件命令
#define COMM_UPFINISHED_CMD					(0x23)			// 程序更新完成


#define COMM_CMDMAX_DATA                (COMM_UPFINISHED_CMD + 1)

  // c. ACK回复内容 - 包含数据错误
#define COMM_ACK_SUCCESS                (0xFF)       // 数据通信成功，无需发送数据
#define COMM_ACK_ERR_CHECKSUM           (0xFE)       // CheckSum/数据 错误需要重发
#define COMM_ACK_MOTOR_OK               (0xFD)       // 电机设定动作完成

#define WEIGHT_BUFF_LENGTH              (20)



/*****Sand Mot Config*******************************************************************/
#define DEV_PHY_MOT_DEF_SPEED                   (400)
#define DEV_PHY_MOT_DEF_STEP                    (1000)
#define DEV_PHY_MOT_DEF_STEP_MAX                (2000)
#define DEV_PHY_MOT_DEF_OVER_TIME               (60)        // 超时时间30Sec

#define DEV_PHY_IO_QUDOU_DEF_TIME               (10)        // 单位ms
#define DEV_PHY_ADC_DELAY_DEF_TIME              (100)       // 单位ms
#define DEV_PHY_SENSOR_RPT_DEF_TIME             (100)       // 单位ms
#define DEV_PHY_HB_RPT_DEF_TIME                 (30)        // 单位Sec
/***************************************************************************************/


enum
{
    E_RPT_TYPE_TIMING = 0,      // 周期性上报
    E_RPT_TYPE_RT,              // 立刻上报
    E_RPT_TYPE_CHANGE,          // 变化上报

    E_RPT_TYPE_DISEN,           // 禁止上报

    E_RPT_MAX
};

typedef enum
{
    E_STOP = 0,
    E_CW,
    E_CCW,
}MOTOR_STATE;

enum
{
    E_SANDMOT_IDLE_T = 0,    // Idle状态
    E_SANDMOT_STEP_T,        // 步进走
    E_SANDMOT_SCRAM_T,       // 急停走
    E_SANDMOT_POS_T,         // 到边停
    E_SANDMOT_MAX_T
};

enum {
    E_DODRV_MODE_ALLOFF = 0,
    E_DODRV_MODE_ALLON,
	E_DODRV_MODE_BLINK,
    E_DODRV_MODE_TEST,

    E_DODRV_MODE_MAX
};

enum
{
    E_SANDMOT_POS_NONE = 0,
    E_SANDMOT_POS_START,        // Right -> Start
    E_SANDMOT_POS_END,          // Left -> End

    E_SANDMOT_POS_MAX
};


// #define MOT_RUN_CFG_INIT    { .type = E_SANDMOT_IDLE_T, .pos = E_SANDMOT_POS_NONE, .mot_sta = E_STOP, .mot_speed = DEV_PHY_MOT_DEF_SPEED, .u16_stepnum = DEV_PHY_MOT_DEF_STEP, .u16_maxstepnum = DEV_PHY_MOT_DEF_STEP_MAX, .u16_overtime = DEV_PHY_MOT_DEF_OVER_TIME }

#define MOT_DEV_PHY_CFG_INIT    {   \
    .io_qudou_time = DEV_PHY_IO_QUDOU_DEF_TIME, \
    .adc_delay_time = DEV_PHY_ADC_DELAY_DEF_TIME,   \
    .sensor_rpt_delay_time = DEV_PHY_SENSOR_RPT_DEF_TIME,   \
    .hb_rpt_delay_time = DEV_PHY_HB_RPT_DEF_TIME,   \    
}


#define NEW_RUN_CFG(_type, _pos, _mot_sta, _mot_speed, _step_num, _max_step, _over_time)    \
{ \
    .type = _type,  \
    .pos = _pos,    \
    .mot_sta = _mot_sta,    \
    .mot_speed = _mot_speed,    \
    .u16_stepnum = _step_num,   \
    .u16_maxstepnum = _max_step,    \
    .u16_overtime = _over_time, \
}
#define NEW_RUN_REAL(_mot_sta, _mot_speed, _mot_curt, _mot_step, _mot_time)  \
{   \
    .mot_sta = _mot_sta,    \
    .mot_speed = _mot_speed,    \
    .u16_mot_curt = _mot_curt,  \
    .u16_mot_stepnum = _mot_step,   \
    .u16_mot_runtime = _mot_time,   \
}
#define NEW_MOT_QT_CFG(_up_time, _down_time, _op_time, _1stnm_time, _2ndnm_time, _op_curt, _nmh_curt, _nml_curt)    \
{   \
    .u08_up_time = _up_time,    \
    .u08_down_time = _down_time,  \
    .u16_op_time = _op_time,    \
    .u16_1stnm_time = _1stnm_time, \
    .u16_2ndnm_time = _2ndnm_time, \
    .u16_op_curt = _op_curt,    \
    .u16_nmh_curt = _nmh_curt,   \
    .u16_nml_curt = _nml_curt,   \
}
#define NEW_DEV_PHY_CFG(_ioqd_time, _adc_delay, _sensor_rpt_delay, _hb_rpt_delay)   \
{   \
    .io_qudou_time = _ioqd_time,  \
    .adc_delay_time = _adc_delay, \
    .sensor_rpt_delay_time = _sensor_rpt_delay,  \
    .hb_rpt_delay_time = _hb_rpt_delay,  \
}

#pragma pack(1)

typedef struct _weight_cfg_t
{
    unsigned char bfenable;     // 使能
    unsigned char size;         // 采集buff
    unsigned int gather_time;        // 采集周期
    unsigned int rpt_time;
}weight_cfg_t;

typedef struct _weight_real_t
{
    unsigned char size;         // buff size
    unsigned int *p_weight;     // weight 质量
}weight_real_t;

typedef struct _sandmot_runcfg_t
{
    unsigned char type;         // step_t / scram_t / stop_t
    unsigned char pos;
    unsigned char mot_sta;          // motor_sta - E_STOP/E_CW/E_CCW
    unsigned char mot_speed;        // motor Speed
    unsigned short u16_stepnum;    // 
    unsigned short u16_maxstepnum; // 
    unsigned short u16_overtime;// 超时时间
}sandmot_runcfg_t;

typedef struct _sandmot_real_t
{
    // unsigned char mot_pos;
    unsigned char mot_sta;
    unsigned char mot_speed;
    unsigned short u16_mot_curt;    // 实时电流
    unsigned short u16_mot_stepnum; // 
    unsigned short u16_mot_runtime; // 
}sandmot_real_t;

typedef struct _sandmot_real_result_t
{
    sandmot_real_t real_value;
    unsigned char result;           // 0: 未完成， 1: 已完成,  其他errorNum
    unsigned char mot_pos_start;    // 
    unsigned char mot_pos_end;      // 
    unsigned int os_time;           // 获取传感器时系统时间
}sandmot_real_result_t;

typedef struct _sandmot_qtcfg_t
{
    unsigned char u08_up_time;      // 升速时间
    unsigned char u08_down_time;    // 降速时间

    unsigned short u16_op_time;
    unsigned short u16_1stnm_time;
    unsigned short u16_2ndnm_time;
    unsigned short u16_op_curt;     // OverPeak Current
    unsigned short u16_nmh_curt;    // Normal High Current
    unsigned short u16_nml_curt;    // Normal Low Current
}sandmot_qtcfg_t;

typedef struct _sandmot_qtreal_t
{
    unsigned char u08_up_time;
    unsigned char u08_down_time;    // 降速时间

}sandmot_qtreal_t;

typedef struct _sensor_sta_t
{
    unsigned short mot_adc;
    unsigned short whq_adc;
    unsigned short dc_adc;
    unsigned short io_det;      // IO _DET值
}sensor_sta_t;

typedef struct _version_t
{
    unsigned char year;
    unsigned char week;
    unsigned char version;
}version_t;

typedef struct _errcode_t
{
    unsigned int errcode_1st;
    unsigned int errcode_2nd;
}errcode_t;

typedef struct _cfgdata_t
{
    unsigned char io_qudou_time;        // IO检测去抖时间
    unsigned char adc_delay_time;       // adc检测周期时间

    unsigned char sensor_rpt_delay_time;    // 传感器商报时间

    unsigned char hb_rpt_delay_time;    // 心跳上报周期时间
}cfgdata_t;

typedef struct _out_pin_t
{
    unsigned char mode;                 // 
    unsigned char times;
    unsigned char on_time;
    unsigned char off_time;
}out_pin_t;

typedef struct _drv_physta_t
{
    sensor_sta_t st_sensor_sta;
    sandmot_qtreal_t st_motqt_real_sta;
    sandmot_qtcfg_t st_motqt_cfg_sta;
    sandmot_real_t  st_motrun_real_sta;
    sandmot_runcfg_t st_motrun_cfg_sta;
    unsigned int weight_data;
    version_t st_version;
    errcode_t st_errcode;
}drv_physta_t;

typedef struct _drv_weight_data_t
{
    unsigned char size;
    unsigned int a_weight[WEIGHT_BUFF_LENGTH];          
}drv_weight_data_t;

#pragma pack()


static const sandmot_runcfg_t cst_mot_runcfg_init = { E_SANDMOT_IDLE_T, E_SANDMOT_POS_NONE, E_STOP, DEV_PHY_MOT_DEF_SPEED, DEV_PHY_MOT_DEF_STEP, DEV_PHY_MOT_DEF_STEP_MAX, DEV_PHY_MOT_DEF_OVER_TIME };
static const cfgdata_t cst_dev_phycfg_init = { DEV_PHY_IO_QUDOU_DEF_TIME, DEV_PHY_ADC_DELAY_DEF_TIME, DEV_PHY_SENSOR_RPT_DEF_TIME, DEV_PHY_HB_RPT_DEF_TIME };

#endif


