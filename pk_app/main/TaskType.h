
/*
 * TaskType.h
 *
 *  Created on: 2018年11月20日
 *      Author: chunchen.cao
 */

#ifndef __TASKTYPE_H_
#define __TASKTYPE_H_


#define NONE                    (0)     // 数据为0
#define FUNC_ERROR              (-1)    // 函数错误

#define DBG_INFO                (1)     // debug 的时候打印信息

// #ifdef __cplusplus
// extern "C" {
// #endif

#define COMM_STATE_OFF          0       // 通信断开
#define COMM_STATE_ON           1       // 通信建立
#define COMM_STATE_MCUREQ       2       // MCU 请求通信状态

enum
{
    E_IOPORTNODE_DCBAT = 0,
    E_IOPORTNODE_MAX
};

// Motor State
typedef enum
{
	E_MOTORSTATE_STOP = 0,
	E_MOTORSTATE_CW,
	E_MOTORSTATE_CCW,
	E_MOTORSTATE_ERR,				// Motor Cannot Finish on Time
	
	E_MOTORSTATE_MAX
}E_MOTORCTRL_STATE;

typedef enum
{
    E_MOTORMODE_NONE = 0,
    E_MOTORMODE_POS,
    E_MOTORMODE_SCRAM,
    E_MOTORMODE_TEST,

    E_MOTORMODE_MAX
}E_MOTORCTRL_MODE;

typedef enum
{
    E_MOTORPOS_NONE = 0,
    E_MOTORPOS_1ST,         // Close Pos
    E_MOTORPOS_2ND,         // Open Pos
    E_MOTORPOS_3RD,
    E_MOTORPOS_4TH,
    E_MOTORPOS_5TH,
    E_MOTORPOS_6TH,
    E_MOTORPOS_7TH,
    E_MOTORPOS_8TH,
    E_MOTORPOS_9TH,

    E_MOTORPOS_MAX
}E_MOTORCTRL_POS;

typedef enum
{
    E_IOINT_M1DET1 = 0,
    E_IOINT_M1DET2,
    E_IOINT_M2DET1,
    E_IOINT_M2DET2,
    E_IOINT_M3DET1,
    E_IOINT_M3DET2,
    E_IOINT_M4DET1,
    E_IOINT_M4DET2,
    E_IOINT_SRDET,          // 释热传感器 -> SR
    E_IOINT_INDET,          // 猫进入红外检测传感器 -> IN

    E_IOINT_MAX                 // 大小设置    
}eIOIntType;

enum
{
	E_DOORL_MOT = 0,
	E_DOORR_MOT,
	E_FANL_MOT,
	E_FANR_MOT,
	E_BDTL_MOT,
	E_BDTR_MOT,
	E_MAX_MOT
};

typedef enum
{
	LED_WIFI = 0,
	LED_FH1_R,
	LED_FH1_G,
	LED_FH1_B,
	LED_FH2_R,
	LED_FH2_G,
	LED_FH2_B,

	LED_MAX,
}eLEDPort;

typedef enum
{
	DOORM_L = 0,
	DOORM_R,
	
	DOORM_MAX
}eDoorMPort;

typedef enum
{
    DOOR_MOT = 0,
    WL_MOT,

    MAX_MOT
}eMotPort;

typedef enum
{
    ADC_CH_DOORM = 0,
    ADC_CH_WLMOT,
    ADC_CH_DC,
    ADC_CH_BAT,
    ADC_CH_MAX
}eADCPort;

typedef struct _datas_t
{
    unsigned short u16Value;
    unsigned char *pu08Data;
}Datas_t;

// 用户端需要关心的数据类型
typedef struct UserData_t
{
    unsigned char u08Result;        // 运行结果
    unsigned char u08block;         // 阻塞/非阻塞; TRUE - 阻塞; FALSE - 非阻塞
    char s08SetTimes;               // 设定发送次数
    unsigned char u08Cmd;           // 命令
    unsigned char u08Node;          // Node
    unsigned char u08Length;        // 接下来数据指针和藏毒
    unsigned char *pu08Data;        // 存储数据的指针
}UserData_t;

// 用于超时机制传送 - 此处传送为超时重传机制中的返回值 - 由传送最底层传送给内部判定
typedef struct CmdTimeOut_t
{
    unsigned char u08Seq;                   // 发送指令对应的Seq
    unsigned char u08Ack;                   // 发送指令对应的Ack，应答
    char s08SendTimes;                      // 数据传输剩余次数
    unsigned char u08Length;                // 接下来指针的数据长度
    unsigned char *pu08Data;                // 存储接收到的数据
}CmdTimeOut_t;

typedef enum
{
    E_UPMCUROM_NONE = 0, 
    E_UPMCUROM_RST,             
    E_UPMCUROM_WAITUP,          // 等待 更新
    E_UPMCUROM_UPING,           // 更新 MCU 程序进行中

    E_UPMCUROM_FINISHED,        // MCU 更新完成

    E_UPMCUROM_MAX,
}e_upmcurom_t;

#pragma pack(1)		// 单字节对其
// 简单电机控制模型
typedef struct _mValue_t
{
    unsigned char u08State;                     // 0: Stop; 1: CW; 2: CCW       
    unsigned short u16Speed;                    // 电机速度
}MValue_t;
// 电机时间控制模型
typedef struct _timeMValue_t
{
    unsigned char u08UpTime;                    // 调速电机升速时间
    unsigned char u08DownTime;                  // 调速电机降速时间
    unsigned short u16AllRunTime;               // 电机控制运行总时间
    unsigned short u16AfterPosTime;             // 电机运行到指定位置后继续运行一段时间
    unsigned short u16OverTime;                 // 电机超时时间
}TimeMValue_t;
// 电机电流模型
typedef struct _currentMValue_t
{
    unsigned short u16OverPeakCurrent;          // 峰值电流
    unsigned short u16HighNormalCurrent;        // 正常电流值 - 高点
    unsigned short u16LowNormalCurrent;         // 正常电流值 - 低点
    unsigned short u16RealTimeCurrent;          // 实时电流值
}CurrentMValue_t;
// 电机电流时间模型
typedef struct CurrentTimeMValue_t
{
    unsigned short u16OverPeakCurrentTime;      // 到达峰值电流时间	
    unsigned short u161stNormalCurrentTime;     // 第一次到达正常电流时间
    unsigned short u162ndNormalCurrentTime;     // 第二次到达正常电流时间
}CurrentTimeMValue_t;
// 电机控制常用模型
typedef struct MotorRunValue_t
{
    unsigned char u08Mode;                      // 0: None; 1: PosMode; 2：ScramMode; 3: TestMode 
    unsigned char u08Pos;                       // 电机地址
    MValue_t strMotorSingle;               // 简易电机控制变量, 使能/方向/速度
    TimeMValue_t strMotorTime;                 // 电机时间控制变量
    unsigned char u081stStepTime;               // uptime,启动电流
    unsigned char u082ndStepTime;               // 第一次位置变化时间
    unsigned char u083rdStepTime;               // 第二次位置变化时间
}MotorRunValue_t;
// 电机控制模型
typedef struct MotorValue_t
{
    unsigned char bfRunUp;
    unsigned char bfCurrentUp;
    unsigned char bfTimeUp;
    MotorRunValue_t stMotorRunValue;            // 电机运行结构变量
    CurrentMValue_t stMotorCurrent;             // 电流控制变量
    CurrentTimeMValue_t stMotorCurrentTime;     // 电流时间控制变量
}MotorValue_t;


// 错误代码模型
typedef struct Err_t 
{
    unsigned int u32OtherErr;
    unsigned int u32OtherErr2;
}Err_t;
// 版本模型
typedef struct Ver_t
{
    unsigned char u08Year;              // 版本发布 - 年
    unsigned char u08Week;              // 版本发布 - 周数
    unsigned char u08Ver;               // 版本发布 - 版本
}Ver_t;
// SEQ模型
typedef struct Seq_t
{
	unsigned char u08DoorM;				// DoorM Seq
	unsigned char u08RotateM;			// RotateM Seq
	unsigned char u08GrabM;				// GrabM Seq
	unsigned char u08RotGrabM;			// RotGrabM Seq
}Seq_t;
// 系统模型
typedef struct Sys_t
{
    Err_t strErr;                       // 系统错误
    Ver_t strVer;                       // 系统版本
	Seq_t strSeq;						// 各个电机控制的Seq
    unsigned char u08ErrCode;           // 系统错误
    unsigned char bfCommSync;           // 通信同步
    unsigned char bfReStart;            // MCU重新启动
    unsigned char u08CommState;         // ESP & MCU 通信状态
}Sys_t;

// 通信结构体
typedef struct Comm_t
{
	unsigned char u08Header0;			// 数据包包头, 0x5A
	unsigned char u08Header1;			// 数据包包头, 0xA5
	unsigned char u08Len;				// 数据包长度
	unsigned char u08Cmd;				// 数据包指令
	unsigned char u08WR;				// ESP -> MCU; 0：W, 1: R
	unsigned char u08Seq;				// MCU 跟随 ESP 传输数据；保证 MCU 回馈 ESP，便于ESP 进行异步区分
	unsigned char u08Ack;				// 0: 失败； 1：成功
	unsigned char u08Node;				// 节点，针对Cmd，会有不同的节点
	unsigned char *pu08Data;			// 实际有效数据
	unsigned short u16CheckSum;			// CheckSum
}Comm_t;

typedef struct _ioport_t_ {
    unsigned bfUp;
    unsigned char u08Node;
    unsigned char u08Sta;
}IOPort_t;

// IO OnOff
typedef struct _onoffvalue_t
{
    unsigned char u08Times;
    unsigned char u08OnTime;            // 时间单位: Sec
    unsigned char u08OffTime;           // 时间单位: Sec
}OnOffValue_t;

// OnOff + Mode
typedef struct _iovalue_t
{
    unsigned char u08Mode;
    OnOffValue_t stOnOffValue;
}IOValue_t;

// RTC 时间
typedef struct _rtcTime_t
{
    unsigned char u08Year;                  // 年      默认 20 - xx
    unsigned char u08Month;                 // 月      
    unsigned char u08Date;                  // 日
    unsigned char u08CSDOW;                 // B7表示CS; 0 - 24H； 1 - 12H；   B3~B0: 表示星期几
    unsigned char u08Hour;                  // 时
    unsigned char u08Min;                   // 分
    unsigned char u08Sec;                   // 秒
}RTCTime_t;

typedef struct _temperHumi_t
{
	short s16WorkLTemper;		// Work L 的温度
	short s16WorkRTemper;		// Work R 的温度
	short s16DispLTemper;		// Disp L 的温度
	short s16DispRTemper;		// Disp R 的温度
	short s16Temper;			// 温湿度传感器中的温度
	short s16Humi;				// 温湿度传感器中的湿度
}temperHumi_t;

typedef struct 
{
	unsigned char u08DoorMLSpTime;
	unsigned char u08DoorMLEpTime;
	unsigned char u08DoorMRSpTime;
	unsigned char u08DoorMREpTime;
}SensorMotTime_t;

typedef struct 
{
	unsigned char u08DoorMLAdcTime;
	unsigned char u08DoorMRAdcTime;
	unsigned char u08FanLAdcTime;
	unsigned char u08FanRAdcTime;
	unsigned char u08HeaterLAdcTime;
	unsigned char u08HeaterRAdcTime;
}MotAdcTime_t;

typedef struct
{

	unsigned short u16DoorML;
	unsigned short u16DoorMR;
	unsigned short u16FanL;
	unsigned short u16FanR;
	unsigned short u16HeaterL;
	unsigned short u16HeaterR;
}OPC_t;

// 设备用户
typedef struct _devInfo_t
{
    unsigned char bfSensorUp;
	unsigned short au16MotAdcValue[ADC_CH_MAX];
	unsigned int au32vDCvol;
	unsigned int au32vBATvol;
	temperHumi_t stTemperHumitValue;
	unsigned char u08DoorM_Det;
	unsigned char u08WLMot_Det;
	unsigned char u08MotFail_Det;
    unsigned char u08DCIN_Det;
	
    unsigned char au08MotErrCode[MAX_MOT];   // 电机错误
	MotorValue_t astMotValue[MAX_MOT];	    // 门电机的操作数据

	SensorMotTime_t stSenMotTime;		    // Sensor Motor Check Time
	MotAdcTime_t stMotAdcTime;			    // Motor Adc CheckTime
	OPC_t stOverPeakCurrent;			    // 电机过流参数
    IOValue_t astLedValue[LED_MAX];

    
    IOPort_t astIOPortValue[E_IOPORTNODE_MAX];

	unsigned char u08IOCtrlEn;			// Bit0: FanL; Bit1: FanR; Bit2: HeaterL; Bit3: CoolL; Bit4: HeaterR; Bit5: CoolR
	unsigned char u08DoorMRSta;
	unsigned char u08DoorMLSta;

    unsigned char bfWeightUp;
    unsigned long u64WeightData;
    unsigned char u08MCUWorkMode;       // 0:Normal Mode; 1:LowPowerMode

    unsigned char bfSyncUp;             // 与MCU进行同步
    unsigned char bfCommSync;           // 同步标志
    unsigned char u08CommState;         // ESP & MCU 通信状态

    unsigned char latestErrCode;        // 最新的ErrorCode
	Err_t stErrCode;                    // 所有的ErrorCode标志位存储
    unsigned char bfVerUp;              // 版本更新
	Ver_t stVersion;    
}DevInfo_t;

// 设备配置
typedef struct _configInfo_t
{
	unsigned short u16SensorRepTime;	// Sensor上报时间 - 单位 100ms;
	unsigned char u08HBRepTime;			// 心跳上报频率 - 单位 Sec;
	
	IOValue_t astLEDCtrlValue[LED_MAX];	// LED 控制信息

	MotorValue_t astMotValue[DOORM_MAX];	// 门电机的操作数据

	SensorMotTime_t stSenMotTime;		// Sensor Motor Check Time
	MotAdcTime_t stMotAdcTime;			// Motor Adc CheckTime

	OPC_t stOverPeakCurrent;			// 电机过流参数    

    /*************************************************************************/
    // ESP8266 上面的信息
    short s16TemperOffValue;            // 温度控制偏移量

}ConfigInfo_t;

typedef struct _sysCtrl_t
{
    unsigned char u08ErrCode;           // 系统错误
    unsigned char bfCommSync;           // 通信同步
    unsigned char bfReStart;            // MCU重新启动
    unsigned char u08CommState;         // ESP & MCU 通信状态

    int as32DoorSta[2];
    unsigned char au08FanSta[2];
    int as32BdtSta[2];
    unsigned int au32BDTHeatStop[2];
    unsigned int au32BDTCoolStop[2];
    unsigned char u08IOMCtrlEN;         // Bit0: FanL; Bit1: FanR; Bit2: HeaterL; Bit3: CoolL; Bit4: HeaterR; Bit5: CoolR

    unsigned char bfMotAdcUp;
    unsigned char bfTemperUp;
    unsigned char bfIOValUp;

        // MCU 更新控制
    // unsigned char u08Up_MCUSta;     //　MCU的状态 0. 无状态，　1. 重新启动，  2. MCU请求更新,  3. 更新数据

}SysCtrl_t;

typedef struct _upmcu_t
{
    unsigned char u08Up_MCUSta;     // MCU的状态 0. 无状态，　1. 重新启动，  2. MCU请求更新,  3. 更新数据,  4. 更新完成
    
    size_t StartAddr;               // MCU存储起始地址
    unsigned int u32RomSize;        // MCU程序大小
    unsigned short u16CheckSum;     // MCU程序代码校验值

    unsigned int u32UpSize;         // MCU更新的程序大小

    unsigned char u08UpMCUSeq;      // 对应seq   - from MCU
    unsigned char bfUpSta;          // 是否更新，0:未更新，1: 更新 - from MCU
    unsigned char upresult;         // 0: 更新出错，　1: 更新成功

}UpMCU_t;

#pragma pack()   // 恢复默认




// #ifdef __cplusplus
// }
// #endif

#endif




