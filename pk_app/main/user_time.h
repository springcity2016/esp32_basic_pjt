/*
 * PETKIT Network Technology (Shanghai) Co., Ltd.
 * All rights reserved.
 *
 * user_time.h
 *
 *  Created on: 2019年8月8日
 *      Author: gjw
 */

#ifndef INCLUDE_USER_TIME_H_
#define INCLUDE_USER_TIME_H_

#define CTIME_AGEING_LEN                         (20)           //老化存储时间字符串长度


////////////////////////////////////////////////////////////////////////
//                      时间相关操作函数设计                          //
////////////////////////////////////////////////////////////////////////
#define PETKIT_YEAR_CHECK                   (2018)
#define PETKIT_YEAR_BEGIN                   (2015)
#define PETKIT_YEAR_MAX                     (2050)
#define YEAR_CENTRY_BEGAIN_2015             (1420070400UL)
#define YEAR_CENTRY_BEGAIN_2099             (4070908800UL)
#define YEAR_CENTRY_BEGAIN_2019             (1546272000UL)
#define YEAR_CENTRY_BEGAIN                  (946684800UL)
#define YEAR_BEGIN                          (2000)
#define YEAR_CYCLE                          (4)

#define YEAR_TEN_SECONDS                    (10)
#define YEAR_30_SECONDS                     (30)
#define YEAR_MIN_SECONDS                    (60)
#define YEAR_FIVE_MIN_SECONDS               (5*YEAR_MIN_SECONDS)
#define YEAR_HOUR_SECONDS                   (60*YEAR_MIN_SECONDS)
#define YEAR_TwoHOUR_SECONDS                (2*YEAR_HOUR_SECONDS)
#define YEAR_HALFDAY_SECONDS                (12*YEAR_HOUR_SECONDS)
#define YEAR_DAY_SECONDS                    (24*YEAR_HOUR_SECONDS)
#define YEAR_DAY_MIN                        (24*60)
#define YEAR_SMON_SECONDS                   (30*YEAR_DAY_SECONDS)
#define YEAR_NORMAL_SECONDS                 (365*YEAR_DAY_SECONDS)
#define YEAR_CYCLE_SECONDS                  ((YEAR_NORMAL_SECONDS * 4) + YEAR_DAY_SECONDS)
#define YEAR_NOFUNC_LOCK_SECONDS            (50*YEAR_NORMAL_SECONDS);
#define PIM_PINGPONG_SECONDS_MAX            (3*YEAR_MIN_SECONDS)
#define WIFI_SCAN_NEXT_SECONDS              (YEAR_MIN_SECONDS)

#define HALF_HOUR_SECONDS                   (1800)
#define QUARTER_SECONDS                     (900)
#define TEN_MIN_SECONDS                     (600)
#define HALF_MIN_SECONDS                    (30)
#define PK_MAX_SECONDS                      (YEAR_NORMAL_SECONDS + YEAR_HOUR_SECONDS)

#define DAY_SECONDS                         (86400)

#define PETKIT_RTC_RO_MODE                  (0)
#define PETKIT_RTC_RW_MODE                  (1)
#define PETKIT_RTC_SET_MODE                 (2)

#pragma pack(4)
typedef struct _sync_timer_t {
    unsigned int            sync_UTC;
    unsigned int            sync_acc;                           //单位s,内部RTC计时
    unsigned int            powOn_acc;                          //单位s,内部RTC计时
} sync_timer_t;

typedef struct  {
    unsigned long long      time_acc_ms;                        //单位ms
    unsigned int            time_base;                          //单位ms
    unsigned int            magic;                              //RTC_MAGIC

    int                     timezone;

    sync_timer_t            sync_info;
} RTC_mem;
#pragma pack()

typedef struct _ctimer_t
{
    unsigned char sec;
    unsigned char min;
    unsigned char hour;
    unsigned char day;
    unsigned char mon;
    unsigned short year;
    unsigned char week;
    int zone;

    short hzone;
    short mzone;

    int os_sec_day;
    int OS_UTC_Sec;
    int OS_run_Sec;
    int OS_runtime;

    sync_timer_t            sync_info;
    struct _ctimer_t        *settimer;
} ctimer_t;

typedef struct _pkHal_ctimer_t
{
    unsigned char sec;          // 秒
    unsigned char min;          // 分
    unsigned char hour;         // 时
    unsigned char day;          // 日
    unsigned char mon;          // 月
    unsigned short year;        // 年
    unsigned char week;         // 周
    int zone;                   // 时区；举个栗子：+800为东8区，-400为西区，有效区间[-1400, 1400]

    int sec_day;                // 每天对应0点的秒数，有效区间[0, 86400)
    unsigned int UTC;           // 0时区 对应 1970年1/1 0:0:0

} pkHal_ctimer_t;

typedef enum
{
    E_UTC_to_LOCAL = 0,
    E_LOCAL_to_UTC,
    E_LOCAL_to_DAY,

    E_TIMESTAMP_MAX
}E_TIME_STAMP_TYPE;

typedef struct _rtc_t
{
    unsigned char sec;          // 秒
    unsigned char min;          // 分
    unsigned char hour;         // 时
    unsigned char day;          // 日
    unsigned char mon;          // 月
    unsigned short year;        // 年
    unsigned char week;         // 周    
}RTCValue_t;

typedef struct _str_t
{
    unsigned short s16len;      // 字符串長度
    char *strChar;                  // 字符串存儲位置
}str_t;

enum
{
    E_DATETIME_YEAR_QIAN = 0,
    E_DATETIME_YEAR_BAI,
    E_DATETIME_YEAR_SHI,
    E_DATETIME_YEAR_GE,
    E_DATETIME_HENG_1ST,
    E_DATETIME_MON_SHI,
    E_DATETIME_MON_GE,
    E_DATETIME_HENG_2ND,
    E_DATETIME_DAY_SHI,
    E_DATETIME_DAY_GE,
    E_DATETIME_KONGGE,
    E_DATETIME_HOUR_SHI,
    E_DATETIME_HOUR_GE,
    E_DATETIME_MAOH_1ST,
    E_DATETIME_MIN_SHI,
    E_DATETIME_MIN_GE,
    E_DATETIME_MAOH_2ND,
    E_DATETIME_SEC_SHI,
    E_DATETIME_SEC_GE,

    E_DATETIME_MAX
};

int pkHal_time_init(void);


void UTC_2_ctime(unsigned int utc_sec, ctimer_t *result, unsigned char daylightSaving);
unsigned int ctime_2_UTC(ctimer_t *currTime, unsigned char daylightSaving);

int ctime_save_str_timer_sec(ctimer_t *ctime, char *timestatmp);
char *ctime_get_str_timer(ctimer_t *ctime);
void ctime_disp_timer(ctimer_t *ctime);
unsigned int ctime_get_UTC_timestamp(ctimer_t *ctime);
unsigned int ctime_get_current_timestamp(ctimer_t *ctime);
unsigned int ctime_get_days_timestamp(ctimer_t *ctime);
int ctime_read_timer_func(ctimer_t *ctime,int timezone);
int ctime_set_timer_func(short year,short mon,short day,short hour,short min,short sec);
// void feeder_time_init(void);
unsigned long long get_esp8266_runtime_ms(char mode, sync_timer_t* p_info);
unsigned int get_esp8266_runtime_sec(char mode, sync_timer_t* p_info);
int Is_RTC_MAGIC_OK(void);
int get_espRTC_UTC(void);
int get_espRTC_sleeptime(void);



#endif /* INCLUDE_USER_TIME_H_ */
