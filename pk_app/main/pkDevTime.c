
/*
 * PETKIT Network Technology (Shanghai) Co., Ltd.
 * All rights reserved.
 * 
 * pkDevRTC.c
 *
 *  Created on: 2019-3-6
 *      Author: chunchen.cao
 */

#define _PK_DEVTIME_C_

// #include "pkHeaders.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "esp_system.h"
// #include "spi_flash.h"
#include "driver/i2c.h"

// #include "driver.h"
// #include "version.h"
#include "user_time.h"
// #include "user_config.h"
#include "petkit_clog.h"
#include "user_lib.h"

#define UTC_BASE_YEAR                                   (1970)
#define MONTH_PER_YEAR                                  (12)
#define DAY_PER_YEAR                                    (365)
#define SEC_PER_DAY                                     (86400)
#define SEC_PER_HOUR                                    (3600)
#define SEC_PER_MIN                                     (60)

#define RTC_MAGIC                                       (0x55aaaa55)
#define RTC_BASE_ADDR                                   (64)

/*******************************************RTC**********************************************************/
enum
{
    E_RTC_INDEX_SEC = 0,
    E_RTC_INDEX_MIN,
    E_RTC_INDEX_HOUR,
    E_RTC_INDEX_DAY,
    E_RTC_INDEX_WEEK,
    E_RTC_INDEX_MONTH,
    E_RTC_INDEX_YEAR,

    E_RTC_INDEX_MAX
} RTC_addr_e;

#define I2C_EXAMPLE_MASTER_SCL_IO           4                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           15               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define ADDR_I2C_DEVICES_RTC_W              0xa2
#define ADDR_I2C_DEVICES_RTC_R              0xa3

#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ADDR_I2C_DEVICES_RTC_CTS1           (0x00)
#define ADDR_I2C_DEVICES_RTC_CTS2           (0x01)
#define ADDR_I2C_DEVICES_RTC_SEC            (0x02)
#define ADDR_I2C_DEVICES_RTC_MIN            (0x03)
#define ADDR_I2C_DEVICES_RTC_HOUR           (0x04)
#define ADDR_I2C_DEVICES_RTC_DAYS           (0x05)
#define ADDR_I2C_DEVICES_RTC_WEEK           (0x06)
#define ADDR_I2C_DEVICES_RTC_MON_C          (0x07)
#define ADDR_I2C_DEVICES_RTC_YEARS          (0x08)
#define ADDR_I2C_DEVICES_RTC_ALARM_MIN      (0x09)
#define ADDR_I2C_DEVICES_RTC_ALARM_HOUR     (0x0A)
#define ADDR_I2C_DEVICES_RTC_ALARM_DAY      (0x0B)
#define ADDR_I2C_DEVICES_RTC_ALARM_WEEK     (0x0C)
#define ADDR_I2C_DEVICES_RTC_CLKFREQ        (0x0D)
#define ADDR_I2C_DEVICES_RTC_TCTL           (0x0E)
#define ADDR_I2C_DEVICES_RTC_TDOWN_CNT      (0x0F)

#define ACK_CHECK_EN                        (0x1)              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       (0x0)              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             (0x0)              /*!< I2C ack value */
#define NACK_VAL                            (0x1)              /*!< I2C nack value */
#define LAST_NACK_VAL                       (0x2)              /*!< I2C last_nack value */

#define I2CCOMM_WAITTIME                    (100/portTICK_RATE_MS)

/******************************************************************************************/
ctimer_t gs_real_time;
int  gs_OS_UTCSec = 0;                                  //定义全局UTC时间变量

// extern int gs_runtime;
// extern low_pow_sta_t g_low_pow_sta;
/*************************************************************************************************/
/*************************************************************************************************/
static const unsigned char g_day_per_mon[MONTH_PER_YEAR] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
static int i2c_master_init(void);
unsigned int system_rtc_clock_cali_proc(void);
unsigned int system_get_rtc_time(void);
/******************************************************************************************/
/*
 * 功能： 判断是否是闰年
 * 参数： year：需要判断的年份数
 * 返回值： 闰年返回1，否则返回0
 */
static unsigned char applib_dt_is_leap_year(unsigned short year)
{
    if ((year % 400) == 0) {
        return 1;
    } else if ((year % 100) == 0) {
        return 0;
    } else if ((year % 4) == 0) {
        return 1;
    } else {
        return 0;
    }
}

/*
 * 功能：得到每个月有多少天
 * 参数：month：需要得到天数的月份数
 *       year：该月所对应的年份数
 * 返回值：该月有多少天
 */
static unsigned char applib_dt_last_day_of_mon(unsigned char month, unsigned short year)
{
    if ((month == 0) || (month > 12)) {
        return g_day_per_mon[1] + applib_dt_is_leap_year(year);
    }

    if (month != 2) {
        return g_day_per_mon[month - 1];
    } else {
        return g_day_per_mon[1] + applib_dt_is_leap_year(year);
    }
}

/*
 * 功能：  根据给定的日期得到对应的星期
 * 参数：  year：给定的年份
 *         month：给定的月份
 *         day：给定的天数
 * 返回值：对应的星期数，0 - 星期天 ... 6 - 星期六
 */
static unsigned char Cal_Zeller_Week(unsigned short year, unsigned char month, unsigned char day)
{
    char century_code, year_code, month_code, day_code;
    int week = 0;

    century_code = year_code = month_code = day_code = 0;

    if (month == 1 || month == 2) {
        century_code = (year - 1) / 100;
        year_code = (year - 1) % 100;
        month_code = month + 12;
        day_code = day;
    } else {
        century_code = year / 100;
        year_code = year % 100;
        month_code = month;
        day_code = day;
    }

    /* 根据蔡勒公式计算星期 */
    week = year_code + year_code / 4 + century_code / 4 - 2 * century_code + 26 * ( month_code + 1 ) / 10 + day_code - 1;
    week = week > 0 ? (week % 7) : ((week % 7) + 7);

    return week;
}

/*
 * 功能： 根据UTC时间戳得到对应的日期
 * 参数： utc_sec：给定的UTC时间戳
 *     result：计算出的结果
 *     daylightSaving：是否是夏令时
 * 返回值： 无
 */
void UTC_2_ctime(unsigned int utc_sec, ctimer_t *result, unsigned char daylightSaving)
{
    int sec = 0;
    int day = 0;
    unsigned short y;
    unsigned char m;
    unsigned short d;

    if ((NULL == result) || (utc_sec < YEAR_CENTRY_BEGAIN)) {
        pkLOG_ERR("UTC_2_ctime ERR!\n");
        return;
    }

    utc_sec = utc_sec + result->zone;

    if (daylightSaving) {
        utc_sec += SEC_PER_HOUR;
    }

    /* hour, min, sec */
    /* hour */
    sec = utc_sec % SEC_PER_DAY;
    result->hour = sec / SEC_PER_HOUR;

    /* min */
    sec %= SEC_PER_HOUR;
    result->min = sec / SEC_PER_MIN;

    /* sec */
    result->sec = sec % SEC_PER_MIN;

    /* year, month, day */
    /* year */
    /* year */
    day = utc_sec / SEC_PER_DAY;
    for (y = UTC_BASE_YEAR; day > 0; y++) {
        d = (DAY_PER_YEAR + applib_dt_is_leap_year(y));
        if (day >= d) {
            day -= d;
        } else {
            break;
        }
    }

    result->year = y;

    for (m = 1; m < MONTH_PER_YEAR; m++) {
        d = applib_dt_last_day_of_mon(m, y);
        if (day >= d) {
            day -= d;
        } else {
            break;
        }
    }

    result->mon = m;
    result->day = (unsigned char) (day + 1);
    /* 根据给定的日期得到对应的星期 */
    result->week = Cal_Zeller_Week(result->year, result->mon, result->day);
}

/*
 * 功能： 根据时间计算出UTC时间戳
 * 参数： currTime：给定的时间
 *     daylightSaving：是否是夏令时
 * 返回值： UTC时间戳
 */
unsigned int ctime_2_UTC(ctimer_t *currTime, unsigned char daylightSaving)
{
    unsigned short i = 0;
    unsigned int no_of_days = 0;
    int utc_time = 0;

    if (currTime->year < UTC_BASE_YEAR) {
        return 0;
    }

    /* year */
    for (i = UTC_BASE_YEAR; i < currTime->year; i++) {
        no_of_days += (DAY_PER_YEAR + applib_dt_is_leap_year(i));
    }

    /* month */
    for (i = 1; i < currTime->mon; i++) {
        no_of_days += applib_dt_last_day_of_mon((unsigned char) i, currTime->year);
    }

    /* day */
    no_of_days += (currTime->day - 1);

    /* sec */
    utc_time = (unsigned int) no_of_days * SEC_PER_DAY + (unsigned int) (currTime->hour * SEC_PER_HOUR +
                                                                currTime->min * SEC_PER_MIN + currTime->sec);

    if (daylightSaving) {
        utc_time -= SEC_PER_HOUR;
    }

    return utc_time;
}

/***********************************************
 * 函数名：ctime_disp_timer
 * 功    能：打印时间信息
 * 参    数：@ctime  时间结构指针
 * 返    回：无
 ***********************************************/
void ctime_disp_timer(ctimer_t *ctime)
{
    char *str=NULL;

    str = ctime_get_str_timer(ctime);
    if (str) {
        pkLOG_INFO(" - - -%s\n",str);
        free(str);
    }
}

/***********************************************
 * 函数名：ctime_save_str_timer_sec
 * 功    能：存储秒时间到一个区域
 * 参    数：@ctime  时间结构指针
 * 返    回：无
 ***********************************************/
int ctime_save_str_timer_sec(ctimer_t *ctime, char *timestatmp)
{
    if(ctime == NULL){
        pkLOG_INFO("------->save the time buffer error\n");
        return pdFALSE;
    }

    if(timestatmp != NULL){
        bzero(timestatmp,CTIME_AGEING_LEN);
        sprintf(timestatmp,"%04d%02d%02d-%02d:%02d:%02d",ctime->year,ctime->mon,ctime->day,ctime->hour,ctime->min,ctime->sec);
        pkLOG_INFO("------->creat time stamp:%s\n",timestatmp);
        return pdTRUE;
    }

    return pdFALSE;
}
/***********************************************
 * 函数名：ctime_get_str_timer
 * 功    能：打印时间信息
 * 参    数：@ctime  时间结构指针
 * 返    回：无
 ***********************************************/
char *ctime_get_str_timer(ctimer_t *ctime)
{
    char *str=NULL;

    str=(char *)malloc(100);
    if(str != NULL){
        bzero(str,100);
        sprintf(str,"%04d-%02d-%02dT%02d:%02d:%02d.0%s%02d%02d",
                ctime->year,ctime->mon,ctime->day,
                ctime->hour,ctime->min,ctime->sec,ctime->zone<0?"-":"+",abs(ctime->zone/3600),abs((ctime->zone%3600)/60));
    }
    return str;
}
/***********************************************
 * 函数名：ctime_get_UTC_timestamp
 * 功    能：获取系统的绝对时间秒数
 * 参    数：@ctime  时间结构指针
 * 返    回：@unsigned int utc绝对时间秒数
 ***********************************************/
unsigned int ctime_get_UTC_timestamp(ctimer_t *ctime)
{
    unsigned int timestamp = 0;

    timestamp = YEAR_CENTRY_BEGAIN;
    if(ctime != NULL){
        timestamp = ctime_get_current_timestamp(ctime);
        //时区的偏差
//      timestamp += ctime->zone;

        pkLOG_DEBUG("timestamp:%d, zone:%d, UTC:%d\n", timestamp, ctime->zone, gs_OS_UTCSec);
        timestamp =timestamp-(ctime->zone);
    }
    return timestamp;
}
/***********************************************
 * 函数名：ctime_get_current_timestamp
 * 功    能：获取系统的相对时间秒数
 * 参    数：@ctime  时间结构指针
 * 返    回：@unsigned int utc绝对时间秒数
 ***********************************************/
unsigned int ctime_get_current_timestamp(ctimer_t *ctime)
{
    unsigned int timestamp;
    int leng,circle,remainder;
    int i;

    timestamp = YEAR_CENTRY_BEGAIN;
    if(ctime != NULL){
        if(ctime->year<YEAR_BEGIN) return timestamp;

        leng = ctime->year-YEAR_BEGIN;
        remainder = leng%YEAR_CYCLE;
        circle = leng/YEAR_CYCLE;

        //计算周期的全部秒数
        timestamp += circle*YEAR_CYCLE_SECONDS;
        //计算未纳入年周期计算的秒数
        //当前年不是相对2000的第一年就需要加一个闰年的天
        if(remainder > 0){
            for(i=0;i<remainder;i++)
            {
                timestamp += YEAR_NORMAL_SECONDS;
                if(i==0){//如果包含了一个闰年就需要多加1天
                    timestamp += YEAR_DAY_SECONDS;
                }
            }
        }

        //计算未纳入周期的月计算秒数，当月不算
        i = 1;
        while(i<ctime->mon)
        {
            timestamp += YEAR_SMON_SECONDS;
            if(i<8){//1-7月，单数就+1变成31天
                if((i&1)>0){
                    timestamp += YEAR_DAY_SECONDS;
                }else if(i == 2){
                    timestamp -= YEAR_DAY_SECONDS;//降低为计算29天
                    if(remainder != 0){                     //如果当前不是闰年，再减一天计算28天
                        timestamp -= YEAR_DAY_SECONDS;
                    }
                }
            }else{//8-12月，双数就+1变成31天
                if(0 == (i&1)){
                    timestamp += YEAR_DAY_SECONDS;
                }
            }
            i++;
        }
        timestamp += (ctime->day-1)*YEAR_DAY_SECONDS;
        timestamp += ctime->hour*YEAR_HOUR_SECONDS;
        timestamp += ctime->min*YEAR_MIN_SECONDS;
        timestamp += ctime->sec;
    }
    return timestamp;
}
/***********************************************
 * 函数名：ctime_get_days_timestamp
 * 功    能：获取当前的相对时间秒数
 * 参    数：@ctime  时间结构指针
 * 返    回：@unsigned int utc绝对时间秒数
 ***********************************************/
unsigned int ctime_get_days_timestamp(ctimer_t *ctime)
{
    unsigned int daystamp;

    daystamp = 0;
    if(ctime != NULL){
        if(ctime->year<YEAR_BEGIN) return 0;

        //计算小时
        daystamp += ctime->hour*YEAR_HOUR_SECONDS;
        //计算分钟
        daystamp += ctime->min*YEAR_MIN_SECONDS;
        //计算秒
        daystamp += ctime->sec;

        //计算当前时间是0点的多少秒，用于计划表的时间对比
        ctime->os_sec_day = daystamp;
    }
    return daystamp;
}

/***********************************************
 * 函数名：ctime_set_timer_func
 * 功    能：系统针对rtc进行初始化，同时计算星期
 * 蔡勒（Zeller）公式 ：
 * w=y+[y/4]+[c/4]-2c+[26(m+1)/10]+d-1
 * 参    数：无
 * 返    回：无
 ***********************************************/
int ctime_set_timer_func(short year,short mon,short day,short hour,short min,short sec)
{
    unsigned char u08Cnt = 0;
    unsigned char au08Data[E_RTC_INDEX_MAX] = { 0 };

    if ((mon == 0) || (day == 0) || (year > 2099)) {
        return ESP_FAIL;
    }

    pkLOG_INFO("SetRTCTime: %d-%d-%d %d:%d:%d\n", year, mon, day, hour, min, sec);

    /**************计算week************/
    // 蔡勒（Zeller）公式 ：
    // w=y+[y/4]+[c/4]-2c+[26(m+1)/10]+d-1
    au08Data[E_RTC_INDEX_WEEK] = Cal_Zeller_Week(year, mon, day);
    /*********************************/
    au08Data[E_RTC_INDEX_DAY] = ((day / 10) << 4) + (day % 10);
    au08Data[E_RTC_INDEX_MONTH] = ((mon / 10) << 4) + (mon % 10);
    if (year > 1999) {
        au08Data[E_RTC_INDEX_MONTH] &= 0x7F;
        au08Data[E_RTC_INDEX_YEAR] = year - 2000;
    } else {
        au08Data[E_RTC_INDEX_MONTH] |= 0x80;
        au08Data[E_RTC_INDEX_YEAR] = year - 1999;
    }
    au08Data[E_RTC_INDEX_YEAR] = ((au08Data[E_RTC_INDEX_YEAR] / 10) << 4) + (au08Data[E_RTC_INDEX_YEAR] % 10);
    au08Data[E_RTC_INDEX_HOUR] = ((hour / 10) << 4) + (hour % 10);
    au08Data[E_RTC_INDEX_MIN] = ((min / 10) << 4) + (min % 10);
    au08Data[E_RTC_INDEX_SEC] = ((sec / 10) << 4) + (sec % 10);

    /*********************************/
    for (u08Cnt = 0; u08Cnt < E_RTC_INDEX_MAX; u08Cnt++) {
        pkLOG_INFO("%02x ", au08Data[u08Cnt]);
    }
    pkLOG_INFO("\n");

    pkLOG_EMERG("write:%02x-%02x-%02x (%d) %02x:%02x:%02x!!\n",year,mon,day,au08Data[E_RTC_INDEX_WEEK],hour,min,sec);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if (ESP_OK != i2c_master_write_byte(cmd, ADDR_I2C_DEVICES_RTC_W, ACK_CHECK_EN)) {
        i2c_master_stop(cmd);
        i2c_cmd_link_delete(cmd);
        pkLOG_EMERG("I2C write byte - I2C Address Error! \n");
        return ESP_FAIL;
    }
    if (ESP_OK != i2c_master_write_byte(cmd, ADDR_I2C_DEVICES_RTC_SEC, ACK_CHECK_EN)) {
        i2c_master_stop(cmd);
        i2c_cmd_link_delete(cmd);
        pkLOG_EMERG("I2C write byte - Rtc Sec Address Error! \n");
        return ESP_FAIL;
    }
    if (ESP_OK != i2c_master_write(cmd, au08Data, E_RTC_INDEX_MAX, ACK_CHECK_EN)) {
        i2c_master_stop(cmd);
        i2c_cmd_link_delete(cmd);
        pkLOG_EMERG("I2C write byte - RTC all datas Error! \n");
        return ESP_FAIL;
    }

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, I2CCOMM_WAITTIME);
    i2c_cmd_link_delete(cmd);

    return pdTRUE;
}

static int _ctime_read_timer(ctimer_t *ctime)
{
    unsigned char sec = 0;
    unsigned char min = 0;
    unsigned char hour = 0;
    unsigned char day = 0;
    unsigned char mon = 0;
    unsigned char year = 0;
    unsigned char week = 0;
    short cen = 0;
    int ret = 0;
    unsigned char au08Data[E_RTC_INDEX_MAX] = { 0 };


    if (ctime == NULL) {
        return ESP_FAIL;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if (ESP_OK != i2c_master_write_byte(cmd, ADDR_I2C_DEVICES_RTC_W, ACK_CHECK_EN)) {
        i2c_master_stop(cmd);
        i2c_cmd_link_delete(cmd);
        printf("I2C write byte - I2C Write Address Error! \n");
        return ESP_FAIL;
    }
    if (ESP_OK != i2c_master_write_byte(cmd, ADDR_I2C_DEVICES_RTC_SEC, ACK_CHECK_EN)) {
        i2c_master_stop(cmd);
        i2c_cmd_link_delete(cmd);
        printf("I2C write byte - I2C Write Rtc SecAddress Error! \n");
        return ESP_FAIL;
    }
    i2c_master_start(cmd);
    if (ESP_OK != i2c_master_write_byte(cmd, ADDR_I2C_DEVICES_RTC_R, ACK_CHECK_EN)) {
        i2c_master_stop(cmd);
        i2c_cmd_link_delete(cmd);
        printf("I2C write byte - I2C Read Address Error! \n");
        return ESP_FAIL;
    }
    if (ESP_OK != i2c_master_read(cmd, au08Data, E_RTC_INDEX_MAX, LAST_NACK_VAL)) {
        i2c_master_stop(cmd);
        i2c_cmd_link_delete(cmd);
        printf("I2C Read byte - I2C Read All Bytes Error! \n");
        return ESP_FAIL;
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, I2CCOMM_WAITTIME);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        printf("I2C Read TimeDatas Error!\n");
       return ret;
    }

    if ((au08Data[E_RTC_INDEX_MONTH] & 0x80) > 0) {
        cen = 1900;
    } else {
        cen = 2000;
    }
    au08Data[E_RTC_INDEX_SEC] = ((au08Data[E_RTC_INDEX_SEC] & 0x70) >> 4) * 10 + (au08Data[E_RTC_INDEX_SEC] & 0x0f);
    au08Data[E_RTC_INDEX_MIN] = ((au08Data[E_RTC_INDEX_MIN] & 0x70) >> 4) * 10 + (au08Data[E_RTC_INDEX_MIN] & 0x0f);
    au08Data[E_RTC_INDEX_HOUR] = ((au08Data[E_RTC_INDEX_HOUR] & 0x30) >> 4) * 10 + (au08Data[E_RTC_INDEX_HOUR] & 0x0f);
    au08Data[E_RTC_INDEX_DAY] = ((au08Data[E_RTC_INDEX_DAY] & 0x30) >> 4) * 10 + (au08Data[E_RTC_INDEX_DAY] & 0x0f);
    au08Data[E_RTC_INDEX_WEEK] = (au08Data[E_RTC_INDEX_WEEK] & 0x07);
    au08Data[E_RTC_INDEX_MONTH] = ((au08Data[E_RTC_INDEX_MONTH] & 0x10) >> 4) * 10+(au08Data[E_RTC_INDEX_MONTH] & 0x0f);
    au08Data[E_RTC_INDEX_YEAR] = ((au08Data[E_RTC_INDEX_YEAR] & 0xf0) >> 4) * 10 + (au08Data[E_RTC_INDEX_YEAR] & 0x0f);

    sec = au08Data[E_RTC_INDEX_SEC];
    min = au08Data[E_RTC_INDEX_MIN];
    hour = au08Data[E_RTC_INDEX_HOUR];
    day = au08Data[E_RTC_INDEX_DAY];
    mon = au08Data[E_RTC_INDEX_MONTH];
    year = au08Data[E_RTC_INDEX_YEAR];
    week = au08Data[E_RTC_INDEX_WEEK];
    cen += year;


    if (sec > 59 || min > 59 || hour > 23 || day > 31
            || (day == 0) || week > 6 || mon > 12
            || (mon == 0) || cen > 2050 || cen < 2010) {

        // pkLOG_EMERG("%d-%d-%d, %d:%d:%d, w:%d\n", cen, mon, day, hour, min, sec, week);
        // pkLOG_EMERG("RTC read error,need init the value.....\n");
        return pdFALSE;
    }

    ctime->sec = sec;
    ctime->min = min;
    ctime->hour = hour;
    ctime->day = day;
    ctime->mon = mon;
    ctime->year = cen;
    ctime->week = week;

    return pdTRUE;
}

/***********************************************
 * 函数名：ctime_read_timer_func
 * 功    能：系统读取rtc时间
 * 参    数：@ctime 时间参数指针
 * 返    回：无
 ***********************************××**********/
int ctime_read_timer_func(ctimer_t *ctime,int timezone)
{
    int i = 0;
    ctimer_t tmp_read;
    int read_UTC = 0;
    int old_UTC = 0;
    int succ_cnt = 0;

    for (i = 0; i < 10; i++) {
        memset(&tmp_read, 0, sizeof(ctimer_t));
        if (pdTRUE == _ctime_read_timer(&tmp_read)) {
            read_UTC = ctime_get_UTC_timestamp(&tmp_read);
            if ((read_UTC < old_UTC + 2) && (read_UTC + 2 > old_UTC)) {
                succ_cnt++;
                if (succ_cnt >= 3) {
                    break;
                }
            } else {
                succ_cnt = 0;
            }
            old_UTC = read_UTC;
        } else {
            succ_cnt = 0;
        }
    }

    if (succ_cnt < 3) {
        return pdFALSE;
    }

    ctime->sec = tmp_read.sec;
    ctime->min = tmp_read.min;
    ctime->hour = tmp_read.hour;
    ctime->day = tmp_read.day;
    ctime->mon = tmp_read.mon;
    ctime->year = tmp_read.year;
    ctime->week = tmp_read.week;
    ctime->zone = timezone;

    ctime->hzone = abs(ctime->zone/3600);
    ctime->mzone = abs((ctime->zone%3600)/60);
    ctime->hzone *= (ctime->zone<0)?(-1):1;

    return pdTRUE;

}

unsigned long long get_esp8266_rtc_ms(char mode, sync_timer_t* p_info)
{
    RTC_mem rtc_time;
    unsigned int rtc_now = 0;
    unsigned int rtc_diff = 0;
    unsigned int acc = 0;
    unsigned long long cal = 0;
    static unsigned int rtc_W_acc = 0;

    memset(&rtc_time, 0, sizeof(RTC_mem));
//    system_rtc_mem_read(RTC_BASE_ADDR, &rtc_time, sizeof(RTC_mem));
    if (RTC_MAGIC != rtc_time.magic) {
        // pkLOG_ERR("rtc magic ERR!!\n");
        memset(&rtc_time, 0, sizeof(RTC_mem));
        rtc_time.magic = RTC_MAGIC;
        // rtc_time.time_acc_ms = (long long)gs_runtime * 1000;
        rtc_time.time_base = system_get_rtc_time();
        rtc_time.sync_info.sync_UTC = 0;
        rtc_time.sync_info.sync_acc = 0;
        rtc_time.sync_info.powOn_acc = 1;
        rtc_W_acc = 0;
        mode = PETKIT_RTC_RW_MODE;
    } else {
        if (((rtc_time.time_acc_ms / 1000) < rtc_time.sync_info.powOn_acc)
                || (rtc_time.sync_info.powOn_acc == 0)) {
            rtc_time.sync_info.powOn_acc = (int)(rtc_time.time_acc_ms / 1000);
        }
        if (NULL == p_info) {
            memcpy(&(gs_real_time.sync_info), &(rtc_time.sync_info), sizeof(sync_timer_t));
        }
    }

    rtc_now = system_get_rtc_time();
    cal = system_rtc_clock_cali_proc();
    cal = ((cal * 1000) >> 12);

    if (rtc_now < rtc_time.time_base) {
        rtc_diff = (0xffffffff - rtc_time.time_base) + rtc_now;
    } else {
        rtc_diff = rtc_now - rtc_time.time_base;
    }
    rtc_time.time_acc_ms += (long long)((long long)cal * (long long)(rtc_diff)) / 1000000;
    rtc_time.time_base = rtc_now;
    rtc_time.timezone = gs_real_time.zone;
    acc = (unsigned int)(rtc_time.time_acc_ms / 1000);

    if ((PETKIT_RTC_RW_MODE == mode) || (acc > rtc_W_acc + YEAR_HOUR_SECONDS)) {
        if ((NULL != p_info) && (PETKIT_RTC_RW_MODE == mode)) {
            memcpy(&(rtc_time.sync_info), p_info, sizeof(sync_timer_t));
            rtc_time.sync_info.sync_acc = acc;
        }
//        system_rtc_mem_write(RTC_BASE_ADDR, &rtc_time, sizeof(RTC_mem));
        rtc_W_acc = acc;
    }
//    if (0 == gs_real_time.cur_time.OS_run_Sec % 10) {
//        pkLOG_INFO("Rtime:%d, OS_UTC_Sec:%d\n", gs_real_time.cur_time.OS_run_Sec, gs_real_time.cur_time.OS_UTC_Sec);
//    }

    return rtc_time.time_acc_ms;
}

// void feeder_time_init(void)
// {
//     gs_real_time.settimer = NULL;
//     gs_real_time.sec = 1;
//     gs_real_time.min = 1;
//     gs_real_time.hour = 1;
//     gs_real_time.day = 1;
//     gs_real_time.mon = 1;
//     gs_real_time.year = 2015;
//     gs_real_time.week = 3;
//     //gs_real_time.zone = gs_dev_info->timezone;

// //    if (pdFALSE == ctime_read_timer_func(&gs_real_time, gs_dev_info->timezone)) {
//     if (pdTRUE != update_ctime_from_RTC(1)) {
//         sys_write_state(ST_RTC_ERR_MASK, MASK_SET);
//         gs_real_time.year = 2015;
//         if (gs_OS_UTCSec > YEAR_CENTRY_BEGAIN) {
//             UTC_2_ctime((unsigned int)gs_OS_UTCSec, &gs_real_time, 0);
//             ctime_disp_timer(&gs_real_time);
//         } else {
//             gs_OS_UTCSec = ctime_get_UTC_timestamp(&gs_real_time);
//         }
//     } else {
//         gs_OS_UTCSec = ctime_get_UTC_timestamp(&gs_real_time);
//     }
//     ctime_get_days_timestamp(&gs_real_time);
//     ctime_disp_timer(&gs_real_time);

//     get_esp8266_runtime_sec(PETKIT_RTC_RW_MODE, NULL);

// }

unsigned long long get_esp8266_runtime_ms(char mode, sync_timer_t* p_info)
{
    return get_esp8266_rtc_ms(mode, p_info);
}

unsigned int get_esp8266_runtime_sec(char mode, sync_timer_t* p_info)
{
    return (unsigned int)(get_esp8266_rtc_ms(mode, p_info) / 1000);
}

int Is_RTC_MAGIC_OK(void)
{
    RTC_mem rtc_time;

    memset(&rtc_time, 0, sizeof(RTC_mem));
//    system_rtc_mem_read(RTC_BASE_ADDR, &rtc_time, sizeof(RTC_mem));
    if (RTC_MAGIC != rtc_time.magic) {
        return -1;
    } else {
        return 0;
    }
}

int get_espRTC_UTC(void)
{
    int esp_rtc_time = get_esp8266_runtime_sec(PETKIT_RTC_RO_MODE, NULL);
    if (gs_real_time.sync_info.sync_UTC && gs_real_time.sync_info.sync_acc) {
        if (esp_rtc_time > gs_real_time.sync_info.sync_acc) {
            return (gs_real_time.sync_info.sync_UTC + (esp_rtc_time - gs_real_time.sync_info.sync_acc));
        }
    }

    return -1;
}

int get_espRTC_sleeptime(void)
{
    int sleeptime = 0;
    // int esp_RTC_sec = get_esp8266_runtime_sec(PETKIT_RTC_RO_MODE, NULL);
    // if ((g_low_pow_sta.start_sleep_acc) && (esp_RTC_sec > g_low_pow_sta.start_sleep_acc)) {
    //     sleeptime = (esp_RTC_sec - g_low_pow_sta.start_sleep_acc);
    //     g_low_pow_sta.start_sleep_acc = 0;
    //     return sleeptime;
    // }

    return -1;
}

////////////////////////////////////////////////////////////////////////

int pkHal_time_init(void)
{
    if (ESP_OK != i2c_master_init())
    {
        printf("I2C Master Init Error! \n");
        return ESP_FAIL;
    }
    return ESP_OK;
}

int pkHal_Set_RTCtime(unsigned short u16Year, unsigned short u16Month, unsigned short u16Day, unsigned short u16Hour, unsigned short u16Min, unsigned short u16Sec)
{
    return ctime_set_timer_func(u16Year, u16Month, u16Day, u16Hour, u16Min, u16Sec);
}   

static unsigned int pkHal_Gettimestamp(const pkHal_ctimer_t *pstTime)
{
    unsigned int u32timestamp = 0;
    unsigned short u16DigitYears = 0;
    unsigned short u16RNCycle = 0;
    unsigned short u16Remainder = 0;
    unsigned short u16Cnt = 0;

    u32timestamp = YEAR_CENTRY_BEGAIN;
    if (pstTime == NULL)
    {
        printf("Local Time Input time NULL! \n");
        return u32timestamp;
    }

    if (pstTime->year < ((unsigned short)YEAR_BEGIN))
    {
        printf("Local Time - year < begin year(2000)  year = %d ! \n", pstTime->year);
        return u32timestamp;
    }

    u16DigitYears = pstTime->year - YEAR_BEGIN;
    u16Remainder = u16DigitYears % YEAR_CYCLE;
    u16RNCycle = u16DigitYears / YEAR_CYCLE;

    // 计算周期的全部秒数
    u32timestamp += u16RNCycle * YEAR_CYCLE_SECONDS;
	//计算未纳入年周期计算的秒数
	//当前年不是相对2000的第一年就需要加一个闰年的天  
    if (u16Remainder > 0)
    {
        for (u16Cnt = 0; u16Cnt < u16Remainder; u16Cnt++)
        {
            u32timestamp += YEAR_NORMAL_SECONDS;
            if (u16Cnt == 0) { u32timestamp += YEAR_DAY_SECONDS; }      // 如果包含了一个闰年就需要多加1天
        }
    }

    //计算未纳入周期的月计算秒数，当月不算
    for (u16Cnt = 1; u16Cnt < pstTime->mon; u16Cnt++)
    {
        u32timestamp += YEAR_SMON_SECONDS;
        if (u16Cnt < 8)
        {
            //1-7月，单数就+1变成31天
            if ((u16Cnt & 0x01) > 0) 
            { 
                u32timestamp += YEAR_DAY_SECONDS; 
            }
            else if (u16Cnt == 2)
            {
                u32timestamp -= YEAR_DAY_SECONDS;//降低为计算29天
				if (u16Remainder != 0){						//如果当前不是闰年，再减一天计算28天
					u32timestamp -= YEAR_DAY_SECONDS;
				}
            }
        }
        else
        {
            // 8-12月，双数就+1变成31天
            if ((u16Cnt & 0x01) == 0)
            {
                u32timestamp += YEAR_DAY_SECONDS;
            }
        }
    }

    u32timestamp += (pstTime->day - 1) * YEAR_DAY_SECONDS;
    u32timestamp += pstTime->hour * YEAR_HOUR_SECONDS;
    u32timestamp += pstTime->min * YEAR_MIN_SECONDS;
    u32timestamp += pstTime->sec;

    return u32timestamp;
}

// 计算当地时戳 
unsigned int pkHal_Get_UTCtoLocaltimestamp(const pkHal_ctimer_t *pstTime)
{
	unsigned int u32timestamp = 0;

    u32timestamp = pkHal_Gettimestamp(pstTime);
    // + 時區
    u32timestamp += (pstTime->zone / 100 * YEAR_HOUR_SECONDS + pstTime->zone % 100 * YEAR_MIN_SECONDS);

    return u32timestamp;
}

unsigned int pkHal_Get_LocalDaystimestamp(const pkHal_ctimer_t *pstTime)
{
    unsigned int u32timestamp = 0;

    if (pstTime == NULL) return u32timestamp;

    if (pstTime->year < YEAR_BEGIN) return u32timestamp;

    //计算小时
    u32timestamp += pstTime->hour * YEAR_HOUR_SECONDS;
    //计算分钟
    u32timestamp += pstTime->min * YEAR_MIN_SECONDS;
    //计算秒
    u32timestamp += pstTime->sec;

    return u32timestamp;
}

// 计算UTC时戳
unsigned int pkHal_Get_LocaltoUTCtimestamp(const pkHal_ctimer_t *pstTime)
{
    unsigned int u32timestamp = 0;
    u32timestamp = YEAR_CENTRY_BEGAIN;
    if (pstTime == NULL) 
    {
        printf("Get UTC timestamp - input time param NULL! \n");
        return u32timestamp;
    }

    u32timestamp = pkHal_Gettimestamp(pstTime);
    
	//时区的偏差
//		timestamp += ctime->zone;
    u32timestamp -= ((pstTime->zone / 100) * YEAR_HOUR_SECONDS + (pstTime->zone % 100 * 60 / 100) * YEAR_MIN_SECONDS);

    return u32timestamp;
}

unsigned int pkHal_Get_Sectimestamp(const pkHal_ctimer_t *pstSrcTime, E_TIME_STAMP_TYPE eMode)
{
    unsigned int u32timestamp = YEAR_CENTRY_BEGAIN;
    if (eMode >= E_TIMESTAMP_MAX)
    {
        return YEAR_CENTRY_BEGAIN;
    }
    switch(eMode)
    {
    case E_UTC_to_LOCAL:
        u32timestamp = pkHal_Get_UTCtoLocaltimestamp(pstSrcTime);
        break;
    case E_LOCAL_to_UTC:
        
        u32timestamp = pkHal_Get_LocaltoUTCtimestamp(pstSrcTime);
        break;
    case E_LOCAL_to_DAY:
        u32timestamp = pkHal_Get_LocalDaystimestamp(pstSrcTime);
        break;
    default: break;
    }

    return u32timestamp;
}
unsigned int pkHal_Set_Sectimestamp(const pkHal_ctimer_t *pstSrcTime, pkHal_ctimer_t *pstDstTime, E_TIME_STAMP_TYPE eMode)
{
    return 0;
}

unsigned long long pkHal_Get_Runtimems(char s08Mode)
{
    return ESP_OK;
}

unsigned int pkHal_Get_Runtimesec(char s08Mode)
{
    return ESP_OK;
}

// struct 数据时间 -> string 时间
short pkHal_Get_Timestring(const pkHal_ctimer_t *pstTime, char *strTime)
{
    if ((pstTime == NULL) || (strTime == NULL)) return ESP_FAIL;

    sprintf(strTime, "%04d-%02d-%02dT%02d:%02d:%02d.0%s%02d%02d",
            pstTime->year, pstTime->mon, pstTime->day, pstTime->hour, pstTime->min, pstTime->sec,
            (pstTime->zone < 0 ? "-" : "+"),
            abs(pstTime->zone / 3600),
            abs(pstTime->zone % 3600) / 60);
    return 50;  // 数据长度格式暂时为50
}

// struct 数据时间 -> string Sec
short pkHal_Get_Secstring(const pkHal_ctimer_t *pstTime, char *strSec)
{
    if ((pstTime == NULL) || (strSec == NULL)) return ESP_FAIL;
    sprintf(strSec, "%04d%02d%02d-%02d:%02d:%02d", pstTime->year, pstTime->mon, pstTime->day, 
            pstTime->hour, pstTime->min, pstTime->sec);
    return 20;
}

void pkHal_SleepMs(unsigned int u32Numms)
{
    vTaskDelay(u32Numms / portTICK_PERIOD_MS);
}

static unsigned int pkHal_Get_RTCtoSec(const RTCValue_t *pstSrc)
{
    unsigned int u32timestamp = 0;
    unsigned short u16DigitYears = 0;
    unsigned short u16RNCycle = 0;
    unsigned short u16Remainder = 0;
    unsigned short u16Cnt = 0;

    u32timestamp = YEAR_CENTRY_BEGAIN;
    if (pstSrc == NULL)
    {
        printf("Local Time Input time NULL! \n");
        return u32timestamp;
    }

    if (pstSrc->year < ((unsigned short)YEAR_BEGIN))
    {
        printf("Local Time - year < begin year(2000)  year = %d ! \n", pstSrc->year);
        return u32timestamp;
    }

    u16DigitYears = pstSrc->year - YEAR_BEGIN;
    u16Remainder = u16DigitYears % YEAR_CYCLE;
    u16RNCycle = u16DigitYears / YEAR_CYCLE;

    // 计算周期的全部秒数
    u32timestamp += u16RNCycle * YEAR_CYCLE_SECONDS;
	//计算未纳入年周期计算的秒数
	//当前年不是相对2000的第一年就需要加一个闰年的天  
    if (u16Remainder > 0)
    {
        for (u16Cnt = 0; u16Cnt < u16Remainder; u16Cnt++)
        {
            u32timestamp += YEAR_NORMAL_SECONDS;
            if (u16Cnt == 0) { u32timestamp += YEAR_DAY_SECONDS; }      // 如果包含了一个闰年就需要多加1天
        }
    }

    //计算未纳入周期的月计算秒数，当月不算
    for (u16Cnt = 1; u16Cnt < pstSrc->mon; u16Cnt++)
    {
        u32timestamp += YEAR_SMON_SECONDS;
        if (u16Cnt < 8)
        {
            //1-7月，单数就+1变成31天
            if ((u16Cnt & 0x01) > 0) 
            { 
                u32timestamp += YEAR_DAY_SECONDS; 
            }
            else if (u16Cnt == 2)
            {
                u32timestamp -= YEAR_DAY_SECONDS;//降低为计算29天
				if (u16Remainder != 0){						//如果当前不是闰年，再减一天计算28天
					u32timestamp -= YEAR_DAY_SECONDS;
				}
            }
        }
        else
        {
            // 8-12月，双数就+1变成31天
            if ((u16Cnt & 0x01) == 0)
            {
                u32timestamp += YEAR_DAY_SECONDS;
            }
        }
    }

    u32timestamp += (pstSrc->day - 1) * YEAR_DAY_SECONDS;
    u32timestamp += pstSrc->hour * YEAR_HOUR_SECONDS;
    u32timestamp += pstSrc->min * YEAR_MIN_SECONDS;
    u32timestamp += pstSrc->sec;

    return u32timestamp;    
}
static unsigned int pkHal_Get_StrtoSec(const str_t *pstrSrc)
{
    unsigned int u32timestamp = 0;
    RTCValue_t _stRTCTime;

    u32timestamp = YEAR_CENTRY_BEGAIN;
    if ((pstrSrc == NULL)
     || (pstrSrc->s16len == 0)
     || (pstrSrc->strChar == NULL))
    {
        printf("Set String to Sec Error! \n");
        return u32timestamp;
    }
    if ((pstrSrc->strChar[E_DATETIME_HENG_1ST] != '-')
     || (pstrSrc->strChar[E_DATETIME_HENG_2ND] != '-')
     || (pstrSrc->strChar[E_DATETIME_KONGGE] != ' ')
     || (pstrSrc->strChar[E_DATETIME_MAOH_1ST] != ':')
     || (pstrSrc->strChar[E_DATETIME_MAOH_2ND] != ':'))
    {
        printf("Input Time String Error! \n");
        return u32timestamp;
    }
    memset(&_stRTCTime, 0, sizeof(RTCValue_t));
    
    _stRTCTime.year = StringtoNum(&(pstrSrc->strChar[E_DATETIME_YEAR_QIAN]), 4);
    _stRTCTime.mon = StringtoNum(&(pstrSrc->strChar[E_DATETIME_MON_SHI]), 2);
    _stRTCTime.day = StringtoNum(&(pstrSrc->strChar[E_DATETIME_DAY_SHI]), 2);
    _stRTCTime.hour = StringtoNum(&(pstrSrc->strChar[E_DATETIME_HOUR_SHI]), 2);
    _stRTCTime.min = StringtoNum(&(pstrSrc->strChar[E_DATETIME_MIN_SHI]), 2);
    _stRTCTime.sec = StringtoNum(&(pstrSrc->strChar[E_DATETIME_SEC_SHI]), 2);
    u32timestamp = pkHal_Get_RTCtoSec(&_stRTCTime);

    return u32timestamp;
}
static int pkHal_Set_RTCtoStr(const RTCValue_t *pstSrc, str_t *pstrDst)
{
    if ((pstSrc == NULL)
     || (pstrDst == NULL)
     || (pstrDst->strChar == NULL)
     || (pstrDst->s16len < E_DATETIME_MAX))
    {
        pstrDst->s16len = 0;
        pstrDst->strChar = NULL;
        printf("Input RTCValue NULL, Error! \n");
        return ESP_FAIL;
    }

    NumtoString(pstSrc->year, 4, &(pstrDst->strChar[E_DATETIME_YEAR_QIAN]));
    pstrDst->strChar[E_DATETIME_HENG_1ST] = '-';
    NumtoString(pstSrc->mon, 2, &(pstrDst->strChar[E_DATETIME_MON_SHI]));
    pstrDst->strChar[E_DATETIME_HENG_2ND] = '-';
    NumtoString(pstSrc->day, 2, &(pstrDst->strChar[E_DATETIME_DAY_SHI]));
    pstrDst->strChar[E_DATETIME_KONGGE] = ' ';
    NumtoString(pstSrc->hour, 2, &(pstrDst->strChar[E_DATETIME_HOUR_SHI]));
    pstrDst->strChar[E_DATETIME_MAOH_1ST] = ':';
    NumtoString(pstSrc->min, 2, &(pstrDst->strChar[E_DATETIME_MIN_SHI]));
    pstrDst->strChar[E_DATETIME_MAOH_2ND] = ':';
    NumtoString(pstSrc->sec, 2, &(pstrDst->strChar[E_DATETIME_SEC_SHI]));

    return ESP_OK;
}

static int pkHal_Set_SectoRTC(const unsigned int u32SrcSec, RTCValue_t *pstDst)
{
    if (pstDst == NULL)
    {
        printf("pkHal_Set_SectoRTC Input Params Error! %s, %d \n", __FILE__, __LINE__);
        return ESP_FAIL;
    }

    // 等待開發
    printf("pkHal_Set_SectoRTC Function Wait Developer to Develop! %s, %d \n", __FILE__, __LINE__);
    
    return ESP_OK;
}


static int pkHal_Set_SectoStr(const unsigned int u32SrcSec, str_t *pstrDst)
{
    if ((pstrDst == NULL)
     || (pstrDst->strChar == NULL)
     || (pstrDst->s16len < E_DATETIME_MAX))
    {
        printf("pkHal_Set_SectoStr Input Params Error! %s, %d \n", __FILE__, __LINE__);
        return ESP_FAIL;
    }

    // 等待開發
    printf("pkHal_Set_SectoRTC Function Wait Developer to Develop! %s, %d \n", __FILE__, __LINE__);

    return ESP_OK;
}
static int pkHal_Set_StrtoRTC(const str_t *pstrSrc, RTCValue_t *pstDst)
{
    if ((pstrSrc == NULL)
     || (pstrSrc->strChar == NULL)
     || (pstrSrc->s16len < E_DATETIME_MAX)
     || (pstDst == NULL))
    {
        printf("pkHal_Set_StrtoRTC Input Params Error! %s, %d \n", __FILE__, __LINE__);
        return ESP_FAIL;
    }

    pstDst->year = StringtoNum(&(pstrSrc->strChar[E_DATETIME_YEAR_QIAN]), 4);
    pstDst->mon = StringtoNum(&(pstrSrc->strChar[E_DATETIME_MON_SHI]), 2);
    pstDst->day = StringtoNum(&(pstrSrc->strChar[E_DATETIME_DAY_SHI]), 2);
    pstDst->hour = StringtoNum(&(pstrSrc->strChar[E_DATETIME_HOUR_SHI]), 2);
    pstDst->min = StringtoNum(&(pstrSrc->strChar[E_DATETIME_MIN_SHI]), 2);
    pstDst->sec = StringtoNum(&(pstrSrc->strChar[E_DATETIME_SEC_SHI]), 2);

    return ESP_OK;
}

unsigned int pkHal_Get_UTC_RTCtoSec(const RTCValue_t *pstSrc)
{
    return (pkHal_Get_RTCtoSec(pstSrc));
}
unsigned int pkHal_Get_UTC_StrtoSec(const str_t *pstrSrc)
{
    return (pkHal_Get_StrtoSec(pstrSrc));
}
int pkHal_Set_UTC_RTCtoStr(const RTCValue_t *pstSrc, str_t *pstrDst)
{
    return pkHal_Set_RTCtoStr(pstSrc, pstrDst);
}
int pkHal_Set_UTC_SectoStr(const unsigned int u32SrcSec, str_t *pstrDst)
{
    return pkHal_Set_SectoStr(u32SrcSec, pstrDst);
}
int pkHal_Set_UTC_StrtoRTC(const str_t *pstrSrc, RTCValue_t *pstDst)
{
    return pkHal_Set_StrtoRTC(pstrSrc, pstDst);
}
int pkHal_Set_UTC_SectoRTC(const unsigned int u32SrcSec, RTCValue_t *pstDst)
{
    return pkHal_Set_SectoRTC(u32SrcSec, pstDst);
}

unsigned int pkHal_Get_Local_RTCtoSec(const RTCValue_t *pstSrc)
{
    return (pkHal_Get_RTCtoSec(pstSrc));
}
unsigned int pkHal_Get_Local_StrtoSec(const str_t *pstrSrc)
{
    return (pkHal_Get_StrtoSec(pstrSrc));
}
int pkHal_Set_Local_RTCtoStr(const RTCValue_t *pstSrc, str_t *pstrDst)
{
    return pkHal_Set_RTCtoStr(pstSrc, pstrDst);
}
int pkHal_Set_Local_SectoStr(const unsigned int u32SrcSec, str_t *pstrDst)
{
    return pkHal_Set_SectoStr(u32SrcSec, pstrDst);
}
int pkHal_Set_Local_StrtoRTC(const str_t *pstrSrc, RTCValue_t *pstDst)
{
    return pkHal_Set_StrtoRTC(pstrSrc, pstDst);
}
int pkHal_Set_Local_SectoRTC(const unsigned int u32SrcSec, RTCValue_t *pstDst)
{
    return pkHal_Set_SectoRTC(u32SrcSec, pstDst);
}

unsigned int pkHal_Get_UTCRTCtoLocalSec(const RTCValue_t *pstSrcUTC, const int s32LocalZone)
{
    unsigned int u32Timestamp = 0;
    unsigned int u32Data = 0;
    if (pstSrcUTC == NULL)
    {
        printf("UTC->Local Input Params Error! %s, %d \n", __FILE__, __LINE__);
        return 0;
    }    
    u32Timestamp = pkHal_Get_RTCtoSec(pstSrcUTC);
    u32Data = u32Timestamp + ((s32LocalZone / 100) * YEAR_HOUR_SECONDS + (s32LocalZone % 100 * 60 / 100) * YEAR_MIN_SECONDS);

    return u32Data;
}

unsigned int pkHal_Get_LocalRTCtoUTCSec(const RTCValue_t *pstSrcLocal, const int s32LocalZone)
{
    unsigned int u32Timestamp = 0;
    unsigned int u32Data = 0;
    if (pstSrcLocal == NULL)
    {
        printf("LOcal->UTC Input Params Error! %s, %d \n", __FILE__, __LINE__);
        return 0;
    }    
    u32Timestamp = pkHal_Get_RTCtoSec(pstSrcLocal);
    u32Data = u32Timestamp - ((s32LocalZone / 100) * YEAR_HOUR_SECONDS + (s32LocalZone % 100 * 60 / 100) * YEAR_MIN_SECONDS);

    return u32Data;    
}

int pkHal_Set_UTCtoLocal_RTC(const RTCValue_t *pstSrcUTC, const int s32LocalZone, RTCValue_t *pstDstLocal)
{
    int s32Min = 0;
    int s32Hour = 0;
    int s32Day = 0;
    int s32Mon = 0;
    int s32Year = 0;
    int bfZF = 0;

    int bfRN = 0;

    if ((pstSrcUTC == NULL)
     || (pstDstLocal == NULL))
    {
        printf("Set UTC->Local RTC Input Params Error! %s, %d \n", __FILE__, __LINE__);
        return ESP_FAIL;
    }

    if (((pstSrcUTC->year % 4 == 0) && (pstSrcUTC->year % 400 != 0))
     || (pstSrcUTC->year % 400 == 0))
    {
        bfRN = 1;
    }

    s32Min = pstSrcUTC->min + s32LocalZone % 100 * 60 / 100;
    printf("s32Min = %d \n", s32Min);
    if (s32Min < 0)
    {
        s32Min += 60;
        bfZF = -1;
    }
    else if (s32Min >= 60)
    {
        s32Min -= 60;
        bfZF = 1;
    }
    s32Hour = pstSrcUTC->hour + s32LocalZone / 100 + bfZF;
    printf("s32Hour = %d \n", s32Hour);
    bfZF = 0;
    if (s32Hour < 0)
    {
        s32Hour += 24;
        bfZF = -1;        
    }
    else if (s32Hour >= 24)
    {
        s32Min -= 24;
        bfZF = 1;
    }
    s32Day = pstSrcUTC->day + bfZF;
    printf("s32Day = %d \n", s32Day);
    bfZF = 0;
    // int bfMon = 0;
    if (s32Day <= 0)
    {
        if ((pstSrcUTC->mon == 2)
         || (pstSrcUTC->mon == 4)
         || (pstSrcUTC->mon == 6)
         || (pstSrcUTC->mon == 8)
         || (pstSrcUTC->mon == 9)
         || (pstSrcUTC->mon == 11))
        {
            s32Day = 31;
        }
        else if ((pstSrcUTC->mon == 5)
              || (pstSrcUTC->mon == 7)
              || (pstSrcUTC->mon == 10)
              || (pstSrcUTC->mon == 12))
        {
            s32Day = 30;
        }
        else if (pstSrcUTC->mon == 1)
        {
            s32Day = 31;
        }
        else if (pstSrcUTC->mon == 3)
        {
            if (bfRN == 1)
            {
                s32Day = 29;
            }
            else
            {
                s32Day = 28;
            }
        }
        bfZF = -1;  // 年減1
    }
    else if (s32Day == 29)
    {
        if ((bfRN != 1) 
         && (pstSrcUTC->mon == 2))
        {
            s32Day = 1;
            bfZF = 1;      // 平年2月
        }
    }
    else if (s32Day == 30)
    {
        if ((bfRN == 1)
         && (pstSrcUTC->mon == 2))
        {
            s32Day = 1;
            bfZF = 1;      // 潤年2月
        }
    }
    else if (s32Day == 31)
    {
        if ((pstSrcUTC->mon == 4)
         || (pstSrcUTC->mon == 6)
         || (pstSrcUTC->mon == 9)
         || (pstSrcUTC->mon == 11))
        {
            s32Day = 1;
            bfZF = 1;
        }
    }
    s32Mon = pstSrcUTC->mon + bfZF;
    printf("s32Mon = %d \n", s32Mon);
    bfZF = 0;
    if (s32Mon <= 0)
    {
        s32Mon += 12; 
        bfZF = -1;
    }
    else if (s32Mon > 12)
    {
        s32Mon -= 12;
        bfZF = 1;
    }
    s32Year = pstSrcUTC->year + bfZF;

    pstDstLocal->sec = pstSrcUTC->sec;
    pstDstLocal->min = s32Min;
    pstDstLocal->hour = s32Hour;
    pstDstLocal->day = s32Day;
    pstDstLocal->mon = s32Mon;
    pstDstLocal->year = s32Year;

    return ESP_OK;
}

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

static int i2c_master_init()
{
    // int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);

}



#undef _PK_DEVTIME_C_


