/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "petkit_clog.h"

#include "pk_hal.h"
#include "user_time.h"
#include "pk_T3_driver.h"
#include "pkDevGPIO.h"
#include "pk_serial.h"
#include "product_test.h"
#include "user_config.h"
#include "pk_app.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

#ifdef PK_RELEASE_E
int petkit_log_level = LOG_NOTCIE;
#else
int petkit_log_level = LOG_INFO;
// int petkit_log_level = LOG_ALL;
#endif


static void task_test_loop(void);

volatile unsigned int gs_os_run_time = 0;

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CHIP_NAME,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    sys_init_phy_iod();
    pkHal_time_init();
    pkhal_gpio_init();
    pkhal_serial_init();
    pk_serial_init();
    pkhal_set_mcupower_sta(PWR_ON);
    pk_detect_taskinit();
    pk_app_init();

    sys_timer_start(1);

    pkLOG_INFO(" start serial taskloop! %d \n", __LINE__);
    xTaskCreate(pk_serial_taskloop, "serial_task", 4096, NULL, 2, NULL);
    
    if (pk_test_check(NULL) == 1) {
        pkLOG_INFO(" start detect taskloop! %d \n", __LINE__);
        xTaskCreate(pk_detect_taskloop, "detect_task", 4096, NULL, 2, NULL);
    } else {
        // xTaskCreate(task_test_loop, "test_task", 4096, NULL, 2, NULL);
        xTaskCreate(pk_detct_aging_test, "aging_task", 4096, NULL, 2, NULL);
    }


}

void sys_force_quittask(unsigned int mask_task)
{

}
void sys_close_wifi(unsigned char force)
{

}
void sys_close_ble(unsigned char force)
{

}


static void _pksys_timer_cb(void *arg)
{
    gs_os_run_time++;
}
static esp_timer_handle_t sys_timer;
#define SYS_TIMER_DEF_TIME_US                 (1000000)       // 单位us
int sys_timer_start(int force)
{
    int _ret = 0;
    if (force) {
        _ret = sys_timer_stop();
        if (_ret) { return _ret; }
        esp_timer_create_args_t _timer_conf = {
            .callback = _pksys_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "sys_timer"
        };
        _ret = esp_timer_create(&_timer_conf, &sys_timer);
        if (_ret) { return _ret; }
        _ret = esp_timer_start_periodic(sys_timer, SYS_TIMER_DEF_TIME_US);
    }
    return _ret;
}
int sys_timer_stop(void)
{
    int _ret = 0;
    if (sys_timer) {
        _ret = esp_timer_stop(sys_timer);
        if (_ret) { return _ret; }
        _ret = esp_timer_delete(sys_timer);
    }
    return _ret;
}
unsigned int sys_get_os_runtime_sec(void)
{
    return gs_os_run_time;
}
void sys_set_os_runtime_sec(unsigned int x_sec)
{
    gs_os_run_time = x_sec;
}

static void _print_timer_Info(ctimer_t time);

static void task_test_loop(void)
{
    int cnt = 0;
    int _level = 0;
    ctimer_t _ctime;
    memset(&_ctime, 0, sizeof(ctimer_t));

    cfgdata_t _config_dat = {
        .io_qudou_time = 10,
        .adc_delay_time = 10,
        .sensor_rpt_delay_time = 10,
        .hb_rpt_delay_time = 30
    };

    while(1) {
        cnt++;
        pkhal_set_mcupower_sta(PWR_ON);
        pkhal_set_catin_IR(ENABLE);
        // printf(" set mcu power pin sta: Power ON! cnt: %d \n", cnt);

        if (cnt == 5) {
            printf(" esp->mcu set normal workmode ! %d \n", __LINE__);
            pkmcu_get_version(0);
            pkmcu_get_errcode(0);
            // pkmcu_reset_mcu(0);
            // pkmcu_set_mcuworkmode(E_LOWPWR_MOE, 0);
            pkmcu_get_exphydata(E_EXPHY_WEIGHT, E_INFO_NOBLK, 1, 0);
            pkmcu_set_motrun_cfgdata(E_SANDMOT_POS_T, E_SANDMOT_POS_START, E_CW, 200, 0, 400, 0xFFFF, 0);
            pkmcu_set_mcuworkmode(E_NORMAL_MODE, 0);
            // pkmcu_set_configdata((const unsigned char *)&_config_dat, sizeof(cfgdata_t), 0);
            // pkLOG_INFO(" mot run start! %d \n", __LINE__);
            // pkmcu_set_motrun_cfgdata(E_SANDMOT_STEP_T, E_CCW, 200, 100, 400, 0xFFFF, 0);
        }

        // if (cnt == 1) {
        //     ctime_set_timer_func(2019, 11, 7, 10, 36, 30);
        // } else if (cnt == 10) {
        //     ctime_read_timer_func(&_ctime, 800);
        //     _print_timer_Info(_ctime);
        // }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void _print_timer_Info(ctimer_t time)
{
    printf("**********************************\n");
    printf(" %d - %d - %d, %d:%d:%d \n", time.year, time.mon, time.day, time.hour, time.min, time.sec);
    printf("**********************************\n");
}
