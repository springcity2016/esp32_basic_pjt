
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"
// #include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"

#include "pk_hal.h"

#define GPIO_PH_PWR         (26)
#define GPIO_PWM_BEEP_OUT   (16)
#define GPIO_WHQ_PWR        (27)
#define GPIO_CATIN          (12)

int pkhal_gpio_init(void)
{
        // 1. MCU Power 控制脚
    gpio_iomux_out(GPIO_PH_PWR, FUNC_GPIO26_GPIO26, 1);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_PH_PWR);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

        // 2. WHQ Power 控制脚
    gpio_iomux_out(GPIO_WHQ_PWR, FUNC_GPIO27_GPIO27, 1);
    // gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_WHQ_PWR);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

        // 3. 猫进入传感器控制使能
    gpio_iomux_out(GPIO_CATIN, FUNC_MTDI_GPIO12, 1);
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO12_U, FUNC_GPIO12_GPIO12);
    // gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_CATIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

     // Beep
    printf(" pwm beep config init! \n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_BEEP_OUT);
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 2000;    //frequency = 2000Hz
    pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA = 60.0%
    // pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings

    pkhal_whqio_init();     // 临时添加

    return 0;
}

#define SERIALA_TXD_GPIO            (1)
#define SERIALA_RXD_GPIO            (3)
int pkhal_serial_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, SERIALA_TXD_GPIO, SERIALA_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);     // 临时取消

    return 0;
}

#define GPIO_TP_IN      (35)
int pkhal_tpio_init(void)
{
    gpio_iomux_out(GPIO_TP_IN, FUNC_GPIO35_GPIO35, 1);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << GPIO_TP_IN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    return 0;
}
int pkhal_get_tpio_level(void)
{
    return gpio_get_level(GPIO_TP_IN);
}


int pkhal_set_mcupower_sta(int sta)
{
    if (sta == PWR_ON) {
        gpio_set_level(GPIO_PH_PWR, 1);
    } else {
        gpio_set_level(GPIO_PH_PWR, 0);
    }
    return 0; 
}

int pkhal_set_whqpower_sta(int sta)
{
    if (sta == PWR_ON) {
        gpio_set_level(GPIO_WHQ_PWR, 1);
    } else {
        gpio_set_level(GPIO_WHQ_PWR, 0);
    }
    return 0; 
}

#define GPIO_QUESHUI_IN (27)
int pkhal_whqio_init(void)
{
    gpio_iomux_out(GPIO_QUESHUI_IN, FUNC_GPIO27_GPIO27, 1);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << GPIO_QUESHUI_IN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    return 0;
}
int pkhal_get_whqio_level(void)
{
    return gpio_get_level(GPIO_QUESHUI_IN);
}


int pkhal_set_catin_IR(int sta)
{
    if (sta == ENABLE) {
        gpio_set_level(GPIO_CATIN, 1);
    } else {
        gpio_set_level(GPIO_CATIN, 0);
    }
    return 0;    
}

#define GPIO_WKPMCU         (22)
int pkhal_wkpmcuport_ctrl(int mode, int sta)
{
    gpio_iomux_out(GPIO_WKPMCU, FUNC_GPIO22_GPIO22, 1);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = mode;
    io_conf.pin_bit_mask = (1ULL << GPIO_WKPMCU);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    if (mode == GPIO_MODE_OUTPUT) {
        gpio_set_level(GPIO_WKPMCU, sta % 2);
    }
    return 0;
}

int pkhal_beep_ctrl(int sta)
{
    if (sta) {
        // gpio_set_level(GPIO_BEEP, 1);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    } else {
        // gpio_set_level(GPIO_BEEP, 0);
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    }
    return 0;
}



TimerHandle_t pkHal_Timer_Start(const char *name, TimerCallbackFunction_t stFunc, unsigned int u32Period, int bfRepeat)
{
    TimerHandle_t _stTimerHandle = xTimerCreate(name, u32Period / (1000 / CONFIG_FREERTOS_HZ), bfRepeat, NULL, stFunc);
    xTimerStart(_stTimerHandle, portMAX_DELAY);
    return _stTimerHandle;
}

int pk_HAL_Timer_Stop(TimerHandle_t stTimeHandle)
{
    xTimerStop(stTimeHandle, portMAX_DELAY);
    return (int)ESP_OK;
}
int pk_HAL_Timer_Delete(TimerHandle_t stTimeHandle)
{
    xTimerDelete(stTimeHandle, portMAX_DELAY);
    return (int)ESP_OK;
}
int pk_Hal_Timer_ChangePeriod(TimerHandle_t stTimeHandle, unsigned int u32Period)
{
    xTimerChangePeriod(stTimeHandle, u32Period, portMAX_DELAY);
    xTimerStart(stTimeHandle, portMAX_DELAY);
    return (int)ESP_OK;
}   

void xTask_mutex_lock(xSemaphoreHandle mutex)
{
    while(xSemaphoreTake(mutex,(portTickType)10/portTICK_RATE_MS)==pdFALSE);
}
int xTask_mutex_trylock(xSemaphoreHandle mutex, int time_ms)
{    
    return xSemaphoreTake(mutex,(portTickType)time_ms/portTICK_RATE_MS);
}
void xTask_mutex_unlock(xSemaphoreHandle mutex)
{
    xSemaphoreGive(mutex);
}

