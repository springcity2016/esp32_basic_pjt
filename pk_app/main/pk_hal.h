
#ifndef __PK_HAL_H_
#define __PK_HAL_H_

#include "freertos/FreeRTOS.h"
// #include "freertos/FreeRTOSConfig.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#ifndef PWR_ON
#define PWR_ON              (1)
#endif
#ifndef PWR_OFF
#define PWR_OFF             (0)
#endif

#ifndef ENABLE
#define ENABLE              (1)
#endif
#ifndef DISABLE
#define DISABLE             (0)
#endif

// typedef struct _sysevent_t
// {
//     char cmd;
//     short value;
//     char *msgptr;   //指针用于携带其他结构体
// }sysevent_t;

int pkhal_gpio_init(void);
int pkhal_serial_init(void);

int pkhal_tpio_init(void);
int pkhal_get_tpio_level(void);

int pkhal_whqio_init(void);
int pkhal_get_whqio_level(void);

int pkhal_set_mcupower_sta(int sta);
int pkhal_set_whqpower_sta(int sta);
int pkhal_set_catin_IR(int sta);

int pkhal_wkpmcuport_ctrl(int mode, int sta);

int pkhal_beep_ctrl(int sta);

TimerHandle_t pkHal_Timer_Start(const char *name, TimerCallbackFunction_t stFunc, unsigned int u32Period, int bfRepeat);
int pk_HAL_Timer_Stop(TimerHandle_t stTimeHandle);
int pk_HAL_Timer_Delete(TimerHandle_t stTimeHandle);
int pk_Hal_Timer_ChangePeriod(TimerHandle_t stTimeHandle, unsigned int u32Period);

void xTask_mutex_lock(xSemaphoreHandle mutex);
int xTask_mutex_trylock(xSemaphoreHandle mutex, int time_ms);
void xTask_mutex_unlock(xSemaphoreHandle mutex);



#endif

