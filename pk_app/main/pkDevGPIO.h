/*
 * PETKIT Network Technology (Shanghai) Co., Ltd.
 * All rights reserved.
 * 
 * pkDevKey.h
 *
 *  Created on: 2019-3-28
 *      Author: gongjw
 */

#ifndef APP_PK_DX_INCLUDE_PKDEVGPIO_H_
#define APP_PK_DX_INCLUDE_PKDEVGPIO_H_
#include "driver/gpio.h"

#define EXT_EVENT_NULL                      (0)
#define EXT_EVENT_SW_PRESS                  (1)   //按键按下
#define EXT_EVENT_SW_SHORT                  (3)   //单击
#define EXT_EVENT_SW_RELEASE                (2)   //按键松开
#define EXT_EVENT_SW_DOUBLE                 (6)   //双击
#define EXT_EVENT_SW_LONG_PRESS             (5)   //长按 5s
#define EXT_EVENT_SW_HALF_LONG_PRESS        (7)   //长按 2s
#define EXT_EVENT_PRODUCT_TEST              (8)   //产测


#define DX_WIFI_KEY_GPIO                    GPIO_NUM_0      // wifi按键
#define DX_QUESHUI_GPIO                     GPIO_NUM_14     // 缺水信号
#define DX_IR_CATIN_GPIO                    GPIO_NUM_13     // 猫进入光栅使能
#define DX_FUNC_KEY_GPIO                    GPIO_NUM_34     // 功能按键
#define DX_LOWPOW_GPIO                      GPIO_NUM_26     // 电源使能
#define DX_FACTORYTEST_GPIO                 GPIO_NUM_35     // 产测模式
#define DX_RGY_CATIN_GPIO                   GPIO_NUM_21     // 热感应传感器猫进入

#define PK_KEY_SAMP_MS                    (10)            //采样定时时间(ms)
#define PK_KEY_JITTER_MS                  (20)            //采样定时时间(ms)
#define PK_KEY_SHORT_MS                   (300)           //短按定时时间(ms)
#define PK_KEY_HALFLONG_MS                (1000)          //半长按定时时间(ms)
#define PK_KEY_LONG_MS                    (5000)          //长按定时时间(ms)

#define PK_KEY_JITTER_CNT                  (PK_KEY_JITTER_MS / PK_KEY_SAMP_MS)
#define PK_KEY_SHORT_CNT                   (PK_KEY_SHORT_MS / PK_KEY_SAMP_MS)
#define PK_KEY_HALFLONG_CNT                (PK_KEY_HALFLONG_MS / PK_KEY_SAMP_MS)
#define PK_KEY_LONG_CNT                    (PK_KEY_LONG_MS / PK_KEY_SAMP_MS)

#define pk_gpio_num_e       gpio_num_t
#define pk_gpio_config_t    gpio_config_t

typedef enum _pk_gpio_e
{
    E_LOWPOW_NODE = 0,
    E_FACTORYTEST_NODE,
    E_WIFI_KEY_NODE,
    E_FUNC_KEY_NODE,
    E_QUESHUI_NODE,
    E_IR_CATIN_NODE,
    E_RGY_CATIN_NODE,       // 热感应，猫进入

    E_GPIO_NODE_MAX
} pk_gpio_e;

typedef struct _pkhal_gpio_cfg_t
{
    pk_gpio_num_e       gpio_num;
    pk_gpio_config_t    cfg;
    gpio_isr_t          isr_handle;
} pkhal_gpio_cfg_t;

typedef enum
{
//    E_OUTCTRL_NONE = 0,
    E_OUTCTRL_MCU_LOWPOW = 0,
    E_OUTCTRL_MAX
} E_OutCtrl;

typedef enum
{
//    E_INLOOP_NONE = 0,
    E_INLOOP_FAC_TEST = 0,
    E_INLOOP_MAX
}E_InLoopCtrl;
typedef enum
{
//    E_INEXTI_NONE = 0,
    E_INEXTI_WIFI_KEY = 0,
    E_INEXTI_FUNC_KEY,
    E_INEXTI_MAX
} E_InExtiCtrl;

typedef enum _pk_gpio_edge_e
{
    E_GPIO_RISING_EDGE = 0,
    E_GPIO_FALLING_EDGE,

    E_GPIO_EDGE_MAX
} pk_gpio_edge_e;

typedef enum _pk_gpio_lvl_e
{
    E_GPIO_LOW_LVL = 0,
    E_GPIO_HIGH_LVL = 1,

    E_GPIO_LVL_MAX
} pk_gpio_lvl_e;

int pkHal_set_IO_Config(pkhal_gpio_cfg_t* p_cfg);
int pkHal_get_IO_Config(pk_gpio_num_e gpio_num, pkhal_gpio_cfg_t* p_cfg);
int pkHal_gpio_input_irq_reg(pkhal_gpio_cfg_t* p_cfg);
int pkHal_gpio_input_irq_unreg(pkhal_gpio_cfg_t* p_cfg);
void pkHal_gpio_input_irq_disable(pkhal_gpio_cfg_t* p_cfg);
void pkHal_gpio_input_irq_enable(pkhal_gpio_cfg_t* p_cfg);
int pkHal_gpio_input_irq_uninit(void);

int pkHal_Set_IOState(pk_gpio_num_e gpio_num, pk_gpio_lvl_e u08State);
pk_gpio_lvl_e pkHal_Get_IOState(pk_gpio_num_e gpio_num);

int pkhal_get_catin_ir(void);       // 获取信号

// for test
int pkhal_get_whq_queshui_sta(void);
int pkhal_get_wifikey_sta(void);
int pkhal_get_funckey_sta(void);
int pkhal_get_factkey_sta(void);

typedef void (*key_event_sendToQueue_cb)(pk_gpio_num_e pin, int eventID);
int pk_dev_init_gpio(key_event_sendToQueue_cb event_send_func);
int pk_dev_init_input_gpio(void);
int pk_GPIO_get_PIN_value_jitt(pk_gpio_num_e pin);
void pk_GPIO_key_timer_stop(void);

#endif /* APP_PK_DX_INCLUDE_PKDEVGPIO_H_ */
