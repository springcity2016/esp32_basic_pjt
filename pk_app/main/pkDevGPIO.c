
/*
 * PETKIT Network Technology (Shanghai) Co., Ltd.
 * All rights reserved.
 * 
 * pkDevKey.c
 *
 *  Created on: 2019-3-11
 *      Author: chunchen.cao
 */

#define _PK_DEVKEY_C_

#include <string.h>

#include "petkit_clog.h"
#include "pkDevGPIO.h"
#include "esp_log.h"
#include "pk_hal.h"

#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"

#define PK_INPUT_KEY_NUM                    (2)
#define PK_COMB_KEY_NUM                     (2)

//#define PK_KEY_SAMP_MS                      (5)             //采样定时时间(ms)
//#define PK_KEY_JITTER_MS                    (15)            //采样定时时间(ms)
//#define PK_KEY_SHORT_MS                     (800)           //短按定时时间(ms)
//#define PK_KEY_HALFLONG_MS                  (1000)          //半长按定时时间(ms)
//#define PK_KEY_LONG_MS                      (5000)          //长按定时时间(ms)
//
//#define PK_KEY_JITTER_CNT                   (PK_KEY_JITTER_MS / PK_KEY_SAMP_MS)
//#define PK_KEY_SHORT_CNT                    (PK_KEY_SHORT_MS / PK_KEY_SAMP_MS)
//#define PK_KEY_HALFLONG_CNT                 (PK_KEY_HALFLONG_MS / PK_KEY_SAMP_MS)
//#define PK_KEY_LONG_CNT                     (PK_KEY_LONG_MS / PK_KEY_SAMP_MS)

//static const char *TAG = "GPIO";

key_event_sendToQueue_cb key_cb_func;
/************************************************************************************************/


typedef enum _pk_key_sta_e
{
    E_KEY_IDLE = 0,                 //结束或完成状态
    E_KEY_PRESS,                    //按压状态
    E_KEY_LONG_PRESS,               //长时间按压状态
    E_KEY_WAIT                      //等待再次按压状态
} pk_key_sta_e;

typedef enum _pk_key_timer_type_e
{
    E_KEY_TIMER_NULL = 0,
    E_KEY_TIMER_JITTER,               //去抖定时器
    E_KEY_TIMER_SHORT,                //短按定时器
    E_KEY_TIMER_HALFLONG,             //半长按定时器
    E_KEY_TIMER_LONG,                 //长按定时器
    E_KEY_TIMER_MAX
} pk_key_timer_type_e;

typedef struct _pk_key_sta_t
{
    pk_gpio_num_e       pin;                        //引脚名
    pk_key_sta_e        pin_sta;                    //引脚所处状态
    pk_gpio_edge_e      pin_edge;                   //中断边沿

    pk_gpio_edge_e      press_edge;                 //按下时触发边沿
    int                 press_lvl;                  //按下时维持电平
    pk_gpio_edge_e      release_edge;               //松开时触发边沿
    int                 release_lvl;                //松开时维持电平

    int                 timer_type;
    int                 key_timer_cnt[E_KEY_TIMER_MAX];
} pk_key_sta_t;

typedef struct _pk_key_timer_t
{
    int                 is_start;
    int                 cnt;
    int                 period_ms;
    TimerHandle_t       handle;
} pk_key_timer_t;

typedef struct _pk_comb_key_t
{
    pk_gpio_num_e       main_key;
    pk_gpio_num_e       sub_key;
} pk_comb_key_t;

/************************************************************************************************/
static void gpio_isr_handler(void *arg);
/************************************************************************************************/
static pkhal_gpio_cfg_t ga_all_gpio_node[E_GPIO_NODE_MAX] = {
        {
            DX_LOWPOW_GPIO,
            {
//                .pin_bit_mask = (1ULL << DX_LOWPOW_GPIO),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = 1,
                .pull_down_en = 0,
                .intr_type = GPIO_INTR_DISABLE,
            },
            NULL,
        },

        {
            DX_FACTORYTEST_GPIO,
            {
//                .pin_bit_mask = (1ULL << DX_FACTORYTEST_GPIO),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = 1,
                .pull_down_en = 0,
                .intr_type = GPIO_INTR_DISABLE,
            },
            NULL,
        },

        {
            DX_WIFI_KEY_GPIO,
            {
//                .pin_bit_mask = (1ULL << DX_WIFI_KEY_GPIO),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = 1,
                .pull_down_en = 0,
                .intr_type = GPIO_INTR_ANYEDGE,
            },
            gpio_isr_handler,
        },

        {
            DX_FUNC_KEY_GPIO,
            {
//                .pin_bit_mask = (1ULL << DX_FUNC_KEY_GPIO),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = 1,
                .pull_down_en = 0,
                .intr_type = GPIO_INTR_ANYEDGE,
            },
            gpio_isr_handler,
        },

        {
            DX_QUESHUI_GPIO,
            {
//                .pin_bit_mask = (1ULL << DX_FACTORYTEST_GPIO),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = 1,
                .pull_down_en = 0,
                .intr_type = GPIO_INTR_DISABLE,
            },
            NULL,
        },

        {
            DX_IR_CATIN_GPIO,
            {
//                .pin_bit_mask = (1ULL << DX_FACTORYTEST_GPIO),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = 1,
                .pull_down_en = 0,
                .intr_type = GPIO_INTR_DISABLE,
            },
            NULL,
        },

        {
            DX_RGY_CATIN_GPIO,
            {
//                .pin_bit_mask = (1ULL << DX_FACTORYTEST_GPIO),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = 1,
                .pull_down_en = 0,
                .intr_type = GPIO_INTR_DISABLE,
            },
            NULL,
        },        
};

/************************************************************************************************/
pk_key_sta_t g_key_sta[PK_INPUT_KEY_NUM] = {
        {
            .pin = DX_WIFI_KEY_GPIO,
            .pin_sta = E_KEY_IDLE,
            .release_edge = E_GPIO_RISING_EDGE,
            .release_lvl = E_GPIO_HIGH_LVL,
            .press_edge = E_GPIO_FALLING_EDGE,
            .press_lvl = E_GPIO_LOW_LVL,
        },
        {
            .pin = DX_FUNC_KEY_GPIO,
            .pin_sta = E_KEY_IDLE,
            .release_edge = E_GPIO_RISING_EDGE,
            .release_lvl = E_GPIO_HIGH_LVL,
            .press_edge = E_GPIO_FALLING_EDGE,
            .press_lvl = E_GPIO_LOW_LVL,
        }
};

pk_comb_key_t g_comb_key_tab[PK_COMB_KEY_NUM] = {
        {
            .main_key = DX_WIFI_KEY_GPIO,
            .sub_key = DX_FUNC_KEY_GPIO
        },
        {
            .main_key = DX_FUNC_KEY_GPIO,
            .sub_key = DX_WIFI_KEY_GPIO
        }
};

pk_key_timer_t g_key_timer = {
        .period_ms = PK_KEY_SAMP_MS,
};
/************************************************************************************************/
int pkHal_set_IO_Config(pkhal_gpio_cfg_t* p_cfg)
{
    if ((NULL == p_cfg) || (p_cfg->gpio_num >= GPIO_NUM_MAX) || (p_cfg->gpio_num < GPIO_NUM_0)) {
        return ESP_FAIL;
    }

    p_cfg->cfg.pin_bit_mask = (1ULL << p_cfg->gpio_num),

    gpio_config((gpio_config_t*)&(p_cfg->cfg));

    if (NULL != p_cfg->isr_handle) {
        pkHal_gpio_input_irq_reg(p_cfg);
    }

    if ((GPIO_INTR_DISABLE != p_cfg->cfg.intr_type) && (NULL != p_cfg->isr_handle)) {
        pkLOG_INFO("irq_enable :%d!!!\n", p_cfg->gpio_num);
        pkHal_gpio_input_irq_enable(p_cfg);
    }

    return 0;
}

int pkHal_get_IO_Config(gpio_num_t gpio_num, pkhal_gpio_cfg_t* p_cfg)
{
    if ((NULL == p_cfg) || (gpio_num >= GPIO_NUM_MAX) || (gpio_num < GPIO_NUM_0)) {
        return ESP_FAIL;
    }

    p_cfg->gpio_num = gpio_num;


    return 0;
}

int pkHal_gpio_input_irq_reg(pkhal_gpio_cfg_t* p_cfg)
{
    if ((NULL == p_cfg) || (NULL == p_cfg->isr_handle) || (p_cfg->gpio_num >= GPIO_NUM_MAX) || (p_cfg->gpio_num < GPIO_NUM_0)) {
        return ESP_FAIL;
    }

    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(p_cfg->gpio_num, p_cfg->isr_handle, (void *) p_cfg->gpio_num);

    return ESP_OK;
}

int pkHal_gpio_input_irq_unreg(pkhal_gpio_cfg_t* p_cfg)
{
    if ((NULL == p_cfg) || (p_cfg->gpio_num >= GPIO_NUM_MAX) || (p_cfg->gpio_num < GPIO_NUM_0)) {
        return ESP_FAIL;
    }

    gpio_isr_handler_remove(p_cfg->gpio_num);

    return ESP_OK;
}

int pkHal_gpio_input_irq_uninit(void)
{
    gpio_uninstall_isr_service();

    return ESP_OK;
}

void pkHal_gpio_input_irq_disable(pkhal_gpio_cfg_t* p_cfg)
{
    // portENTER_CRITICAL();
    // _xt_isr_mask(1 << ETS_GPIO_INUM);
    ets_isr_mask(1 << ETS_DPORT_INUM);
    // portEXIT_CRITICAL();

}

void pkHal_gpio_input_irq_enable(pkhal_gpio_cfg_t* p_cfg)
{
    // portENTER_CRITICAL();
    ets_isr_mask(1 << ETS_DPORT_INUM);
    // _xt_isr_unmask(1 << ETS_GPIO_INUM);
    // portEXIT_CRITICAL();

}

int pkHal_Set_IOState(pk_gpio_num_e gpio_num, pk_gpio_lvl_e u08State)
{
    if ((gpio_num >= GPIO_NUM_MAX)
            || (gpio_num < GPIO_NUM_0)) {
        return ESP_FAIL;
    }
    return gpio_set_level(gpio_num, (uint32_t)u08State);
}

pk_gpio_lvl_e pkHal_Get_IOState(pk_gpio_num_e gpio_num)
{
    if ((gpio_num >= GPIO_NUM_MAX)
            || (gpio_num < GPIO_NUM_0)) {
        return ESP_FAIL;
    }
    return gpio_get_level(gpio_num);
}

int pkhal_get_catin_ir(void)       // 获取信号
{
    return gpio_get_level(DX_IR_CATIN_GPIO);
}

// for test
int pkhal_get_whq_queshui_sta(void)
{
    return gpio_get_level(DX_QUESHUI_GPIO);
}
int pkhal_get_wifikey_sta(void)
{
    return gpio_get_level(DX_WIFI_KEY_GPIO);
}
int pkhal_get_funckey_sta(void)
{
    return gpio_get_level(DX_FUNC_KEY_GPIO);
}
int pkhal_get_factkey_sta(void)
{
    return gpio_get_level(DX_FACTORYTEST_GPIO);
}

void key_timer_mark(int *state, pk_key_timer_type_e mask)
{
    //pkLOG_INFO("1state:%d, %p\n", *state, state);
    int local_mask = *state;
    local_mask |= (1 << mask);
    *state = local_mask;
    //pkLOG_INFO("2state:%d\n", *state);
}

void key_timer_unmark(int *state, pk_key_timer_type_e mask)
{
    int local_mask = *state;
    local_mask &= (~(1 << mask));
    *state = local_mask;
}

void key_timer_unmark_all(int *state)
{
    *state = E_KEY_TIMER_NULL;
}

int key_timer_get_mark(int state, pk_key_timer_type_e mask)
{
    return (state & (1 << mask)) ? pdTRUE : pdFALSE;
}

static pk_key_sta_t *gpio_get_key_sta(pk_gpio_num_e pin)
{
    int i = 0;
    for (i = 0; i < PK_INPUT_KEY_NUM; i++) {
        if (g_key_sta[i].pin == pin) {
            return &(g_key_sta[i]);
        }
    }

    return NULL;
}

static int gpio_check_comb_key(pk_key_sta_t *p_key_cnt)
{
    int i = 0;
    pk_key_sta_t *p_key = NULL;

    for (i = 0; i < PK_COMB_KEY_NUM; i++) {
        if (g_comb_key_tab[i].main_key != p_key_cnt->pin) {
            continue;
        }

        p_key = gpio_get_key_sta(g_comb_key_tab[i].sub_key);
        if (NULL != p_key) {
            if ((E_KEY_PRESS == p_key->pin_sta)
                    || (E_KEY_LONG_PRESS == p_key->pin_sta)) {
                p_key->pin_sta = E_KEY_IDLE;
                return pdTRUE;
            }
        }
    }

    return pdFALSE;
}

static void key_press_proc(pk_key_sta_t *p_key_cnt)
{
//    pkLOG_INFO("Pin:%d-press\n", p_key_cnt->pin);
//    pkLOG_INFO("Pin:%d-pin_edge:%d, pe:%d, re:%d, lvl:%d, sta:%d\n",
//            p_key_cnt->pin, p_key_cnt->pin_edge, p_key_cnt->press_edge, p_key_cnt->release_edge,
//            pkHal_Get_IOState(p_key_cnt->pin), p_key_cnt->pin_sta);

//    key_event_sendToQueue(p_key_cnt->pin, EXT_EVENT_SW_PRESS);
//    if (E_LOWPOW_SLEEP == g_low_pow_sta.low_pow_sta) {
//        g_low_pow_sta.wakeup_mode = E_WAKEUP_BTN_MODE;
//    }
    key_cb_func(p_key_cnt->pin, EXT_EVENT_SW_PRESS);

    if (E_KEY_WAIT == p_key_cnt->pin_sta) {//按键被第二次(与第一次按下时间不超过300ms)按下
        p_key_cnt->pin_sta = E_KEY_IDLE;
        p_key_cnt->key_timer_cnt[E_KEY_TIMER_SHORT] = 0;
        //如果此时该按键的组合键处于被按下或长按状态,输出产测事件
        if (gpio_check_comb_key(p_key_cnt)) {
//            pkLOG_INFO("Product_Event!!!\n");
            key_cb_func(p_key_cnt->pin, EXT_EVENT_PRODUCT_TEST);
        } else {
//            pkLOG_INFO("Double Click Event!!!\n");
            key_cb_func(p_key_cnt->pin, EXT_EVENT_SW_DOUBLE);
        }
    } else {  //按键第一次被按下
        p_key_cnt->pin_sta = E_KEY_PRESS;
        p_key_cnt->key_timer_cnt[E_KEY_TIMER_HALFLONG] = 0;
        key_timer_mark(&(p_key_cnt->timer_type), E_KEY_TIMER_HALFLONG);
        p_key_cnt->key_timer_cnt[E_KEY_TIMER_LONG] = 0;
        key_timer_mark(&(p_key_cnt->timer_type), E_KEY_TIMER_LONG);
    }
}

static void key_release_proc(pk_key_sta_t *p_key_cnt)
{
    //按键松开
//    pkLOG_INFO("Pin:%d-release\n", p_key_cnt->pin);
//    pkLOG_INFO("Pin:%d-pin_edge:%d, pe:%d, re:%d, lvl:%d, sta:%d\n",
//            p_key_cnt->pin, p_key_cnt->pin_edge, p_key_cnt->press_edge, p_key_cnt->release_edge,
//            pkHal_Get_IOState(p_key_cnt->pin), p_key_cnt->pin_sta);
    key_cb_func(p_key_cnt->pin, EXT_EVENT_SW_RELEASE);

    if (E_KEY_PRESS == p_key_cnt->pin_sta) {                    //如果按键在按压状态下被释放
        p_key_cnt->pin_sta = E_KEY_WAIT;
        key_timer_mark(&(p_key_cnt->timer_type), E_KEY_TIMER_SHORT); //等待300ms
    } else {
        p_key_cnt->pin_sta = E_KEY_IDLE;
    }
}

static void key_timer_jitter_proc(pk_key_sta_t *p_key_cnt)
{
//    pkLOG_INFO("22Pin:%d-pin_edge:%d, lvl:%d, type:%d, ss:%d\n",
//            p_key_cnt->pin, p_key_cnt->pin_edge,
//            pkHal_Get_IOState(p_key_cnt->pin), p_key_cnt->timer_type, p_key_cnt->key_timer_cnt[E_KEY_TIMER_SHORT]);
//    pkLOG_INFO("j:%d\n",p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER]);
    if (p_key_cnt->press_edge == p_key_cnt->pin_edge) {
        if (p_key_cnt->press_lvl == pkHal_Get_IOState(p_key_cnt->pin)) {
            p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER]++;
        } else {
            key_timer_unmark(&(p_key_cnt->timer_type), E_KEY_TIMER_JITTER);
            p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER] = 0;
            p_key_cnt->pin_sta = E_KEY_IDLE;
            return;
        }

        if (p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER] >= PK_KEY_JITTER_CNT) {
            key_timer_unmark(&(p_key_cnt->timer_type), E_KEY_TIMER_JITTER);
            p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER] = 0;
            key_press_proc(p_key_cnt);
        }
    } else if (p_key_cnt->release_edge == p_key_cnt->pin_edge) {
        if (p_key_cnt->release_lvl == pkHal_Get_IOState(p_key_cnt->pin)) {
            p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER]++;
        } else {
            key_timer_unmark(&(p_key_cnt->timer_type), E_KEY_TIMER_JITTER);
            p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER] = 0;
            p_key_cnt->pin_sta = E_KEY_IDLE;
            return;
        }

        if (p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER] >= PK_KEY_JITTER_CNT) {
            key_timer_unmark(&(p_key_cnt->timer_type), E_KEY_TIMER_JITTER);
            p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER] = 0;
            key_release_proc(p_key_cnt);
        }
    } else {
        key_timer_unmark(&(p_key_cnt->timer_type), E_KEY_TIMER_JITTER);
        p_key_cnt->key_timer_cnt[E_KEY_TIMER_JITTER] = 0;
        p_key_cnt->pin_sta = E_KEY_IDLE;
    }
}

/**
 * 如果按键在第一次按下-松开的300ms后,依然在等待再次按压,则为单击事件
 * @param argv 按键结构体指针
 */
static void key_timer_short_proc(pk_key_sta_t *p_key_cnt)
{
    p_key_cnt->key_timer_cnt[E_KEY_TIMER_SHORT]++;

    if (p_key_cnt->key_timer_cnt[E_KEY_TIMER_SHORT] >= PK_KEY_SHORT_CNT) {
        key_timer_unmark(&(p_key_cnt->timer_type), E_KEY_TIMER_SHORT);
        if (E_KEY_WAIT == p_key_cnt->pin_sta) {
//            pkLOG_INFO("Pin:%d short Event\n", p_key_cnt->pin);
            key_cb_func(p_key_cnt->pin, EXT_EVENT_SW_SHORT);
        }
        p_key_cnt->pin_sta = E_KEY_IDLE;
    }

}
/**
 * 如果按键依然在按压状态,则输出长按2s事件,并改状态为长按状态
 * @param argv 按键结构体指针
 */
static void key_timer_halflong_proc(pk_key_sta_t *p_key_cnt)
{
    p_key_cnt->key_timer_cnt[E_KEY_TIMER_HALFLONG]++;
    if (p_key_cnt->key_timer_cnt[E_KEY_TIMER_HALFLONG] >= PK_KEY_HALFLONG_CNT) {
        key_timer_unmark(&(p_key_cnt->timer_type), E_KEY_TIMER_HALFLONG);
        if (E_KEY_PRESS == p_key_cnt->pin_sta) {
            pkLOG_INFO("Pin:%d halfLong Press\n", p_key_cnt->pin);
            key_cb_func(p_key_cnt->pin, EXT_EVENT_SW_HALF_LONG_PRESS);
            p_key_cnt->pin_sta = E_KEY_LONG_PRESS;
        }
    }
}
/**
 * 如果按键在长按状态,则输出长按5s事件
 * @param argv 按键结构体指针
 */
static void key_timer_long_proc(pk_key_sta_t *p_key_cnt)
{
    p_key_cnt->key_timer_cnt[E_KEY_TIMER_LONG]++;

    if (p_key_cnt->key_timer_cnt[E_KEY_TIMER_LONG] >= PK_KEY_LONG_CNT) {
        key_timer_unmark(&(p_key_cnt->timer_type), E_KEY_TIMER_LONG);
        if (E_KEY_LONG_PRESS == p_key_cnt->pin_sta) {
//            pkLOG_INFO("Pin:%d Long Press\n", p_key_cnt->pin);
            key_cb_func(p_key_cnt->pin, EXT_EVENT_SW_LONG_PRESS);
        }
    }
}

static void key_timer_proc(pk_key_sta_t *p_key_cnt, pk_key_timer_type_e type)
{
    switch (type) {
        case E_KEY_TIMER_JITTER:
            key_timer_jitter_proc(p_key_cnt);
            break;
        case E_KEY_TIMER_SHORT:
            key_timer_short_proc(p_key_cnt);
            break;
        case E_KEY_TIMER_HALFLONG:
            key_timer_halflong_proc(p_key_cnt);
            break;
        case E_KEY_TIMER_LONG:
            key_timer_long_proc(p_key_cnt);
            break;
        default:
            break;
    }
}

static void key_TimerCallback(TimerHandle_t xTimer)
{
    int i = 0;
    bool have_timer = 0;
    unsigned char u08Cnt = 0;

    g_key_timer.cnt++;

//    pkLOG_DEBUG("key_TimerCallback :%d\n", g_key_timer.cnt);
    for (u08Cnt = 0; u08Cnt < E_INEXTI_MAX; u08Cnt++) {
        for (i = 1; i < E_KEY_TIMER_MAX; i++) {
            if (key_timer_get_mark(g_key_sta[u08Cnt].timer_type, i)) {
                key_timer_proc(&(g_key_sta[u08Cnt]), i);
                have_timer = 1;
            }
        }
    }

    if (!have_timer) {
        g_key_timer.is_start = 0;
        g_key_timer.cnt = 0;

        pk_HAL_Timer_Stop(xTimer);
        pk_HAL_Timer_Delete(xTimer);
    }
}

static void pk_GPIO_key_timer_start(unsigned char key)
{
    if (0 == g_key_timer.is_start) {
        memset(g_key_sta[key].key_timer_cnt, 0, sizeof(g_key_sta[key].key_timer_cnt));
        g_key_timer.cnt = 0;
        g_key_timer.is_start = 1;
        g_key_timer.handle = pkHal_Timer_Start("key_timer", key_TimerCallback, g_key_timer.period_ms, 1);
    }
}

void pk_GPIO_key_timer_stop(void)
{
    int i = 0;
    unsigned char u08Cnt = 0;

    g_key_timer.is_start = 0;
    g_key_timer.cnt = 0;
    for (u08Cnt = 0; u08Cnt < E_INEXTI_MAX; u08Cnt++) {
        for (i = 1; i < E_KEY_TIMER_MAX; i++) {
            key_timer_unmark_all(&(g_key_sta[u08Cnt].timer_type));
        }
    }
    if (g_key_timer.handle) {
        pk_HAL_Timer_Stop(g_key_timer.handle);
        pk_HAL_Timer_Delete(g_key_timer.handle);
        g_key_timer.handle = 0;
    }
}

static void gpio_isr_handler(void *arg)
{
    unsigned char u08Cnt = 0;
    uint32_t gpio_num = (uint32_t) arg;

    pkHal_gpio_input_irq_disable(NULL);

    for (u08Cnt = 0; u08Cnt < PK_INPUT_KEY_NUM; u08Cnt++) {
        if (g_key_sta[u08Cnt].pin == gpio_num) {
            if (E_GPIO_LOW_LVL == pkHal_Get_IOState(gpio_num)) {
                g_key_sta[u08Cnt].pin_edge = E_GPIO_FALLING_EDGE;
            } else {
                g_key_sta[u08Cnt].pin_edge = E_GPIO_RISING_EDGE;
            }

            g_key_sta[u08Cnt].key_timer_cnt[E_KEY_TIMER_JITTER] = 0;
            key_timer_mark(&(g_key_sta[u08Cnt].timer_type), E_KEY_TIMER_JITTER);
            pk_GPIO_key_timer_start(u08Cnt);
            break;
        }
    }

    pkHal_gpio_input_irq_enable(NULL);
}

int pk_dev_init_input_gpio(void)
{
//    pkLOG_INFO("input_gpio :%d!!!\n", ga_all_gpio_node[E_WIFI_KEY_NODE].gpio_num);
//    pkLOG_INFO("input_gpio typ :%d!!!\n", ga_all_gpio_node[E_WIFI_KEY_NODE].cfg.intr_type);
//
//    pkLOG_INFO("input_gpio :%d!!!\n", ga_all_gpio_node[E_FUNC_KEY_NODE].gpio_num);
//    pkLOG_INFO("input_gpio typ :%d!!!\n", ga_all_gpio_node[E_FUNC_KEY_NODE].cfg.intr_type);

    pkHal_set_IO_Config(&(ga_all_gpio_node[E_FACTORYTEST_NODE]));
    pkHal_set_IO_Config(&(ga_all_gpio_node[E_WIFI_KEY_NODE]));
    pkHal_set_IO_Config(&(ga_all_gpio_node[E_FUNC_KEY_NODE]));
    pkHal_set_IO_Config(&(ga_all_gpio_node[E_QUESHUI_NODE]));
    pkHal_set_IO_Config(&(ga_all_gpio_node[E_IR_CATIN_NODE]));
    pkHal_set_IO_Config(&(ga_all_gpio_node[E_RGY_CATIN_NODE]));

    return 0;
}

static int pk_dev_init_output_gpio(void)
{
    pkHal_set_IO_Config(&(ga_all_gpio_node[E_LOWPOW_NODE]));

    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);

    // gpio_config_t io_conf;
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_OUTPUT_OD;
    // io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // io_conf.pull_down_en = 0;
    // io_conf.pull_up_en = 1;
    // io_conf.def_dir = 1;
    // gpio_config(&io_conf);
    // // pkMCU_SetPowerSta(E_POWER_STA_ON);

    // vTaskDelay((portTickType)100/portTICK_RATE_MS);
    // io_conf.def_dir = 0;
    // gpio_config(&io_conf);
    // // pkMCU_SetPowerSta(E_POWER_STA_ON);

    return 0;
}

int pk_dev_init_gpio(key_event_sendToQueue_cb event_send_func)
{
    pkLOG_INFO("init_gpio!!!\n");

    key_cb_func = event_send_func;
    pk_dev_init_input_gpio();
    pk_dev_init_output_gpio();

    return 0;
}

int pk_GPIO_get_PIN_value_jitt(pk_gpio_num_e pin)
{
    int val = 0;
    int i = 0;
    val = pkHal_Get_IOState(pin);

    for (i = 0; i < 4; i++) {
        vTaskDelay(20 / portTICK_RATE_MS);
        if (val == pkHal_Get_IOState(pin)) {
            continue;
        } else {
            return -1;
        }
    }

    return val;
}

#if 0
typedef enum
{
    E_KEYFUNC_NONE = 0,
    E_KEYFUNC_CC,
    E_KEYFUNC_WIFI,
    E_KEYFUNC_RS,
    E_KEYFUNC_MAX
}eKeyFuncType;

#define KEY_FUNC_NULL                       (-1)            // 空按键
#define KEY_FUNC_CC                         (GPIO_NUM_0)      // 产品测试按键
#define KEY_FUNC_WIFI                       (GPIO_NUM_1)      // WIFI功能按键
#define KEY_FUNC_RS                         (GPIO_NUM_2)      // 电机启动停止控制按键

#define EXT_SCAN_OFF                        (0) //关闭扫描
#define EXT_SCAN_ON                         (1) //开启扫描

#define EXT_KEY_SCAN_HFLEN                  (3)   //按键扫描长度10mS unit
#define EXT_KEY_SCAN_LEN                    (50)  //按键扫描长度10mS unit
#define EXT_KEY_SCAN_LONG                   (300) //按键扫描长度10mS unit

#define KEY_SAMP_TIME_MS                    (20)            //采样定时时间
#define KEY_SHORT_TIME_MS                   (300)           //短按定时时间
#define KEY_HALFLONG_TIME_MS                (1000)          //半长按定时时间
#define KEY_LONG_TIME_MS                    (5000)          //长按定时时间


static KeyCtrl_t mstDevkeyValue[E_KEYFUNC_MAX] = {
        {
            .s16PinNum = KEY_FUNC_NULL,
            .ePinSta = E_KEY_IDLE
        },
        {
            .s16PinNum = KEY_FUNC_CC,
            .ePinSta = E_KEY_IDLE            
        },
        {
            .s16PinNum = KEY_FUNC_WIFI,
            .ePinSta = E_KEY_IDLE
        },
        {
            .s16PinNum = KEY_FUNC_RS,
            .ePinSta = E_KEY_IDLE            
        }
    };


#define HALSYS_EVENT_MAX            50
xQueueHandle xQueueHalSystemEvent;

static void btn_press_proc(KeyCtrl_t *stKey);
static void btn_release_proc(KeyCtrl_t *stKey);
static void key_timer_jitt_cb(void *argv);
static void key_timer_short_cb(void *argv);
static void key_timer_halflong_cb(void *argv);
static void key_timer_long_cb(void *argv);
static void Key_Func(eKeyTimerType eTimer, KeyCtrl_t *stKey);

static void Key_isr_handler(void *arg);            // 按键中断处理函数

static void gpio_key_gpio_PinFuncSelect(void);

int pkHal_Key_ConfigInit(void)
{
    unsigned char u08Cnt = 0x00;
    gpio_config_t _stIOConfig = { 0, 0, 0, 0, 0 };

    // 1. 管脚配置  
    gpio_key_gpio_PinFuncSelect();

    for (u08Cnt = 0x00; u08Cnt < E_KEYFUNC_MAX; u08Cnt++)
    {
        if (mstDevkeyValue[u08Cnt].s16PinNum == KEY_FUNC_NULL)
        {
            continue;
        }
        _stIOConfig.pin_bit_mask = mstDevkeyValue[u08Cnt].s16PinNum;
        _stIOConfig.mode = GPIO_MODE_DEF_INPUT;
        _stIOConfig.pull_up_en = 1;
        _stIOConfig.intr_type = GPIO_INTR_ANYEDGE;      // both rising and falling edge
        gpio_config(&_stIOConfig);
    }
    gpio_install_isr_service(0);        //install gpio isr service
    for (u08Cnt = 0x00; u08Cnt < E_KEYFUNC_MAX; u08Cnt++)
    {
        if (mstDevkeyValue[u08Cnt].s16PinNum == KEY_FUNC_NULL)
        {
            continue;
        }
        gpio_isr_handler_add(u08Cnt, Key_isr_handler, (void *)&u08Cnt);
    }

    // 2. Create Task Communication QueueHandle
    xQueueHalSystemEvent = xQueueCreate(HALSYS_EVENT_MAX, sizeof(SysEvent_t));
    if (xQueueHalSystemEvent == NULL)
    {
        printf("pkHal_Key Config Init - Event Queue Error! \n");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void Key_Func(eKeyTimerType eTimer, KeyCtrl_t *stKey)
{
    if (stKey == NULL)
    {   
        //printf("Key_Func stKeyValue = NULL!!\n");
        return;
    }

    esp_timer_create_args_t stTimeArgs;
    memset(&stTimeArgs, 0, sizeof(esp_timer_create_args_t));

    esp_timer_stop(stKey->astKeyTimer[eTimer]);
    esp_timer_delete(stKey->astKeyTimer[eTimer]);

    switch(eTimer)
    {
    case E_KEY_TIMER_JITTER:
        {
            stTimeArgs.name = "Timer Jitter";
            stTimeArgs.arg = stKey;
            stTimeArgs.callback = key_timer_jitt_cb;
            esp_timer_create(&stTimeArgs, &(stKey->astKeyTimer[eTimer]));
            //pkHal_Timer_Create(const char *name, void (*func)(void *), void *user_data, esp_timer_handle_t *pstOutHandle);
            esp_timer_start_once(stKey->astKeyTimer[eTimer], KEY_SAMP_TIME_MS);
        }    
        break;
    case E_KEY_TIMER_SHORT:
        {
            stTimeArgs.name = "Timer Short";
            stTimeArgs.arg = stKey;
            stTimeArgs.callback = key_timer_short_cb;
            esp_timer_create(&stTimeArgs, &(stKey->astKeyTimer[eTimer]));
            esp_timer_start_once(stKey->astKeyTimer[eTimer], KEY_SHORT_TIME_MS);            
        }
        break;
    case E_KEY_TIMER_HALFLONG:
        {
            stTimeArgs.name = "Timer HalfLong";
            stTimeArgs.arg = stKey;
            stTimeArgs.callback = key_timer_halflong_cb;
            esp_timer_create(&stTimeArgs, &(stKey->astKeyTimer[eTimer]));
            esp_timer_start_once(stKey->astKeyTimer[eTimer], KEY_HALFLONG_TIME_MS);             
        }
        break;
    case E_KEY_TIMER_LONG:
        {
            stTimeArgs.name = "Timer Long";
            stTimeArgs.arg = stKey;
            stTimeArgs.callback = key_timer_long_cb;
            esp_timer_create(&stTimeArgs, &(stKey->astKeyTimer[eTimer]));
            esp_timer_start_once(stKey->astKeyTimer[eTimer], KEY_LONG_TIME_MS);               
        }
        break;
    default: break;
    }
}

static void btn_press_proc(KeyCtrl_t *stKey)
{
    sys_xQueSendToBackFromISR(xQueueHalSystemEvent, EXT_EVENT_SW_PRESS, stKey->s16PinNum, NULL);
    if (stKey->ePinSta == E_KEY_WAIT)   
    {
        //按键被第二次(与第一次按下时间不超过300ms)按下
        sys_xQueSendToBackFromISR(xQueueHalSystemEvent, EXT_EVENT_SW_DOUBLE, stKey->s16PinNum, NULL);
    }
    else
    {
        stKey->ePinSta = E_KEY_PRESS;
        Key_Func(E_KEY_TIMER_HALFLONG, stKey);
        Key_Func(E_KEY_TIMER_LONG, stKey);
    }
}
static void btn_release_proc(KeyCtrl_t *stKey)
{
    sys_xQueSendToBackFromISR(xQueueHalSystemEvent, EXT_EVENT_SW_RELEASE, stKey->s16PinNum, NULL);
    if (stKey->s16PinNum == E_KEY_PRESS)
    {
        stKey->ePinSta = E_KEY_WAIT;
        Key_Func(E_KEY_TIMER_SHORT, stKey);
    }
    else
    {
        stKey->ePinSta = E_KEY_IDLE;
    }
}
static void key_timer_jitt_cb(void *argv)
{
    if (argv == NULL) { return; }
    KeyCtrl_t *stKey = (KeyCtrl_t *)argv;
    unsigned char _u08PinState = gpio_get_level(stKey->s16PinNum);
    if (stKey->ePinEdge == GPIO_INTR_NEGEDGE)
    {
        // Falling
        if (_u08PinState)
        {
            stKey->ePinSta = E_KEY_IDLE;
        }
        else
        {
            btn_press_proc(stKey);
        }
    }
    else if (stKey->ePinEdge == GPIO_INTR_POSEDGE)
    {
        // Rising
        if (!_u08PinState)
        {
            stKey->ePinSta = E_KEY_IDLE;
        }
        else
        {
            btn_release_proc(stKey);
        }
    }
    else
    {
        stKey->ePinSta = E_KEY_IDLE;
    }
    gpio_set_intr_type(stKey->s16PinNum, GPIO_INTR_ANYEDGE);
}
static void key_timer_short_cb(void *argv)
{
    KeyCtrl_t *pstKey = (KeyCtrl_t *)argv;

    if (E_KEY_WAIT == pstKey->ePinSta)
    {
        sys_xQueSendToBackFromISR(xQueueHalSystemEvent, EXT_EVENT_SW_SHORT, pstKey->s16PinNum, NULL);
    }
    pstKey->ePinSta = E_KEY_IDLE;
}
static void key_timer_halflong_cb(void *argv)
{
    KeyCtrl_t *pstKey = (KeyCtrl_t *)argv;
    if (E_KEY_PRESS == pstKey->ePinSta)
    {
        sys_xQueSendToBackFromISR(xQueueHalSystemEvent, EXT_EVENT_SW_HALF_LONGPRESS, pstKey->s16PinNum, NULL);
        pstKey->ePinSta = E_KEY_LONG_PRESS;
    }
}
static void key_timer_long_cb(void *argv)
{
    KeyCtrl_t *pstKey = (KeyCtrl_t *)argv;

    if (E_KEY_LONG_PRESS == pstKey->ePinSta)
    {
        sys_xQueSendToBackFromISR(xQueueHalSystemEvent, EXT_EVENT_SW_LONGPRESS, pstKey->s16PinNum, NULL);
    }
}

static void Key_isr_handler(void *arg)
{
    unsigned int u32PinNum = (unsigned int)arg;
    if (u32PinNum >= GPIO_NUM_MAX) { return; }
    gpio_uninstall_isr_service(0);      //disable interrupt
    unsigned int u32PinSta = GPIO_REG_READ(GPIO_STATUS_ADDRESS);    //读取GPIO中断状态

    if (u32PinSta & BIT(mstDevkeyValue[u32PinNum].s16PinNum))
    {
        gpio_set_intr_type(u32PinNum, GPIO_INTR_DISABLE);        
        if(gpio_get_level(mstDevkeyValue[u32PinNum].s16PinNum) == 0)
        {
            // Falling
            mstDevkeyValue[u32PinNum].ePinEdge = GPIO_INTR_NEGEDGE;
        }
        else
        {
            // Rising
            mstDevkeyValue[u32PinNum].ePinEdge = GPIO_INTR_POSEDGE;        
        }
        Key_Func(E_KEY_TIMER_JITTER, &mstDevkeyValue[u32PinNum]);
    }

    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, u32PinSta);         //clear interrupt mask    
    gpio_install_isr_service(0);        //install gpio isr service
}

static void gpio_key_gpio_PinFuncSelect(void)
{
    unsigned char u08Cnt = 0x00;
    for (u08Cnt = 0; u08Cnt < E_KEYFUNC_MAX; u08Cnt++)
    {
        if (mstDevkeyValue[u08Cnt].s16PinNum == KEY_FUNC_NULL)
        {
            continue;
        }
        switch(u08Cnt)
        {
            case GPIO_NUM_0: PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0); break;
            case GPIO_NUM_1: PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_GPIO1); break;
            case GPIO_NUM_2: PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); break;
            case GPIO_NUM_3: PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_GPIO3); break;
            case GPIO_NUM_4: PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4); break;
            case GPIO_NUM_5: PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5); break;
            case GPIO_NUM_6: PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CLK_U, FUNC_GPIO6); break;
            case GPIO_NUM_7: PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA0_U, FUNC_GPIO7); break;
            case GPIO_NUM_8: PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA1_U, FUNC_GPIO8); break;
            case GPIO_NUM_9: PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA2_U, FUNC_GPIO9); break;
            case GPIO_NUM_10: PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_GPIO10); break;
            case GPIO_NUM_11: PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CMD_U, FUNC_GPIO11); break;
            case GPIO_NUM_12: PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12); break;
            case GPIO_NUM_13: PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13); break;
            case GPIO_NUM_14: PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14); break;
            case GPIO_NUM_15: PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15); break;
            default: continue; break;
        }
    }
}
#endif

#undef _PK_DEVKEY_C_

