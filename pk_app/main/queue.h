/*
 * queue.h
 *
 *  Created on: 2018-6-1
 *      Author: gongjw
 */

#ifndef APP_D2_INCLUDE_QUEUE_H_
#define APP_D2_INCLUDE_QUEUE_H_

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct _sysevent_t
{
    char cmd;
    short value;
    char *msgptr;   //指针用于携带其他结构体
}sysevent_t;

void sys_xQuecleanAll(xQueueHandle que);
int  sys_xQueSendToFrontFromISR(xQueueHandle dir,char cmd,short value,char *msg);
int  sys_xQueSendToBackFromISR(xQueueHandle dir,char cmd,short value,char *msg);
int  sys_xQueSendToFront(xQueueHandle dir,char cmd,short value,char *msg);
int  sys_xQueSendToBack(xQueueHandle dir,char cmd,short value,char *msg);

/**********progress step****************************************************************/
typedef struct _step_item_t
{
    int step_name;                      // running / shake / still
    unsigned char *p_params;            // 配置参数
    struct _step_item_t *prev;
    struct _step_item_t *next;
}step_item_t;

typedef struct _prgress_queue_t
{
    unsigned char mcount;
    unsigned char mindex;
    struct _step_item_t *p_q_stopper;
    xSemaphoreHandle mutex;
}progress_queue_t;

void slist_qinit(progress_queue_t *q);
void slist_qfree(progress_queue_t *q);
void slist_qdisp(progress_queue_t *q);
void slist_step_free(step_item_t *slist);
void slist_step_putq_begin(progress_queue_t *q, step_item_t *slist);
void slist_step_putq_last(progress_queue_t *q, step_item_t *slist);
step_item_t *slist_step_create(int step_name, unsigned char *params, int size);
step_item_t *slist_step_getq_begin(progress_queue_t *q);
step_item_t *slist_step_getq_last(progress_queue_t *q);
step_item_t *slist_step_getq_current(progress_queue_t *q);
step_item_t *slist_step_getq_params(progress_queue_t *q, unsigned char *params);
step_item_t *slist_step_getq_params_and_stepname(progress_queue_t *q, unsigned char *params, int step_name);
int slist_step_checkq_params(progress_queue_t *q, unsigned char *params);
int slist_step_checkq_params_and_stepname(progress_queue_t *q, unsigned char *params, int step_name);
int slist_step_empty(progress_queue_t *q);
/***************************************************************************************/


#endif
