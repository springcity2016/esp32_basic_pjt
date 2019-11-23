/*
 * queue.c
 *
 *  Created on: 2018-5-30
 *      Author: gongjw
 */
// #include "esp_common.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "user_config.h"
#include "petkit_clog.h"
#include "queue.h"

#include "pk_hal.h"

//                             系统的事件队列设计                                    //
////////////////////////////////////////////////////////////////////////
void sys_xQuecleanAll(xQueueHandle que)
{
    sysevent_t msg;
    memset(&msg, 0, sizeof(sysevent_t));

    while (xQueueReceive(que, (void *)&msg, (portTickType)(10) / portTICK_RATE_MS)) {
        if (msg.msgptr) {
            free(msg.msgptr);
            msg.msgptr = NULL;
        }
        memset(&msg, 0, sizeof(sysevent_t));
    }
}
/***********************************************
 * 函数名：sys_xQueSendToFrontISR
 * 功    能：中断中向其他任务发送消息
 * 参    数：@dir  接收消息目标  @cmd 命令 @value 参数  @msg 参数
 * 返    回：无
 ***********************************************/
int  sys_xQueSendToFrontFromISR(xQueueHandle dir,char cmd,short value,char *msg)
{
    sysevent_t event;
    int ret;
    int tick;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    event.cmd = cmd;
    event.value = value;
    event.msgptr = msg;
    tick = 1;
    send_SendToFrontFromISR:
    ret = xQueueSendToFrontFromISR(dir, &event, &xHigherPriorityTaskWoken);
    if(pdFALSE == ret){
        tick++;
        if(tick>5){
            if(msg){free(msg);}
            msg = NULL;
        }else{
            goto send_SendToFrontFromISR;
        }
    }
    return ret;
}
/**********************************************
 * 函数名：sys_xQueSendToBackISR
 * 功    能：中断中向其他任务发送消息
 * 参    数：@dir  接收消息目标  @cmd 命令 @value 参数  @msg 参数
 * 返    回：无
 **********************************************/
int  sys_xQueSendToBackFromISR(xQueueHandle dir,char cmd,short value,char *msg)
{
    sysevent_t event;
    int ret;
    int tick;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    event.cmd = cmd;
    event.value = value;
    event.msgptr = msg;
    tick = 1;
    send_SendToBackFromISR:
    ret = xQueueSendToBackFromISR(dir, &event, &xHigherPriorityTaskWoken);
    if(pdFALSE == ret){
        tick++;
        if(tick>5){
            if(msg){free(msg);}
            msg = NULL;
        }else{
            goto send_SendToBackFromISR;
        }
    }
    return ret;
}
/**********************************************
 * 函数名：sys_send_msg_other_task
 * 功    能：向其他任务发送消息
 * 参    数：@dir  接收消息目标  @cmd 命令 @value 参数  @msg 参数
 * 返    回：无
 **********************************************/
int  sys_xQueSendToFront(xQueueHandle dir,char cmd,short value,char *msg)
{
    sysevent_t event;
    int ret;
    int tick;

    event.cmd = cmd;
    event.value = value;
    event.msgptr = msg;
    tick = 1;
    send_SendToFront:
    ret = xQueueSendToFront(dir, &event, 20*((portTickType)1/portTICK_RATE_MS));
    if(pdFALSE == ret){
        tick++;
        if(tick>5){
            if(msg){free(msg);}
            msg = NULL;
        }else{
            goto send_SendToFront;
        }
    }
    return ret;
}
/***********************************************
 * 函数名：sys_xQueSendToBack
 * 功    能：向其他任务发送消息
 * 参    数：@dir  接收消息目标  @cmd 命令 @value 参数  @msg 参数
 * 返    回：无
 ************************************************/
int  sys_xQueSendToBack(xQueueHandle dir,char cmd,short value,char *msg)
{
    sysevent_t event;
    int ret;
    int tick;

    event.cmd = cmd;
    event.value = value;
    event.msgptr = msg;
    tick = 1;
    send_SendToBack:
    ret = xQueueSendToBack(dir, &event, 20*((portTickType)1/portTICK_RATE_MS));
    if(pdFALSE == ret){
        tick++;
        if(tick>5){
            if(msg){free(msg);}
            msg = NULL;
        }else{
            goto send_SendToBack;
        }
    }
    return ret;
}


/**********progress step****************************************************************/

void slist_qinit(progress_queue_t *q)
{
    if (q == NULL) { return; }

    q->p_q_stopper = slist_step_create(-1, NULL, 0);
    q->p_q_stopper->next = q->p_q_stopper;
    q->p_q_stopper->prev = q->p_q_stopper;
    q->mutex = xSemaphoreCreateMutex();
    q->mcount = 0;
    q->mindex = 0;
}
void slist_qfree(progress_queue_t *q)
{
    step_item_t *slist = NULL;
    if (q == NULL) {
        return;
    }
    if (xTask_mutex_trylock(q->mutex, 1000)) {
        slist = q->p_q_stopper->next;
        while(slist != q->p_q_stopper) {
            q->p_q_stopper->next = slist->next;
            slist->next->prev= q->p_q_stopper;
            slist_step_free(slist);
            slist = q->p_q_stopper->next;
        }
        q->p_q_stopper->next = q->p_q_stopper;
        q->p_q_stopper->prev = q->p_q_stopper;
        q->mcount = 0;
        q->mindex = 0;
        xTask_mutex_unlock(q->mutex);
    }
}
void slist_qdisp(progress_queue_t *q)
{
    step_item_t *slist = NULL;
    int index = 0;
    if (NULL == q) { return; }
    if (xTask_mutex_trylock(q->mutex, 1000)) {
        index = 0;
        slist = q->p_q_stopper->next;
        while(slist != q->p_q_stopper) {
            // pkLOG_EMERG("");
            slist = slist->next;
        }
        xTask_mutex_unlock(q->mutex);
    }
}
void slist_step_free(step_item_t *slist)
{
    if (slist) {
        if (slist->p_params) { free(slist->p_params); slist->p_params = NULL; }
        free(slist); slist = NULL;
    }
}
void slist_step_putq_begin(progress_queue_t *q, step_item_t *slist)
{
    if ((q == NULL) || (slist == NULL)) { return; }
    if (xTask_mutex_trylock(q->mutex, 1000)) {
        slist->next = q->p_q_stopper->next;
        q->p_q_stopper->next->prev = slist;
        q->p_q_stopper->next = slist;
        slist->prev = q->p_q_stopper;
        q->mcount++;
        q->mindex = 1;
        xTask_mutex_unlock(q->mutex);
    }
}
void slist_step_putq_last(progress_queue_t *q, step_item_t *slist)
{
    if ((q == NULL) || (slist == NULL)) { return; }
    if (xTask_mutex_trylock(q->mutex, 1000)) {
        slist->prev = q->p_q_stopper->prev;
        q->p_q_stopper->prev->next = slist;
        q->p_q_stopper->prev = slist;
        slist->next = q->p_q_stopper;
        q->mcount++;
        if ((q->mindex == 0) || (q->mindex >= q->mcount)) {
            q->mindex = 1;
        }
        xTask_mutex_unlock(q->mutex);
    }
}
step_item_t *slist_step_create(int step_name, unsigned char *params, int size)
{
    step_item_t *slist = NULL;
    slist = (char *)malloc(sizeof(step_item_t));
    if (slist == NULL) {
        pkLOG_EMERG(" %s func, malloc step_item_t list failed! %d \n", __func__, __LINE__);
    } else {
        memset(slist, 0, sizeof(step_item_t));
        slist->prev = slist->next = NULL;
        if (params) {
            slist->p_params = (unsigned char *)malloc(size);
            if (NULL == slist->p_params) {
                free(slist); slist = NULL;
                return slist;
            }
            memset(slist->p_params, 0, size);
            memcpy(slist->p_params, params, size);
        } else {
            slist->p_params = NULL;
        }
        slist->step_name = step_name;
    }
    return slist;
}
step_item_t *slist_step_getq_begin(progress_queue_t *q)
{
    step_item_t *p_tmp = NULL;
    if (q == NULL) { return NULL; }

    if (xTask_mutex_trylock(q->mutex, 1000)) {
        p_tmp = q->p_q_stopper->next;
        if (p_tmp == q->p_q_stopper) {
            q->mcount = 0;
            xTask_mutex_unlock(q->mutex);
            return NULL;
        }
        q->p_q_stopper->next = p_tmp->next;
        p_tmp->next->prev = q->p_q_stopper;
        p_tmp->prev = NULL;
        p_tmp->next = NULL;
        q->mcount--;
        q->mindex--;
        xTask_mutex_unlock(q->mutex);
    }
    return p_tmp;
}
step_item_t *slist_step_getq_last(progress_queue_t *q)
{
    step_item_t *p_tmp = NULL;
    if (q == NULL) { return NULL; }
    
    if (xTask_mutex_trylock(q->mutex, 1000)) {
        p_tmp = q->p_q_stopper->prev;
        if (p_tmp == q->p_q_stopper) {
            q->mcount = 0;
            xTask_mutex_unlock(q->mutex);
            return NULL;
        }
        q->p_q_stopper->prev = p_tmp->prev;
        p_tmp->prev->next = q->p_q_stopper;
        p_tmp->prev=NULL;
        p_tmp->next=NULL;
        q->mcount--;   
        q->mindex--;     
        xTask_mutex_unlock(q->mutex);
    }
    return p_tmp;
}
step_item_t *slist_step_getq_current(progress_queue_t *q)
{
    step_item_t *p_tmp = NULL;
    step_item_t *slist = NULL;
    int index = 0;
    if (NULL == q) { return pdFALSE; }
    if (xTask_mutex_trylock(q->mutex, 1000)) {
        p_tmp = NULL;
        if (q->mindex) {
            index = q->mindex;
            slist = q->p_q_stopper->next;
            while(slist != q->p_q_stopper) {
                index--;
                if (index == 0) {
                    p_tmp = slist;
                    break;
                }
                slist = slist->next;
            }
        }
        xTask_mutex_unlock(q->mutex);
    }
    return p_tmp;
}
step_item_t *slist_step_getq_params(progress_queue_t *q, unsigned char *params);
step_item_t *slist_step_getq_params_and_stepname(progress_queue_t *q, unsigned char *params, int step_name);
int slist_step_checkq_params(progress_queue_t *q, unsigned char *params);
int slist_step_checkq_params_and_stepname(progress_queue_t *q, unsigned char *params, int step_name);
int slist_step_empty(progress_queue_t *q)
{
    if (q == NULL) { return 0; }
    if (q->p_q_stopper == q->p_q_stopper->next) {
        q->mcount = 0;
        return 0;
    } else {
        return -1;
    }
}
/***************************************************************************************/


