#ifndef _SYSTEM_INIT_H
#define _SYSTEM_INIT_H

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "dlink.h"
#include "config.h"

extern tagNode_t g_node_list;
extern configInfo_t g_configInfo;

extern QueueHandle_t UART1RecvMsgQueue;
extern QueueHandle_t UART2RecvMsgQueue;
extern QueueHandle_t UART3RecvMsgQueue;
extern QueueHandle_t UpLoadEventMsgQueue;
extern QueueHandle_t eventMsgQueue;
extern QueueHandle_t LowRfidMsgQueue;

extern SemaphoreHandle_t printMutex;
extern SemaphoreHandle_t rfidDataMutex;

extern SemaphoreHandle_t chainDownDetectSemaphore;

#define PRIORITIES_LED_STATUS_TASK            configMAX_PRIORITIES - 5
#define PRIORITIES_WDT_TASK                   configMAX_PRIORITIES - 4
#define PRIORITIES_UART3_RCV_TASK             configMAX_PRIORITIES - 3
#define PRIORITIES_UART2_RCV_TASK             configMAX_PRIORITIES - 2
#define PRIORITIES_UART1_RCV_TASK             configMAX_PRIORITIES - 1


#define UART1_QUEUE_NUM       10
#define UART1_QUEUE_LEN       128

#define UART2_QUEUE_NUM       10
#define UART2_QUEUE_LEN       10

#define UART3_QUEUE_NUM       5
#define UART3_QUEUE_LEN       24

#define EVENT_QUEUE_NUM       10
#define EVENT_QUEUE_LEN       4+16

#define LOWRFID_QUEUE_NUM     10
#define LOWRFID_QUEUE_LEN     4

#define UHFRFID_LEN           UART3_QUEUE_LEN
#define UHFRFID_DATA_LEN      UHFRFID_LEN/2

#define MOTOR_CTRL_AUTH_ON         1
#define MOTOR_CTRL_AUTH_OFF        0

void sys_mutex_unlock(SemaphoreHandle_t xMutex);
void sys_mutex_lock(SemaphoreHandle_t xMutex);
void platform_init(void);
void os_task_init(void);

#endif
