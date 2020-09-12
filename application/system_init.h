#ifndef _SYSTEM_INIT_H
#define _SYSTEM_INIT_H

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#define KEY_EVENT_QUEUE_NUM         10
#define KEY_EVENT_QUEUE_LEN         4

extern QueueHandle_t KeyEventMsgQueue;
extern SemaphoreHandle_t printMutex;

//event type
typedef enum {
	EVENT_MSG_KEY1 = 1,
	EVENT_MSG_KEY2,
	EVENT_MSG_KEY3,
	EVENT_MSG_KEY4,
	EVENT_MSG_KEY5,
    EVENT_MSG_MAX
} eventMsgType_e;

#define PRIORITIES_LED_STATUS_TASK            configMAX_PRIORITIES - 5
#define PRIORITIES_WDT_TASK                   configMAX_PRIORITIES - 4
#define PRIORITIES_UART3_RCV_TASK             configMAX_PRIORITIES - 3
#define PRIORITIES_UART2_RCV_TASK             configMAX_PRIORITIES - 2
#define PRIORITIES_UART1_RCV_TASK             configMAX_PRIORITIES - 1
#define PRIORITIES_RF_RCV_TASK                configMAX_PRIORITIES - 1

void sys_mutex_lock(SemaphoreHandle_t xMutex);
void sys_mutex_unlock(SemaphoreHandle_t xMutex);

void platform_init(void);
void os_task_init(void);

#endif
