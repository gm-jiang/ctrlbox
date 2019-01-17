#ifndef _USER_TASK_H
#define _USER_TASK_H

#include "stm32f10x.h"
#include "string.h"
#include "dbg_print.h"
#include "bsp_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define PRIORITIES_LAMP_TASK								configMAX_PRIORITIES - 7
#define PRIORITIES_WCS485_MSG_TASK					configMAX_PRIORITIES - 1
#define PRIORITIES_STC_MSG_TASK							configMAX_PRIORITIES - 2
#define PRIORITIES_UHFRFID_MSG_TASK					configMAX_PRIORITIES - 4
#define PRIORITIES_WATCH_DOG_TASK						configMAX_PRIORITIES - 5
#define PRIORITIES_UHFRFID_DETECT_TASK			configMAX_PRIORITIES - 3
#define PRIORITIES_CHAINDOWN_SENSOR_TASK		configMAX_PRIORITIES - 6

#define WCS_MSG_LEN									128  //the max length of the frame buffer
#define STC_MSG_LEN									10  //the max length of the frame buffer
#define UHF_MSG_LEN									24  //the max length of the frame buffer
#define UART_QUEUE_NUM							5  //the max length of the uart msg queue

#define LOWRFID_QUEUE_NUM						5  //the max length of the uart msg queue
#define LOWRFID_QUEUE_LEN						10  //the max length of the uart msg queue

//the receive message queue of uart
extern QueueHandle_t wcs485RecvMsgQueue;
extern QueueHandle_t stcRecvMsgQueue;
extern QueueHandle_t uhfRFIDRecvMsgQueue;
extern QueueHandle_t chainDownRfidOpenedQueue;
extern QueueHandle_t eventMsgQueue;

extern SemaphoreHandle_t chainDownDetectSemaphore;
extern SemaphoreHandle_t chainDownDataSemaphore;
extern SemaphoreHandle_t uhfMsgSemaphore;
extern SemaphoreHandle_t printMutex;


void sys_mutex_lock(SemaphoreHandle_t xMutex);
void sys_mutex_unlock(SemaphoreHandle_t xMutex);
void platform_init(void);
void lamp_task(void *pvParameters);
void wcs485_msg_task(void *pvParameters);
void stc_msg_task(void *pvParameters);
void uhfRFID_msg_task(void *pvParameters);
void watch_dog_task(void *pvParameters);
void chainDown_sensor_task(void *pvParameters);

#endif

