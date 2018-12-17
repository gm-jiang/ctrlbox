#ifndef _USER_TASK_H
#define _USER_TASK_H
#include "stm32f10x.h"
#include "string.h"
#include "dbg_print.h"
#include "bsp_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define PRIORITIES_LAMP_TASK								configMAX_PRIORITIES - 7
#define PRIORITIES_WCS485_MSG_TASK					configMAX_PRIORITIES - 1
#define PRIORITIES_STC_MSG_TASK							configMAX_PRIORITIES - 2
#define PRIORITIES_UHFRFID_MSG_TASK					configMAX_PRIORITIES - 4
#define PRIORITIES_WATCH_DOG_TASK						configMAX_PRIORITIES - 5
#define PRIORITIES_UHFRFID_DETECT_TASK			configMAX_PRIORITIES - 3
#define PRIORITIES_CHAINDOWN_SENSOR_TASK		configMAX_PRIORITIES - 6

//
void lamp_task(void *pvParameters);
void wcs485_msg_task(void *pvParameters);
void stc_msg_task(void *pvParameters);
void uhfRFID_msg_task(void *pvParameters);
void watch_dog_task(void *pvParameters);
void uhfRFID_detect_task(void *pvParameters);
void chainDown_sensor_task(void *pvParameters);

#endif

