#ifndef _SYSTEM_INIT_H
#define _SYSTEM_INIT_H

#include "stm32f10x.h"

#include "FreeRTOS.h"



#define PRIORITIES_LED_STATUS_TASK            configMAX_PRIORITIES - 5
#define PRIORITIES_WDT_TASK                   configMAX_PRIORITIES - 4
#define PRIORITIES_UART3_RCV_TASK             configMAX_PRIORITIES - 3
#define PRIORITIES_UART2_RCV_TASK             configMAX_PRIORITIES - 2
#define PRIORITIES_UART1_RCV_TASK             configMAX_PRIORITIES - 1


void platform_init(void);
void os_task_init(void);

#endif
