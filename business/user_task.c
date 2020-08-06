#include "stm32f10x.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_port.h" //use watchdog and led
#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"

void task_led_status(void *pvParameters)
{
	while(1)
	{
		bsp_power_status_led_set(1);
		vTaskDelay(1000);
		bsp_power_status_led_set(0);
		vTaskDelay(1000);	
		bsp_IWDG_feed();
	}
}
