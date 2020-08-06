#include "stm32f10x.h"
#include "system_init.h"
#include "bsp_port.h"
#include "FreeRTOS.h"
#include <string.h>
#include "user_task.h"


/************Notice !!!**************
***All global variable define here***
*************************************/

void platform_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);

	//disable interrupts
	portDISABLE_INTERRUPTS();
	bsp_all_gpio_configuration();
	bsp_power_status_led_init();
	//bsp_uart1_init();
	bsp_IWDG_init(IWDG_Prescaler_64, 3125); //5s
	//enable interrupts
	portENABLE_INTERRUPTS();
}

void os_task_init(void)
{
	BaseType_t ret;

	//controlbox run status task
	ret = xTaskCreate(task_led_status, "led_status", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_LED_STATUS_TASK, NULL);
	if (ret != pdPASS) {
		//dbg_print(PRINT_LEVEL_ERROR, "create task_led_status failed\r\n");
	}
}
