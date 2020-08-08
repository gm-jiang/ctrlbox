#include "stm32f10x.h"
#include "system_init.h"
#include "bsp_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "user_task.h"
#include "lcd.h"
#include "radio_init.h"

/************Notice !!!**************
***All global variable define here***
*************************************/

void platform_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);

    //disable interrupts
    portDISABLE_INTERRUPTS();
    bsp_gpio_config();
    bsp_uart1_init();
    bsp_power_status_led_init();
    bsp_key_init();
    LCD_Init();
    radio_init();

    //bsp_IWDG_init(IWDG_Prescaler_64, 3125); //5s
    //enable interrupts
    portENABLE_INTERRUPTS();
}

void os_task_init(void)
{
    BaseType_t ret;
#if 1
    //controlbox run RF task
    ret = xTaskCreate(task_rf315, "rf315", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_RF_RCV_TASK, NULL);
    if (ret != pdPASS) {
        //dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
    ret = xTaskCreate(task_rf330, "rf330", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_RF_RCV_TASK, NULL);
    if (ret != pdPASS) {
        //dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
    ret = xTaskCreate(task_rf433, "rf433", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_RF_RCV_TASK, NULL);
    if (ret != pdPASS) {
        //dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
    ret = xTaskCreate(task_led_status, "led", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_RF_RCV_TASK, NULL);
    if (ret != pdPASS) {
        //dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
#endif
}
