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

QueueHandle_t KeyEventMsgQueue = NULL;


static void sys_msg_queue_init(void)
{
	KeyEventMsgQueue = xQueueCreate(KEY_EVENT_QUEUE_NUM, KEY_EVENT_QUEUE_LEN);
}

static void key_board_init(void)
{
    bsp_key1_init();
    bsp_key2_init();
    bsp_key3_init();
    bsp_key4_init();
}

void platform_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);

    //disable interrupts
    portDISABLE_INTERRUPTS();
    bsp_gpio_config();
    bsp_uart1_init();
    key_board_init();
    bsp_power_status_led_init();
    sys_msg_queue_init();
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

    ret = xTaskCreate(task_rf4xx, "rf4xx", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_RF_RCV_TASK, NULL);
    if (ret != pdPASS) {
        //dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
#endif
#if 1
    ret = xTaskCreate(task_key_detect, "lcd", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES, NULL);
    if (ret != pdPASS) {
        //dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
#endif
}
