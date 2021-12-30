#include "stm32f10x.h"
#include "system_init.h"
#include "bsp_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "user_task.h"
#include "lcd.h"
#include "radio_init.h"
#include "dbg_print.h"

#define SW_V    "1.00"
#define HW_V    "1.00"
#define SN      "20200800"

/************Notice !!!**************
***All global variable define here***
*************************************/

QueueHandle_t KeyEventMsgQueue = NULL;
SemaphoreHandle_t printMutex = NULL;

static void sys_msg_queue_init(void)
{
	KeyEventMsgQueue = xQueueCreate(KEY_EVENT_QUEUE_NUM, KEY_EVENT_QUEUE_LEN);
}

static void sys_mutex_init(void)
{
	printMutex = xSemaphoreCreateMutex();
}

void sys_mutex_lock(SemaphoreHandle_t xMutex)
{
	xSemaphoreTake(xMutex, portMAX_DELAY);
}

void sys_mutex_unlock(SemaphoreHandle_t xMutex)
{
	xSemaphoreGive(xMutex);
}

static void sys_init_completed(void)
{
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n************************Start Run Application*************************");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n======================================================================");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=              (C) COPYRIGHT 2020 AHU XXXXXXXXXXXXX                  =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=                                                                    =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=     RF-Decode-Controlbox Application  (Version 1.0.1)              =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=                                                                    =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=     System Initialize completed...SW: %s HW: %s SN: %s   =", SW_V, HW_V, SN);
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=                                                                    =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=                                      By AHU Application Team       =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n======================================================================");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n\r\n");
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
    bsp_power_test_init();
    bsp_power_ctrl_init();
    
    while(bsp_power_test_read() == 1);
    if (bsp_power_test_read() == 0) {
        bsp_power_ctrl_set(0);
    }
    while(bsp_power_test_read() == 0);

    //GPIO_Config();
    bsp_uart1_init();
    key_board_init();
    sys_msg_queue_init();
    sys_mutex_init();
    LCD_Init();
    radio_init();

    //bsp_IWDG_init(IWDG_Prescaler_64, 3125); //5s
    //enable interrupts
    portENABLE_INTERRUPTS();
    sys_init_completed();
}

void os_task_init(void)
{
    BaseType_t ret;


#if 1
    ret = xTaskCreate(task_key_detect, "lcd", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES, NULL);
    if (ret != pdPASS) {
        dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
#endif
}

void os_rf_recv_task_create(void)
{
    BaseType_t ret;

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

    ret = xTaskCreate(task_rf430, "rf430", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_RF_RCV_TASK, NULL);
    if (ret != pdPASS) {
        //dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
}

TaskHandle_t handle_rf_send;
void os_rf_send_task_create(void)
{
    BaseType_t ret;
    ret = xTaskCreate(task_rf430_send, "rf_send", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_RF_SEND_TASK, handle_rf_send);
    if (ret != pdPASS) {
        //dbg_print(PRINT_LEVEL_ERROR, "create failed\r\n");
    }
}

void os_rf_send_task_delete(void)
{
    vTaskDelete(handle_rf_send);
}
