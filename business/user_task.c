#include "stm32f10x.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_port.h" //use watchdog and led
#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"
#include "radio_recv.h"
#include "mt_common.h"

extern unsigned char rf315_en;
extern unsigned char rf330_en;
extern unsigned char rf433_en;

#if 1
void task_led_status(void *pvParameters)
{
    while(1)
    {
        bsp_power_status_led_set(0);
        vTaskDelay(500*20);
        bsp_power_status_led_set(1);
        vTaskDelay(3000*20);
    }
}
#endif

void task_rf315(void *pvParameters)
{
    while(1)
    {
        RF315_IN();
        if (rf315_en == 4) {
            rf315_en = 0;
            bsp_power_status_led_set(0);
            delay_us(100);
            bsp_power_status_led_set(1);
        }
        //vTaskDelay(1);
    }
}

void task_rf330(void *pvParameters)
{
    while(1)
    {
        RF330_IN();
        if (rf330_en == 4) {
            rf330_en = 0;
            bsp_power_status_led_set(0);
            delay_us(100);
            bsp_power_status_led_set(1);
        }
        //vTaskDelay(1);
    }
}

void task_rf433(void *pvParameters)
{
    while(1)
    {
        RF433_IN();
        if (rf433_en == 4) {
            rf433_en = 0;
            bsp_power_status_led_set(0);
            delay_us(100);
            bsp_power_status_led_set(1);
        }
        //vTaskDelay(1);
    }
}
