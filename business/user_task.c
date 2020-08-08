#include "stm32f10x.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_port.h" //use watchdog and led
#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"
#include "radio_recv.h"
#include "mt_common.h"

extern unsigned char rf_en;

void task_led_status(void *pvParameters)
{
    while(1)
    {
        //bsp_power_status_led_set(1);
        //vTaskDelay(1000);
        //bsp_power_status_led_set(0);
        //vTaskDelay(1000);
        RF315_IN();
        if (rf_en == 4) {
            rf_en = 0;
            bsp_power_status_led_set(0);
            delay_us(50);
            bsp_power_status_led_set(1);
        }
        bsp_IWDG_feed();
    }
}
