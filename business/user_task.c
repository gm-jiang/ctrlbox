#include "stm32f10x.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_port.h" //use watchdog and led
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "user_task.h"
#include "radio_recv.h"
#include "mt_common.h"
#include "system_init.h"

#include "lcd.h"
#include "gui.h"
#include "pic.h"

extern unsigned char rf315_en;
extern unsigned char rf330_en;
extern unsigned char rf433_en;
extern unsigned char rf4xx_en;


#if 1
void task_key_detect(void *pvParameters)
{
    BaseType_t ret;
    eventMsgType_e eventMsg;

	while(1)
	{
		ret = xQueueReceive(KeyEventMsgQueue, &eventMsg, portMAX_DELAY);
		if (ret == pdTRUE) {
            switch (eventMsg)
            {
                case EVENT_MSG_KEY1:
                    LCD_Clear(BLACK);
                    break;
                case EVENT_MSG_KEY2:
                    LCD_Clear(BLUE);
                    break;
                case EVENT_MSG_KEY3:
                    LCD_Clear(RED);
                    break;
                case EVENT_MSG_KEY4:
                    LCD_Clear(YELLOW);
                    break;
                default:
                    break;
            }
		}
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
            //USART1_Send();
            bsp_power_status_led_set(0);
            delay_us(100);
            bsp_power_status_led_set(1);
        }
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
    }
}

void task_rf4xx(void *pvParameters)
{
    while(1)
    {
        RF4XX_IN();
        if (rf4xx_en == 4) {
            rf4xx_en = 0;
            bsp_power_status_led_set(0);
            delay_us(100);
            bsp_power_status_led_set(1);
        }
    }
}
