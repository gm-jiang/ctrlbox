/**
  ******************************************************************************
  * @file    USART/HyperTerminal_Interrupt/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "dbg_print.h"
#include "user_task.h"

#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
		 this is done through SystemInit() function which is called from startup
		 file (startup_stm32f10x_xx.s) before to branch to application main.
		 To reconfigure the default setting of SystemInit() function, refer to
		 system_stm32f10x.c file
	 */
	BaseType_t ret;

	platform_init();

	//create lamp task
	ret = xTaskCreate(lamp_task, "lamp", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_LAMP_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_print(PRINT_LEVEL_ERROR, " create led_task failed\r\n");
	}
	else
	{
		dbg_print(PRINT_LEVEL_INFO, "create led_task successful\r\n");
	}

	//create watchdog task
	ret = xTaskCreate(watch_dog_task, "watch_dog", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_WATCH_DOG_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_print(PRINT_LEVEL_ERROR, " create watch_dog_task failed\r\n");
	}
	else
	{
		dbg_print(PRINT_LEVEL_INFO, "create watch_dog_task successful\r\n");
	}

	//create wcs485_msg task
	ret = xTaskCreate(wcs485_msg_task, "wcs485_msg", configMINIMAL_STACK_SIZE*10, NULL, PRIORITIES_WCS485_MSG_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_print(PRINT_LEVEL_ERROR, " create wcs485_msg_task failed\r\n");
	}
	else
	{
		dbg_print(PRINT_LEVEL_INFO, "create wcs485_msg_task successful\r\n");
	}

	//create stc_msg task
	ret = xTaskCreate(stc_msg_task, "stc_msg", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_STC_MSG_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_print(PRINT_LEVEL_ERROR, " create stc_msg_task failed\r\n");
	}
	else
	{
		dbg_print(PRINT_LEVEL_INFO, "create stc_msg_task successful\r\n");
	}

	//create uhfRFID_msg task
	ret = xTaskCreate(uhfRFID_msg_task, "uhfRFID_msg", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_UHFRFID_MSG_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_print(PRINT_LEVEL_ERROR, " create uhfRFID_msg_task failed\r\n");
	}
	else
	{
		dbg_print(PRINT_LEVEL_INFO, "create uhfRFID_msg_task successful\r\n");
	}

	ret = xTaskCreate(chainDown_sensor_task, "chainDown_sensor", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_CHAINDOWN_SENSOR_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_print(PRINT_LEVEL_ERROR, " create chainDown_sensor_task failed\r\n");
	}
	else
	{
		dbg_print(PRINT_LEVEL_INFO, "create chainDown_sensor_task successful\r\n");
	}

	// Start the FreeRTOS scheduler
	vTaskStartScheduler();
	// Should never reach there
	while(1);
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
