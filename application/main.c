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
#include "string.h"
#include "dbg_print.h"
#include "bsp_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"

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
	//return value of creating task
	BaseType_t ret;

	platform_Init();

	//create lamp task
	ret = xTaskCreate(lamp_task, "lamp", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_LAMP_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_Print(PRINT_LEVEL_DEBUG, " create led_task failed\n");
	}
	else
	{
		dbg_Print(PRINT_LEVEL_DEBUG, "create led_task successful\n");
	}

	#if 0
	//create watchdog task
	ret = xTaskCreate(watch_dog_task, "watch_dog", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_WATCH_DOG_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_Print(PRINT_LEVEL_DEBUG, " create watch_dog_task failed\n");
	}
	else
	{
		dbg_Print(PRINT_LEVEL_DEBUG, "create watch_dog_task successful\n");
	}
	#endif
	
	//create wcs485_msg task
	ret = xTaskCreate(wcs485_msg_task, "wcs485_msg", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_WCS485_MSG_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_Print(PRINT_LEVEL_DEBUG, " create wcs485_msg_task failed\n");
	}
	else
	{
		dbg_Print(PRINT_LEVEL_DEBUG, "create wcs485_msg_task successful\n");
	}
	
	//create stc_msg task
	ret = xTaskCreate(stc_msg_task, "stc_msg", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_STC_MSG_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_Print(PRINT_LEVEL_DEBUG, " create stc_msg_task failed\n");
	}
	else
	{
		dbg_Print(PRINT_LEVEL_DEBUG, "create stc_msg_task successful\n");
	}

	//create uhfRFID_msg task
	ret = xTaskCreate(uhfRFID_msg_task, "uhfRFID_msg", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_UHFRFID_MSG_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_Print(PRINT_LEVEL_DEBUG, " create uhfRFID_msg_task failed\n");
	}
	else
	{
		dbg_Print(PRINT_LEVEL_DEBUG, "create uhfRFID_msg_task successful\n");
	}
	
	//create uhfRFID_msg task
	ret = xTaskCreate(uhfRFID_detect_task, "uhfRFID_sensor_detect", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_UHFRFID_DETECT_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_Print(PRINT_LEVEL_DEBUG, " create uhfRFID_detect_task failed\n");
	}
	else
	{
		dbg_Print(PRINT_LEVEL_DEBUG, "create uhfRFID_detect_task successful\n");
	}
	
	ret = xTaskCreate(chainDown_sensor_task, "chainDown_sensor", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_CHAINDOWN_SENSOR_TASK, NULL);
	if(ret != pdPASS)
	{
		dbg_Print(PRINT_LEVEL_DEBUG, " create chainDown_sensor_task failed\n");
	}
	else
	{
		dbg_Print(PRINT_LEVEL_DEBUG, "create chainDown_sensor_task successful\n");
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
