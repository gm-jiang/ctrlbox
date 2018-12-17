/**
  ******************************************************************************
  * @file    USART/HyperTerminal_Interrupt/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "stm32f10x_it.h"
#include "bsp_port.h"
#include <string.h>

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

//emergency stop key detect
void EXTI0_IRQHandler(void)
{
	if(port_GetEmerStopKeyEXT_IRQStatus() != RESET)
	{
		sleep_Us(5*1000);
		if(port_CheckEmerStopKeyEXT_IRQ() == 0)
		{
			wcs485_MotorCtrl(TURN_OFF);
		}
		else if(port_CheckEmerStopKeyEXT_IRQ() == 1)
		{
			wcs485_MotorCtrl(TURN_ON);
		}
		
		EXTI_ClearITPendingBit(EMER_STOP_KEY_DETECTIRQ_EXTI);
	}
} 

//chain down finished detect sensor
void EXTI4_IRQHandler(void)
{
	BaseType_t pxHigherPriorityTaskWoken;
	if(port_GetChainDownFinishEXT_IRQStatus() != RESET)
	{
		//sleep_Us(5*100);
		if(port_CheckChainDownFinishEXT_IRQ() == 0)
		{
			//sensor_ChainDownStatus();
			xSemaphoreGiveFromISR(chainDownDetectSemaphore, &pxHigherPriorityTaskWoken);
		}
		//dbg_Print(PRINT_LEVEL_DEBUG, "detected key\n");
		while(port_CheckChainDownFinishEXT_IRQ() != 1);
		EXTI_ClearITPendingBit(CHAIN_DOWN_FINISH_DETECTIRQ_EXTI);
	}
}

//static uint8_t key_flag = 0;
/**
  * @brief  This function handles USARTx global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	BaseType_t pxHigherPriorityTaskWoken;
	BaseType_t xHigherPriorityTaskWoken;
	eventMsgFrame_t eventMsg, tmpMsg;
	BaseType_t ret;
	
	if(port_GetEXT_IRQStatus() != RESET)
	{
		sleep_Us(5*1000);
		if(port_CheckEXT_IRQ() == 0)
		{
			
			lampAndKeyStatus = lampAndKeyStatus | KEY_STATUS;
			eventMsg.msgType = EVENT_MSG_KEY_DOWN;
			eventMsg.msg[0] = 1;
			xSemaphoreTakeFromISR(eventMsgSemaphore, &pxHigherPriorityTaskWoken);
			ret = xQueueSendFromISR(eventMsgQueue, &eventMsg, &xHigherPriorityTaskWoken);
			if(ret == errQUEUE_FULL)
			{
				xQueueReceiveFromISR(eventMsgQueue, &tmpMsg, &xHigherPriorityTaskWoken);
				xQueueSendFromISR(eventMsgQueue, &eventMsg, &xHigherPriorityTaskWoken);
			}	
			xSemaphoreGive(eventMsgSemaphore);
			//lampAndKeyStatus = lampAndKeyStatus & (~0x07);
			//lamp_Ctrl(TRICOLOR_LAMP_RED | TRICOLOR_LAMP_GREEN | TRICOLOR_LAMP_YELLOW, TURN_OFF);
		}
		//dbg_Print(PRINT_LEVEL_DEBUG, "detected key\n");
		while(port_CheckEXT_IRQ() != 1);
		EXTI_ClearITPendingBit(KEYIRQ_EXTI);
	}
	
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
static uint8_t uart1_data_index = 0;
static uartMsgFrame_t uart1FrameData;
void USART1_IRQHandler(void)
{
	uint8_t res;
	uint8_t crc8 = 0;
	BaseType_t xHigherPriorityTaskWoken;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //????(?????????0x0d 0x0a??)
	{
		res = USART_ReceiveData(USART1);//(USART1->DR);	//????????
		if((uart1_data_index == 0)&&(res != WCS_FRAME_HEAD_1))
			return;
		
		if((uart1_data_index == 1)&&(res != WCS_FRAME_HEAD_2))
		{
			uart1_data_index = 0;
			return;
		}
		
		if(uart1_data_index == WCS_485_ADDR_INDEX)
		{
			if((res != mcu485Addr) && (res != WCS_BROADCAST_ADDR)) 
			{
				uart1_data_index = 0;
				return;
			}
		}
		
		uart1FrameData.msg[uart1_data_index] = res;
		uart1_data_index++;
		if(uart1_data_index == (uart1FrameData.msg[WCS_FRAME_LEN_INDEX] + 3))
		{
			crc8 = cal_crc8(&(uart1FrameData.msg[0]), uart1FrameData.msg[WCS_FRAME_LEN_INDEX] + 3 - 1);
			if(crc8 == uart1FrameData.msg[uart1_data_index - 1])
			{
				//uart1FrameData.msgType = MSG_DEBUG_TYPE;
				xQueueSendFromISR(wcs485RecvMsgQueue, &uart1FrameData, &xHigherPriorityTaskWoken);
			}
			uart1_data_index = 0;
		}
		uart1_data_index = uart1_data_index % UART_MSG_LEN;	
	}
}

/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */


/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	uint8_t tmp_char = 0;
	static uint8_t state = 0;
	static uint8_t recv_cnt = 0;
	static uint8_t recv_buf[MT_UART_MSG_LEN] = {0};
	BaseType_t xHigherPriorityTaskWoken;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		tmp_char = USART_ReceiveData(USART2);
		switch (state)
		{
			case SYN_STATE:
				if (tmp_char == MT_LOW_RFID_FRAME_HEAD)
				{
					state = LEN_STATE;
					recv_buf[SYN_INDEX] = tmp_char;
				}
				break;
			case LEN_STATE:
				state = DAT_STATE;
				recv_buf[LEN_INDEX] = tmp_char;
				break;
			case DAT_STATE:
				recv_buf[DAT_INDEX + recv_cnt++] = tmp_char;
				if (recv_cnt == recv_buf[1] - 1)
				{
					state = CRC_STATE;
					recv_cnt = 0;
				}
				break;
			case CRC_STATE:
				recv_buf[CRC_INDEX] = tmp_char;
				state = SYN_STATE;
				if (recv_buf[CRC_INDEX] == check_Sum(recv_buf, CRC_CHECK_LEN))
				{
					xQueueSendFromISR(stcRecvMsgQueue, recv_buf, &xHigherPriorityTaskWoken);
					memset(recv_buf, 0, MT_UART_MSG_LEN);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
				}
				break;
			default:
				break;
		}
	}
}

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	uint8_t tmp_char = 0;
	static uint8_t state = 0;
	static uint8_t recv_cnt = 0;
	static uint8_t recv_buf[MT_UHF_MSG_LEN] = {0};
	BaseType_t xHigherPriorityTaskWoken;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		tmp_char = USART_ReceiveData(USART3);
		switch (state)
		{
			case SYN_STATE:
				if (tmp_char == MT_UHF_RFID_FRAME_HEAD)
				{
					state = DAT_STATE;
				}
				break;
			case DAT_STATE:
				recv_buf[recv_cnt++] = tmp_char;
				if (recv_cnt == MT_UHF_MSG_LEN)
				{
					state = SYN_STATE;
					recv_cnt = 0;
					xQueueSendFromISR(uhfRFIDRecvMsgQueue, recv_buf, &xHigherPriorityTaskWoken);
					memset(recv_buf, 0, MT_UHF_MSG_LEN);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
				}
				break;
			default:
				break;
		}
	}
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
