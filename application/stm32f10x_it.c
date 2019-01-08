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
#include <string.h>
#include "bsp_port.h"
#include "user_task.h"
#include "mt_common.h"
#include "wcs_parser.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

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
	BaseType_t ret;
	BaseType_t xHigherPriorityTaskWoken;
	eventMsgFrame_t eventMsg, tmpMsg;

	if(port_GetEmerStopKeyEXT_IRQStatus() != RESET)
	{
		mt_sleep_us(5*1000);
		if(port_CheckEmerStopKeyEXT_IRQ() == 0)
		{
			eventMsg.msgType = EVENT_MSG_EMER_STOP;
			ret = xQueueSendFromISR(eventMsgQueue, &eventMsg, &xHigherPriorityTaskWoken);
			if(ret == errQUEUE_FULL)
			{
				xQueueReceiveFromISR(eventMsgQueue, &tmpMsg, &xHigherPriorityTaskWoken);
				xQueueSendFromISR(eventMsgQueue, &eventMsg, &xHigherPriorityTaskWoken);
			}
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
		if(port_CheckChainDownFinishEXT_IRQ() == 0)
		{
			xSemaphoreGiveFromISR(chainDownDetectSemaphore, &pxHigherPriorityTaskWoken);
		}
		EXTI_ClearITPendingBit(CHAIN_DOWN_FINISH_DETECTIRQ_EXTI);
	}
}

/**
  * @brief  This function handles USARTx global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	eventMsgFrame_t eventMsg, tmpMsg;
	BaseType_t ret;
	
	if(port_GetEXT_IRQStatus() != RESET)
	{
		mt_sleep_us(5*1000);
		if(port_CheckEXT_IRQ() == 0)
		{
			eventMsg.msgType = EVENT_MSG_KEY_DOWN;
			ret = xQueueSendFromISR(eventMsgQueue, &eventMsg, &xHigherPriorityTaskWoken);
			if(ret == errQUEUE_FULL)
			{
				xQueueReceiveFromISR(eventMsgQueue, &tmpMsg, &xHigherPriorityTaskWoken);
				xQueueSendFromISR(eventMsgQueue, &eventMsg, &xHigherPriorityTaskWoken);
			}	
		}
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
void USART1_IRQHandler(void)
{
	uint8_t tmp_char = 0;
	static uint8_t state = 0;
	static uint8_t recv_cnt = 0;
	static uint8_t recv_buf[WCS_MSG_LEN] = {0};
	BaseType_t xHigherPriorityTaskWoken;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		tmp_char = USART_ReceiveData(USART1);
		switch (state)
		{
			case SYN_STATE:
				if (tmp_char == STC_FRAME_SYN)
				{
					state = STX_STATE;
					recv_buf[SYN_INDEX] = tmp_char;
				}
				break;
			case STX_STATE:
				if (tmp_char == STC_FRAME_STX)
				{
					state = LEN_STATE;
					recv_buf[STX_INDEX] = tmp_char;
				}
				break;
			case LEN_STATE:
				state = DAT_STATE;
				recv_buf[LEN_INDEX] = tmp_char;
				break;
			case DAT_STATE:
				recv_buf[DAT_INDEX + recv_cnt++] = tmp_char;
				if (recv_cnt == recv_buf[LEN_INDEX])
				{
					state = SYN_STATE;
					recv_cnt = 0;
					if (recv_buf[DAT_INDEX] != g_mcu485Addr && recv_buf[DAT_INDEX] != WCS_BROADCAST_ADDR)
					{
						memset(recv_buf, 0, WCS_MSG_LEN);
						return;
					}
					if (recv_buf[recv_buf[LEN_INDEX] + 2] == mt_cal_crc8(recv_buf, recv_buf[LEN_INDEX] + 2))
					{
						xQueueSendFromISR(wcs485RecvMsgQueue, recv_buf, &xHigherPriorityTaskWoken);
						memset(recv_buf, 0, WCS_MSG_LEN);
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
					}
				}
				break;
			default:
				break;
		}
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
	static uint8_t recv_buf[STC_MSG_LEN] = {0};
	BaseType_t xHigherPriorityTaskWoken;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		tmp_char = USART_ReceiveData(USART2);
		switch (state) 
		{
			case SYN_STATE:
				if (tmp_char == STC_FRAME_SYN)
				{
					state = STX_STATE;
					recv_buf[SYN_INDEX] = tmp_char;
				}
				break;
			case STX_STATE:
				if (tmp_char == STC_FRAME_STX)
				{
					state = LEN_STATE;
					recv_buf[STX_INDEX] = tmp_char;
				}
				break;
			case LEN_STATE:
				state = DAT_STATE;
				recv_buf[LEN_INDEX] = tmp_char;
				break;
			case DAT_STATE:
				recv_buf[DAT_INDEX + recv_cnt++] = tmp_char;
				if (recv_cnt == recv_buf[LEN_INDEX])
				{
					state = SYN_STATE;
					recv_cnt = 0;
					if (recv_buf[CRC_INDEX] == mt_check_sum(recv_buf, CRC_CHECK_LEN))
					{
						xQueueSendFromISR(stcRecvMsgQueue, recv_buf, &xHigherPriorityTaskWoken);
						memset(recv_buf, 0, STC_MSG_LEN);
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
					}
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
	static uint8_t recv_buf[UHF_MSG_LEN] = {0};
	BaseType_t xHigherPriorityTaskWoken;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		tmp_char = USART_ReceiveData(USART3);
		switch (state)
		{
			case SYN_STATE:
				if (tmp_char == UHF_RFID_FRAME_HEAD)
				{
					state = DAT_STATE;
					recv_cnt = 0;
				}
				break;
			case DAT_STATE:
				recv_buf[recv_cnt++] = tmp_char;
				if (recv_cnt == UHF_MSG_LEN)
				{
					state = SYN_STATE;
					recv_cnt = 0;
					xQueueSendFromISR(uhfRFIDRecvMsgQueue, recv_buf, &xHigherPriorityTaskWoken);
					memset(recv_buf, 0, UHF_MSG_LEN);
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
