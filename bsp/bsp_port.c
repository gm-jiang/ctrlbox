#include "stm32f10x.h"
#include "bsp_port.h"

/*lamp status and key status
		bit0:red							[0,OFF][1,ON], 
    bit1:green						[0,OFF][1,ON], 
		bit2:yellow						[0,OFF][1,ON],
		bit3:debug_led				[0,OFF][1,ON],
		bit4:key_status				[0,UNVALID][1,VALID]
*/
uint8_t g_lampStatus = 0;

void bsp_gpio_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure all unused GPIO port pins in Analog Input mode (floating input
	* trigger OFF), this will reduce the power consumption and increase the device
	* immunity against EMI/EMC */

	// Enable GPIOs clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
												 RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
						             RCC_APB2Periph_GPIOE, ENABLE);

	// Set all GPIO pins as analog inputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Disable GPIOs clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
						             RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
						             RCC_APB2Periph_GPIOE, DISABLE);
}

void bsp_releasekey_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	// Enable GPIO used as key IRQ for interrupt
	GPIO_InitStructure.GPIO_Pin = KEYIRQ;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPD;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ
	GPIO_Init(KEYIRQ_GPIO, &GPIO_InitStructure);

	/* Connect EXTI Line to GPIO Pin */
	GPIO_EXTILineConfig(KEYIRQ_EXTI_PORT, KEYIRQ_EXTI_PIN);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = KEYIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//IRQ polarity is high by default
	EXTI_InitStructure.EXTI_LineCmd = KEYIRQ_EXTI_USEIRQ;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = KEYIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_REALEASE_KEY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = KEYIRQ_EXTI_USEIRQ;

	NVIC_Init(&NVIC_InitStructure);
}

void bsp_emerstopkey_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// Enable GPIO used as key IRQ for interrupt
	GPIO_InitStructure.GPIO_Pin = EMER_STOP_KEY_DETECTIRQ_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPD;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ
	GPIO_Init(EMER_STOP_KEY_DETECTIRQ_GPIO, &GPIO_InitStructure);

	/* Connect EXTI Line to GPIO Pin */
	GPIO_EXTILineConfig(EMER_STOP_KEY_DETECTIRQ_EXTI_PORT, EMER_STOP_KEY_DETECTIRQ_EXTI_PIN);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = EMER_STOP_KEY_DETECTIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = EMER_STOP_KEY_DETECTIRQ_EXTI_USEIRQ;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EMER_STOP_KEY_DETECTIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_EMER_STOP_KEY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = EMER_STOP_KEY_DETECTIRQ_EXTI_USEIRQ;

	NVIC_Init(&NVIC_InitStructure);
}

void bsp_chaindown_finish_sensor_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	// Enable GPIO used as key IRQ for interrupt
	GPIO_InitStructure.GPIO_Pin = CHAIN_DOWN_FINISH_DETECTIRQ_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPD;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ
	GPIO_Init(CHAIN_DOWN_FINISH_DETECTIRQ_GPIO, &GPIO_InitStructure);

	/* Connect EXTI Line to GPIO Pin */
	GPIO_EXTILineConfig(CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_PORT, CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_PIN);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = CHAIN_DOWN_FINISH_DETECTIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//IRQ polarity is high by default
	EXTI_InitStructure.EXTI_LineCmd = CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_USEIRQ;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_CHAIN_DOWN_FINISH_SENSOR;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_USEIRQ;

	NVIC_Init(&NVIC_InitStructure);
}

void bsp_ctrlbox_funcion_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = CTRLBOX_FUNCTION_BIT0_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CTRLBOX_FUNCTION_BIT0_GPIO, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = CTRLBOX_FUNCTION_BIT1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IN_FLOATING;	//
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CTRLBOX_FUNCTION_BIT1_GPIO, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = CTRLBOX_FUNCTION_BIT2_GPIO_PIN | CTRLBOX_FUNCTION_BIT3_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IN_FLOATING;	//
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CTRLBOX_FUNCTION_BIT2_GPIO, &GPIO_InitStructure);
}

void bsp_wcs_uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = USART1_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_WCS485;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void bsp_stc_uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = USART2_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = USART2_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART2_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = USART2_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_STC;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);
}

void bsp_uhfrfid_uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART3_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART3_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = USART3_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_UHFRFID;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART3, ENABLE);
}

void bsp_dbg_uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = UART4_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART4_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = UART4_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART4_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = UART4_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_DBG;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(UART4, ENABLE);
}

/*  IWDG init
 * Tout = prv/40 * rlv (s)
 *      prv[4,8,16,32,64,128,256]
 * prv:the value of Prescaler:
 *     @arg IWDG_Prescaler_4: IWDG prescaler set to 4
 *     @arg IWDG_Prescaler_8: IWDG prescaler set to 8
 *     @arg IWDG_Prescaler_16: IWDG prescaler set to 16
 *     @arg IWDG_Prescaler_32: IWDG prescaler set to 32
 *     @arg IWDG_Prescaler_64: IWDG prescaler set to 64
 *     @arg IWDG_Prescaler_128: IWDG prescaler set to 128
 *     @arg IWDG_Prescaler_256: IWDG prescaler set to 256
 *
   rlv:the value of reload
 * IWDG_Init(IWDG_Prescaler_64 ,625);  // IWDG 1s
 *                        (64/40)*625 = 1s
 */
void bsp_IWDG_init(uint8_t prv ,uint16_t rlv)
{
	IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );
	IWDG_SetPrescaler( prv );
	IWDG_SetReload( rlv );
	IWDG_ReloadCounter();
	IWDG_Enable();    
}

void bsp_mcu_485RE_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = WCS_485_RE_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(WCS_485_RE_GPIO, &GPIO_InitStructure);

	GPIO_ResetBits(WCS_485_RE_GPIO, WCS_485_RE_GPIO_PIN);
}

void bsp_uhfrfid_detect_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = UHFRFID_DETECT_SENSOR_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(UHFRFID_DETECT_SENSOR_GPIO, &GPIO_InitStructure);
}

//lamp init
void bsp_lamp_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = DEBUG_LED_RED_GPIO_PIN | DEBUG_LED_YEL_GPIO_PIN | DEBUG_LED_GRE_GPIO_PIN | \
																TRICOLOR_LAMP_RED_GPIO_PIN | TRICOLOR_LAMP_GREEN_GPIO_PIN;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(DEBUG_LED_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = TRICOLOR_LAMP_YELLOW_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(TRICOLOR_LAMP_YELLOW_GPIO, &GPIO_InitStructure);

	bsp_lamp_ctrl(TRICOLOR_LAMP_RED | TRICOLOR_LAMP_GREEN | TRICOLOR_LAMP_YELLOW | DEBUG_LED_RED | DEBUG_LED_YEL | DEBUG_LED_GRE, TURN_OFF);
}

void bsp_chaindown_ctrl_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = CHAIN_DOWN_CTRL_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CHAIN_DOWN_CTRL_GPIO, &GPIO_InitStructure);

	if(g_mcuConfigInfo.valveCtrlLevel == LEVEL_CTRL_HIGH)
		GPIO_SetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
	else
		GPIO_ResetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
}

void bsp_motor_ctrl_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = CHAIN_MOTOR_CTRL_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CHAIN_MOTOR_CTRL_GPIO, &GPIO_InitStructure);

	if(g_mcuConfigInfo.motorCtrlLevel == LEVEL_CTRL_HIGH)
		GPIO_SetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
	else
		GPIO_ResetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
}

void bsp_enable_485_pin(void)
{
	GPIO_SetBits(WCS_485_RE_GPIO, WCS_485_RE_GPIO_PIN);
}

void bsp_disable_485_pin(void)
{
	GPIO_ResetBits(WCS_485_RE_GPIO, WCS_485_RE_GPIO_PIN);
}

void bsp_uart1_send(uint8_t *buf, uint16_t length)
{
	uint16_t i;
	for(i = 0; i < length; i++)
	{
		USART_SendData(USART1, buf[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  }
}

void bsp_uart2_send(uint8_t *buf, uint16_t length)
{
	uint16_t i;
	for(i = 0; i < length; i++)
	{
		USART_SendData(USART2, buf[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  }
}

void bsp_uart3_send(uint8_t *buf, uint16_t length)
{
	uint16_t i;
	for(i = 0; i < length; i++)
	{
		USART_SendData(USART3, buf[i]);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
  }
}

void bsp_uart4_send(uint8_t *buf, uint16_t length)
{
	uint16_t i;
	for(i = 0; i < length; i++)
	{
		USART_SendData(UART4, buf[i]);
		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
  }
}

//lamp ctrl, lamp: TRICOLOR_LAMP_RED | TRICOLOR_LAMP_GREEN | TRICOLOR_LAMP_YELLOW | DEBUG_LED
void bsp_lamp_ctrl(uint8_t lamp, statusCtrlType_e lampCtrl)
{
	if(lampCtrl == TURN_ON)
	{
		if((lamp & TRICOLOR_LAMP_RED) == TRICOLOR_LAMP_RED)
		{
			if(g_mcuConfigInfo.lampCtrlLevel == LEVEL_CTRL_HIGH)
				GPIO_ResetBits(TRICOLOR_LAMP_RED_GPIO, TRICOLOR_LAMP_RED_GPIO_PIN);
			else
				GPIO_SetBits(TRICOLOR_LAMP_RED_GPIO, TRICOLOR_LAMP_RED_GPIO_PIN);
			
			g_lampStatus = g_lampStatus | TRICOLOR_LAMP_RED;
		}
		if((lamp & TRICOLOR_LAMP_GREEN) == TRICOLOR_LAMP_GREEN)
		{
			if(g_mcuConfigInfo.lampCtrlLevel == LEVEL_CTRL_HIGH)
				GPIO_ResetBits(TRICOLOR_LAMP_GREEN_GPIO, TRICOLOR_LAMP_GREEN_GPIO_PIN);
			else
				GPIO_SetBits(TRICOLOR_LAMP_GREEN_GPIO, TRICOLOR_LAMP_GREEN_GPIO_PIN);
			
			g_lampStatus = g_lampStatus | TRICOLOR_LAMP_GREEN;
		}
		if((lamp & TRICOLOR_LAMP_YELLOW) == TRICOLOR_LAMP_YELLOW)
		{
			if(g_mcuConfigInfo.lampCtrlLevel == LEVEL_CTRL_HIGH)
				GPIO_ResetBits(TRICOLOR_LAMP_YELLOW_GPIO, TRICOLOR_LAMP_YELLOW_GPIO_PIN);
			else
				GPIO_SetBits(TRICOLOR_LAMP_YELLOW_GPIO, TRICOLOR_LAMP_YELLOW_GPIO_PIN);
			
			g_lampStatus = g_lampStatus | TRICOLOR_LAMP_YELLOW;
		}
		if((lamp & DEBUG_LED_RED) == DEBUG_LED_RED)
		{
			GPIO_ResetBits(DEBUG_LED_GPIO, DEBUG_LED_RED_GPIO_PIN);
			g_lampStatus = g_lampStatus | DEBUG_LED_RED;
		}
		if((lamp & DEBUG_LED_YEL) == DEBUG_LED_YEL)
		{
			GPIO_ResetBits(DEBUG_LED_GPIO, DEBUG_LED_YEL_GPIO_PIN);
			g_lampStatus = g_lampStatus | DEBUG_LED_YEL;
		}
		if((lamp & DEBUG_LED_GRE) == DEBUG_LED_GRE)
		{
			GPIO_ResetBits(DEBUG_LED_GPIO, DEBUG_LED_GRE_GPIO_PIN);
			g_lampStatus = g_lampStatus | DEBUG_LED_GRE;
		}
	}
	else if(lampCtrl == TURN_OFF)
	{
		if((lamp & TRICOLOR_LAMP_RED) == TRICOLOR_LAMP_RED)
		{
			if(g_mcuConfigInfo.lampCtrlLevel == LEVEL_CTRL_HIGH)
				GPIO_SetBits(TRICOLOR_LAMP_RED_GPIO, TRICOLOR_LAMP_RED_GPIO_PIN);
			else
				GPIO_ResetBits(TRICOLOR_LAMP_RED_GPIO, TRICOLOR_LAMP_RED_GPIO_PIN);
			
			g_lampStatus = g_lampStatus & (~TRICOLOR_LAMP_RED);
		}
		if((lamp & TRICOLOR_LAMP_GREEN) == TRICOLOR_LAMP_GREEN)
		{
			if(g_mcuConfigInfo.lampCtrlLevel == LEVEL_CTRL_HIGH)
				GPIO_SetBits(TRICOLOR_LAMP_GREEN_GPIO, TRICOLOR_LAMP_GREEN_GPIO_PIN);
			else
				GPIO_ResetBits(TRICOLOR_LAMP_GREEN_GPIO, TRICOLOR_LAMP_GREEN_GPIO_PIN);
			
			g_lampStatus = g_lampStatus & (~TRICOLOR_LAMP_GREEN);
		}
		if((lamp & TRICOLOR_LAMP_YELLOW) == TRICOLOR_LAMP_YELLOW)
		{
			if(g_mcuConfigInfo.lampCtrlLevel == LEVEL_CTRL_HIGH)
				GPIO_SetBits(TRICOLOR_LAMP_YELLOW_GPIO, TRICOLOR_LAMP_YELLOW_GPIO_PIN);
			else
				GPIO_ResetBits(TRICOLOR_LAMP_YELLOW_GPIO, TRICOLOR_LAMP_YELLOW_GPIO_PIN);
			
			g_lampStatus = g_lampStatus & (~TRICOLOR_LAMP_YELLOW);
		}
		if((lamp & DEBUG_LED_RED) == DEBUG_LED_RED)
		{
			GPIO_SetBits(DEBUG_LED_GPIO, DEBUG_LED_RED_GPIO_PIN);
			g_lampStatus = g_lampStatus & (~DEBUG_LED_RED);
		}
		if((lamp & DEBUG_LED_YEL) == DEBUG_LED_YEL)
		{
			GPIO_SetBits(DEBUG_LED_GPIO, DEBUG_LED_YEL_GPIO_PIN);
			g_lampStatus = g_lampStatus & (~DEBUG_LED_YEL);
		}
		if((lamp & DEBUG_LED_GRE) == DEBUG_LED_GRE)
		{
			GPIO_SetBits(DEBUG_LED_GPIO, DEBUG_LED_GRE_GPIO_PIN);
			g_lampStatus = g_lampStatus & (~DEBUG_LED_GRE);
		}
	}
}

//watchdog feed
void bsp_IWDG_feed(void)
{
	IWDG_ReloadCounter();
}

//read flash halfword 16bit
uint16_t bsp_FLASH_ReadHalfWord(uint32_t address)
{
  return *(__IO uint16_t*)address; 
}

//read word 32bit
uint32_t bsp_FLASH_ReadWord(uint32_t address)
{
  uint32_t temp1,temp2;
  temp1=*(__IO uint16_t*)address; 
  temp2=*(__IO uint16_t*)(address + 2); 
  return (temp2<<16) + temp1;
}

//read more data once
void bsp_FLASH_ReadMoreData(uint32_t startAddress, uint16_t *readData, uint16_t countToRead)
{
  uint16_t dataIndex;
  for(dataIndex = 0;dataIndex < countToRead;dataIndex++)
  {
    readData[dataIndex] = bsp_FLASH_ReadHalfWord(startAddress + dataIndex * 2);
  }
}

//write more data once
void bsp_FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite)
{
	uint16_t dataIndex;
	uint32_t offsetAddress;
	uint32_t sectorPosition;
	uint32_t sectorStartAddress;
	
  if((startAddress < FLASH_BASE) ||((startAddress + countToWrite * 2) >= (FLASH_BASE + 1024 * FLASH_SIZE)))
  {
    return;//invalid addr
  }
  FLASH_Unlock();         //unlock
  offsetAddress = startAddress - FLASH_BASE;               //calc actual offset(sub 0X08000000)
  sectorPosition = offsetAddress / SECTOR_SIZE;            //calc address of sector
 
  sectorStartAddress = sectorPosition * SECTOR_SIZE + FLASH_BASE;    //the first address of sector

  FLASH_ErasePage(sectorStartAddress);//erase this address
  
  for(dataIndex = 0;dataIndex < countToWrite;dataIndex++)
  {
    FLASH_ProgramHalfWord(startAddress + dataIndex * 2,writeData[dataIndex]);
  }
  
  FLASH_Lock();//lock
}
