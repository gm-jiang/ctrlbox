#include "stm32f10x.h"
#include "bsp_port.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#include "dbg_print.h"

//the receive message queue of uart
QueueHandle_t wcs485RecvMsgQueue = NULL;
QueueHandle_t stcRecvMsgQueue = NULL;
QueueHandle_t uhfRFIDRecvMsgQueue = NULL;
//QueueHandle_t chainDownRfidGetedQueue = NULL;
QueueHandle_t chainDownRfidOpenedQueue = NULL;
//event queue
QueueHandle_t eventMsgQueue = NULL;
//event semaphore
SemaphoreHandle_t eventMsgSemaphore = NULL;
//chain down sensor detected semaphore
SemaphoreHandle_t chainDownDetectSemaphore = NULL;
//chain down rfid data semaphore
SemaphoreHandle_t chainDownDataSemaphore = NULL;
SemaphoreHandle_t uhfMsgSemaphore = NULL;
//mcu cpuID
//uint32_t cpuID[3];
/*chain down stc rfid
		byte0:flag bit0[0,invalid][1,valid(get id)] bit1[0,invalid][1,valid(match id)] bit2[0,invalid][1,valid(chaindown finish)]
		byte1~byte4:rfid id data
*/
//uint8_t chainDownRFID[CHAIN_DOWN_STATUS_DATA_LEN * CHAIN_DOWN_DATA_BUF_GROUP_NUM];
chainDownData_t chainDownData[CHAIN_DOWN_DATA_BUF_LEN];

//485 addr
uint8_t mcu485Addr = 0;
uint8_t mcuFuncConfig = 0;

/*lamp status and key status
		bit0:red							[0,OFF][1,ON], 
    bit1:green						[0,OFF][1,ON], 
		bit2:yellow						[0,OFF][1,ON],
		bit3:debug_led				[0,OFF][1,ON],
		bit4:key_status				[0,UNVALID][1,VALID]
*/
uint8_t lampAndKeyStatus;

void GPIO_Configuration(void)
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

void releasekeyInt_Init(void)
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
	
	lampAndKeyStatus = lampAndKeyStatus & (~KEY_STATUS);
}

void emerStopkeyInt_Init(void)
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
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//double edge
	EXTI_InitStructure.EXTI_LineCmd = EMER_STOP_KEY_DETECTIRQ_EXTI_USEIRQ;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EMER_STOP_KEY_DETECTIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_EMER_STOP_KEY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = EMER_STOP_KEY_DETECTIRQ_EXTI_USEIRQ;

	NVIC_Init(&NVIC_InitStructure);
	
	//lampAndKeyStatus = lampAndKeyStatus & (~KEY_STATUS);
}

void chainDownFinishSensorInt_Init(void)
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
	
	//lampAndKeyStatus = lampAndKeyStatus & (~KEY_STATUS);
}

void wcs485Uart_Init(void)
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

void stcUart_Init(void)
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

void uhfRfidUart_Init(void)
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
void IWDG_Init(uint8_t prv ,uint16_t rlv)
{    
	IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );
	IWDG_SetPrescaler( prv );
	IWDG_SetReload( rlv );
	IWDG_ReloadCounter();
	IWDG_Enable();    
}

void mcu_485RE_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = WCS_485_RE_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(WCS_485_RE_GPIO,&GPIO_InitStructure);
	
	GPIO_ResetBits(WCS_485_RE_GPIO, WCS_485_RE_GPIO_PIN);
}

void uhfRfidDetect_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = UHFRFID_DETECT_SENSOR_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(UHFRFID_DETECT_SENSOR_GPIO,&GPIO_InitStructure);
	
	//GPIO_ResetBits(WCS_485_RE_GPIO, WCS_485_RE_GPIO_PIN);
	
}

//lamp init
void lamp_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = DEBUG_LED_GPIO_PIN | TRICOLOR_LAMP_RED_GPIO_PIN | TRICOLOR_LAMP_GREEN_GPIO_PIN | TRICOLOR_LAMP_YELLOW_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(DEBUG_LED_GPIO, &GPIO_InitStructure);
	/*
	lampAndKeyStatus = lampAndKeyStatus & (~TRICOLOR_LAMP_RED);
	lampAndKeyStatus = lampAndKeyStatus & (~TRICOLOR_LAMP_GREEN);
	lampAndKeyStatus = lampAndKeyStatus & (~TRICOLOR_LAMP_YELLOW);
	lampAndKeyStatus = lampAndKeyStatus & (~DEBUG_LED);
	*/
	lamp_Ctrl(TRICOLOR_LAMP_RED | TRICOLOR_LAMP_GREEN | TRICOLOR_LAMP_YELLOW | DEBUG_LED, TURN_OFF);
}

void chainDown_CtrlInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = CHAIN_DOWN_CTRL_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CHAIN_DOWN_CTRL_GPIO,&GPIO_InitStructure);
	
	GPIO_SetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
}

void motor_CtrlInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = CHAIN_MOTOR_CTRL_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CHAIN_MOTOR_CTRL_GPIO,&GPIO_InitStructure);
	
	GPIO_SetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
}

//platform init ,init the receive queue of uart
void platform_Init()
{
	uint8_t i;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	GPIO_Configuration();
	releasekeyInt_Init();
	emerStopkeyInt_Init();
	chainDownFinishSensorInt_Init();
	lamp_Init();
	chainDown_CtrlInit();
	motor_CtrlInit();
	mcu_485RE_Init();
	uhfRfidDetect_Init();
	IWDG_Init(IWDG_Prescaler_64, 3125); //5s

	//disable interrupts
	portDISABLE_INTERRUPTS();

	wcs485Uart_Init();
	stcUart_Init();
	uhfRfidUart_Init();

	//get_CpuID();

	//set_485Addr(3);
	mcu485Addr = get_485Addr();
	dbg_Print(PRINT_LEVEL_ERROR, "485 addr:%d\n", mcu485Addr);

	mcuFuncConfig = get_CustomerConfig();
	if (mcuFuncConfig != CHAIN_DOWN_CTRLBOX && 
			mcuFuncConfig != UHF_RFID_CTRLBOX && 
			mcuFuncConfig != RFID_CHECK_CTRLBOX)
	{
		mcuFuncConfig = CHAIN_DOWN_CTRLBOX;
	}
	dbg_Print(PRINT_LEVEL_ERROR, "device customer config:%d\n", mcuFuncConfig);

	wcs485RecvMsgQueue = xQueueCreate(UART_QUEUE_LEN, sizeof(uartMsgFrame_t));
	if(wcs485RecvMsgQueue == NULL)
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create wcs485RecvMsgQueue failed\n");
		while(1);
	}

	stcRecvMsgQueue = xQueueCreate(MT_UART_QUEUE_LEN, MT_UART_MSG_LEN);
	if(stcRecvMsgQueue == NULL)
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create stcRecvMsgQueue failed\n");
		while(1);
	}
	
	uhfRFIDRecvMsgQueue = xQueueCreate(MT_UART_QUEUE_LEN, MT_UHF_MSG_LEN);
	if(uhfRFIDRecvMsgQueue == NULL) 
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create uhfRFIDRecvMsgQueue failed\n");
		while(1);
	}
	
	eventMsgQueue = xQueueCreate(EVENT_MSG_QUEUE_LEN, sizeof(eventMsgFrame_t));
	if(eventMsgQueue == NULL) 
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create eventMsgQueue failed\n");
		while(1);
	}
	
	eventMsgSemaphore = xSemaphoreCreateBinary();
	if(eventMsgSemaphore == NULL) 
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create eventMsgSemaphore failed\n");
		while(1);
	}
	xSemaphoreGive(eventMsgSemaphore);
	
	memset(chainDownData, 0, sizeof(chainDownData_t));
	
	chainDownDetectSemaphore = xSemaphoreCreateBinary();
	if(chainDownDetectSemaphore == NULL) 
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create chainDownDetectSemaphore failed\n");
		while(1);
	}
	
	chainDownDataSemaphore = xSemaphoreCreateBinary();
	if(chainDownDataSemaphore == NULL) 
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create chainDownDataSemaphore failed\n");
		while(1);
	}
	xSemaphoreGive(chainDownDataSemaphore);
	
	chainDownRfidOpenedQueue = xQueueCreate(CHAIN_DOWN_RFID_OPENED_QUEUE_LEN, sizeof(chainDownMsgFrame_t));
	if(chainDownRfidOpenedQueue == NULL)
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create chainDownRfidOpenedQueue failed\n");
		while(1);
	}

	uhfMsgSemaphore = xSemaphoreCreateBinary();
	if(uhfMsgSemaphore == NULL) 
	{
		dbg_Print(PRINT_LEVEL_ERROR, "create uhfMsgSemaphore failed\n");
		while(1);
	}

	//enable interrupts
	portENABLE_INTERRUPTS();

	for(i = 0; i < 20; i++)
	{
		lamp_Ctrl(DEBUG_LED, TURN_ON);
		sleep_Us(50*1000);
		lamp_Ctrl(DEBUG_LED, TURN_OFF);
		sleep_Us(50*1000);
	}
}

void wcs485Uart_SendData(uint8_t *buf,uint16_t length)
{
	uint16_t i;
	//USART1->SR;
	GPIO_SetBits(WCS_485_RE_GPIO, WCS_485_RE_GPIO_PIN);
	//delay_us(800);
	for(i = 0; i < length;i++)
	{
		USART_SendData(USART1, buf[i]);
		while( USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  }
	sleep_Us(100);
	GPIO_ResetBits(WCS_485_RE_GPIO, WCS_485_RE_GPIO_PIN);
	//delay_us(100);
}

void stcUart_SendData(uint8_t *buf,uint16_t length)
{
	uint16_t i;
	//USART2->SR;
	for(i = 0; i < length;i++)
	{
		USART_SendData(USART2, buf[i]);
		while( USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  }
}

void uhfRfidUart_SendData(uint8_t *buf,uint16_t length)
{
	uint16_t i;
	//USART3->SR;
	for(i = 0; i < length;i++)
	{
		USART_SendData(USART3, buf[i]);
		while( USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
  }
}

// delay us
void sleep_Us(uint32_t time_us)
{
	uint32_t i = 0;
	for(i = 0;i < time_us;i++){
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}
}

//delay ms
void sleep_Ms(uint32_t time_ms)
{
	vTaskDelay(time_ms/portTICK_RATE_MS);
}

//lamp ctrl, lamp: TRICOLOR_LAMP_RED | TRICOLOR_LAMP_GREEN | TRICOLOR_LAMP_YELLOW | DEBUG_LED
void lamp_Ctrl(uint8_t lamp, statusCtrlType_e lampCtrl)
{
	if(lampCtrl == TURN_ON)
	{
		if((lamp & TRICOLOR_LAMP_RED) == TRICOLOR_LAMP_RED)
		{
			GPIO_ResetBits(TRICOLOR_LAMP_RED_GPIO, TRICOLOR_LAMP_RED_GPIO_PIN);
			lampAndKeyStatus = lampAndKeyStatus | TRICOLOR_LAMP_RED;
		}
		if((lamp & TRICOLOR_LAMP_GREEN) == TRICOLOR_LAMP_GREEN)
		{
			GPIO_ResetBits(TRICOLOR_LAMP_GREEN_GPIO, TRICOLOR_LAMP_GREEN_GPIO_PIN);
			lampAndKeyStatus = lampAndKeyStatus | TRICOLOR_LAMP_GREEN;
		}
		if((lamp & TRICOLOR_LAMP_YELLOW) == TRICOLOR_LAMP_YELLOW)
		{
			GPIO_ResetBits(TRICOLOR_LAMP_YELLOW_GPIO, TRICOLOR_LAMP_YELLOW_GPIO_PIN);
			lampAndKeyStatus = lampAndKeyStatus | TRICOLOR_LAMP_YELLOW;
		}
		if((lamp & DEBUG_LED) == DEBUG_LED)
		{
			GPIO_ResetBits(DEBUG_LED_GPIO, DEBUG_LED_GPIO_PIN);
			lampAndKeyStatus = lampAndKeyStatus | DEBUG_LED;
		}
	}
	else if(lampCtrl == TURN_OFF)
	{
		if((lamp & TRICOLOR_LAMP_RED) == TRICOLOR_LAMP_RED)
		{
			GPIO_SetBits(TRICOLOR_LAMP_RED_GPIO, TRICOLOR_LAMP_RED_GPIO_PIN);
			lampAndKeyStatus = lampAndKeyStatus & (~TRICOLOR_LAMP_RED);
		}
		if((lamp & TRICOLOR_LAMP_GREEN) == TRICOLOR_LAMP_GREEN)
		{
			GPIO_SetBits(TRICOLOR_LAMP_GREEN_GPIO, TRICOLOR_LAMP_GREEN_GPIO_PIN);
			lampAndKeyStatus = lampAndKeyStatus & (~TRICOLOR_LAMP_GREEN);
		}
		if((lamp & TRICOLOR_LAMP_YELLOW) == TRICOLOR_LAMP_YELLOW)
		{
			GPIO_SetBits(TRICOLOR_LAMP_YELLOW_GPIO, TRICOLOR_LAMP_YELLOW_GPIO_PIN);
			lampAndKeyStatus = lampAndKeyStatus & (~TRICOLOR_LAMP_YELLOW);
		}
		if((lamp & DEBUG_LED) == DEBUG_LED)
		{
			GPIO_SetBits(DEBUG_LED_GPIO, DEBUG_LED_GPIO_PIN);
			lampAndKeyStatus = lampAndKeyStatus & (~DEBUG_LED);
		}
	}
}

//the check sum of stc frame
uint8_t check_Sum(uint8_t *uBuff, uint8_t uBuffLen)
{
	uint8_t i,uSum = 0;
	for(i=0;i<uBuffLen;i++)
	{
		uSum = uSum + uBuff[i];
	}
	uSum = (~uSum) + 1;
	return uSum;
}

//calc crc8,CRC8/MAXIM,X8+X5+X4+1
uint8_t cal_crc8(uint8_t *uBuff,uint32_t uBuffLen)
{
	uint8_t crc = 0;
	uint8_t i;

	while(uBuffLen--)
	{
		crc ^= *uBuff++;
		for(i = 0;i < 8;i++)
		{
			if(crc & 0x01)
			{
				crc = (crc >> 1) ^ 0x8C;
			}
			else 
				crc >>= 1;
		}
	}
	return crc;
}

//watchdog feed
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();
}

/*
void get_CpuID(void)
{
	cpuID[0]=*(vu32*)(0x1ffff7e8);
	cpuID[1]=*(vu32*)(0x1ffff7ec);
	cpuID[2]=*(vu32*)(0x1ffff7f0);
}
*/

//flash
//read halfword 16bit
uint16_t FLASH_ReadHalfWord(uint32_t address)
{
  return *(__IO uint16_t*)address; 
}
//read word  32bit
uint32_t FLASH_ReadWord(uint32_t address)
{
  uint32_t temp1,temp2;
  temp1=*(__IO uint16_t*)address; 
  temp2=*(__IO uint16_t*)(address + 2); 
  return (temp2<<16) + temp1;
}
//read more data once
void FLASH_ReadMoreData(uint32_t startAddress,uint16_t *readData,uint16_t countToRead)
{
  uint16_t dataIndex;
  for(dataIndex = 0;dataIndex < countToRead;dataIndex++)
  {
    readData[dataIndex] = FLASH_ReadHalfWord(startAddress + dataIndex * 2);
  }
}

//write more data once
void FLASH_WriteMoreData(uint32_t startAddress,uint16_t *writeData,uint16_t countToWrite)
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

//get 485 address
uint8_t get_485Addr(void)
{
	uint16_t data;
	uint8_t mcu485Addr = 0;
	data = FLASH_ReadHalfWord(MCU_485_ADDR_BASE);
	mcu485Addr = data & 0xff;
	return mcu485Addr;
}

//set 485 address
void set_485Addr(uint8_t addr)
{
	uint16_t data = addr;
	FLASH_WriteMoreData(MCU_485_ADDR_BASE, &data, 1);
}

//set device sn
uint8_t get_DevSn(uint8_t *snBuf, uint8_t *snLen)
{
	uint16_t buf[WCS_485_DEVICE_SN_LEN];
	uint8_t i = 0;
	uint8_t j = 0;
	if(snBuf == NULL)
	{
		return RTN_FAIL;
	}
	
	FLASH_ReadMoreData(MCU_485_DEV_SN_BASE, buf, WCS_485_DEVICE_SN_LEN);
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	snBuf[i++] = (buf[j++]) & 0xff;
	
	*snLen = i;
	
	return RTN_SUCCESS;
}

//get device sn
uint8_t set_DevSn(uint8_t *snBuf, uint8_t snLen)
{
	uint16_t buf[WCS_485_DEVICE_SN_LEN];
	uint8_t i = 0;
	uint8_t j = 0;
	if(snBuf == NULL)
	{
		return RTN_FAIL;
	}
	
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	buf[i++] = snBuf[j++];
	
	if(i != snLen)
		return RTN_FAIL;
	
	FLASH_WriteMoreData(MCU_485_DEV_SN_BASE, buf, snLen);
	
	return RTN_SUCCESS;
}

//get customer config
uint8_t get_CustomerConfig(void)
{
	uint16_t data;
	uint8_t config = 0;
	data = FLASH_ReadHalfWord(MCU_485_DEV_CUSTOMER_BASE);
	config = data & 0xff;
	return config;
}	

//set customer config
void set_CustomerConfig(uint8_t config)
{
	uint16_t data = config;
	FLASH_WriteMoreData(MCU_485_DEV_CUSTOMER_BASE, &data, 1);
}

/*
uint8_t get_ChainDownValidBuf(void)
{
	uint8_t i, ret;
	ret = 0xff;
	for(i = 0; i < CHAIN_DOWN_DATA_BUF_GROUP_NUM; i++)
	{
		if(chainDownRFID[i * CHAIN_DOWN_STATUS_DATA_LEN] == CHAIN_DOWN_STATUS_DATA_INVALID)
		{
			ret = i * CHAIN_DOWN_STATUS_DATA_LEN;
			break;
		}
	}
	return ret;
}
*/

uint8_t add_ChainDownData(uint8_t *buf, uint8_t len)
{
	uint8_t ret = RTN_SUCCESS;
	uint8_t i, max = 0;
	
	if((len != STC_RFID_ID_LEN) || (buf == NULL))
		return RTN_FAIL;
	
	xSemaphoreTake(chainDownDataSemaphore, portMAX_DELAY);
	for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
	{
		if((chainDownData[i].valid == CHAIN_DOWN_DATA_VALID) && (memcmp(chainDownData[i].rfid, buf, STC_RFID_ID_LEN) == 0))
		{
			xSemaphoreGive(chainDownDataSemaphore);
			return ret;
		}
	}
		
	for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
	{
		if(chainDownData[i].valid == CHAIN_DOWN_DATA_INVALID)
		{
			memcpy(chainDownData[i].rfid, buf, STC_RFID_ID_LEN);
			chainDownData[i].age = 0;
			chainDownData[i].valid = CHAIN_DOWN_DATA_VALID;
			break;
		}
	}
	
	if(i == CHAIN_DOWN_DATA_BUF_LEN)
	{
		max = 0;
		for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
		{
			if(chainDownData[i].age >= chainDownData[max].age)
			{
				max = i;
			}
		}
		memcpy(chainDownData[max].rfid, buf, STC_RFID_ID_LEN);
		chainDownData[max].age = 0;
		chainDownData[max].valid = CHAIN_DOWN_DATA_VALID;
	}
	
	for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
	{
		if(chainDownData[i].valid == CHAIN_DOWN_DATA_VALID)
		{
			chainDownData[i].age = (chainDownData[i].age + 1) % CHAIN_DOWN_DATA_AGE_MAX;
		}
	}
	xSemaphoreGive(chainDownDataSemaphore);
	
	return ret;
}

void wcs485_Update485AddrOrSNackSend(uint8_t *dataBuf, uint8_t dataLen)
{
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	//uint8_t sendLen = 0;
	uint8_t i = 0;
	sendBuf[i++] = WCS_FRAME_HEAD_1;
	sendBuf[i++] = WCS_FRAME_HEAD_2;
	i++;
	sendBuf[i++] = mcu485Addr;
	if((dataLen > 0) && (dataBuf != NULL))
	{
		memcpy(&sendBuf[i], dataBuf, dataLen);
		i = i + dataLen;
	}
	sendBuf[WCS_FRAME_LEN_INDEX] = i + 1 - (WCS_FRAME_LEN_INDEX + 1);
	sendBuf[i] = cal_crc8(sendBuf, i);
	wcs485Uart_SendData(sendBuf, i + 1);
}

void wcs485_ChainOpen(void)
{
	GPIO_ResetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
	sleep_Ms(200);
	GPIO_SetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
}

void wcs485_MotorCtrl(statusCtrlType_e statusCtrl)
{
	if(statusCtrl == TURN_ON)
	{
		GPIO_SetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
	}
	else
	{
		GPIO_ResetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
	}
}

uint8_t wcs485_Encode(wcsFrameType_e frameType, uint8_t *dataBuf, uint8_t dataLen, uint8_t *encodeBuf, uint8_t *encodeLen)
{
	uint8_t i = 0;
	if(encodeBuf == NULL)
		return RTN_FAIL;
	
	encodeBuf[i++] = WCS_FRAME_HEAD_1;
	encodeBuf[i++] = WCS_FRAME_HEAD_2;
	i++;
	encodeBuf[i++] = mcu485Addr;
	
	switch(frameType)
	{
		case WCS_FRAME_QUERY_EMPTY_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_EMPTY_ACK;
			break;
		case WCS_FRAME_QUERY_CHAIN_UP_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_CHAIN_UP_ACK;
			break;
		case WCS_FRAME_QUERY_CHAIN_DOWN_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_CHAIN_DOWN_ACK;
			break;
		case WCS_FRAME_QUERY_ORDER_STATUS_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_ORDER_STATUS_ACK;
			break;
		case WCS_FRAME_QUERY_KEY_DOWN_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_KEY_DOWN_ACK;
			break;
		case WCS_FRAME_QUERY_ALARM_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_ALARM_ACK;
			break;
		case WCS_FRAME_CHAIN_DOWN_CTRL_ACK_E:
			encodeBuf[i++] = WCS_FRAME_CHAIN_DOWN_CTRL_ACK;
			break;
		case WCS_FRAME_ORDER_STATUS_CTRL_ACK_E:
			encodeBuf[i++] = WCS_FRAME_ORDER_STATUS_CTRL_ACK;
			break;
		default:
			return RTN_FAIL;
	}
	
	if((dataLen > 0) && (dataBuf != NULL))
	{
		memcpy(&encodeBuf[i], dataBuf, dataLen);
		i = i + dataLen;
	}
	
	encodeBuf[WCS_FRAME_LEN_INDEX] = i + 1 - (WCS_FRAME_LEN_INDEX + 1);
	
	encodeBuf[i] = cal_crc8(encodeBuf, i);
	
	*encodeLen = i + 1;
	
	return RTN_SUCCESS;
}

wcsFrameType_e wcs485_Decode(uint8_t *originBuf, uint8_t originLen, uint8_t *decodeBuf,uint8_t *decodeLen)
{
	wcsFrameType_e frameType = WCS_FRAME_INVALID_E;
	
	if((originBuf == NULL) || (decodeBuf == NULL))
		return WCS_FRAME_INVALID_E;
	
	switch(originBuf[WCS_485_FUNC_INDEX])
	{
		case WCS_FRAME_QUERY:
			frameType = WCS_FRAME_QUERY_E;
			*decodeLen = 0;
			break;
		case WCS_FRAME_CHAIN_DOWN_CTRL:
			frameType = WCS_FRAME_CHAIN_DOWN_CTRL_E;
			*decodeLen = originBuf[WCS_FRAME_LEN_INDEX] - 3;
			memcpy(decodeBuf, &originBuf[WCS_485_FUNC_INDEX + 1], *decodeLen);
			break;
		case WCS_FRAME_ORDER_STATUS_CTRL:
			frameType = WCS_FRAME_ORDER_STATUS_CTRL_E;
			*decodeLen = originBuf[WCS_FRAME_LEN_INDEX] - 3;
			memcpy(decodeBuf, &originBuf[WCS_485_FUNC_INDEX + 1], *decodeLen);
			break;
		default:
			return WCS_FRAME_INVALID_E;
	}
	return frameType;
}

uint8_t wcs485_QueryEmptyAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_EMPTY_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs485Uart_SendData(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainUpAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	//uint8_t dataBuf[WCS_SEND_BUF_LEN];
	//uint8_t dataLen = 10;
	ret = wcs485_Encode(WCS_FRAME_QUERY_CHAIN_UP_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs485Uart_SendData(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainDownAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	//uint8_t dataBuf[WCS_SEND_BUF_LEN];
	//uint8_t dataLen = 10;
	ret = wcs485_Encode(WCS_FRAME_QUERY_CHAIN_DOWN_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs485Uart_SendData(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_OrderStatusAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	//uint8_t tmpBuf = lampAndKeyStatus & 0x07;
	ret = wcs485_Encode(WCS_FRAME_QUERY_ORDER_STATUS_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs485Uart_SendData(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainKeyDownAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_KEY_DOWN_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs485Uart_SendData(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainAlarmAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_ALARM_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs485Uart_SendData(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainDownCmdAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_CHAIN_DOWN_CTRL_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs485Uart_SendData(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_OrderStatusCmdAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_ORDER_STATUS_CTRL_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs485Uart_SendData(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_QueryCmd()
{
	uint8_t ret = RTN_SUCCESS;
	eventMsgFrame_t eventMsg;
	BaseType_t queueRet;
	#if 0
	if(lampAndKeyStatus & KEY_STATUS)
	{
		ret = wcs485_ChainKeyDownAckSend();
	}
	#endif
	queueRet = xQueueReceive(eventMsgQueue, &eventMsg, 0);
	if(queueRet != pdTRUE)
	{
		ret = wcs485_QueryEmptyAckSend();
	}
	else
	{
		switch(eventMsg.msgType)
		{
			case EVENT_MSG_CHAIN_UP:
				ret = wcs485_ChainUpAckSend(eventMsg.msg, UHF_RFID_LABLE_DATA_LEN + STC_RFID_ID_LEN);
				break;
			case EVENT_MSG_CHAIN_DOWN:
				ret = wcs485_ChainDownAckSend(eventMsg.msg, STC_RFID_ID_LEN);
				break;
			case EVENT_MSG_ORDER_STATUS:
				ret = wcs485_OrderStatusAckSend(eventMsg.msg, WCS_ORDER_STATUS_LEN);
				break;
			case EVENT_MSG_KEY_DOWN:
				ret = wcs485_ChainKeyDownAckSend();
				break;
			case EVENT_MSG_ALARM:
				ret = wcs485_ChainAlarmAckSend();
				break;
			default:
				break;
		}
	}
	return ret;
}

uint8_t wcs485_ChainDownCmd(uint8_t *dataBuf, uint8_t dataLen)
{
	uint8_t ret;
	ret = wcs485_ChainDownCmdAckSend();
	/*
	//xSemaphoreTake(chainDownRfidSemaphore, portMAX_DELAY);
	//chainDownRFID[CHAIN_DOWN_STATUS_FLAG_INDEX] = CHAIN_DOWN_STATUS_GET_RFID;
	//ret = get_ChainDownValidBuf();
	if(ret < 0xff)
	{
		if((dataBuf != NULL) && (dataLen >= STC_RFID_ID_LEN))
		{
			memcpy(&chainDownRFID[ret + 1], dataBuf, STC_RFID_ID_LEN);
			chainDownRFID[ret] = chainDownRFID[ret] | CHAIN_DOWN_STATUS_GET_RFID;
		}	
	}
	//xSemaphoreGive(chainDownRfidSemaphore);
	*/
	ret = add_ChainDownData(dataBuf, STC_RFID_ID_LEN);
	
	return ret;
}

uint8_t wcs485_OrderStatusCmd(uint8_t *dataBuf, uint8_t dataLen)
{
	uint8_t ret;
	eventMsgFrame_t eventMsg, tmpMsg;
	BaseType_t queue_send_ret;
	
	ret = wcs485_OrderStatusCmdAckSend();
	if(dataBuf[0] & TRICOLOR_LAMP_RED)
	{
		lamp_Ctrl(TRICOLOR_LAMP_RED, TURN_ON);
	}
	else
	{
		lamp_Ctrl(TRICOLOR_LAMP_RED, TURN_OFF);
	}
	
	if(dataBuf[0] & TRICOLOR_LAMP_GREEN)
	{
		lamp_Ctrl(TRICOLOR_LAMP_GREEN, TURN_ON);
	}
	else
	{
		lamp_Ctrl(TRICOLOR_LAMP_GREEN, TURN_OFF);
	}
	
	if(dataBuf[0] & TRICOLOR_LAMP_YELLOW)
	{
		lamp_Ctrl(TRICOLOR_LAMP_YELLOW, TURN_ON);
	}
	else
	{
		lamp_Ctrl(TRICOLOR_LAMP_YELLOW, TURN_OFF);
	}
	
	eventMsg.msgType = EVENT_MSG_ORDER_STATUS;
	eventMsg.msg[0] = lampAndKeyStatus & 0x07;
	xSemaphoreTake(eventMsgSemaphore, portMAX_DELAY);
	xQueueSend(eventMsgQueue, &eventMsg, 0);
	if(queue_send_ret == errQUEUE_FULL)
	{
		xQueueReceive(eventMsgQueue, &tmpMsg, 0);
		xQueueSend(eventMsgQueue, &eventMsg, 0);
	}
	xSemaphoreGive(eventMsgSemaphore);
	
	return ret;
}

/*
//light sensor,after chain down
void sensor_ChainDownStatus(void)
{
	eventMsgFrame_t eventMsg, tmpMsg;
	BaseType_t queue_send_ret;
	uint8_t i;
	
	for(i = 0; i < CHAIN_DOWN_DATA_BUF_GROUP_NUM; i++)
	{
		if(chainDownRFID[i * CHAIN_DOWN_STATUS_DATA_LEN] == (CHAIN_DOWN_STATUS_GET_RFID | CHAIN_DOWN_STATUS_MATCH_ID))
		{
			//xSemaphoreTakeFromISR(chainDownRfidSemaphore, &pxHigherPriorityTaskWoken);
			chainDownRFID[i * CHAIN_DOWN_STATUS_DATA_LEN] = CHAIN_DOWN_STATUS_DATA_INVALID;
			
			xSemaphoreTake(eventMsgSemaphore, portMAX_DELAY);
			eventMsg.msgType = EVENT_MSG_CHAIN_DOWN;
			memcpy(eventMsg.msg, &chainDownRFID[i * CHAIN_DOWN_STATUS_DATA_LEN + 1], STC_RFID_ID_LEN);
			xQueueSend(eventMsgQueue, &eventMsg, 0);
			if(queue_send_ret == errQUEUE_FULL)
			{
				xQueueReceive(eventMsgQueue, &tmpMsg, 0);
				xQueueSend(eventMsgQueue, &eventMsg, 0);
			}
			xSemaphoreGive(eventMsgSemaphore);
			
			//xSemaphoreGive(chainDownRfidSemaphore);
		}
	}
	
}
*/

