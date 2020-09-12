#include "stm32f10x.h"
#include "bsp_port.h"

void bsp_all_gpio_configuration(void)
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

void bsp_gpio_config(void)
{ 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
}

void bsp_uart1_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;

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
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
#if 0
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_WCS485;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
    USART_Cmd(USART1, ENABLE);
}

void bsp_uart2_init(void)
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

void bsp_uart3_init(void)
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

void bsp_dbg_uart4_init(void)
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

void bsp_power_status_led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = POWER_LED_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(POWER_LED_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(POWER_LED_GPIO, POWER_LED_GPIO_PIN);
}

void bsp_power_status_led_set(uint8_t status)
{
    if (status) {
        GPIO_ResetBits(POWER_LED_GPIO, POWER_LED_GPIO_PIN);
    } else {
        GPIO_SetBits(POWER_LED_GPIO, POWER_LED_GPIO_PIN);
    }
}

#if 0
void bsp_key_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(KEY2_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin =KEY2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(KEY3_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin =KEY3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(KEY3_GPIO_PORT, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(KEY4_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin =KEY4_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(KEY4_GPIO_PORT, &GPIO_InitStructure);
}
#endif

void bsp_key1_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // Enable GPIO used as key IRQ for interrupt
    GPIO_InitStructure.GPIO_Pin = KEY1_DETECTIRQ_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPU;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ
    GPIO_Init(KEY1_DETECTIRQ_GPIO, &GPIO_InitStructure);

    /* Connect EXTI Line to GPIO Pin */
    GPIO_EXTILineConfig(KEY1_DETECTIRQ_EXTI_PORT, KEY1_DETECTIRQ_EXTI_PIN);

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = KEY1_DETECTIRQ_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = KEY1_DETECTIRQ_EXTI_USEIRQ;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = KEY1_DETECTIRQ_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_KEY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = KEY1_DETECTIRQ_EXTI_USEIRQ;

    NVIC_Init(&NVIC_InitStructure);
}

void bsp_key2_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // Enable GPIO used as key IRQ for interrupt
    GPIO_InitStructure.GPIO_Pin = KEY2_DETECTIRQ_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ
    GPIO_Init(KEY2_DETECTIRQ_GPIO, &GPIO_InitStructure);

    /* Connect EXTI Line to GPIO Pin */
    GPIO_EXTILineConfig(KEY2_DETECTIRQ_EXTI_PORT, KEY2_DETECTIRQ_EXTI_PIN);

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = KEY2_DETECTIRQ_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = KEY2_DETECTIRQ_EXTI_USEIRQ;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = KEY2_DETECTIRQ_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_KEY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = KEY2_DETECTIRQ_EXTI_USEIRQ;

    NVIC_Init(&NVIC_InitStructure);
}

void bsp_key3_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // Enable GPIO used as key IRQ for interrupt
    GPIO_InitStructure.GPIO_Pin = KEY3_DETECTIRQ_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ
    GPIO_Init(KEY3_DETECTIRQ_GPIO, &GPIO_InitStructure);

    /* Connect EXTI Line to GPIO Pin */
    GPIO_EXTILineConfig(KEY3_DETECTIRQ_EXTI_PORT, KEY3_DETECTIRQ_EXTI_PIN);

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = KEY3_DETECTIRQ_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = KEY3_DETECTIRQ_EXTI_USEIRQ;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = KEY3_DETECTIRQ_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_KEY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = KEY3_DETECTIRQ_EXTI_USEIRQ;

    NVIC_Init(&NVIC_InitStructure);
}

void bsp_key4_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // Enable GPIO used as key IRQ for interrupt
    GPIO_InitStructure.GPIO_Pin = KEY4_DETECTIRQ_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ
    GPIO_Init(KEY4_DETECTIRQ_GPIO, &GPIO_InitStructure);

    /* Connect EXTI Line to GPIO Pin */
    GPIO_EXTILineConfig(KEY4_DETECTIRQ_EXTI_PORT, KEY4_DETECTIRQ_EXTI_PIN);

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = KEY4_DETECTIRQ_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = KEY4_DETECTIRQ_EXTI_USEIRQ;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = KEY4_DETECTIRQ_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_KEY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = KEY4_DETECTIRQ_EXTI_USEIRQ;

    NVIC_Init(&NVIC_InitStructure);
}

void bsp_key5_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    // Enable GPIO used as key IRQ for interrupt
    GPIO_InitStructure.GPIO_Pin = KEY5_DETECTIRQ_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ
    GPIO_Init(KEY5_DETECTIRQ_GPIO, &GPIO_InitStructure);

    /* Connect EXTI Line to GPIO Pin */
    GPIO_EXTILineConfig(KEY5_DETECTIRQ_EXTI_PORT, KEY5_DETECTIRQ_EXTI_PIN);

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = KEY5_DETECTIRQ_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = KEY5_DETECTIRQ_EXTI_USEIRQ;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = KEY5_DETECTIRQ_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PREEMPTION_PRIORITY_KEY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = KEY5_DETECTIRQ_EXTI_USEIRQ;

    NVIC_Init(&NVIC_InitStructure);
}


void GPIO_Pin_Setting(GPIO_TypeDef *gpio, uint16_t nPin, GPIOSpeed_TypeDef speed, GPIOMode_TypeDef mode)
{
    u16 i;
    u32 nCfg, nMask = 0x0F;
    
    nCfg  = (mode&0x10) ?speed :0;
    nCfg |= mode & 0x0C;
    
    if(nPin == 0)
        return;
    
    if(nPin < 0x0100)
    {
        for(i=nPin; (0x01&i)==0; i >>= 1) {
            nCfg <<= 4;
            nMask <<= 4;
        }
        
        gpio->CRL &= ~nMask;
        gpio->CRL |= nCfg;
    }
    else
    {
        for(i=(nPin>>8); (0x01&i)==0; i >>= 1) {
            nCfg <<= 4;
            nMask <<= 4;
        }
        
        gpio->CRH &= ~nMask;
        gpio->CRH |= nCfg;
    }
    
    if(GPIO_Mode_IPD==mode)
        gpio->BRR = nPin;
    
    else if(GPIO_Mode_IPU==mode)
        gpio->BSRR = nPin;
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

#include "mt_common.h"

uint8_t item_buff[7] = {0x7E, 0x05, 0xA0, 0x00, 0x01, 0xA6, 0xEF};


void USART1_Send(void)
{
    int i1;	
    delay_us(20);

    for(i1=0;i1<7;i1++)
    {			
        USART_SendData(USART1, item_buff[i1]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        delay_us(5);		
    }

    delay_us(20);
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
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler( prv );
    IWDG_SetReload( rlv );
    IWDG_ReloadCounter();
    IWDG_Enable();
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
