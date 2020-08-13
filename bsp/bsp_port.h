#ifndef _BSP_PORT_H
#define _BSP_PORT_H

#include "stm32f10x.h"

/***********************************GPIO DEFINE***************************************/

//USART1
#define USART1_RX_PIN              GPIO_Pin_10
#define USART1_RX_SOURCE           GPIO_PinSource10
#define USART1_TX_PIN              GPIO_Pin_9
#define USART1_TX_SOURCE           GPIO_PinSource9
#define USART1_PORT                GPIOA
#define USART1_BAUDRATE            9600

//USART2
#define USART2_RX_PIN              GPIO_Pin_3
#define USART2_RX_SOURCE           GPIO_PinSource3
#define USART2_TX_PIN              GPIO_Pin_2
#define USART2_TX_SOURCE           GPIO_PinSource2
#define USART2_PORT                GPIOA
#define USART2_BAUDRATE            9600

//USART3
#define USART3_RX_PIN              GPIO_Pin_11
#define USART3_RX_SOURCE           GPIO_PinSource11
#define USART3_TX_PIN              GPIO_Pin_10
#define USART3_TX_SOURCE           GPIO_PinSource10
#define USART3_PORT                GPIOB
#define USART3_BAUDRATE            115200

//UART4
#define UART4_RX_PIN               GPIO_Pin_11
#define UART4_RX_SOURCE            GPIO_PinSource11
#define UART4_TX_PIN               GPIO_Pin_10
#define UART4_TX_SOURCE            GPIO_PinSource10
#define UART4_PORT                 GPIOC
#define UART4_BAUDRATE             115200

//flash define
#define FLASH_SIZE                 256      //MCU flash(k)
#if FLASH_SIZE < 256
  #define SECTOR_SIZE              1024    //Byte
#else
  #define SECTOR_SIZE              2048    //Byte
#endif

//the define of debug gpio
#define POWER_LED_GPIO              GPIOA
#define POWER_LED_GPIO_PIN          GPIO_Pin_2

#define	TURN_ON                     0x01
#define	TURN_OFF                    0x00


#define KEY1_GPIO_PORT    	GPIOB
#define KEY1_GPIO_CLK 	    RCC_APB2Periph_GPIOB
#define KEY1_GPIO_PIN       GPIO_Pin_1

#define KEY2_GPIO_PORT    	GPIOB
#define KEY2_GPIO_CLK 	    RCC_APB2Periph_GPIOB
#define KEY2_GPIO_PIN       GPIO_Pin_10

#define KEY3_GPIO_PORT    	GPIOB	
#define KEY3_GPIO_CLK 	    RCC_APB2Periph_GPIOB
#define KEY3_GPIO_PIN       GPIO_Pin_11

#define KEY4_GPIO_PORT      GPIOB
#define KEY4_GPIO_CLK       RCC_APB2Periph_GPIOB
#define KEY4_GPIO_PIN       GPIO_Pin_12

//key1
#define KEY1_DETECTIRQ_GPIO                 GPIOB
#define KEY1_DETECTIRQ_GPIO_PIN             GPIO_Pin_1
#define KEY1_DETECTIRQ_EXTI                 EXTI_Line1
#define KEY1_DETECTIRQ_EXTI_PORT            GPIO_PortSourceGPIOB
#define KEY1_DETECTIRQ_EXTI_PIN             GPIO_PinSource1
#define KEY1_DETECTIRQ_EXTI_IRQn            EXTI1_IRQn
#define KEY1_DETECTIRQ_EXTI_USEIRQ          ENABLE
#define KEY1_DETECTIRQ_EXTI_NOIRQ           DISABLE
#define port_GetKey1EXT_IRQStatus()         EXTI_GetITStatus(KEY1_DETECTIRQ_EXTI)
#define port_DisableKey1EXT_IRQ()           NVIC_DisableIRQ(KEY1_DETECTIRQ_EXTI_IRQn)
#define port_EnableKey1EXT_IRQ()            NVIC_EnableIRQ(KEY1_DETECTIRQ_EXTI_IRQn)
#define port_CheckKey1EXT_IRQ()             GPIO_ReadInputDataBit(KEY1_DETECTIRQ_GPIO, KEY1_DETECTIRQ_GPIO_PIN)

//key1
#define KEY1_DETECTIRQ_GPIO                 GPIOB
#define KEY1_DETECTIRQ_GPIO_PIN             GPIO_Pin_1
#define KEY1_DETECTIRQ_EXTI                 EXTI_Line1
#define KEY1_DETECTIRQ_EXTI_PORT            GPIO_PortSourceGPIOB
#define KEY1_DETECTIRQ_EXTI_PIN             GPIO_PinSource1
#define KEY1_DETECTIRQ_EXTI_IRQn            EXTI1_IRQn
#define KEY1_DETECTIRQ_EXTI_USEIRQ          ENABLE
#define KEY1_DETECTIRQ_EXTI_NOIRQ           DISABLE
#define port_GetKey1EXT_IRQStatus()         EXTI_GetITStatus(KEY1_DETECTIRQ_EXTI)
#define port_DisableKey1EXT_IRQ()           NVIC_DisableIRQ(KEY1_DETECTIRQ_EXTI_IRQn)
#define port_EnableKey1EXT_IRQ()            NVIC_EnableIRQ(KEY1_DETECTIRQ_EXTI_IRQn)
#define port_CheckKey1EXT_IRQ()             GPIO_ReadInputDataBit(KEY1_DETECTIRQ_GPIO, KEY1_DETECTIRQ_GPIO_PIN)

//key2
#define KEY2_DETECTIRQ_GPIO                 GPIOB
#define KEY2_DETECTIRQ_GPIO_PIN             GPIO_Pin_10
#define KEY2_DETECTIRQ_EXTI                 EXTI_Line10
#define KEY2_DETECTIRQ_EXTI_PORT            GPIO_PortSourceGPIOB
#define KEY2_DETECTIRQ_EXTI_PIN             GPIO_PinSource10
#define KEY2_DETECTIRQ_EXTI_IRQn            EXTI15_10_IRQn
#define KEY2_DETECTIRQ_EXTI_USEIRQ          ENABLE
#define KEY2_DETECTIRQ_EXTI_NOIRQ           DISABLE
#define port_GetKey2EXT_IRQStatus()         EXTI_GetITStatus(KEY2_DETECTIRQ_EXTI)
#define port_DisableKey2EXT_IRQ()           NVIC_DisableIRQ(KEY2_DETECTIRQ_EXTI_IRQn)
#define port_EnableKey2EXT_IRQ()            NVIC_EnableIRQ(KEY2_DETECTIRQ_EXTI_IRQn)
#define port_CheckKey2EXT_IRQ()             GPIO_ReadInputDataBit(KEY2_DETECTIRQ_GPIO, KEY2_DETECTIRQ_GPIO_PIN)

//key3
#define KEY3_DETECTIRQ_GPIO                 GPIOB
#define KEY3_DETECTIRQ_GPIO_PIN             GPIO_Pin_11
#define KEY3_DETECTIRQ_EXTI                 EXTI_Line11
#define KEY3_DETECTIRQ_EXTI_PORT            GPIO_PortSourceGPIOB
#define KEY3_DETECTIRQ_EXTI_PIN             GPIO_PinSource11
#define KEY3_DETECTIRQ_EXTI_IRQn            EXTI15_10_IRQn
#define KEY3_DETECTIRQ_EXTI_USEIRQ          ENABLE
#define KEY3_DETECTIRQ_EXTI_NOIRQ           DISABLE
#define port_GetKey3EXT_IRQStatus()         EXTI_GetITStatus(KEY3_DETECTIRQ_EXTI)
#define port_DisableKey3EXT_IRQ()           NVIC_DisableIRQ(KEY3_DETECTIRQ_EXTI_IRQn)
#define port_EnableKey3EXT_IRQ()            NVIC_EnableIRQ(KEY3_DETECTIRQ_EXTI_IRQn)
#define port_CheckKey3EXT_IRQ()             GPIO_ReadInputDataBit(KEY3_DETECTIRQ_GPIO, KEY3_DETECTIRQ_GPIO_PIN)

//key4
#define KEY4_DETECTIRQ_GPIO                 GPIOB
#define KEY4_DETECTIRQ_GPIO_PIN             GPIO_Pin_12
#define KEY4_DETECTIRQ_EXTI                 EXTI_Line12
#define KEY4_DETECTIRQ_EXTI_PORT            GPIO_PortSourceGPIOB
#define KEY4_DETECTIRQ_EXTI_PIN             GPIO_PinSource12
#define KEY4_DETECTIRQ_EXTI_IRQn            EXTI15_10_IRQn
#define KEY4_DETECTIRQ_EXTI_USEIRQ          ENABLE
#define KEY4_DETECTIRQ_EXTI_NOIRQ           DISABLE
#define port_GetKey4EXT_IRQStatus()         EXTI_GetITStatus(KEY4_DETECTIRQ_EXTI)
#define port_DisableKey4EXT_IRQ()           NVIC_DisableIRQ(KEY4_DETECTIRQ_EXTI_IRQn)
#define port_EnableKey4EXT_IRQ()            NVIC_EnableIRQ(KEY4_DETECTIRQ_EXTI_IRQn)
#define port_CheckKey4EXT_IRQ()             GPIO_ReadInputDataBit(KEY4_DETECTIRQ_GPIO, KEY4_DETECTIRQ_GPIO_PIN)

/***************************INTTERRUPT PRIORITY DEFINE**********************************/
#define PREEMPTION_PRIORITY_WCS485                      7
#define PREEMPTION_PRIORITY_UHFRFID                     8
#define PREEMPTION_PRIORITY_STC                         9
#define PREEMPTION_PRIORITY_DBG                         10
#define PREEMPTION_PRIORITY_KEY                         13
#define PREEMPTION_PRIORITY_REALEASE_KEY                14
#define PREEMPTION_PRIORITY_CHAIN_DOWN_FINISH_SENSOR    15




/**************************************CMT2300  ??h·418Mhz***********************************************/
#define CMT_CSB_GPIO                GPIOC
#define CMT_CSB_GPIO_PIN            GPIO_Pin_0

#define CMT_SDIO_GPIO               GPIOC
#define CMT_SDIO_GPIO_PIN           GPIO_Pin_1

#define CMT_SCLK_GPIO               GPIOC
#define CMT_SCLK_GPIO_PIN           GPIO_Pin_2

#define CMT_GPIO3_GPIO              GPIOC
#define CMT_GPIO3_GPIO_PIN          GPIO_Pin_3

/**************************************CMT2300 315Mhz ???·***********************************************/
#define CMT_CSB_GPIO_RF315                GPIOB
#define CMT_CSB_GPIO_RF315_PIN            GPIO_Pin_6

#define CMT_SDIO_GPIO_RF315               GPIOB
#define CMT_SDIO_GPIO_RF315_PIN           GPIO_Pin_7
        
#define CMT_SCLK_GPIO_RF315               GPIOB
#define CMT_SCLK_GPIO_RF315_PIN           GPIO_Pin_8

#define CMT_GPIO3_GPIO_RF315              GPIOB
#define CMT_GPIO3_GPIO_RF315_PIN          GPIO_Pin_9


/**************************************CMT2300 430Mhz ????·***********************************************/
#define CMT_CSB_GPIO_RF330                GPIOC
#define CMT_CSB_GPIO_RF330_PIN            GPIO_Pin_9

#define CMT_SDIO_GPIO_RF330               GPIOA
#define CMT_SDIO_GPIO_RF330_PIN           GPIO_Pin_8
        
#define CMT_SCLK_GPIO_RF330               GPIOA
#define CMT_SCLK_GPIO_RF330_PIN           GPIO_Pin_11

#define CMT_GPIO3_GPIO_RF330              GPIOA
#define CMT_GPIO3_GPIO_RF330_PIN          GPIO_Pin_12		

/**************************************CMT2300 433.092Mhz ????·***********************************************/
#define CMT_CSB_GPIO_RF433                GPIOB
#define CMT_CSB_GPIO_RF433_PIN            GPIO_Pin_15

#define CMT_SDIO_GPIO_RF433               GPIOC
#define CMT_SDIO_GPIO_RF433_PIN           GPIO_Pin_6
        
#define CMT_SCLK_GPIO_RF433               GPIOC
#define CMT_SCLK_GPIO_RF433_PIN           GPIO_Pin_7

#define CMT_GPIO3_GPIO_RF433              GPIOC
#define CMT_GPIO3_GPIO_RF433_PIN          GPIO_Pin_8		
		

#define BUZZER_GPIO                 GPIOA
#define BUZZER_GPIO_PIN             GPIO_Pin_10


#define SET_GPIO_OUT(x)             GPIO_Pin_Setting(x, x##_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP)
#define SET_GPIO_IN(x)              GPIO_Pin_Setting(x, x##_PIN, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING)
#define SET_GPIO_OD(x)              GPIO_Pin_Setting(x, x##_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_OD)
#define SET_GPIO_AIN(x)             GPIO_Pin_Setting(x, x##_PIN, GPIO_Speed_50MHz, GPIO_Mode_AIN)
#define SET_GPIO_AFOUT(x)           GPIO_Pin_Setting(x, x##_PIN, GPIO_Speed_50MHz, GPIO_Mode_AF_PP)
#define SET_GPIO_AFOD(x)            GPIO_Pin_Setting(x, x##_PIN, GPIO_Speed_50MHz, GPIO_Mode_AF_OD)
#define SET_GPIO_H(x)               (x->BSRR = x##_PIN) //GPIO_SetBits(x, x##_PIN)
#define SET_GPIO_L(x)               (x->BRR  = x##_PIN) //GPIO_ResetBits(x, x##_PIN)
#define READ_GPIO_PIN(x)            (((x->IDR & x##_PIN)!=Bit_RESET) ?1 :0) //GPIO_ReadInputDataBit(x, x##_PIN) 


/***************************INTTERRUPT PRIORITY DEFINE**********************************/
#define PREEMPTION_PRIORITY_WCS485                      7
#define PREEMPTION_PRIORITY_UHFRFID                     8
#define PREEMPTION_PRIORITY_STC                         9
#define PREEMPTION_PRIORITY_DBG                         10
#define PREEMPTION_PRIORITY_EMER_STOP_KEY               13
#define PREEMPTION_PRIORITY_REALEASE_KEY                14
#define PREEMPTION_PRIORITY_CHAIN_DOWN_FINISH_SENSOR	15

void bsp_all_gpio_configuration(void);
void bsp_gpio_config(void);
void bsp_uart1_init(void);
void bsp_uart2_init(void);
void bsp_uart3_init(void);
void bsp_dbg_uart4_init(void);
void GPIO_Pin_Setting(GPIO_TypeDef *gpio, uint16_t nPin, GPIOSpeed_TypeDef speed, GPIOMode_TypeDef mode);
void bsp_uart1_send(uint8_t *buf, uint16_t length);
void bsp_uart2_send(uint8_t *buf, uint16_t length);
void bsp_uart3_send(uint8_t *buf, uint16_t length);
void bsp_uart4_send(uint8_t *buf, uint16_t length);
void USART1_Send(void);
void bsp_power_status_led_init(void);
void bsp_power_status_led_set(uint8_t status);
//void bsp_key_init(void);
void bsp_key1_init(void);
void bsp_key2_init(void);
void bsp_key3_init(void);
void bsp_key4_init(void);
void bsp_IWDG_init(uint8_t prv ,uint16_t rlv);
void bsp_IWDG_feed(void);

//flash operate function
uint16_t bsp_FLASH_ReadHalfWord(uint32_t address);
uint32_t bsp_FLASH_ReadWord(uint32_t address);
void bsp_FLASH_ReadMoreData(uint32_t startAddress, uint16_t *readData, uint16_t countToRead);
void bsp_FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);

#endif
