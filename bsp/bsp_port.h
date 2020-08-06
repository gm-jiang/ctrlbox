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
#define FLASH_SIZE                 256          //MCU flash(k)
#if FLASH_SIZE < 256
  #define SECTOR_SIZE              1024    //Byte
#else
  #define SECTOR_SIZE              2048    //Byte
#endif

//the define of debug gpio
#define POWER_LED_GPIO		                    	  GPIOA
#define POWER_LED_GPIO_PIN		                    GPIO_Pin_2

#define	TURN_ON                                   0x01
#define	TURN_OFF                                  0x00


/***************************INTTERRUPT PRIORITY DEFINE**********************************/
#define PREEMPTION_PRIORITY_WCS485										7
#define PREEMPTION_PRIORITY_UHFRFID										8
#define PREEMPTION_PRIORITY_STC												9
#define PREEMPTION_PRIORITY_DBG												10
#define PREEMPTION_PRIORITY_EMER_STOP_KEY							13
#define PREEMPTION_PRIORITY_REALEASE_KEY							14
#define PREEMPTION_PRIORITY_CHAIN_DOWN_FINISH_SENSOR	15

void bsp_all_gpio_configuration(void);
void bsp_uart1_init(void);
void bsp_uart2_init(void);
void bsp_uart3_init(void);
void bsp_dbg_uart4_init(void);
void bsp_uart1_send(uint8_t *buf, uint16_t length);
void bsp_uart2_send(uint8_t *buf, uint16_t length);
void bsp_uart3_send(uint8_t *buf, uint16_t length);
void bsp_uart4_send(uint8_t *buf, uint16_t length);
void bsp_power_status_led_init(void);
void bsp_power_status_led_set(uint8_t status);
void bsp_IWDG_init(uint8_t prv ,uint16_t rlv);
void bsp_IWDG_feed(void);

//flash operate function
uint16_t bsp_FLASH_ReadHalfWord(uint32_t address);
uint32_t bsp_FLASH_ReadWord(uint32_t address);
void bsp_FLASH_ReadMoreData(uint32_t startAddress, uint16_t *readData, uint16_t countToRead);
void bsp_FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);

#endif
