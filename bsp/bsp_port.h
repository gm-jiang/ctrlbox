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

//USART1 MAX485 ctrl pin
#define WCS_485_RE_GPIO                          GPIOC
#define WCS_485_RE_GPIO_PIN                      GPIO_Pin_9


//UHF RFID detect sensor
#define UHFRFID_DETECT_SENSOR_GPIO			          GPIOC
#define UHFRFID_DETECT_SENSOR_GPIO_PIN	          GPIO_Pin_5

//chain down
#define CHAIN_DOWN_CTRL_GPIO						          GPIOC
#define CHAIN_DOWN_CTRL_GPIO_PIN				          GPIO_Pin_7

//chain motor
#define CHAIN_MOTOR_CTRL_GPIO					            GPIOC
#define CHAIN_MOTOR_CTRL_GPIO_PIN			          	GPIO_Pin_8

//the define of ctrlbox function
#define CTRLBOX_FUNCTION_BIT0_GPIO                GPIOC
#define CTRLBOX_FUNCTION_BIT0_GPIO_PIN            GPIO_Pin_12
#define CTRLBOX_FUNCTION_BIT1_GPIO                GPIOD
#define CTRLBOX_FUNCTION_BIT1_GPIO_PIN            GPIO_Pin_2
#define CTRLBOX_FUNCTION_BIT2_GPIO                GPIOB
#define CTRLBOX_FUNCTION_BIT2_GPIO_PIN            GPIO_Pin_3
#define CTRLBOX_FUNCTION_BIT3_GPIO                GPIOB
#define CTRLBOX_FUNCTION_BIT3_GPIO_PIN            GPIO_Pin_4

//the define of debug gpio
#define DEBUG_LED_GPIO		                    	  GPIOB
#define DEBUG_LED_RED_GPIO_PIN		                GPIO_Pin_5
#define DEBUG_LED_YEL_GPIO_PIN		                GPIO_Pin_6
#define DEBUG_LED_GRE_GPIO_PIN		                GPIO_Pin_7
//the define of red lamp gpio
#define TRICOLOR_LAMP_RED_GPIO		                GPIOB
#define TRICOLOR_LAMP_RED_GPIO_PIN		            GPIO_Pin_14
//the define of green lamp gpio
#define TRICOLOR_LAMP_GREEN_GPIO		              GPIOB
#define TRICOLOR_LAMP_GREEN_GPIO_PIN		          GPIO_Pin_15
//the define of yellow lamp gpio
#define TRICOLOR_LAMP_YELLOW_GPIO		              GPIOC
#define TRICOLOR_LAMP_YELLOW_GPIO_PIN		          GPIO_Pin_6

#define	TURN_ON                                   0x01
#define	TURN_OFF                                  0x00

//motor run status
#define MOTOR_STATUS_STOP                         0x00
#define MOTOR_STATUS_START                        0x01

//lamp define
#define TRICOLOR_LAMP_RED                         0x01
#define TRICOLOR_LAMP_GREEN                       0x02
#define TRICOLOR_LAMP_YELLOW                      0x04

/***************************INTTERRUPT PRIORITY DEFINE**********************************/
#define PREEMPTION_PRIORITY_WCS485										7
#define PREEMPTION_PRIORITY_UHFRFID										8
#define PREEMPTION_PRIORITY_STC												9
#define PREEMPTION_PRIORITY_DBG												10
#define PREEMPTION_PRIORITY_EMER_STOP_KEY							13
#define PREEMPTION_PRIORITY_REALEASE_KEY							14
#define PREEMPTION_PRIORITY_CHAIN_DOWN_FINISH_SENSOR	15

//release key
#define KEYIRQ                     								GPIO_Pin_7
#define KEYIRQ_GPIO                								GPIOA
#define KEYIRQ_EXTI                								EXTI_Line7
#define KEYIRQ_EXTI_PORT           								GPIO_PortSourceGPIOA
#define KEYIRQ_EXTI_PIN            								GPIO_PinSource7
#define KEYIRQ_EXTI_IRQn           								EXTI9_5_IRQn
#define KEYIRQ_EXTI_USEIRQ         								ENABLE
#define KEYIRQ_EXTI_NOIRQ          								DISABLE
#define port_GetEXT_IRQStatus()    								EXTI_GetITStatus(KEYIRQ_EXTI)
#define port_DisableEXT_IRQ()      								NVIC_DisableIRQ(KEYIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()       								NVIC_EnableIRQ(KEYIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()        								GPIO_ReadInputDataBit(KEYIRQ_GPIO, KEYIRQ)

//chain finished detect sensor
#define CHAIN_DOWN_FINISH_DETECTIRQ_GPIO					GPIOC
#define CHAIN_DOWN_FINISH_DETECTIRQ_GPIO_PIN			GPIO_Pin_4
#define CHAIN_DOWN_FINISH_DETECTIRQ_EXTI          EXTI_Line4
#define CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_PORT     GPIO_PortSourceGPIOC
#define CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_PIN      GPIO_PinSource4
#define CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_IRQn     EXTI4_IRQn
#define CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_USEIRQ   ENABLE
#define CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_NOIRQ    DISABLE
#define port_GetChainDownFinishEXT_IRQStatus()    EXTI_GetITStatus(CHAIN_DOWN_FINISH_DETECTIRQ_EXTI)
#define port_DisableChainDownFinishEXT_IRQ()     	NVIC_DisableIRQ(CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_IRQn)
#define port_EnableChainDownFinishEXT_IRQ()       NVIC_EnableIRQ(CHAIN_DOWN_FINISH_DETECTIRQ_EXTI_IRQn)
#define port_CheckChainDownFinishEXT_IRQ()        GPIO_ReadInputDataBit(CHAIN_DOWN_FINISH_DETECTIRQ_GPIO, CHAIN_DOWN_FINISH_DETECTIRQ_GPIO_PIN)

//emergency stop key
#define EMER_STOP_KEY_DETECTIRQ_GPIO							GPIOB
#define EMER_STOP_KEY_DETECTIRQ_GPIO_PIN					GPIO_Pin_0
#define EMER_STOP_KEY_DETECTIRQ_EXTI          		EXTI_Line0
#define EMER_STOP_KEY_DETECTIRQ_EXTI_PORT     		GPIO_PortSourceGPIOB
#define EMER_STOP_KEY_DETECTIRQ_EXTI_PIN      		GPIO_PinSource0
#define EMER_STOP_KEY_DETECTIRQ_EXTI_IRQn     		EXTI0_IRQn
#define EMER_STOP_KEY_DETECTIRQ_EXTI_USEIRQ   		ENABLE
#define EMER_STOP_KEY_DETECTIRQ_EXTI_NOIRQ    		DISABLE
#define port_GetEmerStopKeyEXT_IRQStatus()    		EXTI_GetITStatus(EMER_STOP_KEY_DETECTIRQ_EXTI)
#define port_DisableEmerStopKeyEXT_IRQ()      		NVIC_DisableIRQ(EMER_STOP_KEY_DETECTIRQ_EXTI_IRQn)
#define port_EnableEmerStopKeyEXT_IRQ()       		NVIC_EnableIRQ(EMER_STOP_KEY_DETECTIRQ_EXTI_IRQn)
#define port_CheckEmerStopKeyEXT_IRQ()        		GPIO_ReadInputDataBit(EMER_STOP_KEY_DETECTIRQ_GPIO, EMER_STOP_KEY_DETECTIRQ_GPIO_PIN)

void bsp_all_gpio_configuration(void);
void bsp_releasekey_init(void);
void bsp_emerstopkey_init(void);
void bsp_chaindown_finish_sensor_init(void);
void bsp_chaindown_finish_sensor_exit_enable(void);
void bsp_chaindown_finish_sensor_exit_disable(void);
void bsp_ctrlbox_mode_pin_init(void);
uint8_t bsp_switch_4bit_value(void);
void bsp_wcs_uart1_init(void);
void bsp_stc_uart2_init(void);
void bsp_uhf_uart3_init(void);
void bsp_dbg_uart4_init(void);
void bsp_mcu_485RE_init(void);
void bsp_status_led_init(void);
void bsp_status_red_led_set(uint8_t status);
void bsp_status_gre_led_set(uint8_t status);
void bsp_status_yel_led_set(uint8_t status);
void bsp_uhfrfid_detect_init(void);
void bsp_lamp_led_ctrl(uint8_t lamp, uint8_t status);
void bsp_lamp_led_init(void);
void bsp_chaindown_ctrl_init(void);
void bsp_motor_ctrl_init(void);
void bsp_motor_ctrl(uint8_t status);
void bsp_chain_open(void);
void bsp_chain_close(void);
void bsp_enable_485_pin(void);
void bsp_disable_485_pin(void);
void bsp_uart1_send(uint8_t *buf, uint16_t length);
void bsp_uart2_send(uint8_t *buf, uint16_t length);
void bsp_uart3_send(uint8_t *buf, uint16_t length);
void bsp_uart4_send(uint8_t *buf, uint16_t length);
void bsp_IWDG_init(uint8_t prv ,uint16_t rlv);
void bsp_IWDG_feed(void);

//flash operate function
uint16_t bsp_FLASH_ReadHalfWord(uint32_t address);
uint32_t bsp_FLASH_ReadWord(uint32_t address);
void bsp_FLASH_ReadMoreData(uint32_t startAddress, uint16_t *readData, uint16_t countToRead);
void bsp_FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);

#endif
