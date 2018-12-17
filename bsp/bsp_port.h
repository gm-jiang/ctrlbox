#ifndef _BSP_PORT_H
#define _BSP_PORT_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/***********************************VERSION DEFINE***************************************/
#define HW_VERSION_STR		"---HW V1.1"
#define SW_VERSION_STR		"---SW V1.0"
#define VERISON_INFO_LEN	10

/***********************************INTTERRUPT PRIORITY DEFINE***************************************/
#define PREEMPTION_PRIORITY_WCS485										7
#define PREEMPTION_PRIORITY_UHFRFID										8
#define PREEMPTION_PRIORITY_STC												9
#define PREEMPTION_PRIORITY_EMER_STOP_KEY							15
#define PREEMPTION_PRIORITY_REALEASE_KEY							16
#define PREEMPTION_PRIORITY_CHAIN_DOWN_FINISH_SENSOR	17

/***********************************GPIO DEFINE***************************************/
//USART1
#define USART1_RX_PIN              GPIO_Pin_10
#define USART1_RX_SOURCE           GPIO_PinSource10
#define USART1_TX_PIN              GPIO_Pin_9
#define USART1_TX_SOURCE           GPIO_PinSource9
#define USART1_PORT                GPIOA
#define USART1_BAUDRATE            115200

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

/* not used
//UART4
#define UART4_RX_PIN               GPIO_Pin_11
#define UART4_RX_SOURCE            GPIO_PinSource11
#define UART4_TX_PIN               GPIO_Pin_10
#define UART4_TX_SOURCE            GPIO_PinSource10
#define UART4_PORT                 GPIOC
#define UART4_BAUDRATE             115200
*/

//USART1 MAX485 ctrl pin
#if 1
#define WCS_485_RE_GPIO						 GPIOC
#define WCS_485_RE_GPIO_PIN				 GPIO_Pin_9
#else
#define WCS_485_RE_GPIO						 GPIOA
#define WCS_485_RE_GPIO_PIN				 GPIO_Pin_8
#endif

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

//UHF RFID detect sensor
#define UHFRFID_DETECT_SENSOR_GPIO			GPIOC
#define UHFRFID_DETECT_SENSOR_GPIO_PIN	GPIO_Pin_5

//chain down
#define CHAIN_DOWN_CTRL_GPIO						GPIOC
#define CHAIN_DOWN_CTRL_GPIO_PIN				GPIO_Pin_7

//chain motor
#define CHAIN_MOTOR_CTRL_GPIO						GPIOC
#define CHAIN_MOTOR_CTRL_GPIO_PIN				GPIO_Pin_8

//the define of debug gpio
#define DEBUG_LED_GPIO		             	GPIOC
#define DEBUG_LED_GPIO_PIN	           	GPIO_Pin_3
//the define of red lamp gpio
#define TRICOLOR_LAMP_RED_GPIO		     	GPIOB
#define TRICOLOR_LAMP_RED_GPIO_PIN     	GPIO_Pin_14
//the define of green lamp gpio
#define TRICOLOR_LAMP_GREEN_GPIO	     	GPIOB
#define TRICOLOR_LAMP_GREEN_GPIO_PIN	 	GPIO_Pin_15
//the define of yellow lamp gpio
#define TRICOLOR_LAMP_YELLOW_GPIO		   	GPIOC
#define TRICOLOR_LAMP_YELLOW_GPIO_PIN	 	GPIO_Pin_6

/***********************************FRAME DEFINE***************************************/
//the max length of the frame buffer
#define UART_MSG_LEN			         50
//the max length of the uart msg queue
#define UART_QUEUE_LEN	           5

#define EVENT_MSG_DATA_LEN				 16
#define EVENT_MSG_QUEUE_LEN				 5

//lamp define
#define TRICOLOR_LAMP_RED		       0x01
#define TRICOLOR_LAMP_GREEN		     0x02
#define TRICOLOR_LAMP_YELLOW		   0x04
#define DEBUG_LED							     0x08
//key define
#define	KEY_STATUS								 0x10

/****************************************************/
#define SYN_STATE                0x00
#define STX_STATE                0x01
#define LEN_STATE                0x02
#define DAT_STATE                0x03
#define CRC_STATE                0x04

#define SYN_INDEX                   0
#define LEN_INDEX                   1
#define DAT_INDEX                   2
#define CRC_INDEX                   6
#define CRC_CHECK_LEN               6

//the length of uhf rfid tag id
#define MT_UHF_MSG_LEN			           24
//the length of low rfid tag id
#define MT_LOW_MSG_LEN			            4
//the length of uart msg 
#define MT_UART_MSG_LEN			           10

//the max length of the uart msg queue
#define MT_UART_QUEUE_LEN	              5

//the frame head of low rfid
#define MT_LOW_RFID_FRAME_HEAD	      0xFE
//the frame head of uhf rfid
#define MT_UHF_RFID_FRAME_HEAD	      0x02

typedef struct _bindMsgFrame_t {
	uint8_t low_flag;
	uint8_t uhf_flag;
	uint8_t uhf_rfid[MT_UHF_MSG_LEN/2];
	uint8_t low_rfid[MT_LOW_MSG_LEN];
}bindMsgFrame_t;
/******************************************************/

//UHF RFID-> USART3
#define UHF_RFID_FRAME_HEAD	       0xFE
#define UHF_RFID_FRAME_LEN_INDEX	 0x01
#define UHF_RFID_LABLE_DATA_LEN		 12

//STC -> USART2
#define STC_FRAME_HEAD		         0xFE
#define STC_FRAME_LEN_INDEX		     0x01
#define STC_RFID_ID_LEN						 4

//WCS  -> USART1
#define WCS_BROADCAST_ADDR				 0xFF
#define WCS_FRAME_HEAD_1		       0xAA
#define WCS_FRAME_HEAD_2		       0x55
#define WCS_FRAME_LEN_INDEX		     0x02
#define WCS_485_ADDR_INDEX				 0x03
#define WCS_485_FUNC_INDEX				 0x04
#define WCS_485_CONFIG_ADDR			 	 0xF1
#define WCS_485_CONFIG_SN			 	 	 0xF2
#define WCS_485_QUERY_ADDR_SN			 0xF3
#define WCS_485_CONFIG_CUSTOMER		 0xF4
#define WCS_485_QUERY_CUSTOMER		 0xF5
#define WCS_485_QUERY_HW_SW_VER		 0xF6
#define WCS_485_DEVICE_SN_LEN			 16
#define WCS_DECODE_BUF_LEN				 25
#define WCS_SEND_BUF_LEN					 30
#define WCS_ORDER_STATUS_LEN			 1
//WCS frame: function ID define
#define	WCS_FRAME_QUERY																			0x00
#define	WCS_FRAME_QUERY_EMPTY_ACK														0x00
#define	WCS_FRAME_QUERY_CHAIN_UP_ACK												0x01
#define	WCS_FRAME_QUERY_CHAIN_DOWN_ACK											0x02
#define WCS_FRAME_QUERY_ORDER_STATUS_ACK										0x03
#define	WCS_FRAME_QUERY_KEY_DOWN_ACK												0x04
#define	WCS_FRAME_QUERY_ALARM_ACK														0x05

#define	WCS_FRAME_CHAIN_DOWN_CTRL														0x02
#define WCS_FRAME_CHAIN_DOWN_CTRL_ACK												0xF2
#define	WCS_FRAME_ORDER_STATUS_CTRL													0x03
#define	WCS_FRAME_ORDER_STATUS_CTRL_ACK											0xF3

//chain down status
/*
#define CHAIN_DOWN_STATUS_FLAG_INDEX  		0x00
#define CHAIN_DOWN_RFID_ID_INDEX					0x01
#define CHAIN_DOWN_STATUS_DATA_INVALID		0x00
#define CHAIN_DOWN_STATUS_GET_RFID				0x01
#define CHAIN_DOWN_STATUS_MATCH_ID				0x02
#define CHAIN_DOWN_STATUS_FINISH					0x04
#define CHAIN_DOWN_STATUS_DATA_LEN				(STC_RFID_ID_LEN + 1)
#define CHAIN_DOWN_DATA_BUF_GROUP_NUM			5
*/
//#define CHAIN_DOWN_RFID_GETED_QUEUE_LEN			5
#define CHAIN_DOWN_RFID_OPENED_QUEUE_LEN	5
#define CHAIN_DOWN_DETECT_TIMEOUT					(2*1000)
#define CHAIN_DOWN_DATA_VALID							1
#define CHAIN_DOWN_DATA_INVALID						0
#define CHAIN_DOWN_DATA_BUF_LEN						CHAIN_DOWN_RFID_OPENED_QUEUE_LEN
#define CHAIN_DOWN_DATA_AGE_MAX						0xff

/***********************************FUNCTION RETURN DEFINE***************************************/
//function return value
#define RTN_SUCCESS																					0x00
#define RTN_FAIL																						0x01

/***********************************485 ADDRESS FLASH DEFINE***************************************/
//flash define
#define FLASH_SIZE                     256          //MCU flash(k)
#if FLASH_SIZE < 256
  #define SECTOR_SIZE           			 1024    //Byte
#else 
  #define SECTOR_SIZE           			 2048    //Byte
#endif
#define MCU_485_ADDR_BASE							 (0x08000000 + 127 * SECTOR_SIZE)
#define MCU_485_DEV_SN_BASE						 (0x08000000 + 126 * SECTOR_SIZE)
#define MCU_485_DEV_CUSTOMER_BASE			 (0x08000000 + 125 * SECTOR_SIZE)
/*
//uart frame type, unused
typedef enum _uartMsgType_e {
	MSG_STC_TYPE,
	MSG_WCS_TYPE,
	MSG_DEBUG_TYPE
}uartMsgType_e; 
*/
/***********************************UART FRAME DEFINE***************************************/
//the queue of uart frame
typedef struct _uartMsgFrame_t {
	//uartMsgType_e msgType;
	uint8_t msg[UART_MSG_LEN];
}uartMsgFrame_t;

//lamp on or off
typedef enum _statusCtrlType_e {
	TURN_ON,
	TURN_OFF
}statusCtrlType_e;

//wcs frame type
typedef enum _wcsFrameType_e {
	WCS_FRAME_QUERY_E,
	WCS_FRAME_QUERY_EMPTY_ACK_E,
	WCS_FRAME_QUERY_CHAIN_UP_ACK_E,
	WCS_FRAME_QUERY_CHAIN_DOWN_ACK_E,
	WCS_FRAME_QUERY_ORDER_STATUS_ACK_E,
	WCS_FRAME_QUERY_KEY_DOWN_ACK_E,
	WCS_FRAME_QUERY_ALARM_ACK_E,
	WCS_FRAME_CHAIN_DOWN_CTRL_E,
	WCS_FRAME_CHAIN_DOWN_CTRL_ACK_E,
	WCS_FRAME_ORDER_STATUS_CTRL_E,
	WCS_FRAME_ORDER_STATUS_CTRL_ACK_E,
	WCS_FRAME_INVALID_E
}wcsFrameType_e;

//event type
typedef enum _eventMsgType_e {
	EVENT_MSG_EMPTY,
	EVENT_MSG_CHAIN_UP,
	EVENT_MSG_CHAIN_DOWN,
	EVENT_MSG_ORDER_STATUS,
	EVENT_MSG_KEY_DOWN,
	EVENT_MSG_ALARM
}eventMsgType_e;

//the queue of event
typedef struct _eventMsgFrame_t {
	eventMsgType_e msgType;
	uint8_t msg[EVENT_MSG_DATA_LEN];
}eventMsgFrame_t;

typedef struct _chainDownMsgFrame_t {
	uint8_t msg[STC_RFID_ID_LEN];
}chainDownMsgFrame_t;

typedef struct _chainDownData_t {
	uint8_t valid;
	uint8_t age;
	uint8_t rfid[STC_RFID_ID_LEN];
}chainDownData_t;
/***********************************VARIABLE STATEMENT***************************************/
//the receive message queue of uart
extern QueueHandle_t wcs485RecvMsgQueue;
extern QueueHandle_t stcRecvMsgQueue;
extern QueueHandle_t uhfRFIDRecvMsgQueue;
//extern QueueHandle_t chainDownRfidGetedQueue;
extern QueueHandle_t chainDownRfidOpenedQueue;
//event queue
extern QueueHandle_t eventMsgQueue;
//event semaphore
extern SemaphoreHandle_t eventMsgSemaphore;
//chain down sensor detected semaphore
extern SemaphoreHandle_t chainDownDetectSemaphore;
//chain down rfid data semaphore
extern SemaphoreHandle_t chainDownDataSemaphore;
extern SemaphoreHandle_t uhfMsgSemaphore;

//extern uint32_t cpuID[3];

/*chain down stc rfid
		byte0:flag bit0[0,invalid][1,valid(get id)] bit1[0,invalid][1,valid(match id)] bit2[0,invalid][1,valid(chaindown finish)]
		byte1~byte4:rfid id data
*/
//extern uint8_t chainDownRFID[CHAIN_DOWN_STATUS_DATA_LEN * CHAIN_DOWN_DATA_BUF_GROUP_NUM];
extern chainDownData_t chainDownData[CHAIN_DOWN_DATA_BUF_LEN];
extern uint8_t mcu485Addr;
extern uint8_t mcuFuncConfig;

/*lamp status and key status
		bit0:red							[0,OFF][1,ON], 
    bit1:green						[0,OFF][1,ON], 
		bit2:yellow						[0,OFF][1,ON],
		bit3:debug_led				[0,OFF][1,ON],
		bit4:key_status				[0,UNVALID][1,VALID]
*/
extern uint8_t lampAndKeyStatus;

/***********************************FUNCTION STATEMENT***************************************/
//
void GPIO_Configuration(void);
void wcs485Uart_Init(void);
void stcUart_Init(void);
void uhfRfidUart_Init(void);
void releasekeyInt_Init(void);
void emerStopkeyInt_Init(void);
void chainDownFinishSensorInt_Init(void);
void mcu_485RE_Init(void);
void uhfRfidDetect_Init(void);
void lamp_Init(void);
void platform_Init(void);
void IWDG_Init(uint8_t prv ,uint16_t rlv);
void chainDown_CtrlInit(void);
void motor_CtrlInit(void);

void wcs485Uart_SendData(uint8_t *buf,uint16_t length);
void stcUart_SendData(uint8_t *buf,uint16_t length);
void uhfRfidUart_SendData(uint8_t *buf,uint16_t length);
void lamp_Ctrl(uint8_t lamp, statusCtrlType_e lampCtrl);
//use nop
void sleep_Us(uint32_t time_us);
//use freeRTOS
void sleep_Ms(unsigned int time_ms);

void wcs485_ChainOpen(void);
void wcs485_MotorCtrl(statusCtrlType_e statusCtrl);
uint8_t add_ChainDownData(uint8_t *buf, uint8_t len);
void wcs485_Update485AddrOrSNackSend(uint8_t *dataBuf, uint8_t dataLen);
uint8_t wcs485_QueryEmptyAckSend(void);
uint8_t wcs485_ChainUpAckSend(uint8_t *buf, uint8_t len);
uint8_t wcs485_ChainDownAckSend(uint8_t *buf, uint8_t len);
uint8_t wcs485_OrderStatusAckSend(uint8_t *buf, uint8_t len);
uint8_t wcs485_ChainKeyDownAckSend(void);
uint8_t wcs485_ChainAlarmAckSend(void);
uint8_t wcs485_ChainDownOrOrderStatusAckSend(void);
wcsFrameType_e wcs485_Decode(uint8_t *originBuf, uint8_t originLen, uint8_t *decodeBuf,uint8_t *decodeLen);

uint8_t wcs485_QueryCmd(void);
uint8_t wcs485_ChainDownCmd(uint8_t *dataBuf, uint8_t dataLen);
uint8_t wcs485_OrderStatusCmd(uint8_t *dataBuf, uint8_t dataLen);

//light sensor,after chain down
void sensor_ChainDownStatus(void);

//the check sum of stc frame
uint8_t check_Sum(uint8_t *uBuff, uint8_t uBuffLen);
//calc crc8,CRC8/MAXIM,X8+X5+X4+1
uint8_t cal_crc8(uint8_t *uBuff,uint32_t uBuffLen);

//watch dog
void IWDG_Feed(void);
//get cpuID for sn
//void get_CpuID(void);

uint8_t get_485Addr(void);
void set_485Addr(uint8_t addr);

uint8_t get_DevSn(uint8_t *snBuf, uint8_t *snLen);
uint8_t set_DevSn(uint8_t *snBuf, uint8_t snLen);

//get customer config
uint8_t get_CustomerConfig(void);
//set customer config
void set_CustomerConfig(uint8_t config);

#endif
