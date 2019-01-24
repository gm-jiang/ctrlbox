#ifndef _WCS_PARSER_H
#define _WCS_PARSER_H

#include <stdint.h>
#include "bsp_port.h"

//chain down status
#define CHAIN_DOWN_RFID_OPENED_QUEUE_NUM	5
#define CHAIN_DOWN_DETECT_TIMEOUT					(2*1000)
#define CHAIN_DOWN_DATA_VALID							1
#define CHAIN_DOWN_DATA_INVALID						0
#define CHAIN_DOWN_DATA_BUF_LEN						15
#define CHAIN_DOWN_DATA_AGE_MAX						0xff

#define EVENT_MSG_DATA_LEN				 16
#define EVENT_MSG_QUEUE_NUM				 10

#define MT_WCS_GET_VER_MSG         0x10
#define MT_WCS_SET_OTA_MSG         0x11

//the queue of uart frame
typedef struct _uartMsgFrame_t {
	uint8_t msg[128];
}uartMsgFrame_t;

//wcs frame type
typedef enum _wcsFrameType_e {
	WCS_FRAME_QUERY_E,
	WCS_FRAME_QUERY_EMPTY_ACK_E,
	WCS_FRAME_QUERY_CHAIN_UP_ACK_E,
	WCS_FRAME_QUERY_CHAIN_DOWN_ACK_E,
	WCS_FRAME_QUERY_ORDER_STATUS_ACK_E,
	WCS_FRAME_QUERY_KEY_DOWN_ACK_E,
	WCS_FRAME_QUERY_ALARM_ACK_E,
	WCS_FRAME_QUERY_START_STOP_ACK_E,
	WCS_FRAME_QUERY_EMER_STOP_ACK_E,
	WCS_FRAME_CHAIN_DOWN_CTRL_E,
	WCS_FRAME_CHAIN_DOWN_CTRL_ACK_E,
	WCS_FRAME_ORDER_STATUS_CTRL_E,
	WCS_FRAME_ORDER_STATUS_CTRL_ACK_E,
	WCS_FRAME_MOTOR_START_STOP_CTRL_E,
	WCS_FRAME_MOTOR_START_STOP_CTRL_ACK_E,
	WCS_FRAME_INVALID_E
}wcsFrameType_e;

//event type
typedef enum _eventMsgType_e {
	EVENT_MSG_EMPTY,
	EVENT_MSG_CHAIN_UP,
	EVENT_MSG_CHAIN_DOWN,
	EVENT_MSG_ORDER_STATUS,
	EVENT_MSG_KEY_DOWN,
	EVENT_MSG_ALARM,
	EVENT_MSG_START_STOP,
	EVENT_MSG_EMER_STOP
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

extern chainDownData_t g_chainDownData[CHAIN_DOWN_DATA_BUF_LEN];

uint8_t add_ChainDownData(uint8_t *buf, uint8_t len);
void wcs485_Update485AddrOrSNackSend(uint8_t *dataBuf, uint8_t dataLen);
void wcs485_ChainOpen(void);
void wcs485_MotorCtrl(statusCtrlType_e statusCtrl);
uint8_t wcs485_Encode(wcsFrameType_e frameType, uint8_t *dataBuf, uint8_t dataLen, uint8_t *encodeBuf, uint8_t *encodeLen);
wcsFrameType_e wcs485_Decode(uint8_t *originBuf, uint8_t originLen, uint8_t *decodeBuf, uint8_t *decodeLen);
uint8_t wcs485_QueryEmptyAckSend(void);
uint8_t wcs485_ChainUpAckSend(uint8_t *buf, uint8_t len);
uint8_t wcs485_ChainDownAckSend(uint8_t *buf, uint8_t len);
uint8_t wcs485_OrderStatusAckSend(uint8_t *buf, uint8_t len);
uint8_t wcs485_ChainKeyDownAckSend(void);
uint8_t wcs485_ChainAlarmAckSend(void);
uint8_t wcs485_QueryStartStopAckSend(uint8_t *buf, uint8_t len);\
uint8_t wcs485_QueryEmerStopAckSend(void);
uint8_t wcs485_ChainDownCmdAckSend(void);
uint8_t wcs485_OrderStatusCmdAckSend(void);
uint8_t wcs485_MotorStartStopCmdAckSend(void);
uint8_t wcs485_QueryCmd(void);
uint8_t wcs485_ChainDownCmd(uint8_t *dataBuf, uint8_t dataLen);
uint8_t wcs485_OrderStatusCmd(uint8_t *dataBuf, uint8_t dataLen);
uint8_t wcs485_MotorStartStopCmd(uint8_t *dataBuf, uint8_t dataLen);
#endif
