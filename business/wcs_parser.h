#ifndef _WCS_PARSER_H
#define _WCS_PARSER_H

#include <stdint.h>

//chain down status
#define CHAIN_DOWN_RFID_OPENED_QUEUE_NUM	5
#define CHAIN_DOWN_DETECT_TIMEOUT					(2*1000)
#define CHAIN_DOWN_DATA_VALID							1
#define CHAIN_DOWN_DATA_INVALID						0
#define CHAIN_DOWN_DATA_BUF_NUM						10
#define CHAIN_DOWN_DATA_AGE_MAX						0xff

#define CHAIN_DOWN_CONFIRM_TIMEOUT        5000

#define EVENT_MSG_DATA_LEN         16
#define BING_RFID_ID_LEN           16
#define LOW_RFID_ID_LEN            4
#define LED_STATUS_LEN             1
#define MOTOR_STATUS_LEN           1
#define VALVE_STATUS_LEN           1


#define RTN_SUCCESS																					0x00
#define RTN_FAIL																						0x01

typedef enum
{
	MT_WCS_POLL_MSG = 0x00,

	MT_WCS_SET_RFID_MSG = 0x81,
	MT_WCS_SET_LED_MSG,
	MT_WCS_SET_MOTO_MSG,
	MT_WCS_SET_VALVE_MSG,
	MT_WCS_RESET_MSG,

	MT_WCS_GET_VER_MSG = 0xB0,
	MT_WCS_SET_OTA_MSG,

	MT_WCS_SET_ADDR_MSG = 0xF1, //0xF1
	MT_WCS_SET_SN_MSG,
	MT_WCS_GET_ADDR_SN_MSG,
	MT_WCS_SET_CONFIG_INFO_MSG,
	MT_WCS_GET_CONFIG_INFO_MSG,
	MT_WCS_GET_SW_HW_VER_MSG,
	MT_WCS_GET_MOTOR_CTRL_AUTH_MSG,
	MT_WCS_SET_MOTOR_CTRL_AUTH_MSG,
} mtSysType_e;

//event type
typedef enum _eventMsgType_e {
	EVENT_MSG_EMPTY,
	EVENT_MSG_CHAIN_UP,
	EVENT_MSG_CHAIN_DOWN,
	EVENT_MSG_LED_STATUS,
	EVENT_MSG_KEY_DOWN,
	EVENT_MSG_ALARM,
	EVENT_MSG_START_STOP,
	EVENT_MSG_EMER_STOP,
	EVENT_MSG_VALVE_CTRL,

	EVENT_MSG_CHAIN_DOWN_TIMEOUT = 0x71
}eventMsgType_e;

//the queue of event
typedef struct _eventMsg_t {
	eventMsgType_e msgType;
	uint8_t msg[EVENT_MSG_DATA_LEN];
} eventMsg_t;

uint8_t wcs_send_event_msg(eventMsg_t *eventmsg);
uint8_t	wcs_add_chaindown_data(uint8_t *buf, uint8_t len);
uint8_t wcs_remove_aged_node(void);
uint8_t wcs_set_led_status(uint8_t *buf, uint8_t len);
uint8_t wcs_set_motor_status(uint8_t *buf, uint8_t len);
uint8_t wcs_valve_test_handle(uint8_t *buf, uint8_t len);

#endif

