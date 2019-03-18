#include <stdlib.h>
#include <string.h>
#include "bsp_port.h"
#include "system_init.h"

#include "wcs_parser.h"
#include "dlink.h"
#include "uart.h"
#include "dbg_print.h"

uint8_t wcs_send_event_msg(eventMsg_t *eventMsg)
{
	uint8_t ret = 0;

	if (eventMsg == NULL)
	{
		dbg_print(PRINT_LEVEL_DEBUG, "Poll message ACK\r\n");
		return send_event_to_center(NULL, 0, EVENT_MSG_EMPTY);
	}

	switch (eventMsg->msgType)
	{
	case EVENT_MSG_CHAIN_UP:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_CHAIN_UP to WCS\r\n");
		ret = send_event_to_center(eventMsg->msg, BING_RFID_ID_LEN, EVENT_MSG_CHAIN_UP);
		break;
	case EVENT_MSG_CHAIN_DOWN:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_CHAIN_DOWN to WCS\r\n");
		ret = send_event_to_center(eventMsg->msg, LOW_RFID_ID_LEN, EVENT_MSG_CHAIN_DOWN);
		break;
	case EVENT_MSG_LED_STATUS:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_LED_STATUS to WCS\r\n");
		ret = send_event_to_center(eventMsg->msg, LED_STATUS_LEN, EVENT_MSG_LED_STATUS);
		break;
	case EVENT_MSG_KEY_DOWN:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_KEY_DOWN to WCS\r\n");
		ret = send_event_to_center(NULL, 0, EVENT_MSG_KEY_DOWN);
		break;
	case EVENT_MSG_ALARM:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_ALARM to WCS\r\n");
		ret = send_event_to_center(NULL, 0, EVENT_MSG_ALARM);
		break;
	case EVENT_MSG_START_STOP:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_START_STOP to WCS\r\n");
		ret = send_event_to_center(eventMsg->msg, MOTOR_STATUS_LEN, EVENT_MSG_START_STOP);
		break;
	case EVENT_MSG_EMER_STOP:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_EMER_STOP to WCS\r\n");
		ret = send_event_to_center(NULL, 0, EVENT_MSG_EMER_STOP);
		break;
	case EVENT_MSG_VALVE_CTRL:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_VALVE_CTRL to WCS\r\n");
		ret = send_event_to_center(eventMsg->msg, VALVE_STATUS_LEN, EVENT_MSG_VALVE_CTRL);
		break;
	case EVENT_MSG_CHAIN_DOWN_TIMEOUT:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload EVENT_MSG_CHAIN_DOWN_TIMEOUT to WCS\r\n");
		ret = send_event_to_center(NULL, 0, EVENT_MSG_CHAIN_DOWN_TIMEOUT);
		break;
	default:
		dbg_print(PRINT_LEVEL_DEBUG, "Upload unknown event to WCS\r\n");
		ret = send_event_to_center(NULL, 0, EVENT_MSG_EMPTY);
		break;
	}
	return ret;
}

uint8_t	wcs_add_chaindown_data(uint8_t *buf, uint8_t len)
{
	tagNode_t *tagNode = NULL;
	send_ack_to_center(NULL, 0, MT_WCS_SET_RFID_MSG);
	sys_mutex_lock(rfidDataMutex);
	if (node_list_find_tagid(&g_node_list, buf, LOW_RFID_ID_LEN) == NULL)
	{
		tagNode = pvPortMalloc(sizeof(tagNode_t));
		tagNode->msg = pvPortMalloc(sizeof(tagNode->msg));
		memset(tagNode->msg, 0x00, sizeof(tagNode->msg));
		tagNode->msg->aged = xTaskGetTickCount();
		memcpy(tagNode->msg->tagId, buf, sizeof(tagNode->msg->tagId));
		node_list_add_tail(&g_node_list, tagNode);
	}
	node_list_print(&g_node_list);
	sys_mutex_unlock(rfidDataMutex);
	return 0;
}

uint8_t wcs_remove_aged_node(void)
{
	uint32_t cur_ts;
	tagNode_t *tagNode = NULL;
	sys_mutex_lock(rfidDataMutex);
	cur_ts = xTaskGetTickCount();
	tagNode = node_list_find_aged(&g_node_list, cur_ts);
	if (tagNode != NULL)
		node_list_remove(&g_node_list, tagNode);
	node_list_print(&g_node_list);
	sys_mutex_unlock(rfidDataMutex);
	return 0;
}

uint8_t wcs_set_led_status(uint8_t *buf, uint8_t len)
{
	BaseType_t queue_send_ret;
	eventMsg_t eventMsg, tmpMsg;

	send_ack_to_center(NULL, 0, MT_WCS_SET_LED_MSG);

	if (buf[0] & TRICOLOR_LAMP_RED)
	{
		bsp_lamp_led_ctrl(TRICOLOR_LAMP_RED, TURN_ON);
	}
	else
	{
		bsp_lamp_led_ctrl(TRICOLOR_LAMP_RED, TURN_OFF);
	}

	if (buf[0] & TRICOLOR_LAMP_GREEN)
	{
		bsp_lamp_led_ctrl(TRICOLOR_LAMP_GREEN, TURN_ON);
	}
	else
	{
		bsp_lamp_led_ctrl(TRICOLOR_LAMP_GREEN, TURN_OFF);
	}

	if (buf[0] & TRICOLOR_LAMP_YELLOW)
	{
		bsp_lamp_led_ctrl(TRICOLOR_LAMP_YELLOW, TURN_ON);
	}
	else
	{
		bsp_lamp_led_ctrl(TRICOLOR_LAMP_YELLOW, TURN_OFF);
	}

	eventMsg.msgType = EVENT_MSG_LED_STATUS;
	eventMsg.msg[0] = buf[0] & 0x07;
	xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
	if(queue_send_ret == errQUEUE_FULL)
	{
		xQueueReceive(UpLoadEventMsgQueue, &tmpMsg, 0);
		xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
	}
	return 0;
}

uint8_t wcs_set_motor_status(uint8_t *buf, uint8_t len)
{
	BaseType_t queue_send_ret;
	eventMsg_t eventMsg, tmpMsg;

	//send_ack_to_center(NULL, 0, MT_WCS_SET_MOTO_MSG);
	if (g_configInfo.motor_auth == MOTOR_CTRL_AUTH_OFF)
		return 0;

	if (buf[0] == MOTOR_STATUS_START)
		bsp_motor_ctrl(TURN_ON);
	else
		bsp_motor_ctrl(TURN_OFF);

	eventMsg.msgType = EVENT_MSG_START_STOP;
	eventMsg.msg[0] = buf[0];
	xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
	if(queue_send_ret == errQUEUE_FULL)
	{
		xQueueReceive(UpLoadEventMsgQueue, &tmpMsg, 0);
		xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
	}
	return 0;
}

uint8_t wcs_valve_test_handle(uint8_t *buf, uint8_t len)
{
	BaseType_t queue_send_ret;
	eventMsg_t eventMsg, tmpMsg;

	send_ack_to_center(NULL, 0, MT_WCS_SET_VALVE_MSG);

	if (buf[0] == TURN_OFF)
		bsp_chain_close();
	if (buf[0] == TURN_ON)
		bsp_chain_open();

	eventMsg.msgType = EVENT_MSG_VALVE_CTRL;
	eventMsg.msg[0] = buf[0];
	xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
	if(queue_send_ret == errQUEUE_FULL)
	{
		xQueueReceive(UpLoadEventMsgQueue, &tmpMsg, 0);
		xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
	}
	return 0;
}


