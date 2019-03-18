#include <string.h>
#include "FreeRTOS.h"
#include "bsp_port.h"
#include "mt_common.h"
#include "dbg_print.h"
#include "system_init.h"
#include "wcs_parser.h"
#include "dlink.h"

void switch_control(void)
{
	bsp_chain_open();
	vTaskDelay(g_configInfo.ctrlLogic.valve_ctrl_delay_ms);
	bsp_chain_close();
}

void chain_down_msg_process(uint8_t *payload, uint8_t payload_len)
{
	BaseType_t sem_ret, queue_ret;
	eventMsg_t eventMsg, tmpMsg;
	tagNode_t *tagNode = NULL;

	sys_mutex_lock(rfidDataMutex);
	tagNode = node_list_find_tagid(&g_node_list, payload, LOW_RFID_ID_LEN);
	if (tagNode != NULL)
	{
		/*enable interrupt of chaindown finsh sensor*/
		bsp_chaindown_finish_sensor_exit_enable();
		/*wait one second for confirm infomation that the packet arrived the destination stage*/
		sem_ret = xSemaphoreTake(chainDownDetectSemaphore, CHAIN_DOWN_CONFIRM_TIMEOUT);
		if (sem_ret == pdTRUE)
		{
			/*turn on switch that let packet get out the damned chain*/
			switch_control();
			eventMsg.msgType = EVENT_MSG_CHAIN_DOWN;
		}
		else
			eventMsg.msgType = EVENT_MSG_CHAIN_DOWN_TIMEOUT;
		/*disable interrupt of chaindown finsh sensor*/
		bsp_chaindown_finish_sensor_exit_disable();
		memcpy(eventMsg.msg, tagNode->msg->tagId, LOW_RFID_ID_LEN);
		queue_ret = xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
		if (queue_ret == errQUEUE_FULL)
		{
			xQueueReceive(UpLoadEventMsgQueue, &tmpMsg, 0);
			xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
		}
		node_list_remove(&g_node_list, tagNode); //fix it-> node_list_remove(&g_node_list, tagNode)
	}
	sys_mutex_unlock(rfidDataMutex);
}

void trigger_uhf_rfid_process(uint8_t *buf, uint8_t len)
{
	BaseType_t sem_ret, queue_ret;

	sem_ret = xSemaphoreTake(chainDownDetectSemaphore, 0);
	if (sem_ret == pdTRUE)
	{
		queue_ret = xQueueSend(LowRfidMsgQueue, buf, 0);
		if(queue_ret == errQUEUE_FULL)
		{
			dbg_print(PRINT_LEVEL_DEBUG, "%s[%d] error\r\n", __FILE__, __LINE__);
		}
	}
}

void uhfrfid_send_cmd(uint16_t delay_ms)
{
	uint8_t cmd[2] = {'K', 'I'};

	bsp_uart3_send(&cmd[0], 1);
	vTaskDelay(delay_ms);
	bsp_uart3_send(&cmd[1], 1);
}

void uhfrfid_message_process(uint8_t *buf, uint8_t len)
{
	BaseType_t queue_ret;
	uint8_t recv_uhf_msg[UHFRFID_LEN] = {0};
	uint8_t bind_msg[BING_RFID_ID_LEN] = {0};
	eventMsg_t eventMsg, tmpMsg;

	uhfrfid_send_cmd(500);
	queue_ret = xQueueReceive(UART3RecvMsgQueue, recv_uhf_msg, 2000);
	if (queue_ret == pdTRUE)
	{
		mt_uhfrfid_convert(recv_uhf_msg, UHFRFID_LEN);
		memcpy(&bind_msg[0], recv_uhf_msg, UHFRFID_DATA_LEN);
		memcpy(&bind_msg[UHFRFID_DATA_LEN], buf, len);
		memcpy(eventMsg.msg, bind_msg, BING_RFID_ID_LEN);
		eventMsg.msgType = EVENT_MSG_CHAIN_UP;
		queue_ret = xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
		if(queue_ret == errQUEUE_FULL)
		{
			xQueueReceive(UpLoadEventMsgQueue, &tmpMsg, 0);
			xQueueSend(UpLoadEventMsgQueue, &eventMsg, 0);
		}
	}
}

void lowrfid_check_process(uint8_t *buf, uint8_t len)
{
	#if 0
	if (1/**/)
	{
		eventMsg.msgType = EVENT_MSG_ALARM;
		queue_send_ret = xQueueSend(eventMsgQueue, &eventMsg, 0);
		if(queue_send_ret == errQUEUE_FULL)
		{
			xQueueReceive(eventMsgQueue, &tmpMsg, 0);
			xQueueSend(eventMsgQueue, &eventMsg, 0);
		}
	}
	#endif
}

void debug_mode_process(uint8_t *buf, uint8_t len)
{
}
