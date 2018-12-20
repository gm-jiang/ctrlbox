#include "user_task.h"
#include "bsp_port.h"
#include "uhf_rfid.h"
#include "mt_common.h"

void lamp_task(void *pvParameters)
{
	while(1)
	{
		lamp_Ctrl(DEBUG_LED, TURN_ON);
		vTaskDelay(500);
		lamp_Ctrl(DEBUG_LED, TURN_OFF);
		vTaskDelay(500);
	}
}

void wcs485_msg_task(void *pvParameters)
{
	BaseType_t queue_recv_ret;
	uartMsgFrame_t recv_msg;
	uint8_t i, ret;
	wcsFrameType_e frameType;
	uint8_t decodeBuf[WCS_DECODE_BUF_LEN];
	uint8_t decodeLen = 0;
	
	while(1)
	{
		queue_recv_ret = xQueueReceive(wcs485RecvMsgQueue, &recv_msg, portMAX_DELAY);
		if(queue_recv_ret == pdTRUE)
		{
			#if 1
			dbg_Print(PRINT_LEVEL_ERROR, "wcs485_msg_task recv:");
			for(i = 0;i < recv_msg.msg[WCS_FRAME_LEN_INDEX] + 3;i++)
			{
				dbg_Print(PRINT_LEVEL_ERROR, "0x%02x", recv_msg.msg[i]);
			}
			dbg_Print(PRINT_LEVEL_ERROR, "\n");
			#endif
			frameType = wcs485_Decode(recv_msg.msg, recv_msg.msg[WCS_FRAME_LEN_INDEX] + 3, decodeBuf, &decodeLen);
			if(frameType != WCS_FRAME_INVALID_E)
			{
				switch(frameType)
				{
					case WCS_FRAME_QUERY_E:
						ret = wcs485_QueryCmd();
						if(ret != RTN_SUCCESS)
						{
							continue;
						}
						break;
					case WCS_FRAME_CHAIN_DOWN_CTRL_E:
						ret = wcs485_ChainDownCmd(decodeBuf, decodeLen);
						if(ret != RTN_SUCCESS)
						{
							continue;
						}
						break;
					case WCS_FRAME_ORDER_STATUS_CTRL_E:
						ret = wcs485_OrderStatusCmd(decodeBuf, decodeLen);
						if(ret != RTN_SUCCESS)
						{
							continue;
						}
						break;
					case WCS_FRAME_MOTOR_START_STOP_CTRL_E:
						ret = wcs485_MotorStartStopCmd(decodeBuf, decodeLen);
						if(ret != RTN_SUCCESS)
						{
							continue;
						}
						break;
					default:
						break;
				}
			}
			
			if(recv_msg.msg[WCS_485_ADDR_INDEX] == WCS_BROADCAST_ADDR)
			{
				if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_CONFIG_ADDR)
				{
					set_485Addr(recv_msg.msg[WCS_485_FUNC_INDEX + 1]);
					mcu485Addr = get_485Addr();
					decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
					decodeBuf[1] = mcu485Addr;				
					wcs485_Update485AddrOrSNackSend(decodeBuf, 2);
				}
				else if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_CONFIG_SN)
				{
					ret = set_DevSn(&(recv_msg.msg[WCS_485_FUNC_INDEX + 1]), WCS_485_DEVICE_SN_LEN);
					if(ret != RTN_SUCCESS)
					{
						continue;
					}
					decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
					ret = get_DevSn(&decodeBuf[1], &decodeLen);	
					if((ret != RTN_SUCCESS) || (decodeLen != WCS_485_DEVICE_SN_LEN))
					{
						continue;
					}					
					wcs485_Update485AddrOrSNackSend(decodeBuf, WCS_485_DEVICE_SN_LEN + 1);
				}
				else if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_QUERY_ADDR_SN)
				{
					decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
					decodeBuf[1] = get_485Addr();
					ret = get_DevSn(&decodeBuf[2], &decodeLen);	
					if((ret != RTN_SUCCESS) || (decodeLen != WCS_485_DEVICE_SN_LEN))
					{
						continue;
					}
					wcs485_Update485AddrOrSNackSend(decodeBuf, WCS_485_DEVICE_SN_LEN + 2);
				}
			}
			
			if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_CONFIG_CUSTOMER)
			{
				set_CustomerConfig(recv_msg.msg[WCS_485_FUNC_INDEX + 1]);
				mcuFuncConfig = get_CustomerConfig();
				decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
				decodeBuf[1] = mcuFuncConfig;				
				wcs485_Update485AddrOrSNackSend(decodeBuf, 2);
			}
			else if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_QUERY_CUSTOMER)
			{
				decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
				decodeBuf[1] = get_CustomerConfig();				
				wcs485_Update485AddrOrSNackSend(decodeBuf, 2);
			}
			else if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_QUERY_HW_SW_VER)
			{
				decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
				memcpy(&decodeBuf[1], (uint8_t *)&HW_VERSION_STR, VERISON_INFO_LEN);
				memcpy(&decodeBuf[1 + VERISON_INFO_LEN], (uint8_t *)&SW_VERSION_STR, VERISON_INFO_LEN);
				wcs485_Update485AddrOrSNackSend(decodeBuf, VERISON_INFO_LEN*2 + 1);
			}
			
		}
		else
		{
			dbg_Print(PRINT_LEVEL_ERROR, "read wcs485_msg recv queue failed\n");
		}
	}
}

void stc_msg_task(void *pvParameters)
{
	uint8_t i;
	BaseType_t queue_recv_ret, queue_send_ret;
	uint8_t recv_msg[MT_UART_MSG_LEN] = {0};
	chainDownMsgFrame_t chainDownMsg, tmpChainDownMsg;
	eventMsgFrame_t eventMsg, tmpMsg;

	while(1)
	{
		queue_recv_ret = xQueueReceive(stcRecvMsgQueue, recv_msg, portMAX_DELAY);
		if(queue_recv_ret == pdTRUE)
		{
			if (mcuFuncConfig == CHAIN_DOWN_CTRLBOX)
			{
				xSemaphoreTake(chainDownDataSemaphore, portMAX_DELAY);
				for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
				{
					if(chainDownData[i].valid == CHAIN_DOWN_DATA_VALID)
					{
						if(memcmp(chainDownData[i].rfid, &recv_msg[DAT_INDEX], STC_RFID_ID_LEN) == 0)
						{
							wcs485_ChainOpen();
							memcpy(chainDownMsg.msg, chainDownData[i].rfid, STC_RFID_ID_LEN);
							queue_send_ret = xQueueSend(chainDownRfidOpenedQueue, &chainDownMsg, 0);
							if(queue_send_ret == errQUEUE_FULL)
							{
								xQueueReceive(chainDownRfidOpenedQueue, &tmpChainDownMsg, 0);
								xQueueSend(chainDownRfidOpenedQueue, &chainDownMsg, 0);
							}
							chainDownData[i].valid = CHAIN_DOWN_DATA_INVALID;
							chainDownData[i].age = 0;
							break;
						}
					}
				}
				xSemaphoreGive(chainDownDataSemaphore);
			}
			if(mcuFuncConfig == UHF_RFID_CTRLBOX)
			{
				//business code
				xSemaphoreGive(uhfMsgSemaphore);
				save_lowrfid_msg(&recv_msg[DAT_INDEX], MT_LOW_MSG_LEN);
			}
			if(mcuFuncConfig == RFID_CHECK_CTRLBOX)
			{
				//business code
				for (i = 0; i < STC_RFID_ID_LEN; i++)
				{
					if (recv_msg[DAT_INDEX + i] != 0 )
						break;
				}
				if (i == STC_RFID_ID_LEN)
				{
					wcs485_MotorCtrl(TURN_OFF);
					eventMsg.msgType = EVENT_MSG_ALARM;
					queue_send_ret = xQueueSend(eventMsgQueue, &eventMsg, 0);
					if(queue_send_ret == errQUEUE_FULL)
					{
						xQueueReceive(eventMsgQueue, &tmpMsg, 0);
						xQueueSend(eventMsgQueue, &eventMsg, 0);
					}
				}
			}
		}
	}
}

void uhfRFID_msg_task(void *pvParameters)
{
	BaseType_t queue_recv_ret, queue_send_ret;
	uint8_t recv_msg[MT_UHF_MSG_LEN] = {0};
	eventMsgFrame_t eventMsg, tmpMsg;

	while(1)
	{
		xSemaphoreTake(uhfMsgSemaphore, portMAX_DELAY);
		uhfrfid_send_cmd();
		queue_recv_ret = xQueueReceive(uhfRFIDRecvMsgQueue, recv_msg, 2000);
		if(queue_recv_ret == pdTRUE)
		{
			//business code
			mt_uhfrfid_convert(recv_msg, MT_UHF_MSG_LEN);
			save_uhfrfid_msg(recv_msg, MT_UHF_MSG_LEN/2);
			//send_msg();
			set_event_msg(&eventMsg, EVENT_MSG_CHAIN_UP);
			queue_send_ret = xQueueSend(eventMsgQueue, &eventMsg, 0);
			if(queue_send_ret == errQUEUE_FULL)
			{
				xQueueReceive(eventMsgQueue, &tmpMsg, 0);
				xQueueSend(eventMsgQueue, &eventMsg, 0);
			}
		}
	}
}

void watch_dog_task(void *pvParameters)
{
	while(1)
	{
		IWDG_Feed();
		sleep_Ms(1000); //1s
	}
}

void chainDown_sensor_task(void *pvParameters)
{
	BaseType_t sem_ret;
	BaseType_t queue_send_ret, queue_recv_ret;
	eventMsgFrame_t eventMsg, tmpMsg;
	chainDownMsgFrame_t chainDownMsg;
	
	while(1)
	{
		queue_recv_ret = xQueueReceive(chainDownRfidOpenedQueue, &chainDownMsg, portMAX_DELAY);
		if(queue_recv_ret == pdTRUE)
		{
			sem_ret = xSemaphoreTake(chainDownDetectSemaphore, CHAIN_DOWN_DETECT_TIMEOUT);
			if(sem_ret == pdTRUE)
			{
				eventMsg.msgType = EVENT_MSG_CHAIN_DOWN;
				memcpy(eventMsg.msg, chainDownMsg.msg, STC_RFID_ID_LEN);
				queue_send_ret = xQueueSend(eventMsgQueue, &eventMsg, 0);
				if(queue_send_ret == errQUEUE_FULL)
				{
					xQueueReceive(eventMsgQueue, &tmpMsg, 0);
					xQueueSend(eventMsgQueue, &eventMsg, 0);
				}
			}
			else
			{
				eventMsg.msgType = EVENT_MSG_CHAIN_DOWN;
				memcpy(eventMsg.msg, chainDownMsg.msg, STC_RFID_ID_LEN);
				queue_send_ret = xQueueSend(eventMsgQueue, &eventMsg, 0);
				if(queue_send_ret == errQUEUE_FULL)
				{
					xQueueReceive(eventMsgQueue, &tmpMsg, 0);
					xQueueSend(eventMsgQueue, &eventMsg, 0);
				}
			}
		}
		else
		{
			
		}
		
	}
}
