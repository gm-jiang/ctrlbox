#include "stm32f10x.h"
#include "user_task.h"
#include "bsp_port.h"
#include "mt_common.h"
#include "uhf_rfid_driver.h"
#include "ctrlbox_conf.h"
#include "wcs_parser.h"

#include "FreeRTOS.h"
#include "task.h"

//the receive message queue of uart
QueueHandle_t wcs485RecvMsgQueue = NULL;
QueueHandle_t stcRecvMsgQueue = NULL;
QueueHandle_t uhfRFIDRecvMsgQueue = NULL;
QueueHandle_t chainDownRfidOpenedQueue = NULL;
QueueHandle_t eventMsgQueue = NULL;

SemaphoreHandle_t chainDownDetectSemaphore = NULL;
SemaphoreHandle_t chainDownDataSemaphore = NULL;
SemaphoreHandle_t uhfMsgSemaphore = NULL;


void system_init_success_led(void)
{
	uint8_t i;
	for(i = 0; i < 20; i++)
	{
		bsp_lamp_ctrl(DEBUG_LED, TURN_ON);
		mt_sleep_us(50*1000);
		bsp_lamp_ctrl(DEBUG_LED, TURN_OFF);
		mt_sleep_us(50*1000);
	}
}

void platform_init(void)
{
	uint8_t ret;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	bsp_gpio_configuration();
	bsp_releasekey_init();
	bsp_emerstopkey_init();
	bsp_chaindown_finish_sensor_init();
	bsp_lamp_init();
	bsp_chaindown_ctrl_init();
	bsp_motor_ctrl_init();
	bsp_mcu_485RE_init();
	bsp_IWDG_init(IWDG_Prescaler_64, 3125); //5s

	//disable interrupts
	portDISABLE_INTERRUPTS();

	bsp_wcs_uart_init();
	bsp_stc_uart_init();
	bsp_uhfrfid_uart_init();

	g_mcu485Addr = get_485_addr();
	if (g_mcu485Addr == 0xFF)
	{
		//dbg_print(PRINT_LEVEL_DEBUG, "ctrlbox 485 addr: %02X...\r\n", g_mcu485Addr);
	}
	/*
	g_mcuFuncConfig = get_customer_config();
	if (g_mcuFuncConfig != CHAIN_DOWN_CTRLBOX &&
			g_mcuFuncConfig != UHF_RFID_CTRLBOX &&
			g_mcuFuncConfig != RFID_CHECK_CTRLBOX)
	{
		g_mcuFuncConfig = CHAIN_DOWN_CTRLBOX;
	}
	*/
	dbg_print(PRINT_LEVEL_DEBUG, "%s %s\r\n", SW_VERSION_STR, HW_VERSION_STR);
	dbg_print(PRINT_LEVEL_DEBUG, "ctrlbox addr: 0x%02X\r\n", g_mcu485Addr);
  //dbg_print(PRINT_LEVEL_DEBUG, "ctrlbox mode: 0x%02X\r\n", g_mcuFuncConfig);
	
	ret = get_config_info(&g_mcuConfigInfo);
	if(ret != RTN_SUCCESS)
	{
		g_mcuConfigInfo.function = CHAIN_DOWN_CTRLBOX;
		//g_mcuConfigInfo.debugLedFlashTime = 500; //ms
		g_mcuConfigInfo.valveCtrlTime = VALVE_CTRL_TIME; //s
		g_mcuConfigInfo.lampCtrlLevel = LEVEL_CTRL_HIGH;
		g_mcuConfigInfo.valveCtrlLevel = LEVEL_CTRL_HIGH;
	}
	else
	{
		if((g_mcuConfigInfo.function != CHAIN_DOWN_CTRLBOX) &&
			 (g_mcuConfigInfo.function != UHF_RFID_CTRLBOX) &&
			 (g_mcuConfigInfo.function != RFID_CHECK_CTRLBOX))
		{
			g_mcuConfigInfo.function = CHAIN_DOWN_CTRLBOX;
		}

		if(g_mcuConfigInfo.valveCtrlTime > VALVE_CTRL_TIME_MAX)
		{
			g_mcuConfigInfo.valveCtrlTime = VALVE_CTRL_TIME_MAX;
		}

		if((g_mcuConfigInfo.lampCtrlLevel != LEVEL_CTRL_HIGH)&&
			 (g_mcuConfigInfo.lampCtrlLevel != LEVEL_CTRL_LOW))
		{
			g_mcuConfigInfo.lampCtrlLevel = LEVEL_CTRL_HIGH;
		}

		if((g_mcuConfigInfo.motorCtrlLevel != LEVEL_CTRL_HIGH)&&
			 (g_mcuConfigInfo.motorCtrlLevel != LEVEL_CTRL_LOW))
		{
			g_mcuConfigInfo.motorCtrlLevel = LEVEL_CTRL_HIGH;
		}

		if((g_mcuConfigInfo.valveCtrlLevel != LEVEL_CTRL_HIGH)&&
			 (g_mcuConfigInfo.valveCtrlLevel != LEVEL_CTRL_LOW))
		{
			g_mcuConfigInfo.valveCtrlLevel = LEVEL_CTRL_HIGH;
		}
	}

	wcs485RecvMsgQueue = xQueueCreate(UART_QUEUE_NUM, WCS_MSG_LEN);
	if(wcs485RecvMsgQueue == NULL)
	{
		dbg_print(PRINT_LEVEL_ERROR, "create wcs485RecvMsgQueue failed\r\n");
		while(1);
	}
	stcRecvMsgQueue = xQueueCreate(UART_QUEUE_NUM, STC_MSG_LEN);
	if(stcRecvMsgQueue == NULL)
	{
		dbg_print(PRINT_LEVEL_ERROR, "create stcRecvMsgQueue failed\r\n");
		while(1);
	}
	uhfRFIDRecvMsgQueue = xQueueCreate(UART_QUEUE_NUM, UHF_MSG_LEN);
	if(uhfRFIDRecvMsgQueue == NULL) 
	{
		dbg_print(PRINT_LEVEL_ERROR, "create uhfRFIDRecvMsgQueue failed\r\n");
		while(1);
	}
	eventMsgQueue = xQueueCreate(EVENT_MSG_QUEUE_NUM, sizeof(eventMsgFrame_t));
	if(eventMsgQueue == NULL)
	{
		dbg_print(PRINT_LEVEL_ERROR, "create eventMsgQueue failed\r\n");
		while(1);
	}
	chainDownRfidOpenedQueue = xQueueCreate(CHAIN_DOWN_RFID_OPENED_QUEUE_NUM, sizeof(chainDownMsgFrame_t));
	if(chainDownRfidOpenedQueue == NULL)
	{
		dbg_print(PRINT_LEVEL_ERROR, "create chainDownRfidOpenedQueue failed\r\n");
		while(1);
	}
  dbg_print(PRINT_LEVEL_INFO, "message queue creation completed\r\n");

	memset(g_chainDownData, 0, sizeof(chainDownData_t));

	chainDownDetectSemaphore = xSemaphoreCreateBinary();
	if(chainDownDetectSemaphore == NULL) 
	{
		dbg_print(PRINT_LEVEL_ERROR, "create chainDownDetectSemaphore failed\r\n");
		while(1);
	}

	chainDownDataSemaphore = xSemaphoreCreateBinary();
	if(chainDownDataSemaphore == NULL) 
	{
		dbg_print(PRINT_LEVEL_ERROR, "create chainDownDataSemaphore failed\n");
		while(1);
	}
	xSemaphoreGive(chainDownDataSemaphore);

	uhfMsgSemaphore = xSemaphoreCreateBinary();
	if(uhfMsgSemaphore == NULL) 
	{
		dbg_print(PRINT_LEVEL_ERROR, "create uhfMsgSemaphore failed\n");
		while(1);
	}
	dbg_print(PRINT_LEVEL_INFO, "message semaphore creation completed\r\n");
	//enable interrupts
	portENABLE_INTERRUPTS();
	system_init_success_led();
	dbg_print(PRINT_LEVEL_DEBUG, "ctrlbox init completed\r\n");
}

void lamp_task(void *pvParameters)
{
	while(1)
	{
		bsp_lamp_ctrl(DEBUG_LED, TURN_ON);
		switch(g_mcuConfigInfo.function)
		{
			case CHAIN_DOWN_CTRLBOX:
				vTaskDelay(CHAIN_DOWN_LAMP_DELAY);
				break;
			case UHF_RFID_CTRLBOX:
				vTaskDelay(UHF_RFID_LAMP_DELAY);
				break;
			case RFID_CHECK_CTRLBOX:
				vTaskDelay(RFID_CHECK_LAMP_DELAY);
				break;
			default:
				vTaskDelay(CHAIN_DOWN_LAMP_DELAY);
				break;
		}
		//vTaskDelay(500);
		bsp_lamp_ctrl(DEBUG_LED, TURN_OFF);
		switch(g_mcuConfigInfo.function)
		{
			case CHAIN_DOWN_CTRLBOX:
				vTaskDelay(CHAIN_DOWN_LAMP_DELAY);
				break;
			case UHF_RFID_CTRLBOX:
				vTaskDelay(UHF_RFID_LAMP_DELAY);
				break;
			case RFID_CHECK_CTRLBOX:
				vTaskDelay(RFID_CHECK_LAMP_DELAY);
				break;
			default:
				vTaskDelay(CHAIN_DOWN_LAMP_DELAY);
				break;
		}
	}
}

void wcs485_msg_task(void *pvParameters)
{
	BaseType_t queue_recv_ret;
	uartMsgFrame_t recv_msg;
	uint8_t ret;
	configInfoType_t configInfo;
	wcsFrameType_e frameType;
	uint8_t decodeBuf[WCS_DECODE_BUF_LEN];
	uint8_t decodeLen = 0;

	while(1)
	{
		queue_recv_ret = xQueueReceive(wcs485RecvMsgQueue, &recv_msg, portMAX_DELAY);
		if(queue_recv_ret == pdTRUE)
		{
			dbg_print_msg(PRINT_LEVEL_DEBUG, "WCS DATA IN: <--", recv_msg.msg[WCS_FRAME_LEN_INDEX] + 3, recv_msg.msg);
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
					set_485_addr(recv_msg.msg[WCS_485_FUNC_INDEX + 1]);
					g_mcu485Addr = get_485_addr();
					decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
					decodeBuf[1] = g_mcu485Addr;				
					wcs485_Update485AddrOrSNackSend(decodeBuf, 2);
				}
				else if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_CONFIG_SN)
				{
					ret = set_dev_sn(&(recv_msg.msg[WCS_485_FUNC_INDEX + 1]), WCS_485_DEVICE_SN_LEN);
					if(ret != RTN_SUCCESS)
					{
						continue;
					}
					decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
					ret = get_dev_sn(&decodeBuf[1], &decodeLen);	
					if((ret != RTN_SUCCESS) || (decodeLen != WCS_485_DEVICE_SN_LEN))
					{
						continue;
					}					
					wcs485_Update485AddrOrSNackSend(decodeBuf, WCS_485_DEVICE_SN_LEN + 1);
				}
				else if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_QUERY_ADDR_SN)
				{
					decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
					decodeBuf[1] = get_485_addr();
					ret = get_dev_sn(&decodeBuf[2], &decodeLen);	
					if((ret != RTN_SUCCESS) || (decodeLen != WCS_485_DEVICE_SN_LEN))
					{
						continue;
					}
					wcs485_Update485AddrOrSNackSend(decodeBuf, WCS_485_DEVICE_SN_LEN + 2);
				}
			}

			if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_CONFIG_CUSTOMER)
			{
				//set_customer_config(recv_msg.msg[WCS_485_FUNC_INDEX + 1]);
				configInfo.function = recv_msg.msg[WCS_485_FUNC_INDEX + 1];
				configInfo.lampCtrlLevel = recv_msg.msg[WCS_485_FUNC_INDEX + 2];
				configInfo.motorCtrlLevel = recv_msg.msg[WCS_485_FUNC_INDEX + 3];
				configInfo.valveCtrlLevel = recv_msg.msg[WCS_485_FUNC_INDEX + 4];
				configInfo.valveCtrlTime = ((recv_msg.msg[WCS_485_FUNC_INDEX + 5])<<8)|(recv_msg.msg[WCS_485_FUNC_INDEX + 6]);
				ret = set_config_info(&configInfo);
				if(ret != RTN_SUCCESS)
				{
					continue;
				}
				//g_mcuFuncConfig = get_customer_config();
				ret = get_config_info(&g_mcuConfigInfo);
				if(ret != RTN_SUCCESS)
				{
					continue;
				}
				
				decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
				decodeBuf[1] = g_mcuConfigInfo.function;
				decodeBuf[2] = g_mcuConfigInfo.lampCtrlLevel;		
				decodeBuf[3] = g_mcuConfigInfo.motorCtrlLevel;
				decodeBuf[4] = g_mcuConfigInfo.valveCtrlLevel;
				decodeBuf[5] = ((g_mcuConfigInfo.valveCtrlTime)>>8)&0xff;
				decodeBuf[6] = (g_mcuConfigInfo.valveCtrlTime)&0xff;
				wcs485_Update485AddrOrSNackSend(decodeBuf, 7);
			}
			else if(recv_msg.msg[WCS_485_FUNC_INDEX] == WCS_485_QUERY_CUSTOMER)
			{
				ret = get_config_info(&configInfo);
				if(ret != RTN_SUCCESS)
				{
					continue;
				}
				decodeBuf[0] = recv_msg.msg[WCS_485_FUNC_INDEX];
				decodeBuf[1] = configInfo.function;
				decodeBuf[2] = configInfo.lampCtrlLevel;		
				decodeBuf[3] = configInfo.motorCtrlLevel;
				decodeBuf[4] = configInfo.valveCtrlLevel;
				decodeBuf[5] = ((configInfo.valveCtrlTime)>>8)&0xff;
				decodeBuf[6] = (configInfo.valveCtrlTime)&0xff;
				wcs485_Update485AddrOrSNackSend(decodeBuf, 7);
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
			dbg_print(PRINT_LEVEL_ERROR, "read wcs485_msg recv queue failed\n");
		}
	}
}

void stc_msg_task(void *pvParameters)
{
	uint8_t i;
	BaseType_t queue_recv_ret, queue_send_ret;
	uint8_t recv_msg[STC_MSG_LEN] = {0};
	chainDownMsgFrame_t chainDownMsg, tmpChainDownMsg;
	eventMsgFrame_t eventMsg, tmpMsg;

	while(1)
	{
		queue_recv_ret = xQueueReceive(stcRecvMsgQueue, recv_msg, portMAX_DELAY);
		if(queue_recv_ret == pdTRUE)
		{
			dbg_print_msg(PRINT_LEVEL_DEBUG, "STC DATA IN:", STC_MSG_LEN, recv_msg);
			if (g_mcuConfigInfo.function == CHAIN_DOWN_CTRLBOX)
			{
				xSemaphoreTake(chainDownDataSemaphore, portMAX_DELAY);
				for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
				{
					if(g_chainDownData[i].valid == CHAIN_DOWN_DATA_VALID)
					{
						if(memcmp(g_chainDownData[i].rfid, &recv_msg[DAT_INDEX], STC_RFID_ID_LEN) == 0)
						{
							wcs485_ChainOpen();
							memcpy(chainDownMsg.msg, g_chainDownData[i].rfid, STC_RFID_ID_LEN);
							queue_send_ret = xQueueSend(chainDownRfidOpenedQueue, &chainDownMsg, 0);
							if(queue_send_ret == errQUEUE_FULL)
							{
								xQueueReceive(chainDownRfidOpenedQueue, &tmpChainDownMsg, 0);
								xQueueSend(chainDownRfidOpenedQueue, &chainDownMsg, 0);
							}
							g_chainDownData[i].valid = CHAIN_DOWN_DATA_INVALID;
							g_chainDownData[i].age = 0;
							break;
						}
					}
				}
				xSemaphoreGive(chainDownDataSemaphore);
			}
			if(g_mcuConfigInfo.function == UHF_RFID_CTRLBOX)
			{
				//business code
				xSemaphoreGive(uhfMsgSemaphore);
				save_lowrfid_msg(&recv_msg[DAT_INDEX], STC_RFID_ID_LEN);
			}
			if(g_mcuConfigInfo.function == RFID_CHECK_CTRLBOX)
			{
				//business code
				for (i = 0; i < STC_RFID_ID_LEN; i++)
				{
					if (recv_msg[DAT_INDEX + i] != 0 )
						break;
				}
				if (i == STC_RFID_ID_LEN)
				{
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
	uint8_t recv_msg[UHF_MSG_LEN] = {0};
	eventMsgFrame_t eventMsg, tmpMsg;

	while(1)
	{
		xSemaphoreTake(uhfMsgSemaphore, portMAX_DELAY);
		uhfrfid_send_cmd();
		queue_recv_ret = xQueueReceive(uhfRFIDRecvMsgQueue, recv_msg, 2000);
		if(queue_recv_ret == pdTRUE)
		{
			//business code
			mt_uhfrfid_convert(recv_msg, UHF_MSG_LEN);
			save_uhfrfid_msg(recv_msg, UHF_MSG_LEN/2);
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
		bsp_IWDG_feed();
		vTaskDelay(1000);
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
	}
}
