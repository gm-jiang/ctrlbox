#include "stm32f10x.h"
#include "user_task.h"
#include "bsp_port.h"
#include "mt_common.h"
#include "uhf_rfid_driver.h"
#include "ctrlbox_conf.h"

#include "wcs_parser.h"
#include "upgrade.h"

#include "FreeRTOS.h"
#include "task.h"

tagNode_t g_node_list;

//the receive message queue of uart
QueueHandle_t wcs485RecvMsgQueue = NULL;
QueueHandle_t stcRecvMsgQueue = NULL;
QueueHandle_t uhfRFIDRecvMsgQueue = NULL;
QueueHandle_t chainDownRfidOpenedQueue = NULL;
QueueHandle_t eventMsgQueue = NULL;
QueueHandle_t lowRFIDMsgQueue = NULL;

SemaphoreHandle_t chainDownDetectSemaphore = NULL;
SemaphoreHandle_t chainDownDataSemaphore = NULL;
SemaphoreHandle_t uhfMsgSemaphore = NULL;

SemaphoreHandle_t printMutex = NULL;

static void ota_msg_process(uint8_t *payload, uint8_t payload_length);
static void wcs_ver_process(uint8_t *payload, uint8_t payload_len);
static void wcs_ota_process(uint8_t *payload, uint8_t payload_len);

static void sys_mutex_init(void)
{
	printMutex = xSemaphoreCreateMutex();
	if(printMutex == NULL) 
	{
		dbg_print(PRINT_LEVEL_ERROR, "create printMutex failed\n");
		while(1);
	}
}

void sys_mutex_lock(SemaphoreHandle_t xMutex)
{
	xSemaphoreTake(xMutex, portMAX_DELAY);
}

void sys_mutex_unlock(SemaphoreHandle_t xMutex)
{
	xSemaphoreGive(xMutex);
}

void message_queue_init(void)
{
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
	lowRFIDMsgQueue = xQueueCreate(LOWRFID_QUEUE_NUM, LOWRFID_QUEUE_LEN);
	if(lowRFIDMsgQueue == NULL)
	{
		dbg_print(PRINT_LEVEL_ERROR, "create lowRFIDMsgQueue failed\r\n");
		while(1);
	}
	dbg_print(PRINT_LEVEL_INFO, "message queue creation completed\r\n");
}

void message_semaphore_init(void)
{
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
}

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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
#ifdef ODD_CODE
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, ODD_OFFSET);
#else
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, EVEN_OFFSET);
#endif
	bsp_gpio_configuration();
	bsp_releasekey_init();
	bsp_emerstopkey_init();
	bsp_chaindown_finish_sensor_init();
	bsp_lamp_init();

	bsp_motor_ctrl_init();
	bsp_mcu_485RE_init();

	//disable interrupts
	//portDISABLE_INTERRUPTS();

	bsp_wcs_uart_init();
	bsp_stc_uart_init();
	bsp_uhfrfid_uart_init();

	sys_mutex_init();
	ctrlbox_configinfo_init();
	dbg_print(PRINT_LEVEL_DEBUG, "%s %s\r\n", SW_VERSION_STR, HW_VERSION_STR);
	//g_mcu485Addr = 0x03;
	dbg_print(PRINT_LEVEL_DEBUG, "ctrlbox addr: 0x%02X\r\n", g_mcu485Addr);
	bsp_chaindown_ctrl_init();
	message_queue_init();
	message_semaphore_init();
	node_list_init(&g_node_list);
	//enable interrupts
	//portENABLE_INTERRUPTS();
	system_init_success_led();
	dbg_print(PRINT_LEVEL_DEBUG, "ctrlbox init completed\r\n");
	bsp_IWDG_init(IWDG_Prescaler_64, 3125); //5s
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
		queue_recv_ret = xQueueReceive(wcs485RecvMsgQueue, &recv_msg, 10*1000);
		if(queue_recv_ret == pdTRUE)
		{
			dbg_print_msg(PRINT_LEVEL_DEBUG, (uint8_t *)"UART1 IN <--", recv_msg.msg[WCS_FRAME_LEN_INDEX] + 3, recv_msg.msg);
			ota_msg_process(&recv_msg.msg[WCS_485_ADDR_INDEX], recv_msg.msg[WCS_FRAME_LEN_INDEX]);
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
			wcs_remove_aged_node();
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
	tagNode_t *tagNode = NULL;

	while(1)
	{
		queue_recv_ret = xQueueReceive(stcRecvMsgQueue, recv_msg, portMAX_DELAY);
		if(queue_recv_ret == pdTRUE)
		{
			dbg_print_msg(PRINT_LEVEL_DEBUG, "STC DATA IN:", STC_MSG_LEN, recv_msg);
			if (g_mcuConfigInfo.function == CHAIN_DOWN_CTRLBOX)
			{
				xSemaphoreTake(chainDownDataSemaphore, portMAX_DELAY);
				tagNode = node_list_find_tagid(&g_node_list, recv_msg, STC_RFID_ID_LEN);
				if (tagNode != NULL)
				{
					wcs485_ChainOpen();
					memcpy(chainDownMsg.msg, tagNode->msg->tagId, STC_RFID_ID_LEN);
					queue_send_ret = xQueueSend(chainDownRfidOpenedQueue, &chainDownMsg, 0);
					if(queue_send_ret == errQUEUE_FULL)
					{
						xQueueReceive(chainDownRfidOpenedQueue, &tmpChainDownMsg, 0);
						xQueueSend(chainDownRfidOpenedQueue, &chainDownMsg, 0);
					}
					node_list_remove(&g_node_list, tagNode);
				}
				xSemaphoreGive(chainDownDataSemaphore);
			}
			if(g_mcuConfigInfo.function == UHF_RFID_CTRLBOX)
			{
				//business code
				xSemaphoreGive(uhfMsgSemaphore);
				xQueueSend(lowRFIDMsgQueue, recv_msg, 0);
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
			if(g_mcuConfigInfo.function == RFID_DEBUG_CTRLBOX)
			{
				wcs485_ChainOpen();
			}
		}
	}
}

void uhfRFID_msg_task(void *pvParameters)
{
	BaseType_t queue_recv_ret1, queue_recv_ret2, queue_send_ret;
	uint8_t recv_uhf_msg[UHF_MSG_LEN] = {0};
	uint8_t recv_low_msg[STC_MSG_LEN] = {0};
	uint8_t bind_msg[UHF_RFID_LABLE_DATA_LEN + STC_RFID_ID_LEN] = {0};
	eventMsgFrame_t eventMsg, tmpMsg;

	while(1)
	{
		xSemaphoreTake(uhfMsgSemaphore, portMAX_DELAY);
		queue_recv_ret1 = xQueueReceive(lowRFIDMsgQueue, recv_low_msg, portMAX_DELAY);
		uhfrfid_send_cmd();
		queue_recv_ret2 = xQueueReceive(uhfRFIDRecvMsgQueue, recv_uhf_msg, 2000);
		if(queue_recv_ret1 == pdTRUE && queue_recv_ret2 == pdTRUE)
		{
			//business code
			mt_uhfrfid_convert(recv_uhf_msg, UHF_MSG_LEN/2);
			memcpy(&bind_msg[0], recv_uhf_msg, UHF_RFID_LABLE_DATA_LEN);
			memcpy(&bind_msg[UHF_RFID_LABLE_DATA_LEN], &recv_low_msg[DAT_INDEX], STC_RFID_ID_LEN);
			eventMsg.msgType = EVENT_MSG_CHAIN_UP;
			memcpy(eventMsg.msg, bind_msg, UHF_RFID_LABLE_DATA_LEN + STC_RFID_ID_LEN);
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
				eventMsg.msgType = EVENT_MSG_CHAIN_DOWN_DETECT_TIMEOUT;
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

static void ota_msg_process(uint8_t *payload, uint8_t payload_length)
{
	uint8_t payload_len = payload_length - 3;
	/*payload[0] == shortaddr  payload[1] == type  payload[2] == data... */
	switch (payload[1])
	{
	case MT_WCS_GET_VER_MSG:
		wcs_ver_process(&payload[2], payload_len);
		break;
	case MT_WCS_SET_OTA_MSG:
		wcs_ota_process(&payload[2], payload_len);
		break;
	default:
		break;
	}
}

static void wcs_ver_process(uint8_t *payload, uint8_t payload_len)
{
	/*add business code*/
	if (mtOtaCbs.pfnOtaVerHandler != NULL)
	{
		ota_ver_msg_t ota_ver_msg;

		memset(&ota_ver_msg, 0, sizeof(ota_ver_msg_t));
		ota_ver_msg.sw = (payload[0] << 8) | payload[1];
		ota_ver_msg.hw = (payload[2] << 8) | payload[3];
		dbg_print(PRINT_LEVEL_DEBUG, "sys_ver_process: sw:%d hw:%d\r\n", ota_ver_msg.sw, ota_ver_msg.hw);
		mtOtaCbs.pfnOtaVerHandler(&ota_ver_msg);
	}
	else
	{
		dbg_print(PRINT_LEVEL_DEBUG, "sys_ver_process is null\r\n");
	}
}

static void wcs_ota_process(uint8_t *payload, uint8_t payload_len)
{
	/*add business code*/
	if (mtOtaCbs.pfnOtaDataHandler != NULL)
	{
		ota_code_msg_t ota_code_msg;

		memset(&ota_code_msg, 0, sizeof(ota_code_msg_t));
		ota_code_msg.seq = (payload[0] << 8) | payload[1];
		ota_code_msg.size = payload[2];
		memcpy(ota_code_msg.msg, &payload[3], ota_code_msg.size);
		dbg_print(PRINT_LEVEL_DEBUG, "sys_ota_process: seq: %d size:%d\r\n", ota_code_msg.seq, ota_code_msg.size);
		mtOtaCbs.pfnOtaDataHandler(&ota_code_msg);
	}
	else
	{
		dbg_print(PRINT_LEVEL_DEBUG, "sys_ota_process is null\r\n");
	}
}

