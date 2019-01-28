#include "ctrlbox_conf.h"
#include "user_task.h"
#include "wcs_parser.h"
#include "wcs_485_driver.h"
#include "mt_common.h"

chainDownData_t g_chainDownData[CHAIN_DOWN_DATA_BUF_LEN];

uint8_t add_ChainDownData(uint8_t *buf, uint8_t len)
{
	uint8_t ret = RTN_SUCCESS;
	uint8_t i, max = 0;
	
	if((len != STC_RFID_ID_LEN) || (buf == NULL))
		return RTN_FAIL;

	xSemaphoreTake(chainDownDataSemaphore, portMAX_DELAY);
	for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
	{
		if((g_chainDownData[i].valid == CHAIN_DOWN_DATA_VALID) && (memcmp(g_chainDownData[i].rfid, buf, STC_RFID_ID_LEN) == 0))
		{
			xSemaphoreGive(chainDownDataSemaphore);
			return ret;
		}
	}
		
	for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
	{
		if(g_chainDownData[i].valid == CHAIN_DOWN_DATA_INVALID)
		{
			memcpy(g_chainDownData[i].rfid, buf, STC_RFID_ID_LEN);
			g_chainDownData[i].age = 0;
			g_chainDownData[i].valid = CHAIN_DOWN_DATA_VALID;
			break;
		}
	}
	
	if(i == CHAIN_DOWN_DATA_BUF_LEN)
	{
		max = 0;
		for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
		{
			if(g_chainDownData[i].age >= g_chainDownData[max].age)
			{
				max = i;
			}
		}
		memcpy(g_chainDownData[max].rfid, buf, STC_RFID_ID_LEN);
		g_chainDownData[max].age = 0;
		g_chainDownData[max].valid = CHAIN_DOWN_DATA_VALID;
	}
	
	for(i = 0; i < CHAIN_DOWN_DATA_BUF_LEN; i++)
	{
		if(g_chainDownData[i].valid == CHAIN_DOWN_DATA_VALID)
		{
			g_chainDownData[i].age = (g_chainDownData[i].age + 1) % CHAIN_DOWN_DATA_AGE_MAX;
		}
	}
	xSemaphoreGive(chainDownDataSemaphore);
	
	return ret;
}

void wcs485_Update485AddrOrSNackSend(uint8_t *dataBuf, uint8_t dataLen)
{
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	//uint8_t sendLen = 0;
	uint8_t i = 0;
	sendBuf[i++] = WCS_FRAME_HEAD_1;
	sendBuf[i++] = WCS_FRAME_HEAD_2;
	i++;
	sendBuf[i++] = g_mcu485Addr;
	if((dataLen > 0) && (dataBuf != NULL))
	{
		memcpy(&sendBuf[i], dataBuf, dataLen);
		i = i + dataLen;
	}
	sendBuf[WCS_FRAME_LEN_INDEX] = i + 1 - (WCS_FRAME_LEN_INDEX + 1);
	sendBuf[i] = mt_cal_crc8(sendBuf, i);
	wcs_send_data(sendBuf, i + 1);
}

void wcs485_ChainOpen(void)
{
	if(g_mcuConfigInfo.valveCtrlLevel == LEVEL_CTRL_HIGH)
		GPIO_ResetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
	else
		GPIO_SetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
	
	vTaskDelay(g_mcuConfigInfo.valveCtrlTime);
	
	if(g_mcuConfigInfo.valveCtrlLevel == LEVEL_CTRL_HIGH)
		GPIO_SetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
	else
		GPIO_ResetBits(CHAIN_DOWN_CTRL_GPIO, CHAIN_DOWN_CTRL_GPIO_PIN);
}

void wcs485_MotorCtrl(statusCtrlType_e statusCtrl)
{
	if(statusCtrl == TURN_ON)
	{
		if(g_mcuConfigInfo.motorCtrlLevel == LEVEL_CTRL_HIGH)
			GPIO_SetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
		else
			GPIO_ResetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
	}
	else
	{
		if(g_mcuConfigInfo.motorCtrlLevel == LEVEL_CTRL_HIGH)
			GPIO_ResetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
		else
			GPIO_SetBits(CHAIN_MOTOR_CTRL_GPIO, CHAIN_MOTOR_CTRL_GPIO_PIN);
	}
}

uint8_t wcs485_Encode(wcsFrameType_e frameType, uint8_t *dataBuf, uint8_t dataLen, uint8_t *encodeBuf, uint8_t *encodeLen)
{
	uint8_t i = 0;
	if(encodeBuf == NULL)
		return RTN_FAIL;
	
	encodeBuf[i++] = WCS_FRAME_HEAD_1;
	encodeBuf[i++] = WCS_FRAME_HEAD_2;
	i++;
	encodeBuf[i++] = g_mcu485Addr;
	
	switch(frameType)
	{
		case WCS_FRAME_QUERY_EMPTY_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_EMPTY_ACK;
			break;
		case WCS_FRAME_QUERY_CHAIN_UP_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_CHAIN_UP_ACK;
			break;
		case WCS_FRAME_QUERY_CHAIN_DOWN_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_CHAIN_DOWN_ACK;
			break;
		case WCS_FRAME_QUERY_ORDER_STATUS_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_ORDER_STATUS_ACK;
			break;
		case WCS_FRAME_QUERY_KEY_DOWN_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_KEY_DOWN_ACK;
			break;
		case WCS_FRAME_QUERY_ALARM_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_ALARM_ACK;
			break;
		case WCS_FRAME_QUERY_START_STOP_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_START_STOP_ACK;
			break;
		case WCS_FRAME_QUERY_EMER_STOP_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_EMER_STOP_ACK;
			break;
		case WCS_FRAME_QUERY_CHAIN_DOWN_DETECT_TIMEOUT_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_CHAIN_DOWN_DETECT_TIMEOUT_ACK;
			break;
		case WCS_FRAME_QUERY_NO_CHAIN_DOWN_TIMEOUT_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_NO_CHAIN_DOWN_TIMEOUT_ACK;
			break;
		case WCS_FRAME_QUERY_VALVE_ERROR_OPEN_ACK_E:
			encodeBuf[i++] = WCS_FRAME_QUERY_VALVE_ERROR_OPEN_ACK;
			break;
		case WCS_FRAME_CHAIN_DOWN_CTRL_ACK_E:
			encodeBuf[i++] = WCS_FRAME_CHAIN_DOWN_CTRL_ACK;
			break;
		case WCS_FRAME_ORDER_STATUS_CTRL_ACK_E:
			encodeBuf[i++] = WCS_FRAME_ORDER_STATUS_CTRL_ACK;
			break;
		case WCS_FRAME_MOTOR_START_STOP_CTRL_ACK_E:
			encodeBuf[i++] = WCS_FRAME_MOTOR_START_STOP_CTRL_ACK;
			break;
		default:
			return RTN_FAIL;
	}
	
	if((dataLen > 0) && (dataBuf != NULL))
	{
		memcpy(&encodeBuf[i], dataBuf, dataLen);
		i = i + dataLen;
	}
	
	encodeBuf[WCS_FRAME_LEN_INDEX] = i + 1 - (WCS_FRAME_LEN_INDEX + 1);
	
	encodeBuf[i] = mt_cal_crc8(encodeBuf, i);
	
	*encodeLen = i + 1;
	
	return RTN_SUCCESS;
}

wcsFrameType_e wcs485_Decode(uint8_t *originBuf, uint8_t originLen, uint8_t *decodeBuf, uint8_t *decodeLen)
{
	wcsFrameType_e frameType = WCS_FRAME_INVALID_E;

	if((originBuf == NULL) || (decodeBuf == NULL))
		return WCS_FRAME_INVALID_E;

	switch(originBuf[WCS_485_FUNC_INDEX])
	{
		case WCS_FRAME_QUERY:
			frameType = WCS_FRAME_QUERY_E;
			*decodeLen = 0;
			break;
		case WCS_FRAME_CHAIN_DOWN_CTRL:
			frameType = WCS_FRAME_CHAIN_DOWN_CTRL_E;
			*decodeLen = originBuf[WCS_FRAME_LEN_INDEX] - 3;
			memcpy(decodeBuf, &originBuf[WCS_485_FUNC_INDEX + 1], *decodeLen);
			break;
		case WCS_FRAME_ORDER_STATUS_CTRL:
			frameType = WCS_FRAME_ORDER_STATUS_CTRL_E;
			*decodeLen = originBuf[WCS_FRAME_LEN_INDEX] - 3;
			memcpy(decodeBuf, &originBuf[WCS_485_FUNC_INDEX + 1], *decodeLen);
			break;
		case WCS_FRAME_MOTOR_START_STOP_CTRL:
			frameType = WCS_FRAME_MOTOR_START_STOP_CTRL_E;
			*decodeLen = originBuf[WCS_FRAME_LEN_INDEX] - 3;
			memcpy(decodeBuf, &originBuf[WCS_485_FUNC_INDEX + 1], *decodeLen);
			break;
		default:
			return WCS_FRAME_INVALID_E;
	}
	return frameType;
}

uint8_t wcs485_QueryEmptyAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_EMPTY_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainUpAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	//uint8_t dataBuf[WCS_SEND_BUF_LEN];
	//uint8_t dataLen = 10;
	ret = wcs485_Encode(WCS_FRAME_QUERY_CHAIN_UP_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainDownAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	//uint8_t dataBuf[WCS_SEND_BUF_LEN];
	//uint8_t dataLen = 10;
	ret = wcs485_Encode(WCS_FRAME_QUERY_CHAIN_DOWN_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_OrderStatusAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_ORDER_STATUS_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainKeyDownAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_KEY_DOWN_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainAlarmAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_ALARM_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_QueryStartStopAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_START_STOP_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_QueryEmerStopAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_EMER_STOP_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainDownDetectTimeoutAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	//uint8_t dataBuf[WCS_SEND_BUF_LEN];
	//uint8_t dataLen = 10;
	ret = wcs485_Encode(WCS_FRAME_QUERY_CHAIN_DOWN_DETECT_TIMEOUT_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_NoChainDownTimeoutAckSend(uint8_t *buf, uint8_t len)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	//uint8_t dataBuf[WCS_SEND_BUF_LEN];
	//uint8_t dataLen = 10;
	ret = wcs485_Encode(WCS_FRAME_QUERY_NO_CHAIN_DOWN_TIMEOUT_ACK_E, buf, len, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_QueryValveErrOpenAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_QUERY_VALVE_ERROR_OPEN_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_ChainDownCmdAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_CHAIN_DOWN_CTRL_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_OrderStatusCmdAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_ORDER_STATUS_CTRL_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_MotorStartStopCmdAckSend(void)
{
	uint8_t ret;
	uint8_t sendBuf[WCS_SEND_BUF_LEN];
	uint8_t sendLen = 0;
	ret = wcs485_Encode(WCS_FRAME_MOTOR_START_STOP_CTRL_ACK_E, NULL, 0, sendBuf, &sendLen);
	if(ret == RTN_SUCCESS)
	{
		wcs_send_data(sendBuf, sendLen);
	}
	return ret;
}

uint8_t wcs485_QueryCmd()
{
	uint8_t ret = RTN_SUCCESS;
	eventMsgFrame_t eventMsg;
	BaseType_t queueRet;

	queueRet = xQueueReceive(eventMsgQueue, &eventMsg, 0);
	if(queueRet != pdTRUE)
	{
		ret = wcs485_QueryEmptyAckSend();
	}
	else
	{
		switch(eventMsg.msgType)
		{
			case EVENT_MSG_CHAIN_UP:
				ret = wcs485_ChainUpAckSend(eventMsg.msg, UHF_RFID_LABLE_DATA_LEN + STC_RFID_ID_LEN);
				break;
			case EVENT_MSG_CHAIN_DOWN:
				ret = wcs485_ChainDownAckSend(eventMsg.msg, STC_RFID_ID_LEN);
				break;
			case EVENT_MSG_ORDER_STATUS:
				ret = wcs485_OrderStatusAckSend(eventMsg.msg, WCS_ORDER_STATUS_LEN);
				break;
			case EVENT_MSG_KEY_DOWN:
				ret = wcs485_ChainKeyDownAckSend();
				break;
			case EVENT_MSG_ALARM:
				ret = wcs485_ChainAlarmAckSend();
				break;
			case EVENT_MSG_START_STOP:
				ret = wcs485_QueryStartStopAckSend(eventMsg.msg, WCS_MOTOR_START_STOP_STATUS_LEN);
				break;
			case EVENT_MSG_EMER_STOP:
				ret = wcs485_QueryEmerStopAckSend();
				break;
			case EVENT_MSG_CHAIN_DOWN_DETECT_TIMEOUT:
				ret = wcs485_ChainDownDetectTimeoutAckSend(eventMsg.msg, STC_RFID_ID_LEN);
				break;
			case EVENT_MSG_NO_CHAIN_DOWN_TIMEOUT:
				ret = wcs485_NoChainDownTimeoutAckSend(eventMsg.msg, STC_RFID_ID_LEN);
				break;
			case EVENT_MSG_VALVE_ERROR_OPEN:
				ret = wcs485_QueryValveErrOpenAckSend();
				break;
			default:
				break;
		}
	}
	return ret;
}

uint8_t wcs485_ChainDownCmd(uint8_t *dataBuf, uint8_t dataLen)
{
	uint8_t ret;
	ret = wcs485_ChainDownCmdAckSend();
	
	ret = add_ChainDownData(dataBuf, STC_RFID_ID_LEN);
	
	return ret;
}

uint8_t wcs485_OrderStatusCmd(uint8_t *dataBuf, uint8_t dataLen)
{
	uint8_t ret;
	eventMsgFrame_t eventMsg, tmpMsg;
	BaseType_t queue_send_ret;
	
	ret = wcs485_OrderStatusCmdAckSend();
	if(dataBuf[0] & TRICOLOR_LAMP_RED)
	{
		bsp_lamp_ctrl(TRICOLOR_LAMP_RED, TURN_ON);
	}
	else
	{
		bsp_lamp_ctrl(TRICOLOR_LAMP_RED, TURN_OFF);
	}
	
	if(dataBuf[0] & TRICOLOR_LAMP_GREEN)
	{
		bsp_lamp_ctrl(TRICOLOR_LAMP_GREEN, TURN_ON);
	}
	else
	{
		bsp_lamp_ctrl(TRICOLOR_LAMP_GREEN, TURN_OFF);
	}
	
	if(dataBuf[0] & TRICOLOR_LAMP_YELLOW)
	{
		bsp_lamp_ctrl(TRICOLOR_LAMP_YELLOW, TURN_ON);
	}
	else
	{
		bsp_lamp_ctrl(TRICOLOR_LAMP_YELLOW, TURN_OFF);
	}
	
	eventMsg.msgType = EVENT_MSG_ORDER_STATUS;
	eventMsg.msg[0] = dataBuf[0] & 0x07;
	xQueueSend(eventMsgQueue, &eventMsg, 0);
	if(queue_send_ret == errQUEUE_FULL)
	{
		xQueueReceive(eventMsgQueue, &tmpMsg, 0);
		xQueueSend(eventMsgQueue, &eventMsg, 0);
	}
	
	return ret;
}

uint8_t wcs485_MotorStartStopCmd(uint8_t *dataBuf, uint8_t dataLen)
{
	uint8_t ret;
	eventMsgFrame_t eventMsg, tmpMsg;
	BaseType_t queue_send_ret;
	
	ret = wcs485_MotorStartStopCmdAckSend();
	
	if(dataBuf[0] == MOTOR_STATUS_START)
	{
		wcs485_MotorCtrl(TURN_ON);
		eventMsg.msg[0] = MOTOR_STATUS_START;
	}
	else
	{
		wcs485_MotorCtrl(TURN_OFF);
		eventMsg.msg[0] = MOTOR_STATUS_STOP;
	}
	
	eventMsg.msgType = EVENT_MSG_START_STOP;
	xQueueSend(eventMsgQueue, &eventMsg, 0);
	if(queue_send_ret == errQUEUE_FULL)
	{
		xQueueReceive(eventMsgQueue, &tmpMsg, 0);
		xQueueSend(eventMsgQueue, &eventMsg, 0);
	}
	
	return ret;
}
