#include "stm32f10x.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_port.h" //use watchdog and led
#include "system_init.h"
#include "dbg_print.h"

#include "FreeRTOS.h"
#include "task.h"

#include "user_task.h"
#include "wcs_parser.h"
#include "rfid_parser.h"
#include "upgrade.h"

#include "uart.h"
#include "config.h"
#include "mt_common.h"

#define INDEX_LEN           2
#define INDEX_DAT           3

static void ctrlbox_status_led(void);

static void wcs_message_process(uint8_t *payload, uint8_t payload_length);
static void lowrfid_message_process(uint8_t *payload, uint8_t payload_length);

static void wcs_poll_msg_process(uint8_t *payload, uint8_t payload_len);
static void wcs_reset_msg_process(uint8_t *payload, uint8_t payload_len);

static void wcs_set_rfid_msg_process(uint8_t *payload, uint8_t payload_len);
static void wcs_set_led_msg_process(uint8_t *payload, uint8_t payload_len);
static void wcs_set_moto_msg_process(uint8_t *payload, uint8_t payload_len);
static void wcs_set_valve_msg_process(uint8_t *payload, uint8_t payload_len);

static void wcs_ver_process(uint8_t *payload, uint8_t payload_len);
static void wcs_ota_process(uint8_t *payload, uint8_t payload_len);

static void set_485_addr_prccess(uint8_t *buf, uint8_t len);
static void set_dev_sn_prccess(uint8_t *buf, uint8_t len);
static void get_addr_sn_prccess(void);
static void set_parm_process(uint8_t *buf, uint8_t len);
static void get_parm_process(void);
static void get_sw_hw_version_process(void);
static void get_motor_ctrl_auth_process(void);
static void set_motor_ctrl_auth_process(uint8_t *buf, uint8_t len);

void task_led_status(void *pvParameters)
{
	while(1)
	{
		ctrlbox_status_led();
		bsp_IWDG_feed();
	}
}

//receive wcs msg task
void task_uart1_receive(void *pvParameters)
{
	BaseType_t ret;
	uint8_t recv_buf[UART1_QUEUE_LEN] = {0};

	while(1)
	{
		ret = xQueueReceive(UART1RecvMsgQueue, &recv_buf, portMAX_DELAY);
		if (ret == pdTRUE)
		{
			dbg_print_msg(PRINT_LEVEL_INFO, "UART1 IN <--", recv_buf[INDEX_LEN] + 3, recv_buf);
			wcs_message_process(&recv_buf[INDEX_DAT], recv_buf[INDEX_LEN]);
		}
	}
}

//receive stc msg task
void task_uart2_receive(void *pvParameters)
{
	BaseType_t ret;
	uint8_t recv_buf[UART2_QUEUE_LEN] = {0};
	static uint8_t last_rfid[LOW_RFID_ID_LEN] = {0};

	while(1)
	{
		ret = xQueueReceive(UART2RecvMsgQueue, &recv_buf, portMAX_DELAY);
		if (ret == pdTRUE)
		{
			if (memcmp(last_rfid, &recv_buf[INDEX_DAT], LOW_RFID_ID_LEN) != 0)
			{
				dbg_print_msg(PRINT_LEVEL_DEBUG, "UART2 IN <--", recv_buf[INDEX_LEN] + 3, recv_buf);
				memcpy(last_rfid, &recv_buf[INDEX_DAT], LOW_RFID_ID_LEN);
				lowrfid_message_process(&recv_buf[INDEX_DAT], recv_buf[INDEX_LEN] - 1);
			}
		}
	}
}

void task_uart3_receive(void *pvParameters)
{
	BaseType_t ret;
	uint8_t recv_buf[LOW_RFID_ID_LEN] = {0};

	while(1)
	{
		ret = xQueueReceive(LowRfidMsgQueue, recv_buf, portMAX_DELAY);
		if (ret == pdTRUE)
		{
			uhfrfid_message_process(recv_buf, LOW_RFID_ID_LEN);
		}
	}
}

static void ctrlbox_status_led(void)
{
	if (g_configInfo.mode == CTRLBOX_CHAIN_DOWN)
	{
		bsp_status_red_led_set(1);
		vTaskDelay(500);
		bsp_status_red_led_set(0);
		vTaskDelay(500);
	}
	if (g_configInfo.mode == CTRLBOX_UHF_RFID)
	{
		bsp_status_yel_led_set(1);
		vTaskDelay(500);
		bsp_status_yel_led_set(0);
		vTaskDelay(500);
	}
}

/* AA 55 LEN |<-------PAYLOAD---------->| CRC8 */
/* AA 55 LEN |<--ADDR TYPE DATA .....-->| CRC8 */
static void wcs_message_process(uint8_t *payload, uint8_t payload_length)
{
	uint8_t payload_len = payload_length - ADDR_LEN - TYPE_LEN - CRC_LEN;
	/*payload[0] == shortaddr  payload[1] == type  payload[2] == data... */

	//Add boardcast address filter function
	switch (payload[1])
	{
	//poll cmd
	case MT_WCS_POLL_MSG:
		wcs_poll_msg_process(&payload[DAT_INDEX], payload_len);
		break;

	//set low rfid cmd
	case MT_WCS_SET_RFID_MSG:
		wcs_set_rfid_msg_process(&payload[DAT_INDEX], payload_len);
		break;
	//control led cmd
	case MT_WCS_SET_LED_MSG:
		wcs_set_led_msg_process(&payload[DAT_INDEX], payload_len);
		break;
	//set motor cmd
	case MT_WCS_SET_MOTO_MSG:
		wcs_set_moto_msg_process(&payload[DAT_INDEX], payload_len);
		break;
	//valve test mode cmd
	case MT_WCS_SET_VALVE_MSG:
		wcs_set_valve_msg_process(&payload[DAT_INDEX], payload_len);
		break;
	case MT_WCS_RESET_MSG:
		wcs_reset_msg_process(&payload[DAT_INDEX], payload_len);

	//ota version cmd
	case MT_WCS_GET_VER_MSG:
		wcs_ver_process(&payload[DAT_INDEX], payload_len);
		break;
	//ota data packet
	case MT_WCS_SET_OTA_MSG:
		wcs_ota_process(&payload[DAT_INDEX], payload_len);
		break;

	//set ctrlbox addr cmd
	case MT_WCS_SET_ADDR_MSG:
		set_485_addr_prccess(&payload[DAT_INDEX], payload_len);
		break;
	//set ctrlbox sn cmd
	case MT_WCS_SET_SN_MSG:
		set_dev_sn_prccess(&payload[DAT_INDEX], payload_len);
		break;
	//get ctrlbox addr & sn cmd
	case MT_WCS_GET_ADDR_SN_MSG:
		get_addr_sn_prccess();
		break;
	//set ctrlbox config info cmd
	case MT_WCS_SET_CONFIG_INFO_MSG:
		set_parm_process(&payload[DAT_INDEX], payload_len);
		break;
	//get ctrlbox config info cmd
	case MT_WCS_GET_CONFIG_INFO_MSG:
		get_parm_process();
		break;
	//get ctrlbox sw & hw info cmd
	case MT_WCS_GET_SW_HW_VER_MSG:
		get_sw_hw_version_process();
		break;

	//get motor ctrl auth for ctrlbox
	case MT_WCS_GET_MOTOR_CTRL_AUTH_MSG:
		get_motor_ctrl_auth_process();
		break;
	//set motor ctrl auth for ctrlbox
	case MT_WCS_SET_MOTOR_CTRL_AUTH_MSG:
		set_motor_ctrl_auth_process(&payload[DAT_INDEX], payload_len);
		break;

	default:
		dbg_print(PRINT_LEVEL_ERROR, "message_process: CMD not handled\r\n");
		break;
	}
}

/* AA 55 LEN PAYLOAD..... CRC  AA 55 LEN [RFID DATA... CRC] */
static void lowrfid_message_process(uint8_t *payload, uint8_t payload_length)
{
	switch (g_configInfo.mode)
	{
	case CTRLBOX_CHAIN_DOWN:
		chain_down_msg_process(payload, payload_length);
		break;
	case CTRLBOX_UHF_RFID:
		trigger_uhf_rfid_process(payload, payload_length);
		break;
	case CTRLBOX_RFID_CHECK:
		lowrfid_check_process(payload, payload_length);
		break;

	case CTRLBOX_RFID_DEBUG:
		break;

	default:
		dbg_print(PRINT_LEVEL_ERROR, "unknown ctrlbox mode\r\n");
		break;
	}
}

static void wcs_poll_msg_process(uint8_t *payload, uint8_t payload_len)
{
	/*add business code*/
	BaseType_t ret;
	eventMsg_t eventMsg;

	//dbg_print(PRINT_LEVEL_DEBUG, "Poll message received\r\n");
	ret = xQueueReceive(UpLoadEventMsgQueue, &eventMsg, 0);
	if(ret == pdTRUE)
		wcs_send_event_msg(&eventMsg);
	else
		wcs_send_event_msg(NULL);
}

static void wcs_reset_msg_process(uint8_t *payload, uint8_t payload_len)
{
	/*add business code*/
	dbg_print(PRINT_LEVEL_DEBUG, "System reset message received\r\n");
	send_ack_to_center(NULL, 0, MT_WCS_RESET_MSG);
	while(1); //wait watch dog reset system!!!
}

static void wcs_set_rfid_msg_process(uint8_t *payload, uint8_t payload_len)
{
	/*add business code*/
	//save the low rfid tag info
	dbg_print(PRINT_LEVEL_DEBUG, "Low RFID tagID message received\r\n");
	wcs_add_chaindown_data(payload, payload_len);
}

static void wcs_set_led_msg_process(uint8_t *payload, uint8_t payload_len)
{
	/*add business code*/
	dbg_print(PRINT_LEVEL_DEBUG, "Set LED status message received\r\n");
	wcs_set_led_status(payload, payload_len);
}

static void wcs_set_moto_msg_process(uint8_t *payload, uint8_t payload_len)
{
	/*add business code*/
	dbg_print(PRINT_LEVEL_DEBUG, "Set MOTOR status message received\r\n");
	wcs_set_motor_status(payload, payload_len);
}

static void wcs_set_valve_msg_process(uint8_t *payload, uint8_t payload_len)
{
	/*add business code*/
	dbg_print(PRINT_LEVEL_DEBUG, "Valve test mode message received\r\n");
	wcs_valve_test_handle(payload, payload_len);
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
		dbg_print(PRINT_LEVEL_DEBUG, "Start download sw: %04x hw: %04x firmware\r\n", \
							ota_ver_msg.sw, ota_ver_msg.hw);
		mtOtaCbs.pfnOtaVerHandler(&ota_ver_msg);
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
		dbg_print(PRINT_LEVEL_DEBUG, "Sequence: %d size: %dByte\r\n", ota_code_msg.seq, ota_code_msg.size);
		mtOtaCbs.pfnOtaDataHandler(&ota_code_msg);
	}
}

static void set_485_addr_prccess(uint8_t *buf, uint8_t len)
{
	/*add business code*/
	dbg_print(PRINT_LEVEL_DEBUG, "Set 485 address message received\r\n");
	set_485_addr(buf[0]);
	g_configInfo.addr = get_485_addr();
	send_ack_to_center(&g_configInfo.addr, DEVICE_ADDR_LEN, MT_WCS_SET_ADDR_MSG);
}

static void set_dev_sn_prccess(uint8_t *buf, uint8_t len)
{
	/*add business code*/
	dbg_print(PRINT_LEVEL_DEBUG, "Set serial num message received\r\n");
	set_dev_sn(buf, len);
	get_dev_sn(g_configInfo.sn);
	send_ack_to_center(g_configInfo.sn, DEVICE_SN_LEN, MT_WCS_SET_SN_MSG);
}

static void get_addr_sn_prccess(void)
{
	/*add business code*/
	uint8_t temp_buf[DEVICE_SN_LEN + DEVICE_ADDR_LEN] = {0};
	dbg_print(PRINT_LEVEL_DEBUG, "Get address & serial num message received\r\n");
	temp_buf[0] = g_configInfo.addr;
	memcpy(&temp_buf[1], g_configInfo.sn, DEVICE_SN_LEN);
	send_ack_to_center(temp_buf, DEVICE_SN_LEN + DEVICE_ADDR_LEN, MT_WCS_GET_ADDR_SN_MSG);
}

static void set_parm_process(uint8_t *buf, uint8_t len)
{
	/*add business code*/
	uint8_t temp_buf[DEVICE_PARM_LEN+1] = {0};
	dbg_print(PRINT_LEVEL_DEBUG, "Set parm message received\r\n");
	set_dev_param(&buf[1], len - 1);
	get_dev_param(&temp_buf[1]);
	temp_buf[0] = g_configInfo.mode;
	memcpy(&g_configInfo.ctrlLogic, &temp_buf[1], DEVICE_PARM_LEN);
	send_ack_to_center(temp_buf, DEVICE_PARM_LEN+1, MT_WCS_SET_CONFIG_INFO_MSG);
}

static void get_parm_process(void)
{
	/*add business code*/
	uint8_t temp_buf[DEVICE_PARM_LEN+1] = {0};
	dbg_print(PRINT_LEVEL_DEBUG, "Get parm message received\r\n");
	memcpy(&temp_buf[1], &g_configInfo.ctrlLogic, DEVICE_PARM_LEN);
	temp_buf[0] = g_configInfo.mode;
	send_ack_to_center(temp_buf, DEVICE_PARM_LEN+1, MT_WCS_GET_CONFIG_INFO_MSG);
}

static void get_sw_hw_version_process(void)
{
	/*add business code*/
	uint8_t temp_buf[VER_INFO_LEN + VER_INFO_LEN];
	dbg_print(PRINT_LEVEL_DEBUG, "Get sw hw version message received\r\n");
	memcpy(&temp_buf[0], (uint8_t *)&HW_V, VER_INFO_LEN);
	memcpy(&temp_buf[VER_INFO_LEN], (uint8_t *)&SW_V, VER_INFO_LEN);
	send_ack_to_center(temp_buf, VER_INFO_LEN + VER_INFO_LEN, MT_WCS_GET_SW_HW_VER_MSG);
}

static void get_motor_ctrl_auth_process(void)
{
	uint8_t auth = 0;
	auth = get_motor_ctrl_auth();
	send_ack_to_center(&auth, 1, MT_WCS_GET_MOTOR_CTRL_AUTH_MSG);
}

static void set_motor_ctrl_auth_process(uint8_t *buf, uint8_t len)
{
	uint8_t auth = 0;
	set_motor_ctrl_auth(buf[0]);
	auth = get_motor_ctrl_auth();
	g_configInfo.motor_auth = auth;
	send_ack_to_center(&auth, 1, MT_WCS_SET_MOTOR_CTRL_AUTH_MSG);
}
