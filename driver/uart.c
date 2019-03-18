#include <string.h>
#include "bsp_port.h"
#include "mt_common.h"
#include "uart.h"
#include "system_init.h"

static uint8_t rs485_send_data(uint8_t *buf, uint8_t length);
static uint8_t send_msg_to_center(uint8_t *data, uint8_t len, uint8_t type);

static uint8_t rs485_send_data(uint8_t *buf, uint8_t length)
{
	bsp_enable_485_pin();
	bsp_uart1_send(buf, length);
	mt_sleep_us(RS485_SEND_DELAY);
	bsp_disable_485_pin();
	return length;
}

static uint8_t send_msg_to_center(uint8_t *data, uint8_t len, uint8_t type)
{
	uint8_t buf[RS485_SEND_BUF] = {0};

	buf[0] = 0XAA;
	buf[1] = 0X55;
	buf[2] = len + 3;
	buf[3] = g_configInfo.addr;
	buf[4] = type;
	if (data != NULL && len != 0)
		memcpy(&buf[5], data, len);
	buf[len+5] = mt_cal_crc8(buf, len + 5);
	return rs485_send_data(buf, len + 6);
}

uint8_t send_event_to_center(uint8_t *data, uint8_t len, uint8_t type)
{
	return send_msg_to_center(data, len, type);
}

uint8_t send_ack_to_center(uint8_t *data, uint8_t len, uint8_t type)
{
	return send_msg_to_center(data, len, type);
}
