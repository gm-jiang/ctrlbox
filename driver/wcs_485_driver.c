#include "mt_common.h"
#include <string.h>
#include "bsp_port.h"
#include "dbg_print.h"
#include "wcs_485_driver.h"

void wcs_send_data(uint8_t *buf, uint16_t length)
{
	bsp_enable_485_pin();
	bsp_uart1_send(buf, length);
	mt_sleep_us(1500);
	bsp_disable_485_pin();
	dbg_print_msg(PRINT_LEVEL_DEBUG, (uint8_t *)"UART1 OUT -->", length, buf);
}

uint8_t send_msg_to_center(uint8_t *data, uint8_t len, uint8_t type)
{
	uint8_t buf[RS485_SEND_BUF] = {0};

	buf[0] = 0XAA;
	buf[1] = 0X55;
	buf[2] = len + 3;
	buf[3] = g_mcu485Addr;
	buf[4] = type;
	memcpy(&buf[5], data, len);
	buf[len+5] = mt_cal_crc8(buf, len + 5);
	wcs_send_data(buf, len + 6);
	return 0;
}

uint8_t send_ack_to_center(uint8_t *data, uint8_t len, uint16_t tid)
{
	return send_msg_to_center(data, len, ACK_FROM_CTRL);
}

uint8_t send_ota_ver_to_center(uint8_t *data, uint8_t len, uint16_t tid)
{
	return send_msg_to_center(data, len, GET_OTA_SW_VER);
}

uint8_t send_ota_code_to_center(uint8_t *data, uint8_t len, uint16_t tid)
{
	return send_msg_to_center(data, len, SET_OTA_CODE);
}
