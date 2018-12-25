#include "mt_common.h"
#include "bsp_port.h"
#include "dbg_print.h"

void wcs_send_data(uint8_t *buf, uint16_t length)
{
	bsp_enable_485_pin();
	bsp_uart1_send(buf, length);
	mt_sleep_us(100);
	bsp_disable_485_pin();
	dbg_print_msg(PRINT_LEVEL_DEBUG, "WCS DATA OUT: -->", length, buf);
}
