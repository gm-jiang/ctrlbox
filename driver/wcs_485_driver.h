#ifndef _WCS_485_DRIVER_H_
#define _WCS_485_DRIVER_H_

#include <stdint.h>
#include "bsp_port.h"

#ifdef __cplusplus
extern "C" {
#endif
	
#define RS485_SEND_BUF      128
	
typedef enum type_msg {
	ACK_FROM_CTRL,
	GET_DEV_INFO,
	GET_SYS_LOG,
	GET_OTA_SW_VER = 0xB0,
	SET_OTA_CODE,
} type_msg_e;

void wcs_send_data(uint8_t *buf, uint16_t length);
uint8_t send_ack_to_center(uint8_t *data, uint8_t len, uint16_t tid);
uint8_t send_ota_ver_to_center(uint8_t *data, uint8_t len, uint16_t tid);
uint8_t send_ota_code_to_center(uint8_t *data, uint8_t len, uint16_t tid);

#ifdef __cplusplus
}
#endif

#endif
