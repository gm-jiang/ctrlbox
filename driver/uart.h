#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RS485_SEND_DELAY    1500
#define RS485_SEND_BUF      128

uint8_t send_event_to_center(uint8_t *data, uint8_t len, uint8_t type);
uint8_t send_ack_to_center(uint8_t *data, uint8_t len, uint8_t type);

#ifdef __cplusplus
}
#endif

#endif
