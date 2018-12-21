#ifndef _WCS_485_DRIVER_H_
#define _WCS_485_DRIVER_H_

#include <stdint.h>
#include "bsp_port.h"

#ifdef __cplusplus
extern "C" {
#endif

void wcs_send_data(uint8_t *buf, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
