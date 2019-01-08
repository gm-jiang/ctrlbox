#ifndef _UHF_RFID_DRIVER_H_
#define _UHF_RFID_DRIVER_H_

#include <stdint.h>
#include "bsp_port.h"
#include "wcs_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

void bind_msg_init(void);
void save_lowrfid_msg(uint8_t *msg, uint8_t msg_len);
void save_uhfrfid_msg(uint8_t *msg, uint8_t msg_len);
void set_event_msg(eventMsgFrame_t *eventMsg, eventMsgType_e msgType);
void uhfrfid_send_cmd(void);

#ifdef __cplusplus
}
#endif

#endif
