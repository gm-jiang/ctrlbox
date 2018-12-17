#ifndef _UHF_RFID_H_
#define _UHF_RFID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bind_msg_init(void);
void save_lowrfid_msg(uint8_t *msg, uint8_t msg_len);
void save_uhfrfid_msg(uint8_t *msg, uint8_t msg_len);
void uhfrfid_send_cmd(void);

#ifdef __cplusplus
}
#endif

#endif
