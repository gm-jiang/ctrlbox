#ifndef _RFID_PARSER_H_
#define _RFID_PARSER_H_

#include <stdint.h>

void chain_down_msg_process(uint8_t *payload, uint8_t payload_len);
void trigger_uhf_rfid_process(uint8_t *buf, uint8_t len);
void uhfrfid_message_process(uint8_t *buf, uint8_t len);
void lowrfid_check_process(uint8_t *buf, uint8_t len);

#endif
