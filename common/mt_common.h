#ifndef _MT_COMMON_H
#define _MT_COMMON_H

#include <stdint.h>

void mt_delay_ms(uint16_t time);
void mt_uhfrfid_convert(uint8_t *buff, uint8_t bufflen);
uint8_t mt_check_sum(uint8_t *ubuff, uint8_t ubufflen);

#endif
