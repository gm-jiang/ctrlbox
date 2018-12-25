#ifndef _MT_COMMON_H
#define _MT_COMMON_H

#include <stdint.h>

void mt_sleep_ms(uint16_t time);
void mt_sleep_us(uint32_t time_us);
uint8_t mt_hex2ascii(uint8_t hex);
void mt_uhfrfid_convert(uint8_t *buff, uint8_t bufflen);
uint8_t mt_check_sum(uint8_t *ubuff, uint8_t ubufflen);
uint8_t mt_cal_crc8(uint8_t *ubuff, uint32_t ubufflen);

#endif
