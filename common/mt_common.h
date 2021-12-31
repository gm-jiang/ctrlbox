#ifndef _MT_COMMON_H
#define _MT_COMMON_H

#include <stdint.h>

#define SW_V         "1.04"
#define HW_V         "1.00"
#define VER_INFO_LEN 4

#define FIFO_TH 32

void mt_sleep_ms(uint16_t time);
void mt_sleep_us(uint32_t time_us);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint8_t mt_hex2ascii(uint8_t hex);
void mt_uhfrfid_convert(uint8_t *buff, uint8_t bufflen);
uint8_t mt_check_sum(uint8_t *ubuff, uint8_t ubufflen);
uint8_t mt_cal_crc8(uint8_t *ubuff, uint32_t ubufflen);
uint16_t mt_get_hw_ver(void);
uint16_t mt_get_sw_ver(void);
uint16_t mt_htons(uint16_t i16);
uint32_t mt_htonl(uint32_t i32);
uint64_t mt_htonll(uint64_t i64);
uint32_t mt_crc32(unsigned char *buf, unsigned int size);

#endif
