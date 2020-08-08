#ifndef __CMT_SPI3_RF330_H
#define __CMT_SPI3_RF330_H

#include "typedefs.h"
#include "stm32f10x_conf.h"

__inline void cmt_spi3_delay_RF330(void);
__inline void cmt_spi3_delay_us_RF330(void);

void cmt_spi3_init_RF330(void);

void cmt_spi3_send_RF330(u8 data8);
u8 cmt_spi3_recv_RF330(void);

void cmt_spi3_write_RF330(u8 addr, u8 dat);
void cmt_spi3_read_RF330(u8 addr, u8* p_dat);

void cmt_spi3_write_fifo_RF330(const u8* p_buf, u16 len);
void cmt_spi3_read_fifo_RF330(u8* p_buf, u16 len);

#endif
