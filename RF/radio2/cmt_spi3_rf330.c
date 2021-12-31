
#include "cmt_spi3_rf330.h"
#include "bsp_port.h"

/* ************************************************************************
*  The following need to be modified by user
*  ************************************************************************ */
#define cmt_spi3_csb_rf330_out() SET_GPIO_OUT(CMT_CSB_GPIO_RF330) //CMT_CSB_GPIO_RF330
//#define cmt_spi3_fcsb_rf330_out()     SET_GPIO_OUT(CMT_FCSB_GPIO_RF330)
#define cmt_spi3_sclk_rf330_out() SET_GPIO_OUT(CMT_SCLK_GPIO_RF330)
#define cmt_spi3_sdio_rf330_out() SET_GPIO_OUT(CMT_SDIO_GPIO_RF330)
#define cmt_spi3_sdio_rf330_in()  SET_GPIO_IN(CMT_SDIO_GPIO_RF330)

#define cmt_spi3_csb_rf330_1() SET_GPIO_H(CMT_CSB_GPIO_RF330)
#define cmt_spi3_csb_rf330_0() SET_GPIO_L(CMT_CSB_GPIO_RF330)

//#define cmt_spi3_fcsb_rf330_1()       SET_GPIO_H(CMT_FCSB_GPIO_RF330)
//#define cmt_spi3_fcsb_rf330_0()       SET_GPIO_L(CMT_FCSB_GPIO_RF330)

#define cmt_spi3_sclk_rf330_1() SET_GPIO_H(CMT_SCLK_GPIO_RF330)
#define cmt_spi3_sclk_rf330_0() SET_GPIO_L(CMT_SCLK_GPIO_RF330)

#define cmt_spi3_sdio_rf330_1()    SET_GPIO_H(CMT_SDIO_GPIO_RF330)
#define cmt_spi3_sdio_rf330_0()    SET_GPIO_L(CMT_SDIO_GPIO_RF330)
#define cmt_spi3_sdio_rf330_read() READ_GPIO_PIN(CMT_SDIO_GPIO_RF330)
/* ************************************************************************ */

void cmt_spi3_delay_RF330(void)
{
    u32 n = 1; //7
    while (n--)
        ;
}

void cmt_spi3_delay_us_RF330(void)
{
    u16 n = 8;
    while (n--)
        ;
}

void cmt_spi3_init_RF330(void)
{
    cmt_spi3_csb_rf330_out();
    cmt_spi3_csb_rf330_out();
    cmt_spi3_csb_rf330_1(); /* CSB has an internal pull-up resistor */

    cmt_spi3_sclk_rf330_0();
    cmt_spi3_sclk_rf330_out();
    cmt_spi3_sclk_rf330_0(); /* SCLK has an internal pull-down resistor */

    cmt_spi3_sdio_rf330_1();
    cmt_spi3_sdio_rf330_out();
    cmt_spi3_sdio_rf330_1();

    //    cmt_spi3_fcsb_rf330_1();
    //    cmt_spi3_fcsb_rf330_out();
    //    cmt_spi3_fcsb_rf330_1();  /* FCSB has an internal pull-up resistor */

    cmt_spi3_delay_RF330();
}

void cmt_spi3_send_RF330(u8 data8)
{
    u8 i;

    for (i = 0; i < 8; i++) {
        cmt_spi3_sclk_rf330_0();

        /* Send byte on the rising edge of SCLK */
        if (data8 & 0x80)
            cmt_spi3_sdio_rf330_1();
        else
            cmt_spi3_sdio_rf330_0();

        cmt_spi3_delay_RF330();

        data8 <<= 1;
        cmt_spi3_sclk_rf330_1();
        cmt_spi3_delay_RF330();
    }
}

u8 cmt_spi3_recv_RF330(void)
{
    u8 i;
    u8 data8 = 0xFF;

    for (i = 0; i < 8; i++) {
        cmt_spi3_sclk_rf330_0();
        cmt_spi3_delay_RF330();
        data8 <<= 1;

        cmt_spi3_sclk_rf330_1();

        /* Read byte on the rising edge of SCLK */
        if (cmt_spi3_sdio_rf330_read())
            data8 |= 0x01;
        else
            data8 &= ~0x01;

        cmt_spi3_delay_RF330();
    }

    return data8;
}

void cmt_spi3_write_RF330(u8 addr, u8 dat)
{
    cmt_spi3_sdio_rf330_1();
    cmt_spi3_sdio_rf330_out();

    cmt_spi3_sclk_rf330_0();
    cmt_spi3_sclk_rf330_out();
    cmt_spi3_sclk_rf330_0();

    //    cmt_spi3_fcsb_rf330_1();
    //    cmt_spi3_fcsb_rf330_out();
    //    cmt_spi3_fcsb_rf330_1();

    cmt_spi3_csb_rf330_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay_RF330();
    cmt_spi3_delay_RF330();

    /* r/w = 0 */
    cmt_spi3_send_RF330(addr & 0x7F);

    cmt_spi3_send_RF330(dat);

    cmt_spi3_sclk_rf330_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay_RF330();
    cmt_spi3_delay_RF330();

    cmt_spi3_csb_rf330_1();

    cmt_spi3_sdio_rf330_1();
    cmt_spi3_sdio_rf330_in();

    //    cmt_spi3_fcsb_rf330_1();
}

void cmt_spi3_read_RF330(u8 addr, u8 *p_dat)
{
    cmt_spi3_sdio_rf330_1();
    cmt_spi3_sdio_rf330_out();

    cmt_spi3_sclk_rf330_0();
    cmt_spi3_sclk_rf330_out();
    cmt_spi3_sclk_rf330_0();

    //    cmt_spi3_fcsb_rf330_1();
    //    cmt_spi3_fcsb_rf330_out();
    //    cmt_spi3_fcsb_rf330_1();

    cmt_spi3_csb_rf330_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay_RF330();
    cmt_spi3_delay_RF330();

    /* r/w = 1 */
    cmt_spi3_send_RF330(addr | 0x80);

    /* Must set SDIO to input before the falling edge of SCLK */
    cmt_spi3_sdio_rf330_in();

    *p_dat = cmt_spi3_recv_RF330();

    cmt_spi3_sclk_rf330_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay_RF330();
    cmt_spi3_delay_RF330();

    cmt_spi3_csb_rf330_1();

    cmt_spi3_sdio_rf330_1();
    cmt_spi3_sdio_rf330_in();

    //    cmt_spi3_fcsb_rf330_1();
}

void cmt_spi3_write_fifo_RF330(const u8 *p_buf, u16 len)
{
    u16 i;

    //    cmt_spi3_fcsb_rf330_1();
    //    cmt_spi3_fcsb_rf330_out();
    //    cmt_spi3_fcsb_rf330_1();

    cmt_spi3_csb_rf330_1();
    cmt_spi3_csb_rf330_out();
    cmt_spi3_csb_rf330_1();

    cmt_spi3_sclk_rf330_0();
    cmt_spi3_sclk_rf330_out();
    cmt_spi3_sclk_rf330_0();

    cmt_spi3_sdio_rf330_out();

    for (i = 0; i < len; i++) {
        //        cmt_spi3_fcsb_rf330_0();

        /* > 1 SCLK cycle */
        cmt_spi3_delay_RF330();
        cmt_spi3_delay_RF330();

        cmt_spi3_send_RF330(p_buf[i]);

        cmt_spi3_sclk_rf330_0();

        /* > 2 us */
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();

        //        cmt_spi3_fcsb_rf330_1();

        /* > 4 us */
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
    }

    cmt_spi3_sdio_rf330_in();

    //    cmt_spi3_fcsb_rf330_1();
}

void cmt_spi3_read_fifo_RF330(u8 *p_buf, u16 len)
{
    u16 i;

    //    cmt_spi3_fcsb_rf330_1();
    //    cmt_spi3_fcsb_rf330_out();
    //    cmt_spi3_fcsb_rf330_1();

    cmt_spi3_csb_rf330_1();
    cmt_spi3_csb_rf330_out();
    cmt_spi3_csb_rf330_1();

    cmt_spi3_sclk_rf330_0();
    cmt_spi3_sclk_rf330_out();
    cmt_spi3_sclk_rf330_0();

    cmt_spi3_sdio_rf330_in();

    for (i = 0; i < len; i++) {
        //        cmt_spi3_fcsb_rf330_0();

        /* > 1 SCLK cycle */
        cmt_spi3_delay_RF330();
        cmt_spi3_delay_RF330();

        p_buf[i] = cmt_spi3_recv_RF330();

        cmt_spi3_sclk_rf330_0();

        /* > 2 us */
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();

        //        cmt_spi3_fcsb_rf330_1();

        /* > 4 us */
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
        cmt_spi3_delay_us_RF330();
    }

    cmt_spi3_sdio_rf330_in();

    //    cmt_spi3_fcsb_rf330_1();
}
