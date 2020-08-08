
#include "cmt_spi3_rf315.h"
#include "bsp_port.h"

/* ************************************************************************
*  The following need to be modified by user
*  ************************************************************************ */
#define cmt_spi3_csb_rf315_out()      SET_GPIO_OUT(CMT_CSB_GPIO_RF315)  //CMT_CSB_GPIO_RF315 
//#define cmt_spi3_fcsb_rf315_out()     SET_GPIO_OUT(CMT_FCSB_GPIO_RF315)
#define cmt_spi3_sclk_rf315_out()     SET_GPIO_OUT(CMT_SCLK_GPIO_RF315)
#define cmt_spi3_sdio_rf315_out()     SET_GPIO_OUT(CMT_SDIO_GPIO_RF315)
#define cmt_spi3_sdio_rf315_in()      SET_GPIO_IN(CMT_SDIO_GPIO_RF315)

#define cmt_spi3_csb_rf315_1()        SET_GPIO_H(CMT_CSB_GPIO_RF315)
#define cmt_spi3_csb_rf315_0()        SET_GPIO_L(CMT_CSB_GPIO_RF315)

//#define cmt_spi3_fcsb_rf315_1()       SET_GPIO_H(CMT_FCSB_GPIO_RF315)
//#define cmt_spi3_fcsb_rf315_0()       SET_GPIO_L(CMT_FCSB_GPIO_RF315)
    
#define cmt_spi3_sclk_rf315_1()       SET_GPIO_H(CMT_SCLK_GPIO_RF315)
#define cmt_spi3_sclk_rf315_0()       SET_GPIO_L(CMT_SCLK_GPIO_RF315)

#define cmt_spi3_sdio_rf315_1()       SET_GPIO_H(CMT_SDIO_GPIO_RF315)
#define cmt_spi3_sdio_rf315_0()       SET_GPIO_L(CMT_SDIO_GPIO_RF315)
#define cmt_spi3_sdio_rf315_read()    READ_GPIO_PIN(CMT_SDIO_GPIO_RF315)
/* ************************************************************************ */
 
void cmt_spi3_delay_RF315(void)
{
    u32 n = 1; //7
    while(n--);
}


void cmt_spi3_delay_us_RF315(void)
{
    u16 n = 8;
    while(n--);
}

void cmt_spi3_init_RF315(void)
{
    cmt_spi3_csb_rf315_out();
    cmt_spi3_csb_rf315_out();
    cmt_spi3_csb_rf315_1();   /* CSB has an internal pull-up resistor */
    
    cmt_spi3_sclk_rf315_0();
    cmt_spi3_sclk_rf315_out();
    cmt_spi3_sclk_rf315_0();   /* SCLK has an internal pull-down resistor */
    
    cmt_spi3_sdio_rf315_1();
    cmt_spi3_sdio_rf315_out();
    cmt_spi3_sdio_rf315_1();
    
//    cmt_spi3_fcsb_rf315_1();
//    cmt_spi3_fcsb_rf315_out();
//    cmt_spi3_fcsb_rf315_1();  /* FCSB has an internal pull-up resistor */

    cmt_spi3_delay_RF315();
}

void cmt_spi3_send_RF315(u8 data8)
{
    u8 i;

    for(i=0; i<8; i++)
    {
       cmt_spi3_sclk_rf315_0();

        /* Send byte on the rising edge of SCLK */
        if(data8 & 0x80)
            cmt_spi3_sdio_rf315_1();
        else            
            cmt_spi3_sdio_rf315_0();

        cmt_spi3_delay_RF315();

        data8 <<= 1;
        cmt_spi3_sclk_rf315_1();
        cmt_spi3_delay_RF315();
    }
}

u8 cmt_spi3_recv_RF315(void)
{
    u8 i;
    u8 data8 = 0xFF;

    for(i=0; i<8; i++)
    {
        cmt_spi3_sclk_rf315_0();
        cmt_spi3_delay_RF315();
        data8 <<= 1;

        cmt_spi3_sclk_rf315_1();

        /* Read byte on the rising edge of SCLK */
        if(cmt_spi3_sdio_rf315_read())
            data8 |= 0x01;
        else
            data8 &= ~0x01;

        cmt_spi3_delay_RF315();
    }

    return data8;
}

void cmt_spi3_write_RF315(u8 addr, u8 dat)
{
    cmt_spi3_sdio_rf315_1();
    cmt_spi3_sdio_rf315_out();

    cmt_spi3_sclk_rf315_0();
    cmt_spi3_sclk_rf315_out();
    cmt_spi3_sclk_rf315_0(); 

//    cmt_spi3_fcsb_rf315_1();
//    cmt_spi3_fcsb_rf315_out();
//    cmt_spi3_fcsb_rf315_1();

    cmt_spi3_csb_rf315_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay_RF315();
    cmt_spi3_delay_RF315();

    /* r/w = 0 */
    cmt_spi3_send_RF315(addr&0x7F);

    cmt_spi3_send_RF315(dat);

    cmt_spi3_sclk_rf315_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay_RF315();
    cmt_spi3_delay_RF315();

    cmt_spi3_csb_rf315_1();
    
    cmt_spi3_sdio_rf315_1();
    cmt_spi3_sdio_rf315_in();
    
//    cmt_spi3_fcsb_rf315_1();    
}

void cmt_spi3_read_RF315(u8 addr, u8* p_dat)
{
    cmt_spi3_sdio_rf315_1();
    cmt_spi3_sdio_rf315_out();

    cmt_spi3_sclk_rf315_0();
    cmt_spi3_sclk_rf315_out();
    cmt_spi3_sclk_rf315_0(); 

//    cmt_spi3_fcsb_rf315_1();
//    cmt_spi3_fcsb_rf315_out();
//    cmt_spi3_fcsb_rf315_1();

    cmt_spi3_csb_rf315_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay_RF315();
    cmt_spi3_delay_RF315();

    /* r/w = 1 */
    cmt_spi3_send_RF315(addr|0x80);

    /* Must set SDIO to input before the falling edge of SCLK */
    cmt_spi3_sdio_rf315_in();
    
    *p_dat = cmt_spi3_recv_RF315();

    cmt_spi3_sclk_rf315_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay_RF315();
    cmt_spi3_delay_RF315();

   cmt_spi3_csb_rf315_1();
    
    cmt_spi3_sdio_rf315_1();
    cmt_spi3_sdio_rf315_in();
    
//    cmt_spi3_fcsb_rf315_1();
}

void cmt_spi3_write_fifo_RF315(const u8* p_buf, u16 len)
{
    u16 i;

//    cmt_spi3_fcsb_rf315_1();
//    cmt_spi3_fcsb_rf315_out();
//    cmt_spi3_fcsb_rf315_1();

    cmt_spi3_csb_rf315_1();
    cmt_spi3_csb_rf315_out();
    cmt_spi3_csb_rf315_1();

    cmt_spi3_sclk_rf315_0();
    cmt_spi3_sclk_rf315_out();
    cmt_spi3_sclk_rf315_0();

    cmt_spi3_sdio_rf315_out();

    for(i=0; i<len; i++)
    {
//        cmt_spi3_fcsb_rf315_0();

        /* > 1 SCLK cycle */
        cmt_spi3_delay_RF315();
        cmt_spi3_delay_RF315();

        cmt_spi3_send_RF315(p_buf[i]);

        cmt_spi3_sclk_rf315_0();

        /* > 2 us */
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();

//        cmt_spi3_fcsb_rf315_1();

        /* > 4 us */
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
    }

    cmt_spi3_sdio_rf315_in();
    
//    cmt_spi3_fcsb_rf315_1();
}

void cmt_spi3_read_fifo_RF315(u8* p_buf, u16 len)
{
    u16 i;

//    cmt_spi3_fcsb_rf315_1();
//    cmt_spi3_fcsb_rf315_out();
//    cmt_spi3_fcsb_rf315_1();

    cmt_spi3_csb_rf315_1();
    cmt_spi3_csb_rf315_out();
    cmt_spi3_csb_rf315_1();

    cmt_spi3_sclk_rf315_0();
    cmt_spi3_sclk_rf315_out();
    cmt_spi3_sclk_rf315_0();

    cmt_spi3_sdio_rf315_in();

    for(i=0; i<len; i++)
    {
//        cmt_spi3_fcsb_rf315_0();

        /* > 1 SCLK cycle */
        cmt_spi3_delay_RF315();
        cmt_spi3_delay_RF315();

        p_buf[i] = cmt_spi3_recv_RF315();

        cmt_spi3_sclk_rf315_0();

        /* > 2 us */
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();

//        cmt_spi3_fcsb_rf315_1();

        /* > 4 us */
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
        cmt_spi3_delay_us_RF315();
    }

    cmt_spi3_sdio_rf315_in();
    
//    cmt_spi3_fcsb_rf315_1();
}
