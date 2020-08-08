#include "radio.h"
#include "radio_rf315.h"
#include "radio_rf330.h"
#include "radio_rf433.h"

void radio_init(void)
{
    RF315_Init();
    if(FALSE == CMT2300A_IsExist_RF315()) {
        while(1); //FIXME!!! just for debug
    } else {
        CMT2300A_GoStby_RF315();
        CMT2300A_ConfigGpio_RF315( CMT2300A_GPIO3_SEL_DOUT);
        CMT2300A_ConfigInterrupt_RF315(CMT2300A_INT_SEL_RX_FIFO_TH, CMT2300A_INT_SEL_PKT_OK);
        CMT2300A_EnableReadFifo_RF315();
        CMT2300A_ClearInterruptFlags_RF315();
        CMT2300A_ClearRxFifo_RF315();
        CMT2300A_GoRx_RF315();
    }

    RF330_Init();
    if(FALSE == CMT2300A_IsExist_RF330()) {
        while(1); //FIXME!!! just for debug
    } else {
        CMT2300A_GoStby_RF330();
        CMT2300A_ConfigGpio_RF330( CMT2300A_GPIO3_SEL_DOUT);
        CMT2300A_ConfigInterrupt_RF330(CMT2300A_INT_SEL_RX_FIFO_TH, CMT2300A_INT_SEL_PKT_OK); //  /* Config INT2 */
        CMT2300A_EnableReadFifo_RF330();
        CMT2300A_ClearInterruptFlags_RF330();
        CMT2300A_ClearRxFifo_RF330();
        CMT2300A_GoRx_RF330();
    }

    RF433_Init();
    if(FALSE == CMT2300A_IsExist_RF433()) {
        while(1); //FIXME!!! just for debug
    } else {
        CMT2300A_GoStby_RF433();
        CMT2300A_ConfigGpio_RF433( CMT2300A_GPIO3_SEL_DOUT);
        CMT2300A_ConfigInterrupt_RF433(CMT2300A_INT_SEL_RX_FIFO_TH, CMT2300A_INT_SEL_PKT_OK); //  /* Config INT2 */
        CMT2300A_EnableReadFifo_RF433();
        CMT2300A_ClearInterruptFlags_RF433();
        CMT2300A_ClearRxFifo_RF433();
        CMT2300A_GoRx_RF433();
    }

    RF_Init_RX();
    if(FALSE == CMT2300A_IsExist()) {
        while(1); //FIXME!!! just for debug
    } else {
        CMT2300A_GoStby();
        CMT2300A_ConfigGpio(CMT2300A_GPIO3_SEL_DOUT);
        CMT2300A_ConfigInterrupt(CMT2300A_INT_SEL_RX_FIFO_TH, CMT2300A_INT_SEL_PKT_OK); //  /* Config INT2 */
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableReadFifo();
        CMT2300A_ClearInterruptFlags();
        CMT2300A_ClearRxFifo();
        CMT2300A_GoRx();
    }
}
