/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, CMOSTEK SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Copyright (C) CMOSTEK SZ.
 */

/*!
 * @file    radio.c
 * @brief   Generic radio handlers
 *
 * @version 1.2
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */

#include "radio_rf315.h"
#include "cmt2300a_params_rf315.h"
#include "cmt_spi3_rf315.h"
#include "cmt2300a_hal_rf315.h"
#include <string.h>

static EnumRF315Status g_nNextRF315State = RF315_STATE_IDLE;
static u8 *g_pRxBuffer = NULL;
static u8 *g_pTxBuffer = NULL;
static u16 g_nRxLength = 0;
static u16 g_nTxLength = 0;

static u32 g_nRxTimeout = INFINITE;
static u32 g_nTxTimeout = INFINITE;
static u32 g_nRxTimeCount = 0;
static u32 g_nTxTimeCount = 0;

static u8 g_nInterrutFlags = 0;

void RF315_Init(void)
{
    u8 tmp;

    CMT2300A_InitGpio_RF315();
    CMT2300A_Init_RF315();

    /* Config registers */
    CMT2300A_ConfigRegBank_RF315(CMT2300A_CMT_BANK_ADDR, g_cmt2300aCmtBank_RF315, CMT2300A_CMT_BANK_SIZE);
    CMT2300A_ConfigRegBank_RF315(CMT2300A_SYSTEM_BANK_ADDR, g_cmt2300aSystemBank_RF315, CMT2300A_SYSTEM_BANK_SIZE);
    CMT2300A_ConfigRegBank_RF315(CMT2300A_FREQUENCY_BANK_ADDR, g_cmt2300aFrequencyBank_RF315, CMT2300A_FREQUENCY_BANK_SIZE);
    CMT2300A_ConfigRegBank_RF315(CMT2300A_DATA_RATE_BANK_ADDR, g_cmt2300aDataRateBank_RF315, CMT2300A_DATA_RATE_BANK_SIZE);
    CMT2300A_ConfigRegBank_RF315(CMT2300A_BASEBAND_BANK_ADDR, g_cmt2300aBasebandBank_RF315, CMT2300A_BASEBAND_BANK_SIZE);
    CMT2300A_ConfigRegBank_RF315(CMT2300A_TX_BANK_ADDR, g_cmt2300aTxBank_RF315, CMT2300A_TX_BANK_SIZE);

    // xosc_aac_code[2:0] = 2
    tmp = (~0x07) & CMT2300A_ReadReg_RF315(CMT2300A_CUS_CMT10);
    CMT2300A_WriteReg_RF315(CMT2300A_CUS_CMT10, tmp | 0x02);

    RF315_Config();
}

void RF315_Config(void)
{
#ifdef ENABLE_ANTENNA_SWITCH_RF315
    /* If you enable antenna switch, GPIO1/GPIO2 will output RX_ACTIVE/TX_ACTIVE,
       and it can't output INT1/INT2 via GPIO1/GPIO2 */
    CMT2300A_EnableAntennaSwitch_RF315(0);

#else
    /* Config GPIOs */
    CMT2300A_ConfigGpio_RF315(
        //        CMT2300A_GPIO1_SEL_INT1 | /* INT1 > GPIO1 */
        //        CMT2300A_GPIO2_SEL_INT2 | /* INT2 > GPIO2 */
        CMT2300A_GPIO3_SEL_DOUT);

    /* Config interrupt */
    CMT2300A_ConfigInterrupt_RF315(CMT2300A_INT_SEL_TX_DONE, /* Config INT1 */
                                   CMT2300A_INT_SEL_PKT_OK   /* Config INT2 */
    );
#endif

    /* Enable interrupt */
    CMT2300A_EnableInterrupt_RF315(CMT2300A_MASK_TX_DONE_EN | CMT2300A_MASK_PKT_DONE_EN);

    /* Disable low frequency OSC calibration */
    CMT2300A_EnableLfosc_RF315(FALSE);

    /* Use a single 64-byte  FIFO for either Tx or Rx */
    CMT2300A_EnableFifoMerge_RF315(TRUE); //�ϲ�FIFO

    CMT2300A_SetFifoThreshold_RF315(FIFO_TH); // ������ֵ

    /* This is optional, only needed when using Rx fast frequency hopping */
    /* See AN142 and AN197 for details */
    //CMT2300A_SetAfcOvfTh(0x27);

    /* Go to sleep for configuration to take effect */

    CMT2300A_GoSleep_RF315();
}

void RF315_SetStatus(EnumRF315Status nStatus)
{
    g_nNextRF315State = nStatus;
}

EnumRF315Status RF315_GetStatus(void)
{
    return g_nNextRF315State;
}

u8 RF315_GetInterruptFlags(void)
{
    return g_nInterrutFlags;
}

void RF315_StartRx(u8 buf[], u16 len, u32 timeout)
{
    g_pRxBuffer = buf;
    g_nRxLength = len;
    g_nRxTimeout = timeout;

    memset(g_pRxBuffer, 0, g_nRxLength);

    g_nNextRF315State = RF315_STATE_RX_START;
}

void RF315_StartTx(u8 buf[], u16 len, u32 timeout)
{
    g_pTxBuffer = buf;
    g_nTxLength = len;
    g_nTxTimeout = timeout;

    g_nNextRF315State = RF315_STATE_TX_START;
}

EnumRF315Result RF315_Process(void)
{
    EnumRF315Result nRes = RF315_BUSY;

    switch (g_nNextRF315State) {
        case RF315_STATE_IDLE: {
            nRes = RF315_IDLE;
            break;
        }

        case RF315_STATE_RX_START: {
            CMT2300A_GoStby_RF315();
            CMT2300A_ClearInterruptFlags_RF315();

            /* Must clear FIFO after enable SPI to read or write the FIFO */
            CMT2300A_EnableReadFifo_RF315();
            CMT2300A_ClearRxFifo_RF315();

            if (FALSE == CMT2300A_GoRx_RF315())
                g_nNextRF315State = RF315_STATE_ERROR;
            else
                g_nNextRF315State = RF315_STATE_RX_WAIT;

            g_nRxTimeCount = CMT2300A_GetTickCount_RF315();

            break;
        }

        case RF315_STATE_RX_WAIT: {
#ifdef ENABLE_ANTENNA_SWITCH_RF315
            if (CMT2300A_MASK_PKT_OK_FLG & CMT2300A_ReadReg_RF315(CMT2300A_CUS_INT_FLAG)) /* Read PKT_OK flag */
#else
            // if(CMT2300A_ReadGpio2_RF315())  /* Read INT2, PKT_OK */
#endif
            {
                g_nNextRF315State = RF315_STATE_RX_DONE;
            }

            if ((INFINITE != g_nRxTimeout) && ((CMT2300A_GetTickCount_RF315() - g_nRxTimeCount) > g_nRxTimeout))
                g_nNextRF315State = RF315_STATE_RX_TIMEOUT;

            break;
        }

        case RF315_STATE_RX_DONE: {
            CMT2300A_GoStby_RF315();

            /* The length need be smaller than 32 */
            CMT2300A_ReadFifo_RF315(g_pRxBuffer, g_nRxLength);

            g_nInterrutFlags = CMT2300A_ClearInterruptFlags_RF315();

            CMT2300A_GoSleep_RF315();

            g_nNextRF315State = RF315_STATE_IDLE;
            nRes = RF315_RX_DONE;
            break;
        }

        case RF315_STATE_RX_TIMEOUT: {
            CMT2300A_GoSleep_RF315();

            g_nNextRF315State = RF315_STATE_IDLE;
            nRes = RF315_RX_TIMEOUT;
            break;
        }

        case RF315_STATE_TX_START: {
            CMT2300A_GoStby_RF315();
            CMT2300A_ClearInterruptFlags_RF315();

            /* Must clear FIFO after enable SPI to read or write the FIFO */
            CMT2300A_EnableWriteFifo_RF315();
            CMT2300A_ClearTxFifo_RF315();

            /* The length need be smaller than 32 */
            CMT2300A_WriteFifo_RF315(g_pTxBuffer, g_nTxLength);

            if (0 == (CMT2300A_MASK_TX_FIFO_NMTY_FLG & CMT2300A_ReadReg_RF315(CMT2300A_CUS_FIFO_FLAG)))
                g_nNextRF315State = RF315_STATE_ERROR;

            if (FALSE == CMT2300A_GoTx_RF315())
                g_nNextRF315State = RF315_STATE_ERROR;
            else
                g_nNextRF315State = RF315_STATE_TX_WAIT;

            g_nTxTimeCount = CMT2300A_GetTickCount_RF315();

            break;
        }

        case RF315_STATE_TX_WAIT: {
#ifdef ENABLE_ANTENNA_SWITCH_RF315
            if (CMT2300A_MASK_TX_DONE_FLG & CMT2300A_ReadReg_RF315(CMT2300A_CUS_INT_CLR1)) /* Read TX_DONE flag */
#else
            //if(CMT2300A_ReadGpio1_RF315())  /* Read INT1, TX_DONE */
#endif
            {
                g_nNextRF315State = RF315_STATE_TX_DONE;
            }

            if ((INFINITE != g_nTxTimeout) && ((CMT2300A_GetTickCount_RF315() - g_nTxTimeCount) > g_nTxTimeout))
                g_nNextRF315State = RF315_STATE_TX_TIMEOUT;

            break;
        }

        case RF315_STATE_TX_DONE: {
            CMT2300A_ClearInterruptFlags_RF315();
            CMT2300A_GoSleep_RF315();

            g_nNextRF315State = RF315_STATE_IDLE;
            nRes = RF315_TX_DONE;
            break;
        }

        case RF315_STATE_TX_TIMEOUT: {
            CMT2300A_GoSleep_RF315();

            g_nNextRF315State = RF315_STATE_IDLE;
            nRes = RF315_TX_TIMEOUT;
            break;
        }

        case RF315_STATE_ERROR: {
            CMT2300A_SoftReset_RF315();
            CMT2300A_DelayMs_RF315(20);

            CMT2300A_GoStby_RF315();
            RF315_Config();

            g_nNextRF315State = RF315_STATE_IDLE;
            nRes = RF315_ERROR;
            break;
        }

        default:
            break;
    }

    return nRes;
}
