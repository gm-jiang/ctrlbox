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
 
#include "radio_rf330.h"
#include "cmt2300a_params_rf330.h"
#include "cmt_spi3_rf330.h"
#include "cmt2300a_hal_rf330.h"
#include <string.h>

static EnumRF330Status g_nNextRF330State = RF330_STATE_IDLE;
static u8* g_pRxBuffer = NULL;
static u8* g_pTxBuffer = NULL;
static u16 g_nRxLength = 0;
static u16 g_nTxLength = 0;

static u32 g_nRxTimeout = INFINITE;
static u32 g_nTxTimeout = INFINITE;
static u32 g_nRxTimeCount = 0;
static u32 g_nTxTimeCount = 0;

static u8 g_nInterrutFlags = 0;

void RF330_Init(void)
{
    u8 tmp;
    
		CMT2300A_InitGpio_RF330();
		CMT2300A_Init_RF330();
    
    /* Config registers */
    CMT2300A_ConfigRegBank_RF330(CMT2300A_CMT_BANK_ADDR       , g_cmt2300aCmtBank_RF330       , CMT2300A_CMT_BANK_SIZE       );
    CMT2300A_ConfigRegBank_RF330(CMT2300A_SYSTEM_BANK_ADDR    , g_cmt2300aSystemBank_RF330    , CMT2300A_SYSTEM_BANK_SIZE    );
    CMT2300A_ConfigRegBank_RF330(CMT2300A_FREQUENCY_BANK_ADDR , g_cmt2300aFrequencyBank_RF330 , CMT2300A_FREQUENCY_BANK_SIZE );
    CMT2300A_ConfigRegBank_RF330(CMT2300A_DATA_RATE_BANK_ADDR , g_cmt2300aDataRateBank_RF330  , CMT2300A_DATA_RATE_BANK_SIZE );
    CMT2300A_ConfigRegBank_RF330(CMT2300A_BASEBAND_BANK_ADDR  , g_cmt2300aBasebandBank_RF330  , CMT2300A_BASEBAND_BANK_SIZE  );
    CMT2300A_ConfigRegBank_RF330(CMT2300A_TX_BANK_ADDR        , g_cmt2300aTxBank_RF330        , CMT2300A_TX_BANK_SIZE        );
    
    // xosc_aac_code[2:0] = 2
    tmp = (~0x07) & CMT2300A_ReadReg_RF330(CMT2300A_CUS_CMT10);
    CMT2300A_WriteReg_RF330(CMT2300A_CUS_CMT10, tmp|0x02);
    
	  RF330_Config();
}

void RF330_Config(void)
{
#ifdef ENABLE_ANTENNA_SWITCH_RF330
    /* If you enable antenna switch, GPIO1/GPIO2 will output RX_ACTIVE/TX_ACTIVE,
       and it can't output INT1/INT2 via GPIO1/GPIO2 */
    CMT2300A_EnableAntennaSwitch_RF330(0);
    
#else
    /* Config GPIOs */
        CMT2300A_ConfigGpio_RF330(
//        CMT2300A_GPIO1_SEL_INT1 | /* INT1 > GPIO1 */
//        CMT2300A_GPIO2_SEL_INT2 | /* INT2 > GPIO2 */
        CMT2300A_GPIO3_SEL_DOUT
        );
    
      /* Config interrupt */
        CMT2300A_ConfigInterrupt_RF330(
        CMT2300A_INT_SEL_TX_DONE, /* Config INT1 */
        CMT2300A_INT_SEL_PKT_OK   /* Config INT2 */
        );
#endif

    /* Enable interrupt */
      CMT2300A_EnableInterrupt_RF330(
        CMT2300A_MASK_TX_DONE_EN| 
        CMT2300A_MASK_PKT_DONE_EN
        );

    
    /* Disable low frequency OSC calibration */
    CMT2300A_EnableLfosc_RF330(FALSE);
    
    /* Use a single 64-byte  FIFO for either Tx or Rx */
    CMT2300A_EnableFifoMerge_RF330(TRUE); //合并FIFO
    
    CMT2300A_SetFifoThreshold_RF330(FIFO_TH); // 设置阈值
    
    /* This is optional, only needed when using Rx fast frequency hopping */
    /* See AN142 and AN197 for details */
    //CMT2300A_SetAfcOvfTh(0x27);
    
    /* Go to sleep for configuration to take effect */
			
    CMT2300A_GoSleep_RF330();
}


void RF330_SetStatus(EnumRF330Status nStatus)
{
    g_nNextRF330State = nStatus;
}

EnumRF330Status RF330_GetStatus(void)
{
    return g_nNextRF330State;
}

u8 RF330_GetInterruptFlags(void)
{
    return g_nInterrutFlags;
}

void RF330_StartRx(u8 buf[], u16 len, u32 timeout)
{
    g_pRxBuffer = buf;
    g_nRxLength = len;
    g_nRxTimeout = timeout;
    
    memset(g_pRxBuffer, 0, g_nRxLength);
    
    g_nNextRF330State = RF330_STATE_RX_START;
}

void RF330_StartTx(u8 buf[], u16 len, u32 timeout)
{
    g_pTxBuffer = buf;
    g_nTxLength = len;
    g_nTxTimeout = timeout;
    
    g_nNextRF330State = RF330_STATE_TX_START;
}

EnumRF330Result RF330_Process(void)
{
    EnumRF330Result nRes = RF330_BUSY;
    
    switch(g_nNextRF330State) 
    {
    case RF330_STATE_IDLE:
    {
        nRes = RF330_IDLE;
        break;
    }
    
    case RF330_STATE_RX_START:
    {
        CMT2300A_GoStby_RF330();
        CMT2300A_ClearInterruptFlags_RF330();
        
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableReadFifo_RF330();
        CMT2300A_ClearRxFifo_RF330();
        
        if(FALSE==CMT2300A_GoRx_RF330())
            g_nNextRF330State = RF330_STATE_ERROR;
        else
            g_nNextRF330State = RF330_STATE_RX_WAIT;
        
        g_nRxTimeCount = CMT2300A_GetTickCount_RF330();
        
        break;
    }
    
    case RF330_STATE_RX_WAIT:
    {
#ifdef ENABLE_ANTENNA_SWITCH_RF330
        if(CMT2300A_MASK_PKT_OK_FLG & CMT2300A_ReadReg_RF330(CMT2300A_CUS_INT_FLAG))  /* Read PKT_OK flag */
#else
       // if(CMT2300A_ReadGpio2_RF330())  /* Read INT2, PKT_OK */
#endif
        {
            g_nNextRF330State = RF330_STATE_RX_DONE;
        }
        
        if( (INFINITE != g_nRxTimeout) && ((CMT2300A_GetTickCount_RF330()-g_nRxTimeCount) > g_nRxTimeout) )
            g_nNextRF330State = RF330_STATE_RX_TIMEOUT;
        
        break;
    }
    
    case RF330_STATE_RX_DONE:
    {
        CMT2300A_GoStby_RF330();

        /* The length need be smaller than 32 */
        CMT2300A_ReadFifo_RF330(g_pRxBuffer, g_nRxLength);

        g_nInterrutFlags = CMT2300A_ClearInterruptFlags_RF330();
            
        CMT2300A_GoSleep_RF330();
        
        g_nNextRF330State = RF330_STATE_IDLE;
        nRes = RF330_RX_DONE;
        break;
    }
    
    case RF330_STATE_RX_TIMEOUT:
    {
        CMT2300A_GoSleep_RF330();
        
        g_nNextRF330State = RF330_STATE_IDLE;
        nRes = RF330_RX_TIMEOUT;
        break;
    }
    
    case RF330_STATE_TX_START:
    {
        CMT2300A_GoStby_RF330();
        CMT2300A_ClearInterruptFlags_RF330();
        
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableWriteFifo_RF330();
        CMT2300A_ClearTxFifo_RF330();
        
        /* The length need be smaller than 32 */
        CMT2300A_WriteFifo_RF330(g_pTxBuffer, g_nTxLength);
        
        if( 0==(CMT2300A_MASK_TX_FIFO_NMTY_FLG & CMT2300A_ReadReg_RF330(CMT2300A_CUS_FIFO_FLAG)) )
            g_nNextRF330State = RF330_STATE_ERROR;

        if(FALSE==CMT2300A_GoTx_RF330())
            g_nNextRF330State = RF330_STATE_ERROR;
        else
            g_nNextRF330State = RF330_STATE_TX_WAIT;
        
        g_nTxTimeCount = CMT2300A_GetTickCount_RF330();
        
        break;
    }
        
    case RF330_STATE_TX_WAIT:
    {
#ifdef ENABLE_ANTENNA_SWITCH_RF330
        if(CMT2300A_MASK_TX_DONE_FLG & CMT2300A_ReadReg_RF330(CMT2300A_CUS_INT_CLR1))  /* Read TX_DONE flag */
#else
        //if(CMT2300A_ReadGpio1_RF330())  /* Read INT1, TX_DONE */
#endif
        {
            g_nNextRF330State = RF330_STATE_TX_DONE;
        }
        
        if( (INFINITE != g_nTxTimeout) && ((CMT2300A_GetTickCount_RF330()-g_nTxTimeCount) > g_nTxTimeout) )
            g_nNextRF330State = RF330_STATE_TX_TIMEOUT;
            
        break;
    }
            
    case RF330_STATE_TX_DONE:
    {
        CMT2300A_ClearInterruptFlags_RF330();
        CMT2300A_GoSleep_RF330();

        g_nNextRF330State = RF330_STATE_IDLE;
        nRes = RF330_TX_DONE;
        break;
    }
    
    case RF330_STATE_TX_TIMEOUT:
    {
        CMT2300A_GoSleep_RF330();
        
        g_nNextRF330State = RF330_STATE_IDLE;
        nRes = RF330_TX_TIMEOUT;
        break;
    }
    
    case RF330_STATE_ERROR:
    {
        CMT2300A_SoftReset_RF330();
        CMT2300A_DelayMs_RF330(20);
        
        CMT2300A_GoStby_RF330();
        RF330_Config();
        
        g_nNextRF330State = RF330_STATE_IDLE;
        nRes = RF330_ERROR;
        break;
    }
    
    default:
        break;
    }
    
    return nRes;
}
