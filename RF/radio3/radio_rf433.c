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
 
#include "radio_rf433.h"
#include "cmt2300a_params_rf433.h"
#include "cmt_spi3_rf433.h"
#include "cmt2300a_hal_rf433.h"
#include <string.h>

static EnumRF433Status g_nNextRF433State = RF433_STATE_IDLE;
static u8* g_pRxBuffer = NULL;
static u8* g_pTxBuffer = NULL;
static u16 g_nRxLength = 0;
static u16 g_nTxLength = 0;

static u32 g_nRxTimeout = INFINITE;
static u32 g_nTxTimeout = INFINITE;
static u32 g_nRxTimeCount = 0;
static u32 g_nTxTimeCount = 0;

static u8 g_nInterrutFlags = 0;

void RF433_Init(void)
{
    u8 tmp;
    
		CMT2300A_InitGpio_RF433();
		CMT2300A_Init_RF433();
    
    /* Config registers */
    CMT2300A_ConfigRegBank_RF433(CMT2300A_CMT_BANK_ADDR       , g_cmt2300aCmtBank_RF433       , CMT2300A_CMT_BANK_SIZE       );
    CMT2300A_ConfigRegBank_RF433(CMT2300A_SYSTEM_BANK_ADDR    , g_cmt2300aSystemBank_RF433    , CMT2300A_SYSTEM_BANK_SIZE    );
    CMT2300A_ConfigRegBank_RF433(CMT2300A_FREQUENCY_BANK_ADDR , g_cmt2300aFrequencyBank_RF433 , CMT2300A_FREQUENCY_BANK_SIZE );
    CMT2300A_ConfigRegBank_RF433(CMT2300A_DATA_RATE_BANK_ADDR , g_cmt2300aDataRateBank_RF433  , CMT2300A_DATA_RATE_BANK_SIZE );
    CMT2300A_ConfigRegBank_RF433(CMT2300A_BASEBAND_BANK_ADDR  , g_cmt2300aBasebandBank_RF433  , CMT2300A_BASEBAND_BANK_SIZE  );
    CMT2300A_ConfigRegBank_RF433(CMT2300A_TX_BANK_ADDR        , g_cmt2300aTxBank_RF433        , CMT2300A_TX_BANK_SIZE        );
    
    // xosc_aac_code[2:0] = 2
    tmp = (~0x07) & CMT2300A_ReadReg_RF433(CMT2300A_CUS_CMT10);
    CMT2300A_WriteReg_RF433(CMT2300A_CUS_CMT10, tmp|0x02);
    
	  RF433_Config();
}

void RF433_Config(void)
{
#ifdef ENABLE_ANTENNA_SWITCH_RF433
    /* If you enable antenna switch, GPIO1/GPIO2 will output RX_ACTIVE/TX_ACTIVE,
       and it can't output INT1/INT2 via GPIO1/GPIO2 */
    CMT2300A_EnableAntennaSwitch_RF433(0);
    
#else
    /* Config GPIOs */
        CMT2300A_ConfigGpio_RF433(
//        CMT2300A_GPIO1_SEL_INT1 | /* INT1 > GPIO1 */
//        CMT2300A_GPIO2_SEL_INT2 | /* INT2 > GPIO2 */
        CMT2300A_GPIO3_SEL_DOUT
        );
    
      /* Config interrupt */
        CMT2300A_ConfigInterrupt_RF433(
        CMT2300A_INT_SEL_TX_DONE, /* Config INT1 */
        CMT2300A_INT_SEL_PKT_OK   /* Config INT2 */
        );
#endif

    /* Enable interrupt */
      CMT2300A_EnableInterrupt_RF433(
        CMT2300A_MASK_TX_DONE_EN| 
        CMT2300A_MASK_PKT_DONE_EN
        );

    
    /* Disable low frequency OSC calibration */
    CMT2300A_EnableLfosc_RF433(FALSE);
    
    /* Use a single 64-byte  FIFO for either Tx or Rx */
    CMT2300A_EnableFifoMerge_RF433(TRUE); //合并FIFO
    
    CMT2300A_SetFifoThreshold_RF433(FIFO_TH); // 设置阈值
    
    /* This is optional, only needed when using Rx fast frequency hopping */
    /* See AN142 and AN197 for details */
    //CMT2300A_SetAfcOvfTh(0x27);
    
    /* Go to sleep for configuration to take effect */
			
    CMT2300A_GoSleep_RF433();
}


void RF433_SetStatus(EnumRF433Status nStatus)
{
    g_nNextRF433State = nStatus;
}

EnumRF433Status RF433_GetStatus(void)
{
    return g_nNextRF433State;
}

u8 RF433_GetInterruptFlags(void)
{
    return g_nInterrutFlags;
}

void RF433_StartRx(u8 buf[], u16 len, u32 timeout)
{
    g_pRxBuffer = buf;
    g_nRxLength = len;
    g_nRxTimeout = timeout;
    
    memset(g_pRxBuffer, 0, g_nRxLength);
    
    g_nNextRF433State = RF433_STATE_RX_START;
}

void RF433_StartTx(u8 buf[], u16 len, u32 timeout)
{
    g_pTxBuffer = buf;
    g_nTxLength = len;
    g_nTxTimeout = timeout;
    
    g_nNextRF433State = RF433_STATE_TX_START;
}

EnumRF433Result RF433_Process(void)
{
    EnumRF433Result nRes = RF433_BUSY;
    
    switch(g_nNextRF433State) 
    {
    case RF433_STATE_IDLE:
    {
        nRes = RF433_IDLE;
        break;
    }
    
    case RF433_STATE_RX_START:
    {
        CMT2300A_GoStby_RF433();
        CMT2300A_ClearInterruptFlags_RF433();
        
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableReadFifo_RF433();
        CMT2300A_ClearRxFifo_RF433();
        
        if(FALSE==CMT2300A_GoRx_RF433())
            g_nNextRF433State = RF433_STATE_ERROR;
        else
            g_nNextRF433State = RF433_STATE_RX_WAIT;
        
        g_nRxTimeCount = CMT2300A_GetTickCount_RF433();
        
        break;
    }
    
    case RF433_STATE_RX_WAIT:
    {
#ifdef ENABLE_ANTENNA_SWITCH_RF433
        if(CMT2300A_MASK_PKT_OK_FLG & CMT2300A_ReadReg_RF433(CMT2300A_CUS_INT_FLAG))  /* Read PKT_OK flag */
#else
       // if(CMT2300A_ReadGpio2_RF433())  /* Read INT2, PKT_OK */
#endif
        {
            g_nNextRF433State = RF433_STATE_RX_DONE;
        }
        
        if( (INFINITE != g_nRxTimeout) && ((CMT2300A_GetTickCount_RF433()-g_nRxTimeCount) > g_nRxTimeout) )
            g_nNextRF433State = RF433_STATE_RX_TIMEOUT;
        
        break;
    }
    
    case RF433_STATE_RX_DONE:
    {
        CMT2300A_GoStby_RF433();

        /* The length need be smaller than 32 */
        CMT2300A_ReadFifo_RF433(g_pRxBuffer, g_nRxLength);

        g_nInterrutFlags = CMT2300A_ClearInterruptFlags_RF433();
            
        CMT2300A_GoSleep_RF433();
        
        g_nNextRF433State = RF433_STATE_IDLE;
        nRes = RF433_RX_DONE;
        break;
    }
    
    case RF433_STATE_RX_TIMEOUT:
    {
        CMT2300A_GoSleep_RF433();
        
        g_nNextRF433State = RF433_STATE_IDLE;
        nRes = RF433_RX_TIMEOUT;
        break;
    }
    
    case RF433_STATE_TX_START:
    {
        CMT2300A_GoStby_RF433();
        CMT2300A_ClearInterruptFlags_RF433();
        
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableWriteFifo_RF433();
        CMT2300A_ClearTxFifo_RF433();
        
        /* The length need be smaller than 32 */
        CMT2300A_WriteFifo_RF433(g_pTxBuffer, g_nTxLength);
        
        if( 0==(CMT2300A_MASK_TX_FIFO_NMTY_FLG & CMT2300A_ReadReg_RF433(CMT2300A_CUS_FIFO_FLAG)) )
            g_nNextRF433State = RF433_STATE_ERROR;

        if(FALSE==CMT2300A_GoTx_RF433())
            g_nNextRF433State = RF433_STATE_ERROR;
        else
            g_nNextRF433State = RF433_STATE_TX_WAIT;
        
        g_nTxTimeCount = CMT2300A_GetTickCount_RF433();
        
        break;
    }
        
    case RF433_STATE_TX_WAIT:
    {
#ifdef ENABLE_ANTENNA_SWITCH_RF433
        if(CMT2300A_MASK_TX_DONE_FLG & CMT2300A_ReadReg_RF433(CMT2300A_CUS_INT_CLR1))  /* Read TX_DONE flag */
#else
        //if(CMT2300A_ReadGpio1_RF433())  /* Read INT1, TX_DONE */
#endif
        {
            g_nNextRF433State = RF433_STATE_TX_DONE;
        }
        
        if( (INFINITE != g_nTxTimeout) && ((CMT2300A_GetTickCount_RF433()-g_nTxTimeCount) > g_nTxTimeout) )
            g_nNextRF433State = RF433_STATE_TX_TIMEOUT;
            
        break;
    }
            
    case RF433_STATE_TX_DONE:
    {
        CMT2300A_ClearInterruptFlags_RF433();
        CMT2300A_GoSleep_RF433();

        g_nNextRF433State = RF433_STATE_IDLE;
        nRes = RF433_TX_DONE;
        break;
    }
    
    case RF433_STATE_TX_TIMEOUT:
    {
        CMT2300A_GoSleep_RF433();
        
        g_nNextRF433State = RF433_STATE_IDLE;
        nRes = RF433_TX_TIMEOUT;
        break;
    }
    
    case RF433_STATE_ERROR:
    {
        CMT2300A_SoftReset_RF433();
        CMT2300A_DelayMs_RF433(20);
        
        CMT2300A_GoStby_RF433();
        RF433_Config();
        
        g_nNextRF433State = RF433_STATE_IDLE;
        nRes = RF433_ERROR;
        break;
    }
    
    default:
        break;
    }
    
    return nRes;
}
