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
 * @file    cmt2300a.h
 * @brief   CMT2300A transceiver RF chip driver
 *
 * @version 1.3
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */

#ifndef __CMT2300A_RF330_H
#define __CMT2300A_RF330_H

#include "typedefs.h"
#include "cmt2300a_defs.h"
#include "cmt2300a_hal_rf330.h"

#ifdef __cplusplus 
extern "C" { 
#endif

#define ENABLE_AUTO_SWITCH_CHIP_STATUS_RF330   /* Enable the auto switch chip status */

/* ************************************************************************
   The following are for chip status controls.
*  ************************************************************************ */
void CMT2300A_SoftReset_RF330(void);
u8 CMT2300A_GetChipStatus_RF330(void);
BOOL CMT2300A_AutoSwitchStatus_RF330(u8 nGoCmd);
BOOL CMT2300A_GoSleep_RF330(void);
BOOL CMT2300A_GoStby_RF330(void);
BOOL CMT2300A_GoTFS_RF330(void);
BOOL CMT2300A_GoRFS_RF330(void);
BOOL CMT2300A_GoTx_RF330(void);
BOOL CMT2300A_GoRx_RF330(void);


/* ************************************************************************
*  The following are for chip interrupts, GPIO, FIFO operations.
*  ************************************************************************ */
void CMT2300A_ConfigGpio_RF330(u8 nGpioSel);
void CMT2300A_ConfigInterrupt_RF330(u8 nInt1Sel, u8 nInt2Sel);
void CMT2300A_SetInterruptPolar_RF330(BOOL bActiveHigh);
void CMT2300A_SetFifoThreshold_RF330(u8 nFifoThreshold);
void CMT2300A_EnableAntennaSwitch_RF330(u8 nMode);
void CMT2300A_EnableInterrupt_RF330(u8 nEnable);
void CMT2300A_EnableRxFifoAutoClear_RF330(BOOL bEnable);
void CMT2300A_EnableFifoMerge_RF330(BOOL bEnable);
void CMT2300A_EnableReadFifo_RF330(void);
void CMT2300A_EnableWriteFifo_RF330(void);
void CMT2300A_RestoreFifo_RF330(void);
u8 CMT2300A_ClearTxFifo_RF330(void);
u8 CMT2300A_ClearRxFifo_RF330(void);
u8 CMT2300A_ClearInterruptFlags_RF330(void);


/* ************************************************************************
*  The following are for Tx DIN operations in direct mode.
*  ************************************************************************ */
void CMT2300A_ConfigTxDin_RF330(u8 nDinSel);
void CMT2300A_EnableTxDin_RF330(BOOL bEnable);
void CMT2300A_EnableTxDinInvert_RF330(BOOL bEnable);


/* ************************************************************************
*  The following are general operations.
*  ************************************************************************ */
BOOL CMT2300A_IsExist_RF330(void);
u8 CMT2300A_GetRssiCode_RF330(void);
int CMT2300A_GetRssiDBm_RF330(void);
void CMT2300A_SetFrequencyChannel_RF330(u8 nChann);
void CMT2300A_SetFrequencyStep_RF330(u8 nOffset);
void CMT2300A_SetPayloadLength_RF330(u16 nLength);
void CMT2300A_EnableLfosc_RF330(BOOL bEnable);
void CMT2300A_EnableLfoscOutput_RF330(BOOL bEnable);
void CMT2300A_EnableAfc_RF330(BOOL bEnable);
void CMT2300A_SetAfcOvfTh_RF330(u8 afcOvfTh);


/* ************************************************************************
*  The following are for chip initializes.
*  ************************************************************************ */
void CMT2300A_Init_RF330(void);
BOOL CMT2300A_ConfigRegBank_RF330(u8 base_addr, const u8 bank[], u8 len);

#ifdef __cplusplus
} 
#endif

#endif
