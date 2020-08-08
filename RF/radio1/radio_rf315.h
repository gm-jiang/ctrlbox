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
 * @file    radio.h
 * @brief   Generic radio handlers
 *
 * @version 1.2
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */
 
#ifndef __RADIO_RF315_H
#define __RADIO_RF315_H

#include "typedefs.h"
#include "cmt2300a_rf315.h"

#ifdef __cplusplus 
extern "C" { 
#endif

/* RF state machine */
typedef enum {
    RF315_STATE_IDLE = 0,
    RF315_STATE_RX_START,
    RF315_STATE_RX_WAIT,
    RF315_STATE_RX_DONE,
    RF315_STATE_RX_TIMEOUT,
    RF315_STATE_TX_START,
    RF315_STATE_TX_WAIT,
    RF315_STATE_TX_DONE,
    RF315_STATE_TX_TIMEOUT,
    RF315_STATE_ERROR,
} EnumRF315Status;

/* RF process function results */
typedef enum {
    RF315_IDLE = 0,
    RF315_BUSY,
    RF315_RX_DONE,
    RF315_RX_TIMEOUT,
    RF315_TX_DONE,
    RF315_TX_TIMEOUT,
    RF315_ERROR,
} EnumRF315Result;

//#define ENABLE_ANTENNA_SWITCH

void RF315_Init(void);
void RF315_Config(void);

void RF315_SetStatus(EnumRF315Status nStatus);
EnumRF315Status RF315_GetStatus(void);
u8 RF315_GetInterruptFlags(void);

void RF315_StartRx(u8 buf[], u16 len, u32 timeout);
void RF315_StartTx(u8 buf[], u16 len, u32 timeout);



EnumRF315Result RF315_Process(void);

#ifdef __cplusplus 
} 
#endif

#endif
