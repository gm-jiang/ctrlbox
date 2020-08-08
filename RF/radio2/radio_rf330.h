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
 
#ifndef __RADIO_RF330_H
#define __RADIO_RF330_H

#include "typedefs.h"
#include "cmt2300a_rf330.h"

#ifdef __cplusplus 
extern "C" { 
#endif

/* RF state machine */
typedef enum {
    RF330_STATE_IDLE = 0,
    RF330_STATE_RX_START,
    RF330_STATE_RX_WAIT,
    RF330_STATE_RX_DONE,
    RF330_STATE_RX_TIMEOUT,
    RF330_STATE_TX_START,
    RF330_STATE_TX_WAIT,
    RF330_STATE_TX_DONE,
    RF330_STATE_TX_TIMEOUT,
    RF330_STATE_ERROR,
} EnumRF330Status;

/* RF process function results */
typedef enum {
    RF330_IDLE = 0,
    RF330_BUSY,
    RF330_RX_DONE,
    RF330_RX_TIMEOUT,
    RF330_TX_DONE,
    RF330_TX_TIMEOUT,
    RF330_ERROR,
} EnumRF330Result;

//#define ENABLE_ANTENNA_SWITCH

void RF330_Init(void);
void RF330_Config(void);

void RF330_SetStatus(EnumRF330Status nStatus);
EnumRF330Status RF330_GetStatus(void);
u8 RF330_GetInterruptFlags(void);

void RF330_StartRx(u8 buf[], u16 len, u32 timeout);
void RF330_StartTx(u8 buf[], u16 len, u32 timeout);



EnumRF330Result RF330_Process(void);

#ifdef __cplusplus 
} 
#endif

#endif
