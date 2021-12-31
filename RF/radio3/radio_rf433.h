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

#ifndef __RADIO_RF433_H
#define __RADIO_RF433_H

#include "typedefs.h"
#include "cmt2300a_rf433.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RF state machine */
typedef enum {
    RF433_STATE_IDLE = 0,
    RF433_STATE_RX_START,
    RF433_STATE_RX_WAIT,
    RF433_STATE_RX_DONE,
    RF433_STATE_RX_TIMEOUT,
    RF433_STATE_TX_START,
    RF433_STATE_TX_WAIT,
    RF433_STATE_TX_DONE,
    RF433_STATE_TX_TIMEOUT,
    RF433_STATE_ERROR,
} EnumRF433Status;

/* RF process function results */
typedef enum {
    RF433_IDLE = 0,
    RF433_BUSY,
    RF433_RX_DONE,
    RF433_RX_TIMEOUT,
    RF433_TX_DONE,
    RF433_TX_TIMEOUT,
    RF433_ERROR,
} EnumRF433Result;

//#define ENABLE_ANTENNA_SWITCH

void RF433_Init(void);
void RF433_Config(void);

void RF433_SetStatus(EnumRF433Status nStatus);
EnumRF433Status RF433_GetStatus(void);
u8 RF433_GetInterruptFlags(void);

void RF433_StartRx(u8 buf[], u16 len, u32 timeout);
void RF433_StartTx(u8 buf[], u16 len, u32 timeout);

EnumRF433Result RF433_Process(void);

#ifdef __cplusplus
}
#endif

#endif
