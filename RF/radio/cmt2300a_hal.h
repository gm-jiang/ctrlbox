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
 * @file    cmt2300a_hal.h
 * @brief   CMT2300A hardware abstraction layer
 *
 * @version 1.2
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */

#ifndef __CMT2300A_HAL_H
#define __CMT2300A_HAL_H

#include "typedefs.h"
#include "bsp_port.h"
#include "cmt_spi3.h"
#include "mt_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************************************
*  The following need to be modified by user
*  ************************************************************************ */
#define CMT2300A_SetGpio1out() SET_GPIO_OUT(CMT_GPIO1_GPIO)
#define CMT2300A_SetGpio3out() SET_GPIO_OUT(CMT_GPIO3_GPIO) //2020-5-25
//#define CMT2300A_SetGpio1In()           SET_GPIO_IN(CMT_GPIO1_GPIO)
//#define CMT2300A_SetGpio2In()           SET_GPIO_IN(CMT_GPIO2_GPIO)
#define CMT2300A_SetGpio3In() SET_GPIO_IN(CMT_GPIO3_GPIO)
//#define CMT2300A_ReadGpio1()            READ_GPIO_PIN(CMT_GPIO1_GPIO)
//#define CMT2300A_ReadGpio2()            READ_GPIO_PIN(CMT_GPIO2_GPIO)
#define CMT2300A_ReadGpio3() READ_GPIO_PIN(CMT_GPIO3_GPIO)
#define RFIN_RF430           READ_GPIO_PIN(CMT_GPIO3_GPIO)
//#define      RFIN	       GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_12 )
#define CMT2300A_DelayMs(ms)    delay_ms(ms)
#define CMT2300A_DelayUs(us)    delay_us(us)
#define CMT2300A_GetTickCount() 0 //g_nSysTickCount //todo
/* ************************************************************************ */

void CMT2300A_GPIO(void);
void CMT2300A_InitGpio_TX(void);
void CMT2300A_InitGpio_RX(void);

u8 CMT2300A_ReadReg(u8 addr);
void CMT2300A_WriteReg(u8 addr, u8 dat);

void CMT2300A_ReadFifo(u8 buf[], u16 len);
void CMT2300A_WriteFifo(const u8 buf[], u16 len);

#ifdef __cplusplus
}
#endif

#endif
