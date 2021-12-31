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

#ifndef __CMT2300A_HAL_RF433_H
#define __CMT2300A_HAL_RF433_H

#include "typedefs.h"
#include "bsp_port.h"
#include "mt_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************************************
*  The following need to be modified by user
*  ************************************************************************ */
#define CMT2300A_SetGpio1In_RF433() SET_GPIO_IN(CMT_GPIO1_GPIO_RF433)
#define CMT2300A_SetGpio2In_RF433() SET_GPIO_IN(CMT_GPIO2_GPIO_RF433)
#define CMT2300A_SetGpio3In_RF433() SET_GPIO_IN(CMT_GPIO3_GPIO_RF433)
#define CMT2300A_ReadGpio1_RF433()  READ_GPIO_PIN(CMT_GPIO1_GPIO_RF433)
#define CMT2300A_ReadGpio2_RF433()  READ_GPIO_PIN(CMT_GPIO2_GPIO_RF433)
#define CMT2300A_ReadGpio3_RF433()  READ_GPIO_PIN(CMT_GPIO3_GPIO_RF433)
#define RFIN_RF433                  READ_GPIO_PIN(CMT_GPIO3_GPIO_RF433)
//#define      RFIN	       GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_12 )
#define CMT2300A_DelayMs_RF433(ms)    delay_ms(ms)
#define CMT2300A_DelayUs_RF433(us)    delay_us(us)
#define CMT2300A_GetTickCount_RF433() 0 //g_nSysTickCount todo
                                        /* ************************************************************************ */

void CMT2300A_GPIO_RF433(void);

void CMT2300A_InitGpio_RF433(void);

u8 CMT2300A_ReadReg_RF433(u8 addr);
void CMT2300A_WriteReg_RF433(u8 addr, u8 dat);

void CMT2300A_ReadFifo_RF433(u8 buf[], u16 len);
void CMT2300A_WriteFifo_RF433(const u8 buf[], u16 len);

#ifdef __cplusplus
}
#endif

#endif
