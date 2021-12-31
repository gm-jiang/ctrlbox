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
 * @file    cmt2300a_hal.c
 * @brief   CMT2300A hardware abstraction layer
 *
 * @version 1.2
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */

#include "cmt2300a_hal_rf315.h"
#include "cmt_spi3_rf315.h"

/*! ********************************************************
* @name    CMT2300A_InitGpio
* @desc    Initializes the CMT2300A interface GPIOs.
*
* *********************************************************/

void CMT2300A_GPIO_RF315(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_2);
}
void CMT2300A_InitGpio_RF315(void)
{
    //	 CMT2300A_SetGpio1In_RF315();
    //    CMT2300A_SetGpio2In_RF315();
    CMT2300A_SetGpio3In_RF315();
    //CMT2300A_GPIO_RF315();
    cmt_spi3_init_RF315();
}

/*! ********************************************************
* @name    CMT2300A_ReadReg
* @desc    Read the CMT2300A register at the specified address.
* @param   addr: register address
* @return  Register value
* *********************************************************/
u8 CMT2300A_ReadReg_RF315(u8 addr)
{
    u8 dat = 0xFF;
    cmt_spi3_read_RF315(addr, &dat);

    return dat;
}

/*! ********************************************************
* @name    CMT2300A_WriteReg
* @desc    Write the CMT2300A register at the specified address.
* @param   addr: register address
*          dat: register value
* *********************************************************/
void CMT2300A_WriteReg_RF315(u8 addr, u8 dat)
{
    cmt_spi3_write_RF315(addr, dat);
}

/*! ********************************************************
* @name    CMT2300A_ReadFifo
* @desc    Reads the contents of the CMT2300A FIFO.
* @param   buf: buffer where to copy the FIFO read data
*          len: number of bytes to be read from the FIFO
* *********************************************************/
void CMT2300A_ReadFifo_RF315(u8 buf[], u16 len)
{
    cmt_spi3_read_fifo_RF315(buf, len);
}

/*! ********************************************************
* @name    CMT2300A_WriteFifo
* @desc    Writes the buffer contents to the CMT2300A FIFO.
* @param   buf: buffer containing data to be put on the FIFO
*          len: number of bytes to be written to the FIFO
* *********************************************************/
void CMT2300A_WriteFifo_RF315(const u8 buf[], u16 len)
{
    cmt_spi3_write_fifo_RF315(buf, len);
}
