#include <string.h>
#include "ctrlbox_conf.h"
#include "wcs_parser.h"
#include "bsp_port.h"


uint8_t g_mcu485Addr = 0;
uint8_t g_mcuFuncConfig = 0;

//get 485 address
uint8_t get_485_addr(void)
{
	uint16_t data;
	uint8_t mcu485Addr = 0;
	data = bsp_FLASH_ReadHalfWord(MCU_485_ADDR_BASE);
	mcu485Addr = data & 0xff;
	return mcu485Addr;
}

//set 485 address
void set_485_addr(uint8_t addr)
{
	uint16_t data = addr;
	bsp_FLASH_WriteMoreData(MCU_485_ADDR_BASE, &data, 1);
}

//set device sn
uint8_t get_dev_sn(uint8_t *snBuf, uint8_t *snLen)
{
	uint16_t buf[WCS_485_DEVICE_SN_LEN];
	uint8_t i = 0;
	uint8_t j = 0;
	if(snBuf == NULL)
	{
		return RTN_FAIL;
	}

	bsp_FLASH_ReadMoreData(MCU_485_DEV_SN_BASE, buf, WCS_485_DEVICE_SN_LEN);
	for (i = 0; i < WCS_485_DEVICE_SN_LEN; i++, j++)
	{
		snBuf[i] = (buf[j]) & 0xff;
	}
	
	*snLen = i;

	return RTN_SUCCESS;
}

//get device sn
uint8_t set_dev_sn(uint8_t *snBuf, uint8_t snLen)
{
	uint16_t buf[WCS_485_DEVICE_SN_LEN];
	uint8_t i = 0;
	uint8_t j = 0;
	if(snBuf == NULL)
	{
		return RTN_FAIL;
	}
	for (i = 0; i < WCS_485_DEVICE_SN_LEN; i++, j++)
	{
		buf[i] = (snBuf[j]) & 0xff;
	}
	if(i != snLen)
		return RTN_FAIL;

	bsp_FLASH_WriteMoreData(MCU_485_DEV_SN_BASE, buf, snLen);

	return RTN_SUCCESS;
}

//get customer config
uint8_t get_customer_config(void)
{
	uint16_t data;
	uint8_t config = 0;
	data = bsp_FLASH_ReadHalfWord(MCU_485_DEV_CUSTOMER_BASE);
	config = data & 0xff;
	return config;
}

//set customer config
void set_customer_config(uint8_t config)
{
	uint16_t data = config;
	bsp_FLASH_WriteMoreData(MCU_485_DEV_CUSTOMER_BASE, &data, 1);
}
