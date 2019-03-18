#include "bsp_port.h"
#include "config.h"

uint8_t set_485_addr(uint8_t addr)
{
	uint16_t data = addr;
	bsp_FLASH_WriteMoreData(ADDR485_PARAM_ADDR, &data, 1);
	return addr;
}

uint8_t get_485_addr(void)
{
	uint16_t data;
	uint8_t mcu485Addr = 0;
	data = bsp_FLASH_ReadHalfWord(ADDR485_PARAM_ADDR);
	mcu485Addr = data & 0xff;
	return mcu485Addr;
}

uint8_t set_dev_sn(uint8_t *buf, uint8_t len)
{
	uint8_t i = 0;
	uint16_t temp_buf[DEVICE_SN_LEN];
	uint32_t tempaddress = SN_PARAM_ADDR;
	for (i = 0; i < DEVICE_SN_LEN; i++)
	{
		temp_buf[i] = (buf[i]) & 0xff;
	}

	bsp_FLASH_WriteMoreData(tempaddress, temp_buf, DEVICE_SN_LEN);
	return 0;
}

uint8_t get_dev_sn(uint8_t *buf)
{
	uint8_t i = 0;
	uint16_t temp_buf[DEVICE_SN_LEN];
	uint32_t tempaddress = SN_PARAM_ADDR;
	bsp_FLASH_ReadMoreData(tempaddress, temp_buf, DEVICE_SN_LEN);
	for (i = 0; i < DEVICE_SN_LEN; i++)
	{
		buf[i] = (temp_buf[i]) & 0xff;
	}
	return 0;
}

uint8_t set_dev_param(uint8_t *buf, uint8_t len)
{
	uint8_t i = 0;
	uint16_t temp_buf[DEVICE_PARM_LEN];
	uint32_t tempaddress = DEV_PARAM_ADDR;
	for (i = 0; i < DEVICE_PARM_LEN; i++)
	{
		temp_buf[i] = (buf[i]) & 0xff;
	}

	bsp_FLASH_WriteMoreData(tempaddress, temp_buf, DEVICE_PARM_LEN);
	return 0;
}

uint8_t get_dev_param(uint8_t *buf)
{
	uint8_t i = 0;
	uint16_t temp_buf[DEVICE_PARM_LEN];
	uint32_t tempaddress = DEV_PARAM_ADDR;
	bsp_FLASH_ReadMoreData(tempaddress, temp_buf, DEVICE_PARM_LEN);
	for (i = 0; i < DEVICE_PARM_LEN; i++)
	{
		buf[i] = (temp_buf[i]) & 0xff;
	}
	return 0;
}

uint8_t set_motor_ctrl_auth(uint8_t auth)
{
	uint16_t data = auth;
	bsp_FLASH_WriteMoreData(MOTOR_CTRL_AUTH_PARAM_ADDR, &data, 1);
	return auth;
}

uint8_t get_motor_ctrl_auth(void)
{
	uint16_t data;
	uint8_t auth = 0;
	data = bsp_FLASH_ReadHalfWord(MOTOR_CTRL_AUTH_PARAM_ADDR);
	auth = data & 0xff;
	return auth;
}

