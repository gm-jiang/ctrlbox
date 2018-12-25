#include "stm32f10x.h"
#include "mt_common.h"
#include <string.h>

void mt_sleep_ms(uint16_t time)
{    
   uint16_t i=0;
	
   while(time--)
   {
      i=8500;
      while(i--); 
   }
}

void mt_sleep_us(uint32_t time_us)
{
	uint32_t i = 0;
	for(i = 0;i < time_us;i++){
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}
}

uint8_t mt_hex2ascii(uint8_t hex)
{
	if((hex >= 0x30) && (hex <= 0x39))//0~9
			hex -= 0x30;
	else if((hex >= 0x41) && (hex <= 0x46))//A~Z
			hex -= 0x37;
	else if((hex >= 0x61) && (hex <= 0x66))//a~z
			hex -= 0x57;
	else hex = 0xff;

	return hex;
}

void mt_uhfrfid_convert(uint8_t *buff, uint8_t bufflen)
{
	uint8_t i;
	uint8_t tmp[30] = {0};

	for (i = 0; i < bufflen; i++)
	{
		tmp[i] = (mt_hex2ascii(buff[i*2])<<4 |  mt_hex2ascii(buff[i*2+1]));
	}
	memcpy(buff, tmp, bufflen);
}

uint8_t mt_check_sum(uint8_t *ubuff, uint8_t ubufflen)
{
	uint8_t i, usum = 0;

	for(i = 0; i < ubufflen; i++)
	{
		usum = usum + ubuff[i];
	}
	usum = (~usum) + 1;

	return usum;
}

uint8_t mt_cal_crc8(uint8_t *ubuff, uint32_t ubufflen)
{
	uint8_t crc = 0;
	uint8_t i;

	while(ubufflen--)
	{
		crc ^= *ubuff++;
		for(i = 0;i < 8;i++)
		{
			if(crc & 0x01)
			{
				crc = (crc >> 1) ^ 0x8C;
			}
			else 
				crc >>= 1;
		}
	}
	return crc;
}
