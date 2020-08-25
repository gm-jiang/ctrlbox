#include "dbg_print.h"
#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include "bsp_port.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "system_init.h" //just use sys_mutex_lock unlock function

static char buffer[PRINT_BUFFER_LEN];

void dbg_print(int print_level, char *fmt, ...)
{
	uint16_t len = 0;
	sys_mutex_lock(printMutex);
	if (print_level > PRINT_LEVEL) 
	{
		//sys_mutex_unlock(printMutex);
		return;
	}
	else
	{
		va_list argp;
		va_start(argp, fmt);
		vsprintf(buffer, fmt, argp);
		len = strlen(buffer);
		bsp_uart1_send((uint8_t *)&buffer, len);
		va_end(argp);
	}
	sys_mutex_unlock(printMutex);
}

void dbg_print_msg(int print_level, uint8_t *preMsg, uint8_t len, uint8_t *msg)
{
	uint8_t i;

	dbg_print(print_level, "%s %d Byte ", preMsg, len);
	for (i = 0; i < len; i++)
	{
			dbg_print(print_level, "%02X ", msg[i]);
	}
	dbg_print(print_level, "\r\n");
}
