#include "dbg_print.h"
#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include "bsp_port.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static char buffer[PRINT_BUFFER_LEN];

void dbg_Print(int print_level, char *fmt, ...)
{
	uint16_t len = 0;
	if (print_level >= PRINT_LEVEL) {
			return;
	} else {
			va_list argp;
			va_start(argp, fmt);
			vsprintf(buffer,fmt, argp);
			len = strlen(buffer);
			buffer[len] = '\r';
			wcs485Uart_SendData((uint8_t *)&buffer, len + 1);
			va_end(argp);
	}
}
