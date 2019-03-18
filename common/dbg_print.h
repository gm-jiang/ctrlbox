#ifndef _log_print_h
#define _log_print_h

#include <stdint.h>

#define PRINT_BUFFER_LEN          512

enum
{
    PRINT_LEVEL_ERROR,
    PRINT_LEVEL_WARNING,
    PRINT_LEVEL_DEBUG,
    PRINT_LEVEL_INFO,
    PRINT_LEVEL_INFO_LOWLEVEL,
    PRINT_LEVEL_VERBOSE
};

#define PRINT_LEVEL PRINT_LEVEL_WARNING

void dbg_print(int printf_level, char *fmt, ...);
void dbg_print_msg(int print_level, uint8_t *preMsg, uint8_t len, uint8_t *msg);

#endif
