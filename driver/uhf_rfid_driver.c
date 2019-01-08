#include <string.h>
#include "uhf_rfid_driver.h"
#include "bsp_port.h"
#include "mt_common.h"


void uhfrfid_send_cmd(void)
{
	uint8_t cmd[2] = {'K', 'I'};
	bsp_uart3_send(&cmd[0], 1);
	mt_sleep_ms(500);
	bsp_uart3_send(&cmd[1], 1);
}
