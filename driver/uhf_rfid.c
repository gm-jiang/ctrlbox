#include <string.h>
#include "uhf_rfid.h"
#include "bsp_port.h"
#include "mt_common.h"

bindMsgFrame_t bindMsgFrame;

void bind_msg_init(void)
{
	memset(&bindMsgFrame, 0x00, sizeof(bindMsgFrame_t));
}

void save_lowrfid_msg(uint8_t *msg, uint8_t msg_len)
{
	bindMsgFrame.low_flag = 1;
	memcpy(bindMsgFrame.low_rfid, msg, msg_len);
}

void save_uhfrfid_msg(uint8_t *msg, uint8_t msg_len)
{
	bindMsgFrame.uhf_flag = 1;
	memcpy(bindMsgFrame.uhf_rfid, msg, msg_len);
}

void set_event_msg(eventMsgFrame_t *eventMsg, eventMsgType_e msgType)
{
	eventMsg->msgType = msgType;
	memcpy(eventMsg->msg, bindMsgFrame.uhf_rfid, 12);
	memcpy(&(eventMsg->msg[UHF_RFID_LABLE_DATA_LEN]), bindMsgFrame.low_rfid, 4);
}

void uhfrfid_send_cmd(void)
{
	uint8_t cmd[2] = {'K', 'I'};

	uhfRfidUart_SendData(&cmd[0], 1);
	mt_delay_ms(500);
	uhfRfidUart_SendData(&cmd[1], 1);
}
