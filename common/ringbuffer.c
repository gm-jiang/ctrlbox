#include <string.h>
#include <stdint.h>
#include "dbg_print.h"
#include "ringbuffer.h"

void ringbuffer_init(RingBuffer_t *rb)
{
	memset(rb, 0x00, sizeof(RingBuffer_t));
}

uint16_t get_readable_data_len(RingBuffer_t *rb)
{
	uint16_t len = 0;

	if (rb->write_point >= rb->read_point)
	{
		len = rb->write_point - rb->read_point;
	}
	else
	{
		len = BUF_LEN - (rb->read_point - rb->write_point);
	}
	return len;
}

uint16_t ringbuffer_put_data(RingBuffer_t *rb, uint8_t *data, uint16_t data_len)
{
	/* if data_len equal BUF_LEN than the read_point equal write_point, but we don't want this happen */
	if (data_len >= BUF_LEN)
	{
		dbg_print(PRINT_LEVEL_ERROR, "data len over load\r\n");
		return 0;
	}
	if (BUF_LEN - rb->write_point >= data_len)
	{
		memcpy(&rb->buffer[rb->write_point], data, data_len);
	}
	if (BUF_LEN - rb->write_point < data_len)
	{
		memcpy(&rb->buffer[rb->write_point], data, BUF_LEN - rb->write_point);
		memcpy(&rb->buffer[0], data + BUF_LEN - rb->write_point, data_len + rb->write_point - BUF_LEN);
	}
	rb->write_point += data_len;
	rb->write_point %= BUF_LEN;
	return data_len;
}

uint16_t ringbuffer_get_data(RingBuffer_t *rb, uint8_t *data, uint16_t data_len)
{
	uint16_t len = 0;

	len = get_readable_data_len(rb);
	if (len == 0)
	{
		dbg_print(PRINT_LEVEL_ERROR, "ring buffer no data\r\n");
		return 0;
	}
	len = len > data_len ?  data_len : len;

	if (rb->write_point > rb->read_point)
	{
		memcpy(data, &rb->buffer[rb->read_point], len);
	}
	else
	{
		if (rb->read_point + len < BUF_LEN)
		{
			memcpy(data, &rb->buffer[rb->read_point], len);
		}
		else
		{
			memcpy(data, &rb->buffer[rb->read_point], BUF_LEN - rb->read_point);
			memcpy(data + (BUF_LEN - rb->read_point), &rb->buffer[0], data_len - (BUF_LEN - rb->read_point));
		}
	}
	rb->read_point += len;
	rb->read_point %= BUF_LEN;
	return len;
}
