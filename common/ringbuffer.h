#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>


#define BUF_LEN  128

typedef struct
{
	unsigned char buffer[BUF_LEN];
	unsigned short read_point;
	unsigned short write_point;
}RingBuffer_t;

void ringbuffer_init(RingBuffer_t *rb);
uint16_t ringbuffer_put_data(RingBuffer_t *rb, uint8_t *data, uint16_t data_len);
uint16_t ringbuffer_get_data(RingBuffer_t *rb, uint8_t *data, uint16_t data_len);

#ifdef __cplusplus
	}
#endif

#endif
