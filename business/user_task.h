#ifndef _USER_TASK_H_
#define _USER_TASK_H_

#ifdef __cplusplus
	extern "C" {
#endif
		
#define INDEX_LEN           2
#define INDEX_DAT           3

#define ADDR_LEN            1
#define TYPE_LEN            1
#define CRC_LEN             1
#define DAT_INDEX           2

void task_led_status(void *pvParameters);
void task_uart1_receive(void *pvParameters);
void task_uart2_receive(void *pvParameters);
void task_uart3_receive(void *pvParameters);

#ifdef __cplusplus
	}
#endif


#endif
