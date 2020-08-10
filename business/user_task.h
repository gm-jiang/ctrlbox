#ifndef _USER_TASK_H_
#define _USER_TASK_H_

#ifdef __cplusplus
	extern "C" {
#endif

void task_led_status(void *pvParameters);

void task_rf315(void *pvParameters);
void task_rf330(void *pvParameters);
void task_rf433(void *pvParameters);
void task_rf4xx(void *pvParameters);

#ifdef __cplusplus
	}
#endif


#endif
