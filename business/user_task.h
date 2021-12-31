#ifndef _USER_TASK_H_
#define _USER_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

void task_key_detect(void *pvParameters);

void task_rf315(void *pvParameters);
void task_rf330(void *pvParameters);
void task_rf433(void *pvParameters);
void task_rf430(void *pvParameters);
void task_rf430_send(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
