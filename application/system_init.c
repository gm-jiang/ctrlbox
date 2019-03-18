#include "stm32f10x.h"
#include "system_init.h"
#include "bsp_port.h"
#include "mt_common.h"
#include "dbg_print.h"
#include "user_task.h"
#include "FreeRTOS.h"
#include <string.h>

#include "upgrade.h" //use the macro define ODD EVEN

/************Notice !!!**************
***All global variable define here***
*************************************/

/*the low rfid tag id node list*/
tagNode_t g_node_list;

/*the ctrlbox config info of addr,sn and mode*/
configInfo_t g_configInfo;

/*the receive message queue of uart*/
QueueHandle_t UART1RecvMsgQueue = NULL;
QueueHandle_t UART2RecvMsgQueue = NULL;
QueueHandle_t UART3RecvMsgQueue = NULL;
QueueHandle_t UpLoadEventMsgQueue = NULL;
QueueHandle_t LowRfidMsgQueue = NULL;

SemaphoreHandle_t printMutex = NULL;
SemaphoreHandle_t rfidDataMutex = NULL;
SemaphoreHandle_t chainDownDetectSemaphore = NULL;

static void sys_init_completed(void);
static void sys_config_info_init(void);
static void sys_msg_queue_init(void);
static void sys_mutex_init(void);
static void sys_semaphore_init(void);

static void sys_init_completed(void)
{
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n************************Start Run Application*************************");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n======================================================================");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=              (C) COPYRIGHT 2019 Meituan Xiaoxiang                  =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=                                                                    =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=     SuspensionChain-Controlbox Application  (Version 1.0.1)        =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=                                                                    =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=     System Initialize completed...SW: %s HW: %s Addr: %02d       =", SW_V, HW_V, g_configInfo.addr);
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=                                                                    =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n=                                      By MT Application Team        =");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n======================================================================");
	dbg_print(PRINT_LEVEL_DEBUG, "\r\n\r\n");
}

static void sys_config_info_init(void)
{
	uint8_t i = 0;
	uint8_t temp_buf[20] = {0};

	memset(temp_buf, 0x00, sizeof(temp_buf));
	g_configInfo.addr = get_485_addr();
	g_configInfo.motor_auth = get_motor_ctrl_auth();
	if (g_configInfo.motor_auth == 0xFF)
		g_configInfo.motor_auth = MOTOR_CTRL_AUTH_OFF;
	g_configInfo.mode = bsp_switch_4bit_value();
	if (g_configInfo.mode == CTRLBOX_UHF_RFID)
		bsp_chaindown_finish_sensor_exit_enable();//enable interrupt of chaindown finsh sensor

	get_dev_sn(temp_buf);
	memcpy(g_configInfo.sn, temp_buf, DEVICE_SN_LEN);

	get_dev_param(temp_buf);
	g_configInfo.ctrlLogic.lamp_ctrl_level = temp_buf[i++];
	g_configInfo.ctrlLogic.motor_ctrl_level = temp_buf[i++];
	g_configInfo.ctrlLogic.valve_ctrl_level = temp_buf[i++];
	g_configInfo.ctrlLogic.light_sensor_trig = temp_buf[i++];
	g_configInfo.ctrlLogic.valve_ctrl_delay_ms = temp_buf[i] << 8 | temp_buf[i+1];
}

static void sys_msg_queue_init(void)
{
	UART1RecvMsgQueue = xQueueCreate(UART1_QUEUE_NUM, UART1_QUEUE_LEN);
	UART2RecvMsgQueue = xQueueCreate(UART2_QUEUE_NUM, UART2_QUEUE_LEN);
	UART3RecvMsgQueue = xQueueCreate(UART3_QUEUE_NUM, UART3_QUEUE_LEN);
	UpLoadEventMsgQueue = xQueueCreate(EVENT_QUEUE_NUM, EVENT_QUEUE_LEN);
	LowRfidMsgQueue = xQueueCreate(LOWRFID_QUEUE_NUM, LOWRFID_QUEUE_LEN);
}

static void sys_mutex_init(void)
{
	printMutex = xSemaphoreCreateMutex();
	rfidDataMutex = xSemaphoreCreateMutex();
}

static void sys_semaphore_init(void)
{
	chainDownDetectSemaphore = xSemaphoreCreateBinary();
}

void sys_mutex_lock(SemaphoreHandle_t xMutex)
{
	xSemaphoreTake(xMutex, portMAX_DELAY);
}

void sys_mutex_unlock(SemaphoreHandle_t xMutex)
{
	xSemaphoreGive(xMutex);
}

void platform_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
#ifdef ODD_CODE
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, ODD_OFFSET);
#else
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, EVEN_OFFSET);
#endif
	//disable interrupts
	portDISABLE_INTERRUPTS();
	bsp_all_gpio_configuration();
	bsp_lamp_led_init();
	bsp_status_led_init();
	bsp_releasekey_init();
	bsp_emerstopkey_init();
	bsp_chaindown_finish_sensor_init();
	bsp_motor_ctrl_init();
	bsp_mcu_485RE_init();
	bsp_chaindown_ctrl_init();
	bsp_wcs_uart1_init();
	bsp_stc_uart2_init();
	bsp_uhf_uart3_init();
	bsp_dbg_uart4_init();
	bsp_IWDG_init(IWDG_Prescaler_64, 3125); //5s
	bsp_ctrlbox_mode_pin_init();
	sys_config_info_init();
	sys_msg_queue_init();
	sys_mutex_init();
	sys_semaphore_init();
	node_list_init(&g_node_list);
	sys_init_completed();
	//enable interrupts
	portENABLE_INTERRUPTS();
}

void os_task_init(void)
{
	BaseType_t ret;

	//controlbox run status task
	ret = xTaskCreate(task_led_status, "led_status", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_LED_STATUS_TASK, NULL);
	if (ret != pdPASS)
		dbg_print(PRINT_LEVEL_ERROR, "create task_led_status failed\r\n");

	//receive wcs msg task
	ret = xTaskCreate(task_uart1_receive, "uart1_receive", configMINIMAL_STACK_SIZE*15, NULL, PRIORITIES_UART1_RCV_TASK, NULL);
	if (ret != pdPASS)
		dbg_print(PRINT_LEVEL_ERROR, "create task_uart1_receive failed\r\n");

	//receive stc msg task
	ret = xTaskCreate(task_uart2_receive, "uart2_receive", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_UART2_RCV_TASK, NULL);
	if (ret != pdPASS)
		dbg_print(PRINT_LEVEL_ERROR, "create task_uart2_receive failed\r\n");

	//receive uhf msg task
	ret = xTaskCreate(task_uart3_receive, "uart3_receive", configMINIMAL_STACK_SIZE, NULL, PRIORITIES_UART3_RCV_TASK, NULL);
	if (ret != pdPASS)
		dbg_print(PRINT_LEVEL_ERROR, "create task_uart3_receive failed\r\n");
}
