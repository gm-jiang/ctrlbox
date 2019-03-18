#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <stdint.h>

//*******************Code Allocation From FLASH**********************//
#define FLASH_PAGE_SIZE          (2048)
#define FLASH_STORE_BASE         ((uint32_t)0x0801B000)  //store misc param zone

//********************Page Allocation From FLASH*********************//
#define FLASH_PAGE_54            FLASH_STORE_BASE
#define FLASH_PAGE_55            FLASH_STORE_BASE + 1*FLASH_PAGE_SIZE;
#define FLASH_PAGE_56            FLASH_STORE_BASE + 2*FLASH_PAGE_SIZE;
#define FLASH_PAGE_57            FLASH_STORE_BASE + 3*FLASH_PAGE_SIZE;
#define FLASH_PAGE_58            FLASH_STORE_BASE + 4*FLASH_PAGE_SIZE;
#define FLASH_PAGE_59            FLASH_STORE_BASE + 5*FLASH_PAGE_SIZE;
#define FLASH_PAGE_60            FLASH_STORE_BASE + 6*FLASH_PAGE_SIZE;
#define FLASH_PAGE_61            FLASH_STORE_BASE + 7*FLASH_PAGE_SIZE;

#define ADDR485_PARAM_ADDR            (0x08000000 + 127 * SECTOR_SIZE)
#define SN_PARAM_ADDR                 (0x08000000 + 126 * SECTOR_SIZE)
#define DEV_PARAM_ADDR                (0x08000000 + 125 * SECTOR_SIZE)
#define MOTOR_CTRL_AUTH_PARAM_ADDR    (0x08000000 + 124 * SECTOR_SIZE)

#define DEVICE_ADDR_LEN          1
#define DEVICE_SN_LEN            16
#define DEVICE_PARM_LEN          6

typedef enum {
	CTRLBOX_CHAIN_DOWN,
	CTRLBOX_UHF_RFID,
	CTRLBOX_RFID_CHECK,
	CTRLBOX_RFID_DEBUG,
} ctrlbox_mode_e;

typedef struct _ctrlLogic_t {
	uint8_t lamp_ctrl_level;
	uint8_t motor_ctrl_level;
	uint8_t valve_ctrl_level;
	uint8_t light_sensor_trig;
	uint16_t valve_ctrl_delay_ms;
} ctrlLogic_t;

typedef struct _config_info_t {
	uint8_t mode;
	uint8_t addr;
	uint8_t motor_auth;
	uint8_t sn[DEVICE_SN_LEN];
	ctrlLogic_t ctrlLogic;
} configInfo_t;

void config_info_init(void);
uint8_t set_485_addr(uint8_t addr);
uint8_t get_485_addr(void);
uint8_t set_dev_sn(uint8_t *buf, uint8_t len);
uint8_t get_dev_sn(uint8_t *buf);
uint8_t set_dev_param(uint8_t *buf, uint8_t len);
uint8_t get_dev_param(uint8_t *buf);

uint8_t set_motor_ctrl_auth(uint8_t auth);
uint8_t get_motor_ctrl_auth(void);

#endif
