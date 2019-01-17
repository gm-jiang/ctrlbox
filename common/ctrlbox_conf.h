#ifndef _CTRLBOX_CONF_H
#define _CTRLBOX_CONF_H

#include <stdint.h>

/***********************************VERSION DEFINE***************************************/
#define HW_VERSION_STR		HW_V
#define SW_VERSION_STR		SW_V

#define VERISON_INFO_LEN	10


/***********************************FUNCTION RETURN DEFINE***************************************/
//function return value
#define RTN_SUCCESS																					0x00
#define RTN_FAIL																						0x01

/***********************************485 ADDRESS FLASH DEFINE***************************************/

#define CHAIN_DOWN_CTRLBOX            0x01
#define UHF_RFID_CTRLBOX              0x02
#define RFID_CHECK_CTRLBOX            0x03
#define RFID_DEBUG_CTRLBOX            0x04


//flash define
#define FLASH_SIZE                     256          //MCU flash(k)
#if FLASH_SIZE < 256
  #define SECTOR_SIZE           			 1024    //Byte
#else 
  #define SECTOR_SIZE           			 2048    //Byte
#endif
#define MCU_485_ADDR_BASE							 (0x08000000 + 127 * SECTOR_SIZE)
#define MCU_485_DEV_SN_BASE						 (0x08000000 + 126 * SECTOR_SIZE)
#define MCU_485_DEV_CUSTOMER_BASE			 (0x08000000 + 125 * SECTOR_SIZE)

#define LEVEL_CTRL_HIGH								1
#define LEVEL_CTRL_LOW								2

#define CHAIN_DOWN_LAMP_DELAY					500 //ms
#define UHF_RFID_LAMP_DELAY					  1000 //ms
#define RFID_CHECK_LAMP_DELAY					2000 //ms

#define VALVE_CTRL_TIME_MIN						100 //ms
#define VALVE_CTRL_TIME								2000 //ms
#define VALVE_CTRL_TIME_MAX						20000 //ms

typedef struct _configInfoType_t {
	uint8_t function;
	uint8_t lampCtrlLevel;
	uint8_t motorCtrlLevel;
	uint8_t valveCtrlLevel;
	uint16_t valveCtrlTime; //ms
}configInfoType_t;

#define CONFIG_INFO_TYPE_LEN					(sizeof(configInfoType_t))

extern uint8_t g_mcu485Addr;
//extern uint8_t g_mcuFuncConfig;
extern configInfoType_t g_mcuConfigInfo;

uint8_t get_485_addr(void);
void set_485_addr(uint8_t addr);
uint8_t get_dev_sn(uint8_t *snBuf, uint8_t *snLen);
uint8_t set_dev_sn(uint8_t *snBuf, uint8_t snLen);
//uint8_t get_customer_config(void);
//void set_customer_config(uint8_t config);
uint8_t get_config_info(configInfoType_t *configInfo);
uint8_t set_config_info(configInfoType_t *configInfo);
void ctrlbox_configinfo_init(void);


#endif
