#ifndef _CTRLBOX_CONF_H
#define _CTRLBOX_CONF_H

#include <stdint.h>

/***********************************VERSION DEFINE***************************************/
#define HW_VERSION_STR		"---HW V1.1"
#define SW_VERSION_STR		"---SW V1.0"
#define VERISON_INFO_LEN	10


/***********************************FUNCTION RETURN DEFINE***************************************/
//function return value
#define RTN_SUCCESS																					0x00
#define RTN_FAIL																						0x01

/***********************************485 ADDRESS FLASH DEFINE***************************************/

#define CHAIN_DOWN_CTRLBOX            0x01
#define UHF_RFID_CTRLBOX              0x02
#define RFID_CHECK_CTRLBOX            0x03


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

extern uint8_t g_mcu485Addr;
extern uint8_t g_mcuFuncConfig;

uint8_t get_485_addr(void);
void set_485_addr(uint8_t addr);
uint8_t get_dev_sn(uint8_t *snBuf, uint8_t *snLen);
uint8_t set_dev_sn(uint8_t *snBuf, uint8_t snLen);
uint8_t get_customer_config(void);
void set_customer_config(uint8_t config);


#endif
