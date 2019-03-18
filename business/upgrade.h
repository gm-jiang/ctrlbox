#ifndef _UPGRADE_H_
#define _UPGRADE_H_

#include <stdint.h>
#include <stdlib.h>

/* When Enable ODD_CODE,Set IROM1 offset to 0x8003000,and Set 0x800F000 When Disable */

#define ODD_CODE

#define ODD_OFFSET               (0x3000)
#define EVEN_OFFSET              (0xF000)

#define ODD                      (1)
#define EVEN                     (0)

#define FW_SIZE                  (48) //48K
#define FW_PAGE_NUM              (24) //(FW_SIZE/2)

#define OTA_PARM_LEN             (12)
#define OTA_MAX_PACKET_LEN       (100)
#define OTA_BUFFER_LEN           (1024)

#define ODD_BOOT_FLAG            (0x55555555)
#define EVEN_BOOT_FLAG           (0xAAAAAAAA)
#define ODD_VALID_FLAG           (0x66666666)
#define EVEN_VALID_FLAG          (0xBBBBBBBB)
#define PARM_FLAG                (0x77777777)

//*******************Code Allocation From FLASH**********************//
#define FLASH_PAGE_SIZE          (2048)
#define FLASH_BOOT_BASE          FLASH_BASE
#define FLASH_ODD_BASE           ((uint32_t)0x08003000)  //Odd code zone 48Kb
#define FLASH_EVEN_BASE          ((uint32_t)0x0800F000)  //Even code zone 48Kb
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

#define OTA_PARM_ADDR            FLASH_PAGE_60
#define OTA_BOOT_FLAG_ADDR       FLASH_PAGE_61

typedef enum ota_reply {
    OTA_SUCCEED=1,
    OTA_CHECK_FAIL,
    OTA_PARM_FAIL,
    OTA_CODE_FAIL
} ota_reply_t;

typedef struct{
    unsigned short sw;
    unsigned short hw;
    char odd_or_even;
    char upgrage_flag;
}__attribute__((packed)) ota_ver_msg_t;

typedef struct setota_msg {
    unsigned short seq;
    unsigned char size;
    unsigned char msg[OTA_MAX_PACKET_LEN];
}__attribute__((packed))ota_code_msg_t;

typedef struct{
    int curr_seq;
    int curr_size;
    int total_size;
    int ota_type;
    uint16_t sw_ver;
    uint8_t hw_ver[2];
    uint32_t crc;
    unsigned char packet_len;
}ota_info_t;

typedef uint8_t (*mtOtaVerHandlerCb_t)(ota_ver_msg_t *msg);
typedef uint8_t (*mtOtaDataHandlerCb_t)(ota_code_msg_t *msg);
typedef uint8_t (*mtOtaErrorHandlerCb_t)(ota_code_msg_t *msg);

typedef struct
{
	mtOtaVerHandlerCb_t pfnOtaVerHandler;
	mtOtaDataHandlerCb_t pfnOtaDataHandler;
	mtOtaErrorHandlerCb_t pfnOtaErrorHandler;
} mtOtaCb_t;

extern mtOtaCb_t mtOtaCbs;

#endif
