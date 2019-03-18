#include <stdlib.h>
#include <string.h>
#include "upgrade.h"
#include "stm32f10x.h"
#include "dbg_print.h"
#include "mt_common.h"

#include "uart.h"

static ota_info_t s_otaInfo;
static int s_writenAddr = 0;

static uint8_t mtOtaVerHandlerCb(ota_ver_msg_t *msg);
static uint8_t mtOtaDataHandlerCb(ota_code_msg_t *msg);
static uint8_t ota_notify_proc(ota_code_msg_t *ota_packet);
static uint8_t ota_param_proc(ota_code_msg_t *ota_packet);
static uint8_t ota_firmware_proc(ota_code_msg_t *ota_packet);
static uint8_t ota_save_parm(uint8_t *msg);
static uint8_t ota_check_crc(void);
static uint8_t ota_set_boot_flag(void);

mtOtaCb_t mtOtaCbs = {mtOtaVerHandlerCb, mtOtaDataHandlerCb, NULL};

uint8_t mtOtaVerHandlerCb(ota_ver_msg_t *msg)
{
	ota_ver_msg_t ota_ver;

	ota_ver.sw = mt_get_sw_ver();
	ota_ver.hw = mt_get_hw_ver();
	if (msg->sw != ota_ver.sw && msg->hw == ota_ver.hw)
	{
		ota_ver.upgrage_flag = 1;
		dbg_print(PRINT_LEVEL_DEBUG, "Need upgrade\r\n");
	}
	else
	{
		ota_ver.upgrage_flag = 0;
		dbg_print(PRINT_LEVEL_DEBUG, "Needn't upgrade\r\n");
	}
#ifdef ODD_CODE
	ota_ver.odd_or_even = EVEN;
	dbg_print(PRINT_LEVEL_DEBUG, "Download EVEN(B) firmware\r\n");
#else
	ota_ver.odd_or_even = ODD;
	dbg_print(PRINT_LEVEL_DEBUG, "Download ODD(A) firmware\r\n");
#endif
	ota_ver.sw = mt_htons(ota_ver.sw);
	ota_ver.hw = mt_htons(ota_ver.hw);
	return send_ack_to_center((uint8_t *)&ota_ver, sizeof(ota_ver_msg_t), 0xB0);
}

uint8_t mtOtaDataHandlerCb(ota_code_msg_t *ota_packet)
{
	uint8_t ret;

	FLASH_Unlock();
	if (ota_packet->seq == 0)
		ret = ota_notify_proc(ota_packet);
	else if (ota_packet->seq == 1)
		ret = ota_param_proc(ota_packet);
	else
		ret = ota_firmware_proc(ota_packet);
	FLASH_Lock();
	return send_ack_to_center((uint8_t *)&ret, 1, 0xB1);
}

static uint8_t ota_notify_proc(ota_code_msg_t *ota_packet)
{
	dbg_print(PRINT_LEVEL_DEBUG, "Notify packet...\r\n", ota_packet->seq);
	if ((ota_packet->size == 0) || (ota_packet->size > OTA_MAX_PACKET_LEN))
	{
		return OTA_PARM_FAIL;
	}
	return OTA_SUCCEED;
}

static uint8_t ota_param_proc(ota_code_msg_t *ota_packet)
{
	uint32_t EraseCounter = 0;
	FLASH_Status FLASHStatus = FLASH_COMPLETE;

	dbg_print(PRINT_LEVEL_DEBUG, "Param packet...\r\n", ota_packet->seq);
	if (ota_packet->size != OTA_PARM_LEN)
	{
		dbg_print(PRINT_LEVEL_ERROR, "---ERROR: packet size[%d] error!!!\r\n", ota_packet->size);
		return OTA_PARM_FAIL;
	}
	if (ota_save_parm(ota_packet->msg))
	{
		dbg_print(PRINT_LEVEL_ERROR, "---ERROR: param save failed!!!\r\n");
		return OTA_PARM_FAIL;
	}
#ifdef ODD_CODE
	s_writenAddr = FLASH_EVEN_BASE;
#else
	s_writenAddr = FLASH_ODD_BASE;
#endif
	s_otaInfo.curr_seq = 2;
	s_otaInfo.curr_size = 0;
	s_otaInfo.total_size = mt_htonl(*((uint32_t *)ota_packet->msg + 1));
	s_otaInfo.crc = mt_htonl(*((uint32_t *)ota_packet->msg + 2));
	dbg_print(PRINT_LEVEL_DEBUG, "---The firmware size = %d crc = %d\r\n", \
						                       s_otaInfo.total_size, s_otaInfo.crc);

  //eraser the firmwarw store zone
	/* Erase the FLASH pages */
	for (EraseCounter = 0; (EraseCounter < FW_PAGE_NUM) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(s_writenAddr + (FLASH_PAGE_SIZE*EraseCounter));
	}
	if (EraseCounter != FW_PAGE_NUM || FLASHStatus != FLASH_COMPLETE)
	{
		dbg_print(PRINT_LEVEL_ERROR, "---ERROR: eraser the firmware zone failed!!!\r\n");
		return OTA_PARM_FAIL;
	}
	dbg_print(PRINT_LEVEL_DEBUG, "---Eraser the firmware zone success\r\n");
	return OTA_SUCCEED;
}

static uint8_t ota_firmware_proc(ota_code_msg_t *ota_packet)
{
	uint32_t i;

	if ((ota_packet->size <= 0) || (ota_packet->size > OTA_MAX_PACKET_LEN))
	{
		dbg_print(PRINT_LEVEL_ERROR, "---ERROR: %d size is error\r\n", ota_packet->size);
		return OTA_CODE_FAIL;
	}
	if (ota_packet->seq < s_otaInfo.curr_seq)
	{
		dbg_print(PRINT_LEVEL_ERROR, "---WARNNING: %d seq is already received\r\n", ota_packet->seq);
		return OTA_SUCCEED;
	}
	else if (ota_packet->seq > s_otaInfo.curr_seq)
	{
		//actually no use
		dbg_print(PRINT_LEVEL_ERROR, "---ERROR: %d seq is mising\r\n", s_otaInfo.curr_seq);
		return OTA_CODE_FAIL;
	}
	//the follow satisfy (ota_packet->seq == g_otaInfo.curr_seq) of course
	s_otaInfo.curr_size += ota_packet->size;
	s_otaInfo.curr_seq++;

	for (i = 0; i < ota_packet->size; i += 4, s_writenAddr += 4)
	{
		/* Program the data received into STM32F10x Flash */
		FLASH_ProgramWord(s_writenAddr, *(uint32_t*)&ota_packet->msg[i]);
		if (*(uint32_t*)s_writenAddr != *(uint32_t*)&ota_packet->msg[i])
		{
			dbg_print(PRINT_LEVEL_ERROR, "---ERROR: FLASH_ProgramWord failed\r\n");
			return OTA_CHECK_FAIL;
		}
	}
	if (s_otaInfo.curr_size == s_otaInfo.total_size)
	{
		if (ota_check_crc() == 0) 
		{
			dbg_print(PRINT_LEVEL_DEBUG, "---CRC is match.\r\n");
			ota_set_boot_flag();
			return OTA_SUCCEED;
		}
		else
		{
			dbg_print(PRINT_LEVEL_DEBUG, "---CRC is mismatch.\r\n");
			return OTA_CHECK_FAIL;
		}
	}
	if (s_otaInfo.curr_size > s_otaInfo.total_size) 
	{
		dbg_print(PRINT_LEVEL_DEBUG, "---ERROR: curr_size(%d) > total_size(%d).\r\n", \
                                       s_otaInfo.curr_size, s_otaInfo.total_size);
		return OTA_CHECK_FAIL;
	}
	return OTA_SUCCEED;
}

static uint8_t ota_save_parm(uint8_t *msg)
{
	uint8_t i;
	FLASH_Status FLASHStatus = FLASH_COMPLETE;
	int tempAddress1 = OTA_PARM_ADDR;

	FLASHStatus = FLASH_ErasePage(tempAddress1);
	if (FLASHStatus != FLASH_COMPLETE)
	{
		dbg_print(PRINT_LEVEL_DEBUG, "---ERROR: FLASH_ErasePage error!!!\r\n");
	}
	for (i = 0; i < OTA_PARM_LEN/sizeof(int); i++)
	{
		FLASH_ProgramWord(tempAddress1, *((uint32_t*)msg + i));
		if (*(uint32_t*)tempAddress1 != *((uint32_t*)msg + i))
		{
			dbg_print(PRINT_LEVEL_DEBUG, "---ERROR: FLASH_ProgramWord failed\r\n");
			return 1;
		}
		tempAddress1 += 4;
	}
	return 0;
}

static uint8_t ota_check_crc(void)
{
	unsigned int crc = 0;
#ifdef ODD_CODE
	int firmwareAddr = FLASH_EVEN_BASE;
#else
	int firmwareAddr = FLASH_ODD_BASE;
#endif

	dbg_print(PRINT_LEVEL_DEBUG, "ota_check_crc: img size = %d\r\n", s_otaInfo.total_size);
	crc = mt_crc32((unsigned char *)firmwareAddr, s_otaInfo.total_size);
	if (s_otaInfo.crc != crc)
	{
		dbg_print(PRINT_LEVEL_DEBUG, "---ERROR: the checksum is mismatch\r\n");
		return 1;
	}
	dbg_print(PRINT_LEVEL_DEBUG, "---SUCESS: the checksum is match\r\n");
	return 0;
}

static uint8_t ota_set_boot_flag(void)
{
	uint32_t boot_flag;
	FLASH_Status FLASHStatus = FLASH_COMPLETE;
	int tempAddress1 = OTA_BOOT_FLAG_ADDR;

	FLASHStatus = FLASH_ErasePage(tempAddress1);
	if (FLASHStatus != FLASH_COMPLETE)
	{
		dbg_print(PRINT_LEVEL_DEBUG, "---ERROR: FLASH_ErasePage error!!!\r\n");
		return 1;
	}
#ifdef ODD_CODE
	boot_flag = EVEN_BOOT_FLAG;
#else
	boot_flag = ODD_BOOT_FLAG;
#endif
	FLASHStatus = FLASH_ProgramWord(tempAddress1, boot_flag);
	if (FLASHStatus != FLASH_COMPLETE)
	{
		dbg_print(PRINT_LEVEL_DEBUG, "---ERROR: FLASH_ProgramWord error!!!\r\n");
		return 1;
	}
	return 0;
}

