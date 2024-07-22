#ifndef FLASHTASK_H
#define FLASHTASK_H

#include "bsp_flash.h"

#define arm_encoder_data_read_from_flash 1
#define arm_auto_mode_record_data_read_from_flash 1
#define UI_param_data_read_from_flash 1

#define arm_auto_mode_record_address    ADDR_FLASH_SECTOR_7+0x00000000
#define arm_encoder_record_address      ADDR_FLASH_SECTOR_7+0x00010000
#define UI_param_record_address         ADDR_FLASH_SECTOR_7+0x00010020

typedef struct
{
    uint32_t address;
    uint32_t *data;
    uint32_t len;
}Flash_write_param_t;

void flashRefresh();


#endif //FLASHTASK_H