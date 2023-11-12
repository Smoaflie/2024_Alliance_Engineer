#include "FlashInstance.h"
#include "memory.h"

static uint32_t FLASH_USER_START_ADDR = 0x0800FC00;

static uint32_t FLASH_USER_END_ADDR = 0x0800FC00;

// static uint32_t FLASH_USER_START_ADDR = 0x08007C00;

// static uint32_t FLASH_USER_END_ADDR = 0x08007C00;

uint8_t FlashWrite(uint32_t *data, uint32_t length)
{
    HAL_FLASH_Unlock(); // 解锁
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages     = 1;
    uint32_t PageError          = 0;

    uint32_t uw_address  = 0;
    uint32_t end_address = 0;
    uint32_t *data_buf   = (void *)0;
    float data_len       = 0;
    float turelength     = 0;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        uw_address  = FLASH_USER_END_ADDR;
        end_address = FLASH_USER_END_ADDR + length;
        data_buf    = data;
        data_len    = 0;
        turelength  = (float)length / 4.0f;
    }

    while (uw_address <= end_address) {

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, uw_address, *data_buf) == HAL_OK) {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len >= turelength) {
                break;
            }
        } else {
            HAL_FLASH_Lock();
            return 0;
        }
        /* code */
    }

    HAL_FLASH_Lock();
    return 1;
}

uint32_t FLASH_EEPROM_Read(uint32_t *buf, uint32_t len)
{
    memcpy(buf, (void *)FLASH_USER_END_ADDR, len * 4);
    return 1;
}
