#pragma once

#include "main.h"

uint8_t FlashWrite(uint32_t *data, uint32_t length);

uint32_t FLASH_EEPROM_Read(uint32_t *buf, uint32_t len);