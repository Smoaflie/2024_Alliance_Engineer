#include "FlashTask.h"
#include "FlashInstance.h"

static FlashData FlashData_ = {0};

FlashData *FlashDataRead()
{
    FLASH_EEPROM_Read((uint32_t *)&FlashData_, sizeof(FlashData));
    // FlashData_.frequence       = 250;
    // FlashData_.CanRTxData.txid = 0x01;
    // FlashData_.CanRTxData.rxid = 0x01;
    return &FlashData_;
}

uint8_t FlashDataWrite(FlashData *__FlashData)
{
    if (FlashWrite((uint32_t *)__FlashData, sizeof(FlashData))) {
        return 1;
    }
    return 0;
}
