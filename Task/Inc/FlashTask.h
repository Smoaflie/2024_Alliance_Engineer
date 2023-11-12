#pragma once
#include "main.h"
#include "CanInstance.h"

#pragma pack(1) // 按1字节对齐

typedef struct __FlashData {
    CanRTx CanRTxData;
    uint32_t frequence;
} FlashData;

#pragma pack()

FlashData *FlashDataRead();

uint8_t FlashDataWrite(FlashData *__FlashData);
