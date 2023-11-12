#pragma once

#include "main.h"

typedef struct _CanRTx {
    uint16_t rxid;
    uint16_t txid;
} CanRTx;

uint8_t CanTransimit(uint8_t *data);

void CanInit(CanRTx *CanRTXPtr);
