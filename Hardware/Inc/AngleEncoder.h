#pragma once

#include "main.h"

typedef struct
{
    uint32_t Angle;
    uint32_t TotalAngle;
    uint8_t Error;

} AngleEncoderData;

AngleEncoderData RecieveData(void);