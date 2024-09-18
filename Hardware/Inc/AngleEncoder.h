#pragma once

#include "main.h"

typedef struct
{
    uint32_t Angle;
    double AngleDouble;
    float speed_aps;
    uint32_t TotalAngle;
    uint8_t Error;

} AngleEncoderData;

AngleEncoderData RecieveData(void);

const uint8_t IsAngleEncoderOnline(void);