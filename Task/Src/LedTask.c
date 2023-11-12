#include "LedTask.h"
#include "Led.h"
#include "CanInstance.h"
#include "AngleEncoder.h"
#include "KeyTask.h"
const CanRTx *CanRTxInstance;
void LedFlash(char LedLabel)
{
    LedControlDelay(LedLabel, GPIO_PIN_SET, 1000);
    for (size_t i = 0; i < CanRTxInstance->rxid; i++) {
        LedToggleDelay(LedLabel, 500);
    }
}

void LedTask()
{
    CanRTxInstance         = CanRTXValueGet();
    uint8_t IsCanRxFLag    = IsCanRx();
    uint8_t IsAngleEncoder = IsAngleEncoderOnline();
    uint8_t IsSetId        = SetIdFlagGet();

    if (IsSetId == 1) {
    } else if (IsCanRxFLag && IsAngleEncoder) {
        LedFlash('r');
    } else {
        LedFlash('g');
    }
}