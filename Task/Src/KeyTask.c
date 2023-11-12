#include "KeyTask.h"
#include "FlashTask.h"
#include "CanInstance.h"
#include "Led.h"

static volatile uint64_t LastKeyTime = 0;
static uint8_t SetIdFlag             = 0;
static uint16_t SetId                = 1;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin) {
        uint64_t NowKeyTime = HAL_GetTick();
        if ((NowKeyTime - LastKeyTime > 2000) && (SetIdFlag == 2)) {
            SetIdFlag = 1;
            SetId     = 1;
            LedControl('b', GPIO_PIN_SET);
        }

        if ((NowKeyTime - LastKeyTime < 500) && (SetIdFlag == 1)) {
            SetId++;
            LedToggle('b');
        }

        LastKeyTime = HAL_GetTick();
    }
}

void Key_Task()
{
    uint64_t NowKeyTime = HAL_GetTick();
    if ((NowKeyTime - LastKeyTime > 5000) && (SetIdFlag == 1)) {
        SetIdFlag                   = 2;
        FlashData *FlashData_       = {0};
        FlashData_                  = FlashDataRead();
        FlashData_->CanRTxData.rxid = SetId;
        FlashData_->CanRTxData.txid = SetId;
        FlashDataWrite(FlashData_);

        CanRTx CanRTxData_ = {0};
        CanRTxData_.rxid   = SetId;
        CanRTxData_.txid   = SetId;

        CanReset(&CanRTxData_);

        LedControl('b', GPIO_PIN_RESET);
    }
}

const uint8_t SetIdFlagGet()
{
    return SetIdFlag;
}
