#include "KeyTask.h"
#include "FlashTask.h"
#include "CanInstance.h"
#include "Led.h"

static volatile uint64_t LastKeyTime = 0;
static volatile uint32_t NowKeyTime  = 0;
static uint8_t SetIdFlag             = 100;
static uint16_t SetId                = 1;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin) {
        NowKeyTime = HAL_GetTick();
        if (((NowKeyTime - LastKeyTime > 1000) && (SetIdFlag == 2))||(SetIdFlag==100)) {
            SetIdFlag = 1;
            SetId     = 0;
            LedControl('b', GPIO_PIN_SET);
        }

        if ((NowKeyTime - LastKeyTime < 1000) && (SetIdFlag == 1)) {
            SetId++;
            LedToggle('b');
        }

        LastKeyTime = HAL_GetTick();
    }
}

void Key_Task()
{
    NowKeyTime = HAL_GetTick();
    if ((NowKeyTime - LastKeyTime > 1000) && (SetIdFlag == 1)) {
        SetIdFlag                   = 2;
        FlashData *FlashData_       = {0};
        FlashData_                  = FlashDataRead();
        FlashData_->CanRTxData.rxid = SetId;
        FlashData_->CanRTxData.txid = SetId;
        FlashDataWrite(FlashData_);

        HAL_Delay(1000);

        __set_FAULTMASK(1); // 关闭所有中端

        NVIC_SystemReset(); // 复位

        // CanRTx CanRTxData_ = {0};
        // CanRTxData_.rxid   = SetId;
        // CanRTxData_.txid   = SetId;

        // CanReset(&CanRTxData_);

        LedControl('b', GPIO_PIN_RESET);
    }
}

const uint8_t SetIdFlagGet()
{
    return SetIdFlag;
}
