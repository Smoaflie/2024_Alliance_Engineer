#include <memory.h>

#include "CanSendTask.h"

#include "CanInstance.h"
#include "FlashTask.h"
#include "AngleEncoder.h"

FlashData *FlashDataRW = {0};

const uint32_t APB1TIMER = 72000000;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void CanTimeInitInstance()
{

    FlashDataRW = FlashDataRead();
    // FlashDataWrite(FlashDataRW);
    CanRTx CanRTXData;
    CanRTXData.rxid = FlashDataRW->CanRTxData.rxid;
    CanRTXData.txid = FlashDataRW->CanRTxData.txid;

    CanInit(&CanRTXData);

    uint32_t frequence = FlashDataRW->frequence;
    uint16_t AutoCount = 0;
    uint16_t Prescaler = 72 - 1;

    if (frequence > 1000) {
        frequence = 1000;
    }

    AutoCount = (uint16_t)((double)1.0 / (double)frequence * ((double)APB1TIMER / (double)(Prescaler - 1)));

    __HAL_TIM_SET_PRESCALER(&htim2, Prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim2, AutoCount - 1);
    __HAL_TIM_SET_COUNTER(&htim2, 0); // 初始时钟中断周期（任务发送频率）

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
}

AngleEncoderData AngleEncoderData_ = {0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2) // can发送
    {
        uint8_t CanSendData[8] = {0};
        memcpy(CanSendData, &AngleEncoderData_.Angle, sizeof(AngleEncoderData_.Angle));
        memcpy(CanSendData + sizeof(AngleEncoderData_.Angle), &AngleEncoderData_.Error, sizeof(AngleEncoderData_.Error));

        CanTransimit(CanSendData);
    }

    if (htim == &htim3) // spi采集
    {
        // static AngleEncoderData AngleEncoderDataLast_ = {0};
        AngleEncoderData_ = RecieveData();
        // AngleEncoderDataLast_                         = AngleEncoderData_;
    }
}
