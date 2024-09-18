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

    CanRTx CanRTXData;
    CanRTXData.rxid = 0;
    // CanRTXData.rxid = FlashDataRW->CanRTxData.rxid;
    CanRTXData.txid = 0x1fb;

    FlashDataRW->frequence = 1000;

    FlashDataWrite(FlashDataRW);

    CanInit(&CanRTXData);

    uint32_t frequence = FlashDataRW->frequence;
    uint16_t AutoCount = 0;
    uint16_t Prescaler = 72 - 1;

    if (frequence > 1000) {
        frequence = 1000;
    }

    AutoCount = (uint16_t)((double)1.0 / (double)frequence * ((double)APB1TIMER / (double)(Prescaler - 1)));

    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim3);

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

        
        if(!CanTransimit(CanSendData)){
            static uint8_t error_cnt = 0;
            error_cnt++;
            if(error_cnt > 1000){
                __set_FAULTMASK(1);
                NVIC_SystemReset();
            }
        }
    }

    if (htim == &htim3) // spi采集
    {
        AngleEncoderData_ = RecieveData();
        if(AngleEncoderData_.Error){
            static uint8_t error_cnt = 0;
            error_cnt++;
            if(error_cnt > 1000){
                __set_FAULTMASK(1);
                NVIC_SystemReset();
            }
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_ErrorCallback could be implemented in the user file
   */
  static uint8_t error_cnt = 0;
    error_cnt++;
    if(error_cnt > 1000){
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
}