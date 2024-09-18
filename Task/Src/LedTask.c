#include "LedTask.h"
#include "Led.h"
#include "CanInstance.h"
#include "AngleEncoder.h"
#include "KeyTask.h"

const CanRTx *CanRTxInstance;


void LedFlash(char LedLabel)
{
    static uint32_t LastTime=0;
    static uint16_t Times=0;
    static char LastLed;
    if(LedLabel!=LastLed)
    {
        Times=0;
        LastTime=HAL_GetTick();
    }
    switch(LedLabel)
    {
        case 'g':
        {
            if(Times<1)
            {
                LedControl(LedLabel,GPIO_PIN_SET);
                if(HAL_GetTick()-LastTime>1000)
                {
                    Times++;
                    LastTime=HAL_GetTick();
                }
            }
             

            if((HAL_GetTick()-LastTime>300)&&(Times>=1))
            {
                LedToggle(LedLabel);
                Times++;

                if(Times >CanRTxInstance->rxid * 2)
                {
                    Times=0;
                }
                LastTime=HAL_GetTick();
            }
            break;
        }
        case  'r':
        {
            LedControl(LedLabel,GPIO_PIN_SET);
            break;
        }
        case 'b':
        {
             LedControl(LedLabel,GPIO_PIN_SET);
            break;
        }

    }
    LastLed=LedLabel;
}


void LedTask()
{
    CanRTxInstance         = CanRTXValueGet();
    uint8_t IsCanRxFLag    = IsCanRx();
    uint8_t IsAngleEncoder = IsAngleEncoderOnline();
    uint8_t IsSetId        = SetIdFlagGet();

    // LedControl('b', GPIO_PIN_SET);

    // if (IsSetId == 1) {
    //     HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
    //     HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
    // } else if ( IsAngleEncoder) {
    //     LedFlash('r');
    // } else {
    //     LedFlash('g');
    // }
}