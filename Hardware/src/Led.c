#include "Led.h"

// void LedControlDelay(char LedLable, GPIO_PinState State, uint32_t DelayTime)
// {
//     HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

//     switch (LedLable) {
//         case 'r': {
//             HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, State);
//             break;
//         }
//         case 'b': {
//             HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, State);
//             break;
//         }
//         case 'g': {
//             HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, State);
//             break;
//         }
//     }
//     HAL_Delay(DelayTime);
// }

void LedControl(char LedLable, GPIO_PinState State)
{
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

    switch (LedLable) {
        case 'r': {
            HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, State);
            break;
        }
        case 'b': {
            HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, State);
            break;
        }
        case 'g': {
            HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, State);
            break;
        }
    }
}

void LedToggle(char LedLable)
{
    switch (LedLable) {
        case 'r': {
            HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
            HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

            break;
        }
        case 'b': {
            HAL_GPIO_TogglePin(BLUE_GPIO_Port, BLUE_Pin);
            HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
            break;
        }
        case 'g': {
            HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);
            HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
            break;
        }
    }
}


// void LedToggleDelay(char LedLable, uint32_t DelayTime)
// {
//     static uint32_t LastTime=HAL_GetTick();
//     static uint32_t TargetTime=0;
//     TargetTime=DelayTime;
//     if(HAL_GetTick()-LastTime>TargetTime)
//     {
//     switch (LedLable) {
//         case 'r': {
//             HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
//             HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

//             break;
//         }
//         case 'b': {
//             HAL_GPIO_TogglePin(BLUE_GPIO_Port, BLUE_Pin);
//             HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
//             break;
//         }
//         case 'g': {
//             HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);
//             HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
//             HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
//             break;
//         }
//     }
//     LastTime=HAL_GetTick();
//     }
// }