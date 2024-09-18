#include "AngleEncoder.h"

AngleEncoderData AngleEncoder = {0};

const uint8_t MagWarning = 0b00000010;
const uint8_t OverSpeed  = 0b00001000;
extern SPI_HandleTypeDef hspi1;
/**
 * @brief 奇偶校验函数
 *
 * @param CheckedDate :待检验数据
 * @param length: 数据位数
 * @param flag：奇校验还是偶校验 0为偶，1为奇
 * @return uint8_t 是否校验成功
 */
uint8_t OddOvenCheck(uint64_t CheckedDate, uint8_t length, uint8_t flag);

/**
 * @brief 接收数据
 *
 * @return AngleEncoderData*
 */
AngleEncoderData RecieveData(void)
{
    GPIO_PinState PinStateAngleEncoder = HAL_GPIO_ReadPin(EncoderAngleError_GPIO_Port, EncoderAngleError_Pin);

    if (PinStateAngleEncoder == GPIO_PIN_RESET) {
        uint8_t TransmitData[4] = {0x83, 0, 0, 0};
        uint8_t RecieveData[4]  = {0};

        HAL_GPIO_WritePin(SPI1CSS_GPIO_Port, SPI1CSS_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef statuse = HAL_SPI_TransmitReceive(&hspi1, TransmitData, RecieveData, 4, 100);
        HAL_GPIO_WritePin(SPI1CSS_GPIO_Port, SPI1CSS_Pin, GPIO_PIN_SET);

        if (statuse) {
            goto error;
        }
        if (!((OddOvenCheck((RecieveData[1] << 8 | RecieveData[2]), 16, 0)) && (OddOvenCheck(RecieveData[3] >> 2, 6, 0)))) {
            goto error;
        } // 奇偶校验不通过

        uint8_t ErrorFlag = (RecieveData[2] & 0b10) | (RecieveData[3] & 0b100);

        if ((ErrorFlag & MagWarning) || (ErrorFlag & OverSpeed)) {
            goto error;
        } // 错误标志位校验不通过

        uint32_t RawData = (RecieveData[3] >> 4) | ((RecieveData[2] & 0b11111100) << 2) | ((RecieveData[1] << 10));

        AngleEncoder.AngleDouble = (double)(RawData) / 262144.0 * 360.0;

        AngleEncoder.Angle = RawData;

        AngleEncoder.Error = 0;
        return AngleEncoder;
    }
error:
    AngleEncoder.Error = 1;
    return AngleEncoder;
}

uint8_t OddOvenCheck(uint64_t CheckedDate, uint8_t length, uint8_t flag)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        switch ((CheckedDate >> i) & 1) {
            case 0: {

                break;
            }
            case 1: {
                sum++;
                break;
            }

            default:
                return 0;
                break;
        }
    }
    if (flag == 0) {
        if (sum % 2 == 0) {
            return 1;
        } else {
            return 0;
        }
    } else {
        if (sum % 2 == 0) {
            return 0;
        } else {
            return 1;
        }
    }
}

const uint8_t IsAngleEncoderOnline()
{
    return AngleEncoder.Error;
}