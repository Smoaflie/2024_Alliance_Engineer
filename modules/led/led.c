/**
 * @Author       : HDC h2019dc@outlook.com
 * @Date         : 2023-09-08
 * @LastEditors  : HDC h2019dc@outlook.com
 * @LastEditTime : 2023-10-28
 * @FilePath     : \2024_Control_New_Framework_Base-dev-all\modules\led\led.c
 * @Description  : LED模块
 *                 针对C板板载LED提供了一些上层接口方便调用，C板LED为单实例，不返回指针
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_pwm.h"
#include "led.h"
#include "stdlib.h"
#include "memory.h"
#include "user_lib.h"

static uint8_t idx;
static LEDInstance *LED[LED_MAX_NUM] = {NULL};
static C_board_LEDInstance *c_led    = NULL;
/**
 * @brief : LED注册
 * @param *led_config
 * @return LED实例
 */
LEDInstance *LEDRegister(LED_Init_Config_s *led_config)
{
    LEDInstance *led_ins = (LEDInstance *)zmalloc(sizeof(LEDInstance)); // 剩下的值暂时都被置零
    led_ins->led_pwm     = PWMRegister(&led_config->pwm_config);
    led_ins->led_switch  = led_config->init_swtich;
    LED[idx++]           = led_ins;
    return led_ins;
}

/**
 * @brief : C板LED注册
 * @return C板LED实例
 */
void C_boardLEDRegister(void)
{
    C_board_LEDInstance *c_led_ins = (C_board_LEDInstance *)zmalloc(sizeof(C_board_LEDInstance));
    LED_Init_Config_s led_config   = {
          .pwm_config = {
              .htim      = &htim5,
              .channel   = 0,
              .period    = 0.001f,
              .dutyratio = 0,
              .callback  = NULL,
              .id        = NULL,
        },
          .init_swtich = 1,
    };
    // 其他参数都一样,只有通道不同
    led_config.pwm_config.channel = TIM_CHANNEL_3;
    c_led_ins->led_R              = LEDRegister(&led_config);

    led_config.pwm_config.channel = TIM_CHANNEL_2;
    c_led_ins->led_G              = LEDRegister(&led_config);

    led_config.pwm_config.channel = TIM_CHANNEL_1;
    c_led_ins->led_B              = LEDRegister(&led_config);

    c_led = c_led_ins;
}
/**
 * @brief :  C板LED颜色设置
 * @param color 颜色 0xRRGGBB
 * @return
 */
void C_board_LEDSet(uint32_t color)
{
    if (c_led == NULL) C_boardLEDRegister();
    c_led->led_R->led_brightness = (float)((color >> 16) & 0xFF) / 255.0f;
    c_led->led_G->led_brightness = (float)((color >> 8) & 0xFF) / 255.0f;
    c_led->led_B->led_brightness = (float)((color >> 0) & 0xFF) / 255.0f;
    LEDSet(c_led->led_R, c_led->led_R->led_brightness);
    LEDSet(c_led->led_G, c_led->led_G->led_brightness);
    LEDSet(c_led->led_B, c_led->led_B->led_brightness);
}
/**
 * @brief : 设置LED亮度
 * @param *_led LED实例
 * @param brightness 亮度 0-1
 * @return
 */
void LEDSet(LEDInstance *_led, float brightness)
{
    _led->led_brightness = brightness;
    PWMSetDutyRatio(_led->led_pwm, _led->led_brightness);
}

/**
 * @brief : 设置LED开关
 * @param *_led LED实例
 * @param led_switch 1开0关
 * @return
 */
void LEDSwitch(LEDInstance *_led, uint8_t led_switch)
{
    if (led_switch == 1) {
        _led->led_switch = 1;
        PWMSetPeriod(_led->led_pwm, _led->led_brightness);
    } else {
        _led->led_switch = 0;
        PWMSetPeriod(_led->led_pwm, 0);
    }
}
