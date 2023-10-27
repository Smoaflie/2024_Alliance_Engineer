/**
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-27 22:09:54
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\modules\led\led.c
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_pwm.h"
#include "led.h"
#include "stdlib.h"
#include "memory.h"
#include "user_lib.h"

static uint8_t idx;
static LEDInstance *bsp_led_ins[LED_MAX_NUM] = {NULL};

LEDInstance *LEDRegister(LED_Init_Config_s *led_config)
{
    LEDInstance *led_ins = (LEDInstance *)zmalloc(sizeof(LEDInstance)); // 剩下的值暂时都被置零
    led_ins->led_pwm     = PWMRegister(&led_config->pwm_config);
    led_ins->led_switch  = led_config->init_swtich;
    bsp_led_ins[idx++]   = led_ins;
    return led_ins;
}

void LEDSet(LEDInstance *_led, float brightness)
{
    _led->led_brightness = brightness;
    PWMSetDutyRatio(_led->led_pwm, _led->led_brightness);
}

void LEDSwitch(LEDInstance *_led, uint8_t led_switch)
{
    if (led_switch == 1) {
        _led->led_switch = 1;
    } else {
        _led->led_switch = 0;
        PWMSetPeriod(_led->led_pwm, 0);
    }
}
