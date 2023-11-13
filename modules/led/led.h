/**
 * @Author       : HDC h2019dc@outlook.com
 * @Date         : 2023-09-08
 * @LastEditors  : HDC h2019dc@outlook.com
 * @LastEditTime : 2023-10-28
 * @FilePath     : \2024_Control_New_Framework_Base-dev-all\modules\led\led.h
 * @Description  : LED模块
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#ifndef _LED_H_
#define _LED_H_

#include "stdint.h"
#include "bsp_pwm.h"

#define LED_MAX_NUM 3

typedef struct
{
    PWMInstance *led_pwm;
    float led_brightness; // 亮度,通过PWM改变
    uint8_t led_switch;   // 开关,on1 off0
    // void (*action_callback)(void); // led动作回调函数
} LEDInstance;
// C板LED统一定义
typedef struct
{
    LEDInstance *led_R;
    LEDInstance *led_G;
    LEDInstance *led_B;
} C_board_LEDInstance;

typedef struct
{
    PWM_Init_Config_s pwm_config;
    uint8_t init_swtich; // 初始化开关
} LED_Init_Config_s;

LEDInstance *LEDRegister(LED_Init_Config_s *led_config);

void C_boardLEDRegister(void);

void C_board_LEDSet(uint32_t color);

void LEDSet(LEDInstance *_led, float brightness);

void LEDSwitch(LEDInstance *_led, uint8_t led_switch);

#endif // _LED_H_
