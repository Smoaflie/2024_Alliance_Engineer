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
#include "bsp_spi.h"

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
            // .htim      = &htim5,
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
/**
 * @brief :  达妙H7板LED颜色设置
 * @param color 颜色 0xRRGGBB
 * @return
 */
void DM_board_LEDSet(uint32_t color)
{
    // 达妙板子的LED模组为WS2812B-3528
    // 它根据时序控制颜色，这里采用SPi模拟时序
    // 因此需根据数据手册及时钟树信息配置正确的SPI发送数据
    static uint32_t last_color;
    static uint8_t txbuf[24];
    static uint8_t WS2812_HighLevel = 0xf0;
    static uint8_t WS2812_LowLevel  = 0xC0;
    if(last_color == color) return;
    last_color=color;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((color>>(i+8))&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((color>>(i+16))&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((color>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    //todo: 后续可加入DMA，减小开支（基本没有意义的优化x
    HAL_SPI_Transmit(&hspi6, (uint8_t *)txbuf, 24, 1);

    /* 以下为详细解析： 
       为方便计算，将SPI时钟树调为24Mhz，SPi分频4，则其波特率为6Mbit/s
       查询WS2812B-3528数据手册，以下为关键参数：
       数据格式：8位表示一个颜色，共24位，高位在前，按GRB顺序
       时钟信号：
       T0H：0.22~0.38us T0L：0.58~1us
       T1H：0.58~1us    T1L：0.58~1us
       单个信号周期为0.8~2us，结合SPi6的时钟频率(0.17us/bit)
       可计算出用2个字节表示一个信号
       需注意SPI未进行传输时电平为高，因此需先拉低电平通知WS2812B（代码中用移位实现）
       其他可直接看代码理解*/
}