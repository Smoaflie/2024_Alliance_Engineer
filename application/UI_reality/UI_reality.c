// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "message_center.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_gpio.h"

static Subscriber_t *UI_reality_data_sub;                   // 用于接收物理UI的数据信息
static UI_reality_Data_s UI_reality_recv;  // 物理UI接收到的信息

static GPIOInstance *UI_reality_GPIO;
static uint32_t color[10];
void UI_reality_Init()
{
    // GPIO_Init_Config_s UI_reality_GPIO_conf = {
    //     .GPIOx = UI_reality_GPIO_Port,
    //     .GPIO_Pin = UI_reality_Pin
    // };
    // UI_reality_GPIO = GPIORegister(&UI_reality_GPIO_conf);

    memset(color,0,sizeof(color));
    UI_reality_data_sub = SubRegister("UI_reality", sizeof(UI_reality_Data_s));
}
static void UI_color_change(uint8_t num,uint32_t* color){
    switch(num){
        case 1:*color =  0xff4546;break;// 红色
        case 2:*color =  0xffa308;break;// 橙色 
        case 3:*color =  0xffee46;break;// 黄色
        case 4:*color =  0xa8ff2d;break;// 绿色
        case 5:*color =  0x45fff2;break;// 青色
        case 6:*color =  0x0000ff;break;// 蓝色
        case 7:*color =  0xf029f6;break;// 紫色
        case 8:*color =  0xff648e;break;// 粉色
        case 9:*color =  0xffffff;break;// 白色
        default: *color = 0x000000;

    }
}
static void UI_color_select(){
    UI_color_change(UI_reality_recv.arm_auto_mode_id,&color[0]);
    UI_color_change(UI_reality_recv.chassis_auto_mod_id,&color[1]);
    UI_color_change(0,&color[2]);
    UI_color_change(0,&color[3]);
    UI_color_change(0,&color[4]);
    UI_color_change(0,&color[5]);
    UI_color_change(0,&color[6]);
    UI_color_change(0,&color[7]);
    UI_color_change(0,&color[8]);
    // UI_color_change(0,&color[9]);
}
void UI_reality_Task()
{
    while(!SubGetMessage(UI_reality_data_sub,&UI_reality_recv));
    UI_reality_recv.arm_auto_mode_id = 5;
    UI_reality_recv.chassis_auto_mod_id = 2;
    UI_color_select();
    memset(color,0,sizeof(color));
    static uint8_t WS2812_HighLevel = 0xf0;
    static uint8_t WS2812_LowLevel  = 0xC0;
    static uint8_t txbuf[10][24];
    for (int j = 0; j < 10; j++){
        for (int i = 0; i < 8; i++){
            txbuf[j][7-i]  = (((color[j]>>(i+8))&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
            txbuf[j][15-i] = (((color[j]>>(i+16))&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
            txbuf[j][23-i] = (((color[j]>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        }
    }
    HAL_SPI_Transmit(&hspi4, (uint8_t *)txbuf, 24*10, 10000);
    
    uint8_t res = 0;
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&hspi4, &res, 1, 0xFFFF);
    }
    // int8_t a;
    // GPIOReset(UI_reality_GPIO);
    // for (int i = 0; i < 10; i++)
    // {
    //     for(int j = 0; j < 24; j++){
    //         // todo:手动模拟时序，shi，能不能改
    //         if((color[i]>>j) & 0x01){
    //             GPIOSet(UI_reality_GPIO);
    //             a=30;while(a--);
    //             GPIOReset(UI_reality_GPIO);
    //             a=30;while(a--);
    //         }else{
    //             GPIOSet(UI_reality_GPIO);
    //             GPIOReset(UI_reality_GPIO);
    //             a=30;while(a--);
    //         }
    //     }
    // }
    // GPIOSet(UI_reality_GPIO);

    
}