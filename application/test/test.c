// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "LKmotor.h"
#include "servo_motor.h"
#include "dji_motor.h"
#include "led.h"
// bsp
#include "encoder.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "DRmotor.h"
static Subscriber_t *UI_reality_data_sub;                   // 用于接收物理UI的数据信息
static UI_reality_Data_s UI_reality_recv;  // 物理UI接收到的信息

static GPIOInstance *UI_reality_GPIO;
static uint32_t color[10];
void selfTestInit()
{
    GPIO_Init_Config_s UI_reality_GPIO_conf = {
        .GPIOx = GPIOE,
        .GPIO_Pin = GPIO_PIN_13,
    };
    UI_reality_GPIO = GPIORegister(&UI_reality_GPIO_conf);

    memset(color,0,sizeof(color));
    UI_reality_data_sub = SubRegister("UI_reality", sizeof(UI_reality_Data_s));
}

void selfTestTask()
{
    memset(color,0,sizeof(color));
    int16_t a;
    GPIOReset(UI_reality_GPIO);
    for (int i = 0; i < 9; i++)
    {
        for(int j = 0; j < 24; j++){
            if(color[i]>>j & 0x01){
                GPIOSet(UI_reality_GPIO);
                a=30;while(a--);
                GPIOReset(UI_reality_GPIO);
                a=30;while(a--);
            }else{
                GPIOSet(UI_reality_GPIO);
                GPIOReset(UI_reality_GPIO);
                a=30;while(a--);
            }
        }
        
        GPIOSet(UI_reality_GPIO);
    }

    
}