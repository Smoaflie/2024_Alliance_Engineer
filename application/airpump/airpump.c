// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "led.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_can.h"
#include "bsp_gpio.h"

static CANInstance *airvalve_can_instance;
static GPIOInstance *airpump_linear,*airpump_arm;

static Subscriber_t *airpump_cmd_data_sub;// 气阀/气泵控制信息收
static Airpump_Cmd_Data_s airpump_cmd_rec;
void AIRPUMPInit()
{
    GPIO_Init_Config_s gpio_conf_airpump_linear = {
        .GPIOx = airpump_linear_GPIO_Port,
        .GPIO_Pin = airpump_linear_Pin,
    };
    GPIO_Init_Config_s gpio_conf_airpump_arm = {
        .GPIOx = airpump_arm_GPIO_Port,
        .GPIO_Pin = airpump_arm_Pin,
    };
    airpump_linear = GPIORegister(&gpio_conf_airpump_linear);
    airpump_arm = GPIORegister(&gpio_conf_airpump_arm);

    CAN_Init_Config_s can_conf_airvalve = {
        .can_handle = &hfdcan2,
        .tx_id = 0x121
    };
    airvalve_can_instance = CANRegister(&can_conf_airvalve);

    airpump_cmd_data_sub = SubRegister("airpump_cmd", sizeof(Airpump_Cmd_Data_s));
}

/* 机器人气阀气泵控制核心任务 */
void AIRPUMPTask()
{
    SubGetMessage(airpump_cmd_data_sub, &airpump_cmd_rec);

    static uint8_t tx_buf[8] = {0x11,0x88,0x99,0x00,0x00,0x00,0x00,0x00};
    static uint32_t coder = 0;
    static uint16_t airvalve_state = 0;
    static uint16_t delay_time = 0;
    
    /*
    airpump_cmd_rec.mode 低位0 -> 8
            0           1           2               3        ……   
        左气泵开关 | 右气泵开关 | 气动取矿状态1 | 气动取矿状态2 ……
    */
    if(airpump_cmd_rec.mode & 0x01){
        GPIOSet(airpump_arm);
    }else{
        GPIOReset(airpump_arm);
    } 

    if(airpump_cmd_rec.mode & 0x02){
        GPIOSet(airpump_linear);
    }else{
        GPIOReset(airpump_linear);
    }

    static uint8_t airvalve_mode_cnt = 0;
    //取左侧矿
    if((airpump_cmd_rec.mode & 0x04) && !(airvalve_state&0x01)){
        if(airvalve_mode_cnt <= 10){
            airvalve_mode_cnt++;
            switch(airvalve_mode_cnt){
                //              0b2X098765X3210987654321
                case 1: coder = 0b0000000000100101101010;break;
                case 2: coder = 0b0000000000100101010110;break;
                case 3: coder = 0b0000000000100110010110;break;
                case 4: coder = 0b0000000000011010010110;break;
                case 5: coder = 0b0000000000011010010101;break;
                case 6: coder = 0b0000000000100110010101;break;
                case 7: coder = 0b0000000000100101010101;break;
                case 8: coder = 0b0000000000100101101001;break;
                case 9: coder = 0b0000000000100101101010;break;
                default: coder = 0;
            }
            delay_time = 600;
            airvalve_state |= 0x80;
        }else{
            airvalve_mode_cnt = 0;
            airvalve_state |=0x01;
            delay_time = 0;
        }
    }else{
        airvalve_state &= ~0x81;
    }
    //取中间矿
    if((airpump_cmd_rec.mode & 0x08) && !(airvalve_state&0x02)){
        // todo：避免发送过程中切换
        if(airvalve_state&0x08){

        }
        if(airvalve_mode_cnt <= 5){
            airvalve_mode_cnt++;
            switch(airvalve_mode_cnt){
                //              0b2X098765X3210987654321
                case 1: coder = 0b0000000000100101101010;break;
                case 2: coder = 0b0000000000011001100110;break;
                case 3: coder = 0b0000000000011001100101;break;
                case 4: coder = 0b0000000000100101101001;break;
                default: coder = 0;
            }
            delay_time = 850;
            airvalve_state |= 0x80;
        }else{
            airvalve_mode_cnt = 0;
            airvalve_state |=0x02;
            delay_time = 0;
        }
    }else{
        airvalve_state &= ~0x82;
    }
    
    // 发送部分，发送标志位(0x80)置位后才会发送
    // 发送标志位在状态切换的时候才会置位
    if(airvalve_state & 0x80){
        memcpy(tx_buf+3,(uint8_t*)&coder,4);
        tx_buf[7]=(uint8_t)(tx_buf[3] + tx_buf[4] + tx_buf[5] + tx_buf[6]);
        memcpy(airvalve_can_instance->tx_buff,tx_buf,8);
        CANTransmit(airvalve_can_instance,1);
        airvalve_state&=~0x80;
    }
    osDelay(delay_time == 0 ? delay_time : 1);
}