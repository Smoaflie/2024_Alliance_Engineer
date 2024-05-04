// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "encoder.h"
#include "led.h"
#include "bsp_usb.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "bsp_usb.h"

#include "test.h"
#include "usermode.hpp"

// #include "usermode.hpp"

static DJIMotorInstance *dji_motor[3];
// 编码器实例
static EncoderInstance_s *encoder[3];

// UART实例
static USARTInstance *uart_rec;

// USB实例
static uint8_t *usb_instance;

// 陀螺仪欧拉角
float eul_rec[3];
float qua_rec[4];
float encoder_Data[3];

static void DecodeUART()
{
    if(uart_rec->recv_buff[0] == 0x5A && uart_rec->recv_buff[1] == 0xA5){
        memcpy((uint8_t*)qua_rec,(uint8_t*)uart_rec->recv_buff+66,16);
    }else{
        return;
    }
    // roll pitch yaw
}

void TestInit()
{
    Encoder_Init_Config_s encoder_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .encoder_type = me02_can,
        };
    encoder_config.can_init_config.rx_id = 0x061;
    encoder_config.offset = 30811;
    encoder[1]                      = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id = 0x072;
    encoder_config.offset = 1870;

    encoder[2]                     = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id = 0x050;
    encoder_config.offset = 11529;

    encoder[0]                     = EncoderInit(&encoder_config);

    Motor_Init_Config_s motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 200, //800
                .Ki            = 0,  // 500
                .Kd            = 0,  // 0
                .IntegralLimit = 5000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 30000,
                },
            .speed_PID = {
                .Kp            = 0.6, // 4.5
                .Ki            = 0,  // 0
                .Kd            = 0,  // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
                },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M2006,
    };
    motor_config.can_init_config.tx_id = 1;
    motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[1]->measure.total_angle;
    dji_motor[1]                           = DJIMotorInit(&motor_config);
    motor_config.can_init_config.tx_id = 2;
    motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[2]->measure.total_angle;
    dji_motor[2]                         = DJIMotorInit(&motor_config);
    motor_config.can_init_config.tx_id = 3;
    motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[0]->measure.total_angle;
    dji_motor[0]                         = DJIMotorInit(&motor_config);

    DJIMotorSetRef(dji_motor[1],0);
    DJIMotorSetRef(dji_motor[2],0);
    DJIMotorSetRef(dji_motor[0],0);
    
    DJIMotorEnable(dji_motor[0]);
    DJIMotorEnable(dji_motor[1]);
    DJIMotorEnable(dji_motor[2]);

    USB_Init_Config_s conf = {.rx_cbk = NULL};  
    usb_instance = USBInit(conf);

    USART_Init_Config_s UART_Init_Config = {
        .module_callback = DecodeUART,
        .recv_buff_size = 82,
        .usart_handle = &huart1,
    };
    uart_rec = USARTRegister(&UART_Init_Config);
}

float ap = 200;
float sp = 0.6;
float ai = 0;
/* 测试任务 */
void _TestTask()
{
    static uint8_t flag = 0;
    static uint8_t d_flag = 0;
    if (!HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin)){
        osDelay(1);
        while(!HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin));
        osDelay(1);
        flag = !flag;
    }
    
    encoder_Data[0] = encoder[0]->measure.angle_single_round;
    encoder_Data[1] = encoder[1]->measure.angle_single_round;
    encoder_Data[2] = encoder[2]->measure.angle_single_round;

    // encoder_Data[0] = encoder[0]->measure.ecd / 20860.7568f;
    // encoder_Data[1] = encoder[1]->measure.ecd / 20860.7568f;
    // encoder_Data[2] = encoder[2]->measure.ecd / 20860.7568f;

    static uint8_t send_buf[30];
    uint8_t head[] = {0xff,0xff};
    // uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f};
    memcpy(send_buf,head,2);
    memcpy((uint8_t*)send_buf+2,(uint8_t*)encoder_Data,12);
    memcpy((uint8_t*)send_buf+14,(uint8_t*)qua_rec,16);
    // memcpy((uint8_t*)send_buf,(uint8_t*)encoder_Data,12);
    // memcpy((uint8_t*)send_buf+12,(uint8_t*)qua_rec,16);
    // memcpy(send_buf+24,tail,4);

    // USBTransmit((uint8_t*)send_buf,28);
    static count = 0;
    if(count++ > 30){
        USBTransmit((uint8_t*)send_buf,30);
        count = 0;
    }

    // Update(encoder_Data,eul_rec);

    dji_motor[0]->motor_controller.angle_PID.Kp = ap;
    dji_motor[1]->motor_controller.angle_PID.Kp = ap;
    dji_motor[2]->motor_controller.angle_PID.Kp = ap;

    dji_motor[0]->motor_controller.angle_PID.Ki = ai;
    dji_motor[1]->motor_controller.angle_PID.Ki = ai;
    dji_motor[2]->motor_controller.angle_PID.Ki = ai;

    dji_motor[0]->motor_controller.speed_PID.Kp = sp;
    dji_motor[1]->motor_controller.speed_PID.Kp = sp;
    dji_motor[2]->motor_controller.speed_PID.Kp = sp;

    if(flag){
        C_board_LEDSet(0xd633ff);

        // DJIMotorEnable(dji_motor[0]);
        // DJIMotorEnable(dji_motor[1]);
        // DJIMotorEnable(dji_motor[2]);

        DJIMotorSetRef(dji_motor[1],encoder[1]->measure.total_angle);
        DJIMotorSetRef(dji_motor[2],encoder[2]->measure.total_angle);
        DJIMotorSetRef(dji_motor[0],encoder[0]->measure.total_angle);

        d_flag = 1;

    } else {
        C_board_LEDSet(0x33ffff);
        // DJIMotorStop(dji_motor[0]);
        // DJIMotorStop(dji_motor[1]);
        // DJIMotorStop(dji_motor[2]);
        if(d_flag){
            
            DJIMotorSetRef(dji_motor[1],0);
            DJIMotorSetRef(dji_motor[2],0);
            DJIMotorSetRef(dji_motor[0],0);

            // DJIMotorSetRef(dji_motor[0],dji_motor[0]->motor_controller.angle_PID.Ref-2);
            // DJIMotorSetRef(dji_motor[1],dji_motor[0]->motor_controller.angle_PID.Ref-2);
            // DJIMotorSetRef(dji_motor[2],dji_motor[0]->motor_controller.angle_PID.Ref-2);
            d_flag = 0;
        }
    }
}

