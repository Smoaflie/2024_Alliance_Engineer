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

#include "test.h"

static DJIMotorInstance *dji_motor[3];
// 编码器实例
static EncoderInstance_s *encoder[3];

// UART实例
// static USARTInstance *uart_rec;

void TestInit()
{
    Encoder_Init_Config_s encoder_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .encoder_type = me02_can,
        };
    encoder_config.can_init_config.rx_id = 0x061;
    encoder[0]                      = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id = 0x072;
    encoder[1]                     = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id = 0x050;
    encoder[2]                     = EncoderInit(&encoder_config);

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
    motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[0]->measure.total_angle;
    dji_motor[0]                           = DJIMotorInit(&motor_config);
    motor_config.can_init_config.tx_id = 2;
    motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[1]->measure.total_angle;
    dji_motor[1]                         = DJIMotorInit(&motor_config);
    motor_config.can_init_config.tx_id = 3;
    motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[2]->measure.total_angle;
    dji_motor[2]                         = DJIMotorInit(&motor_config);

    DJIMotorSetRef(dji_motor[0],-22.99f);
    DJIMotorSetRef(dji_motor[1],14.53f);
    DJIMotorSetRef(dji_motor[2],129.15f);
        DJIMotorEnable(dji_motor[0]);
        DJIMotorEnable(dji_motor[1]);
        DJIMotorEnable(dji_motor[2]);
    // USART_Init_Config_s UART_Init_Config = {
    //     .module_callback = USARTRecCBK,
    //     .recv_buff_size = 28,
    //     .usart_handle = &huart6,
    // };
    // uart_rec = USARTRegister(&UART_Init_Config);
}

float ap = 200;
float sp = 0.6;
float ai = 0;
/* 测试任务 */
void _TestTask()
{
    static uint8_t flag = 0;
    static uint8_t d_flag = 0;
    if (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)){
        osDelay(1);
        while(!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin));
        osDelay(1);
        flag = !flag;
    }

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

        DJIMotorSetRef(dji_motor[0],encoder[0]->measure.total_angle);
        DJIMotorSetRef(dji_motor[1],encoder[1]->measure.total_angle);
        DJIMotorSetRef(dji_motor[2],encoder[2]->measure.total_angle);

        d_flag = 1;

    } else {
        C_board_LEDSet(0x33ffff);
        // DJIMotorStop(dji_motor[0]);
        // DJIMotorStop(dji_motor[1]);
        // DJIMotorStop(dji_motor[2]);
        if(d_flag){
            
            DJIMotorSetRef(dji_motor[0],-22.99f);
        DJIMotorSetRef(dji_motor[1],14.53f);
        DJIMotorSetRef(dji_motor[2],129.15f);

            // DJIMotorSetRef(dji_motor[0],dji_motor[0]->motor_controller.angle_PID.Ref-2);
            // DJIMotorSetRef(dji_motor[1],dji_motor[0]->motor_controller.angle_PID.Ref-2);
            // DJIMotorSetRef(dji_motor[2],dji_motor[0]->motor_controller.angle_PID.Ref-2);
            d_flag = 0;
        }
    }
}

