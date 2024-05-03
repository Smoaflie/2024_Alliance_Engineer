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
// static USARTInstance* imu_usart_instance;
// static float gimbal_yaw_angle;
// static uint8_t gimbal_rec[100];
// static void imu_usart_callback(){
//     uint16_t crc,payload_len;
//     memcpy(gimbal_rec,imu_usart_instance->recv_buff,imu_usart_instance->recv_buff_size);
//     if(gimbal_rec[0]==0x5A && gimbal_rec[1]==0xA5)
//     {
//         payload_len = gimbal_rec[2] + (gimbal_rec[3]<<8);
//         // crc = gimbal_rec[4] + (gimbal_rec[5]<<8);
//         // uint16_t crc_d  =crc_16(gimbal_rec+6,payload_len);
//         // if(crc == crc_d)
//         if(payload_len == 7)
//             gimbal_yaw_angle = (int16_t)(gimbal_rec[11]+(gimbal_rec[12]<<8))*0.01f;
//     }
// }
static DJIMotorInstance *air_sucker_motor;

void selfTestInit()
{
    // USART_Init_Config_s uart_conf;
    // uart_conf.module_callback = imu_usart_callback;
    // uart_conf.usart_handle = &huart9;
    // uart_conf.recv_buff_size = 13;
    // imu_usart_instance = USARTRegister(&uart_conf);
    Motor_Init_Config_s air_sucker_motor_config = {
        .can_init_config.can_handle   = &hfdcan2,
        .can_init_config.tx_id        = 4,
        .controller_param_init_config = {
            .torque_PID = {
                .Kp            = 0, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 50,
                .MaxOut        = 10,
            },
            .angle_PID = {
                .Kp            = 10, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 50,
                .MaxOut        = 10000,
            },
            .speed_PID = {
                .Kp            = 1, // 4.5
                .Ki            = 0, // 0
                .Kd            = 0.001, // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP | TORQUE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    air_sucker_motor = DJIMotorInit(&air_sucker_motor_config);
    
}

static float zero_angle;

//吸盘朝前
static void airsucker_forward(){
    // DJIMotorEnable(air_sucker_motor);
    DJIMotorOuterLoop(air_sucker_motor,ANGLE_LOOP);
    DJIMotorSetRef(air_sucker_motor,zero_angle+3600);
}
//吸盘朝上
static void airsucker_up(){
    // DJIMotorEnable(air_sucker_motor);
    static uint16_t init_cnt = 0,init_flag=0;
    if(init_cnt > 1000){
        if(init_flag==0){
            init_flag=1;
            air_sucker_motor->measure.total_round = 0;
            zero_angle = air_sucker_motor->measure.angle_single_round>180?(air_sucker_motor->measure.angle_single_round-360):air_sucker_motor->measure.angle_single_round;
        }
        DJIMotorOuterLoop(air_sucker_motor,ANGLE_LOOP);
        DJIMotorSetRef(air_sucker_motor,zero_angle);
    }else{
        DJIMotorOuterLoop(air_sucker_motor,SPEED_LOOP);
        DJIMotorSetRef(air_sucker_motor,-5000);
        if(air_sucker_motor->measure.speed_aps > -500)   init_cnt++;
        else init_cnt = 0;
    }
    
    
}

// float LKangle = 0,gimbalangle=0;
/* 机器人机械臂控制核心任务 */
void selfTestTask()
{
    static float ref;
    static uint8_t mode,state;
    switch(state){
        case 0:DJIMotorStop(air_sucker_motor);break;
        case 1:DJIMotorEnable(air_sucker_motor);
    }
    switch(mode){
        case 0:DJIMotorOuterLoop(air_sucker_motor,SPEED_LOOP);break;
        case 1:DJIMotorOuterLoop(air_sucker_motor,ANGLE_LOOP);break;
        case 2:DJIMotorOuterLoop(air_sucker_motor,TORQUE_LOOP);break;
    }
    DJIMotorSetRef(air_sucker_motor,ref);
    switch(mode){
        case 3:airsucker_forward();break;
        case 4:airsucker_up();
    }
}