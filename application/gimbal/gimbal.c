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
#include "led.h"
#include "crc16.h"
// bsp
#include "encoder.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "servo_motor.h"

static LKMotorInstance* motor;
static ServoInstance* gimbalmoto;

static Subscriber_t *gimbal_sub;                   // 用于订阅云台的控制命令
static Publisher_t *gimbal_pub;                   // 用于发送云台的数据信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 云台应用接收的信息
static Gimbal_Data_s     gimbal_data_send;// 云台发布的信息
static float gimbal_yaw_angle,gimbal_pitch_angle;
static uint8_t gimbal_rec[30];
static USARTInstance* imu_usart_instance;
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
void GIMBALInit()
{
    Motor_Init_Config_s config ={
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 1, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 0,
                .MaxOut        = 30,
            },
            .speed_PID = {
                .Kp            = 40, // 0
                .Ki            = 0,  // 0
                .Kd            = 0,      // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 100,
                .MaxOut        = 1000, // 20000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type            = LK_MS5005,
        .can_init_config.tx_id = 1};
    motor = LKMotorInit(&config);

    Servo_Init_Config_s servo_config = {
        // 舵机安装选择的定时器及通道
        // C板有常用的7路PWM输出:TIM1-1,2,3,4 TIM8-1,2,3
        .htim    = &htim2,
        .Channel = TIM_CHANNEL_1,
        // 舵机的初始化模式和类型
        .Servo_Angle_Type = Start_mode,
        .Servo_type       = Servo180,
    };
    gimbalmoto = ServoInit(&servo_config);
    
    gimbal_pitch_angle = 0;

    gimbal_sub = SubRegister("gimbal_cmd", sizeof(gimbal_cmd_recv));
    gimbal_pub = PubRegister("gimbal_data", sizeof(gimbal_data_send));
    // USART_Init_Config_s uart_conf;
    // uart_conf.module_callback = imu_usart_callback;
    // uart_conf.usart_handle = &huart9;
    // uart_conf.recv_buff_size = 13;
    // imu_usart_instance = USARTRegister(&uart_conf);
}

/* 机器人机械臂控制核心任务 */
void GIMBALTask()
{
    while(!SubGetMessage(gimbal_sub, &gimbal_cmd_recv));

    if(gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)    
    {
        LKMotorStop(motor);
        Servo_Motor_Stop(gimbalmoto);
    }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE)
    {
        Servo_Motor_Start(gimbalmoto);
        LKMotorEnable(motor);

        gimbal_pitch_angle -= gimbal_cmd_recv.pitch;
        VAL_LIMIT(gimbal_pitch_angle, 0, 65);    /* 限幅 */

        gimbal_yaw_angle -= gimbal_cmd_recv.yaw;

        Servo_Motor_FreeAngle_Set(gimbalmoto,(int16_t)gimbal_pitch_angle);
        LKMotorSetRef(motor,gimbal_yaw_angle-gimbal_cmd_recv.arm_big_yaw_offset);
    }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_RESET){
        Servo_Motor_Start(gimbalmoto);
        LKMotorEnable(motor);

        gimbal_pitch_angle = 0;
        motor->measure.total_round = 0;
        motor->measure.total_angle = motor->measure.angle_single_round;
        gimbal_yaw_angle = 0;
        Servo_Motor_FreeAngle_Set(gimbalmoto,(int16_t)gimbal_pitch_angle);
        LKMotorSetRef(motor,gimbal_yaw_angle);
    }
    gimbal_data_send.yaw = gimbal_yaw_angle-gimbal_cmd_recv.arm_big_yaw_offset;
    gimbal_data_send.pitch = gimbal_pitch_angle;
    PubPushMessage(gimbal_pub,&gimbal_data_send);
}