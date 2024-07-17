// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "servo_motor.h"
#include "led.h"
#include "crc16.h"
// bsp
#include "encoder.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "servo_motor.h"
#include "user_lib.h"

#define GetAngleBetween360(a) ((a)-(360*(int32_t)((a)/360)))
#define GetAngleBetween180(angle) ((angle)>180)?((angle)-360):(((angle)<-180)?((angle)+360):(angle))

DJIMotorInstance* gimbal_yaw_motor;
static ServoInstance* gimbal_pitch_motor;

static Subscriber_t *gimbal_sub;                   // 用于订阅云台的控制命令
static Publisher_t *gimbal_pub;                   // 用于发送云台的数据信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 云台应用接收的信息
static Gimbal_Data_s     gimbal_data_send;// 云台发布的信息
static float gimbal_yaw_angle = 0,gimbal_pitch_angle;

static GPIOInstance *relay_contro_gpio; //继电器io口

static CANInstance* IMU_gimbal;
static float gimbal_eulur[3];
static float gimbal_imu_offset;
static float gimbal_imu_yaw_total_angle,gimbal_imu_yaw_total_round,gimbal_imu_yaw_last_angle,gimbal_imu_yaw_single_angle = 0;
static uint8_t gimbal_imu_init;

void IMU_CAN_REC_CALLBACK(CANInstance* instance)
{
    int16_t eulur_rec[3];
    memcpy(eulur_rec, instance->rx_buff, instance->rx_len);

    gimbal_eulur[0] = eulur_rec[0] * 0.01f;
    gimbal_eulur[1] = eulur_rec[1] * 0.01f;
    gimbal_imu_yaw_last_angle = gimbal_imu_yaw_single_angle;
    gimbal_imu_yaw_single_angle = gimbal_eulur[2] = eulur_rec[2] * 0.01f; //yaw

    if ((gimbal_imu_yaw_single_angle - gimbal_imu_yaw_last_angle) > 180)
        gimbal_imu_yaw_total_round--;
    else if ((gimbal_imu_yaw_single_angle - gimbal_imu_yaw_last_angle) < -180)
        gimbal_imu_yaw_total_round++;
        
    gimbal_imu_yaw_total_angle = 360*gimbal_imu_yaw_total_round + gimbal_imu_yaw_single_angle - gimbal_imu_offset;
}
void GimbalInit_Communication()
{
    CAN_Init_Config_s imu_can_config = {
        .can_handle = &hfdcan2,
        .rx_id = 0x381,
        .can_module_callback = IMU_CAN_REC_CALLBACK,
    };
    IMU_gimbal = CANRegister(&imu_can_config);

    gimbal_sub = SubRegister("gimbal_cmd", sizeof(gimbal_cmd_recv));
    gimbal_pub = PubRegister("gimbal_data", sizeof(gimbal_data_send));
}
void GimbalInit_Motor()
{    Motor_Init_Config_s gimbal_motor_config = {
        .can_init_config = {
            .can_handle   = &hfdcan2,
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1.5, // 3.5
                .Ki            = 0,  // 0
                .Kd            = 0.001,  // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16384,
                },
            .angle_PID = {
                .Kp            = 30,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 20000,
                .MaxOut        = 1500,
            },
            .other_angle_feedback_ptr = &gimbal_imu_yaw_total_angle,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
        },
        .motor_type = GM6020,
    };
    gimbal_yaw_motor                                                               = DJIMotorInit(&gimbal_motor_config);
    gimbal_yaw_motor->measure.offset_ecd = 6922;

    Servo_Init_Config_s servo_config = {
        // 舵机安装选择的定时器及通道
        // C板有常用的7路PWM输出:TIM1-1,2,3,4 TIM8-1,2,3
        .htim    = &htim2,
        .Channel = TIM_CHANNEL_1,
        // 舵机的初始化模式和类型
        .Servo_Angle_Type = Start_mode,
        .Servo_type       = Servo180,
    };
    gimbal_pitch_motor = ServoInit(&servo_config);
    gimbal_pitch_angle = 145;
    Servo_Motor_FreeAngle_Set(gimbal_pitch_motor,(int16_t)gimbal_pitch_angle);

}
void GimbalInit_IO()
{
    GPIO_Init_Config_s gpio_conf_relay_contro = {
        .GPIOx = relay_contro_GPIO_Port,
        .GPIO_Pin = relay_contro_Pin,
    };
    relay_contro_gpio = GPIORegister(&gpio_conf_relay_contro);
    GPIOSet(relay_contro_gpio);
}
void GimbalInit_Param()
{
}

void GimbalSubMessage()
{
    while(!SubGetMessage(gimbal_sub, &gimbal_cmd_recv));
}
void GimbalParamPretreatment()
{
    /* 初始化云台陀螺仪yaw角度偏移量(使之零点为车辆正方向) */
    if(!gimbal_imu_init)
    {
        // while((gimbal_yaw_motor->dt == 0) || (gimbal_cmd_recv.arm_big_yaw_offset==0))    {SubGetMessage(gimbal_sub, &gimbal_cmd_recv);osDelay(1);}
        
        gimbal_imu_offset = gimbal_imu_yaw_total_angle - gimbal_cmd_recv.arm_big_yaw_offset - gimbal_yaw_motor->measure.total_angle;
        gimbal_imu_init = 1;

        // static uint8_t reset_gimbal_zero_point[] = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        // CANTransmit_once(gimbal_yaw_motor->motor_can_ins->can_handle,
        //                 gimbal_yaw_motor->motor_can_ins->tx_id,
        //                 reset_gimbal_zero_point, 2);

    }

    DJIMotorEnable(gimbal_yaw_motor);

    /* 云台电机掉线监测 */
    // static uint16_t cnt = 0;
    // if(!LKMotorIsOnline(gimbal_yaw_motor) || gimbal_cmd_recv.gimbal_debug_flag)
    // {
    //     DJIMotorStop(gimbal_yaw_motor);
    //     cnt++;
    //     if(cnt > 500)
    //     {
    //         GPIOReset(relay_contro_gpio);
    //         LOGWARNING("[Gimbal] Gimbal yaw motor lost, trying to restart it.");
    //         osDelay(100);
    //         GPIOSet(relay_contro_gpio);
    //         osDelay(2000);
    //         gimbal_yaw_angle = gimbal_cmd_recv.arm_big_yaw_offset;
    //         gimbal_imu_yaw_total_round = 0;
    //     }
    // }else cnt = 0;
}
void GimbalContro()
{
    if(gimbal_imu_init)
    {
        if(gimbal_yaw_angle>gimbal_imu_yaw_total_angle + 360) gimbal_yaw_angle-=360;
        else if(gimbal_yaw_angle < gimbal_imu_yaw_total_angle - 360)    gimbal_yaw_angle+=360;
        gimbal_yaw_motor->motor_settings.angle_feedback_source = OTHER_FEED; //todo:
        /* 云台控制 */
        if(gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)    
        {
            DJIMotorStop(gimbal_yaw_motor);
            // Servo_Motor_Stop(gimbal_pitch_motor);
        }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE)
        {
            VAL_LIMIT(gimbal_cmd_recv.pitch, -5, 5);
            gimbal_pitch_angle += gimbal_cmd_recv.pitch;
            VAL_LIMIT(gimbal_pitch_angle, 0, 150);    /* 限幅 */

            gimbal_yaw_angle -= gimbal_cmd_recv.yaw;

            Servo_Motor_FreeAngle_Set(gimbal_pitch_motor,(int16_t)gimbal_pitch_angle);

            DJIMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle);
            // DJIMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle-gimbal_cmd_recv.arm_big_yaw_offset);
        }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_FOLLOW_YAW){

            gimbal_pitch_angle += gimbal_cmd_recv.pitch;
            VAL_LIMIT(gimbal_pitch_angle, 0, 180);    /* 限幅 */

            gimbal_yaw_angle = gimbal_cmd_recv.arm_big_yaw_offset;

            Servo_Motor_FreeAngle_Set(gimbal_pitch_motor,(int16_t)gimbal_pitch_angle);

            DJIMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle);
            // DJIMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle);
        }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_RESET){

            // gimbal_pitch_angle = 0;

            DJIMotorSetRef(gimbal_yaw_motor,0);

                gimbal_yaw_motor->measure.total_round = (gimbal_yaw_motor->measure.total_round==-1) ? -1 : 0;
                gimbal_yaw_motor->measure.total_angle = gimbal_yaw_motor->measure.angle_single_round;

            gimbal_yaw_angle = 0;
            gimbal_imu_yaw_total_round = 0;
            gimbal_imu_yaw_total_angle = gimbal_imu_yaw_single_angle + gimbal_imu_offset;

            gimbal_imu_offset = gimbal_imu_yaw_single_angle;

            // Servo_Motor_FreeAngle_Set(gimbal_pitch_motor,(int16_t)gimbal_pitch_angle);
        }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_RESET_WITH_ROTATE){
            gimbal_yaw_motor->motor_settings.angle_feedback_source = MOTOR_FEED;

                gimbal_yaw_motor->measure.total_round = 0;
                gimbal_yaw_motor->measure.total_angle = gimbal_yaw_motor->measure.angle_single_round;

            gimbal_imu_yaw_total_round = 0;
            gimbal_imu_yaw_total_angle = gimbal_imu_yaw_single_angle;
            gimbal_yaw_angle = 0;

            DJIMotorSetRef(gimbal_yaw_motor,0);
        }
    }
}
void GimbalPubMessage()
{
    float angle = gimbal_imu_yaw_single_angle - gimbal_imu_offset;
    gimbal_data_send.yaw_imu = GetAngleBetween180(GetAngleBetween360(angle));
   
    angle = gimbal_yaw_motor->measure.angle_single_round + gimbal_cmd_recv.arm_big_yaw_offset;
    gimbal_data_send.yaw_motor = GetAngleBetween180(GetAngleBetween360(angle));

    angle = gimbal_data_send.yaw_imu - gimbal_data_send.yaw_motor;
    gimbal_data_send.yaw_offset = (angle>180)?(angle-360):((angle<-180)?(angle+360):angle);
    gimbal_data_send.pitch = gimbal_pitch_angle;
    gimbal_data_send.pitch_imu = gimbal_eulur[0];
    PubPushMessage(gimbal_pub,&gimbal_data_send);
}

void GimbalDebugInterface()
{
    ;
}