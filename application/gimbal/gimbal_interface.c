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

#define GetAngleBetween360(a) ((a)-(360*(int32_t)((a)/360)))
#define GetAngleBetween180(angle) ((angle)>180)?((angle)-360):(((angle)<-180)?((angle)+360):(angle))

static LKMotorInstance* gimbal_yaw_motor;
static ServoInstance* gimbal_pitch_motor;

static Subscriber_t *gimbal_sub;                   // 用于订阅云台的控制命令
static Publisher_t *gimbal_pub;                   // 用于发送云台的数据信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 云台应用接收的信息
static Gimbal_Data_s     gimbal_data_send;// 云台发布的信息
static float gimbal_yaw_angle,gimbal_pitch_angle;

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
{
    Motor_Init_Config_s config ={
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 0.4, // 1
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
            .other_angle_feedback_ptr = &gimbal_imu_yaw_total_angle,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type            = LK_MS5005,
        .can_init_config.tx_id = 1,
        .motor_contro_type = TORQUE_LOOP_CONTRO,
        };
    gimbal_yaw_motor = LKMotorInit(&config);

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
}
void GimbalInit_IO()
{
    GPIO_Init_Config_s gpio_conf_relay_contro = {
        .GPIOx = relay_contro_GPIO_Port,
        .GPIO_Pin = relay_contro_Pin,
    };
    relay_contro_gpio = GPIORegister(&gpio_conf_relay_contro);
}
void GimbalInit_Param()
{
    gimbal_pitch_angle = 0;
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
        while((gimbal_yaw_motor->measure.feed_dt == 0) || (gimbal_cmd_recv.arm_big_yaw_offset==0))    {SubGetMessage(gimbal_sub, &gimbal_cmd_recv);osDelay(1);}
        
        gimbal_imu_offset = gimbal_imu_yaw_total_angle - gimbal_cmd_recv.arm_big_yaw_offset - gimbal_yaw_motor->measure.total_angle;
        gimbal_imu_init = 1;
    }

    /* 云台电机掉线监测 */
    if(!LKMotorIsOnline(gimbal_yaw_motor))
    {
        osDelay(10000);
        if(!LKMotorIsOnline(gimbal_yaw_motor))
        {
            GPIOReset(relay_contro_gpio);
            LOGWARNING("[Gimbal] Gimbal yaw motor lost, trying to restart it.");
            osDelay(2000);
            GPIOSet(relay_contro_gpio);
        }
    }
}
void GimbalContro()
{
    if(gimbal_imu_init)
    {
        /* 云台控制 */
        if(gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)    
        {
            LKMotorStop(gimbal_yaw_motor);
            Servo_Motor_Stop(gimbal_pitch_motor);
        }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE)
        {
            Servo_Motor_Start(gimbal_pitch_motor);
            LKMotorEnable(gimbal_yaw_motor);

            gimbal_pitch_angle -= gimbal_cmd_recv.pitch;
            VAL_LIMIT(gimbal_pitch_angle, 0, 65);    /* 限幅 */

            gimbal_yaw_angle -= gimbal_cmd_recv.yaw;

            Servo_Motor_FreeAngle_Set(gimbal_pitch_motor,(int16_t)gimbal_pitch_angle);
            LKMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle);
            // LKMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle-gimbal_cmd_recv.arm_big_yaw_offset);
        }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_FOLLOW_YAW){
            Servo_Motor_Start(gimbal_pitch_motor);
            LKMotorEnable(gimbal_yaw_motor);

            gimbal_pitch_angle -= gimbal_cmd_recv.pitch;
            VAL_LIMIT(gimbal_pitch_angle, 0, 65);    /* 限幅 */

            gimbal_yaw_angle = 0;

            Servo_Motor_FreeAngle_Set(gimbal_pitch_motor,(int16_t)gimbal_pitch_angle);
            LKMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle+gimbal_cmd_recv.arm_big_yaw_offset);
            // LKMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle);
        }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_RESET){
            Servo_Motor_Start(gimbal_pitch_motor);
            LKMotorEnable(gimbal_yaw_motor);

            // gimbal_pitch_angle = 0;
            LKMotorSetRef(gimbal_yaw_motor,gimbal_yaw_angle);

                gimbal_yaw_motor->measure.total_round = (gimbal_yaw_motor->measure.total_round==-1) ? -1 : 0;
                gimbal_yaw_motor->measure.total_angle = gimbal_yaw_motor->measure.angle_single_round;

            gimbal_yaw_angle = 0;
            gimbal_imu_yaw_total_round = 0;
            gimbal_imu_yaw_total_angle = gimbal_imu_yaw_single_angle + gimbal_imu_offset;

            gimbal_imu_offset = gimbal_imu_yaw_single_angle;

            // Servo_Motor_FreeAngle_Set(gimbal_pitch_motor,(int16_t)gimbal_pitch_angle);
        }else if(gimbal_cmd_recv.gimbal_mode == GIMBAL_RESET_WITH_ROTATE){
            Servo_Motor_Start(gimbal_pitch_motor);
            LKMotorEnable(gimbal_yaw_motor);

                gimbal_yaw_motor->measure.total_round = 0;
                gimbal_yaw_motor->measure.total_angle = gimbal_yaw_motor->measure.angle_single_round;

            gimbal_imu_yaw_total_round = 0;
            gimbal_imu_yaw_total_angle = gimbal_imu_yaw_single_angle;
            gimbal_yaw_angle = 0;

            LKMotorSetRef(gimbal_yaw_motor,0);
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
    PubPushMessage(gimbal_pub,&gimbal_data_send);
}

void GimbalDebugInterface()
{
    ;
}