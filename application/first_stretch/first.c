#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

#include "first.h"


static DJIMotorInstance *left_speed_motor, *right_speed_motor;
static Publisher_t *first_stretch_pub;                          // 一级应用消息发布者(一级反馈给cmd)
static Subscriber_t *first_stretch_sub;                         // cmd控制消息订阅者
static First_Stretch_Upload_Data_s first_stretch_feedback_data; // 回传给cmd的一级状态信息
static First_Stretch_Ctrl_Cmd_s first_stretch_cmd_recv;         // 来自cmd的控制信息

void First_Stretch_Init()
{
    Motor_Init_Config_s first_stretch_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1,
                .Ki            = 0,
                .Kd            = 0,
                // .DeadBand      = 0.1f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0,
                .MaxOut = 15000,
            },
            .angle_PID = {
                .Kp            = 1,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0,
                .MaxOut        = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
        },
        .motor_type = M3508};
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    // right_speed_motor = DJIMotorInit(&first_stretch_right_config);
    // left_speed_motor  = DJIMotorInit(&first_stretch_left_config);
    first_stretch_config.can_init_config.tx_id                             = 1;
    first_stretch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    left_speed_motor                                                            = DJIMotorInit(&first_stretch_config);

    first_stretch_config.can_init_config.tx_id                             = 2;
    first_stretch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    right_speed_motor                                                            = DJIMotorInit(&first_stretch_config);

    first_stretch_pub = PubRegister("first_stretch_feed", sizeof(First_Stretch_Upload_Data_s));
    first_stretch_sub = SubRegister("first_stretch_cmd", sizeof(First_Stretch_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void First_Stretch_Task()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(first_stretch_sub, &first_stretch_cmd_recv);

    DJIMotorEnable(left_speed_motor);
    DJIMotorEnable(right_speed_motor);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (first_stretch_cmd_recv.first_stretch_mode) {
        // 停止
        case FIRST_STOP:
            DJIMotorStop(left_speed_motor);
            DJIMotorStop(right_speed_motor);
            break;
        default:
            break;
    }
    DJIMotorSetRef(left_speed_motor, first_stretch_cmd_recv.first_left);
    DJIMotorSetRef(right_speed_motor, first_stretch_cmd_recv.first_right);
    

    // // 设置反馈数据,主要是imu和yaw的ecd
    first_stretch_feedback_data.new_left_encoder  = left_speed_motor->measure.total_angle;
    first_stretch_feedback_data.new_right_encoder = right_speed_motor->measure.total_angle;

    // 推送消息
    PubPushMessage(first_stretch_pub, (void *)&first_stretch_feedback_data);
}