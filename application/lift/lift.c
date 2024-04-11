#include "lift.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "encoder.h"

static DJIMotorInstance *lift_left_motor, *lift_right_motor;
static Publisher_t *lift_pub;                          // 升降应用消息发布者(升降反馈给cmd)
static Subscriber_t *lift_sub;                         // cmd控制消息订阅者
static Lift_Upload_Data_s lift_feedback_data; // 回传给cmd的升降状态信息
static Lift_Ctrl_Cmd_s lift_cmd_recv;         // 来自cmd的控制信息

void Lift_Init()
{
// 升降左电机
    Motor_Init_Config_s lift_left_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .tx_id      = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 0,
                .Ki            = 0,
                .Kd            = 0,
                .DeadBand      = 0.1f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0,
                .MaxOut = 0,
            },
            .speed_PID = {
                .Kp            = 0,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0,
                .MaxOut        = 0,
            },
            //.other_angle_feedback_ptr = &left_angle_motor->measure.total_angle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            //.other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    // 升降右电机
    Motor_Init_Config_s lift_right_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .tx_id      = 6,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 0,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0,
                .MaxOut        = 0,
            },
            .speed_PID = {
                .Kp            = 0,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0,
                .MaxOut        = 0,
            },
            //.other_angle_feedback_ptr = &right_angle_motor->measure.total_angle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            //.other_speed_feedback_ptr = (&gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    lift_left_motor = DJIMotorInit(&lift_left_config);
    lift_right_motor  = DJIMotorInit(&lift_right_config);

    lift_pub = PubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
    lift_sub = SubRegister("lift_cmd", sizeof(Lift_Ctrl_Cmd_s));
}

void Lift_Task()
{
// 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(lift_sub, &lift_cmd_recv);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (lift_cmd_recv.lift_mode) {
        // 停止
        case LIFT_STOP:
            DJIMotorStop(lift_left_motor);
            DJIMotorStop(lift_right_motor);
            break;
        // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case LIFT: // 后续只保留此模式
            DJIMotorEnable(lift_left_motor);
            DJIMotorEnable(lift_right_motor);
            //DJIMotorChangeFeed(lift_left_motor, ANGLE_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(left_speed_motor, SPEED_LOOP, OTHER_FEED);
            //DJIMotorChangeFeed(lift_right_motor, ANGLE_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(right_speed_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorSetRef(lift_left_motor, lift_cmd_recv.left); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            DJIMotorSetRef(lift_right_motor, lift_cmd_recv.right);
            break;
        default:
            break;
    }

    

    // 设置反馈数据,主要是imu和yaw的ecd
    lift_feedback_data.new_left_encoder  = lift_left_motor->measure.total_angle;
    lift_feedback_data.new_right_encoder = lift_right_motor->measure.total_angle;

    // 推送消息
    PubPushMessage(lift_pub, (void *)&lift_feedback_data);
}