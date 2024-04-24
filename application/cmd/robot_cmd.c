// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "can_comm.h"
#include "led.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t *chassis_cmd_pub;        // 底盘控制消息发布者
static Publisher_t *gimbal_cmd_pub;        // 云台控制消息发布者
static Publisher_t *arm_cmd_pub;    // 机械臂控制信息发布者

static Subscriber_t *arm_state_sub; // 机械臂状态接收者

static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 发送给云台应用的信息
static Arm_Cmd_Data_s arm_cmd_data; // 发送给机械臂应用的信息
static Arm_State_Data_s arm_state;

static RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回

static Robot_Status_e robot_state; // 机器人整体工作状态



void RobotCMDInit()
{
    rc_data     = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    robot_state = ROBOT_STOP;                // 启动时机器人为停止状态

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    arm_cmd_pub     = PubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    arm_state_sub = SubRegister("arm_state",sizeof(Arm_State_Data_s));

    memset(&arm_cmd_data, 0, sizeof(arm_cmd_data));
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 左侧开关状态为[下],控制底盘云台
    if (switch_is_down(rc_data->rc.switch_left)) {
        arm_cmd_data.mode = ARM_ZERO_FORCE;
        if (switch_is_mid(rc_data->rc.switch_right)) // 右侧开关状态[中],底盘跟随云台
        {
            // chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
        } else if (switch_is_up(rc_data->rc.switch_right)) // 右侧开关状态[上],底盘和云台分离,底盘保持不转动
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
        }
        // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
        gimbal_cmd_send.yaw -= rc_data->rc.rocker_r_/660.0*0.5;
        gimbal_cmd_send.pitch -= rc_data->rc.rocker_r1/660.0*0.2;
        VAL_LIMIT(gimbal_cmd_send.pitch, 0, 65);
        chassis_cmd_send.vx = 10.0f * (float)rc_data->rc.rocker_l_; // _水平方向
        chassis_cmd_send.vy = 10.0f * (float)rc_data->rc.rocker_l1; // 1数值方向
        // chassis_cmd_send.wz = 100.0f * (float)rc_data->rc.rocker_l1/660.0; // 1数值方向
    }
    // 左侧开关为[中]，控制底盘臂臂
    if (switch_is_mid(rc_data->rc.switch_left)) {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
        arm_cmd_data.mode = ARM_FREE_MODE;
        chassis_cmd_send.vx = 10.0f * (float)rc_data->rc.rocker_l_; // _水平方向
        chassis_cmd_send.vy = 10.0f * (float)rc_data->rc.rocker_l1; // 1数值方向
        // arm_cmd_data.Translation_x = rc_data->rc.rocker_l1 / 660.0 *0.0001;
        // arm_cmd_data.Translation_y = -rc_data->rc.rocker_l_/ 660.0 *0.0001;
        arm_cmd_data.Position_z -= 0.5 * rc_data->rc.rocker_r1 / 660.0;
        arm_cmd_data.Rotation_yaw += 0.06 * rc_data->rc.rocker_r_ / 660.0;
        /* 限幅 */
        LIMIT_MIN_MAX(arm_cmd_data.Position_z, -616, 616);
        LIMIT_MIN_MAX(arm_cmd_data.Rotation_yaw, -180, 180);
    }
    // 左侧开关为[上]，自由控制臂臂
    if (switch_is_up(rc_data->rc.switch_left)) {
        // gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
        // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        arm_cmd_data.mode = ARM_REFER_MODE;
        // chassis_cmd_send.vx = 10.0f * (float)rc_data->rc.rocker_l_; // _水平方向
        // chassis_cmd_send.vy = 10.0f * (float)rc_data->rc.rocker_l1; // 1数值方向
        arm_cmd_data.Translation_x = rc_data->rc.rocker_l1 / 660.0 *0.00015;
        arm_cmd_data.Translation_y = -rc_data->rc.rocker_l_/ 660.0 *0.00015;
        arm_cmd_data.Roatation_Vertical = -rc_data->rc.rocker_r_/ 660.0 *0.0015;
        arm_cmd_data.Roatation_Horizontal  = rc_data->rc.rocker_r1/ 660.0 *0.0015;
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.vx = rc_data->key[KEY_PRESS].w * 300 - rc_data->key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data->key[KEY_PRESS].s * 300 - rc_data->key[KEY_PRESS].d * 300;

    gimbal_cmd_send.yaw += (float)rc_data->mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data->mouse.y / 660 * 10;

    switch (rc_data->key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
        case 0:
            chassis_cmd_send.chassis_speed_buff = 40;
            break;
        case 1:
            chassis_cmd_send.chassis_speed_buff = 60;
            break;
        case 2:
            chassis_cmd_send.chassis_speed_buff = 80;
            break;
        default:
            chassis_cmd_send.chassis_speed_buff = 100;
            break;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    static uint8_t flag = 1;
    if (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
        osDelay(1);
        while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
            ;
        flag = !flag&1;
    }

    // 双下为急停
    if (robot_state == ROBOT_STOP ||  (switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right) || flag == 0)) // 还需添加重要应用和模块离线的判断
    {
        DM_board_LEDSet(0xd633ff);
        if(robot_state == ROBOT_READY)LOGERROR("[CMD] emergency stop!");
        robot_state                   = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        arm_cmd_data.mode             = ARM_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data->rc.switch_right) && flag == 1) {
        DM_board_LEDSet(0x33ffff);
        if(robot_state == ROBOT_STOP)   LOGINFO("[CMD] reinstate, robot ready");
        robot_state = ROBOT_READY;
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(arm_state_sub,&arm_state);
    arm_cmd_data.init_flag = arm_state.init_flag;
    if(arm_cmd_data.init_flag & Reset_arm_cmd_param_flag){
        if(arm_cmd_data.init_flag & Big_Yaw_motor_pub_reset)
        {
            arm_cmd_data.Position_z = 0;
            arm_cmd_data.init_flag &= ~(Big_Yaw_motor_pub_reset);
        }
        if(arm_cmd_data.init_flag & Z_motor_pub_reset)
        {
            arm_cmd_data.Rotation_yaw = 0;
            arm_cmd_data.init_flag &= ~(Z_motor_pub_reset);
        }
        arm_cmd_data.init_flag &= ~(Reset_arm_cmd_param_flag);    
    }
    RemoteControlSet();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
        // DM_board_LEDSet(0x00000000);

    PubPushMessage(arm_cmd_pub, (void *)&arm_cmd_data);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
