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
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

/* cmd应用包含的模块实例指针和交互信息存储*/
CANCommInstance *can_comm_master; // 发布给副板遥控器信息
static Publisher_t *chassis_cmd_pub; // 底盘控制消息发布者
static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回

// static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
// static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息
// static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

#ifdef ARM_BOARD
static Publisher_t *arm_cmd_pub;
static Arm_Cmd_Data_s arm_cmd_data;
#endif



void RobotCMDInit()
{
    rc_data     = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    robot_state = ROBOT_READY;                // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入

    CANComm_Init_Config_s can_comm_conf = {
        // .recv_data_len = sizeof(RC_ctrl_t),
        .can_config = {
            .can_handle = &hfdcan2,
            .tx_id = 0x128,
        },
        .send_data_len = 7,
    };
    can_comm_master = CANCommInit(&can_comm_conf);

#ifdef CHASSIS_BOARD
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
#endif
#ifdef ARM_BOARD
    arm_cmd_pub        = PubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    memset(&arm_cmd_data,0,sizeof(arm_cmd_data));
    arm_cmd_data.state = 0x01;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        // chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_FREE_MODE;
    }

    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式
    {
        // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        // ...
    }
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) /*|| vision_recv_data->target_state == NO_TARGET*/) { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    }
    // 云台软件限位

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
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
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 && robot_state == ROBOT_READY) // 还需添加重要应用和模块离线的判断
    {
        robot_state                   = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        arm_cmd_data.state            |= 0x01;

        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (robot_state == ROBOT_STOP && switch_is_up(rc_data[TEMP].rc.switch_right)) {
        arm_cmd_data.state &= ~0x01;
        robot_state        = ROBOT_READY;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
#ifdef ARM_BOARD
    // arm_cmd_data.Translation_x = rc_data[TEMP].rc.rocker_l1 / 660.0 *0.0001;
    arm_cmd_data.Translation_x = rc_data[TEMP].rc.switch_left;
    // arm_cmd_data.Translation_y = -rc_data[TEMP].rc.rocker_l_/ 660.0 *0.0001;
    arm_cmd_data.Translation_y  = rc_data[TEMP].rc.switch_right;
    // arm_cmd_data.Position_z = rc_data[TEMP].rc.rocker_r1 / 660.0;
    // arm_cmd_data.Rotation_yaw   = rc_data[TEMP].rc.rocker_r_ / 660.0;
    
    if(robot_state == ROBOT_READY && !(arm_cmd_data.state & 0x01)){
        arm_cmd_data.Position_z -= 0.5 * rc_data[TEMP].rc.rocker_r1 / 660.0;
        arm_cmd_data.Rotation_yaw   += 0.06 * rc_data[TEMP].rc.rocker_r_ / 660.0;
    }
    /* 限幅 */
    LIMIT_MIN_MAX(arm_cmd_data.Position_z,0,616);
    LIMIT_MIN_MAX(arm_cmd_data.Rotation_yaw,-180,180);

    arm_cmd_data.state &= ~0x02;
    if(rc_data[TEMP].rc.dial < -300){
        arm_cmd_data.state |= 0x02;
    }
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    PubPushMessage(arm_cmd_pub, (void *)&arm_cmd_data);
#endif

    static uint8_t can_comm_send_data[7];
    static uint8_t send_robot_state = 0;
    send_robot_state = 0;
    if(rc_data[TEMP].rc.dial >  300) send_robot_state |= 0x01;
    if(rc_data[TEMP].rc.dial < -300) send_robot_state |= 0x02;
    if(switch_is_up(rc_data[TEMP].rc.switch_right))  send_robot_state |= 0x04;
    memcpy(can_comm_send_data,(uint8_t*)&rc_data[TEMP].rc.rocker_l_,2);
    memcpy(can_comm_send_data+2,(uint8_t*)&rc_data[TEMP].rc.rocker_l1,2);
    memcpy(can_comm_send_data+4,(uint8_t*)&rc_data[TEMP].rc.rocker_r_,2);
    memcpy(can_comm_send_data+6,&send_robot_state,1);
    CANCommSend(can_comm_master,(uint8_t*)can_comm_send_data);

#ifdef CHASSIS_BOARD
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left) || switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[中/下],遥控器控制
        RemoteControlSet();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif
}
