/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"
#include "dji_motor.h"
#include "encoder.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD     1356 // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0    // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD         4073 // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_POS_UP_LIMIT_ECD    3670 // 云台竖直方向高处限位编码器值,若对云台有机械改动需要修改
#define PITCH_POS_DOWN_LIMIT_ECD  4455 // 云台竖直方向低处限位编码器值,若对云台有机械改动需要修改
#define PITCH_FEED_TYPE           1    // 云台PITCH轴反馈值来源:编码器为0,陀螺仪为1
#define PITCH_ECD_UP_ADD          0    // 云台抬升时编码器变化趋势,增为1,减为0 (陀螺仪变化方向应相同)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE         10    // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE             350   // 纵向轴距(前进后退方向)
#define TRACK_WIDTH            300   // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL           60    // 轮子半径
#define REDUCTION_RATIO_WHEEL  19.2f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

// 其他参数(尽量所有参数集中到此文件)
#define BUZZER_SILENCE 0 // 蜂鸣器静音,1为静音,0为正常

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum {
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
} chassis_mode_e;


typedef enum{
    
    LIFT,
    LIFT_STOP,
    LIFT_INIT,
}lift_mode_e;

//一级伸出模式设定
typedef enum{
    FIRST_YAW,
    FIRST_STRETCH,
    FIRST_STOP,
    FIRST_INIT,
}first_stretch_mode_e;
//二级伸出模式设定
typedef enum{
    SECOND_STRETCH,
    SECOND_STOP,
    SECOND_INIT,
}second_stretch_mode_e;
//横移模式设定
typedef enum {
    HORIZONTAL_ZERO_FORCE=0,
    HORIZONTAL_MOVE,
    HORIZONTAL_INIT,
} Horizontal_mode_e;
//前端模式设定
typedef enum{
    
    ROLL,
    PITCH,
    FORWARD_STOP,
    FORWARD_INIT_PITCH,
    FORWARD_INIT_ROLL,
}forward_mode_e;
// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

//////////////////待完善////////////////////////
typedef struct{
    //自动模式

}Auto_Data_s;

///////////////////////////////////////////////

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

typedef struct
{ // 一级伸出角度控制
    float left;
    float right;
    lift_mode_e lift_mode;

} Lift_Ctrl_Cmd_s;
// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 一级伸出角度控制
    float first_left;
    float first_right;
    first_stretch_mode_e first_stretch_mode;

} First_Stretch_Ctrl_Cmd_s;
typedef struct
{ // 一级伸出角度控制
    float second_left;
    float second_right;
    second_stretch_mode_e second_stretch_mode;

} Second_Stretch_Ctrl_Cmd_s;
typedef struct
{
    // 控制部分
    int32_t Horizontal_MechAngle;
    // int32_t Up_MechAngle_left;
    // int32_t Up_MechAngle_right;
    Horizontal_mode_e Horizontal_mode;
    // UI部分
    //  ...
} Horizontal_Ctrl_Cmd_s;
typedef struct
{ 
   forward_mode_e Forward_mode;
   // 记录最后一次的pitch编码器的角度
    float last_angle;     // pitch的最后一次编码器角度
    float relevant_angle; // pitch和roll的相对角度
    // 动之前的roll编码器
    float roll_last_angle; // roll的最后一次编码器角度
    float final_angle;     // 最后的角度
    int8_t mode;           // pitch和roll的模式
    int8_t last_mode;
    int16_t angel_output;
    int16_t angel_output1;
}Forward_Ctrl_Cmd_s; 



/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat; // 剩余枪口热量
    // Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color; // 0 for blue, 1 for red

} Chassis_Upload_Data_s;

typedef struct
{
    DJIMotorInstance *lift_left_speed_data,*lift_right_speed_data;
    float new_left_encoder;
    float new_right_encoder;

} Lift_Upload_Data_s;

typedef struct
{ 
    DJIMotorInstance *first_stretch_left_speed_data,*first_stretch_right_speed_data;
    EncoderInstance_s *first_stretch_left_angle_data,*first_stretch_right_angle_data;
    float new_left_encoder;
    float new_right_encoder;

}First_Stretch_Upload_Data_s; 
typedef struct
{ 
    DJIMotorInstance *second_stretch_left_speed_data,*second_stretch_right_speed_data;
    float new_left_angle;
    float new_right_angle;

}Second_Stretch_Upload_Data_s; 

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) 
    // attitude_t chassis_imu_data;
#endif
    // 后续增加真实横移量
    float Horizontal_Movement; //暂定左正右负
    float now_angel;
} Horizontal_Upload_Data_s;
typedef struct
{ 
   float new_left_angle;
    float new_forward_angle;

}Forward_Upload_Data_s; 

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H