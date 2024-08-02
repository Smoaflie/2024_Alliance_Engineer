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

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ROBOT_TEST

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD     2711 // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0    // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD         3412 // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE           0    // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE           0    // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE             350   // 纵向轴距(前进后退方向)
#define TRACK_WIDTH            300   // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL           60    // 轮子半径
#define REDUCTION_RATIO_WHEEL  19.2f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

//陀螺仪默认环境温度
#define BMI088_AMBIENT_TEMPERATURE 25.0f

#define GYRO2GIMBAL_DIR_YAW           1 // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH         1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL          1 // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 其他参数(尽量所有参数集中到此文件)
#define BUZZER_SILENCE 0 // 蜂鸣器静音,1为静音,0为正常

// 陀螺仪校准数据，开启陀螺仪校准后可从INS中获取
#define BMI088_PRE_CALI_GYRO_X_OFFSET -0.000909539289f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET 0.00354450056f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.000225723968f
// 陀螺仪默认环境温度
#define BMI088_AMBIENT_TEMPERATURE 25.0f
// 设置陀螺仪数据相较于云台的yaw,pitch,roll的方向
#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, -1.0f, 0.0f},                  \
        {1.0f, 0.0f, 0.0f},             \
    {                                    \
        0.0f, 0.0f, 1.0f                \
    }

#define INS_YAW_ADDRESS_OFFSET   2  // 陀螺仪数据相较于云台的yaw的方向
#define INS_PITCH_ADDRESS_OFFSET 1  // 陀螺仪数据相较于云台的pitch的方向
#define INS_ROLL_ADDRESS_OFFSET  0  // 陀螺仪数据相较于云台的roll的方向

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
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移和受控旋转
    CHASSIS_NO_FOLLOW_CONVERTMODE,         // 不跟随，允许全向平移和受控旋转-兑矿模式转90°
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_FOLLOW_GIMBAL_YAW_REVERSE, // 跟随模式，但头朝向背面
} chassis_mode_e;

// 云台模式设置
typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_RESET,          // 云台复位，指向正前方
    GIMBAL_FOLLOW_YAW,     // 云台跟随大Yaw
    GIMBAL_RESET_WITH_ROTATE,   // 云台小陀螺复位
} gimbal_mode_e;

// 臂臂模式设置
typedef enum {
    ARM_ZERO_FORCE = 0, // 电流零输入
    ARM_BASE_CONTRO_MODE,      // 臂臂基础控制(大Yaw&Z)
    ARM_FIXED,          // 保持位置
    ARM_CUSTOM_CONTRO,  // 自定义控制器(上位机控制，优先级最高)
    ARM_AUTO_MODE
} arm_mode_e;

// 机器人控制模式
typedef enum{
    ControlMode_None = 0,
    ControlMode_FetchCube,
    ControlMode_ConvertCube,
    ControlMode_Move,
    ControlMode_ReverseMove,
    ControlMode_RotateMove
}_RobotControlMode;
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
    float gimbal_pitch_imu;
    float arm_height;
    chassis_mode_e chassis_mode;
    _RobotControlMode robotControlMode;

    uint8_t special_func_flag;  // 特殊动作
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    // float chassis_rotate_wz;
    float arm_big_yaw_offset;

    gimbal_mode_e gimbal_mode;

    uint8_t gimbal_debug_flag;
} Gimbal_Ctrl_Cmd_s;

// gimbal发布的云台数据,由cmd订阅
typedef struct
{ // 云台角度控制
    float yaw_imu;
    float yaw_motor;
    float yaw_offset;
    float pitch;
    float pitch_imu;
} Gimbal_Data_s;

// airpump发布的云台数据,由cmd订阅
typedef struct
{ // 云台角度控制
    int airvalve_mode;
    int air_auto_mode_selecting;
    int16_t pump_air_arm;
    int16_t pump_air_valve;
} Airpump_Data_s;

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


/* ----------------臂臂的收发数据----------------*/
/**
 * @brief 由cmd订阅和发送.
 *
 */

typedef struct {
    float big_yaw_angle;
    float mid_yaw_angle;
    float assorted_yaw_angle;
    float assorted_roll_angle;
    float tail_motor_angle;
    float height;
} arm_controller_data_s;
typedef struct {
    struct{
        uint8_t bigyaw : 1;
        uint8_t midyaw : 1;
        uint8_t assortedyaw : 1;
        uint8_t assortedroll : 1;
        uint8_t tail : 1;
        uint8_t height : 1;
    }motor_state;

    struct{
        uint8_t bigyaw : 1;
        uint8_t assortedyaw : 1;
        uint8_t assortedroll : 1;
        uint8_t tail : 1;
    }encoder_state;
}_joint_state;
typedef struct{
    arm_controller_data_s current_data;
    arm_controller_data_s target_data;
    float big_yaw_angle;
    float arm_height;

    uint8_t arm_to_airvalve;

    uint8_t auto_mode_state;
    uint8_t arm_auto_mode_selecting;

    _joint_state joint_state;
    uint8_t host_working_state; //上位机运行状态
}Arm_Data_s;

typedef struct{
    float Translation_x;
    float Translation_y;
    float Roatation_Vertical;
    float Roatation_Horizontal;
    uint8_t Rotation_mode;
    uint8_t Translation_mode;
    // 上四个为控制末端位姿，下两个为控制臂指向
    float Position_z;
    float Rotation_yaw;
    arm_mode_e contro_mode; // 臂臂控制模式
    uint8_t auto_mode; // 臂臂自动模式

    uint8_t convertArmControlByController; //兑矿模式下使用遥控器控制臂

    struct{
        uint8_t optimize_signal : 2; // 优化信号（修正臂的yaw偏向和混合roll角度）
        uint8_t switch_custom_controller_mode_call : 1; //切换自定义控制器模式
        uint8_t sucker_call : 2; // 吸盘动作命令
        uint8_t halt_force_call : 1;  // 强制停止命令
        uint8_t halt_temp_call : 1;  // 临时暂停命令
        uint8_t reset_init_flag : 1; // 软复位标志
        uint8_t reset_z_init_flag : 1; // z轴重置标定
        uint8_t z_slowly_down_call : 1; // z轴缓慢下降命令
        uint8_t reset_convertArmPose_call : 1; //兑矿模式下重置臂姿态
    }call;

    struct{
        uint8_t reset_encoder_offset_value  : 1;
        uint8_t selected_encoder_id         : 3;
        int8_t modify_encoder_offset_value[4];
        uint8_t selected_auto_mode_id       : 8;
        uint8_t auto_mode_record_pause_call : 1;
        uint8_t auto_mode_record_start_call : 1;
        
        uint8_t arm_pump_on_call : 1;
        uint8_t arm_pump_off_call : 1; 
        uint8_t valve_pump_on_call : 1; 
        uint8_t valve_pump_off_call : 1; 
        uint8_t jaw_loose_call : 1; 
        uint8_t jaw_tighten_call : 1; 
        uint8_t assorted_joint_enable_call : 1; 
        uint8_t assorted_joint_disable_call : 1; 
        uint8_t apply_delay_call : 1;
    }debug;
}Arm_Cmd_Data_s;

/* 气阀/气泵控制 */
typedef struct{
    uint8_t airpump_mode;
    uint8_t airvalve_mode;

    uint8_t arm_to_airvalve;
    uint8_t init_call;  // 相关参数重初始化请求（当机器人死亡复活后由cmd置位）
    uint8_t halt_force_call;  // 强制停止命令
    uint8_t halt_temp_call;  // 临时停止命令
    float sucker_offset_angle; //偏移角度
    uint8_t tmp_flag;

    uint8_t in_debug_call;
    uint8_t out_debug_call;
    uint8_t debug_call;
}Airpump_Cmd_Data_s;
typedef struct{
    int8_t debug_flag : 1; //debug模式

    int8_t add_ui : 1;
    int8_t delete_ui : 1;
    int8_t switch_type : 2;
    int8_t switch_color : 2;
    int8_t switch_selected_ui : 2;
    int8_t undo : 1;
    int8_t redo : 1;
    int8_t width : 2;
    int8_t param1 : 2;
    int8_t param2 : 2;
    int8_t param3 : 2;
    int8_t param4 : 2;
    int8_t copy : 1;
    int8_t paste : 1;
    int8_t cut : 1;
    int8_t pos_upORdown : 8;
    int8_t pos_leftORright : 8;
    int8_t reset_to_center : 1;
    int8_t save_flag : 1;
}UI_debug_param;

typedef struct
{ 
    uint8_t rc_connection_mode_t : 1;    //遥控器连接
    uint8_t vision_connection_mode_t : 1;    //图传连接
    uint8_t custom_contro_connection_mode_t : 1;    //自定义控制器连接
    uint8_t host_connection_mode_t : 1;    //上位机通讯连接
    uint8_t pump_arm_mode_t : 1;    //臂气泵
    uint8_t pump_valve_mode_t : 1;    //推杆气泵
    uint8_t arm_mode : 3;   //臂状态圈 0-无动作 1-进行中 2-暂停
    uint8_t valve_mode : 3; //推杆状态圈 0-无动作 1-进行中 2-暂停

    uint8_t Contro_mode; //控制模式
    uint8_t arm_selected_mode;
    uint8_t arm_selected_mode_state : 1; // 0表示当前模式(运行/暂停中)，1为选中(待确认后覆盖当前模式)
    uint8_t valve_selected_mode;
    uint8_t valve_selected_mode_state : 1; // 0表示当前模式(运行/暂停中)，1为选中(待确认后覆盖当前模式)
    
    uint8_t arm_temp_halt_selected : 1;
    uint8_t valve_halt_selected : 1;

    uint8_t relay_contr_state : 1; //臂继电器状态
    float gimbal_offset_angle;
    arm_controller_data_s arm_current_data;
    arm_controller_data_s arm_target_data;

    int16_t pump_air_arm;
    int16_t pump_air_valve;
    uint8_t selected_song;

    uint8_t UI_refresh_request;
    _joint_state joint_state;
    
    UI_debug_param debug;
}UI_data_t;



#pragma pack() // 关闭字节对齐,结束前面的#pragma pack(1)

/* 一些自定义的宏定义 */
// 臂臂自动模式
#define Arm_walk_state 1   // 臂臂行走态
#define Recycle_arm_in 2 // 臂臂回收到肚子内
#define Recycle_arm_out 3 // 臂臂从肚子内伸出
#define Arm_get_goldcube_right 4 // 臂臂取右侧金矿
#define Arm_fetch_cube_from_warehouse_up 5 // 臂臂从上矿仓取矿
#define Arm_fetch_cube_from_warehouse_down 6 // 臂臂从下矿仓取矿
#define Arm_get_silvercube_left 7// 取小资源岛左侧矿
#define Arm_get_silvercube_mid  8// 取小资源岛中间矿
#define Arm_get_silvercube_right 9// 取小资源岛右侧矿
#define Arm_fetch_gronded_cube 10 // 取地矿姿势
#define Arm_ConvertCube 11 // 兑矿模式
#define Arm_straighten 12 // 臂伸直
#define Arm_block_front 13 // 挡前装甲板
#define Arm_block_side 14 // 挡侧装甲板
#define Arm_block_back 15 // 挡后装甲板
#define Arm_place_cube_in_warehouse_up 16 // 放上矿仓
#define Arm_place_cube_in_warehouse_down 17 // 放下矿仓
// 臂臂控制模式
#define Arm_Control_with_Chassis 1// 控制底盘臂臂
#define Arm_Control_only_Arm     2// 仅控制臂臂
#define Arm_Control_by_Custom_controller 4// 自定义控制器
#define Arm_Control_by_vision   8  // 视觉控制
// 臂臂吸盘旋转
#define Arm_sucker_clockwise_rotation 1 //吸盘顺时针旋转
#define Arm_sucker_anticlockwise_rotation 2 //吸盘逆时针旋转
#define Arm_sucker_none_rotation    0   //吸盘不旋转
// 气泵开关命令
#define AIRPUMP_ARM_OPEN 0x01   //开臂臂的气泵
#define AIRPUMP_LINEAR_OPEN 0x02//开气推杆的气泵
#define AIRPUMP_ARM_CLOSE 0x04  //关臂臂的气泵
#define AIRPUMP_LINEAR_CLOSE 0x08 //关推杆的气泵
#define AIRPUMP_SWITCH(mode,object)         ((mode) & (object)) ? ((mode) &= ~(object)) : ((mode) |= (object))
//气推杆控制命令
#define AIRVALVE_LEFT_CUBE 0x04//取左侧矿模式
#define AIRVALVE_MIDDLE_CUBE 0x08//取中间矿模式
#define AIRVALVE_CLAW_FOREWARD 0x10 //夹爪前伸
#define AIRVALVE_CLAW_UP       0x20 //夹爪上抬
#define AIRVALVE_CLAW_LOOSE    0x40 //夹爪松开
#define AIRVALVE_CLAW_TIGHTEN  0x80 //夹爪锁紧
//吸盘电机控制命令
#define SUCKER_MOTOR_INIT 0x80//初始化吸盘电机
#define AIRPUMP_STOP 0x40//失能气泵
#define AIRVALVE_STOP 0x20//失能吸盘电机
//底盘特殊动作
#define CHASSIS_SLOPE_MOVE_L 0x01         // 向左斜向平移，直到手动停止或触发红外开关
#define CHASSIS_SLOPE_MOVE_R 0x02         // 向右斜向平移，直到手动停止或触发红外开关

/* 全局变量 */

//气泵状态 - 全局变量
extern uint8_t airpump_arm_state,airpump_linear_state;

#endif // !ROBOT_DEF_H