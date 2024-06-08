/**
 * @file motor_def.h
 * @author neozng
 * @brief  电机通用的数据结构定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "controller.h"
#include "stdint.h"
#include "bsp_can.h"

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum
{
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP = 0b0010,
    ANGLE_LOOP = 0b0100,
    TORQUE_LOOP = 0b1000,

    // only for checking
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THREE_LOOP = 0b0111,
} Closeloop_Type_e;

typedef enum
{
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01,
    SPEED_FEEDFORWARD = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED,
} Feedback_Source_e;

/* 电机正反转标志 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

/* 反馈量正反标志 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e;
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENABLED = 1,
} Motor_Working_Type_e;

/* 异常检测 */
typedef enum
{
    MOTOR_ERROR_DETECTION_NONE = 0b000,
    MOTOR_ERROR_DETECTION_CRASH = 0b001,
    MOTOR_ERROR_DETECTION_STUCK = 0b010,
    MOTOR_ERROR_PROTECTED = 0b100,
} Motor_Error_Detection_Type_e;
/* 异常代码 */
typedef enum
{
    MOTOR_ERROR_NONE = 0b00,
    MOTOR_ERROR_CRASH = 0b01,
    MOTOR_ERROR_STUCK = 0b10,
} Motor_ErrorCode_Type_e;
typedef struct
{
    Motor_Error_Detection_Type_e error_detection_flag; // 是否启用异常检测
    
    float *speed;   // 当前速度
    float *last_speed;      // 上次速度
    float *current; // 当前电流

    float *stuck_current_ptr;   // 堵转电流大小
    float stuck_speed;   // 堵转速度大小
    float max_current;   // (正常情况下)最大输出电流值

    uint8_t stuck_cnt;
    uint16_t crash_detective_sensitivity; // 碰撞检测灵敏度

    float tmp_variable; // 临时变量，给回调函数存储需要的数据 //todo:可以优化？

    void (*error_callback)(void* motor); // 异常回调函数

    Motor_ErrorCode_Type_e ErrorCode;  // 异常代码
} Motor_Error_Detection_s;
/* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
typedef struct
{
    Closeloop_Type_e outer_loop_type;              // 最外层的闭环,未设置时默认为最高级的闭环
    Closeloop_Type_e close_loop_type;              // 使用几个闭环(串级)
    Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
    Feedback_Source_e angle_feedback_source;       // 角度反馈类型
    Feedback_Source_e speed_feedback_source;       // 速度反馈类型
    Feedfoward_Type_e feedforward_flag;            // 前馈标志
} Motor_Control_Setting_s;

/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
// 后续增加前馈数据指针
typedef struct
{
    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    PIDInstance torque_PID;

    float output_current;     // 输出电流
    float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
} Motor_Controller_s;

typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 编码器值
    float angle_single_round; // 单圈角度
    float last_speed_aps;     // 上一次读取的角速度,单位为:度/秒
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
    
} Motor_Measure_s;


/* 电机类型枚举 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
    LK_MS5005,
    DR_PDA04,
    DR_B0X,
} Motor_Type_e;

typedef struct
{
    Motor_Type_e motor_type;        // 电机类型
    
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例

    Motor_Working_Type_e stop_flag; // 启停标志
    Motor_Error_Detection_s motor_error_detection; // 异常检测
    Motor_Measure_s measure;            // 电机测量值
} Motor_Base_s;
/* 电机控制方式枚举 */
typedef enum
{
    TORQUE_LOOP_CONTRO = 0, //电流/扭矩开环控制
    ANGLE_LOOP_CONTRO = 1,  //位置闭环控制(由电机支持)
} Motor_Contro_Type_e;
/**
 * @brief 电机控制器初始化结构体,包括三环PID的配置以及两个反馈数据来源指针
 *        如果不需要某个控制环,可以不设置对应的pid config
 *        需要其他数据来源进行反馈闭环,不仅要设置这里的指针还需要在Motor_Control_Setting_s启用其他数据来源标志
 */
typedef struct
{
    float *other_angle_feedback_ptr; // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr; // 速度反馈数据指针,单位为angle per sec

    float *speed_feedforward_ptr;   // 速度前馈数据指针
    float *current_feedforward_ptr; // 电流前馈数据指针

    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
    PID_Init_Config_s torque_PID;
} Motor_Controller_Init_s;

/* 用于初始化CAN电机的结构体,各类电机通用 */
typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;
    Motor_Contro_Type_e motor_contro_type;  //控制类型
    Motor_Error_Detection_s motor_error_detection_config;
} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H
