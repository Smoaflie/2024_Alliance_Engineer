#ifndef YSA1_H
#define YSA1_H

#include "stdint.h"
#include "bsp_usart.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define YS_MOTOR_MX_CNT      4 // 最多允许4个YS电机使用多电机指令,挂载在一条总线上
#define RAD_TO_ANGLE         57.29578f
#define YSmotor_rx_data_size 78
#define YSmotor_tx_len       34
typedef struct // 9025
{
    uint8_t MotorID;
    uint8_t Mode;     // 0停转  5开环  10闭环
    int16_t Torque;   // Max 128
    int16_t Speed;    // Max 256
    int32_t Position; // Max 823549  	电机位置 = Position * 9.1（减速比）  rad
    int32_t Roll_Position;
    uint16_t K_P; // Max 16  			实际电机位置刚度 = K_P / 1024
    uint16_t K_W; // Max 32  		实际电机速度刚度 = K_W / 2048
    uint8_t Temp; // 当前温度
    int16_t Current_Torque;
    float Current_Torque_Nm;
    int16_t Current_Speed;
    int16_t Current_Acc;
    int32_t Current_Pos;
    float Current_Pos_rad;
    float Current_Pos_angle;
    float Standard_Pos;
} YSMotor_Measure_t;

typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    USART_Init_Config_s usart_init_config;

} YSMotor_Init_Config_s;

typedef struct
{
    YSMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;   // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入
    float *current_feedforward_ptr; // 电流前馈指针
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    DaemonInstance *daemon;
    USARTInstance *motor_usart_ins;
    YSMotor_Init_Config_s YSmotor_usart;
} YSMotorInstance;

/**
 * @brief 初始化YS电机
 *
 * @param config 电机配置
 * @return YSMotorInstance* 返回实例指针
 */
YSMotorInstance *YSMotorInit(YSMotor_Init_Config_s *config, UART_HandleTypeDef *ysmotor_usart_handle);

/**
 * @brief 解算宇树电机接收函数
 *
 * @param config 电机配置
 * @return YSMotorInstance* 返回实例指针
 */
static void YSMotor_Data_Decode();

/**
 * @brief 设置参考值
 * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
 *
 * @param motor 要设置的电机
 * @param ref 设定值
 */
void YSMotorSetRef(YSMotorInstance *motor, float ref);

/**
 * @brief 为所有YS电机计算pid/反转/模式控制,并通过bspcan发送电流值(发送CAN报文)
 *
 */
void YSMotorControl();

/**
 * @brief 停止YS电机,之后电机不会响应任何指令
 *
 * @param motor
 */
void YSMotorStop(YSMotorInstance *motor);

/**
 * @brief 启动YS电机
 *
 * @param motor
 */
void YSMotorEnable(YSMotorInstance *motor);

/**
 * @brief 为所有YS电机计算pid/反转/模式控制,并通过bspusart发送电流值(发送USART报文)
 *
 */
void YSMotorControl();

#endif // YSA1_H
