#ifndef DRmotor_H
#define DRmotor_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DR_MOTOR_MX_CNT 4 // 最多允许4个DR电机使用多电机指令,挂载在一条总线上

#define I_MIN -2000
#define I_MAX 2000
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define REDUCTION_RATIO_DRIVEN 1
#define CURRENT_TORQUE_COEF_DR 0.003645f // 电流设定值转换成扭矩的系数,算出来的设定值除以这个系数就是扭矩值

typedef struct
{
    Motor_Type_e motor_type;        // 电机类型
    
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例

    Motor_Working_Type_e stop_flag; // 启停标志
    Motor_Error_Detection_s motor_error_detection; // 异常检测
    Motor_Measure_s measure;            // 电机测量值
   
    DaemonInstance *daemon;
    uint32_t feed_cnt;
    float dt;

    uint8_t reboot_call;    //复位请求，置位时停止电机发送
    uint8_t reboot_delay_cnt;
} DRMotorInstance;

/**
 * @brief 初始化DR电机
 *
 * @param config 电机配置
 * @return DRMotorInstance* 返回实例指针
 */
DRMotorInstance *DRMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置参考值
 * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
 *
 * @param motor 要设置的电机
 * @param ref 设定值
 */
void DRMotorSetRef(DRMotorInstance *motor, float ref);

/**
 * @brief 为所有DR电机计算pid/反转/模式控制,并通过bspcan发送电流值(发送CAN报文)
 *
 */
void DRMotorControl();

/**
 * @brief 停止DR电机,之后电机不会响应任何指令
 *
 * @param motor
 */
void DRMotorStop(DRMotorInstance *motor);

/**
 * @brief 启动DR电机
 *
 * @param motor
 */
void DRMotorEnable(DRMotorInstance *motor);

uint8_t DRMotorIsOnline(DRMotorInstance *motor);

void DRMotorReset(DRMotorInstance* motor);

#endif // DRmotor_H
