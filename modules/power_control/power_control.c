#include "power_control.h"
#include "message_center.h"
#include <stdlib.h>
#include <math.h>
#include "user_lib.h"

/*功率控制计算部分*/
/**
 * @brief 将电调力矩电流控制值转换为当前力矩
 * 
 * @param coefficient 系数结构体
 * @param motor_output 电调返回的电流值
 * 
 * @return present_torque 返回电机当前的力矩值
 */
float CmdtoTorqueCali(Coefficient_t *coefficient,float motor_output)
{
    float present_torque,torque_current;
    torque_current = coefficient->cmd_to_torque * motor_output;
    present_torque = coefficient-> torque * torque_current;
    return present_torque;
}
/**
 * @brief 将目标力矩转换为电调力矩电流控制值
 * 
 * @param coefficient 系数结构体
 * @param target_torque 目标力矩值
 * 
 * @return motor_input 返回电调力矩电流控制值，用以给电机输入
 */
float TorquetoCmdCali(Coefficient_t *coefficient,Physical_Quantity_t *physical_quantity)
{
    float motor_input,torque_current;
    torque_current = physical_quantity->target_torque / coefficient-> torque;
    motor_input = torque_current / coefficient->cmd_to_torque;
    return motor_input;
}
/**
 * @brief 使用转速与当前力矩求解电机当前的总功率
 * 
 * @param physical_quantity 求解中用到的物理量
 * 
 * @return physical_quantity 返回计算完毕的物理量指针 
 */
float TotalPowerCali(Physical_Quantity_t *physical_quantity,Coefficient_t *coefficient,float motor_output,float motor_speed)
{
    physical_quantity->current_torque = CmdtoTorqueCali(coefficient,motor_output);
    //机械功率Pm=Tω
    physical_quantity->current_machine_power = physical_quantity->current_torque * motor_speed;
    //Pin = Pm + k1 * ω^2+ k2 * τ^2 + a
    physical_quantity->current_total_power = physical_quantity->current_machine_power + (coefficient->k1 * pow(motor_speed,2)) + (coefficient->k2 * pow(physical_quantity->current_torque,2)) + coefficient->constant;
    return physical_quantity->current_total_power;
}
/**
 * @brief 使用功率求解目标力矩值
 * 
 * @param physical_quantity 求解中用到的物理量
 * @param coefficient 需要用到的参数
 * 
 * @return physical_quantity 返回计算完毕的物理量指针
 */
void TargetTorqueCali(Physical_Quantity_t *physical_quantity,Coefficient_t *coefficient,float motor_speed,float output)
{
    int a,b,c;
    //a=k1
    a = coefficient->k1;
    //b=ω(RPM)
    b = motor_speed;
    //c=k2*ω^2+a-Pmax
    c = coefficient->k2 * pow(motor_speed,2) + coefficient->constant - physical_quantity->max_power;
    if(output > 0)
    {
        /*physical_quantity->target_torque=(-b + 
        
        
        
        
        
        f((b * b) - (4.0f * a * c))) / (2.0f * a);*/
    }
    else if(output < 0)
    {
        physical_quantity->target_torque=(-b - sqrtf((b * b) - (4.0f * a * c))) / (2.0f * a);
    }
}
void PowerDistribution(Physical_Quantity_t *physical_quantity,Coefficient_t *coefficient,float motor_output,float motor_speed)       
{
    float Pcmd = TotalPowerCali(physical_quantity,coefficient,motor_output,motor_speed);
    coefficient->power_distribution = physical_quantity->max_power / Pcmd;
    if(coefficient->power_distribution > 1)
    {
        physical_quantity->distributed_power = Pcmd;
    }
    else
    {
        physical_quantity->distributed_power = coefficient->power_distribution * Pcmd;
    }
}

/*初始化函数部分*/

/**
 * @brief 对功率计算中需要用到的系数进行初始化设定
 * 
 * @param coefficient 需要被初始化的参数结构体
 * 
 * @return coefficient 初始化完成的参数结构体
 */
void* CoefficientInit(Coefficient_t *coefficient)
{
    float reduction_ratio;
    //根据不同电机的减速比进行修改，默认为M3508手册给出的减速比
    if(coefficient->reduction_ratio != 0)
    {
        reduction_ratio = coefficient->reduction_ratio;
    }
    else
    {
        reduction_ratio = 0.0520746310219994f;
    }
    //力矩电流与电调力矩电流控制值之间的转化系数，数值由20/16384得到
    coefficient->cmd_to_torque = 0.001220703125f;
    //力矩电流常数，为转矩常数与电机减速比相乘得到的转子力矩电流常数
    coefficient->torque = 0.3f * reduction_ratio;
    //功率模型中力矩二次方的系数
    coefficient->k1 = 1.23e-07;
    //功率模型中转速二次方的系数
    coefficient->k2 = 1.453e-07;
    //功率模型中的常量
    coefficient->constant=4.081f;
    return coefficient;
}

/**
 * @brief 将目标力矩转换为电调力矩电流控制值
 * 
 * @param coefficient 系数结构体
 * @param target_torque 目标力矩值
 * 
 * @return motor_input 返回电调力矩电流控制值，用以给电机输入
 */
void PhysicalQuantityInit(Physical_Quantity_t *physical_quantity)
{
    physical_quantity->max_power = MAX_POWER;
}

/*对外可被调用的功率控制接口*/
/**
 * @brief 功率控制初始化
 * 
 * @param power_control_instance 功率控制实例
 */
PowerControlInstance *PowerControlInit(void)
{
    PowerControlInstance *instance = (PowerControlInstance *)zmalloc(sizeof(PowerControlInstance));
    CoefficientInit(&instance->coefficient);
    PhysicalQuantityInit(&instance->physical_quantity);
    return instance;
}
/**
 * @brief 功率控制主要函数
 * 
 * @param power_control_instance 功率控制实例
 * @param motor_output 电机返回的电调力矩电流控制值
 * @param motor_speed 电机返回的电机当前转速
 */
float PowerControl(PowerControlInstance *power_control_instance,float motor_output,float motor_speed)
{
    float motor_input;
    PowerDistribution(&power_control_instance->physical_quantity,&power_control_instance->coefficient,motor_output,motor_speed);
    TargetTorqueCali(&power_control_instance->physical_quantity,&power_control_instance->coefficient,motor_speed,motor_output);
    motor_input = TorquetoCmdCali(&power_control_instance->coefficient,&power_control_instance->physical_quantity);
    return motor_input;
}