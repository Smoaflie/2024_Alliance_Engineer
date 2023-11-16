#ifndef POWER_CONTROl
#define POWER_CONTROl

#define MAX_POWER 60

typedef struct{
    //力矩电流与电调力矩电流控制值之间的转化系数
    float cmd_to_torque;
    //力矩电流常数
    float torque;
    //功率模型中力矩二次方的系数
    float k1;
    //功率模型中转速二次方的系数
    float k2;
    //功率模型中的常量
    float constant;
    //功率分配常数
    float power_distribution;
    //减速比
    float reduction_ratio;
}Coefficient_t;

typedef struct{
    //目标力矩值
    float target_torque;
    //当前力矩值
    float current_torque;
    //当前机械功率
    float current_machine_power;
    //当前总功率
    float current_total_power;
    //最大机械功率
    float max_power;
    //经过重分配后的功率
    float distributed_power;
}Physical_Quantity_t;

typedef struct{
    //参数结构体
    Coefficient_t coefficient;
    //物理量结构体
    Physical_Quantity_t physical_quantity;
}PowerControlInstance;

extern PowerControlInstance *PowerControlInit(void);
extern float PowerControl(PowerControlInstance *power_control_instance,float motor_output,float motor_speed);

#endif