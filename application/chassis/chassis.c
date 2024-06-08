#include "chassis_interface.h"

void ChassisInit()
{
    ChassisInit_Motor();
    ChassisInit_Communication();
    ChassisInit_IO();
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    ChassisSubMessage();

    // 底盘模式选择&设定移动速度
    ChassisModeSelect();
    // 底盘特殊功能处理
    // SpecialFuncApply();
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 设置电机速度
    SetChassisRef();

}
