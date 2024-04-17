#ifndef ARM_H
#define ARM_H

/**
 * @brief 初始化机械臂,会被RobotInit()调用
 * 
 */
void ArmInit();

/**
 * @brief 机械臂任务
 * 
 */
void ArmTask();

void USARTRecCBK(void);
#endif // !ARM_H