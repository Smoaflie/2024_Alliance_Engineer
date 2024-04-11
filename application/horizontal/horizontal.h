#ifndef B4C73E9C_7A14_4393_A411_91644A2D2371
#define B4C73E9C_7A14_4393_A411_91644A2D2371
#ifndef HORIZONTAL_H_
#define HORIZONTAL_H_

/**
 * @brief 升降应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void Horizontal_Init();

/**
 * @brief 升降应用任务,放入实时系统以一定频率运行
 * 
 */
void Horizontal_Task();

#endif
#endif /* B4C73E9C_7A14_4393_A411_91644A2D2371 */
