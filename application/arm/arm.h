#ifndef ARM_H
#define ARM_H

#define Z_motor_init_clt 0x01
#define Big_Yaw_motor_init_clt 0x02
#define Z_motor_pub_reset 0x04
#define Big_Yaw_motor_pub_reset 0x08

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