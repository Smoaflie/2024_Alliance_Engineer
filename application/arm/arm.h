#ifndef ARM_H
#define ARM_H

#define Z_motor_init_clt 0x01
#define Z_motor_pub_reset 0x04

/* 臂斜坡标志 */
#define Arm_joint_ramp_flag 0x01
#define Arm_joint_ramp_doing 0x02
#define Arm_height_ramp_flag 0x04
#define Arm_height_ramp_doing 0x08
#define Arm_target_ramp_flag 0x10
#define Arm_target_ramp_doing 0x20
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