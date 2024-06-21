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
#define Arm_sucker_ramp_flag 0x40
#define Arm_sucker_ramp_doing 0x80
typedef struct {
    uint8_t rotationMode;
    float rotationUp_Down;
    float rotationLeft_Right;
    float YawRotation;
}ARM_ROTATE_PARAM;
typedef struct {
    float Front_Back;
    float Left_Right;
    uint8_t translateMode;
}ARM_TRANSLATE_PARAM;
typedef struct {
    HostInstance *host_instance; // 上位机接口
    uint8_t host_rec_flag;       // 上位机接收标志位
    uint8_t host_send_buf[33];   // 上位机发送缓冲区
    uint8_t sent_package_flag;     // 发送包标识
    uint8_t recv_package_flag;     // 接收包标识
    ARM_TRANSLATE_PARAM translate_param; //平移操作参数
    ARM_ROTATE_PARAM rotate_param;    //旋转操作参数
}HOST_ARM_COMM;

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