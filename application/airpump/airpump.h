#ifndef AIRPUMP_H // 气阀/气泵/气推杆三任务合一
#define AIRPUMP_H

//气泵开关命令
#define AIRPUMP_ARM_OPEN 0x01   //开臂臂的气泵
#define AIRPUMP_LINEAR_OPEN 0x02//开气推杆的气泵
//气推杆取矿命令
#define AIRVALVE_LEFT_CUBE 0x04//取左侧矿模式
#define AIRVALVE_MIDDLE_CUBE 0x08//取中间矿模式
//吸盘电机控制命令
#define SUCKER_MOTOR_INIT 0x80//初始化吸盘电机
#define AIRPUMP_SUCKER_ALL_STOP 0x40//失能吸盘电机和气泵

//气推杆状态命令字
#define AIRVALVE_LEFT_CUBE_DOING 0x01   //取左侧矿-进行中
#define AIRVALVE_MIDDLE_CUBE_DOING 0x02 //取中间矿-进行中
#define AIRVALVE_LEFT_CUBE_DONE 0x10    //取左侧矿-已完成
#define AIRVALVE_MIDDLE_CUBE_DONE 0x20  //取中间矿-已完成
#define AIRVALVE_SEND_CODER 0x80    //发送命令
/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 *
 */
void AIRPUMPInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 *
 */
void AIRPUMPTask();

#endif // AIRPUMP_H
