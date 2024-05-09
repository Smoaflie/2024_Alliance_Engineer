#ifndef AIRPUMP_H // 气阀/气泵/气推杆三任务合一
#define AIRPUMP_H

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
