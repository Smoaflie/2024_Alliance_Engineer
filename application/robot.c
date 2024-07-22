/**
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-26 21:51:44
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\application\robot.c
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "buzzer.h"

#define ROBOT_DEF_PARAM_WARNING
// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#include "test.h"
#include "arm.h"

#include "dji_motor.h"
#include "DRmotor.h"
#include "gimbal.h"
#include "airpump.h"
#include "UI.h"
#include "flashtask.h"
#include "iwdg.h"

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();

    BSPInit();

#ifndef ROBOT_TEST
    DM_board_LEDSet(0x000000);
    RobotCMDInit();

    ChassisInit();
    ArmInit();
    GIMBALInit();
    AIRPUMPInit();
#endif

#ifdef ROBOT_TEST
    selfTestInit();
#endif
    // buzzer_one_note(0xf0, 0.5);

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    // RobotCMDTask();
    // ArmTask();
    // ChassisTask();
    // selfTestTask();
}