/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "LKmotor.h"
#include "bsp_log.h"
#include "led.h"
#include "user_lib.h"

// TASK
#include "robot_cmd.h"
#include "buzzer.h"
#include "daemon.h"
#include "arm.h"
#include "airpump.h"
#include "chassis.h"
#include "gimbal.h"
#include "motor_task.h"
#include "test.h"


void TestTask(void *argument)
{
    UNUSED(argument);
    // DWT_GetDeltaT(&feed_dwt_cntt);

    while (1) {
        // static GPIO_PinState key1_state;
        // key1_state = HAL_GPIO_ReadPin(Z_limit_detect_GPIO_Port,Z_limit_detect_Pin);

        // DM_board_LEDSet(0x33ffff);
        // osDelay(500);
        // DM_board_LEDSet(0x000000);
        // osDelay(500);
        // RobotTask();
        // feed_dtt = DWT_GetDeltaT(&feed_dwt_cntt);

        osDelay(1);
    }
}

void _cmdTASK(void *argument)
{
    UNUSED(argument);
    while (1) {
        RobotCMDTask();
        osDelay(1);
    }
}

void _airpumpTASK(void *argument)
{
    UNUSED(argument);
    while (1) {
        AIRPUMPTask();
        osDelay(1);
    }
}

void _gimbalTASK(void *argument)
{
    UNUSED(argument);
    while (1) {
        GIMBALTask();
        LKMotorControl();
        osDelay(2);
    }
}

void _chassisTASK(void *argument)
{
    UNUSED(argument);
    while (1) {
        ChassisTask();

        osDelay(1);
    }
}

void _armTASK(void *argument)
{
    UNUSED(argument);
    while (1) {
        ArmTask();
        osDelay(1);
    }
}

void _motorTASK(void *argument)
{
    UNUSED(argument);
    while (1) {
        MotorControlTask();
        osDelay(1);
    }
}

void _DaemonTask(void *argument)
{
    UNUSED(argument);
    while (1) {
        DaemonTask();
    }
}

void _BuzzerTask(void *argument)
{
    UNUSED(argument);
    while (1) {
        BuzzerTask(argument);
    }
}