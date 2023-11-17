/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_init.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"

#include "bsp_log.h"
//! 任务直接在cubeMX中配置,不再使用这种方式
#if 0
osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
        osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
        insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
        // 后续修改为读取传感器数据准备好的中断处理,

        osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
        motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

        osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
        daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

        osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
        robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

        osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
        uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

        HTMotorControlInit(); // 没有注册HT电机则不会执行
}


__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;) {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;) {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;) {
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
        osDelay(5);
    }
}

__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    LOGINFO("[freeRTOS] UI Task Start");
    MyUIInit();
    LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
    for (;;) {
        // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
        UITask();
        osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
    }
}
#endif
#include "led.h"
#include "buzzer.h"
#include "ins_task.h"
#include "chassis.h"
#include "user_lib.h"

__attribute__((noreturn)) void StartINSTASK(void *argument)
{
    UNUSED(argument);
    static uint32_t ins_time;
    static float ins_dt;
    LOGINFO("[freeRTOS] INS Task Start");
    while (1) {
        INS_Task();
        ins_dt = 1000 * DWT_GetDeltaT(&ins_time);
        if (ins_dt > 1.2f)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]ms", &ins_dt);
        osDelay(1);
    }
}

extern void MotorControlTask();

__attribute__((noreturn)) void TestTask(void *argument)
{
    UNUSED(argument);
    osDelay(500);
    BuzzerPlay(StartUP_sound);

    while (1) {
        C_board_LEDSet(0x33ffff);
        /*osDelay(500);
        C_board_LEDSet(0xd633ff);
        osDelay(500);*/
        MotorControlTask();

        osDelay(1);
    }
}
