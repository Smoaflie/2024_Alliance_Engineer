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

uint8_t key[3];
__attribute__((noreturn)) void TestTask(void *argument)
{
    UNUSED(argument);
    osDelay(500);
    BuzzerPlay(StartUP_sound);
    
    while (1) {
        RobotTask();
        MotorControlTask();
        key[0] = HAL_GPIO_ReadPin(key_1_GPIO_Port,key_1_Pin);
        key[1] = HAL_GPIO_ReadPin(key_2_GPIO_Port,key_2_Pin);
        osDelay(1);
    }
}
