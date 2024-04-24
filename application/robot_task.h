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
#include "LKmotor.h"
#include "DRmotor.h"
#include "bsp_log.h"
#include "led.h"
#include "buzzer.h"
#include "ins_task.h"
#include "chassis.h"
#include "user_lib.h"
#include "gimbal.h"

#include "robot_cmd.h"

uint32_t feed_dwt_cntt,feed_dtt;

void TestTask(void *argument)
{
    UNUSED(argument);
    DWT_GetDeltaT(&feed_dwt_cntt);

    while (1) {
        static GPIO_PinState key1_state;
        key1_state = HAL_GPIO_ReadPin(Z_limit_detect_GPIO_Port,Z_limit_detect_Pin);
        

        RobotTask();
        feed_dtt = DWT_GetDeltaT(&feed_dwt_cntt);


        MotorControlTask();
        osDelay(1); 
    }
}

void motorControlTask(void *argument){
    UNUSED(argument);
    while(1)
    {
        
        // GIMBALTask();
        // LKMotorControl();
        osDelay(2);
    }
}