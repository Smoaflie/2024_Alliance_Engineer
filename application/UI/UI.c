// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "LKmotor.h"
#include "servo_motor.h"
#include "dji_motor.h"
#include "led.h"
#include "referee_UI.h"
// bsp
#include "encoder.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "DRmotor.h"

void selfUIInit()
{
    UITaskInit();
}

// float LKangle = 0,gimbalangle=0;
/* 机器人机械臂控制核心任务 */
void selfUITask()
{
    UITask();
    // UICharRefresh();
}