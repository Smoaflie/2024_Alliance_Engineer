// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "LKmotor.h"
#include "DRmotor.h"
#include "led.h"
// bsp
#include "encoder.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

void selfTestInit()
{

}

/* 机器人机械臂控制核心任务 */
void selfTestTask()
{
    DM_board_LEDSet(0x000000);
    osDelay(1000);
    DM_board_LEDSet(0xff0000);
    osDelay(1000);
}