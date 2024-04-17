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
#include "servo_motor.h"

static ServoInstance *leftservomoto;

void selfTestInit()
{
    //初始化参数
    Servo_Init_Config_s config={
        //舵机安装选择的定时器及通道
        //C板有常用的7路PWM输出:TIM1-1,2,3,4 TIM8-1,2,3
        .htim=&htim2,
        .Channel=TIM_CHANNEL_1,
        //舵机的初始化模式和类型
        .Servo_Angle_Type=Start_mode,
        .Servo_type=Servo180,
    };
    // 设置好参数后进行初始化并保留返回的指针
    leftservomoto = ServoInit(&config);
}

int32_t angle = 0;
/* 机器人机械臂控制核心任务 */
void selfTestTask()
{
    Servo_Motor_FreeAngle_Set(leftservomoto,angle);
    DM_board_LEDSet(0x000000);
    ServeoMotorControl();
    osDelay(1000);
    DM_board_LEDSet(0xff0000);
    osDelay(1000);
}