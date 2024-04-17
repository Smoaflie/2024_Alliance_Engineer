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


static DRMotorInstance *test_motor; // 电机实例
static EncoderInstance_s *assorted_yaw_encoder;
void selfTestInit()
{
    Encoder_Init_Config_s encoder_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
        }};
    encoder_config.can_init_config.rx_id = 0x1df;
    assorted_yaw_encoder                      = EncoderInit(&encoder_config);
}

/* 机器人机械臂控制核心任务 */
void selfTestTask()
{
    DRMotorEnable(test_motor);

    if(!HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
    {
        C_board_LEDSet(0xd633ff);
        DRMotorSetRef(test_motor,0);
    }
    else
    {
        C_board_LEDSet(0x33ffff);
        DRMotorStop(test_motor);
    }
    DRMotorControl();
}