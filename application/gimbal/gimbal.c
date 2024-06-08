#include "gimbal_interface.h"

void GIMBALInit()
{
    GimbalInit_Motor();
    GimbalInit_Communication();
    GimbalInit_IO();
    GimbalInit_Param();
}

/* 机器人机械臂控制核心任务 */
void GIMBALTask()
{
    GimbalSubMessage();
    GimbalParamPretreatment();
    GimbalContro();
    GimbalPubMessage();

    GimbalDebugInterface();
}