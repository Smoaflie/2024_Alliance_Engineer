#include "robot_cmd_interface.h"

void RobotCMDInit()
{
    RobotCMDInit_VisionLine();
    RobotCMDInit_RC();
    RobotCMDInit_Param();
    RobotCMDInit_IO();
    RobotCMDInit_Communication();
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    RobotActive();
    RobotCMDSubMessage();

    RobotCMDParamPretreatment();
    
    RobotCMDGenerateCommand();
    
    RobotCMDDebugInterface();
    RobotCMDPubMessage();

    RobotCMDDebugInterface();
}
