#include "arm_interface.h"
// 上位机解析回调函数
void HOST_RECV_CALLBACK();

void ArmInit()
{
    ArmInit_Encoder();
    ArmInit_Motor();
    ArmInit_Communication();
    ArmInit_IO();
    ArmInit_Param();
}

/* 机器人机械臂控制核心任务 */
void ArmTask()
{
    //接收控制信息
    ArmSubMessage();
    //臂各种参数的预处理
    ArmParamPretreatment();
    //臂控制
    ArmControInterface();
    //向上位机发送数据包
    ArmCommunicateHOST();
    //发布臂任务数据
    ArmPubMessage();

    //调试用接口
    ArmDebugInterface();
}
