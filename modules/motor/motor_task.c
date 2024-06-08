#include "motor_task.h"
#include "LKmotor.h"
#include "DRmotor.h"
#include "HT04.h"
#include "dji_motor.h"
#include "dji_motor.h"
#include "step_motor.h"
#include "servo_motor.h"

void MotorControlTask()
{
    static uint8_t cnt = 0; //设定不同电机的任务频率
    cnt++;
    if(cnt%2==0)
    {
        LKMotorControl();
        DRMotorControl();  
    }
    DJIMotorControl();
     
    

    // legacy support
    // 由于ht04电机的反馈方式为接收到一帧消息后立刻回传,以此方式连续发送可能导致总线拥塞
    // 为了保证高频率控制,HTMotor中提供了以任务方式启动控制的接口,可通过宏定义切换
    // HTMotorControl();
    // 将所有的CAN设备集中在一处发送,最高反馈频率仅能达到500Hz,为了更好的控制效果,应使用新的HTMotorControlInit()接口

    ServeoMotorControl();

    // StepMotorControl();
}
