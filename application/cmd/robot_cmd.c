// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "bsp_can.h"
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回

static Robot_Status_e robot_state; // 机器人整体工作状态

void RobotCMDInit()
{
    rc_data          = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

static void RemoteControlSet()
{
    
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{   

}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
// static void EmergencyHandler()
// {
//     // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
//     if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
//     {
//         robot_state                   = ROBOT_STOP;
//         LOGERROR("[CMD] emergency stop!");
//     }
//     // 遥控器右侧开关为[上],恢复正常运行
//     if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
//         robot_state               = ROBOT_READY;
//         LOGINFO("[CMD] reinstate, robot ready");
//     }
// }

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
       //遥控器左下右中，切换为电脑模式
    //遥控器其余状态为遥控器模式
    if (switch_is_down(rc_data[TEMP].rc.switch_left)&&switch_is_mid(rc_data[TEMP].rc.switch_right)){
        MouseKeySet();
   }
   else {
    RemoteControlSet();
   }
    
}
