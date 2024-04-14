// app
#include "robot_def.h"
#include "robot_cmd.h"
//#include "second.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
//#include "forward.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

//以后这块搞点宏
#define PITCH_RUN_MODE 1
#define ROLL_RUN_MODE 2
#define STOP_MODE 3
#define Rotation_Ratio 1.5



/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回

static Publisher_t *first_stretch_cmd_pub;            // 一级控制消息发布者
static Subscriber_t *first_stretch_feed_sub;          // 一级反馈信息订阅者
static First_Stretch_Ctrl_Cmd_s first_stretch_cmd_send;      // 传递给一级的控制信息
static First_Stretch_Upload_Data_s first_stretch_fetch_data; // 从一级获取的反馈信息

static Publisher_t *second_stretch_cmd_pub;            // 二级控制消息发布者
static Subscriber_t *second_stretch_feed_sub;          // 二级反馈信息订阅者
static Second_Stretch_Ctrl_Cmd_s second_stretch_cmd_send;      // 传递给二级的控制信息
static Second_Stretch_Upload_Data_s second_stretch_fetch_data; // 从二级获取的反馈信息

static Publisher_t *lift_cmd_pub;            // 升降控制消息发布者
static Subscriber_t *lift_feed_sub;          // 升降反馈信息订阅者
static Lift_Ctrl_Cmd_s lift_cmd_send;      // 传递给升降的控制信息
static Lift_Upload_Data_s lift_fetch_data; // 从升降获取的反馈信息

static Publisher_t *horizontal_cmd_pub;            // 横移控制消息发布者
static Subscriber_t *horizontal_feed_sub;          // 横移反馈信息订阅者
static Horizontal_Ctrl_Cmd_s horizontal_cmd_send;      // 传递给横移的控制信息
static Horizontal_Upload_Data_s horizontal_fetch_data; // 从横移获取的反馈信息

static Publisher_t *forward_cmd_pub;            // 前端控制消息发布者
static Subscriber_t *forward_feed_sub;          // 前端反馈信息订阅者
static Forward_Ctrl_Cmd_s forward_cmd_send;      // 传递给前端的控制信息
static Forward_Upload_Data_s forward_fetch_data; // 从前端获取的反馈信息

PC_Mode_t PC_Mode;

static Robot_Status_e robot_state; // 机器人整体工作状态

void RobotCMDInit()
{
    rc_data          = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    
    lift_cmd_pub = PubRegister("lift_cmd", sizeof(Lift_Ctrl_Cmd_s));
    lift_feed_sub = SubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
    first_stretch_cmd_pub  = PubRegister("first_stretch_cmd", sizeof(First_Stretch_Ctrl_Cmd_s));  
     first_stretch_feed_sub = SubRegister("first_stretch_feed", sizeof(First_Stretch_Upload_Data_s));
    second_stretch_cmd_pub  = PubRegister("second_stretch_cmd", sizeof(Second_Stretch_Ctrl_Cmd_s));
     second_stretch_feed_sub = SubRegister("second_stretch_feed", sizeof(Second_Stretch_Upload_Data_s));
     chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
     chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    // forward_cmd_pub  = PubRegister("forward_cmd", sizeof(Forward_Ctrl_Cmd_s));
    // forward_feed_sub = SubRegister("forward_feed", sizeof(Forward_Upload_Data_s));
    // horizontal_cmd_pub = PubRegister("horizontal_cmd", sizeof(Horizontal_Ctrl_Cmd_s));
    // horizontal_feed_sub = SubRegister("horizontal_feed", sizeof(Horizontal_Upload_Data_s));


    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

void mode_record();
void control_forward();



/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
//以下变量为动的最后一刻存储的角度数据
float last_first_right_angle,last_first_left_angle;
float last_second_right_angle,last_second_left_angle;
float last_lift_right_angle,last_lift_left_angle;
float last_horizontal_angle;

//
int is_range(int a){
    if ((a> 5)||(a< -5)){
        return 1;
    }
    else {
        return 0;
    }
}
static void RemoteControlSet()
{
    
    if ((switch_is_up(rc_data[TEMP].rc.switch_right))&&switch_is_up(rc_data[TEMP].rc.switch_left)) {
        if(is_range(rc_data[TEMP].rc.rocker_l1)||is_range(rc_data[TEMP].rc.rocker_l_)||is_range(rc_data[TEMP].rc.rocker_r_)){
            chassis_cmd_send.vx = rc_data[TEMP].rc.rocker_l_*10; // 系数待测
            chassis_cmd_send.vy = rc_data[TEMP].rc.rocker_l1*10;
            chassis_cmd_send.wz = rc_data[TEMP].rc.rocker_r_*5;
        }

        if(is_range(rc_data[TEMP].rc.rocker_r1))
        {
            lift_cmd_send.lift_mode = LIFT;
            lift_cmd_send.left = rc_data[TEMP].rc.rocker_r1 + lift_fetch_data.new_left_encoder;
            lift_cmd_send.right = -rc_data[TEMP].rc.rocker_r1+ lift_fetch_data.new_right_encoder;
            last_lift_right_angle = lift_fetch_data.new_right_encoder; 
            last_lift_left_angle = lift_fetch_data.new_left_encoder;
        }
        
        else if(1 - is_range(rc_data[TEMP].rc.rocker_r1))
        {
            lift_fetch_data.new_right_encoder = last_lift_right_angle; 
            lift_fetch_data.new_left_encoder = last_lift_left_angle;
        }
    }

    // 右侧开关状态[中],左侧开关状态[中]
    if ((switch_is_mid(rc_data[TEMP].rc.switch_right))&&switch_is_mid(rc_data[TEMP].rc.switch_left)) 
    {
        //一级伸出 yaw 
        if(1-is_range(rc_data[TEMP].rc.rocker_l_)&&(is_range(rc_data[TEMP].rc.rocker_r1)))
        {
            first_stretch_cmd_send.first_stretch_mode = FIRST_STRETCH;
            first_stretch_cmd_send.first_left = -rc_data[TEMP].rc.rocker_r1/660.0*15000 + first_stretch_fetch_data.new_left_encoder;
            first_stretch_cmd_send.first_right = rc_data[TEMP].rc.rocker_r1/660.0*15000 + first_stretch_fetch_data.new_right_encoder;
            last_first_right_angle =first_stretch_cmd_send.first_right; 
            last_first_left_angle = first_stretch_cmd_send.first_left;
        }
        else if (1-is_range(rc_data[TEMP].rc.rocker_r1)&&(is_range(rc_data[TEMP].rc.rocker_l_)))
        {
            first_stretch_cmd_send.first_stretch_mode = FIRST_YAW;
            first_stretch_cmd_send.first_left = -rc_data[TEMP].rc.rocker_l_/660.0*15000 + first_stretch_fetch_data.new_left_encoder;
            first_stretch_cmd_send.first_right = -rc_data[TEMP].rc.rocker_l_/660.0*15000 + first_stretch_fetch_data.new_right_encoder;
            last_first_right_angle = first_stretch_cmd_send.first_right; 
            last_first_left_angle = first_stretch_cmd_send.first_left;
        }
        else if (1-is_range(rc_data[TEMP].rc.rocker_r1)&&(1 - is_range(rc_data[TEMP].rc.rocker_l_)))
        {
            first_stretch_cmd_send.first_left = last_first_left_angle;
           first_stretch_cmd_send.first_right = last_first_right_angle;
        }
    
        //二级伸出 
        if(is_range(rc_data[TEMP].rc.rocker_l1))
        {
            second_stretch_cmd_send.second_stretch_mode = SECOND_STRETCH;
            second_stretch_cmd_send.second_left = rc_data[TEMP].rc.rocker_l1;
            second_stretch_cmd_send.second_right = -rc_data[TEMP].rc.rocker_l1;
        }
        
        else if(1-is_range(rc_data[TEMP].rc.rocker_l1))
        {
            // second_stretch_cmd_send.second_stretch_mode = SECOND_STRETCH;
            // second_stretch_cmd_send.second_left = last_second_left_angle;
            // second_stretch_cmd_send.second_right = last_second_right_angle;
        }

    
    } 


    // 右侧开关状态[上],左侧开关状态[中]
    if ((switch_is_up(rc_data[TEMP].rc.switch_right))&&switch_is_mid(rc_data[TEMP].rc.switch_left)) 
    {
        //气泵
        if (is_range(rc_data[TEMP].rc.dial)){
            //待补充
        }
        //横移
        if(is_range(rc_data[TEMP].rc.rocker_r_))
        {
            horizontal_cmd_send.Horizontal_MechAngle = rc_data[TEMP].rc.rocker_r_/100 + horizontal_fetch_data.now_angel;
            last_horizontal_angle = horizontal_cmd_send.Horizontal_MechAngle;
        }
        else if(1 - is_range(rc_data[TEMP].rc.rocker_r_))
        {
            horizontal_cmd_send.Horizontal_MechAngle =  last_horizontal_angle;
        }

        //升降
        // if(is_range(rc_data[TEMP].rc.rocker_l1))
        // {
        //     lift_cmd_send.lift_mode = LIFT;
        //     lift_cmd_send.left = rc_data[TEMP].rc.rocker_l1 + lift_fetch_data.new_left_encoder;
        //     lift_cmd_send.right = -rc_data[TEMP].rc.rocker_l1+ lift_fetch_data.new_right_encoder;
        //     last_lift_right_angle = lift_fetch_data.new_right_encoder; 
        //     last_lift_left_angle = lift_fetch_data.new_left_encoder;
        // }
        
        // else if(1 - is_range(rc_data[TEMP].rc.rocker_l1))
        // {
        //     lift_fetch_data.new_right_encoder = last_lift_right_angle; 
        //     lift_fetch_data.new_left_encoder = last_lift_left_angle;
        // }
        
        //前端
        // control_forward();
        // mode_record();

    }

    //双下
    if ((switch_is_down(rc_data[TEMP].rc.switch_right))&&switch_is_down(rc_data[TEMP].rc.switch_left)){
        if (is_range(rc_data[TEMP].rc.dial))
        {

        }
            //重新上电
        
    }

    
    //差个升降的限位，上车测数据再说
    //以上是升降的控制逻辑
}


/** @todo 每个部位的复位
 * @brief 键盘模式控制
 *
 */

void PC_Mode_Set(PC_Mode_t * mode){
    if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].w){
        *mode=PC_Walk;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].s){
        *mode=PC_Get_Money;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].f){
        *mode=PC_To_Begin_ALL;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].r){
        *mode=DA_MIAO_Reset_All;
    }
    // 以上是四种大模式的判断
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{   
    PC_Mode_Set(&PC_Mode);

    if (PC_Mode==PC_Walk){
        //行走模式
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;
    chassis_cmd_send.wz = rc_data[TEMP].key[KEY_PRESS].q * 300 - rc_data[TEMP].key[KEY_PRESS].e * 300;
    }

    else if (PC_Mode==PC_Get_Money){//兑矿模式0.
        //前端
        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].q){
            //pitch归中
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a){
            //roll归中
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].z ||rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s){
            //一级伸出归中 （同样也是一级yaw归中
            first_stretch_cmd_send.first_stretch_mode=FIRST_INIT;
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x){
            //二级伸出归中 （同样也是二级yaw归中，若禁用二级yaw无影响
            second_stretch_cmd_send.second_stretch_mode=SECOND_INIT;
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].e){
            //升降归位
            lift_cmd_send.lift_mode=LIFT_INIT;
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d){
            //横移归中

            // @todo 
            //为了防止干涉，这里应该判断伸出状态，再决定是否归位
            //等转正了再完善（bushi）
            horizontal_cmd_send.Horizontal_mode=HORIZONTAL_INIT;

        }

        //这里左右电机默认镜像，若反转应改正负
        first_stretch_cmd_send.first_left+=(rc_data[TEMP].key[KEY_PRESS].z*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].z*100+rc_data[TEMP].key[KEY_PRESS].s*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s*100);
        first_stretch_cmd_send.first_right+=(rc_data[TEMP].key[KEY_PRESS].z*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].z*100-rc_data[TEMP].key[KEY_PRESS].s*100+rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s*100);

        second_stretch_cmd_send.second_left+=rc_data[TEMP].key[KEY_PRESS].x*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x*100;
        second_stretch_cmd_send.second_left-=rc_data[TEMP].key[KEY_PRESS].x*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x*100;
        
        lift_cmd_send.left+=rc_data[TEMP].key[KEY_PRESS].x*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x*100;
        lift_cmd_send.right-=rc_data[TEMP].key[KEY_PRESS].x*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x*100;

        horizontal_cmd_send.Horizontal_MechAngle+=rc_data[TEMP].key[KEY_PRESS].d*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d*100;
    }

    else if (PC_Mode==PC_To_Begin_ALL){
        //全回到初始位置
        first_stretch_cmd_send.first_stretch_mode=FIRST_INIT;
        second_stretch_cmd_send.second_stretch_mode=SECOND_INIT;
        lift_cmd_send.lift_mode=LIFT_INIT;
        horizontal_cmd_send.Horizontal_mode=HORIZONTAL_INIT;
    }

    else if (PC_Mode==DA_MIAO_Reset_All){
        //重新上电达妙板子
    }


    // gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    // gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
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
//         first_stretch_cmd_send.first_stretch_mode   = FIRST_STOP;
//         second_stretch_cmd_send.first_stretch_mode   = SECOND_STOP;
//         chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
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

    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(lift_feed_sub, (void *)&lift_fetch_data);
    SubGetMessage(first_stretch_feed_sub, (void *)&first_stretch_fetch_data);
    SubGetMessage(second_stretch_feed_sub, (void *)&second_stretch_fetch_data);
    // SubGetMessage(forward_feed_sub, (void *)&forward_fetch_data);
    // SubGetMessage(horizontal_feed_sub, (void *)&horizontal_fetch_data);
    
    // PubPushMessage(forward_cmd_pub, (void *)&forward_cmd_send);
    // PubPushMessage(horizontal_cmd_pub, (void *)&horizontal_cmd_send);·
       //遥控器左下右中，切换为电脑模式
    //遥控器其余状态为遥控器模式
    if (switch_is_down(rc_data[TEMP].rc.switch_left)&&switch_is_mid(rc_data[TEMP].rc.switch_right)){
        MouseKeySet();
   }
   else {
    RemoteControlSet();
   }
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(lift_cmd_pub, (void *)&lift_cmd_send);
    PubPushMessage(first_stretch_cmd_pub, (void *)&first_stretch_cmd_send);
    PubPushMessage(second_stretch_cmd_pub, (void *)&second_stretch_cmd_send);
}





//下面这个是前端的不用鸟它


// int8_t mode;           // pitch和roll的模式
// int8_t last_mode;
// float last_angle;     // pitch的最后一次编码器角度
// float relevant_angle; // pitch和roll的相对角度
// // 动之前的roll编码器
// float roll_last_angle; // roll的最后一次编码器角度
// void mode_change();

// void control_forward()
// {
//     mode_change();
//     if (mode ==PITCH_RUN_MODE) //pitch
//     {
//         last_angle = forward_fetch_data.new_left_angle;
//     }
//     if (mode == ROLL_RUN_MODE) //roll
//     {
//         relevant_angle = (forward_fetch_data.new_forward_angle - roll_last_angle) * Rotation_Ratio;
//         last_angle     = forward_fetch_data.new_left_angle;
//     }
//     if (mode != ROLL_RUN_MODE && last_mode == ROLL_RUN_MODE) {
//         relevant_angle = 0;
//     }
//     if (mode != ROLL_RUN_MODE) {
//         roll_last_angle = forward_fetch_data.new_forward_angle;
//     }
//     forward_cmd_send.final_angle  = rc_data[TEMP].rc.rocker_l_ + rc_data[TEMP].rc.rocker_r1 + last_angle - relevant_angle; // 把roll动的时候的pitch编码器转过的角度减去
//     forward_cmd_send.angel_output = PIDCalculate(encoder_pid, forward_fetch_data.new_left_angle,forward_cmd_send.final_angle);

//     if (mode == ROLL_RUN_MODE) {
//         forward_cmd_send.angel_output1 = -forward_cmd_send.angel_output;
//     } else {
//         forward_cmd_send.angel_output1 = forward_cmd_send.angel_output;
//     }

// }

// void mode_change()
// {
    
//         if (((rc_data[TEMP].rc.rocker_l_ < 5)&&(rc_data[TEMP].rc.rocker_l_ > -5))&&((rc_data[TEMP].rc.rocker_r1 > 5)&&(rc_data[TEMP].rc.rocker_r1 < -5))) {
//             mode = PITCH_RUN_MODE;
//             forward_cmd_send.Forward_mode = PITCH;
//         } else if (((rc_data[TEMP].rc.rocker_l_ > 5)&&(rc_data[TEMP].rc.rocker_l_ < -5))&&((rc_data[TEMP].rc.rocker_r1 < 5)&&(rc_data[TEMP].rc.rocker_r1 > -5))) {
//             mode = ROLL_RUN_MODE;
//             forward_cmd_send.Forward_mode = ROLL;
//         } else if (((rc_data[TEMP].rc.rocker_l_ < 5)&&(rc_data[TEMP].rc.rocker_l_ > -5))&&((rc_data[TEMP].rc.rocker_r1 < 5)&&(rc_data[TEMP].rc.rocker_r1 > -5))) {
//             mode = STOP_MODE;
//             forward_cmd_send.Forward_mode = PITCH;
//         }
// }
// void mode_record()
// {
//     last_mode = mode;
// }