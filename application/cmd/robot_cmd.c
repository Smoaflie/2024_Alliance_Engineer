// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "can_comm.h"
#include "led.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t *chassis_cmd_pub;        // 底盘控制消息发布者
static Publisher_t *gimbal_cmd_pub;        // 云台控制消息发布者
static Publisher_t *arm_cmd_pub;    // 机械臂控制信息发布者
static Publisher_t *airpump_cmd_pub;// 气阀/气泵控制信息发布者

static Subscriber_t *gimbal_data_sub;                   // 用于接收云台的数据信息
static Subscriber_t *arm_data_sub;                   // 用于接收臂臂的数据信息

static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 发送给云台应用的信息
static Arm_Cmd_Data_s arm_cmd_data; // 发送给机械臂应用的信息
static Airpump_Cmd_Data_s airpump_cmd_send; // 发送给气阀/气泵控制应用的信息

static Gimbal_Data_s     gimbal_data_recv;// 云台发布的信息
static Arm_Data_s     arm_data_recv;// 云台发布的信息

static RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回

static Robot_Status_e robot_state; // 机器人整体工作状态

static int8_t dial_flag = 0;
static uint8_t pump_state = 0;
static uint8_t redlight_flag = 0;


void RobotCMDInit()
{
    rc_data     = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    robot_state = ROBOT_STOP;                // 启动时机器人为停止状态，避免一上电创死场地

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    arm_cmd_pub     = PubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    airpump_cmd_pub = PubRegister("airpump_cmd",sizeof(Airpump_Cmd_Data_s));

    gimbal_data_sub = SubRegister("gimbal_data", sizeof(Gimbal_Data_s));
    arm_data_sub = SubRegister("arm_data", sizeof(Arm_Data_s));

    memset(&arm_cmd_data, 0, sizeof(arm_cmd_data));
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    /* 检测拨轮状态，因为这东西很容易坏，不能直接判断当前值 */
    static int16_t dial_cnt = 0;
    if(rc_data->rc.dial == 660 && dial_cnt>=0) dial_cnt++;
    else if(rc_data->rc.dial <=-580 && dial_cnt<=0) dial_cnt--;
    else    dial_cnt=0;
    if(dial_cnt>100)    dial_flag=-1;   //down
    else if(dial_cnt<-40)  dial_flag=1;//up
    else dial_flag = 0;
    
    // 左侧开关状态为[下],控制底盘云台
    if (switch_is_down(rc_data->rc.switch_left)) {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_CONTRO; // 底盘使能
        gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE; // 云台使能
        arm_cmd_data.contro_mode = ARM_FIXED; // 臂臂固定
        
        // 左摇杆控制底盘移动，右摇杆控制底盘旋转和云台pitch
        gimbal_cmd_send.yaw = rc_data->rc.rocker_r_/660.0*0.5; // 云台电机旋转
        gimbal_cmd_send.pitch = rc_data->rc.rocker_r1/660.0*0.2; // 云台pitch
        chassis_cmd_send.vx = 40.0f * (float)rc_data->rc.rocker_l_; // 底盘水平方向
        chassis_cmd_send.vy = 40.0f * (float)rc_data->rc.rocker_l1; // 底盘竖值方向
        
        // 右侧开关为[中]，拨轮控制底盘旋转
        if(switch_is_mid(rc_data->rc.switch_right))
        {
            if(dial_flag == 1)  chassis_cmd_send.wz = 1500.0f; // 底盘顺时针旋转
            else if(dial_flag == -1)  chassis_cmd_send.wz = -1500.0f; // 底盘逆时针旋转
            else chassis_cmd_send.wz = 0;
        }

        // 右侧开关为[上]，选择自动模式
        if(switch_is_up(rc_data->rc.switch_right)){
            // 拨轮向上时选择模式
            // 不同的模式间用LED颜色区分
            // todo：后续ui中可加上当前选择的模式标识
            static uint8_t leftswitch_is_down_mode_switch_id = 0;
            static uint8_t leftswitch_is_down_mode_switch_state = 0;
            if(dial_flag == 1 && !(leftswitch_is_down_mode_switch_state & 0x01)){
                leftswitch_is_down_mode_switch_state |= 0x01;
                leftswitch_is_down_mode_switch_id++;
                if(leftswitch_is_down_mode_switch_id>2)   leftswitch_is_down_mode_switch_id=1;
            }else if(!(dial_flag == 1)){
                leftswitch_is_down_mode_switch_state &= ~0x01;
            }
            //LED标示模式
            switch(leftswitch_is_down_mode_switch_id){
                case 1:DM_board_LEDSet(0xff0000);break;// 红色
                case 2:DM_board_LEDSet(0xFF8C00);break;// 橙色
                case 3:DM_board_LEDSet(0xffff00);break;// 黄色
                default:DM_board_LEDSet(0x000000);
            }
            // 波轮向下时应用所选模式
            if(dial_flag == -1){
                switch(leftswitch_is_down_mode_switch_id){
                    case 1:airpump_cmd_send.mode|=0x04;break;// 1:气推杆取左侧矿，红色
                    case 2:airpump_cmd_send.mode|=0x08;break;// 2:气推杆取中间矿，橙色 
                    case 3:gimbal_cmd_send.gimbal_mode=GIMBAL_RESET;break;// 3:云台复位，黄色
                    default:break;
                }
            }else{
                airpump_cmd_send.mode&=~0x04;airpump_cmd_send.mode&=~0x08;
            }
        }
    }

    // 左侧开关为[中]，控制底盘臂臂 or 单独控制臂臂
    if (switch_is_mid(rc_data->rc.switch_left)) {
        static uint8_t arm_contro_func = 0; //标志控制方式，0为底盘臂臂，1为单独臂臂
        if(arm_contro_func == 0){    //控制底盘臂臂
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_CONTRO; // 底盘使能
            gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE; // 云台使能
            arm_cmd_data.contro_mode = ARM_POSE_CONTRO_MODE; // 臂臂位置控制
            // 左摇杆控制底盘移动，右摇杆控制臂的旋转和竖直平移
            chassis_cmd_send.vx = 10.0f * (float)rc_data->rc.rocker_l_; // 底盘水平方向
            chassis_cmd_send.vy = 10.0f * (float)rc_data->rc.rocker_l1; // 底盘竖值方向
            arm_cmd_data.Position_z = 0.5 * rc_data->rc.rocker_r1 / 660.0; // 臂竖直平移
            arm_cmd_data.Rotation_yaw = 0.06 * rc_data->rc.rocker_r_ / 660.0; // 臂的旋转
        }else if(arm_contro_func == 1){   //单独控制臂臂
            arm_cmd_data.contro_mode = ARM_REFER_MODE;
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; // 底盘使能
            gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE; // 云台使能
            // 左摇杆控制臂臂前后移动，右摇杆控制roll&pitch
            arm_cmd_data.Translation_x = -rc_data->rc.rocker_l1 / 660.0 *0.0002;
            arm_cmd_data.Translation_y = rc_data->rc.rocker_l_/ 660.0 *0.0002;
            arm_cmd_data.Roatation_Horizontal = rc_data->rc.rocker_r_/ 660.0 *0.08 ;
            arm_cmd_data.Roatation_Vertical  = -rc_data->rc.rocker_r1/ 660.0 *0.08 ;
        }

        // 右侧开关为[中]，拨轮控制吸盘roll
        if(switch_is_mid(rc_data->rc.switch_right))
        {
            // 拨轮向上吸盘顺时针旋转，向下吸盘逆时针旋转
            if(dial_flag == 1)  arm_cmd_data.sucker_state = 1;
            else if(dial_flag == -1)  arm_cmd_data.sucker_state = -1;
            else arm_cmd_data.sucker_state = 0;
        }
        
        // 右侧开关为[下]，波轮上控制气泵开关 | 拨轮下臂臂操作方式
        if(switch_is_down(rc_data->rc.switch_right)){
            // 拨轮向上，切换气泵开关
            if(dial_flag == 1 && !(pump_state & 0x01)){
                airpump_cmd_send.mode & 0x01 ? (airpump_cmd_send.mode &= ~0x01) : (airpump_cmd_send.mode |= 0x01);
                pump_state |= 0x01;
            }else if(!(dial_flag == 1)){
                pump_state &= ~0x01;
            }
            // 拨轮向下时，切换臂臂操作方式
            static uint8_t switch_flag = 0;
            if(dial_flag == -1 && !switch_flag){
                arm_contro_func = !(arm_contro_func && 0x01);
                switch_flag = 1;
            }else{
                switch_flag = 0;
            }
        }

        // 右侧开关为[上]，切换臂臂自动变换模式
        if(switch_is_up(rc_data->rc.switch_right)){
            // 拨轮向上时选择模式
            // 不同的模式间用LED颜色区分
            // todo：后续ui中可加上当前选择的模式标识
            static uint8_t leftswitch_is_mid_mode_switch_id = 0;
            static uint8_t leftswitch_is_mid_mode_switch_state = 0;
            if(dial_flag == 1 && !(leftswitch_is_mid_mode_switch_state & 0x01)){
                leftswitch_is_mid_mode_switch_state |= 0x01;
                leftswitch_is_mid_mode_switch_id++;
                if(leftswitch_is_mid_mode_switch_id>6)   leftswitch_is_mid_mode_switch_id=1;
            }else if(!(dial_flag == 1)){
                leftswitch_is_mid_mode_switch_state &= ~0x01;
            }
            //LED标示模式
            switch(leftswitch_is_mid_mode_switch_id){
                case 1:DM_board_LEDSet(0xff0000);break;// 红色
                case 2:DM_board_LEDSet(0xFF8C00);break;// 橙色 
                case 3:DM_board_LEDSet(0xffff00);break;// 黄色
                case 4:DM_board_LEDSet(0x00FF00);break;// 绿色
                case 5:DM_board_LEDSet(0x00FFFF);break;// 青色
                case 6:DM_board_LEDSet(0x0000FF);break;// 蓝色
                case 7:DM_board_LEDSet(0x8B00FF);break;// 紫色
                case 8:DM_board_LEDSet(0xff77ff);break;// 粉色
                default:DM_board_LEDSet(0x000000);
            }
            // 波轮向下时应用所选模式
            if(dial_flag == -1){
                switch(leftswitch_is_mid_mode_switch_id){
                    case 1:arm_cmd_data.auto_mode = Reset_arm_cmd_param_flag;break;// 1:重置臂臂状态，红色
                    case 2:arm_cmd_data.auto_mode = Recycle_arm_out;break;// 2:臂臂回收到肚子外, 橙色
                    case 3:arm_cmd_data.auto_mode = Recycle_arm_in;break;// 3:臂臂收回肚子，黄色 
                    case 4:arm_cmd_data.auto_mode = Arm_get_goldcube_mid;break;// 4:取中间矿,绿色
                    case 5:arm_cmd_data.auto_mode = Arm_get_goldcube_left;break;// 5:取左侧矿,青色
                    case 6:arm_cmd_data.auto_mode = Arm_fetch_cube_from_warehouse1;break; //6:从矿仓1中取矿，蓝色
                    case 7:arm_cmd_data.auto_mode = Arm_fetch_cube_from_warehouse1;break; //7:从矿仓2中取矿，紫色
                }
            }else{
                arm_cmd_data.auto_mode = 0;
            }
        }
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_CONTRO; // 底盘使能
    gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE; // 云台使能
    arm_cmd_data.contro_mode = ARM_POSE_CONTRO_MODE; // 臂臂位置控制

    chassis_cmd_send.vx = rc_data->key[KEY_PRESS].w * 300 - rc_data->key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data->key[KEY_PRESS].s * 300 - rc_data->key[KEY_PRESS].d * 300;

    chassis_cmd_send.wz = (rc_data->key[KEY_PRESS].q - rc_data->key[KEY_PRESS].e)  * 2500.0f * (float)rc_data->mouse.x/660.0; // 底盘旋转

    gimbal_cmd_send.yaw += (float)rc_data->mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data->mouse.y / 660 * 10;

    switch (rc_data->key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
        case 0:
            chassis_cmd_send.chassis_speed_buff = 40;
            break;
        case 1:
            chassis_cmd_send.chassis_speed_buff = 60;
            break;
        case 2:
            chassis_cmd_send.chassis_speed_buff = 80;
            break;
        default:
            chassis_cmd_send.chassis_speed_buff = 100;
            break;
    }

    // g开关气泵 ctrl+g是臂气泵 shift+g是推杆气泵
    if(rc_data->key[Key_Ctrl].g && !(pump_state & 0x01)){
        airpump_cmd_send.mode & 0x01 ? (airpump_cmd_send.mode &= ~0x01) : (airpump_cmd_send.mode |= 0x01);
        pump_state |= 0x01;
    }else if(!(rc_data->key[Key_Ctrl].g)){
        pump_state &= ~0x01;
    }
    if(rc_data->key[Key_Shift].g && !(pump_state & 0x02)){
        airpump_cmd_send.mode & 0x02 ? (airpump_cmd_send.mode &= ~0x02) : (airpump_cmd_send.mode |= 0x02);
        pump_state |= 0x02;
    }else if(!(rc_data->key[Key_Shift].g)){
        pump_state &= ~0x02;
    }

    // zx控制取中心资源岛矿 z让推杆取 x让臂臂取
    // ctrl+z/x 取中心资源岛左侧/中间的金矿
    if(rc_data->key[Key_Ctrl].z){
        airpump_cmd_send.mode|=0x04;airpump_cmd_send.mode&=~0x08;
    }
    if(rc_data->key[Key_Ctrl].x){
        airpump_cmd_send.mode|=0x04;airpump_cmd_send.mode&=~0x08;
    }
    // shift+z/x 取中心资源岛中间/右侧的金矿
    if(rc_data->key[Key_Shift].z){
        airpump_cmd_send.mode|=0x08;airpump_cmd_send.mode&=~0x04;
    }
    if(rc_data->key[Key_Shift].x){
        airpump_cmd_send.mode|=0x04;airpump_cmd_send.mode&=~0x08;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    static uint8_t flag = 1;
    if (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
        osDelay(1);
        while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
            ;
        flag = !flag&1;
    }

    // 双下为急停
    if (robot_state == ROBOT_STOP ||  (switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right)) || flag == 0) // 还需添加重要应用和模块离线的判断
    {
        if(robot_state == ROBOT_READY)  LOGERROR("[CMD] emergency stop!");
        robot_state                   = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        arm_cmd_data.contro_mode      = ARM_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        airpump_cmd_send.mode |= 0x40;
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data->rc.switch_right) && flag == 1) {
        if(robot_state == ROBOT_STOP)   LOGINFO("[CMD] reinstate, robot ready");
        robot_state = ROBOT_READY;
        airpump_cmd_send.mode &= ~0x40;
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(gimbal_data_sub,&gimbal_data_recv);
    SubGetMessage(arm_data_sub,&arm_data_recv);

    chassis_cmd_send.offset_angle = gimbal_data_recv.yaw + arm_data_recv.big_yaw_angle;

    // 监测红外测距模块状态
    if(HAL_GPIO_ReadPin(redLight_detect_GPIO_Port,redLight_detect_Pin)==GPIO_PIN_SET){
        redlight_flag=1;    // 1为有物体遮挡
    }else redlight_flag = 0;

    // 左侧开关为上时，使用键鼠操作
    if(switch_is_up(rc_data->rc.switch_left)){
        MouseKeySet();
    }else{ //否则使用遥控器操作
        RemoteControlSet();
    }

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    PubPushMessage(arm_cmd_pub, (void *)&arm_cmd_data);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(airpump_cmd_pub, (void *)&airpump_cmd_send);
}
