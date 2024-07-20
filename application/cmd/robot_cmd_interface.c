// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "flashtask.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "can_comm.h"
#include "buzzer.h"
#include "led.h"
#include "tool.h"
#include "vision_rec.h"
#include "user_lib.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t *chassis_cmd_pub;        // 底盘控制消息发布者
static Publisher_t *gimbal_cmd_pub;        // 云台控制消息发布者
static Publisher_t *arm_cmd_pub;    // 机械臂控制信息发布者
static Publisher_t *airpump_cmd_pub;// 气阀/气泵控制信息发布者
static Publisher_t *UI_reality_pub;// 物理UI信息发布者
static Publisher_t *UI_cmd_pub;// 选手端UI命令发布者

static Subscriber_t *gimbal_data_sub;                   // 用于接收云台的数据信息
static Subscriber_t *arm_data_sub;                   // 用于接收臂臂的数据信息
static Subscriber_t *air_data_sub;                   // 用于接收气阀气泵的数据信息

static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 发送给云台应用的信息
static Arm_Cmd_Data_s arm_cmd_send; // 发送给机械臂应用的信息
static Airpump_Cmd_Data_s airpump_cmd_send; // 发送给气阀/气泵控制应用的信息
static UI_reality_Data_s UI_reality_send;  // 发送给物理UI的信息
static UI_data_t UI_cmd_send;               // 发送给选手端UI的信息

static Gimbal_Data_s     gimbal_data_recv;// 云台发布的信息
static Arm_Data_s     arm_data_recv;// 臂任务发布的信息
static Airpump_Data_s air_data_recv;// 气阀气泵任务发布的信息    

static RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回
RC_ctrl_t* vision_rc_data;           // 图传遥控数据

static Robot_Status_e robot_state; // 机器人整体工作状态

static int8_t dial_flag = 0;    //遥控器拨轮状态标志
static uint8_t redlight_flag = 0;//红外测距模块状态标志
static uint8_t zero_output_init_flag = 0; //防止忘记双下而写死的标志位

/* 自动模式id */
static uint8_t arm_auto_mode_id = 1;    //臂臂自动模式
static uint8_t chassis_auto_mod_id = 1; //底盘自动模式

// 地盘速度斜坡函数
static GPIOInstance *relay_contro_gpio; //继电器io口

_RobotControlMode ControlMode = ControlMode_FetchCube; //控制模式

static uint8_t debug_switch_arm_auto_mode = 0;


void RobotCMDInit_VisionLine()
{
    USART_Init_Config_s vision_usart_conf = {
        .module_callback = vision_recv_callback,
        .checkout_callback = NULL,
        .recv_buff_size  = USART_RXBUFF_LIMIT-1,
        .usart_handle    = &huart1, // 达妙板子的原理图写USART3，但实际管脚对应的是UART1
    };
    vision_rc_data = malloc(sizeof(RC_ctrl_t));
    memset(vision_rc_data,0,sizeof(RC_ctrl_t));
    vision_usart       = USARTRegister(&vision_usart_conf);// 图传串口
}
void RobotCMDInit_RC()
{
    rc_data     = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
}
void RobotCMDInit_Param()
{
    robot_state = ROBOT_STOP;                // 启动时机器人为停止状态，避免一上电创死场地
    memset(&arm_cmd_send, 0, sizeof(arm_cmd_send));
}
void RobotCMDInit_IO()
{
    GPIO_Init_Config_s gpio_conf_relay_contro = {
        .GPIOx = relay_contro_GPIO_Port,
        .GPIO_Pin = relay_contro_Pin,
    };
    relay_contro_gpio = GPIORegister(&gpio_conf_relay_contro);
}
void RobotCMDInit_Communication()
{
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    arm_cmd_pub     = PubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    airpump_cmd_pub = PubRegister("airpump_cmd",sizeof(Airpump_Cmd_Data_s));
    UI_reality_pub = PubRegister("UI_reality",sizeof(UI_reality_Data_s));
    UI_cmd_pub     = PubRegister("UI", sizeof(UI_data_t));

    gimbal_data_sub = SubRegister("gimbal_data", sizeof(Gimbal_Data_s));
    arm_data_sub = SubRegister("arm_data", sizeof(Arm_Data_s));
    air_data_sub = SubRegister("air_data", sizeof(Airpump_Data_s));
}
static void arm_auto_mode_select(){
    switch(arm_auto_mode_id){
        case 0:
            break;// 0:无自动模式，白色 
        case 1: //红
            arm_cmd_send.auto_mode = Arm_walk_state;break; //行走模式
        case 2: //橙
            arm_cmd_send.auto_mode = Recycle_arm_in;ControlMode = ControlMode_Move;break; //收臂臂回肚子
        case 3: //黄
            arm_cmd_send.auto_mode = debug_switch_arm_auto_mode;break; //调试模式
        case 4: //绿
            arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse_up;ControlMode = ControlMode_ConvertCube;break; //从矿仓中取矿 W1
        case 5: //青
            arm_cmd_send.auto_mode = Arm_get_goldcube_right;break;// 金矿右 GL
        case 6: //蓝
            arm_cmd_send.auto_mode = Arm_get_silvercube_left;break;// 银矿左 SL
        case 7: //紫
            arm_cmd_send.auto_mode = Arm_get_silvercube_mid;break;// 银矿中   SM
        case 8: //粉
            arm_cmd_send.auto_mode = Arm_get_silvercube_right;break; //银矿右    SR
        case 9:
            arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse_down;ControlMode = ControlMode_ConvertCube;break; //从矿仓2中取矿
        case 10:
            arm_cmd_send.auto_mode = Arm_fetch_gronded_cube;break; // 地矿
        case 11:
            arm_cmd_send.auto_mode = Arm_ConvertCube;ControlMode = ControlMode_ConvertCube;break; //兑矿姿态
    }
}
static void chassis_auto_mode_select(){
    switch(chassis_auto_mod_id){
            case 1:airpump_cmd_send.airvalve_mode=AIRVALVE_LEFT_CUBE;break;// 1:气推杆取左侧矿，红色
            case 2:airpump_cmd_send.airvalve_mode=AIRVALVE_MIDDLE_CUBE;break;// 2:气推杆取中间矿，橙色 
            default:break;
        }
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet_SwitchLeftMid()
{
    // ControlMode = ControlMode_FetchCube;

    static uint8_t arm_contro_mode_id = 1; //标志控制方式
    switch(arm_contro_mode_id){
        case Arm_Control_with_Chassis:
            // 左摇杆控制底盘移动，右摇杆控制臂的旋转和竖直平移
            chassis_cmd_send.vx = 10.0f * rc_data->rc.rocker_l_; // 底盘水平方向
            chassis_cmd_send.vy = 10.0f * rc_data->rc.rocker_l1; // 底盘竖值方向
            arm_cmd_send.Position_z = 0.5 * rc_data->rc.rocker_r1 / 660.0; // 臂竖直平移
            arm_cmd_send.Rotation_yaw = 0.06 * rc_data->rc.rocker_r_ / 660.0; // 臂的旋转
            break;
        case Arm_Control_only_Arm:
            // 左摇杆控制臂臂前后移动，右摇杆控制roll&pitch
            arm_cmd_send.Translation_x = -rc_data->rc.rocker_l1 / 660.0 *0.0002;
            arm_cmd_send.Translation_y = -rc_data->rc.rocker_l_/ 660.0 *0.0002;
            arm_cmd_send.Roatation_Horizontal = rc_data->rc.rocker_r_/ 660.0 *0.08 ;
            arm_cmd_send.Roatation_Vertical  = -rc_data->rc.rocker_r1/ 660.0 *0.08 ;
            break;
        case Arm_Control_by_Custom_controller:
            // 左摇杆控制底盘移动，右摇杆控制底盘旋转和云台pitch
            chassis_cmd_send.wz = 10.0f * rc_data->rc.rocker_r_; // 底盘旋转
            gimbal_cmd_send.pitch = rc_data->rc.rocker_r1/660.0*0.2; // 云台pitch
            chassis_cmd_send.vx = 40.0f * rc_data->rc.rocker_l_; // 底盘水平方向
            chassis_cmd_send.vy = 40.0f * rc_data->rc.rocker_l1; // 底盘竖值方向
            break;
        case Arm_Control_by_vision:
            // 左摇杆控制底盘移动，右摇杆控制底盘旋转和云台pitch
            chassis_cmd_send.wz = 10.0f * rc_data->rc.rocker_r_; // 底盘旋转
            gimbal_cmd_send.pitch = rc_data->rc.rocker_r1/660.0*0.2; // 云台pitch
            chassis_cmd_send.vx = 40.0f * rc_data->rc.rocker_l_; // 底盘水平方向
            chassis_cmd_send.vy = 40.0f * rc_data->rc.rocker_l1; // 底盘竖值方向
            break;
    }

    // 右侧开关为[中]，拨轮控制吸盘roll
    if(switch_is_mid(rc_data->rc.switch_right))
    {
        // 拨轮向上吸盘顺时针旋转，向下吸盘逆时针旋转
        if(dial_flag == 1)  arm_cmd_send.call.sucker_call = Arm_sucker_clockwise_rotation;
        else if(dial_flag == -1)  arm_cmd_send.call.sucker_call = Arm_sucker_anticlockwise_rotation;
        else arm_cmd_send.call.sucker_call = Arm_sucker_none_rotation;
    }
    
    // 右侧开关为[下]，波轮上发送视觉识别信号 | 拨轮下臂臂操作方式
    if(switch_is_down(rc_data->rc.switch_right)){
        // // 拨轮向上，发送视觉识别信号（需处于视觉控制模式）
        // if(dial_flag == 1){
        //     arm_cmd_send.vision_signal = 1;
        // }else{
        //     arm_cmd_send.vision_signal = 0;
        // }

        // 拨轮向下时，切换臂臂操作方式
        static uint8_t switch_dial_down_flag = 0;
        if(dial_flag == -1 && !(switch_dial_down_flag==1)){
            arm_contro_mode_id<<=1;
            switch_dial_down_flag = 1;
            if(arm_contro_mode_id > 0x10) {arm_contro_mode_id=1;}

        }else if(!(dial_flag == -1)){
            switch_dial_down_flag = 0;
        }
    }

    // 右侧开关为[上]，切换臂臂自动变换模式
    if(switch_is_up(rc_data->rc.switch_right)){
        // 拨轮向上时选择模式
        // 不同的模式间用LED颜色区分
        // todo：后续ui中可加上当前选择的模式标识
        static uint8_t switch_flag = 0;
        if(dial_flag == 1 && !(switch_flag & 0x01)){
            ControlMode = ControlMode_FetchCube;
            switch_flag |= 0x01;
            arm_auto_mode_id++;
            if(arm_auto_mode_id>11)   arm_auto_mode_id=1;
        }else if(!(dial_flag == 1)){
            switch_flag &= ~0x01;
        }
        // 拨轮向下时应用模式
        if(dial_flag == -1){
            arm_cmd_send.contro_mode = ARM_AUTO_MODE;
            arm_auto_mode_select();
        }
    }
}
static uint8_t Reset_Param(uint8_t bool_){
    static uint16_t reset_press_cnt = 0;
    if(bool_){
        reset_press_cnt++;
        if(reset_press_cnt==1){
            arm_cmd_send.call.reset_init_flag = 1;
            airpump_cmd_send.init_call = 1;
        }else{
            arm_cmd_send.call.reset_init_flag = 0;
            airpump_cmd_send.init_call = 0;
        }
            
        if(reset_press_cnt > 1000){
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
        return 1;
    }else reset_press_cnt=0;
    return 0;
}
static void RemoteControlSet_SwitchLeftDown()
{
    arm_cmd_send.contro_mode = ARM_FIXED; // 臂臂固定
    
    // 左摇杆控制底盘移动，右摇杆控制云台
    gimbal_cmd_send.yaw = rc_data->rc.rocker_r_/660.0*0.5; // 云台电机旋转
    gimbal_cmd_send.pitch = rc_data->rc.rocker_r1/660.0*0.2; // 云台pitch
    chassis_cmd_send.vx = 80.0f * rc_data->rc.rocker_l_; // 底盘水平方向
    chassis_cmd_send.vy = 80.0f * rc_data->rc.rocker_l1; // 底盘竖值方向
    
    // 右侧开关为[中]，拨轮控制底盘旋转
    if(switch_is_mid(rc_data->rc.switch_right))
    {
        // if(dial_flag == 1)  chassis_cmd_send.wz = 1500.0f; // 底盘顺时针旋转
        // else if(dial_flag == -1)  chassis_cmd_send.wz = -1500.0f; // 底盘逆时针旋转
        // else chassis_cmd_send.wz = 0;
        if(dial_flag == 1)  ControlMode = ControlMode_Move;
        if(dial_flag == -1)
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        else chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }

    // 右侧开关为[上]，选择自动模式
    if(switch_is_up(rc_data->rc.switch_right)){
        // 拨轮向上时选择模式
        // 不同的模式间用LED颜色区分
        static uint8_t switch_flag = 0;
        if(dial_flag == 1 && !(switch_flag & 0x01)){
            switch_flag |= 0x01;
            chassis_auto_mod_id++;
            if(chassis_auto_mod_id>4)   chassis_auto_mod_id=1;
        }else if(!(dial_flag == 1)){
            switch_flag &= ~0x01;
        }
        //LED标示模式
        // switch(chassis_auto_mod_id){
        //     case 1:DM_board_LEDSet(0xff0000);break;// 红色
        //     case 2:DM_board_LEDSet(0xffa308);break;// 橙色
        //     case 3:DM_board_LEDSet(0xffee46);break;// 黄色
        // }
        // 波轮向下时应用所选模式
        if(dial_flag == -1)
            chassis_auto_mode_select();
        // static uint8_t ControlMode_ReverseMove_switch = 0;
        // if(dial_flag == -1 && !ControlMode_ReverseMove_switch)
        // {
        //     if(ControlMode == ControlMode_Move) ControlMode = ControlMode_ReverseMove;
        //     else if(ControlMode == ControlMode_ReverseMove) ControlMode = ControlMode_Move;
        //     ControlMode_ReverseMove_switch = 1;
        // }else if(!(dial_flag == -1))
        // {
        //     ControlMode_ReverseMove_switch = 0;
        // }
    }
}
static void RemoteControlSet()
{    
    //双下同时拨轮向上保持三秒复位C板
    // if(Reset_Param((switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right) && dial_flag==1)))
        // robot_state = ROBOT_STOP;
    // else{
        // 左侧开关状态为[下],控制底盘云台
        if (switch_is_down(rc_data->rc.switch_left)) 
            RemoteControlSet_SwitchLeftDown();

        // 左侧开关为[中]，控制底盘臂臂 or 单独控制臂臂
        if (switch_is_mid(rc_data->rc.switch_left))
            RemoteControlSet_SwitchLeftMid();
    // }
        
}
/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeyControlSet_Debug(){
    /* UI相关操作 */
    UI_debug_param *debug = &UI_cmd_send.debug;
    debug->debug_flag = 1;
    // static uint8_t switch_delay[3] = {0};
    static uint8_t switch_bool[3] = {0};
    if(UI_cmd_pub->first_subs->temp_size==0){
        if(!rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            debug->pos_upORdown = 20*(rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s + rc_data->mouse.y);
            debug->pos_leftORright = 20*(rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d + rc_data->mouse.x);
            debug->param1 = rc_data->key[KEY_PRESS].g-rc_data->key[KEY_PRESS].f;
            debug->width = rc_data->key[KEY_PRESS].v-rc_data->key[KEY_PRESS].c;
            debug->add_ui = rc_data->key[KEY_PRESS].b;
            debug->reset_to_center = rc_data->key[KEY_PRESS].r;
            // if(switch_delay[0]-- == 0)
            if(detect_edge(&switch_bool[0], rc_data->key[KEY_PRESS].e||rc_data->key[KEY_PRESS].q) == EDGE_RISING)
                debug->switch_selected_ui = rc_data->key[KEY_PRESS].e-rc_data->key[KEY_PRESS].q;
            // if(switch_delay[1]-- == 0)
            if(detect_edge(&switch_bool[1], rc_data->key[KEY_PRESS].x||rc_data->key[KEY_PRESS].z) == EDGE_RISING)
                debug->switch_type = rc_data->key[KEY_PRESS].x-rc_data->key[KEY_PRESS].z;
            
        }else if(rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            debug->pos_upORdown = (rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s + rc_data->mouse.y);
            debug->pos_leftORright = (rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d + rc_data->mouse.x);
            debug->param2 = rc_data->key[KEY_PRESS].g-rc_data->key[KEY_PRESS].f;
            debug->undo = rc_data->key[KEY_PRESS].z;
            debug->cut = rc_data->key[KEY_PRESS].x;
            debug->paste = rc_data->key[KEY_PRESS].v;
            debug->copy = rc_data->key[KEY_PRESS].c;
            debug->delete_ui = rc_data->key[KEY_PRESS].b;
            // if(switch_delay[2]-- == 0)
            if(detect_edge(&switch_bool[2], rc_data->key[KEY_PRESS].e||rc_data->key[KEY_PRESS].q) == EDGE_RISING)
                debug->switch_color = rc_data->key[KEY_PRESS].e-rc_data->key[KEY_PRESS].q;
        }else if(!rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
            debug->param3 = rc_data->key[KEY_PRESS].g-rc_data->key[KEY_PRESS].f;
        }else{
            debug->param4 = rc_data->key[KEY_PRESS].g-rc_data->key[KEY_PRESS].f;
        }

        // if(debug->switch_selected_ui)   switch_delay[0] = 3;
        // if(debug->switch_type)  switch_delay[1] = 3;
        // if(debug->switch_color) switch_delay[2] = 3;
    }

    /* ctrl+shift+b 保存修改到flash */
    static uint16_t save_press_cnt = 0;
    if(rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift && rc_data->key[KEY_PRESS].b && rc_data->mouse.press_l){
        save_press_cnt++;
        robot_state = ROBOT_STOP;
        UI_cmd_send.debug.save_flag = 1;
            
        if(save_press_cnt > 2000){
            buzzer_one_note(0x50, 0.1);
            flashRefresh();
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
    }else save_press_cnt=0;
}
static void MouseKeyControlSet_Normal()
{
    robot_state = ROBOT_READY;
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; //底盘跟随云台
    UI_cmd_send.arm_selected_mode = 0;
    UI_cmd_send.valve_selected_mode = 0;
    UI_cmd_send.arm_temp_halt_selected = 0;
    UI_cmd_send.valve_temp_halt_selected = 0;

    

    // wasd 控制底盘全向移动 中速/ctrl慢速/shift高速
    // 其中 云台自由模式下仅支持超低速和慢速
    const float speed[4] = {15000, 30000, 50000, 6000};
    if(rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift && (ControlMode==ControlMode_ConvertCube || ControlMode==ControlMode_FetchCube)) {   //超低速
        chassis_cmd_send.vy = (rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s) * speed[3];
        chassis_cmd_send.vx = (rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d) * speed[3];
    }else if((!rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift) && !(ControlMode==ControlMode_ConvertCube || ControlMode==ControlMode_FetchCube)) {   //wasd-中速
        chassis_cmd_send.vy = (rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s) * speed[1];
        chassis_cmd_send.vx = (rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d) * speed[1];
    }else if((rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift) || (ControlMode==ControlMode_ConvertCube || ControlMode==ControlMode_FetchCube)) {   //ctrl+wasd-慢速
        chassis_cmd_send.vy = (rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s) * speed[0];
        chassis_cmd_send.vx = (rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d) * speed[0];
    }else if((!rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift)) {   //ctrl+wasd-快速
        chassis_cmd_send.vy = (rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s) * speed[2];
        chassis_cmd_send.vx = (rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d) * speed[2];
    }

    /* q&e */
        // q'e / shift+q'e控制底盘旋转
        if(!rc_data->key[KEY_PRESS].ctrl){
            chassis_cmd_send.wz = -(rc_data->key[KEY_PRESS].q - rc_data->key[KEY_PRESS].e)  * 10000.0f; // 底盘旋转
        }
        // ctrl+q'e低速旋转
        if(rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)
            chassis_cmd_send.wz = -(rc_data->key[KEY_PRESS].q - rc_data->key[KEY_PRESS].e)  * 3000.0f; // 底盘旋转
        // ctrl+shift+q'e控制臂臂旋转    
        if(rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift)   
            arm_cmd_send.Rotation_yaw = (rc_data->key[KEY_PRESS_WITH_SHIFT].e - rc_data->key[KEY_PRESS_WITH_SHIFT].q) * 0.045;
    /* r&f */
    // ctrl+r'f控制臂竖直移动
    if(!rc_data->key[KEY_PRESS].shift)
        arm_cmd_send.Position_z = (rc_data->key[KEY_PRESS_WITH_CTRL].r - rc_data->key[KEY_PRESS_WITH_CTRL].f) * 0.3;
    // shift+r'f控制臂 行走姿态 | 收回肚子
    if(!rc_data->key[KEY_PRESS].ctrl){ 
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].r){
            UI_cmd_send.arm_selected_mode = Arm_walk_state;
            if(rc_data->mouse.press_l){
                arm_cmd_send.auto_mode = Arm_walk_state;//shift+r+左键确认 - 臂行走姿态
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
            }
        }
        else if(rc_data->key[KEY_PRESS_WITH_SHIFT].f){
            UI_cmd_send.arm_selected_mode = Recycle_arm_in;
            if(rc_data->mouse.press_l){
                // ControlMode = ControlMode_Move;
                arm_cmd_send.auto_mode = Recycle_arm_in;//shift+f+左键确认 - 臂收回肚子
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
            }
        }
            
    }
    
    /* 鼠标平移 */
        //鼠标平移控制云台
        // 示例输入数据
        // uint8_t a = 20;
        // static float input[100] = {0};
        // static float output[60];
        // static uint8_t x_cnt = 0;
        // // 滤波器参数，alpha值介于0和1之间
        // float alpha = 0.1;
        // if(x_cnt<a)    {input[x_cnt] = rc_data->mouse.x;x_cnt++;}
        // else{
        //     // low_pass_filter(input, output, 30, alpha);
        //     int16_t sum = 0;
        //     for(int i = 0; i < a; i++) sum+=input[i];
        //     x_cnt = 0;
        //     gimbal_cmd_send.pitch = -rc_data->mouse.y / 20.0;
        //     gimbal_cmd_send.yaw = sum/a * 0.1;
        //     VAL_LIMIT(gimbal_cmd_send.yaw, -6, 6);
        // }
        gimbal_cmd_send.yaw = rc_data->mouse.x_average * 0.01;
        gimbal_cmd_send.pitch = -rc_data->mouse.y / 20.0;

            
        
    //ctrl + g 切换行走模式正反向
        static uint8_t ControlMode_ReverseMove_switch = 0;
        if(rc_data->key[KEY_PRESS_WITH_CTRL].g && !rc_data->key[KEY_PRESS].shift && !ControlMode_ReverseMove_switch)
        {
            if(ControlMode == ControlMode_Move) ControlMode = ControlMode_ReverseMove;
            else if(ControlMode == ControlMode_ReverseMove) ControlMode = ControlMode_Move;
            ControlMode_ReverseMove_switch = 1;
        }else if(!(rc_data->key[KEY_PRESS_WITH_CTRL].g && !rc_data->key[KEY_PRESS].shift))
        {
            ControlMode_ReverseMove_switch = 0;
        }
        
    /* shift+gzx控制取中心资源岛矿 z让推杆取 x让臂臂取 */
        // shift+z | shift+x 推杆取中心资源岛左侧|中间的金矿
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].z && !rc_data->key[KEY_PRESS].ctrl){
            UI_cmd_send.valve_selected_mode = 2;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_FetchCube;
                airpump_cmd_send.airvalve_mode = AIRVALVE_LEFT_CUBE;    
            }
        }else if(rc_data->key[KEY_PRESS_WITH_SHIFT].x && !rc_data->key[KEY_PRESS].ctrl){
            UI_cmd_send.valve_selected_mode = 1;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_FetchCube;
                airpump_cmd_send.airvalve_mode = AIRVALVE_MIDDLE_CUBE;
            }
        }
        // shift+g 臂臂取中心资源岛右侧的金矿
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].g && !rc_data->key[KEY_PRESS].ctrl){
            UI_cmd_send.arm_selected_mode = Arm_get_goldcube_right;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_FetchCube;
                arm_cmd_send.auto_mode = Arm_get_goldcube_right;
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
            }
        }
    /* ctrl+zx控制吸盘旋转*/
        if(rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift && rc_data->key[KEY_PRESS].z)  
            arm_cmd_send.call.sucker_call = Arm_sucker_clockwise_rotation;
        else if(rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift && rc_data->key[KEY_PRESS].x)  
            arm_cmd_send.call.sucker_call = Arm_sucker_anticlockwise_rotation;
    /* v取银矿 */
        // v 取左侧银矿
        if(rc_data->key[KEY_PRESS].v && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            UI_cmd_send.arm_selected_mode = Arm_get_silvercube_left;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_FetchCube;
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
                arm_cmd_send.auto_mode = Arm_get_silvercube_left;
            }
        }else
        // ctrl+v 取右侧银矿
        if(rc_data->key[KEY_PRESS].v && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            UI_cmd_send.arm_selected_mode = Arm_get_silvercube_right;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_FetchCube;
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
                arm_cmd_send.auto_mode = Arm_get_silvercube_right;
            }
        }else
        // shift+v 取中间银矿
        if(rc_data->key[KEY_PRESS].v && !rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
            UI_cmd_send.arm_selected_mode = Arm_get_silvercube_mid;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_FetchCube;
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
                arm_cmd_send.auto_mode = Arm_get_silvercube_mid;
            }
        }
    /* b */
        // b 蜂鸣器鸣叫
        if(rc_data->key[KEY_PRESS].b && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            buzzer_one_note(0xff,0.1);
        }
        // ctrl+b 从下矿仓取矿
        if(rc_data->key[KEY_PRESS_WITH_CTRL].b&& !rc_data->key[KEY_PRESS].shift){
            UI_cmd_send.arm_selected_mode = Arm_fetch_cube_from_warehouse_down;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_ConvertCube;
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
                arm_cmd_send.auto_mode =Arm_fetch_cube_from_warehouse_down;
            }
        }
        // shift+b 从上矿仓取矿
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].b && !rc_data->key[KEY_PRESS].ctrl){
            UI_cmd_send.arm_selected_mode = Arm_fetch_cube_from_warehouse_up;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_ConvertCube;
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
                arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse_up;
            }
        }
        // ctrl+shift+b 重置z轴标定
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].b && rc_data->key[KEY_PRESS].ctrl){
            arm_cmd_send.call.reset_z_init_flag = 1;
        }else{
            arm_cmd_send.call.reset_z_init_flag = 0;
        }

    // r键 强制刷新/切换UI
    if(rc_data->key[KEY_PRESS].r && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)
        UI_cmd_send.UI_refresh_request = 1;
    else
        UI_cmd_send.UI_refresh_request = 0;
    // ctrl+shift+f 开关臂气泵
        static uint8_t pump_switch_state = 0;
        if(rc_data->key[KEY_PRESS_WITH_CTRL].f && rc_data->key[KEY_PRESS].shift && !(pump_switch_state & 0x01)){
            (airpump_arm_state&0x01) ? (airpump_arm_state&=~0x01) : (airpump_arm_state|=0x01);
            pump_switch_state |= 0x01;
        }else if(!(rc_data->key[KEY_PRESS_WITH_CTRL].f && rc_data->key[KEY_PRESS].shift)){
            pump_switch_state &= ~0x01;
        }
    // ctrl+shift+g开关推杆气泵
        if((rc_data->key[KEY_PRESS_WITH_SHIFT].g && rc_data->key[KEY_PRESS].ctrl) && !(pump_switch_state & 0x02)){
            (airpump_linear_state&0x01) ? (airpump_linear_state&=~0x01) : (airpump_linear_state|=0x01);
            pump_switch_state |= 0x02;
        }else if(!(rc_data->key[KEY_PRESS_WITH_SHIFT].g && rc_data->key[KEY_PRESS].ctrl)){
            pump_switch_state &= ~0x02;
        }
    // ctrl+shift+z 强制终止推杆自动模式
    if(rc_data->key[KEY_PRESS].z && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
        airpump_cmd_send.halt_force_call = 1;
    }else{
        airpump_cmd_send.halt_force_call = 0;
    }
    // ctrl+shift+x 强制终止臂臂自动模式
    if(rc_data->key[KEY_PRESS].x && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
        arm_cmd_send.call.halt_force_call = 1;
    }else{
        arm_cmd_send.call.halt_force_call = 0;
    }
    // C键
        //shift+c，修正臂解算方式（默认为右解)
        if(rc_data->key[KEY_PRESS].c && !rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift)
            arm_cmd_send.call.optimize_signal |= 0x01;
        else
            arm_cmd_send.call.optimize_signal &= ~0x01;
        //ctrl+c，(兑矿模式下)切换自定义控制器模式,
        if(rc_data->key[KEY_PRESS].c && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)
            arm_cmd_send.call.switch_custom_controller_mode_call = 1;
        else
            arm_cmd_send.call.switch_custom_controller_mode_call = 0;

        // (+左键)ctrl+shift+c 修正混合roll偏差角
        if(rc_data->key[KEY_PRESS].c && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift && rc_data->mouse.press_l)
            arm_cmd_send.call.optimize_signal |= 0x02;
        else
            arm_cmd_send.call.optimize_signal &= ~0x02;
        
            
    /* 鼠标右键 */
        //ctrl+shift+右键 云台小陀螺
        if(rc_data->mouse.press_r && (rc_data->key[KEY_PRESS].shift && rc_data->key[KEY_PRESS].ctrl))
            gimbal_cmd_send.gimbal_mode = GIMBAL_RESET_WITH_ROTATE;
    /* 鼠标左键 */
        // ctrl+鼠标左键 Z轴缓慢下降
        if(rc_data->mouse.press_l && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)
            arm_cmd_send.call.z_slowly_down_call = 1;
        else arm_cmd_send.call.z_slowly_down_call = 0;
        // z/x+鼠标左键 暂停(继续)自动动作
        //z+鼠标左键 暂停(继续)推杆自动动作
        if((rc_data->key[KEY_PRESS].z && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)){
            UI_cmd_send.valve_temp_halt_selected = 1;
            if(rc_data->mouse.press_l)
                airpump_cmd_send.halt_temp_call = 1;
        }else   airpump_cmd_send.halt_temp_call = 0;
        // x+鼠标左键 暂停(继续)臂臂自动动作
        if(rc_data->key[KEY_PRESS].x && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            UI_cmd_send.arm_temp_halt_selected = 1;
            if(rc_data->mouse.press_l)
                arm_cmd_send.call.halt_temp_call = 1;
        }else   arm_cmd_send.call.halt_temp_call = 0;
        // c+鼠标左键 切换为兑矿模式
        if(rc_data->key[KEY_PRESS].c && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            UI_cmd_send.arm_selected_mode = Arm_ConvertCube;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_ConvertCube;
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
                arm_cmd_send.auto_mode = Arm_ConvertCube;
            }
        }
        // f+鼠标左键 复位为取地矿姿势
        if((rc_data->key[KEY_PRESS].f && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)){
            UI_cmd_send.arm_selected_mode = Arm_fetch_gronded_cube;
            if(rc_data->mouse.press_l){
                ControlMode = ControlMode_FetchCube;
                arm_cmd_send.auto_mode = Arm_fetch_gronded_cube;
                arm_cmd_send.contro_mode = ARM_AUTO_MODE;
            }
        }
        // g+鼠标左键 切换为行走模式
        if((rc_data->mouse.press_l && rc_data->key[KEY_PRESS].g && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift))
            ControlMode = (gimbal_data_recv.yaw_motor<=90&&gimbal_data_recv.yaw_motor>=-90) ? ControlMode_Move : ControlMode_ReverseMove;
        // 仅鼠标左键（除wasdqe) 切换为取矿模式
        if(rc_data->mouse.press_l && !rc_data->mouse.press_r && !(rc_data->key[KEY_PRESS].keys & ~0x00cf))
            ControlMode = ControlMode_FetchCube;
        // 鼠标右键+鼠标左键 底盘小陀螺
        static uint8_t chassis_rotate_switch = 0;
        if(rc_data->mouse.press_r && rc_data->mouse.press_l && !(rc_data->key[KEY_PRESS].shift && rc_data->key[KEY_PRESS].ctrl) && !chassis_rotate_switch){
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        }  
}
static void MouseKeyControlSet(){
    // ctrl+shift+r 重置C板
    if(Reset_Param(rc_data->key[KEY_PRESS].r && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift))
        robot_state = ROBOT_STOP;
    else{
        static bool debug_mode = 0;
        static uint8_t switch_flag = 0;
        EdgeType edge = detect_edge(&switch_flag, rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift && rc_data->key[KEY_PRESS].v && rc_data->mouse.press_l);
        if(edge == EDGE_RISING) debug_mode = !debug_mode;
        
        if(debug_mode){
            MouseKeyControlSet_Debug();
        }else{
            MouseKeyControlSet_Normal();
        }
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
    // 双下为急停
    if (robot_state == ROBOT_STOP ||  (switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right))
        || (rc_data->rc.switch_left==0&&rc_data->rc.switch_right==0)//遥控器离线时，也触发急停
        || ((rc_data->rc.switch_left&0xfC)||(rc_data->rc.switch_right&0xFC))) //遥控器数据异常时触发急停
    {
        if(robot_state == ROBOT_READY)  LOGERROR("[CMD] emergency stop!");
        robot_state                   = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        arm_cmd_send.contro_mode      = ARM_ZERO_FORCE;
        airpump_cmd_send.airpump_mode = AIRPUMP_STOP;
        airpump_cmd_send.airvalve_mode= AIRVALVE_STOP;
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data->rc.switch_right) || robot_state == ROBOT_READY) {
        if(robot_state == ROBOT_STOP)   LOGINFO("[CMD] reinstate, robot ready");
        robot_state = ROBOT_READY;
        airpump_cmd_send.airpump_mode &= ~AIRPUMP_STOP;

    }
}

// 监测红外测距模块状态
static void redlight_detect(){
    if(HAL_GPIO_ReadPin(redLight_detect_GPIO_Port,redLight_detect_Pin)==GPIO_PIN_SET){
        redlight_flag=1;    // 1为有物体遮挡
    }else redlight_flag = 0;
}
// 任务间消息处理
static void MessageCenterDispose(){
    static _RobotControlMode LastControlMode;
    static uint16_t chassis_delay_time = 0;
    if(LastControlMode != ControlMode){
        if((ControlMode == ControlMode_ReverseMove && LastControlMode == ControlMode_Move)
            || (ControlMode == ControlMode_Move && LastControlMode == ControlMode_ReverseMove)){
            chassis_delay_time = 500;
            gimbal_cmd_send.yaw += 180;
        }
    }
    if(chassis_delay_time){
        chassis_delay_time--;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    }
    LastControlMode = ControlMode;

    chassis_cmd_send.gimbal_pitch_imu = gimbal_data_recv.pitch_imu;
    chassis_cmd_send.robotControlMode = ControlMode;
    chassis_cmd_send.offset_angle = gimbal_data_recv.yaw_motor; //计算底盘偏差角度
    chassis_cmd_send.arm_height = arm_data_recv.current_data.height;
    gimbal_cmd_send.arm_big_yaw_offset = arm_data_recv.current_data.big_yaw_angle;   //云台因大Yaw移动产生的偏转角
    airpump_cmd_send.arm_to_airvalve = arm_data_recv.arm_to_airvalve; //臂任务对夹爪状态的控制

    UI_cmd_send.arm_current_data = arm_data_recv.current_data;
    UI_cmd_send.arm_target_data = arm_data_recv.target_data;
    UI_cmd_send.arm_mode = arm_data_recv.auto_mode_state;
    UI_cmd_send.valve_mode = air_data_recv.airvalve_mode;
    {
        if(!UI_cmd_send.arm_selected_mode){
            UI_cmd_send.arm_selected_mode = arm_data_recv.arm_auto_mode_selecting;
            UI_cmd_send.arm_selected_mode_state = 0;
        }else{
            UI_cmd_send.arm_selected_mode_state = 1;
        }
        if(!UI_cmd_send.valve_selected_mode){
            UI_cmd_send.valve_selected_mode = air_data_recv.air_auto_mode_selecting;
            UI_cmd_send.valve_selected_mode_state = 0;
        }else{
            UI_cmd_send.valve_selected_mode_state = 1;
        }
    }
    UI_cmd_send.Contro_mode = (robot_state == ROBOT_STOP ? 0 : (int)ControlMode);
    UI_cmd_send.rc_connection_mode_t = RemoteControlIsOnline();
    UI_cmd_send.vision_connection_mode_t = vision_connection_state;
    UI_cmd_send.custom_contro_connection_mode_t = custom_contro_connection_state;
    UI_cmd_send.gimbal_offset_angle = (chassis_cmd_send.offset_angle>0?chassis_cmd_send.offset_angle:chassis_cmd_send.offset_angle+360);
    UI_cmd_send.pump_arm_mode_t = airpump_arm_state;
    UI_cmd_send.pump_valve_mode_t = airpump_linear_state;
}
// 额外操作，因遥控器键位不足，使用左拨杆为[上]时的拨轮状态进行额外的状态控制
static void extra_Control(){
    //todo: 为了避免该函数覆盖掉键鼠操作，此函数必须在键鼠操作前调用
    //      但其未来仍可能触发奇怪的错误，最好能改进
    //左拨杆为[上]，右拨杆为[下]时，用拨轮控制气泵开关
    if(switch_is_down(rc_data->rc.switch_right)){
        static uint8_t dial_switch1_state = 0;
        //拨轮向上时，开关臂气泵
        if(dial_flag==1 && !(dial_switch1_state & 0x01)){
            (airpump_cmd_send.airpump_mode&0x01) ? (airpump_cmd_send.airpump_mode&=~0x01) : (airpump_cmd_send.airpump_mode|=0x01);
            (airpump_arm_state&0x01) ? (airpump_arm_state&=~0x01) : (airpump_arm_state|=0x01);
            dial_switch1_state |= 0x01;
        }else if(!(dial_flag==1)){
            dial_switch1_state &= ~0x01;
        }
        //拨轮向下时，开关推杆气泵
        if(dial_flag==-1 && !(dial_switch1_state & 0x02)){
            (airpump_cmd_send.airpump_mode&0x02) ? (airpump_cmd_send.airpump_mode&=~0x02) : (airpump_cmd_send.airpump_mode|=0x02);
            (airpump_linear_state&0x01) ? (airpump_linear_state&=~0x01) : (airpump_linear_state|=0x01);
            dial_switch1_state |= 0x02;
        }else if(!(dial_flag==-1)){
            dial_switch1_state &= ~0x02;
        }
    }

    //左拨杆为[上]，右拨杆为[中]时，用拨轮控制臂结算办法
    /* 该功能源于臂安装的编码器与实际角度存在1:2的比值，上电位置不同会导致整个臂臂的结算异常，需手动修正 */
    if(switch_is_mid(rc_data->rc.switch_right)){
        //拨轮向上时，修正yaw偏向（默认为右偏)(上升沿触发)
        if(dial_flag==1)
            arm_cmd_send.call.optimize_signal |= 0x01;
        else
            arm_cmd_send.call.optimize_signal &= ~0x01;
        //拨轮向下时，修正混合roll偏差角
        if(dial_flag==-1)
            arm_cmd_send.call.optimize_signal |= 0x02;
        else
            arm_cmd_send.call.optimize_signal &= ~0x02;
    }
}
void RobotActive()
{
    while(zero_output_init_flag!=1)
    {
        if((switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right)) || rc_data->key[KEY_PRESS].r || vision_rc_data->key[KEY_PRESS].r)
        {
            zero_output_init_flag=1;
            BuzzerStop();
        }
        else{
            osDelay(1);
            BuzzerPlay(RoboMaster_You);
        }


    }
}
void RobotCMDSubMessage()
{
    SubGetMessage(gimbal_data_sub,&gimbal_data_recv);
    SubGetMessage(arm_data_sub,&arm_data_recv);
    SubGetMessage(air_data_sub,&air_data_recv);
}
void RobotCMDParamPretreatment()
{
    airpump_cmd_send.airvalve_mode = 0;arm_cmd_send.auto_mode = 0;gimbal_cmd_send.yaw=0;gimbal_cmd_send.pitch=0;arm_cmd_send.call.sucker_call=0;
    airpump_cmd_send.init_call = 0;arm_cmd_send.call.reset_init_flag = 0;
    /* 检测拨轮状态，因为这东西很容易坏，不能直接判断当前值 */
    static int16_t dial_cnt = 0;
    if(rc_data->rc.dial >= 660 && dial_cnt>=0) dial_cnt++;
    else if(rc_data->rc.dial <=-660 && dial_cnt<=0) dial_cnt--;
    else    dial_cnt=0;
    if(dial_cnt>20)    dial_flag=-1;   //down
    else if(dial_cnt<-20)  dial_flag=1;//up
    else dial_flag = 0;

    // 监测红外测距模块状态
    redlight_detect();

    //如果遥控器离线，切换为图传链路
    if(!RemoteControlIsOnline())
    {
            memcpy(rc_data,vision_rc_data,sizeof(RC_ctrl_t));
            rc_data->rc.switch_left = 1;
            rc_data->rc.switch_right = 2;
    }
}
void RobotCMDGenerateCommand()
{
    gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE; // 云台使能
    arm_cmd_send.contro_mode = ARM_BASE_CONTRO_MODE;
    /*
        云台始终为陀螺仪模式 todo:加入陀螺仪掉线的备选方案
        底盘模式决定优先级：紧急制动 > 其他应用要求 > 控制模式确定的底盘自由模式 > 用户直接设定(目前仅小陀螺) > 控制模式确定的底盘移动模式
        控制模式仅能由用户设置，默认为正向跟随模式，改变模式的可能情况有"应用自动模式同时切换""直接指定"
            tips:控制模式在自动模式结束后不会自动切回，todo:加入UI提醒操作手    
    */
    

    if(switch_is_up(rc_data->rc.switch_left)){
        // 左侧开关为上时，使用键鼠操作
        MouseKeyControlSet();
        extra_Control(); //由于遥控器键位不足，占用四个左侧拨杆为[上]时的操作
    }else{ 
        //否则使用遥控器操作
        RemoteControlSet();
    }

    if(chassis_cmd_send.chassis_mode != CHASSIS_ROTATE)
    {
        if(ControlMode == ControlMode_Move)
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; //底盘跟随云台
        else if(ControlMode == ControlMode_ReverseMove)
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW_REVERSE; //底盘跟随云台（换头
    }
    if(ControlMode == ControlMode_ConvertCube)
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW_CONVERTMODE;  //云台自由移动
    if(ControlMode == ControlMode_FetchCube)
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;  //云台自由移动
    if(arm_data_recv.auto_mode_state & 0x01)
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;  //云台自由移动
        
    if(ControlMode == ControlMode_ConvertCube && arm_cmd_send.auto_mode == 0){
        arm_cmd_send.contro_mode = ARM_CUSTOM_CONTRO;
        arm_cmd_send.auto_mode = 0;
    }
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    MessageCenterDispose(); // 消息处理，统合&转发各任务信息

}
void RobotCMDPubMessage()
{
    PubPushMessage(arm_cmd_pub, (void *)&arm_cmd_send);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(airpump_cmd_pub, (void *)&airpump_cmd_send);
    // PubPushMessage(UI_reality_pub, (void *)&UI_reality_send);

    if(UI_cmd_pub->first_subs->temp_size==0){
        PubPushMessage(UI_cmd_pub, (void *)&UI_cmd_send);
        memset(&UI_cmd_send.debug, 0, sizeof(UI_debug_param));
    }
    
}
void RobotCMDDebugInterface()
{   
    static uint8_t edge_detect[12] = {0};
    static uint8_t switch_num = 0;
    airpump_cmd_send.in_debug_call = 0;
    airpump_cmd_send.out_debug_call = 0;
    arm_cmd_send.debug.auto_mode_record_pause_call = 0;
    arm_cmd_send.debug.auto_mode_record_start_call = 0;
    arm_cmd_send.debug.assorted_joint_enable_call = 0;
    arm_cmd_send.debug.assorted_joint_disable_call = 0;
    arm_cmd_send.debug.apply_delay_call = 0;
    arm_cmd_send.debug.jaw_loose_call = 0;
    arm_cmd_send.debug.jaw_tighten_call = 0;
    EdgeType edge_1 = detect_edge(&edge_detect[0], dial_flag==1);
    if(edge_1 == EDGE_RISING){
        if((switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right))){
            arm_cmd_send.debug.auto_mode_record_pause_call = 1;
        }else if((switch_is_mid(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right))){
            switch_num = (switch_num+1)%9;
            switch(switch_num){
                case 1:DM_board_LEDSet(0xff0000);debug_switch_arm_auto_mode=Arm_get_goldcube_right;break;// 红色
                case 2:DM_board_LEDSet(0xffa308);debug_switch_arm_auto_mode=Arm_fetch_cube_from_warehouse_down;break;// 橙色
                case 3:DM_board_LEDSet(0xffee46);debug_switch_arm_auto_mode=Arm_fetch_cube_from_warehouse_up;break;// 黄色
                case 4:DM_board_LEDSet(0x444444);debug_switch_arm_auto_mode=Arm_get_silvercube_left;break;// 白色
                case 5:DM_board_LEDSet(0x43c9b0);debug_switch_arm_auto_mode=Arm_get_silvercube_mid;break;// 青色
                case 6:DM_board_LEDSet(0x2b74ce);debug_switch_arm_auto_mode=Arm_get_silvercube_right;break;// 蓝色
                case 7:DM_board_LEDSet(0xc586b6);debug_switch_arm_auto_mode=Recycle_arm_in;break;// 粉色
                case 8:DM_board_LEDSet(0x111111);debug_switch_arm_auto_mode=0;break;//
                default: DM_board_LEDSet(0x000000);debug_switch_arm_auto_mode=0; //灭
            }
            arm_cmd_send.debug.selected_auto_mode_id = debug_switch_arm_auto_mode;
        }
    }

    static uint8_t flash_write_success_flag = 0;
    EdgeType edge_2 = detect_edge(&edge_detect[1], (switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right)) && dial_flag==-1);
    if(edge_2 == EDGE_RISING){
        if(switch_num == 8){
            buzzer_one_note(0x50, 0.1);
            flashRefresh();
            flash_write_success_flag = 1;
            buzzer_one_note(0xf0, 0.1);
            // __set_FAULTMASK(1);
            // NVIC_SystemReset();
        }else{
            arm_cmd_send.debug.auto_mode_record_start_call = 1;
            airpump_cmd_send.in_debug_call = 1;
        }
    }

    EdgeType edge_key[9];
    edge_key[0] = detect_edge(&edge_detect[2], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_l1 < -650);
    edge_key[1] = detect_edge(&edge_detect[3], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_l1 > 650);
    edge_key[2] = detect_edge(&edge_detect[4], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_l_ > 650);
    edge_key[3] = detect_edge(&edge_detect[5], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_l_ < -650);
    edge_key[4] = detect_edge(&edge_detect[6], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_r_ > 650);
    edge_key[5] = detect_edge(&edge_detect[7], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_r1 < -650);
    // edge_key[6] = detect_edge(&edge_detect[8], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_r_ > 650);
    // edge_key[7] = detect_edge(&edge_detect[9], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_r1 > 650);
    edge_key[8] = detect_edge(&edge_detect[10], switch_is_down(rc_data->rc.switch_right) && rc_data->rc.rocker_r_ < -650);
    edge_key[9] = detect_edge(&edge_detect[11], switch_is_mid(rc_data->rc.switch_right));
    arm_cmd_send.debug.arm_pump_off_call = edge_key[0] == EDGE_RISING;
    arm_cmd_send.debug.arm_pump_on_call = edge_key[1] == EDGE_RISING;
    arm_cmd_send.debug.valve_pump_on_call = edge_key[2] == EDGE_RISING;
    arm_cmd_send.debug.valve_pump_off_call = edge_key[3] == EDGE_RISING;
    arm_cmd_send.debug.jaw_loose_call = edge_key[4] == EDGE_RISING;
    arm_cmd_send.debug.jaw_tighten_call = edge_key[5] == EDGE_RISING;
    // arm_cmd_send.debug.assorted_joint_enable_call = edge_key[7] == EDGE_RISING;
    // arm_cmd_send.debug.assorted_joint_disable_call = edge_key[8] == EDGE_RISING;
    arm_cmd_send.debug.apply_delay_call = edge_key[9] == EDGE_RISING;

    // if(edge_key[6] == EDGE_RISING)  airpump_cmd_send.out_debug_call = 1;

    if(switch_num ==0){
        if(switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right) && dial_flag==1){
            Reset_Param(1);
            robot_state = ROBOT_STOP;
            EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
        }
    }

    if(flash_write_success_flag)
        Reset_Param(1);
} 