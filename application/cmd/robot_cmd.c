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
#include "buzzer.h"
#include "led.h"
#include "tool.h"
#include "vision_rec.h"
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
static ramp_t chassis_speed_ramp[3][2]; // l/m/h x/y 低中高速
static int32_t ramp_feriod = 2000; //斜坡周期
void RobotCMDInit()
{
    rc_data     = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    robot_state = ROBOT_STOP;                // 启动时机器人为停止状态，避免一上电创死场地
    zero_output_init_flag = 1;               // 启动时置为1，当遥控器双下过一次后复位

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    arm_cmd_pub     = PubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    airpump_cmd_pub = PubRegister("airpump_cmd",sizeof(Airpump_Cmd_Data_s));
    UI_reality_pub = PubRegister("UI_reality",sizeof(UI_reality_Data_s));
    UI_cmd_pub     = PubRegister("UI", sizeof(UI_data_t));

    gimbal_data_sub = SubRegister("gimbal_data", sizeof(Gimbal_Data_s));
    arm_data_sub = SubRegister("arm_data", sizeof(Arm_Data_s));
    air_data_sub = SubRegister("air_data", sizeof(Airpump_Data_s));

    USART_Init_Config_s vision_usart_conf = {
        .module_callback = vision_recv_callback,
        .checkout_callback = NULL,
        .recv_buff_size  = USART_RXBUFF_LIMIT-1,
        .usart_handle    = &huart1, // 达妙板子的原理图写USART3，但实际管脚对应的是UART1
    };
    vision_rc_data = malloc(sizeof(RC_ctrl_t));
    memset(vision_rc_data,0,sizeof(RC_ctrl_t));
    vision_usart       = USARTRegister(&vision_usart_conf);// 图传串口

    memset(&arm_cmd_send, 0, sizeof(arm_cmd_send));

    ramp_init(&chassis_speed_ramp[0][0],ramp_feriod);ramp_init(&chassis_speed_ramp[1][0],ramp_feriod);ramp_init(&chassis_speed_ramp[2][0],ramp_feriod);
    ramp_init(&chassis_speed_ramp[0][1],ramp_feriod);ramp_init(&chassis_speed_ramp[1][1],ramp_feriod);ramp_init(&chassis_speed_ramp[2][1],ramp_feriod);
}

static void arm_auto_mode_select(){
    switch(arm_auto_mode_id){
        case 0:
            break;// 0:无自动模式，白色 
        case 1: //红
            arm_cmd_send.auto_mode = Reset_arm_cmd_param_flag;break; //行走模式
        case 2: //橙
            arm_cmd_send.auto_mode = Recycle_arm_in;break; //收臂臂回肚子
        case 3: //黄
            arm_cmd_send.auto_mode = Recycle_arm_out;break; //臂臂伸出肚子
        case 4: //绿
            arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse1;break; //从矿仓中取矿 W1
        case 5: //青
            arm_cmd_send.auto_mode = Arm_get_goldcube_right;break;// 金矿右 GL
        case 6: //蓝
            arm_cmd_send.auto_mode = Arm_get_silvercube_left;break;// 银矿左 SL
        case 7: //紫
            arm_cmd_send.auto_mode = Arm_get_silvercube_mid;break;// 银矿中   SM
        case 8: //粉
            arm_cmd_send.auto_mode = Arm_get_silvercube_right;break; //银矿右    SR
        case 9:
            arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse2;break; //从矿仓2中取矿
    }
}
static void chassis_auto_mode_select(){
    switch(chassis_auto_mod_id){
            case 1:airpump_cmd_send.airvalve_mode=AIRVALVE_LEFT_CUBE;break;// 1:气推杆取左侧矿，红色
            case 2:airpump_cmd_send.airvalve_mode=AIRVALVE_MIDDLE_CUBE;break;// 2:气推杆取中间矿，橙色 
            case 3:chassis_cmd_send.special_func_flag=CHASSIS_SLOPE_MOVE_L;break; // 靠墙向左平移
            case 4:chassis_cmd_send.special_func_flag=CHASSIS_SLOPE_MOVE_R;break; // 靠墙向右平移
            default:break;
        }
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{    
    //双下同时拨轮向上保持三秒复位C板
    static uint16_t reset_press_cnt = 0;
    if (switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right) && dial_flag==1){
        reset_press_cnt++;
        if(reset_press_cnt > 1000){
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
    }else reset_press_cnt=0;
    // 左侧开关状态为[下],控制底盘云台
    if (switch_is_down(rc_data->rc.switch_left)) {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_CONTRO; // 底盘使能
        gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE; // 云台使能
        arm_cmd_send.contro_mode = ARM_FIXED; // 臂臂固定
        
        // 左摇杆控制底盘移动，右摇杆控制云台
        gimbal_cmd_send.yaw = rc_data->rc.rocker_r_/660.0*0.5; // 云台电机旋转
        gimbal_cmd_send.pitch = rc_data->rc.rocker_r1/660.0*0.2; // 云台pitch
        chassis_cmd_send.vx = 40.0f * rc_data->rc.rocker_l_; // 底盘水平方向
        chassis_cmd_send.vy = 40.0f * rc_data->rc.rocker_l1; // 底盘竖值方向
        
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
        }
    }

    // 左侧开关为[中]，控制底盘臂臂 or 单独控制臂臂
    if (switch_is_mid(rc_data->rc.switch_left)) {
        static uint8_t arm_contro_mode_id = 1; //标志控制方式
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_CONTRO; // 底盘使能
        gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE; // 云台使能
        switch(arm_contro_mode_id){
            case Arm_Control_with_Chassis:
                arm_cmd_send.contro_mode = ARM_POSE_CONTRO_MODE; // 臂臂位置控制
                // 左摇杆控制底盘移动，右摇杆控制臂的旋转和竖直平移
                chassis_cmd_send.vx = 10.0f * rc_data->rc.rocker_l_; // 底盘水平方向
                chassis_cmd_send.vy = 10.0f * rc_data->rc.rocker_l1; // 底盘竖值方向
                arm_cmd_send.Position_z = 0.5 * rc_data->rc.rocker_r1 / 660.0; // 臂竖直平移
                arm_cmd_send.Rotation_yaw = 0.06 * rc_data->rc.rocker_r_ / 660.0; // 臂的旋转
                break;
            case Arm_Control_only_Arm:
                arm_cmd_send.contro_mode = ARM_REFER_MODE;
                // 左摇杆控制臂臂前后移动，右摇杆控制roll&pitch
                arm_cmd_send.Translation_x = -rc_data->rc.rocker_l1 / 660.0 *0.0002;
                arm_cmd_send.Translation_y = -rc_data->rc.rocker_l_/ 660.0 *0.0002;
                arm_cmd_send.Roatation_Horizontal = rc_data->rc.rocker_r_/ 660.0 *0.08 ;
                arm_cmd_send.Roatation_Vertical  = -rc_data->rc.rocker_r1/ 660.0 *0.08 ;
                break;
            case Arm_Control_by_Custom_controller:
                arm_cmd_send.contro_mode = ARM_CUSTOM_CONTRO;
                // 左摇杆控制底盘移动，右摇杆控制底盘旋转和云台pitch
                chassis_cmd_send.wz = 10.0f * rc_data->rc.rocker_r_; // 底盘旋转
                gimbal_cmd_send.pitch = rc_data->rc.rocker_r1/660.0*0.2; // 云台pitch
                chassis_cmd_send.vx = 40.0f * rc_data->rc.rocker_l_; // 底盘水平方向
                chassis_cmd_send.vy = 40.0f * rc_data->rc.rocker_l1; // 底盘竖值方向
                break;
            case Arm_Control_by_vision:
                arm_cmd_send.contro_mode = ARM_VISION_CONTRO;
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
            if(dial_flag == 1)  arm_cmd_send.sucker_state = Arm_sucker_clockwise_rotation;
            else if(dial_flag == -1)  arm_cmd_send.sucker_state = Arm_sucker_anticlockwise_rotation;
            else arm_cmd_send.sucker_state = Arm_sucker_none_rotation;
        }
        
        // 右侧开关为[下]，波轮上发送视觉识别信号 | 拨轮下臂臂操作方式
        if(switch_is_down(rc_data->rc.switch_right)){
            // 拨轮向上，发送视觉识别信号（需处于视觉控制模式）
            if(dial_flag == 1){
                arm_cmd_send.vision_signal = 1;
            }else{
                arm_cmd_send.vision_signal = 0;
            }

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
                switch_flag |= 0x01;
                arm_auto_mode_id++;
                if(arm_auto_mode_id>9)   arm_auto_mode_id=1;
            }else if(!(dial_flag == 1)){
                switch_flag &= ~0x01;
            }
            //LED标示模式
            // switch(arm_auto_mode_id){
            //     case 1:DM_board_LEDSet(0xff4546);break;// 红色
            //     case 2:DM_board_LEDSet(0xffa308);break;// 橙色 
            //     case 3:DM_board_LEDSet(0xffee46);break;// 黄色
            //     case 4:DM_board_LEDSet(0xa8ff2d);break;// 绿色
            //     case 5:DM_board_LEDSet(0x45fff2);break;// 青色
            //     case 6:DM_board_LEDSet(0x0000ff);break;// 蓝色
            //     case 7:DM_board_LEDSet(0xf029f6);break;// 紫色
            //     case 8:DM_board_LEDSet(0xff648e);break;// 粉色
            //     case 9:DM_board_LEDSet(0xffffff);break;// 白色
            // }
            // 拨轮向下时应用模式
            if(dial_flag == -1){
                arm_auto_mode_select();
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
    robot_state = ROBOT_READY;
    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_CONTRO; // 底盘使能
    gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE; // 云台使能
    arm_cmd_send.contro_mode = ARM_CONTROL_BY_KEYBOARD; // 臂臂位置控制

    

    // wasd 控制底盘全向移动 中速/ctrl慢速/shift高速
        static float current_speed_x = 0;
        static float current_speed_y = 0;
            
        if(!rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift) {   //wasd-中速
            chassis_cmd_send.vy = current_speed_y + ((rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s)* 15000 - current_speed_y) * ramp_calc(&chassis_speed_ramp[1][1]);
            chassis_cmd_send.vx = current_speed_x + ((rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d)* 15000 - current_speed_x) * ramp_calc(&chassis_speed_ramp[1][0]);
            ramp_init(&chassis_speed_ramp[0][0],ramp_feriod);ramp_init(&chassis_speed_ramp[2][0],ramp_feriod);
            ramp_init(&chassis_speed_ramp[0][1],ramp_feriod);ramp_init(&chassis_speed_ramp[2][1],ramp_feriod);
        }
        else if(rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift) {   //ctrl+wasd-慢速
            chassis_cmd_send.vy = current_speed_y + ((rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s)* 5000 - current_speed_y) * ramp_calc(&chassis_speed_ramp[0][1]);
            chassis_cmd_send.vx = current_speed_x + ((rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d)* 5000 - current_speed_x) * ramp_calc(&chassis_speed_ramp[0][0]);
            ramp_init(&chassis_speed_ramp[1][0],ramp_feriod);ramp_init(&chassis_speed_ramp[2][0],ramp_feriod);
            ramp_init(&chassis_speed_ramp[1][1],ramp_feriod);ramp_init(&chassis_speed_ramp[2][1],ramp_feriod);
        }
        else if(!rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift) {   //shift+wasd-高速
            chassis_cmd_send.vy = current_speed_y + ((rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s)* 25000 - current_speed_y) * ramp_calc(&chassis_speed_ramp[2][1]);
            chassis_cmd_send.vx = current_speed_x + ((rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d)* 25000 - current_speed_x) * ramp_calc(&chassis_speed_ramp[2][0]);
            ramp_init(&chassis_speed_ramp[0][0],ramp_feriod);ramp_init(&chassis_speed_ramp[1][0],ramp_feriod);
            ramp_init(&chassis_speed_ramp[0][1],ramp_feriod);ramp_init(&chassis_speed_ramp[1][1],ramp_feriod);
        }
        
        current_speed_x = chassis_cmd_send.vx;
        current_speed_y = chassis_cmd_send.vy;

        // ctrl+shift+w/s 微调推杆吸盘
        if(rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
            airpump_cmd_send.sucker_offset_angle += 0.01*(rc_data->key[KEY_PRESS_WITH_CTRL].w-rc_data->key[KEY_PRESS_WITH_CTRL].s);
        }
        

    /* q&e */
        // q'e / shift+q'e控制底盘旋转
        if(!rc_data->key[KEY_PRESS].ctrl){
            chassis_cmd_send.wz = -(rc_data->key[KEY_PRESS].q - rc_data->key[KEY_PRESS].e)  * 2500.0f; // 底盘旋转
        }
        // ctrl+q'e控制底盘贴墙移动
        if(rc_data->key[KEY_PRESS].q && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)
                chassis_cmd_send.special_func_flag = CHASSIS_SLOPE_MOVE_L;
        else if(rc_data->key[KEY_PRESS].e && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)
                chassis_cmd_send.special_func_flag = CHASSIS_SLOPE_MOVE_R;
        else chassis_cmd_send.special_func_flag = 0;
        // ctrl+shift+q'e控制臂臂旋转    
        if(rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift)   
            arm_cmd_send.Rotation_yaw = (rc_data->key[KEY_PRESS_WITH_SHIFT].e - rc_data->key[KEY_PRESS_WITH_SHIFT].q) * 0.06;
    /* r&f */
    // ctrl+r'f控制 臂复位 | 收回肚子
        if(rc_data->key[KEY_PRESS_WITH_CTRL].r)
            arm_cmd_send.auto_mode = Reset_arm_cmd_param_flag;//ctrl+r 臂复位
        else if(rc_data->key[KEY_PRESS_WITH_CTRL].f)   
            arm_cmd_send.auto_mode = Recycle_arm_in;//ctrl+f 臂收回肚子
        // shift+r'f控制臂竖直移动
        arm_cmd_send.Position_z = (rc_data->key[KEY_PRESS_WITH_SHIFT].r - rc_data->key[KEY_PRESS_WITH_SHIFT].f) * 0.5;

    /* 鼠标平移 */
        // 鼠标平移控制底盘旋转和云台pitch(按住右键时)
        if(rc_data->mouse.press_r){
            chassis_cmd_send.wz = rc_data->mouse.x  * 50.0f;
            gimbal_cmd_send.pitch = -rc_data->mouse.y / 50.0;
        }else{
        //鼠标平移控制云台(未按住右键时)
            gimbal_cmd_send.yaw = rc_data->mouse.x * 0.01;
            gimbal_cmd_send.pitch = -rc_data->mouse.y / 50.0;
        }
    /* 鼠标右键 */
        // 鼠标右键-云台跟随Yaw,且鼠标平移控制底盘旋转
        if(rc_data->mouse.press_r && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            gimbal_cmd_send.gimbal_mode = GIMBAL_FOLLOW_YAW;
        }else
        // ctrl+鼠标右键-云台跟随大Yaw+大Yaw复位
        if(rc_data->mouse.press_r && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            arm_cmd_send.auto_mode = Arm_big_yaw_reset;
        }else
        // shift+鼠标右键-云台跟随大Yaw+视觉兑矿模式
        if(rc_data->mouse.press_r && !rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            arm_cmd_send.contro_mode = Arm_Control_by_vision;
        }
        // ctrl+shitf+鼠标右键-云台小陀螺后复位
        if(rc_data->mouse.press_r && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
            gimbal_cmd_send.gimbal_mode = GIMBAL_RESET_WITH_ROTATE;
        }
    //ctrl + g开关臂气泵
        static uint8_t pump_switch_state = 0;
        if(rc_data->key[KEY_PRESS_WITH_CTRL].g && !rc_data->key[KEY_PRESS].shift && !(pump_switch_state & 0x01)){
            (airpump_cmd_send.airpump_mode&0x01) ? (airpump_cmd_send.airpump_mode&=~0x01) : (airpump_cmd_send.airpump_mode|=0x01);
            pump_switch_state |= 0x01;
        }else if(!(rc_data->key[KEY_PRESS_WITH_CTRL].g)){
            pump_switch_state &= ~0x01;
        }
    // ctrl+shift+g开关推杆气泵
        if((rc_data->key[KEY_PRESS_WITH_SHIFT].g && rc_data->key[KEY_PRESS].ctrl) && !(pump_switch_state & 0x02)){
            (airpump_cmd_send.airpump_mode&0x02) ? (airpump_cmd_send.airpump_mode&=~0x02) : (airpump_cmd_send.airpump_mode|=0x02);
            pump_switch_state |= 0x02;
        }else if(!(rc_data->key[KEY_PRESS_WITH_SHIFT].g && rc_data->key[KEY_PRESS].ctrl)){
            pump_switch_state &= ~0x02;
        }
    /* shift+gzx控制取中心资源岛矿 z让推杆取 x让臂臂取 */
        // shift+g | shift+z 推杆取中心资源岛左侧|中间的金矿
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].g && !rc_data->key[KEY_PRESS].ctrl){
            airpump_cmd_send.airvalve_mode = AIRVALVE_LEFT_CUBE;
        }else if(rc_data->key[KEY_PRESS_WITH_SHIFT].z){
            airpump_cmd_send.airvalve_mode = AIRVALVE_MIDDLE_CUBE;
        }
        // shift+x 臂臂取中心资源岛右侧的金矿
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].x && !rc_data->key[KEY_PRESS].ctrl){
            arm_cmd_send.auto_mode = Arm_get_goldcube_right;
        }
    /* ctrl+zx控制吸盘旋转*/
        if(rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift && rc_data->key[KEY_PRESS].z)  
            arm_cmd_send.sucker_state = Arm_sucker_clockwise_rotation;
        else if(rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift && rc_data->key[KEY_PRESS].x)  
            arm_cmd_send.sucker_state = Arm_sucker_anticlockwise_rotation;
    /* v取银矿 */
        // v 取左侧银矿
        if(rc_data->key[KEY_PRESS].v && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            arm_cmd_send.auto_mode = Arm_get_silvercube_left;
        }else
        // ctrl+v 取中间银矿
        if(rc_data->key[KEY_PRESS].v && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            arm_cmd_send.auto_mode = Arm_get_silvercube_mid;
        }else
        // shift+v 取右边银矿
        if(rc_data->key[KEY_PRESS].v && !rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
            arm_cmd_send.auto_mode = Arm_get_silvercube_right;
        }
    /* b应用所选自动模式 */
        // b 蜂鸣器鸣叫
        if(rc_data->key[KEY_PRESS].b && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift){
            buzzer_one_note(0xff,0.1);
        }
        // ctrl+b 从下矿仓取矿
        if(rc_data->key[KEY_PRESS_WITH_CTRL].b&& !rc_data->key[KEY_PRESS].shift){
            arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse2;
        }
        // shift+b 从上矿仓取矿
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].b && !rc_data->key[KEY_PRESS].ctrl){
            arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse1;
        }

        if(rc_data->key[KEY_PRESS].b && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
          airpump_cmd_send.tmp_flag = 1;
    }else{
        airpump_cmd_send.tmp_flag = 0;
    }
    // r键 强制刷新/切换UI
    if(rc_data->key[KEY_PRESS].r && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)
        UI_cmd_send.UI_refresh_request = 1;
    else
        UI_cmd_send.UI_refresh_request = 0;
    // ctrl+shift+r 重置C板
    static uint16_t reset_press_cnt = 0;
    if(rc_data->key[KEY_PRESS].r && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
        reset_press_cnt++;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        arm_cmd_send.contro_mode      = ARM_ZERO_FORCE;
        airpump_cmd_send.airpump_mode = AIRPUMP_STOP;
        airpump_cmd_send.airvalve_mode= AIRVALVE_STOP;
        if(reset_press_cnt > 1000){
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
    }else if(!(rc_data->key[KEY_PRESS].r && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift)){
        reset_press_cnt=0;
    }
    // ctrl+shift+z 强制终止推杆自动模式
    if(rc_data->key[KEY_PRESS].z && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
        airpump_cmd_send.halt_force_call = 1;
    }else{
        airpump_cmd_send.halt_force_call = 0;
    }
    // ctrl+shift+x 强制终止臂臂自动模式
    if(rc_data->key[KEY_PRESS].z && rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift){
        arm_cmd_send.halt_force_call = 1;
    }else{
        arm_cmd_send.halt_force_call = 0;
    }
    // C键 兑矿姿态|切换编码器识别位姿|切换臂结算方式
        static uint8_t keyboard_switch_arm_param_state = 0;
        // c-兑矿模式
        static uint8_t switch_pose_contro_mode_flag = 0;

        if(rc_data->key[KEY_PRESS].c && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift && !(switch_pose_contro_mode_flag & 0x01)){
            arm_cmd_send.convert_flag = arm_cmd_send.convert_flag&1 ? 0 : 1;
            switch_pose_contro_mode_flag |= 0x01;
        }else if(!(rc_data->key[KEY_PRESS].c && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)){
            switch_pose_contro_mode_flag &= ~0x01;
        }
        //shift+c，修正yaw偏向（默认为右偏)
        if(rc_data->key[KEY_PRESS].c && !rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift && !(keyboard_switch_arm_param_state & 0x01)){
            (arm_cmd_send.optimize_signal&0x01) ? (arm_cmd_send.optimize_signal&=~0x01) : (arm_cmd_send.optimize_signal|=0x01);
            keyboard_switch_arm_param_state |= 0x01;
        }else if(!(rc_data->key[KEY_PRESS].c && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)){
            keyboard_switch_arm_param_state &= ~0x01;
        }
        //ctrl+c，修正混合roll偏差角
        if(rc_data->key[KEY_PRESS].c && rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift && !(keyboard_switch_arm_param_state & 0x02)){
            (arm_cmd_send.optimize_signal&0x02) ? (arm_cmd_send.optimize_signal&=~0x02) : (arm_cmd_send.optimize_signal|=0x02);
            keyboard_switch_arm_param_state |= 0x02;
        }else if(!(rc_data->key[KEY_PRESS].c && !rc_data->key[KEY_PRESS].ctrl && rc_data->key[KEY_PRESS].shift)){
            keyboard_switch_arm_param_state &= ~0x02;
        }
    /* 鼠标左键 */
        // ctrl+鼠标左键-自动取地面矿
        if(rc_data->mouse.press_l && rc_data->key[KEY_PRESS].ctrl){
            arm_cmd_send.auto_mode = Fetch_gronded_cube;
        }
        // ctrl+shift+鼠标左键-视觉兑矿识别一次
        if(rc_data->mouse.press_l && rc_data->key[KEY_PRESS].ctrl){
            arm_cmd_send.vision_signal = 1;
        }else arm_cmd_send.vision_signal = 0;
        // z/x+鼠标左键 暂停(继续)自动动作
        static uint8_t switch_arm_auto_mode_on,switch_valve_auto_mode_on,switch_auto_mode_on_flag;
        static uint8_t current_arm_auto_mode_on,current_valve_auto_mode_on;
        current_arm_auto_mode_on = arm_cmd_send.auto_mode==0 ? current_arm_auto_mode_on : arm_cmd_send.auto_mode; 
        current_valve_auto_mode_on = airpump_cmd_send.airvalve_mode==0 ? current_valve_auto_mode_on : airpump_cmd_send.airvalve_mode; 
        //z+鼠标左键 暂停(继续)推杆自动动作
        if((rc_data->mouse.press_l && rc_data->key[KEY_PRESS].z && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift) && !(switch_auto_mode_on_flag&0x01)){
            if(switch_valve_auto_mode_on && switch_valve_auto_mode_on != current_valve_auto_mode_on){
                airpump_cmd_send.airvalve_mode = switch_valve_auto_mode_on;
                switch_valve_auto_mode_on=0;
            }else{
                switch_valve_auto_mode_on = current_valve_auto_mode_on;
                airpump_cmd_send.halt_temp_call = 1;
            }
            switch_auto_mode_on_flag |= 0x01;
        }else if(!(rc_data->mouse.press_l && rc_data->key[KEY_PRESS].z && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)){
            airpump_cmd_send.halt_temp_call = 0;
            switch_auto_mode_on_flag &= ~0x01;
        }
        // x+鼠标左键 暂停(继续)臂臂自动动作
        if((rc_data->mouse.press_l && rc_data->key[KEY_PRESS].x && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift) && !(switch_auto_mode_on_flag&0x02)){
            if(switch_arm_auto_mode_on && switch_arm_auto_mode_on == current_arm_auto_mode_on){
                arm_cmd_send.auto_mode = switch_valve_auto_mode_on;
                switch_arm_auto_mode_on=0;
            }else{
                switch_arm_auto_mode_on = current_arm_auto_mode_on;
                arm_cmd_send.halt_temp_call = 1;
            }
            switch_auto_mode_on_flag |= 0x02;
        }else if(!(rc_data->mouse.press_l && rc_data->key[KEY_PRESS].x && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)){
            arm_cmd_send.halt_temp_call = 0;
            switch_auto_mode_on_flag &= ~0x02;
        }
        // f+鼠标左键 复位为取地矿姿势
        if((rc_data->mouse.press_l && rc_data->key[KEY_PRESS].f && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)){
            arm_cmd_send.auto_mode = Fetch_gronded_cube_P;
        }
        // g+鼠标左键 复位为前伸姿势
        if((rc_data->mouse.press_l && rc_data->key[KEY_PRESS].g && !rc_data->key[KEY_PRESS].ctrl && !rc_data->key[KEY_PRESS].shift)){
            arm_cmd_send.auto_mode = Arm_forward_P;
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
    chassis_cmd_send.offset_angle = - arm_data_recv.big_yaw_angle + gimbal_data_recv.yaw; //计算底盘偏差角度
    gimbal_cmd_send.arm_big_yaw_offset = -arm_data_recv.big_yaw_angle;   //云台因大Yaw移动产生的偏转角
    // UI_reality_send.arm_auto_mode_id = arm_auto_mode_id;
    // UI_reality_send.chassis_auto_mod_id = chassis_auto_mod_id;
    airpump_cmd_send.arm_to_airvalve = arm_data_recv.arm_to_airvalve; //臂任务对夹爪状态的控制
    airpump_cmd_send.arm_to_airpump = arm_data_recv.arm_to_airpump; //臂臂任务控制气泵，优先级比键鼠控制高 

    UI_cmd_send.pump_one_mode_t = air_data_recv.pump_state & 0x01 ? 1 : 0;
    UI_cmd_send.pump_two_mode_t = air_data_recv.pump_state & 0x02 ? 1 : 0;
    
}
// 额外操作，因遥控器键位不足，使用左拨杆为[上]时的拨轮状态进行额外的状态控制
static void extra_Control(){
    //todo: 为了避免该函数覆盖掉键鼠操作，此函数必须在键鼠操作前调用
    //      但其未来仍可能触发奇怪的错误，最好能改进

    //左拨杆为[上]，右拨杆为[上]时，用拨轮强制重置重要关节零点
    if(switch_is_up(rc_data->rc.switch_right)){
        static uint16_t reset_yaw_press_cnt = 0;
        if(dial_flag==1){
            reset_yaw_press_cnt++;
            gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            arm_cmd_send.contro_mode      = ARM_ZERO_FORCE;
            airpump_cmd_send.airpump_mode = AIRPUMP_STOP;
            airpump_cmd_send.airvalve_mode= AIRVALVE_STOP;
            if(reset_yaw_press_cnt > 1000){
                static uint8_t tx_buf_PDA04_reset_1[] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                CANTransmit_once(&hfdcan3,
                                0x28,
                                tx_buf_PDA04_reset_1, 1000);
                static uint8_t tx_buf_PDA04_reset_2[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                CANTransmit_once(&hfdcan3,
                                0x28,
                                tx_buf_PDA04_reset_2, 1000);
                osDelay(500);
                __set_FAULTMASK(1);
                NVIC_SystemReset();
            }
        }else if(!(dial_flag==1)){
            reset_yaw_press_cnt=0;
        }

            arm_cmd_send.contro_mode = ARM_POSE_CONTRO_MODE_1; // 臂臂位置控制模式1：基于原点坐标系平移
            arm_cmd_send.Translation_x = -rc_data->rc.rocker_l1 / 660.0 *0.0002;
            arm_cmd_send.Translation_y = -rc_data->rc.rocker_l_/ 660.0 *0.0002;
            arm_cmd_send.Roatation_Horizontal = -rc_data->rc.rocker_r_/ 660.0 *0.2 ;
            arm_cmd_send.Roatation_Vertical  = -rc_data->rc.rocker_r1/ 660.0 *0.08 ;
    
    }

    //左拨杆为[上]，右拨杆为[下]时，用拨轮控制气泵开关
    if(switch_is_down(rc_data->rc.switch_right)){
        static uint8_t dial_switch1_state = 0;
        //拨轮向上时，开关臂气泵
        if(dial_flag==1 && !(dial_switch1_state & 0x01)){
            (airpump_cmd_send.airpump_mode&0x01) ? (airpump_cmd_send.airpump_mode&=~0x01) : (airpump_cmd_send.airpump_mode|=0x01);
            dial_switch1_state |= 0x01;
        }else if(!(dial_flag==1)){
            dial_switch1_state &= ~0x01;
        }
        //拨轮向下时，开关推杆气泵
        if(dial_flag==-1 && !(dial_switch1_state & 0x02)){
            (airpump_cmd_send.airpump_mode&0x02) ? (airpump_cmd_send.airpump_mode&=~0x02) : (airpump_cmd_send.airpump_mode|=0x02);
            dial_switch1_state |= 0x02;
        }else if(!(dial_flag==-1)){
            dial_switch1_state &= ~0x02;
        }
    }

    //左拨杆为[上]，右拨杆为[中]时，用拨轮控制臂结算办法
    /* 该功能源于臂安装的编码器与实际角度存在1:2的比值，上电位置不同会导致整个臂臂的结算异常，需手动修正 */
    if(switch_is_mid(rc_data->rc.switch_right)){
        static uint8_t dial_switch2_state = 0;
        //拨轮向上时，修正yaw偏向（默认为右偏)
        if(dial_flag==1 && !(dial_switch2_state & 0x01)){
            (arm_cmd_send.optimize_signal&0x01) ? (arm_cmd_send.optimize_signal&=~0x01) : (arm_cmd_send.optimize_signal|=0x01);
            dial_switch2_state |= 0x01;
        }else if(!(dial_flag==1)){
            dial_switch2_state &= ~0x01;
        }
        //拨轮向下时，修正混合roll偏差角
        if(dial_flag==-1 && !(dial_switch2_state & 0x02)){
            (arm_cmd_send.optimize_signal&0x02) ? (arm_cmd_send.optimize_signal&=~0x02) : (arm_cmd_send.optimize_signal|=0x02);
            dial_switch2_state |= 0x02;
        }else if(!(dial_flag==-1)){
            dial_switch2_state &= ~0x02;
        }

            arm_cmd_send.contro_mode = ARM_POSE_CONTRO_MODE_0; // 臂臂位置控制模式0：基于原点坐标系平移
            arm_cmd_send.Translation_x = -rc_data->rc.rocker_l1 / 660.0 *0.0002;
            arm_cmd_send.Translation_y = -rc_data->rc.rocker_l_/ 660.0 *0.0002;
            arm_cmd_send.Roatation_Horizontal = -rc_data->rc.rocker_r_/ 660.0 *0.2 ;
            arm_cmd_send.Roatation_Vertical  = -rc_data->rc.rocker_r1/ 660.0 *0.08 ;
    }
}
// 状态检测，处理当前各外设状态的数据并置入UI发送结构体中
static void state_detection_for_UI(){
    // UI_cmd_send.pump_one_mode_t = 0;    //后续由气压计数据判断
    // UI_cmd_send.pump_two_mode_t = 0;    //后续由气压计数据判断
    
}
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    if(zero_output_init_flag==1)
    {
        if((switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right)))
            zero_output_init_flag=0;
        return;
    }

    SubGetMessage(gimbal_data_sub,&gimbal_data_recv);
    SubGetMessage(arm_data_sub,&arm_data_recv);
    SubGetMessage(air_data_sub,&air_data_recv);
    airpump_cmd_send.airvalve_mode = 0;arm_cmd_send.auto_mode = 0;gimbal_cmd_send.yaw=0;gimbal_cmd_send.pitch=0;chassis_cmd_send.special_func_flag=0,arm_cmd_send.sucker_state=0;

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
    static uint8_t switch_to_vision_rc_flag = 0;
    if(rc_data->rc.switch_left==0 && rc_data->rc.switch_right==0)
    {
        if(vision_rc_data->key->ctrl && vision_rc_data->key->shift && vision_rc_data->key->b)
            switch_to_vision_rc_flag |= 0x01;
        if(switch_to_vision_rc_flag & 0X01){
            memcpy(rc_data,vision_rc_data,sizeof(RC_ctrl_t));
            rc_data->rc.switch_left = 1;
            rc_data->rc.switch_right = 2;
        }
    }else{
        switch_to_vision_rc_flag &= ~0x01;
    }
    
    // 左侧开关为上时，使用键鼠操作
    if(switch_is_up(rc_data->rc.switch_left)){
        MouseKeySet();
        extra_Control(); //由于遥控器键位不足，占用四个左侧拨杆为[上]时的操作
    }else{ //否则使用遥控器操作
        RemoteControlSet();
    }

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    MessageCenterDispose(); // 消息处理，统合&转发各任务信息
    
    state_detection_for_UI();   // 状态检测，处理当前各外设状态的数据并置入UI发送结构体中

    PubPushMessage(arm_cmd_pub, (void *)&arm_cmd_send);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(airpump_cmd_pub, (void *)&airpump_cmd_send);
    PubPushMessage(UI_reality_pub, (void *)&UI_reality_send);
    PubPushMessage(UI_cmd_pub, (void *)&UI_cmd_send);
}
