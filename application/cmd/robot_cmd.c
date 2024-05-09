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
static Publisher_t *UI_reality_pub;// 物理UI信息发布者

static Subscriber_t *gimbal_data_sub;                   // 用于接收云台的数据信息
static Subscriber_t *arm_data_sub;                   // 用于接收臂臂的数据信息

static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 发送给云台应用的信息
static Arm_Cmd_Data_s arm_cmd_send; // 发送给机械臂应用的信息
static Airpump_Cmd_Data_s airpump_cmd_send; // 发送给气阀/气泵控制应用的信息
static UI_reality_Data_s UI_reality_send;  // 发送给物理UI的信息

static Gimbal_Data_s     gimbal_data_recv;// 云台发布的信息
static Arm_Data_s     arm_data_recv;// 云台发布的信息

static RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回

static Robot_Status_e robot_state; // 机器人整体工作状态

static int8_t dial_flag = 0;    //遥控器拨轮状态标志
static uint8_t redlight_flag = 0;//红外测距模块状态标志

/* 自动模式id */
static uint8_t arm_auto_mode_id = 1;    //臂臂自动模式
static uint8_t chassis_auto_mod_id = 0; //底盘自动模式

void RobotCMDInit()
{
    rc_data     = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    robot_state = ROBOT_STOP;                // 启动时机器人为停止状态，避免一上电创死场地

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    arm_cmd_pub     = PubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    airpump_cmd_pub = PubRegister("airpump_cmd",sizeof(Airpump_Cmd_Data_s));
    UI_reality_pub = PubRegister("UI_reality",sizeof(UI_reality_Data_s));

    gimbal_data_sub = SubRegister("gimbal_data", sizeof(Gimbal_Data_s));
    arm_data_sub = SubRegister("arm_data", sizeof(Arm_Data_s));

    memset(&arm_cmd_send, 0, sizeof(arm_cmd_send));
}

static void arm_auto_mode_select(){
    switch(arm_auto_mode_id){
        case 1:arm_cmd_send.auto_mode = Reset_arm_cmd_param_flag;break;// 1:重置臂臂状态，红色
        case 2:arm_cmd_send.auto_mode = Recycle_arm_out;break;// 2:臂臂回收到肚子外, 橙色
        case 4:arm_cmd_send.auto_mode = Arm_get_goldcube_mid;break;// 4:取中间矿,绿色
        case 5:arm_cmd_send.auto_mode = Arm_get_goldcube_right;break;// 5:取左侧矿,青色
        case 6:arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse1;break; //6:从矿仓1中取矿，蓝色
        case 7:arm_cmd_send.auto_mode = Arm_fetch_cube_from_warehouse1;break; //7:从矿仓2中取矿，紫色
    }
}
static void chassis_auto_mode_select(){
    switch(chassis_auto_mod_id){
            case 1:airpump_cmd_send.airvalve_mode=AIRVALVE_LEFT_CUBE;break;// 1:气推杆取左侧矿，红色
            case 2:airpump_cmd_send.airvalve_mode=AIRVALVE_MIDDLE_CUBE;break;// 2:气推杆取中间矿，橙色 
            case 3:gimbal_cmd_send.gimbal_mode=GIMBAL_RESET;break;// 3:云台复位，黄色
        }
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
        arm_cmd_send.contro_mode = ARM_FIXED; // 臂臂固定
        
        // 左摇杆控制底盘移动，右摇杆控制云台
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
            static uint8_t switch_flag = 0;
            if(dial_flag == 1 && !(switch_flag & 0x01)){
                switch_flag |= 0x01;
                chassis_auto_mod_id++;
                if(chassis_auto_mod_id>3)   chassis_auto_mod_id=1;
            }else if(!(dial_flag == 1)){
                switch_flag &= ~0x01;
            }
            //LED标示模式
            switch(chassis_auto_mod_id){
                case 1:DM_board_LEDSet(0xff0000);break;// 红色
                case 2:DM_board_LEDSet(0xffa308);break;// 橙色
                case 3:DM_board_LEDSet(0xffee46);break;// 黄色
            }
            // 波轮向下时应用所选模式
            airpump_cmd_send.airvalve_mode=0;
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
                chassis_cmd_send.vx = 10.0f * (float)rc_data->rc.rocker_l_; // 底盘水平方向
                chassis_cmd_send.vy = 10.0f * (float)rc_data->rc.rocker_l1; // 底盘竖值方向
                arm_cmd_send.Position_z = 0.5 * rc_data->rc.rocker_r1 / 660.0; // 臂竖直平移
                arm_cmd_send.Rotation_yaw = 0.06 * rc_data->rc.rocker_r_ / 660.0; // 臂的旋转
                break;
            case Arm_Control_only_Arm:
                arm_cmd_send.contro_mode = ARM_REFER_MODE;
                // 左摇杆控制臂臂前后移动，右摇杆控制roll&pitch
                arm_cmd_send.Translation_x = -rc_data->rc.rocker_l1 / 660.0 *0.0002;
                arm_cmd_send.Translation_y = rc_data->rc.rocker_l_/ 660.0 *0.0002;
                arm_cmd_send.Roatation_Horizontal = rc_data->rc.rocker_r_/ 660.0 *0.08 ;
                arm_cmd_send.Roatation_Vertical  = -rc_data->rc.rocker_r1/ 660.0 *0.08 ;
                break;
            case Arm_Control_by_Custom_controller:
                arm_cmd_send.contro_mode = ARM_CUSTOM_CONTRO;
                // 左摇杆控制底盘移动，右摇杆控制底盘旋转和云台pitch
                chassis_cmd_send.wz = 10.0f * (float)rc_data->rc.rocker_r_; // 底盘旋转
                gimbal_cmd_send.pitch = rc_data->rc.rocker_r1/660.0*0.2; // 云台pitch
                chassis_cmd_send.vx = 40.0f * (float)rc_data->rc.rocker_l_; // 底盘水平方向
                chassis_cmd_send.vy = 40.0f * (float)rc_data->rc.rocker_l1; // 底盘竖值方向
                break;
            case Arm_Control_by_vision:
                arm_cmd_send.contro_mode = ARM_VISION_CONTRO;
                // 左摇杆控制底盘移动，右摇杆控制底盘旋转和云台pitch
                chassis_cmd_send.wz = 10.0f * (float)rc_data->rc.rocker_r_; // 底盘旋转
                gimbal_cmd_send.pitch = rc_data->rc.rocker_r1/660.0*0.2; // 云台pitch
                chassis_cmd_send.vx = 40.0f * (float)rc_data->rc.rocker_l_; // 底盘水平方向
                chassis_cmd_send.vy = 40.0f * (float)rc_data->rc.rocker_l1; // 底盘竖值方向
                break;
            case Fetch_gronded_cube:
                arm_cmd_send.contro_mode = ARM_POSE_CONTRO_MODE; // 臂臂位置控制
                // 左摇杆控制底盘移动，右摇杆控制底盘旋转和臂的竖直平移
                chassis_cmd_send.vx = 10.0f * (float)rc_data->rc.rocker_l_; // 底盘水平方向
                chassis_cmd_send.vy = 10.0f * (float)rc_data->rc.rocker_l1; // 底盘竖值方向
                arm_cmd_send.Position_z = 0.5 * rc_data->rc.rocker_r1 / 660.0; // 臂竖直平移
                chassis_cmd_send.wz = 10.0f * (float)rc_data->rc.rocker_r_; // 底盘旋转
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
        
        // 右侧开关为[下]，波轮上控制气泵开关 | 拨轮下臂臂操作方式
        if(switch_is_down(rc_data->rc.switch_right)){
            // 拨轮向上，发送视觉识别信号（需处于视觉控制模式）
            if(dial_flag == 1){
                arm_cmd_send.vision_signal = 1;
            }else{
                arm_cmd_send.vision_signal = 0;
            }

            // 拨轮向下时，切换臂臂操作方式
            static uint8_t switch_flag = 0;
            if(dial_flag == -1 && !switch_flag){
                arm_contro_mode_id++;
                if(arm_contro_mode_id > 4) arm_contro_mode_id=1;
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
            static uint8_t switch_flag = 0;
            if(dial_flag == 1 && !(switch_flag & 0x01)){
                switch_flag |= 0x01;
                arm_auto_mode_id++;
                if(arm_auto_mode_id>6)   arm_auto_mode_id=1;
            }else if(!(dial_flag == 1)){
                switch_flag &= ~0x01;
            }
            //LED标示模式
            switch(arm_auto_mode_id){
                case 1:DM_board_LEDSet(0xff4546);break;// 红色
                case 2:DM_board_LEDSet(0xffa308);break;// 橙色 
                case 3:DM_board_LEDSet(0xffee46);break;// 黄色
                case 4:DM_board_LEDSet(0xa8ff2d);break;// 绿色
                case 5:DM_board_LEDSet(0x45fff2);break;// 青色
                case 6:DM_board_LEDSet(0x0000ff);break;// 蓝色
                case 7:DM_board_LEDSet(0xf029f6);break;// 紫色
                case 8:DM_board_LEDSet(0xff648e);break;// 粉色
                case 9:DM_board_LEDSet(0xffffff);break;// 白色
            }
            // 拨轮向下时应用模式
            if(dial_flag == -1)
                arm_auto_mode_select();
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
    arm_cmd_send.contro_mode = ARM_CONTROL_BY_KEYBOARD; // 臂臂位置控制
    airpump_cmd_send.airvalve_mode = 0;arm_cmd_send.auto_mode = 0;

    // wasd控制底盘全向移动
    chassis_cmd_send.vx = (rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s) * 300; // 系数待测
    chassis_cmd_send.vy = (rc_data->key[KEY_PRESS].s - rc_data->key[KEY_PRESS].d) * 300;
    // q'e控制底盘旋转
    chassis_cmd_send.wz = (rc_data->key[KEY_PRESS].q - rc_data->key[KEY_PRESS].e)  * 2500.0f; // 底盘旋转
    // shift+q'e控制臂臂旋转
    arm_cmd_send.Rotation_yaw = (rc_data->key[KEY_PRESS_WITH_SHIFT].e - rc_data->key[KEY_PRESS_WITH_SHIFT].q) * 0.06;
    // shift+r'f控制臂竖直移动
    arm_cmd_send.Position_z = (rc_data->key[KEY_PRESS_WITH_SHIFT].r - rc_data->key[KEY_PRESS_WITH_SHIFT].f) * 0.5;
    // 鼠标平移控制云台(未按住ctrl时)
    if(rc_data->key[KEY_PRESS].ctrl != KEY_PRESS){
        gimbal_cmd_send.yaw += (float)rc_data->mouse.x / 660 * 10; // 系数待测
        gimbal_cmd_send.pitch += (float)rc_data->mouse.y / 660 * 10;
    }else{
    // 鼠标平移控制底盘旋转和云台pitch
        chassis_cmd_send.wz += (float)rc_data->mouse.x / 660 * 1000.0f; // 系数待测
        gimbal_cmd_send.pitch += (float)rc_data->mouse.y / 660 * 10;
    }
    // 鼠标右键-云台复位
    if(rc_data->mouse.press_l==KEY_PRESS){
        gimbal_cmd_send.gimbal_mode = GIMBAL_RESET;
    }
    // ctrl+鼠标右键-云台复位+大Yaw归中
    if(rc_data->mouse.press_l==KEY_PRESS && rc_data->key[KEY_PRESS].ctrl){
        gimbal_cmd_send.gimbal_mode = GIMBAL_RESET;
        arm_cmd_send.auto_mode = Arm_big_yaw_reset;
    }
    // ctrl+鼠标左键-自动取地面矿
    if(rc_data->mouse.press_l==KEY_PRESS && rc_data->key[KEY_PRESS].ctrl){
        arm_cmd_send.auto_mode = Arm_big_yaw_reset;
    }
    // C键设置底盘速度
    switch (rc_data->key_count[KEY_PRESS][Key_C] % 4) 
    {
        case 0:chassis_cmd_send.chassis_speed_buff = 40;break;        
        case 1:chassis_cmd_send.chassis_speed_buff = 60;break;
        case 2:chassis_cmd_send.chassis_speed_buff = 80;break;
        default:chassis_cmd_send.chassis_speed_buff = 100;break;
    }

    /* g开关气泵 */ 
        // ctrl+g是臂气泵 
        static uint8_t pump_switch_state = 0;
        if(rc_data->key[KEY_PRESS_WITH_CTRL].g && !(pump_switch_state & 0x01)){
            AIRPUMP_SWITCH(airpump_cmd_send.airpump_mode,AIRPUMP_ARM_OPEN);
            pump_switch_state |= 0x01;
        }else if(!(rc_data->key[KEY_PRESS_WITH_CTRL].g)){
            pump_switch_state &= ~0x01;
        }
        // shift+g是推杆气泵
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].g && !(pump_switch_state & 0x02)){
            AIRPUMP_SWITCH(airpump_cmd_send.airpump_mode,AIRPUMP_LINEAR_OPEN);
            pump_switch_state |= 0x02;
        }else if(!(rc_data->key[KEY_PRESS_WITH_SHIFT].g)){
            pump_switch_state &= ~0x02;
        }

    /* zx控制取中心资源岛矿 z让推杆取 x让臂臂取 */
        // ctrl+z | shift+z 推杆取中心资源岛左侧|中间的金矿
        if(rc_data->key[KEY_PRESS_WITH_CTRL].z){
            airpump_cmd_send.airvalve_mode = AIRVALVE_LEFT_CUBE;
        }else if(rc_data->key[KEY_PRESS_WITH_SHIFT].z){
            airpump_cmd_send.airvalve_mode = AIRVALVE_MIDDLE_CUBE;
        }
        // ctrl+x | shift+x 臂臂取中心资源岛中间|右侧的金矿
        if(rc_data->key[KEY_PRESS_WITH_CTRL].x){
            arm_cmd_send.auto_mode = Arm_get_goldcube_mid;
        }else if(rc_data->key[KEY_PRESS_WITH_SHIFT].x){
            arm_cmd_send.auto_mode = Arm_get_goldcube_right;
        }
    /* v切换自动模式ID */
        // ctrl+v 切换臂自动模式ID
        static uint8_t switch_auto_mode_id = 0;
        if(rc_data->key[KEY_PRESS_WITH_CTRL].v && !(switch_auto_mode_id&0x01)){
            arm_auto_mode_id++;
            if(arm_auto_mode_id>6)   arm_auto_mode_id=1;
            switch_auto_mode_id |= 0x01;
        }else if(!(rc_data->key[KEY_PRESS_WITH_CTRL].v)){
            switch_auto_mode_id &= ~0x01;
        }
        // shift+v 切换推杆自动模式ID
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].v && !(switch_auto_mode_id&0x02)){
            chassis_auto_mod_id++;
            if(chassis_auto_mod_id>2)   chassis_auto_mod_id=1;
            switch_auto_mode_id |= 0x02;
        }else if(!(rc_data->key[KEY_PRESS_WITH_SHIFT].v)){
            switch_auto_mode_id &= ~0x02;
        }
    /* b应用所选自动模式 */
        // ctrl+b 应用当前臂自动模式
        if(rc_data->key[KEY_PRESS_WITH_CTRL].b){
            arm_auto_mode_select();
        }
        // shift+v 应用当前推杆自动模式
        if(rc_data->key[KEY_PRESS_WITH_SHIFT].b){
            chassis_auto_mode_select();
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
    if (robot_state == ROBOT_STOP ||  (switch_is_down(rc_data->rc.switch_left) && switch_is_down(rc_data->rc.switch_right))) // 还需添加重要应用和模块离线的判断
    {
        if(robot_state == ROBOT_READY)  LOGERROR("[CMD] emergency stop!");
        robot_state                   = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        arm_cmd_send.contro_mode      = ARM_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
        airpump_cmd_send.airpump_mode = AIRPUMP_STOP;
        airpump_cmd_send.airvalve_mode= AIRVALVE_STOP;
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data->rc.switch_right)) {
        if(robot_state == ROBOT_STOP)   LOGINFO("[CMD] reinstate, robot ready");
        robot_state = ROBOT_READY;
    }
}

// 监测红外测距模块状态
static void redlight_detect(){
    if(HAL_GPIO_ReadPin(redLight_detect_GPIO_Port,redLight_detect_Pin)==GPIO_PIN_SET){
        redlight_flag=1;    // 1为有物体遮挡
    }else redlight_flag = 0;
}
// 计算底盘与云台的偏差角
static void calChassisOffset(){
    chassis_cmd_send.offset_angle = gimbal_data_recv.yaw + arm_data_recv.big_yaw_angle;
}
// 额外操作，因遥控器键位不足，使用左拨杆为[上]时的拨轮状态进行额外的状态控制
static void extra_Control(){
    //todo: 为了避免该函数覆盖掉键鼠操作，此函数必须在键鼠操作前调用
    //      但其未来仍可能触发奇怪的错误，最好能改进

    //左拨杆为[上]，右拨杆为[上]时，用拨轮控制气泵开关
    if(switch_is_up(rc_data->rc.switch_right)){
        static uint8_t dial_switch_state = 0;
        //拨轮向上时，开关臂气泵
        if(dial_flag==1 && !(dial_switch_state & 0x01)){
            AIRPUMP_SWITCH(airpump_cmd_send.airpump_mode,AIRPUMP_ARM_OPEN);
            dial_switch_state |= 0x01;
        }else if(!(rc_data->key[KEY_PRESS_WITH_CTRL].g)){
            dial_switch_state &= ~0x01;
        }
        //拨轮向下时，开关推杆气泵
        if(dial_flag==1 && !(dial_switch_state & 0x02)){
            AIRPUMP_SWITCH(airpump_cmd_send.airpump_mode,AIRPUMP_LINEAR_OPEN);
            dial_switch_state |= 0x02;
        }else if(!(rc_data->key[KEY_PRESS_WITH_SHIFT].g)){
            dial_switch_state &= ~0x02;
        }
    }

    //左拨杆为[上]，右拨杆为[中]时，用拨轮控制臂结算办法
    /* 该功能源于臂安装的编码器与实际角度存在1:2的比值，上电位置不同会导致整个臂臂的结算异常，需手动修正 */
    if(switch_is_up(rc_data->rc.switch_right)){
        static uint8_t dial_switch_state = 0;
        //拨轮向上时，修正yaw偏向（默认为右偏)
        if(dial_flag==1 && !(dial_switch_state & 0x01)){
            (arm_cmd_send.optimize_signal&0x01) ? (arm_cmd_send.optimize_signal&=~0x01) : (arm_cmd_send.optimize_signal|=0x01);
            dial_switch_state |= 0x01;
        }else if(!(dial_flag==1)){
            dial_switch_state &= ~0x01;
        }
        //拨轮向下时，修正混合roll偏差角
        if(dial_flag==-1 && !(dial_switch_state & 0x02)){
            (arm_cmd_send.optimize_signal&0x02) ? (arm_cmd_send.optimize_signal&=~0x02) : (arm_cmd_send.optimize_signal|=0x02);
            dial_switch_state |= 0x02;
        }else if(!(dial_flag==-1)){
            dial_switch_state &= ~0x02;
        }
    }
}
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(gimbal_data_sub,&gimbal_data_recv);
    SubGetMessage(arm_data_sub,&arm_data_recv);


    // 监测红外测距模块状态
    redlight_detect();

    // 左侧开关为上时，使用键鼠操作
    if(switch_is_up(rc_data->rc.switch_left)){
        extra_Control();
        MouseKeySet();
    }else{ //否则使用遥控器操作
        RemoteControlSet();
    }

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    calChassisOffset(); // 计算底盘与云台的偏差角

    UI_reality_send.arm_auto_mode_id = arm_auto_mode_id;
    UI_reality_send.chassis_auto_mod_id = chassis_auto_mod_id;

    PubPushMessage(arm_cmd_pub, (void *)&arm_cmd_send);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(airpump_cmd_pub, (void *)&airpump_cmd_send);
    PubPushMessage(UI_reality_pub, (void *)&UI_reality_send);
}
