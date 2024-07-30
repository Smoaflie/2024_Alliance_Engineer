/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_gpio.h"

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_init.h"
#include "tool.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE  (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL  (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define chassis_max_speed 45000

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令


static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static PIDInstance *chassis_follow_pid;

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float current_chassis_vx, current_chassis_vy; // 底盘实时速度
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

static GPIOInstance *redLight_detect_gpio; // 红外测距传感器

static float current_feedforward_lb = 0; 
static float current_feedforward_lf = 0; 
static float current_feedforward_rb = 0; 
static float current_feedforward_rf = 0; 

void ChassisInit_Motor()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hfdcan3,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3, // 3.5
                .Ki            = 0,  // 0
                .Kd            = 0.005,  // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16384,
                },
            .current_PID = {
                .Kp            = 0.5, // 0.4
                .Ki            = 0,   // 0
                .Kd            = 0,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 15000,
            },
        },
        .controller_setting_init_config = {
            .feedforward_flag      = CURRENT_FEEDFORWARD,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &current_feedforward_lf;
    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &current_feedforward_lb;
    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);
    
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &current_feedforward_rb;
    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    //右轮阻力太大了，单独加点前馈
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &current_feedforward_rf;
    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    PID_Init_Config_s chassis_follow_pid_config = {
                          .Kp            = 500, // 1500
                          .Ki            = 0,    // 0
                          .Kd            = 10,    // 0
                          .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                          .IntegralLimit = 0,
                          .MaxOut        = 20000,
                      };
    chassis_follow_pid  = PIDRegister(&chassis_follow_pid_config);
}
void ChassisInit_Communication()
{
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
}
void ChassisInit_IO()
{
    GPIO_Init_Config_s gpio_conf_redlight_detection = {
        .GPIOx = redLight_detect_GPIO_Port,
        .GPIO_Pin = redLight_detect_Pin,
    };
    redLight_detect_gpio = GPIORegister(&gpio_conf_redlight_detection);
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

static void update_chassis_velocity(float* chassis_v, float current_chassis_v, float limit_add_speed, float limit_add_speed_stop) {
    if (current_chassis_v > 0) {
        if (*chassis_v - current_chassis_v > limit_add_speed) {
            *chassis_v = current_chassis_v + limit_add_speed;
        } else if (*chassis_v - current_chassis_v < -limit_add_speed_stop) {
            *chassis_v = current_chassis_v - limit_add_speed_stop;
        }
    } else {
        if (*chassis_v - current_chassis_v > limit_add_speed_stop) {
            *chassis_v = current_chassis_v + limit_add_speed_stop;
        } else if (*chassis_v - current_chassis_v < -limit_add_speed) {
            *chassis_v = current_chassis_v - limit_add_speed;
        }
    }
}
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
void MecanumCalculate()
{
    //根据当前速度对目标速度进行限制
    static float limit_add_speed_x,limit_add_speed_x_stop,limit_add_speed_y,limit_add_speed_y_stop;
    static float limit_add_speed_x_v[2] = {4000,2200};
    static float limit_add_speed_y_v[2] = {15000, 2200};
    static float limit_add_speed_x_stop_v[2] = {2000,900};
    static float limit_add_speed_y_stop_v[2] = {3000,900};

    if(chassis_cmd_recv.chassis_mode != CHASSIS_ROTATE){
        if(chassis_cmd_recv.arm_height <= -490){
            limit_add_speed_x = limit_add_speed_x_v[0];
            limit_add_speed_y = limit_add_speed_y_v[0];
            limit_add_speed_x_stop = limit_add_speed_x_stop_v[0];
            limit_add_speed_y_stop = limit_add_speed_y_stop_v[0];
        }else{
            limit_add_speed_x = limit_add_speed_x_v[1];
            limit_add_speed_y = limit_add_speed_y_v[1];
            limit_add_speed_x_stop = limit_add_speed_x_stop_v[1];
            limit_add_speed_y_stop = limit_add_speed_y_stop_v[1];
        }
        update_chassis_velocity(&chassis_vx,current_chassis_vx,limit_add_speed_x,limit_add_speed_x_stop);
        update_chassis_velocity(&chassis_vy,current_chassis_vy,limit_add_speed_y,limit_add_speed_y_stop);

        float chassis_speed_max = (chassis_max_speed-fabsf(chassis_cmd_recv.wz))/2*sqrtf(2.0f);
        float chassis_speed = sqrtf(powf(chassis_vx,2) + powf(chassis_vy,2));
        float chassis_speed_rad = asinf(chassis_speed/fabsf(chassis_vx));
        VAL_LIMIT(chassis_speed, -chassis_speed_max, chassis_speed_max);
        VAL_LIMIT(chassis_vx, -chassis_speed*sinf(chassis_speed_rad), chassis_speed*sinf(chassis_speed_rad));
        VAL_LIMIT(chassis_vy, -chassis_speed*cosf(chassis_speed_rad), chassis_speed*cosf(chassis_speed_rad));

        static float kp = -2;
        static uint8_t current_feedforward_chassis = 0x0f;
        if(fabsf(current_chassis_vx)<3000)  current_feedforward_chassis = 0x0f;
        else current_feedforward_chassis = 0x00;

        current_feedforward_lb = current_feedforward_chassis&0x01 ? chassis_vx * kp : 0; 
        current_feedforward_lf = current_feedforward_chassis&0x02 ? -chassis_vx * kp : 0; 
        current_feedforward_rb = current_feedforward_chassis&0x04 ? chassis_vx * kp : 0; 
        current_feedforward_rf = current_feedforward_chassis&0x08 ? -chassis_vx * kp : 0;
    }else{
        float chassis_rotate_speed_max = 20000;
        float chassis_speed_max = (chassis_max_speed-chassis_rotate_speed_max*1.5)/2*sqrtf(2.0f);
        float chassis_speed = sqrtf(powf(chassis_vx,2) + powf(chassis_vy,2));
        float chassis_speed_rad = asinf(chassis_speed/chassis_vx);
        VAL_LIMIT(chassis_speed, -chassis_speed_max, chassis_speed_max);
        VAL_LIMIT(chassis_vx, -chassis_speed*sinf(chassis_speed_rad), chassis_speed*sinf(chassis_speed_rad));
        VAL_LIMIT(chassis_vy, -chassis_speed*cosf(chassis_speed_rad), chassis_speed*cosf(chassis_speed_rad));
        if(chassis_cmd_recv.wz<0)
            VAL_LIMIT(chassis_cmd_recv.wz,-chassis_max_speed-(fabsf(chassis_vx)+fabsf(chassis_vy)), -chassis_rotate_speed_max);
        else
            VAL_LIMIT(chassis_cmd_recv.wz,chassis_rotate_speed_max, chassis_max_speed-(fabsf(chassis_vx)+fabsf(chassis_vy)));
        
        static float kp = -1.5;
        static uint8_t current_feedforward_chassis = 0x00;
        current_feedforward_lb = current_feedforward_chassis&0x01 ? (chassis_vx - chassis_vy) * kp : 0; 
        current_feedforward_lf = current_feedforward_chassis&0x02 ? (-chassis_vx - chassis_vy) * kp : 0; 
        current_feedforward_rb = current_feedforward_chassis&0x04 ? (chassis_vx + chassis_vy) * kp : 0; 
        current_feedforward_rf = current_feedforward_chassis&0x08 ? (-chassis_vx + chassis_vy) * kp : 0;
    }

    //计算得目标速度
    // vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    // vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    // vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    // vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz;
    vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz;
    extern float UI_debug_value[7];
    UI_debug_value[0] = chassis_vx;
    UI_debug_value[1] = chassis_vy;
    UI_debug_value[2] = chassis_cmd_recv.wz;
    // UI_debug_value[3] =  vt_lf;
    // UI_debug_value[4] = vt_rf;
    // UI_debug_value[5] = vt_rb;
    // UI_debug_value[6] = vt_lb;
    UI_debug_value[3] =  motor_lf->measure.speed_aps;
    UI_debug_value[4] = motor_rf->measure.speed_aps;
    UI_debug_value[5] = motor_rb->measure.speed_aps;
    UI_debug_value[6] = motor_lb->measure.speed_aps;
}

/**
 * @brief 设置电机参考值
 *
 */
void SetChassisRef()
{
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

//底盘特殊功能处理
void SpecialFuncApply(){
    // static GPIO_PinState redlight_last_state = 0; //红外测距状态 1为触发
    // static GPIO_PinState redlight_state = 0; //红外测距状态 1为触发
    // static uint16_t redlight_detection_cnt = 0;
    // // redlight_state = GPIORead(redLight_detect_gpio);
    // redlight_state = HAL_GPIO_ReadPin(redLight_detect_GPIO_Port,redLight_detect_Pin);

    // static uint8_t func_doing_state = 0;
    // //斜坡向左移动-切换
    // static uint8_t slope_move_l_switch = 0;
    // if(chassis_cmd_recv.special_func_flag == CHASSIS_SLOPE_MOVE_L && !(slope_move_l_switch==1)){
    //     func_doing_state &= ~0x02;
    //     (func_doing_state & 0x01) ? (func_doing_state&=~0x01) : (func_doing_state|=0x01); 
    //     slope_move_l_switch = 1;
    //     redlight_last_state = redlight_state;
    // }else if(!(chassis_cmd_recv.special_func_flag == CHASSIS_SLOPE_MOVE_L)){
    //     slope_move_l_switch=0;
    // }
    // //斜坡向右移动-切换
    // static uint8_t slope_move_r_switch = 0;
    // if(chassis_cmd_recv.special_func_flag == CHASSIS_SLOPE_MOVE_R && !(slope_move_r_switch==1)){
    //     func_doing_state &= ~0x01;
    //     (func_doing_state & 0x02) ? (func_doing_state&=~0x02) : (func_doing_state|=0x02); 
    //     slope_move_r_switch = 1;
    //     redlight_last_state = redlight_state;
    // }else if(!(chassis_cmd_recv.special_func_flag == CHASSIS_SLOPE_MOVE_R)){
    //     slope_move_r_switch=0;
    // }
    // //斜向移动
    // if(func_doing_state & 0x03){
    //     if(redlight_state != redlight_last_state){
    //         chassis_vx = 0;
    //         chassis_vy = 0;
    //         redlight_detection_cnt++;
    //         if(redlight_detection_cnt > 200)
    //             func_doing_state &= ~0x03;
    //     }else{
    //         // static float d_v = 2500;
    //         redlight_detection_cnt = 0;
    //         chassis_vx = (((func_doing_state&0x02)>>1) - (func_doing_state&0x01)) * 2500;
    //         chassis_vy = 800;
    //     }
    // }
    
}

void ChassisSubMessage()
{
    while(!SubGetMessage(chassis_sub, &chassis_cmd_recv)){
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
}
void ChassisGetCurrentSpeed()
{
    //todo: 目前在旋转与平移中间态时存在错误数据
    float vc_lf,vc_rf,vc_lb,vc_rb;
    vc_lf = motor_lf->measure.speed_aps + chassis_cmd_recv.wz * LF_CENTER;
    vc_rf = motor_rf->measure.speed_aps + chassis_cmd_recv.wz * RF_CENTER;
    vc_lb = motor_lb->measure.speed_aps + chassis_cmd_recv.wz * LB_CENTER;
    vc_rb = motor_rb->measure.speed_aps + chassis_cmd_recv.wz * RB_CENTER;
    // vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    // vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    // vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    // vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
    current_chassis_vx = -(-(vc_lf + vc_rf) + (vc_lb + vc_rb))/4.0f;
    current_chassis_vy = -(-(vc_lf - vc_rf) - (vc_lb - vc_rb))/4.0f;
}
void ChassisModeSelect()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    if(chassis_cmd_recv.arm_height <= -490)
        chassis_follow_pid->MaxOut = 40000;
    else
        chassis_follow_pid->MaxOut = 20000;

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW: // 底盘维持全向机动,并可受控旋转
            chassis_cmd_recv.wz = chassis_cmd_recv.wz;
            // chassis_cmd_recv.offset_angle = 0;
            break;
        case CHASSIS_NO_FOLLOW_CONVERTMODE:
            chassis_cmd_recv.wz = chassis_cmd_recv.wz;
            // chassis_cmd_recv.offset_angle = -90;
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
            // chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * fabsf(chassis_cmd_recv.offset_angle);
            // chassis_cmd_recv.wz = -15 * chassis_cmd_recv.offset_angle * fabsf(chassis_cmd_recv.offset_angle);
            // VAL_LIMIT(chassis_cmd_recv.wz,-8000,8000);
            chassis_cmd_recv.wz = PIDCalculate(chassis_follow_pid,0,-chassis_cmd_recv.offset_angle);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW_REVERSE:
            static float de;
            de = ((chassis_cmd_recv.offset_angle+180) > 180 ? chassis_cmd_recv.offset_angle+180-360 : chassis_cmd_recv.offset_angle+180);
            chassis_cmd_recv.wz = PIDCalculate(chassis_follow_pid,0,-de);
            // chassis_cmd_recv.wz = -15  * de * fabsf(de);
            // VAL_LIMIT(chassis_cmd_recv.wz,-8000,8000);
            break;
        case CHASSIS_ROTATE: // 自旋,同时保持全向机动；
            if(chassis_cmd_recv.vx > 40000 || chassis_cmd_recv.vy > 40000)
                chassis_cmd_recv.wz = PIDCalculate(chassis_follow_pid,0,-chassis_cmd_recv.offset_angle);
            else{
                static uint16_t mode_selected_cnt=0,mode_selected = 0;
                if(mode_selected_cnt++ > 600)  {mode_selected = rand()%2;mode_selected_cnt=0;}

                if(chassis_cmd_recv.arm_height <= -240)
                    chassis_cmd_recv.wz = 40000 + 10000*mode_selected;
                else
                    chassis_cmd_recv.wz = 20000 + 5000*mode_selected;
            }                
            break;
        default:
            break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta  = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta  = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
}
//车身倾倒检测
//todo:反方向倾倒+参数优化
void RobotTumbleDetect(){
    chassis_vy += chassis_cmd_recv.gimbal_pitch_imu * 1000;
}    
void ChassisDebugInterface()
{
    // static float kp=5.5,ki=0,kd=0.0145;
    // motor_lf->motor_controller.speed_PID.Kp = kp;
    // motor_rf->motor_controller.speed_PID.Kp = kp;
    // motor_lb->motor_controller.speed_PID.Kp = kp;
    // motor_rb->motor_controller.speed_PID.Kp = kp;

    // motor_lf->motor_controller.speed_PID.Ki = ki;
    // motor_rf->motor_controller.speed_PID.Ki = ki;
    // motor_lb->motor_controller.speed_PID.Ki = ki;
    // motor_rb->motor_controller.speed_PID.Ki = ki;

    // motor_lf->motor_controller.speed_PID.Kd = kd;
    // motor_rf->motor_controller.speed_PID.Kd = kd;
    // motor_lb->motor_controller.speed_PID.Kd = kd;
    // motor_rb->motor_controller.speed_PID.Kd = kd;
}