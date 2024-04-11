#include "horizontal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "referee_init.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"

#ifdef ONE_BOARD
static Publisher_t *Horizontal_pub;                    // 用于发布升降的数据
static Subscriber_t *Horizontal_sub;                   // 用于订阅升降的控制命令
#endif

static Horizontal_Ctrl_Cmd_s Horizontal_cmd_recv;         // 升降接收到的控制命令
static Horizontal_Upload_Data_s Horizontal_feedback_data; // 升降回传的反馈数据

static DJIMotorInstance *motor_Horizontal; // left right forward back

static float vt_Horizontal;// 底盘速度解算后的临时输出,待进行限幅
void Horizontal_Init()
{
    // 两个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s Horizontal_motor_config = {
        .can_init_config.can_handle   = &hfdcan3,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 10, // 4.5
                .Ki            = 0,  // 0
                .Kd            = 0,  // 0
                .IntegralLimit = 3000,
                //@？？？这是啥
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                /////////////////////////////////////////没看懂
                .MaxOut        = 12000,
                0},
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
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    Horizontal_motor_config.can_init_config.tx_id                             = 3;
    Horizontal_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_Horizontal                                                            = DJIMotorInit(&Horizontal_motor_config);
                                                 
    #ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    Horizontal_sub = SubRegister("Horizontal_cmd", sizeof(Horizontal_Ctrl_Cmd_s));
    Horizontal_pub = PubRegister("Horizontal_feed", sizeof(Horizontal_Upload_Data_s));
#endif // ONE_BOARD
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitHorizontalOutput()
{
    DJIMotorSetRef(motor_Horizontal, vt_Horizontal);
}


/* 机器人底盘控制核心任务 */
void Horizontal_Task()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(Horizontal_sub, &Horizontal_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    Horizontal_cmd_recv = *(Horizontal_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // Horizontal_BOARD

    if (Horizontal_cmd_recv.Horizontal_mode == HORIZONTAL_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_Horizontal);
    } else { // 正常工作
        DJIMotorEnable(motor_Horizontal);
    }

    switch (Horizontal_cmd_recv.Horizontal_mode) {
        case HORIZONTAL_MOVE:
            break;
        default:
            break;
    }
    LimitHorizontalOutput();
    // 推送反馈消息
#ifdef ONE_BOARD
    Horizontal_feedback_data.now_angel = motor_Horizontal->measure.total_angle;
    PubPushMessage(Horizontal_pub, (void *)&Horizontal_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(Horizontal_can_comm, (void *)&Horizontal_feedback_data);
#endif // Horizontal_BOARD
}
