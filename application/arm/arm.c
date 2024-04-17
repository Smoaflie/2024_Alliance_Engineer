// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "DRmotor.h"
#include "encoder.h"
#include "led.h"
#include "bsp_usb.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"

#include "arm.h"
#include "decode.h"

#define assorted_yaw_encoder_offset  162828
#define assorted_roll_encoder_offset 180089
#define tail_motor_encoder_offset    22504
#define tail_roll_encoder_offset     0 // 152746

#define big_yaw_speed_limit          27
#define z_speed_limit                15000
#define middle_speed_limit           25
#define assorted_speed_limit         30000
#define tail_motor_speed_limit       40000
#define tail_roll_speed_limit        10000

#define z_motor_ReductionRatio       46.185567f
#define middle_ReductionRatio
#define tail_motor_ReductionRatio
#define tail_roll_ReductionRatio
/*
整个机械臂共有三个Yaw，两个Roll（不计大柱子的yaw）
*/
// 电机实例
static DRMotorInstance *mid_yaw_motor;                            // 臂Yaw电机
static DJIMotorInstance *assorted_motor_up, *assorted_motor_down; //  两个2006电机配合控制中段roll&yaw
static DJIMotorInstance *tail_motor;                              //  控制末端yaw的2006电机
static DJIMotorInstance *tail_roll_motor;                         //  控制末端roll吸盘的2006电机

// 编码器实例
static EncoderInstance_s *assorted_yaw_encoder, *assorted_roll_encoder, *tail_motor_encoder, *tail_roll_encoder; // 四个编码器，大YAW不另设编码器

// PID实例
static PIDInstance *assorted_yaw_pid, *assorted_roll_pid; // 中段两个2006电机只有速度环，在机械臂任务中通过这两个PID计算出二者的应达到的速度

// 臂关节角度控制
// static Transform    arm_controller_TF;
static arm_controller_data_s arm_controller_data;
static arm_controller_data_s arm_recv_controller_data; // 自定义控制器的控制数据

// 臂末端位姿信息收发
static Subscriber_t *arm_controller_sub;
static Arm_Cmd_Data_s arm_cmd_rec_data;

static HostInstance *host_instance; // 上位机接口
static uint8_t host_rec_flag;       // 上位机接收标志位

#ifdef ROBOT_TEST
static DRMotorInstance *big_yaw_motor; // 大YAW电机
static DJIMotorInstance *z_motor;      // Z轴电机
static float big_yaw_motor_ref, z_motor_ref = 0;
#endif

// 上位机解析回调函数
static void HOST_RECV_CALLBACK()
{
    memcpy(&arm_recv_controller_data, host_instance->comm_instance, sizeof(arm_controller_data_s));

    arm_recv_controller_data.big_yaw_angle *= -1;
    // arm_recv_controller_data.height *=100;   // 现行方案考虑不使用自定义控制器
    arm_recv_controller_data.height = arm_recv_controller_data.height;
    arm_recv_controller_data.assorted_roll_angle *= 1;
    arm_recv_controller_data.assorted_yaw_angle *= -1;
    arm_recv_controller_data.tail_motor_angle *= -1;
    arm_recv_controller_data.mid_yaw_angle *= -1;
    host_rec_flag = 1;
}

void ArmInit()
{
    Encoder_Init_Config_s encoder_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        }};
    Motor_Init_Config_s mid_yaw_motor_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 12.0, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 25,
                .MaxOut        = middle_speed_limit,
            },
            .speed_PID = {
                .Kp            = 0.5445, // 0
                .Ki            = 1.604,  // 0
                .Kd            = 0,      // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 0,
                .MaxOut        = 10, // 20000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type            = DR_B0X,
        .can_init_config.tx_id = 1};
    Motor_Init_Config_s assorted_motor_config = {
        .can_init_config.can_handle   = &hfdcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1, // 4.5
                .Ki            = 0, // 0
                .Kd            = 0, // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M2006,
    };
    Motor_Init_Config_s tail_motor_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .tx_id      = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 2000, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 50,
                .MaxOut        = tail_motor_speed_limit,
            },
            .speed_PID = {
                .Kp            = 2, // 4.5
                .Ki            = 0, // 0
                .Kd            = 0, // 0
                .IntegralLimit = 200,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
            .other_angle_feedback_ptr = &tail_motor_encoder->measure.total_angle,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            // .outer_loop_type       = SPEED_LOOP,
            // .close_loop_type       = SPEED_LOOP,
            .outer_loop_type    = ANGLE_LOOP,
            .close_loop_type    = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M2006,
    };
    Motor_Init_Config_s tail_roll_motor_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .tx_id      = 4,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 1000, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 50,
                .MaxOut        = tail_roll_speed_limit,
            },
            .speed_PID = {
                .Kp            = 0.2, // 4.5
                .Ki            = 1,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 200,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
            .other_angle_feedback_ptr = &tail_roll_encoder->measure.total_angle,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            // .outer_loop_type       = SPEED_LOOP,
            // .close_loop_type       = SPEED_LOOP,
            .outer_loop_type    = ANGLE_LOOP,
            .close_loop_type    = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    Motor_Init_Config_s big_yaw_init_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 1, // 0
                .Ki            = 0, // 0
                .Kd            = 0, // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 25,
                .MaxOut        = big_yaw_speed_limit,
            },
            .speed_PID = {
                .Kp            = 1, // 0
                .Ki            = 0, // 0
                .Kd            = 0, // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 0,
                .MaxOut        = 10, // 20000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type            = DR_PDA04,
        .can_init_config.tx_id = 2};
    Motor_Init_Config_s z_motor_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
            .tx_id      = 6,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 6, // 0
                .Ki            = 0, // 0
                .Kd            = 0, // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 4000,
                .MaxOut        = z_speed_limit,
                .DeadBand      = 80,
            },
            .speed_PID = {
                .Kp            = 1.2,   // 4.5
                .Ki            = 0.8,   // 0
                .Kd            = 0.006, // 0
                .IntegralLimit = 200,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    PID_Init_Config_s assorted_yaw_pid_config = {
                          .Kp            = 1000, // 0
                          .Ki            = 0,    // 0
                          .Kd            = 0,    // 0
                          .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                          .IntegralLimit = 0,
                          .MaxOut        = assorted_speed_limit, // 20000
                      },
                      assorted_roll_pid_config = {
                          .Kp            = 1000, // 0
                          .Ki            = 0,    // 0
                          .Kd            = 0,    // 0
                          .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                          .IntegralLimit = 0,
                          .MaxOut        = assorted_speed_limit, // 20000
                      };

    HostInstanceConf host_conf = {
        .callback  = HOST_RECV_CALLBACK,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = 28,
    };
    //编码器初始化
    encoder_config.can_init_config.rx_id        = 0x1fb;
    encoder_config.offset                       = assorted_yaw_encoder_offset;
    assorted_yaw_encoder                        = EncoderInit(&encoder_config);
    encoder_config.can_init_config.can_handle   = &hfdcan1;
    encoder_config.can_init_config.rx_id        = 0x2ff;
    encoder_config.offset                       = assorted_roll_encoder_offset;
    assorted_roll_encoder                       = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id        = 0x3ff;
    encoder_config.offset                       = tail_motor_encoder_offset;
    tail_motor_encoder                          = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id        = 0x4ff;
    encoder_config.offset                       = tail_roll_encoder_offset;
    tail_roll_encoder                           = EncoderInit(&encoder_config);
    //电机初始化
    mid_yaw_motor                               = DRMotorInit(&mid_yaw_motor_config);
    assorted_motor_config.can_init_config.tx_id = 1;
    assorted_motor_up                           = DJIMotorInit(&assorted_motor_config);
    assorted_motor_config.can_init_config.tx_id = 2;
    assorted_motor_down                         = DJIMotorInit(&assorted_motor_config);
    tail_motor                                  = DJIMotorInit(&tail_motor_config);
    tail_roll_motor                             = DJIMotorInit(&tail_roll_motor_config);
    big_yaw_motor                               = DRMotorInit(&big_yaw_init_config);
    z_motor                                     = DJIMotorInit(&z_motor_config);
    //外置PID初始化
    assorted_yaw_pid  = PIDRegister(&assorted_yaw_pid_config);
    assorted_roll_pid = PIDRegister(&assorted_roll_pid_config);
    //消息收发初始化
    arm_controller_sub = SubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    host_instance      = HostInit(&host_conf); // 上位机通信串口
}

/* 一些私有函数 */
static void set_big_yaw_angle(float angle)
{
    VAL_LIMIT(angle, -200, 200);
    angle *= 2;
    DRMotorSetRef(big_yaw_motor, angle);
}
static void set_z_height(float z_height)
{
    VAL_LIMIT(z_height, 0, 615);
    z_height *= z_motor_ReductionRatio;
    DJIMotorSetRef(z_motor, z_height);
}
static void set_mid_yaw_angle(float angle)
{
    VAL_LIMIT(angle, -108, 77);
    DRMotorSetRef(mid_yaw_motor, angle);
}
static void set_mid_rAy_angle(float roll_angle, float yaw_angle)
{
    VAL_LIMIT(roll_angle, -180, 180);
    VAL_LIMIT(yaw_angle, -79, 79);
    float speed_yaw, speed_roll, speed_up, speed_down;
    speed_roll = PIDCalculate(assorted_roll_pid, assorted_roll_encoder->measure.total_angle, roll_angle);
    speed_yaw  = PIDCalculate(assorted_yaw_pid, assorted_yaw_encoder->measure.total_angle, yaw_angle);

    speed_up   = speed_yaw - speed_roll;
    speed_down = speed_yaw + speed_roll;

    DJIMotorSetRef(assorted_motor_up, speed_up);
    DJIMotorSetRef(assorted_motor_down, speed_down);
}
static void set_tail_motor_angle(float angle)
{
    VAL_LIMIT(angle, -90, 90);
    DJIMotorSetRef(tail_motor, angle);
}
static void set_tail_roll_angle(float angle)
{
    VAL_LIMIT(angle, -90, 90);
    DJIMotorSetRef(tail_roll_motor, angle);
}

// 设置各关节目标角度
static void set_arm_angle(arm_controller_data_s *data)
{
    set_big_yaw_angle(data->big_yaw_angle);
    set_z_height(data->height);
    set_mid_yaw_angle(data->mid_yaw_angle);
    set_mid_rAy_angle(data->assorted_roll_angle, data->assorted_yaw_angle);
    set_tail_motor_angle(data->tail_motor_angle);
}

/* 机器人机械臂控制核心任务 */
void ArmTask()
{
    static uint8_t flag = 0;
    if (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
        osDelay(1);
        while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
            ;
        osDelay(1);
        flag = !flag;
    }
    SubGetMessage(arm_controller_sub, &arm_cmd_rec_data);

    if (flag) {
        C_board_LEDSet(0xd633ff);

        DRMotorEnable(mid_yaw_motor);
        DJIMotorEnable(assorted_motor_up);
        DJIMotorEnable(assorted_motor_down);
        DJIMotorEnable(tail_motor);
        DJIMotorEnable(tail_roll_motor);
        DRMotorEnable(big_yaw_motor);
        DJIMotorEnable(z_motor);
        if (arm_cmd_rec_data.Translation_x == 3) {
            arm_controller_data.height              = arm_cmd_rec_data.Position_z;
            arm_controller_data.big_yaw_angle       = arm_cmd_rec_data.Rotation_yaw;
            arm_controller_data.mid_yaw_angle       = 0;
            arm_controller_data.assorted_yaw_angle  = 0;
            arm_controller_data.assorted_roll_angle = 0;
            arm_controller_data.tail_motor_angle    = -90;
        } else if (arm_cmd_rec_data.Translation_x == 1) {
            arm_controller_data.height              = arm_cmd_rec_data.Position_z;
            arm_controller_data.big_yaw_angle       = arm_cmd_rec_data.Rotation_yaw;
            arm_controller_data.mid_yaw_angle       = -8.60533619f;
            arm_controller_data.assorted_yaw_angle  = -48.1565247f;
            arm_controller_data.assorted_roll_angle = -132.504623f;
            arm_controller_data.tail_motor_angle    = 48.186348f;
        } else if (arm_cmd_rec_data.Translation_x == 2) {
            arm_controller_data.height        = arm_cmd_rec_data.Position_z + arm_recv_controller_data.height;
            arm_controller_data.big_yaw_angle = arm_cmd_rec_data.Rotation_yaw + arm_recv_controller_data.big_yaw_angle;

            memcpy((uint8_t *)&arm_controller_data + 8, (uint8_t *)&arm_recv_controller_data + 8, sizeof(arm_recv_controller_data) - 8);
        }
        set_arm_angle(&arm_controller_data);
        if (arm_cmd_rec_data.Translation_y == 2) {
            DJIMotorSetRef(tail_roll_motor, *tail_roll_motor->motor_controller.other_angle_feedback_ptr + 3);
        }
    } else {
        C_board_LEDSet(0x33ffff);

        DRMotorStop(mid_yaw_motor);
        DJIMotorStop(assorted_motor_up);
        DJIMotorStop(assorted_motor_down);
        DJIMotorStop(tail_motor);
        DJIMotorStop(tail_roll_motor);

#ifdef ROBOT_TEST
        DRMotorStop(big_yaw_motor);
        DJIMotorStop(z_motor);
#endif
    }

    if (arm_cmd_rec_data.state & 0x01) {
        DRMotorStop(mid_yaw_motor);
        DJIMotorStop(assorted_motor_up);
        DJIMotorStop(assorted_motor_down);
        DJIMotorStop(tail_motor);
        DJIMotorStop(tail_roll_motor);
#ifdef ROBOT_TEST
        DRMotorStop(big_yaw_motor);
        DJIMotorStop(z_motor);
#endif
    }
}
