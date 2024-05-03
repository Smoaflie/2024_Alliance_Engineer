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

#define assorted_up_encoder_offset  62007
#define assorted_yaw_encoder_offset 167392
#define tail_motor_encoder_offset   142065
#define tail_roll_encoder_offset    0 // 152746

#define big_yaw_speed_limit         27
#define z_speed_limit               15000
#define middle_speed_limit          25
#define assorted_speed_limit        30000
#define tail_motor_speed_limit      70000
#define tail_roll_speed_limit       10000

#define z_motor_ReductionRatio      46.185567f
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
static EncoderInstance_s *assorted_up_encoder, *assorted_yaw_encoder, *tail_motor_encoder, *tail_roll_encoder; // 四个编码器，大YAW不另设编码器

// PID实例
static PIDInstance *assorted_yaw_pid, *assorted_roll_pid; // 中段两个2006电机只有速度环，在机械臂任务中通过这两个PID计算出二者的应达到的速度

// 臂关节角度控制
static Transform arm_controller_TF;
static arm_controller_data_s arm_param_t;
static arm_controller_data_s arm_origin_place;
static arm_controller_data_s arm_controller_data;
static arm_controller_data_s arm_recv_controller_data; // 自定义控制器的控制数据

static Subscriber_t *arm_cmd_data_sub;  // 臂臂控制信息收发

// 臂臂控制数据
static Arm_Cmd_Data_s arm_cmd_recv_data;

static HostInstance *host_instance; // 上位机接口
static uint8_t host_rec_flag;       // 上位机接收标志位

static DRMotorInstance *big_yaw_motor; // 大YAW电机
static DJIMotorInstance *z_motor;      // Z轴电机

static GPIO_PinState Z_limit_sensor_gpio, big_yaw_limit_sensor_gpio; // 限位传感器io
static Arm_State_Data_s arm_state;                                   // 臂臂状态

static uint8_t Control_ARM_flag = 0;    // 控制臂臂位姿标志
static uint8_t ARM_IN_flag = 0; // 臂臂是否处于回收状态标志
static Vector3 arm_rc_contro_place;
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
    arm_state.init_flag = 0;
    memset(&arm_controller_TF, 0, sizeof(arm_controller_TF));
    arm_controller_TF.localPosition.z = 0;
    arm_controller_TF.localPosition.x = 0.695f;

    Encoder_Init_Config_s encoder_config;
    // 编码器初始化
    encoder_config.encoder_type = MT6825;
    encoder_config.can_init_config.can_handle = &hfdcan1;
    encoder_config.can_init_config.rx_id      = 0x1fb;
    encoder_config.offset                     = assorted_up_encoder_offset;
    assorted_up_encoder                       = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id      = 0x2fb;
    encoder_config.offset                     = assorted_yaw_encoder_offset;
    assorted_yaw_encoder                      = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id      = 0x3ff;
    encoder_config.offset                     = tail_motor_encoder_offset;
    tail_motor_encoder                        = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id      = 0x4ff;
    encoder_config.offset                     = tail_roll_encoder_offset;
    // tail_roll_encoder                           = EncoderInit(&encoder_config);
    Motor_Init_Config_s mid_yaw_motor_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 1.5, // 0
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
                .IntegralLimit = 0.5,
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
                .Kp            = 2, // 4.5
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
                .Kp            = 20000, // 0
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
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            // .outer_loop_type    = ANGLE_LOOP,
            // .close_loop_type    = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    Motor_Init_Config_s big_yaw_init_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
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
            .tx_id      = 3,
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
                .Kp            = 1.2,   // 1.2,   // 4.5
                .Ki            = 0.8,   // 0.8,   // 0
                .Kd            = 0.006, // 0.006, // 0
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

    // 电机初始化
    mid_yaw_motor                               = DRMotorInit(&mid_yaw_motor_config);
    assorted_motor_config.can_init_config.tx_id = 1;
    assorted_motor_up                           = DJIMotorInit(&assorted_motor_config);
    assorted_motor_config.can_init_config.tx_id = 2;
    assorted_motor_down                         = DJIMotorInit(&assorted_motor_config);
    tail_motor                                  = DJIMotorInit(&tail_motor_config);
    tail_roll_motor                             = DJIMotorInit(&tail_roll_motor_config);
    big_yaw_motor                               = DRMotorInit(&big_yaw_init_config);
    z_motor                                     = DJIMotorInit(&z_motor_config);
    // 外置PID初始化
    assorted_yaw_pid  = PIDRegister(&assorted_yaw_pid_config);
    assorted_roll_pid = PIDRegister(&assorted_roll_pid_config);
    // 消息收发初始化
    arm_cmd_data_sub   = SubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
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
    if (arm_state.init_flag & Z_motor_init_clt)
        VAL_LIMIT(z_height, -575, 30);
    else
        VAL_LIMIT(z_height, -575, 575);
    z_height *= z_motor_ReductionRatio;
    DJIMotorSetRef(z_motor, z_height);
}
static void set_mid_yaw_angle(float angle)
{
    VAL_LIMIT(angle, -124, 115);
    DRMotorSetRef(mid_yaw_motor, angle);
}
static void set_mid_rAy_angle(float roll_angle, float yaw_angle)
{
    roll_angle = roll_angle - 360 * (int16_t)(roll_angle / 360);
    roll_angle = roll_angle > 180 ? (roll_angle - 360) : (roll_angle < -180 ? (roll_angle + 360) : roll_angle);
    // VAL_LIMIT(roll_angle, -180, 180);
    VAL_LIMIT(yaw_angle, -90, 85);
    float speed_yaw, speed_roll, speed_up, speed_down;
    static float assorted_yaw_angle, assorted_roll_angle;
    assorted_yaw_angle = assorted_yaw_encoder->measure.total_angle;
    // todo:shi
    assorted_roll_angle = assorted_up_encoder->measure.total_angle;
    assorted_roll_angle = (assorted_roll_angle + assorted_yaw_encoder->measure.total_angle) / 2.0;
    assorted_roll_angle = assorted_roll_angle - 360 * (int16_t)(assorted_roll_angle / 360);
    assorted_roll_angle = assorted_roll_angle > 180 ? (assorted_roll_angle - 360) : (assorted_roll_angle < -180 ? (assorted_roll_angle + 360) : assorted_roll_angle);

    // 过零点处理
    if (!limit_bool(assorted_roll_angle - roll_angle, 180, -180)) {
        if (assorted_roll_angle - roll_angle > 180) roll_angle += 360;
        if (assorted_roll_angle - roll_angle < -180) roll_angle -= 360;
    }
    speed_roll = PIDCalculate(assorted_roll_pid, assorted_roll_angle, roll_angle);
    speed_yaw  = -PIDCalculate(assorted_yaw_pid, assorted_yaw_angle, yaw_angle);

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
// static void set_tail_roll_angle(float angle)
// {
//     VAL_LIMIT(angle, -90, 90);
//     DJIMotorSetRef(tail_roll_motor, angle);
// }

// 设置各关节目标角度
static void set_arm_angle(arm_controller_data_s *data)
{
    set_z_height(data->height);
    // if(data->height*z_motor_ReductionRatio < 30)
    set_big_yaw_angle(-data->big_yaw_angle);
    set_mid_yaw_angle(-data->mid_yaw_angle);
    set_mid_rAy_angle(data->assorted_roll_angle, data->assorted_yaw_angle);
    set_tail_motor_angle(-data->tail_motor_angle);
}

// Z轴寻找标定位（原理为等待触发Z限位开关）
static void Z_limit_sensor_detect()
{
    Z_limit_sensor_gpio = HAL_GPIO_ReadPin(Z_limit_detect_GPIO_Port, Z_limit_detect_Pin);
    if (Z_limit_sensor_gpio == GPIO_PIN_RESET) {
        arm_state.init_flag |= Z_motor_init_clt;
        if ((z_motor->measure.total_round > 2 || z_motor->measure.total_round < -2)) {
            z_motor->measure.total_round = 0;
            arm_param_t.height = 0;
            DJIMotorStop(z_motor);

            arm_state.init_flag |= Z_motor_pub_reset;
        }
    }
}

// YAW寻找标定位（原理为等待触发YAW限位开关）
static void big_yaw_limit_sensor_detect()
{
    big_yaw_limit_sensor_gpio = HAL_GPIO_ReadPin(YAW_limit_detect_GPIO_Port,YAW_limit_detect_Pin);
    // if(big_yaw_limit_sensor_gpio ==GPIO_PIN_RESET && (big_yaw_motor->measure.total_angle > 50 ||big_yaw_motor->measure.total_angle<-50 )){
    //     uint8_t tx_buf_reset[] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //     CANTransmit_once(big_yaw_motor->motor_can_ins->can_handle,
    //                         (big_yaw_motor->motor_can_ins->tx_id & (0x1f << 5)) + 0x08,
    //                         tx_buf_reset, 0.1);
    //     DRMotorStop(big_yaw_motor);
    //     arm_state.init_flag |= Big_Yaw_motor_init_clt;
    // arm_state.init_flag |= Big_Yaw_motor_pub_reset;
    // }
}

// 根据控制数据对末端TF进行变换
static uint8_t ControlArm(Transform *TF)
{
    /* 原逆解算式，直接计算末端位姿的变换矩阵 */
    // 原末端位姿变换矩阵
    // float last_TransformationMatrix_data[16] = {
    //     1 - 2 * powf(TF->localRotation.y, 2) - 2 * powf(TF->localRotation.z, 2), 2 * (TF->localRotation.x * TF->localRotation.y - TF->localRotation.z * TF->localRotation.w), 2 * (TF->localRotation.x * TF->localRotation.z + TF->localRotation.y * TF->localRotation.w), TF->localPosition.x,
    //     2 * (TF->localRotation.x * TF->localRotation.y + TF->localRotation.z * TF->localRotation.w), 1 - 2 * powf(TF->localRotation.x, 2) - 2 * powf(TF->localRotation.z, 2), 2 * (TF->localRotation.y * TF->localRotation.z - TF->localRotation.x * TF->localRotation.w), TF->localPosition.y,
    //     2 * (TF->localRotation.x * TF->localRotation.z - TF->localRotation.y * TF->localRotation.w), 2 * (TF->localRotation.y * TF->localRotation.z + TF->localRotation.x * TF->localRotation.w), 1 - 2 * powf(TF->localRotation.x, 2) - 2 * powf(TF->localRotation.y, 2), TF->localPosition.z,
    //     0, 0, 0, 1};
    // Matrix last_TransformationMatrix = {4, 4, last_TransformationMatrix_data};

    // float TransformationMatrix_data[16] = {
    //     cosf(data->Roatation_Vertical) * cosf(data->Roatation_Horizontal), -sinf(data->Roatation_Vertical), cosf(data->Roatation_Vertical) * sinf(data->Roatation_Horizontal), data->Translation_x,
    //     sinf(data->Roatation_Vertical) * cosf(data->Roatation_Horizontal), cosf(data->Roatation_Vertical), sinf(data->Roatation_Vertical) * sinf(data->Roatation_Horizontal), data->Translation_y,
    //     -sinf(data->Roatation_Horizontal), 0, cosf(data->Roatation_Horizontal), 0,
    //     0, 0, 0, 1};
    // Matrix TransformationMatrix = {4, 4, TransformationMatrix_data};
    // M_mul(&Matrix_keep, &last_TransformationMatrix, &TransformationMatrix);

    /* 改版逆解算式，从arm2处重构TF树 */
    // arm2处的旋转矩阵
    float arm3_roll_angle_new                            = degree2rad_limited(arm_origin_place.assorted_roll_angle);
    float TransformationMatrix_arm2_origin__origin__q_data[9] = {
        1, 0, 0,
        0, cosf(arm3_roll_angle_new),   -sinf(arm3_roll_angle_new),
        0, sinf(arm3_roll_angle_new),   cosf(arm3_roll_angle_new),
        };
    Matrix TransformationMatrix_arm2_origin__origin__q_m = {3, 3, TransformationMatrix_arm2_origin__origin__q_data};
    // arm3相对于arm2的旋转矩阵
    float arm4_pitch_angle_new                           = degree2rad_limited(arm_origin_place.tail_motor_angle);
    float TransformationMatrix_arm3_arm2__arm2__q_data[9] = {
        cosf(arm4_pitch_angle_new),0, sinf(arm4_pitch_angle_new),
        0, 1, 0,
        -sinf(arm4_pitch_angle_new),0,  cosf(arm4_pitch_angle_new),
    };
    Matrix TransformationMatrix_arm3_arm2__arm2__q_m = {3, 3, TransformationMatrix_arm3_arm2__arm2__q_data};
    // target旋转矩阵
    float TransformationMatrix_target_origin__origin__q_data[9];
    Matrix TransformationMatrix_target_origin__origin__q_m = {3,3,TransformationMatrix_target_origin__origin__q_data};
    M_mul(&TransformationMatrix_target_origin__origin__q_m,&TransformationMatrix_arm2_origin__origin__q_m,&TransformationMatrix_arm3_arm2__arm2__q_m);
    // sub平移矩阵
    float TransformationMatrix_target_sub__origin__p_data[3] = {-arm_cmd_recv_data.Translation_x,arm_cmd_recv_data.Translation_y,0};
    Matrix TransformationMatrix_target_sub__origin__p_m = {3,1,TransformationMatrix_target_sub__origin__p_data};
    M_mul(&Matrix_keep, &TransformationMatrix_target_origin__origin__q_m,&TransformationMatrix_target_sub__origin__p_m);
    Vector3 sub_vec;
    memcpy((float*)&sub_vec,(float*)&Matrix_data_keep,sizeof(Vector3));
    arm_rc_contro_place.x += sub_vec.x;
    arm_rc_contro_place.y += sub_vec.y;
    arm_rc_contro_place.z += sub_vec.z;
    // arm4平移矩阵
    float TransformationMartix_target_arm3__origin__p_data[3] = {arm4,0,0};
    Matrix TransformationMartix_target_arm3__origin__p_m = {3,1,TransformationMartix_target_arm3__origin__p_data};
    M_mul(&Matrix_keep, &TransformationMatrix_target_origin__origin__q_m,&TransformationMartix_target_arm3__origin__p_m);

    // 获取最终位姿
    Transform new_TF = {
        .localPosition = {
            .x = arm1+arm2+arm3 + Matrix_data_keep[0] + arm_rc_contro_place.x,
            .y = Matrix_data_keep[1] + arm_rc_contro_place.y,
            .z = Matrix_data_keep[2] + arm_rc_contro_place.z,
        },
    };
    rotationToQuaternion((float *)TransformationMatrix_target_origin__origin__q_data,&new_TF.localRotation);

    // 判断末端位置是否超出有效解范围
    float TF_X;
    TF_X       = sqrtf(new_TF.localPosition.x * new_TF.localPosition.x + new_TF.localPosition.y * new_TF.localPosition.y);
    if (limit_bool(TF_X,0.745f,0.4608f) && limit_bool(new_TF.localPosition.z,0.03+arm4,-0.575-arm4))
    {
        memcpy(TF, &new_TF, sizeof(Transform));

        arm_controller_data_s update_data;
        if(Update_angle(new_TF, &update_data)) {
            
            // memcpy(TF, &new_TF, sizeof(Transform));
            
            if (Control_ARM_flag == 0){
                arm_origin_place.big_yaw_angle-=update_data.big_yaw_angle;
                arm_origin_place.height-=TF->localPosition.z * 1000;
            }
            arm_param_t.big_yaw_angle      = arm_origin_place.big_yaw_angle + update_data.big_yaw_angle;
            arm_param_t.mid_yaw_angle      = update_data.mid_yaw_angle;
            arm_param_t.assorted_yaw_angle = update_data.assorted_yaw_angle;
            arm_param_t.assorted_roll_angle = arm_origin_place.assorted_roll_angle;
            arm_param_t.tail_motor_angle = arm_origin_place.tail_motor_angle;
            arm_param_t.height = arm_origin_place.height + TF->localPosition.z * 1000;
            memcpy(&arm_controller_data,&arm_param_t,sizeof(arm_controller_data));
            return 2;
        }
        
        return 0;
    }
    arm_rc_contro_place.x -= sub_vec.x;
    arm_rc_contro_place.y -= sub_vec.y;
    arm_rc_contro_place.z -= sub_vec.z;
    return 0;
}

/* 机器人机械臂控制核心任务 */
void ArmTask()
{
    // 如未接收到cmd命令，卡死在这一步
    while(!SubGetMessage(arm_cmd_data_sub, &arm_cmd_recv_data)){
        DRMotorStop(mid_yaw_motor);
        DJIMotorStop(assorted_motor_up);
        DJIMotorStop(assorted_motor_down);
        DJIMotorStop(tail_motor);
        DRMotorStop(big_yaw_motor);
        DJIMotorStop(z_motor);
        DJIMotorStop(tail_roll_motor);
    }

    // DRMotorEnable(big_yaw_motor);
    // DRMotorEnable(mid_yaw_motor);
    // DJIMotorEnable(assorted_motor_up);
    // DJIMotorEnable(assorted_motor_down);
    // DJIMotorEnable(tail_motor);
    // DJIMotorEnable(z_motor);
    // DJIMotorEnable(tail_roll_motor);
    
    // 快速设置臂臂位置
    if(arm_cmd_recv_data.init_flag & Reset_arm_cmd_param_flag){
        ARM_IN_flag = 0;
        memset(&arm_param_t,0,sizeof(arm_param_t));
    }
    else if(arm_cmd_recv_data.init_flag & Recycle_arm_in){
        ARM_IN_flag = 1;
    }

    

    //进行限位检测
    Z_limit_sensor_detect();
    big_yaw_limit_sensor_detect();

    //控制末端吸盘roll
    if(arm_cmd_recv_data.sucker_state == 1){
        DJIMotorSetRef(tail_roll_motor,10000);
    }else if(arm_cmd_recv_data.sucker_state == -1){
        DJIMotorSetRef(tail_roll_motor,-10000);
    }else{
        DJIMotorSetRef(tail_roll_motor,0);
    }

    if (arm_cmd_recv_data.mode == ARM_POSE_CONTRO_MODE) {
        Control_ARM_flag = 0;

        arm_param_t.height        += arm_cmd_recv_data.Position_z;
        arm_param_t.big_yaw_angle -= arm_cmd_recv_data.Rotation_yaw;
    } else if (arm_cmd_recv_data.mode == ARM_REFER_MODE && ARM_IN_flag == 0) {
        if (Control_ARM_flag == 0) {
            memcpy(&arm_origin_place,&arm_param_t,sizeof(arm_param_t));
            memset(&arm_param_t,0,sizeof(arm_rc_contro_place));
            ControlArm(&arm_controller_TF);
            Control_ARM_flag = 1;
        }else{
            arm_origin_place.assorted_roll_angle += arm_cmd_recv_data.Roatation_Horizontal;
            arm_origin_place.tail_motor_angle += arm_cmd_recv_data.Roatation_Vertical;
            LIMIT_MIN_MAX(arm_origin_place.tail_motor_angle,-90,90);
            if(!ControlArm(&arm_controller_TF))
            {
                arm_origin_place.assorted_roll_angle -= arm_cmd_recv_data.Roatation_Horizontal;
                arm_origin_place.tail_motor_angle -= arm_cmd_recv_data.Roatation_Vertical;
            }
        }
        /* 下方为自定义控制器 */
        // if(host_rec_flag == 1){
        //     arm_param_t.big_yaw_angle = arm_recv_controller_data.big_yaw_angle;
        //     memcpy((uint8_t *)&arm_param_t + 8, (uint8_t *)&arm_recv_controller_data + 8, sizeof(arm_recv_controller_data) - 8);
        //     host_rec_flag = 0;
        // }
    } else if (arm_cmd_recv_data.mode == ARM_ZERO_FORCE) {
        Control_ARM_flag = 0;

        DRMotorStop(mid_yaw_motor);
        DJIMotorStop(assorted_motor_up);
        DJIMotorStop(assorted_motor_down);
        DJIMotorStop(tail_motor);
        DRMotorStop(big_yaw_motor);
        DJIMotorStop(z_motor);
        DJIMotorStop(tail_roll_motor);
    }else if(arm_cmd_recv_data.mode == ARM_FIXED){
        Control_ARM_flag = 0;
    }

    set_arm_angle(&arm_param_t);
}
