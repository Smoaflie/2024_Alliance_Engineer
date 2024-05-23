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
#include "tool.h"
#include "crc_ref.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"

#include "arm.h"
#include "decode.h"

#define assorted_up_encoder_offset  64898
#define assorted_yaw_encoder_offset 167392
#define tail_motor_encoder_offset   142065
#define tail_roll_encoder_offset    0 // 152746

#define big_yaw_speed_limit         40
#define z_speed_limit               15000
#define middle_speed_limit          40
#define assorted_speed_limit        30000
#define tail_motor_speed_limit      70000
#define tail_roll_speed_limit       10000

#define z_motor_ReductionRatio      46.185567f
/*
整个机械臂共有三个Yaw，两个Roll（不计大柱子的yaw）
*/
// 电机实例
static DRMotorInstance *mid_yaw_motor;                            // 臂Yaw电机
static DJIMotorInstance *assorted_motor_up, *assorted_motor_down; //  两个2006电机配合控制中段roll&yaw
static DJIMotorInstance *tail_motor;                              //  控制末端yaw的2006电机
static DJIMotorInstance *tail_roll_motor;                         //  控制末端roll吸盘的2006电机
static void*             joint_motor[7];                          // 按顺序存储关节电机的指针
// 编码器实例
static EncoderInstance_s *assorted_up_encoder, *assorted_yaw_encoder, *tail_motor_encoder, *tail_roll_encoder; // 四个编码器，大YAW不另设编码器

// PID实例
static PIDInstance *assorted_yaw_pid, *assorted_roll_pid; // 中段两个2006电机只有速度环，在机械臂任务中通过这两个PID计算出二者的应达到的速度

// 臂关节角度控制
static Transform arm_controller_TF; //臂臂末端位姿
static arm_controller_data_s arm_param_t;   //臂臂关节临时参数
static arm_controller_data_s arm_origin_place;  //用于遥控器控制臂臂时提供基准点
static arm_controller_data_s arm_contro_data;   //臂臂控制数据(关节目标角度)
static arm_controller_data_s arm_recv_host_data; // 上位机的传来的关节角度值
static arm_controller_data_s arm_auto_mode_data; // 臂臂自动模式目标值

static Subscriber_t *arm_cmd_sub;  // 臂臂控制信息收
static Publisher_t *arm_data_sub;  // 臂臂数据信息发

// 臂臂控制数据
static Arm_Cmd_Data_s arm_cmd_recv;
static Arm_Data_s arm_data_send;

static HostInstance *host_instance; // 上位机接口
static USARTInstance *host_uart_instance;
static uint8_t host_rec_flag;       // 上位机接收标志位
static uint8_t host_send_buf[33];   // 上位机发送缓冲区

static DRMotorInstance *big_yaw_motor; // 大YAW电机
static DJIMotorInstance *z_motor;      // Z轴电机

static GPIO_PinState Z_GPIO;
static GPIOInstance *Z_limit_sensor_gpio; // 限位传感器io

static uint8_t arm_init_flag;   // 臂臂初始化标志
static uint8_t Control_ARM_flag = 0;    // 控制臂臂位姿标志
static uint8_t Arm_ramp_to_target_position_flag = 0;  // 臂臂斜坡移动至目标点标志
static uint8_t MUC_mode_flag = 0; // 给小电脑的模式标志
static uint8_t Arm_inside_flag = 0; // 臂臂收回肚子标志

static Vector3 arm_rc_contro_place;
static float assorted_yaw_angle, assorted_roll_angle, big_yaw_angle, big_yaw_origin_angle;   //关节角度值
static float Z_current_height;  //Z轴高度
static uint8_t joint_crash_flag;    // 关节碰撞标志

/* 自动模式相关 */
static int32_t arm_height_ramp_feriod,arm_joint_ramp_feriod; //自动模式斜坡周期
static uint8_t auto_mode_step_id[11] = {0};//各自动模式所进行到的步骤id
static uint16_t auto_mode_doing_state = 0; //各自动模式进行时标志位 
static float arm_automode_target_offset_x,arm_automode_target_offset_y,arm_automode_target_mode; //自动模式斜坡平移的一些参数
static float custom_contro_offset_yaw;

/* 图传链路接收数据 */
float qua_rec[4];
float encoder_Data[3];
uint8_t custom_controller_comm_recv = 0; //自定义发送的自定义标志位	
uint8_t custom_controller_data_refresh_flag = 0; // 图传链路数据更新标志

/* 右手坐标系下关节角度 */
static float big_yaw_angle_;
static float mid_yaw_angle_; 
static float assorted_yaw_angle_;
static float assorted_roll_angle_;
static float tail_motor_angle_;

static void reset_z_detect(){
    arm_init_flag = 0;
}
// Z轴标定（原理为等待触发Z限位开关）
static void Z_limit_sensor_detect()
{
    // Z_GPIO = HAL_GPIO_ReadPin(Z_limit_detect_GPIO_Port, Z_limit_detect_Pin);
    // if (Z_GPIO == GPIO_PIN_RESET) {
        arm_init_flag |= Z_motor_init_clt;
        if ((z_motor->measure.total_round > 2 || z_motor->measure.total_round < -2)) {
            z_motor->measure.total_round = 0;
            arm_param_t.height = 0;
            DJIMotorStop(z_motor);
        }
    // }
}
// 上位机解析回调函数
static void HOST_RECV_CALLBACK()
{
    uint8_t rec_buf[26];
    memcpy(rec_buf,(uint8_t*)host_instance->rec_buf,host_instance->rec_len);
    if(rec_buf[0]==0xFF&&rec_buf[1]==0x52){
        memcpy((uint8_t*)&arm_recv_host_data, rec_buf+2, sizeof(arm_controller_data_s));

        arm_recv_host_data.big_yaw_angle *= 1;
        // arm_recv_host_data.height *=100;   // 现行方案考虑不使用自定义控制器控制高度
        // arm_recv_host_data.height = arm_recv_host_data.height;
        arm_recv_host_data.height = arm_contro_data.height;
        arm_recv_host_data.assorted_roll_angle *= 1;
        arm_recv_host_data.assorted_yaw_angle *= 1;
        arm_recv_host_data.tail_motor_angle *= -1;
        arm_recv_host_data.mid_yaw_angle *= 1;
        host_rec_flag = 1;
    }   
}
// 上位机解析回调函数
static void HOST_USART_RECV_CALLBACK()
{
    uint8_t rec_buf[26];
    memcpy(rec_buf,(uint8_t*)host_uart_instance->recv_buff,host_uart_instance->recv_buff_size);
    if(rec_buf[0]==0xFF&&rec_buf[1]==0x52){
        memcpy((uint8_t*)&arm_recv_host_data, rec_buf+2, sizeof(arm_controller_data_s));

        arm_recv_host_data.big_yaw_angle *= 1;
        // arm_recv_host_data.height *=100;   // 现行方案考虑不使用自定义控制器控制高度
        // arm_recv_host_data.height = arm_recv_host_data.height;
        arm_recv_host_data.height = arm_contro_data.height;
        arm_recv_host_data.assorted_roll_angle *= 1;
        arm_recv_host_data.assorted_yaw_angle *= 1;
        arm_recv_host_data.tail_motor_angle *= -1;
        arm_recv_host_data.mid_yaw_angle *= 1;
        host_rec_flag = 1;
    }   
}

void ArmInit()
{
    USART_Init_Config_s host_uart_instance_conf = {
        .module_callback = HOST_USART_RECV_CALLBACK,
        .checkout_callback = NULL,
        .recv_buff_size  = 26,
        .usart_handle    = &huart10, // 达妙板子的原理图写USART3，但实际管脚对应的是UART1
    };
    host_uart_instance       = USARTRegister(&host_uart_instance_conf);// 图传串口

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
    encoder_config.can_init_config.rx_id      = 0x4fb;
    encoder_config.offset                     = tail_roll_encoder_offset;
    // tail_roll_encoder                           = EncoderInit(&encoder_config);
    Motor_Init_Config_s mid_yaw_motor_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 10, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 25,
                .MaxOut        = middle_speed_limit,
            },
            .speed_PID = {
                .Kp            = 0.5, // 0
                .Ki            = 1.604,  // 0
                .Kd            = 0.005,      // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 2,
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
                .Kp            = 0.35, // 4.5
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
                .Kp            = 0.8, // 4.5
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
                .Kp            = 10, // 0
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
            // .angle_feedback_source = OTHER_FEED,
            .angle_feedback_source = MOTOR_FEED,
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
            .can_handle = &hfdcan3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 3, // 0
                .Ki            = 0, // 0
                .Kd            = 0, // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 25,
                .MaxOut        = big_yaw_speed_limit,
            },
            .speed_PID = {
                .Kp            = 0.4, // 0
                .Ki            = 0.001, // 0
                .Kd            = 0.001, // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 1,
                .MaxOut        = 10, // 20000
            },
            .other_angle_feedback_ptr = &big_yaw_angle,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
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
                          .Kp            = 2000, // 0
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
        // .comm_mode = HOST_USART,
        .RECV_SIZE = 26,
        // .usart_handle = &huart10,
    };

    GPIO_Init_Config_s Z_limit_sensor_gpio_conf = {
        .GPIOx = Z_limit_detect_GPIO_Port,
        .GPIO_Pin = Z_limit_detect_Pin,
        .exti_mode = GPIO_EXTI_MODE_FALLING,
        .gpio_model_callback = Z_limit_sensor_detect,
    };
    // IO口初始化
    Z_limit_sensor_gpio = GPIORegister(&Z_limit_sensor_gpio_conf);
    // 电机初始化
    joint_motor[0] = big_yaw_motor                               = DRMotorInit(&big_yaw_init_config);
    joint_motor[1] = mid_yaw_motor                               = DRMotorInit(&mid_yaw_motor_config);
    assorted_motor_config.can_init_config.tx_id = 1;
    joint_motor[2] = assorted_motor_up                           = DJIMotorInit(&assorted_motor_config);
    assorted_motor_config.can_init_config.tx_id = 2;
    joint_motor[3] = assorted_motor_down                         = DJIMotorInit(&assorted_motor_config);
    joint_motor[4] = tail_motor                                  = DJIMotorInit(&tail_motor_config);
    joint_motor[5] = tail_roll_motor                             = DJIMotorInit(&tail_roll_motor_config);
    joint_motor[6] = z_motor                                     = DJIMotorInit(&z_motor_config);
    

    // 外置PID初始化
    assorted_yaw_pid  = PIDRegister(&assorted_yaw_pid_config);
    assorted_roll_pid = PIDRegister(&assorted_roll_pid_config);
    // 消息收发初始化
    arm_cmd_sub   = SubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    arm_data_sub  = PubRegister("arm_data", sizeof(Arm_Data_s));
    host_instance      = HostInit(&host_conf); // 上位机通信串口
    host_send_buf[0]=0xff;host_send_buf[1]=0x52;

}

/* 一些私有函数 */
static void cal_Z_height()  //计算Z轴高度
{
    Z_current_height = z_motor->measure.total_angle / z_motor_ReductionRatio;
}
static void cal_big_yaw_angle(){ ////计算大Yaw角度
    big_yaw_angle = big_yaw_motor->measure.total_angle / 2.0f;
}

static void cal_mid_rAy_angle() //计算混合关节的yaw&roll
{
    assorted_yaw_angle = assorted_yaw_encoder->measure.total_angle;
    // todo:shi
    assorted_roll_angle = (assorted_up_encoder->measure.total_angle + assorted_yaw_encoder->measure.total_angle) / 2.0;
    if(arm_cmd_recv.optimize_signal&0x02)   assorted_roll_angle+=180;   //可选混合roll优化
    assorted_roll_angle = assorted_roll_angle - 360 * (int16_t)(assorted_roll_angle / 360);
    assorted_roll_angle = assorted_roll_angle > 180 ? (assorted_roll_angle - 360) : (assorted_roll_angle < -180 ? (assorted_roll_angle + 360) : assorted_roll_angle);

}

static void cal_joing_angle(){ //计算各关节角度
    cal_mid_rAy_angle();cal_Z_height();cal_big_yaw_angle();

    big_yaw_angle_ = -big_yaw_angle;
    mid_yaw_angle_ = -mid_yaw_motor->measure.total_angle;
    assorted_yaw_angle_ = assorted_yaw_angle;
    assorted_roll_angle_= assorted_roll_angle;
    tail_motor_angle_ = -tail_motor_encoder->measure.total_angle;

    float a1 = abs(assorted_yaw_angle_);
        float a2 = 180 - abs(mid_yaw_angle_);
        float a3 = 180 - a1 - a2;
        big_yaw_origin_angle = (mid_yaw_angle_>=0) ? (big_yaw_angle_ + a3) : (big_yaw_angle_ - a3); 
        float n = sqrtf(arm1*arm1 + arm2*arm2 - 2*arm1*arm2*cosf(a2*DEGREE_2_RAD));
        float a4 = asinf(arm2*sinf(a2)/n) - a3;
        // memset(&arm_rc_contro_place,0,sizeof(arm_rc_contro_place));
        // arm_rc_contro_place.x = arm3 + n*sqrtf(1-sinf(a4)*sinf(a4));
        // arm_rc_contro_place.y = assorted_yaw_angle_>=0 ? -n*sinf(a4) : n*sinf(a4);
        // arm_rc_contro_place.z = 0;
}

static void set_big_yaw_angle(float angle)
{
    VAL_LIMIT(angle, -200, 200);
    DRMotorSetRef(big_yaw_motor, angle);
}
static void set_z_height(float z_height)
{
    if (arm_init_flag & Z_motor_init_clt)
        VAL_LIMIT(z_height, -620, 30);
    else
        VAL_LIMIT(z_height, -620, 620);
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

    VAL_LIMIT(yaw_angle, -90, 85);
    float speed_yaw, speed_roll, speed_up, speed_down;
    cal_mid_rAy_angle();
    
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
// 设置各关节目标角度 按照右手坐标系
static void set_arm_angle(arm_controller_data_s *data)
{
    set_z_height(data->height);

    //Yaw偏向可选优化sucker_state
    // if(arm_cmd_recv.optimize_signal & 0x01){
    //     set_big_yaw_angle(-(big_yaw_origin_angle-data->mid_yaw_angle));
    //     set_mid_yaw_angle(-(data->big_yaw_angle-big_yaw_origin_angle));
    //     set_mid_rAy_angle(data->assorted_roll_angle, -data->assorted_yaw_angle);
    // }else{
        set_big_yaw_angle(-data->big_yaw_angle);
        set_mid_yaw_angle(-data->mid_yaw_angle);
        set_mid_rAy_angle(data->assorted_roll_angle, data->assorted_yaw_angle);
    // }
    
    set_tail_motor_angle(-data->tail_motor_angle);
}

// 根据控制数据变换末端位姿（重构TF树）
static uint8_t cal_ARM_TF(Transform* TF,uint8_t mode,float dx,float dy){
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

    /*
            ↑x
            |
        y←——O———
           z|    
    */
    Vector3 sub_vec;
    //mode:0 基于吸盘朝向进行平移
    if(mode == 0){
        /* 改版正解算式，从arm2处(arm2初始坐标+偏移坐标)重构TF树 */
        // arm2处的旋转矩阵
        float arm3_roll_angle_new                            = degree2rad_limited(assorted_roll_angle_);
        float TransformationMatrix_arm2_origin__origin__q_data[9] = {
            1, 0, 0,
            0, cosf(arm3_roll_angle_new),   -sinf(arm3_roll_angle_new),
            0, sinf(arm3_roll_angle_new),   cosf(arm3_roll_angle_new),
            };
        Matrix TransformationMatrix_arm2_origin__origin__q_m = {3, 3, TransformationMatrix_arm2_origin__origin__q_data};
        // arm3相对于arm2的旋转矩阵
        float arm4_pitch_angle_new                           = degree2rad_limited(tail_motor_angle_);
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
        float TransformationMatrix_target_sub__origin__p_data[3] = {dx,dy,0};
        Matrix TransformationMatrix_target_sub__origin__p_m = {3,1,TransformationMatrix_target_sub__origin__p_data};
        M_mul(&Matrix_keep, &TransformationMatrix_target_origin__origin__q_m,&TransformationMatrix_target_sub__origin__p_m);

        memcpy((float*)&sub_vec,(float*)&Matrix_data_keep,sizeof(Vector3));
        arm_rc_contro_place.x += sub_vec.x;
        arm_rc_contro_place.y += sub_vec.y;
        arm_rc_contro_place.z += sub_vec.z;
        // arm4平移矩阵
        float TransformationMartix_target_arm3__origin__p_data[3] = {arm4,0,0};
        Matrix TransformationMartix_target_arm3__origin__p_m = {3,1,TransformationMartix_target_arm3__origin__p_data};
        M_mul(&Matrix_keep, &TransformationMatrix_target_origin__origin__q_m,&TransformationMartix_target_arm3__origin__p_m);

        // 获取最终位姿
        TF->localPosition.x = Matrix_data_keep[0] + arm_rc_contro_place.x;
        TF->localPosition.y = Matrix_data_keep[1] + arm_rc_contro_place.y;
        TF->localPosition.z = Matrix_data_keep[2] + arm_rc_contro_place.z;
        rotationToQuaternion((float *)TransformationMatrix_target_origin__origin__q_data,&TF->localRotation);
    }else if(mode==1){
    // mode:1 基于世界坐标系进行平移
        sub_vec.x = dx;sub_vec.y = dy;sub_vec.z = 0;
        arm_rc_contro_place.x += sub_vec.x;
        arm_rc_contro_place.y += sub_vec.y;
        arm_rc_contro_place.z += sub_vec.z;
        TF->localPosition.x += sub_vec.x;
        TF->localPosition.y += sub_vec.y;
    }

    // 判断末端位置是否超出有效解范围
    float TF_X       = sqrtf(TF->localPosition.x * TF->localPosition.x + TF->localPosition.y * TF->localPosition.y);
    // 未超出则返回1
    if (limit_bool(TF_X,0.745f,0.430f) && limit_bool(TF->localPosition.z,0.03+arm4,-0.575-arm4))
        return 1;
    // 超出则撤销更改，返回0
    arm_rc_contro_place.x -= sub_vec.x;
    arm_rc_contro_place.y -= sub_vec.y;
    arm_rc_contro_place.z -= sub_vec.z;
    return 0;
}
//根据末端TF树计算关节角度
static uint8_t cal_ARM_joint_angle(Transform* new_TF)
{
    // memcpy(&arm_controller_TF, new_TF, sizeof(Transform)); //方便监视

    arm_controller_data_s update_data;
    if(Update_angle(new_TF, &update_data)) {
        // if (Control_ARM_flag == 0){
        //     arm_origin_place.big_yaw_angle-=update_data.big_yaw_angle;
        //     arm_origin_place.height-=new_TF->localPosition.z * 1000;
        // }else if(Control_ARM_flag==2){
        //     memset(&arm_origin_place,0,sizeof(arm_origin_place));
        // }
        arm_param_t.big_yaw_angle      = arm_origin_place.big_yaw_angle + update_data.big_yaw_angle;
        arm_param_t.mid_yaw_angle      = update_data.mid_yaw_angle;
        arm_param_t.assorted_yaw_angle = update_data.assorted_yaw_angle;
        arm_param_t.assorted_roll_angle = arm_origin_place.assorted_roll_angle;
        arm_param_t.tail_motor_angle = arm_origin_place.tail_motor_angle;
        arm_param_t.height = arm_origin_place.height + new_TF->localPosition.z * 1000;
        return 2;
    }
    return 0;
}
//臂臂使能
static void ArmEnable(){
    DRMotorEnable(big_yaw_motor);
    DRMotorEnable(mid_yaw_motor);
    DJIMotorEnable(assorted_motor_up);
    DJIMotorEnable(assorted_motor_down);
    DJIMotorEnable(tail_motor);
    DJIMotorEnable(z_motor);
    DJIMotorEnable(tail_roll_motor);
}
//臂臂失能
static void ArmDisable(){
    DRMotorStop(mid_yaw_motor);
    DJIMotorStop(assorted_motor_up);
    DJIMotorStop(assorted_motor_down);
    DJIMotorStop(tail_motor);
    DRMotorStop(big_yaw_motor);
    DJIMotorStop(z_motor);
    DJIMotorStop(tail_roll_motor);
}
/* 臂臂各关节堵转检测，原理是当其输出较大电流但未发生位移超过1s时，认为其卡住了，将整个臂臂强制离线 */
// todo:后续看该如何加入复活的操作
static void ArmStuckDetection(){
    static uint16_t joint_move_cnt[5];
    static float joint_origin_angle[5];
    static uint8_t joint_error_flag = 0;
    DJIMotorInstance* djimotor_ptr;
    DRMotorInstance* DRmotor_ptr;
    if(joint_crash_flag !=1 ){
        for(int i = 0;i<5;i++){
            if(((DJIMotorInstance*)joint_motor[i])->motor_type == M2006 || ((DJIMotorInstance*)joint_motor[i])->motor_type == M3508){
                djimotor_ptr = (DJIMotorInstance*)joint_motor[i];
                if(abs(djimotor_ptr->motor_controller.speed_PID.Output) >= djimotor_ptr->motor_controller.speed_PID.MaxOut/4.0f && djimotor_ptr->stop_flag!=0){
                    if(joint_move_cnt[i]==0){
                        joint_origin_angle[i]=djimotor_ptr->measure.total_angle;
                    }
                    joint_move_cnt[i]++;
                    if(joint_move_cnt[i] > 1000 && limit_bool(djimotor_ptr->measure.total_angle-joint_origin_angle[i],10,-10)){
                        joint_error_flag |= (0x0001<<i);
                    }else{
                        joint_move_cnt[i]=0;
                    }
                }else{
                    joint_move_cnt[i] = 0;
                }
            }else{
                DRmotor_ptr = (DRMotorInstance*)joint_motor[i];
                if(abs(DRmotor_ptr->speed_PID.Output) >= DRmotor_ptr->speed_PID.MaxOut/4.0f && DRmotor_ptr->stop_flag!=0){
                    if(joint_move_cnt[i]==0){
                        joint_origin_angle[i]=DRmotor_ptr->measure.total_angle;
                    }
                    joint_move_cnt[i]++;
                    if(joint_move_cnt[i] > 1000 && limit_bool(DRmotor_ptr->measure.total_angle-joint_origin_angle[i],2,-2)){
                        joint_error_flag |= (0x0001<<i);
                    }else{
                        joint_move_cnt[i]=0;
                    }
                }else{
                    joint_move_cnt[i] = 0;
                }
            }
            if(joint_error_flag){
                LOGWARNING("[arm] joint was crashed, id: %x", joint_error_flag);
                joint_crash_flag = 1;
            }
        }
    }
    if(joint_crash_flag == 1){
        ArmDisable();
    }
}

//斜坡设定臂臂关节目标角度-偏差值
static void ArmParamSet_ramp_offset(int32_t ramp_feriod,float big_yaw_angle,float mid_yaw_angle,float assorted_yaw_angle,float assorted_roll_angle,float tail_motor_angle){
    arm_auto_mode_data.big_yaw_angle = big_yaw_angle_ + big_yaw_angle;
    arm_auto_mode_data.mid_yaw_angle = mid_yaw_angle_ + mid_yaw_angle;
    arm_auto_mode_data.assorted_yaw_angle = assorted_yaw_angle_ + assorted_yaw_angle;
    arm_auto_mode_data.assorted_roll_angle = assorted_roll_angle_ + assorted_roll_angle;
    arm_auto_mode_data.tail_motor_angle = tail_motor_angle_ + tail_motor_angle;

    arm_joint_ramp_feriod = ramp_feriod;
    Arm_ramp_to_target_position_flag |= Arm_joint_ramp_flag;
}
//斜坡设定臂臂关节目标角度-绝对值
static void ArmParamSet_ramp(int32_t ramp_feriod,float big_yaw_angle,float mid_yaw_angle,float assorted_yaw_angle,float assorted_roll_angle,float tail_motor_angle){
    arm_auto_mode_data.big_yaw_angle = big_yaw_angle;
    arm_auto_mode_data.mid_yaw_angle = mid_yaw_angle;
    arm_auto_mode_data.assorted_yaw_angle = assorted_yaw_angle;
    arm_auto_mode_data.assorted_roll_angle = assorted_roll_angle;
    arm_auto_mode_data.tail_motor_angle = tail_motor_angle;

    arm_joint_ramp_feriod = ramp_feriod;
    Arm_ramp_to_target_position_flag |= Arm_joint_ramp_flag;
}
//斜坡平移臂末端
static void ArmTargetParamSet_ramp_offset(int32_t ramp_feriod,uint8_t mode,float x_offset,float y_offset){
    arm_automode_target_offset_x = x_offset;
    arm_automode_target_offset_y = y_offset;
    arm_automode_target_mode = mode;
    arm_joint_ramp_feriod = ramp_feriod;
    Arm_ramp_to_target_position_flag |= Arm_target_ramp_flag;
}
//斜坡设定Z轴高度-绝对值
static void Z_heightSet_ramp(int32_t ramp_feriod, float height){
    arm_auto_mode_data.height = height;

    arm_height_ramp_feriod = ramp_feriod;
    Arm_ramp_to_target_position_flag |= Arm_height_ramp_flag;
}
//斜坡平移Z轴-偏差值
static void Z_heightSet_ramp_offset(int32_t ramp_feriod, float height){
    arm_auto_mode_data.height = Z_current_height + height;

    arm_height_ramp_feriod = ramp_feriod;
    Arm_ramp_to_target_position_flag |= Arm_height_ramp_flag;
}
//设置末端吸盘roll偏移值
static void ArmTailRollOffset(float offset_angle){
    DJIMotorOuterLoop(tail_roll_motor,ANGLE_LOOP);
    DJIMotorSetRef(tail_roll_motor,*(tail_roll_motor->motor_controller.other_angle_feedback_ptr) + offset_angle);
}
//基于TF树控制臂臂
//mode:0基于吸盘平移，mode:1基于原点坐标系平移
static void ArmControByTF(Transform *TF,uint8_t mode,float x_offset,float y_offset,float roll_offset,float pitch_offset){
    if (Control_ARM_flag == 0) {
        memcpy(&arm_origin_place,&arm_param_t,sizeof(arm_param_t));
        arm_origin_place.height = Z_current_height;
        arm_origin_place.mid_yaw_angle = -mid_yaw_motor->measure.total_angle;
        arm_origin_place.assorted_yaw_angle = assorted_yaw_angle;
        arm_origin_place.assorted_roll_angle = assorted_roll_angle;
        arm_origin_place.tail_motor_angle =  -tail_motor_encoder->measure.total_angle;
        arm_origin_place.big_yaw_angle = big_yaw_origin_angle;
        // float a1 = abs(assorted_yaw_angle_);
        // float a2 = 180 - abs(mid_yaw_angle_);
        // float a3 = 180 - a1 - a2;
        // arm_origin_place.big_yaw_angle = (mid_yaw_angle_>=0) ? (big_yaw_angle_ + a3) : (big_yaw_angle_ - a3); 
        // float n = sqrtf(arm1*arm1 + arm2*arm2 - 2*arm1*arm2*cosf(a2*DEGREE_2_RAD));
        // float a4 = asinf(arm2*sinf(a2)/n) - a3;
        memset(&arm_rc_contro_place,0,sizeof(arm_rc_contro_place));
        arm_rc_contro_place.x = arm1 + arm2 + arm3;
        // arm_rc_contro_place.x = arm3 + n*sqrtf(1-sinf(a4)*sinf(a4));
        // arm_rc_contro_place.y = assorted_yaw_angle_>=0 ? -n*sinf(a4) : n*sinf(a4);
        // arm_rc_contro_place.z = 0;

        cal_ARM_TF(&arm_controller_TF,0,x_offset,y_offset);
        Control_ARM_flag = 1;
    }
    if(cal_ARM_TF(&arm_controller_TF,mode,x_offset,y_offset)){
        arm_origin_place.assorted_roll_angle += roll_offset;
        arm_origin_place.tail_motor_angle += pitch_offset;
        LIMIT_MIN_MAX(arm_origin_place.tail_motor_angle,-90,90);
        if(!cal_ARM_joint_angle(TF))
        {
            arm_origin_place.assorted_roll_angle -= roll_offset;
            arm_origin_place.tail_motor_angle -= pitch_offset;
        }
    }
}
//臂臂自动模式
static void ArmSetAutoMode(){
    static uint16_t delay_time; //当前步骤延时时间，让臂臂控制气推杆时有一定的等待时间
    if(!(Arm_ramp_to_target_position_flag == 0)) return;  //如正执行着其他操作，则退出
    
    //臂臂从肚子伸出
    //执行其他模式时，需先将臂臂从肚子里取出
    // if((Arm_inside_flag==1 && arm_cmd_recv.auto_mode!=0 && arm_cmd_recv.auto_mode!=Recycle_arm_in) || auto_mode_doing_state&(0x0001<<7)){
    if((Arm_inside_flag==1 && arm_cmd_recv.auto_mode==Recycle_arm_out) || auto_mode_doing_state&(0x0001<<7)){
        auto_mode_doing_state = 0;
        auto_mode_doing_state |= (0x0001<<7);
        if(++auto_mode_step_id[7] > 3) {auto_mode_step_id[7]=0;auto_mode_doing_state&=~(0x0001<<7);return;}
        uint8_t current_auto_mode_step_id = auto_mode_step_id[7];
        memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
        auto_mode_step_id[7] = current_auto_mode_step_id;
        switch(current_auto_mode_step_id){
            case 1:ArmParamSet_ramp(1000,-33.493,116.523773,90.5924835,-96.5074844,-73.6624374);break;
            case 2:Z_heightSet_ramp(1000,-315.116913);break;
            case 3:ArmParamSet_ramp(1000,-40.1328545,5.32804918,89.7918472,-104.0686646,-88.7563782);Arm_inside_flag=0;break;
        }
        return;
    }
    //大Yaw复位
    if(arm_cmd_recv.auto_mode == Arm_big_yaw_reset){
        arm_param_t.big_yaw_angle = 0;
        return;
    }
    //臂臂复位
    if(arm_cmd_recv.auto_mode == Reset_arm_cmd_param_flag){
        ArmParamSet_ramp(2000,-16.2440605,121.290161,-82.473938,5.30348206,-88.7453918);
        if(arm_init_flag & Z_motor_init_clt)
            Z_heightSet_ramp(2000,-148);
        return;
    }

    // 除复位和伸出臂臂外一切自动模式都要在 Z轴初始化完毕 且 臂臂不在肚子内 才能用
    if((arm_init_flag & Z_motor_init_clt) && Arm_inside_flag==0){
        //如果目前的自动模式有延时需求（比如移动夹爪），将进入此函数
        if(delay_time!=0){
            delay_time--;
            return;
        }
        //以下为各自动模式
        //地矿姿势
        if ((arm_cmd_recv.auto_mode==Fetch_gronded_cube_P) || (auto_mode_doing_state & (0x0001<<8))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<8);
            if(++auto_mode_step_id[8] > 1) {auto_mode_step_id[8]=0;auto_mode_doing_state&=~(0x0001<<8);return;}
            uint8_t current_auto_mode_step_id = auto_mode_step_id[8];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[8] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:ArmParamSet_ramp(1500,-2.65963554,9.99989223,91.1747589,3.97276497,90.7150879);break;
            }
            return;
        }
        //前伸姿势
        if ((arm_cmd_recv.auto_mode==Arm_forward_P) || (auto_mode_doing_state & (0x0001<<9))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<9);
            if(++auto_mode_step_id[0] > 1) {auto_mode_step_id[9]=0;auto_mode_doing_state&=~(0x0001<<9);return;}
            uint8_t current_auto_mode_step_id = auto_mode_step_id[9];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[9] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:ArmParamSet_ramp(4000,0,0,0,0,0);Z_heightSet_ramp(4000,0);break;
            }
            return;
        }
        //收臂臂回肚子
        if ((arm_cmd_recv.auto_mode==Recycle_arm_in) || (auto_mode_doing_state & (0x0001<<0))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<0);
            if(++auto_mode_step_id[0] > 3) {auto_mode_step_id[0]=0;auto_mode_doing_state&=~(0x0001<<0);return;}
            uint8_t current_auto_mode_step_id = auto_mode_step_id[0];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[0] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:ArmParamSet_ramp(1000,-40.1328545,5.32804918,89.7918472,-104.0686646,-88.7563782);Z_heightSet_ramp(1000,-510.375061);break;
                case 2:ArmParamSet_ramp(1000,-33.493,119.269684,84.7710648,-96.5074844,-73.6624374);break;
                case 3:ArmParamSet_ramp(500,1.5181303,119.269684,84.7710648,-96.5074844,-73.6624374);Arm_inside_flag=1;break;
            }
            return;
        }
        //取右侧金矿
        if ((arm_cmd_recv.auto_mode==Arm_get_goldcube_right) || (auto_mode_doing_state & (0x0001<<1))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<1);
            if(++auto_mode_step_id[1] > 11) {auto_mode_step_id[1]=0;auto_mode_doing_state&=~(0x0001<<1);return;}
            int t=1;if(auto_mode_step_id[t] == 5 || auto_mode_step_id[t] == 9) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[1];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[1] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_ramp(500,-163.547638);break;
                case 2:ArmParamSet_ramp(1600,-87.6319504,27.0226898,90.7902374,90,60.3486938);break;
                case 3:Z_heightSet_ramp(500,-497.650665);break;
                case 4:ArmParamSet_ramp(2000,-83.6757507,66.9962463,30.7715321,89.5769196,1.0743103);break;
                case 5:ArmParamSet_ramp(1400,-74.9983292,54.3336639,34.7623405,88.4357147,1.03173828);arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;break;
                case 6:ArmParamSet_ramp(2000,-83.6757507,66.9962463,30.7715321,89.5769196,1.0743103);break;
                case 7:Z_heightSet_ramp_offset(300,69.18985);break;
                case 8:ArmParamSet_ramp(2000,-97.2660217,78.6398926,29.6838799,89.5769196,1.0743103);break;
                case 9:ArmParamSet_ramp(2000,-138.88736,71.164772,81.2677765,89.5769196,1.0743103);break;
                case 10:Z_heightSet_ramp(500,-163.547638);break;
                case 11:memset(&arm_data_send,0,sizeof(arm_data_send));
            }
            return;
        }
        //从上矿仓取矿，~~~~~~并将下矿仓的矿石移到上矿仓~~~~~~
        if ((arm_cmd_recv.auto_mode==Arm_fetch_cube_from_warehouse1) || (auto_mode_doing_state & (0x0001<<2))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<2);
            if(++auto_mode_step_id[2] > 14) {auto_mode_step_id[2]=0;auto_mode_doing_state&=~(0x0001<<2);return;}
            // int t=2;if(auto_mode_step_id[t] == 2 || auto_mode_step_id[t] == 3  || auto_mode_step_id[t] == 4 || auto_mode_step_id[t] == 9 || auto_mode_step_id[t] == 10) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[2];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[2] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:ArmParamSet_ramp(2000,-2.11363196,3.643929,4.22839069,3.97276497,90.7150879);Z_heightSet_ramp(2000,-51.8345871);break;
                case 2:ArmParamSet_ramp(3000,50.0072174,61.6729698,74.0812912,3.97276497,90.7150879);break;                    
                case 3:Z_heightSet_ramp(800,-90.694397);break;
                case 4:Z_heightSet_ramp_offset(800,-6.5);arm_data_send.arm_to_airpump&=~AIRPUMP_ARM_CLOSE;arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;arm_data_send.arm_to_airpump&=~AIRPUMP_LINEAR_OPEN;arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_LOOSE;break;
                case 5:Z_heightSet_ramp(500,0);break;
                // case 6:arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_FOREWARD;delay_time = 200;break; //夹爪前伸
                case 7:ArmParamSet_ramp(800,29.1283932,94.9285202,43.5830498,6.77496338,-11);break;
                case 8:ArmParamSet_ramp(1600,0,82.0207977,16.9410286,6.77496338,-11);break; 
                case 10:ArmParamSet_ramp(800,2.14790106,94.1105499,-83.3500977,6.77496338,-11);
                // case 11:arm_data_send.arm_to_airpump &= ~AIRPUMP_LINEAR_OPEN;arm_data_send.arm_to_airpump |= AIRPUMP_LINEAR_CLOSE;break;  //推杆关泵
                // case 12:arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_TIGHTEN;delay_time = 400;break; //夹爪抓取
                // case 13:arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_UP;arm_data_send.arm_to_airpump |= AIRPUMP_LINEAR_CLOSE;delay_time = 2000;break; //夹爪后拉
                case 14:memset(&arm_data_send,0,sizeof(arm_data_send));arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;
            }
            return;
        }
        //从下矿仓取矿
        if ((arm_cmd_recv.auto_mode==Arm_fetch_cube_from_warehouse2) || (auto_mode_doing_state & (0x0001<<11))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<11);
            if(++auto_mode_step_id[11] > 7) {auto_mode_step_id[11]=0;auto_mode_doing_state&=~(0x0001<<11);return;}
            // int t=11;if(auto_mode_step_id[t] == 2) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[11];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[11] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_ramp(600,0);break;
                case 2:ArmParamSet_ramp(1000,-10.0819511,92.8755875,44.306778,3.97276497,90.7150879);break;
                case 3:Z_heightSet_ramp(800,-225.332123);break;                  
                case 4:Z_heightSet_ramp_offset(2000,-20);arm_data_send.arm_to_airpump&=~AIRPUMP_ARM_CLOSE;arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;arm_data_send.arm_to_airpump&=~AIRPUMP_LINEAR_OPEN;arm_data_send.arm_to_airpump |= AIRPUMP_LINEAR_CLOSE;break;
                case 5:Z_heightSet_ramp(600,0);break;
                case 6:ArmParamSet_ramp(1000,2.14790106,94.1105499,-83.3500977,3.97276497,90.7150879);
                case 7:memset(&arm_data_send,0,sizeof(arm_data_send));arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;
            }
            return;
        }
        //取左侧银矿
        if ((arm_cmd_recv.auto_mode==Arm_get_silvercube_left) || (auto_mode_doing_state & (0x0001<<3))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<3);
            if(++auto_mode_step_id[3] > 14) {auto_mode_step_id[3]=0;auto_mode_doing_state&=~(0x0001<<3);return;}
            int t=3;if(auto_mode_step_id[t] == 3 || auto_mode_step_id[t] == 7) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[3];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[3] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_ramp(500,-120);break;
                case 2:ArmParamSet_ramp(600,18.2633591,60,-53.3500977,3.97276497,90.7150879);break;
                case 3:ArmParamSet_ramp(800,36.2633591,-6.26889324,36.693203,3.97276497,90.7150879);break;
                case 4:Z_heightSet_ramp(400,-172.915146);break;
                //wait
                case 5:Z_heightSet_ramp(800,-210);break;
                case 6:Z_heightSet_ramp_offset(2000,-20);arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_LOOSE;break;
                case 7:Z_heightSet_ramp(400,0);break;
                //矿仓上
                case 8:arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_FOREWARD;ArmParamSet_ramp(2000,45.262928,80.7936249,54.1945381,3.97276497,90.7150879);break;
                case 9:ArmTailRollOffset(8379.7548899781);delay_time = 4000;break;
                case 10:Z_heightSet_ramp(800,-65);break;
                case 11:arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_UP;delay_time = 1000;break;
                case 12:arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_TIGHTEN;delay_time = 1000;break;
                case 13:arm_data_send.arm_to_airpump &= ~AIRPUMP_ARM_OPEN;arm_data_send.arm_to_airpump |= AIRPUMP_ARM_CLOSE;delay_time=1000;break;
                case 14:ArmParamSet_ramp(2000,2.14790106,94.1105499,-83.3500977,3.97276497,90.7150879);break;
                case 15:Z_heightSet_ramp(400,-120);break;
                case 16:memset(&arm_data_send,0,sizeof(arm_data_send));
            }
        }
        //取中间银矿
        if ((arm_cmd_recv.auto_mode==Arm_get_silvercube_mid) || (auto_mode_doing_state & (0x0001<<4))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<4);
            if(++auto_mode_step_id[4] > 14) {auto_mode_step_id[4]=0;auto_mode_doing_state&=~(0x0001<<4);return;}
            int t=4;if(auto_mode_step_id[t] == 5 || auto_mode_step_id[t] == 8) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[4];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[4] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_ramp(500,-120);break;
                case 2:ArmParamSet_ramp(2000,2.14790106,94.1105499,-83.3500977,3.97276497,90.7150879);
                case 3:ArmParamSet_ramp(600,-60.6141701,81.6064606,60.8962402,3.97276497,90.7150879);break;
                case 4:ArmParamSet_ramp(800,-30.4179478,52.9236412,56.3712158,3.97276497,90.7150879);break;
                case 5:Z_heightSet_ramp(400,-172.915146);break;
                //wait
                case 6:Z_heightSet_ramp(400,-210);break;
                case 7:Z_heightSet_ramp_offset(1500,-20);arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;break;
                case 8:Z_heightSet_ramp(800,0);break;
                //wait
                case 9:ArmParamSet_ramp(4000,1.06551158,78.8425293,65.3374939,3.97276497,90.7150879);break;
                case 10:Z_heightSet_ramp(800,-229.834473);break;
                case 11:arm_data_send.arm_to_airpump &= ~AIRPUMP_ARM_OPEN;arm_data_send.arm_to_airpump|=AIRPUMP_ARM_CLOSE;arm_data_send.arm_to_airpump |= AIRPUMP_LINEAR_OPEN;delay_time=1000;break;
                case 12:Z_heightSet_ramp(800,0);break;
                case 13:ArmParamSet_ramp(2000,2.14790106,94.1105499,-83.3500977,3.97276497,90.7150879);break;
                case 14:Z_heightSet_ramp(400,-172.915146);break;
                case 15:memset(&arm_data_send,0,sizeof(arm_data_send));arm_data_send.arm_to_airpump |= AIRPUMP_LINEAR_OPEN;break;
            }
        }
        //取右侧银矿
        // if ((arm_cmd_recv.auto_mode==Arm_get_silvercube_right) || (auto_mode_doing_state & (0x0001<<5))){
        //     auto_mode_doing_state = 0;
        //     auto_mode_doing_state |= (0x0001<<5);
        //     if(++auto_mode_step_id[5] > 9) {auto_mode_step_id[5] = 0;auto_mode_doing_state&=~(0x0001<<5);return;}
        //     int t=5;if(auto_mode_step_id[t] == 3) auto_mode_doing_state&=~(0x0001<<t);
        //     uint8_t current_auto_mode_step_id = auto_mode_step_id[5];
        //     memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
        //     auto_mode_step_id[5] = current_auto_mode_step_id;
        //     switch(current_auto_mode_step_id){
        //         case 1:ArmParamSet_ramp(2000,2.14790106,94.1105499,-83.3500977,3.97276497,90.7150879);break;
        //         case 2:Z_heightSet_ramp(400,-172.915146);break;
        //         case 3:ArmParamSet_ramp(800,-38.6365166,6.61812353,83.1231003,3.97276497,90.7150879);break;
                
        //         case 4:Z_heightSet_ramp(800,-210);break;
        //         case 5:Z_heightSet_ramp_offset(800,-20);arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;break;
        //         case 6:Z_heightSet_ramp(400,20);break;
        //         case 7:ArmParamSet_ramp(2000,-78.3753891,80.8179398,82.9555588,3.76951218,90.5914917);break;
        //         case 8:ArmParamSet_ramp(2000,-50.1841698,79.4516525,69.245903,3.76951218,90.5914917);break;
        //         case 9:ArmParamSet_ramp(2000,-2.58515072,63.7057724,41.5807762,-87.8186111,90.6670227);Z_heightSet_ramp(400,-230);break;
        //     }
        // }

    }
}
static void ArmApplyAutoMode(){
    static arm_controller_data_s arm_auto_origin_data;
    static float joint_offset[5],z_height_offset;
    static ramp_t joint_ramp[5],z_height_ramp;
    static uint8_t target_set_over; //电机达到设定值标志
    static float offset_x_translation,offset_y_translation;
    if(Arm_ramp_to_target_position_flag & Arm_joint_ramp_flag){
        memcpy(&arm_auto_origin_data,&arm_contro_data,sizeof(arm_controller_data_s));
            joint_offset[0] = arm_auto_mode_data.big_yaw_angle - arm_auto_origin_data.big_yaw_angle;
            joint_offset[1] = arm_auto_mode_data.mid_yaw_angle - arm_auto_origin_data.mid_yaw_angle;
            joint_offset[2] = arm_auto_mode_data.assorted_yaw_angle - arm_auto_origin_data.assorted_yaw_angle;
            joint_offset[3] = arm_auto_mode_data.assorted_roll_angle - arm_auto_origin_data.assorted_roll_angle;
            joint_offset[4] = arm_auto_mode_data.tail_motor_angle - arm_auto_origin_data.tail_motor_angle;

            ramp_init(&joint_ramp[0],arm_joint_ramp_feriod);
            ramp_init(&joint_ramp[1],arm_joint_ramp_feriod);
            ramp_init(&joint_ramp[2],arm_joint_ramp_feriod);
            ramp_init(&joint_ramp[3],arm_joint_ramp_feriod);
            ramp_init(&joint_ramp[4],arm_joint_ramp_feriod);
        Arm_ramp_to_target_position_flag &= ~Arm_joint_ramp_flag;
        Arm_ramp_to_target_position_flag |= Arm_joint_ramp_doing;
    }else if(Arm_ramp_to_target_position_flag & Arm_joint_ramp_doing){
        arm_contro_data.big_yaw_angle       = arm_auto_origin_data.big_yaw_angle        + joint_offset[0]*ramp_calc(&joint_ramp[0]);
        arm_contro_data.mid_yaw_angle       = arm_auto_origin_data.mid_yaw_angle        + joint_offset[1]*ramp_calc(&joint_ramp[1]);
        arm_contro_data.assorted_yaw_angle  = arm_auto_origin_data.assorted_yaw_angle   + joint_offset[2]*ramp_calc(&joint_ramp[2]);
        arm_contro_data.assorted_roll_angle = arm_auto_origin_data.assorted_roll_angle  + joint_offset[3]*ramp_calc(&joint_ramp[3]);
        arm_contro_data.tail_motor_angle    = arm_auto_origin_data.tail_motor_angle     + joint_offset[4]*ramp_calc(&joint_ramp[4]);
        memcpy(&arm_param_t,&arm_contro_data,sizeof(arm_controller_data_s));
        for(int i=0;i<5;i++)    target_set_over|=(0x0001<<(joint_ramp[i].out==1?i+1:0));
    }

    if(Arm_ramp_to_target_position_flag & Arm_height_ramp_flag){
        arm_auto_origin_data.height = arm_contro_data.height;
        z_height_offset = arm_auto_mode_data.height - arm_auto_origin_data.height;
        ramp_init(&z_height_ramp,arm_height_ramp_feriod);
        Arm_ramp_to_target_position_flag &= ~Arm_height_ramp_flag;
        Arm_ramp_to_target_position_flag |= Arm_height_ramp_doing;
    }else if(Arm_ramp_to_target_position_flag & Arm_height_ramp_doing){
        arm_contro_data.height = arm_auto_origin_data.height + z_height_offset * ramp_calc(&z_height_ramp);
        arm_param_t.height = arm_contro_data.height;
        if(z_height_ramp.out==1)    target_set_over|=0x40;
    }

    if(Arm_ramp_to_target_position_flag & Arm_target_ramp_flag){
        Control_ARM_flag = 0;
        offset_x_translation = arm_automode_target_offset_x / (float)arm_height_ramp_feriod;
        offset_y_translation = arm_automode_target_offset_y / (float)arm_height_ramp_feriod;
        ArmControByTF(&arm_controller_TF,arm_automode_target_mode,0,0,0,0);
        Arm_ramp_to_target_position_flag &= ~Arm_target_ramp_flag;
        Arm_ramp_to_target_position_flag |= Arm_target_ramp_doing;
    }else if(Arm_ramp_to_target_position_flag & Arm_target_ramp_doing){
        ArmControByTF(&arm_controller_TF,arm_automode_target_mode,offset_x_translation,offset_y_translation,0,0);
        arm_height_ramp_feriod--;
        
        if(arm_height_ramp_feriod<=0)    target_set_over|=0x80;
    }

    if((target_set_over&0x3f)==0x3f){
        Arm_ramp_to_target_position_flag &= ~Arm_joint_ramp_doing;
        target_set_over &= ~0x3f;
    }
    if(target_set_over & 0x40){
        Arm_ramp_to_target_position_flag &= ~Arm_height_ramp_doing;
        target_set_over &= ~0x40;
    }
    if(target_set_over & 0x80){
        Arm_ramp_to_target_position_flag &= ~Arm_target_ramp_doing;
        target_set_over &= ~0x80;
    }
}
// Z轴匀速下降参数
static void Z_down_limited_torque(){
    DJIMotorSetRef(z_motor,Z_current_height-10*z_motor_ReductionRatio);
}
// 将臂臂当前位置设置为目标位置
static void reset_arm_param(){
    arm_param_t.big_yaw_angle = -big_yaw_angle;
    arm_param_t.height = Z_current_height;
    arm_param_t.mid_yaw_angle = -mid_yaw_motor->measure.total_angle;
    arm_param_t.assorted_yaw_angle = assorted_yaw_angle;
    arm_param_t.assorted_roll_angle = assorted_roll_angle;
    arm_param_t.tail_motor_angle =  -tail_motor_encoder->measure.total_angle;
    memcpy(&arm_contro_data,&arm_param_t,sizeof(arm_controller_data_s));
}
// 臂的操作手控制函数
static void ArmApplyControMode(){
    static uint8_t arm_position_init_flag = 0;
    if(arm_position_init_flag==0 && arm_cmd_recv.contro_mode != ARM_ZERO_FORCE){
        //每次上电时先保存下当前的角度值，防止臂臂初始位姿为前伸把自己创死
        reset_arm_param();
        ArmDisable();
        arm_position_init_flag = 1;
        return;
    }

    // 控制臂臂 大YAW&Z
    arm_param_t.big_yaw_angle -= arm_cmd_recv.Rotation_yaw;
    if((arm_cmd_recv.contro_mode == ARM_POSE_CONTRO_MODE_1 || arm_cmd_recv.contro_mode == ARM_POSE_CONTRO_MODE_0 || arm_cmd_recv.contro_mode == ARM_POSE_CONTRO_TARGET_MODE)){
        arm_origin_place.big_yaw_angle -= arm_cmd_recv.Rotation_yaw;
    }
    arm_param_t.height += arm_cmd_recv.Position_z;
    if (arm_init_flag & Z_motor_init_clt)
        VAL_LIMIT(arm_param_t.height, -620, 30);
    else
        VAL_LIMIT(arm_param_t.height, -620, 620);

    // 控制臂臂 末端平移&旋转
    // 根据接收数据判断是否需要重置TF树位置
    if(!((arm_cmd_recv.contro_mode == ARM_POSE_CONTRO_MODE_1 || arm_cmd_recv.contro_mode == ARM_POSE_CONTRO_MODE_0) && arm_cmd_recv.convert_flag == 1)){
        Control_ARM_flag = 0;
    }else if(arm_cmd_recv.convert_flag == 1){
        ArmControByTF(&arm_controller_TF,(arm_cmd_recv.contro_mode == ARM_POSE_CONTRO_MODE_1 ? 1 : 0),-arm_cmd_recv.Translation_x,arm_cmd_recv.Translation_y,arm_cmd_recv.Roatation_Horizontal,arm_cmd_recv.Roatation_Vertical);
    }

    memcpy(&arm_contro_data,&arm_param_t,sizeof(arm_controller_data_s));    
}
//上位机控制
static void host_control(){
    /* 处理上位机发送的控制包 */
    if(host_rec_flag==1){
        host_rec_flag=0;

        //todo:视觉信号的判断
        if(custom_controller_comm_recv & 0x01 || arm_cmd_recv.contro_mode == ARM_VISION_CONTRO)
        {
            if(arm_cmd_recv.contro_mode == ARM_VISION_CONTRO){
                arm_recv_host_data.assorted_roll_angle = -arm_recv_host_data.assorted_roll_angle;
                memcpy(&arm_auto_mode_data,&arm_recv_host_data,sizeof(arm_controller_data_s));
                memcpy(&arm_contro_data,&arm_recv_host_data,sizeof(arm_controller_data_s));
                memcpy(&arm_param_t,&arm_contro_data,sizeof(arm_controller_data_s));
            }
            else if(arm_cmd_recv.contro_mode == ARM_CUSTOM_CONTRO){
                // memcpy(&arm_auto_mode_data,&arm_recv_host_data,sizeof(arm_controller_data_s));
                // Arm_ramp_to_target_position_flag |= Arm_joint_ramp_flag;
                // arm_auto_mode_data.big_yaw_angle += big_yaw_origin_angle;
                if(limit_bool(big_yaw_angle_ + custom_contro_offset_yaw,arm_recv_host_data.big_yaw_angle + 4,arm_recv_host_data.big_yaw_angle - 4)){
                    custom_contro_offset_yaw = big_yaw_angle_ - custom_contro_offset_yaw;
                }
                custom_contro_offset_yaw -= arm_cmd_recv.Rotation_yaw; 
                memcpy(&arm_contro_data,&arm_recv_host_data,sizeof(arm_controller_data_s));
                arm_contro_data.big_yaw_angle += custom_contro_offset_yaw;
                memcpy(&arm_param_t,&arm_contro_data,sizeof(arm_controller_data_s));
            }
            

            // arm_joint_ramp_feriod = 20;
            // Arm_ramp_to_target_position_flag |= Arm_joint_ramp_flag;
            // arm_height_ramp_feriod = 20;
            // Arm_ramp_to_target_position_flag |= Arm_height_ramp_flag;
        }
        
    }

    /* 向上位机发送控制请求 */
    //视觉控制优先级最高
    if(arm_cmd_recv.contro_mode == ARM_VISION_CONTRO && arm_cmd_recv.vision_signal == 1){
        MUC_mode_flag = 0x04;
        memcpy(host_send_buf+30,&MUC_mode_flag,1);
        // uint16_t verify_code = 0;
        // for(int i=0;i<30;i++){
        //     verify_code+=host_send_buf[i];
        // }
        // memcpy(host_send_buf+31,&verify_code,sizeof(verify_code));
        USARTSend(host_uart_instance,host_send_buf,31,USART_TRANSFER_BLOCKING);
        // HostSend(host_instance,host_send_buf,31);
    }
    // else{
    //     MUC_mode_flag = 0x00;
    //     memcpy(host_send_buf+30,&MUC_mode_flag,1);
    //     HostSend(host_instance,host_send_buf,31);
    // }
    //自定义控制器
    if(arm_cmd_recv.contro_mode==ARM_CUSTOM_CONTRO && custom_controller_data_refresh_flag==1){
        memcpy(host_send_buf+2,encoder_Data,12);
        memcpy(host_send_buf+14,qua_rec,16);
        MUC_mode_flag = 1;
        memcpy(host_send_buf+30,&MUC_mode_flag,1);
        uint16_t verify_code = 0;
        for(int i=0;i<30;i++){
            verify_code+=host_send_buf[i];
        }
        memcpy(host_send_buf+31,&verify_code,sizeof(verify_code));
        USARTSend(host_uart_instance,host_send_buf,31,USART_TRANSFER_BLOCKING);
        // HostSend(host_instance,host_send_buf,31);
        custom_controller_data_refresh_flag=0;
    }
}
// 控制末端吸盘roll
static void Arm_tail_sucker_contro(){
    if(arm_cmd_recv.sucker_state == 1 || custom_controller_comm_recv&0x02){
        DJIMotorOuterLoop(tail_roll_motor,SPEED_LOOP);
        DJIMotorSetRef(tail_roll_motor,10000);
        // DJIMotorOuterLoop(tail_roll_motor,ANGLE_LOOP);
        // DJIMotorSetRef(tail_roll_motor,3000);
    }else if(arm_cmd_recv.sucker_state == -1 || custom_controller_comm_recv&0x04){
        DJIMotorOuterLoop(tail_roll_motor,SPEED_LOOP);
        DJIMotorSetRef(tail_roll_motor,-10000);
        // DJIMotorOuterLoop(tail_roll_motor,ANGLE_LOOP);
        // DJIMotorSetRef(tail_roll_motor,-3000);
    }else{
        if(tail_roll_motor->motor_settings.outer_loop_type == SPEED_LOOP){
            DJIMotorOuterLoop(tail_roll_motor,SPEED_LOOP);
            DJIMotorSetRef(tail_roll_motor,0);
        }
            
    }
}
/* 机器人机械臂控制核心任务 */
void ArmTask()
{
    
    // 如未接收到cmd命令，卡死在这一步
    while(!SubGetMessage(arm_cmd_sub, &arm_cmd_recv)){
        ArmDisable();
    }
    ArmEnable();
    
    if(arm_cmd_recv.reset_init_flag){
        reset_z_detect();
    }

    /* todo:编码器上电时单圈角度大于180°时，有时候会程序未进入（圈数-1，使初始角度维持在±180）的操作，原因不明 */
    if(tail_motor_encoder->measure.total_angle >= 100){
        tail_motor_encoder->measure.total_round = -1;
        tail_motor_encoder->measure.total_angle = -360+tail_motor_encoder->measure.angle_single_round;
        if(tail_motor_encoder->measure.total_angle <=-100)
        {
            tail_motor_encoder->measure.total_round = 0;
            tail_motor_encoder->measure.total_angle = tail_motor_encoder->measure.angle_single_round;
        }

        DJIMotorStop(tail_motor);
    }
    if(assorted_yaw_encoder->measure.total_angle >= 100){
        assorted_yaw_encoder->measure.total_round = -1;
        assorted_yaw_encoder->measure.total_angle = -360+assorted_yaw_encoder->measure.angle_single_round;
        if(assorted_yaw_encoder->measure.total_angle <=-100)
        {
            assorted_yaw_encoder->measure.total_round = 0;
            assorted_yaw_encoder->measure.total_angle = assorted_yaw_encoder->measure.angle_single_round;
        }
        DJIMotorStop(assorted_motor_down);
        DJIMotorStop(assorted_motor_up);
    }

    //机器人若死亡，需重新标定
    if(arm_cmd_recv.init_call == 1){
        arm_init_flag = 0;
    }

    //自定义控制器模式由自定义控制进入
    if((custom_controller_comm_recv & 1) && (arm_cmd_recv.contro_mode!=ARM_ZERO_FORCE)){
        arm_cmd_recv.contro_mode = ARM_CUSTOM_CONTRO;
    }
    //进行Z轴标定-丢给中断了
    // Z_GPIO = HAL_GPIO_ReadPin(Z_limit_detect_GPIO_Port, Z_limit_detect_Pin);
    // Z_limit_sensor_detect();

    //计算臂臂关节角度，方便调试
    cal_joing_angle();
    

    /* 臂臂控制中，上位机优先级最高，自动模式靠后，手动模式最低 */
    if(arm_cmd_recv.contro_mode == ARM_VISION_CONTRO || arm_cmd_recv.contro_mode == ARM_CUSTOM_CONTRO){
        //上位机控制监测
        host_control();
    }else {
        //监测自动操作请求
        ArmSetAutoMode();
    }

    //如设定了目标点，先移到位，再允许操作手进行自定义操作
    if(Arm_ramp_to_target_position_flag!=0){
        ArmApplyAutoMode();
    }else{
        if(!(arm_cmd_recv.auto_mode==Recycle_arm_in))
            ArmApplyControMode();
    }

    // 臂臂紧急制动
    if (arm_cmd_recv.contro_mode == ARM_ZERO_FORCE) { 
        ArmDisable();
    }

    // 临时暂停
    if(arm_cmd_recv.halt_temp_call == 1){
        reset_arm_param();
        auto_mode_doing_state = 0;
        Arm_ramp_to_target_position_flag = 0;
    }

    // 强制停止
    if (arm_cmd_recv.halt_force_call == 1) {
        ArmDisable();
        reset_arm_param();
        memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
        auto_mode_doing_state = 0;
        Arm_ramp_to_target_position_flag = 0;
    }

    //臂臂堵转检测
    // ArmStuckDetection();
    
    //设定控制命令
    set_arm_angle(&arm_contro_data);

    //控制末端吸盘roll
    Arm_tail_sucker_contro();
    
    if(arm_cmd_recv.auto_mode == Fetch_gronded_cube && Arm_inside_flag!=1)    Z_down_limited_torque();

    arm_data_send.big_yaw_angle = big_yaw_angle;
    arm_data_send.auto_mode_doing_state = auto_mode_doing_state;
    arm_data_send.control_mode_t = custom_controller_comm_recv;
    PubPushMessage(arm_data_sub,&arm_data_send);
    // host_send_buf[30] = 0x01;
    // HostSend(host_instance,host_send_buf,31);
    // USARTSend(host_uart_instance,host_send_buf,31,USART_TRANSFER_BLOCKING);
}
