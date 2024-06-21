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
#include "bsp_flash.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"

#include "arm.h"
#include "decode.h"

#define assorted_up_encoder_offset  0
#define assorted_yaw_encoder_offset 1
#define tail_motor_encoder_offset   2

#define big_yaw_speed_limit         25
#define z_speed_limit               15000
#define middle_speed_limit          13
#define assorted_yaw_speed_limit        11000
#define assorted_roll_speed_limit        30000
#define tail_motor_speed_limit      50000
#define tail_roll_speed_limit       15000

#define z_motor_ReductionRatio      46.185567f
/*
整个机械臂共有三个Yaw，两个Roll（不计大柱子的yaw）
*/
// 电机实例
static DRMotorInstance *big_yaw_motor; // 大YAW电机
static DJIMotorInstance *z_motor;      // Z轴电机
static DRMotorInstance *mid_yaw_motor;                            // 臂Yaw电机
static DJIMotorInstance *assorted_motor_up, *assorted_motor_down; //  两个2006电机配合控制中段roll&yaw
static DJIMotorInstance *tail_motor;                              //  控制末端yaw的2006电机
static DJIMotorInstance *tail_roll_motor;                         //  控制末端roll吸盘的2006电机
static void*             joint_motor[7];                          // 按顺序存储关节电机的指针
static uint8_t          joint_error_flag[7];                      // 存储关节异常信息
static uint16_t          joint_error_dispose_cnt[7] = {0};                      // 存储关节异常处理过程值
static float big_yaw_stuck_current = 0.5;
static float mid_yaw_stuck_current = 1;
static float assorted_stuck_current = 1500;
static float tail_stuck_current = 900;
static float tail_roll_stuck_current = 6000;
static float z_stuck_current = 10000;   /* 各关节堵转电流 */
static float assorted_detected_speed, assorted_detected_last_speed;
// 编码器实例
static uint32_t encoder_offset[3] = {5452, 167766, 92519};
static EncoderInstance_s *assorted_up_encoder, *assorted_yaw_encoder, *tail_motor_encoder, *tail_roll_encoder; // 四个编码器，大YAW不另设编码器

// PID实例
static PIDInstance *assorted_yaw_pid, *assorted_roll_pid; // 中段两个2006电机只有速度环，在机械臂任务中通过这两个PID计算出二者的应达到的速度

// 臂关节角度控制
static arm_controller_data_s arm_param_t;   //臂臂关节临时参数
static arm_controller_data_s arm_contro_data;   //臂臂控制数据(关节目标角度)
static arm_controller_data_s arm_current_data;   //臂臂当前数据(关节角度)
static arm_controller_data_s arm_recv_host_data; // 上位机的传来的关节角度值
static arm_controller_data_s arm_auto_mode_data; // 臂臂自动模式目标值
            static float arm_custom_control_origin_height;

static Subscriber_t *arm_cmd_sub;  // 臂臂控制信息收
static Publisher_t *arm_data_sub;  // 臂臂数据信息发

// 臂臂控制数据
static Arm_Cmd_Data_s arm_cmd_recv;
static Arm_Data_s arm_data_send;

//与上位机通讯
HOST_ARM_COMM host_comm;

static GPIOInstance *Z_limit_sensor_gpio; // 限位传感器io

static uint8_t arm_init_flag;   // 臂臂初始化标志
static uint8_t Arm_goto_target_position_flag = 0;  // 臂臂移动至目标点标志
static uint8_t Arm_inside_flag = 0; // 臂臂收回肚子标志

static float assorted_yaw_angle, assorted_roll_angle;   //关节角度值
static float Z_current_height;  //Z轴高度

/* 自动模式相关 */
static int32_t arm_height_outtime,arm_joint_outtime,arm_sucker_outtime; //自动模式最大执行时间
static uint8_t auto_mode_step_id[13] = {0};//各自动模式所进行到的步骤id
static uint16_t auto_mode_doing_state = 0; //各自动模式进行时标志位 
static uint16_t auto_mode_delay_time; //自动模式步骤延时时间

/* 图传链路接收数据 */
extern float qua_rec[4];
extern float encoder_Data[3];
extern uint8_t custom_controller_comm_recv; //自定义发送的自定义标志位	
static uint8_t custom_control_enable = 0;

static float roll_offset_angle;

static uint8_t nuc_flag;
// Z轴标定（原理为等待触发Z限位开关）
static void Z_limit_sensor_detect()
{
    arm_init_flag |= Z_motor_init_clt;
    if ((z_motor->measure.total_round > 2 || z_motor->measure.total_round < -2)) {
        z_motor->measure.total_round = 0;
        arm_param_t.height = 0;
        DJIMotorStop(z_motor);
    }
}
// 上位机解析回调函数
static void HOST_RECV_CALLBACK()
{
    uint8_t rec_buf[28];
    memcpy(rec_buf,(uint8_t*)host_comm.host_instance->recv_buff,host_comm.host_instance->recv_buff_size);
    if(rec_buf[0]==0xFF&&rec_buf[1]==0x52){
        nuc_flag = rec_buf[2];
        memcpy((uint8_t*)&arm_recv_host_data, rec_buf+3, sizeof(arm_controller_data_s));
        arm_recv_host_data.big_yaw_angle *= 1;
        arm_recv_host_data.height = (arm_recv_host_data.height-0.4) * 700;
        arm_recv_host_data.mid_yaw_angle *= 1;
        arm_recv_host_data.assorted_roll_angle *= 1;
        arm_recv_host_data.assorted_yaw_angle *= 1;
        arm_recv_host_data.tail_motor_angle *= 1;
        host_comm.host_rec_flag = 1;
    }
}
// 关节异常处理
static void JointErrorCallback()
{
    Motor_Base_s* motor;
    for(int i = 0; i < 7; i++)
    {
        motor = joint_motor[i];
        joint_error_flag[i] = motor->motor_error_detection.ErrorCode;
        if (joint_error_flag[i] & MOTOR_ERROR_CRASH)
            motor->motor_error_detection.ErrorCode |= MOTOR_ERROR_STUCK;
        if (joint_error_flag[i] & MOTOR_ERROR_STUCK)
        {
            motor->motor_controller.speed_PID.MaxOut = *motor->motor_error_detection.stuck_current_ptr;

            if((abs(*motor->motor_error_detection.speed) > motor->motor_error_detection.stuck_speed))
                joint_error_dispose_cnt[i]++;
            if((joint_error_dispose_cnt[i] > 10) || (abs(*motor->motor_error_detection.current) < *motor->motor_error_detection.stuck_current_ptr))
            {
                joint_error_dispose_cnt[i] = 0;
                motor->motor_error_detection.ErrorCode = 0;
                motor->motor_controller.speed_PID.MaxOut = motor->motor_error_detection.max_current;
                LOGWARNING("[arm_joint_error] Joint recover, id [%d]", i);
            
            }
        }
    }
}
void ArmInit_Encoder()
{
    flash_read(ADDR_FLASH_SECTOR_7,encoder_offset,3);
    Encoder_Init_Config_s encoder_config;
    // 编码器初始化
    encoder_config.encoder_type = MT6825;
    encoder_config.can_init_config.can_handle = &hfdcan1;
    encoder_config.can_init_config.rx_id      = 0x1fb;
    encoder_config.offset                     = encoder_offset[0];
    assorted_up_encoder                       = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id      = 0x2fb;
    encoder_config.offset                     = encoder_offset[1];
    assorted_yaw_encoder                      = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id      = 0x3fb;
    encoder_config.offset                     = encoder_offset[2];
    tail_motor_encoder                        = EncoderInit(&encoder_config);
}
void ArmInit_Motor()
{
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
                .MaxOut        = 5, // 10
            },
            .other_angle_feedback_ptr = &arm_current_data.big_yaw_angle,
        },
        .motor_error_detection_config = {
            .error_detection_flag = MOTOR_ERROR_DETECTION_STUCK | MOTOR_ERROR_DETECTION_CRASH,
            .stuck_current_ptr = &big_yaw_stuck_current,
            .crash_detective_sensitivity = 8,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type            = DR_PDA04,
        .can_init_config.tx_id = 2};
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
                .MaxOut        = 5, // 20000
            },
            .other_angle_feedback_ptr = &arm_current_data.mid_yaw_angle,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_error_detection_config = {
            .error_detection_flag = MOTOR_ERROR_DETECTION_STUCK | MOTOR_ERROR_DETECTION_CRASH,
            .stuck_current_ptr = &mid_yaw_stuck_current,
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
                .MaxOut        = 2000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_error_detection_config = {
            .error_detection_flag = MOTOR_ERROR_DETECTION_STUCK | MOTOR_ERROR_DETECTION_CRASH,
            .stuck_current_ptr = &assorted_stuck_current,
            .speed = &assorted_detected_speed,
            .last_speed = &assorted_detected_last_speed,
            .stuck_speed = 50,
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
                .MaxOut        = 1000,
            },
            .other_angle_feedback_ptr = &arm_current_data.tail_motor_angle,
        },
        .motor_error_detection_config = {
            .error_detection_flag = MOTOR_ERROR_DETECTION_STUCK | MOTOR_ERROR_DETECTION_CRASH,
            .stuck_current_ptr = &tail_stuck_current,
            // .speed = &tail_motor_encoder->measure.speed_aps,
            // .last_speed = &tail_motor_encoder->measure.last_speed_aps,
            .stuck_speed = 1000,
            .crash_detective_sensitivity = 2,
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
        .motor_error_detection_config = {
            .error_detection_flag = MOTOR_ERROR_DETECTION_STUCK,
            .stuck_current_ptr = &tail_roll_stuck_current,
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
                .IntegralLimit = 2000,
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
        .motor_error_detection_config = {
            .error_detection_flag = MOTOR_ERROR_DETECTION_STUCK | MOTOR_ERROR_DETECTION_CRASH,
            .stuck_current_ptr = &z_stuck_current,
        },
        .motor_error_detection_config = {
            // .error_detection_flag = MOTOR_ERROR_DETECTION_STUCK,
            .stuck_current_ptr = &z_stuck_current,
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
                          .Kp            = 1500, // 0
                          .Ki            = 0,    // 0
                          .Kd            = 0,    // 0
                          .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                          .IntegralLimit = 0,
                          .MaxOut        = assorted_yaw_speed_limit, // 20000
                      },
                      assorted_roll_pid_config = {
                          .Kp            = 1000, // 0
                          .Ki            = 0,    // 0
                          .Kd            = 0,    // 0
                          .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                          .IntegralLimit = 0,
                          .MaxOut        = assorted_roll_speed_limit, // 20000
    };
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
}
void ArmInit_Communication()
{
    HostInstanceConf host_conf = {
        .callback  = HOST_RECV_CALLBACK,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = 27,
    };
    host_comm.host_instance      = HostInit(&host_conf); // 上位机通信串口
    // 消息收发初始化
    arm_cmd_sub   = SubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    arm_data_sub  = PubRegister("arm_data", sizeof(Arm_Data_s));

    host_comm.host_send_buf[0]=0xff;host_comm.host_send_buf[1]=0x52;
    host_comm.sent_package_flag = 1;
}
void ArmInit_IO()
{
    GPIO_Init_Config_s Z_limit_sensor_gpio_conf = {
        .GPIOx = Z_limit_detect_GPIO_Port,
        .GPIO_Pin = Z_limit_detect_Pin,
        .exti_mode = GPIO_EXTI_MODE_FALLING,
        .gpio_model_callback = Z_limit_sensor_detect,
    };
    // IO口初始化
    Z_limit_sensor_gpio = GPIORegister(&Z_limit_sensor_gpio_conf);
}

/* 一些私有函数 */
static void add_aroll_offset_angle(){ // 修改roll偏移量
    roll_offset_angle += arm_cmd_recv.aroll_angle_offset;
}

static void cal_mid_rAy_angle() //计算混合关节的yaw&roll
{
    assorted_yaw_angle = assorted_yaw_encoder->measure.total_angle;
    // todo:shi
    assorted_roll_angle = (assorted_up_encoder->measure.total_angle + assorted_yaw_encoder->measure.total_angle) / 2.0;
    //可选-修正混合关节roll角度
    static uint8_t optimize_apply_flag = 0;
    if(arm_cmd_recv.optimize_signal&0x02 && !optimize_apply_flag)
    {
        roll_offset_angle = roll_offset_angle > 180 ? roll_offset_angle-180 : roll_offset_angle+180;   
        optimize_apply_flag = 1;
    }else if(!(arm_cmd_recv.optimize_signal&0x02)){
        optimize_apply_flag = 0;
    }
    assorted_roll_angle = assorted_roll_angle - 360 * (int16_t)(assorted_roll_angle / 360);
    assorted_roll_angle = assorted_roll_angle > 180 ? (assorted_roll_angle - 360) : (assorted_roll_angle < -180 ? (assorted_roll_angle + 360) : assorted_roll_angle);
    
    assorted_roll_angle += roll_offset_angle; //roll偏移量
}

static void cal_joing_angle(){ //计算各关节角度

    cal_mid_rAy_angle();
    arm_current_data.big_yaw_angle  = -(big_yaw_motor->measure.total_angle / 2.0f);
    arm_current_data.height         =   Z_current_height= z_motor->measure.total_angle / z_motor_ReductionRatio;
    arm_current_data.mid_yaw_angle  = -mid_yaw_motor->measure.total_angle;
    arm_current_data.assorted_yaw_angle = assorted_yaw_angle;
    arm_current_data.assorted_roll_angle = assorted_roll_angle;
    arm_current_data.tail_motor_angle = -tail_motor_encoder->measure.total_angle;
    
    
}

static void set_big_yaw_angle(float angle)
{
    VAL_LIMIT(angle, -140, 140);
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
    VAL_LIMIT(angle, -100, 120);
    DRMotorSetRef(mid_yaw_motor, angle);
}
static void set_mid_rAy_angle(float roll_angle, float yaw_angle)
{
    roll_angle = roll_angle - 360 * (int16_t)(roll_angle / 360);
    roll_angle = roll_angle > 180 ? (roll_angle - 360) : (roll_angle < -180 ? (roll_angle + 360) : roll_angle);

    VAL_LIMIT(yaw_angle, -90, 85);
    float speed_yaw, speed_roll, speed_up, speed_down;
    // cal_mid_rAy_angle();
    
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
    VAL_LIMIT(angle, -90, 90.5);
    DJIMotorSetRef(tail_motor, angle);
}
// 设置各关节目标角度 按照右手坐标系
static void set_arm_angle(arm_controller_data_s *data)
{
    set_z_height(data->height);
    set_big_yaw_angle(data->big_yaw_angle);
    set_mid_yaw_angle(data->mid_yaw_angle);
    set_mid_rAy_angle(data->assorted_roll_angle, data->assorted_yaw_angle);
    set_tail_motor_angle(data->tail_motor_angle);
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

//斜坡设定臂臂关节目标角度-绝对值
static void ArmParamSet_ramp(int32_t outtime,float big_yaw_angle,float mid_yaw_angle,float assorted_yaw_angle,float assorted_roll_angle,float tail_motor_angle){
    arm_auto_mode_data.big_yaw_angle = big_yaw_angle;
    arm_auto_mode_data.mid_yaw_angle = mid_yaw_angle;
    arm_auto_mode_data.assorted_yaw_angle = assorted_yaw_angle;
    arm_auto_mode_data.assorted_roll_angle = assorted_roll_angle;
    arm_auto_mode_data.tail_motor_angle = tail_motor_angle;

    arm_joint_outtime = outtime;
    Arm_goto_target_position_flag |= Arm_joint_ramp_flag;
}
//斜坡平移臂末端
// static void ArmTargetParamSet_ramp_offset(int32_t ramp_feriod,uint8_t mode,float x_offset,float y_offset){
//     arm_automode_target_offset_x = x_offset;
//     arm_automode_target_offset_y = y_offset;
//     arm_automode_target_mode = mode;
//     arm_joint_outtime = ramp_feriod;
//     Arm_goto_target_position_flag |= Arm_target_ramp_flag;
// }
//使臂到达范围高度内
static void Z_heightSet_range(int32_t outtime, float range_low, float range_high, float setting_height){
    if(arm_current_data.height <= range_high && arm_current_data.height >= range_low){
        return;
    }else{
        arm_auto_mode_data.height = setting_height;

        arm_height_outtime = outtime;
        Arm_goto_target_position_flag |= Arm_height_ramp_flag;
    }
}
//斜坡设定Z轴高度-绝对值
static void Z_heightSet_ramp(int32_t outtime, float height){
    arm_auto_mode_data.height = height;

    arm_height_outtime = outtime;
    Arm_goto_target_position_flag |= Arm_height_ramp_flag;
}
//斜坡平移Z轴-偏差值
static void Z_heightSet_ramp_offset(int32_t outtime, float height){
    arm_auto_mode_data.height = Z_current_height + height;

    arm_height_outtime = outtime;
    Arm_goto_target_position_flag |= Arm_height_ramp_flag;
}
//设置末端吸盘roll偏移值
static void ArmTailRollOffset(int32_t outtime, float offset_angle){
    arm_sucker_outtime = outtime;
    DJIMotorOuterLoop(tail_roll_motor,ANGLE_LOOP);
    DJIMotorSetRef(tail_roll_motor,tail_roll_motor->measure.total_angle + offset_angle);

    Arm_goto_target_position_flag |= Arm_sucker_ramp_flag;
}
//臂臂自动模式
static void ArmSetAutoMode(){
    if(!(Arm_goto_target_position_flag == 0)) return;  //如正执行着其他操作，则退出
    
    //臂臂从肚子伸出
    //执行其他模式时，需先将臂臂从肚子里取出
    if((Arm_inside_flag && arm_cmd_recv.auto_mode && arm_cmd_recv.auto_mode!=Recycle_arm_in) || auto_mode_doing_state&(0x0001<<7) || arm_cmd_recv.auto_mode==Recycle_arm_out){
        auto_mode_doing_state = 0;
        auto_mode_doing_state |= (0x0001<<7);
        if(++auto_mode_step_id[7] > 6) {auto_mode_step_id[7]=0;auto_mode_doing_state&=~(0x0001<<7);return;}
        // int t=7;if(auto_mode_step_id[t] == 1 || auto_mode_step_id[t] == 3) auto_mode_doing_state&=~(0x0001<<t);
        uint8_t current_auto_mode_step_id = auto_mode_step_id[7];
        memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
        auto_mode_step_id[7] = current_auto_mode_step_id;
        switch(current_auto_mode_step_id){
            case 1:ArmParamSet_ramp(4000,0,119.557182,83.1560593,67.1859589,90.5);break;
            case 2:ArmParamSet_ramp(2000,-26,119.557182,83.1560593,67.1859589,90.5);break;
            case 3:Z_heightSet_ramp_offset(1000,230);break;
            case 4:ArmParamSet_ramp(2000,-26,55.8287811,86.9134064,67.1859589,90.5);break;
            case 5:Z_heightSet_ramp_offset(800,100); Arm_inside_flag=0;break;
            case 6:if(arm_init_flag & Z_motor_init_clt) Z_heightSet_range(800,-160,20,-120);break;
        }
        return;
    }
    
    // 除伸出臂臂外一切自动模式都要在 Z轴初始化完毕 且 臂臂不在肚子内 才能用
    if((arm_init_flag & Z_motor_init_clt) && Arm_inside_flag==0){
        //以下为各自动模式
        //收臂臂回肚子
        if ((arm_cmd_recv.auto_mode==Recycle_arm_in) || (auto_mode_doing_state & (0x0001<<0))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<0);
            if(++auto_mode_step_id[0] > 7) {auto_mode_step_id[0]=0;auto_mode_doing_state&=~(0x0001<<0);return;}
            // int t=0;if(auto_mode_step_id[t] == 2 || auto_mode_step_id[t] == 4) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[0];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[0] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_ramp(1000,-100);break;
                case 2:ArmParamSet_ramp(4000,-40.1328545,70,83.1560593,67.1859589,90.5);break;
                case 3:Z_heightSet_ramp(1000,-301.788391);break;
                case 4:ArmParamSet_ramp(4000,-29.0567245,119.557182,83.1560593,67.1859589,90.5);break;
                case 5:Z_heightSet_ramp(1000,-500);break;
                case 6:ArmParamSet_ramp(2000,0,119.557182,83.1560593,67.1859589,90.5);break;
                case 7:Z_heightSet_ramp(1000,-570); auto_mode_step_id[0]=0;auto_mode_doing_state&=~(0x0001<<0);Arm_inside_flag =1;break;
                //由于Arm_inside_flag置1后无法再进入该位置，因此需同时清除stepid和自动模式状态量
            }
            return;
        }
        //臂臂行走模式
        if(arm_cmd_recv.auto_mode == Reset_arm_cmd_param_flag && !Arm_inside_flag && (arm_init_flag & Z_motor_init_clt)){
            ArmParamSet_ramp(2000,102.6894,-80.4619751,-83.837616,0, 90.5);
            // ArmParamSet_ramp(1000,-16.2440605,110.290161,-82.473938,0, -90);
            Z_heightSet_range(800,-160,20,-120);
            return;
        }
        //地矿姿势
        if ((arm_cmd_recv.auto_mode==Arm_fetch_gronded_cube) || (auto_mode_doing_state & (0x0001<<8))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<8);
            if(++auto_mode_step_id[8] > 1) {auto_mode_step_id[8]=0;auto_mode_doing_state&=~(0x0001<<8);return;}
            uint8_t current_auto_mode_step_id = auto_mode_step_id[8];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[8] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:ArmParamSet_ramp(1500,-2.65963554,9.99989223,91.1747589,0, 90.5);break;
            }
            return;
        }
        //兑矿姿势
        if ((arm_cmd_recv.auto_mode==Arm_ConvertCube) || (auto_mode_doing_state & (0x0001<<9))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<9);
            if(++auto_mode_step_id[9] > 3) {auto_mode_step_id[9]=0;auto_mode_doing_state&=~(0x0001<<9);return;}
            uint8_t current_auto_mode_step_id = auto_mode_step_id[9];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[9] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:
                    memcpy(&arm_auto_mode_data, &arm_current_data, sizeof(Arm_Data_s));
                    arm_auto_mode_data.big_yaw_angle = -60;

                    arm_joint_outtime = 2000;
                    Arm_goto_target_position_flag |= Arm_joint_ramp_flag;
                    break;
                case 2:Z_heightSet_range(800,-160,-60,-120);break;
                case 3:ArmParamSet_ramp(2000,-60,-80.4619751,-83.837616,0, 90.5);break;
            }
            return;
        }
        //取右侧金矿
        if ((arm_cmd_recv.auto_mode==Arm_get_goldcube_right) || (auto_mode_doing_state & (0x0001<<1))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<1);
            if(++auto_mode_step_id[1] > 18) {auto_mode_step_id[1]=0;auto_mode_doing_state&=~(0x0001<<1);return;}
            // int t=1;if(auto_mode_step_id[t] == 14 || auto_mode_step_id[t] == 15) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[1];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[1] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_range(800,-160,20,-150);break;
                case 2:ArmParamSet_ramp(4000,-118.019653,80.3512421,44.24086,0,0);break;
                case 3:Z_heightSet_ramp(2000,-500);break;
                case 4:ArmParamSet_ramp(2000,-122.619492,95.0278549,31.6861496,0, 0);break;
                case 5:ArmParamSet_ramp(1400,-99.7973938,79.543335,27.8738689,0, 0);break;
                case 6:ArmParamSet_ramp(1000,-85.9664917,77.9112091,10.4933853,0, 0);airpump_arm_state=1;break;
                case 7:ArmParamSet_ramp(1000,-79.094635,60.0441895,34.6552238,0, 0);auto_mode_delay_time = 400;break;
                case 8:ArmParamSet_ramp(1000,-70.5521851,49.5737457,34.3791924,0, 3);auto_mode_delay_time = 800;break;
                case 9:Z_heightSet_ramp_offset(1000,50);ArmParamSet_ramp(1000,-79.094635,60.0441895,34.6552238,0, 0);break;
                case 10:ArmParamSet_ramp(1000,-100.219162,70.8017273,39.0113335,0, 0);break;
                case 11:ArmParamSet_ramp(1000,-110.731659,79.457222,39.0676384,0, 0);break;
                case 12:ArmParamSet_ramp(1000,-119.958344,83.6705627,41.3102379,0, 0);break;
                case 13:ArmParamSet_ramp(1000,-134.377579,79.5574646,63.6332283,0, 0);break;
                case 14:ArmParamSet_ramp(1000,-138.092636,57.1531143,87.3954391,0, 0);break;
                case 15:Z_heightSet_ramp(1000,0);break;
                case 16:ArmParamSet_ramp(1000,102.6894,-80.4619751,-83.837616,0, 90.5);break;
                case 17:Z_heightSet_ramp(1000,-65);break;
                case 18:memset(&arm_data_send,0,sizeof(arm_data_send));break;
            }
            return;
        }
        //从上矿仓取矿
        if ((arm_cmd_recv.auto_mode==Arm_fetch_cube_from_warehouse1) || (auto_mode_doing_state & (0x0001<<2))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<2);
            if(++auto_mode_step_id[2] > 14) {auto_mode_step_id[2]=0;auto_mode_doing_state&=~(0x0001<<2);return;}
            // int t=2;if(auto_mode_step_id[t] == 4) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[2];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[2] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_range(800,-20,20,0);break;
                case 2:ArmParamSet_ramp(4000,39.6229019,87.8980637,50.4495468,0, 90.5);break;
                case 3:Z_heightSet_ramp(1000,-100);auto_mode_delay_time = 1000;airpump_arm_state = 1;arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_LOOSE;break;                   
                case 5:Z_heightSet_ramp(1000,0);break;
                case 7:ArmParamSet_ramp(2000,-80.4619751,87.8980637,50.4495468,0, 90.5);break;
                case 8:ArmParamSet_ramp(2000,-60,-80.4619751,-83.837616,0, 90.5);break;
                // case 8:ArmParamSet_ramp(800,-95.7200012,0,0,0, 0);break; 
                case 14:memset(&arm_data_send,0,sizeof(arm_data_send));break;
            }
            return;
        }
        //从下矿仓取矿
        if ((arm_cmd_recv.auto_mode==Arm_fetch_cube_from_warehouse2) || (auto_mode_doing_state & (0x0001<<11))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<11);
            if(++auto_mode_step_id[11] > 8) {auto_mode_step_id[11]=0;auto_mode_doing_state&=~(0x0001<<11);return;}
            // int t=11;if(auto_mode_step_id[t] == 3) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[11];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[11] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_range(800,-80,20,-60);break;
                case 2:ArmParamSet_ramp(2000,-11.9717426,100.782181,34.6771965,0, 90.5);break;
                case 3:Z_heightSet_ramp(100,-220.332123);auto_mode_delay_time = 1000;airpump_arm_state=1;airpump_linear_state=0;break;                  
                // case 4:Z_heightSet_ramp_offset(2000,-20);arm_data_send.arm_to_airpump&=~AIRPUMP_ARM_CLOSE;arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;arm_data_send.arm_to_airpump&=~AIRPUMP_LINEAR_OPEN;arm_data_send.arm_to_airpump |= AIRPUMP_LINEAR_CLOSE;airpump_arm_state=1;airpump_linear_state=0;break;
                case 5:Z_heightSet_ramp(600,0);break;
                // case 6:ArmParamSet_ramp(1000,-80.4619751,108.565987,10.2612972,0, 90.5);break;
                // case 7:ArmParamSet_ramp(1000,-80.46197516,48.8115616,10.2612972,0, 90.5);break;
                case 8:ArmParamSet_ramp(2000,-60,-80.4619751,-83.837616,0, 90.5);break;
                // case 8:ArmParamSet_ramp(800,-95.7200012,0,0,0, 0);break; 
            }
            return;
        }
        //取左侧银矿
        if ((arm_cmd_recv.auto_mode==Arm_get_silvercube_left) || (auto_mode_doing_state & (0x0001<<3))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<3);
            if(++auto_mode_step_id[3] > 16) {auto_mode_step_id[3]=0;auto_mode_doing_state&=~(0x0001<<3);return;}
            // int t=3;if(auto_mode_step_id[t] == 2) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[3];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[3] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_range(800,-160,20,-150);ArmParamSet_ramp(4000,61.0629578,4.94228649,-57.9302979,0,90);break;
                // case 2:break;
                // case 3:ArmParamSet_ramp(2000,36.2633591,-6.26889324,36.693203,0, 90.5);break;
                case 3:Z_heightSet_ramp(1000,-215);auto_mode_delay_time = 1000;airpump_arm_state=1;break;
                // case 5:Z_heightSet_ramp(800,-1900);break;
                // case 6:Z_heightSet_ramp_offset(2000,-20);arm_data_send.arm_to_airpump |= AIRPUMP_ARM_OPEN;arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_LOOSE;airpump_arm_state=1;break;
                //wait
                case 4:Z_heightSet_ramp(1000,0);break;
                case 5:ArmParamSet_ramp(4000,63.0299492,2.84913659,0,0,90);ArmTailRollOffset(2000, 12819);break;
                //矿仓上
                case 6:arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_FOREWARD;ArmParamSet_ramp(2000,7.08790779,46.6193237,86.4094086,0, 90.5);break;
                case 7:ArmParamSet_ramp(2000,42.0428123,45.8032913,85.8326187,0, 90.5);break;
                case 8:ArmParamSet_ramp(2000,42.2840767,47.8979836,85.35012054,0, 90.5);break;
                // case 9:ArmParamSet_ramp(2000,54.010807,44.3637238,78.9578857,0, 90.5);break;
                case 10:ArmParamSet_ramp(2000,50.9057617,67.0916061,61.8424454,0, 90.5);break;
                case 11:Z_heightSet_ramp(800,-55);arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_UP;auto_mode_delay_time = 1000;break;
                case 12:arm_data_send.arm_to_airvalve = AIRVALVE_CLAW_TIGHTEN;auto_mode_delay_time = 1000;break;
                case 13:arm_data_send.arm_to_airpump &= ~AIRPUMP_ARM_OPEN;arm_data_send.arm_to_airpump |= AIRPUMP_ARM_CLOSE;auto_mode_delay_time=1000;airpump_arm_state=0;break;
                case 14:ArmParamSet_ramp(2000,61.0565071,-26.2618942,-85.1202698,0, 90.5);break;
                case 15:ArmParamSet_ramp(4000,61.0565071,-26.2618942,-85.1202698,0, 90.5);break;
                case 16:Z_heightSet_ramp(400,-150);memset(&arm_data_send,0,sizeof(arm_data_send));
            }
        }
        //取中间银矿
        if ((arm_cmd_recv.auto_mode==Arm_get_silvercube_mid) || (auto_mode_doing_state & (0x0001<<4))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<4);
            if(++auto_mode_step_id[4] > 14) {auto_mode_step_id[4]=0;auto_mode_doing_state&=~(0x0001<<4);return;}
            // int t=4;if(auto_mode_step_id[t] == 5 ) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[4];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[4] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_range(800,-160,20,-150);break;
                case 2:Z_heightSet_ramp(1000,-215);auto_mode_delay_time = 1000;airpump_arm_state=1;break;
                case 3:Z_heightSet_ramp(1000,0);break;
                case 4:ArmParamSet_ramp(2000,93.6364975,0.00349164009,-85.5597229,0, 90.5);ArmTailRollOffset(2000, 960);break;
                case 5:ArmParamSet_ramp(2000,-14.7307987,106.565987,12.2612972,0, 90.5);break;
                case 10:Z_heightSet_ramp(1000,-229.834473);airpump_linear_state=1;break;
                case 11:auto_mode_delay_time=1000;airpump_arm_state=0;break;
                case 12:Z_heightSet_ramp(1000,0);break;
                case 13:ArmParamSet_ramp(2000,69.1973419,5.78616905,-87.3834839,0, 90.5);break;
                // case 14:Z_heightSet_ramp(400,-172.915146);break;
                case 15:break;
            }
        }
        //取右侧银矿
        if ((arm_cmd_recv.auto_mode==Arm_get_silvercube_right) || (auto_mode_doing_state & (0x0001<<5))){
            auto_mode_doing_state = 0;
            auto_mode_doing_state |= (0x0001<<5);
            if(++auto_mode_step_id[5] > 7) {auto_mode_step_id[5] = 0;auto_mode_doing_state&=~(0x0001<<5);return;}
            // int t=5;if(auto_mode_step_id[t] == 3) auto_mode_doing_state&=~(0x0001<<t);
            uint8_t current_auto_mode_step_id = auto_mode_step_id[5];
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_step_id[5] = current_auto_mode_step_id;
            switch(current_auto_mode_step_id){
                case 1:Z_heightSet_range(800,-160,20,-150);break;
                // case 2:Z_heightSet_ramp(1000,-120);break;
                case 3:ArmParamSet_ramp(2000,24.7690201,-22.8266144,-86.8012085,0, 90.5);airpump_arm_state=1;break;
                case 4:Z_heightSet_ramp(2000,-210);auto_mode_delay_time = 1000;break;
                case 5:Z_heightSet_ramp(1000,0);break;
                case 6:ArmParamSet_ramp(2000,102.6894,-80.4619751,-83.837616,0, 90.5);break;
                case 7:Z_heightSet_ramp(1000,-65);break;
            }
        }
    }
}
static uint8_t ArmJointInPlace(float Tolerance){
    return (abs((arm_current_data.big_yaw_angle - arm_auto_mode_data.big_yaw_angle)) <= Tolerance
            && abs((arm_current_data.mid_yaw_angle - arm_auto_mode_data.mid_yaw_angle)) <= Tolerance
            && abs((arm_current_data.assorted_yaw_angle - arm_auto_mode_data.assorted_yaw_angle)) <= Tolerance
            && abs((arm_current_data.assorted_roll_angle - arm_auto_mode_data.assorted_roll_angle)) <= Tolerance
            && abs((arm_current_data.tail_motor_angle - arm_auto_mode_data.tail_motor_angle)) <= Tolerance);
}
static uint8_t ArmHeightInPlace(float Tolerance){
    return (abs(Z_current_height - arm_auto_mode_data.height) <= Tolerance);
}
static void ArmApplyAutoMode(){
    if(Arm_goto_target_position_flag & Arm_joint_ramp_flag){
        memcpy(&arm_contro_data,&arm_auto_mode_data,sizeof(arm_controller_data_s));
        arm_contro_data.height = Z_current_height;
        memcpy(&arm_param_t,&arm_contro_data,sizeof(arm_controller_data_s));
        Arm_goto_target_position_flag &= ~Arm_joint_ramp_flag;
        Arm_goto_target_position_flag |= Arm_joint_ramp_doing;
    }
    if(Arm_goto_target_position_flag & Arm_joint_ramp_doing){
        if(arm_joint_outtime-- <= 0)
            Arm_goto_target_position_flag &= ~Arm_joint_ramp_doing;
    }
    if(Arm_goto_target_position_flag & Arm_height_ramp_flag){
        arm_param_t.height = arm_contro_data.height = arm_auto_mode_data.height;
        
        Arm_goto_target_position_flag &= ~Arm_height_ramp_flag;
        Arm_goto_target_position_flag |= Arm_height_ramp_doing;
    }
    if(Arm_goto_target_position_flag & Arm_height_ramp_doing){
        if(arm_height_outtime-- <= 0)
            Arm_goto_target_position_flag &= ~Arm_height_ramp_doing;
    }
    if(Arm_goto_target_position_flag & Arm_sucker_ramp_flag){
        Arm_goto_target_position_flag &= ~Arm_sucker_ramp_flag;
        Arm_goto_target_position_flag |= Arm_sucker_ramp_doing;
    }
    if(Arm_goto_target_position_flag & Arm_sucker_ramp_doing){
        if(arm_sucker_outtime-- <= 0)
            Arm_goto_target_position_flag &= ~Arm_sucker_ramp_doing;
    }

    if(ArmJointInPlace(2.0f)){
        Arm_goto_target_position_flag &= ~Arm_joint_ramp_doing;
        arm_joint_outtime = 0;
    }
    if(ArmHeightInPlace(8.0f)){
        Arm_goto_target_position_flag &= ~Arm_height_ramp_doing;
        arm_height_outtime = 0;
    }
    if(abs((tail_roll_motor->motor_controller.angle_PID.Ref) - (tail_roll_motor->motor_controller.angle_PID.Measure)) < 1.0f)
    {
        Arm_goto_target_position_flag &= ~Arm_sucker_ramp_doing;
        arm_sucker_outtime = 0;
    }
}
// Z轴匀速下降参数
static void Z_down_limited_torque(){
    DJIMotorSetRef(z_motor,z_motor->measure.total_angle-20*z_motor_ReductionRatio);
}
// 将臂臂当前位置设置为目标位置
static void reset_arm_param(){
    memcpy(&arm_param_t,&arm_current_data,sizeof(arm_controller_data_s));
    memcpy(&arm_contro_data,&arm_param_t,sizeof(arm_controller_data_s));
}
// 臂的操作手控制函数
static void ArmApplyControMode(){
    static uint8_t arm_position_init_flag = 0;
    if(arm_position_init_flag==0 && arm_cmd_recv.contro_mode != ARM_ZERO_FORCE){
        //每次上电时先保存下当前的角度值，防止臂臂初始位姿为前伸把自己创死
        reset_arm_param();
        arm_position_init_flag = 1;
        //判断臂是否在肚子内
        if((arm_current_data.big_yaw_angle > -42 && arm_current_data.big_yaw_angle < 10.0f)
            && (arm_current_data.mid_yaw_angle > 70)
            && (arm_current_data.assorted_yaw_angle > 70)
            && (!(arm_init_flag & Z_motor_init_clt) || arm_current_data.height < -285))
            Arm_inside_flag = 1;
        return;
    }

    if(!Arm_inside_flag){
        // 控制臂臂 大YAW&Z
        arm_param_t.big_yaw_angle -= arm_cmd_recv.Rotation_yaw;
        arm_param_t.height += arm_cmd_recv.Position_z;
        arm_custom_control_origin_height += arm_cmd_recv.Position_z;    //todo:
        if (arm_init_flag & Z_motor_init_clt)
            VAL_LIMIT(arm_param_t.height, -620, 30);
        else
            VAL_LIMIT(arm_param_t.height, -620, 620);

        // host_comm.sent_package_flag = arm_cmd_recv.host_sent_mode;

        host_comm.translate_param.Front_Back = arm_cmd_recv.Translation_x;
        host_comm.translate_param.Left_Right = arm_cmd_recv.Translation_y;
        host_comm.translate_param.translateMode = arm_cmd_recv.Translation_mode;

        host_comm.rotate_param.rotationUp_Down = arm_cmd_recv.Roatation_Vertical;
        host_comm.rotate_param.rotationLeft_Right = arm_cmd_recv.Roatation_Horizontal;
        host_comm.rotate_param.YawRotation = arm_cmd_recv.Rotation_yaw;
        host_comm.rotate_param.rotationMode = arm_cmd_recv.Rotation_mode;
    }
        

    memcpy(&arm_contro_data,&arm_param_t,sizeof(arm_controller_data_s));    
}
//上位机控制
static void host_control(){
    //兑矿模式-开关自定义控制器模式
    if(arm_cmd_recv.contro_mode == ARM_CUSTOM_CONTRO){
        host_comm.sent_package_flag = 1;
        static uint8_t switch_flag = 0;
        if(custom_controller_comm_recv & 0x01 && !switch_flag){
            custom_control_enable = !custom_control_enable;
            switch_flag = 1;
        }else if(!(custom_controller_comm_recv & 0x01)) switch_flag = 0;
        if(custom_control_enable){
            if(host_comm.host_rec_flag){
                memcpy(&arm_auto_mode_data,&arm_recv_host_data,sizeof(arm_controller_data_s));
                arm_auto_mode_data.height = arm_recv_host_data.height + arm_custom_control_origin_height;
                arm_auto_mode_data.big_yaw_angle += -90;
                arm_joint_outtime = arm_height_outtime = 30;
                Arm_goto_target_position_flag |= Arm_joint_ramp_flag;
                Arm_goto_target_position_flag |= Arm_height_ramp_flag;
                host_comm.host_rec_flag = 0;
            }
        }
    }else{
        custom_control_enable = 0;
        arm_custom_control_origin_height = arm_current_data.height;
    }

}

// 控制末端吸盘roll
static void Arm_tail_sucker_contro(){
    if(arm_cmd_recv.sucker_call == 1 || custom_controller_comm_recv&0x02){
        DJIMotorOuterLoop(tail_roll_motor,SPEED_LOOP);
        DJIMotorSetRef(tail_roll_motor,10000);
    }else if(arm_cmd_recv.sucker_call == -1 || custom_controller_comm_recv&0x04){
        DJIMotorOuterLoop(tail_roll_motor,SPEED_LOOP);
        DJIMotorSetRef(tail_roll_motor,-10000);
    }else{
        if(tail_roll_motor->motor_settings.outer_loop_type == SPEED_LOOP){
            DJIMotorOuterLoop(tail_roll_motor,SPEED_LOOP);
            DJIMotorSetRef(tail_roll_motor,0);
        }
            
    }
}
static void ArmModifyFlashParam(){
    arm_data_send.flash_data.address = ADDR_FLASH_SECTOR_7;
    arm_data_send.flash_data.data = encoder_offset;
    arm_data_send.flash_data.len = 3;
    if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==GPIO_PIN_RESET)  arm_cmd_recv.assorted_roll_encoder_amend_call = 1;

    encoder_offset[assorted_up_encoder_offset] += arm_cmd_recv.assorted_roll_encoder_amend_call;
    assorted_up_encoder->offset = encoder_offset[assorted_up_encoder_offset];
    encoder_offset[tail_motor_encoder_offset] += arm_cmd_recv.tail_encoder_amend_call;
    tail_motor_encoder->offset  =  encoder_offset[tail_motor_encoder_offset];
}
/* 臂各种参数的预处理 */
void ArmParamPretreatment()
{
    //由上位机指令判断是否需要重新标定
    if(arm_cmd_recv.reset_init_flag){
        arm_init_flag = 0;
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

    //调整roll角度
    add_aroll_offset_angle();

    //计算臂臂关节角度，方便调试
    cal_joing_angle();

    //关节异常处理
    JointErrorCallback();
    
    //设置assorted关节堵转检测时应监视的速度值
    //因为堵转监测是在电机模块中进行的，对于使用双编码器确定速度的关节不太方便
    //todo:优化
    if(abs(assorted_yaw_encoder->measure.speed_aps) > abs(assorted_up_encoder->measure.speed_aps)/2.0f)
    {
        assorted_detected_speed = assorted_yaw_encoder->measure.speed_aps;
        assorted_detected_last_speed = assorted_yaw_encoder->measure.last_speed_aps;
    }
    else
    {
        assorted_detected_speed = assorted_up_encoder->measure.speed_aps / 2.0f;
        assorted_detected_last_speed = assorted_up_encoder->measure.last_speed_aps / 2.0f;
    }

    //按需修改编码器偏移量(flash操作)
    ArmModifyFlashParam();
}
void ArmSubMessage()
{
    while(!SubGetMessage(arm_cmd_sub, &arm_cmd_recv))   ArmDisable();   // 如未接收到cmd命令，卡死在这一步
    ArmEnable();
}
void ArmControInterface()
{
    if(!(arm_cmd_recv.contro_mode == ARM_ZERO_FORCE)){
        // 必须在上一个操作完成后，再进行下一操作
        if(Arm_goto_target_position_flag==0)
        {
            /* 先识别自动模式 */
            if(arm_cmd_recv.contro_mode == ARM_AUTO_MODE || auto_mode_doing_state){
                //如果上次执行的自动模式有延时需求（比如移动夹爪），将等待
                if(auto_mode_delay_time!=0)
                    auto_mode_delay_time--;
                else    
                    //监测自动操作请求
                    ArmSetAutoMode();
            }else{
                //监测上位机关节移动命令
                host_control();
            }
        }
        

        //如设定了目标点，先移到位，再允许操作手进行自定义操作
        if(Arm_goto_target_position_flag!=0){
            ArmApplyAutoMode();
        }else{
            ArmApplyControMode();
        }
        // 临时暂停
        if(arm_cmd_recv.halt_temp_call == 1){
            reset_arm_param();
            auto_mode_doing_state = 0;
            Arm_goto_target_position_flag = 0;
            auto_mode_delay_time = 0;
        }

        // 强制停止
        if (arm_cmd_recv.halt_force_call == 1) {
            ArmDisable();
            reset_arm_param();
            memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
            auto_mode_doing_state = 0;
            Arm_goto_target_position_flag = 0;
            auto_mode_delay_time = 0;
        }
        /*发送控制命令*/
        //控制臂各关节
        set_arm_angle(&arm_contro_data);
        //控制末端吸盘roll
        Arm_tail_sucker_contro();
        //Z轴慢速下降
        if(Arm_inside_flag!=1 && arm_cmd_recv.z_slowly_down_call)    Z_down_limited_torque();

    }else{
        // 臂臂紧急制动
        ArmDisable();
        auto_mode_doing_state = 0;
        Arm_goto_target_position_flag = 0;
        auto_mode_delay_time = 0;
    }
}
void ArmCommunicateHOST()
{
    //与上位机的通信 限制发送帧率为50hz
    static uint16_t cnt = 0;
    if(cnt++ > 20)
    {
        switch(host_comm.sent_package_flag)
        {
            case 4:
                host_comm.host_send_buf[2] = 0x01<<3;                
                memcpy(host_comm.host_send_buf+3, &arm_current_data, sizeof(arm_controller_data_s));
                break;
            case 1:
                host_comm.host_send_buf[2] = 0x01<<0;
                memcpy(host_comm.host_send_buf+3, (uint8_t*)&encoder_Data, sizeof(encoder_Data));
                memcpy(host_comm.host_send_buf+15, (uint8_t*)&qua_rec, sizeof(qua_rec));
                break;
            case 2:
                host_comm.host_send_buf[2] = 0x01<<1;
                memcpy(host_comm.host_send_buf+3, (uint8_t*)&encoder_Data, sizeof(encoder_Data));
                memcpy(host_comm.host_send_buf+15, (uint8_t*)&host_comm.rotate_param, sizeof(ARM_ROTATE_PARAM));
                break;
            case 3:
                host_comm.host_send_buf[2] = 0x01<<2;
                memcpy(host_comm.host_send_buf+3, (uint8_t*)&host_comm.translate_param, sizeof(ARM_TRANSLATE_PARAM));
                memcpy(host_comm.host_send_buf+15, (uint8_t*)&host_comm.rotate_param, sizeof(ARM_ROTATE_PARAM));
                
                break;
        }
        host_comm.host_send_buf[2] |= (arm_cmd_recv.optimize_signal & 0x01)<<7;
        HostSend(host_comm.host_instance, host_comm.host_send_buf, sizeof(host_comm.host_send_buf));
    }
}
void ArmPubMessage()
{   
    /*发布数据信息*/
    arm_data_send.big_yaw_angle = arm_current_data.big_yaw_angle;
    arm_data_send.auto_mode_doing_state = 0;
    for (int i = 0; i < sizeof(auto_mode_step_id); i++)
        arm_data_send.auto_mode_doing_state = (auto_mode_step_id[i] || arm_data_send.auto_mode_doing_state);
    PubPushMessage(arm_data_sub,&arm_data_send);
}
void ArmDebugInterface()
{
    // arm_cmd_recv.contro_mode = ARM_CUSTOM_CONTRO;
    // host_control();


    // if(arm_cmd_recv.debug_flag)
    // {
    //     static float debug_value = 80;
        
    //     float speed_yaw  = -PIDCalculate(assorted_yaw_pid, assorted_yaw_angle, debug_value);

    //     DJIMotorSetRef(assorted_motor_up, speed_yaw);
    //     DJIMotorSetRef(assorted_motor_down, speed_yaw);

    //     DJIMotorEnable(assorted_motor_up);
    //     DJIMotorEnable(assorted_motor_down);

        // DJIMotorEnable(tail_motor);
        // DJIMotorSetRef(tail_motor,debug_value);
        // DJIMotorEnable(assorted_motor_up);
        // DJIMotorSetRef(assorted_motor_up,debug_value);
        // DJIMotorEnable(assorted_motor_down);
        // DJIMotorSetRef(assorted_motor_down,debug_value);
        // DRMotorEnable(mid_yaw_motor);
        // DJIMotorOuterLoop(mid_yaw_motor, SPEED_LOOP);
        // DRMotorSetRef(mid_yaw_motor, debug_value);

        // DRMotorEnable(big_yaw_motor);
        // DJIMotorOuterLoop(big_yaw_motor, SPEED_LOOP);
        // DRMotorSetRef(big_yaw_motor,debug_value);
    // }
}