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
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"

#include "arm.h"
#include "decode.h"

#define assorted_up_encoder_offset  3294
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
static Transform arm_controller_TF; //臂臂末端位姿(仅调试监视用)
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
static uint8_t host_rec_flag;       // 上位机接收标志位
static uint8_t host_send_buf[33];   // 上位机发送缓冲区

static DRMotorInstance *big_yaw_motor; // 大YAW电机
static DJIMotorInstance *z_motor;      // Z轴电机

static GPIO_PinState Z_limit_sensor_gpio, big_yaw_limit_sensor_gpio; // 限位传感器io

static uint8_t arm_init_flag;   // 臂臂初始化标志
static uint8_t Control_ARM_flag = 0;    // 控制臂臂位姿标志
static uint8_t Arm_ramp_to_target_position_flag = 0;  // 臂臂斜坡移动至目标点标志
static uint8_t MUC_mode_flag = 0; // 给小电脑的模式标志

static Vector3 arm_rc_contro_place;
static float assorted_yaw_angle, assorted_roll_angle;   //关节角度值
static float Z_current_height;  //Z轴高度
static uint8_t joint_crash_flag;    // 关节碰撞标志
static float big_yaw_angle_offset; //大Yaw关节偏移值
static float big_yaw_angle;     //大Yaw关节角度值
// 自定义控制器接收数据(图传链路)
static float qua_rec[4];
static float encoder_Data[3];
static USARTInstance *vision_usart;
static uint8_t custom_controller_data_refresh_flag = 0; // 图传链路数据更新标志
// 图传链路
static void vision_recv_callback(){
    uint8_t rec_buf[30];
    memcpy(rec_buf,vision_usart->recv_buff,30);
    if(rec_buf[0]==0xff && rec_buf[1]==0xff){
        memcpy((uint8_t*)encoder_Data,(uint8_t*)rec_buf+2,12);
        memcpy((uint8_t*)qua_rec,(uint8_t*)rec_buf+14,16);
        custom_controller_data_refresh_flag = 1;
    }
}
// 上位机解析回调函数
static void HOST_RECV_CALLBACK()
{
    uint8_t rec_buf[26];
    memcpy(rec_buf,(uint8_t*)host_instance->comm_instance,host_instance->rec_len);
    if(rec_buf[0]==0xFF&&rec_buf[1]==0x52){
        memcpy((uint8_t*)&arm_recv_host_data, rec_buf+2, sizeof(arm_controller_data_s));

        arm_recv_host_data.big_yaw_angle *= -1;
        // arm_recv_host_data.height *=100;   // 现行方案考虑不使用自定义控制器
        arm_recv_host_data.height = arm_recv_host_data.height;
        arm_recv_host_data.assorted_roll_angle *= 1;
        arm_recv_host_data.assorted_yaw_angle *= -1;
        arm_recv_host_data.tail_motor_angle *= -1;
        arm_recv_host_data.mid_yaw_angle *= -1;
        host_rec_flag = 1;
    }   
}

void ArmInit()
{
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
                .Kp            = 10, // 0
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
        .RECV_SIZE = 26,
    };
    USART_Init_Config_s vision_usart_conf = {
        .module_callback = vision_recv_callback,
        .recv_buff_size  = 30,
        .usart_handle    = &huart1, // 达妙板子的原理图写USART3，但实际管脚对应的是UART1
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
    // 消息收发初始化
    arm_cmd_sub   = SubRegister("arm_cmd", sizeof(Arm_Cmd_Data_s));
    arm_data_sub  = PubRegister("arm_data", sizeof(Arm_Data_s));
    host_instance      = HostInit(&host_conf); // 上位机通信串口
    vision_usart       = USARTRegister(&vision_usart_conf);// 图传串口
    host_send_buf[0]=0xff;host_send_buf[1]=0x52;
}

/* 一些私有函数 */
static void cal_Z_height()  //计算Z轴高度
{
    Z_current_height = z_motor->measure.total_angle / z_motor_ReductionRatio;
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
static void cal_big_yaw_angle(){ ////计算大Yaw角度
    big_yaw_angle = (big_yaw_motor->measure.total_angle + big_yaw_angle_offset)/2.0f;
}

static void set_big_yaw_angle(float angle)
{
    VAL_LIMIT(angle, -200, 200);
    DRMotorSetRef(big_yaw_motor, angle);
}
static void set_z_height(float z_height)
{
    if (arm_init_flag & Z_motor_init_clt)
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

    //Yaw偏向可选优化
    if(arm_cmd_recv.optimize_signal & 0x01){
        set_big_yaw_angle(data->big_yaw_angle);
        set_mid_yaw_angle(data->mid_yaw_angle);
        set_mid_rAy_angle(data->assorted_roll_angle, -data->assorted_yaw_angle);
    }else{
        set_big_yaw_angle(-data->big_yaw_angle);
        set_mid_yaw_angle(-data->mid_yaw_angle);
        set_mid_rAy_angle(data->assorted_roll_angle, data->assorted_yaw_angle);
    }
    
    set_tail_motor_angle(-data->tail_motor_angle);
}

// Z轴寻找标定位（原理为等待触发Z限位开关）
static void Z_limit_sensor_detect()
{
    Z_limit_sensor_gpio = HAL_GPIO_ReadPin(Z_limit_detect_GPIO_Port, Z_limit_detect_Pin);
    if (Z_limit_sensor_gpio == GPIO_PIN_RESET) {
        arm_init_flag |= Z_motor_init_clt;
        if ((z_motor->measure.total_round > 2 || z_motor->measure.total_round < -2)) {
            z_motor->measure.total_round = 0;
            arm_param_t.height = 0;
            DJIMotorStop(z_motor);

            arm_init_flag |= Z_motor_pub_reset;
        }
    }
}
// YAW寻找标定位（原理为等待触发YAW限位开关）
static void big_yaw_limit_sensor_detect()
{
    big_yaw_limit_sensor_gpio = HAL_GPIO_ReadPin(YAW_limit_detect_GPIO_Port,YAW_limit_detect_Pin);
    // if(big_yaw_limit_sensor_gpio ==GPIO_PIN_RESET && (big_yaw_motor->measure.total_angle > 2 ||big_yaw_motor->measure.total_angle<-2 )){
    //     big_yaw_angle_offset = big_yaw_motor->measure.total_angle-180;
    //     arm_init_flag |= Big_Yaw_motor_init_clt;
    // arm_init_flag |= Big_Yaw_motor_pub_reset;
    // }
}

// 根据控制数据获取末端位姿
static uint8_t cal_ARM_TF(Transform* TF){
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
    float TransformationMatrix_target_sub__origin__p_data[3] = {-arm_cmd_recv.Translation_x,arm_cmd_recv.Translation_y,0};
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
    TF->localPosition.x = arm1+arm2+arm3 + Matrix_data_keep[0] + arm_rc_contro_place.x;
    TF->localPosition.y = Matrix_data_keep[1] + arm_rc_contro_place.y;
    TF->localPosition.z = Matrix_data_keep[2] + arm_rc_contro_place.z;
    rotationToQuaternion((float *)TransformationMatrix_target_origin__origin__q_data,&TF->localRotation);
    // 判断末端位置是否超出有效解范围
    float TF_X       = sqrtf(TF->localPosition.x * TF->localPosition.x + TF->localPosition.y * TF->localPosition.y);
    // 未超出则返回1
    if (limit_bool(TF_X,0.745f,0.4608f) && limit_bool(TF->localPosition.z,0.03+arm4,-0.575-arm4))
        return 1;
    // 超出则撤销更改，返回0
    arm_rc_contro_place.x -= sub_vec.x;
    arm_rc_contro_place.y -= sub_vec.y;
    arm_rc_contro_place.z -= sub_vec.z;
    return 0;
}
//根据末端位姿计算关节角度
static uint8_t cal_ARM_joint_angle(Transform* new_TF)
{
    memcpy(&arm_controller_TF, new_TF, sizeof(Transform)); //方便监视

    arm_controller_data_s update_data;
    if(Update_angle(new_TF, &update_data)) {
        if (Control_ARM_flag == 0){
            arm_origin_place.big_yaw_angle-=update_data.big_yaw_angle;
            arm_origin_place.height-=new_TF->localPosition.z * 1000;
        }else if(Control_ARM_flag==2){
            memset(&arm_origin_place,0,sizeof(arm_origin_place));
        }
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
/* 臂臂各关节碰撞检测，原理是当其以较大输出电流运动超过5s时，认为其卡住了，将整个臂臂强制离线 */
// todo:后续看该如何加入复活的操作
static void ArmCrashDetection(){
    static uint16_t joint_crash_cnt[5];
    DJIMotorInstance* djimotor_ptr;
    DRMotorInstance* DRmotor_ptr;
    for(int i = 0;i<5;i++){
        if(((DJIMotorInstance*)joint_motor[i])->motor_type == M2006){
            djimotor_ptr = (DJIMotorInstance*)joint_motor[i];
            if(djimotor_ptr->motor_controller.speed_PID.Output == djimotor_ptr->motor_controller.speed_PID.MaxOut && djimotor_ptr->stop_flag!=0){
                joint_crash_cnt[i]++;
            }else{
                joint_crash_cnt[i] = 0;
            }
            
        }else{
            DRmotor_ptr = (DRMotorInstance*)joint_motor[i];
            if(DRmotor_ptr->speed_PID.Output == DRmotor_ptr->speed_PID.MaxOut && DRmotor_ptr->stop_flag!=0){
                joint_crash_cnt[i]++;
            }else{
                joint_crash_cnt[i] = 0;
            }
        }
        if(joint_crash_cnt[i] > 5000){
            if(i==0)    continue;//忽略大yaw，大然电机有自己的碰撞检测
            LOGWARNING("[arm] joint was crashed, id: %d", (i));
            joint_crash_flag = 1;
        }
    }

    if(joint_crash_flag == 1){
        ArmDisable();
    }
}

//快速修改臂臂关节目标角度
static void ArmParamSet(arm_controller_data_s *arm_data,float big_yaw_angle,float mid_yaw_angle,float assorted_yaw_angle,float assorted_roll_angle,float tail_motor_angle){
    arm_data->big_yaw_angle = big_yaw_angle;
    arm_data->mid_yaw_angle = mid_yaw_angle;
    arm_data->assorted_yaw_angle = assorted_yaw_angle;
    arm_data->assorted_roll_angle = assorted_roll_angle;
    arm_data->tail_motor_angle = tail_motor_angle;
    if(arm_data == &arm_auto_mode_data){
        Arm_ramp_to_target_position_flag = 1;
    }
}
//臂臂自动模式
static void ArmSetAutoMode(){
    static uint8_t current_step_clt_flag = 0;
    static uint8_t step_id[5] = {0};
    if(!Arm_ramp_to_target_position_flag == 0) return;
    if(arm_cmd_recv.auto_mode == Arm_big_yaw_reset){
        arm_param_t.big_yaw_angle = 0;
        return;
    }
    if(arm_cmd_recv.auto_mode == Reset_arm_cmd_param_flag && current_step_clt_flag == 0){
        ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);
        // todo : 要不要复位Z轴？
        // memset(&arm_param_t,0,sizeof(arm_param_t));
        current_step_clt_flag = 1;
    }
    // 除复位外一切自动模式都要在Z轴和Yaw轴初始化完毕后才能用
    if(arm_init_flag & Z_motor_init_clt){
        switch(arm_cmd_recv.auto_mode){
            case Recycle_arm_out:
                if(current_step_clt_flag == 0){
                        if(limit_bool(Z_current_height,30,-472)){
                        ArmParamSet(&arm_auto_mode_data,-8.71066189,88.2430878,68.3408966,0,90);
                        current_step_clt_flag = 1;
                    }
                }
                break;
            case Arm_get_goldcube_mid:
                if(current_step_clt_flag == 0){
                        if(++step_id[1] > 7) step_id[1] = 1;
                        uint8_t current_step_id = step_id[1];
                        memset(step_id,0,sizeof(step_id));
                        step_id[1] = current_step_id;
                        switch(current_step_id){
                            case 1:ArmParamSet(&arm_auto_mode_data,-8.71066189,88.2430878,68.3408966,0,90);break;
                            case 2:arm_param_t.height = -163.547638;break;
                            case 3:ArmParamSet(&arm_auto_mode_data,-118.402702,84.9066162,52.7223587,0,0);break;
                            case 4:arm_param_t.height = -459.219788;break;
                            case 5:ArmParamSet(&arm_auto_mode_data,-79.3,53.6279755,38.5952225,0,0);break;
                            case 6:ArmParamSet(&arm_auto_mode_data,-126.7,84.9066162,51.8324623,0,0);break;
                            case 7:arm_param_t.height = -163.547638;break;
                            default:break;
                        }
                        current_step_clt_flag = 1;
                }
                break;
            case Arm_get_goldcube_right:
                if(current_step_clt_flag == 0){
                        if(++step_id[2] > 4) step_id[2] = 1;
                        uint8_t current_step_id = step_id[2];
                        memset(step_id,0,sizeof(step_id));
                        step_id[2] = current_step_id;
                        switch(current_step_id){
                            case 1:ArmParamSet(&arm_auto_mode_data,-57.7395706,118.620804,-61.7206116,0,0);break;
                            // case 2:ArmParamSet(&arm_auto_mode_data,-33.7127266,72.9181747,-37.2868652,0,0);break;
                            case 3:ArmParamSet(&arm_auto_mode_data,-11.7228384,20.3095837,-0.724121094,0,0);break;
                            case 4:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            default:break;
                        }
                        current_step_clt_flag = 1;
                }
                break;
            case Arm_fetch_cube_from_warehouse1:
                if(current_step_clt_flag == 0){
                        if(++step_id[3] > 4) step_id[3] = 1;
                        uint8_t current_step_id = step_id[3];
                        memset(step_id,0,sizeof(step_id));
                        step_id[3] = current_step_id;
                        switch(current_step_id){
                            case 1:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            case 2:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            case 3:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            case 4:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            default:break;
                        }
                        current_step_clt_flag = 1;
                }
                break;
            case Arm_fetch_cube_from_warehouse2:
                if(current_step_clt_flag == 0){
                        if(++step_id[4] > 4) step_id[4] = 1;
                        uint8_t current_step_id = step_id[4];
                        memset(step_id,0,sizeof(step_id));
                        step_id[4] = current_step_id;
                        switch(current_step_id){
                            case 1:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            case 2:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            case 3:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            case 4:ArmParamSet(&arm_auto_mode_data,0,0,0,0,0);break;
                            default:break;
                        }
                        current_step_clt_flag = 1;
                }
                break;
            default:
                current_step_clt_flag = 0;
                break;
        }
    }
}
static void ArmApplyAutoMode(){
    static arm_controller_data_s arm_auto_origin_data;
    static float joint_offset[5];
    static ramp_t joint_ramp[5];
    static uint8_t joint_set_over;
    if(Arm_ramp_to_target_position_flag == 1){
        memcpy(&arm_auto_origin_data,&arm_contro_data,sizeof(arm_controller_data_s));
            joint_offset[0] = arm_auto_mode_data.big_yaw_angle - arm_auto_origin_data.big_yaw_angle;
            joint_offset[1] = arm_auto_mode_data.mid_yaw_angle - arm_auto_origin_data.mid_yaw_angle;
            joint_offset[2] = arm_auto_mode_data.assorted_yaw_angle - arm_auto_origin_data.assorted_yaw_angle;
            joint_offset[3] = arm_auto_mode_data.assorted_roll_angle - arm_auto_origin_data.assorted_roll_angle;
            joint_offset[4] = arm_auto_mode_data.tail_motor_angle - arm_auto_origin_data.tail_motor_angle;

            ramp_init(&joint_ramp[0],100);
            ramp_init(&joint_ramp[1],100);
            ramp_init(&joint_ramp[2],100);
            ramp_init(&joint_ramp[3],100);
            ramp_init(&joint_ramp[4],100);
        Arm_ramp_to_target_position_flag = 2;
    }else{
        arm_contro_data.big_yaw_angle       = arm_auto_origin_data.big_yaw_angle        + joint_offset[0]*ramp_calc(&joint_ramp[0]);
        arm_contro_data.mid_yaw_angle       = arm_auto_origin_data.mid_yaw_angle        + joint_offset[1]*ramp_calc(&joint_ramp[1]);
        arm_contro_data.assorted_yaw_angle  = arm_auto_origin_data.assorted_yaw_angle   + joint_offset[2]*ramp_calc(&joint_ramp[2]);
        arm_contro_data.assorted_roll_angle = arm_auto_origin_data.assorted_roll_angle  + joint_offset[3]*ramp_calc(&joint_ramp[3]);
        arm_contro_data.tail_motor_angle    = arm_auto_origin_data.tail_motor_angle     + joint_offset[4]*ramp_calc(&joint_ramp[4]);
        memcpy(&arm_param_t,&arm_contro_data,sizeof(arm_controller_data_s));
        for(int i=0;i<5;i++)    joint_set_over|=(0x01<<(joint_ramp[i].out==1?i+1:0));
    }

    if(joint_set_over==0x3f){
        Arm_ramp_to_target_position_flag = 0;
        joint_set_over = 0;
    }
}

static void ArmApplyControMode(){
    if (arm_cmd_recv.contro_mode == ARM_CONTROL_BY_KEYBOARD){
    // 如果为键鼠控制臂，则不用分那么多模式（按键足够多）
        // 控制臂臂 大YAW&Z
        arm_param_t.big_yaw_angle -= arm_cmd_recv.Rotation_yaw;
        arm_param_t.height += arm_cmd_recv.Position_z;
        if (arm_init_flag & Z_motor_init_clt)
            VAL_LIMIT(arm_param_t.height, -575, 30);
        else
            VAL_LIMIT(arm_param_t.height, -575, 575);

    }else{
    // 否则按模式控制臂臂各处
        if (arm_cmd_recv.contro_mode == ARM_POSE_CONTRO_MODE) { // 控制臂臂 大YAW&Z
            Control_ARM_flag = 0;

            arm_param_t.big_yaw_angle -= arm_cmd_recv.Rotation_yaw;
            arm_param_t.height += arm_cmd_recv.Position_z;
            if (arm_init_flag & Z_motor_init_clt)
                VAL_LIMIT(arm_param_t.height, -575, 30);
            else
                VAL_LIMIT(arm_param_t.height, -575, 575);

            memcpy(&arm_contro_data,&arm_param_t,sizeof(arm_controller_data_s));
        } else if (arm_cmd_recv.contro_mode == ARM_REFER_MODE) { // 控制臂臂 末端位姿
            Transform TF_p;
            if(cal_ARM_TF(&TF_p)){
                if (Control_ARM_flag == 0) {
                    memcpy(&arm_origin_place,&arm_param_t,sizeof(arm_param_t));
                    memset(&arm_param_t,0,sizeof(arm_rc_contro_place));
                    cal_ARM_joint_angle(&TF_p);
                    Control_ARM_flag = 1;
                }else{
                    arm_origin_place.assorted_roll_angle += arm_cmd_recv.Roatation_Horizontal;
                    arm_origin_place.tail_motor_angle += arm_cmd_recv.Roatation_Vertical;
                    LIMIT_MIN_MAX(arm_origin_place.tail_motor_angle,-90,90);
                    if(!cal_ARM_joint_angle(&TF_p))
                    {
                        arm_origin_place.assorted_roll_angle -= arm_cmd_recv.Roatation_Horizontal;
                        arm_origin_place.tail_motor_angle -= arm_cmd_recv.Roatation_Vertical;
                    }
                }
                memcpy(&arm_contro_data,&arm_param_t,sizeof(arm_controller_data_s));
            }
        }else if(arm_cmd_recv.contro_mode == ARM_FIXED){ // 不控制臂臂，保持当前位置
            Control_ARM_flag = 0;

            static uint8_t switch_flag = 0;
            if(switch_flag==0){
                //每次上电时先保存下当前的角度值，防止臂臂初始位姿为前伸把自己创死
                arm_param_t.big_yaw_angle = -big_yaw_angle;
                arm_param_t.height = Z_current_height;
                arm_param_t.mid_yaw_angle = -mid_yaw_motor->measure.total_angle;
                arm_param_t.assorted_yaw_angle = assorted_yaw_angle;
                arm_param_t.assorted_roll_angle = assorted_roll_angle;
                arm_param_t.tail_motor_angle =  -tail_motor_encoder->measure.total_angle;
                switch_flag = 1;
                memcpy(&arm_contro_data,&arm_param_t,sizeof(arm_controller_data_s));
            }
        }
    }

    // 失能臂臂
    if (arm_cmd_recv.contro_mode == ARM_ZERO_FORCE) { 
        Control_ARM_flag = 0;

        ArmDisable();
    }
    
}
//上位机控制
static void host_control(){
    /* 处理上位机发送的控制包 */
    if(host_rec_flag==1){
        host_rec_flag=0;
        Arm_ramp_to_target_position_flag = 1;
        memcpy(&arm_auto_mode_data,&arm_recv_host_data,sizeof(arm_controller_data_s));
    }

    /* 向上位机发送控制请求 */
    //视觉控制优先级最高
    if(arm_cmd_recv.contro_mode == Arm_Control_by_vision && arm_cmd_recv.vision_signal == 1){
        MUC_mode_flag = 0x04;
        memcpy(host_send_buf+30,&MUC_mode_flag,1);
        uint16_t verify_code = 0;
        for(int i=0;i<30;i++){
            verify_code+=host_send_buf[i];
        }
        memcpy(host_send_buf+31,&verify_code,sizeof(verify_code));
        HostSend(host_instance,host_send_buf,33);
    }
    //自定义控制器
    if(arm_cmd_recv.contro_mode==Arm_Control_by_Custom_controller && custom_controller_data_refresh_flag==1){
        memcpy(host_send_buf+2,encoder_Data,12);
        memcpy(host_send_buf+14,qua_rec,16);
        MUC_mode_flag = 0;
        memcpy(host_send_buf+30,&MUC_mode_flag,1);
        uint16_t verify_code = 0;
        for(int i=0;i<30;i++){
            verify_code+=host_send_buf[i];
        }
        memcpy(host_send_buf+31,&verify_code,sizeof(verify_code));
        HostSend(host_instance,host_send_buf,33);
        custom_controller_data_refresh_flag=0;
    }
}
// 控制末端吸盘roll
static void Arm_tail_sucker_contro(){
    if(arm_cmd_recv.sucker_state == 1){
        DJIMotorSetRef(tail_roll_motor,10000);
    }else if(arm_cmd_recv.sucker_state == -1){
        DJIMotorSetRef(tail_roll_motor,-10000);
    }else{
        DJIMotorSetRef(tail_roll_motor,0);
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

    
    //进行限位检测
    Z_limit_sensor_detect();
    big_yaw_limit_sensor_detect();

    //计算臂臂关节角度，方便调试
    cal_big_yaw_angle();
    cal_mid_rAy_angle();
    cal_Z_height();
    
    //控制末端吸盘roll
    Arm_tail_sucker_contro();

    /* 臂臂控制中，上位机优先级最高，自动模式靠后，手动模式最低 */
    if(arm_cmd_recv.contro_mode == (Arm_Control_by_Custom_controller|Arm_Control_by_vision)){
        //上位机控制监测
        host_control();
    }else {
        //监测自动操作请求
        ArmSetAutoMode();
    }

    //如设定了目标点，先移到位，再允许操作手进行自定义操作
    if(Arm_ramp_to_target_position_flag&(0x01|0x02)){
        ArmApplyAutoMode();
    }else{
        ArmApplyControMode();
    }

    //臂臂碰撞检测
    ArmCrashDetection();
    
    //设定控制命令
    set_arm_angle(&arm_contro_data);


    if(arm_init_flag & Big_Yaw_motor_init_clt)
        arm_data_send.big_yaw_angle = big_yaw_angle;
    PubPushMessage(arm_data_sub,&arm_data_send);
}
