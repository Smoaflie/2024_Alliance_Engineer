#include "YSA1.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "user_lib.h"

static uint8_t idx;                                                                                   // 记录注册宇树电机的个数
static YSMotorInstance *YSmotor_instance[YS_MOTOR_MX_CNT] = {NULL};                                   // 最多可以有四个宇树电机实例
static USARTInstance *sender_instance[YS_MOTOR_MX_CNT]    = {NULL};                                   // 注册后保存串口实例，方便解算数据
static uint8_t A1_Motor1_Tx_Message[34]                   = {0};                                      // 发送数据帧
YSMotor_Measure_t *YSMotor_Measure_TX;                                                                // 发送时使用的电机数据
YSMotorInstance *YSMotorInit(YSMotor_Init_Config_s *config, UART_HandleTypeDef *ysmotor_usart_handle) // 宇树电机注册函数
{
    YSMotorInstance *motor = (YSMotorInstance *)zmalloc(sizeof(YSMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->usart_init_config.module_callback = YSMotor_Data_Decode;
    config->usart_init_config.recv_buff_size  = YSmotor_rx_data_size;
    config->usart_init_config.usart_handle    = ysmotor_usart_handle;
    motor->motor_usart_ins                    = USARTRegister(&config->usart_init_config);

    YSmotor_instance[idx++] = motor;

    return motor;
}

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void YSMotor_Data_Decode()
{
    for (size_t i = 0; i < idx; ++i) {
        uint8_t *rx_buff = sender_instance[i]->recv_buff;

        size_t Motor_id = rx_buff[2];
        YSMotor_Measure_t rx_motor;
        rx_motor.MotorID           = rx_buff[2];
        rx_motor.Temp              = rx_buff[6];
        rx_motor.Current_Torque    = (rx_buff[12]) | (rx_buff[13] << 8);
        rx_motor.Current_Speed     = (rx_buff[14]) | (rx_buff[15] << 8);
        rx_motor.Current_Acc       = (rx_buff[26]) | (rx_buff[27] << 8);
        rx_motor.Current_Pos       = (rx_buff[30]) | (rx_buff[31] << 8) | (rx_buff[32] << 16) | (rx_buff[33] << 24);
        rx_motor.Current_Pos_rad   = (float)rx_motor.Current_Pos * 2.0f * PI / 16384 / 9.1f;
        rx_motor.Current_Pos_angle = rx_motor.Current_Pos_rad * RAD_TO_ANGLE;
        rx_motor.Current_Torque_Nm = (float)rx_motor.Current_Torque / 256 * 9.1f;

        YSmotor_instance[Motor_id]->measure = rx_motor;
    }
}

void YSMotorStop(YSMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void YSMotorEnable(YSMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENABLED;
}

void YSMotorSetRef(YSMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void YSMotorControl()
{
    float pid_measure, pid_ref;
    int16_t set;
    YSMotorInstance *motor;
    YSMotor_Measure_t *measure       = NULL;
    Motor_Control_Setting_s *setting = NULL;

    for (size_t i = 0; i < idx; ++i) {
        motor   = YSmotor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        pid_ref = motor->pid_ref;

        *YSMotor_Measure_TX = YSmotor_instance[i]->measure;
        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP) {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_angle_feedback_ptr;
            else
                pid_measure = measure->Torque;
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->speed_feedforward_ptr;
        }

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_speed_feedback_ptr;
            else
                pid_measure = measure->Speed;
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor->current_feedforward_ptr;
        }

        if (setting->close_loop_type & CURRENT_LOOP) {
            pid_ref = PIDCalculate(&motor->current_PID, measure->Torque, pid_ref);
        }

        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;

        YSMotor_Measure_TX->Torque = set;

        A1_Motor1_Tx_Message[0]  = 0xFE;
        A1_Motor1_Tx_Message[1]  = 0xEE;
        A1_Motor1_Tx_Message[2]  = YSMotor_Measure_TX->MotorID;
        A1_Motor1_Tx_Message[4]  = YSMotor_Measure_TX->Mode; // 电机运行模式
        A1_Motor1_Tx_Message[5]  = 0xFF;
        A1_Motor1_Tx_Message[12] = YSMotor_Measure_TX->Torque; // Max 128
        A1_Motor1_Tx_Message[13] = YSMotor_Measure_TX->Torque >> 8;
        A1_Motor1_Tx_Message[14] = YSMotor_Measure_TX->Speed; // Max 256
        A1_Motor1_Tx_Message[15] = YSMotor_Measure_TX->Speed >> 8;
        A1_Motor1_Tx_Message[16] = YSMotor_Measure_TX->Position; // MAX 823549  电机位置 = Positin * 9.1（减速比） rad
        A1_Motor1_Tx_Message[17] = YSMotor_Measure_TX->Position >> 8;
        A1_Motor1_Tx_Message[18] = YSMotor_Measure_TX->Position >> 16;
        A1_Motor1_Tx_Message[19] = YSMotor_Measure_TX->Position >> 24;
        A1_Motor1_Tx_Message[20] = YSMotor_Measure_TX->K_P; // Max 16 电机位置刚度
        A1_Motor1_Tx_Message[21] = YSMotor_Measure_TX->K_P >> 8;
        A1_Motor1_Tx_Message[22] = YSMotor_Measure_TX->K_W; // Max 32 电机速度刚度
        A1_Motor1_Tx_Message[23] = YSMotor_Measure_TX->K_W >> 8;

        USARTSend(sender_instance[i], A1_Motor1_Tx_Message, YSmotor_tx_len, USART_TRANSFER_DMA);
    }
}