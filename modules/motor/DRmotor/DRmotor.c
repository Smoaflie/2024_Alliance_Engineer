#include "DRMotor.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

uint8_t idx = 0;
static DRMotorInstance *drmotor_instance[DR_MOTOR_MX_CNT] = {NULL};

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void DRMotorDecode(CANInstance *_instance)
{
    DRMotorInstance *motor = (DRMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    DRMotor_Measure_t *measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    float factor = 0.01f;//系数 电机是将速度和扭矩数据*100后以整数方式传输的

    short speed,torque;
    *(((uint8_t*)(&measure->total_angle)) + 0) = rx_buff[0];
    *(((uint8_t*)(&measure->total_angle)) + 1) = rx_buff[1];
    *(((uint8_t*)(&measure->total_angle)) + 2) = rx_buff[2];
    *(((uint8_t*)(&measure->total_angle)) + 3) = rx_buff[3];
    *(((uint8_t*)(&speed)) + 0) = rx_buff[4];
    *(((uint8_t*)(&speed)) + 1) = rx_buff[5];
    *(((uint8_t*)(&torque)) + 0) = rx_buff[6];
    *(((uint8_t*)(&torque)) + 1) = rx_buff[7];
    measure->speed_rads=speed*factor;
    measure->real_current=torque*factor;

    // measure->speed_rads = (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
    //                       DEGREE_2_RAD * SPEED_SMOOTH_COEF * measure->speed_rads;

    // measure->real_current = (1 - CURRENT_SMOOTH_COEF) * measure->real_current +
    //                         CURRENT_SMOOTH_COEF * measure->real_current;

    // measure->temperature = rx_buff[1];

    measure->total_round = measure->total_angle / 360;
    measure->angle_single_round = measure->total_angle - 360 * measure->total_round;
}

static void DRMotorLostCallback(void *motor_ptr)
{
    DRMotorInstance *motor = (DRMotorInstance *)motor_ptr;
    LOGWARNING("[DRMotor] motor lost, id: %d", motor->motor_can_ins->tx_id);
}

DRMotorInstance *DRMotorInit(Motor_Init_Config_s *config)
{
    DRMotorInstance *motor = (DRMotorInstance *)malloc(sizeof(DRMotorInstance));
    // motor = (DRMotorInstance *)malloc(sizeof(DRMotorInstance));
    memset(motor, 0, sizeof(DRMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    motor->motor_type = config->motor_type;

    config->can_init_config.id = motor;
    config->can_init_config.can_module_callback = DRMotorDecode;

    switch(config->motor_type){
        case DR_PDA04:
            config->can_init_config.rx_id = (config->can_init_config.tx_id << 5) + 1;
            break;
        case DR_B0X:
        default:
            config->can_init_config.rx_id = (config->can_init_config.tx_id << 5) + 0x03;
            break;
    }

    config->can_init_config.tx_id = (config->can_init_config.tx_id << 5) + 0x1d;  //0x1d为控制扭矩的命令ID
    motor->motor_can_ins = CANRegister(&config->can_init_config);

    //DR_PDA04电机每次上电都需要手动发送数据包开启实时状态反馈
    if(motor->motor_type == DR_PDA04){
        CANInstance can_tmp_transmit_instance = {
            .can_handle = motor->motor_can_ins->can_handle,
            .tx_mailbox = motor->motor_can_ins->tx_mailbox,
            .txconf = motor->motor_can_ins->txconf,
            .tx_buff = {0xF1,0x55,0x03,0x00,0x01,0x00,0x00,0x00},
            .txconf.StdId = (config->can_init_config.tx_id << 5) + 0x1f,
        };
        CANTransmit(&can_tmp_transmit_instance, 2);
    }
    

    DRMotorStop(motor);//   默认关闭，避免出问题
    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);
    drmotor_instance[idx++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback = DRMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5, // 50ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}

/* 第一个电机的can instance用于发送数据,向其tx_buff填充数据 */
void DRMotorControl()
{
    float pid_measure, pid_ref;
    float set;
    DRMotorInstance *motor;
    DRMotor_Measure_t *measure;
    Motor_Control_Setting_s *setting;

    for (size_t i = 0; i < idx; ++i)
    {
        motor = drmotor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        pid_ref = motor->pid_ref;

        //DR_B0x电机不支持实施状态反馈，需要自己发送查询命令
        if(motor->motor_type == DR_B0X){
            CANInstance can_tmp_transmit_instance = {
                .can_handle = motor->motor_can_ins->can_handle,
                .tx_mailbox = motor->motor_can_ins->tx_mailbox,
                .txconf = motor->motor_can_ins->txconf,
                .tx_buff = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                .txconf.StdId = (motor->motor_can_ins->tx_id & (0x1f << 5)) + 0x1e,
                .txconf.DLC = 8,    // Q: 如果不手动设置DLC，在运行一段时间后会发长度为0的包，不知道为什么
            };
            CANTransmit(&can_tmp_transmit_instance, 2);
        }

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle;
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->speed_feedforward_ptr;
        }

        //电机反转判断
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed_rads;
            pid_ref = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor->current_feedforward_ptr;
        }

        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor->current_PID, measure->real_current, pid_ref);
        }

        set = pid_ref;
        
        memset(motor->motor_can_ins->tx_buff, 0, sizeof(motor->motor_can_ins->tx_buff));

        memcpy(motor->motor_can_ins->tx_buff, &set, sizeof(float));
        motor->motor_can_ins->tx_buff[6] = 0x01;

        if (motor->stop_flag == MOTOR_STOP)
        { // 若该电机处于停止状态,直接将发送buff置零
            memset(motor->motor_can_ins->tx_buff, 0, sizeof(float));
        }
        
        //发送
        CANTransmit(motor->motor_can_ins, 2);
    }        
}

void DRMotorStop(DRMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DRMotorEnable(DRMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENABLED;
}

void DRMotorSetRef(DRMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

uint8_t DRMotorIsOnline(DRMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}
