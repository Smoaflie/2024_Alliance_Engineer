#include "DRmotor.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

uint8_t idx                                               = 0;
static DRMotorInstance *drmotor_instance[DR_MOTOR_MX_CNT] = {NULL};

void DRMotorErrorDetection(DRMotorInstance *motor);

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void DRMotorDecode(CANInstance *_instance)
{
    DRMotorInstance *motor     = (DRMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    Motor_Measure_s *measure = &motor->measure;
    uint8_t *rx_buff           = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    float factor = 0.01f; // 系数 电机是将速度和扭矩数据*100后以整数方式传输的

    short speed, torque;
    *(((uint8_t *)(&measure->total_angle)) + 0) = rx_buff[0];
    *(((uint8_t *)(&measure->total_angle)) + 1) = rx_buff[1];
    *(((uint8_t *)(&measure->total_angle)) + 2) = rx_buff[2];
    *(((uint8_t *)(&measure->total_angle)) + 3) = rx_buff[3];
    *(((uint8_t *)(&speed)) + 0)                = rx_buff[4];
    *(((uint8_t *)(&speed)) + 1)                = rx_buff[5];
    *(((uint8_t *)(&torque)) + 0)               = rx_buff[6];
    *(((uint8_t *)(&torque)) + 1)               = rx_buff[7];
    measure->last_speed_aps                     = measure->speed_aps;
    measure->speed_aps                          = speed * factor;
    measure->real_current                       = torque * factor;

    measure->total_round        = measure->total_angle / 360;
    measure->angle_single_round = measure->total_angle - 360 * measure->total_round;

    DRMotorErrorDetection(motor);
}

static void DRMotorLostCallback(void *motor_ptr)
{
    DRMotorInstance *motor = (DRMotorInstance *)motor_ptr;
    LOGWARNING("[DRMotor] motor lost, id: %d", (motor->motor_can_instance->tx_id & (0x1f<<5)) >> 5);

    if (motor->motor_type == DR_PDA04) {
        static uint8_t tx_buf_enable_PDA04_recall[] = {0xF1, 0x55, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00};
        CANTransmit_once(motor->motor_can_instance->can_handle,
                         (motor->motor_can_instance->tx_id & (0x1f << 5)) + 0x1f,
                         tx_buf_enable_PDA04_recall, 2);    
    }
    if (motor->motor_type == DR_B0X) {
        static uint8_t tx_buf_B0X_recall[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        CANTransmit_once(motor->motor_can_instance->can_handle,
                         (motor->motor_can_instance->tx_id & (0x1f << 5)) + 0x1e,
                         tx_buf_B0X_recall, 2);
    }
}

DRMotorInstance *DRMotorInit(Motor_Init_Config_s *config)
{
    DRMotorInstance *motor = (DRMotorInstance *)malloc(sizeof(DRMotorInstance));
    // motor = (DRMotorInstance *)malloc(sizeof(DRMotorInstance));
    memset(motor, 0, sizeof(DRMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    PIDInit(&motor->motor_controller.torque_PID, &config->controller_param_init_config.torque_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;

    motor->motor_error_detection.current = (config->motor_error_detection_config.current!=NULL ? config->motor_error_detection_config.current : &motor->motor_controller.output_current);
    motor->motor_error_detection.last_speed = (config->motor_error_detection_config.last_speed!=NULL ? config->motor_error_detection_config.last_speed : &motor->motor_controller.speed_PID.Last_Measure);
    motor->motor_error_detection.speed = (config->motor_error_detection_config.speed!=NULL ? config->motor_error_detection_config.speed : &motor->motor_controller.speed_PID.Measure);
    motor->motor_error_detection.stuck_speed = config->motor_error_detection_config.stuck_speed==0 ? 1.0f : config->motor_error_detection_config.stuck_speed;
    motor->motor_error_detection.crash_detective_sensitivity = config->motor_error_detection_config.crash_detective_sensitivity==0 ? 5 : config->motor_error_detection_config.crash_detective_sensitivity;
    motor->motor_error_detection.max_current = config->motor_error_detection_config.max_current==0 ? motor->motor_controller.speed_PID.MaxOut : config->motor_error_detection_config.max_current;
    motor->motor_error_detection.stuck_current_ptr = config->motor_error_detection_config.stuck_current_ptr;
    motor->motor_error_detection.error_callback = config->motor_error_detection_config.error_callback;
    motor->motor_error_detection.error_detection_flag = config->motor_error_detection_config.error_detection_flag;

    motor->motor_type = config->motor_type;

    config->can_init_config.id                  = motor;
    config->can_init_config.can_module_callback = DRMotorDecode;

    switch (config->motor_type) {
        case DR_PDA04:
            config->can_init_config.rx_id = (config->can_init_config.tx_id << 5) + 1;
            break;
        case DR_B0X:
        default:
            config->can_init_config.rx_id = (config->can_init_config.tx_id << 5) + 0x03;
            break;
    }

    config->can_init_config.tx_id = (config->can_init_config.tx_id << 5) + 0x1d; // 0x1d为控制扭矩的命令ID
    motor->motor_can_instance          = CANRegister(&config->can_init_config);

    // 避免大然电机进入保护，每次初始化都将其重启
    // DRMotorReset(motor);
    // DR_PDA04电机每次上电都需要手动发送数据包开启实时状态反馈
    if (motor->motor_type == DR_PDA04) {
        static uint8_t tx_buf_enable_PDA04_recall[] = {0xF1, 0x55, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00};
        CANTransmit_once(motor->motor_can_instance->can_handle,
                         (config->can_init_config.tx_id & (0x1f << 5)) + 0x1f,
                         tx_buf_enable_PDA04_recall, 2);
    }

    DRMotorStop(motor); //   默认关闭，避免出问题
    
    drmotor_instance[idx++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback     = DRMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 2, // 20ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}

/* 第一个电机的can instance用于发送数据,向其tx_buff填充数据 */
void DRMotorControl()
{
    float pid_measure, pid_ref;
    DRMotorInstance *motor;
    Motor_Measure_s *measure;
    Motor_Control_Setting_s *setting;
    Motor_Controller_s *motor_controller;

    for (size_t i = 0; i < idx; ++i) {
        motor   = drmotor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        pid_ref = motor_controller->pid_ref;

        //复位请求处理（延时100ms不发送信息)
        if(motor->reboot_call){
            if(motor->reboot_delay_cnt==0)  motor->reboot_call = 0;
            else    motor->reboot_delay_cnt--;
        }
        // DR_B0x电机不支持实施状态反馈，需要自己发送查询命令
        if (motor->motor_type == DR_B0X && !motor->reboot_call) {
            static uint8_t tx_buf_B0X_recall[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            CANTransmit_once(motor->motor_can_instance->can_handle,
                             (motor->motor_can_instance->tx_id & (0x1f << 5)) + 0x1e,
                             tx_buf_B0X_recall, 0.1);
        }

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP) {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle;
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;
        }

        // 电机反转判断
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) {
            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed_aps;
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor_controller->current_feedforward_ptr;
        }

        if (setting->close_loop_type & CURRENT_LOOP) {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        motor_controller->output_current = pid_ref;

        if (motor->stop_flag == MOTOR_STOP) { // 若该电机处于停止状态,直接将发送buff置零
            memset(&motor_controller->output_current, 0, sizeof(float));
        }

        // 发送
        memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
        memcpy(motor->motor_can_instance->tx_buff, &motor_controller->output_current, sizeof(float));
        motor->motor_can_instance->tx_buff[6] = 0x01;
        CANTransmit(motor->motor_can_instance, 0.1);
    }
}

void DRMotorStop(DRMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void MotorOuterLoop(DRMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}
void DRMotorEnable(DRMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENABLED;
}

void DRMotorSetRef(DRMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

uint8_t DRMotorIsOnline(DRMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}

// 异常检测
void DRMotorErrorDetection(DRMotorInstance *motor)
{
    /* todo:理论上该有个更泛用性的代码 */
    // 碰撞检测
    if(motor->motor_error_detection.error_detection_flag & MOTOR_ERROR_DETECTION_CRASH)
    {
        if(motor->motor_error_detection.stuck_current_ptr == NULL)
        {
            uint16_t can_bus;
            can_bus                 = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
            LOGWARNING("[DRmotor] You should set the stuck current, can bus [%d] , id [%d]", can_bus, (motor->motor_can_instance->tx_id & (0x1f<<5)) >> 5);
            while(1);
        }else
        {
            static uint8_t debug_bool__[5];
            static float debug_value[2];
            debug_value[0] = fabsf(*motor->motor_error_detection.last_speed) - fabsf(*motor->motor_error_detection.speed);
            debug_value[1] = fabsf(*motor->motor_error_detection.last_speed / (float)motor->motor_error_detection.crash_detective_sensitivity);
            debug_bool__[4] = debug_value[0] > debug_value[1];
            debug_bool__[0] = !(motor->motor_error_detection.ErrorCode & MOTOR_ERROR_CRASH);
            debug_bool__[1] = (fabsf(*motor->motor_error_detection.last_speed) - fabsf(*motor->motor_error_detection.speed) > fabsf(*motor->motor_error_detection.last_speed / (float)motor->motor_error_detection.crash_detective_sensitivity));
            debug_bool__[2] = (fabsf(*motor->motor_error_detection.last_speed) > motor->motor_error_detection.stuck_speed) ;
            debug_bool__[3] = (fabsf(*motor->motor_error_detection.current) > fabsf(*motor->motor_error_detection.stuck_current_ptr));
            if(!(motor->motor_error_detection.ErrorCode & MOTOR_ERROR_CRASH) && 
                (fabsf(*motor->motor_error_detection.last_speed) - fabsf(*motor->motor_error_detection.speed) > fabsf(*motor->motor_error_detection.last_speed / (float)motor->motor_error_detection.crash_detective_sensitivity)) && 
                (fabsf(*motor->motor_error_detection.last_speed) > motor->motor_error_detection.stuck_speed) && 
                (fabsf(*motor->motor_error_detection.current) > fabsf(*motor->motor_error_detection.stuck_current_ptr)) && 
                motor->stop_flag)
            {
                motor->motor_error_detection.ErrorCode |= MOTOR_ERROR_DETECTION_CRASH;

                uint16_t can_bus;
                can_bus                 = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
                LOGWARNING("[DRmotor] Motor crashed! can bus [%d] , id [%d]", can_bus, (motor->motor_can_instance->tx_id & (0x1f<<5)) >> 5);
            }
        }
    }
    // 堵转检测
    if(motor->motor_error_detection.error_detection_flag  & MOTOR_ERROR_DETECTION_STUCK)
    {
        if(motor->motor_error_detection.stuck_current_ptr == NULL)
        {
            uint16_t can_bus;
            can_bus                 = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
            LOGWARNING("[DRmotor] You must set the stuck current, can bus [%d] , id [%d]", can_bus, (motor->motor_can_instance->tx_id & (0x1f<<5)) >> 5);
            while(1);
        }else
        {
            if(!(motor->motor_error_detection.ErrorCode & MOTOR_ERROR_DETECTION_STUCK) && (fabsf(*(motor->motor_error_detection.speed)) < motor->motor_error_detection.stuck_speed) && (fabsf(*(motor->motor_error_detection.current)) > *(motor->motor_error_detection.stuck_current_ptr)) && motor->stop_flag)
            {
                motor->motor_error_detection.stuck_cnt++;
                if(motor->motor_error_detection.stuck_cnt > 10)
                {
                    motor->motor_error_detection.ErrorCode |= MOTOR_ERROR_DETECTION_STUCK;

                    uint16_t can_bus;
                    can_bus                 = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
                    LOGWARNING("[DRmotor] Motor stucked! can bus [%d] , id [%d]", can_bus, (motor->motor_can_instance->tx_id & (0x1f<<5)) >> 5);
                }
            }
            else
            {
                motor->motor_error_detection.stuck_cnt = 0;
            }
        }
    }

    // 调用异常回调函数
    if(motor->motor_error_detection.ErrorCode != MOTOR_ERROR_NONE)
        if(motor->motor_error_detection.error_callback != NULL)
            motor->motor_error_detection.error_callback(motor);
}   

//大然电机软重启
void DRMotorReset(DRMotorInstance* motor){
    motor->reboot_call = 1;
    motor->reboot_delay_cnt = 100;
    if(motor->motor_type == DR_PDA04){
        static uint8_t tx_buf_DRmotor_reboot[] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        CANTransmit_once(motor->motor_can_instance->can_handle,
                            (motor->motor_can_instance->tx_id & (0x1f << 5)) + 0x08,
                            tx_buf_DRmotor_reboot, 10);
    }else if(motor->motor_type == DR_B0X){
        static uint8_t tx_buf_DRmotor_clearError[] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        CANTransmit_once(motor->motor_can_instance->can_handle,
                            (motor->motor_can_instance->tx_id & (0x1f << 5)) + 0x08,
                            tx_buf_DRmotor_clearError, 10);
        static uint8_t tx_buf_reset_mode[] = {0x33, 0x75, 0x03, 0x00, 0x08, 0x00, 0x00, 0x00};
        CANTransmit_once(motor->motor_can_instance->can_handle,
                         (motor->motor_can_instance->tx_id & (0x1f << 5)) + 0x1f,
                         tx_buf_reset_mode, 2);    
    }
}