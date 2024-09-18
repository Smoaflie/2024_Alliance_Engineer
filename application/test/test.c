// app
#include "robot_def.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "encoder.h"
#include "led.h"
#include "bsp_usb.h"
#include "crc_ref.h"
#include "can_comm.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "bsp_usb.h"

#include "test.h"
#include "usermode.hpp"
#include "AHRS.h"

// #include "usermode.hpp"

static DJIMotorInstance *dji_motor[3];
// 编码器实例
static EncoderInstance_s *encoder[3];


// UART实例
static USARTInstance *uart_rec; //陀螺仪串口
static USARTInstance *vision_uart_rec;  //图传链路串口

// 陀螺仪欧拉角
float eul_rec[3];   //欧拉角-调试用
float qua_rec[4];   //发送数据
float qua_rec_raw_f[4];//原始数据
float qua_origin[4];//零点位置
float encoder_Data[3];//编码器

// int16_t qua_rec_raw[4];

// void decode_encoder_can(CANInstance * instance_){
//     memcpy((uint8_t*)qua_rec_raw_f, (uint8_t*)instance_->rx_buff, sizeof(qua_rec_raw));
//     for(int i = 0; i<4; i++){
//         qua_rec_raw_f[i] = qua_rec_raw_f[i] / 10000.0;
//     }
// }

typedef struct {
    float w, x, y, z;
} Quaternion;
static Quaternion rec_q,origin_q;


// 保留位
static uint8_t send_flag = 0;

static void DecodeUART()
{
    if(uart_rec->recv_buff[0] == 0x5A && uart_rec->recv_buff[1] == 0xA5){
        memcpy((uint8_t*)qua_rec_raw_f,(uint8_t*)uart_rec->recv_buff+66,16);
    }else{
        // HAL_UART_DeInit(&huart1);
        // HAL_UART_Init(&huart1);
        // HAL_UART_Receive_DMA(&huart1,uart_rec->recv_buff,uart_rec->recv_buff_size);
        return;
    }
}
static float angle_kp = 400;//200
static float speed_kp = 0.2;//0.6
void TestInit()
{
    Encoder_Init_Config_s encoder_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .encoder_type = me02_can,
        };
    encoder_config.can_init_config.rx_id = 0x061;
    encoder_config.offset = 30811;
    encoder[1]                      = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id = 0x072;
    encoder_config.offset = 1870;

    encoder[2]                     = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id = 0x050;
    encoder_config.offset = 11529;

    encoder[0]                     = EncoderInit(&encoder_config);

    // Motor_Init_Config_s motor_config = {
    //     .can_init_config.can_handle   = &hcan1,
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp            = angle_kp, //800
    //             .Ki            = 0,  // 500
    //             .Kd            = 0,  // 0
    //             .IntegralLimit = 5000,
    //             .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .MaxOut        = 10000,
    //             },
    //         .speed_PID = {
    //             .Kp            = speed_kp, // 4.5
    //             .Ki            = 0,  // 0
    //             .Kd            = 0,  // 0
    //             .IntegralLimit = 3000,
    //             .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .MaxOut        = 5000,
    //             },
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = OTHER_FEED,
    //         .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type       = ANGLE_LOOP,
    //         .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
    //         .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
    //     },
    //     .motor_type = M2006,
    // };
    // motor_config.can_init_config.tx_id = 1;
    // motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[1]->measure.total_angle;
    // dji_motor[1]                           = DJIMotorInit(&motor_config);
    // motor_config.can_init_config.tx_id = 2;
    // motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[2]->measure.total_angle;
    // dji_motor[2]                         = DJIMotorInit(&motor_config);
    // motor_config.can_init_config.tx_id = 3;
    // motor_config.controller_param_init_config.other_angle_feedback_ptr = &encoder[0]->measure.total_angle;
    // dji_motor[0]                         = DJIMotorInit(&motor_config);

    // DJIMotorSetRef(dji_motor[1],0);
    // DJIMotorSetRef(dji_motor[2],0);
    // DJIMotorSetRef(dji_motor[0],0);
    
    // DJIMotorEnable(dji_motor[0]);
    // DJIMotorEnable(dji_motor[1]);
    // DJIMotorEnable(dji_motor[2]);


    USART_Init_Config_s UART_Init_Config = {
        .module_callback = DecodeUART,
        .recv_buff_size = 82,
        .usart_handle = &huart1,
    };
    uart_rec = USARTRegister(&UART_Init_Config);

    USART_Init_Config_s vision_UART_Init_Config = {
        .module_callback = NULL,
        .usart_handle = &huart6,
    };
    vision_uart_rec = USARTRegister(&vision_UART_Init_Config);

    memset(&qua_origin,0,sizeof(qua_origin));

    memset(&origin_q,0,sizeof(Quaternion));
    origin_q.w=1;

}




// 四元数乘法
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

// 四元数逆运算
Quaternion quaternion_inverse(Quaternion q) {
    Quaternion result;
    float norm_sq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    result.w = q.w / norm_sq;
    result.x = -q.x / norm_sq;
    result.y = -q.y / norm_sq;
    result.z = -q.z / norm_sq;
    return result;
}

Quaternion f_2_q(float f[4]){
    Quaternion q;
    q.w = f[0];
    q.x = f[1];
    q.y = f[2];
    q.z = f[3];
    return q;
}
/* 测试任务 */
void _TestTask()
{
    static uint8_t flag = 0;
    static uint8_t d_flag = 0;
    static uint32_t key_cnt = 0;
    if (HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin) == GPIO_PIN_RESET){
        key_cnt++;
        if(key_cnt > 3000)  origin_q =  f_2_q(qua_rec_raw_f);
        // while(HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin) == GPIO_PIN_RESET);
        // osDelay(1);
        flag = 1;
    }else {key_cnt = 0;flag=0;}
    send_flag = flag;
    (HAL_GPIO_ReadPin(key_2_GPIO_Port, key_2_Pin) == GPIO_PIN_RESET) ? (send_flag|=0x02) : (send_flag&=~0x02);
    (HAL_GPIO_ReadPin(key_3_GPIO_Port, key_3_Pin) == GPIO_PIN_RESET) ? (send_flag|=0x04) : (send_flag&=~0x04);

    //编码器值
    encoder_Data[0] = encoder[0]->measure.angle_single_round;
    encoder_Data[1] = encoder[1]->measure.angle_single_round;
    encoder_Data[2] = encoder[2]->measure.angle_single_round;

    //四元数
        static float roll__,yaw__,pitch__;
        static Quaternion c,origin_q_n;
        rec_q = f_2_q(qua_rec_raw_f);
        origin_q_n = quaternion_inverse(origin_q);
        c = quaternion_multiply(origin_q_n, rec_q);
        
        qua_rec[0] = c.w;
        qua_rec[1] = c.x;
        qua_rec[2] = c.y;
        qua_rec[3] = c.z;

        get_angle(qua_rec,&yaw__,&pitch__,&roll__);
        yaw__*= RAD_TO_ANGLE;
        pitch__*= RAD_TO_ANGLE;
        roll__*= RAD_TO_ANGLE;

    //延时-贴近图传链路的极限30hz发送频率
    static uint8_t count = 0;
    if(count++ > 34){
        static uint8_t frame_head = 0xA5;
        static uint16_t command = 0x302;
        static uint16_t data_len = 30;
        static uint8_t seq = 0;
        static uint8_t crc_8 = 0;
        static uint16_t check_sum;

        static uint8_t head = 0xff;
        static uint8_t send_buf[39];

        seq++;
        memcpy((uint8_t*)send_buf,(uint8_t*)&frame_head,1);
        memcpy((uint8_t*)send_buf+1,(uint8_t*)&data_len,2);
        memcpy((uint8_t*)send_buf+3,(uint8_t*)&seq,1);
        crc_8 = Get_CRC8_Check_Sum(send_buf,4,0xFF);
        memcpy((uint8_t*)send_buf+4,(uint8_t*)&crc_8,1);
        memcpy((uint8_t*)send_buf+5,(uint8_t*)&command,2);

        memcpy((uint8_t*)send_buf+7,(uint8_t*)&head,1);
        memcpy((uint8_t*)send_buf+8,(uint8_t*)&send_flag,1);
        memcpy((uint8_t*)send_buf+9,(uint8_t*)encoder_Data,12);
        memcpy((uint8_t*)send_buf+21,(uint8_t*)qua_rec,16);

        check_sum = Get_CRC16_Check_Sum(send_buf,37,0xFFFF);
        memcpy((uint8_t*)send_buf+37,(uint8_t*)&check_sum,2);
        count = 0;
        USARTSend(vision_uart_rec,(uint8_t*)send_buf,39,USART_TRANSFER_DMA);
    }

    // dji_motor[0]->motor_controller.angle_PID.Kp = angle_kp;
    // dji_motor[1]->motor_controller.angle_PID.Kp = angle_kp;
    // dji_motor[2]->motor_controller.angle_PID.Kp = angle_kp;

    // dji_motor[0]->motor_controller.speed_PID.Kp = speed_kp;
    // dji_motor[1]->motor_controller.speed_PID.Kp = speed_kp;
    // dji_motor[2]->motor_controller.speed_PID.Kp = speed_kp;

    // if(flag){
    //     C_board_LEDSet(0xd633ff);
    //     DJIMotorEnable(dji_motor[1]);
    //     DJIMotorEnable(dji_motor[0]);
    //     DJIMotorEnable(dji_motor[2]);
    //     DJIMotorSetRef(dji_motor[1],encoder[1]->measure.total_angle);
    //     DJIMotorSetRef(dji_motor[2],encoder[2]->measure.total_angle);
    //     DJIMotorSetRef(dji_motor[0],encoder[0]->measure.total_angle);

    //     d_flag = 1;

    // } else {
    //     C_board_LEDSet(0x33ffff);
    //     if(d_flag){
    //         DJIMotorStop(dji_motor[1]);
    //         DJIMotorStop(dji_motor[0]);
    //         DJIMotorStop(dji_motor[2]);
    //         // DJIMotorSetRef(dji_motor[1],0);
    //         // DJIMotorSetRef(dji_motor[2],0);
    //         // DJIMotorSetRef(dji_motor[0],0);

    //         d_flag = 0;
    //     }
    // }
}

