// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "LKmotor.h"
#include "servo_motor.h"
#include "led.h"
// bsp
#include "encoder.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "DRmotor.h"
static USARTInstance* imu_usart_instance;
static float gimbal_yaw_angle;
static uint8_t gimbal_rec[100];
static void imu_usart_callback(){
    uint16_t crc,payload_len;
    memcpy(gimbal_rec,imu_usart_instance->recv_buff,imu_usart_instance->recv_buff_size);
    if(gimbal_rec[0]==0x5A && gimbal_rec[1]==0xA5)
    {
        payload_len = gimbal_rec[2] + (gimbal_rec[3]<<8);
        // crc = gimbal_rec[4] + (gimbal_rec[5]<<8);
        // uint16_t crc_d  =crc_16(gimbal_rec+6,payload_len);
        // if(crc == crc_d)
        if(payload_len == 7)
            gimbal_yaw_angle = (int16_t)(gimbal_rec[11]+(gimbal_rec[12]<<8))*0.01f;
    }
}
void selfTestInit()
{
    USART_Init_Config_s uart_conf;
    uart_conf.module_callback = imu_usart_callback;
    uart_conf.usart_handle = &huart9;
    uart_conf.recv_buff_size = 13;
    imu_usart_instance = USARTRegister(&uart_conf);
}

float LKangle = 0,gimbalangle=0;
/* 机器人机械臂控制核心任务 */
void selfTestTask()
{

}