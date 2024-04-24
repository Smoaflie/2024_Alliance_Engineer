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
void imu_usart_callback(){
    memcpy(gimbal_rec,imu_usart_instance->recv_buff,82);
    memcpy((uint8_t*)&gimbal_yaw_angle,imu_usart_instance->recv_buff+62,4);
}
void selfTestInit()
{
    USART_Init_Config_s uart_conf;
    uart_conf.module_callback = imu_usart_callback;
    uart_conf.usart_handle = &huart8;
    uart_conf.recv_buff_size = 82;
    imu_usart_instance = USARTRegister(&uart_conf);
}

float LKangle = 0,gimbalangle=0;
/* 机器人机械臂控制核心任务 */
void selfTestTask()
{

}