#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u

#pragma pack(1)
typedef struct
{
    float pitch;
    float yaw;
    uint16_t coordinate_X;
    uint16_t coordinate_Y;
} Vision_Recv_s;

typedef enum {
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED  = 2,
} Enemy_Color_e;

typedef enum {
    VISION_MODE_AIM        = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF   = 2
} Work_Mode_e;

typedef enum {
    BULLET_SPEED_NONE = 0,
    BIG_AMU_16        = 16,
    SMALL_AMU_30      = 30,
} Bullet_Speed_limit_e;

typedef struct
{
    Enemy_Color_e enemy_color;
    Work_Mode_e work_mode;
    Bullet_Speed_limit_e bullet_Speed_limit;
    float bullet_speed_current;
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
// void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif // !MASTER_PROCESS_H