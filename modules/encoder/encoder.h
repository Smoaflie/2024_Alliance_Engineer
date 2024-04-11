/***
 * @Author       : Smoaflie
 * @Date         : 2024-01-21
 * @FilePath     : \2024_Control_New_Framework_Base-dev-all\modules\encoder\encoder.h
 * @Description  : 第三方编码器注册及使用(参考`dji_motor.c`)
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include "bsp_can.h"
#include "stdint.h"
#include "daemon.h"

#define ENCODER_CNT 5

#define MT6825_ECD_TO_DEG 0.0013733f // (360/262,144),将编码器值转化为角度制
#define me02_ECD_TO_DEG 0.01098633f // (360/32768)
/* 编码器类型枚举 */
typedef enum
{
    MT6825 = 0,
    me02_can,
} Encoder_Type_e;

typedef struct
{
    uint32_t last_ecd;        // 上一次读取的编码器值
    uint32_t ecd;             // 0-65535,刻度总共有65535格
    float angle_single_round; // 单圈角度
    float speed_aps;    // 旋转速度,度每秒

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} Encoder_Measure_s;

/* 用于初始化的结构体 */
typedef struct
{
    CAN_Init_Config_s can_init_config;

    Encoder_Type_e encoder_type; // 编码器类型
    uint32_t offset;    //编码器偏移量|可选
} Encoder_Init_Config_s;

typedef struct
{
    Encoder_Measure_s measure;            // 编码器测量值

    uint32_t offset;    //编码器偏移量|可选

    Encoder_Type_e encoder_type; // 编码器类型
    CANInstance *encoder_can_instance; // 编码器CAN实例

    DaemonInstance* daemon;

    uint32_t feed_cnt;
    float dt;
} EncoderInstance_s;


// 编码器初始化,返回一个编码器实例
EncoderInstance_s *EncoderInit(Encoder_Init_Config_s *config);

#endif
