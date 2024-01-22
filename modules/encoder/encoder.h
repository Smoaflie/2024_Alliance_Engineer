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

#define ENCODER_CNT 2

#define ECD_ANGLE_TO_DEG 0.0013733f // (360/262,144),将编码器值转化为角度制

typedef struct
{
    uint32_t last_ecd;        // 上一次读取的编码器值
    uint32_t ecd;             // 0-65535,刻度总共有65535格
    float angle_single_round; // 单圈角度

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} Encoder_Measure_s;

/* 用于初始化的结构体 */
typedef struct
{
    CAN_Init_Config_s can_init_config;
} Encoder_Init_Config_s;

typedef struct
{
    Encoder_Measure_s measure;            // 电机测量值

    CANInstance *encoder_can_instance; // 电机CAN实例

    DaemonInstance* daemon;

    uint32_t feed_cnt;
    float dt;
} EncoderInstance_s;

// 编码器初始化,返回一个编码器实例
EncoderInstance_s *EncoderInit(Encoder_Init_Config_s *config);

#endif
