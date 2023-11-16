/*
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-18 23:26:21
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\modules\super_cap\super_cap.h
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2023-10-18 23:01:07
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"

#pragma pack(1)
/* 超级电容发送信息 */
typedef struct
{
    float CapVot;         // 电压
    uint8_t open_flag;    // 开关指示
} SuperCap_Msg_s;

/* 超级电容接收信息 */
typedef struct 
{
    uint8_t power_relay_flag;           //继电器开启状态
    uint8_t power_level;                //功率等级
    uint16_t chassic_power_remaining;   //剩余功率
}SuperCap_Msg_g;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins;     // CAN实例
    SuperCap_Msg_s cap_msg_s; // 超级电容发送信息
    SuperCap_Msg_g cap_msg_g; //超级电容接收信息
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 * @attention:data是超级电容接收信息的数组，只有四位，注意不要超出范围
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 * @brief 发送超级电容控制信息
 *
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 */
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);

#endif // !SUPER_CAP_Hd
