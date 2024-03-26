/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

static uint8_t reg_flag = 0;       // 注册标识符，保证板子只注册一个USB实例，多了就报错
static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
// 注意usb单个数据包(Full speed模式下)最大为64byte,超出可能会出现丢包情况

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    if (reg_flag == 1) {
        while (1)
            LOGERROR("[bsp_usb]You cant register more instance for usb.");
    }
    // usb的软件复位(模拟拔插)在usbd_conf.c中的HAL_PCD_MspInit()中
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // 获取接收数据指针
    // usb的接收回调函数会在这里被设置,并将数据保存在bsp_usb_rx_buffer中
    LOGINFO("USB init success");
    reg_flag = 1;
    return bsp_usb_rx_buffer;
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_FS(buffer, len); // 发送
}

void USBRefresh()
{
    // 重新枚举usb设备
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    DWT_Delay(0.1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    DWT_Delay(0.1);
}