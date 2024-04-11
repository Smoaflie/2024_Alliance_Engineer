/**
 * @file master_process.c
 * @author smoaflie
 * @brief  module for recv&send host data
 * @date 2023-3-26
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "daemon.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "bsp_usb.h"

static uint8_t idx = 0; // register idx,是该文件的全局上位机索引,在注册时使用
static HostInstance *host_instance[Host_Instance_MX_CNT];

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 */
static void HostOfflineCallback(void *instance)
{
    HostInstance *_instance = (HostInstance *)instance;
    switch(_instance->comm_mode){
        case HOST_USART:
            USARTServiceInit(_instance->comm_instance);
            break;
        case HOST_VCP:
            LOGWARNING("[vision] vision offline, restart communication.");
            USBRefresh();
            break;
    }
}

HostInstance *HostInit(HostInstanceConf *host_conf)
{
    if(idx >= Host_Instance_MX_CNT){
        while(1)
            LOGERROR("[master_process]You cant register more instance for host.");
    }
    HostInstance *_instance = (HostInstance *)malloc(sizeof(HostInstance));
    memset(_instance,0,sizeof(HostInstance));
    _instance->comm_mode = host_conf->comm_mode;

    switch(host_conf->comm_mode){
        case HOST_USART:
           // USART_Init_Config_s usart_conf;
            // usart_conf.module_callback = host_conf->callback;
            // usart_conf.recv_buff_size = host_conf->RECV_SIZE;
            // usart_conf.usart_handle = host_conf->usart_handle;
            // _instance->comm_instance = USARTRegister(&usart_conf);
            break;
        case HOST_VCP:
          //  USB_Init_Config_s usb_conf = {.rx_cbk = host_conf->callback};
            //_instance->comm_instance = USBInit(usb_conf);
            break;
        default:
            while (1)
                LOGERROR("[master_process]You must select correct mode for HOST.");
    }

    // 为上位机实例注册守护进程
    Daemon_Init_Config_s daemon_conf = {
        .callback = HostOfflineCallback, // 离线时调用的回调函数,会重启实例
        .owner_id = _instance,
        .reload_count = 10,
    };
    _instance->daemon = DaemonRegister(&daemon_conf);

    host_instance[idx++] = _instance;

    return _instance;
}

/**
 * @brief 发送函数
 *
 * @param instance 上位机实例
 * @param sendbuf  发送内容
 * @param tx_len 发送长度
 *
 */
void HostSend(HostInstance *instance,uint8_t *send_buf,uint16_t tx_len)
{
    switch(instance->comm_mode){
        case HOST_USART:
            // todo：是否需要对发送方式进行定制（即轮询/中断/DMA三种模式）
            USARTSend((USARTInstance*)instance->comm_instance, send_buf, tx_len, USART_TRANSFER_DMA);
            break;
        case HOST_VCP:
            USBTransmit(send_buf, tx_len);
            break;
    }
}
