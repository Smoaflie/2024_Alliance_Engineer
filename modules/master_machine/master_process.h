#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "bsp_usb.h"
#include "daemon.h"
#include "vision.h" // 可以在不修改代码的情况下更新model层

#define Host_Instance_MX_CNT 1

// 模块回调函数,用于解析协议
typedef void (*host_module_callback)();

typedef enum {
    HOST_USART = 0, // 串口通信
    HOST_VCP,       // 虚拟串口/USB通信
} host_comm_mode;

typedef struct {
    void *comm_instance;      // 通信实例，会被转换为串口或USB通信实例
    host_comm_mode comm_mode; // 通信方式
    DaemonInstance *daemon;   // 守护进程
    uint16_t recv_buff_size;         // 接收长度
    uint8_t  *recv_buff;        // 接收缓冲区
    host_module_callback callback;   // 解析收到的数据的回调函数
} HostInstance;

typedef struct {
    host_comm_mode comm_mode;         // 通信方式
    UART_HandleTypeDef *usart_handle; // 串口模式下，串口通信对应的handle
    host_module_callback callback;   // 解析收到的数据的回调函数
    uint8_t RECV_SIZE;                // 接收数据大小
    uint16_t rate;                    // 回报率，用以指示上位机发送数据的频率，用以配置看门狗（0则不设看门狗）
} HostInstanceConf;

/**
 * @brief 调用此函数初始化上位机
 *
 */
HostInstance *HostInit(HostInstanceConf *host_conf);

/**
 * @brief 发送数据
 *
 */
void HostSend(HostInstance *instance, uint8_t *send_buf, uint16_t tx_len);

#endif // !MASTER_PROCESS_H