#include "externcmd.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "crc_ref.h"
#include "rm_referee.h"

#include <memory.h>

static uint8_t ExternCmdInitFlag = 0; // 初始化标志

ext_robot_command_pack ExternCmdData = {0};

// 图传链路拥有的串口实例
static USARTInstance *ExternCmdUsartInstance;
static DaemonInstance *ExternCmdDaemonInstance;

const uint8_t ExternCmdDataBuffer = 255u; // 图传链路接收缓冲区大小

const uint8_t ExternCmdDataSize = 12u; // 图传链路接收缓冲区大小

static referee_info_t referee_info; // 图传链路数据

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff)
{
    uint16_t judge_length; // 统计一帧数据长度
    if (buff == NULL)      // 空数据包，则不作任何处理
        return;

    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF] == REFEREE_SOF) {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE) {
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE) {
                // 2个8位拼成16位int
                referee_info.CmdID = (buff[6] << 8 | buff[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (referee_info.CmdID) {
                    case 0x0304: // 图传链路标识
                    {
                        ExternCmdData.lastcmd = ExternCmdData.nowcmd;
                        memcpy(&ExternCmdData.nowcmd, (buff + DATA_Offset), (size_t)ExternCmdDataSize);
                        break;
                    }
                }
            }
        }
        // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5) { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
            JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
        }
    }
}

/**
 * @brief 图传链路回调函数,用于注册到bsp_usart的回调函数中
 *
 */
static void ExternCmdRxCallback()
{
    DaemonReload(ExternCmdDaemonInstance);
    // 先喂狗
    JudgeReadData(ExternCmdUsartInstance->recv_buff); // 进行协议解析
}

static void ExternCmdLostCallback()
{
    memset(&ExternCmdData.lastcmd, 0, sizeof(ExternCmdData.lastcmd)); // 清空遥控器数据
    memset(&ExternCmdData.nowcmd, 0, sizeof(ExternCmdData.nowcmd));   // 清空遥控器数据
    USARTServiceInit(ExternCmdUsartInstance);                         // 尝试重新启动接收
    LOGWARNING("[ExternCmd] ExternCmd lost");
}

/**
 * @brief 图传链路初始化
 *
 * @param rc_usart_handle
 * @return ext_robot_command_pack* 图传链路接收数据
 */

ext_robot_command_pack *ExternCmdRegister(UART_HandleTypeDef *ExternCmdUsartHandle)
{
    USART_Init_Config_s conf;
    conf.module_callback   = ExternCmdRxCallback;
    conf.usart_handle      = ExternCmdUsartHandle;
    conf.recv_buff_size    = ExternCmdDataBuffer;
    ExternCmdUsartInstance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback     = ExternCmdLostCallback,
        .owner_id     = NULL, // 只有1个遥控器,不需要owner_id
    };
    ExternCmdDaemonInstance = DaemonRegister(&daemon_conf);

    ExternCmdInitFlag = 1;

    return &ExternCmdData;
}

/**
 * @brief 检查图传链路是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t ExternCmdIsOnline()
{
    if (ExternCmdInitFlag) {
        return DaemonIsOnline(ExternCmdDaemonInstance);
    }
    return 0;
}