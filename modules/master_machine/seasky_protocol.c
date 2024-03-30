/**
 * @file seasky_protocol.c
 * @author Liu Wei
 * @author modified by Neozng
 * @brief 湖南大学RoBoMatster串口通信协议
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "seasky_protocol.h"
#include "crc8.h"
#include "memory.h"

/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        if (CRC8_Check_Sum(&rx_buf[0], 4))
        {
            pro->header.data_length = rx_buf[1];
            pro->header.crc_check = rx_buf[2];
            pro->cmd_id = (rx_buf[4] << 8) | rx_buf[3];
            return 1;
        }
    }
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_data(Vision_Send_s *tx_buf_data,     // 待发送的数据
                            uint8_t       *tx_buf,          // 待发送的数据帧
                            uint8_t       *tx_buf_len)      // 待发送的数据帧长度
{
    static uint8_t crc_8_data;

    /*帧头部分*/
    tx_buf[0] = PROTOCOL_CMD_ID;           //帧头ID
    tx_buf[1] = sizeof(tx_buf_data) + 4;   //数据长度
    tx_buf[2] = crc_8(&tx_buf[0], 2);      // 获取CRC8校验位

    /*数据部分*/
    memcpy(&tx_buf[3],tx_buf_data,sizeof(Vision_Send_s));

    /*整包校验*/ 
    crc_8_data = crc_8(&tx_buf[0], sizeof(tx_buf_data) + 3);
    tx_buf[sizeof(tx_buf_data) + 3] = crc_8_data;

    *tx_buf_len = sizeof(tx_buf_data) + 4;
}
/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
uint16_t get_protocol_info(uint8_t *rx_buf,          // 接收到的原始数据
                           uint8_t *rx_data)         // 接收的数据存储地址
{
    // 放在静态区,避免反复申请栈上空间
    static protocol_rm_struct pro;

    if (protocol_heade_Check(&pro, rx_buf))
    {
        if (CRC8_Check_Sum(&rx_buf[0], pro.header.data_length))
        {
            memcpy(&rx_buf[5], rx_data, pro.header.data_length - 5);
            return pro.cmd_id;
        }
    }
    return 0;
}
