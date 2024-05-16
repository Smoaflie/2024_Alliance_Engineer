#include "referee_protocol.h"
#include "bsp_usart.h"

// 自定义控制器接收数据(图传链路)
static USARTInstance *vision_usart;
static float qua_rec[4];
static float encoder_Data[3];
static uint8_t custom_controller_comm_recv = 0; //自定义发送的自定义标志位	
static uint8_t custom_controller_data_refresh_flag = 0; // 图传链路数据更新标志
static uint8_t custom_controller_data[30];
// 图传链路
static referee_info_t referee_vision_info;			  // 图传链路数据

static void JudgeVisionReadData(uint8_t* buff){
    uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_vision_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// 帧尾CRC16校验
            // todo:始终无法校验成功，暂时关闭
			// if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			// {
				// 2个8位拼成16位int
				referee_vision_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (referee_vision_info.CmdID)
				{
                    case 0x0302:
                        memcpy(&custom_controller_data, (buff + DATA_Offset), 30);
                        memcpy((uint8_t*)&custom_controller_comm_recv,(uint8_t*)custom_controller_data+1,1);

                        memcpy((uint8_t*)encoder_Data,(uint8_t*)custom_controller_data+2,12);
							// encoder_Data[0] = -encoder_Data[0];
							// encoder_Data[1] = -encoder_Data[1];
							// encoder_Data[2] = -encoder_Data[2];
                        memcpy((uint8_t*)qua_rec,(uint8_t*)custom_controller_data+14,16);
                        custom_controller_data_refresh_flag = 1;
					    break;
                    default:
                        break;
                }
			// }
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_vision_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			JudgeVisionReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_vision_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

static void vision_recv_callback(){
    JudgeVisionReadData(vision_usart->recv_buff);
}