#ifndef __EXTERNCMD_H_
#define __EXTERNCMD_H_
#include "main.h"
#include "usart.h"
#pragma pack(1)
typedef struct _ext_robot_command_t {
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} ext_robot_command_t;

typedef struct _ext_robot_command_pack {
    ext_robot_command_t lastcmd;
    ext_robot_command_t nowcmd;
} ext_robot_command_pack;

#pragma pack()
/**
 * @brief 图传链路初始化
 *
 * @param rc_usart_handle
 * @return ext_robot_command_pack* 图传链路接收数据
 */

ext_robot_command_pack *ExternCmdRegister(UART_HandleTypeDef *ExternCmdUsartHandle);

/**
 * @brief 检查图传链路是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t ExternCmdIsOnline();

#endif