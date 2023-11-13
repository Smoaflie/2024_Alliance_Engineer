#ifndef _REFEREE_INIT_H_
#define _REFEREE_INIT_H_

#include "rm_referee.h"
#include "robot_def.h"
#include "referee_UI.h"
typedef void (*MyUIInit_Ptr)(void);

typedef void (*MyUIRefresh_Ptr)(void);

referee_info_t *RefereeHardwareInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
int8_t UITaskInit(MyUIInit_Ptr MyuiInit, MyUIRefresh_Ptr MyUIRefresh);

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask();

#endif // !_REFEREE_INIT_H_