#include "referee_init.h"
#include "memory.h"

MyUIInit_Ptr MyuiInitLocal;
MyUIRefresh_Ptr MyUIRefreshLocal;
referee_info_t *referee_info_t_ptr;

referee_info_t *RefereeHardwareInit(UART_HandleTypeDef *referee_usart_handle)
{
    referee_info_t_ptr = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    referee_info_t_ptr->init_flag = 1;

    return referee_info_t_ptr;
}

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
int8_t UITaskInit(MyUIInit_Ptr MyuiInit, MyUIRefresh_Ptr MyUIRefresh)
{
    if (referee_info_t_ptr->init_flag != 1)
    {
        return -1; // 硬件未初始化
    }

    MyuiInitLocal = MyuiInit;
    MyUIRefreshLocal = MyUIRefresh;
    return 0;
}

static uint8_t UiInitFlag = 0;
/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask()
{
    if (referee_info_t_ptr->init_flag == 1)
    {
        if (!UiInitFlag)
        {
            (*MyuiInitLocal)();
            UiInitFlag = 1;
        }
        else
        {
            (*MyUIRefreshLocal)();
        }
    }
}