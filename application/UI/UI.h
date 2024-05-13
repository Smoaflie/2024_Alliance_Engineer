#ifndef UI_H
#define UI_H

#include "referee_UI.h"

typedef struct
{ 
    uint8_t pump_one_mode_t;    //气泵1
    uint8_t pump_two_mode_t;    //气泵2
    uint8_t auto_mode_t;        //自动模式
    uint8_t arm_mode_t;         //臂姿态  
    uint8_t rotate_mode_t;      //陀螺模式
}UI_data_t;

extern void get_referee_data(referee_info_t *referee_data);
extern void MyUIRefresh(void);
extern void MyUIInit(void);

#endif