

#include "referee_init.h"
#include "referee_UI.h"
#include "referee_protocol.h"
#include "message_center.h"
#include "rm_referee.h"
#include "UI_ref.h"
#include "ui.h"
#include <string.h>

referee_info_t *referee_data;
UI_data_t UI_data_recv;
Subscriber_t *UI_cmd_sub;

uint8_t UI_Seq = 0;

static char char_arm[50];
static char char_arm_mode[50];
static char char_valve[50];
static char char_valve_mode[50];

// 辅助线图形变量
static Graph_Data_t auxiliary_line_one;
static Graph_Data_t auxiliary_line_two;
static Graph_Data_t circle_one;
static Graph_Data_t circle_two;
static Graph_Data_t circle_three;
static Graph_Data_t float_one;
static Graph_Data_t float_two;

static String_Data_t arm;         //臂状态
static String_Data_t arm_mode;
static String_Data_t valve;       //气推杆状态
static String_Data_t valve_mode;         

void get_referee_data(referee_info_t *referee_data)
{
    referee_data                               = referee_data;
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->referee_id.Robot_ID; // 计算客户端ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}

static void ui_refresh(){
 //清除所有UI
    UIDelete(&referee_data->referee_id, UI_Data_Del_ALL, 0);

    //字符串赋值
    memset(char_arm,        '\0', sizeof(arm));
    memset(char_arm_mode,   '\0', sizeof(arm_mode));
    memset(char_valve,      '\0', sizeof(valve));
    memset(char_valve_mode, '\0', sizeof(valve_mode));

   
    //标定线（数据待改）
    UILineDraw(&auxiliary_line_one, "111", Graphic_Operate_ADD, 5, Graphic_Color_Yellow, 2 ,816,560,635,0);
    UILineDraw(&auxiliary_line_two, "112", Graphic_Operate_ADD, 5, Graphic_Color_Yellow, 2 ,970,560,1186,0);
    
    //圆
    UICircleDraw(&circle_one,  "113",Graphic_Operate_ADD,5,Graphic_Color_White,15,150,800,20);
    UICircleDraw(&circle_two,  "114",Graphic_Operate_ADD,5,Graphic_Color_White,15,150,720,20);

    //浮点数（待补充气压值）
    // UIFloatDraw(&float_one,"116",Graphic_Operate_ADD,5,Graphic_Color_Green,20,3,3,210,820,pump1);
    // UIFloatDraw(&float_two,"117",Graphic_Operate_ADD,5,Graphic_Color_Green,20,3,3,210,735,pump2);

    UIGraphRefresh(&referee_data->referee_id, 5, auxiliary_line_one, auxiliary_line_two,circle_one,circle_two,float_one);
    UIGraphRefresh(&referee_data->referee_id, 1,  float_two);
    
    //字符串
    // sprintf(arm.show_Data," ARM_MODE:");
    // UICharDraw(&arm,"001",Graphic_Operate_ADD,5,Graphic_Color_Green,25,5,1450,820," ARM_MODE:");
    // UICharRefresh(&referee_data->referee_id, arm);

    // sprintf(arm_mode.show_Data," NULL");
    // UICharDraw(&arm_mode,"002",Graphic_Operate_ADD,5,Graphic_Color_Green,25,5,1680,820,char_arm_mode);
    // UICharRefresh(&referee_data->referee_id, arm_mode);

    // sprintf(valve.show_Data," VALVE:");
    // UICharDraw(&valve,"003",Graphic_Operate_ADD,5,Graphic_Color_Green,25,5,1450,735,char_valve);
    // UICharRefresh(&referee_data->referee_id, valve);

    // sprintf(valve_mode.show_Data," NULL");
    // UICharDraw(&valve_mode,"004",Graphic_Operate_ADD,5,Graphic_Color_Green,25,5,1680,735,char_valve_mode);
    // UICharRefresh(&referee_data->referee_id, valve_mode);
    UICharDraw(&arm,"001",Graphic_Operate_ADD,5,Graphic_Color_Green,15,5,1450,820," Control_MODE: %s","false");
    UICharRefresh(&referee_data->referee_id, arm);

}

void MyUIInit(void)
{
    referee_data = RefereeHardwareInit(&huart7);
    
    osDelay(200);

    get_referee_data(referee_data);

    UI_cmd_sub = SubRegister("UI",sizeof(UI_data_t));

    ui_refresh();
}


void MyUIRefresh(void)
{
    SubGetMessage(UI_cmd_sub, &UI_data_recv);

    //如UI刷新请求置位，则刷新UI
    if(UI_data_recv.UI_refresh_request == 1){
        ui_refresh();
        return;
    }

    if(UI_data_recv.control_mode_t==1)
    {
         UICharDraw(&arm,"001",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,5,1450,820," Control_MODE: %s","true");
    }
    else
    {
         UICharDraw(&arm,"001",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,5,1450,820," Control_MODE: %s","false");
    }

    UICharRefresh(&referee_data->referee_id, arm);

    //气泵状态
    if(UI_data_recv.pump_one_mode_t == 1)
        UICircleDraw(&circle_one,  "113",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,150,800,20);
    else
        UICircleDraw(&circle_one,  "113",Graphic_Operate_CHANGE,5,Graphic_Color_White,15,150,800,20);

    if(UI_data_recv.pump_two_mode_t == 1)
        UICircleDraw(&circle_two,  "114",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,150,720,20);
    else    
        UICircleDraw(&circle_two,  "114",Graphic_Operate_CHANGE,5,Graphic_Color_White,15,150,720,20);
    UIGraphRefresh(&referee_data->referee_id, 2, circle_one, circle_two);



}
        