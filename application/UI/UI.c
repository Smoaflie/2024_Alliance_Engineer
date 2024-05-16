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

static char char_pump_one_mode[50];
static char char_pump_two_mode[50];
static char char_auto_mode[50];
static char char_arm_mode[50];
static char char_rotate_mode[50];

// 辅助线图形变量
static Graph_Data_t auxiliary_line_one;
static Graph_Data_t auxiliary_line_two;
static Graph_Data_t circle_one;
static Graph_Data_t circle_two;
static Graph_Data_t circle_three;
static Graph_Data_t circle_four;
static Graph_Data_t circle_five;

static String_Data_t pump_one_mode;    //气泵1
static String_Data_t pump_two_mode;    //气泵2
static String_Data_t auto_mode;        //自动模式
static String_Data_t arm_mode;         //臂姿态
static String_Data_t rotate_mode;      //陀螺模式

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
    memset(char_pump_one_mode, '\0', sizeof(pump_one_mode));
    memset(char_pump_two_mode, '\0', sizeof(pump_two_mode));
    memset(char_auto_mode,     '\0', sizeof(auto_mode));
    memset(char_arm_mode,      '\0', sizeof(arm_mode));
    memset(char_rotate_mode,   '\0', sizeof(rotate_mode));
   
    //标定线（数据待改）
    UILineDraw(&auxiliary_line_one, "111", Graphic_Operate_ADD, 5, Graphic_Color_Yellow, 2 ,524,461,759,0);
    UILineDraw(&auxiliary_line_two, "112", Graphic_Operate_ADD, 5, Graphic_Color_Yellow, 2 ,1335,461,1142,0);
    
    //圆
    UICircleDraw(&circle_one,  "113",Graphic_Operate_ADD,5,Graphic_Color_White,15,150,800,20);
    UICircleDraw(&circle_two,  "114",Graphic_Operate_ADD,5,Graphic_Color_White,15,150,720,20);
    UICircleDraw(&circle_three,"115",Graphic_Operate_ADD,5,Graphic_Color_White,15,150,640,20);
    UICircleDraw(&circle_four, "116",Graphic_Operate_ADD,5,Graphic_Color_White,15,150,560,20);
    UICircleDraw(&circle_five, "117",Graphic_Operate_ADD,5,Graphic_Color_White,15,150,480,20);

    UIGraphRefresh(&referee_data->referee_id, 5, auxiliary_line_one, auxiliary_line_two,circle_one,circle_two,circle_three,circle_four,circle_five);
    
    //字符串
    sprintf(pump_one_mode.show_Data,"OFF");
    UICharDraw(&pump_one_mode,"001",Graphic_Operate_ADD,5,Graphic_Color_Green,20,10,200,830,char_pump_one_mode);
    UICharRefresh(&referee_data->referee_id, pump_one_mode);

    sprintf(pump_two_mode.show_Data,"OFF");
    UICharDraw(&pump_two_mode,"002",Graphic_Operate_ADD,5,Graphic_Color_Green,20,10,200,750,char_pump_two_mode);
    UICharRefresh(&referee_data->referee_id, pump_two_mode);
    
    sprintf(auto_mode.show_Data," ");
    UICharDraw(&auto_mode,"003",Graphic_Operate_ADD,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
    UICharRefresh(&referee_data->referee_id, auto_mode);
    
    sprintf(arm_mode.show_Data,"IN");
    UICharDraw(&arm_mode,"004",Graphic_Operate_ADD,5,Graphic_Color_Green,20,10,200,590,char_arm_mode);
    UICharRefresh(&referee_data->referee_id, arm_mode);

    sprintf(rotate_mode.show_Data,"OFF");
    UICharDraw(&rotate_mode,"005",Graphic_Operate_ADD,5,Graphic_Color_Green,20,10,200,510,char_rotate_mode);   
    UICharRefresh(&referee_data->referee_id, rotate_mode);
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

    //气泵1
    if(UI_data_recv.pump_one_mode_t==1)
    {
        UICircleDraw(&circle_one,  "113",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,150,800,20);
        sprintf(pump_one_mode.show_Data,"ON");
        UICharDraw(&pump_one_mode,"001",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,830,char_pump_one_mode);
        UICharRefresh(&referee_data->referee_id, pump_one_mode);
    }
    else if(UI_data_recv.pump_one_mode_t==0)
    {
        UICircleDraw(&circle_one,  "113",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,150,800,20);
        sprintf(pump_one_mode.show_Data,"OFF");
        UICharDraw(&pump_one_mode,"001",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,830,char_pump_one_mode);
        UICharRefresh(&referee_data->referee_id, pump_one_mode);
    }

    //气泵2
    if(UI_data_recv.pump_two_mode_t==1)
    {
        UICircleDraw(&circle_two,  "114",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,150,720,20);
        sprintf(pump_two_mode.show_Data,"ON");
        UICharDraw(&pump_two_mode,"002",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,750,char_pump_two_mode);
        UICharRefresh(&referee_data->referee_id, pump_two_mode);
    }
    else if(UI_data_recv.pump_two_mode_t==0)
    {
        UICircleDraw(&circle_two,  "114",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,150,720,20);
        sprintf(pump_two_mode.show_Data,"OFF");
        UICharDraw(&pump_two_mode,"002",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,750,char_pump_two_mode);
        UICharRefresh(&referee_data->referee_id, pump_two_mode);
    }

    //自动模式
    if(UI_data_recv.auto_mode_t==1)//金矿中
    {
        UICircleDraw(&circle_three,"115",Graphic_Operate_CHANGE,5,Graphic_Color_Yellow,15,150,640,20);
        sprintf(auto_mode.show_Data,"GM");
        UICharDraw(&auto_mode,"003",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
        UICharRefresh(&referee_data->referee_id, auto_mode);
    }
    else if(UI_data_recv.auto_mode_t==3)//金矿左
    {
        UICircleDraw(&circle_three,"115",Graphic_Operate_CHANGE,5,Graphic_Color_Purplish_red,15,150,640,20);
        sprintf(auto_mode.show_Data,"GL");
        UICharDraw(&auto_mode,"003",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
        UICharRefresh(&referee_data->referee_id, auto_mode);
    }
    else if(UI_data_recv.auto_mode_t==2)//银矿左
    {
        UICircleDraw(&circle_three,"115",Graphic_Operate_CHANGE,5,Graphic_Color_Orange,15,150,640,20);
        sprintf(auto_mode.show_Data,"SL");
        UICharDraw(&auto_mode,"003",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
        UICharRefresh(&referee_data->referee_id, auto_mode);
    }
    else if(UI_data_recv.auto_mode_t==2)//银矿中
    {
        UICircleDraw(&circle_three,"115",Graphic_Operate_CHANGE,5,Graphic_Color_Pink,15,150,640,20);
        sprintf(auto_mode.show_Data,"SM");
        UICharDraw(&auto_mode,"003",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
        UICharRefresh(&referee_data->referee_id, auto_mode);
    }
    else if(UI_data_recv.auto_mode_t==2)//银矿右
    {
        UICircleDraw(&circle_three,"115",Graphic_Operate_CHANGE,5,Graphic_Color_Cyan,15,150,640,20);
        sprintf(auto_mode.show_Data,"SR");
        UICharDraw(&auto_mode,"003",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
        UICharRefresh(&referee_data->referee_id, auto_mode);
    }
    else if(UI_data_recv.auto_mode_t==2)//矿仓1
    {
        UICircleDraw(&circle_three,"115",Graphic_Operate_CHANGE,5,Graphic_Color_Main,15,150,640,20);
        sprintf(auto_mode.show_Data,"W1");
        UICharDraw(&auto_mode,"003",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
        UICharRefresh(&referee_data->referee_id, auto_mode);
    }

    else if(UI_data_recv.auto_mode_t==2)//矿仓2
    {
        UICircleDraw(&circle_three,"115",Graphic_Operate_CHANGE,5,Graphic_Color_Black,15,150,640,20);
        sprintf(auto_mode.show_Data,"W2");
        UICharDraw(&auto_mode,"003",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
        UICharRefresh(&referee_data->referee_id, auto_mode);
    }
    else
    {
        UICircleDraw(&circle_three,"115",Graphic_Operate_CHANGE,5,Graphic_Color_White,15,150,640,20);
        sprintf(auto_mode.show_Data," ");
        UICharDraw(&auto_mode,"003",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,670,char_auto_mode);
        UICharRefresh(&referee_data->referee_id, auto_mode);
    }

    //臂姿态
    if(UI_data_recv.arm_mode_t==1)
    {
        UICircleDraw(&circle_four, "116",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,150,560,20);
        sprintf(arm_mode.show_Data,"OUT");
        UICharDraw(&arm_mode,"004",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,590,char_arm_mode);
        UICharRefresh(&referee_data->referee_id, arm_mode);
    }
    else if(UI_data_recv.arm_mode_t==1)
    {
        UICircleDraw(&circle_four, "116",Graphic_Operate_CHANGE,5,Graphic_Color_White,15,150,560,20);
        sprintf(arm_mode.show_Data,"IN");
        UICharDraw(&arm_mode,"004",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,590,char_arm_mode);
        UICharRefresh(&referee_data->referee_id, arm_mode);
    }

    //陀螺模式
    if(UI_data_recv.rotate_mode_t==1)
    {
        UICircleDraw(&circle_five, "117",Graphic_Operate_CHANGE,5,Graphic_Color_Green,15,150,480,20);
        sprintf(rotate_mode.show_Data,"ON");
        UICharDraw(&rotate_mode,"005",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,510,char_rotate_mode);   
        UICharRefresh(&referee_data->referee_id, rotate_mode);
    }
    else if(UI_data_recv.rotate_mode_t==0)
    {
        UICircleDraw(&circle_five, "117",Graphic_Operate_CHANGE,5,Graphic_Color_White,15,150,480,20);
        sprintf(rotate_mode.show_Data,"OFF");
        UICharDraw(&rotate_mode,"005",Graphic_Operate_CHANGE,5,Graphic_Color_Green,20,10,200,510,char_rotate_mode);   
        UICharRefresh(&referee_data->referee_id, rotate_mode);
    }
}