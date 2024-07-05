

#include "referee_init.h"
#include "referee_UI.h"
#include "referee_protocol.h"
#include "message_center.h"
#include "rm_referee.h"
#include "UI_ref.h"
#include "UI.h"
#include <string.h>

referee_info_t *referee_data;

UI_data_t UI_data_recv;
Subscriber_t *UI_cmd_sub;

uint8_t UI_Seq = 0;

 static char S_ControMode_None[] = "NONE       ";
 static char S_ControMode_FetchCube[] = "FREE       ";
 static char S_ControlMode_ConvertCube[] = "Convert    ";
 static char S_ControlMode_Move[] = "Move       ";
 static char S_ControlMode_ReverseMove[] = "ReverseMove";
 static char S_ControlMode_RotateMove[] = "Rotate     ";

 static char S_ArmMode_NONE[] = "NONE       ";
 static char S_ArmMode_ArmWalk[] = "Walk       ";
 static char S_ArmMode_ArmIn[] = "IN         ";
 static char S_ArmMode_OUT[] = "OUT        ";
 static char S_ArmMode_goldcube_right[] = "goldright  ";
 static char S_ArmMode_warehouse1[] = "store_up   ";
 static char S_ArmMode_warehouse2[] = "store_down ";
 static char S_ArmMode_silvercube_left[] = "silve_left ";
 static char S_ArmMode_silvercube_mid[] = "silve_mid  ";
 static char S_ArmMode_silvercube_right[] = "silve_right";
 static char S_ArmMode_gronded_cube[] = "grond      ";
 static char S_ArmMode_ConvertCube[] = "Convert    ";

 static char S_ValveMode_None[] = "NONE     ";
 static char S_ValveMode_goldcube_mid[] = "gold_mid ";
 static char S_ValveMode_goldcube_left[] = "gold_left";

static Graph_Data_t* graph_group[4][20] = {NULL};    //存储图形UI指针
static uint8_t graph_idx[4] = {0};
static uint8_t graph_refresh_flag[4][32] = {0};

static String_Data_t* string_group[4][20] = {NULL};    //存储字符UI指针
static uint8_t string_idx[4] = {0};
static uint8_t string_refresh_flag[4][32] = {0};

// 图形变量
static Graph_Data_t circle_pumpArm;
static Graph_Data_t circle_pumpValve;
static Graph_Data_t circle_armAutoMode;
static Graph_Data_t circle_valveAutoMode;
static Graph_Data_t circle_customControConnection;
static Graph_Data_t circle_visitonConnection;
static Graph_Data_t circle_remoteConnection;
static Graph_Data_t rectangle_Z;
static Graph_Data_t line_z;
static Graph_Data_t float_z;
static Graph_Data_t Arc_armour_1;
static Graph_Data_t Arc_armour_2;
static Graph_Data_t Arc_armour_3;
static Graph_Data_t Arc_armour_4;
static Graph_Data_t float_current_bigyaw;
static Graph_Data_t float_current_midyaw;
static Graph_Data_t float_current_asyaw;
static Graph_Data_t float_current_asroll;
static Graph_Data_t float_current_tail;
static Graph_Data_t float_target_bigyaw;
static Graph_Data_t float_target_midyaw;
static Graph_Data_t float_target_asyaw;
static Graph_Data_t float_target_asroll;
static Graph_Data_t float_target_tail;

static String_Data_t string_controMode;
static String_Data_t string_armAutoMode;
static String_Data_t string_valveAutoMode;
static String_Data_t string_armPump;
static String_Data_t string_valvePump;
static String_Data_t string_controMode_;
static String_Data_t string_armAutoMode_;
static String_Data_t string_valveAutoMode_;
static String_Data_t string_BIGYAW;
static String_Data_t string_MIDYAW;
static String_Data_t string_ASYAW;
static String_Data_t string_ASROLL;
static String_Data_t string_TAIL;

struct{
    uint32_t pos_x;
    uint32_t pos_y;
    float height;
}height_makerLine;
struct{
    uint32_t pos_x;
    uint32_t pos_y;
    uint32_t dx;
    uint32_t dy;
    uint32_t width;
    float offset_angle;
}armour_maker;
void get_referee_data(referee_info_t *referee_data)
{
    referee_data                               = referee_data;
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->referee_id.Robot_ID; // 计算客户端ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}
static void UISetGraphPriority(Graph_Data_t *graph, uint8_t priority){
    if(priority > 3)    while(1)    LOGERROR("!priority must be 0-3.");
    graph_group[priority][graph_idx[priority]++] = graph;
}
static void UISetCharPriority(String_Data_t *string_data, uint8_t priority){
    if(priority > 3)    while(1)    LOGERROR("!priority must be 0-3.");
    string_group[priority][string_idx[priority]++] = string_data;
}
static void ui_init_refresh(){
    UICharRefresh(&referee_data->referee_id, string_controMode);
    UICharRefresh(&referee_data->referee_id, string_armAutoMode);
    UICharRefresh(&referee_data->referee_id, string_valveAutoMode);
    UICharRefresh(&referee_data->referee_id, string_armPump);
    UICharRefresh(&referee_data->referee_id, string_valvePump);
    UICharRefresh(&referee_data->referee_id, string_controMode_);
    UICharRefresh(&referee_data->referee_id, string_armAutoMode_);
    UICharRefresh(&referee_data->referee_id, string_valveAutoMode_);
    UICharRefresh(&referee_data->referee_id, string_BIGYAW);
    UICharRefresh(&referee_data->referee_id, string_MIDYAW);
    UICharRefresh(&referee_data->referee_id, string_ASYAW);
    UICharRefresh(&referee_data->referee_id, string_ASROLL);
    UICharRefresh(&referee_data->referee_id, string_TAIL);
    UIGraphRefresh(&referee_data->referee_id, 5, float_current_bigyaw,float_current_midyaw,float_current_asyaw,float_current_asroll,float_current_tail);
    UIGraphRefresh(&referee_data->referee_id, 5, float_target_bigyaw,float_target_midyaw,float_target_asyaw,float_target_asroll,float_target_tail);
    UIGraphRefresh(&referee_data->referee_id, 7, circle_pumpArm,circle_pumpValve,circle_armAutoMode,circle_valveAutoMode,circle_customControConnection,circle_visitonConnection,circle_remoteConnection);
    UIGraphRefresh(&referee_data->referee_id, 7, circle_pumpArm,circle_pumpValve,circle_armAutoMode,circle_valveAutoMode,circle_customControConnection,circle_visitonConnection,circle_remoteConnection);
    UIGraphRefresh(&referee_data->referee_id, 7, rectangle_Z,line_z,float_z,Arc_armour_1,Arc_armour_2,Arc_armour_3,Arc_armour_4);

    // Graph_Data_t* graph_data[8];
    // uint8_t selected_graph_cnt = 0;
    // int8_t searching_graph_priority = 3, searching_graph_idx[4] = {0};
    // uint8_t UI_Init_flag = 0;
    // while(!UI_Init_flag){
    //     while(selected_graph_cnt < 7){
    //         graph_data[selected_graph_cnt++] = graph_group[searching_graph_priority][searching_graph_idx[searching_graph_priority]++];
    //         selected_graph_cnt++;
    //         if(graph_group[searching_graph_priority][searching_graph_idx[searching_graph_priority]] == NULL){
    //             searching_graph_idx[searching_graph_priority] = 0;
    //             searching_graph_priority--;
    //             if(searching_graph_priority<0)  {UI_Init_flag = 1;searching_graph_priority = 3;break;}
    //         }
    //     }
    //     selected_graph_cnt = 0;
    //     UIGraphRefresh(&referee_data->referee_id, 7, *graph_data[0], *graph_data[1], *graph_data[2], *graph_data[3], *graph_data[4], *graph_data[5] ,*graph_data[6]);
    // }
}
static void ui_reload(){
    /*UI结构体初始化*/
    //圆
    UICircleDraw(&circle_pumpArm,  "10",Graphic_Operate_ADD,1,Graphic_Color_White,15,150,750,15);
    UICircleDraw(&circle_pumpValve,  "11",Graphic_Operate_ADD,1,Graphic_Color_White,15,150,700,15);
    UICircleDraw(&circle_armAutoMode,"20",Graphic_Operate_ADD,1,Graphic_Color_White,15,1250,750,5);
    UICircleDraw(&circle_valveAutoMode, "21",Graphic_Operate_ADD,1,Graphic_Color_White,15,1250,680,5);
    UICircleDraw(&circle_remoteConnection, "30",Graphic_Operate_ADD,1,Graphic_Color_White,15,150,850,15);
    UICircleDraw(&circle_visitonConnection, "31",Graphic_Operate_ADD,1,Graphic_Color_White,15,230,850,15);
    UICircleDraw(&circle_customControConnection, "32",Graphic_Operate_ADD,1,Graphic_Color_White,15,310,850,15);
    UISetGraphPriority(&circle_armAutoMode, 2);
    UISetGraphPriority(&circle_valveAutoMode, 2);
    UISetGraphPriority(&circle_pumpArm, 1);
    UISetGraphPriority(&circle_pumpValve, 1);
    UISetGraphPriority(&circle_remoteConnection, 3);
    UISetGraphPriority(&circle_visitonConnection, 3);
    UISetGraphPriority(&circle_customControConnection, 3);
    //静态字符串
    UICharDraw(&string_controMode,"a0",Graphic_Operate_ADD,2,Graphic_Color_Purplish_red,25,3,1290,840,"Control:");
    UICharDraw(&string_armAutoMode,"a1",Graphic_Operate_ADD,2,Graphic_Color_Purplish_red,25,3,1290,765,"ARM_AUTO:");
    UICharDraw(&string_valveAutoMode,"a2",Graphic_Operate_ADD,2,Graphic_Color_Purplish_red,25,3,1290,700,"VALVE_AUTO:");
    UICharDraw(&string_armPump,"a3",Graphic_Operate_ADD,2,Graphic_Color_Purplish_red,25,3,195,770,"ARM_PUMP");
    UICharDraw(&string_valvePump,"a4",Graphic_Operate_ADD,2,Graphic_Color_Purplish_red,25,3,195,715,"VALVE_PUMP");
    UISetCharPriority(&string_controMode, 0);
    UISetCharPriority(&string_armAutoMode, 0);
    UISetCharPriority(&string_valveAutoMode, 0);
    UISetCharPriority(&string_armPump, 0);
    UISetCharPriority(&string_valvePump, 0);
    UICharDraw(&string_BIGYAW,"a5",Graphic_Operate_ADD,2,Graphic_Color_Main,15,3,540,285,"BIGYAW");
    UICharDraw(&string_MIDYAW,"a6",Graphic_Operate_ADD,2,Graphic_Color_Main,15,3,540,255,"MIDYAW");
    UICharDraw(&string_ASYAW,"a7",Graphic_Operate_ADD,2,Graphic_Color_Main,15,3,540,225,"AS_YAW");
    UICharDraw(&string_ASROLL,"a8",Graphic_Operate_ADD,2,Graphic_Color_Main,15,3,540,195,"ASROLL");
    UICharDraw(&string_TAIL,"a9",Graphic_Operate_ADD,2,Graphic_Color_Main,15,3,540,165,"TAIL");
    UISetCharPriority(&string_BIGYAW, 0);
    UISetCharPriority(&string_MIDYAW, 0);
    UISetCharPriority(&string_ASYAW, 0);
    UISetCharPriority(&string_ASROLL, 0);
    UISetCharPriority(&string_TAIL, 0);
    //动态字符串
    UICharDraw(&string_controMode_,"d0",Graphic_Operate_ADD,3,Graphic_Color_Orange,25,3,1565,840,S_ControMode_None);
    UICharDraw(&string_armAutoMode_,"d1",Graphic_Operate_ADD,3,Graphic_Color_Orange,25,3,1565,765,S_ArmMode_NONE);
    UICharDraw(&string_valveAutoMode_,"d2",Graphic_Operate_ADD,3,Graphic_Color_Orange,25,3,1565,700,S_ValveMode_None);
    UISetCharPriority(&string_controMode_, 3);
    UISetCharPriority(&string_armAutoMode_, 3);
    UISetCharPriority(&string_valveAutoMode_, 3);
    //矩形
    UIRectangleDraw(&rectangle_Z, "01", Graphic_Operate_ADD, 1, Graphic_Color_Main, 2, height_makerLine.pos_x+130, height_makerLine.pos_y-310, height_makerLine.pos_x+180, height_makerLine.pos_y);
    UISetGraphPriority(&rectangle_Z, 0);
    //高度指示线(直线+浮点数)
    UILineDraw(&line_z, "22", Graphic_Operate_ADD, 1, Graphic_Color_Purplish_red, 2 , height_makerLine.pos_x, height_makerLine.pos_y, height_makerLine.pos_x+130, height_makerLine.pos_y);
    UIFloatDraw(&float_z, "23", Graphic_Operate_ADD, 1, Graphic_Color_Purplish_red, 20, 1, 3, height_makerLine.pos_x, height_makerLine.pos_y+35, 0);
    UISetGraphPriority(&line_z, 2);
    UISetGraphPriority(&float_z, 2);
    //臂状态
    UIFloatDraw(&float_current_bigyaw, "21", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 650, 285, 0);
    UIFloatDraw(&float_current_midyaw, "22", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 650, 255, 0);
    UIFloatDraw(&float_current_asyaw, "23", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 650, 225, 0);
    UIFloatDraw(&float_current_asroll, "24", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 650, 195, 0);
    UIFloatDraw(&float_current_tail, "25", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 650, 165, 0);
    UIFloatDraw(&float_target_bigyaw, "26", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 780, 285, 0);
    UIFloatDraw(&float_target_midyaw, "27", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 780, 255, 0);
    UIFloatDraw(&float_target_asyaw, "28", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 780, 225, 0);
    UIFloatDraw(&float_target_asroll, "29", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 780, 195, 0);
    UIFloatDraw(&float_target_tail, "20", Graphic_Operate_ADD, 5, Graphic_Color_Purplish_red, 15, 1, 3, 780, 165, 0);
    UISetGraphPriority(&float_current_bigyaw, 2);
    UISetGraphPriority(&float_current_midyaw, 2);
    UISetGraphPriority(&float_current_asyaw, 2);
    UISetGraphPriority(&float_current_asroll, 2);
    UISetGraphPriority(&float_current_tail, 2);
    UISetGraphPriority(&float_target_bigyaw, 2);
    UISetGraphPriority(&float_target_midyaw, 2);
    UISetGraphPriority(&float_target_asyaw, 2);
    UISetGraphPriority(&float_target_asroll, 2);
    UISetGraphPriority(&float_target_tail, 2);
    //臂动作
    //todo:
    //装甲板
    UIArcDraw(&Arc_armour_1, "24", Graphic_Operate_ADD, 1, Graphic_Color_Main, 155, 205, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, armour_maker.dx, armour_maker.dy);
    UIArcDraw(&Arc_armour_2, "25", Graphic_Operate_ADD, 1, Graphic_Color_Main, 245, 295, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, armour_maker.dx, armour_maker.dy);
    UIArcDraw(&Arc_armour_3, "26", Graphic_Operate_ADD, 1, Graphic_Color_Cyan, 335, 385, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, armour_maker.dx, armour_maker.dy);
    UIArcDraw(&Arc_armour_4, "27", Graphic_Operate_ADD, 1, Graphic_Color_Main, 65,  115, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, armour_maker.dx, armour_maker.dy);
    UISetGraphPriority(&Arc_armour_1, 2);
    UISetGraphPriority(&Arc_armour_2, 2);
    UISetGraphPriority(&Arc_armour_3, 2);
    UISetGraphPriority(&Arc_armour_4, 2);
    //清除所有UI
    UIDelete(&referee_data->referee_id, UI_Data_Del_ALL, 0);
    ui_init_refresh();
}

void MyUIInit(void)
{
    /*数据初始化*/
    height_makerLine.pos_x = 1230;
    height_makerLine.pos_y = 430;
    armour_maker.pos_x = 960;armour_maker.pos_y = 540;
    armour_maker.dx = 100;armour_maker.dy = 100;armour_maker.width = 10;

    /*裁判系统初始化*/
    referee_data = RefereeHardwareInit(&huart7);
    
    osDelay(200);

    get_referee_data(referee_data);

    UI_cmd_sub = SubRegister("UI",sizeof(UI_data_t));

    ui_reload();
}
static void UI_StateSwitchDetect_Graph(Graph_Data_t* graph, uint8_t cnt, int flag, ...){
    static uint8_t idx[32];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, int);
    va_end(ap);

    if(graph->color != idx[flag]){
        UICircleDraw(graph, (char*)graph->graphic_name,Graphic_Operate_CHANGE,graph->layer,idx[flag],graph->width,graph->start_x,graph->start_y,graph->radius);
        graph_refresh_flag[(uint8_t)graph->graphic_name[2]-'0'][((uint8_t)graph->graphic_name[1]-'0')] = 1;
    }
}
static void UI_ColorSwitchDetect_Char(String_Data_t *graph, uint8_t cnt, int flag, ...){
    static int idx[8];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, int);
    va_end(ap);
    
    if(graph->Graph_Control.color != idx[flag]){
        UICharDraw(graph,(char*)graph->Graph_Control.graphic_name,Graphic_Operate_CHANGE,graph->Graph_Control.layer,idx[flag],graph->Graph_Control.start_angle,graph->Graph_Control.width,graph->Graph_Control.start_x,graph->Graph_Control.start_y,graph->show_Data);
        string_refresh_flag[(uint8_t)(graph->Graph_Control.graphic_name[2]-'a')][(uint8_t)(graph->Graph_Control.graphic_name[1]-'0')] = 1;
    }
}
static void UI_WidthSwitchDetect_Char(String_Data_t *graph, uint8_t cnt, int flag, ...){
    static int idx[8];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, int);
    va_end(ap);
    
    if(graph->Graph_Control.width != idx[flag]){
        UICharDraw(graph,(char*)graph->Graph_Control.graphic_name,Graphic_Operate_CHANGE,graph->Graph_Control.layer,graph->Graph_Control.color,graph->Graph_Control.start_angle,idx[flag],graph->Graph_Control.start_x,graph->Graph_Control.start_y,graph->show_Data);
        string_refresh_flag[(uint8_t)(graph->Graph_Control.graphic_name[2]-'a')][(uint8_t)(graph->Graph_Control.graphic_name[1]-'0')] = 1;
    }
}
static void UI_StringSwitchDetect_Char(String_Data_t *graph, uint8_t cnt, int flag, ...){
    static char* idx[32];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, char*);
    va_end(ap);
    
    if(strcmp(graph->show_Data, idx[flag])){
        UICharDraw(graph,(char*)graph->Graph_Control.graphic_name,Graphic_Operate_CHANGE,graph->Graph_Control.layer,graph->Graph_Control.color,graph->Graph_Control.start_angle,graph->Graph_Control.width,graph->Graph_Control.start_x,graph->Graph_Control.start_y,idx[flag]);
        string_refresh_flag[(uint8_t)(graph->Graph_Control.graphic_name[2]-'a')][(uint8_t)(graph->Graph_Control.graphic_name[1]-'0')] = 1;
    }
}
void MyUIRefresh(void)
{
    SubGetMessage(UI_cmd_sub, &UI_data_recv);
    
    /* 动态圆圈 */
    {   
        // 遥控器连接
        UI_StateSwitchDetect_Graph(&circle_remoteConnection, 2, UI_data_recv.rc_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 图传连接
        UI_StateSwitchDetect_Graph(&circle_visitonConnection, 2, UI_data_recv.vision_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 自定义控制器连接 
        UI_StateSwitchDetect_Graph(&circle_customControConnection, 2, UI_data_recv.custom_contro_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 臂气泵
        UI_StateSwitchDetect_Graph(&circle_pumpArm, 2, UI_data_recv.pump_arm_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 推杆气泵
        UI_StateSwitchDetect_Graph(&circle_pumpValve, 2, UI_data_recv.pump_valve_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 臂模式
        UI_StateSwitchDetect_Graph(&circle_armAutoMode, 3, UI_data_recv.arm_mode, Graphic_Color_White, Graphic_Color_Green, Graphic_Color_Yellow);
        // 推杆模式
        UI_StateSwitchDetect_Graph(&circle_valveAutoMode, 3, UI_data_recv.valve_mode, Graphic_Color_White, Graphic_Color_Green, Graphic_Color_Yellow);
    }
    /* 动态字符串 */
    {
        // 控制模式
        UI_StringSwitchDetect_Char(&string_controMode_, 6, UI_data_recv.Contro_mode, S_ControMode_None, S_ControMode_FetchCube, S_ControlMode_ConvertCube, S_ControlMode_Move, S_ControlMode_ReverseMove, S_ControlMode_RotateMove);
        // 臂自动模式
        UI_WidthSwitchDetect_Char(&string_armAutoMode_, 2, UI_data_recv.arm_temp_halt_selected, 3, 5);
        UI_ColorSwitchDetect_Char(&string_armAutoMode_, 2, UI_data_recv.arm_selected_mode_state, Graphic_Color_Orange, Graphic_Color_Cyan);
        UI_StringSwitchDetect_Char(&string_armAutoMode_, 12, UI_data_recv.arm_selected_mode, S_ArmMode_NONE, S_ArmMode_ArmWalk, S_ArmMode_ArmIn, S_ArmMode_OUT, S_ArmMode_goldcube_right, S_ArmMode_warehouse1, S_ArmMode_warehouse2, S_ArmMode_silvercube_left, S_ArmMode_silvercube_mid, S_ArmMode_silvercube_right, S_ArmMode_gronded_cube, S_ArmMode_ConvertCube);
        // 推杆自动模式
        UI_WidthSwitchDetect_Char(&string_valveAutoMode_, 2, UI_data_recv.valve_temp_halt_selected, 3, 5);
        UI_ColorSwitchDetect_Char(&string_valveAutoMode_, 2, UI_data_recv.valve_selected_mode_state, Graphic_Color_Orange, Graphic_Color_Cyan);
        UI_StringSwitchDetect_Char(&string_valveAutoMode_, 3, UI_data_recv.valve_selected_mode, S_ValveMode_None, S_ValveMode_goldcube_mid, S_ValveMode_goldcube_left);
    }
    /* 多状态动态UI */
    {
        // 动态高度标注
        if(height_makerLine.height != UI_data_recv.arm_current_data.height){
            const float safe_height = -200.0f;
            height_makerLine.height = UI_data_recv.arm_current_data.height;
            if(height_makerLine.height > safe_height){
                UIFloatDraw(&float_z, "23", Graphic_Operate_CHANGE, 1, Graphic_Color_Green, 20, 2, 3, height_makerLine.pos_x, height_makerLine.pos_y+height_makerLine.height/2+30, (int32_t)(height_makerLine.height*10000)/10);
                UILineDraw(&line_z, "22", Graphic_Operate_CHANGE, 1, Graphic_Color_Green, 2 , height_makerLine.pos_x, height_makerLine.pos_y+height_makerLine.height/2, height_makerLine.pos_x+130, height_makerLine.pos_y+height_makerLine.height/2);
            }else{
                UIFloatDraw(&float_z, "23", Graphic_Operate_CHANGE, 1, Graphic_Color_Purplish_red, 20, 2, 3, height_makerLine.pos_x, height_makerLine.pos_y+height_makerLine.height/2+30, (int32_t)(height_makerLine.height*10000)/10);
                UILineDraw(&line_z, "22", Graphic_Operate_CHANGE, 1, Graphic_Color_Purplish_red, 2 , height_makerLine.pos_x, height_makerLine.pos_y+height_makerLine.height/2, height_makerLine.pos_x+130, height_makerLine.pos_y+height_makerLine.height/2);
            }
            graph_refresh_flag[2][2] = 1;
            graph_refresh_flag[2][3] = 1;
        }
        // 动态装甲板
        if(armour_maker.offset_angle != UI_data_recv.gimbal_offset_angle){
            armour_maker.offset_angle = UI_data_recv.gimbal_offset_angle;
            float start_angle,end_angle;
            start_angle = armour_maker.offset_angle+155;end_angle = armour_maker.offset_angle+205;
            if(start_angle > 360){start_angle-=360;end_angle-=360;}
            UIArcDraw(&Arc_armour_1, "24", Graphic_Operate_CHANGE, 1, Graphic_Color_Main, start_angle, end_angle, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, armour_maker.dx, armour_maker.dy);
            start_angle = armour_maker.offset_angle+245;end_angle = armour_maker.offset_angle+295;
            if(start_angle > 360){start_angle-=360;end_angle-=360;}
            UIArcDraw(&Arc_armour_2, "25", Graphic_Operate_CHANGE, 1, Graphic_Color_Main, start_angle, end_angle, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, armour_maker.dx, armour_maker.dy);
            start_angle = armour_maker.offset_angle+335;end_angle = armour_maker.offset_angle+385;
            if(start_angle > 360){start_angle-=360;end_angle-=360;}
            UIArcDraw(&Arc_armour_3, "26", Graphic_Operate_CHANGE, 1, Graphic_Color_Cyan, start_angle, end_angle, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, armour_maker.dx, armour_maker.dy);
            UIArcDraw(&Arc_armour_4, "27", Graphic_Operate_CHANGE, 1, Graphic_Color_Main, armour_maker.offset_angle+65,  armour_maker.offset_angle+115, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, armour_maker.dx, armour_maker.dy);
            graph_refresh_flag[2][4] = 1;
            graph_refresh_flag[2][5] = 1;
            graph_refresh_flag[2][6] = 1;
            graph_refresh_flag[2][7] = 1;
        }
        // 动态臂状态
        {
            UIFloatDraw(&float_current_bigyaw, "21", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 650, 285, 1000*UI_data_recv.arm_current_data.big_yaw_angle);
            UIFloatDraw(&float_current_midyaw, "22", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 650, 255, 1000*UI_data_recv.arm_current_data.mid_yaw_angle);
            UIFloatDraw(&float_current_asyaw, "23", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 650, 225, 1000*UI_data_recv.arm_current_data.assorted_yaw_angle);
            UIFloatDraw(&float_current_asroll, "24", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 650, 195, 1000*UI_data_recv.arm_current_data.assorted_roll_angle);
            UIFloatDraw(&float_current_tail, "25", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 650, 165, 1000*UI_data_recv.arm_current_data.tail_motor_angle);
            UIFloatDraw(&float_target_bigyaw, "26", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 780, 285, 1000*UI_data_recv.arm_target_data.big_yaw_angle);
            UIFloatDraw(&float_target_midyaw, "27", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 780, 255, 1000*UI_data_recv.arm_target_data.mid_yaw_angle);
            UIFloatDraw(&float_target_asyaw, "28", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 780, 225, 1000*UI_data_recv.arm_target_data.assorted_yaw_angle);
            UIFloatDraw(&float_target_asroll, "29", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 780, 195, 1000*UI_data_recv.arm_target_data.assorted_roll_angle);
            UIFloatDraw(&float_target_tail, "20", Graphic_Operate_CHANGE, 5, Graphic_Color_White, 15, 1, 3, 780, 165, 1000*UI_data_recv.arm_target_data.tail_motor_angle);
        }
    }
    
    // /* 轮询发送 */
    // //-先刷新字符串
    // static uint8_t priority_p,idx_p;
    // priority_p = 3; idx_p = 0;
    // while(1){
    //     if(string_group[priority_p][idx_p]==NULL){
    //         idx_p = 0;
    //         if(priority_p!=0)   {priority_p--;continue;}
    //         else    break;
    //     }
    //     if(string_refresh_flag[priority_p][idx_p]){
    //         string_refresh_flag[priority_p][idx_p] = 0;
    //         UICharRefresh(&referee_data->referee_id, *string_group[4][idx_p]);
    //     }
    //     idx_p++;
    // }
    // //-刷新图形
    // static Graph_Data_t* graph_group_temp[8];
    // static int graph_group_idx;
    // graph_group_idx = 0;
    // priority_p = 3; idx_p = 0;
    // while(graph_group_idx < 7){
    //     if(graph_group[priority_p][idx_p]==NULL){
    //         idx_p = 0;
    //         if(priority_p!=0)   priority_p--;
    //         else{
    //             graph_group_temp[graph_group_idx++] = graph_group[2][0];
    //         }
    //         continue;
    //     }
    //     if(graph_refresh_flag[priority_p][idx_p]){
    //         graph_refresh_flag[priority_p][idx_p] = 0;
    //         graph_group_temp[graph_group_idx++] = graph_group[priority_p][idx_p];
    //     }
    //     idx_p++;
    // }
    // UIGraphRefresh(&referee_data->referee_id, 7 ,*graph_group_temp[0], *graph_group_temp[1], *graph_group_temp[2], *graph_group_temp[3], *graph_group_temp[4], *graph_group_temp[5], *graph_group_temp[6]);

    
    if(string_refresh_flag[3][0]) 
        {string_refresh_flag[3][0]=0;UICharRefresh(&referee_data->referee_id, string_controMode_);}   
    if(string_refresh_flag[3][1]) {string_refresh_flag[3][1]=0;UICharRefresh(&referee_data->referee_id, string_armAutoMode_);}
    if(string_refresh_flag[3][2]) {string_refresh_flag[3][2]=0;UICharRefresh(&referee_data->referee_id, string_valveAutoMode_);}
    UIGraphRefresh(&referee_data->referee_id, 5, float_current_bigyaw,float_current_midyaw,float_current_asyaw,float_current_asroll,float_current_tail);
    UIGraphRefresh(&referee_data->referee_id, 5, float_target_bigyaw,float_target_midyaw,float_target_asyaw,float_target_asroll,float_target_tail);
    UIGraphRefresh(&referee_data->referee_id, 7, circle_pumpArm,circle_pumpValve,circle_armAutoMode,circle_valveAutoMode,circle_customControConnection,circle_visitonConnection,circle_remoteConnection);
    UIGraphRefresh(&referee_data->referee_id, 7, rectangle_Z,line_z,float_z,Arc_armour_1,Arc_armour_2,Arc_armour_3,Arc_armour_4);
}