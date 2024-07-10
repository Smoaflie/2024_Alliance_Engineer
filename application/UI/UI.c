

#include "referee_init.h"
#include "referee_UI.h"
#include "referee_protocol.h"
#include "message_center.h"
#include "rm_referee.h"
#include "UI_ref.h"
#include "UI.h"
#include <string.h>
#include "flashtask.h"
#include "UI_interface.h"

referee_info_t *referee_data;

UI_data_t UI_data_recv;
Subscriber_t *UI_cmd_sub;

uint8_t UI_Seq = 0;

uint32_t UI_param_p;
Flash_write_param_t get_UI_param(){
    Flash_write_param_t data;
    data.address = UI_param_record_address;
    data.data =  (uint32_t*)&UI_param_p;
    data.len  = sizeof(UI_param_p);
    return data;
}
static const char S_ControMode_None[] = "NONE       ";
static const char S_ControMode_FetchCube[] = "FREE       ";
static const char S_ControlMode_ConvertCube[] = "Convert    ";
static const char S_ControlMode_Move[] = "Move       ";
static const char S_ControlMode_ReverseMove[] = "ReverseMove";
static const char S_ControlMode_RotateMove[] = "Rotate     ";

static const char S_ArmMode_NONE[] = "NONE       ";
static const char S_ArmMode_ArmWalk[] = "Walk       ";
static const char S_ArmMode_ArmIn[] = "IN         ";
static const char S_ArmMode_OUT[] = "OUT        ";
static const char S_ArmMode_goldcube_right[] = "goldright  ";
static const char S_ArmMode_warehouse1[] = "store_up   ";
static const char S_ArmMode_warehouse2[] = "store_down ";
static const char S_ArmMode_silvercube_left[] = "silve_left ";
static const char S_ArmMode_silvercube_mid[] = "silve_mid  ";
static const char S_ArmMode_silvercube_right[] = "silve_right";
static const char S_ArmMode_gronded_cube[] = "grond      ";
static const char S_ArmMode_ConvertCube[] = "Convert    ";

static const char S_ValveMode_None[] = "NONE     ";
static const char S_ValveMode_goldcube_mid[] = "gold_mid ";
static const char S_ValveMode_goldcube_left[] = "gold_left";



// 图形变量
static UI_GRAPH_INSTANCE* circle_pumpArm;
static UI_GRAPH_INSTANCE* circle_pumpValve;
static UI_GRAPH_INSTANCE* circle_armAutoMode;
static UI_GRAPH_INSTANCE* circle_valveAutoMode;
static UI_GRAPH_INSTANCE* circle_customControConnection;
static UI_GRAPH_INSTANCE* circle_visitonConnection;
static UI_GRAPH_INSTANCE* circle_remoteConnection;
static UI_GRAPH_INSTANCE* rectangle_Z;
static UI_GRAPH_INSTANCE* line_z;
static UI_GRAPH_INSTANCE* float_z;
static UI_GRAPH_INSTANCE* Arc_armour_1;
static UI_GRAPH_INSTANCE* Arc_armour_2;
static UI_GRAPH_INSTANCE* Arc_armour_3;
static UI_GRAPH_INSTANCE* Arc_armour_4;
static UI_GRAPH_INSTANCE* float_current_bigyaw;
static UI_GRAPH_INSTANCE* float_current_midyaw;
static UI_GRAPH_INSTANCE* float_current_asyaw;
static UI_GRAPH_INSTANCE* float_current_asroll;
static UI_GRAPH_INSTANCE* float_current_tail;
static UI_GRAPH_INSTANCE* float_target_bigyaw;
static UI_GRAPH_INSTANCE* float_target_midyaw;
static UI_GRAPH_INSTANCE* float_target_asyaw;
static UI_GRAPH_INSTANCE* float_target_asroll;
static UI_GRAPH_INSTANCE* float_target_tail;


static UI_STRING_INSTANCE* string_controMode;
static UI_STRING_INSTANCE* string_armAutoMode;
static UI_STRING_INSTANCE* string_valveAutoMode;
static UI_STRING_INSTANCE* string_armPump;
static UI_STRING_INSTANCE* string_valvePump;
static UI_STRING_INSTANCE* string_controMode_;
static UI_STRING_INSTANCE* string_armAutoMode_;
static UI_STRING_INSTANCE* string_valveAutoMode_;
static UI_STRING_INSTANCE* string_BIGYAW;
static UI_STRING_INSTANCE* string_MIDYAW;
static UI_STRING_INSTANCE* string_ASYAW;
static UI_STRING_INSTANCE* string_ASROLL;
static UI_STRING_INSTANCE* string_TAIL;

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
    int32_t offset_angle;
}armour_maker;
void get_referee_data(referee_info_t *referee_data)
{
    referee_data                               = referee_data;
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->referee_id.Robot_ID; // 计算客户端ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}

static void ui_reload(){
    /*UI结构体初始化*/
    //圆
    circle_pumpArm = UI_Graph_Init(GraphType_Round, 2, 1, Graphic_Color_White,15,150,750,15);
    circle_pumpValve = UI_Graph_Init(GraphType_Round, 2, 1, Graphic_Color_White,15,150,700,15);
    circle_armAutoMode = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,1250,750,5);
    circle_valveAutoMode = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,1250,680,5);
    circle_customControConnection = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,310,850,15);
    circle_visitonConnection = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,230,850,15);
    circle_remoteConnection = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,150,850,15);
    //静态字符串
    string_controMode = UI_String_Init(0,2,Graphic_Color_Purplish_red,25,3,1290,840,"Control:");
    string_armAutoMode = UI_String_Init(0,2,Graphic_Color_Purplish_red,25,3,1290,765,"ARM_AUTO:");
    string_valveAutoMode = UI_String_Init(0,2,Graphic_Color_Purplish_red,25,3,1290,700,"VALVE_AUTO:");
    string_armPump = UI_String_Init(0,2,Graphic_Color_Purplish_red,25,3,195,770,"ARM_PUMP");
    string_valvePump = UI_String_Init(0,2,Graphic_Color_Purplish_red,25,3,195,715,"VALVE_PUMP");
    
    string_BIGYAW = UI_String_Init(0,2,Graphic_Color_Main,15,3,540,285,"BIGYAW");
    string_MIDYAW = UI_String_Init(0,2,Graphic_Color_Main,15,3,540,255,"MIDYAW");
    string_ASYAW = UI_String_Init(0,2,Graphic_Color_Main,15,3,540,225,"AS_YAW");
    string_ASROLL = UI_String_Init(0,2,Graphic_Color_Main,15,3,540,195,"ASROLL");
    string_TAIL = UI_String_Init(0,2,Graphic_Color_Main,15,3,540,165,"TAIL");
    //动态字符串
    string_controMode_ = UI_String_Init(0,3,Graphic_Color_Orange,25,3,1565,840,S_ControMode_None);
    string_armAutoMode_ = UI_String_Init(0,3,Graphic_Color_Orange,25,3,1565,765,S_ArmMode_NONE);
    string_valveAutoMode_ = UI_String_Init(0,3,Graphic_Color_Orange,25,3,1565,700,S_ValveMode_None);
    //矩形
    rectangle_Z = UI_Graph_Init(GraphType_Rect, 0, 1, Graphic_Color_Main, 2, height_makerLine.pos_x+130, height_makerLine.pos_y-310, 50, 310);
    //高度指示线(直线+浮点数)
    line_z = UI_Graph_Init(GraphType_Line, 1, 1, Graphic_Color_Purplish_red, 2 , height_makerLine.pos_x, height_makerLine.pos_y, 130, (double)-90);
    float_z = UI_Graph_Init(GraphType_Float, 1, 1, Graphic_Color_Purplish_red, 3, height_makerLine.pos_x, height_makerLine.pos_y+35, 20, 1, 0);
    //臂状态
    float_current_bigyaw = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 650, 285, 15, 1, 0);
    float_current_midyaw = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 650, 255, 15, 1, 0);
    float_current_asyaw = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 650, 225, 15, 1, 0);
    float_current_asroll = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 650, 195, 15, 1, 0);
    float_current_tail = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 650, 165, 15, 1, 0);
    float_target_bigyaw = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 780, 285, 15, 1, 0);
    float_target_midyaw = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 780, 255, 15, 1, 0);
    float_target_asyaw = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 780, 225, 15, 1, 0);
    float_target_asroll = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 780, 195, 15, 1, 0);
    float_target_tail = UI_Graph_Init(GraphType_Float, 0, 5, Graphic_Color_Purplish_red, 3, 780, 165, 15, 1, 0);
    //臂动作
    //todo:
    //装甲板
    Arc_armour_1 = UI_Graph_Init(GraphType_Arc, 3, 1, Graphic_Color_Main, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, 335, 50, armour_maker.dx, armour_maker.dy);
    Arc_armour_2 = UI_Graph_Init(GraphType_Arc, 3, 1, Graphic_Color_Cyan, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, 65, 50, armour_maker.dx, armour_maker.dy);
    Arc_armour_3 = UI_Graph_Init(GraphType_Arc, 3, 1, Graphic_Color_Cyan, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, 155, 50, armour_maker.dx, armour_maker.dy);
    Arc_armour_4 = UI_Graph_Init(GraphType_Arc, 3, 1, Graphic_Color_Cyan, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, 245, 50, armour_maker.dx, armour_maker.dy);
    //清除所有UI
    UIDelete(&referee_data->referee_id, UI_Data_Del_ALL, 0);
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
static void UI_StateSwitchDetect_Graph(UI_GRAPH_INSTANCE* instance, uint8_t cnt, int flag, ...){
    Graph_Data_t *graph = &instance->graph;
    static uint8_t idx[32];

    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, int);
    va_end(ap);
    if(graph->color != idx[flag]){
        instance->color = idx[flag];
    }
}
static void UI_ColorSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...){
   static int idx[8];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, int);
    va_end(ap);
    
    if(instance->string.Graph_Control.color != idx[flag]){
        instance->color = idx[flag];
    }
}
static void UI_WidthSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...){
   static int idx[8];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, int);
    va_end(ap);
    
    if(instance->string.Graph_Control.width != idx[flag]){
        instance->width = idx[flag];
    }
}
static void UI_StringSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...){
   static const char* idx[32];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap,const char*);
    va_end(ap);
    
    if(strcmp(instance->string.show_Data, idx[flag])){
        instance->char_p = idx[flag];
    }
}
void MyUIRefresh(void)
{
    SubGetMessage(UI_cmd_sub, &UI_data_recv);
    
    /* 动态圆圈 */
    {   

        // 遥控器连接
        UI_StateSwitchDetect_Graph(circle_remoteConnection, 2, UI_data_recv.rc_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 图传连接
        UI_StateSwitchDetect_Graph(circle_visitonConnection, 2, UI_data_recv.vision_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 自定义控制器连接 
        UI_StateSwitchDetect_Graph(circle_customControConnection, 2, UI_data_recv.custom_contro_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 臂气泵
        UI_StateSwitchDetect_Graph(circle_pumpArm, 2, UI_data_recv.pump_arm_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 推杆气泵
        UI_StateSwitchDetect_Graph(circle_pumpValve, 2, UI_data_recv.pump_valve_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 臂模式
        UI_StateSwitchDetect_Graph(circle_armAutoMode, 3, UI_data_recv.arm_mode, Graphic_Color_White, Graphic_Color_Green, Graphic_Color_Yellow);
        // 推杆模式
        UI_StateSwitchDetect_Graph(circle_valveAutoMode, 3, UI_data_recv.valve_mode, Graphic_Color_White, Graphic_Color_Green, Graphic_Color_Yellow);
    }
    /* 动态字符串 */
    {
        // 控制模式
        UI_StringSwitchDetect_Char(string_controMode_, 6, UI_data_recv.Contro_mode, S_ControMode_None, S_ControMode_FetchCube, S_ControlMode_ConvertCube, S_ControlMode_Move, S_ControlMode_ReverseMove, S_ControlMode_RotateMove);
        // 臂自动模式
        UI_WidthSwitchDetect_Char(string_armAutoMode_, 2, UI_data_recv.arm_temp_halt_selected, 3, 5);
        UI_ColorSwitchDetect_Char(string_armAutoMode_, 2, UI_data_recv.arm_selected_mode_state, Graphic_Color_Orange, Graphic_Color_Cyan);
        UI_StringSwitchDetect_Char(string_armAutoMode_, 12, UI_data_recv.arm_selected_mode, S_ArmMode_NONE, S_ArmMode_ArmWalk, S_ArmMode_ArmIn, S_ArmMode_OUT, S_ArmMode_goldcube_right, S_ArmMode_warehouse1, S_ArmMode_warehouse2, S_ArmMode_silvercube_left, S_ArmMode_silvercube_mid, S_ArmMode_silvercube_right, S_ArmMode_gronded_cube, S_ArmMode_ConvertCube);
        // 推杆自动模式
        UI_WidthSwitchDetect_Char(string_valveAutoMode_, 2, UI_data_recv.valve_temp_halt_selected, 3, 5);
        UI_ColorSwitchDetect_Char(string_valveAutoMode_, 2, UI_data_recv.valve_selected_mode_state, Graphic_Color_Orange, Graphic_Color_Cyan);
        UI_StringSwitchDetect_Char(string_valveAutoMode_, 3, UI_data_recv.valve_selected_mode, S_ValveMode_None, S_ValveMode_goldcube_mid, S_ValveMode_goldcube_left);
    }
    /* 多状态动态UI */
    {
        // 动态高度标注
        if(height_makerLine.height != UI_data_recv.arm_current_data.height){
            const float safe_height = -200.0f;
            height_makerLine.height = UI_data_recv.arm_current_data.height;
            if(height_makerLine.height > safe_height){
                float_z->color = Graphic_Color_Green;
                line_z->color = Graphic_Color_Green;
            }else{
                float_z->color = Graphic_Color_Purplish_red;
                line_z->color = Graphic_Color_Purplish_red;
            }
            float_z->param.Float.value = height_makerLine.height;
            float_z->pos_y = height_makerLine.pos_y+height_makerLine.height/2 + 35;
            line_z->pos_y = height_makerLine.pos_y+height_makerLine.height/2;
        }
        // 动态装甲板
        if(armour_maker.offset_angle != UI_data_recv.gimbal_offset_angle){
            armour_maker.offset_angle = UI_data_recv.gimbal_offset_angle;
            Arc_armour_1->param.Arc.start_angle = armour_maker.offset_angle+335;
            Arc_armour_2->param.Arc.start_angle = armour_maker.offset_angle+65;
            Arc_armour_3->param.Arc.start_angle = armour_maker.offset_angle+155;
            Arc_armour_4->param.Arc.start_angle = armour_maker.offset_angle+245;        }
        // 臂关节数据
        {
            float_current_bigyaw->param.Float.value = UI_data_recv.arm_current_data.big_yaw_angle;
            float_current_midyaw->param.Float.value = UI_data_recv.arm_current_data.mid_yaw_angle;
            float_current_asyaw->param.Float.value = UI_data_recv.arm_current_data.assorted_yaw_angle;
            float_current_asroll->param.Float.value = UI_data_recv.arm_current_data.assorted_roll_angle;
            float_current_tail->param.Float.value = UI_data_recv.arm_current_data.tail_motor_angle;
            float_target_bigyaw->param.Float.value = UI_data_recv.arm_target_data.big_yaw_angle;
            float_target_midyaw->param.Float.value = UI_data_recv.arm_target_data.mid_yaw_angle;
            float_target_asyaw->param.Float.value = UI_data_recv.arm_target_data.assorted_yaw_angle;
            float_target_asroll->param.Float.value = UI_data_recv.arm_target_data.assorted_roll_angle;
            float_target_tail->param.Float.value = UI_data_recv.arm_target_data.tail_motor_angle;        }
    }
    
    UI_String_Refresh();
    UI_Graph_Refresh();
}