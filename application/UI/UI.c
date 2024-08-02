

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
#include "UI_user_defined.h"

float UI_debug_value[7];

referee_info_t *referee_data;

UI_data_t UI_data_recv;
Subscriber_t *UI_cmd_sub;

uint8_t UI_Seq = 0;

uint32_t UI_param_p;
Flash_write_param_t get_UI_param(){
    for(int i = 0; i < USER_DEFINED_UI_MAXNUM; i++){
        user_defined_ui_config[i].color = user_defined_ui[i]->color;
        user_defined_ui_config[i].layer = user_defined_ui[i]->layer;
        user_defined_ui_config[i].param = user_defined_ui[i]->param;
        user_defined_ui_config[i].pos_x = user_defined_ui[i]->pos_x;
        user_defined_ui_config[i].pos_y = user_defined_ui[i]->pos_y;
        user_defined_ui_config[i].priority = user_defined_ui[i]->priority;
        user_defined_ui_config[i].type = user_defined_ui[i]->type;
        user_defined_ui_config[i].width = user_defined_ui[i]->width;
    }
    Flash_write_param_t data;
    data.address = UI_param_record_address;
    data.data =  (uint32_t*)user_defined_ui_config;
    data.len  = sizeof(user_defined_ui_config);
    return data;
}

static const char S_ControMode[][12] = {
    "NONE       ",
    "FREE       ",
    "Convert    ",
    "Move       ",
    "ReverseMove",
    "Rotate     "
};
static const char S_ArmMode[][12] = {
    "NONE       ",
    "Walk       ",
    "IN         ",
    "OUT        ",
    "goldright  ",
    "store_up   ",
    "store_down ",
    "silve_left ",
    "silve_mid  ",
    "silve_right",
    "grond      ",
    "Convert    ",
    "straighten ",
    "block_front",
    "block_side ",
    "block_back ",
    "place_up   ",
    "place_down "
};

static const char S_ValveMode[][10] = {
    "NONE     ",
    "gold_mid ",
    "gold_left",
};
static const char S_SongList[][16] = {
    "StartUP",
    "No_RC",
    "You",
    "Prepare",
    "Test",
    "DIDIDA",
    "GuYongZhe",
    "YongZheDouELong",
    "DuoLaAMeng"
};
/*正常模式下UI*/
static UI_GRAPH_INSTANCE* SIGN_pumpArm;
static UI_GRAPH_INSTANCE* SIGN_pumpValve;
static UI_GRAPH_INSTANCE* circle_armAutoMode;
static UI_GRAPH_INSTANCE* circle_valveAutoMode;
static UI_GRAPH_INSTANCE* circle_customControConnection;
static UI_GRAPH_INSTANCE* circle_hostConnection;
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
static UI_GRAPH_INSTANCE* number_pump_air_arm;
static UI_GRAPH_INSTANCE* number_pump_air_valve;
static UI_GRAPH_INSTANCE* circle_jointMotorState[6];
static UI_GRAPH_INSTANCE* circle_jointEncoderState[4];

static UI_STRING_INSTANCE* string_controMode;
static UI_STRING_INSTANCE* string_armAutoMode;
static UI_STRING_INSTANCE* string_valveAutoMode;
static UI_STRING_INSTANCE* string_armPump;
static UI_STRING_INSTANCE* string_valvePump;
static UI_STRING_INSTANCE* string_controMode_;
static UI_STRING_INSTANCE* string_armAutoMode_;
static UI_STRING_INSTANCE* string_valveAutoMode_;
static UI_STRING_INSTANCE* string_selectedSong;
static UI_STRING_INSTANCE* string_jointMotorState;
static UI_STRING_INSTANCE* string_jointEncoderState;



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
    /*自定义UI*/
    UserDefinedUI_init();
    /*UI注册-正常*/
    //圆
    circle_armAutoMode = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,1250,750,5);
    circle_valveAutoMode = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,1250,680,5);
    circle_customControConnection = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,310,850,15);
    circle_hostConnection = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,390,850,15);
    circle_visitonConnection = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,230,850,15);
    circle_remoteConnection = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,150,850,15);
    
    //静态字符串
    string_controMode = UI_String_Init(0,2,Graphic_Color_Purplish_red,25,3,1290,840,"Control:");
    string_armAutoMode = UI_String_Init(0,2,Graphic_Color_Purplish_red,25,3,1290,765,"ARM_AUTO:");
    string_valveAutoMode = UI_String_Init(0,2,Graphic_Color_Purplish_red,25,3,1290,700,"VALVE_AUTO:");
    
    //动态字符串    
    string_controMode_ = UI_String_Init(0,3,Graphic_Color_Orange,25,3,1565,840,S_ControMode[0]);
    string_armAutoMode_ = UI_String_Init(0,3,Graphic_Color_Orange,25,3,1565,765,S_ArmMode[0]);
    string_valveAutoMode_ = UI_String_Init(0,3,Graphic_Color_Orange,25,3,1565,700,S_ValveMode[0]);
    string_selectedSong = UI_String_Init(0,3,Graphic_Color_Orange,25,3,1565,635,S_SongList[0]);
    //矩形
    rectangle_Z = UI_Graph_Init(GraphType_Rect, 0, 1, Graphic_Color_Main, 2, height_makerLine.pos_x+130, height_makerLine.pos_y-310, 50, 310);
    //高度指示线(直线+浮点数)
    line_z = UI_Graph_Init(GraphType_Line, 1, 1, Graphic_Color_Purplish_red, 2 , height_makerLine.pos_x, height_makerLine.pos_y, 130, -90);
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
    UI_BatchDisable_Graph(5, float_current_bigyaw,float_current_midyaw,float_current_asyaw,float_current_asroll,float_current_tail);
    UI_BatchDisable_Graph(5, float_target_bigyaw,float_target_midyaw,float_target_asyaw,float_target_asroll,float_target_tail);
        // UI_BatchEnable_Graph(7,float_current_bigyaw,float_current_midyaw,float_current_asyaw,float_target_bigyaw,float_target_midyaw,float_target_asyaw,float_target_asroll);
    //臂动作
    //todo:
    //气泵状态
    SIGN_pumpArm = UI_Graph_Init(GraphType_Arc, 2, 1, Graphic_Color_White,10,1050,540,40,100,50,80);
    SIGN_pumpValve = UI_Graph_Init(GraphType_Arc, 2, 1, Graphic_Color_White,10,870,540,220,100,50,80);
    number_pump_air_arm = UI_Graph_Init(GraphType_Number, 0, 5, Graphic_Color_White, 3, 1130, 465, 25, 0);
    number_pump_air_valve = UI_Graph_Init(GraphType_Number, 0, 5, Graphic_Color_White, 3, 730, 465, 25, 0);
    string_armPump = UI_String_Init(0,2,Graphic_Color_White,25,3,1090,465,"A");
    string_valvePump = UI_String_Init(0,2,Graphic_Color_White,25,3,830,465,"V");
    //装甲板
    Arc_armour_1 = UI_Graph_Init(GraphType_Arc, 3, 1, Graphic_Color_Main, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, 335, 50, armour_maker.dx, armour_maker.dy);
    Arc_armour_2 = UI_Graph_Init(GraphType_Arc, 3, 1, Graphic_Color_Cyan, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, 65, 50, armour_maker.dx, armour_maker.dy);
    Arc_armour_3 = UI_Graph_Init(GraphType_Arc, 3, 1, Graphic_Color_Cyan, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, 155, 50, armour_maker.dx, armour_maker.dy);
    Arc_armour_4 = UI_Graph_Init(GraphType_Arc, 3, 1, Graphic_Color_Cyan, armour_maker.width, armour_maker.pos_x, armour_maker.pos_y, 245, 50, armour_maker.dx, armour_maker.dy);
    UI_BatchDisable_Graph(3, Arc_armour_2, Arc_armour_3,Arc_armour_4);
    //关节状态
    string_jointMotorState = UI_String_Init(0,3,Graphic_Color_Orange,25,3,540,130,"MOTOR");
    string_jointEncoderState = UI_String_Init(0,3,Graphic_Color_Orange,25,3,540,95,"ENCODER");
    for(int i=0; i<6; i++)  circle_jointMotorState[i] = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,650+i*8,130,2);
    for(int i=0; i<4; i++)  circle_jointEncoderState[i] = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,15,650+i*8,95,2);
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

static void UI_operate(){
    {   
        // 遥控器连接
        UI_StateSwitchDetect_Graph(circle_remoteConnection, 2, UI_data_recv.rc_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 图传连接
        UI_StateSwitchDetect_Graph(circle_visitonConnection, 2, UI_data_recv.vision_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 自定义控制器连接 
        UI_StateSwitchDetect_Graph(circle_customControConnection, 2, UI_data_recv.custom_contro_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 上位机连接
        UI_StateSwitchDetect_Graph(circle_hostConnection, 2, UI_data_recv.host_connection_mode_t, Graphic_Color_White, Graphic_Color_Green);
        // 臂模式
        UI_StateSwitchDetect_Graph(circle_armAutoMode, 3, UI_data_recv.arm_mode, Graphic_Color_White, Graphic_Color_Green, Graphic_Color_Yellow);
        // 推杆模式
        UI_StateSwitchDetect_Graph(circle_valveAutoMode, 3, UI_data_recv.valve_mode, Graphic_Color_White, Graphic_Color_Green, Graphic_Color_Yellow);
        // 气路气压值
        number_pump_air_arm->param.Number.value = UI_data_recv.pump_air_arm;
        number_pump_air_valve->param.Number.value = UI_data_recv.pump_air_valve;
        UI_StateSwitchDetect_Graph(number_pump_air_arm, 2, UI_data_recv.pump_air_arm<-250, Graphic_Color_White, Graphic_Color_Green);
        UI_StateSwitchDetect_Graph(number_pump_air_valve, 2, UI_data_recv.pump_air_valve<-250, Graphic_Color_White, Graphic_Color_Green);
        // 气泵
        UI_StateSwitchDetect_Graph(SIGN_pumpArm, 3, UI_data_recv.pump_arm_mode_t+(UI_data_recv.pump_air_arm<-250), Graphic_Color_White, Graphic_Color_Purplish_red, Graphic_Color_Green);
        UI_StateSwitchDetect_Graph(SIGN_pumpValve, 3, UI_data_recv.pump_valve_mode_t+(UI_data_recv.pump_air_valve<-250), Graphic_Color_White, Graphic_Color_Purplish_red, Graphic_Color_Green);
    }
    {
        // 控制模式
        UI_StringSwitchDetect_Char(string_controMode_, 6, UI_data_recv.Contro_mode, S_ControMode[0],S_ControMode[1],S_ControMode[2],S_ControMode[3],S_ControMode[4],S_ControMode[5]);
        // 臂自动模式
        UI_WidthSwitchDetect_Char(string_armAutoMode_, 2, UI_data_recv.arm_temp_halt_selected, 3, 5);
        UI_ColorSwitchDetect_Char(string_armAutoMode_, 2, UI_data_recv.arm_selected_mode_state, Graphic_Color_Orange, Graphic_Color_Cyan);
        UI_StringSwitchDetect_Char(string_armAutoMode_, 18, UI_data_recv.arm_selected_mode, S_ArmMode[0],S_ArmMode[1],S_ArmMode[2],S_ArmMode[3],S_ArmMode[4],S_ArmMode[5],S_ArmMode[6],S_ArmMode[7],S_ArmMode[8],S_ArmMode[9],S_ArmMode[10],S_ArmMode[11],S_ArmMode[12],S_ArmMode[13],S_ArmMode[14],S_ArmMode[15],S_ArmMode[16],S_ArmMode[17]);
        // 推杆自动模式
        UI_WidthSwitchDetect_Char(string_valveAutoMode_, 2, UI_data_recv.valve_halt_selected, 3, 5);
        UI_ColorSwitchDetect_Char(string_valveAutoMode_, 2, UI_data_recv.valve_selected_mode_state, Graphic_Color_Orange, Graphic_Color_Cyan);
        UI_StringSwitchDetect_Char(string_valveAutoMode_, 3, UI_data_recv.valve_selected_mode, S_ValveMode[0],S_ValveMode[1],S_ValveMode[2]);
        //歌曲选择
        UI_StringSwitchDetect_Char(string_selectedSong, sizeof(S_SongList)/sizeof(S_SongList[0]), UI_data_recv.selected_song, S_SongList[0],S_SongList[1],S_SongList[2],S_SongList[3],S_SongList[4],S_SongList[5],S_SongList[6],S_SongList[7],S_SongList[8]);
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
            Arc_armour_4->param.Arc.start_angle = armour_maker.offset_angle+245;        
        }
        // 臂关节数据
        {
            // float_current_bigyaw->param.Float.value = UI_data_recv.arm_current_data.big_yaw_angle;
            // float_current_midyaw->param.Float.value = UI_data_recv.arm_current_data.mid_yaw_angle;
            // float_current_asyaw->param.Float.value = UI_data_recv.arm_current_data.assorted_yaw_angle;
            // float_current_asroll->param.Float.value = UI_data_recv.arm_current_data.assorted_roll_angle;
            // float_current_tail->param.Float.value = UI_data_recv.arm_current_data.tail_motor_angle;
            // float_target_bigyaw->param.Float.value = UI_data_recv.arm_target_data.big_yaw_angle;
            // float_target_midyaw->param.Float.value = UI_data_recv.arm_target_data.mid_yaw_angle;
            // float_target_asyaw->param.Float.value = UI_data_recv.arm_target_data.assorted_yaw_angle;
            // float_target_asroll->param.Float.value = UI_data_recv.arm_target_data.assorted_roll_angle;
            // float_target_tail->param.Float.value = UI_data_recv.arm_target_data.tail_motor_angle;        
        }
        // DEBUG
        {
            // float_current_bigyaw->param.Float.value = UI_debug_value[0];
            // float_current_midyaw->param.Float.value = UI_debug_value[1];
            // float_current_asyaw->param.Float.value = UI_debug_value[2];

            // float_target_bigyaw->param.Float.value = UI_debug_value[3];
            // float_target_midyaw->param.Float.value = UI_debug_value[4];
            // float_target_asyaw->param.Float.value = UI_debug_value[5];
            // float_target_asroll->param.Float.value = UI_debug_value[6];
        }
        // 关节状态
        {
            //电机
            UI_StateSwitchDetect_Graph(circle_jointMotorState[0], 2, UI_data_recv.joint_state.motor_state.bigyaw, Graphic_Color_White, Graphic_Color_Green);
            UI_StateSwitchDetect_Graph(circle_jointMotorState[1], 2, UI_data_recv.joint_state.motor_state.midyaw, Graphic_Color_White, Graphic_Color_Green);
            UI_StateSwitchDetect_Graph(circle_jointMotorState[2], 2, UI_data_recv.joint_state.motor_state.assortedyaw, Graphic_Color_White, Graphic_Color_Green);
            UI_StateSwitchDetect_Graph(circle_jointMotorState[3], 2, UI_data_recv.joint_state.motor_state.assortedroll, Graphic_Color_White, Graphic_Color_Green);
            UI_StateSwitchDetect_Graph(circle_jointMotorState[4], 2, UI_data_recv.joint_state.motor_state.tail, Graphic_Color_White, Graphic_Color_Green);
            UI_StateSwitchDetect_Graph(circle_jointMotorState[5], 2, UI_data_recv.joint_state.motor_state.height, Graphic_Color_White, Graphic_Color_Green);
            //编码器
            UI_StateSwitchDetect_Graph(circle_jointEncoderState[0], 2, UI_data_recv.joint_state.encoder_state.bigyaw, Graphic_Color_White, Graphic_Color_Green);
            UI_StateSwitchDetect_Graph(circle_jointEncoderState[1], 2, UI_data_recv.joint_state.encoder_state.assortedyaw, Graphic_Color_White, Graphic_Color_Green);
            UI_StateSwitchDetect_Graph(circle_jointEncoderState[2], 2, UI_data_recv.joint_state.encoder_state.assortedroll, Graphic_Color_White, Graphic_Color_Green);
            UI_StateSwitchDetect_Graph(circle_jointEncoderState[3], 2, UI_data_recv.joint_state.encoder_state.tail, Graphic_Color_White, Graphic_Color_Green);
        }
    }
}
void MyUIRefresh(void)
{
    SubGetMessage(UI_cmd_sub, &UI_data_recv);
    
    static uint8_t debug_flag_switch = 0;
    EdgeType debug_flag_edge = detect_edge(&debug_flag_switch, UI_data_recv.debug.debug_flag);
    if(debug_flag_edge == EDGE_RISING)
        UserDefinedUI_DISPLAY();
    else if(debug_flag_edge == EDGE_FALLING)
        UserDefinedUI_UNDISPLAY();

    if(UI_data_recv.debug.debug_flag){
        UserDefinedUI_operate(&UI_data_recv.debug);
    }else{
        UI_operate();
    }
    
    if(UI_data_recv.UI_refresh_request){
        UI_Graph_Refresh();
        UI_String_Refresh();
    }

    UI_String_Update();
    UI_Graph_Update();
}