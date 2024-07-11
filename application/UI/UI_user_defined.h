#include "UI_interface.h"

#define VALUE_BOUND(value,max,min)  (((value)>(max))?(min):((value)<(min)?(max):(value)))
static const char UI_DEBUG_STRING_ID[] = "ID:";
static const char UI_DEBUG_STRING_WIDTH[] = "WIDTH:";
static const char UI_DEBUG_STRING_POSX[] = "POS_X:";
static const char UI_DEBUG_STRING_POSY[] = "POS_Y:";
static const char UI_DEBUG_STRING_PARAM1[] = "PARAM1:";
static const char UI_DEBUG_STRING_PARAM2[] = "PARAM2:";
static const char UI_DEBUG_STRING_PARAM3[] = "PARAM3:";
static const char UI_DEBUG_STRING_PARAM4[] = "PARAM4:";

static const char UI_DEBUG_STRING_TYPE_Rect[] = "GraphType_Rect";
static const char UI_DEBUG_STRING_TYPE_Line[] = "GraphType_Line";
static const char UI_DEBUG_STRING_TYPE_Round[] = "GraphType_Round";
static const char UI_DEBUG_STRING_TYPE_Elipse[] = "GraphType_Elipse";
static const char UI_DEBUG_STRING_TYPE_Arc[] = "GraphType_Arc";

//自定义UI
#define USER_DEFINED_UI_MAXNUM 30
static UI_GRAPH_INSTANCE* user_defined_ui[USER_DEFINED_UI_MAXNUM];
static UI_GRAPH_CONFIG_s user_defined_ui_config[USER_DEFINED_UI_MAXNUM] = {0};
static uint8_t user_defined_ui_idx_pointer = 0;


/*调试模式下UI*/
static struct{
    union{
        UI_STRING_INSTANCE *string_group[11];
        struct{
            UI_STRING_INSTANCE *sign;
            UI_STRING_INSTANCE *string_id;
            UI_STRING_INSTANCE *string_type;
            UI_STRING_INSTANCE *string_color;
            UI_STRING_INSTANCE *string_width;
            UI_STRING_INSTANCE *string_pos_x;
            UI_STRING_INSTANCE *string_pos_y;
            UI_STRING_INSTANCE *string_param1;
            UI_STRING_INSTANCE *string_param2;
            UI_STRING_INSTANCE *string_param3;
            UI_STRING_INSTANCE *string_param4;
        };
    }string;
    union{
        UI_GRAPH_INSTANCE *graph_group[8];
        struct{
            UI_GRAPH_INSTANCE *graph_id;
            UI_GRAPH_INSTANCE *graph_width;
            UI_GRAPH_INSTANCE *graph_pos_x;
            UI_GRAPH_INSTANCE *graph_pos_y;
            UI_GRAPH_INSTANCE *graph_param1;
            UI_GRAPH_INSTANCE *graph_param2;
            UI_GRAPH_INSTANCE *graph_param3;
            UI_GRAPH_INSTANCE *graph_param4;
        };
    }graph;
}debug_ui_union;

static void UserDefinedUI_DISPLAY(){
    for(int i = 0; i < 8; i++){
        debug_ui_union.graph.graph_group[i]->delete_call = 0;
        if(debug_ui_union.graph.graph_group[i]->active_flag==0)
            debug_ui_union.graph.graph_group[i]->init_call = 1;
    }
    for(int i = 0; i < 11; i++){
        debug_ui_union.string.string_group[i]->delete_call = 0;
        if(debug_ui_union.string.string_group[i]->active_flag==0)
            debug_ui_union.string.string_group[i]->init_call = 1;
    }
}
static void UserDefinedUI_UNDISPLAY(){
    for(int i = 0; i < 8; i++){
        debug_ui_union.graph.graph_group[i]->init_call = 0;
        if(debug_ui_union.graph.graph_group[i]->active_flag==1)
            debug_ui_union.graph.graph_group[i]->delete_call = 1;
    }
    for(int i = 0; i < 11; i++){
        debug_ui_union.string.string_group[i]->init_call = 0;
        if(debug_ui_union.string.string_group[i]->active_flag==1)
            debug_ui_union.string.string_group[i]->delete_call = 1;
    }
}
void UserDefinedUI_init(){
#if UI_param_data_read_from_flash
    UI_GRAPH_CONFIG_s wrong_config_s;
    memset(&wrong_config_s,0xff,sizeof(UI_GRAPH_CONFIG_s));
    for(int i = 0; i < USER_DEFINED_UI_MAXNUM; i++){
        if(memcmp(&wrong_config_s, (uint8_t*)(UI_param_record_address + i*sizeof(UI_GRAPH_CONFIG_s)), sizeof(UI_GRAPH_CONFIG_s)) == 0)
            memset((uint32_t*)(user_defined_ui_config + i), 0, sizeof(UI_GRAPH_CONFIG_s));
        else
            flash_read(UI_param_record_address + i*sizeof(UI_GRAPH_CONFIG_s), (uint32_t*)(user_defined_ui_config + i), sizeof(UI_GRAPH_CONFIG_s));
    }
#endif
    for(int i = 0; i < USER_DEFINED_UI_MAXNUM; i++){
        user_defined_ui[i] = UI_Graph_Init_byConfig(&user_defined_ui_config[i]);
        // user_defined_ui[i]->init_call = 0;
    }

    debug_ui_union.string.sign = UI_String_Init(0,0,Graphic_Color_Pink,35,3,385,890,"IN DEBUG_MODE");
    debug_ui_union.string.string_id = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,285,UI_DEBUG_STRING_ID);
    debug_ui_union.string.string_type = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,255,UI_DEBUG_STRING_TYPE_Rect);
    debug_ui_union.string.string_width = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,225,UI_DEBUG_STRING_WIDTH);
    debug_ui_union.string.string_pos_x = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,195,UI_DEBUG_STRING_POSX);
    debug_ui_union.string.string_pos_y = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,165,UI_DEBUG_STRING_POSY);
    debug_ui_union.string.string_param1 = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,135,UI_DEBUG_STRING_PARAM1);
    debug_ui_union.string.string_param2 = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,105,UI_DEBUG_STRING_PARAM2);
    debug_ui_union.string.string_param3 = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,75,UI_DEBUG_STRING_PARAM3);
    debug_ui_union.string.string_param4 = UI_String_Init(0,0,Graphic_Color_Pink,15,3,915,45,UI_DEBUG_STRING_PARAM4);

    debug_ui_union.graph.graph_id = UI_Graph_Init(GraphType_Number, 0, 0, Graphic_Color_Pink, 3, 1030, 285, 15, user_defined_ui_idx_pointer);
    debug_ui_union.graph.graph_width = UI_Graph_Init(GraphType_Number, 0, 0, Graphic_Color_Pink, 3, 1030, 225, 15, user_defined_ui[user_defined_ui_idx_pointer]->width);
    debug_ui_union.graph.graph_pos_x = UI_Graph_Init(GraphType_Number, 0, 0, Graphic_Color_Pink, 3, 1030, 195, 15, user_defined_ui[user_defined_ui_idx_pointer]->pos_x);
    debug_ui_union.graph.graph_pos_y = UI_Graph_Init(GraphType_Number, 0, 0, Graphic_Color_Pink, 3, 1030, 165, 15, user_defined_ui[user_defined_ui_idx_pointer]->pos_y);
    debug_ui_union.graph.graph_param1 = UI_Graph_Init(GraphType_Number, 0, 0, Graphic_Color_Pink, 3, 1030, 135, 15, user_defined_ui[user_defined_ui_idx_pointer]->param.param_1);
    debug_ui_union.graph.graph_param2 = UI_Graph_Init(GraphType_Number, 0, 0, Graphic_Color_Pink, 3, 1030, 105, 15, user_defined_ui[user_defined_ui_idx_pointer]->param.param_2);
    debug_ui_union.graph.graph_param3 = UI_Graph_Init(GraphType_Number, 0, 0, Graphic_Color_Pink, 3, 1030, 75, 15, user_defined_ui[user_defined_ui_idx_pointer]->param.param_3);
    debug_ui_union.graph.graph_param4 = UI_Graph_Init(GraphType_Number, 0, 0, Graphic_Color_Pink, 3, 1030, 45, 15, user_defined_ui[user_defined_ui_idx_pointer]->param.param_4);

    UserDefinedUI_UNDISPLAY();
}
static void UserDefinedUIRefresh(){
    UI_StringSwitchDetect_Char(debug_ui_union.string.string_type, 5, user_defined_ui[user_defined_ui_idx_pointer]->type, UI_DEBUG_STRING_TYPE_Rect, UI_DEBUG_STRING_TYPE_Line,UI_DEBUG_STRING_TYPE_Round,UI_DEBUG_STRING_TYPE_Elipse,UI_DEBUG_STRING_TYPE_Arc);
    debug_ui_union.string.string_type->color = user_defined_ui[user_defined_ui_idx_pointer]->color;

    debug_ui_union.graph.graph_id->param.Number.value = user_defined_ui_idx_pointer;
    debug_ui_union.graph.graph_width->param.Number.value = user_defined_ui[user_defined_ui_idx_pointer]->width;
    debug_ui_union.graph.graph_pos_x->param.Number.value = user_defined_ui[user_defined_ui_idx_pointer]->pos_x;
    debug_ui_union.graph.graph_pos_y->param.Number.value = user_defined_ui[user_defined_ui_idx_pointer]->pos_y;
    debug_ui_union.graph.graph_param1->param.Number.value = user_defined_ui[user_defined_ui_idx_pointer]->param.param_1;
    debug_ui_union.graph.graph_param2->param.Number.value = user_defined_ui[user_defined_ui_idx_pointer]->param.param_2;
    debug_ui_union.graph.graph_param3->param.Number.value = user_defined_ui[user_defined_ui_idx_pointer]->param.param_3;
    debug_ui_union.graph.graph_param4->param.Number.value = user_defined_ui[user_defined_ui_idx_pointer]->param.param_4;
}
static void addUI(){
    user_defined_ui[user_defined_ui_idx_pointer]->init_call = 1;
}
static void deleteUI(){
    user_defined_ui[user_defined_ui_idx_pointer]->delete_call = 1;
}
static void switchUIColor(int value){
    user_defined_ui[user_defined_ui_idx_pointer]->color = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->color + value, 8, 0);
}
static void switchUIType(int value){
    user_defined_ui[user_defined_ui_idx_pointer]->type = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->type + value, UI_GrathType_NUM-1, 0);
}
static void switchUIPointer(int value){
    user_defined_ui_idx_pointer = VALUE_BOUND(user_defined_ui_idx_pointer + value, USER_DEFINED_UI_MAXNUM-1, 0);
}
static void moveUI_upORdown(int value){
    user_defined_ui[user_defined_ui_idx_pointer]->pos_y = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->pos_y + value, 1280, 0);
}
static void moveUI_leftORright(int value){
    user_defined_ui[user_defined_ui_idx_pointer]->pos_x = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->pos_x + value, 2120, 0);
}
static void changeUI_width(int value){
    user_defined_ui[user_defined_ui_idx_pointer]->width = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->width + value, 300, 0);
}
static void changeUI_resetCenter(){
    user_defined_ui[user_defined_ui_idx_pointer]->pos_x = 1920/2;
    user_defined_ui[user_defined_ui_idx_pointer]->pos_y = 1080/2;
}
static void changeUI_copyORpaste(uint8_t mode){
    static UI_GRAPH_INSTANCE config = {0};
    switch(mode){
        case 0: //copy
            memcpy((uint8_t*)(&config), (uint8_t*)(user_defined_ui[user_defined_ui_idx_pointer])+sizeof(Graph_Data_t)+3, sizeof(UI_GRAPH_CONFIG_s));
            break;
        case 1: //paste
            memcpy((uint8_t*)(user_defined_ui[user_defined_ui_idx_pointer])+sizeof(Graph_Data_t)+3, (uint8_t*)(&config), sizeof(UI_GRAPH_CONFIG_s));
            break;
        case 2: //cut
            memcpy((uint8_t*)(&config), (uint8_t*)(user_defined_ui[user_defined_ui_idx_pointer])+sizeof(Graph_Data_t)+3, sizeof(UI_GRAPH_CONFIG_s));
            memset((uint8_t*)(user_defined_ui[user_defined_ui_idx_pointer])+sizeof(Graph_Data_t)+3, 0, sizeof(UI_GRAPH_CONFIG_s));

        default:
            break;
    }
}
static void changeUI_param1(int value){
    switch(user_defined_ui[user_defined_ui_idx_pointer]->type){
        case GraphType_Line:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Line.length = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Line.length+value, 2000, 0);
            break;
        case GraphType_Arc: 
            user_defined_ui[user_defined_ui_idx_pointer]->param.Arc.start_angle = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Arc.start_angle+value, 360, 0);
            break;
        case GraphType_Elipse:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Ellipse.radius_x = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Ellipse.radius_x+value, 300, 0);
            break;
        case GraphType_Rect:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Rect.width = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Rect.width+value, 1900, 0);
            break;
        case GraphType_Round:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Round.radius = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Round.radius+value, 300, 0);
            break;
        case GraphType_Ray:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Ray.line_length = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Ray.line_length+value, 2000, 0);
            break;
        default:
            break;
    }
}
static void changeUI_param2(int value){
    switch(user_defined_ui[user_defined_ui_idx_pointer]->type){
        case GraphType_Line:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Line.rotate_angle = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Line.rotate_angle+value, 360, 0);
            break;
        case GraphType_Arc:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Arc.angle_len = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Arc.angle_len+value, 360, 0);
            break;
        case GraphType_Elipse:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Ellipse.radius_y = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Ellipse.radius_y+value, 300, 0);
            break;
        case GraphType_Rect:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Rect.height = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Rect.height+value, 1000, 0);
            break;
        case GraphType_Ray:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Ray.rotate_angle = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Ray.rotate_angle+value, 360, 0);
            break;
        default:
            break;
    }
}
static void changeUI_param3(int value){
    switch(user_defined_ui[user_defined_ui_idx_pointer]->type){
        case GraphType_Arc:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Arc.radius_x = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Arc.radius_x+value, 300, 0);
            break;
        case GraphType_Ray:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Ray.offcenter_lenth = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Ray.offcenter_lenth+value, 2000, 0);
            break;
        default:
            break;
    }
}
static void changeUI_param4(int value){
    switch(user_defined_ui[user_defined_ui_idx_pointer]->type){
        case GraphType_Arc:
            user_defined_ui[user_defined_ui_idx_pointer]->param.Arc.radius_y = VALUE_BOUND(user_defined_ui[user_defined_ui_idx_pointer]->param.Arc.radius_y+value, 300, 0);
            break;
        default:
            break;
    }
}


void UserDefinedUI_operate(UI_debug_param *message){
    switchUIType(message->switch_type);
    switchUIColor(message->switch_color);
    switchUIPointer(message->switch_selected_ui);
    moveUI_upORdown(message->pos_upORdown);
    moveUI_leftORright(message->pos_leftORright);
    changeUI_param1(message->param1);
    changeUI_param2(message->param2);
    changeUI_param3(message->param3);
    changeUI_param4(message->param4);
    changeUI_width(message->width);
    
    if(message->add_ui)
        addUI();
    if(message->delete_ui)
        deleteUI();
    if(message->reset_to_center)
        changeUI_resetCenter();
    if(message->copy)
        changeUI_copyORpaste(0);
    if(message->paste)
        changeUI_copyORpaste(1);
    if(message->cut)
        changeUI_copyORpaste(2);
        
    UserDefinedUIRefresh();
}