#include "UI_interface.h"
static UI_GRAPH_INSTANCE* graph_instance_group[UI_Graph_PRIORITY_LEVEL][255] = {NULL};    //存储图形UI指针
static uint8_t graph_idx_priority[UI_Graph_PRIORITY_LEVEL] = {0};
static UI_GRAPH_INSTANCE* graph_instance[255];
static uint8_t graph_idx_total = 0;

static UI_STRING_INSTANCE* string_instance_group[UI_String_PRIORITY_LEVEL][255] = {NULL};    //存储字符UI指针
static uint8_t string_idx_priority[UI_String_PRIORITY_LEVEL] = {0};
static UI_STRING_INSTANCE* string_instance[255];
static uint8_t string_idx_total = 0;



extern referee_info_t *referee_data;

/*
return: 1图形更新 0图形未改变
*/
uint8_t UI_Graph_Setting(UI_GRAPH_INSTANCE *instance, RefreshMode_ mode){
    Graph_Data_t *graph = &instance->graph;
    Graph_Data_t graph_t;
    uint32_t Graph_Operate;
    memset(&graph_t, 0, sizeof(Graph_Data_t));
    switch(mode){
        case RefreshMode_ADD:   Graph_Operate=Graphic_Operate_ADD;break;
        case RefreshMode_CHANGE:Graph_Operate=Graphic_Operate_CHANGE;break;
        case RefreshMode_DELETE:Graph_Operate=Graphic_Operate_DEL;break;
    }
    switch(instance->type){
        case GraphType_Rect:
            uint32_t Rect_end_x = instance->pos_x + instance->param.Rect.width;
            uint32_t Rect_end_y = instance->pos_y + instance->param.Rect.height;
            UIRectangleDraw(&graph_t, instance->graphic_name, Graph_Operate, instance->layer, instance->color, instance->width, instance->pos_x, instance->pos_y, Rect_end_x, Rect_end_y);break;
        case GraphType_Line:
            uint32_t Line_end_x = instance->pos_x - instance->param.Line.length*sinf(instance->param.Line.rotate_angle*DEGREE_2_RAD);
            uint32_t Line_end_y = instance->pos_y + instance->param.Line.length*cosf(instance->param.Line.rotate_angle*DEGREE_2_RAD);
            UILineDraw(&graph_t, instance->graphic_name, Graph_Operate, instance->layer, instance->color, instance->width, instance->pos_x, instance->pos_y, Line_end_x, Line_end_y);break;
        case GraphType_Round:
            UICircleDraw(&graph_t, instance->graphic_name, Graph_Operate, instance->layer, instance->color, instance->width, instance->pos_x, instance->pos_y, instance->param.Round.radius);break;
        case GraphType_Elipse:
            UIOvalDraw(&graph_t, instance->graphic_name, Graph_Operate, instance->layer, instance->color, instance->width, instance->pos_x, instance->pos_y, instance->param.Ellipse.radius_x, instance->param.Ellipse.radius_y);break;
        case GraphType_Arc:
            uint32_t start_angle = instance->param.Arc.start_angle % 360;
            uint32_t end_angle = (instance->param.Arc.start_angle + instance->param.Arc.angle_len)%360;
            UIArcDraw(&graph_t, instance->graphic_name, Graph_Operate, instance->layer, instance->color, start_angle, end_angle, instance->width, instance->pos_x, instance->pos_y, instance->param.Arc.radius_x, instance->param.Arc.radius_y);break;
        case GraphType_Number:
            UIIntDraw(&graph_t, instance->graphic_name, Graph_Operate, instance->layer, instance->color, instance->param.Number.size, instance->width, instance->pos_x, instance->pos_y, instance->param.Number.value);break;
        case GraphType_Float:
            int32_t value = (int32_t)(instance->param.Float.value * 1000) / (10*instance->param.Float.digit==0?1:10*instance->param.Float.digit) * (10*instance->param.Float.digit);
            UIFloatDraw(&graph_t, instance->graphic_name, Graph_Operate, instance->layer, instance->color, instance->param.Float.size, instance->param.Float.digit, instance->width, instance->pos_x, instance->pos_y, value);break;
        case GraphType_Ray:
            uint32_t Ray_start_x = instance->pos_x - instance->param.Ray.offcenter_lenth*sinf(instance->param.Ray.rotate_angle*DEGREE_2_RAD);
            uint32_t Ray_start_y = instance->pos_y + instance->param.Ray.offcenter_lenth*cosf(instance->param.Ray.rotate_angle*DEGREE_2_RAD);
            uint32_t Ray_end_x = instance->pos_x - (instance->param.Ray.offcenter_lenth+instance->param.Ray.line_length)*sinf(instance->param.Ray.rotate_angle*DEGREE_2_RAD);
            uint32_t Ray_end_y = instance->pos_y + (instance->param.Ray.offcenter_lenth+instance->param.Ray.line_length)*cosf(instance->param.Ray.rotate_angle*DEGREE_2_RAD);
            UILineDraw(&graph_t, instance->graphic_name, Graph_Operate, instance->layer, instance->color, instance->width, Ray_start_x, Ray_start_y, Ray_end_x, Ray_end_y);break;
    }

    if(memcmp(&graph_t, graph, sizeof(Graph_Data_t)) != 0){
        memcpy(graph, &graph_t, sizeof(Graph_Data_t));
        return 1;
    }
    return 0;
}

/*
return: 1字符更新 0字符未改变
*/
uint8_t UI_String_Setting(UI_STRING_INSTANCE *instance, RefreshMode_ mode){
    String_Data_t *string_ = &instance->string;
    String_Data_t string_t;
    uint32_t Graph_Operate;
    memset(&string_t, 0, sizeof(String_Data_t));

    switch(mode){
        case RefreshMode_ADD:   Graph_Operate=Graphic_Operate_ADD;break;
        case RefreshMode_CHANGE:Graph_Operate=Graphic_Operate_CHANGE;break;
        case RefreshMode_DELETE:Graph_Operate=Graphic_Operate_DEL;break;
    }

    UICharDraw(&string_t,instance->graphic_name,Graph_Operate,instance->layer,instance->color,instance->size,instance->width,instance->pos_x,instance->pos_y,instance->char_p);

    if(memcmp(&string_t, string_, sizeof(String_Data_t)) != 0){
        memcpy(string_, &string_t, sizeof(String_Data_t));
        return 1;
    }
    return 0;
}

UI_GRAPH_INSTANCE* UI_Graph_Init_byConfig(UI_GRAPH_CONFIG_s *config){
    UI_GRAPH_INSTANCE *instance = (UI_GRAPH_INSTANCE *)malloc(sizeof(UI_GRAPH_INSTANCE));
    memset(instance, 0, sizeof(UI_GRAPH_INSTANCE));

    instance->type  = config->type;
    instance->color = config->color;
    instance->width = config->width;
    instance->layer = config->layer;
    instance->priority = config->priority;
    instance->param = config->param;
    instance->pos_x = config->pos_x;
    instance->pos_y = config->pos_y;

    instance->graphic_name[0] = 1 + instance->priority;
    instance->graphic_name[1] = 1 + graph_idx_priority[instance->priority];
    graph_instance[graph_idx_total++] = graph_instance_group[instance->priority][graph_idx_priority[instance->priority]++] = instance;

    instance->init_call = 1;

    return instance;
}
UI_GRAPH_INSTANCE* UI_Graph_Init(
        GraphType_ type,
        uint32_t priority,
        uint32_t layer,
        uint32_t color,
	    uint32_t width,
        uint32_t pos_x,
        uint32_t pos_y,
        ...
        ){
    UI_GRAPH_CONFIG_s config;
        config.type = type;
        config.priority = priority;
        config.layer = layer;
        config.color = color;
        config.width = width;
        config.pos_x = pos_x;
        config.pos_y = pos_y;

    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, pos_y); // 初始化 va_list 变量为一个参数列表
    switch(config.type){
        case GraphType_Rect:
            config.param.Rect.width = va_arg(ap, int32_t);
            config.param.Rect.height = va_arg(ap, int32_t);
            break;
        case GraphType_Line:
            config.param.Line.length = va_arg(ap, int32_t);
            config.param.Line.rotate_angle = va_arg(ap, int32_t);
            break;
        case GraphType_Round:
            config.param.Round.radius = va_arg(ap, int32_t);
            break;
        case GraphType_Elipse:
            config.param.Ellipse.radius_x = va_arg(ap, int32_t);
            config.param.Ellipse.radius_y = va_arg(ap, int32_t);
            break;
        case GraphType_Arc:
            config.param.Arc.start_angle = va_arg(ap, int32_t);
            config.param.Arc.angle_len = va_arg(ap, int32_t);
            config.param.Arc.radius_x = va_arg(ap, int32_t);
            config.param.Arc.radius_y = va_arg(ap, int32_t);
            break;
        case GraphType_Ray:
            config.param.Ray.rotate_angle = va_arg(ap, int32_t);
            config.param.Ray.unused_param2 = va_arg(ap, int32_t);
            config.param.Ray.line_length = va_arg(ap, int32_t);
            config.param.Ray.offcenter_lenth = va_arg(ap, int32_t);
            break;
        case GraphType_Number:
            config.param.Number.size = va_arg(ap, int32_t);
            config.param.Number.value = va_arg(ap, int32_t);
            break;
        case GraphType_Float:
            config.param.Float.size = va_arg(ap, int32_t);
            config.param.Float.digit = va_arg(ap, int32_t);
            config.param.Float.value = va_arg(ap, double);
            break;
        
    }
    va_end(ap);

    return UI_Graph_Init_byConfig(&config);
}

UI_STRING_INSTANCE* UI_String_Init(
        uint32_t priority,
        uint32_t layer,
        uint32_t color,
        uint32_t size,
	    uint32_t width,
        uint32_t pos_x,
        uint32_t pos_y,
        const char* char_p
        ){
    UI_STRING_INSTANCE *instance = (UI_STRING_INSTANCE *)malloc(sizeof(UI_STRING_INSTANCE));
    memset(instance, 0, sizeof(UI_STRING_INSTANCE));

    instance->color = color;
    instance->width = width;
    instance->layer = layer;
    instance->priority = priority;
    instance->pos_x = pos_x;
    instance->pos_y = pos_y;
    instance->size = size;
    instance->char_p = char_p;

    instance->graphic_name[0] = 'a' + instance->priority;
    instance->graphic_name[1] = 1 + string_idx_priority[instance->priority];
    string_instance[string_idx_total++] = string_instance_group[instance->priority][string_idx_priority[instance->priority]++] = instance;

    instance->init_call = 1;

    return instance;
}


void UI_Graph_Refresh(){
    //遍历所有UI字符串实例
    for(int i = 0; i<graph_idx_total; i++){
        graph_instance[i]->refresh_call = 1;
    }
}
void UI_Graph_Update(){
    //按优先级遍历所有UI图形实例，填入发送缓冲区
    Graph_Data_t UI_Graph_send_buf[7] = {0};
    static uint8_t UI_Graph_send_buf_cnt = 0; //缓冲区内已有图形数
    static uint8_t idle_refresh_idx = 0;
    for(int i = UI_Graph_PRIORITY_LEVEL-1; i >= 0; i--){
        for(int j = 0; j < graph_idx_priority[i]; j++){

            if(graph_instance_group[i][j]->init_call && !graph_instance_group[i][j]->active_flag){
                graph_instance_group[i][j]->init_call = 0;
                graph_instance_group[i][j]->delete_call = 0;
                UI_Graph_Setting(graph_instance_group[i][j], RefreshMode_ADD);
                graph_instance_group[i][j]->active_flag = 1;
                graph_instance_group[i][j]->send_flag = 1;
            }else if(graph_instance_group[i][j]->delete_call && graph_instance_group[i][j]->active_flag){
                graph_instance_group[i][j]->init_call = 0;
                graph_instance_group[i][j]->delete_call = 0;
                UI_Graph_Setting(graph_instance_group[i][j], RefreshMode_DELETE);
                graph_instance_group[i][j]->active_flag = 0;
                graph_instance_group[i][j]->send_flag = 1;
            }else if(graph_instance_group[i][j]->refresh_call){
                graph_instance_group[i][j]->refresh_call = 0;
                if(UI_Graph_Setting(graph_instance_group[i][j], graph_instance_group[i][j]->active_flag?RefreshMode_ADD:RefreshMode_DELETE))
                    graph_instance_group[i][j]->send_flag = 1;
            }else if(graph_instance_group[i][j]->active_flag){
                if(UI_Graph_Setting(graph_instance_group[i][j], RefreshMode_CHANGE))    graph_instance_group[i][j]->send_flag = 1;
            }

            if(graph_instance_group[i][j]->send_flag && --graph_instance_group[i][j]->delay<=0){
                graph_instance_group[i][j]->send_flag = 0;
                graph_instance_group[i][j]->delay = UI_Graph_Update_freq;
                UI_Graph_send_buf[UI_Graph_send_buf_cnt++] = graph_instance_group[i][j]->graph;
                if(UI_Graph_send_buf_cnt==7){
                    UI_Graph_send_buf_cnt = 0;
                    UIGraphRefresh(&referee_data->referee_id, 7, UI_Graph_send_buf[0], UI_Graph_send_buf[1], UI_Graph_send_buf[2], UI_Graph_send_buf[3], UI_Graph_send_buf[4], UI_Graph_send_buf[5], UI_Graph_send_buf[6]);
                    return;
                }
            }

        }
    }
    while(UI_Graph_send_buf_cnt < 7){
        UI_Graph_Setting(graph_instance[idle_refresh_idx], graph_instance[idle_refresh_idx]->active_flag?RefreshMode_ADD:RefreshMode_DELETE);
        UI_Graph_send_buf[UI_Graph_send_buf_cnt++] = graph_instance[idle_refresh_idx]->graph;
        idle_refresh_idx++;idle_refresh_idx%=graph_idx_total;
    }
    UI_Graph_send_buf_cnt = 0;
    UIGraphRefresh(&referee_data->referee_id, 7, UI_Graph_send_buf[0], UI_Graph_send_buf[1], UI_Graph_send_buf[2], UI_Graph_send_buf[3], UI_Graph_send_buf[4], UI_Graph_send_buf[5], UI_Graph_send_buf[6]);
}

void UI_String_Refresh(){
    //遍历所有UI字符串实例
    for(int i = 0; i<string_idx_total; i++){
        string_instance[i]->refresh_call = 1;
    }
}
void UI_String_Update(){
    //按优先级遍历所有UI字符串实例
    for(int i = UI_String_PRIORITY_LEVEL-1; i >= 0; i--){
        for(int j = 0; j < string_idx_priority[i]; j++){    
            if(string_instance_group[i][j]->init_call && !string_instance_group[i][j]->active_flag){
                string_instance_group[i][j]->init_call = 0;
                UI_String_Setting(string_instance_group[i][j], RefreshMode_ADD);
                string_instance_group[i][j]->active_flag = 1;
                string_instance_group[i][j]->send_flag = 1;
            }else if(string_instance_group[i][j]->delete_call && string_instance_group[i][j]->active_flag){
                string_instance_group[i][j]->delete_call = 0;
                UI_String_Setting(string_instance_group[i][j], RefreshMode_DELETE);
                string_instance_group[i][j]->active_flag = 0;
                string_instance_group[i][j]->send_flag = 1;
            }else if(string_instance_group[i][j]->refresh_call){
                string_instance_group[i][j]->refresh_call = 0;
                if(UI_String_Setting(string_instance_group[i][j], string_instance_group[i][j]->active_flag?RefreshMode_ADD:RefreshMode_DELETE))
                    string_instance_group[i][j]->send_flag = 1;
            }else if(string_instance_group[i][j]->active_flag){
                if(UI_String_Setting(string_instance_group[i][j], RefreshMode_CHANGE))
                    string_instance_group[i][j]->send_flag = 1;
            }

            if(string_instance_group[i][j]->send_flag && --string_instance_group[i][j]->delay<=0){
                string_instance_group[i][j]->send_flag = 0;
                string_instance_group[i][j]->delay = UI_String_Update_freq;
                UICharRefresh(&referee_data->referee_id, string_instance_group[i][j]->string);
                return;
            }
        }
    }
}


void UI_StateSwitchDetect_Graph(UI_GRAPH_INSTANCE* instance, uint8_t cnt, int flag, ...){
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
void UI_ColorSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...){
   static int idx[8];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, int);
    va_end(ap);
    
    if(instance->string.Graph_Control.color != idx[flag]){
        instance->color = idx[flag];
    }
}
void UI_WidthSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...){
   static int idx[8];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap, int);
    va_end(ap);
    
    if(instance->string.Graph_Control.width != idx[flag]){
        instance->width = idx[flag];
    }
}
void UI_StringSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...){
   static const char* idx[32];
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, flag); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++)    idx[i] = va_arg(ap,const char*);
    va_end(ap);
    
    if(strcmp(instance->string.show_Data, idx[flag])){
        instance->char_p = idx[flag];
    }
}
void UI_BatchEnable_Graph(uint8_t cnt, ...){
    UI_GRAPH_INSTANCE* graph;
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, cnt); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++){
        graph = va_arg(ap,UI_GRAPH_INSTANCE*);
        graph->init_call = 1;
        graph->delete_call = 0;
    }
    va_end(ap);
}
void UI_DisableAll_Graph(){
    for(int i=0; i<graph_idx_total; i++){
        graph_instance[i]->init_call = 0;
        graph_instance[i]->delete_call = 1;
    }
}
void UI_BatchDisable_Graph(uint8_t cnt, ...){
    UI_GRAPH_INSTANCE* graph;
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, cnt); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++){
        graph = va_arg(ap,UI_GRAPH_INSTANCE*);
        graph->init_call = 0;
        graph->delete_call = 1;
    }
    va_end(ap);
}
void UI_DisableAll_String(){
    for(int i=0; i<string_idx_total; i++){
        string_instance[i]->init_call = 0;
        string_instance[i]->delete_call = 1;
    }
}
void UI_BatchDisable_String(uint8_t cnt, ...){
    UI_STRING_INSTANCE* string_;
    va_list ap;		   // 创建一个 va_list 类型变量
	va_start(ap, cnt); // 初始化 va_list 变量为一个参数列表
    for(int i = 0; i < cnt; i++){
        string_ = va_arg(ap,UI_STRING_INSTANCE*);
        string_->init_call = 0;
        string_->delete_call = 1;
    }
    va_end(ap);
}