#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "main.h"
#include "referee_protocol.h"
#include "referee_UI.h"
#include "UI_ref.h"
#include "general_def.h"

#define UI_String_Refresh_freq 1
#define UI_Graph_Refresh_freq 1
#define UI_Graph_PRIORITY_LEVEL 4
#define UI_String_PRIORITY_LEVEL 1
#define UI_GrathType_NUM 8
#pragma pack(1) 

typedef enum{
    GraphType_Rect = 0,
    GraphType_Line,
    GraphType_Round,
    GraphType_Elipse,
    GraphType_Arc,
    GraphType_Ray,
    GraphType_Number,
    GraphType_Float,
}GraphType_;
typedef enum{
    RefreshMode_ADD = 0,
    RefreshMode_CHANGE,
    RefreshMode_DELETE
}RefreshMode_;
typedef union{
    struct{
        int32_t unused_param1;
        int32_t unused_param2;
        int32_t width ;
        int32_t height;
    }Rect;
    struct{
        int32_t unused_param1;
        int32_t unused_param2;
        int32_t length;
        int32_t rotate_angle;
    }Line;
    struct{
        int32_t rotate_angle;
        int32_t unused_param2;
        int32_t line_length;
        int32_t offcenter_lenth;
    }Ray;
    struct{
        int32_t unused_param1;
        int32_t unused_param2;
        int32_t radius;
        int32_t unused_param4;
    }Round;
    struct{
        int32_t unused_param1;
        int32_t unused_param2;
        int32_t radius_x;
        int32_t radius_y;
    }Ellipse;
    struct{
        int32_t start_angle;
        int32_t angle_len;
        int32_t radius_x;
        int32_t radius_y;
    }Arc;
    struct{
        int32_t size;
        int32_t unused_param2;
        int32_t unused_param3;
        int32_t unused_param4;
        int32_t value;
    }Number;
    struct{
        int32_t size;
        int32_t digit;
        int32_t unused_param3;
        int32_t unused_param4;
        double value;
    }Float;
    struct{
        int32_t param_1;
        int32_t param_2;
        int32_t param_3;
        int32_t param_4;
    };
    
}UI_GRAPH_PARAM_;

typedef struct{
    Graph_Data_t graph;

	char graphic_name[3];
    
    GraphType_ type : 3;
    
    uint32_t priority : 3;
    uint32_t layer : 4;
    uint32_t pos_x : 11;
    uint32_t pos_y : 11;
    uint32_t color : 4;
	uint32_t width : 10;
    UI_GRAPH_PARAM_ param;

    uint8_t send_flag : 1;
    uint8_t active_flag : 1;  
    uint8_t init_call : 1;  
    uint8_t delete_call : 1;  
    int8_t delay : 3;  
}UI_GRAPH_INSTANCE;
typedef struct{
    GraphType_ type : 3;

    uint32_t priority : 3;
    uint32_t layer : 4;
    uint32_t pos_x : 11;
    uint32_t pos_y : 11;
    uint32_t color : 4;
	uint32_t width : 10;
    UI_GRAPH_PARAM_ param;
}UI_GRAPH_CONFIG_s;

typedef struct{
    String_Data_t string;

	char graphic_name[3];
    uint32_t priority : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t size : 9;
	uint32_t width : 10;
    uint32_t pos_x : 11;
    uint32_t pos_y : 11;
    
    const char* char_p;

    uint8_t send_flag : 1;
    uint8_t active_flag : 1;  
    uint8_t init_call : 1;  
    uint8_t delete_call : 1;  
    int8_t delay : 3;
}UI_STRING_INSTANCE;

#pragma pack() 

UI_GRAPH_INSTANCE* UI_Graph_Init_byConfig(UI_GRAPH_CONFIG_s *config);
UI_GRAPH_INSTANCE* UI_Graph_Init(
        GraphType_ type,
        uint32_t priority,
        uint32_t layer,
        uint32_t color,
	    uint32_t width,
        uint32_t pos_x,
        uint32_t pos_y,
        ...
        );
UI_STRING_INSTANCE* UI_String_Init(
        uint32_t priority,
        uint32_t layer,
        uint32_t color,
        uint32_t size,
	    uint32_t width,
        uint32_t pos_x,
        uint32_t pos_y,
        const char* char_p
        );
uint8_t UI_Graph_Setting(UI_GRAPH_INSTANCE *instance, RefreshMode_ mode);
void UI_Graph_Refresh();
void UI_String_Refresh();

void UI_StateSwitchDetect_Graph(UI_GRAPH_INSTANCE* instance, uint8_t cnt, int flag, ...);
void UI_BatchEnable_Graph(uint8_t cnt, ...);
void UI_BatchDisable_Graph(uint8_t cnt, ...);

void UI_ColorSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...);
void UI_WidthSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...);
void UI_StringSwitchDetect_Char(UI_STRING_INSTANCE *instance, uint8_t cnt, int flag, ...);
void UI_BatchDisable_String(uint8_t cnt, ...);
#endif //UI_INTERFACE_