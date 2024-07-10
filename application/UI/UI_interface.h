#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "main.h"
#include "referee_protocol.h"
#include "referee_UI.h"
#include "UI_ref.h"
#include "general_def.h"

typedef enum{
    GraphType_Rect = 0,
    GraphType_Line,
    GraphType_Round,
    GraphType_Elipse,
    GraphType_Arc,
    GraphType_Number,
    GraphType_Float
}GraphType_;
typedef enum{
    RefreshMode_ADD = 0,
    RefreshMode_CHANGE,
    RefreshMode_DELETE
}RefreshMode_;
typedef union{
    struct{
        uint32_t width  : 11;
        uint32_t height : 11;

    }Rect;
    struct{
        uint32_t length : 11;
        double rotate_angle;
    }Line;
    struct{
        uint32_t radius : 10;
    }Round;
    struct{
        uint32_t radius_x : 11;
        uint32_t radius_y : 11;
    }Ellipse;
    struct{
        uint32_t radius_x : 11;
        uint32_t radius_y : 11;
        uint32_t start_angle : 9;
        uint32_t angle_len : 9;
    }Arc;
    struct{
        uint32_t size : 9;
        int32_t value;
    }Number;
    struct{
        uint32_t size : 9;
        uint32_t digit: 3;
        double value;
    }Float;
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
    uint8_t init_call : 1;  
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
    uint8_t init_call : 1;  
    int8_t delay : 3;
}UI_STRING_INSTANCE;

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

#endif //UI_INTERFACE_