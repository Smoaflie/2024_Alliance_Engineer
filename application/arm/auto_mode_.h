#include "robot_def.h"

typedef union{
    struct{
        uint8_t bigyaw_offset : 1;
        uint8_t midyaw_offset : 1;
        uint8_t assortedyaw_offset : 1;
        uint8_t assortedroll_offset : 1;
        //
        uint8_t tail_offset : 1;
        uint8_t height_offset : 1;
        uint8_t height_range : 1;
        uint8_t func_deley : 1; //该步骤是否需要延时(步骤延时不可跳过)
        //
        uint8_t arm_pump : 2;   //0不变 1开 2关
        uint8_t valve_pump : 2;
        //
        uint8_t airvalve_state : 3;//夹爪状态 0不变 1前伸 2上抬 3松开 4锁紧
        uint8_t roll_rotate : 1;
    };
    uint16_t setting_total;
}AUTO_MODE_SETTING_;
typedef struct{
    AUTO_MODE_SETTING_ setting;
    uint16_t delay_time;
    float    bigyaw;
    float    midyaw;
    float    assortedyaw;
    float    assortedroll;
    float    tail;
    float    height;
    float    reserved;  //保留段
}AUTO_MODE_STEP_;
typedef struct{
    uint16_t id;
    uint16_t step;
    const AUTO_MODE_STEP_* auto_mode_step;
}ARM_AUTO_MODE_;
struct Arm_Data_s;
struct arm_controller_data_s;

#define AUTO_MODE_IDX 13
#define arm_offset_   0x001F
#define arm_setting   0x0000
#define height_offset_ 0x0020
#define height_setting 0x0000
#define height_range_    0x0040
#define arm_pump_on_    0x0100
#define arm_pump_off_   0x0200
#define linear_pump_on_ 0x0400
#define linear_pump_off_ 0x0800
#define func_delay_     0x0080
#define valve_forward_    0x1000
#define valve_up_   0x2000
#define valve_loose_ 0x3000
#define valve_tighten_ 0x4000
#define roll_rotate_ 0x8000
#define assorted_joint_offset 0x000C

/* 自动模式相关 */
static int32_t arm_height_outtime,arm_joint_outtime,arm_sucker_outtime; //自动模式最大执行时间
static uint16_t auto_mode_step_id[AUTO_MODE_IDX] = {0};//各自动模式所进行到的步骤id
static uint16_t auto_mode_doing_state = 0; //各自动模式进行时标志位 
static uint16_t auto_mode_delay_time; //自动模式步骤延时时间
static arm_controller_data_s arm_auto_mode_data; // 臂臂自动模式目标值

static inline arm_controller_data_s* get_arm_current_data();
static inline uint8_t* get_Arm_goto_target_position_flag();
static inline Arm_Data_s* get_arm_data_send_p();
static void ArmTailRollOffset(int32_t outtime, float offset_angle);
/*
    监视自动操作请求
    param:
        [in]auto_mode 存储自动操作信息的结构体
        [in]selected_mode 选中的模式
    return:[out]code    0:未执行(id不匹配),1:开始执行,2:执行中,3:执行结束
*/
uint8_t MonitorArmAutoRequest(const ARM_AUTO_MODE_* auto_mode, uint16_t selected_mode){
    uint16_t mode_id = auto_mode->id;
    uint16_t selected_mode_id = selected_mode;
    const AUTO_MODE_STEP_* step;
    arm_controller_data_s* arm_current_data_p = get_arm_current_data();
    uint8_t* Arm_goto_target_position_flag_p = get_Arm_goto_target_position_flag();
    Arm_Data_s* arm_data_send_p = get_arm_data_send_p();
    if (mode_id == 0)  return 0;
    if ((selected_mode_id==mode_id) || (auto_mode_doing_state == (0x0001<<mode_id))){
        auto_mode_doing_state = 0;
        auto_mode_doing_state = (0x0001<<mode_id);
        
        uint16_t current_auto_mode_step_id = auto_mode_step_id[mode_id];
        memset(auto_mode_step_id,0,sizeof(auto_mode_step_id));
        auto_mode_step_id[mode_id] = current_auto_mode_step_id;
        step = &auto_mode->auto_mode_step[auto_mode_step_id[mode_id]];
        {   

            //在进行第一步时，先达到指定高度(±30cm)再移动臂
            if(auto_mode_step_id[mode_id] == 0 && fabsf(step->height-arm_current_data_p->height) >= 30 && !(step->setting.setting_total&(height_offset_+height_range_))){
                arm_auto_mode_data.height        = step->setting.height_offset ? arm_current_data_p->height+step->height :   step->height;
                arm_height_outtime = step->delay_time;
                (*Arm_goto_target_position_flag_p) |= Arm_height_ramp_flag;
                return 1;
            }

            arm_auto_mode_data.big_yaw_angle = step->setting.bigyaw_offset ? arm_current_data_p->big_yaw_angle+step->bigyaw : step->bigyaw;
            arm_auto_mode_data.mid_yaw_angle = step->setting.midyaw_offset ? arm_current_data_p->mid_yaw_angle+step->midyaw : step->midyaw;
            arm_auto_mode_data.assorted_yaw_angle = step->setting.assortedyaw_offset ? arm_current_data_p->assorted_yaw_angle+step->assortedyaw : step->assortedyaw;
            arm_auto_mode_data.assorted_roll_angle = step->setting.assortedroll_offset ? arm_current_data_p->assorted_roll_angle+step->assortedroll : step->assortedroll;
            arm_auto_mode_data.tail_motor_angle = step->setting.tail_offset ? arm_current_data_p->tail_motor_angle+step->tail : step->tail;
            if(step->setting.height_range)
                arm_auto_mode_data.height        = 
                    (arm_current_data_p->height <= step->reserved && arm_current_data_p->height >= step->height) ? arm_current_data_p->height: (step->height+step->reserved)/2.0f;
            else
                arm_auto_mode_data.height        = step->setting.height_offset ? arm_current_data_p->height+step->height :   step->height;

            arm_joint_outtime = step->delay_time;
            arm_height_outtime = step->delay_time;
            (*Arm_goto_target_position_flag_p) |= (Arm_height_ramp_flag|Arm_joint_ramp_flag);

            if(step->setting.func_deley) auto_mode_delay_time = step->reserved;
            if(step->setting.arm_pump & 0x01)    airpump_arm_state = 1;
            else if(step->setting.arm_pump & 0x02)   airpump_arm_state = 0;
            if(step->setting.valve_pump & 0x01)  airpump_linear_state = 1;
            else if(step->setting.valve_pump & 0x02) airpump_linear_state = 0;
            if(step->setting.airvalve_state & 0x03) arm_data_send_p->arm_to_airvalve = AIRVALVE_CLAW_LOOSE;
            else if(step->setting.airvalve_state & 0x04) arm_data_send_p->arm_to_airvalve = AIRVALVE_CLAW_TIGHTEN;
            if(step->setting.roll_rotate)    ArmTailRollOffset(step->delay_time, step->reserved);
        }
        auto_mode_step_id[mode_id]++;
        if(auto_mode_step_id[mode_id] == 1 && auto_mode->step != 1)    return 1;
        if(auto_mode_step_id[mode_id] == auto_mode->step){
            auto_mode_doing_state = 0;
            auto_mode_step_id[mode_id] = 0;
            memset(arm_data_send_p,0,sizeof(Arm_Data_s));
            return 3;
        }else return 2;
    }else{
        return 0;
    }
}

typedef struct{
    AUTO_MODE_STEP_ Arm_get_goldcube_right_step[300];
    ARM_AUTO_MODE_ Arm_get_goldcube_right_func;
    AUTO_MODE_STEP_ Arm_fetch_cube_from_warehouse_up_step[100];
    ARM_AUTO_MODE_ Arm_fetch_cube_from_warehouse_up_func;
    AUTO_MODE_STEP_ Arm_fetch_cube_from_warehouse_down_step[100];
    ARM_AUTO_MODE_ Arm_fetch_cube_from_warehouse_down_func;
    AUTO_MODE_STEP_ Arm_get_silvercube_left_step[100];
    ARM_AUTO_MODE_ Arm_get_silvercube_left_func;
    AUTO_MODE_STEP_ Recycle_arm_in_step[70];
    ARM_AUTO_MODE_ Recycle_arm_in_func;
    AUTO_MODE_STEP_ Arm_get_silvercube_mid_step[100];
    ARM_AUTO_MODE_ Arm_get_silvercube_mid_func;
    AUTO_MODE_STEP_ Arm_get_silvercube_right_step[100];
    ARM_AUTO_MODE_ Arm_get_silvercube_right_func;
}ARM_AUTO_MODE_DATA_s;
static ARM_AUTO_MODE_DATA_s ARM_AUTO_MODE_DATA_;

const AUTO_MODE_STEP_ Arm_get_goldcube_right_step[] = {
    {{.setting_total = arm_offset_ | height_range_},  800, 0,      0,      0,      0,      0,    -224.54,   20},
    {{.setting_total = arm_setting | height_offset_}, 4000, -118.019653,80.3512421,44.24086,0,0},
    {{.setting_total = arm_offset_ | height_setting}, 2000, 0,      0,      0,      0,      0,    -492},
    {{.setting_total = arm_setting | height_offset_}, 2000, -122.619492,95.0278549,31.6861496,0, 0},
    {{.setting_total = arm_setting | height_offset_}, 1400,-99.7973938,79.543335,27.8738689,0, 0},
    {{.setting_total = arm_setting | height_offset_ | arm_pump_on_}, 1000,-85.9664917,77.9112091,10.4933853,0, 0,0},
    {{.setting_total = arm_setting | height_offset_ | func_delay_}, 1000,-79.094635,60.0441895,34.6552238,0, 0,    0,  400},
    {{.setting_total = arm_setting | height_offset_ | func_delay_}, 1000,-70.5521851,49.5737457,34.3791924,0, 3,   0,  1300},
    {{.setting_total = arm_setting | height_offset_}, 1000,-79.094635,60.0441895,34.6552238,0, 0, 74},
    {{.setting_total = arm_setting | height_offset_}, 1000,-100.219162,70.8017273,39.0113335,0, 0},
    {{.setting_total = arm_setting | height_offset_}, 1000,-110.731659,79.457222,39.0676384,0, -8},
    {{.setting_total = arm_setting | height_offset_}, 1000,-119.958344,83.6705627,41.3102379,0, 0},
    {{.setting_total = arm_setting | height_offset_}, 1000,-134.377579,79.5574646,63.6332283,0, 0},
    {{.setting_total = arm_setting | height_offset_}, 1000,-138.092636,57.1531143,87.3954391,0, 0},
    {{.setting_total = arm_offset_ | height_setting}, 2000, 0,      0,      0,      0,      0,    0},
    {{.setting_total = arm_setting | height_offset_}, 1000,102.6894,-80.4619751,-83.837616,0, 90.5},
    {{.setting_total = arm_offset_ | height_setting}, 2000, 0,      0,      0,      0,      0,    -91.22},
};
const ARM_AUTO_MODE_ Arm_get_goldcube_right_func = {
    .auto_mode_step = Arm_get_goldcube_right_step,
    .id =  Arm_get_goldcube_right,
    .step = 17
};

//todo z减速比变了，这里还没改
// const AUTO_MODE_STEP_ Arm_fetch_cube_from_warehouse_up_step[] = {
//     {{.setting_total = arm_offset_ | height_range_},  800, 0,      0,      0,      0,      0,    -20,   20},
//     {{.setting_total = arm_setting | height_offset_}, 4000,39.6229019,87.8980637,50.4495468,0, 90.5},
//     {{.setting_total = arm_offset_ | height_setting | func_delay_ | arm_pump_on_ | valve_loose_}, 1000, 0,      0,      0,      0,      0,    -100,   1000},
//     {{.setting_total = arm_offset_ | height_setting}, 1000, 0,      0,      0,      0,      0,    0},
//     {{.setting_total = arm_setting | height_offset_}, 2000,-80.4619751,87.8980637,50.4495468,0, 90.5},
//     {{.setting_total = arm_setting | height_offset_}, 2000,-60,-80.4619751,-83.837616,0, 90.5},
// };
// const ARM_AUTO_MODE_ Arm_fetch_cube_from_warehouse_up_func = {
//     .auto_mode_step = Arm_fetch_cube_from_warehouse_up_step,
//     .id =  Arm_fetch_cube_from_warehouse_up,
//     .step = 6
// };

// const AUTO_MODE_STEP_ Arm_fetch_cube_from_warehouse_down_step[] = {
//     {{.setting_total = arm_offset_ | height_range_},  800, 0,      0,      0,      0,      0,    -80,   20},
//     {{.setting_total = arm_setting | height_offset_}, 2000,-11.9717426,100.782181,34.6771965,0, 90.5},
//     {{.setting_total = arm_offset_ | height_setting | func_delay_ | arm_pump_on_ | linear_pump_off_}, 1000, 0,      0,      0,      0,      0,    -220.332123,   1000},
//     {{.setting_total = arm_offset_ | height_setting}, 600, 0,      0,      0,      0,      0,    0},
//     {{.setting_total = arm_setting | height_offset_}, 2000,-60,-80.4619751,-83.837616,0, 90.5},
// };
// const ARM_AUTO_MODE_ Arm_fetch_cube_from_warehouse_down_func = {
//     .auto_mode_step = Arm_fetch_cube_from_warehouse_down_step,
//     .id =  Arm_fetch_cube_from_warehouse_down,
//     .step = 5
// };

// const AUTO_MODE_STEP_ Arm_get_silvercube_left_step[] = {
//     {{.setting_total = height_range_ | arm_offset_},  4000, 61.0629578,4.94228649,-57.9302979,0,90,    -160,   20},
//     {{.setting_total = arm_offset_ | func_delay_ | arm_pump_on_}, 1000, 0,      0,      0,      0,      0,    -215,     1000},
//     {{.setting_total = arm_offset_}, 1000, 0,      0,      0,      0,      0,    0},
//     {{.setting_total = height_offset_ | roll_rotate_}, 4000,63.0299492,2.84913659,0,0,90,0,12819},
//     {{.setting_total = height_offset_ | valve_forward_}, 2000,-60,-80.4619751,-83.837616,0, 90.5},
//     {{.setting_total = height_offset_}, 2000,42.0428123,45.8032913,85.8326187,0, 90.5},
//     {{.setting_total = height_offset_}, 2000,42.2840767,47.8979836,85.35012054,0, 90.5},
//     {{.setting_total = height_offset_}, 2000,50.9057617,67.0916061,61.8424454,0, 90.5},
//     {{.setting_total = arm_offset_ | valve_up_ | func_delay_}, 1000, 0,      0,      0,      0,      0,    -55,     1000},
//     {{.setting_total = height_offset_ | arm_offset_ | valve_tighten_ | func_delay_}, 1000, 0,      0,      0,      0,      0,    0,     1000},
//     {{.setting_total = height_offset_ | arm_offset_ | arm_pump_off_ | func_delay_}, 1000, 0,      0,      0,      0,      0,    0,     1000},
//     {{.setting_total = height_offset_}, 2000,61.0565071,-26.2618942,-85.1202698,0, 90.5},
//     {{.setting_total = height_offset_}, 4000,61.0565071,-26.2618942,-85.1202698,0, 90.5},
//     {{.setting_total = arm_offset_}, 400, 0,      0,      0,      0,      0,    -150},
// };
// const ARM_AUTO_MODE_ Arm_get_silvercube_left_func = {
//     .auto_mode_step = Arm_fetch_cube_from_warehouse_down_step,
//     .id =  Arm_get_silvercube_left,
//     .step = 14
// };

const AUTO_MODE_STEP_ Arm_ConvertCube_step[] = {
    {{.setting_total = height_offset_ },  2000, -80,      0,      0,      0,      0,    0},
};
const ARM_AUTO_MODE_ Arm_ConvertCube_func = {
    .auto_mode_step = Arm_ConvertCube_step,
    .id =  Arm_ConvertCube,
    .step = 1
};

const AUTO_MODE_STEP_ Arm_fetch_gronded_cube_step[] = {
    {{.setting_total = height_offset_},  1500,-2.65963554,9.99989223,91.1747589,0, 90.5,    0},
};
const ARM_AUTO_MODE_ Arm_fetch_gronded_cube_func = {
    .auto_mode_step = Arm_fetch_gronded_cube_step,
    .id =  Arm_fetch_gronded_cube,
    .step = 1
};

const AUTO_MODE_STEP_ Arm_straighten_step[] = {
    {{.setting_total = height_offset_},  1500,0,0,0,0,0,0},
};
const ARM_AUTO_MODE_ Arm_straighten_func = {
    .auto_mode_step = Arm_straighten_step,
    .id =  Arm_straighten,
    .step = 1
};

const AUTO_MODE_STEP_ Recycle_arm_out_step[] = {
    {{.setting_total = arm_offset_ | height_offset_},  1000,0,0,0,0,0,30},
    {{.setting_total = arm_offset_ | height_offset_},  2000,-19,0,0,0,0,0},
    {{.setting_total = arm_offset_ | height_offset_},  1000,0,0,0,0,0,50},
    {{.setting_total = arm_offset_ | height_offset_},  2000,-20,-60,3,0,0,0},
    {{.setting_total = arm_offset_ | height_offset_},  1000,0,0,0,0,0,300},
};
const ARM_AUTO_MODE_ Recycle_arm_out_func = {
    .auto_mode_step = Recycle_arm_out_step,
    .id =  Recycle_arm_out,
    .step = 5
};

const AUTO_MODE_STEP_ Arm_walk_state_step[] = {
    {{.setting_total = arm_offset_ | height_setting},  2000,0,0,0,0, 0, 0},
    {{.setting_total = height_offset_},  2000,0,-100.68,-58.29,9.847, 90.5, 0},
    {{.setting_total = 0},  2000,0,-84,-80,13, 90.5, -481.277954},
};
const ARM_AUTO_MODE_ Arm_walk_state_func = {
    .auto_mode_step = Arm_walk_state_step,
    .id =  Arm_walk_state,
    .step = 3
};

// const AUTO_MODE_STEP_ Recycle_arm_in_step[] = {
//     {{.setting_total = arm_offset_},  1000,0,   0,  0,  0,  0,  -100},
//     {{.setting_total = height_offset_},  4000,-40.1328545,70,83.1560593,67.1859589,90.5,    0},
//     {{.setting_total = arm_offset_ | height_offset_},  1000,0,   0,  0,  0,  0,  -330},
//     {{.setting_total = height_offset_},  4000,10,44.5,0,0,0,    0},
//     {{.setting_total = arm_offset_ | height_offset_},  1000,0,   0,  0,  0,  0,  -70},
//     {{.setting_total = height_offset_},  4000,10,44.5,0,0,0,    0},
//     // {{.setting_total = arm_offset_},  1000,0,   0,  0,  0,  0,  -570},
// };
// const ARM_AUTO_MODE_ Recycle_arm_in_func = {
//     .auto_mode_step = Recycle_arm_in_step,
//     .id =  Recycle_arm_in,
//     .step = 6
// };
