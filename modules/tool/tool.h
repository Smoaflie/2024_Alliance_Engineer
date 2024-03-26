#pragma once

#include "main.h"
#include "rm_referee.h"

#define bool  _Bool
#define true  1
#define false 0

#define RAMP_GEN_DAFAULT \
    {                    \
        0,               \
            0,           \
            0,           \
    }

#define RAMP_TIME 3000

// 斜坡类型，计算WASD移动映射在底盘的速度
typedef struct ramp_t
{
  int32_t count; //计数值
  int32_t scale; //规模
  float   out; //输出
}ramp_t;

extern referee_info_t referee_info; // 裁判系统数据

extern int heat_control ;    // 热量控制
extern float heat_remain ;    // 剩余热量
extern float local_heat ;     // 本地热量
extern int One_bullet_heat ; // 打一发消耗热量

extern ramp_t fb_ramp;
extern ramp_t lr_ramp;
extern ramp_t rotate_ramp;
extern ramp_t slow_ramp;
extern ramp_t close_ramp;

void ramp_init(ramp_t *ramp, int32_t scale);

float ramp_calc(ramp_t *ramp);

extern int Trig_time;

