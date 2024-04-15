#include "tool.h"

ramp_t fb_ramp     = RAMP_GEN_DAFAULT;
ramp_t lr_ramp     = RAMP_GEN_DAFAULT;
ramp_t rotate_ramp = RAMP_GEN_DAFAULT;
ramp_t slow_ramp   = RAMP_GEN_DAFAULT;
ramp_t close_ramp  = RAMP_GEN_DAFAULT;

int heat_control    = 20; // 热量控制
float heat_remain   = 0;  // 剩余热量
float local_heat    = 0;  // 本地热量
int One_bullet_heat = 10; // 打一发消耗热量

int Trig_time = 0; // 发射触发时间

void ramp_init(ramp_t *ramp, int32_t scale)
{
    ramp->count = 0;
    ramp->scale = scale;
}

float ramp_calc(ramp_t *ramp)
{
    if (ramp->scale <= 0)
        return 0;

    if (ramp->count++ >= ramp->scale)
        ramp->count = ramp->scale;

    ramp->out = ramp->count / ((float)ramp->scale);
    return ramp->out;
}

