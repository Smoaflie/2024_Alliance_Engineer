/**
 ******************************************************************************
 * @file	 user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "stdlib.h"
#include "memory.h"
#include "user_lib.h"
#include "math.h"
#include "main.h"
#include "bsp_dwt.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

void *zmalloc(size_t size)
{
    void *ptr = malloc(size);
    memset(ptr, 0, size);
    return ptr;
}

// 快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0) {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
    if (num > Limit) {
        num = Limit;
    } else if (num < -Limit) {
        num = -Limit;
    }
    return num;
}

// 判断符号位
float sign(float value)
{
    if (value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue) {
        Value = 0.0f;
    }
    return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        float len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        float len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - (float)integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

// 三维向量归一化
float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

// 计算模长
float NormOf3d(float *v)
{
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float *buf, uint8_t len)
{
    float sum = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        buf[i] = buf[i + 1];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    sum += new_data;
    return sum / len;
}

void MatInit(mat *m, uint8_t row, uint8_t col)
{
    m->numCols = col;
    m->numRows = row;
    m->pData   = (float *)zmalloc(row * col * sizeof(float));
}
/**
 * @brief :  正弦扫频生成器
 * @param[in] F_start 起始频率
 * @param[in] F_end 终止频率
 * @param[in] repeat_time 周期重复次数
 * @param[out] *SE_signal 结束生成标志
 * @param[out] *F_out 当前频率
 * @return 正弦值（0~1）
 */
float sin_signal_generate(float F_start, float F_end, float repeat_time, uint8_t *SE_signal, float *F_out)
{
    static float F = 0;
    if (F == 0) F = F_start;
    static float lasttime = 0;
    // 频率超过限定，返回0
    if (F > F_end) {
        *SE_signal = 0;
        return 0;
    }
    // 保证sin初值为0
    if (lasttime == 0) lasttime = DWT_GetTimeline_s();

    float nowtime = DWT_GetTimeline_s();
    *F_out        = F;
    // 计算正弦值
    float cnt = arm_sin_f32(2 * PI * F * (nowtime - lasttime));
    // 频率递增
    if (nowtime - lasttime > ((1 / F) * repeat_time)) {
        if (F < 24)
            F += 0.5f;
        else if (F >= 24 && F <= 120)
            F += 2;
        else
            F += 4;

        lasttime = DWT_GetTimeline_s();
    }
    return cnt;
}
