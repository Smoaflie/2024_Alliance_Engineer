#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "bmi088.h"
#include "cmsis_os.h"
#include "user_lib.h"

#define INS_YAW_ADDRESS_OFFSET   0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET  2
typedef struct
{
    struct {
        float INS_gyro[3];
        float INS_accel[3];
        float INS_mag[3];
        float INS_quat[4];
        float INS_angle[3];
    } INS_data;
    // 欧拉角输出，单位°
    struct {
        float yaw;
        float pitch;
        float roll;
    } output;
    BMI088Instance *BMI088;
    float timing_time; // 任务运行的时间 单位 s
} INS_Instance;

/**
 * @brief 初始化惯导解算系统
 */
INS_Instance *INS_Init(BMI088Instance *bmi088);

/**
 * @brief 此函数放入实时系统中
 */
void INS_Task(void);


#endif
