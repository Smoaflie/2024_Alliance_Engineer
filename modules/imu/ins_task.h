#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "bmi088.h"
#include "cmsis_os.h"
#include "user_lib.h"

// #define IMU_DEF_PARAM_WARNING
// 编译warning,提醒开发者修改传感器参数
#ifndef IMU_DEF_PARAM_WARNING
#define IMU_DEF_PARAM_WARNING
#endif // !IMU_DEF_PARAM_WARNING

// 陀螺仪默认环境温度
#define BMI088_AMBIENT_TEMPERATURE 25.0f
// 设置陀螺仪数据相较于云台的yaw,pitch,roll的方向
#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f},                  \
        {0.0f, -1.0f, 0.0f},             \
    {                                    \
        0.0f, 0.0f, -1.0f                \
    }

#define INS_YAW_ADDRESS_OFFSET   2  // 陀螺仪数据相较于云台的yaw的方向
#define INS_PITCH_ADDRESS_OFFSET 1  // 陀螺仪数据相较于云台的pitch的方向
#define INS_ROLL_ADDRESS_OFFSET  0  // 陀螺仪数据相较于云台的roll的方向

// 陀螺仪校准数据，开启陀螺仪校准后可从INS中获取
#define BMI088_PRE_CALI_GYRO_X_OFFSET -0.000267979165f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET 0.000386821659f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.0041627204f

typedef struct
{
    struct {
        float INS_gyro[3];
        float INS_accel[3];
        float INS_mag[3];
        float INS_quat[4];
    } INS_data;

    struct {
        float INS_angle[3];        // 弧度制欧拉角
        float Yaw_total_angle;     // 云台总偏转角度
        float INS_angle_deg[3];    // 欧拉角输出，单位°
        float Yaw_total_angle_deg; // 云台总偏转角度，单位°
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
