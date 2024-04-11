#ifndef GENERAL_DEF_H
#define GENERAL_DEF_H

// 一些module的通用数值型定义,注意条件macro兼容,一些宏可能在math.h中已经定义过了

#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2 1.5707963268f // 2 pi

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec

// #define SAMPLING //用以进行采样时直接对电机输出电流值使用，不使用时请及时关闭

#endif // !GENERAL_DEF_H