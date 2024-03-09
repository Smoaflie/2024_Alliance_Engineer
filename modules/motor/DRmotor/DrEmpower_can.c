/**************************************************
大然机器人-智能一体化关节函数库

适用平台：STM32平台
库版本号：v2.1
测试主控版本：STM32f103c8t6
*************************************************/
#include "DrEmpower_can.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32F1xx.h"
#include <stdio.h>
#include "can.h"

#define ENABLE_INPUT_VALIDITY_CHECK 1   /* 输入合法性检查。为 0 时不编译空指针判断等检查，可减小程序体积、加快处理速度。 */

#define SERVO_MALLOC(size) malloc(size) /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_FREE(ptr) free(ptr)       /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_DELAY(n) HAL_Delay(n)     /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_SPRINTF sprintf           /* 使用这个函数需要在 Keil 里勾选 Use MicroLIB 或在 STM32CubeIDE 里勾选 Use float with printf from newlib-nano */

char command[64];       // 命令发送缓冲区
char DaTemp[4];
uint8_t i;
uint8_t rx_buffer[8];
uint16_t can_id = 0x00;
int8_t READ_FLAG = 0;  // 读取结果标志位

int8_t enable_replay_state = 1;  //如需要打开运动控制指令实时状态返回功能，请将该变量改为1，并将下面的MOTOR_NUM设置为总线上的最大关节ID号

//"""
//内部辅助函数，用户无需使用
//"""

// CAN发送函数
void send_command(uint8_t id_num, char cmd, unsigned char *data,uint8_t rt )
{
    short id_list = (id_num << 5) + cmd;
    Can_Send_Msg(id_list, 8, data);
}

void SERVO_DELAY_US(uint8_t tick)
{
    for(uint8_t t = 0; t<tick; t++);
}

//CAN接收函数
void receive_data(void)
{
    uint8_t OutTime_mark=0;
    uint32_t  OutTime=0;
    while(OutTime_mark==0)
    {
        if( READ_FLAG ==1) OutTime_mark=1;
        SERVO_DELAY_US(1);
        OutTime++;
        if( OutTime ==0xfffe) OutTime_mark=1;
    }
}


// 数据格式转换函数，decode是将二进制(bytes)转化成人看的懂得数据，encode反之
/*
float，short，unsigned short，int，unsigned int五种数据类型与byte类型的转换
五种数据对应名：0,1,2,3,4;
使用方法，在函数中调用时：
    首先要在调用前对结构体内的数据进行赋值
    value_data[3]：若将五种类型的数据转换为byte，则赋值
    byte_data[8]：若将byte转换为五种数据的类型，则赋值
    type_data[3]类型名赋值（必要）
    length：数据个数赋值（必要）

若将五种类型的数据转换为byte，调用format_data(data_struct data_list , char * str)，
    参数为：结构体指针，要做的操作（输入“encode”）
若将byte转换为五种数据的类型，调用format_data(data_struct data_list , char * str)，
    参数为：结构体指针，要做的操作（输入“decode”）

下面两个函数在上述两个函数中自动调用，不需要使用者主动调用
    byte2value()
    value2byte()

type类型：
type_data=0, 浮点数（float）,数据长度32位,符号‘f’;
type_data=1, 无符号短整数（unsigned short int）,数据长度16位,符号‘u16’;
type_data=2, 有符号短整数（short int）,数据长度16位,符号‘s16’;
type_data=3, 无符号短整数（unsigned int）,数据长度32位,符号‘u32’;
type_data=4, 有符号短整数（int）,数据长度32位,符号‘s32’;

*/
struct format_data_struct
{
    float value_data[4];//若将五种类型的数据转换为byte，则赋值
    unsigned char byte_data[8];//若将byte转换为五种数据的类型，则赋值
    int type_data[4];//类型名赋值（必要）
    int length;//数据个数赋值（必要）
}*data_struct,data_list;  //定义全局变量 data_list，用于进行CAN数据encode和decode过程中存储参数输入及结果输出

// note: the bit order is in accordance with intel little endian
static inline void uint16_to_data(uint16_t val, uint8_t *data)
{
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint16_t data_to_uint16(uint8_t *data)
{
    uint16_t tmp_uint16;
    tmp_uint16 = (((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint16;
}
static inline void uint_to_data(uint32_t val, uint8_t *data)
{
    data[3] = (uint8_t)(val >> 24);
    data[2] = (uint8_t)(val >> 16);
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint32_t data_to_uint(uint8_t *data)
{
    uint32_t tmp_uint;
    tmp_uint = (((uint32_t)data[3] << 24) + ((uint32_t)data[2] << 16) + ((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint;
}
static inline void int16_to_data(int16_t val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
}
static inline int16_t data_to_int16(uint8_t *data)
{
    int16_t tmp_int16;
    *(((uint8_t*)(&tmp_int16)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int16)) + 1) = data[1];
    return tmp_int16;
}
static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline int data_to_int(uint8_t *data)
{
    int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}
static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

//不需主动调用
void byte2value()
{
    int value_index = 0;
    int byte_index = 0;
    while (1)
    {
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            data_list.value_data[value_index] = (float)data_to_float(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 1:
        {
            data_list.value_data[value_index] = (float)data_to_uint16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 2:
        {
            data_list.value_data[value_index] = (float)data_to_int16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 3:
        {
            data_list.value_data[value_index] = (float)data_to_uint(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 4:
        {
            data_list.value_data[value_index] = (float)data_to_int(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index>=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}
//不需主动调用
void value2byte()
{
    int byte_index=0;
    int value_index=0;
    while(1)
    {
        if (data_list.type_data[value_index]==0)
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 1:
        {
            uint16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 2:
        {
            int16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 3:
        {
            uint_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 4:
        {
            int_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index >=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}

//参数结构体指针，要做的操作（五种type转byte输入“encode”，byte转五种type输入“decode”）
void format_data( float *value_data, int *type_data,int length, char * str)
{
    data_list.length=length;
    for (int i = 0; i < length; i++)
    {
        data_list.value_data[i]= value_data[i];
        data_list.type_data[i] = type_data[i];
    }
    if (strcmp(str,"encode")==0)
    {
        value2byte();
    }
    if (strcmp(str,"decode")==0)
    {
        for (int i = 0; i < 8; i++)
        {
            data_list.byte_data[i]=rx_buffer[i];
        }
        byte2value();
    }
}

/**
 * @brief 单个关节角度控制函数。
 * 控制指定关节编号的关节按照指定的速度转动到指定的角度（绝对角度，相对于关节零点位置）。
 *
 * @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param angle 关节角度（-360~360）*n，支持大角度转动
 * @param t mode=0,无作用，直接给0即可; mode=1, 运动时间（s）; mode =2, 前馈速度（r/min)
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
 * @param mode 角度控制模式选择，关节支持三种角度控制模式，
 *             mode = 0: 多个关节轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 多个关节梯形轨迹模式，此时speed用运动时间t（s）表示，param为目标加速度（(r/min)/s）。
 *             mode = 2: 前馈控制模式，这种模式下的t为前馈速度，param为前馈扭矩。前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @note 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0报错并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
         另外如果这种模式下accel=0，关节以最快速度运动到angle,speed参数不再其作用。
 */
void preset_angle(uint8_t id_num, float angle, float t, float param, int mode)
{
    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_time = (int)(fabs(t) / factor);
        if (param > 300)
            param = 300;
        int  s16_width = (int)(fabs(param / factor));

        float value_data[3]= {f_angle, s16_time, s16_width};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    else if(mode == 1)
    {
        float f_angle = angle;
        int s16_time = (int)(fabs(t) / factor);
        int s16_accel = (int)((fabs(param)) / factor);

        float value_data[3]= {f_angle,s16_time,s16_accel};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    else if(mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((t) / factor);
        int s16_torque_ff = (int)((param) / factor);

        float value_data[3]= {f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
}
/**
 * @brief 单个关节速度预设函数。
 * 预设指定关节编号的关节的目标速度，之后需要用mv指令启动转动。
 *
 * @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param speed 目标速度（r/min）
 * @param param mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
 * @param mode 控制模式选择
 *             mode=1, 速度前馈控制模式，关节将目标速度直接设为speed
 *             mode!=1,速度爬升控制模式，关节将按照目标加速度变化到speed。
 */
void preset_speed(uint8_t id_num, float speed, float param, int mode)
{
    float factor = 0.01;
    float f_speed = speed;
    if (mode == 0)
    {
        int  s16_torque = (int)((param) / factor);
        if (f_speed == 0)
            s16_torque = 0;
        int s16_input_mode = (int)(1 / factor);

        float value_data[3]= {f_speed, s16_torque, s16_input_mode};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
    }
    else
    {
        int s16_ramp_rate = (int)((param) / factor);
        int s16_input_mode = (int)(2 / factor);

        float value_data[3]= {f_speed, s16_ramp_rate, s16_input_mode};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
    }
    send_command(id_num, 0x0C,data_list.byte_data, 0);
}

/**
 * @brief 单个关节力矩预设函数。
 * 预设指定关节编号的关节目标扭矩（Nm）
 *
 * @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param torque 关节输出（Nm)
 * @param param mode=1,改参数无意义；mode!=1,扭矩上升速度（Nm/s）
 * @param mode 控制模式选择
 *             mode=1, 扭矩直接控制模式，关节将目标扭矩直接设为torque
 *             mode!=1,扭矩爬升控制模式，关节将按照扭矩上升速率变化到torque。
 */
void preset_torque(uint8_t id_num, float torque, float param, int mode)
{

    float factor = 0.01;
    float f_torque = torque;
    int s16_ramp_rate, s16_input_mode;
    if (mode == 0)
    {
        s16_input_mode = (int)(1 / factor);
        s16_ramp_rate = 0;
    }
    else
    {
        s16_input_mode = (int)(6 / factor);
        s16_ramp_rate =  (int)((param) / factor);
    }

    float value_data[3]= {f_torque, s16_ramp_rate, s16_input_mode};
    int type_data[3]= {0,2,2};

    format_data(value_data,type_data,3,"encode");

    send_command(id_num,0x0C,data_list.byte_data,0);
}

// 功能函数，用户使用

// 运动控制功能

/**
 * @brief 单个关节角度控制函数。
 * 控制指定关节编号的关节按照指定的速度转动到指定的角度（绝对角度，相对于关节零点位置）。
 *
 * @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param angle 关节角度（-360~360）*n，支持大角度转动
 * @param speed 最大速度限制或前馈速度（r/min）
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
 * @param mode 角度控制模式选择，关节支持三种角度控制模式，
 *             mode = 0: 轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 梯形轨迹模式，这种模式下可以指定运动过程中的速度（speed）和启停加速度（accel）。
 *             mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @note 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会报错报错导致关节报motor error 并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
 *       另外如果这种模式下accel=0，关节以最快速度运动到angle,speed参数不再其作用。
 */
void set_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_speed = (int)((abs(speed)) / factor);
        if (param > 300)
            param = 300;
        int s16_width = (int)(abs(param / factor));

        float value_data[3]= {f_angle,s16_speed,s16_width};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num,0x19,data_list.byte_data,0);
    }
    else if (mode == 1)
    {
        if ((speed > 0)&&(param > 0))
        {
            float f_angle = angle;
            int s16_speed = (int)((abs(speed)) / factor);
            int s16_accel = (int)((abs(param)) / factor);

            float value_data[3]= {f_angle,s16_speed,s16_accel};
            int type_data[3]= {0,2,2};

            format_data(value_data,type_data,3,"encode");

            send_command(id_num,0x1A,data_list.byte_data,0);
        }
    }
    else if (mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((speed) / factor);
        int s16_torque_ff = (int)((param) / factor);

        float value_data[3]= {f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num, 0x1B, data_list.byte_data, 0);
    }
}

/**
 * @brief 多个关节控制函数。
 * 控制指定关节编号的关节按照指定的速度转动到指定的角度，保证多个关节同时到达目标角度。
 *
 * @param id_list 关节编号组成的数组
 * @param angle_list 关节角度组成的数组
 * @param speed 最大的关节转动的速度（r/min）或前馈速度
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈速度（r/min)
 * @param mode 角度控制模式选择，关节支持三种角度控制模式，
 *             mode = 0: 多个关节轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 多个关节梯形轨迹模式，此时speed为多个关节中的最快速度（r/min），param为目标加速度（(r/min)/s）。
 *             mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @param n 数组长度
 */
void set_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{

#if ENABLE_INPUT_VALIDITY_CHECK
    if (id_list == NULL || angle_list == NULL) return;
#endif
    static int last_count = 0;
    static uint8_t *last_id_list = NULL;
    static float *current_angle_list = NULL;

    uint8_t state = 0;
    for (size_t i = 0; i < n; i++)
    {
        if (last_id_list[i] != id_list[i])
        {
            state = 1;
            break;
        }
    }

    if (last_count != n || state)
    {
        last_count = n;
        SERVO_FREE(last_id_list);
        SERVO_FREE(current_angle_list);
        last_id_list = SERVO_MALLOC(n * sizeof(uint8_t));
        current_angle_list = SERVO_MALLOC(n * sizeof(float));
        memcpy(last_id_list, id_list, n * sizeof(uint8_t));
        for (size_t i = 0; i < n; i++)
            current_angle_list[i] = get_state(id_list[i]).angle;
    }
    if (mode == 0)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i],angle_list[i],speed,param,mode);
        unsigned char order_num = 0x10;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};

        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    else if (mode == 1)
    {
        if ((speed < 0) ||( param < 0)) return;
        float delta_angle = 0;
        float t = 0;
        float fabs_temp;
        for (size_t i = 0; i < n; i++)
        {
            if (delta_angle < (fabs_temp = fabs(angle_list[i] - current_angle_list[i])))
                delta_angle = fabs_temp;
        }
        if (delta_angle <= (6 * speed * speed / fabs(param)))
                t = 2 * sqrt(delta_angle / (6 * fabs(param)));
            else
                t = speed / fabs(param) + delta_angle / (6 * speed);
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i],angle_list[i],t,param,mode);
        preset_angle(255,0,t,param,mode); // 解决最大 id 号响应滞后问题
        unsigned char order_num = 0x11;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");
        send_command(0,0x08,data_list.byte_data,0); // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    else if( mode == 2)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i],speed,param,mode);
        unsigned char order_num = 0x12;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");
        send_command(0,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    memcpy(current_angle_list, angle_list, n * sizeof(float)); 
}

/**
 * @brief 单个关节相对角度控制函数。
 * 控制指定关节编号的关节按照指定的速度相对转动指定的角度（相对角度，相对于关节当前位置）。
 *
 * @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param angle 关节相对角度（-360~360）*n，支持大角度转动
 * @param speed 最大速度限制或前馈速度（r/min）
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
 * @param mode 角度控制模式选择，关节支持三种角度控制模式，
 *             mode = 0: 轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 梯形轨迹模式，这种模式下可以指定运动过程中的速度（speed）和启停加速度（accel）。
 *             mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @note 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会报错并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
 *       另外如果这种模式下accel=0，关节以最快速度运动到angle,speed参数不再其作用。
 */
void step_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_speed = (int)((abs(speed)) / factor);
        if (param > 300)
            param = 300;
        int s16_width = (int)(abs(param / factor));

        float value_data[3]= {f_angle,s16_speed,s16_width};
        int type_data[3]= {0,2,2};
        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);

        float value_data_[3]= {0x10, 1, 0};
        int type_data_[3]= {3, 2, 1};
        format_data(value_data_, type_data_, 3, "encode");
        send_command(id_num,0x08,data_list.byte_data,0);
    }
    else if (mode == 1)
    {
        if ((speed > 0)&&(param > 0))
        {
            float f_angle = angle;
            int s16_speed = (int)((abs(speed)) / factor);
            int s16_accel = (int)((abs(param)) / factor);

            float value_data[3]= {f_angle,s16_speed,s16_accel};
            int type_data[3]= {0,2,2};
            format_data(value_data,type_data,3,"encode");
            send_command(id_num,0x0C,data_list.byte_data,0);

            float value_data_[3]= {0x11, 1, 0};
            int type_data_[3]= {3, 2, 1};
            format_data(value_data_, type_data_, 3, "encode");
            send_command(id_num,0x08,data_list.byte_data,0);
        }
    }
    else if (mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((speed) / factor);
        int s16_torque_ff = (int)((param) / factor);

        float value_data[3]= {f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]= {0,2,2};
        format_data(value_data,type_data,3,"encode");
        send_command(id_num, 0x0C, data_list.byte_data, 0);

        float value_data_[3]= {0x12, 1, 0};
        int type_data_[3]= {3, 2, 1};
        format_data(value_data_, type_data_, 3, "encode");
        send_command(id_num,0x08,data_list.byte_data,0);
    }
}
/**
 * @brief 多个关节相对角度控制函数。
 * 控制指定关节编号的关节按照指定的时间相对转动给定角度。
 *
 * @param id_list 关节编号组成的列表
 * @param angle_list 关节角度组成的列表
 * @param speed 最大的关节转动的速度（r/min）或前馈速度
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈速度（r/min)
 * @param mode 角度控制模式选择，关节支持三种角度控制模式，
 *             mode = 0: 多个关节轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 多个关节梯形轨迹模式，此时speed为多个关节中的最快速度（r/min），param为目标加速度（(r/min)/s）。
 *             mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @param n 数组长度
 */
void step_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{
#if ENABLE_INPUT_VALIDITY_CHECK
    if (id_list == NULL || angle_list == NULL) return;
#endif
    if (mode == 0)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        unsigned char order_num = 0x10;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0); //需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    else if( mode == 1)
    {
        if (speed <= 0 || param <= 0) return;
        float delta_angle = 0;
        float t = 0;
        float fabs_temp;
        for (size_t i = 0; i < n; i++)
        {
            if(delta_angle < (fabs_temp = fabs(angle_list[i])))
                delta_angle = fabs_temp;
        }
        if(delta_angle <= (6 * speed * speed / fabs(param)))
            t = 2 * sqrt(delta_angle / (6 * fabs(param)));
        else
            t = speed / fabs(param) + delta_angle / (6 * speed);
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], t, param, mode);
        preset_angle(255,0,t,param,mode); // 解决最大 id 号响应滞后问题
        unsigned char order_num = 0x11;

        float value_data[3]= {order_num,2,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");
        send_command(0, 0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
//        send_command(0, 0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    else if (mode == 2)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        unsigned char order_num = 0x12;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");
        send_command(0,0x08,data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
    }
}
/**
 * @brief 单个关节力位混合控制函数。
 * 控制指定 ID 编号的一体化关节按照限定的转速和力矩转动到指定的角度（绝对角度，相对于用户设定的零点角度）
 *
 * @param id_num 需一体化关节 ID 编号,如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle 一体化关节角度（°）。
 * @param speed 限定转速值（r/min）。
 * @param torque 限定力矩值（Nm)。
 *            
 * @note 当设置的转速相对于力矩过大，或力矩相对于转速过小，则关节无法在短时间内提供足够的加速度使得转速将为 0，此时若关节未遇阻力会出现在目标角度过冲
         现象，此为物理规律，暂时没有好的解决办法
 */
void set_angle_adaptive(uint8_t id_num, float angle, float speed, float torque)
{
    float factor = 0.01;
    float f_angle = angle;
    int s16_speed = (int)((abs(speed)) / factor);
    int s16_torque = (int)(abs(torque / factor));

    float value_data[3]= {f_angle,s16_speed,s16_torque};
    int type_data[3]= {0,2,2};

    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x0B,data_list.byte_data,0);
}
/**
 * @brief 多个关节力位混合控制函数。
 * 控制指定 ID 编号的一体化关节按照限定的转速和力矩转动到指定的角度（绝对角度，相对于用户设定的零点角度）
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param angle_list 一体化关节角度（°）组成的列表。
 * @param speed_list 限定转速值（r/min）组成的列表。
 * @param torque_list 限定力矩值（Nm)组成的列表。
 * @param n 数组长度
 *            
 * @note 当设置的转速相对于力矩过大，或力矩相对于转速过小，则关节无法在短时间内提供足够的加速度使得转速将为 0，此时若关节未遇阻力会出现在目标角度过冲
         现象，此为物理规律，暂时没有好的解决办法
 */
void set_angles_adaptive(uint8_t id_list[], float angle_list[], float speed_list[], float torque_list[], size_t n)
{
    if (id_list == NULL || angle_list == NULL || speed_list == NULL || torque_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_angle(id_list[i], angle_list[i], fabs(speed_list[i]), fabs(torque_list[i]), 1);
    preset_angle(255, angle_list[i], fabs(speed_list[i]), fabs(torque_list[i]), 1); // 解决最大 id 号响应滞后问题
    unsigned char order_num = 0x11;
    float value_data[3]= {order_num,3,1};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(0,0x08,data_list.byte_data,0);
}
/**
 * @brief 单个关节等待函数
 * 延时等待直到给定关节到达目标位置(只对角度控制指令有效)
 *
 * @param id_num 需要重启的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 ** @return 无
 */
void position_done(uint8_t id_num)
{
    int traj_done = 0;
    while(traj_done == 0 || READ_FLAG == -1)
    {
        traj_done = read_property(id_num, 32002,3);
    }
}
/**
 * @brief 多个关节等待函数
 * 程序等待（阻塞）直到所有关节都到达目标位置(只对角度控制指令有效)
 *
 * @param id_list: 关节编号组成的列表
 ** @return 无
 */
void positions_done(uint8_t *id_list,size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        position_done(id_list[i]);
    }
}
/**
 * @brief 单个关节速度控制函数。
 * 控制指定关节编号的关节按照指定的速度连续整周转动。
 *
 * @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param speed 目标速度（r/min）
 * @param param mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
 * @param mode 控制模式选择
 *             mode=1, 速度前馈控制模式，关节将目标速度直接设为speed
 *             mode!=1,速度爬升控制模式，关节将按照目标加速度axis0.controller.config_.vel_ramp_rate变化到speed。
 * @note 在速度爬升模式下，如果目标加速度设置为0，则关节速度将保持当前值不变。
 */
void set_speed(uint8_t id_num, float speed, float param, int mode)
{
    float factor = 0.01;
    float f_speed = speed;
    if (mode == 0)
    {
        int s16_torque = (int)((param) / factor);
        if( f_speed == 0)
            s16_torque = 0;
        unsigned short u16_input_mode = 1;

        float value_data[3]= {f_speed,s16_torque,u16_input_mode};
        int type_data[3]= {0,2,1};
        format_data(value_data,type_data,3,"encode");
    }
    else
    {
        int s16_ramp_rate = (int)((param) / factor);
        unsigned short u16_input_mode = 2;

        float value_data[3]= {f_speed,s16_ramp_rate,u16_input_mode};
        int type_data[3]= {0,2,1};
        format_data(value_data,type_data,3,"encode");
    }
    send_command(id_num,0x1c,data_list.byte_data,0);
}
/**
 * @brief 多个关节速度控制函数。
 * 控制指定多个关节编号的关节按照指定的速度连续整周转动。
 *
 * @param id_list 关节编号组成的列表
 * @param speed_list 关节目标速度（r/min）组成的列表
 * @param param mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
 * @param mode 控制模式选择
 *             mode=1, 速度前馈控制模式，关节将目标速度直接设为speed
 *             mode!=1,速度爬升控制模式，关节将按照目标加速度变化到speed。
 * @param n 数组长度
 */
void set_speeds(uint8_t *id_list, float *speed_list, float param, float mode, size_t n)
{
    if (id_list == NULL || speed_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_speed(id_list[i], speed_list[i], param, mode);
    preset_speed(255, speed_list[i], param, mode); // 解决最大 id 号响应滞后问题
    unsigned char order_num = 0x13;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(0, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 单个关节力矩（电流）闭环控制函数。
 * 控制指定关节编号的关节输出指定的扭矩（Nm）
 *
 * @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param torque 关节输出（Nm)
 * @param param mode=1,改参数无意义；mode!=1,扭矩上升速度（Nm/s）
 * @param mode 控制模式选择
 *             mode=1, 扭矩直接控制模式，关节将目标扭矩直接设为torque
 *             mode!=1,扭矩爬升控制模式，关节将按照扭矩上升速率变化到torque。
 * @note 扭矩爬升控制模式下，如果点击扭矩上升速率为0，则点击扭矩将在当前值保持不变。
 */
void set_torque(uint8_t id_num, float torque, float param, int mode)
{
    float factor = 0.01;
    int u16_input_mode,s16_ramp_rate;
    float f_torque = torque;
    if (mode == 0)
    {
        u16_input_mode = 1;
        s16_ramp_rate = 0;
    }
    else
    {
        u16_input_mode = 6;
        s16_ramp_rate = (int)((param) / factor);
    }
    float value_data[3]= {f_torque, s16_ramp_rate,u16_input_mode};
    int type_data[3]= {0,2,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x1d,data_list.byte_data,0);
}
/**
 * @brief 多个个关节力矩控制函数。
 * 同时控制多个关节编号的关节目标扭矩（Nm）
 *
 * @param id_list 关节编号组成的列表
 * @param torque_list 关节目标扭矩（Nm)组成的列表
 * @param param mode=1,改参数无意义；mode!=1,扭矩上升速度（Nm/s）
 * @param mode 控制模式选择
 *             mode=1, 扭矩直接控制模式，关节将目标扭矩直接设为torque
 *             mode!=1,扭矩爬升控制模式，关节将按照扭矩上升速率变化到torque。
 * @param n 数组长度
 */
void set_torques(uint8_t *id_list, float *torque_list, float param, int mode, size_t n)
{
    if (id_list == NULL || torque_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_torque(id_list[i], torque_list[i], param, mode);
    preset_torque(255, torque_list[i], param, mode); // 解决最大 id 号响应滞后问题
    unsigned char order_num = 0x14;
    float value_data[3]= {order_num};
    int type_data[3]= {3};
    format_data(value_data,type_data,1,"encode");
    send_command(0, 0x08, data_list.byte_data, 0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
* @brief 单个关节阻抗控制函数。
* 对指定关节编号的关节进行阻抗控制。
*
* @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
* @param pos 关节目标角度（度）
* @param vel 关节目标速度（r/min）
* @param tff 前馈扭矩（Nm)
* @param kp 刚度系数(rad/Nm)
* @param kd 阻尼系数(rad/s/Nm)
* @note 阻抗控制为MIT开源方案中的控制模式，其目标输出扭矩计算公式如下：
        torque = kp*( pos – pos_) + t_ff + kd*(vel – vel_)
        其中pos_和vel_分别为输出轴当前实际位置（degree）和当前实际速度（r/min）, kp和kd为刚度系数和阻尼系数，系数比例与MIT等效
*/
void impedance_control(uint8_t id_num, float pos, float vel, float tff, float kp, float kd)
{
    float factor = 0.01;
    kp = fabs(kp);
    kd = fabs(kd);
    if (kp == 0)
        return;
    float angle_set = (- kd * vel - tff) / kp + pos;
    preset_angle(id_num,angle_set,vel, tff, 2);
    unsigned char order_num = 0x15;
    float value_data[3]= {order_num,(int)(kp / factor),(int)(kd / factor)};
    int type_data[3]= {3,2,2};
    format_data(value_data,type_data,3,"encode");
    send_command(0, 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
* @brief 多个关节阻抗控制函数。
* 对指定关节编号的关节进行阻抗控制。
*
* @param id_list 一体化关节 ID 编号组成的列表。
* @param angle_list 一体化关节目标角度组成的列表（°）。
* @param speed_list 一体化关节目标转速组成的列表（r/min）。
* @param tff_list 前馈力矩组成的列表（Nm）。
* @param kp_list 角度刚度系数组成的列表（Nm/°），每个元素均需大于 0。
* @param kd_list 转速阻尼系数组成的列表（Nm/(r/min)），每个元素均需大于 0。
* @param n 数组长度
*
* @note  该函数直接控制关节输出力矩，其目标输出力矩计算公式如下：
              torque = kp_list[i] * (angle_list[i] – angle_[i]) + tff_list[i] + kd_list[i] * (speed_list[i] – speed_[i])
        其中 angle_[i] 和 speed_[i] 分别为对应关节输出轴当前实际角度（度）和当前实际转速（r/min）, kp_list[i] 和 kd_list[i]
        为刚度系数和阻尼系数。
*/
void impedance_control_multi(uint8_t id_list[], float angle_list[], float speed_list[], float tff_list[], float kp_list[], float kd_list[], size_t n)
{
    if (id_list == NULL || angle_list == NULL || speed_list == NULL || tff_list == NULL || kp_list == NULL || kd_list == NULL) return;
    float factor = 0.001;
    float angle_set_list[n];
    for (size_t i = 0; i < n; i++)
    {
        if (kp_list[i] == 0) return;
        angle_set_list[i] = (- kd_list[i] * speed_list[i] - tff_list[i]) / kp_list[i] + angle_list[i];
        preset_angle(id_list[i],angle_set_list[i],speed_list[i], tff_list[i], 2);
        unsigned char order_num = 0x16;
        float value_data[3]= {order_num,(int)abs(kp_list[i] / factor),(int)abs(kd_list[i] / factor)};
        int type_data[3]= {3,2,2};
        format_data(value_data,type_data,3,"encode");
        send_command(id_list[i], 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
        HAL_Delay(1);
    }

    unsigned char order_num = 0x17;
    float value_data_[3]= {order_num,1,1};
    int type_data_[3]= {3,0,0};
    format_data(value_data_,type_data_,3,"encode");
    send_command(0, 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
* @brief 单个一体化关节运动跟随与助力函数。
*
* @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时总线上有多个一体化关节，则多个一体化关节都会执行该操作。
* @param angle 标角度（°），即助力目标角度，该值减去关节当前角度即为助力行程，目标角度范围为 -300~300°。
* @param speed 限定转速（r/min），即助力的限定转速，防止助力力矩引起的一直加速导致转速过快。
* @param angle_err 角度差值（°），表示运动跟随与助力的角度灵敏度。
* @param speed_err 转速差值（r/min），表示运动跟随与助力的转速灵敏度。
* @param torque 助力力矩（Nm)。

* @note  当关节在停止状态下检测到角度差 angle_err 和转速差 speed_err 时向目标角度 angle 方向提供力矩大小为 torque 的助力，
         并在到达 angle 后停止并保持位置。
         注：
           a、当助力与外部驱动力之和大于阻力，关节会持续转动；
           b、当助力与外部驱动力之和小于阻力，关键开始减速，当转速小于 2 倍转速差值 speed_err 时，关节停止输出助力；
           c、一般情况下，该功能为人进行助力，强烈建议用户将助力力矩设置在人力所能及的范围内，即人力可使关节停止转动；
           d、若必须设置超出人力的力矩，则必须在合理位置设置牢固的机械限位，以避免超出运动范围给人或物体带来损伤。
*/
void motion_aid(uint8_t id_num, float angle, float speed, float angle_err, float speed_err, float torque)
{
    float factor = 0.01;
    if (angle < -300 || angle > 300) return;
    float value_data[4]= {(int)(angle/factor), (int)(angle_err/factor), (int)(speed_err/factor), (int)(torque/factor)};
    int type_data[4]= {2, 1, 1, 2};
    format_data(value_data,type_data,4,"encode");
    send_command(id_num,0x0D,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    set_speed_adaptive(id_num,speed);
    set_speed_adaptive(id_num,speed);
}
/**
* @brief 多个一体化关节运动跟随与助力函数。
*
* @param id_list 一体化关节 ID 编号组成的列表。
* @param angle_list 目标角度组成的列表（°），即助力目标角度，该值减去关节当前角度即为助力行程。
* @param speed_list 限定转速组成的列表（r/min），即助力的限定转速，防止助力力矩引起的一直加速导致转速过快。
* @param angle_err_list 角度差值（°）组成的列表，表示运动跟随与助力的角度灵敏度。
* @param speed_err_list 转速差值（r/min）组成的列表，表示运动跟随与助力的转速灵敏度。
* @param torque_list 助力力矩（Nm)组成的列表。
* @param n 数组长度。
*
* @note  指定多个一体化关节进行运动跟随与助力。
         当关节在停止状态下检测到角度差 angle_er_list[i] 和转速差 speed_err_list[i] 时向目标角度 angle_list[i] 方向提供力矩大小为
         torque_list[i] 的助力，并在到达 angle_list[i] 后停止并保持位置。
         注：
           a、当助力与外部驱动力之和大于阻力，关节会持续转动；
           b、当助力与外部驱动力之和小于阻力，关键开始减速，当转速小于 2 倍转速差值 speed_err_list[i] 时，关节停止输出助力；
           c、一般情况下，该功能为人进行助力，强烈建议用户将助力力矩设置在人力所能及的范围内，即人力可使关节停止转动；
           d、若必须设置超出人力的力矩，则必须在合理位置设置牢固的机械限位，以避免超出运动范围给人或物体带来损伤。
*/
void motion_aid_multi(uint8_t id_list[], float angle_list[], float speed_list[], float angle_err_list[], float speed_err_list[], float torque_list[], size_t n)
{
    if (id_list == NULL || angle_list == NULL || speed_list == NULL || angle_err_list == NULL || speed_err_list == NULL || torque_list == NULL) return;
    float factor = 0.01;
    for (size_t i = 0; i < n; i++)
    {
        if (angle_list[i] < -300 || angle_list[i] > 300) return;
    }
    for (size_t i = 0; i < n; i++)
    {
        float value_data[4]= {(int)(angle_list[i]/factor), (int)(angle_err_list[i]/factor), (int)(speed_err_list[i]/factor), (int)(torque_list[i]/factor)};
        int type_data[4]= {2, 1, 1, 2};
        format_data(value_data,type_data,4,"encode");
        send_command(id_list[i],0x06,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
        send_command(255,0x06,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    unsigned char order_num = 0x11;
    float value_data_[3]= {order_num,4,1};
    int type_data_[3]= {3,1,0};
    format_data(value_data_,type_data_,3,"encode");
    send_command(0, 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
    SERVO_DELAY(100); // 延时 0.1s
    for (size_t i = 0; i < n; i++)
    {
        set_speed_adaptive(id_list[i], speed_list[i]);
    }
}
/**
* @brief 设置一体化关节转自适应转速函数。
*
* @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
* @param speed_adaptive 自适应转速限制（r/min）（必须大于 0）。
*
* @note  设置一体化关节转速限制 speed_adaptive （r/min），此后关节自适应转速绝对值不超过 speed_adaptive。
         注意：
            1. 该函数执行完转速限制在本次开机运行期间有效，关机或重启后将失效。
            2. 若想关机重启前取消该限制，只需设置一个非常大的数值，比如令 speed_adaptive = 100000。
            3. 本函数需在运动指令之后运行。
*/
void set_speed_adaptive(uint8_t id_num, float speed_adaptive)
{
    if (speed_adaptive <=0) return;
    preset_angle(id_num, fabs(speed_adaptive), 0, 0, 1);
    preset_angle(id_num, fabs(speed_adaptive), 0, 0, 1);
    unsigned char order_num = 0x20;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
* @brief 设置一体化关节转自适应力矩函数。
*
* @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
* @param torque_adaptive 自适应力矩限制（Nm）（必须大于 0）。
*
* @note  设置一体化关节力矩限制 torque_adaptive （Nm），此后关节自适应力矩绝对值不超过 torque_adaptive。
         注意：
            1. 该函数执行完力矩限制在本次开机运行期间有效，关机或重启后将失效。
            2. 若想关机重启前取消该限制，只需设置一个非常大的数值，比如令 torque_adaptive = 100000。
            3. 本函数需在运动指令之后运行。
*/
void set_torque_adaptive(uint8_t id_num, float torque_adaptive)
{
    if (torque_adaptive <=0) return;
    preset_angle(id_num, fabs(torque_adaptive), 0, 0, 1);
    preset_angle(id_num, fabs(torque_adaptive), 0, 0, 1);
    unsigned char order_num = 0x21;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
* @brief 设置一体化关节控制环 PID 函数。
*
* @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
* @param P 位置增益（必须大于 0）。
* @param I 积分增益（必须大于 0）。
* @param D 转速增益（必须大于 0）。
*
* @note  设置一体化关节控制环的位置增益 P、转速增益 D、积分增益 I，以便实现调整关节控制性能的目的。
         该函数执行完 PID 的值在本次开机运行期间有效，关机或重启后将失效。
         如用户决定永久使用某组 PID 则可在使用该函数设置 PID 后，请紧接着使用 save_config 函数。
*/
void set_pid(uint8_t id_num, float P, float I, float D)
{
    if (P <= 0 || I <= 0 || D <= 0) return;
    write_property(id_num, 32102, 0, P);
    write_property(id_num, 32104, 0, I);
    write_property(id_num, 32103, 0, D);
}
/**
 * @brief 急停函数
 * 控制关节紧急停止。关节急停后将切换到IDLE待机模式，关节卸载并生成ERROR_ESTOP_REQUESTED错误标志，不再响应set_angle/speed/torque指令。
 * 如果要恢复正常控制模式，需要首先用clear_error清除错误标志后,然后用set_mode函数将模式设置为2（闭环控制模式）。
 *
 * @param id_num 需要急停的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 */
void estop(uint8_t id_num)
{
    unsigned char order_num = 0x06;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}
// 参数设置功能

/**
 * @brief 设置关节ID号。
 * 改变关节ID号（掉电保存）
 *
 * @param id_num 需要重新设置编号的关节编号,如果不知道当前关节编号，可以用0广播，但是这时总线上只能连一个关节，否则多个关节会被设置成相同编号
 * @param new_id 新关节编号，关节ID号范围为1~63
 */
void set_id(uint8_t id_num, int new_id)
{
    write_property(id_num,31001,3,new_id);
    save_config(new_id);
}
/**
 * @brief 设置关节CAN波特率。
 * 设置CAN波特率（掉电保存）
 *
 * @param id_num 需要重新设置波特率的关节编号,如果不知道当前关节编号，可以用0广播。
 * @param baud_rate CAN波特率，支持125k,250k,500k,1M中任意一种,修改成功后需手动将主控CAN波特率也修改为相同值。
 */
void set_can_baud_rate(uint8_t id_num, int baud_rate)
{
    write_property(id_num,21001,3,baud_rate);
    save_config(id_num);
}
/* @brief 设置关节模式。
* 设置关节进入不同的控制模式。
*
* @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
* @param mode 关节模式编号
*             mode = 1: IDLE待机模式，关节将关掉PWM输出，关节卸载
*             mode = 2: 闭环控制模式，set_angle， set_speed, set_torque函数必须在闭环控制模式下才能进行控制。（关节上电后的默认模式）
* @note 模式3和模式4是用来校准关节和编码器参数，出厂前已完成校准，正常情况下不要使用。
*/
void set_mode(uint8_t id_num, int mode)
{
    if (mode == 1)
        write_property(id_num,30003,3,1);
    else if (mode == 2)
        write_property(id_num,30003,3, 2);
}
/**
 * @brief 设置关节零点位置函数
 * 设置当前位置为关节输出轴零点，设置完后当前位置为0度
 *
 * @param id_num 需要设置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 */
void  set_zero_position(uint8_t id_num)
{

    unsigned char order_num = 0x05;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
    save_config(id_num); // 保存后掉电不丢失
}
/**
 * @brief 设置一体化关节零点角度函数，当次启动有效，重启后失效，此时需将关节编码器电源线拔掉
 * 设置当前位置为关节输出轴零点，设置完后当前位置为0度
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
void set_zero_position_temp(uint8_t id_num)
{
    unsigned char order_num = 0x23;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
 * @brief 设置关节软件限位极限位置
 * 设置关节预输出轴软件限位极限位置值，设置成功后关节在位置、速度及扭矩控制模式关节输出轴将被限制在[angle_min, angle_max]范围内
 *  （注意：当前输出轴位置必须在[angle_min, angle_max]范围内，否则将设置失败）
 *
 * @param id_num 需要设置的关节编号,如果不知道当前关节编号，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param angle_min 软件限位最小角度（该参数与dr.output_shaft.angle_min对应）
 * @param angle_max 软件限位最大角度（该参数与dr.output_shaft.angle_max对应）
 * @return 是否设置成功
 */
int8_t set_angle_range(uint8_t id_num, float angle_min, float angle_max)
{

    float pos_vel = get_state(id_num).angle;
    if (READ_FLAG == 1)
    {
        if (pos_vel < angle_min || pos_vel > angle_max || READ_FLAG != 1) return -1;
        write_property(id_num, 38004, 0, angle_min);
        write_property(id_num, 38005, 0, angle_max);
        write_property(id_num, 38006, 3, 1);
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
 * @brief 取消本次运行期间一体化关节运行过程中的角度限位。
 *
 * @param id_num 需要设置的关节编号,如果不知道当前关节编号，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 */
int8_t disable_angle_range(uint8_t id_num)
{
    write_property(id_num, 38006, 3, 0);
    if (read_property(id_num, 38006, 3) == 0)
        return 1;
    else
        return 0;
}
/**
 * @brief 设置一体化关节极限角度属性，设置成功后一体化关节的可控制的转动角度将限定在[angle_min, angle_max]范围内，每次开机重启均默认有效。
 * 设置关节预输出轴软件限位极限位置值，设置成功后关节在位置、速度及扭矩控制模式关节输出轴将被限制在[angle_min, angle_max]范围内
 *  （注意：当前输出轴位置必须在[angle_min, angle_max]范围内，否则将设置失败）
 *
 * @param id_num 需要设置的关节编号,如果不知道当前关节编号，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param angle_min 软件限位最小角度（该参数与dr.config.angle_min对应）
 * @param angle_max 软件限位最大角度（该参数与dr.config.angle_max对应）
 * @return 是否设置成功
 * 
 * @note a、使用该函数时输出轴角度必须在[angle_min, angle_max]范围内，否则将设置失败；
         b、限位范围设置成功后，再执行 save_config() 函数，每次开机重启后均有效；
         c、如需取消该属性影响，请使用 disable_angle_range_config() 函数将该属性关闭，则本次开机该属性不起作用；
            若随后使用 save_config() 则该属性将永久失去，如需找回该属性，则再次使用本函数即可。
 */
int8_t set_angle_range_config(uint8_t id_num, float angle_min, float angle_max)
{
    float angle = get_angle(id_num);
    if (READ_FLAG == 1)
    {
        if (angle < angle_min || angle > angle_max || READ_FLAG != 1) return -1;
        write_property(id_num, 31202, 0, angle_min);
        write_property(id_num, 31203, 0, angle_max);
        write_property(id_num, 31201, 3, 1);
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
 * @brief 取消一体化关节角度限位属性。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @return 是否设置成功
 * 
 */
int8_t disable_angle_range_config(uint8_t id_num)
{
    write_property(id_num, 31201, 3, 0);
    if (read_property(id_num, 31201, 3) == 0)
        return 1;
    else
        return 0;
}
/**
 * @brief 取消一体化关节转速限制属性。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * 
 */
void set_speed_limit(uint8_t id_num, float speed_limit)
{
    if (speed_limit <= 0) return;
    preset_angle(id_num, fabs(speed_limit), 0, 0, 1);
    unsigned char order_num = 0x18;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
 * @brief 取消一体化关节力矩限制属性。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * 
 */
void set_torque_limit(uint8_t id_num, float torque_limit)
{
    if (torque_limit <= 0) return;
    preset_angle(id_num, fabs(torque_limit), 0, 0, 1);
    unsigned char order_num = 0x19;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
 * @brief 修改关节属性参数
 * 修改关节属性参数，这里的属性参数为关节控制参数
 *
 * @param id_num 需要修改的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 * @param param_address 需要读取的属性参数地址，例如"dr.voltage"，"dr.config.can_id"等，具体参数名称见parameter_interface.py文件里property_address字典里的键值。
 * @param param_type 需要读取的属性参数数据类型。
 * @param value 对应参数的目标值。
 */
void write_property(uint8_t id_num,unsigned short param_address,int8_t param_type,float value)
{
    float value_data[3]= {param_address,param_type,value};
    int type_data[3]= {1,1,param_type};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x1F,data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
}
/**
 * @brief 读取关节ID。
 *
 * @param id_num 需要读取的关节编号,如果不知道当前关节编号，可以用0广播，但是这时总线上只能连一个关节，否则将报错。
 * @return 关节的 ID 号
 */
uint8_t get_id(uint8_t id_num)
{
    return read_property(id_num,31001,3);
}
/**
 * @brief 读取关节角度。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return 关节角度 (°)
 */
float get_angle(uint8_t id_num)
{
    return read_property(id_num,38001,0);
}
/**
 * @brief 读取关节转速。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return 关节转速 (r/min)
 */
float get_speed(uint8_t id_num)
{
    return read_property(id_num,38002,0);
}
/**
 * @brief 读取关节力矩。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return 关节力矩 (Nm)
 *
 */
float get_torque(uint8_t id_num)
{
    return read_property(id_num,38003,0);
}
/**
 * @brief 使能关节状态反馈。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 *
 */
void enable_angle_speed_torque_state(uint8_t id_num)
{
	write_property(id_num, 22001, 3, 1);
}
/**
 * @brief 设置角度、转速、力矩状态实时反馈时间间隔函数，单位 ms，默认为 2
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @param n_ms 角度、转速、力矩状态实时反馈时间间隔，单位 ms，当总线中不同 ID 号关节数量为 n 时，请将该值设置为 2n
 */
void set_state_feedback_rate_ms(uint8_t id_num, uint32_t n_ms)
{
	write_property(id_num, 31002, 3, n_ms);
}
/**
 * @brief 读取单个一体化关节角度、转速、力矩实时状态反馈的函数。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @param n 总线中正在状态反馈的关节数量
 * @return 关节角度、转速、力矩组成的结构体 angle_speed_torque
 * @note id_num 不可以为零
 */
struct angle_speed_torque angle_speed_torque_state(uint8_t id_num)
{
	READ_FLAG=0;
	struct angle_speed_torque angle_speed_torque = {0, 0, 0};
	while ((id_num != (uint8_t)(((can_id & 0x07E0) >> 5)&0xFF)))
	{
		receive_data();
	}
	if (id_num == ((uint8_t)((can_id & 0x07E0) >> 5)&0xFF))
	{
		float factor = 0.01f;
		float value_data[3]= {0,0,0};
		int type_data[3]= {0,2,2};
		format_data(value_data,type_data,3,"decode");
		angle_speed_torque.angle = data_list.value_data[0];
		angle_speed_torque.speed = data_list.value_data[1]*factor;
		angle_speed_torque.torque = data_list.value_data[2]*factor;
	}
	return angle_speed_torque;
}
/**
 * @brief 关闭使能关节状态反馈。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 *
 */
void disable_angle_speed_torque_state(uint8_t id_num)
{
	write_property(id_num, 22001, 3, 0);
}
/**
 * @brief 读取关节PID。
 *
 * @param id_num 一体化关节 ID 编号,如果不知道当前一体化关节 ID 编号，可以用 0广播，但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @@return struct PID 关节pid的结构体: [P, I, D]
 */
struct PID get_pid(uint8_t id_num)
{
	struct PID pid = {0,0,0};
	pid.P = read_property(id_num,32102,0);
	pid.D = read_property(id_num,32103,0);
	pid.I =  read_property(id_num,32104,0);
    return pid;
}
/**
 * @brief 读取关节的当前位置和速度
 * 读取关节输出轴当前位置和速度列表，单位分别为度（°）和转每分钟(r/min)
 *
 *同时作为实时状态（实时位置、实时速度、实时扭矩、是否到达目标位置、是否报错）快速读取接口，进行实时控制时可采用该函数进行快速读取关节状态
 *注：1. 但是需要将MOTOR_NUM变量根据最大的关节ID号进行调整，保证MOTOR_NUM大于或等于最大的关节ID号；
      2. enable_reply_state值不影响快速读取接口，只影响发送控制指令时是否实时返回关节状态；

 * @return 关节的位置和速度
 * @param id_num 需要读取的关节编号,如果不知道当前关节编号，可以用0广播，但是这时总线上只能连一个关节，否则将报错。
 * @return struct servo_state 储存关节位置和速度的结构体
 */
struct servo_state get_state(uint8_t id_num)
{
    struct servo_state state = {0, 0};
    state.angle = read_property(id_num,38001,0);
    state.speed = read_property(id_num,38002,0);
    return state;
}

/**
 * @brief 读取关节的当前电压和电流
 * 读取关节当前电压和q轴电流列表，单位分别为伏（V）和安(A)
 *
 * @param id_num 需要读取的关节编号,如果不知道当前关节编号，可以用0广播，但是这时总线上只能连一个关节，否则将报错。
 * @return struct servo_volcur 储存关节电压和电流的结构体
 */
struct servo_volcur get_volcur(uint8_t id_num)
{
    struct servo_volcur volcur = {0, 0};
    volcur.vol  = read_property(id_num, 1, 0);
    if(READ_FLAG==1) {
        volcur.cur  = read_property(id_num, 33201,0);
    }
    else READ_FLAG=-1;
    return volcur;
}
/**
 * @brief 读取关节属性参数
 * 读取关节属性参数，这里的属性参数包括关节状态量及关节控制参数
 *
 * @param id_num 需要读取的关节编号,如果不知道当前关节编号，可以用0广播，但是这时总线上只能连一个关节，否则将报错。
 * @param param_address  需要读取的属性参数的键码。
 * @param param_type 需要读取的属性参数的数据类型。
 * @return 对应属性参数的值
 */
float read_property(uint8_t id_num,int param_address,int param_type)
{
    float value_data[3]= {param_address,param_type,0};
    int type_data[3]= {1,1,3};
    format_data(value_data,type_data,3,"encode");
    READ_FLAG=0;
    send_command(id_num,0x1E,data_list.byte_data,0);// 需要用标准帧（数据帧）进行发送，不能用远程帧

    receive_data();

    if (READ_FLAG == 1)
    {
        float value_data[3]= {0,0,0};
        int type_data[3]= {1,1,param_type};
        format_data(value_data,type_data,3,"decode");

        float value=data_list.value_data[2];
        return value;
    }
    else
    {
        READ_FLAG=-1;
        return 0;
    }
}

/*
其他系统辅助函数，一般情况下无需使用
*/


/**
 * @brief 保存配置函数
 * 正常情况下，通过write_property修改的属性关节上电重启之后，会恢复为修改前的直，如果想永久保存，则需要用save_config函数将相关参数保存到flash中，掉电不丢失。
 *
 * @param id_num 需要保存配置的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 */
void save_config(uint8_t id_num)
{

    unsigned char order_num = 0x01;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08, data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
    SERVO_DELAY(2000);
}
/**
 * @brief 关节重启函数
 * 关节软件重启，效果与重新上电类似。
 *
 * @param id_num 需要重启的关节ID编号,如果不知道当前关节ID，可以用0广播，如果总线上有多个关节，则多个关节都会执行该操作。
 */
void reboot(uint8_t id_num)
{

    unsigned char order_num = 0x03;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧

}

/**
 * @brief 恢复出厂设置，不改变用户设置的 ID 号。
 * 程序等待（阻塞）直到所有关节都到达目标位置(只对角度控制指令有效)
 *
 * @param id_num: 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 ** @return 无
 */
void init_config(uint8_t id_num)
{
    float value_data[3]= {id_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x0E, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}



