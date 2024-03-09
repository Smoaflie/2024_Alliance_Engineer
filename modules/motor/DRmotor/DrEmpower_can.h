#ifndef __DrEmpower_can__
#define __DrEmpower_can__

#include <stdint.h>
#include <stdio.h>

struct servo_state
{
    float angle;
    float speed;
};

struct servo_volcur
{
    float vol;
    float cur;
};

struct PID
{
    float P;
    float I;
    float D;
};

struct angle_speed_torque
{
	float angle;
	float speed;
	float torque;
};

void format_data( float *value_data, int *type_data,int length, char * str);
void preset_angle(uint8_t id_num, float angle, float t, float param, int mode);
void preset_speed(uint8_t id_num, float speed, float param, int mode);
void preset_torque(uint8_t id_num, float torque, float param, int mode);

/* 设置 ID 号 */
void set_id(uint8_t id_num, int new_id);

/* 运动控制 */
void set_angle(uint8_t id_num, float angle, float speed, float param, int mode);
void set_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n);
void step_angle(uint8_t id_num, float angle, float speed, float param, int mode);
void step_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n);
void set_angle_adaptive(uint8_t id_num, float angle, float speed, float torque);
void set_angles_adaptive(uint8_t id_list[], float angle_list[], float speed_list[], float torque_list[], size_t n);
void impedance_control(uint8_t id_num, float pos, float vel, float tff, float kp, float kd);
void impedance_control_multi(uint8_t id_list[], float angle_list[], float speed_list[], float tff_list[], float kp_list[], float kd_list[], size_t n);
void motion_aid(uint8_t id_num, float angle, float speed, float angle_err, float speed_err, float torque);
void motion_aid_multi(uint8_t id_list[], float angle_list[], float speed_list[], float angle_err_list[], float speed_err_list[], float torque_list[], size_t n);
void position_done(uint8_t id_num);
void positions_done(uint8_t *id_list,size_t n);
void set_speed(uint8_t id_num, float speed, float param, int mode);
void set_speeds(uint8_t *id_list, float *speed_list, float param, float mode, size_t n);
void set_torque(uint8_t id_num, float torque, float param, int mode);
void set_torques(uint8_t *id_list, float *torque_list, float param, int mode, size_t n);
void estop(uint8_t id_num);

/* 参数回读 */
uint8_t get_id(uint8_t id_num);
float get_angle(uint8_t id_num);
float get_speed(uint8_t id_num);
struct servo_state get_state(uint8_t id_num);
float get_torque(uint8_t id_num);
struct servo_volcur get_volcur(uint8_t id_num);
void enable_angle_speed_torque_state(uint8_t id_num);
void set_state_feedback_rate_ms(uint8_t id_num, uint32_t n_ms);
struct angle_speed_torque angle_speed_torque_state(uint8_t id_num);
void disable_angle_speed_torque_state(uint8_t id_num);
struct PID get_pid(uint8_t id_num);
float read_property(uint8_t id_num,int param_address, int param_type);

/* 参数设置 */
void set_zero_position_temp(uint8_t id_num);
void set_zero_position(uint8_t id_num);
int8_t set_angle_range(uint8_t id_num, float angle_min, float angle_max);
int8_t disable_angle_range(uint8_t id_num);
int8_t set_angle_range_config(uint8_t id_num, float angle_min, float angle_max);
int8_t disable_angle_range_config(uint8_t id_num);
void set_speed_limit(uint8_t id_num, float speed_limit);
void set_torque_limit(uint8_t id_num, float torque_limit);
void set_speed_adaptive(uint8_t id_num, float speed_adaptive);
void set_torque_adaptive(uint8_t id_num, float torque_adaptive);
void set_pid(uint8_t id_num, float P, float I, float D);
void set_mode(uint8_t id_num, int mode);
void set_can_baud_rate(uint8_t id_num, int baud_rate);
void write_property(uint8_t id_num,unsigned short param_address,int8_t param_type,float value);
void save_config(uint8_t id_num);

/* 辅助功能 */
void reboot(uint8_t id_num);
void init_config(uint8_t id_num);
#endif /* __DrEmpower_can__ */
