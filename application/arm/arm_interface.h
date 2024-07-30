#ifndef ARM_INTERFACE_H
#define ARM_INTERFACE_H

#define assorted_up_encoder_offset  0
#define assorted_yaw_encoder_offset 1
#define tail_motor_encoder_offset   2
#define big_yaw_encoder_offset   3

#define big_yaw_speed_limit         80
#define z_speed_limit               15000
#define mid_yaw_speed_limit          20
#define assorted_yaw_speed_limit        15000//20000 //11000
#define assorted_roll_speed_limit        15000//40000 //30000
#define tail_motor_speed_limit      50000
#define tail_roll_speed_limit       15000

#define big_yaw_reduction_ratio 0.00105805702f
#define mid_yaw_reduction_ratio 0.006485538f
#define assorted_yaw_reduction_ratio 0.0000068093846f
#define assorted_roll_reduction_ratio 0.000003205588f
#define tail_motor_reduction_ratio 0.00000140197421f
#define z_motor_reduction_ratio 0.0000283f

//期望目标速度单位 °/s
#define DRmotor_speed_ratio         1
#define MT6825_speed_ratio          1
#define big_yaw_speed_ratio         DRmotor_speed_ratio
#define mid_yaw_speed_ratio         DRmotor_speed_ratio
#define assorted_yaw_speed_ratio    MT6825_speed_ratio
#define assorted_roll_speed_ratio   MT6825_speed_ratio
#define tail_motor_speed_ratio      MT6825_speed_ratio

#define z_motor_ReductionRatio      32.91f//46.185567f  



void ArmInit_Encoder();
void ArmInit_Motor();
void ArmInit_Communication();
void ArmInit_IO();
void ArmInit_Param();

void ArmSubMessage();
void ArmParamPretreatment();
void ArmControInterface();
void ArmCommunicateHOST();
void ArmPubMessage();
void ArmDebugInterface();
#endif // ARM_INTERFACE_H