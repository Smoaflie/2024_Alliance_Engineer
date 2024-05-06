// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "led.h"
#include "dji_motor.h"
#include "airpump.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_can.h"
#include "bsp_gpio.h"

static CANInstance *airvalve_can_instance;
static GPIOInstance *airpump_linear,*airpump_arm;
static DJIMotorInstance *air_sucker_motor;
static Subscriber_t *airpump_cmd_data_sub;// 气阀/气泵控制信息收
static Airpump_Cmd_Data_s airpump_cmd_rec;

static float sucker_zero_angle;
static uint8_t sucker_motor_init_flag = 0;
static uint16_t sucker_motor_init_cnt = 0;
static uint8_t  sucker_motor_mode = 0;
void AIRPUMPInit()
{
    GPIO_Init_Config_s gpio_conf_airpump_linear = {
        .GPIOx = airpump_linear_GPIO_Port,
        .GPIO_Pin = airpump_linear_Pin,
    };
    GPIO_Init_Config_s gpio_conf_airpump_arm = {
        .GPIOx = airpump_arm_GPIO_Port,
        .GPIO_Pin = airpump_arm_Pin,
    };
    Motor_Init_Config_s air_sucker_motor_config = {
        .can_init_config.can_handle   = &hfdcan2,
        .can_init_config.tx_id  = 4,
        .controller_param_init_config = {
            .torque_PID = {
                .Kp            = 0, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 50,
                .MaxOut        = 10,
            },
            .angle_PID = {
                .Kp            = 10, // 0
                .Ki            = 0,    // 0
                .Kd            = 0,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 50,
                .MaxOut        = 10000,
            },
            .speed_PID = {
                .Kp            = 1, // 4.5
                .Ki            = 0, // 0
                .Kd            = 0.001, // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP | TORQUE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    CAN_Init_Config_s can_conf_airvalve = {
        .can_handle = &hfdcan2,
        .tx_id = 0x121
    };

    airpump_linear = GPIORegister(&gpio_conf_airpump_linear);
    airpump_arm = GPIORegister(&gpio_conf_airpump_arm);

    air_sucker_motor = DJIMotorInit(&air_sucker_motor_config);

    airvalve_can_instance = CANRegister(&can_conf_airvalve);

    airpump_cmd_data_sub = SubRegister("airpump_cmd", sizeof(Airpump_Cmd_Data_s));
}

//吸盘朝前
static void airsucker_forward(){
    sucker_motor_mode = 2;
}
//吸盘朝上
static void airsucker_up(){
    sucker_motor_mode = 1;
}
//吸盘电机初始化
static void airsucker_motor_init(){
    sucker_motor_init_flag= 0;
    sucker_motor_init_cnt = 0;
}

/* 机器人气阀气泵控制核心任务 */
void AIRPUMPTask()
{
    while(!SubGetMessage(airpump_cmd_data_sub, &airpump_cmd_rec));

    static uint8_t airvalve_tx_buf[8] = {0x11,0x88,0x99,0x00,0x00,0x00,0x00,0x00};
    static uint32_t airvalve_tx_coder = 0;
    static uint16_t airvalve_state = 0;
    static uint8_t airvalve_mode_cnt = 0;
    static int16_t airvalve_delay_time = 0;
    
    //todo:我感觉用define字符串表示0xnn更难看……

    /*
    airpump_cmd_rec.mode
        低位0 -> 8
        0 0x01      1 0x02      2  0x04        3  0x08      ……
        左气泵开关 | 右气泵开关 | 气动取矿状态1 | 气动取矿状态2 ……
    */
    if(airpump_cmd_rec.mode & AIRPUMP_ARM_OPEN){
        GPIOReset(airpump_arm); //气泵拉低为开
    }else{
        GPIOSet(airpump_arm);
    }

    if(airpump_cmd_rec.mode & AIRPUMP_LINEAR_OPEN){
        GPIOReset(airpump_linear);
    }else{
        GPIOSet(airpump_linear);
    }

    /*气推杆取矿是一个连续的过程，为防止中途切换模式出现不可知的错误，我们规定：
    用变量airvalve_mode_cnt标示当前推杆整个流程中的第n个步骤
    用变量airvalve_state标示当前状态，其低四位指示当前模式(0x0F)，高四位指示完成标志(0xF0)
    其中0001标示左侧矿，0010标示中间矿，0100作保留位，1000作发送准备位
    对于延时位，已知该任务以1000Hz频率运行，另设变量airvalve_delay_time标示推杆每个动作的间隔时间，通过if-else实现延时
    具体流程直接看代码*/
    //取左侧矿
    if(!(airvalve_state&(AIRVALVE_LEFT_CUBE_DONE|AIRVALVE_MIDDLE_CUBE_DOING))  &&  ((airpump_cmd_rec.mode&AIRVALVE_LEFT_CUBE) || (airvalve_state&AIRVALVE_LEFT_CUBE_DOING))){ //(过程未完成&&未进行其他推杆任务)&&(cmd切换为该模式||模式进行标志位置位)
        if(airvalve_mode_cnt <= 10){//共9个步骤
            airvalve_state |= AIRVALVE_LEFT_CUBE_DOING; //设置进行标志位，防止中途模式被切换
            airvalve_state&=~AIRVALVE_MIDDLE_CUBE_DONE; // 状态切换，清相关标志位
            if(airvalve_delay_time <= 0)
            {
                airvalve_mode_cnt++;
                airvalve_state |= AIRVALVE_SEND_CODER;
                switch(airvalve_mode_cnt){
                    //                          0b2X098765X3210987654321
                    case 1: airvalve_tx_coder = 0b0000000001100101101010;airvalve_delay_time = 2000;airsucker_up();break;
                    case 2: airvalve_tx_coder = 0b0000000001100101010110;airvalve_delay_time = 600;airsucker_forward();break;
                    case 3: airvalve_tx_coder = 0b0000000001100110010110;airvalve_delay_time = 600;break;
                    case 4: airvalve_tx_coder = 0b1000000000011010010110;airvalve_delay_time = 600;break;
                    case 5: airvalve_tx_coder = 0b1000000000011010010101;airvalve_delay_time = 600;break;
                    case 6: airvalve_tx_coder = 0b0000000000100110010101;airvalve_delay_time = 600;break;
                    case 7: airvalve_tx_coder = 0b0000000000100101010101;airvalve_delay_time = 600;break;
                    case 8: airvalve_tx_coder = 0b0000000000100101101001;airvalve_delay_time = 600;break;
                    case 9: airvalve_tx_coder = 0b0000000001100101101010;airvalve_delay_time = 600;break;
                    default: airvalve_tx_coder =0b0000000001100101101010;airsucker_up();
                }
            }else{
                airvalve_delay_time--;//计数-1，延时1ms
            }
        }else{
            airvalve_mode_cnt = 0;  //复位步骤号
            airvalve_state &= ~AIRVALVE_LEFT_CUBE_DOING;//复位进行标志
            airvalve_state |= AIRVALVE_LEFT_CUBE_DONE; //置位完成标志
        }
    }
    //取中间矿
    if(!(airvalve_state&(AIRVALVE_LEFT_CUBE_DOING|AIRVALVE_MIDDLE_CUBE_DONE))  &&  ((airpump_cmd_rec.mode&AIRVALVE_MIDDLE_CUBE) || (airvalve_state&AIRVALVE_MIDDLE_CUBE_DOING))){ //(过程未完成&&未进行其他推杆任务)&&(cmd切换为该模式||模式进行标志位置位)
        if(airvalve_mode_cnt <= 6){//共9个步骤
            airvalve_state |= AIRVALVE_MIDDLE_CUBE_DOING; //设置进行标志位，防止中途模式被切换
            airvalve_state&=~AIRVALVE_LEFT_CUBE_DONE; // 状态切换，清相关标志位
            if(airvalve_delay_time <= 0)
            {
                airvalve_mode_cnt++;
                airvalve_state |= AIRVALVE_SEND_CODER;

                switch(airvalve_mode_cnt){
                    //                          0b2X098765X3210987654321
                    case 1: airvalve_tx_coder = 0b0000000001100101101010;airvalve_delay_time = 2000;airsucker_up();break;//归位
                    case 2: airvalve_tx_coder = 0b1000000000011001100110;airvalve_delay_time = 1300;airsucker_forward();break;//伸出
                    case 3: airvalve_tx_coder = 0b0000000001011001100110;airvalve_delay_time = 850;break;//小回
                    case 4: airvalve_tx_coder = 0b0000000001011001100101;airvalve_delay_time = 1500;break;//抬升
                    case 5: airvalve_tx_coder = 0b0000000001100101101001;airvalve_delay_time = 1500;break;//缩回
                    case 6: airvalve_tx_coder = 0b0000000001100101101010;airvalve_delay_time = 600;break;
                    default: airvalve_tx_coder =0b0000000001100101101010;airsucker_up();
                }
            }else{
                airvalve_delay_time--;//计数-1，延时1ms
            }
        }else{
            airvalve_mode_cnt = 0;  //复位步骤号
            airvalve_state &= ~AIRVALVE_MIDDLE_CUBE_DOING;//复位进行标志
            airvalve_state |= AIRVALVE_MIDDLE_CUBE_DONE; //置位完成标志
        }
    }

    // 发送部分，发送标志位(0x80)置位后才会发送
    // 由该条件，发送动作在状态切换的时候才会进行
    if(airvalve_state & AIRVALVE_SEND_CODER){
        memcpy(airvalve_tx_buf+3,(uint8_t*)&airvalve_tx_coder,4);
        airvalve_tx_buf[7]=(uint8_t)(airvalve_tx_buf[3] + airvalve_tx_buf[4] + airvalve_tx_buf[5] + airvalve_tx_buf[6]);
        memcpy(airvalve_can_instance->tx_buff,airvalve_tx_buf,8);
        CANTransmit(airvalve_can_instance,1);
        airvalve_state&=~AIRVALVE_SEND_CODER;
    }

    if(sucker_motor_mode == 1){ // up

        DJIMotorEnable(air_sucker_motor);
        if(sucker_motor_init_cnt > 50){
            if(sucker_motor_init_flag==0){
                sucker_motor_init_flag=1;
                air_sucker_motor->measure.total_round = 0;
                sucker_zero_angle = air_sucker_motor->measure.angle_single_round>180?(air_sucker_motor->measure.angle_single_round-360):air_sucker_motor->measure.angle_single_round;
            }
            DJIMotorOuterLoop(air_sucker_motor,ANGLE_LOOP);
            DJIMotorSetRef(air_sucker_motor,sucker_zero_angle);
        }else{
            DJIMotorOuterLoop(air_sucker_motor,SPEED_LOOP);
            DJIMotorSetRef(air_sucker_motor,-8000);
            if(air_sucker_motor->measure.speed_aps > -500)   sucker_motor_init_cnt++;
            else sucker_motor_init_cnt = 0;
        }

    }else if(sucker_motor_mode == 2){ //forward
        DJIMotorEnable(air_sucker_motor);
        DJIMotorOuterLoop(air_sucker_motor,ANGLE_LOOP);
        DJIMotorSetRef(air_sucker_motor,sucker_zero_angle+3600);
    }

    if(airpump_cmd_rec.mode & AIRPUMP_SUCKER_ALL_STOP){
        DJIMotorStop(air_sucker_motor);
        GPIOSet(airpump_arm);
        GPIOSet(airpump_linear);
    }
}
