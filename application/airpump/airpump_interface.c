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

//推杆操作宏定义
#define CLAW_UP() {airvalve_tx_coder&=~(0x00000001<<18);airvalve_tx_coder|=0x00000001<<14;}
#define CLAW_TIGHTEN() {airvalve_tx_coder&=~(0x00000001<<19);airvalve_tx_coder|=0x00000001<<15;}
#define CLAW_LOOSE() {airvalve_tx_coder&=~(0x00000001<<15);airvalve_tx_coder|=0x00000001<<19;}
#define CLAW_FORWARD() {airvalve_tx_coder&=~(0x00000001<<14);airvalve_tx_coder|=0x00000001<<18;}

static CANInstance *airvalve_can_instance;
static GPIOInstance *airpump_linear,*airpump_arm;
static DJIMotorInstance *air_sucker_motor;
static Subscriber_t *airpump_cmd_data_sub;// 气阀/气泵控制信息收
static Airpump_Cmd_Data_s airpump_cmd_rec;

static float sucker_zero_angle;
static uint8_t sucker_motor_init_flag = 0;
static uint16_t sucker_motor_init_cnt = 0;
static uint8_t  sucker_motor_mode = 0;

static uint8_t airvalve_tx_buf[8] = {0x11,0x88,0x99,0x00,0x00,0x00,0x00,0x00};
static uint32_t airvalve_tx_coder = 0;
static uint16_t airpump_state = 0;
static uint16_t airvalve_state = 0;
static uint8_t airvalve_mode_cnt = 0;
static int16_t airvalve_delay_time = 0;
static uint8_t airvalve_state_in_auto_mode = 0;

//气泵状态 - 全局变量
uint8_t airpump_arm_state=0;
uint8_t airpump_linear_state=0;

static Publisher_t *air_data_pub;        // 气阀气泵消息发布者
static Airpump_Data_s air_data_send;    // 气阀气泵消息
void AirpumpInit_IO()
{
    GPIO_Init_Config_s gpio_conf_airpump_arm = {
        .GPIOx = airpump_arm_GPIO_Port,
        .GPIO_Pin = airpump_arm_Pin,
    };
    GPIO_Init_Config_s gpio_conf_airpump_linear = {
        .GPIOx = airpump_linear_GPIO_Port,
        .GPIO_Pin = airpump_linear_Pin,
    };

    airpump_linear = GPIORegister(&gpio_conf_airpump_linear);
    airpump_arm = GPIORegister(&gpio_conf_airpump_arm);
}
void AirpumpInit_Motor()
{
    Motor_Init_Config_s air_sucker_motor_config = {
        .can_init_config.can_handle   = &hfdcan1,
        .can_init_config.tx_id  = 1,
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
                .MaxOut        = 8000,
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

    air_sucker_motor = DJIMotorInit(&air_sucker_motor_config);
}
void AirpumpInit_Communication()
{
    CAN_Init_Config_s can_conf_airvalve = {
        .can_handle = &hfdcan2,
        .tx_id = 0x121
    };
    airvalve_can_instance = CANRegister(&can_conf_airvalve);

    airpump_cmd_data_sub = SubRegister("airpump_cmd", sizeof(Airpump_Cmd_Data_s));

    air_data_pub = PubRegister("air_data", sizeof(Airpump_Data_s));
}
void AirpumpInit_Param()
{
    // 推杆初始状态
    airvalve_tx_coder = 0b0000001101100101101001;
    memcpy(airvalve_tx_buf+3,(uint8_t*)&airvalve_tx_coder,4);
    airvalve_tx_buf[7]=(uint8_t)(airvalve_tx_buf[3] + airvalve_tx_buf[4] + airvalve_tx_buf[5] + airvalve_tx_buf[6]);
    memcpy(airvalve_can_instance->tx_buff,airvalve_tx_buf,8);
    CANTransmit(airvalve_can_instance,1);
    // 吸盘电机初始化请求标志
    sucker_motor_mode = 3;
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
    sucker_motor_mode = 3;
}

void AirpumpSubMessage()
{
    while(!SubGetMessage(airpump_cmd_data_sub, &airpump_cmd_rec));
}
void AirpumpParamPretreatment()
{
    //若机器人死亡，需重新初始化吸盘电机-命令由cmd发出
    if(airpump_cmd_rec.init_call == 1){
        airsucker_motor_init();
    }
}
void AirpumpContro_Valve()
{
    /* 推杆气路控制 */
    /*气推杆取矿是一个连续的过程，为防止中途切换模式出现不可知的错误，我们规定：
    用变量airvalve_mode_cnt标示当前推杆整个流程中的第n个步骤
    用变量airvalve_state标示当前状态，其低四位指示当前模式(0x0F)，高四位指示完成标志(0xF0)
    其中0001标示左侧矿，0010标示中间矿，0100作保留位，1000作发送准备位
    对于延时位，已知该任务以1000Hz频率运行，另设变量airvalve_delay_time标示推杆每个动作的间隔时间，通过if-else实现延时
    具体流程直接看代码*/
    //取左侧矿
    if((!(airvalve_state&(AIRVALVE_MIDDLE_CUBE_DOING))  &&  ((airpump_cmd_rec.airvalve_mode&AIRVALVE_LEFT_CUBE))) || (airvalve_state&AIRVALVE_LEFT_CUBE_DOING)){ //(过程未完成&&未进行其他推杆任务)&&(cmd切换为该模式||模式进行标志位置位)
        if(airvalve_mode_cnt <= 10){//共10个步骤
            airvalve_state |= AIRVALVE_LEFT_CUBE_DOING; //设置进行标志位，防\\\止中途模式被切换
            airvalve_state&=~AIRVALVE_MIDDLE_CUBE_DONE; // 状态切换，清相关标志位
            if(airvalve_delay_time <= 0)
            {
                airvalve_mode_cnt++;
                airvalve_state |= AIRVALVE_SEND_CODER;
                switch(airvalve_mode_cnt){
                    //                          0b2X098765X3210987654321
                    case 1: airvalve_tx_coder = 0b0000001101100101101001;airvalve_delay_time = 300;airsucker_forward();airvalve_state_in_auto_mode=0;break;//初始状态
                    case 2: airvalve_tx_coder = 0b0000001101100101010110;airvalve_delay_time = 300;break;//下降+第一段前伸
                    case 3: airvalve_tx_coder = 0b0000001101100110010110;airvalve_delay_time = 300;break;//旋转
                    case 4: airvalve_tx_coder = 0b1000001100011010010110;airvalve_delay_time = 1200;airpump_state|=0x02;airvalve_state_in_auto_mode=1;airpump_linear_state=1;break;//伸出
                    case 5: airvalve_tx_coder = 0b0000001101011010010110;airvalve_delay_time = 300;break;//小回
                    case 6: airvalve_tx_coder = 0b0000001101011010010101;airvalve_delay_time = 300;airvalve_state&=~AIRVALVE_LEFT_CUBE_DOING;break;//抬升
                    case 7: airvalve_tx_coder = 0b0000001100100110010101;airvalve_delay_time = 300;break;//缩回
                    case 8: airvalve_tx_coder = 0b0000001100100101010101;airvalve_delay_time = 300;break;//旋转
                    case 9: airvalve_tx_coder = 0b0000001100100101101001;airvalve_delay_time = 300;break;//第一段缩回
                    case 10:airvalve_tx_coder = 0b0000001101100101101001;airvalve_delay_time = 300;airsucker_up();airvalve_state&=~AIRVALVE_LEFT_CUBE_DOING;airvalve_state_in_auto_mode=0;break;//初始状态
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
    if((!(airvalve_state&(AIRVALVE_LEFT_CUBE_DOING))  &&  ((airpump_cmd_rec.airvalve_mode&AIRVALVE_MIDDLE_CUBE))) || (airvalve_state&AIRVALVE_MIDDLE_CUBE_DOING)){ //(过程未完成&&未进行其他推杆任务)&&(cmd切换为该模式||模式进行标志位置位)
        if(airvalve_mode_cnt <= 11){//共11个步骤
            airvalve_state |= AIRVALVE_MIDDLE_CUBE_DOING; //设置进行标志位，防止中途模式被切换
            airvalve_state&=~AIRVALVE_LEFT_CUBE_DONE; // 状态切换，清相关标志位
            if(airvalve_delay_time <= 0)
            {
                airvalve_mode_cnt++;
                airvalve_state |= AIRVALVE_SEND_CODER;

                switch(airvalve_mode_cnt){
                    //                          0b2X098765X3210987654321
                    case 1: airvalve_tx_coder = 0b0000001101100101101001;airvalve_delay_time = 300;airsucker_forward();airvalve_state_in_auto_mode=0;break;//初始状态
                    case 2: airvalve_tx_coder = 0b1000001100011001100110;airvalve_delay_time = 1500;airpump_state|=0x02;airvalve_state_in_auto_mode=1;airpump_linear_state=1;break;//伸出+下降
                    case 3: airvalve_tx_coder = 0b0000001101011001100110;airvalve_delay_time = 300;break;//小回
                    case 4: airvalve_tx_coder = 0b0000001101011001100101;airvalve_delay_time = 1200;airvalve_state&=~AIRVALVE_MIDDLE_CUBE_DOING;break;//抬升
                    case 5: airvalve_tx_coder = 0b0000001101100101101001;airvalve_delay_time = 300;break;//缩回
                    case 6: airvalve_tx_coder = 0b0000001101100101101001;airvalve_delay_time = 1200;break;//初始状态
                    case 7: airvalve_tx_coder = 0b0011000001100101101001;airvalve_delay_time = 600;airsucker_up();break;//夹爪前伸
                    case 8: airvalve_delay_time = 4000;break;//上抬
                    case 9: airvalve_tx_coder = 0b0001001001100101101001;airvalve_delay_time = 1000;airpump_state&=~0x02;airvalve_state_in_auto_mode=0;airpump_linear_state=0;break;//夹爪夹
                    case 10: airvalve_tx_coder =0b0000001101100101101001;airvalve_delay_time = 300;break;//夹爪后移
                    case 11: airvalve_tx_coder =0b0000001101100101101001;airvalve_delay_time = 300;airvalve_state&=~AIRVALVE_MIDDLE_CUBE_DOING;break;//初始状态
                }
            }else{
                airvalve_delay_time--;//计数-1，延时1ms
            }
        }else{
            airvalve_mode_cnt = 0;  //复位步骤号
            airvalve_state &= ~AIRVALVE_MIDDLE_CUBE_DOING;//复位进行标志
            airvalve_state |= AIRVALVE_MIDDLE_CUBE_DONE; //置位完成标志
        }
    }else if(airpump_cmd_rec.arm_to_airvalve){
        //由臂臂控制夹爪，优先级低
        //todo：任务执行顺次需再缕一缕
        switch(airpump_cmd_rec.arm_to_airvalve){
            case AIRVALVE_CLAW_UP:  CLAW_UP();break;
            case AIRVALVE_CLAW_FOREWARD:    CLAW_FORWARD();break;
            case AIRVALVE_CLAW_LOOSE: CLAW_LOOSE();break;
            case AIRVALVE_CLAW_TIGHTEN: CLAW_TIGHTEN();break;
        }
        airvalve_state |= AIRVALVE_SEND_CODER;
    }

    // 发送部分，发送标志位(0x80)置位 或者 计时器到达1000后才会发送
    // 由该条件，发送动作在状态切换的时候才会进行
    static uint16_t send_time_cnt = 0;
    send_time_cnt++;
    if((airvalve_state & AIRVALVE_SEND_CODER) || send_time_cnt>=1000){
        memcpy(airvalve_tx_buf+3,(uint8_t*)&airvalve_tx_coder,4);
        airvalve_tx_buf[7]=(uint8_t)(airvalve_tx_buf[3] + airvalve_tx_buf[4] + airvalve_tx_buf[5] + airvalve_tx_buf[6]);
        memcpy(airvalve_can_instance->tx_buff,airvalve_tx_buf,8);
        CANTransmit(airvalve_can_instance,1);
        airvalve_state&=~AIRVALVE_SEND_CODER;
        send_time_cnt = 0;
    }
}
void AirpumpContro_Pump()
{
    /* 气泵控制 */
    // static uint8_t switch_flag = 0;
    //操作手控制臂气泵
    // if(airpump_cmd_rec.airpump_mode & Airpump_ARM_OPEN && !(switch_flag&0x01)){
    //     (airpump_state & 0x01) ? (airpump_state&=~0x01) : (airpump_state|=0x01);
    //     switch_flag |= 0x01;
    // }else if(!(airpump_cmd_rec.airpump_mode & Airpump_ARM_OPEN)){
    //     switch_flag &= ~0x01;
    // } 
    // if(airpump_cmd_rec.airpump_mode & Airpump_LINEAR_OPEN && !(switch_flag&0x02)){
    //     (airpump_state & 0x02) ? (airpump_state&=~0x02) : (airpump_state|=0x02);
    //     switch_flag |= 0x02;
    // }else if(!(airpump_cmd_rec.airpump_mode & Airpump_LINEAR_OPEN)){
    //     switch_flag &= ~0x02;
    // }  

    // if(airpump_state & Airpump_ARM_OPEN || airpump_cmd_rec.arm_to_airpump & Airpump_ARM_OPEN){
    //     air_data_send.pump_state |= 0x01;
    //     // GPIOReset(airpump_arm); //气泵拉低为开
    // }
    // else if(!(airpump_state & Airpump_ARM_OPEN) || airpump_cmd_rec.arm_to_airpump & Airpump_ARM_CLOSE){
    //     air_data_send.pump_state &= ~0x01;
    //     // GPIOSet(airpump_arm);
    // }
    // if(airpump_state & Airpump_LINEAR_OPEN || airpump_cmd_rec.arm_to_airpump & Airpump_LINEAR_OPEN){
    //     air_data_send.pump_state |= 0x02;
    //     // GPIOReset(airpump_linear); //气泵拉低为开
    // }else{
    //     air_data_send.pump_state &= ~0x02;
    //     // GPIOSet(airpump_linear);
    // }

    // if(airpump_cmd_rec.airpump_mode & 0x01){
    //     if(air_data_send.pump_state & 0x01){
    //         air_data_send.pump_state &=~0x01;
    //         // GPIOSet(airpump_arm);
    //     }else{
    //         air_data_send.pump_state |= 0x01;
    //         // GPIOReset(airpump_arm);
    //     }
    // }
    
    // if(airpump_cmd_rec.airpump_mode & 0x02){
    //     if(air_data_send.pump_state & 0x02){
    //         air_data_send.pump_state &=~0x02;
    //         // GPIOSet(airpump_linear);
    //     }else{
    //         air_data_send.pump_state |= 0x02;
    //         // GPIOReset(airpump_linear);
    //     }
    // }

    // if(air_data_send.pump_state & 0x01)
    //     GPIOReset(airpump_arm);
    // else
    //     GPIOSet(airpump_arm);
    // if(air_data_send.pump_state & 0x02)
    //     GPIOReset(airpump_linear);
    // else    
    //     GPIOSet(airpump_linear);

    if(airpump_arm_state == 1)
        GPIOReset(airpump_arm);
    else
        GPIOSet(airpump_arm);
    if(airpump_linear_state == 1)
        GPIOReset(airpump_linear);
    else    
        GPIOSet(airpump_linear);
    // static int debug_v;
    // if(debug_v){
    //     switch(debug_v){
    //         case AIRVALVE_CLAW_UP:  CLAW_UP();break;
    //         case AIRVALVE_CLAW_FOREWARD:    CLAW_FORWARD();break;
    //         case AIRVALVE_CLAW_LOOSE: CLAW_LOOSE();break;
    //         // case AIRVALVE_CLAW_LOOSE:
    //         //     airvalve_tx_coder&=~(0x00000001<<15);
    //         //     airvalve_tx_coder|=0x00000001<<19;
    //         //     break;
    //         case AIRVALVE_CLAW_TIGHTEN: CLAW_TIGHTEN();break;
    //     }
    //     airpump_state |= AIRVALVE_SEND_CODER;
    // }
}
void AirpumpContro_Sucker()
{
    /* 推杆吸盘电机控制 */
    if(sucker_motor_mode == 1){ // up
        static uint16_t sucker_stuck_cnt = 0;
        if(abs(air_sucker_motor->measure.speed_aps) < abs(air_sucker_motor->motor_controller.speed_PID.Ref)-200) sucker_stuck_cnt++;
        else sucker_stuck_cnt = 0;
        if(sucker_stuck_cnt > 1000){
            air_sucker_motor->measure.total_round = 0;
            air_sucker_motor->measure.last_ecd = air_sucker_motor->measure.ecd;
            air_sucker_motor->measure.total_angle = air_sucker_motor->measure.angle_single_round;
            sucker_zero_angle = air_sucker_motor->measure.total_angle;
            sucker_stuck_cnt = 0;
        }

        DJIMotorEnable(air_sucker_motor);
        DJIMotorOuterLoop(air_sucker_motor,ANGLE_LOOP);
        DJIMotorSetRef(air_sucker_motor,sucker_zero_angle);
    }
    if(sucker_motor_mode == 2 || airpump_cmd_rec.tmp_flag==1){ //forward
        DJIMotorEnable(air_sucker_motor);
        DJIMotorOuterLoop(air_sucker_motor,ANGLE_LOOP);
        DJIMotorSetRef(air_sucker_motor,sucker_zero_angle+3408+airpump_cmd_rec.sucker_offset_angle);
    }else if(sucker_motor_mode == 3 && !(airpump_cmd_rec.airvalve_mode & AIRVALVE_STOP)){ //init
        DJIMotorEnable(air_sucker_motor);
        if(sucker_motor_init_cnt > 1000){
            if(sucker_motor_init_flag==0){
                sucker_motor_init_flag=1;
                air_sucker_motor->measure.total_round = 0;
                air_sucker_motor->measure.last_ecd = air_sucker_motor->measure.ecd;
                air_sucker_motor->measure.total_angle = air_sucker_motor->measure.angle_single_round;
                sucker_zero_angle = air_sucker_motor->measure.total_angle+20;
            }
            DJIMotorOuterLoop(air_sucker_motor,ANGLE_LOOP);
            DJIMotorSetRef(air_sucker_motor,sucker_zero_angle);
            sucker_motor_mode = 1;
        }else{
            DJIMotorOuterLoop(air_sucker_motor,SPEED_LOOP);
            DJIMotorSetRef(air_sucker_motor,-3000);
            if(air_sucker_motor->measure.speed_aps > -500)   sucker_motor_init_cnt++;
            else sucker_motor_init_cnt = 0;
        }
    }

    if(airpump_cmd_rec.airpump_mode & AIRPUMP_STOP){
        air_data_send.pump_state &= ~0x03;
        GPIOSet(airpump_arm);
        GPIOSet(airpump_linear);
    }
    if(airpump_cmd_rec.airvalve_mode & AIRVALVE_STOP){
        DJIMotorStop(air_sucker_motor);
    }
}
void AirpumpEmergencyHandler()
{
    //强制暂停
    if(airpump_cmd_rec.halt_force_call == 1){
        airvalve_state = 0;
        airvalve_mode_cnt = 0;
    }
    //临时暂停
    static uint16_t airvalve_state_log;
    static uint8_t halt_temp_switch_flag = 0;
    if(airvalve_state)   airvalve_state_log = 0;
    airvalve_state_log = airvalve_state;
    if(airpump_cmd_rec.halt_temp_call == 1 && !halt_temp_switch_flag){
        if(airvalve_state_log){
            airvalve_state = airvalve_state_log;
            airvalve_state_log = 0;
        }else{
            airvalve_state_log = airvalve_state;
            airvalve_state = 0;
        }
        airvalve_state = 0;
        halt_temp_switch_flag = 1;
    }if(!(airpump_cmd_rec.halt_temp_call == 1)) halt_temp_switch_flag = 0;
}
void AirpumpPubMessage()
{
    PubPushMessage(air_data_pub,&air_data_send);
}
void AirpumpDebugInterface()
{
    ;
}