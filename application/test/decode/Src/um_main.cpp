
#include "usermode.hpp"
#include "controller.hpp"
#include <cstdio>
#include <cstring>

using namespace std;

float mpu[3] = {0,0,0};
float Me02_Data[3]={0,0,0};

Controller CusControl = Controller(Me02_Data, mpu);

Attitue Current;

// 每帧执行
void Update(float *encoder_Data, float *eul_rec)
{
    memcpy(Me02_Data,encoder_Data,12);
    memcpy(mpu,eul_rec,12);

    static unsigned char flag = 1;
    if(flag){
        CusControl.UpdateAttitue();
        CusControl.UpdateAttitue();
        CusControl.ZeroController();
        flag = 0;
    }
    
    // todo： 重新设置零点
    // if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET) {
    //     Zero_Me02_Data();
    //     CusControl.ZeroController();
    // }

    CusControl.UpdateAttitue();
    Current = CusControl.GetAttitue();
    // printf("x= %0.5f,y= %0.5f,z= %0.5f,pitch= %0.5f,roll= %0.5f ,yaw= %0.5f\n",
    //        Current.Position.Get().x, Current.Position.Get().y, Current.Position.Get().z,
    //        Current.EularAngle.Get().x, Current.EularAngle.Get().y, Current.EularAngle.Get().z);
    // printf("%0.5f,%0.5f,%0.5f,0\n",
    //        Current.EularAngle.Get().x, Current.EularAngle.Get().y, Current.EularAngle.Get().z);

    // HAL_UART_Transmit(&huart1, (uint8_t *)a, 300, 3);
}