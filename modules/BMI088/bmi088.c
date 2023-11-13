#include "bmi088_regNdef.h"
#include "bmi088.h"
#include "user_lib.h"
#include "buzzer.h"
#include "robot_def.h"

// ---------------------------以下私有函数,用于读写BMI088寄存器封装,blocking--------------------------------//
/**
 * @brief 读取BMI088寄存器Accel. BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg 待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
static void BMI088AccelRead(BMI088Instance *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
    if (len > 6)
        while (1)
            ;
    // 一次读取最多6个字节,加上两个dummy data    第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
    static uint8_t tx[8]; // 读取,第一个字节为0x80|reg ,第二个是dummy data,后面的没用都是dummy write
    static uint8_t rx[8]; // 前两个字节是dummy data,第三个开始是真正的数据
    tx[0] = 0x80 | reg;   // 静态变量每次进来还是上次的值,所以要每次都要给tx[0]赋值0x80
    SPITransRecv(bmi088->spi_acc, rx, tx, len + 2);
    memcpy(dataptr, rx + 2, len); // @todo : memcpy有额外开销,后续可以考虑优化,在SPI中加入接口或模式,使得在一次传输结束后不释放CS,直接接着传输
}

/**
 * @brief 读取BMI088寄存器Gyro, BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg  待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
static void BMI088GyroRead(BMI088Instance *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
    if (len > 6)
        while (1)
            ;
    // 一次读取最多6个字节,加上一个dummy data  ,第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
    static uint8_t tx[7] = {0x80}; // 读取,第一个字节为0x80 | reg ,之后是dummy data
    static uint8_t rx[7];          // 第一个是dummy data,第三个开始是真正的数据
    tx[0] = 0x80 | reg;
    SPITransRecv(bmi088->spi_gyro, rx, tx, len + 1);
    memcpy(dataptr, rx + 1, len); // @todo : memcpy有额外开销,后续可以考虑优化,在SPI中加入接口或模式,使得在一次传输结束后不释放CS,直接接着传输
}

/**
 * @brief 写accel寄存器.对spitransmit形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
static void BMI088AccelWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    SPITransmit(bmi088->spi_acc, tx, 2);
}

/**
 * @brief 写gyro寄存器.形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
static void BMI088GyroWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    SPITransmit(bmi088->spi_gyro, tx, 2);
}
// -------------------------以上为私有函数,封装了BMI088寄存器读写函数,blocking--------------------------------//

// -------------------------以下为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//
#define BMI088REG   0
#define BMI088DATA  1
#define BMI088ERROR 2
// BMI088初始化配置数组for accel,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};
// BMI088初始化配置数组for gyro,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};
// @attention : 以上两个数组配合各自的初始化函数使用. 若要修改请参照BMI088 datasheet

/**
 * @brief 初始化BMI088加速度计,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE if any problems here
 */
static uint8_t BMI088AccelInit(BMI088Instance *bmi088)
{
    uint8_t WhoAmI_check = 0;

    // 加速度计以I2C模式启动,需要一次上升沿来切换到SPI模式,因此进行一次fake write
    BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1);
    DWT_Delay(0.001f);

    BMI088AccelWriteSingleReg(bmi088, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE); // 软复位
    DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME / 1000);

    // 检查ID,如果不是0x1E(bmi088 whoami寄存器值),则返回错误
    BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1);
    if (WhoAmI_check != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001f);
    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Accel_Init_Table) / sizeof(BMI088_Accel_Init_Table[0]); i++) {
        reg  = BMI088_Accel_Init_Table[i][BMI088REG];
        data = BMI088_Accel_Init_Table[i][BMI088DATA];
        BMI088AccelWriteSingleReg(bmi088, reg, data); // 写入寄存器
        DWT_Delay(0.01f);
        BMI088AccelRead(bmi088, reg, &data, 1); // 写完之后立刻读回检查
        DWT_Delay(0.01f);
        if (data != BMI088_Accel_Init_Table[i][BMI088DATA])
            error |= BMI088_Accel_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,如果retry次数用完了,则返回error
    }
    // 设置灵敏度
    switch (BMI088_Accel_Init_Table[3][1]) {
        case BMI088_ACC_RANGE_3G:
            bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
            break;
        case BMI088_ACC_RANGE_6G:
            bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
            break;
        case BMI088_ACC_RANGE_12G:
            bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_12G_SEN;
            break;
        case BMI088_ACC_RANGE_24G:
            bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_24G_SEN;
            break;
        default:
            break;
    }
    return (uint8_t)error;
}

/**
 * @brief 初始化BMI088陀螺仪,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE
 */
static uint8_t BMI088GyroInit(BMI088Instance *bmi088)
{
    // 后续添加reset和通信检查?
    // code to go here ...
    BMI088GyroWriteSingleReg(bmi088, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE); // 软复位
    DWT_Delay(0.08f);

    // 检查ID,如果不是0x0F(bmi088 whoami寄存器值),则返回错误
    uint8_t WhoAmI_check = 0;
    BMI088GyroRead(bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1);
    if (WhoAmI_check != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001f);

    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Gyro_Init_Table) / sizeof(BMI088_Gyro_Init_Table[0]); i++) {
        reg  = BMI088_Gyro_Init_Table[i][BMI088REG];
        data = BMI088_Gyro_Init_Table[i][BMI088DATA];
        BMI088GyroWriteSingleReg(bmi088, reg, data); // 写入寄存器
        DWT_Delay(0.001f);
        BMI088GyroRead(bmi088, reg, &data, 1); // 写完之后立刻读回对应寄存器检查是否写入成功
        DWT_Delay(0.001f);
        if (data != BMI088_Gyro_Init_Table[i][BMI088DATA])
            error |= BMI088_Gyro_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,尝试重新写入.如果retry次数用完了,则返回error
    }
    // 设置灵敏度
    switch (BMI088_Gyro_Init_Table[0][1]) {
        case BMI088_GYRO_2000:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
            break;
        case BMI088_GYRO_1000:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_1000_SEN;
            break;
        case BMI088_GYRO_500:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_500_SEN;
            break;
        case BMI088_GYRO_250:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_250_SEN;
            break;
        case BMI088_GYRO_125:
            bmi088->BMI088_GYRO_SEN = BMI088_GYRO_125_SEN;
            break;
        default:
            break;
    }
    return (uint8_t)error;
}
// -------------------------以上为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
void BMI088_temp_control(BMI088Instance *bmi088)
{
    static uint8_t temp_constant_time = 0;
    static uint8_t first_temperate    = 0; // 第一次达到设定温度
    static float target_temp          = 0;
    target_temp                       = bmi088->ambient_temperature + 10; // 推荐比环境温度高10度
    if (target_temp > 45.0f) target_temp = 45.0f;                         // 限制在45度以内

    if (first_temperate) {
        PIDCalculate(bmi088->heat_pid, bmi088->temperature, target_temp);
        // 限制在正数范围
        if (bmi088->heat_pid->Output < 0.0f) {
            bmi088->heat_pid->Output = 0.0f;
        }
        if(bmi088->heat_pid->Iout<0){
            bmi088->heat_pid->Iout = 0;
        }
        PWMSetDutyRatio(bmi088->heat_pwm, bmi088->heat_pid->Output);
    } else {
        // 在没有达到设置的温度-4，一直最大功率加热
        PWMSetDutyRatio(bmi088->heat_pwm, 0.95f);
        if (bmi088->temperature > target_temp - 4) {
            temp_constant_time++;
            if (temp_constant_time > 200) {
                // 达到设置温度，设置积分项，加速收敛
                first_temperate        = 1;
                bmi088->heat_pid->Iout = 0.05f;
            }
        }
    }
}

// -------------------------以下为私有函数,private用于IT模式下的中断处理---------------------------------//

static void BMI088AccSPIFinishCallback(SPIInstance *spi)
{
    static BMI088Instance *bmi088;
    static uint16_t callback_time = 0;
    bmi088                        = (BMI088Instance *)(spi->id);
    // 如果是加速度计的中断,则启动加速度计数据读取,并转换为实际值
    if (bmi088->update_flag.acc == 1) {
        for (uint8_t i = 0; i < 3; i++)
            bmi088->acc[i] = bmi088->BMI088_ACCEL_SEN * (float)(int16_t)(((bmi088->acc_raw[2 * i + 1]) << 8) | bmi088->acc_raw[2 * i]);
        bmi088->update_flag.acc = 0;
    }

    if (callback_time >= 500) {
        BMI088AccelRead(bmi088, BMI088_TEMP_M, bmi088->temp_raw, 2);
        int16_t temperate_raw_temp;
        temperate_raw_temp = (int16_t)((bmi088->temp_raw[0] << 3) | (bmi088->temp_raw[1] >> 5));
        if (temperate_raw_temp > 1023) temperate_raw_temp -= 2048;

        bmi088->temperature = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        // 设置环境温度
        if (bmi088->ambient_temperature < 0) {
            bmi088->ambient_temperature = bmi088->temperature;
        }
        BMI088_temp_control(bmi088);
        callback_time = 0;
    }
    callback_time++;
}
extern osThreadId_t instaskHandle; // 本来是全局变量直接extern
static void BMI088GyroSPIFinishCallback(SPIInstance *spi)
{
    static BMI088Instance *bmi088 = NULL;
    bmi088                        = (BMI088Instance *)(spi->id);
    // 将陀螺仪数据转换为实际值
    for (uint8_t i = 0; i < 3; i++)
        bmi088->gyro[i] = bmi088->BMI088_GYRO_SEN * (float)(int16_t)(((bmi088->gyro_raw[2 * i + 1]) << 8) | bmi088->gyro_raw[2 * i]);
    bmi088->update_flag.gyro = 0;
    // 由于SPI速率为10.5Mbit/s，超出了BMI088的10Mbit/s限制，所以有些陀螺仪数据会出错
    // 有尝试调低速率，但是这样加速度计中断在开机几秒后便无法触发，原因未知
    ///@todo 找到原因降低速率，使数据恢复正常，可能原因是初始化需要较低的速率，之后可以拉满
    // 下面的函数用于过滤错误数据,无法完全过滤
    // begin
    for (uint8_t i = 0; i < 3; i++) {
        if (fabsf(bmi088->last_gyro[i] - bmi088->gyro[i]) > 10.0f)
            return;
    }
    for (uint8_t i = 0; i < 3; i++) {
        bmi088->last_gyro[i] = bmi088->gyro[i];
    }
    // end
    bmi088->update_flag.imu_ready = 1;
    osThreadFlagsSet(instaskHandle, IMU_READY_FLAG); // 通知主线程IMU数据准备完毕（以陀螺仪中断为准 1000Hz）
}

static void BMI088AccINTCallback(GPIOInstance *gpio)
{
    static BMI088Instance *bmi088;
    bmi088 = (BMI088Instance *)(gpio->id);
    // 启动加速度计数据读取,并转换为实际值
    BMI088AccelRead(bmi088, BMI088_ACCEL_XOUT_L, bmi088->acc_raw, 6);
    bmi088->update_flag.acc = 1;
    // 读取完毕会调用BMI088AccSPIFinishCallback
}

static void BMI088GyroINTCallback(GPIOInstance *gpio)
{
    static BMI088Instance *bmi088 = NULL;
    bmi088                        = (BMI088Instance *)(gpio->id);
    // 启动陀螺仪数据读取,并转换为实际值
    BMI088GyroRead(bmi088, BMI088_GYRO_X_L, bmi088->gyro_raw, 6);
    bmi088->update_flag.gyro = 1;
    // 读取完毕会调用BMI088GyroSPIFinishCallback
}

// -------------------------以上为私有函数,private用于IT模式下的中断处理---------------------------------//

// -------------------------以下为私有函数,用于改变BMI088的配置--------------------------------//
static void BMI088SetMode(BMI088Instance *bmi088Instance, BMI088_Work_Mode_e mode)
{
    bmi088Instance->work_mode = mode;
    if (mode == BMI088_BLOCK_PERIODIC_MODE) {
        SPISetMode(bmi088Instance->spi_acc, SPI_BLOCK_MODE);
        SPISetMode(bmi088Instance->spi_gyro, SPI_BLOCK_MODE);
    } else if (mode == BMI088_BLOCK_TRIGGER_MODE) {
        SPISetMode(bmi088Instance->spi_acc, SPI_DMA_MODE);
        SPISetMode(bmi088Instance->spi_gyro, SPI_DMA_MODE);
    }
}
// -------------------------以上为私有函数,用于改变BMI088的配置--------------------------------//

// -------------------------以下为公有函数,用于注册BMI088,标定和数据读取--------------------------------//

/**
 * @brief
 * @param bmi088
 * @return BMI088_Data_t
 */
uint8_t BMI088Acquire(BMI088Instance *bmi088, BMI088_Data_t *data_store)
{
    // 如果是blocking模式,则主动触发一次读取并返回数据
    if (bmi088->work_mode == BMI088_BLOCK_PERIODIC_MODE) {
        static uint8_t buf[6] = {0}; // 最多读取6个byte(gyro/acc,temp是2)
        // 读取accel的x轴数据首地址,bmi088内部自增读取地址 // 3* sizeof(int16_t)
        BMI088AccelRead(bmi088, BMI088_ACCEL_XOUT_L, buf, 6);
        for (uint8_t i = 0; i < 3; i++)
            data_store->acc[i] = bmi088->BMI088_ACCEL_SEN * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]);
        BMI088GyroRead(bmi088, BMI088_GYRO_X_L, buf, 6); // 连续读取3个(3*2=6)轴的角速度
        for (uint8_t i = 0; i < 3; i++)
            data_store->gyro[i] = bmi088->BMI088_GYRO_SEN * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]);
        BMI088AccelRead(bmi088, BMI088_TEMP_M, buf, 2); // 读温度,温度传感器在accel上
        data_store->temperature = (float)(int16_t)((buf[0] << 3) | (buf[1] >> 5)) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        // 更新BMI088自身结构体数据
        for (uint8_t i = 0; i < 3; i++) {
            bmi088->acc[i]  = data_store->acc[i];
            bmi088->gyro[i] = data_store->gyro[i];
        }
        bmi088->temperature = data_store->temperature;
        return 1;
    }
    // 如果是IT模式,则检查标志位.当传感器数据准备好会触发外部中断,中断服务函数会将标志位置1
    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE && bmi088->update_flag.imu_ready == 1) {
        data_store->acc[0]            = bmi088->acc[0];
        data_store->acc[1]            = bmi088->acc[1];
        data_store->acc[2]            = bmi088->acc[2];
        data_store->gyro[0]           = bmi088->gyro[0];
        data_store->gyro[1]           = bmi088->gyro[1];
        data_store->gyro[2]           = bmi088->gyro[2];
        data_store->temperature       = bmi088->temperature;
        bmi088->update_flag.imu_ready = 0;
        return 1;
    }
    // 如果数据还没准备好,则返回空数据
    if (bmi088->update_flag.imu_ready == 0) {
        data_store = NULL;
        return 0;
    }
    return 255;
}

/**
 * @brief :  读取BMI088的IMU更新完成标志位
 * @param *bmi088
 * @return 1 数据准备完毕 0 没有数据
 */
uint8_t BMI088Acquire_IT_Status(BMI088Instance *bmi088)
{
    // 只有中断才能读取标志位
    if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE && bmi088->update_flag.imu_ready == 1) {
        bmi088->update_flag.imu_ready = 0;
        return 1;
    } else
        return 0;
}

#pragma message ("REMEMBER TO CHANGE CALI PARAMETERS IF YOU CHOOSE NOT TO CALIBRATE ONLINE(parameters in robot_def.h)")
#define GYRO_CALIBRATE_TIME 20000 // 20s
/**
 * @brief BMI088 gyro 标定
 * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
 * @param _bmi088 待标定的BMI088实例
 */
void BMI088CalibrateIMU(BMI088Instance *_bmi088)
{
    if (_bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE) // 性感bmi088在线标定
    {
        static uint16_t cali_time_count = 0;
        while (cali_time_count < GYRO_CALIBRATE_TIME) {
            if (cali_time_count % 1000 == 0) {
                buzzer_one_note(1047, 0.2f);
            }
            BMI088_Data_t bmi088_data = {0};
            BMI088Acquire(_bmi088, &bmi088_data);
            bmi088_data.gyro[0] += _bmi088->gyro_offset[0];
            bmi088_data.gyro[1] += _bmi088->gyro_offset[1];
            bmi088_data.gyro[2] += _bmi088->gyro_offset[2];
            _bmi088->gyro_offset[0] -= 0.0003f * bmi088_data.gyro[0];
            _bmi088->gyro_offset[1] -= 0.0003f * bmi088_data.gyro[1];
            _bmi088->gyro_offset[2] -= 0.0003f * bmi088_data.gyro[2];
            cali_time_count++;
            DWT_Delay(0.001f);
        }
    }
    // 导入数据
    else if (_bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE) {
        _bmi088->gyro_offset[0] = BMI088_PRE_CALI_GYRO_X_OFFSET;
        _bmi088->gyro_offset[1] = BMI088_PRE_CALI_GYRO_Y_OFFSET;
        _bmi088->gyro_offset[2] = BMI088_PRE_CALI_GYRO_Z_OFFSET;
    }
}

BMI088Instance *BMI088Register(BMI088_Init_Config_s *config)
{
    // 申请内存
    BMI088Instance *bmi088_instance = (BMI088Instance *)zmalloc(sizeof(BMI088Instance));
    // 从右向左赋值,让bsp instance保存指向bmi088_instance的指针(父指针),便于在底层中断中访问bmi088_instance
    config->acc_int_config.id =
        config->gyro_int_config.id =
            config->spi_acc_config.id =
                config->spi_gyro_config.id =
                    config->heat_pwm_config.id = bmi088_instance;

    // 根据参数选择工作模式
    if (config->work_mode == BMI088_BLOCK_PERIODIC_MODE) {
        config->spi_acc_config.spi_work_mode  = SPI_BLOCK_MODE;
        config->spi_gyro_config.spi_work_mode = SPI_BLOCK_MODE;
        // callbacks are all NULL
    } else if (config->work_mode == BMI088_BLOCK_TRIGGER_MODE) {
        config->spi_gyro_config.spi_work_mode = SPI_DMA_MODE; // 如果DMA资源不够,可以用SPI_IT_MODE
        config->spi_gyro_config.spi_work_mode = SPI_DMA_MODE;
        // 设置回调函数
        config->spi_acc_config.callback             = BMI088AccSPIFinishCallback;
        config->spi_gyro_config.callback            = BMI088GyroSPIFinishCallback;
        config->acc_int_config.gpio_model_callback  = BMI088AccINTCallback;
        config->gyro_int_config.gpio_model_callback = BMI088GyroINTCallback;
        bmi088_instance->acc_int                    = GPIORegister(&config->acc_int_config); // 只有在非阻塞模式下才需要注册中断
        bmi088_instance->gyro_int                   = GPIORegister(&config->gyro_int_config);
    }
    // 注册实例
    bmi088_instance->spi_acc  = SPIRegister(&config->spi_acc_config);
    bmi088_instance->spi_gyro = SPIRegister(&config->spi_gyro_config);
    bmi088_instance->heat_pwm = PWMRegister(&config->heat_pwm_config);

    bmi088_instance->heat_pid = PIDRegister(&config->heat_pid_config);

    bmi088_instance->ambient_temperature = -273; // 环境温度初值
    DWT_GetDeltaT(&bmi088_instance->bias_dwt_cnt);
    // 初始化时使用阻塞模式
    BMI088SetMode(bmi088_instance, BMI088_BLOCK_PERIODIC_MODE);
    // 初始化acc和gyro
    BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;
    do {
        error = BMI088_NO_ERROR;
        error |= BMI088AccelInit(bmi088_instance);
        error |= BMI088GyroInit(bmi088_instance);
        // 可以增加try out times,超出次数则返回错误
    } while (error != 0);
    bmi088_instance->cali_mode = config->cali_mode;
    BMI088CalibrateIMU(bmi088_instance);               // 标定acc和gyro
    BMI088SetMode(bmi088_instance, config->work_mode); // 恢复工作模式

    return bmi088_instance;
}
