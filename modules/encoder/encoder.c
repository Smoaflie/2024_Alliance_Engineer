#include "encoder.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

static uint8_t idx = 0; // register idx,是该文件的全局索引,在注册时使用
/* 编码器的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static EncoderInstance_s *encoder_instance[ENCODER_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

static void DecodeEncoder(CANInstance *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    uint8_t *rxbuff                 = _instance->rx_buff;
    EncoderInstance_s *encoder      = (EncoderInstance_s *)_instance->id;
    Encoder_Measure_s *measure = &encoder->measure; // measure要多次使用,保存指针减小访存开销
    float offset                    = encoder->offset;

    DaemonReload(encoder->daemon);
    encoder->dt = DWT_GetDeltaT(&encoder->feed_cnt);

    // 解析数据
    measure->last_ecd           = measure->ecd;
    // measure->ecd                = ((uint32_t)_instance->rx_buff[0]);
    measure->ecd                = rxbuff[2] << 16 | rxbuff[1] << 8 | rxbuff[0];

    measure->ecd = measure->ecd >= offset ? (measure->ecd - offset) : (measure->ecd + 262144 - offset) ;

    measure->angle_single_round = ECD_ANGLE_TO_DEG * (float)measure->ecd;

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if ((int32_t)(measure->ecd - measure->last_ecd) > 131072)
        measure->total_round--;
    else if ((int32_t)(measure->ecd - measure->last_ecd) < -131072)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void EncoderLostCallback(void* encoder_ptr)
{
    EncoderInstance_s *encoder = (EncoderInstance_s *)encoder_ptr;
    uint16_t can_bus        = encoder->encoder_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("Encoder lost, can bus [%d] , id [%d]", can_bus, encoder->encoder_can_instance->tx_id);
}

// 编码器初始化,返回一个编码器实例
EncoderInstance_s *EncoderInit(Encoder_Init_Config_s *config)
{
    EncoderInstance_s *instance = (EncoderInstance_s *)malloc(sizeof(EncoderInstance_s));
    memset(instance, 0, sizeof(EncoderInstance_s));

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = DecodeEncoder; // set callback
    config->can_init_config.id                  = instance;       // set id,eq to address(it is identity)
    instance->encoder_can_instance                = CANRegister(&config->can_init_config);

    //配置偏移量
    instance->offset = config->offset;

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback     = EncoderLostCallback,
        .owner_id     = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    encoder_instance[idx++] = instance;
    
    return instance;
}