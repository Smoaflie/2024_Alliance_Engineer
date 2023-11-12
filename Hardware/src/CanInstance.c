#include "CanInstance.h"

const uint16_t rxid = 0x01;
const uint16_t txid = 0x01;
extern CAN_HandleTypeDef hcan;

CanRTx CanRTXValue = {0};

void CanAddFilter(CAN_HandleTypeDef *can_handle)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can_filter_idx = 0;

    can_filter_conf.FilterMode           = CAN_FILTERMODE_IDLIST;                                // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
    can_filter_conf.FilterScale          = CAN_FILTERSCALE_16BIT;                                // 使用16位id模式,即只有低16位有效
    can_filter_conf.FilterFIFOAssignment = (CanRTXValue.rxid & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1; // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    can_filter_conf.SlaveStartFilterBank = 14;                                                   // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
    can_filter_conf.FilterIdLow          = CanRTXValue.rxid << 5;                                // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
    can_filter_conf.FilterBank           = can_filter_idx++;                                     // 根据can_handle判断是CAN1还是CAN2,然后自增
    can_filter_conf.FilterActivation     = CAN_FILTER_ENABLE;

    HAL_CAN_ConfigFilter(can_handle, &can_filter_conf);
}

void CanInit(CanRTx *CanRTXPtr)
{
    CanRTXValue.rxid = CanRTXPtr->rxid;
    CanRTXValue.txid = CanRTXPtr->txid;

    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    CanAddFilter(&hcan);
}

void CanDataProcess(uint8_t *data)
{
}

uint8_t CanTransimit(uint8_t *data)
{
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) // 等待邮箱空闲
    {
    }

    CAN_TxHeaderTypeDef txconf = {0};
    txconf.StdId               = CanRTXValue.txid;
    txconf.IDE                 = CAN_ID_STD;
    txconf.RTR                 = CAN_RTR_DATA;
    txconf.DLC                 = 0x08;
    uint32_t tx_mailbox        = 0;

    if (HAL_CAN_AddTxMessage(&hcan, &txconf, data, &tx_mailbox)) {

        return 0;
    }
    return 1; // 发送成功
}

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    if (_hcan == &hcan) {
        static CAN_RxHeaderTypeDef rxconf; // 同上
        uint8_t can_rx_buff[8];
        while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
        {
            HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
            if (rxconf.StdId == CanRTXValue.rxid) {
                CanDataProcess(can_rx_buff);
            }
        }
    }
}

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}