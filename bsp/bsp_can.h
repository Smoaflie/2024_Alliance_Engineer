#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "can.h"

#define MX_REGISTER_DEVICE_CNT 12  // maximum number of device can be registered to CAN service
                                   // this number depends on the load of CAN bus.
#define MX_CAN_FILTER_CNT (4 * 14) // temporarily useless
#define DEVICE_CAN_CNT 2           // CAN1,CAN2

/* can instance typedef, every module registered to CAN should have this variable */
typedef struct tmp
{
    CAN_HandleTypeDef* can_handle;
    uint32_t tx_id;
    uint32_t tx_mailbox;
    uint8_t tx_buff[8]; 
    uint8_t rx_buff[8];
    uint32_t rx_id;
    void (*can_module_callback)(struct tmp*); // callback needs an instance to tell among registered ones
} can_instance;

/* module callback,which resolve protocol when new mesg arrives*/
typedef void (*can_callback)(can_instance*);

/**
 * @brief transmit mesg through CAN device
 * 
 * @param _instance can instance owned by module
 */
void CANTransmit(can_instance* _instance);

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 * 
 * @param _instance can instance owned by a specific device, remember to initialize it!
 * 
 */
can_instance* CANRegister(uint8_t tx_id,uint8_t rx_id,CAN_HandleTypeDef* can_handle,can_callback module_callback);


#endif
