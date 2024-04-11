<!--
 * @Descripttion: 
 * @version: 
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:27:57
-->
# super_can

## 代码结构

.h中放置的是数据定义和外部接口，以及协议的定义和宏，.c中包含一些私有函数。

## 外部接口

```c
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s* supercap_config);
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);
```
## 私有函数和变量

```c
static SuperCapInstance *super_cap_instance = NULL;
static uint8_t *rxbuff;
static void SuperCapRxCallback(can_instance *_instance)
```

`SuperCapRxCallback()`是super cap初始化can实例时的回调函数，用于can接收中断，进行协议解析。

## 使用范例

初始化时设置如下：

```c
SuperCap_Init_Config_s capconfig = {
		.can_config = {
			.can_handle = &hcan1,
			.rx_id = 0x301,
			.tx_id = 0x302
		},
		.recv_data_len = 4*sizeof(uint16_t),
		.send_data_len = sizeof(uint8_t)
	};
SuperCapInstance *ins =SuperCapInit(&capconfig);
```


发送通过`SuperCapSend()`，建议使用强制类型转换：

```c
uint16_t tx = 0x321;
SuperCapSend(ins, (uint8_t*)&tx);
```

## 通信协议（待修改）
接收数据
```c
typedef struct 
{
    uint8_t power_relay_flag;           //继电器开启状态
    uint8_t power_level;                //功率等级
    uint16_t chassic_power_remaining;   //剩余功率
}SuperCap_Msg_g;
```

```c
发送数据
typedef struct
{
    float CapVot;         // 电压
    uint8_t open_flag;    // 开关指示
} SuperCap_Msg_s;
```

旧的协议
```c
CAN1_Tx_Data[0] = power_relay;
CAN1_Tx_Data[1] = power_level;
CAN1_Tx_Data[2] = chassic_power_remaining;
CAN1_Tx_Data[3] = 0;
CAN1_Tx_Data[4] = 0;
CAN1_Tx_Data[5] = 0;
CAN1_Tx_Data[6] = 0;
CAN1_Tx_Data[7] = 0;
--------------------------------------------------------------------
SuperCap_Info.CapVot = bit8TObit32(CAN1_Rx_Data);
SuperCap_Info.InputCurrent = bit8TObit32(&CAN1_Rx_Data[4]);
SuperCap_Info.id = 0x300;
--------------------------------------------------------------------
switch (Referee_Inf.game_robot_state.chassis_power_limit)
{
	case 45:
	    power_level = 1;
	    break;
	case 50:
		power_level = 2;
	    break;
	case 55:
		power_level = 3;
	    break;
	case 60:
		power_level = 4;
	    break;
	case 80:
		power_level = 5;
	    break;
	case 100:
		power_level = 6;
	    break;
	default:
		power_level = 7;
	    break;
}

chassic_power_remaining=Referee_Inf.game_robot_state.chassis_power_limit - Referee_Inf.power_heat_data.chassis_power;
```
