## 图传链路
图传链路的数据解析和键鼠是同一套协议，走的是串口。

### 使用方法

```c
ext_robot_command_pack *ExternCmdRegister(UART_HandleTypeDef *ExternCmdUsartHandle);
```

使用该函数初始化图传链路，参数为该图传链路使用的串口handle

```c
uint8_t ExternCmdIsOnline();
```

使用该函数可以检查图传链路是否在线


