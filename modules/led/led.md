# LED

用于点亮LED，设置不同的颜色

## 使用范例

```c
static C_board_LEDInstance *c_led = NULL;
c_led                             = C_boardLEDRegister();

C_board_LEDSet(c_led, 0x33ffff);
osDelay(500);
C_board_LEDSet(c_led, 0xd633ff);
osDelay(500);
```

set函数传入的是RGB值

可通过[取色器](https://www.w3cschool.cn/tools/index?name=cpicker)获取

当前没有外置LED需求，所以其他功能已隐藏。