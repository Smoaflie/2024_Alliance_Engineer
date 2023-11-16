# INS_task

<p align='middle'>Alliance @HDC</p>

## 初始化

```c
INS_Init(BMI088_);
```

调用INS_Init函数即可初始化，传入参数为BMI088实例指针，用于将INS模块与BMI088模块绑定

该函数会返回一个`INS_Instance`指针，可用于保存INS实例以及获取INS数据

## 解算部分

采用DJI官方解算库`AHRS.lib`内部结构未公开

采用的算法应该是mahony互补滤波，计算量小，性能不错。经过实测，C板168MHz平均解算时间为`50.595us`，为其他任务腾出了充足的CPU时间
