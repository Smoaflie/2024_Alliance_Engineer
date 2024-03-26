# 关于tool

## ramp

计算键盘按键给轮子发送的电流值。

### ramp类型

```c
typedef struct ramp_t
{
  int32_t count; //计数值
  int32_t scale; //规模
  float   out; //输出
}ramp_t;
```

out为输出值，它是一个比例；

count为计数值，运算中count越大，输出值out越大；

scale为规模值，scale越大，输出out达到1的时间越长；

### ramp初始化函数

```c
void ramp_init(ramp_t *ramp, int32_t scale)
{
    ramp->count = 0;
    ramp->scale = scale;
}
```

将count清零，scale初始化为设定值。

### 计算比例函数

```c
float ramp_calc(ramp_t *ramp)
{
    if (ramp->scale <= 0)
        return 0;

    if (ramp->count++ >= ramp->scale)
        ramp->count = ramp->scale;

    ramp->out = ramp->count / ((float)ramp->scale);
    return ramp->out;
}
```

规模值不为0时，输出值out为count与scale的比值，直到count等于scale，之后输出值恒为1。

### 变量

```C
ramp_t fb_ramp     = RAMP_GEN_DAFAULT;
ramp_t lr_ramp     = RAMP_GEN_DAFAULT;
ramp_t rotate_ramp = RAMP_GEN_DAFAULT;
ramp_t slow_ramp   = RAMP_GEN_DAFAULT;
```



## 热量控制

### 变量

```c
int heat_control    = 20; // 热量控制
float heat_remain   = 0;  // 剩余热量
float local_heat    = 0;  // 本地热量
int One_bullet_heat = 10; // 打一发消耗热量
```

具体值待测。