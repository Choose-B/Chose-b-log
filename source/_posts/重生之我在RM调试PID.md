---
title: 重生之我在RM调试PID
tag:
  - 笔记
  - PID
index_img: /img/RM-logo.png
mathjax: true
excerpt: '我重生了，上一世我在VEX被PID杀的片甲不留。这一世，我要在RoboMaster夺回我的一切!(怒)'
date: 2025-11-4 09:56:13
category: RoboMaster
---
## 引入
PID即：`Proportional`（比例）、`Integral`（积分）、`Differential`（微分）的缩写。顾名思义，PID控制算法是结合比例、积分和微分三种环节于一体的控制算法  
PID控制的实质就是根据输入的偏差值，按照比例、积分、微分的函数关系进行运算，运算结果用以控制输出  
![PID算法图](/img/RM-Note/2-4-1.webp)  
### 偏差
`偏差`即**预定目标**和**当前状态**之间的差值  
我们设 预定值`target`是我们希望系统平衡时传感器的返回值  
传感器实时返回的数值为`measure`  
那么根据`偏差`的定义，我们可以得到`偏差`的计算式:
{% mathjax %} error = target - measure {% endmathjax %}
其中`error`为我们所求的偏差值
### 比例算法
> 成比例地反映控制系统的偏差信号，偏差一旦产生，立即产生控制作用以减小偏差。

比例算法的核心思想非常简单：离目标越远，就应该调整的越快  
我们举 `量取1L水`的例子来说明  
如果桶里面只有 10ml 水 ，那么你倒水的时候就会哗哗的往里面倒  
而如果此时 桶快满了 ，你为了不倒多了，你就会选择慢慢的向下加  

将这样的思想抽象成数学语言:  
记比例算法的输出值为 `P`  
那么 `error` 越大 ， `P`就应该越大
{% mathjax %}  
P \propto error  
{% endmathjax %}  
即
{% mathjax %}  
P = k_p · error  
{% endmathjax %}  
其中 {% mathjax %}k_p{% endmathjax %} 是我们引入的系数，称为`比例系数`
### 积分算法
> 积分环节的作用，主要用于消除静差提高系统的无差度。
#### 定义
积分算法是对比例算法的补充  
还是`量取1L水`的例子,但是这次桶破了一个洞:
> 为了方便说明，不妨假设每秒的漏水为{% mathjax %}10ml{% endmathjax %} , {% mathjax %}k_p=0.8{% endmathjax %}

你还是用`比例算法`的思想去倒水  
然而在你加水加到了{% mathjax %}920ml{% endmathjax %}时，你会发现此时水位已经不再上涨  
这是因为此时你加水的速度是
{% mathjax %}  
(1000-920)\times0.8=10 \space (ml)  
{% endmathjax %}  
和漏水的速度持平

如果你可以关注到水位一直都在{% mathjax %}920ml{% endmathjax %}处这个现象的话,你可以尝试根据累计水位和目标之间的偏差来计算你到底应该额外加入多少的水  
记积分算法的输出值为`I`,那么:
{% mathjax %}  
I \propto \int_{0}^{T} error·{\rm d}t  
{% endmathjax %}  
即
{% mathjax %}  
I = k_i · \int_{0}^{T} error·{\rm d}t  
{% endmathjax %}  
其中{% mathjax %}k_i{% endmathjax %}为我们引入的常量`积分系数`  
{% mathjax %}T{% endmathjax %}为pid算法从开始运行到当前时间的计时
#### 离散化
显然,{% mathjax %}\lim_{dt \to 0^{+}}{% endmathjax %}的情况在现实中是无法实现的,我们只能采取近似的计算方法  
积分算法的理论计算式离散化后可以用下式计算:  
{% mathjax %}  
I = k_i · \sum_{t=0}^{T} error·\Delta t  
{% endmathjax %}  
用C++实现:
```cpp
I = I + ki*error*(time_now - time_last);
time_last = time_now;
```
#### 积分限制
引入积分算法后可能出现以下情况:  
1. 以电机为例，在电机的启停或设定值大幅变化时，系统在较短时间内产生了很大的偏差。此时积分迅速积累，就会造成控制量输出远远大于电机的极限输入控制量，从而会引起很大的超调，甚至会产生震荡。  
2. 积分饱和：当系统一直存在一个方向的偏差时，积分会不断增大，会造成控制量进入饱和区，一旦出现反向的偏差时，需要很长时间才能推出饱和区，而去响应反向偏差。也以电机为例，电机在积分饱和时，电机响应延时较大，会出现电机超出目标位置，需来回调整数次才能稳定。  

此时就有必要限制积分算法的输出  
如果{% mathjax %}I{% endmathjax %}超出了某一预定的范围{% mathjax %}[-I_{max},I_{max}]{% endmathjax %},可以对I的大小进行限制
```cpp
//积分限幅
if ( fabs(pid->i) > pid->i_limit ){
   pid->i = pid->i > 0 ? pid->i_limit : -pid->i_limit;
}
```

#### 积分分离
积分分离同样是一种避免过调的手段  
既然积分算法的目的是消除静差，而且会在偏差值过于大的时候异常运行，那么不妨对积分算法做这样的限制:**需要积分算法的时候再开启积分算法，不需要就关闭**  
设定积分控制阈值{% mathjax %} \epsilon {% endmathjax %}，根据{% mathjax %} \left | error \right |{% endmathjax %}和{% mathjax %} \epsilon {% endmathjax %}的大小关系来决定积分项的开关系数{% mathjax %} \beta {% endmathjax %}的取值:   
{% mathjax %} 
当\left | error \right | \leq  \epsilon,\beta=1,采用PID控制,避免系统静差\\
当\left | error \right | > \epsilon,\beta=0,采用PD控制,避免产生过大的超调
{% endmathjax %}  


### 微分算法
> 微分环节的作用能反映偏差信号的变化趋势（变化速率），并能在偏差信号的值变得太大之前，在系统中引入一个有效的早期修正信号，从而加快系统的动作速度，减小调节时间。
>
#### 定义
微分算法同样是对比例算法的补充  
这次的例子换成`在手指上面立筷子`吧  

立筷子还不简单,你轻易的让筷子保持了竖直  
正当我们放松的时候，一阵阴风袭来，把筷子稍稍吹歪了  
* 只用比例算法？  
  筷子才歪这么一点，我都不需要怎么动哎  
  然后很快筷子就倒了
* 加上积分算法?
  本来筷子是保持平衡的，也就是  

  {% mathjax %}   
  output=P+I=0   
  {% endmathjax %}   

  {% mathjax %}  
  \because P=K_p·error=0  
  {% endmathjax %}   

  {% mathjax %}  
  \therefore I_{t=0}=0  
  {% endmathjax %}  
  此时积分算法能起到的效果和比例算法没有什么差别了

看来我们需要新的方法  
立筷子之所以容易失败，是因为筷子只要有一点偏差，如果不迅速的加以修正，筷子就会快速的倒下  
如果在{% mathjax %}P{% endmathjax %}和{% mathjax %}I{% endmathjax %}的基础上，再增加一项{% mathjax %}D{% endmathjax %}，使得  
{% mathjax %}  
D \propto 筷子高度的瞬时变化率  
{% endmathjax %}  
问题就可以迎刃而解  
注意到`微分`是衡量`瞬时变化率`的很好的数学工具，于是我们可以写出:  
{% mathjax %}  
D \propto \tfrac{d}{dt} error  
{% endmathjax %}
即  
{% mathjax %}  
D = k_d·\tfrac{d}{dt} error  
{% endmathjax %}
其中{% mathjax %}k_d{% endmathjax %}为我们引入的`微分系数`
#### 离散化
同样的，由于{% mathjax %}\lim_{dt \to 0^{+}}{% endmathjax %}在现实中无法实现，微分算法的计算同样需要离散化  
{% mathjax %}  
D = k_d·\tfrac{ \Delta error }{ \Delta t } 
{% endmathjax %}  
用c++实现:  
```cpp
D = kd*(error-error_last)/(time_now-time_last);
error_last = error;
time_last = time_now;
```
#### 微分先行  
在某些给定值频繁且大幅变化的场合，微分项常常会引起系统的振荡。这是因为我们的微分项是对偏差值{% mathjax %}error{% endmathjax %}的微分,当短时间内实际值变化不大而目标值突变的情况下，偏差值会发生突变，从而让微分项错误的介入控制过程。   
明明实际值没有多大变化，微分项没理由发生突变啊。**那只对实际值微分不就好了**  

## 综合使用
使用{% mathjax %}P{% endmathjax %}、{% mathjax %}I{% endmathjax %}、{% mathjax %}D{% endmathjax %}三种算法，足以满足大多数输出的控制了
总输出值的理想计算式:  
{% mathjax %}  
output = P + I + D  
{% endmathjax %}  
如果全部带入:  
{% mathjax %}  
output = k_p·error + k_i·\int_{0}^{T}error·dt + k_d·\tfrac{d}{dt}error  
{% endmathjax %}  
离散化后可以用C++实现  
{% mathjax %}  
output = k_p·error + k_i·\sum_{t=0}^{T}error·\Delta t + k_d·\tfrac{ \Delta error }{ \Delta t }  
{% endmathjax %}  

{% fold info @贴上我自己的代码吧 %}
```c
// pid.h
# pragma once
#include "main.h"

/**
 * @brief PID控制器结构体
 * 
 * @param frequency 采样频率
 * @param target 目标值
 * @param measure 测量值
 * @param last_measure 上次测量值
 * @param error 误差值
 * @param p 比例项
 * @param k_p 比例系数
 * @param i 积分项
 * @param k_i 积分系数
 * @param i_limit 积分项限幅值
 * @param episilon 积分分离阈值
 * @param beta 积分分离开关
 * @param d 微分项
 * @param k_d 微分系数
 * @param max_output 最大输出
 */
typedef struct {
    int frequency;//采样频率

    int target;//目标值

    float measure;//测量值
    float last_measure;//上次测量值

    float error;//误差值
    
    float p;//比例项
    float k_p;//比例系数

    float i;//积分项
    float k_i;//积分系数
    float i_limit;//积分项限幅值
    int episilon;//积分分离阈值
    uint8_t beta;//积分分离开关

    float d;//微分项
    float k_d;//微分系数

    float max_output;//最大输出
}PID;

/**
 * @brief   初始化PID结构体
 * 
 * @param pid 指向需要初始化的pid对象   
 * @param k_p 比例系数
 * @param k_i 积分系数
 * @param k_d 微分系数
 * @param target 目标值
 * @param frequency 采样频率
 * @param max_output 最大输出
 * @param i_limit 积分项最大值
 * @param episilon 积分分离阈值
 * @return 是否初始化成功 1-成功 0-失败
 */
uint8_t PID_Init( PID* pid,
                float k_p,
                float k_i,
                float k_d,
                float target,
                int frequency,
                float max_output,
                float i_limit,
                float episilon
            );

/**
 * @brief 计算控制量
 * 
 * @param pid 指向对某一被控量进行控制的pid对象
 * @param measure 反馈值
 * @return 控制量 
 */
float PID_Calc( PID* pid, float measure);

```

```c
// pid.c
# include "pid.h"

uint8_t PID_Init( PID* pid,
                float k_p,
                float k_i,
                float k_d,
                float target,
                int frequency,
                float max_output,
                float i_limit,
                float episilon
            ) {
    if (pid == NULL || frequency <= 0 || max_output < 0 || i_limit < 0) {
        return 0;
    }

    pid->k_p = k_p;
    pid->k_i = k_i;
    pid->k_d = k_d;
    pid->target = target;
    pid->frequency = frequency;
    pid->max_output = max_output;
    pid->i_limit = i_limit;
    pid->episilon = episilon;

    pid->error = 0.0f;
    pid->p = 0.0f;
    pid->i = 0.0f;
    pid->d = 0.0f;
    pid->beta = 0;
    pid->last_measure = 0.0f;

    return 1;
}

float PID_Calc(PID* pid, float measure) {
    pid->measure = measure;
    pid->last_measure = pid->last_measure * 0.9f + measure * 0.1f;//低通滤波
    pid->error = pid->target - measure;

    // Proportional term
    pid->p = pid->k_p * pid->error;

    // Integral term
    // 积分分离
    if ( fabs(pid->error) < pid->episilon ) {
        pid->beta = 1;
    }
    else {
        pid->beta = 0;
    }

    if ( pid->beta ) {
        pid->i += pid->k_i * pid->error / pid->frequency ;
        //积分限幅
        if ( fabs(pid->i) > pid->i_limit ){
            pid->i = pid->i > 0 ? pid->i_limit : -pid->i_limit;
        }
    }
    else {
        pid->i = 0;
    }
    

    // Derivative term
    // 使用微分先行
    pid->d = (measure - pid->last_measure) * pid->k_d * pid->frequency;

    float output = pid->p + pid->i * pid->beta + pid->d;
    return fabs(output) > pid->max_output ? 
            output > 0 ? pid->max_output : -pid->max_output : 
            output;
}

```
{% endfold %}


## 调试参数

### VOFA+ 上位机  

[VOFA+](https://www.vofa.plus/)是一个直观灵活强大的上位机。它可以帮助我们显示调试过程中参数的图像。  
要使用vofa+,你首先得安装它。这个软件功能是免费的，给作者爆米则可以更改这个软件的皮肤。  

#### VOFA+ 参数设置  
打开VOFA+,可以看到一个简洁的界面:
* 左上角的圆点表示和串口的连接状态，单击可以切换是否和串口连接
  - 灰色-关闭连接
  - 蓝色常亮-连接上串口但是没有数据收发
  - 蓝色快速闪烁-正常进行数据收发
* 左侧侧边栏有三个按钮
  - 协议和连接
    + 数据引擎: 选择`FireWater`模式,发送数据的格式可以看协议概览
    + 数据接口：选`串口`
    + 端口号: 需要选择连接了TTL-USB的电脑端口，如果不会判断那就插拔TTL-USB，会变更的端口就是正确的端口
    + 波特率：需要和设置的串口波特率一致。如果你没有修改过串口波特率那就使用默认的`115200`即可
    + 数据流控，校验位，数据位数，停止位数：同理，和串口的设置保持一致，如果你没有改过那就使用默认的
  - 命令 - 我们一般用不到这个
  - 控件 - 可以选择需要的控件加入到标签页中
* 中间上半部分是标签页区，可以摆放组件，比如图中的摆放了一个波形图组件
* 中间下半时数据流显示区，只要把`HEX`切换为`Abc`就可以（十六进制显示切换为字符串显示）
* 右侧是数据区，显示传回的不同的数据项

![VOFA+界面](/img/RM-Note/4-1.png)  

#### 代码适配 
串口发送数据的代码是从学长那里继承来的，我实际使用的有一些不同(原文见文章末尾参考部分)  
值得提醒的是，**数据发送给上位机的方式是DMA，请务必确认有无在CubeMX中打开对应串口的DMA**  
在项目中添加文件`ano_vofa.h`和`ano_vofa.c`,然后复制以下的文件内容

```c
// ano_vofa.h
#ifndef _ANO_VOFA_H_
#define _ANO_VOFA_H_

#include "stm32h7xx_hal.h"
// #include "cmsis_os.h"
// #include "driver_usart.h"
#include "usart.h"

void UsartDmaPrintf(UART_HandleTypeDef *uartx, const char *format, ...);

#ifdef _CMSIS_OS_H // 使用cmsis_os.h的代码的串口结构体类型是UART_RxBuffer_t，没有用RTOS的是老版代码，为了兼容性，添加此宏定义
extern UART_RxBuffer_t uart5_buffer;
#else // 使用UART_RxBuffer则将#include "cmsis_os.h"注释掉

#endif

extern uint8_t Usart5_TxBuffer_Vofa[128] __attribute__((at(0x24020200))); // VOFA+

#endif

```

```c
// ano_vofa.c
#include "ano_vofa.h"
#include <stdarg.h>
#include <stdio.h>

uint8_t Usart5_TxBuffer_Vofa[128]; // __attribute__((at()))
// 如果遇到DMA不能收发的问题，可以尝试在这里加上 __attribute__((at(0x24020200)))
// 这一段的意思是强制让这个数组的首地址为0x24020200
// 原理是 DMA 收发时只能访问位于地址0x24020000之后的数据

/*------------------------------- VOFA+ ---------------------------------*/
void UsartDmaPrintf(UART_HandleTypeDef *uartx, const char*format, ...)
{
    uint16_t len;
    va_list args;
    va_start(args, format);
    len = vsnprintf((char *)Usart5_TxBuffer_Vofa, sizeof(Usart5_TxBuffer_Vofa) + 1, (char*)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(uartx, Usart5_TxBuffer_Vofa, len);
}

```
这样就可以使用函数`UsartDmaPrintf(UART_HandleTypeDef *uartx, const char*format, ...)`来便携地实现数据发送了  
使用此函数的方法类似`printf`,比如:   
```c
UsartDmaPrintf(&huart5,"%d, %d\r\n",motor_chassis[1].speed_rpm,motor_speed_pid.target);
```
这样VOFA+正常接收到数据时会识别出两组数据  
{% note danger %}
**千万不要热插拔**  
热插拔指的是在正在传输数据时断开电脑和TTL-USB(或者烧写器)的连接  
这一行为会导致 TTL-USB 发送数据的状态未解除，不能再进行正常的数据发送  
表现形式为连接后VOFA+的蓝灯常亮不闪烁  
**解决方法**  
完全断电TTL-USB (即同时断开电脑连接和电源供电)  
之后可以正常使用  
{% endnote %}  

### 超参数调试
**超参数**，这里指`k_p`,`k_i`,`k_d`三个参数。它们难以通过数学计算得出，基本上只能通过~~玄学~~经验来试出来  
不过调试超参数还是有方法可依的  

#### STEP.1 从PD开始调节
禁用I算法和D算法(就是让`k_i`和`k_d`等于0)  
`k_p`从一个较小的值开始调试(比如k_p = 0.4)  
![k_p=0.4;k_i=0;k_d=0](/source/img/RM-Note/4-2.png)  
然后逐步提高k_p的值，直到接近恰好没有发生过调的临界点
![k_p=0.75;k_i=0;k_d=0](/source/img/RM-Note/4-3.png)  
如果希望更快响应，可以让k_p更大一点，发生轻微过调后，加入d算法  
由于本实验不是`非稳定平衡系统`,所以D算法不是必须的  
{% fold info @稳定平衡，恒平衡和非稳定平衡 %}
* **稳定平衡**  
  稳定平衡是即使受到扰动也会自行回归平衡状态的系统   
  最典型的例子是放在凹弧面底部的小球，这个小球即使偏移了底部的位置，也会自发回归平衡状态
* **非稳定平衡**  
  非稳定平衡是受到扰动会无法自发回归平衡状态的系统  
  最典型的例子是放在凸弧面的顶部的小球。小球恰好在凸弧面顶部时是恰好平衡的，但是只要受到扰动，就会从顶部滚落，自发远离平衡状态  
  在实际中要实现这种平衡，必须在外部干预，比如使用PID算法来控制  
  平衡步兵就属于这种系统
* **恒平衡**   
  恒平衡是不论处于何种状态都可以平衡的系统  
  一个放在水平平面的小球，无论怎么被扰动，都处于水平面上。在这个系统中，小球既不会有“自发回归”的趋势，也不会有“自发远离”的趋势
{% endfold %}
#### STEP.2 加入I控制，消除静差
完成上一步的调节后，逐步增加`k_i`的值，提高I算法控制的权重  
![k_p=0.75;k_i=0.4;k_d=0](/source/img/RM-Note/4-4.png)  
可见加入I算法后静差几乎消失，被控量曲线能够较好地贴合目标曲线  
`k_i`应该略小于恰好不发生过调的临界位置，过小会导致消除静差的过程过慢  
下面是发生了过调的反例  
![k_p=0.75;k_i=0.6;k_d=0](/source/img/RM-Note/4-5.png)  

#### STEP.3 测试PID控制的跟随能力
实际使用时`target`值很少会有不改变的情况  
在控制过程中频繁大幅度地改变`target`值，来观察曲线的跟随能力  
![突然大幅度改变target](/source/img/RM-Note/4-6.png)

---END---
---
参考:
[从不懂到会用！PID从理论到实践~](https://www.bilibili.com/video/BV1B54y1V7hp),华南小虎队,BV1B54y1V7hp  
[【中科大RM电控合集】PID接口与电机闭环控制编程](https://www.bilibili.com/video/BV17m4y1L7E5),TrojanGeneric,BV17m4y1L7E5  
[学会PID-基于板球平衡系统-初中基础就能听懂的简单讲](https://www.bilibili.com/video/BV1xL4y147ea),程欢欢的智能控制集,BV1xL4y147ea  
[图文详解PID调参&积分分离PID控制算法](https://blog.csdn.net/m0_46577050/article/details/136354116),宁静致远2021  
{% fold info @CUBOT电控培训文档 %}
# 第四次培训

## 匿名上位机 / VOFA+上位机的添加

### 下载

- 匿名官网下载：[匿名产品资料:资料下载链接汇总-匿名科创](https://www.anotc.com/wiki/%E5%8C%BF%E5%90%8D%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/%E8%B5%84%E6%96%99%E4%B8%8B%E8%BD%BD%E9%93%BE%E6%8E%A5%E6%B1%87%E6%80%BB)
- vofa+官网下载：[代码配酒，bug没有-VOFA+](https://www.vofa.plus/)
  
### 代码添加

- 创建头文件并添加：

```c
    #ifndef _ANO_VOFA_H_
    #define _ANO_VOFA_H_

    #include "stm32h7xx_hal.h"
    // #include "cmsis_os.h"
    // #include "driver_usart.h"
    #include "usart.h"

    /*-------------------------------匿名上位机---------------------------------*/
    #define BYTE0(dwTemp) (*(char *)(&dwTemp))
    #define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
    #define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
    #define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

    void ANO_Send_Up_Computer(UART_HandleTypeDef *uartx,
                              int16_t user1, int16_t user2, int16_t user3, int16_t user4, int16_t user5, int16_t user6);
    void UsartDmaPrintf(UART_HandleTypeDef *uartx, const char *format, ...);
    // uint8_t vofa_Callback(uint8_t *recBuffer, uint16_t len);
    #ifdef _CMSIS_OS_H // 使用cmsis_os.h的代码的串口结构体类型是UART_RxBuffer_t，没有用RTOS的是老版代码，为了兼容性，添加此宏定义
    extern UART_RxBuffer_t uart5_buffer;
    #else // 使用UART_RxBuffer则将#include "cmsis_os.h"注释掉
    // extern UART_RxBuffer uart5_buffer;
    #endif
    extern uint8_t Usart5_TxBuffer_ANO[20] __attribute__((at(0x24020000)));   // 匿名上位机
    extern uint8_t Usart5_TxBuffer_Vofa[128] __attribute__((at(0x24020200))); // VOFA+

    #endif

```

- 创建源文件并添加：

```c
    #include "ano_vofa.h"
    #include <stdarg.h>
    #include <stdio.h>
    uint8_t Usart5_TxBuffer_ANO[20];
    uint8_t Usart5_TxBuffer_Vofa[128];

    /*-------------------------------匿名上位机---------------------------------*/
    void ANO_Send_Up_Computer(UART_HandleTypeDef *uartx, int16_t user1, int16_t user2, int16_t user3, int16_t user4, int16_t user5, int16_t user6)
    {
        uint8_t _cnt                = 0;
        Usart5_TxBuffer_ANO[_cnt++] = 0xAA;
        Usart5_TxBuffer_ANO[_cnt++] = 0xFF;
        Usart5_TxBuffer_ANO[_cnt++] = 0xF1;
        Usart5_TxBuffer_ANO[_cnt++] = 12;

        Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user1);
        Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user1);

        Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user2);
        Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user2);

        Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user3);
        Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user3);

        Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user4);
        Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user4);

        Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user5);
        Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user5);

        Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user6);
        Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user6);

        uint8_t sc = 0;
        uint8_t ac = 0;

        for (uint8_t i = 0; i < (Usart5_TxBuffer_ANO[3] + 4); i++) {
            sc += Usart5_TxBuffer_ANO[i];
            ac += sc;
        }
        Usart5_TxBuffer_ANO[_cnt++] = sc;
        Usart5_TxBuffer_ANO[_cnt++] = ac;

        HAL_UART_Transmit_DMA(uartx, Usart5_TxBuffer_ANO, _cnt);
    }
    /*------------------------------- VOFA+ ---------------------------------*/
    void UsartDmaPrintf(UART_HandleTypeDef *uartx, const char*format, ...)
    {
        uint16_t len;
        va_list args;
        va_start(args, format);
        len = vsnprintf((char *)Usart5_TxBuffer_Vofa, sizeof(Usart5_TxBuffer_Vofa) + 1, (char*)format, args);
        va_end(args);
        HAL_UART_Transmit_DMA(uartx, Usart5_TxBuffer_Vofa, len);
    }

```

## PID算法

- PID算法公式：
  - 连续：

    $$
    u(t)=K_p e(t)+K_i \int_{0}^{t} e(t)dt+K_d \frac{de(t)}{dt}
    $$

  - 离散：

    \[  
    u(k) = K_p e(k) + K_i \sum_{j=0}^{k} e(j) + K_d [e(k) - e(k-1)]  
    \]

- $e(t)$：误差 = 设定值 - 实际值
- $K_p$：比例增益
- $K_i$：积分增益
- $K_d$：微分增益
- $u(t)$：控制量

- PID算法的特点：
  - 快速响应：当误差积分到一定程度时，积分增益将起到稳定作用，使得输出值快速响应。
  - 稳定性：当误差积分到一定程度时，积分增益将起到稳定作用，使得输出值稳定。
  - 阻尼作用：当误差积分到一定程度时，积分增益将起到阻尼作用，使得输出值减小。
  - 输出限制：当输出值超过限制时，输出值将被限制。
- 代码实现(**需要自己动手写代码，大体就是下面四个步骤**)：
  - 定义PID结构体：
  - 初始化PID结构体：
  - PID算法：
  - 输出限制：
  
{% endfold %}  