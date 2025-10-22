---
title: 重生之我在RM调试PID
tag:
  - 笔记
  - PID
index_img: /img/RM-logo.png
mathjax: true
excerpt: '我重生了，上一世我在VEX被PID杀的片甲不留。这一世，我要在RoboMaster夺回我的一切!(怒)'
date: 2025-10-20 14:23:13
category: RoboMaster
---
## 引入
PID即：`Proportional`（比例）、`Integral`（积分）、`Differential`（微分）的缩写。顾名思义，PID控制算法是结合比例、积分和微分三种环节于一体的控制算法  
PID控制的实质就是根据输入的偏差值，按照比例、积分、微分的函数关系进行运算，运算结果用以控制输出  
![PID算法图](/img/RM-Note/2-4-1/webp)  
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
void I_limit(float& I,float I_max){
    if (I>I_max){
        I = I_max;
    }
    else if (I<-I_max){
        I = -I_max;
    }
    return;
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
    pid->last_measure = pid->last_measure * 0.9f + measure * 0.1f;
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
