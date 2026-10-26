---
title: Kalman Filter
tag:
  - Math
  - Observer
achieve: true
hide: true
index_img: /img/Eyjafjalla.avif
mathjax: true
excerpt: '从连续与离散卡尔曼滤波的推导，到单电机与两轴云台建模，再到 STM32H7 上的 KF、EKF、UKF 实现。'
date: 2026-10-09 14:02:20
category: 笔记
---

## 1. 卡尔曼滤波器在估计什么

编码器可以测角度，但读数有噪声；对角度直接差分能得到速度，但差分会放大噪声。电机动力学可以预测运动，但负载、摩擦和参数误差会让预测逐渐偏离实际。**卡尔曼滤波器（Kalman Filter，KF）把模型预测与传感器观测结合起来，同时估计状态和估计误差的协方差。**

每次递推分成两件事：

1. **预测**：用上一时刻的状态、输入与运动模型，计算现在的状态及不确定性。
2. **校正**：比较实际观测与预测观测，根据双方的不确定性调整预测。

滤波器可以估计没有直接测量的状态，例如仅由编码器角度估计电机角速度。能否做到，取决于模型与传感器组合的**可观测性**，并非把变量写进状态向量就一定能估计它。

{% note info %}
在线性模型、正确的初始统计量、已知噪声协方差及相应不相关假设下，KF 给出线性最小均方误差估计。若初始状态和噪声还满足联合高斯条件，它给出的均值就是所有估计器中的最小均方误差估计，协方差也完整描述了高斯后验。**非高斯噪声不等于 KF 不能用，但不能直接宣称其具有完整的贝叶斯最优性。**
{% endnote %}

本文先讲连续和离散的经典 KF，再用单电机说明建模与应用；接着推导两轴云台的非线性模型，分别应用扩展卡尔曼滤波（Extended Kalman Filter，EKF，也常称“拓展卡尔曼滤波”）和无迹卡尔曼滤波（Unscented Kalman Filter，UKF）。最后给出 STM32H7 + CMSIS-DSP 的 C++ 实现。

### 1.1 符号与基本假设

| 符号 | 意义 | 维度 |
|---|---|---|
| {% mathjax %}x,u,z{% endmathjax %} | 状态、已知输入、观测 | {% mathjax %}n\times1,\ r\times1,\ m\times1{% endmathjax %} |
| {% mathjax %}A,B,H{% endmathjax %} | 连续状态矩阵、输入矩阵、观测矩阵 | {% mathjax %}n\times n,\ n\times r,\ m\times n{% endmathjax %} |
| {% mathjax %}F,B_d{% endmathjax %} | 离散状态转移矩阵、输入矩阵 | {% mathjax %}n\times n,\ n\times r{% endmathjax %} |
| {% mathjax %}\hat x_{k\mid k-1}=\hat x_k^-{% endmathjax %} | 使用截至上次观测得到的本次先验估计 | {% mathjax %}n\times1{% endmathjax %} |
| {% mathjax %}\hat x_{k\mid k}=\hat x_k^+{% endmathjax %} | 加入本次观测后的后验估计 | {% mathjax %}n\times1{% endmathjax %} |
| {% mathjax %}P^-,P^+{% endmathjax %} | 先验、后验误差协方差 | {% mathjax %}n\times n{% endmathjax %} |
| {% mathjax %}Q_d,R_d{% endmathjax %} | 离散过程噪声、观测噪声协方差 | {% mathjax %}n\times n,\ m\times m{% endmathjax %} |
| {% mathjax %}\nu,S,K{% endmathjax %} | 创新、创新协方差、卡尔曼增益 | {% mathjax %}m\times1,\ m\times m,\ n\times m{% endmathjax %} |

误差协方差定义为：

{% mathjax '{"conversion":{"display":true}}' %}
e=x-\hat x,\qquad P=\mathbb E[(e-\mathbb E[e])(e-\mathbb E[e])^T].
{% endmathjax %}

在无偏条件下可简写为 {% mathjax %}P=\mathbb E[ee^T]{% endmathjax %}。对角元素是各状态的误差方差，非对角元素表示误差相关性。角度方差的单位是 {% mathjax %}\mathrm{rad}^2{% endmathjax %}，角速度方差的单位是 {% mathjax %}(\mathrm{rad/s})^2{% endmathjax %}，不能将所有对角元素理解为同一个物理量。

下面默认过程噪声与观测噪声均为零均值、时间白噪声，彼此及初始估计误差不相关；输入视为已知。非零偏置应先校准，或者作为额外状态建模。相关噪声需要相应修改公式。

## 2. 经典 KF：连续时间公式

### 2.1 连续随机模型

连续白噪声没有普通函数意义下的有限瞬时方差，因此用随机微分方程书写更准确：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
\mathrm dx&=(Ax+Bu)\,\mathrm dt+G\,\mathrm dw,\\
\mathrm dy&=Hx\,\mathrm dt+\mathrm dv,\\
\mathbb E[\mathrm dw\,\mathrm dw^T]&=Q_c\,\mathrm dt,\qquad
\mathbb E[\mathrm dv\,\mathrm dv^T]=R_c\,\mathrm dt.
\end{aligned}
{% endmathjax %}

这里 {% mathjax %}y{% endmathjax %} 是累计观测过程，{% mathjax %}Q_c,R_c{% endmathjax %} 是连续噪声强度；{% mathjax %}G{% endmathjax %} 把过程噪声映射到状态。矩阵可以随时间变化，下面省略其时间下标。

### 2.2 Kalman–Bucy 方程

**状态估计与增益：**

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{\mathrm d\hat x=(A\hat x+Bu)\,\mathrm dt
+K_c(\mathrm dy-H\hat x\,\mathrm dt)},\qquad
\boxed{K_c=PH^TR_c^{-1}}.
{% endmathjax %}

**误差协方差的微分 Riccati 方程：**

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{\dot P=AP+PA^T+GQ_cG^T-PH^TR_c^{-1}HP}.
{% endmathjax %}

工程文献常写成 {% mathjax %}\dot{\hat x}=A\hat x+Bu+K_c(z-H\hat x){% endmathjax %}，其中 {% mathjax %}z{% endmathjax %} 是形式上的瞬时观测。它应结合上面的随机模型理解。连续滤波在持续观测中同时预测与校正，不能把它直接当成“每隔 1 ms 执行一次离散更新”。连续滤波与输出反馈的关系可参考 [Harvard 的控制与估计讲义](https://hankyang.seas.harvard.edu/OptimalControlEstimation/output-feedback.html)。

{% fold info @推导：连续误差协方差与最优增益 %}
由状态与估计方程相减：

{% mathjax '{"conversion":{"display":true}}' %}
\mathrm de=(A-K_cH)e\,\mathrm dt+G\,\mathrm dw-K_c\,\mathrm dv.
{% endmathjax %}

用 Itô 乘积规则计算 {% mathjax %}\mathrm d(ee^T){% endmathjax %}。独立噪声的交叉项期望为零，而二次变差项保留下来：

{% mathjax '{"conversion":{"display":true}}' %}
\dot P=(A-K_cH)P+P(A-K_cH)^T+GQ_cG^T+K_cR_cK_c^T.
{% endmathjax %}

将所有含增益的项配方，令 {% mathjax %}K_* = PH^TR_c^{-1}{% endmathjax %}：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
\dot P={}&AP+PA^T+GQ_cG^T-PH^TR_c^{-1}HP\\
&+(K_c-K_*)R_c(K_c-K_*)^T.
\end{aligned}
{% endmathjax %}

最后一项为半正定矩阵。取 {% mathjax %}K_c=K_*{% endmathjax %} 消去它，得到 Kalman–Bucy 增益与 Riccati 方程。给定高斯初始分布及线性高斯模型，这一估计同时具有条件均值的解释。
{% endfold %}

## 3. 经典 KF：离散时间公式

### 3.1 模型与递推

将上次后验推进到本次采样时刻：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
x_k&=F_{k-1}x_{k-1}+B_{d,k-1}u_{k-1}+w_{k-1},\\
z_k&=H_kx_k+v_k,\\
\mathbb E[w_{k-1}w_{k-1}^T]&=Q_{d,k-1},\qquad
\mathbb E[v_kv_k^T]=R_{d,k}.
\end{aligned}
{% endmathjax %}

**第一步：预测。**

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{\hat x_k^-=F_{k-1}\hat x_{k-1}^++B_{d,k-1}u_{k-1}},\qquad
\boxed{P_k^-=F_{k-1}P_{k-1}^+F_{k-1}^T+Q_{d,k-1}}.
{% endmathjax %}

**第二步：计算创新及其不确定性。**

{% mathjax '{"conversion":{"display":true}}' %}
\nu_k=z_k-H_k\hat x_k^-,\qquad S_k=H_kP_k^-H_k^T+R_{d,k}.
{% endmathjax %}

**第三步：计算增益，校正状态。**

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{K_k=P_k^-H_k^TS_k^{-1}},\qquad
\boxed{\hat x_k^+=\hat x_k^-+K_k\nu_k}.
{% endmathjax %}

**第四步：更新误差协方差。** 适合浮点实现的 Joseph 形式为：

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{P_k^+=(I-K_kH_k)P_k^-(I-K_kH_k)^T+K_kR_{d,k}K_k^T}.
{% endmathjax %}

在精确算术和最优增益下，它可化简为 {% mathjax %}P_k^+=(I-K_kH_k)P_k^-{% endmathjax %}。后者运算较少，但有限精度下更容易损失对称性与半正定性。

标量、直接观测时 {% mathjax %}K=P^-/(P^-+R_d){% endmathjax %}：预测越不可靠、观测越可靠，观测对更新的影响越大。一般矩阵增益不是简单的百分比，元素也不要求全部位于 0 到 1 之间。

{% fold info @推导：预测误差与预测协方差 %}
预测误差为：

{% mathjax '{"conversion":{"display":true}}' %}
e_k^-=F_{k-1}e_{k-1}^++w_{k-1}.
{% endmathjax %}

对外积取期望，并利用上次后验误差与本次过程噪声不相关：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
P_k^-&=\mathbb E[(F e^++w)(F e^++w)^T]\\
&=FP_{k-1}^+F^T+Q_{d,k-1}.
\end{aligned}
{% endmathjax %}

已知输入对真实系统和预测模型的贡献相同，因此在误差方程中相消。输入误差若不可忽略，就要映射进过程噪声，或扩大状态模型。
{% endfold %}

{% fold info @推导：Joseph 形式与最小均方误差增益 %}
略去本次下标。对任意增益 {% mathjax %}K{% endmathjax %}：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
e^+&=x-\hat x^--K(z-H\hat x^-)
=(I-KH)e^- -Kv,\\
P^+&=(I-KH)P^-(I-KH)^T+KR_dK^T.
\end{aligned}
{% endmathjax %}

这就是 Joseph 形式。展开并记 {% mathjax %}S=HP^-H^T+R_d{% endmathjax %}：

{% mathjax '{"conversion":{"display":true}}' %}
P^+=P^- -KHP^- -P^-H^TK^T+KSK^T.
{% endmathjax %}

最小化均方误差 {% mathjax %}\mathbb E[\|e^+\|^2]=\operatorname{tr}(P^+){% endmathjax %}，对矩阵 {% mathjax %}K{% endmathjax %} 求导：

{% mathjax '{"conversion":{"display":true}}' %}
\frac{\partial\operatorname{tr}(P^+)}{\partial K}
=-2P^-H^T+2KS=0
\quad\Longrightarrow\quad K=P^-H^TS^{-1}.
{% endmathjax %}

也可以配方得到 {% mathjax %}P^+=P^- -P^-H^TS^{-1}HP^-+(K-K_*)S(K-K_*)^T{% endmathjax %}。当 {% mathjax %}S\succ0{% endmathjax %} 时，最后一项证明这一增益确实给出最小值。将最优增益代回即可得到简化协方差公式。
{% endfold %}

### 3.2 连续模型如何变成离散模型

若一个采样周期内 {% mathjax %}A,B,G,Q_c{% endmathjax %} 恒定，输入采用零阶保持，周期为 {% mathjax %}T{% endmathjax %}，则：

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{F=e^{AT}},\qquad
\boxed{B_d=\int_0^T e^{As}B\,\mathrm ds},\qquad
\boxed{Q_d=\int_0^T e^{As}GQ_cG^Te^{A^Ts}\,\mathrm ds}.
{% endmathjax %}

短周期的一阶近似为 {% mathjax %}F\approx I+AT,\ B_d\approx BT,\ Q_d\approx GQ_cG^TT{% endmathjax %}。必须验证近似误差；“运行频率高”本身不是误差足够小的证明。

{% fold info @推导：零阶保持与离散过程噪声 %}
线性随机系统在一个周期内的解为：

{% mathjax '{"conversion":{"display":true}}' %}
x(t+T)=e^{AT}x(t)+\int_0^T e^{A(T-s)}Bu\,\mathrm ds
+\underbrace{\int_0^T e^{A(T-s)}G\,\mathrm dw(s)}_{w_d}.
{% endmathjax %}

输入恒定时，可以提出积分号。变量替换后得到 {% mathjax %}B_d{% endmathjax %}。利用 Itô 等距关系计算随机积分的协方差，得到：

{% mathjax '{"conversion":{"display":true}}' %}
\mathbb E[w_dw_d^T]
=\int_0^T e^{A(T-s)}GQ_cG^Te^{A^T(T-s)}\,\mathrm ds=Q_d.
{% endmathjax %}

因此 {% mathjax %}Q_d{% endmathjax %} 一般随采样间隔变化，而且通常具有非零的交叉项。
{% endfold %}

{% note warning %}
**不要把连续噪声强度直接填进离散协方差。** 离散编码器的 {% mathjax %}R_d{% endmathjax %} 应由实际采样输出的误差方差确定。只有当观测确实是上述连续白噪声观测在周期内的平均值时，平均噪声才有 {% mathjax %}R_d=R_c/T{% endmathjax %}；此时观测的是周期平均状态，仍需处理它与采样时刻状态的关系。对普通编码器不能机械套用这个换算。
{% endnote %}

## 4. 应用一：单电机的角度与角速度估计

### 4.1 从机械方程得到状态空间模型

假设电流环明显快于外层估计周期，电流产生的轴端力矩可以视为已知输入。忽略弹性传动、饱和及库仑摩擦的非线性，机械方程为：

{% mathjax '{"conversion":{"display":true}}' %}
J\dot\omega=\tau-b\omega+d_\tau,\qquad
\dot\theta=\omega,\qquad \tau=K_t i.
{% endmathjax %}

{% mathjax %}J{% endmathjax %} 是折算到所选轴的总惯量，{% mathjax %}b{% endmathjax %} 是黏性阻尼，{% mathjax %}K_t{% endmathjax %} 是该轴的等效力矩常数。电流优先使用时间对齐后的实际反馈；用电流指令代替时，要把电流跟踪误差计入模型不确定性。

令 {% mathjax %}x=[\theta,\omega]^T{% endmathjax %}，以 {% mathjax %}\tau{% endmathjax %} 为输入：

{% mathjax '{"conversion":{"display":true}}' %}
A=\begin{bmatrix}0&1\\0&-b/J\end{bmatrix},\quad
B=G=\begin{bmatrix}0\\1/J\end{bmatrix},\quad
H=\begin{bmatrix}1&0\end{bmatrix},\quad z=\theta+v.
{% endmathjax %}

此时 {% mathjax %}Q_c=q_\tau{% endmathjax %} 是随机扰动力矩的连续噪声强度。若输入选择电流，就将 {% mathjax %}B{% endmathjax %} 换为 {% mathjax %}[0,K_t/J]^T{% endmathjax %}；过程噪声映射仍由扰动的物理定义决定。

{% fold info @推导：单电机的可观测性 %}
连续可观测性矩阵为：

{% mathjax '{"conversion":{"display":true}}' %}
\mathcal O=\begin{bmatrix}H\\HA\end{bmatrix}
=\begin{bmatrix}1&0\\0&1\end{bmatrix}.
{% endmathjax %}

它满秩，因此在已知输入和模型成立的条件下，仅测角度也能估计角速度。离散模型中 {% mathjax %}\mathcal O_d=[H^T,(HF)^T]^T{% endmathjax %} 的行列式为后文的 {% mathjax %}r(T){% endmathjax %}，在 {% mathjax %}T>0{% endmathjax %} 时不为零。可观测不代表任意噪声和任意采样周期下都能获得同样好的精度。
{% endfold %}

### 4.2 精确离散化

记 {% mathjax %}a=b/J{% endmathjax %}，并定义：

{% mathjax '{"conversion":{"display":true}}' %}
E=e^{-aT},\qquad r(T)=\frac{1-E}{a},\qquad
s(T)=\frac{T-r(T)}{a}.
{% endmathjax %}

于是零阶保持模型为：

{% mathjax '{"conversion":{"display":true}}' %}
F=\begin{bmatrix}1&r(T)\\0&E\end{bmatrix},\qquad
B_d=\frac1J\begin{bmatrix}s(T)\\r(T)\end{bmatrix}.
{% endmathjax %}

当 {% mathjax %}a\to0{% endmathjax %} 时，{% mathjax %}r(T)\to T,\ s(T)\to T^2/2{% endmathjax %}。实现时必须处理这个极限，避免除零或小数相减损失精度。

对应连续白噪声力矩，令 {% mathjax %}q_a=q_\tau/J^2{% endmathjax %}，则：

{% mathjax '{"conversion":{"display":true}}' %}
Q_d=q_a\begin{bmatrix}
\displaystyle\frac{T-2r(T)+(1-E^2)/(2a)}{a^2}&\displaystyle\frac{r(T)^2}{2}\\
\displaystyle\frac{r(T)^2}{2}&\displaystyle\frac{1-E^2}{2a}
\end{bmatrix}.
{% endmathjax %}

零阻尼极限为：

{% mathjax '{"conversion":{"display":true}}' %}
Q_d=q_a\begin{bmatrix}T^3/3&T^2/2\\T^2/2&T\end{bmatrix}.
{% endmathjax %}

{% fold info @推导：电机离散矩阵及噪声交叉项 %}
恒定输入下，速度解是 {% mathjax %}\omega(t+T)=E\omega(t)+\tau r(T)/J{% endmathjax %}。再对速度积分，得到 {% mathjax %}\theta(t+T)=\theta(t)+r(T)\omega(t)+\tau s(T)/J{% endmathjax %}。

过程噪声在剩余时间 {% mathjax %}s{% endmathjax %} 的传播向量为：

{% mathjax '{"conversion":{"display":true}}' %}
e^{As}G=\frac1J\begin{bmatrix}r(s)\\e^{-as}\end{bmatrix}.
{% endmathjax %}

将外积积分就得到 {% mathjax %}Q_d{% endmathjax %}。例如 {% mathjax %}r'(s)=e^{-as}{% endmathjax %}，故角度与角速度的交叉项为：

{% mathjax '{"conversion":{"display":true}}' %}
Q_{\theta\omega}=q_a\int_0^T r(s)r'(s)\,\mathrm ds=q_a r(T)^2/2.
{% endmathjax %}

零阻尼时，向量变为 {% mathjax %}[s,1]^T/J{% endmathjax %}，积分直接给出 {% mathjax %}T^3/3,T^2/2,T{% endmathjax %}。若另假设“每周期抽取一个恒定随机加速度”，得到的是另一种 {% mathjax %}Q_d{% endmathjax %}；不能混用两种噪声假设的公式和参数。
{% endfold %}

### 4.3 实际使用步骤

1. 统一轴端、方向和单位：角度用 rad，速度用 rad/s，力矩用 N·m；齿轮比与效率要反映在轴端惯量和等效力矩常数中。
2. 用静止数据估计编码器误差方差 {% mathjax %}R_d{% endmathjax %}。运动中若量化、振动或传输误差明显，补充相应数据，不将静态方差视为万能值。
3. 用第一次有效角度初始化 {% mathjax %}\hat\theta{% endmathjax %}，速度未知时先设为零，并给较大的速度初始方差。
4. 每周期用实际时间间隔和对应区间的输入执行预测；有新的有效观测才校正一次。
5. 用记录数据检查速度估计、创新、延迟和协方差。增大过程噪声通常让滤波器更愿意修正模型预测；增大观测噪声通常减弱观测影响。

模型误差持续有方向时，单纯放大 {% mathjax %}Q_d{% endmathjax %} 无法代替建模。例如固定负载力矩可以扩成扰动状态，编码器偏置可以扩成偏置状态，但要重新检查可观测性。

## 5. 非线性系统：线性化与 EKF

### 5.1 局部线性化的含义

对离散非线性系统：

{% mathjax '{"conversion":{"display":true}}' %}
x_k=f_d(x_{k-1},u_{k-1})+w_{k-1},\qquad z_k=h(x_k)+v_k,
{% endmathjax %}

EKF 在当前估计附近对**误差**做一阶近似。Jacobian 为：

{% mathjax '{"conversion":{"display":true}}' %}
F_{k-1}=\left.\frac{\partial f_d}{\partial x}\right|_{\hat x_{k-1}^+,u_{k-1}},\qquad
H_k=\left.\frac{\partial h}{\partial x}\right|_{\hat x_k^-}.
{% endmathjax %}

不要将状态预测误写成 {% mathjax %}\hat x^-=F\hat x^+{% endmathjax %}。通常 {% mathjax %}f_d(\hat x,u)\ne F\hat x{% endmathjax %}；只有线性模型或已正确保留仿射项时才能这样处理。

{% fold info @推导：Taylor 展开如何得到 EKF %}
在上次后验附近展开：

{% mathjax '{"conversion":{"display":true}}' %}
f_d(x,u)\approx f_d(\hat x^+,u)+F(x-\hat x^+).
{% endmathjax %}

选择 {% mathjax %}\hat x^-=f_d(\hat x^+,u){% endmathjax %} 后，误差满足 {% mathjax %}e^-\approx Fe^++w{% endmathjax %}，所以预测协方差仍按线性误差模型传播。观测函数在本次先验处展开：

{% mathjax '{"conversion":{"display":true}}' %}
h(x)\approx h(\hat x^-)+He^-.
{% endmathjax %}

创新变为 {% mathjax %}\nu=z-h(\hat x^-)\approx He^-+v{% endmathjax %}，于是可以沿用 KF 的增益和误差更新。被丢弃的高阶项正是线性化误差的一部分，模型强非线性、初始误差较大时可能明显影响一致性与收敛。
{% endfold %}

### 5.2 连续时间 EKF

考虑加性噪声连续系统：

{% mathjax '{"conversion":{"display":true}}' %}
\mathrm dx=f_c(x,u)\,\mathrm dt+G\,\mathrm dw,\qquad
\mathrm dy=h(x)\,\mathrm dt+\mathrm dv.
{% endmathjax %}

令 {% mathjax %}A=\partial f_c/\partial x{% endmathjax %}、{% mathjax %}H=\partial h/\partial x{% endmathjax %}，均在当前估计处计算，则连续 EKF 为：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
\mathrm d\hat x&=f_c(\hat x,u)\,\mathrm dt+K_c[\mathrm dy-h(\hat x)\,\mathrm dt],\\
K_c&=PH^TR_c^{-1},\\
\dot P&=AP+PA^T+GQ_cG^T-PH^TR_c^{-1}HP.
\end{aligned}
{% endmathjax %}

**连续动力学 + 离散传感器**则是另一种常见组合：在两次观测之间积分 {% mathjax %}\dot{\hat x}=f_c(\hat x,u){% endmathjax %} 和 {% mathjax %}\dot P=AP+PA^T+GQ_cG^T{% endmathjax %}，收到观测时再做一次离散校正。不要同时连续注入同一观测、又将它当作新观测重复离散校正。

### 5.3 离散时间 EKF

**预测：**

{% mathjax '{"conversion":{"display":true}}' %}
\hat x_k^-=f_d(\hat x_{k-1}^+,u_{k-1}),\qquad
P_k^-=F_{k-1}P_{k-1}^+F_{k-1}^T+Q_{d,k-1}.
{% endmathjax %}

**校正：**

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
\nu_k&=z_k-h(\hat x_k^-),& S_k&=H_kP_k^-H_k^T+R_{d,k},\\
K_k&=P_k^-H_k^TS_k^{-1},& \hat x_k^+&=\hat x_k^-+K_k\nu_k,\\
P_k^+&=(I-K_kH_k)P_k^-(I-K_kH_k)^T+K_kR_{d,k}K_k^T.
\end{aligned}
{% endmathjax %}

若噪声以 {% mathjax %}f_d(x,u,w){% endmathjax %}、{% mathjax %}h(x,v){% endmathjax %} 的方式非加性进入，还需对噪声求 Jacobian，将对应项改为 {% mathjax %}LQ_dL^T{% endmathjax %} 与 {% mathjax %}MR_dM^T{% endmathjax %}。本文代码使用加性噪声，避免把这两种情况混在一起。

## 6. 应用二：yaw + pitch 云台的耦合模型

### 6.1 几何、状态与质量归属

采用底座固定、yaw 轴竖直、pitch 轴水平且相交的两自由度云台。定义 yaw 角 {% mathjax %}\theta{% endmathjax %}，pitch 角 {% mathjax %}\phi{% endmathjax %}；俯仰组件水平时 {% mathjax %}\phi=0{% endmathjax %}，抬头为正。力矩正方向与角度正方向一致。

**pitch 电机的机壳、支架随 yaw 转动；转子及安装在俯仰输出端的机构还随 pitch 转动。** 两部分都贡献 yaw 惯量，必须按实际固定关系归属，不能只在 pitch 方程里考虑 pitch 电机。

令俯仰运动组件的质量为 {% mathjax %}m{% endmathjax %}，质心沿其水平零位前向距轴交点 {% mathjax %}l{% endmathjax %}。其质心主惯量为 {% mathjax %}I_x,I_y,I_z{% endmathjax %}，其中 body-y 为 pitch 轴，body-x 为零位前向。假设主轴对齐、无惯性积。定义：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
J_p&=I_y+ml^2,\\
I_c&=I_z+ml^2,\qquad I_s=I_x,\\
D(\phi)&=J_0+I_c\cos^2\phi+I_s\sin^2\phi.
\end{aligned}
{% endmathjax %}

{% mathjax %}J_0{% endmathjax %} 包含 yaw 本体、随 yaw 但不随 pitch 倾转的支架和 pitch 电机机壳等绕 yaw 轴的惯量；俯仰输出侧的电机转子、负载等计入 {% mathjax %}I_x,I_y,I_z,m,l{% endmathjax %} 对应的组件。这样既不遗漏，也不重复计算。减速器高速转子等效惯量在高精度模型中还需单独折算，本文采用整体等效参数。

{% note secondary %}
这个模型用于展示两轴耦合及滤波器实现。轴不相交、存在惯性积、底座运动、传动柔性或摩擦显著非线性时，需要重新写能量或刚体方程。已有云台笔记的符号不能直接移植到另一套角度定义；下面以本节的能量定义为准。
{% endnote %}

### 6.2 动力学方程

记 {% mathjax %}b_y,b_p{% endmathjax %} 为两轴黏性阻尼，{% mathjax %}\tau_y,\tau_p{% endmathjax %} 为轴端输入力矩。模型为：

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{D(\phi)\ddot\theta+D'(\phi)\dot\phi\dot\theta+b_y\dot\theta=\tau_y},
{% endmathjax %}

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{J_p\ddot\phi-\frac12D'(\phi)\dot\theta^2+mgl\cos\phi+b_p\dot\phi=\tau_p},
{% endmathjax %}

其中：

{% mathjax '{"conversion":{"display":true}}' %}
D'=2(I_s-I_c)\sin\phi\cos\phi,\qquad
D''=2(I_s-I_c)(\cos^2\phi-\sin^2\phi).
{% endmathjax %}

两轴耦合来自 {% mathjax %}D'{% endmathjax %}：pitch 改变 yaw 的惯量，yaw 转速也会影响 pitch 加速度。即使两个电机分别闭环控制，也不能据此断言机械动力学彼此独立。

{% fold info @推导：从能量得到两轴耦合、离心与重力项 %}
俯仰组件的质心坐标可以写为：

{% mathjax '{"conversion":{"display":true}}' %}
r_c=l\begin{bmatrix}\cos\phi\cos\theta\\\cos\phi\sin\theta\\\sin\phi\end{bmatrix},\quad
\|\dot r_c\|^2=l^2(\dot\phi^2+\cos^2\phi\dot\theta^2).
{% endmathjax %}

绕质心的旋转动能贡献 yaw 项 {% mathjax %}(I_z\cos^2\phi+I_x\sin^2\phi)\dot\theta^2/2{% endmathjax %} 和 pitch 项 {% mathjax %}I_y\dot\phi^2/2{% endmathjax %}。再加上质心平移动能和 yaw 固定组件的动能：

{% mathjax '{"conversion":{"display":true}}' %}
\mathcal T=\frac12D(\phi)\dot\theta^2+\frac12J_p\dot\phi^2,\qquad
\mathcal V=mgl\sin\phi.
{% endmathjax %}

取 {% mathjax %}\mathcal L=\mathcal T-\mathcal V{% endmathjax %}，对 {% mathjax %}q\in\{\theta,\phi\}{% endmathjax %} 应用：

{% mathjax '{"conversion":{"display":true}}' %}
\frac{\mathrm d}{\mathrm dt}\frac{\partial\mathcal L}{\partial\dot q}
-\frac{\partial\mathcal L}{\partial q}=\tau_q-b_q\dot q.
{% endmathjax %}

yaw 方程中 {% mathjax %}\mathrm d(D\dot\theta)/\mathrm dt=D\ddot\theta+D'\dot\phi\dot\theta{% endmathjax %}。pitch 方程中 {% mathjax %}\partial\mathcal T/\partial\phi=D'\dot\theta^2/2{% endmathjax %}，而 {% mathjax %}\partial\mathcal V/\partial\phi=mgl\cos\phi{% endmathjax %}，于是得到正文两式。

检查特例：当俯仰组件可近似为距轴 {% mathjax %}l{% endmathjax %} 的质点时，{% mathjax %}I_c=ml^2,I_s=0{% endmathjax %}，故 {% mathjax %}-D'\dot\theta^2/2=+ml^2\sin\phi\cos\phi\dot\theta^2{% endmathjax %} 出现在 pitch 方程左侧。重力项在 {% mathjax %}\phi=0{% endmathjax %} 为 {% mathjax %}+mgl{% endmathjax %}，对应需要正向力矩保持水平；这两个检查有助于发现符号错误。
{% endfold %}

### 6.3 写成非线性状态方程

选择：

{% mathjax '{"conversion":{"display":true}}' %}
x=\begin{bmatrix}\theta&\phi&\omega_y&\omega_p\end{bmatrix}^T,\qquad
u=\begin{bmatrix}\tau_y&\tau_p\end{bmatrix}^T.
{% endmathjax %}

令两轴加速度为：

{% mathjax '{"conversion":{"display":true}}' %}
a_y=\frac{\tau_y-b_y\omega_y-D'\omega_p\omega_y}{D},\qquad
a_p=\frac{\tau_p-b_p\omega_p+\tfrac12D'\omega_y^2-mgl\cos\phi}{J_p}.
{% endmathjax %}

则：

{% mathjax '{"conversion":{"display":true}}' %}
f_c(x,u)=\begin{bmatrix}\omega_y\\\omega_p\\a_y\\a_p\end{bmatrix},\qquad
h(x)=\begin{bmatrix}\theta\\\phi\end{bmatrix},\qquad
H=\begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix}.
{% endmathjax %}

本文仅用两轴编码器，观测函数本身是线性的；非线性来自动力学。如果加入 IMU，机体系陀螺仪输出通常不直接等于 {% mathjax %}[\dot\theta,\dot\phi]{% endmathjax %}，需要先根据安装姿态推导观测函数。

### 6.4 完整 Jacobian 与离散化

连续动力学的 Jacobian 为：

{% mathjax '{"conversion":{"display":true}}' %}
A(x,u)=\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
0&a_{y,\phi}&a_{y,\omega_y}&a_{y,\omega_p}\\
0&a_{p,\phi}&a_{p,\omega_y}&a_{p,\omega_p}
\end{bmatrix},
{% endmathjax %}

各非零偏导是：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
a_{y,\phi}&=\frac{-D''\omega_p\omega_y-a_yD'}{D},&
a_{y,\omega_y}&=\frac{-b_y-D'\omega_p}{D},&
a_{y,\omega_p}&=-\frac{D'\omega_y}{D},\\
a_{p,\phi}&=\frac{\tfrac12D''\omega_y^2+mgl\sin\phi}{J_p},&
a_{p,\omega_y}&=\frac{D'\omega_y}{J_p},&
a_{p,\omega_p}&=-\frac{b_p}{J_p}.
\end{aligned}
{% endmathjax %}

{% fold info @推导：状态相关惯量为什么还会贡献 Jacobian 项 %}
写 {% mathjax %}a_y=N/D{% endmathjax %}，其中 {% mathjax %}N=\tau_y-b_y\omega_y-D'\omega_p\omega_y{% endmathjax %}。商法则给出：

{% mathjax '{"conversion":{"display":true}}' %}
\frac{\partial a_y}{\partial\phi}
=\frac{N'D-ND'}{D^2}
=\frac{-D''\omega_p\omega_y-a_yD'}{D}.
{% endmathjax %}

只对分子求导会遗漏 {% mathjax %}-a_yD'/D{% endmathjax %}。pitch 加速度的分母 {% mathjax %}J_p{% endmathjax %} 恒定；重力项的导数为 {% mathjax %}\partial(-mgl\cos\phi)/\partial\phi=mgl\sin\phi{% endmathjax %}，其余项逐项求导即可得到正文结果。
{% endfold %}

为使模型、公式和代码一一对应，下面采用前向 Euler：

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{f_d(x,u)=x+Tf_c(x,u)},\qquad \boxed{F=I+TA(x,u)}.
{% endmathjax %}

示例用零均值、常强度的**加性角加速度噪声**近似未建模扰动：

{% mathjax '{"conversion":{"display":true}}' %}
G=\begin{bmatrix}0&0\\0&0\\1&0\\0&1\end{bmatrix},\quad
Q_c=\operatorname{diag}(q_y,q_p),\quad
Q_d\approx T GQ_cG^T.
{% endmathjax %}

这是一阶噪声离散化，忽略周期内传播到角度的高阶项。它与前面电机的精确噪声离散化精度不同。若从随机扰动力矩出发，则噪声映射含 {% mathjax %}1/D(\phi){% endmathjax %}、{% mathjax %}1/J_p{% endmathjax %}，形成状态相关噪声；不能仍将其当作同一个常量加性噪声模型。

默认周期是 1 ms，代码接受实际 {% mathjax %}T{% endmathjax %}，并拒绝非正、非有限或超过 10 ms 的步长。10 ms 是教学接口的保护上限，不是稳定性保证。大速度、大协方差或快速耦合运动时应减小步长，或使用更高阶积分；换用 RK4 后还需同步计算对应离散映射的 Jacobian，不能继续声称 {% mathjax %}I+TA{% endmathjax %} 是精确导数。

## 7. 无迹卡尔曼滤波 UKF

### 7.1 为什么使用 Sigma 点

非线性函数一般满足 {% mathjax %}\mathbb E[f(x)]\ne f(\mathbb E[x]){% endmathjax %}。EKF 用均值处的函数值预测状态，并用一阶导数传播协方差；分布范围较大时，曲率的影响可能显著。

**无迹变换（Unscented Transform，UT）用少量确定性的 Sigma 点匹配输入分布的均值和协方差，将这些点逐一通过非线性函数，再重构输出统计量。** UKF 把这种传播方式放进预测与校正流程，不需要解析 Jacobian。

这些点不是随机抽样，也不是粒子滤波中大量带概率权重的粒子。UKF 仍以均值和协方差近似分布，不能完整表达多峰后验；它也不能保证在所有非线性问题上都比 EKF 准确或稳定。无迹变换与 UKF 的原始算法可参考 [Wan 与 van der Merwe 的论文](https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf)。

### 7.2 Sigma 点及权重

给定 {% mathjax %}n{% endmathjax %} 维均值 {% mathjax %}\mu{% endmathjax %} 和协方差 {% mathjax %}P{% endmathjax %}，定义缩放参数：

{% mathjax '{"conversion":{"display":true}}' %}
\lambda=\alpha^2(n+\kappa)-n,\qquad c=n+\lambda>0.
{% endmathjax %}

取 Cholesky 分解 {% mathjax %}P=LL^T{% endmathjax %}，使用**下三角矩阵的列**生成点：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
\chi_0&=\mu,\\
\chi_i&=\mu+\sqrt c\,L_{:,i},\qquad i=1,\ldots,n,\\
\chi_{i+n}&=\mu-\sqrt c\,L_{:,i},\qquad i=1,\ldots,n.
\end{aligned}
{% endmathjax %}

均值权重与协方差权重分别为：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
W_0^{(m)}&=\frac\lambda c,&
W_0^{(c)}&=\frac\lambda c+(1-\alpha^2+\beta),\\
W_i^{(m)}&=W_i^{(c)}=\frac1{2c},&&i=1,\ldots,2n.
\end{aligned}
{% endmathjax %}

{% mathjax %}\alpha{% endmathjax %} 控制点的扩散，{% mathjax %}\kappa{% endmathjax %} 是附加缩放参数，{% mathjax %}\beta{% endmathjax %} 利用分布先验，高斯输入常取 2。均值权重之和是 1；协方差权重不要求和为 1，不能将二者混用。

{% fold info @推导：Sigma 点如何匹配输入均值和协方差 %}
正负点成对相消，因此：

{% mathjax '{"conversion":{"display":true}}' %}
\sum_{i=0}^{2n}W_i^{(m)}\chi_i=\mu.
{% endmathjax %}

中心点的偏差为零，其额外协方差权重不影响输入协方差。其余各对点贡献：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
\sum_{i=0}^{2n}W_i^{(c)}(\chi_i-\mu)(\chi_i-\mu)^T
&=\sum_{j=1}^n\frac{2}{2c}(\sqrt c L_{:,j})(\sqrt c L_{:,j})^T\\
&=LL^T=P.
\end{aligned}
{% endmathjax %}

函数 {% mathjax %}f(\mu+\delta){% endmathjax %} 的 Taylor 展开中，成对点会抵消奇次项，并保留由协方差决定的二阶均值贡献；这说明它能表达均值附近的部分曲率效应。**匹配输入前两阶矩不意味着非线性变换后的所有统计量都精确**，高阶精度还与函数、分布、点集和参数有关。
{% endfold %}

### 7.3 离散 UKF 的完整递推

以下采用与 EKF 相同的加性噪声模型。先从 {% mathjax %}(\hat x_{k-1}^+,P_{k-1}^+){% endmathjax %} 生成 Sigma 点，逐点预测：

{% mathjax '{"conversion":{"display":true}}' %}
\chi_{k,i}^{f}=f_d(\chi_{k-1,i},u_{k-1}),\qquad
\hat x_k^-=\sum_{i=0}^{2n}W_i^{(m)}\chi_{k,i}^{f},
{% endmathjax %}

{% mathjax '{"conversion":{"display":true}}' %}
P_k^-=\sum_{i=0}^{2n}W_i^{(c)}
(\chi_{k,i}^{f}-\hat x_k^-)(\chi_{k,i}^{f}-\hat x_k^-)^T+Q_{d,k-1}.
{% endmathjax %}

**从加入过程噪声后的 {% mathjax %}(\hat x_k^-,P_k^-){% endmathjax %} 重新生成 Sigma 点 {% mathjax %}\bar\chi_{k,i}{% endmathjax %}**，再通过观测函数：

{% mathjax '{"conversion":{"display":true}}' %}
\zeta_{k,i}=h(\bar\chi_{k,i}),\qquad
\hat z_k=\sum_iW_i^{(m)}\zeta_{k,i}.
{% endmathjax %}

计算创新协方差和状态—观测互协方差：

{% mathjax '{"conversion":{"display":true}}' %}
\begin{aligned}
S_k&=\sum_iW_i^{(c)}(\zeta_{k,i}-\hat z_k)(\zeta_{k,i}-\hat z_k)^T+R_{d,k},\\
C_{xz,k}&=\sum_iW_i^{(c)}(\bar\chi_{k,i}-\hat x_k^-)(\zeta_{k,i}-\hat z_k)^T.
\end{aligned}
{% endmathjax %}

最后校正：

{% mathjax '{"conversion":{"display":true}}' %}
\boxed{K_k=C_{xz,k}S_k^{-1}},\quad
\boxed{\hat x_k^+=\hat x_k^-+K_k(z_k-\hat z_k)},\quad
\boxed{P_k^+=P_k^--K_kS_kK_k^T}.
{% endmathjax %}

{% fold info @推导：UKF 的增益与协方差更新 %}
以 {% mathjax %}\delta x=x-\hat x^-{% endmathjax %}、{% mathjax %}\delta z=z-\hat z{% endmathjax %} 构造线性校正 {% mathjax %}\delta x^+=\delta x-K\delta z{% endmathjax %}。已知其近似二阶统计量为 {% mathjax %}P^-,S,C_{xz}{% endmathjax %}，则：

{% mathjax '{"conversion":{"display":true}}' %}
P^+=P^- -KC_{xz}^T-C_{xz}K^T+KSK^T.
{% endmathjax %}

对迹求导得 {% mathjax %}KS=C_{xz}{% endmathjax %}，故 {% mathjax %}K=C_{xz}S^{-1}{% endmathjax %}，代回得到 {% mathjax %}P^+=P^--KSK^T{% endmathjax %}。若将状态和观测近似为联合高斯分布，也可以由高斯条件分布公式得到相同结果。

UT 提供近似矩，而这里的增益是在这些近似统计量下得到的线性最优校正；这不等于非线性系统的真实后验一定为高斯。
{% endfold %}

{% note warning %}
如果只在预测协方差里加 {% mathjax %}Q_d{% endmathjax %}，随后仍直接复用未包含该噪声的传播点，通常会漏掉过程噪声对观测统计量的贡献。本文采用“重新生成点”的加性 UKF 方案。另一种方案是将状态与噪声一起增广，再正确传播增广 Sigma 点；非加性噪声常需要这种处理。
{% endnote %}

### 7.4 用 UKF 估计同一云台

云台有 4 个状态，使用 9 个 Sigma 点。EKF 每次在一个状态估计处计算动力学及 Jacobian；UKF 每次在 9 个点计算同一个非线性离散动力学。两者使用相同的力矩输入、编码器角度、初始协方差和噪声定义，方便比较。

示例选择 {% mathjax %}\alpha=1,\beta=2,\kappa=0{% endmathjax %}，于是 {% mathjax %}\lambda=0{% endmathjax %}，中心均值权重为 0、中心协方差权重为 2，其余权重均为 {% mathjax %}1/8{% endmathjax %}。这是适合演示和 float32 运算的起点；需要结合实际分布调参。直接照搬很小的 {% mathjax %}\alpha{% endmathjax %} 可能产生绝对值很大的正负权重，加剧单精度抵消误差。

本文的角度状态始终使用连续解绕值，故普通加权平均适用。若状态改为周期角度，则不能将 179° 与 −179° 直接平均成 0°；应使用圆周均值及一致的角度残差。四元数等流形状态还需要专门的状态误差表示。

连续模型的应用方式仍是先获得一个周期的非线性状态转移，再按本节进行离散 UKF。连续时间 UKF 存在多种构造，本节不把离散 Sigma 点递推冒充为某个唯一的连续 UKF 方程。

## 8. STM32H7 上的 C++ 实现

### 8.1 库、编译配置与数值约定

以下代码使用 C++11 和 CMSIS-DSP 的 float32 矩阵 API，可保存为 `kalman.hpp`。状态和工作区维数在编译时固定，没有动态内存分配。滤波器对象在调用示例中使用静态存储，函数内仍有固定大小的临时矩阵，任务栈大小应依据编译器栈报告和实际调用链确定。

以 STM32H750 的 Cortex-M7 工程为例，开启 FPU，确保启动代码在使用浮点前已启用它；GCC 的典型选项为：

```text
-std=c++11 -O2 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
```

所选芯片、启动文件、编译选项及链接库的浮点 ABI 必须一致。使用 CMSIS-DSP 时加入其 `Include`、`PrivateInclude` 和 CMSIS Core 头文件路径，链接对应库，或编译本文用到的 `arm_mat_*_f32.c` 源文件；开发阶段给库源码开启 `ARM_MATH_MATRIX_CHECK`。不要同时编译单函数源码和包含它们的聚合源码。

代码所需的矩阵函数是乘法、转置、加法、减法、缩放、Cholesky 分解和上下三角求解。矩阵按行连续存储，输入输出使用不同缓冲区。描述符和布局见 [CMSIS-DSP Matrix Functions](https://arm-software.github.io/CMSIS-DSP/main/group__groupMatrix.html)；分解输出上三角需要预先清零，参见 [Cholesky 文档](https://arm-software.github.io/CMSIS-DSP/main/group__MatrixChol.html)。

公式中的 {% mathjax %}KS=C_{xz}{% endmathjax %} 在代码中通过 {% mathjax %}SK^T=C_{xz}^T{% endmathjax %} 求解。若 {% mathjax %}S=LL^T{% endmathjax %}，先解 {% mathjax %}LY=C_{xz}^T{% endmathjax %}，再解 {% mathjax %}L^TK^T=Y{% endmathjax %}。接口参考 [CMSIS-DSP 三角求解文档](https://arm-software.github.io/CMSIS-DSP/main/group__MatrixInv.html)。

{% note info %}
CMSIS-DSP 是利用处理器指令、FPU 及平台适用优化的软件库；STM32H7 并没有因为调用它就多出一个独立的矩阵乘法外设。本文未使用 Eigen。密集矩阵运算交给 CMSIS-DSP，元素赋值、求和、对称化和参数计算用普通 C++。电机离散系数采用 double 中间量及短周期级数避免消减误差，状态与矩阵仍为 float32；固定周期时可缓存这些系数。
{% endnote %}

### 8.2 公共矩阵工具与校正核心

下面三个代码块按顺序组成同一个头文件。公共校正函数针对线性观测；云台示例的 {% mathjax %}h(x){% endmathjax %} 恰好为线性函数，因此 EKF 也可以直接使用它。一般非线性观测应单独计算 {% mathjax %}h(\hat x^-){% endmathjax %} 及 {% mathjax %}H{% endmathjax %}，不能仅把 Jacobian 当作原观测函数。

代码通过归一化创新平方（NIS）筛选异常观测：

{% mathjax %}\operatorname{NIS}=\nu^TS^{-1}\nu.{% endmathjax %}

示例阈值为一维观测 9、二维观测 11.83，近似对应高斯一致模型下的 99.73% 卡方分位。实际使用要结合日志调整；持续拒绝通常需要检查初始化、延时、噪声或模型。拒绝校正时保留本次预测，不伪装成校正成功。

{% fold info @C++：公共矩阵工具与校正核心 %}
```cpp
#pragma once
#include "arm_math.h"
#include <cmath>
#include <cstddef>

namespace kf {
template<int R, int C> struct Mat {
    float32_t d[R*C]{}; // row-major; no heap
    float32_t& operator()(int r, int c) { return d[r*C+c]; }
    float32_t operator()(int r, int c) const { return d[r*C+c]; }
    arm_matrix_instance_f32 view() {
        return {uint16_t(R), uint16_t(C), d};
    }
    arm_matrix_instance_f32 view() const {
        // CMSIS descriptor lacks const pData; the selected input APIs are read-only.
        return {uint16_t(R), uint16_t(C), const_cast<float32_t*>(d)};
    }
};
template<int R, int C> bool finite(const Mat<R,C>& a) {
    for (float v : a.d) if (!std::isfinite(v)) return false;
    return true;
}
template<int R, int K, int C>
bool mul(const Mat<R,K>& a, const Mat<K,C>& b, Mat<R,C>& c) {
    auto av=a.view(), bv=b.view(), cv=c.view();
    return arm_mat_mult_f32(&av,&bv,&cv)==ARM_MATH_SUCCESS;
}
template<int R, int C>
bool trans(const Mat<R,C>& a, Mat<C,R>& b) {
    auto av=a.view(), bv=b.view();
    return arm_mat_trans_f32(&av,&bv)==ARM_MATH_SUCCESS;
}
template<int R, int C>
bool add(const Mat<R,C>& a, const Mat<R,C>& b, Mat<R,C>& c) {
    auto av=a.view(), bv=b.view(), cv=c.view();
    return arm_mat_add_f32(&av,&bv,&cv)==ARM_MATH_SUCCESS;
}
template<int R, int C>
bool sub(const Mat<R,C>& a, const Mat<R,C>& b, Mat<R,C>& c) {
    auto av=a.view(), bv=b.view(), cv=c.view();
    return arm_mat_sub_f32(&av,&bv,&cv)==ARM_MATH_SUCCESS;
}
template<int R, int C>
bool scale(const Mat<R,C>& a, float s, Mat<R,C>& b) {
    auto av=a.view(), bv=b.view();
    return arm_mat_scale_f32(&av,s,&bv)==ARM_MATH_SUCCESS;
}
template<int N> Mat<N,N> identity() {
    Mat<N,N> a; for(int i=0;i<N;++i) a(i,i)=1.0f; return a;
}
template<int N> void sym(Mat<N,N>& p) {
    for(int i=0;i<N;++i) for(int j=0;j<i;++j) {
        float v=0.5f*(p(i,j)+p(j,i)); p(i,j)=p(j,i)=v;
    }
}
template<int N> bool chol(const Mat<N,N>& a, Mat<N,N>& l) {
    l=Mat<N,N>{}; // CMSIS may leave upper triangle untouched
    if(!finite(a)) return false;
    auto av=a.view(), lv=l.view();
    return arm_mat_cholesky_f32(&av,&lv)==ARM_MATH_SUCCESS && finite(l);
}
// Solve S X = B, with S positive definite. All buffers are distinct.
template<int N, int C>
bool solveSpd(const Mat<N,N>& s, const Mat<N,C>& b, Mat<N,C>& x) {
    Mat<N,N> l,lt; Mat<N,C> y;
    if(!chol(s,l)||!trans(l,lt)) return false;
    auto lv=l.view(), ltv=lt.view(), bv=b.view(), yv=y.view(), xv=x.view();
    return arm_mat_solve_lower_triangular_f32(&lv,&bv,&yv)==ARM_MATH_SUCCESS
        && arm_mat_solve_upper_triangular_f32(&ltv,&yv,&xv)==ARM_MATH_SUCCESS
        && finite(x);
}
template<int N> struct Gaussian { Mat<N,1> x; Mat<N,N> p; };
template<int N> bool valid(Gaussian<N>& g) {
    sym(g.p); Mat<N,N> l;
    return finite(g.x)&&chol(g.p,l);
}
inline bool validDt(float dt) { return std::isfinite(dt)&&dt>0.0f&&dt<=0.01f; }

template<int N>
bool predictCov(const Mat<N,N>& f, const Mat<N,N>& p,
                const Mat<N,N>& q, Mat<N,N>& out) {
    Mat<N,N> ft,t,v;
    return trans(f,ft)&&mul(f,p,t)&&mul(t,ft,v)&&add(v,q,out);
}
// Linear observation update; also used by the example EKF (h is linear).
// A rejected/failed update leaves g at its prediction.
template<int N, int M>
bool correct(Gaussian<N>& g, const Mat<M,N>& h,
             const Mat<M,M>& r, const Mat<M,1>& z, float nisLimit) {
    if(!finite(z)||!finite(r)||!std::isfinite(nisLimit)||nisLimit<=0) return false;
    Mat<M,1> zh,nu,wh; Mat<N,M> ht,c,k;
    Mat<M,N> ct,kt; Mat<M,M> hc,s;
    if(!mul(h,g.x,zh)||!sub(z,zh,nu)||!trans(h,ht)||!mul(g.p,ht,c)
       ||!mul(h,c,hc)||!add(hc,r,s)||!trans(c,ct)
       ||!solveSpd(s,ct,kt)||!trans(kt,k)||!solveSpd(s,nu,wh)) return false;
    float nis=0; for(int i=0;i<M;++i) nis+=nu(i,0)*wh(i,0);
    if(!std::isfinite(nis)||nis>nisLimit) return false;
    Gaussian<N> next;
    Mat<N,1> dx; Mat<N,N> kh,a,at,ap,apat,krkt;
    Mat<N,M> kr;
    if(!mul(k,nu,dx)||!add(g.x,dx,next.x)||!mul(k,h,kh)
       ||!sub(identity<N>(),kh,a)||!trans(a,at)||!mul(a,g.p,ap)
       ||!mul(ap,at,apat)||!mul(k,r,kr)||!mul(kr,kt,krkt)
       ||!add(apat,krkt,next.p)||!valid(next)) return false;
    g=next; return true;
}
```
{% endfold %}

### 8.3 单电机 KF 与云台 EKF

电机离散系数在小阻尼、短周期下使用级数计算，避免公式中接近的量相减；零阻尼也有有限极限。`MotorKF::predict` 接收轴端力矩，需要先按实际电机参数由电流换算。

云台 EKF 使用第 6 节完整 Jacobian。代码中的 `qy`、`qp` 是加性角加速度噪声的谱密度，与电机的随机力矩谱密度 `qTau` 含义不同。

{% fold info @C++：电机 KF、云台模型与 EKF %}
```cpp
// Motor: x=[angle, rate], u=shaft torque; qTau is torque noise PSD.
struct Motor {
    float j=0.002f, b=0.004f, kt=0.1f, qTau=1.0e-5f;
    bool matrices(float dt, Mat<2,2>& f, Mat<2,1>& bt, Mat<2,2>& q) const {
        if(!validDt(dt)||!std::isfinite(j)||j<=0||!std::isfinite(b)||b<0
           ||!std::isfinite(qTau)||qTau<0) return false;
        const double a=double(b)/j, t=dt, z=a*t;
        double e=std::exp(-z), r,s,v;
        if(z<0.01) { // stable near zero damping / short sampling intervals
            r=t*(1-z/2+z*z/6-z*z*z/24+z*z*z*z/120);
            s=t*t*(0.5-z/6+z*z/24-z*z*z/120+z*z*z*z/720);
            v=t*t*t*(1.0/3-z/4+7*z*z/60-z*z*z/24+31*z*z*z*z/2520);
        } else {
            r=-std::expm1(-z)/a;
            s=(t-r)/a;
            v=(t-2*r-std::expm1(-2*z)/(2*a))/(a*a);
        }
        const double w=(a==0)?t:-std::expm1(-2*z)/(2*a);
        const double qa=double(qTau)/(double(j)*j);
        f={}; bt={}; q={};
        f(0,0)=1; f(0,1)=float(r); f(1,1)=float(e);
        bt(0,0)=float(s/j); bt(1,0)=float(r/j);
        q(0,0)=float(qa*v); q(0,1)=q(1,0)=float(qa*r*r/2);
        q(1,1)=float(qa*w);
        return finite(f)&&finite(bt)&&finite(q);
    }
};
struct MotorKF {
    Gaussian<2> g;
    Motor model;
    Mat<1,2> h;
    Mat<1,1> r;
    void init(float angle, float rate=0) {
        g={}; h={}; r={}; g.x(0,0)=angle; g.x(1,0)=rate;
        g.p(0,0)=1.0e-3f; g.p(1,1)=1.0f;
        h(0,0)=1; r(0,0)=1.0e-4f;
    }
    bool predict(float torque, float dt) {
        if(!std::isfinite(torque)) return false;
        Mat<2,2> f,q; Mat<2,1> bt,fx,bu; Mat<1,1> u;
        Gaussian<2> next; u(0,0)=torque;
        if(!model.matrices(dt,f,bt,q)||!mul(f,g.x,fx)||!mul(bt,u,bu)
           ||!add(fx,bu,next.x)||!predictCov(f,g.p,q,next.p)||!valid(next)) return false;
        g=next; return true;
    }
    bool update(float angle) {
        Mat<1,1> z; z(0,0)=angle; return correct(g,h,r,z,9.0f);
    }
};

// Gimbal: x=[yaw, pitch, yaw_rate, pitch_rate], u=[yaw_torque,pitch_torque].
// Ic, Is, Jp are whole-assembly inertias; J0 includes the carried pitch motor housing.
struct Gimbal {
    float j0=0.003f, ic=0.010f, is=0.004f, jp=0.008f;
    float by=0.01f, bp=0.01f, mgl=0.20f;
    // Constant additive acceleration PSD; not a state-dependent torque PSD.
    float qy=0.04f, qp=0.04f;
    bool flow(const Mat<4,1>& x, const Mat<2,1>& u,
              Mat<4,1>& dx, Mat<4,4>* jac=nullptr) const {
        if(!finite(x)||!finite(u)) return false;
        const float p=x(1,0), wy=x(2,0), wp=x(3,0);
        const float s=std::sin(p), c=std::cos(p);
        const float d=j0+ic*c*c+is*s*s;
        const float dp=2*(is-ic)*s*c, ddp=2*(is-ic)*(c*c-s*s);
        if(!std::isfinite(d)||d<=0||!std::isfinite(jp)||jp<=0) return false;
        const float ay=(u(0,0)-by*wy-dp*wp*wy)/d;
        const float ap=(u(1,0)-bp*wp+0.5f*dp*wy*wy-mgl*c)/jp;
        dx={}; dx(0,0)=wy; dx(1,0)=wp; dx(2,0)=ay; dx(3,0)=ap;
        if(jac) {
            *jac={}; (*jac)(0,2)=1; (*jac)(1,3)=1;
            (*jac)(2,1)=(-ddp*wp*wy-ay*dp)/d;
            (*jac)(2,2)=(-by-dp*wp)/d; (*jac)(2,3)=-dp*wy/d;
            (*jac)(3,1)=(0.5f*ddp*wy*wy+mgl*s)/jp;
            (*jac)(3,2)=dp*wy/jp; (*jac)(3,3)=-bp/jp;
        }
        return finite(dx) && (!jac || finite(*jac));
    }
    bool step(const Mat<4,1>& x, const Mat<2,1>& u, float dt,
              Mat<4,1>& out, Mat<4,4>* f=nullptr) const {
        if(!validDt(dt)) return false;
        Mat<4,1> dx,inc; Mat<4,4> a,ad;
        if(!flow(x,u,dx,f?&a:nullptr)||!scale(dx,dt,inc)||!add(x,inc,out)) return false;
        return !f || (scale(a,dt,ad)&&add(identity<4>(),ad,*f));
    }
    bool noise(float dt, Mat<4,4>& q) const {
        if(!validDt(dt)||!std::isfinite(qy)||!std::isfinite(qp)||qy<0||qp<0) return false;
        q={};
        q(2,2)=qy*dt; q(3,3)=qp*dt; // first-order continuous-noise discretization
        return true;
    }
};
inline Mat<2,4> gimbalH() {
    Mat<2,4> h; h(0,0)=h(1,1)=1; return h;
}
inline Mat<2,1> observe(const Mat<4,1>& x) {
    Mat<2,1> z; z(0,0)=x(0,0); z(1,0)=x(1,0); return z;
}
inline Gaussian<4> initialGimbal(float yaw, float pitch) {
    Gaussian<4> g; g.x(0,0)=yaw; g.x(1,0)=pitch;
    g.p(0,0)=g.p(1,1)=1.0e-3f; g.p(2,2)=g.p(3,3)=0.25f; return g;
}
inline Mat<2,2> encoderR() {
    Mat<2,2> r; r(0,0)=r(1,1)=1.0e-4f; return r;
}
struct GimbalEKF {
    Gaussian<4> g;
    Gimbal model;
    Mat<2,2> r=encoderR();
    void init(float yaw,float pitch) { g=initialGimbal(yaw,pitch); }
    bool predict(const Mat<2,1>& u,float dt) {
        Gaussian<4> next; Mat<4,4> f,q;
        if(!model.step(g.x,u,dt,next.x,&f)||!model.noise(dt,q)
           ||!predictCov(f,g.p,q,next.p)||!valid(next)) return false;
        g=next; return true;
    }
    bool update(const Mat<2,1>& z) { return correct(g,gimbalH(),r,z,11.83f); }
};
```
{% endfold %}

### 8.4 通用 UKF 核心与云台实例

UKF 的预测回调负责把一个 Sigma 点推进一个周期；观测回调负责将状态映射成观测。`update` 从已经包含过程噪声的预测协方差重新生成点。示例采用普通协方差 UKF，后验减法可能受舍入误差影响，因此对称化后还会检查正定性；高动态范围问题可进一步考虑平方根 UKF。

{% fold info @C++：UKF 核心、云台实例与角度解绕 %}
```cpp
template<int N,int M> struct UKF {
    Gaussian<N> g;
    Mat<M,M> r;
    // alpha=1, beta=2, kappa=0 => lambda=0. Positive covariance weights.
    // A starting choice for float32 examples, not an optimal universal tuning.
    static float wm(int i) { return i==0?0.0f:1.0f/(2*N); }
    static float wc(int i) { return i==0?2.0f:1.0f/(2*N); }
    Mat<N,1> points[2*N+1]; // shared workspace, object is not reentrant
    bool sigma() {
        Mat<N,N> l;
        if(!chol(g.p,l)) return false;
        points[0]=g.x; const float spread=std::sqrt(float(N));
        for(int j=0;j<N;++j) for(int k=0;k<N;++k) {
            points[1+j](k,0)=g.x(k,0)+spread*l(k,j);
            points[1+N+j](k,0)=g.x(k,0)-spread*l(k,j);
        }
        return true;
    }
    template<class Step>
    bool predict(Step step,const Mat<N,N>& q) {
        if(!finite(q)||!sigma()) return false;
        Gaussian<N> next;
        for(int i=0;i<2*N+1;++i) {
            Mat<N,1> y,w;
            if(!step(points[i],y)) return false;
            points[i]=y;
            if(!scale(y,wm(i),w)) return false;
            // scalar accumulation; dense matrix products use CMSIS
            for(int j=0;j<N;++j) next.x(j,0)+=w(j,0);
        }
        Mat<N,N> cov;
        for(int i=0;i<2*N+1;++i) {
            Mat<N,1> d; Mat<1,N> dt; Mat<N,N> outer,w;
            if(!sub(points[i],next.x,d)||!trans(d,dt)||!mul(d,dt,outer)
               ||!scale(outer,wc(i),w)) return false;
            for(int j=0;j<N*N;++j) cov.d[j]+=w.d[j];
        }
        if(!add(cov,q,next.p)||!valid(next)) return false;
        g=next; return true;
    }
    template<class Observe>
    bool update(const Mat<M,1>& z,Observe h,float nisLimit) {
        if(!finite(z)||!finite(r)||!std::isfinite(nisLimit)||nisLimit<=0||!sigma()) return false;
        // Regenerate from (x-, P-) including Q, not from the previous propagated points.
        Mat<M,1> zs[2*N+1],mean;
        for(int i=0;i<2*N+1;++i) {
            zs[i]=h(points[i]); if(!finite(zs[i])) return false;
            for(int j=0;j<M;++j) mean(j,0)+=wm(i)*zs[i](j,0);
        }
        Mat<M,M> cov,s; Mat<N,M> cross;
        for(int i=0;i<2*N+1;++i) {
            Mat<N,1> dx; Mat<M,1> dz; Mat<1,M> dzt;
            Mat<M,M> zz,wzz; Mat<N,M> xz,wxz;
            if(!sub(points[i],g.x,dx)||!sub(zs[i],mean,dz)||!trans(dz,dzt)
               ||!mul(dz,dzt,zz)||!scale(zz,wc(i),wzz)
               ||!mul(dx,dzt,xz)||!scale(xz,wc(i),wxz)) return false;
            for(int j=0;j<M*M;++j) cov.d[j]+=wzz.d[j];
            for(int j=0;j<N*M;++j) cross.d[j]+=wxz.d[j];
        }
        Mat<M,N> ct,kt; Mat<N,M> k,ks;
        Mat<M,1> nu,wh; Mat<N,1> dx; Mat<N,N> reduction;
        Gaussian<N> next;
        if(!add(cov,r,s)||!trans(cross,ct)||!solveSpd(s,ct,kt)||!trans(kt,k)
           ||!sub(z,mean,nu)||!solveSpd(s,nu,wh)) return false;
        float nis=0; for(int i=0;i<M;++i) nis+=nu(i,0)*wh(i,0);
        if(!std::isfinite(nis)||nis>nisLimit) return false;
        if(!mul(k,nu,dx)||!add(g.x,dx,next.x)||!mul(k,s,ks)
           ||!mul(ks,kt,reduction)||!sub(g.p,reduction,next.p)||!valid(next)) return false;
        g=next; return true;
    }
};
struct GimbalUKF {
    UKF<4,2> core;
    Gimbal model;
    void init(float yaw,float pitch) {
        core.g=initialGimbal(yaw,pitch); core.r=encoderR();
    }
    bool predict(const Mat<2,1>& u,float dt) {
        Mat<4,4> q; if(!model.noise(dt,q)) return false;
        return core.predict([&](const Mat<4,1>& x,Mat<4,1>& y) {
            return model.step(x,u,dt,y);
        },q);
    }
    bool update(const Mat<2,1>& z) { return core.update(z,observe,11.83f); }
};

// Each encoder needs its own unwrapper; sample-to-sample motion must be < pi.
struct Unwrap {
    bool started=false; float previous=0, continuous=0;
    bool push(float wrapped,float& out) {
        if(!std::isfinite(wrapped)) return false;
        if(!started) { previous=continuous=wrapped; started=true; }
        else {
            constexpr float pi=3.14159265358979323846f;
            continuous+=std::remainder(wrapped-previous,2*pi); previous=wrapped;
        }
        out=continuous; return std::isfinite(out);
    }
};
} // namespace kf
```
{% endfold %}

### 8.5 初始化、编码器解绕与周期调用

以下示例展示三种滤波器各自的接口。`kfStep` 中同时运行两种云台滤波器是为了做对照；产品中选择需要的一种即可。所有参数均为教学值，不能直接当作某台云台的辨识结果。

角度进入滤波器前需要解绕。`Unwrap` 要求相邻有效观测间真实转角绝对值小于 π；长时间丢帧、高速跨越多圈或编码器复位时，这个条件可能不成立，应使用编码器多圈计数或重新同步。很长时间的累计角度还应重设局部角度原点，避免 float32 数值过大而损失分辨率。

{% fold info @C++：调用示例（kalman_example.cpp） %}
```cpp
#include "kalman.hpp"

static kf::MotorKF motor;
static kf::GimbalEKF ekf;
static kf::GimbalUKF ukf;
static kf::Unwrap motorAngle,yawAngle,pitchAngle;
static bool ready=false;

// All three angles must be a valid, time-aligned initial snapshot.
bool kfInit(float motorWrapped,float yawWrapped,float pitchWrapped) {
    if(!std::isfinite(motorWrapped)||!std::isfinite(yawWrapped)
       ||!std::isfinite(pitchWrapped)) return false;
    motorAngle={}; yawAngle={}; pitchAngle={};
    float m,y,p;
    if(!motorAngle.push(motorWrapped,m)||!yawAngle.push(yawWrapped,y)
       ||!pitchAngle.push(pitchWrapped,p)) return false;
    motor.init(m); ekf.init(y,p); ukf.init(y,p); ready=true;
    return true;
}

struct StepResult {
    bool predicted[3]{}; // motor KF, gimbal EKF, gimbal UKF
    bool corrected[3]{};
};

// torques describe the preceding dt interval, not the next control command.
// New observations must belong to the end of that interval.
StepResult kfStep(float dt,float motorTorque,float yawTorque,float pitchTorque,
                  bool newMotor,float motorWrapped,
                  bool newGimbal,float yawWrapped,float pitchWrapped) {
    StepResult result;
    if(!ready||!kf::validDt(dt)) return result;
    kf::Mat<2,1> u; u(0,0)=yawTorque; u(1,0)=pitchTorque;
    result.predicted[0]=motor.predict(motorTorque,dt);
    result.predicted[1]=ekf.predict(u,dt);
    result.predicted[2]=ukf.predict(u,dt);
    float a;
    if(newMotor&&motorAngle.push(motorWrapped,a)&&result.predicted[0])
        result.corrected[0]=motor.update(a);
    if(newGimbal&&std::isfinite(yawWrapped)&&std::isfinite(pitchWrapped)) {
        kf::Mat<2,1> z;
        if(yawAngle.push(yawWrapped,z(0,0))&&pitchAngle.push(pitchWrapped,z(1,0))) {
            if(result.predicted[1]) result.corrected[1]=ekf.update(z);
            if(result.predicted[2]) result.corrected[2]=ukf.update(z);
        }
    }
    return result;
}

// At the control/estimation task, use a hardware timestamp to obtain dt.
// kfInit(...) is called once after the first complete encoder snapshot.
// const StepResult s=kfStep(dt, actualMotorCurrent*motor.model.kt,
//     actualYawTorque, actualPitchTorque,
//     motorFrameIsNew, motorEncoderRad,
//     gimbalFrameIsNew, yawEncoderRad, pitchEncoderRad);
// motor.g.x(1,0): estimated motor rate
// ekf.g.x(2,0), ekf.g.x(3,0): estimated yaw/pitch rates
// ukf.core.g.x: UKF estimate, used for comparison
```
{% endfold %}

`newMotor`、`newGimbal` 表示本次收到**尚未校正过的新观测**。没有观测时继续预测，不拿上一帧重复校正。两个云台编码器若异步到达，应实现各自的一维观测更新及对应噪声，或先进行明确的时间同步，不能仅因为缓存中同时有两个数就认为它们属于同一时刻。

`predict` 失败时状态保留在上次成功的时间点；`update` 失败时保留本次预测。上层必须记录成功状态时间戳：预测失败后暂停使用该输出，并在恢复时重新初始化或按缺失区间分段推进，不能在下一周期直接假装时间已追上。代码以 `bool` 暴露成功与否，工程中可扩成错误枚举与计数器，区分异常观测、数值故障和输入故障。

### 8.6 数值和任务调度注意事项

- {% mathjax %}P,Q_d,R_d{% endmathjax %} 应对称，初始 {% mathjax %}P{% endmathjax %} 和观测噪声通常取正定，过程噪声可半正定。本文 Cholesky 核心要求待分解矩阵正定；零方差、重复状态或病态尺度需要重新处理模型。
- 对称化 {% mathjax %}P\leftarrow(P+P^T)/2{% endmathjax %} 只修复小的反对称舍入误差，不能把错误协方差变成正确协方差。代码在分解失败时返回失败，不悄悄增大对角元素掩盖问题。
- 为保留 NaN/Inf 检查的意义，本文验证不用 `-ffast-math`；若工程启用它，需要重新评估有限数检查和异常行为。
- 矩阵描述符不长期保存到结构体中，避免复制滤波器后仍指向旧对象。UKF 点集工作区由对象持有，同一对象不能从任务和中断并发调用。
- 用定时器或硬件时间戳计算实际采样间隔。中断可负责采样和通知，滤波放在预算明确的控制任务中；CAN、UART、DMA 缓冲区的数据应先形成一致快照。
- 使用 DMA 时按 STM32H7 的内存域和 D-cache 一致性要求配置缓冲区；这属于采集链路，CMSIS-DSP 不会替应用自动维护 DMA 缓存一致性。
- 日志至少记录时间戳、输入、观测、预测、后验、创新、拒绝次数、分解失败次数及执行周期。以估计速度替换差分速度反馈后，重新检查闭环相位和延迟；滤波不是控制器稳定性的替代证明。

STM32H750 的处理器、FPU 与存储器信息以 [ST 官方产品资料](https://www.st.com/en/microcontrollers-microprocessors/stm32h750vb.html)为准。这里不提供未经板上实测的“每次只需多少微秒”。

## 9. KF、EKF、UKF 如何选择

### 9.1 能力和应用对比

| 项目 | 经典 KF | EKF | UKF |
|---|---|---|---|
| 状态与观测模型 | 线性／仿射，可时变 | 可微非线性 | 可直接计算的非线性映射 |
| 非线性处理 | 需要先得到适用的线性模型 | 当前估计附近一阶线性化 | Sigma 点通过非线性函数 |
| Jacobian | 无需非线性 Jacobian | 需要推导、自动微分或数值计算 | 不需要 Jacobian |
| 统计精度 | 线性模型下均值、协方差传播精确 | 局部一阶近似，高阶误差可能明显 | 能体现部分曲率效应，仍为近似 |
| 主要风险 | 模型、噪声或时间对齐错误 | Jacobian 错误、初值差、线性化失效 | 权重抵消、分解失败、Sigma 点进入无效区域 |
| 常见场景 | 电机线性机械模型、局部匀速／匀加速跟踪 | 耦合云台、机器人定位、姿态误差估计 | Jacobian 难维护、曲率显著且维数适中的估计 |
| 工程工作量 | 通常最少 | 模型和导数维护成本较高 | 点传播及数值维护成本较高 |
| 本文实例 | 单电机，2 状态、1 观测 | 云台，4 状态、2 观测 | 同一云台，9 个 Sigma 点 |

若电机模型确实线性，使用 UKF 不会凭空获得更多传感器信息。云台模型可微且 Jacobian 容易验证时，EKF 通常是合理起点；UKF 可以用于比较线性化误差的影响，但仍要检查模型误差是否才是主要问题。

### 9.2 计算量不能只用“大 O”判断

令状态维数为 {% mathjax %}n{% endmathjax %}、观测维数为 {% mathjax %}m{% endmathjax %}，单次状态与观测函数成本为 {% mathjax %}C_f,C_h{% endmathjax %}，Jacobian 成本为 {% mathjax %}C_J{% endmathjax %}。一般稠密协方差实现的矩阵部分具有以下典型量级：

{% mathjax '{"conversion":{"display":true}}' %}
\mathcal O(n^3+n^2m+nm^2+m^3).
{% endmathjax %}

| 成本 | KF | EKF | 本文加性 UKF |
|---|---|---|---|
| 模型相关运算 | 线性映射，常矩阵可预计算 | 大约一次状态／观测求值，加 Jacobian | {% mathjax %}2n+1{% endmathjax %} 次状态及观测求值 |
| 附加分解 | 创新协方差的求解 | 同 KF | 还需状态协方差的 Cholesky；本文预测、观测各一次 |
| 主要状态存储 | {% mathjax %}\mathcal O(n^2+m^2+nm){% endmathjax %} | 同量级，另需 Jacobian 工作区 | 同量级，额外保存 Sigma 点与传播结果 |

UKF 的外积累加也达到 {% mathjax %}\mathcal O(n^3){% endmathjax %}，所以它与 EKF 可具有相同的渐近量级，但常数、三角函数次数、内存流量和实时抖动不同。复杂动力学的 9 次求值可能比几次小矩阵乘法更贵；解析 Jacobian 特别复杂时，结论也可能改变。稀疏结构、标量观测和稳态增益还能显著降低 KF/EKF 的成本。

最终应在相同模型、噪声、数据、编译选项下比较估计误差、创新一致性、最大执行时间及内存；不能用滤波后曲线更平滑作为唯一指标。

## 10. 验证与参考资料

### 10.1 可重复检查的项目

1. **线性退化**：将同一个线性系统交给 KF、EKF 和 UKF，在容许浮点误差范围内比较均值与协方差。UKF 也要包含非零过程噪声，以检查观测阶段是否漏加其影响。
2. **Jacobian**：在不同俯仰角和角速度下，将解析 Jacobian 与有限差分对照，尤其检查状态相关惯量的商法则项。
3. **动力学特例**：静止水平时输入 {% mathjax %}\tau_y=0,\tau_p=mgl{% endmathjax %} 应保持平衡；无阻尼、无输入时能量随积分步长减小而趋于守恒。
4. **耦合与步长**：给云台两个轴同时输入，用更细步长 RK4 生成参考运动；比较 Euler 在 1 ms 和更小周期的误差，检查角速度估计及协方差正定性。
5. **采集故障**：测试 ±π 跨圈、短时丢帧、异常观测、NaN、非法步长及不可分解协方差；失败不能污染已提交状态。
6. **渲染**：执行 `npm run build`，检查连续／离散章节独立、推导可折叠、折叠内公式正常、代码完整和表格可读。

### 10.2 本文示例的验证结果

从正文提取三个头文件代码块及调用示例，使用 CMSIS-DSP 1.16.2 的实际矩阵函数源码验证，未用模拟矩阵 API 替代库实现。主机标量实现的数值检查与 Cortex-M7 交叉编译分开进行：

| 检查 | 结果 |
|---|---|
| 电机过程噪声 | 与数值积分一致，包括零阻尼及不同采样周期 |
| 线性退化，1500 次递推 | KF/EKF 一致；KF/UKF 状态最大绝对差约 {% mathjax %}6.12\times10^{-6}{% endmathjax %}，协方差元素最大差约 {% mathjax %}3.95\times10^{-7}{% endmathjax %} |
| 云台解析 Jacobian，80 组状态 | 与双精度有限差分对照，最大缩放误差约 {% mathjax %}2.41\times10^{-7}{% endmathjax %}，缩放方式为绝对差除以 {% mathjax %}1+|\text{数值导数}|{% endmathjax %} |
| 静态平衡与保守系统 | 水平重力补偿平衡成立；细步长 RK4 的无阻尼、无输入运动通过能量检查 |
| Euler 步长减半 | 1 s 运动相对细步长 RK4 的状态误差范数由约 0.0445 降到 0.0220；该范数混合角度与角速度，仅用于同一测试的收敛对照 |
| 编码器跨圈与故障 | 跨 ±π、丢观测、NaN/Inf、非法步长、异常观测、非正定协方差路径通过；失败保留应保留的状态 |
| ARM 编译／链接 | GCC 13.3.1、C++11、Cortex-M7、`fpv5-d16`、hard-float；正文调用示例及所需 CMSIS-DSP 源码通过编译和无启动文件链接，系统调用使用 newlib 占位桩 |

云台跟踪测试采用本文教学参数，真实初值为 {% mathjax %}[0.1,0.2,1,-0.3]^T{% endmathjax %}，估计器角度正确初始化、速度初值为零。输入为 {% mathjax %}\tau_y=0.015\sin(2t){% endmathjax %}、{% mathjax %}\tau_p=0.20\cos\phi+0.02\sin(4t){% endmathjax %}。参考轨迹每个 1 ms 周期内用 10 个 RK4 子步推进；编码器叠加幅值 0.004 rad 的确定性正弦扰动，每 47 个周期丢一次观测。3 s 仿真排除前 0.5 s 后，EKF 的 yaw/pitch 速度 RMSE 约为 0.00255/0.00360 rad/s，UKF 也约为 0.00255/0.00360 rad/s。**这组确定性测试用于检查代码，不构成通用精度保证，也未展示 UKF 必然优于 EKF。**

博客的 Fluid `fold`、`note` 默认同步渲染，嵌套异步 `mathjax` 时可能截断正文。仓库中的 `scripts/async-math-containers.js` 保留主题原渲染函数，将这两个容器改为等待子标签完成；公式块显式启用 display 模式。构建检查同时核对源文公式数与生成 SVG 数，以及折叠块中的推导内容，避免只看构建退出码。

**尚未进行真实 STM32H7 板上运行、实时性测量、传感器标定和闭环控制验证。** ARM 链接检查没有 MCU 启动代码和目标内存布局，不能代替实际固件工程的集成与烧录验证。

### 10.3 参考资料

- [Welch 与 Bishop：An Introduction to the Kalman Filter](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)：经典 KF 与 EKF 的入门资料。
- [Harvard：Optimal Control and Estimation，第 6 章](https://hankyang.seas.harvard.edu/OptimalControlEstimation/output-feedback.html)：连续滤波与非线性估计讲义。
- [Taghvaei 等：Kalman Filter and its Modern Extensions for the Continuous-time Nonlinear Filtering Problem](https://arxiv.org/abs/1702.07241)：连续随机滤波背景。
- [Wan 与 van der Merwe：The Unscented Kalman Filter for Nonlinear Estimation](https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf)：Sigma 点、缩放参数和 UKF 算法。
- [Arm：CMSIS-DSP Matrix Functions](https://arm-software.github.io/CMSIS-DSP/main/group__groupMatrix.html)：矩阵布局及 API。
- [Arm：Cholesky and LDLT decompositions](https://arm-software.github.io/CMSIS-DSP/main/group__MatrixChol.html)、[三角求解 API](https://arm-software.github.io/CMSIS-DSP/main/group__MatrixInv.html)：分解、求解与错误状态。
- [ST：STM32H750VB 产品资料](https://www.st.com/en/microcontrollers-microprocessors/stm32h750vb.html)：示例平台的硬件信息。

本文电机和云台的推导以文中定义的轴、惯量归属与噪声模型为准；代码参数用于说明算法流程，部署前须用实际机构数据辨识和验证。
