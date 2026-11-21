---
title: 基于反步法的云台非线性控制器
tag:
  - Math
  - 云台
  - 非线性控制
  - 反步法
index_img: /img/Eyjafjalla.avif
mathjax: true
excerpt: '基于云台动力学模型，用反步法(Backstepping)设计pitch/yaw两轴的非线性控制器，将重力、离心力与科里奥利耦合作为前馈抵消'
date: 2026-11-18 23:42:59
category: RoboMaster
---
## Introduction
云台的pitch、yaw两轴并不独立：pitch抬起后yaw轴的等效转动惯量会改变，yaw高速转动时pitch会受到离心力矩，两轴同时转动时yaw还会受到科里奥利耦合力矩，再加上pitch轴始终存在的重力矩。  
传统的串级PID把这些都当成扰动，只能被动抑制。本文在云台动力学模型的基础上，用**反步法**(Backstepping)设计非线性控制器，把重力项、离心力项和耦合项作为前馈直接抵消，使两轴的误差动态解耦且可证明稳定。

## 理论推导
### 动力学模型
由云台动力学解算可得两轴的力矩方程:
{% mathjax %}
\tau_y = \underbrace{mgl_A\cos\theta}_{重力项} + \underbrace{J_{Ay}\alpha_y}_{惯性项} + \underbrace{\left[(J_{Q\perp} - J_{Q\parallel})\sin\theta\cos\theta + ml_A(l_y + l_A\cos\theta)\sin\theta\right]\omega_z^2}_{离心力项}
{% endmathjax %}<br>

{% mathjax %}
\tau_z = \underbrace{\left[J_{Cz} + J_{Bz} + J_{Az}(\theta)\right]\alpha_z}_{惯性项} - \underbrace{2\left[(J_{Q\perp} - J_{Q\parallel})\sin\theta\cos\theta + ml_A(l_y + l_A\cos\theta)\sin\theta\right]\omega_y\omega_z}_{科里奥利耦合项}
{% endmathjax %}<br>
其中运动学关系为
{% mathjax %}
\dot{\theta} = \omega_y, \quad \ddot{\theta} = \alpha_y, \quad \dot{\psi} = \omega_z, \quad \ddot{\psi} = \alpha_z
{% endmathjax %}<br>
{% mathjax %}\theta{% endmathjax %}为pitch角，{% mathjax %}\psi{% endmathjax %}为yaw角，{% mathjax %}\tau_y{% endmathjax %}、{% mathjax %}\tau_z{% endmathjax %}分别为pitch、yaw电机的输出力矩  
两个方程中的方括号是同一个关于{% mathjax %}\theta{% endmathjax %}的函数，为书写方便，记
{% mathjax %}
\Gamma(\theta) = (J_{Q\perp} - J_{Q\parallel})\sin\theta\cos\theta + ml_A(l_y + l_A\cos\theta)\sin\theta, \quad J_z(\theta) = J_{Cz} + J_{Bz} + J_{Az}(\theta)
{% endmathjax %}<br>
其中pitch部分绕yaw轴的转动惯量随pitch角变化:
{% mathjax %}
J_{Az}(\theta) = J_{Q\perp}\cos^2\theta + J_{Q\parallel}\sin^2\theta + m(l_y + l_A\cos\theta)^2
{% endmathjax %}<br>
离心力项和科里奥利耦合项都来源于此。对{% mathjax %}\theta{% endmathjax %}求导可得
{% mathjax %}
\frac{\partial J_z}{\partial \theta} = \frac{\partial J_{Az}}{\partial \theta} = -2\Gamma(\theta)
{% endmathjax %}<br>
这正是拉格朗日方程给出的结构：pitch方程中{% mathjax %}\omega_z^2{% endmathjax %}的系数为{% mathjax %}-\frac{1}{2}\frac{\partial J_z}{\partial \theta} = \Gamma{% endmathjax %}，yaw方程中{% mathjax %}\omega_y\omega_z{% endmathjax %}的系数为{% mathjax %}\frac{\partial J_z}{\partial \theta} = -2\Gamma{% endmathjax %}
> 控制律中{% mathjax %}J_z(\theta){% endmathjax %}需要每个周期按当前的pitch角重新计算

### 状态空间方程
设
{% mathjax %}
\begin{cases}
x_1 = \theta \\
x_2 = \omega_y \\
x_3 = \psi \\
x_4 = \omega_z \\
u_1 = \tau_y \\
u_2 = \tau_z
\end{cases}
{% endmathjax %}<br>
则
{% mathjax %}
\begin{cases}
\dot{x}_1 = x_2 \\
\dot{x}_2 = \dfrac{1}{J_{Ay}}\left\{u_1 - mgl_A\cos x_1 - \left[(J_{Q\perp} - J_{Q\parallel})\sin x_1\cos x_1 + ml_A(l_y + l_A\cos x_1)\sin x_1\right]x_4^2\right\} \\
\dot{x}_3 = x_4 \\
\dot{x}_4 = \dfrac{1}{J_{Cz} + J_{Bz} + J_{Az}(x_1)}\left\{u_2 + 2\left[(J_{Q\perp} - J_{Q\parallel})\sin x_1\cos x_1 + ml_A(l_y + l_A\cos x_1)\sin x_1\right]x_2x_4\right\}
\end{cases}
{% endmathjax %}<br>
即
{% mathjax %}
\begin{cases}
\dot{x}_1 = x_2 \\
\dot{x}_2 = \dfrac{1}{J_{Ay}}\left[u_1 - mgl_A\cos x_1 - \Gamma(x_1)x_4^2\right] \\
\dot{x}_3 = x_4 \\
\dot{x}_4 = \dfrac{1}{J_z(x_1)}\left[u_2 + 2\Gamma(x_1)x_2x_4\right]
\end{cases}
{% endmathjax %}<br>
控制目标是让角度跟踪目标值: {% mathjax %}x_1 \to x_{1t}{% endmathjax %}, {% mathjax %}x_3 \to x_{3t}{% endmathjax %}

### 反步法设计
系统是两条"角度→角速度→力矩"的积分链，控制量{% mathjax %}u{% endmathjax %}只出现在角速度的方程中。反步法的思路是从离控制量最远的状态开始，逐级把下一级状态当作**虚拟控制量**来设计

#### 第一步：角度误差
设角度误差
{% mathjax %}
\delta_1 = x_{1t} - x_1, \quad \delta_3 = x_{3t} - x_3
{% endmathjax %}<br>
取李雅普诺夫函数
{% mathjax %}
V_1 = \frac{1}{2}\delta_1^2 + \frac{1}{2}\delta_3^2
{% endmathjax %}<br>
显然{% mathjax %}V_1{% endmathjax %}正定(PD)，求导
{% mathjax %}
\begin{aligned}
\dot{V}_1 &= \nabla V_1 \cdot f \\
&= \begin{bmatrix} \delta_1 & \delta_3 \end{bmatrix}\begin{bmatrix} \dot{\delta}_1 & \dot{\delta}_3 \end{bmatrix}^T \\
&= \delta_1(\dot{x}_{1t} - \dot{x}_1) + \delta_3(\dot{x}_{3t} - \dot{x}_3) \\
&= \delta_1(\dot{x}_{1t} - x_2) + \delta_3(\dot{x}_{3t} - x_4)
\end{aligned}
{% endmathjax %}<br>
令
{% mathjax %}
\dot{x}_{1t} - x_2 = -k_1\delta_1, \quad \dot{x}_{3t} - x_4 = -k_3\delta_3, \quad k_1, k_3 > 0
{% endmathjax %}<br>
则{% mathjax %}\dot{V}_1{% endmathjax %}负定(ND)，{% mathjax %}(\delta_1, \delta_3) = (0, 0){% endmathjax %}为系统的渐近稳定点
> **注**：{% mathjax %}x_2{% endmathjax %}、{% mathjax %}x_4{% endmathjax %}是状态而不是控制量，不能直接"令"它们取某个值。这里实际是在给出虚拟控制量的期望值{% mathjax %}x_{2t}{% endmathjax %}、{% mathjax %}x_{4t}{% endmathjax %}，上面的结论只在{% mathjax %}x_2 = x_{2t}{% endmathjax %}、{% mathjax %}x_4 = x_{4t}{% endmathjax %}时成立，第二步正是要处理{% mathjax %}x_2 \neq x_{2t}{% endmathjax %}的情况

记虚拟控制量
{% mathjax %}
x_{2t} = \dot{x}_{1t} + k_1\delta_1, \quad x_{4t} = \dot{x}_{3t} + k_3\delta_3
{% endmathjax %}<br>
问题变为{% mathjax %}x_2 \to x_{2t}{% endmathjax %}, {% mathjax %}x_4 \to x_{4t}{% endmathjax %}

#### 第二步：角速度误差
设角速度误差
{% mathjax %}
\delta_2 = x_{2t} - x_2, \quad \delta_4 = x_{4t} - x_4
{% endmathjax %}<br>
取李雅普诺夫函数
{% mathjax %}
V_2 = V_1 + \frac{1}{2}\delta_2^2 + \frac{1}{2}\delta_4^2
{% endmathjax %}<br>
显然{% mathjax %}V_2{% endmathjax %}正定。注意此时{% mathjax %}x_2 \neq x_{2t}{% endmathjax %}，第一步中的{% mathjax %}\dot{V}_1{% endmathjax %}要重新计算。由{% mathjax %}x_2 = x_{2t} - \delta_2{% endmathjax %}、{% mathjax %}x_4 = x_{4t} - \delta_4{% endmathjax %}得
{% mathjax %}
\begin{aligned}
\dot{V}_1 &= \delta_1(\dot{x}_{1t} - x_{2t} + \delta_2) + \delta_3(\dot{x}_{3t} - x_{4t} + \delta_4) \\
&= -k_1\delta_1^2 + \delta_1\delta_2 - k_3\delta_3^2 + \delta_3\delta_4
\end{aligned}
{% endmathjax %}<br>
> {% mathjax %}V_1{% endmathjax %}本身没有变化，仍然正定；失去负定性的只是{% mathjax %}\dot{V}_1{% endmathjax %}，多出的交叉项{% mathjax %}\delta_1\delta_2 + \delta_3\delta_4{% endmathjax %}留到这一步处理  
> 交叉项不能省略：若第一步的{% mathjax %}\dot{x}_{1t} - x_2 = -k_1\delta_1{% endmathjax %}精确成立，则{% mathjax %}\delta_2 = \dot{x}_{1t} + k_1\delta_1 - x_2 \equiv 0{% endmathjax %}，第二步就没有必要了

于是
{% mathjax %}
\begin{aligned}
\dot{V}_2 &= \dot{V}_1 + \begin{bmatrix} \delta_2 & \delta_4 \end{bmatrix}\begin{bmatrix} \dot{\delta}_2 \\ \dot{\delta}_4 \end{bmatrix} \\
&= \underbrace{-k_1\delta_1^2 - k_3\delta_3^2}_{ND} + \delta_2(\dot{x}_{2t} - \dot{x}_2 + \delta_1) + \delta_4(\dot{x}_{4t} - \dot{x}_4 + \delta_3)
\end{aligned}
{% endmathjax %}<br>
令
{% mathjax %}
\dot{x}_{2t} - \dot{x}_2 = -k_2\delta_2 - \delta_1, \quad \dot{x}_{4t} - \dot{x}_4 = -k_4\delta_4 - \delta_3, \quad k_2, k_4 > 0
{% endmathjax %}<br>
其中{% mathjax %}-\delta_1{% endmathjax %}、{% mathjax %}-\delta_3{% endmathjax %}用于抵消交叉项，则
{% mathjax %}
\dot{V}_2 = -k_1\delta_1^2 - k_2\delta_2^2 - k_3\delta_3^2 - k_4\delta_4^2
{% endmathjax %}<br>
对任意正的增益都负定，{% mathjax %}(\delta_1, \delta_2, \delta_3, \delta_4) = (0, 0, 0, 0){% endmathjax %}为系统的渐近稳定点
> 若省去{% mathjax %}-\delta_1{% endmathjax %}、{% mathjax %}-\delta_3{% endmathjax %}，则{% mathjax %}\dot{V}_2{% endmathjax %}中残留{% mathjax %}-k_1\delta_1^2 + \delta_1\delta_2 - k_2\delta_2^2{% endmathjax %}，只有在{% mathjax %}4k_1k_2 > 1{% endmathjax %}时才负定。例如{% mathjax %}k_1 = k_2 = 0.2{% endmathjax %}，在{% mathjax %}\delta_1 = \delta_2 = 1{% endmathjax %}处{% mathjax %}\dot{V}_2 = 0.6 > 0{% endmathjax %}

#### 控制律
虚拟控制量的导数为
{% mathjax %}
\begin{aligned}
\dot{x}_{2t} &= \ddot{x}_{1t} + k_1\dot{\delta}_1 = \ddot{x}_{1t} + k_1\dot{x}_{1t} - k_1\dot{x}_1 = \ddot{x}_{1t} + k_1\dot{x}_{1t} - k_1x_2 \\
\dot{x}_{4t} &= \ddot{x}_{3t} + k_3\dot{\delta}_3 = \ddot{x}_{3t} + k_3\dot{x}_{3t} - k_3\dot{x}_3 = \ddot{x}_{3t} + k_3\dot{x}_{3t} - k_3x_4
\end{aligned}
{% endmathjax %}<br>
代入{% mathjax %}\dot{x}_{2t} - \dot{x}_2 = -k_2\delta_2 - \delta_1{% endmathjax %}:
{% mathjax %}
\ddot{x}_{1t} + k_1\dot{x}_{1t} - k_1x_2 - \frac{1}{J_{Ay}}\left[u_1 - mgl_A\cos x_1 - \Gamma(x_1)x_4^2\right] = -k_2(x_{2t} - x_2) - (x_{1t} - x_1)
{% endmathjax %}<br>
整理得
{% mathjax %}
J_{Ay}\left[\ddot{x}_{1t} + k_1\dot{x}_{1t} - k_1x_2 + k_2(x_{2t} - x_2) + (x_{1t} - x_1)\right] = u_1 - mgl_A\cos x_1 - \Gamma(x_1)x_4^2
{% endmathjax %}<br>
同理，代入{% mathjax %}\dot{x}_{4t} - \dot{x}_4 = -k_4\delta_4 - \delta_3{% endmathjax %}:
{% mathjax %}
\ddot{x}_{3t} + k_3\dot{x}_{3t} - k_3x_4 - \frac{1}{J_z(x_1)}\left[u_2 + 2\Gamma(x_1)x_2x_4\right] = -k_4(x_{4t} - x_4) - (x_{3t} - x_3)
{% endmathjax %}<br>
整理得
{% mathjax %}
J_z(x_1)\left[\ddot{x}_{3t} + k_3\dot{x}_{3t} - k_3x_4 + k_4(x_{4t} - x_4) + (x_{3t} - x_3)\right] = u_2 + 2\Gamma(x_1)x_2x_4
{% endmathjax %}<br>
最终得到控制律:
{% mathjax %}
u_1 = J_{Ay}\left[\ddot{x}_{1t} + k_1\dot{x}_{1t} - k_1x_2 + k_2(x_{2t} - x_2) + (x_{1t} - x_1)\right] + mgl_A\cos x_1 + \left[(J_{Q\perp} - J_{Q\parallel})\sin x_1\cos x_1 + ml_A(l_y + l_A\cos x_1)\sin x_1\right]x_4^2
{% endmathjax %}<br>

{% mathjax %}
u_2 = \left[J_{Cz} + J_{Bz} + J_{Az}(x_1)\right]\left[\ddot{x}_{3t} + k_3\dot{x}_{3t} - k_3x_4 + k_4(x_{4t} - x_4) + (x_{3t} - x_3)\right] - 2\left[(J_{Q\perp} - J_{Q\parallel})\sin x_1\cos x_1 + ml_A(l_y + l_A\cos x_1)\sin x_1\right]x_2x_4
{% endmathjax %}<br>
> 方括号内是期望的角加速度，乘以转动惯量得到产生该角加速度所需的力矩；方括号外的重力项、离心力项和耦合项则是前馈，用来抵消模型中已知的非线性

## 可改进点
> 以下内容尚未并入正文，正文中与之冲突的说法以本节为准

### 控制律的等价形式与调参
记角度误差{% mathjax %}e = x_{1t} - x_1{% endmathjax %}，则{% mathjax %}\delta_1 = e{% endmathjax %}，{% mathjax %}\delta_2 = \dot{e} + k_1e{% endmathjax %}。正文的{% mathjax %}u_1{% endmathjax %}可以改写为
{% mathjax %}
u_1 = J_{Ay}\left[\ddot{x}_{1t} + (k_1 + k_2)\dot{e} + (k_1k_2 + 1)e\right] + mgl_A\cos x_1 + \Gamma(x_1)x_4^2
{% endmathjax %}<br>
闭环误差动态为
{% mathjax %}
\ddot{e} + (k_1 + k_2)\dot{e} + (k_1k_2 + 1)e = 0
{% endmathjax %}<br>
可见这个反步控制器等价于**计算力矩法**：前馈抵消非线性后，剩下的就是一个带加速度前馈的PD控制器，等效{% mathjax %}K_p = k_1k_2 + 1{% endmathjax %}，{% mathjax %}K_d = k_1 + k_2{% endmathjax %}。这给出了调参的物理意义：
* 特征方程的判别式为{% mathjax %}(k_1 - k_2)^2 - 4{% endmathjax %}，因此{% mathjax %}|k_1 - k_2| \geq 2{% endmathjax %}时为过阻尼或临界阻尼，否则为欠阻尼。特别地，{% mathjax %}k_1 = k_2 = k{% endmathjax %}时极点为{% mathjax %}-k \pm j{% endmathjax %}，总带有一点振荡
* 极点不能超过电机电流环带宽和控制频率允许的范围，否则离散化与电流环延迟会让实际系统振荡
> 多出的"{% mathjax %}+1{% endmathjax %}"来自{% mathjax %}V_2{% endmathjax %}中角度误差和角速度误差的权重都取了{% mathjax %}\frac{1}{2}{% endmathjax %}，二者单位不同。可以取{% mathjax %}V_2 = V_1 + \frac{\lambda}{2}\delta_2^2{% endmathjax %}({% mathjax %}\lambda > 0{% endmathjax %})，此时令{% mathjax %}\dot{\delta}_2 = -k_2\delta_2 - \delta_1/\lambda{% endmathjax %}即可消去交叉项，特征方程变为{% mathjax %}s^2 + (k_1 + k_2)s + k_1k_2 + 1/\lambda = 0{% endmathjax %}。{% mathjax %}\lambda{% endmathjax %}越大，极点越接近{% mathjax %}-k_1{% endmathjax %}、{% mathjax %}-k_2{% endmathjax %}

### 目标轨迹的导数
控制律需要{% mathjax %}\dot{x}_{1t}{% endmathjax %}、{% mathjax %}\ddot{x}_{1t}{% endmathjax %}、{% mathjax %}\dot{x}_{3t}{% endmathjax %}、{% mathjax %}\ddot{x}_{3t}{% endmathjax %}。目标角来自遥控器或自瞄，直接差分两次会把噪声放大到不可用:
* 自瞄: 目标观测器(如EKF)的状态中通常已经包含目标的速度和加速度，可以由预测模型直接解析得到目标角的一二阶导数
* 遥控器: 用跟踪微分器(TD)或轨迹规划对目标角做平滑，同时输出其一二阶导数
* 实在拿不到时取{% mathjax %}\ddot{x}_{t} = 0{% endmathjax %}，只保留速度前馈，代价是跟踪加速运动的目标时有滞后

### yaw角误差过零处理
yaw轴可以无限旋转，{% mathjax %}\psi{% endmathjax %}在{% mathjax %}\pm\pi{% endmathjax %}处跳变。{% mathjax %}\delta_3 = x_{3t} - x_3{% endmathjax %}必须归一化到{% mathjax %}(-\pi, \pi]{% endmathjax %}，否则目标从{% mathjax %}179°{% endmathjax %}变到{% mathjax %}-179°{% endmathjax %}时云台会反向转一整圈。pitch轴有机械限位，不存在这个问题

### 角速度的坐标系
模型中{% mathjax %}x_2 = \dot{\theta}{% endmathjax %}、{% mathjax %}x_4 = \dot{\psi}{% endmathjax %}是关节角速度，而陀螺仪测得的是其安装刚体坐标系下的角速度。若IMU装在pitch上，按{% mathjax %}R = R_z(\psi)R_y(\theta){% endmathjax %}的转动顺序有
{% mathjax %}
\omega^b_x = -\dot{\psi}\sin\theta, \quad \omega^b_y = \dot{\theta}, \quad \omega^b_z = \dot{\psi}\cos\theta
{% endmathjax %}<br>
因此应取{% mathjax %}x_2 = \omega^b_y{% endmathjax %}，{% mathjax %}x_4 = \omega^b_z\cos\theta - \omega^b_x\sin\theta{% endmathjax %}，而不是直接用{% mathjax %}\omega^b_z{% endmathjax %}(具体符号取决于各轴正方向的约定)。pitch角较大时，二者的差别不可忽略

### 模型误差与扰动
控制律完全依赖模型抵消非线性，而模型参数({% mathjax %}m{% endmathjax %}、{% mathjax %}l_A{% endmathjax %}、各转动惯量)只能近似得到，且没有考虑摩擦:
* 重力补偿有误差时，pitch会出现稳态误差，而纯PD结构无法消除。可以在{% mathjax %}\delta_1{% endmathjax %}上加积分项(积分反步法)，或用扩张状态观测器(ESO)估计总扰动后补偿
* 重力矩系数{% mathjax %}mgl_A{% endmathjax %}是主要的不确定参数，也可以用自适应反步法在线估计
* 小陀螺时底盘相对云台高速旋转，yaw轴的轴承和电机摩擦取决于**相对转速**{% mathjax %}\dot{\psi} - \omega_{chassis}{% endmathjax %}，会持续拖拽云台。可以在{% mathjax %}u_2{% endmathjax %}中加入摩擦前馈{% mathjax %}b(\dot{\psi} - \omega_{chassis}) + c\,\text{sgn}(\dot{\psi} - \omega_{chassis}){% endmathjax %}，参数通过实验辨识

### 执行器饱和
李雅普诺夫证明默认{% mathjax %}u_1{% endmathjax %}、{% mathjax %}u_2{% endmathjax %}能被完全执行。电机电流有上限，大误差时控制量饱和，抵消不再精确，稳定性证明也不再成立。实际使用时:
* 目标角经过轨迹规划后再送入控制器，避免大阶跃
* 饱和期间冻结积分项(若加了积分)
* 更系统的做法是指令滤波反步法(command filtered backstepping)，把饱和与虚拟控制量求导一并处理

### 改进后的实现
综合正文控制律与以上几点(yaw角过零处理、输出限幅)的控制器:
```c++
typedef struct {
    float J_Ay, J_z0, J_Qperp, J_Qpar, m, l_A, l_y, g;   // J_z0 = J_Cz + J_Bz
    float k1, k2, k3, k4;
    float tau_y_max, tau_z_max;
} GimbalParam;

static float wrap_pi(float a)
{
    while (a > PI)   a -= 2.0f * PI;
    while (a <= -PI) a += 2.0f * PI;
    return a;
}

// 耦合项 Gamma(theta)
static float coupling(const GimbalParam *p, float th)
{
    float s = sinf(th), c = cosf(th);
    return (p->J_Qperp - p->J_Qpar) * s * c + p->m * p->l_A * (p->l_y + p->l_A * c) * s;
}

// yaw轴转动惯量 J_z(theta) = J_Cz + J_Bz + J_Az(theta)
static float yaw_inertia(const GimbalParam *p, float th)
{
    float s = sinf(th), c = cosf(th), r = p->l_y + p->l_A * c;
    return p->J_z0 + p->J_Qperp * c * c + p->J_Qpar * s * s + p->m * r * r;
}

// th_t, dth_t, ddth_t: pitch目标角及其一二阶导数; psi_t, dpsi_t, ddpsi_t 同理
// th, w_y, psi, w_z: 反馈的关节角与关节角速度(见「角速度的坐标系」)
void gimbal_backstepping(const GimbalParam *p,
                         float th_t, float dth_t, float ddth_t,
                         float psi_t, float dpsi_t, float ddpsi_t,
                         float th, float w_y, float psi, float w_z,
                         float *tau_y, float *tau_z)
{
    float G = coupling(p, th);

    // pitch
    float d1 = th_t - th;                                   // delta_1
    float d2 = dth_t + p->k1 * d1 - w_y;                    // delta_2 = x2t - x2
    float a_y = ddth_t + p->k1 * (dth_t - w_y) + p->k2 * d2 + d1;
    *tau_y = p->J_Ay * a_y + p->m * p->g * p->l_A * cosf(th) + G * w_z * w_z;

    // yaw
    float d3 = wrap_pi(psi_t - psi);                        // delta_3, 过零处理
    float d4 = dpsi_t + p->k3 * d3 - w_z;                   // delta_4 = x4t - x4
    float a_z = ddpsi_t + p->k3 * (dpsi_t - w_z) + p->k4 * d4 + d3;
    *tau_z = yaw_inertia(p, th) * a_z - 2.0f * G * w_y * w_z;

    *tau_y = fminf(fmaxf(*tau_y, -p->tau_y_max), p->tau_y_max);
    *tau_z = fminf(fmaxf(*tau_z, -p->tau_z_max), p->tau_z_max);
}
```

---
AIGC: 
Claude Opus 5.5, 手稿转录、推导校核与可改进点
