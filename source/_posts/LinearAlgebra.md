---
title: 线代知识大杂烩
tag:
  - 线代
index_img: /img/Eyjafjalla.avif
mathjax: true
excerpt: '常用线代结论笔记，以供复习考试或应用推导用'
date: 2026-07-05 12:18:10
category: 笔记
---
# 基本结论
## 矩阵乘法
$$
\mathbf{A}(\mathbf{B}+\mathbf{C})=\mathbf{AB}+\mathbf{AC} \\
(\mathbf{B}+\mathbf{C})\mathbf{A}=\mathbf{BA}+\mathbf{CA} \\
(\mathbf{AB})\mathbf{C}=\mathbf{A}(\mathbf{BC}) \\
\lambda(\mathbf{AB})=(\lambda \mathbf{A})\mathbf{B}=\mathbf{A}(\lambda \mathbf{B}) \\
\mathbf{EAE}=\mathbf{A} \\
\mathbf{A}^{k+l}=\mathbf{A}^k \mathbf{A}^l \\
(\mathbf{A}^k)^l = \mathbf{A}^{kl}
$$
## 矩阵转置
$$
(\mathbf{A}^T)^T=\mathbf{A} \\
(k \mathbf{A})^T = k \mathbf{A}^T \\
(\mathbf{A}+\mathbf{B})^T = \mathbf{A}^T + \mathbf{B}^T \\
(\mathbf{AB})^T=\mathbf{B}^T\mathbf{A}^T \\
(\mathbf{A}_1\mathbf{A}_2...\mathbf{A}_s)^T=\mathbf{A}_s^T...\mathbf{A}_2^T\mathbf{A}_1^T
$$
### 对称矩阵
$$
对于同阶对称矩阵\mathbf{A}^T=\mathbf{A},\mathbf{B}^T=\mathbf{B}有 \\
\mathbf{A+\mathbf{B}},\mathbf{A}-\mathbf{B},k\mathbf{A}均为对称矩阵 \\
$$
$$
对于m\times n矩阵\mathbf{A},\mathbf{A}^T\mathbf{A}为n阶对称矩阵,\mathbf{AA}^T为m阶对称矩阵
$$
## 可逆矩阵
$$
\mathbf{AA}^{-1}=\mathbf{A}^{-1}\mathbf{A}=\mathbf{E} \\
(\mathbf{A}^{-1})^{-1}=\mathbf{A} \\
(k\mathbf{A})^{-1}=\frac{1}{k}\mathbf{A}^{-1} \\
(\mathbf{AB})^{-1} = \mathbf{B}^{-1}\mathbf{A}^{-1} \\
(\mathbf{A}_1\mathbf{A}_2...\mathbf{A}_s)^{-1}=\mathbf{A}_s^{-1}...\mathbf{A}_2^{-1}\mathbf{A}_1^{-1} \\
(\mathbf{A}^T)^{-1} = (\mathbf{A}^{-1})^T
$$
### 初等矩阵
$Define:$单位矩阵只经过一次初等变换而得到的矩阵  
$$
\mathbf{E} \xrightarrow{r_i \lrarr r_j} \mathbf{E}_{ij} \\
\mathbf{E} \xrightarrow{kr_i} \mathbf{E}_i(k) \\
\mathbf{E} \xrightarrow{r_j+kr_i} \mathbf{E}_{ij}(k)
$$
对矩阵$\mathbf{A}$作初等**行**变换等价于用对于的初等矩阵**左**乘矩阵$\mathbf{A}$  
对矩阵$\mathbf{A}$作初等**列**变换等价于用对于的初等矩阵**右**乘矩阵$\mathbf{A}$  

## 行列式
$$
当\mathbf{A}为方阵 \Rightarrow
\begin{cases}
  设\mathbf{A} \underset{(i \neq j)}{\xrightarrow{r_i \lrarr r_j}} \mathbf{B}, 则 \left | \mathbf{B} \right | = -\left | \mathbf{A} \right | \\
  设\mathbf{A} \xrightarrow{r_j + kr_i} \mathbf{B}, 则 \left | \mathbf{B} \right | = \left | \mathbf{A} \right | \\
  设\mathbf{A} \underset{(k \neq 0)}{\xrightarrow{kr_i}} \mathbf{B}, 则 \left | \mathbf{B} \right | = k\left | \mathbf{A} \right | \\
\end{cases}
$$
$$
\left | \mathbf{A} \right | = \left | \mathbf{A}^T \right | \\
若\mathbf{A},\mathbf{B}为方阵 \Rightarrow 
\begin{cases}
  \left | \mathbf{AB} \right | = \left | \mathbf{A} \right | \left | \mathbf{B} \right | \\
  \left | \mathbf{AB} \right | = \left | \mathbf{BA} \right |
\end{cases}
$$
### 行列式展开定理
$$
按第k列展开\sum_{k=1}^n a_{ik}\mathbf{A}_{jk} = 
\begin{cases}
  \left | \mathbf{A} \right | , j = i, \\
  0, j \neq i,
\end{cases} \\
按第k行展开\sum_{k=1}^n a_{ki}\mathbf{A}_{kj} = 
\begin{cases}
  \left | \mathbf{A} \right | , j = i, \\
  0, j \neq i,
\end{cases} \\
其中i,j = 1,2,...,m
$$
### 伴随矩阵

$$
\mathbf{AA}^*=\mathbf{A}^*\mathbf{A}=\left | \mathbf{A} \right |\mathbf{E} \\
\mathbf{A}^{-1} = \frac{1}{\left | \mathbf{A} \right |} \mathbf{A}^*
$$
## 分块矩阵

将矩阵 $\mathbf{A}$ 用纵线和横线分成若干小块：

$$
\mathbf{A}=
\begin{pmatrix}
\mathbf{A}_{11} & \mathbf{A}_{12} & \cdots & \mathbf{A}_{1t} \\
\mathbf{A}_{21} & \mathbf{A}_{22} & \cdots & \mathbf{A}_{2t} \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{A}_{s1} & \mathbf{A}_{s2} & \cdots & \mathbf{A}_{st}
\end{pmatrix}
$$
### 分块加法

两个**行数相同、列数相同**且**分块方式相同**的矩阵相加时，对应子块相加：

$$
\mathbf{A}+\mathbf{B}=
\begin{pmatrix}
\mathbf{A}_{11}+\mathbf{B}_{11} & \cdots & \mathbf{A}_{1t}+\mathbf{B}_{1t} \\
\vdots & \ddots & \vdots \\
\mathbf{A}_{s1}+\mathbf{B}_{s1} & \cdots & \mathbf{A}_{st}+\mathbf{B}_{st}
\end{pmatrix}
$$
### 分块数乘

$$
k\mathbf{A}=
\begin{pmatrix}
k\mathbf{A}_{11} & \cdots & k\mathbf{A}_{1t} \\
\vdots & \ddots & \vdots \\
k\mathbf{A}_{s1} & \cdots & k\mathbf{A}_{st}
\end{pmatrix}
$$
### 分块乘法

将 $\mathbf{A}_{m\times n}$ 和 $\mathbf{B}_{n\times l}$ 分块时，要求 **$\mathbf{A}$ 的列的分法与 $\mathbf{B}$ 的行的分法一致**：

$$
\mathbf{C}_{ij}=\sum_{k=1}^{t} \mathbf{A}_{ik}\mathbf{B}_{kj}
$$
### 分块转置

$$
\mathbf{A}^T=
\begin{pmatrix}
\mathbf{A}_{11}^T & \mathbf{A}_{21}^T & \cdots & \mathbf{A}_{s1}^T \\
\mathbf{A}_{12}^T & \mathbf{A}_{22}^T & \cdots & \mathbf{A}_{s2}^T \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{A}_{1t}^T & \mathbf{A}_{2t}^T & \cdots & \mathbf{A}_{st}^T
\end{pmatrix}
$$
即：子块先各自转置，再把子块本身当作元素进行转置。

### 分块对角矩阵

$$
\mathbf{A}=
\begin{pmatrix}
\mathbf{A}_1 & \mathbf{O} & \cdots & \mathbf{O} \\
\mathbf{O} & \mathbf{A}_2 & \cdots & \mathbf{O} \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{O} & \mathbf{O} & \cdots & \mathbf{A}_s
\end{pmatrix}
$$
称为**分块对角矩阵**（或准对角矩阵），记作 $\mathbf{A}=\text{diag}(\mathbf{A}_1,\mathbf{A}_2,\dots,\mathbf{A}_s)$。

**性质：**
- 行列式：$\left | \mathbf{A} \right | = \left | \mathbf{A}_1 \right |\cdot \left | \mathbf{A}_2 \right |\cdots \left | \mathbf{A}_s \right |$
- 幂运算：$\mathbf{A}^k=\text{diag}(\mathbf{A}_1^k,\mathbf{A}_2^k,\dots,\mathbf{A}_s^k)$
- 可逆性：$\mathbf{A}$ 可逆 $\Leftrightarrow$ 每个 $\mathbf{A}_i$ 都可逆，且 $\mathbf{A}^{-1}=\text{diag}(\mathbf{A}_1^{-1},\mathbf{A}_2^{-1},\dots,\mathbf{A}_s^{-1})$

### 分块三角矩阵

上三角分块矩阵（对角块均为方阵）：

$$
\mathbf{A}=
\begin{pmatrix}
\mathbf{A}_{11} & \mathbf{A}_{12} & \cdots & \mathbf{A}_{1s} \\
\mathbf{O} & \mathbf{A}_{22} & \cdots & \mathbf{A}_{2s} \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{O} & \mathbf{O} & \cdots & \mathbf{A}_{ss}
\end{pmatrix}
$$
**性质：**
- 行列式：$\left | \mathbf{A} \right | = \left | \mathbf{A}_{11} \right |\cdot \left | \mathbf{A}_{22} \right |\cdots \left | \mathbf{A}_{ss} \right |$
- 对于分块下三角矩阵同理

### 常用分块矩阵公式

**拉普拉斯展开式：**

$$
\begin{vmatrix}
\mathbf{A} & \mathbf{O} \\
\mathbf{C} & \mathbf{B}
\end{vmatrix}=
\begin{vmatrix}
\mathbf{A} & \mathbf{C} \\
\mathbf{O} & \mathbf{B}
\end{vmatrix}=
\left | \mathbf{A} \right | \cdot \left | \mathbf{B} \right |
$$
$$
\begin{vmatrix}
\mathbf{O} & \mathbf{A} \\
\mathbf{B} & \mathbf{C}
\end{vmatrix}=
\begin{vmatrix}
\mathbf{C} & \mathbf{A} \\
\mathbf{B} & \mathbf{O}
\end{vmatrix}=
(-1)^{mn} \left | \mathbf{A} \right | \cdot \left | \mathbf{B} \right |
\quad (\mathbf{A}为m阶,\mathbf{B}为n阶)
$$
**分块矩阵求逆（$\mathbf{A}$ 可逆）：**

$$
\begin{pmatrix}
\mathbf{A} & \mathbf{O} \\
\mathbf{O} & \mathbf{B}
\end{pmatrix}^{-1}=
\begin{pmatrix}
\mathbf{A}^{-1} & \mathbf{O} \\
\mathbf{O} & \mathbf{B}^{-1}
\end{pmatrix}
$$
$$
\begin{pmatrix}
\mathbf{A} & \mathbf{C} \\
\mathbf{O} & \mathbf{B}
\end{pmatrix}^{-1}=
\begin{pmatrix}
\mathbf{A}^{-1} & -\mathbf{A}^{-1}\mathbf{C}\mathbf{B}^{-1} \\
\mathbf{O} & \mathbf{B}^{-1}
\end{pmatrix}
$$
$$
\begin{pmatrix}
\mathbf{A} & \mathbf{O} \\
\mathbf{C} & \mathbf{B}
\end{pmatrix}^{-1}=
\begin{pmatrix}
\mathbf{A}^{-1} & \mathbf{O} \\
-\mathbf{B}^{-1}\mathbf{C}\mathbf{A}^{-1} & \mathbf{B}^{-1}
\end{pmatrix}
$$
## 线性相关性
**唯一表示定理**：设向量组$\mathbf{\alpha}_1,\mathbf{\alpha}_2,...,\mathbf{\alpha}_n$线性无关，又设该向量组添加一个向量$\beta$后的向量组$\mathbf{\alpha}_1,\mathbf{\alpha}_2,...,\mathbf{\alpha}_n,\beta$线性相关，则向量$\beta$可由向量组$\mathbf{\alpha}_1,\mathbf{\alpha}_2,...,\mathbf{\alpha}_n$线性表示且表示式唯一

**线性无关的证明**： 对于向量组$\mathbf{\alpha}_1,\mathbf{\alpha}_2,...,\mathbf{\alpha}_n$，设实数$k_1,k_2,...,k_n$满足$k_1\mathbf{\alpha}_1+k_2\mathbf{\alpha}_2+...+k_n\mathbf{\alpha}_n=0$  
若$k_i,i=1,2,...,n$不全为0，则线性相关；若全为0，则线性无关
### 秩

$$
r(V) = n \Leftrightarrow 向量组V内的向量线性无关 \\
r(V) < n \Leftrightarrow 向量组V内的向量线性无关
$$
**任意矩阵的行秩与列秩相等，其值等于它对应的最简阶梯型矩阵或阶梯型矩阵的非零行的行数**  
**初等变换不改变矩阵的秩**
**规定零矩阵的秩为0**

设$\mathbf{P},\mathbf{Q}$都是可逆矩阵，则
$$
r(\mathbf{PAQ}) = r(A)
$$
矩阵的秩有如下性质
$$
r(\mathbf{A}^T)=r(\mathbf{A}) \\
r(\mathbf{A}_{m\times n}) \leq \text{min}(m,n) \\
r(\mathbf{AB}) \leq \text{min} ( r(\mathbf{A}) , r(\mathbf{B}) ) \\
r(\mathbf{A}) = r \Leftrightarrow \mathbf{A}中有一个r阶子式D_r \neq 0, 且所有r+1阶子式（如果存在的话）都等于0
$$
$$
对于任意n阶(n \leq 2)矩阵\mathbf{A},有r(\mathbf{A}^*) = 
\begin{cases}
  n \quad r(\mathbf{A}) = n \\
  1 \quad r(\mathbf{A}) = n-1 \\
  0 \quad r(\mathbf{A}) < n-1
\end{cases}
$$
### 向量空间

**维数**$\text{dim}$：向量空间$V$的基所含向量的个数  

**变基**：
设$r$维向量空间$V=\text{span}(\alpha_1,...,\alpha_r)=\text{span}(\beta_1,...,\beta_r)$，则$\beta_1,...,\beta_r$可由$\alpha_1,...,\alpha_r$线性表示：

$$
\begin{bmatrix}
  \beta_1,...,\beta_r
\end{bmatrix}
=
\begin{bmatrix}
  \alpha_1,...,\alpha_r
\end{bmatrix}
\begin{bmatrix}
  p_{11} & p_{12} & ... & p_{1r} \\
  p_{21} & p_{22} & ... & p_{2r} \\
  \vdots & \vdots & & \vdots \\
  p_{r1} & p_{r2} & ... & p_{rr} \\
\end{bmatrix}
$$
称矩阵$\mathbf{P}=\begin{bmatrix} p_{ij} \end{bmatrix}$为由基$\begin{bmatrix} \alpha_1,...,\alpha_r \end{bmatrix}$到基$\begin{bmatrix} \beta_1,...,\beta_r \end{bmatrix}$的**过渡矩阵**  

过渡矩阵一定为可逆矩阵  
要将基$\begin{bmatrix} \alpha_1,...,\alpha_r \end{bmatrix}$下的坐标$\mathbf{x}$转换到基$\begin{bmatrix} \beta_1,...,\beta_r \end{bmatrix}$下的坐标$\mathbf{y}$,需要左乘过渡矩阵的逆矩阵  

$$
\mathbf{y} = \mathbf{P}^{-1} \mathbf{x}
$$
## 特征值

**求解**

由 $\det(\mathbf{A}-\lambda\mathbf{E})=0$ 求特征值 $\lambda_i$，再对每个 $\lambda_i$ 解齐次方程组 $(\mathbf{A}-\lambda_i\mathbf{E})\mathbf{x}=\mathbf{O}$ 求特征向量。

$$
\det(\mathbf{A}-\lambda\mathbf{E}) = 
\begin{vmatrix}
a_{11}-\lambda & a_{12} & \cdots & a_{1n} \\
a_{21} & a_{22}-\lambda & \cdots & a_{2n} \\
\vdots & \vdots & \ddots & \vdots \\
a_{n1} & a_{n2} & \cdots & a_{nn}-\lambda
\end{vmatrix}
$$
展开后为 $\lambda$ 的 $n$ 次多项式，称**特征多项式**。特征方程 $\det(\mathbf{A}-\lambda\mathbf{E})=0$ 的根即特征值。

**性质**

$$
\mathbf{A}\alpha = \lambda \alpha \Rightarrow \mathbf{A}^k\alpha = \lambda^k\alpha \\
\mathbf{A}\alpha = \lambda \alpha \Rightarrow \mathbf{A}^{-1}\alpha = \lambda^{-1}\alpha \quad (\mathbf{A}可逆) \\
设\varphi (z)=a_0+a_1z+...+a_m z^m,则\mathbf{A}\alpha = \lambda \alpha \Rightarrow \varphi(\mathbf{A})\alpha = \varphi(\lambda)\alpha \\
\mathbf{A}和\mathbf{A}^T有相同的特征值 \\
不同特征值对应的特征向量线性无关
$$
若 $n$ 阶矩阵 $\mathbf{A}$ 的特征值为 $\lambda_1,\lambda_2,...,\lambda_n$，则：

$$
\sum_{i=1}^n \lambda_i = \operatorname{tr}(\mathbf{A}) \\
\prod_{i=1}^n \lambda_i = \left | \mathbf{A} \right |
$$
**代数重数**：特征值 $\lambda_i$ 作为特征多项式根的重数，记作 $m_i$。

**几何重数**：特征值 $\lambda_i$ 对应的特征子空间的维数 $= n - r(\mathbf{A}-\lambda_i\mathbf{E})$，记作 $d_i$。

恒有 $1 \leq d_i \leq m_i \leq n$。

### 方阵的对角化

$\mathbf{A}$ 可对角化 $\Leftrightarrow$ 存在可逆矩阵 $\mathbf{P}$ 使 $\mathbf{P}^{-1}\mathbf{AP}=\mathbf{\Lambda}=\operatorname{diag}(\lambda_1,...,\lambda_n)$。

**充要条件：**
- $\mathbf{A}$ 有 $n$ 个线性无关的特征向量
- 每个特征值的几何重数 = 代数重数，即 $d_i = m_i$
- $n$ 阶矩阵有 $n$ 个不同特征值 $\Rightarrow$ 可对角化（充分非必要）

**求解步骤：**
1. 求特征值 $\lambda_1,...,\lambda_n$
2. 对每个 $\lambda_i$，解 $(\mathbf{A}-\lambda_i\mathbf{E})\mathbf{x}=\mathbf{O}$ 得线性无关的特征向量
3. 以特征向量为列构成 $\mathbf{P}$，对应的特征值排成 $\mathbf{\Lambda}$
4. 验证 $\mathbf{P}^{-1}\mathbf{AP}=\mathbf{\Lambda}$

## Euclid空间

### 向量内积

$Define:$ 在 $n$ 维实向量空间 $\mathbb{R}^n$ 中定义**内积**：

$$
\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle = \alpha_1\beta_1 + \alpha_2\beta_2 + \cdots + \alpha_n\beta_n = \mathbf{\alpha}^T\mathbf{\beta}
$$
定义了内积的 $\mathbb{R}^n$ 称为 **Euclid空间**。

**内积性质（$\mathbf{\alpha},\mathbf{\beta},\mathbf{\gamma} \in \mathbb{R}^n,\ k \in \mathbb{R}$）：**

$$
\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle = \left\langle \mathbf{\beta}, \mathbf{\alpha} \right\rangle \\
\left\langle k\mathbf{\alpha}, \mathbf{\beta} \right\rangle = k\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle \\
\left\langle \mathbf{\alpha} + \mathbf{\beta}, \mathbf{\gamma} \right\rangle = \left\langle \mathbf{\alpha}, \mathbf{\gamma} \right\rangle + \left\langle \mathbf{\beta}, \mathbf{\gamma} \right\rangle \\
\left\langle \mathbf{\alpha}, \mathbf{\alpha} \right\rangle \geq 0,\ 且\ \left\langle \mathbf{\alpha}, \mathbf{\alpha} \right\rangle = 0 \Leftrightarrow \mathbf{\alpha} = \mathbf{O}
$$
**范数（长度）：**
$$
\|\mathbf{\alpha}\| = \sqrt{\left\langle \mathbf{\alpha}, \mathbf{\alpha} \right\rangle} = \sqrt{\alpha_1^2 + \alpha_2^2 + \cdots + \alpha_n^2}
$$
**夹角：**
$$
\cos\langle \mathbf{\alpha}, \mathbf{\beta} \rangle = \frac{\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle}{\|\mathbf{\alpha}\| \cdot \|\mathbf{\beta}\|}
$$
**正交：** $\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle = 0 \Leftrightarrow \mathbf{\alpha} \perp \mathbf{\beta}$

### 标准正交基

若向量组 $\mathbf{\eta}_1,\mathbf{\eta}_2,\dots,\mathbf{\eta}_r$ 满足：
- 两两正交：$\left\langle \mathbf{\eta}_i, \mathbf{\eta}_j \right\rangle = 0 \ (i \neq j)$
- 均为单位向量：$\|\mathbf{\eta}_i\| = 1$

则称其为**标准正交向量组**。构成向量空间基的标准正交向量组称为**标准正交基**。

标准正交基下坐标计算简便：若 $\mathbf{\eta}_1,\dots,\mathbf{\eta}_n$ 为标准正交基，则 $\mathbf{\alpha} = \sum_{i=1}^n \left\langle \mathbf{\alpha}, \mathbf{\eta}_i \right\rangle \mathbf{\eta}_i$。

### 施密特正交化

将线性无关组 $\mathbf{\alpha}_1,\mathbf{\alpha}_2,\dots,\mathbf{\alpha}_r$ 化为标准正交组 $\mathbf{\eta}_1,\mathbf{\eta}_2,\dots,\mathbf{\eta}_r$。

**求解步骤：**
1. 正交化（得 $\mathbf{\beta}_1,\dots,\mathbf{\beta}_r$）：

$$
\mathbf{\beta}_1 = \mathbf{\alpha}_1 \\
\mathbf{\beta}_k = \mathbf{\alpha}_k - \sum_{i=1}^{k-1} \frac{\left\langle \mathbf{\alpha}_k, \mathbf{\beta}_i \right\rangle}{\left\langle \mathbf{\beta}_i, \mathbf{\beta}_i \right\rangle} \mathbf{\beta}_i \quad (k=2,3,\dots,r)
$$
2. 单位化（得 $\mathbf{\eta}_1,\dots,\mathbf{\eta}_r$）：

$$
\mathbf{\eta}_k = \frac{\mathbf{\beta}_k}{\|\mathbf{\beta}_k\|} \quad (k=1,2,\dots,r)
$$
### 正交矩阵

$Define:$ 若 $n$ 阶实矩阵 $\mathbf{Q}$ 满足 $\mathbf{Q}^T\mathbf{Q} = \mathbf{E}$（即 $\mathbf{Q}^{-1} = \mathbf{Q}^T$），则称 $\mathbf{Q}$ 为**正交矩阵**。

**性质：**

$$
\mathbf{Q}为正交矩阵 \Leftrightarrow \mathbf{Q}的列(行)向量组构成标准正交基 \\
\mathbf{Q}为正交矩阵 \Leftrightarrow \mathbf{Q}^T = \mathbf{Q}^{-1} \\
\left | \mathbf{Q} \right | = \pm 1 \\
若\mathbf{Q}_1,\mathbf{Q}_2为正交矩阵，则\mathbf{Q}_1\mathbf{Q}_2也是正交矩阵
$$
## 实对称矩阵

### 实对称矩阵的特征值和特征向量

$Define:$ $\mathbf{A}$ 为实对称矩阵 $\Leftrightarrow$ $\mathbf{A}^T = \mathbf{A}$ 且 $\mathbf{A}$ 的元素均为实数。

**性质：**
- 实对称矩阵的特征值均为实数
- 实对称矩阵的属于不同特征值的特征向量相互正交
- 实对称矩阵的每个特征值的几何重数 = 代数重数

### 对角化

$\mathbf{A}$ 实对称 $\Rightarrow$ 存在正交矩阵 $\mathbf{Q}$，使：

$$
\mathbf{Q}^{-1}\mathbf{AQ} = \mathbf{Q}^T\mathbf{AQ} = \operatorname{diag}(\lambda_1,\lambda_2,\dots,\lambda_n)
$$
**求解步骤：**
1. 求 $|\mathbf{A}-\lambda\mathbf{E}|=0$ 的根，得特征值 $\lambda_1,\dots,\lambda_n$（均为实数）
2. 对每个 $\lambda_i$，解 $(\mathbf{A}-\lambda_i\mathbf{E})\mathbf{x}=\mathbf{O}$ 得基础解系（特征向量）
3. 若 $\lambda_i$ 为重根，对其特征向量作 Schmidt 正交化
4. 将所有特征向量单位化，以它们为列构成正交矩阵 $\mathbf{Q}$
5. 得 $\mathbf{Q}^{-1}\mathbf{AQ} = \mathbf{Q}^T\mathbf{AQ} = \operatorname{diag}(\lambda_1,\dots,\lambda_n)$

## 二次齐次多项式

### 二次型

$Define:$ $n$ 元**二次型**：

$$
f(x_1,x_2,\dots,x_n) = \sum_{i=1}^n \sum_{j=1}^n a_{ij} x_i x_j = \mathbf{x}^T\mathbf{Ax}
$$
其中 $\mathbf{x} = (x_1,x_2,\dots,x_n)^T$，$\mathbf{A}$ 为 $n$ 阶**实对称矩阵**（$a_{ij}=a_{ji}$），称为二次型的**矩阵**。

构造 $\mathbf{A}$ 的方法：对角元 $a_{ii}$ 为 $x_i^2$ 的系数；非对角元 $a_{ij}=a_{ji}$ 为 $x_i x_j$ 系数的一半。

### 矩阵的合同

$Define:$ 对于 $n$ 阶方阵 $\mathbf{A},\mathbf{B}$，若存在可逆矩阵 $\mathbf{C}$ 使 $\mathbf{B} = \mathbf{C}^T\mathbf{AC}$，则称 $\mathbf{A}$ 与 $\mathbf{B}$ **合同**。

**性质：**
- 合同关系具有自反性、对称性、传递性
- 合同变换保持对称性：若 $\mathbf{A}$ 对称，则 $\mathbf{B}$ 也对称
- 合同变换不改变秩：$r(\mathbf{B}) = r(\mathbf{A})$

### 用正交变换化二次型为标准形

存在正交变换 $\mathbf{x} = \mathbf{Qy}$（$\mathbf{Q}$ 正交），使：

$$
f = \lambda_1 y_1^2 + \lambda_2 y_2^2 + \cdots + \lambda_n y_n^2
$$
其中 $\lambda_i$ 为 $\mathbf{A}$ 的特征值。此即**主轴定理**：任何实二次型均可通过正交变换化为标准形，系数恰为矩阵的特征值。

$$
f = \mathbf{x}^T\mathbf{Ax} \xrightarrow{\mathbf{x} = \mathbf{Qy}} f = \mathbf{y}^T(\mathbf{Q}^T\mathbf{AQ})\mathbf{y} = \sum_{i=1}^n \lambda_i y_i^2
$$
**求解步骤：**
1. 写出二次型的对称矩阵 $\mathbf{A}$（对角元为平方项系数，非对角元为交叉项系数的一半）
2. 对 $\mathbf{A}$ 作正交对角化，得特征值 $\lambda_i$ 和正交矩阵 $\mathbf{Q}$
3. 令 $\mathbf{x} = \mathbf{Qy}$，得标准形 $f = \sum \lambda_i y_i^2$

## 正定二次型与正定矩阵

### 二次型的规范形

由**惯性定理**：二次型经任何可逆线性变换化为标准形后，正平方项个数 $p$ 和负平方项个数 $q$ 不变（与变换方式无关）。

$p$——**正惯性指数**，$q$——**负惯性指数**，$p-q$——**符号差**，$p+q = r(\mathbf{A})$。

令 $f = y_1^2 + \cdots + y_p^2 - y_{p+1}^2 - \cdots - y_{p+q}^2$（系数均为 $\pm 1$），此即二次型的**规范形**。

由**霍尔维茨定理**（Sylvester 判别法）：$\mathbf{A}$ 正定 $\Leftrightarrow$ $\mathbf{A}$ 的各阶顺序主子式均 $> 0$：

$$
\Delta_k = 
\begin{vmatrix}
a_{11} & \cdots & a_{1k} \\
\vdots & \ddots & \vdots \\
a_{k1} & \cdots & a_{kk}
\end{vmatrix}
> 0 \quad (k=1,2,\dots,n)
$$
**负定：** 奇数阶顺序主子式 $< 0$，偶数阶 $> 0$

### 正定二次形

$Define:$ 若对任意 $\mathbf{x} \neq \mathbf{O}$，恒有 $f(\mathbf{x}) = \mathbf{x}^T\mathbf{Ax} > 0$，则称 $f$ 为**正定二次型**，$\mathbf{A}$ 为**正定矩阵**。

**判别条件（以下等价）：**
- $\mathbf{A}$ 正定
- $\mathbf{A}$ 的特征值全为正
- $f$ 的正惯性指数 $p = n$
- $\mathbf{A}$ 的各阶顺序主子式 $> 0$
- 存在可逆矩阵 $\mathbf{C}$ 使 $\mathbf{A} = \mathbf{C}^T\mathbf{C}$

**半正定：** $f \geq 0$（$\forall \mathbf{x} \neq \mathbf{O}$）$\Leftrightarrow$ 特征值均 $\geq 0$ $\Leftrightarrow$ 正惯性指数 $p = r(\mathbf{A}) < n$

# 方程组的解问题
$$
\mathbf{Ax}=\mathbf{O}仅有零解 \\
\Leftrightarrow \mathbf{Ax}=\mathbf{b} 有唯一解 \\
\Leftrightarrow \left | \mathbf{A} \right |  \neq 0 \\
\Leftrightarrow \mathbf{A}可逆 \\
\Leftrightarrow \mathbf{A}可以分解为有限个初等矩阵的乘积 \\
\Leftrightarrow \mathbf{A}的列向量组线性无关 \\
\Leftrightarrow \mathbf{A}的行向量组线性无关 \\
\Leftrightarrow \mathbf{b}可由\mathbf{A}的列向量组线性表示且表示式唯一 \\
\Leftrightarrow r(\mathbf{A}) = n
$$
## 线性方程组解的结构

### 基础解系

$Define:$ 齐次线性方程组 $\mathbf{A}_{m\times n}\mathbf{x}=\mathbf{O}$ 的解空间的**基**称为该方程组的一个**基础解系**。

设 $r(\mathbf{A})=r<n$，则将 $\mathbf{A}$ 通过初等行变换化为最简阶梯型，取 $n-r$ 个自由未知量（通常取不在主元列的变量），分别令其中一个为 $1$、其余为 $0$，回代解得主元变量的值，得到的 $n-r$ 个线性无关的解向量 $\mathbf{\xi}_1,\mathbf{\xi}_2,\dots,\mathbf{\xi}_{n-r}$ 即构成基础解系。

**解空间维数：** $\dim\{\mathbf{x} \mid \mathbf{Ax}=\mathbf{O}\} = n - r(\mathbf{A})$

### 齐次线性方程组 $\mathbf{A}_{m\times n}\mathbf{x}=\mathbf{O}$

$$
\mathbf{Ax}=\mathbf{O}\ 必有零解\ \mathbf{x}=\mathbf{O} \\
\mathbf{Ax}=\mathbf{O}\ 有非零解 \Leftrightarrow r(\mathbf{A}) < n \\
\mathbf{Ax}=\mathbf{O}\ 仅有零解 \Leftrightarrow r(\mathbf{A}) = n
$$
**解的结构（$r(\mathbf{A}) < n$ 时）：**

$$
\mathbf{x} = c_1\mathbf{\xi}_1 + c_2\mathbf{\xi}_2 + \cdots + c_{n-r}\mathbf{\xi}_{n-r}, \quad c_i \in \mathbb{R}
$$
其中 $\mathbf{\xi}_1,\dots,\mathbf{\xi}_{n-r}$ 为基础解系。

**求解步骤：**
1. 对系数矩阵 $\mathbf{A}$ 作初等行变换，化为阶梯型，求出 $r(\mathbf{A})$
2. 若 $r=n$，仅有零解；若 $r<n$，继续
3. 进一步化为最简阶梯型，确定主元变量与自由变量
4. 令自由变量依次取 $(1,0,\dots,0)^T,(0,1,\dots,0)^T,\dots,(0,0,\dots,1)^T$，回代求出基础解系
5. 通解为基础解系的任意线性组合

### 非齐次线性方程组 $\mathbf{A}_{m\times n}\mathbf{x}=\mathbf{b}\ (\mathbf{b} \neq \mathbf{O})$

**有解判定：**

$$
\mathbf{Ax}=\mathbf{b}\ 有解 \Leftrightarrow r(\mathbf{A}) = r(\mathbf{A} \mid \mathbf{b}) \\
\begin{cases}
  r(\mathbf{A}) = r(\mathbf{A} \mid \mathbf{b}) = n \Rightarrow 有唯一解 \\
  r(\mathbf{A}) = r(\mathbf{A} \mid \mathbf{b}) < n \Rightarrow 有无穷多解 \\
  r(\mathbf{A}) \neq r(\mathbf{A} \mid \mathbf{b}) \Rightarrow 无解
\end{cases}
$$
**解的结构（有解时）：**

$$
\mathbf{x} = \mathbf{\eta}^* + c_1\mathbf{\xi}_1 + c_2\mathbf{\xi}_2 + \cdots + c_{n-r}\mathbf{\xi}_{n-r}
$$
即：**导出组 $\mathbf{Ax}=\mathbf{O}$ 的通解 + 原方程组的一个特解 $\mathbf{\eta}^*$**。

**求解步骤：**
1. 对增广矩阵 $(\mathbf{A} \mid \mathbf{b})$ 作初等行变换，化为阶梯型
2. 比较 $r(\mathbf{A})$ 与 $r(\mathbf{A} \mid \mathbf{b})$：不等则无解
3. 继续化为最简阶梯型，确定主元变量与自由变量
4. 令所有自由变量 $=0$，回代求出一个特解 $\mathbf{\eta}^*$
5. 令 $\mathbf{b}=\mathbf{O}$，求导出组的基础解系 $\mathbf{\xi}_1,\dots,\mathbf{\xi}_{n-r}$
6. 写出通解 $\mathbf{x} = \mathbf{\eta}^* + \sum c_i\mathbf{\xi}_i$

# 方阵相似
$\text{Define} \colon$
若对于$n$阶方阵$\mathbf{A},\mathbf{B},\exist \mathbf{P}$为可逆矩阵，使得
$$
\mathbf{P}^{-1}\mathbf{AP} = \mathbf{B}
$$
则称$\mathbf{A}$和$\mathbf{B}$相似，记作$\mathbf{A} \sim \mathbf{B}$  

**性质**  
* 自反性 $\mathbf{A} \sim \mathbf{A}$
* 对称性 $\mathbf{A} \sim \mathbf{B} \Leftrightarrow \mathbf{B} \sim \mathbf{A}$
* 传递性 $\mathbf{A} \sim \mathbf{B},\mathbf{B} \sim \mathbf{C} \Rightarrow \mathbf{A} \sim \mathbf{C}$
$$
\mathbf{A} \sim \mathbf{B} \Rightarrow 
\begin{cases}
  又\varphi(x)=a_0+a_1x+...+a_mx^m,则\varphi(\mathbf{A}) \sim \varphi(\mathbf{B}) \\
  又\mathbf{A}可逆，则\mathbf{B}也可逆且\mathbf{A}^{-1} \sim \mathbf{B}^{-1} \\
  \left | \lambda \mathbf{E} - \mathbf{A} \right | = \left | \lambda \mathbf{E} - \mathbf{B} \right | \\
  \mathbf{A}和\mathbf{B}的特征值相同 \\
  \left | \mathbf{A} \right | = \left | \mathbf{B} \right | \\
  tr(\mathbf{A}) = tr(\mathbf{B}) \\
  r(\mathbf{A}) = r(\mathbf{B})
\end{cases}
$$
以上均为必要条件，但非充分。例如特征值相同不能保证相似，除非两个矩阵均可对角化且特征值（含重数）完全相同。
