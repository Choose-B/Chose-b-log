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
{% mathjax %}
\mathbf{A}(\mathbf{B}+\mathbf{C})=\mathbf{AB}+\mathbf{AC}
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{B}+\mathbf{C})\mathbf{A}=\mathbf{BA}+\mathbf{CA}
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{AB})\mathbf{C}=\mathbf{A}(\mathbf{BC})
{% endmathjax %}<br>
{% mathjax %}
\lambda(\mathbf{AB})=(\lambda \mathbf{A})\mathbf{B}=\mathbf{A}(\lambda \mathbf{B})
{% endmathjax %}<br>
{% mathjax %}
\mathbf{EAE}=\mathbf{A}
{% endmathjax %}<br>
{% mathjax %}
\mathbf{A}^{k+l}=\mathbf{A}^k \mathbf{A}^l
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{A}^k)^l = \mathbf{A}^{kl}
{% endmathjax %}<br>
## 矩阵转置
{% mathjax %}
(\mathbf{A}^T)^T=\mathbf{A}
{% endmathjax %}<br>
{% mathjax %}
(k \mathbf{A})^T = k \mathbf{A}^T
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{A}+\mathbf{B})^T = \mathbf{A}^T + \mathbf{B}^T
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{AB})^T=\mathbf{B}^T\mathbf{A}^T
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{A}_1\mathbf{A}_2...\mathbf{A}_s)^T=\mathbf{A}_s^T...\mathbf{A}_2^T\mathbf{A}_1^T
{% endmathjax %}<br>
### 对称矩阵
{% mathjax %}
对于同阶对称矩阵\mathbf{A}^T=\mathbf{A},\mathbf{B}^T=\mathbf{B}有
{% endmathjax %}<br>
{% mathjax %}
\mathbf{A+\mathbf{B}},\mathbf{A}-\mathbf{B},k\mathbf{A}均为对称矩阵 \\
{% endmathjax %}<br>
{% mathjax %}
对于m\times n矩阵\mathbf{A},\mathbf{A}^T\mathbf{A}为n阶对称矩阵,\mathbf{AA}^T为m阶对称矩阵
{% endmathjax %}<br>
## 可逆矩阵
{% mathjax %}
\mathbf{AA}^{-1}=\mathbf{A}^{-1}\mathbf{A}=\mathbf{E}
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{A}^{-1})^{-1}=\mathbf{A}
{% endmathjax %}<br>
{% mathjax %}
(k\mathbf{A})^{-1}=\frac{1}{k}\mathbf{A}^{-1}
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{AB})^{-1} = \mathbf{B}^{-1}\mathbf{A}^{-1}
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{A}_1\mathbf{A}_2...\mathbf{A}_s)^{-1}=\mathbf{A}_s^{-1}...\mathbf{A}_2^{-1}\mathbf{A}_1^{-1}
{% endmathjax %}<br>
{% mathjax %}
(\mathbf{A}^T)^{-1} = (\mathbf{A}^{-1})^T
{% endmathjax %}<br>
### 初等矩阵
{% mathjax %}Define:{% endmathjax %}单位矩阵只经过一次初等变换而得到的矩阵  
{% mathjax %}
\mathbf{E} \xrightarrow{r_i \leftrightarrow r_j} \mathbf{E}_{ij}
{% endmathjax %}<br>
{% mathjax %}
\mathbf{E} \xrightarrow{kr_i} \mathbf{E}_i(k)
{% endmathjax %}<br>
{% mathjax %}
\mathbf{E} \xrightarrow{r_j+kr_i} \mathbf{E}_{ij}(k)
{% endmathjax %}<br>
对矩阵{% mathjax %}\mathbf{A}{% endmathjax %}作初等**行**变换等价于用对于的初等矩阵**左**乘矩阵{% mathjax %}\mathbf{A}{% endmathjax %}  
对矩阵{% mathjax %}\mathbf{A}{% endmathjax %}作初等**列**变换等价于用对于的初等矩阵**右**乘矩阵{% mathjax %}\mathbf{A}{% endmathjax %}  

## 行列式
{% mathjax %}
当\mathbf{A}为方阵 \Rightarrow
\begin{cases}
  设\mathbf{A} \underset{(i \neq j)}{\xrightarrow{r_i \leftrightarrow r_j}} \mathbf{B}, 则 \left | \mathbf{B} \right | = -\left | \mathbf{A} \right | \\
  设\mathbf{A} \xrightarrow{r_j + kr_i} \mathbf{B}, 则 \left | \mathbf{B} \right | = \left | \mathbf{A} \right | \\
  设\mathbf{A} \underset{(k \neq 0)}{\xrightarrow{kr_i}} \mathbf{B}, 则 \left | \mathbf{B} \right | = k\left | \mathbf{A} \right | \\
\end{cases}
{% endmathjax %}<br>
{% mathjax %}
\left | \mathbf{A} \right | = \left | \mathbf{A}^T \right |
{% endmathjax %}<br>
{% mathjax %}
若\mathbf{A},\mathbf{B}为方阵 \Rightarrow 
\begin{cases}
  \left | \mathbf{AB} \right | = \left | \mathbf{A} \right | \left | \mathbf{B} \right | \\
  \left | \mathbf{AB} \right | = \left | \mathbf{BA} \right |
\end{cases}
{% endmathjax %}<br>
### 行列式展开定理
{% mathjax %}
按第k列展开\sum_{k=1}^n a_{ik}\mathbf{A}_{jk} = 
\begin{cases}
  \left | \mathbf{A} \right | , j = i, \\
  0, j \neq i,
\end{cases}
{% endmathjax %}<br>
{% mathjax %}
按第k行展开\sum_{k=1}^n a_{ki}\mathbf{A}_{kj} = 
\begin{cases}
  \left | \mathbf{A} \right | , j = i, \\
  0, j \neq i,
\end{cases}
{% endmathjax %}<br>
{% mathjax %}
其中i,j = 1,2,...,m
{% endmathjax %}<br>
### 伴随矩阵

{% mathjax %}
\mathbf{AA}^*=\mathbf{A}^*\mathbf{A}=\left | \mathbf{A} \right |\mathbf{E}
{% endmathjax %}<br>
{% mathjax %}
\mathbf{A}^{-1} = \frac{1}{\left | \mathbf{A} \right |} \mathbf{A}^*
{% endmathjax %}<br>
## 分块矩阵

将矩阵 {% mathjax %}\mathbf{A}{% endmathjax %} 用纵线和横线分成若干小块：

{% mathjax %}
\mathbf{A}=
\begin{pmatrix}
\mathbf{A}_{11} & \mathbf{A}_{12} & \cdots & \mathbf{A}_{1t} \\
\mathbf{A}_{21} & \mathbf{A}_{22} & \cdots & \mathbf{A}_{2t} \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{A}_{s1} & \mathbf{A}_{s2} & \cdots & \mathbf{A}_{st}
\end{pmatrix}
{% endmathjax %}<br>
### 分块加法

两个**行数相同、列数相同**且**分块方式相同**的矩阵相加时，对应子块相加：

{% mathjax %}
\mathbf{A}+\mathbf{B}=
\begin{pmatrix}
\mathbf{A}_{11}+\mathbf{B}_{11} & \cdots & \mathbf{A}_{1t}+\mathbf{B}_{1t} \\
\vdots & \ddots & \vdots \\
\mathbf{A}_{s1}+\mathbf{B}_{s1} & \cdots & \mathbf{A}_{st}+\mathbf{B}_{st}
\end{pmatrix}
{% endmathjax %}<br>
### 分块数乘

{% mathjax %}
k\mathbf{A}=
\begin{pmatrix}
k\mathbf{A}_{11} & \cdots & k\mathbf{A}_{1t} \\
\vdots & \ddots & \vdots \\
k\mathbf{A}_{s1} & \cdots & k\mathbf{A}_{st}
\end{pmatrix}
{% endmathjax %}<br>
### 分块乘法

将 {% mathjax %}\mathbf{A}_{m\times n}{% endmathjax %} 和 {% mathjax %}\mathbf{B}_{n\times l}{% endmathjax %} 分块时，要求 **{% mathjax %}\mathbf{A}{% endmathjax %} 的列的分法与 {% mathjax %}\mathbf{B}{% endmathjax %} 的行的分法一致**：

{% mathjax %}
\mathbf{C}_{ij}=\sum_{k=1}^{t} \mathbf{A}_{ik}\mathbf{B}_{kj}
{% endmathjax %}<br>
### 分块转置

{% mathjax %}
\mathbf{A}^T=
\begin{pmatrix}
\mathbf{A}_{11}^T & \mathbf{A}_{21}^T & \cdots & \mathbf{A}_{s1}^T \\
\mathbf{A}_{12}^T & \mathbf{A}_{22}^T & \cdots & \mathbf{A}_{s2}^T \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{A}_{1t}^T & \mathbf{A}_{2t}^T & \cdots & \mathbf{A}_{st}^T
\end{pmatrix}
{% endmathjax %}<br>
即：子块先各自转置，再把子块本身当作元素进行转置。

### 分块对角矩阵

{% mathjax %}
\mathbf{A}=
\begin{pmatrix}
\mathbf{A}_1 & \mathbf{O} & \cdots & \mathbf{O} \\
\mathbf{O} & \mathbf{A}_2 & \cdots & \mathbf{O} \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{O} & \mathbf{O} & \cdots & \mathbf{A}_s
\end{pmatrix}
{% endmathjax %}<br>
称为**分块对角矩阵**（或准对角矩阵），记作 {% mathjax %}\mathbf{A}=\text{diag}(\mathbf{A}_1,\mathbf{A}_2,\dots,\mathbf{A}_s){% endmathjax %}。

**性质：**
- 行列式：{% mathjax %}\left | \mathbf{A} \right | = \left | \mathbf{A}_1 \right |\cdot \left | \mathbf{A}_2 \right |\cdots \left | \mathbf{A}_s \right |{% endmathjax %}
- 幂运算：{% mathjax %}\mathbf{A}^k=\text{diag}(\mathbf{A}_1^k,\mathbf{A}_2^k,\dots,\mathbf{A}_s^k){% endmathjax %}
- 可逆性：{% mathjax %}\mathbf{A}{% endmathjax %} 可逆 {% mathjax %}\Leftrightarrow{% endmathjax %} 每个 {% mathjax %}\mathbf{A}_i{% endmathjax %} 都可逆，且 {% mathjax %}\mathbf{A}^{-1}=\text{diag}(\mathbf{A}_1^{-1},\mathbf{A}_2^{-1},\dots,\mathbf{A}_s^{-1}){% endmathjax %}

### 分块三角矩阵

上三角分块矩阵（对角块均为方阵）：

{% mathjax %}
\mathbf{A}=
\begin{pmatrix}
\mathbf{A}_{11} & \mathbf{A}_{12} & \cdots & \mathbf{A}_{1s} \\
\mathbf{O} & \mathbf{A}_{22} & \cdots & \mathbf{A}_{2s} \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{O} & \mathbf{O} & \cdots & \mathbf{A}_{ss}
\end{pmatrix}
{% endmathjax %}<br>
**性质：**
- 行列式：{% mathjax %}\left | \mathbf{A} \right | = \left | \mathbf{A}_{11} \right |\cdot \left | \mathbf{A}_{22} \right |\cdots \left | \mathbf{A}_{ss} \right |{% endmathjax %}
- 对于分块下三角矩阵同理

### 常用分块矩阵公式

**拉普拉斯展开式：**

{% mathjax %}
\begin{vmatrix}
\mathbf{A} & \mathbf{O} \\
\mathbf{C} & \mathbf{B}
\end{vmatrix}=
\begin{vmatrix}
\mathbf{A} & \mathbf{C} \\
\mathbf{O} & \mathbf{B}
\end{vmatrix}=
\left | \mathbf{A} \right | \cdot \left | \mathbf{B} \right |
{% endmathjax %}<br>
{% mathjax %}
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
{% endmathjax %}<br>
**分块矩阵求逆（{% mathjax %}\mathbf{A}{% endmathjax %} 可逆）：**

{% mathjax %}
\begin{pmatrix}
\mathbf{A} & \mathbf{O} \\
\mathbf{O} & \mathbf{B}
\end{pmatrix}^{-1}=
\begin{pmatrix}
\mathbf{A}^{-1} & \mathbf{O} \\
\mathbf{O} & \mathbf{B}^{-1}
\end{pmatrix}
{% endmathjax %}<br>
{% mathjax %}
\begin{pmatrix}
\mathbf{A} & \mathbf{C} \\
\mathbf{O} & \mathbf{B}
\end{pmatrix}^{-1}=
\begin{pmatrix}
\mathbf{A}^{-1} & -\mathbf{A}^{-1}\mathbf{C}\mathbf{B}^{-1} \\
\mathbf{O} & \mathbf{B}^{-1}
\end{pmatrix}
{% endmathjax %}<br>
{% mathjax %}
\begin{pmatrix}
\mathbf{A} & \mathbf{O} \\
\mathbf{C} & \mathbf{B}
\end{pmatrix}^{-1}=
\begin{pmatrix}
\mathbf{A}^{-1} & \mathbf{O} \\
-\mathbf{B}^{-1}\mathbf{C}\mathbf{A}^{-1} & \mathbf{B}^{-1}
\end{pmatrix}
{% endmathjax %}<br>
## 线性相关性
**唯一表示定理**：设向量组{% mathjax %}\mathbf{\alpha}_1,\mathbf{\alpha}_2,...,\mathbf{\alpha}_n{% endmathjax %}线性无关，又设该向量组添加一个向量{% mathjax %}\beta{% endmathjax %}后的向量组{% mathjax %}\mathbf{\alpha}_1,\mathbf{\alpha}_2,...,\mathbf{\alpha}_n,\beta{% endmathjax %}线性相关，则向量{% mathjax %}\beta{% endmathjax %}可由向量组{% mathjax %}\mathbf{\alpha}_1,\mathbf{\alpha}_2,...,\mathbf{\alpha}_n{% endmathjax %}线性表示且表示式唯一

**线性无关的证明**： 对于向量组{% mathjax %}\mathbf{\alpha}_1,\mathbf{\alpha}_2,...,\mathbf{\alpha}_n{% endmathjax %}，设实数{% mathjax %}k_1,k_2,...,k_n{% endmathjax %}满足{% mathjax %}k_1\mathbf{\alpha}_1+k_2\mathbf{\alpha}_2+...+k_n\mathbf{\alpha}_n=0{% endmathjax %}  
若{% mathjax %}k_i,i=1,2,...,n{% endmathjax %}不全为0，则线性相关；若全为0，则线性无关
### 秩

{% mathjax %}
r(V) = n \Leftrightarrow 向量组V内的向量线性无关
{% endmathjax %}<br>
{% mathjax %}
r(V) < n \Leftrightarrow 向量组V内的向量线性无关
{% endmathjax %}<br>
**任意矩阵的行秩与列秩相等，其值等于它对应的最简阶梯型矩阵或阶梯型矩阵的非零行的行数**  
**初等变换不改变矩阵的秩**
**规定零矩阵的秩为0**

设{% mathjax %}\mathbf{P},\mathbf{Q}{% endmathjax %}都是可逆矩阵，则
{% mathjax %}
r(\mathbf{PAQ}) = r(A)
{% endmathjax %}<br>
矩阵的秩有如下性质
{% mathjax %}
r(\mathbf{A}^T)=r(\mathbf{A})
{% endmathjax %}<br>
{% mathjax %}
r(\mathbf{A}_{m\times n}) \leq \text{min}(m,n)
{% endmathjax %}<br>
{% mathjax %}
r(\mathbf{AB}) \leq \text{min} ( r(\mathbf{A}) , r(\mathbf{B}) )
{% endmathjax %}<br>
{% mathjax %}
r(\mathbf{A}) = r \Leftrightarrow \mathbf{A}中有一个r阶子式D_r \neq 0, 且所有r+1阶子式（如果存在的话）都等于0
{% endmathjax %}<br>
{% mathjax %}
对于任意n阶(n \leq 2)矩阵\mathbf{A},有r(\mathbf{A}^*) = 
\begin{cases}
  n \quad r(\mathbf{A}) = n \\
  1 \quad r(\mathbf{A}) = n-1 \\
  0 \quad r(\mathbf{A}) < n-1
\end{cases}
{% endmathjax %}<br>
### 向量空间

**维数**{% mathjax %}\text{dim}{% endmathjax %}：向量空间{% mathjax %}V{% endmathjax %}的基所含向量的个数  

**变基**：
设{% mathjax %}r{% endmathjax %}维向量空间{% mathjax %}V=\text{span}(\alpha_1,...,\alpha_r)=\text{span}(\beta_1,...,\beta_r){% endmathjax %}，则{% mathjax %}\beta_1,...,\beta_r{% endmathjax %}可由{% mathjax %}\alpha_1,...,\alpha_r{% endmathjax %}线性表示：

{% mathjax %}
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
{% endmathjax %}<br>
称矩阵{% mathjax %}\mathbf{P}=\begin{bmatrix} p_{ij} \end{bmatrix}{% endmathjax %}为由基{% mathjax %}\begin{bmatrix} \alpha_1,...,\alpha_r \end{bmatrix}{% endmathjax %}到基{% mathjax %}\begin{bmatrix} \beta_1,...,\beta_r \end{bmatrix}{% endmathjax %}的**过渡矩阵**  

过渡矩阵一定为可逆矩阵  
要将基{% mathjax %}\begin{bmatrix} \alpha_1,...,\alpha_r \end{bmatrix}{% endmathjax %}下的坐标{% mathjax %}\mathbf{x}{% endmathjax %}转换到基{% mathjax %}\begin{bmatrix} \beta_1,...,\beta_r \end{bmatrix}{% endmathjax %}下的坐标{% mathjax %}\mathbf{y}{% endmathjax %},需要左乘过渡矩阵的逆矩阵  

{% mathjax %}
\mathbf{y} = \mathbf{P}^{-1} \mathbf{x}
{% endmathjax %}<br>
## 特征值

**求解**

由 {% mathjax %}\det(\mathbf{A}-\lambda\mathbf{E})=0{% endmathjax %} 求特征值 {% mathjax %}\lambda_i{% endmathjax %}，再对每个 {% mathjax %}\lambda_i{% endmathjax %} 解齐次方程组 {% mathjax %}(\mathbf{A}-\lambda_i\mathbf{E})\mathbf{x}=\mathbf{O}{% endmathjax %} 求特征向量。

{% mathjax %}
\det(\mathbf{A}-\lambda\mathbf{E}) = 
\begin{vmatrix}
a_{11}-\lambda & a_{12} & \cdots & a_{1n} \\
a_{21} & a_{22}-\lambda & \cdots & a_{2n} \\
\vdots & \vdots & \ddots & \vdots \\
a_{n1} & a_{n2} & \cdots & a_{nn}-\lambda
\end{vmatrix}
{% endmathjax %}<br>
展开后为 {% mathjax %}\lambda{% endmathjax %} 的 {% mathjax %}n{% endmathjax %} 次多项式，称**特征多项式**。特征方程 {% mathjax %}\det(\mathbf{A}-\lambda\mathbf{E})=0{% endmathjax %} 的根即特征值。

**性质**

{% mathjax %}
\mathbf{A}\alpha = \lambda \alpha \Rightarrow \mathbf{A}^k\alpha = \lambda^k\alpha
{% endmathjax %}<br>
{% mathjax %}
\mathbf{A}\alpha = \lambda \alpha \Rightarrow \mathbf{A}^{-1}\alpha = \lambda^{-1}\alpha \quad (\mathbf{A}可逆)
{% endmathjax %}<br>
{% mathjax %}
设\varphi (z)=a_0+a_1z+...+a_m z^m,则\mathbf{A}\alpha = \lambda \alpha \Rightarrow \varphi(\mathbf{A})\alpha = \varphi(\lambda)\alpha
{% endmathjax %}<br>
{% mathjax %}
\mathbf{A}和\mathbf{A}^T有相同的特征值
{% endmathjax %}<br>
{% mathjax %}
不同特征值对应的特征向量线性无关
{% endmathjax %}<br>
若 {% mathjax %}n{% endmathjax %} 阶矩阵 {% mathjax %}\mathbf{A}{% endmathjax %} 的特征值为 {% mathjax %}\lambda_1,\lambda_2,...,\lambda_n{% endmathjax %}，则：

{% mathjax %}
\sum_{i=1}^n \lambda_i = \operatorname{tr}(\mathbf{A})
{% endmathjax %}<br>
{% mathjax %}
\prod_{i=1}^n \lambda_i = \left | \mathbf{A} \right |
{% endmathjax %}<br>
**代数重数**：特征值 {% mathjax %}\lambda_i{% endmathjax %} 作为特征多项式根的重数，记作 {% mathjax %}m_i{% endmathjax %}。

**几何重数**：特征值 {% mathjax %}\lambda_i{% endmathjax %} 对应的特征子空间的维数 {% mathjax %}= n - r(\mathbf{A}-\lambda_i\mathbf{E}){% endmathjax %}，记作 {% mathjax %}d_i{% endmathjax %}。

恒有 {% mathjax %}1 \leq d_i \leq m_i \leq n{% endmathjax %}。

### 方阵的对角化

{% mathjax %}\mathbf{A}{% endmathjax %} 可对角化 {% mathjax %}\Leftrightarrow{% endmathjax %} 存在可逆矩阵 {% mathjax %}\mathbf{P}{% endmathjax %} 使 {% mathjax %}\mathbf{P}^{-1}\mathbf{AP}=\mathbf{\Lambda}=\operatorname{diag}(\lambda_1,...,\lambda_n){% endmathjax %}。

**充要条件：**
- {% mathjax %}\mathbf{A}{% endmathjax %} 有 {% mathjax %}n{% endmathjax %} 个线性无关的特征向量
- 每个特征值的几何重数 = 代数重数，即 {% mathjax %}d_i = m_i{% endmathjax %}
- {% mathjax %}n{% endmathjax %} 阶矩阵有 {% mathjax %}n{% endmathjax %} 个不同特征值 {% mathjax %}\Rightarrow{% endmathjax %} 可对角化（充分非必要）

**求解步骤：**
1. 求特征值 {% mathjax %}\lambda_1,...,\lambda_n{% endmathjax %}
2. 对每个 {% mathjax %}\lambda_i{% endmathjax %}，解 {% mathjax %}(\mathbf{A}-\lambda_i\mathbf{E})\mathbf{x}=\mathbf{O}{% endmathjax %} 得线性无关的特征向量
3. 以特征向量为列构成 {% mathjax %}\mathbf{P}{% endmathjax %}，对应的特征值排成 {% mathjax %}\mathbf{\Lambda}{% endmathjax %}
4. 验证 {% mathjax %}\mathbf{P}^{-1}\mathbf{AP}=\mathbf{\Lambda}{% endmathjax %}

## Euclid空间

### 向量内积

{% mathjax %}Define:{% endmathjax %} 在 {% mathjax %}n{% endmathjax %} 维实向量空间 {% mathjax %}\mathbb{R}^n{% endmathjax %} 中定义**内积**：

{% mathjax %}
\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle = \alpha_1\beta_1 + \alpha_2\beta_2 + \cdots + \alpha_n\beta_n = \mathbf{\alpha}^T\mathbf{\beta}
{% endmathjax %}<br>
定义了内积的 {% mathjax %}\mathbb{R}^n{% endmathjax %} 称为 **Euclid空间**。

**内积性质（{% mathjax %}\mathbf{\alpha},\mathbf{\beta},\mathbf{\gamma} \in \mathbb{R}^n,\ k \in \mathbb{R}{% endmathjax %}）：**

{% mathjax %}
\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle = \left\langle \mathbf{\beta}, \mathbf{\alpha} \right\rangle
{% endmathjax %}<br>
{% mathjax %}
\left\langle k\mathbf{\alpha}, \mathbf{\beta} \right\rangle = k\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle
{% endmathjax %}<br>
{% mathjax %}
\left\langle \mathbf{\alpha} + \mathbf{\beta}, \mathbf{\gamma} \right\rangle = \left\langle \mathbf{\alpha}, \mathbf{\gamma} \right\rangle + \left\langle \mathbf{\beta}, \mathbf{\gamma} \right\rangle
{% endmathjax %}<br>
{% mathjax %}
\left\langle \mathbf{\alpha}, \mathbf{\alpha} \right\rangle \geq 0,\ 且\ \left\langle \mathbf{\alpha}, \mathbf{\alpha} \right\rangle = 0 \Leftrightarrow \mathbf{\alpha} = \mathbf{O}
{% endmathjax %}<br>
**范数（长度）：**
{% mathjax %}
\|\mathbf{\alpha}\| = \sqrt{\left\langle \mathbf{\alpha}, \mathbf{\alpha} \right\rangle} = \sqrt{\alpha_1^2 + \alpha_2^2 + \cdots + \alpha_n^2}
{% endmathjax %}<br>
**夹角：**
{% mathjax %}
\cos\langle \mathbf{\alpha}, \mathbf{\beta} \rangle = \frac{\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle}{\|\mathbf{\alpha}\| \cdot \|\mathbf{\beta}\|}
{% endmathjax %}<br>
**正交：** {% mathjax %}\left\langle \mathbf{\alpha}, \mathbf{\beta} \right\rangle = 0 \Leftrightarrow \mathbf{\alpha} \perp \mathbf{\beta}{% endmathjax %}

### 标准正交基

若向量组 {% mathjax %}\mathbf{\eta}_1,\mathbf{\eta}_2,\dots,\mathbf{\eta}_r{% endmathjax %} 满足：
- 两两正交：{% mathjax %}\left\langle \mathbf{\eta}_i, \mathbf{\eta}_j \right\rangle = 0 \ (i \neq j){% endmathjax %}
- 均为单位向量：{% mathjax %}\|\mathbf{\eta}_i\| = 1{% endmathjax %}

则称其为**标准正交向量组**。构成向量空间基的标准正交向量组称为**标准正交基**。

标准正交基下坐标计算简便：若 {% mathjax %}\mathbf{\eta}_1,\dots,\mathbf{\eta}_n{% endmathjax %} 为标准正交基，则 {% mathjax %}\mathbf{\alpha} = \sum_{i=1}^n \left\langle \mathbf{\alpha}, \mathbf{\eta}_i \right\rangle \mathbf{\eta}_i{% endmathjax %}。

### 施密特正交化

将线性无关组 {% mathjax %}\mathbf{\alpha}_1,\mathbf{\alpha}_2,\dots,\mathbf{\alpha}_r{% endmathjax %} 化为标准正交组 {% mathjax %}\mathbf{\eta}_1,\mathbf{\eta}_2,\dots,\mathbf{\eta}_r{% endmathjax %}。

**求解步骤：**
1. 正交化（得 {% mathjax %}\mathbf{\beta}_1,\dots,\mathbf{\beta}_r{% endmathjax %}）：

{% mathjax %}
\mathbf{\beta}_1 = \mathbf{\alpha}_1
{% endmathjax %}<br>
{% mathjax %}
\mathbf{\beta}_k = \mathbf{\alpha}_k - \sum_{i=1}^{k-1} \frac{\left\langle \mathbf{\alpha}_k, \mathbf{\beta}_i \right\rangle}{\left\langle \mathbf{\beta}_i, \mathbf{\beta}_i \right\rangle} \mathbf{\beta}_i \quad (k=2,3,\dots,r)
{% endmathjax %}<br>
2. 单位化（得 {% mathjax %}\mathbf{\eta}_1,\dots,\mathbf{\eta}_r{% endmathjax %}）：

{% mathjax %}
\mathbf{\eta}_k = \frac{\mathbf{\beta}_k}{\|\mathbf{\beta}_k\|} \quad (k=1,2,\dots,r)
{% endmathjax %}<br>
### 正交矩阵

{% mathjax %}Define:{% endmathjax %} 若 {% mathjax %}n{% endmathjax %} 阶实矩阵 {% mathjax %}\mathbf{Q}{% endmathjax %} 满足 {% mathjax %}\mathbf{Q}^T\mathbf{Q} = \mathbf{E}{% endmathjax %}（即 {% mathjax %}\mathbf{Q}^{-1} = \mathbf{Q}^T{% endmathjax %}），则称 {% mathjax %}\mathbf{Q}{% endmathjax %} 为**正交矩阵**。

**性质：**

{% mathjax %}
\mathbf{Q}为正交矩阵 \Leftrightarrow \mathbf{Q}的列(行)向量组构成标准正交基
{% endmathjax %}<br>
{% mathjax %}
\mathbf{Q}为正交矩阵 \Leftrightarrow \mathbf{Q}^T = \mathbf{Q}^{-1}
{% endmathjax %}<br>
{% mathjax %}
\left | \mathbf{Q} \right | = \pm 1
{% endmathjax %}<br>
{% mathjax %}
若\mathbf{Q}_1,\mathbf{Q}_2为正交矩阵，则\mathbf{Q}_1\mathbf{Q}_2也是正交矩阵
{% endmathjax %}<br>
## 实对称矩阵

### 实对称矩阵的特征值和特征向量

{% mathjax %}Define:{% endmathjax %} {% mathjax %}\mathbf{A}{% endmathjax %} 为实对称矩阵 {% mathjax %}\Leftrightarrow{% endmathjax %} {% mathjax %}\mathbf{A}^T = \mathbf{A}{% endmathjax %} 且 {% mathjax %}\mathbf{A}{% endmathjax %} 的元素均为实数。

**性质：**
- 实对称矩阵的特征值均为实数
- 实对称矩阵的属于不同特征值的特征向量相互正交
- 实对称矩阵的每个特征值的几何重数 = 代数重数

### 对角化

{% mathjax %}\mathbf{A}{% endmathjax %} 实对称 {% mathjax %}\Rightarrow{% endmathjax %} 存在正交矩阵 {% mathjax %}\mathbf{Q}{% endmathjax %}，使：

{% mathjax %}
\mathbf{Q}^{-1}\mathbf{AQ} = \mathbf{Q}^T\mathbf{AQ} = \operatorname{diag}(\lambda_1,\lambda_2,\dots,\lambda_n)
{% endmathjax %}<br>
**求解步骤：**
1. 求 {% mathjax %}|\mathbf{A}-\lambda\mathbf{E}|=0{% endmathjax %} 的根，得特征值 {% mathjax %}\lambda_1,\dots,\lambda_n{% endmathjax %}（均为实数）
2. 对每个 {% mathjax %}\lambda_i{% endmathjax %}，解 {% mathjax %}(\mathbf{A}-\lambda_i\mathbf{E})\mathbf{x}=\mathbf{O}{% endmathjax %} 得基础解系（特征向量）
3. 若 {% mathjax %}\lambda_i{% endmathjax %} 为重根，对其特征向量作 Schmidt 正交化
4. 将所有特征向量单位化，以它们为列构成正交矩阵 {% mathjax %}\mathbf{Q}{% endmathjax %}
5. 得 {% mathjax %}\mathbf{Q}^{-1}\mathbf{AQ} = \mathbf{Q}^T\mathbf{AQ} = \operatorname{diag}(\lambda_1,\dots,\lambda_n){% endmathjax %}

## 二次齐次多项式

### 二次型

{% mathjax %}Define:{% endmathjax %} {% mathjax %}n{% endmathjax %} 元**二次型**：

{% mathjax %}
f(x_1,x_2,\dots,x_n) = \sum_{i=1}^n \sum_{j=1}^n a_{ij} x_i x_j = \mathbf{x}^T\mathbf{Ax}
{% endmathjax %}<br>
其中 {% mathjax %}\mathbf{x} = (x_1,x_2,\dots,x_n)^T{% endmathjax %}，{% mathjax %}\mathbf{A}{% endmathjax %} 为 {% mathjax %}n{% endmathjax %} 阶**实对称矩阵**（{% mathjax %}a_{ij}=a_{ji}{% endmathjax %}），称为二次型的**矩阵**。

构造 {% mathjax %}\mathbf{A}{% endmathjax %} 的方法：对角元 {% mathjax %}a_{ii}{% endmathjax %} 为 {% mathjax %}x_i^2{% endmathjax %} 的系数；非对角元 {% mathjax %}a_{ij}=a_{ji}{% endmathjax %} 为 {% mathjax %}x_i x_j{% endmathjax %} 系数的一半。

### 矩阵的合同

{% mathjax %}Define:{% endmathjax %} 对于 {% mathjax %}n{% endmathjax %} 阶方阵 {% mathjax %}\mathbf{A},\mathbf{B}{% endmathjax %}，若存在可逆矩阵 {% mathjax %}\mathbf{C}{% endmathjax %} 使 {% mathjax %}\mathbf{B} = \mathbf{C}^T\mathbf{AC}{% endmathjax %}，则称 {% mathjax %}\mathbf{A}{% endmathjax %} 与 {% mathjax %}\mathbf{B}{% endmathjax %} **合同**。

**性质：**
- 合同关系具有自反性、对称性、传递性
- 合同变换保持对称性：若 {% mathjax %}\mathbf{A}{% endmathjax %} 对称，则 {% mathjax %}\mathbf{B}{% endmathjax %} 也对称
- 合同变换不改变秩：{% mathjax %}r(\mathbf{B}) = r(\mathbf{A}){% endmathjax %}

### 用正交变换化二次型为标准形

存在正交变换 {% mathjax %}\mathbf{x} = \mathbf{Qy}{% endmathjax %}（{% mathjax %}\mathbf{Q}{% endmathjax %} 正交），使：

{% mathjax %}
f = \lambda_1 y_1^2 + \lambda_2 y_2^2 + \cdots + \lambda_n y_n^2
{% endmathjax %}<br>
其中 {% mathjax %}\lambda_i{% endmathjax %} 为 {% mathjax %}\mathbf{A}{% endmathjax %} 的特征值。此即**主轴定理**：任何实二次型均可通过正交变换化为标准形，系数恰为矩阵的特征值。

{% mathjax %}
f = \mathbf{x}^T\mathbf{Ax} \xrightarrow{\mathbf{x} = \mathbf{Qy}} f = \mathbf{y}^T(\mathbf{Q}^T\mathbf{AQ})\mathbf{y} = \sum_{i=1}^n \lambda_i y_i^2
{% endmathjax %}<br>
**求解步骤：**
1. 写出二次型的对称矩阵 {% mathjax %}\mathbf{A}{% endmathjax %}（对角元为平方项系数，非对角元为交叉项系数的一半）
2. 对 {% mathjax %}\mathbf{A}{% endmathjax %} 作正交对角化，得特征值 {% mathjax %}\lambda_i{% endmathjax %} 和正交矩阵 {% mathjax %}\mathbf{Q}{% endmathjax %}
3. 令 {% mathjax %}\mathbf{x} = \mathbf{Qy}{% endmathjax %}，得标准形 {% mathjax %}f = \sum \lambda_i y_i^2{% endmathjax %}

## 正定二次型与正定矩阵

### 二次型的规范形

由**惯性定理**：二次型经任何可逆线性变换化为标准形后，正平方项个数 {% mathjax %}p{% endmathjax %} 和负平方项个数 {% mathjax %}q{% endmathjax %} 不变（与变换方式无关）。

{% mathjax %}p{% endmathjax %}——**正惯性指数**，{% mathjax %}q{% endmathjax %}——**负惯性指数**，{% mathjax %}p-q{% endmathjax %}——**符号差**，{% mathjax %}p+q = r(\mathbf{A}){% endmathjax %}。

令 {% mathjax %}f = y_1^2 + \cdots + y_p^2 - y_{p+1}^2 - \cdots - y_{p+q}^2{% endmathjax %}（系数均为 {% mathjax %}\pm 1{% endmathjax %}），此即二次型的**规范形**。

由**霍尔维茨定理**（Sylvester 判别法）：{% mathjax %}\mathbf{A}{% endmathjax %} 正定 {% mathjax %}\Leftrightarrow{% endmathjax %} {% mathjax %}\mathbf{A}{% endmathjax %} 的各阶顺序主子式均 {% mathjax %}> 0{% endmathjax %}：

{% mathjax %}
\Delta_k = 
\begin{vmatrix}
a_{11} & \cdots & a_{1k} \\
\vdots & \ddots & \vdots \\
a_{k1} & \cdots & a_{kk}
\end{vmatrix}
> 0 \quad (k=1,2,\dots,n)
{% endmathjax %}<br>
**负定：** 奇数阶顺序主子式 {% mathjax %}< 0{% endmathjax %}，偶数阶 {% mathjax %}> 0{% endmathjax %}

### 正定二次形

{% mathjax %}Define:{% endmathjax %} 若对任意 {% mathjax %}\mathbf{x} \neq \mathbf{O}{% endmathjax %}，恒有 {% mathjax %}f(\mathbf{x}) = \mathbf{x}^T\mathbf{Ax} > 0{% endmathjax %}，则称 {% mathjax %}f{% endmathjax %} 为**正定二次型**，{% mathjax %}\mathbf{A}{% endmathjax %} 为**正定矩阵**。

**判别条件（以下等价）：**
- {% mathjax %}\mathbf{A}{% endmathjax %} 正定
- {% mathjax %}\mathbf{A}{% endmathjax %} 的特征值全为正
- {% mathjax %}f{% endmathjax %} 的正惯性指数 {% mathjax %}p = n{% endmathjax %}
- {% mathjax %}\mathbf{A}{% endmathjax %} 的各阶顺序主子式 {% mathjax %}> 0{% endmathjax %}
- 存在可逆矩阵 {% mathjax %}\mathbf{C}{% endmathjax %} 使 {% mathjax %}\mathbf{A} = \mathbf{C}^T\mathbf{C}{% endmathjax %}

**半正定：** {% mathjax %}f \geq 0{% endmathjax %}（{% mathjax %}\forall \mathbf{x} \neq \mathbf{O}{% endmathjax %}）{% mathjax %}\Leftrightarrow{% endmathjax %} 特征值均 {% mathjax %}\geq 0{% endmathjax %} {% mathjax %}\Leftrightarrow{% endmathjax %} 正惯性指数 {% mathjax %}p = r(\mathbf{A}) < n{% endmathjax %}

# 方程组的解问题
{% mathjax %}
\mathbf{Ax}=\mathbf{O}仅有零解
{% endmathjax %}<br>
{% mathjax %}
\Leftrightarrow \mathbf{Ax}=\mathbf{b} 有唯一解
{% endmathjax %}<br>
{% mathjax %}
\Leftrightarrow \left | \mathbf{A} \right |  \neq 0
{% endmathjax %}<br>
{% mathjax %}
\Leftrightarrow \mathbf{A}可逆
{% endmathjax %}<br>
{% mathjax %}
\Leftrightarrow \mathbf{A}可以分解为有限个初等矩阵的乘积
{% endmathjax %}<br>
{% mathjax %}
\Leftrightarrow \mathbf{A}的列向量组线性无关
{% endmathjax %}<br>
{% mathjax %}
\Leftrightarrow \mathbf{A}的行向量组线性无关
{% endmathjax %}<br>
{% mathjax %}
\Leftrightarrow \mathbf{b}可由\mathbf{A}的列向量组线性表示且表示式唯一
{% endmathjax %}<br>
{% mathjax %}
\Leftrightarrow r(\mathbf{A}) = n
{% endmathjax %}<br>
## 线性方程组解的结构

### 基础解系

{% mathjax %}Define:{% endmathjax %} 齐次线性方程组 {% mathjax %}\mathbf{A}_{m\times n}\mathbf{x}=\mathbf{O}{% endmathjax %} 的解空间的**基**称为该方程组的一个**基础解系**。

设 {% mathjax %}r(\mathbf{A})=r<n{% endmathjax %}，则将 {% mathjax %}\mathbf{A}{% endmathjax %} 通过初等行变换化为最简阶梯型，取 {% mathjax %}n-r{% endmathjax %} 个自由未知量（通常取不在主元列的变量），分别令其中一个为 {% mathjax %}1{% endmathjax %}、其余为 {% mathjax %}0{% endmathjax %}，回代解得主元变量的值，得到的 {% mathjax %}n-r{% endmathjax %} 个线性无关的解向量 {% mathjax %}\mathbf{\xi}_1,\mathbf{\xi}_2,\dots,\mathbf{\xi}_{n-r}{% endmathjax %} 即构成基础解系。

**解空间维数：** {% mathjax %}\dim\{\mathbf{x} \mid \mathbf{Ax}=\mathbf{O}\} = n - r(\mathbf{A}){% endmathjax %}

### 齐次线性方程组 {% mathjax %}\mathbf{A}_{m\times n}\mathbf{x}=\mathbf{O}{% endmathjax %}

{% mathjax %}
\mathbf{Ax}=\mathbf{O}\ 必有零解\ \mathbf{x}=\mathbf{O}
{% endmathjax %}<br>
{% mathjax %}
\mathbf{Ax}=\mathbf{O}\ 有非零解 \Leftrightarrow r(\mathbf{A}) < n
{% endmathjax %}<br>
{% mathjax %}
\mathbf{Ax}=\mathbf{O}\ 仅有零解 \Leftrightarrow r(\mathbf{A}) = n
{% endmathjax %}<br>
**解的结构（{% mathjax %}r(\mathbf{A}) < n{% endmathjax %} 时）：**

{% mathjax %}
\mathbf{x} = c_1\mathbf{\xi}_1 + c_2\mathbf{\xi}_2 + \cdots + c_{n-r}\mathbf{\xi}_{n-r}, \quad c_i \in \mathbb{R}
{% endmathjax %}<br>
其中 {% mathjax %}\mathbf{\xi}_1,\dots,\mathbf{\xi}_{n-r}{% endmathjax %} 为基础解系。

**求解步骤：**
1. 对系数矩阵 {% mathjax %}\mathbf{A}{% endmathjax %} 作初等行变换，化为阶梯型，求出 {% mathjax %}r(\mathbf{A}){% endmathjax %}
2. 若 {% mathjax %}r=n{% endmathjax %}，仅有零解；若 {% mathjax %}r<n{% endmathjax %}，继续
3. 进一步化为最简阶梯型，确定主元变量与自由变量
4. 令自由变量依次取 {% mathjax %}(1,0,\dots,0)^T,(0,1,\dots,0)^T,\dots,(0,0,\dots,1)^T{% endmathjax %}，回代求出基础解系
5. 通解为基础解系的任意线性组合

### 非齐次线性方程组 {% mathjax %}\mathbf{A}_{m\times n}\mathbf{x}=\mathbf{b}\ (\mathbf{b} \neq \mathbf{O}){% endmathjax %}

**有解判定：**

{% mathjax %}
\mathbf{Ax}=\mathbf{b}\ 有解 \Leftrightarrow r(\mathbf{A}) = r(\mathbf{A} \mid \mathbf{b})
{% endmathjax %}<br>
{% mathjax %}
\begin{cases}
  r(\mathbf{A}) = r(\mathbf{A} \mid \mathbf{b}) = n \Rightarrow 有唯一解 \\
  r(\mathbf{A}) = r(\mathbf{A} \mid \mathbf{b}) < n \Rightarrow 有无穷多解 \\
  r(\mathbf{A}) \neq r(\mathbf{A} \mid \mathbf{b}) \Rightarrow 无解
\end{cases}
{% endmathjax %}<br>
**解的结构（有解时）：**

{% mathjax %}
\mathbf{x} = \mathbf{\eta}^* + c_1\mathbf{\xi}_1 + c_2\mathbf{\xi}_2 + \cdots + c_{n-r}\mathbf{\xi}_{n-r}
{% endmathjax %}<br>
即：**导出组 {% mathjax %}\mathbf{Ax}=\mathbf{O}{% endmathjax %} 的通解 + 原方程组的一个特解 {% mathjax %}\mathbf{\eta}^*{% endmathjax %}**。

**求解步骤：**
1. 对增广矩阵 {% mathjax %}(\mathbf{A} \mid \mathbf{b}){% endmathjax %} 作初等行变换，化为阶梯型
2. 比较 {% mathjax %}r(\mathbf{A}){% endmathjax %} 与 {% mathjax %}r(\mathbf{A} \mid \mathbf{b}){% endmathjax %}：不等则无解
3. 继续化为最简阶梯型，确定主元变量与自由变量
4. 令所有自由变量 {% mathjax %}=0{% endmathjax %}，回代求出一个特解 {% mathjax %}\mathbf{\eta}^*{% endmathjax %}
5. 令 {% mathjax %}\mathbf{b}=\mathbf{O}{% endmathjax %}，求导出组的基础解系 {% mathjax %}\mathbf{\xi}_1,\dots,\mathbf{\xi}_{n-r}{% endmathjax %}
6. 写出通解 {% mathjax %}\mathbf{x} = \mathbf{\eta}^* + \sum c_i\mathbf{\xi}_i{% endmathjax %}

# 方阵相似
{% mathjax %}\text{Define} \colon{% endmathjax %}
若对于{% mathjax %}n{% endmathjax %}阶方阵{% mathjax %}\mathbf{A},\mathbf{B},存在 \mathbf{P}{% endmathjax %}为可逆矩阵，使得
{% mathjax %}
\mathbf{P}^{-1}\mathbf{AP} = \mathbf{B}
{% endmathjax %}<br>
则称{% mathjax %}\mathbf{A}{% endmathjax %}和{% mathjax %}\mathbf{B}{% endmathjax %}相似，记作{% mathjax %}\mathbf{A} \sim \mathbf{B}{% endmathjax %}  

**性质**  
* 自反性 {% mathjax %}\mathbf{A} \sim \mathbf{A}{% endmathjax %}
* 对称性 {% mathjax %}\mathbf{A} \sim \mathbf{B} \Leftrightarrow \mathbf{B} \sim \mathbf{A}{% endmathjax %}
* 传递性 {% mathjax %}\mathbf{A} \sim \mathbf{B},\mathbf{B} \sim \mathbf{C} \Rightarrow \mathbf{A} \sim \mathbf{C}{% endmathjax %}
{% mathjax %}
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
{% endmathjax %}<br>
以上均为必要条件，但非充分。例如特征值相同不能保证相似，除非两个矩阵均可对角化且特征值（含重数）完全相同。
