



# 模型参考自适应控制

## 陈拥瑜

模型参考自适应控制(model reference adaptive control, 简称 mrac) 可以视为跟随参考模型 (reference model)的自适应伺服系统(adaptive servo system) 。 更具体的说，对于相同的输入我们希望被控制系统的输出特性和参考系统的输出特性是一致的。

![MRAC_Structure](MRAC_Structure.png)上图是模型参考自适应控制的基本框图，控制器的参数根据参考模型的输出和实际系统输出的误差实时变化。

## 基于Lyapunov稳定性设计MRAC

### Lyapunov稳定性理论

#### 非线性时不变系统稳定性分析

考虑非线性时不变系统
$$
\dot{x} = f(x)~~~f(0) = 0~~~~~~(1)
$$
首先我们求取平衡点 (equilibrium point) , 令$f(x) = 0$ 得到解: $x(t) = 0$, 我们需要保证该解存在并且是唯一的.  为此函数$f(x)$在原点附近是局部Lipschitz连续的 (locally lipschitz continuous)
$$
||f(x) - f(y)|| \leq L||x - y||~~~~ L > 0~~~~~~(2)
$$


下面我们需要证明平衡点$x (t) = 0$是稳定的.  

Lyapunov稳定性定义如下:
$$
如果对于给定的\epsilon > 0, 存在\delta(\epsilon) > 0, 当初始状态||x(0)|| < \delta, 后续状态满足性质: ||x(t)|| < \epsilon, 0 \leq t \leq \infty \\
那么我们称平衡点x(t) = 0是稳定 (stable)~的
$$

$$
如果当初始状态||x(0)|| \leq \delta, 后续状态变化满足 ||x(t)|| \to 0, t \to \infty, \\
那么我们称平衡点x(t) = 0是渐进稳定 (asymptotically~stable)~的.
$$

$$
如果对于任何初始数值, 解都是渐进稳定的, 那么我们称解是全局渐进稳定的(globally~asymptotically~stable)
$$

![ConceptsStability](/home/chenyongyu/Pictures/ConceptsStability.png)

注意上面所说的稳定性能都是针对于系统平衡点的.



正定函数和正半定函数的定义
$$
一个连续可导函数 V : R^{n} \to R 被称为在区域U \in R^{n} 的正定函数 \\
如果: 
1. V(0) = 0 \\
2. V(x) > 0, x \in U 并且 x \neq 0
$$


一个函数被称为正半定如果把上述条件2改为$V(x) \geq 0  $, 那么该函数是正半定的

#### 寻找Lyapunov函数

假设线性系统
$$
\dfrac{dx}{dt} = Ax~~~~~~(3)
$$
是渐进稳定的. 对任何对称正定矩阵$Q$, 存在唯一的对称正定矩阵$P$, 满足
$$
A^{T}P + PA = -Q~~~~~~(4)
$$
并且函数$V (x) = x^{T}Px$是Lyapunov函数. 



根据上面的原则只要我们规定好$Q$矩阵, 在已知系统矩阵$A$的情况下,  我们可以求解出满足要求的矩阵$P$, 然后构造Lyapunov函数,   假设系统所有的特征值都在左半平面.  
$$
A = \left[ \begin{matrix} a_{1} & a_{2} \\ a_{3} & a_{4} \end{matrix} \right]~~~~~~(5)
$$
假设矩阵$Q$为正定矩阵, $q_{1} \gt 0 $, $q_{2} \gt 0$
$$
Q = \left[ \begin{matrix} q_{1} & 0 \\ 0 & q_{2} \end{matrix} \right]~~~~~~(6)
$$
矩阵$P$形式如下
$$
P = \left[ \begin{matrix} p_{1} & p_{2} \\ p_{2} & p_{3} \end{matrix} \right]~~~~~~(7)
$$


将公式$(5)$, $(6)$和公式$(7)$带入公式$(4)$, 我们得到如下公式:
$$
\left[ \begin{matrix} 2a_{1} & 2a_{3} & 0\\ a_{1} & a_{1} + a_{4} & a_{3} \\ 0 & 2a_{2} & 2a_{4} \end{matrix} \right]
\left[ \begin{matrix} p_{1}\\ p_{2} \\ p_{3} \end{matrix} \right]
=
\left[ \begin{matrix} -q_{1}\\ 0 \\ -q_{2} \end{matrix} \right]~~~~~~(8)
$$
我们可以根据上述公式求解出$p_{1}$, $p_{2}$和$p_{3}$的数值,从而构造出矩阵$P$.

#### 非线性时变系统稳定性分析

考虑线性时变系统
$$
\dfrac{dx}{dt}  = f(x, t)~~~~~~(9)
$$
原点$(0, 0)$是系统$(9)$的平衡点如果$f(0, t) = 0, ~\forall ~t\geq 0$.    
$$
一致Lyapunov稳定性: \\
x(t) = 0是一致稳定: 如果对于\epsilon > 0, 存在\delta(\epsilon) > 0, 与初始时间t_{0}无关, 使得当t\to \infty,~x (t) \to 0, \\ 
对于所有的初始时间t_{0}和初始状态||x(t_{0})||
$$


### 基本步骤

1. 确定控制器结构
2. 推导误差方程
3. 找出Lyapunov方程
4. 确定满足满足Lyapunov理论自适应法则 (adaption law) 

### 一阶系统 (first- order system) Lyapunov MRAC 设计

对于车辆横向和纵向执行系统, 我们都可以使用简单的一阶系统对其进行建模

系统模型
$$
\dfrac{dy}{dt} = -ay + bu~~~~~~~(10)
$$

参考模型
$$
\dfrac{dy_{m}}{dt} = -a_{m}y_{m} + b_{m}u_{c}~~~~~~~(11)
$$
控制器
$$
u = \theta_{1}u_{c} + \theta_{2}y~~~~~~~(12)
$$

接下来我们引入误差变量
$$
e= y- y_{m}~~~~~~~(13)
$$


我们希望减小误差变量, 因此我们需要对上述误差量进行求导
$$
\dfrac{de}{dt} = -a_{m}e - (b\theta_{2} + a - a_{m})y + (b\theta_{1} - b_{m})u_{c}~~~~~~~(14)
$$
对于模型参考自适应控制, 我们需要根据当前误差$e$实时的去修改参数$\theta_{1}$和$\theta_{2}$ , 我们假设$b\gamma > 0$ 并且引入下面二次函数作为Lyapunov函数
$$
V(e, \theta_{1}, \theta_{2}) = \dfrac{1}{2}(e^{2} + \dfrac{1}{b \gamma}(b\theta_{2} + a - a_{m})^{2} + \dfrac{1}{b\gamma}(b\theta_{1} - b_{m})^{2})~~~~~~(15)
$$
 我们希望建立一种参数调整机制, 使得参数误差量$e$最终可以收敛到$0$. 要想满足这一要求Lyapunov函数的导数$\dfrac{dV}{dt}$ 需要是负数.
$$
\dfrac{dV}{dt} = e\dfrac{de}{dt} + \dfrac{1}{\gamma}(b\theta_{2}+a-a{m})\dfrac{d\theta_{2}}{dt} + \dfrac{1}{\gamma}(b\theta_{1} - b_{m})\dfrac{d\theta_{1}}{dt} \\
=-a_{m}e^{2} + \dfrac{1}{\gamma}(b\theta_{2} + a - a_{m})(\dfrac{d\theta_{2}}{dt} - \gamma ye) + \dfrac{1}{\gamma}(b\theta_{1} - b_{m})(\dfrac{d\theta_{1}}{dt}+\gamma u_{c}e)~~~~~~(16)
$$
如果我们采取如下参数更新的方式
$$
\dfrac{d\theta_{1}}{dt} = -\gamma u_{c}e~~~~~~(17)
$$

$$
\dfrac{d\theta_{2}}{dt} = \gamma y e~~~~~~(18)
$$

式(12)可以简化为
$$
\dfrac{dV}{dt} = -a_{m}e^{2}~~~~~~~(19)
$$


Lyapunov函数的一阶导数是负半定的而不是负定的

$V$的二阶倒数
$$
\dfrac{d^{2}V}{dt^{2}} = -2a_{m}e\dfrac{de}{dt} = -2a_{m}e(-a_{m}e - (b\theta_{2} + a - a_{m})y + (b\theta_{1} - b_{m})u_{c})~~~~~~(20)
$$

二阶导数$\ddot{V}$ 是有界限的, 因此误差$e$是可以收敛至0的. 

![Block_Diagram_MRAC_Lyapunov](/home/chenyongyu/Pictures/Block_Diagram_MRAC_Lyapunov.png)