# 模型参考自适应控制

模型参考自适应控制(model reference adaptive control, 简称 mrac) 可以视为跟随参考模型 (reference model)的自适应伺服系统(adaptive servo system) 。 更具体的说，对于相同的输入我们希望被控制系统的输出特性和参考系统的输出特性是一致的。

![MRAC_Structure](/home/ubuntu/MRAC_Structure.png)上图是模型参考自适应控制的基本框图，控制器的参数根据参考模型的输出和实际系统输出的误差实时变化。+-

## 基于Lyapunov稳定性设计MRAC

### Lyapunov稳定性理论

考虑非线性时变系统
$$
\dot{x} = f(x,t)~with~f(0, t)=0 
$$
假设初始$t=t_{0}$ , $||x_{0}|| = \delta$ , 如果初始状态不是平衡状态$x^{*} = 0$，那么后面会发生什么，可以 分为下面四种情况

1. 系统是稳定的。对于足够小的$\delta$ , $x$落在以$x^{*}$为圆心，以$\epsilon$为半径的圆内。如果 $\delta$的选取与初始时间$t_{0}$无关，那么系统被称为一致稳定(uniformly stable)。
2. 系统是不稳定的，对于任意的$\epsilon$, 
3. 系统是渐近稳定的，

### 基本步骤

1. 确定控制器结构
2. 推导误差方程
3. 找出Lyapunov方程
4. 推导满足Lyapunov理论的自适应变化率

### 一阶系统例子(first- order system)

系统模型
$$
\dfrac{dy}{dt} = -ay + ku
$$

参考模型
$$
\dfrac{dy_{m}}{dt} = -a_{m}y_{m} + b_{m}u_{c} 
$$
控制器
$$
u = \theta_{1}u_{c} + \theta_{2}y
$$
