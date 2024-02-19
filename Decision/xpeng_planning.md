# 小鹏规划算法文献分析

## Optimal Vehicle Trajectory Planning for Static Obstacle Avoidance using Nonlinear Optimization

使用非线性优化进行静态障碍物避让的最佳车辆轨迹规划

### 问题定义

#### A. Vehicle Kinematic  Model

运动学模型详细表示了在给定输入下车辆状态如何演化。我们假设线性化的jerk($\frac{da}{dt}$)，和曲率变化率$\dot \kappa = \frac{d\kappa}{dt}$为模型的控制变量。对给定的时间间隔$\Delta t$， 从状态$s_0$到$s_1$ 的转移通过以下公式，假设过程中$j$和$\dot \kappa$保持恒定：
$$
a_1 = a_0 + \Delta a(\Delta t) = a_0 + \int^{\Delta t}_0j(t)dt = a_0 + j\Delta t \tag1
$$

$$
v_1 = v_0 + \Delta v(\Delta t)=v_0+\int^{\Delta t}_0a(t)dt = v_0 + a_0\Delta t + \frac{1}{2}j\Delta t^2 \tag2
$$

$$
\kappa_1 = \kappa_0 + \Delta \kappa(\Delta t) = \kappa_0 + \int^{\Delta t}_0\dot \kappa(t)dt = \kappa_0 + \dot \kappa\Delta t \tag3
$$

泰勒展开
$$
\theta_1 = \theta_0 + \Delta \theta(\Delta t) = \theta_0 + \int^{\Delta s}_0 \dot \kappa(s)ds = \theta_0+\int^{\Delta t}_0 \kappa(s(t))s'(t)dt \\
=\theta_0 + \kappa_0 v_0 \Delta t + \frac{1}{2}(\kappa_0 a_0 + v_0\dot \kappa)\Delta t^2 + \frac{1}{3}(\frac{1}{2}\kappa_0j + a_0\dot\kappa)\Delta t^3 + \frac{1}{8}\dot\kappa j\Delta t^4 \tag4
$$

$$
x_1 = x_0 + \Delta x(\Delta t)= x_0 + \int^{\Delta s}_0 cos(\theta(s))ds = x_0 + \int^{\Delta t}_0 cos(\theta(s(t)))s'(t)dt \tag5
$$


$$
y_1 = y_0 + \Delta y(\Delta t)= y_0 + \int^{\Delta s}_0 sin(\theta(s))ds = x_0 + \int^{\Delta t}_0 sin(\theta(s(t)))s'(t)dt \tag6
$$
对$\Delta x$和$\Delta y$的处理：这种形式的积分通常没有闭解。为了解决这个问题，我们使用Gaussian-Legendre quadrature(高斯-勒让德正交)来近似积分。然后将积分问题转化为给定高斯节点上目标函数值的加权求和:
$$
\Delta x(\Delta t) = \int^{\Delta t}_0 cos(\theta(s(t)))s'(t)dt \approx \frac{1}{2}\Delta t\sum^N_{i=1}w_i cos(\theta(s(\frac{1}{2} \Delta t \xi_i + \frac{1}{2}\Delta t))) s'(\frac{1}{2} \Delta t \xi_i + \frac{1}{2}\Delta t)
$$

$$
\Delta y(\Delta t) = \int^{\Delta t}_0 sin(\theta(s(t)))s'(t)dt \approx \frac{1}{2}\Delta t\sum^N_{i=1}w_i sin(\theta(s(\frac{1}{2} \Delta t \xi_i + \frac{1}{2}\Delta t))) s'(\frac{1}{2} \Delta t \xi_i + \frac{1}{2}\Delta t)
$$

N是高斯-勒让德正交的阶数它决定了近似能达到的精度。$\xi$和$w_i$是相应的权重，这里取N=10

#### B. Trajectory Planning Problem Input

- 初始状态 $s_0$
- 车辆的几何参数，包括长度、宽度、轴距
- 车辆的动力学边界，包括最大转向角 $\alpha_{max}$，最大转角率$\dot\alpha_{max}$,加速度和jerk边界
- 车辆动力学模型
- 参考线$l_{ref}$，一般是二维的道路的中心线
- 可行使的走廊，如下图，它由道路边界和周围的静态障碍物组成的两条定向折线组成。
- 目标车速

![xpeng_figure3](E:\PnC\pnc_research\elements\xpeng_figure3.PNG)

## Optimization Formulation

#### B. Objective Formulation

车辆轨迹的最优性评价包括以下成本因素，目标函数是这些成本的加权总和:

- 向心加速度：这个因素会抵消向心加速度，因此自动驾驶汽车需要在弯道上减速或在弯道上直行。$f_{centri_acc} = \sum (v^2_i\kappa_i)^2$
- Centripetal jerk: 这一因素抑制了车辆在高速行驶时方向盘的突变。$f_{centri_jerk} = \sum(2v_i a_i \kappa_i + v^2_i\dot\kappa_i)^2 $
- 曲率变化率：$f_\dot\kappa = \sum\dot\kappa^2_i$
- Linear jerk: $f_{linear_jerk} = \sum j^2_i$
- 到参考线的距离：$f_{lat_distance}=\sum L_{\delta^l}(d((x_i,y_i),P_{(x,y)}(x_i,y_i,l_{ref})))$

- 目标速度：$f_{\Delta v}= \sum L_{\delta^v}(v_i-v_{target})$

$L_\delta$表示huber loss函数，广泛用于robust regression. 我们使用Huber损失来模拟到参考线的距离和接近目标速度的代价。与其他二次形式的成本因素相比，Huber损失函数具有这样的性质:如果输入在用户定义的δ范围内，则成本为二次形式，如果输入在用户定义的δ范围外，则成本为线性。

我们使用Huber损失来模拟到参考线的距离和接近目标速度的代价。与其他二次形式的成本因素相比，Huber损失函数具有这样的性质:如果输入在用户定义的δ范围内，则成本为二次形式，如果输入在用户定义的δ范围外，则成本为线性。这样就避免了当出现异常输入时，潜在的成本因素对整体目标函数的支配作用。以距离基准线的代价项为例，当自动驾驶车辆发起变道时，一种自然的方法是将基准线从自我车道(自我车辆所在的车道)切换到目标车道。如果采用成本的二次形式，在变道开始时，该成本因子的数值会变得过大，从而导致其他成本因子基本上失效，并导致剧烈的横向运动。同样的想法也适用于在自我车辆从静止状态开始时接近目标速度因子，以避免剧烈的纵向运动。



