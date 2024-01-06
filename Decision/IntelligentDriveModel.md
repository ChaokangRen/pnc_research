# Intelligent Driver Model

## Elementary Car-Following Models
 
distance headway = (distance) gap + length of the leading vehicle,  
(time) headway = time gap + occupancy time interval of the leading vehicle.

### Mathematical Description
![car-following-models](/Decision/elements/car-following_models.png "car-following")  
定义：  
$x_{\alpha}(t)$: 理解为在道路上的道路起点的相对位置  
$v_{\alpha}(t)$: 车速  
$l_{\alpha}$: 车长  
另外一些其他变量如加速度$\dot v = dv/dt$，以及刹车灯或者指示灯的亮灭。从车辆的位置和长度，可以得到车辆的间隙距离：  
$$
\begin{equation}
s_{\alpha} = x_{\alpha-1}-l_{\alpha-1} - x_{\alpha} = x_l - l_l - x_{\alpha}
\end{equation}
$$  
为了简便，有时会将$\alpha-1$写成$l$，后续章节会将间隙$s_{alpha}$表示为自车车速，前车位置和前车车速的函数。在连续时间模型中，驾驶员的响应直接以加速度函数$a_{mic}(s,v,v_l)给出，从而产生一组以下形式的耦合常微分方程:  
$$
\begin{equation}
\dot x_{\alpha}(t) = \frac{{\rm d}x_{\alpha}(t)}{{\rm d}t} = v_{\alpha}(t)
\end{equation}
$$
$$
\begin{equation}
\dot v_{\alpha}(t) = \frac{{\rm d}v_{\alpha}(t)}{{\rm d}t} = a_{mic}(s_{\alpha},v_{\alpha},v_l) = \tilde a_{mic}(s_{\alpha},v_{\alpha},\Delta v_{\alpha})
\end{equation}
$$ 
在大多数加速度函数中，大多数前车的车速仅以速度差的形式输入，即：
$$
\begin{equation}
\Delta v_{alpha} = v_{\alpha} - v_{\alpha -1}= v_{\alpha}  - v_l
\end{equation} 
$$
所以可以推导出：
$$
\begin{equation}
\tilde a_{mic}(s_{\alpha},v_{\alpha},\Delta v_{\alpha}) = a_{mic}(s_{\alpha},v_{\alpha},v_{\alpha}-\Delta v) 
\end{equation}
$$
式(1)也可以重新写为：
$$
\begin{equation}
\dot s_{alpha}(t) = \frac{{\rm d}s_{\alpha}(t)}{{\rm d}t} = v_l(t) - v_{\alpha}(t) = -\Delta v_{\alpha}(t)
\end{equation} 
$$
除此之外，还有离散时间跟车模型，其中时间不是建模为连续变量，而是离散为有限且通常恒定的时间步长，获得一般形式的迭代耦合图，而不是微分方程
$$
\begin{equation}
v_{\alpha}(t + \Delta t) = v_{mic}(s_{\alpha}(t),v_{\alpha}(t),v_l(t))
\end{equation} 
$$
$$
\begin{equation}
x_{\alpha}(t + \Delta t) = x_{\alpha}(t) + \frac{v_{\alpha}(t) +v_{\alpha}(t +\Delta t)}{2}\Delta t
\end{equation} 
$$
假设更新时间步长Δt恒定，一种简单但有效的显式更新方法是，在每一个时间周期内，给定加速度恒定的假设。即：

$$
\begin{equation}
v_{\alpha}(t+\Delta t) = v_{\alpha}(t) + a_{mic}(s_{\alpha}(t),v_{\alpha}(t),v_l(t)) \Delta t
\end{equation} 
$$
$$
\begin{equation}
x_{\alpha}(t+\Delta t) = x_{\alpha}(t) + \frac{v_{\alpha}(t) + v_{\alpha}(t + \Delta t)}{2}\Delta t
\end{equation} 
$$
如果设置以下方式，那么带ballistic更新方法的连续时间模型在数学上就等价与离散时间模型：
$$
\begin{equation}
a_{mic}(s,v,v_l)=\frac{v_{mic}(s,v,v_l)-v}{\Delta t}
\end{equation} 
$$

### Steady State Equilibrium and the Fundamental Diagram
状态平衡的意思是在真实道路环境的一种平衡，从技术上讲，这意味着所有驾驶员和车辆的模型参数都是相同的，即表征相应模型的加速度或速度函数不依赖于车辆内在参数。从建模的角度来看，稳态平衡具有以下两个条件：
* Homogeneous traffic:所有的车辆处于相同的车速($v_{\alpha}=v$),并且与前车保持相同的间隙(s_{\alpha}=s)
* No accelerations:$\dot v_{\alpha} = 0 or v_{\alpha}(t +\Delta t) = v_{\alpha}(t)$
对于具有$a_{mic}$或$\tilde a_{mic}(s,v,0)$形式的加速函数的时间连续模型，这意味着:  
$$
\begin{equation}
a_{mic}(s,v,v) = 0 \,\, or \,\, \tilde a_{mic}(s,v,0) = 0
\end{equation} 
$$
同时，条件
$$
\begin{equation}
v_{mic}(s,v,v) = v
\end{equation} 
$$
对式(7)的离散模型是有效的。根据模型的不同，微观稳态关系 (12) 或 (13) 可以求解
* 均衡速度$v_e(s)$,是对间隙的函数
* 均衡间隙$s_e(v)$,在给定速度下  

**Microscopic fundamental diagram**  
方程（12）和（13）允许可能的稳态的一维流形，可以用距离间隙s参数化，并用平衡速度函数$v_e(s)$描述，也称为微观基本图。
**Transition to macroscopic relations**为了获得距离间隙s和密度ρ之间的微观-宏观关系，我们直接定义密度为每条道路长度的车辆数量。对于一个给定的车长$l$,得到：
$$
\begin{equation}
s_{\alpha} = s = \frac{1}{\rho}-l
\end{equation} 
$$
此外，稳态平衡意味着所有车辆的速度相同并且等于宏观速度。
$$
\begin{equation}
V(x,t) = <v_{\alpha}(t)>=v_e(s)
\end{equation} 
$$
根据这些关系，我们可以推导出宏观稳态速度-密度图和宏观基本图:  
$$
\begin{equation}
V_e(\rho)=v_e(\frac{1}{\rho}-l),\,\,Q_e(\rho)=\rho v_e(\frac{1}{\rho} - l)
\end{equation} 
$$










---
## Car-Following Models Based on Driving Strategies
### 1.Model Criteria
与最小模型相比，编码驾驶行为的加速度或速度函数至少应该对以下方面进行建模:  
1. 加速度是速度的严格递减函数，而且，在没有其他车辆或者障碍物约束的情况下，车辆的加速是让车速趋近于期望速度的：  
$$
\begin{equation}
\frac{\partial a_{mic}(s,v,v_l)}{\partial v} < 0,
\lim_{s\rightarrow\infty}a_{mic}(s,v_0,v_l)=0 
\end{equation}
$$  
2. 加速度是对前车的距离s的递增函数：  

$$
\begin{equation}
\frac{\partial a_{mic}(s,v,v_l)}{\partial s} \geq 0,
\lim_{s\rightarrow\infty}a_{mic}(s,v_0,v_l)=0 
\end{equation}
$$
    成为等式的条件是其他车辆或者障碍物对自车不产生任何影响，定义为：  
$$
\begin{equation}
a_{free}(v)= \lim_{s\rightarrow \infty} a_{mic}(s,v,v_l)=\geq a_{mic}(s,v,v_l)
\end{equation}
$$
3. 加速度是前车速度的增函数。结合要求（1），这也意味着加速度随着接近前车（或障碍物）的速度而减小（减速度增大）：

$$
\begin{equation}
\frac{\partial \tilde a_{mic}(s,v,\Delta v)}{\partial \Delta v} \leq 0 or 
\frac{\partial \tilde a_{mic}(s,v,v_l)}{\partial \Delta v_l} \geq 0,
\frac{\lim_{s\rightarrow\infty}a_{mic}(s,v_0,v_l)}{\partial v_l}=0 

\end{equation}
$$

4. 于前车保持最小间隙，(保险杠到保险杠的距离)$s_0$(在静止时间也是如此)，如果由过去的事件导致间隙小于$s_0$，也不会重新回到$s_0$:
$$
\begin{equation}
a_{mic}(s,0,v_l) = 0 {\,\,}for {\,\,} all {\,\,}v_l\geq 0,s\leq s_0
\end{equation}
$$
满足这些要求的跟车模型是完整的，因为它可以一致地描述单车道交通中可能出现的所有情况。特别是，(i)所有车辆相互作用都是有限范围的,(ii)后面的车辆不会被“拖拽”，有:
$$
\begin{equation}
a_{mic}(s,v,v_l) \leq a_mic(\infty,v,v'_l) = a_{free}(v) \,\,\, for \,\,all\,\,\, s,v,v_l,and\, v'_l
\end{equation}
$$
(iii)均衡速度是存在的，它具有已经为最佳速度函数假设的属性  
$$
\begin{equation}
v'_e(s)\geq 0,\,\,\, v_e(0) = 0,\,\,\, \lim_{s\rightarrow \infty}v_e(s) = v_0
\end{equation}
$$
### Gipps' Model
#### Safe Speed
引入一个安全速度的概念，$v_{safe}(s,v_l)$,基于以下假设：
1. 制动机动总是以恒定的减速度b执行
2. 存在一个恒定的反应时间
3. 即使前车突然减速，甚至到完全停止下来，与前车的间隙也会不会小于一个最小值$s_0$  

通过条件一，可以计算出前车的刹车距离：$\Delta x_l = \frac{v^2_l}{{2b}}$
通过条件二，计算带反应时间的自车刹车距离，$\Delta x = v\Delta t +  \frac{v^2}{{2b}}$
通过条件三，计算出与前车的间距：
$$
\begin{equation}
s \geq s_0 + v\Delta t +  \frac{v^2}{{2b}} - \frac{v^2_l}{{2b}}
\end{equation}
$$
由此可以计算出安全速度为：
$$
\begin{equation}
v_{safe}(s,v_l) = -b * \Delta t + \sqrt {b^2\Delta t^2 + v^2_l + 2b(s - s_0)}
\end{equation}
$$

#### Model Equation
The simplified Gipp's model is defined as an iterated map with the 'safe speed' as ite main component:
$$
\begin{equation}
v(t + \Delta t) = min[v + a\Delta t,v_0,v_{safe}(s,v_l)] \,\,\,\, Gipps' model
\end{equation}
$$

该模型反应了如下特性：
* 仿真时间和反应时间一致
* 如果当前车速比 $v_{safe}+a\Delta t$或者 $ v_0 - a \Delta t$ 大，那么车速会在下一次迭代中变为 $v_0$ 或 $v_{safe}$ 中的最小值。
* 否则车辆就会以恒定的加速度a加速到安全速度或者期望速度。

#### Steady-State Equilibrium
the homogeneous(同质化) steady state implies $v(t+\Delta t) = v_l = v$,thus:  
$$
\begin{equation}
v = min(v_0,v_safe) = min(v_0,-b\Delta t + \sqrt{b^2\Delta t^2 +v^2 + 2b(s-s_0)})
\end{equation}
$$

其中，生成稳定状态的车速间隙关系式为：
$$
\begin{equation}
v_e(s)=max(0,min(v_0,\frac{s-s_0}{\Delta t}))
\end{equation}
$$

假设车长为$l$,the familiar "triangular" fundamental diagram:
$$
\begin{equation}
Q_e(\rho) = min(v_0\rho,\frac{1-\rho l_{eff}}{\Delta t})
\end{equation}
$$
其中，$l_{eff}=(l+s_0)$

### Intelligent Driver Model
#### Required Model Properties
IDM模型需要以下基础的假设：
1. 加速度满足Model Criteria 提到的完整模型的一般条件的五个公式。
2. 与前车的平衡保险杠到保险杠距离不小于“安全距离”$s_0 +vT$其中$s_0$是最小（保险杠到保险杠）间隙，T 是（保险杠到保险杠）与领先车辆的时间差距。
3. 刹车策略，智能控制车辆如何在障碍物或者红绿灯前停下来。在正常条件下，刹车方式是舒适的，加速度为舒适值b,并在达到稳态跟车情况或完全停止之前平稳地降至零。在危急情况下，减速度超过舒适值直至避免危险，剩余的制动操作（如果适用）将以常规舒适减速继续进行。
4. 不同驾驶模式之间的转换是smooth的，或者说jerk是在舒适范围内的。等价于说加速度函数 $a_{mic}(s,v,v_l)$(或者 $\tilde a_{mic}(s,v,\Delta v))$中的三个变量都是可微分的。
5. 该模型应该尽可能简洁。 每个模型参数应该只描述驾驶行为的一个方面(这有利于模型校准)。此外，参数应符合直观的解释并假设合理的值。

#### Mathematical Description
上面要求的特性可有下面加速度公式实现：
$$
\begin{equation}
\dot v = a\left[ 1 - \left(\frac{v}{v_0}\right)^ \delta - \left(\frac{s^*(v,\Delta v)}{s}\right)^2 \right]
\end{equation}
$$

$$
\begin{equation}
s^*(v,\Delta v) = s_0 +max\left(0,vT+\frac{v\Delta v}{2\sqrt{ab}}\right)
\end{equation}
$$
动态项$\frac{v\Delta v}{2\sqrt(ab)}$是智能刹车策略。  

#### Parameters
从上述公式可以看出：
* 当车辆从静止开始加速时，加速度最大值为a,并且加速度随着速度的增大了而减小，直至速度为 $v_0$时，加速度为0，参数 $\delta$作用如下：其值越大，接近期望速度时加速度减小得越晚,范围在 $(1,\infty)$
* 跟车时，跟车间隙为安全距离 $(s_0,vT)$,当接近较慢或停止的车辆时，减速度通常不超过舒适减速度 b． 在这些情况之间的转换期间，加速功能是平稳的。
* 当接近较慢或停止的车辆时，减速度通常不超过舒适减速度 b． 在这些情况之间的转换期间，加速功能是平稳的。
由于 IDM 没有明确的反应时间，并且其驾驶行为是根据连续可微的加速度函数给出的，因此 IDM 比人类驾驶员更接近地描述了自适应巡航控制 (ACC) 的半自动驾驶的特性。 然而，它可以轻松扩展以捕捉人为因素，例如估计错误、反应时间或观察前方的几辆车。
![IDM-models](/Decision/elements/IDM_model.png "idm") 
与之前讨论的模型相比，IDM 明确区分了安全时间间隙T、速度适应时间 $\tau = v_0/a$ 和反应时间 $T_r$,如下表，这使我们不仅能够在模型中反映 ACC 和人类驾驶员之间的概念差异，还能区分更细微的驾驶风格，例如“缓慢但追尾”(高的$\tau = v_0/a$值，低的T值)，或“敏捷而安全的驾驶”(低的$\tau$值，正常的T值)。此外，所有这些驾驶风格都可以被 ACC 系统（反应时间 Tr≈0，原始 IDM）、细心的驾驶员（Tr相对较小，扩展 IDM）和昏昏欲睡的驾驶员（Tr相对较大，扩展 IDM）独立采用.
![table](/Decision/elements/table1.png "table1")  

#### Intelligent Braking Strategy
IDM模型中所需的距离 $s*$中的项 $v\Delta v /(2ab)$是自车接近前车时候的动态行为。由于需要从平衡状态到平衡状态的连续转换，平衡项 $s_0 + vT$ 始终影响 $s^*$ 为了研究制动策略本身，我们将这些项以及IDM加速方程的自由加速项 $a[1-(v/v_0)^\delta ]$ 设置为零。当接近一个静止车辆或者红灯时，会得到:

$$
\begin{equation}
\dot v = -a\left( \frac{s^*}{s}\right)^2 = - \frac{av^2(\Delta v)^2}{4abs^2} = - \left( \frac{v^2}{2s}\right)^2 \frac{1}{b}
\end{equation}
$$
运动减速度定义为：
$$
\begin{equation}
b_{kin}=\frac{v^2}{2s}
\end{equation}
$$
于是加速度公式可以写为：
$$
\begin{equation}
\dot v = - \frac{b^2_{kin}}{b}
\end{equation}
$$
当以减速度 $b_{kin}$ 制动时，制动距离恰好是到前车的距离，因此 $b_{kin}$ 是防止碰撞所需的最小减速度。所以IDM的自调节制动策略为：  
* “危急情况”的定义是 $b_{kin}$ 大于舒适减速度 b.在这种情况下，实际的减速甚至比必要的还要强，$|\dot v| = b^2_{kin}/b > b_{kin}$,这种过度补偿会减少 $b_{kin}$ 从而有助于“重新控制”情况.
* 在非紧急情况中 $(b_{kin} < b)$,实际的减速度小于运动学减速度， $b_{kin}$随着时间的推移而增加并接近舒适减速度。它的微分公式为：
$$
\begin{equation}
\frac{{\rm d}b_{kin}}{{\rm d}t} = \frac{vb_{kin}}{s b} (b - b_{kin})
\end{equation}
$$

#### Steady-State Equilibrium
通过假设 $\dot v = \Delta v = 0$ 从加速度函数中可以得到稳定均衡状态下的IMD模型：  
$$
\begin{equation}
1 - \left( \frac{v}{v_0} \right)^\delta - \left( \frac{s_0 +vT}{s}\right)^2 = 0
\end{equation}
$$
那么有：
$$
\begin{equation}
s = s_e(v) = \frac{s_0 + vT}{\sqrt{1 - \left(\frac{v}{v_0}\right)^\delta}}
\end{equation}
$$

### Improved Acceleration Function

IDM模型在以下几个方面仍存在一定缺陷：  
* 如果实际速度超过了期望速度(比如说进入了一个限速更低的区域)，减速度异常的大，特别是对于加速度指数 $\delta $的大值。
* 接近期望速度 $v_0$时，稳态间隙变得远大于 $s^*(v,0) = s_0,vT$ ，因此模型参数T失去了其作为所需时间间隙的意义。这意味着一排相同的驾驶员和车辆的分散程度比观察到的要多得多。此外，并非所有汽车都能达到所需的速度。
* 如果实际间隙远小于期望间隙（如果变道时另一辆车切得太近，可能会发生这种情况，则重新获得所需间隙的制动反应会被放大。  

为了改善 $v > v_0$ 的行为，我们要求在没有与其他车辆或障碍物相互作用的情况下，最大减速度不得超过舒适减速度b。参数 $\delta$ 在新的机制中也应保留其含义，即，对于低值，导致平滑减速到新的期望速度，对于高值，更加“机械(robotically)”地减速。进一步的，自由加速度函数 $a_{free}(v)$应该是微分连续的，并且在 $v \leq v_0$时保持不变，于是有 $a_{free}(v)= \lim_{s\rightarrow \infty}(s,v,\Delta v),\,\,\,for\,\, v \leq v_0 $,可能满足这些条件的最简单的自由加速度函数是:
$$
\begin{equation}
a_{free}(v) = 
\left\{
\begin{aligned}
a\left[1 - \left(\frac{v}{v_0}\right)^\delta \right] \,\,\, if\,\,v \leq v_0\\
-b\left[1 - \left(\frac{v_0}{v}\right)^{a\delta/b }\right] \,\,\, if\,\,v > v_0
\end{aligned}
\right.
\end{equation}
$$
为了改善接近期望速度时的行为，我们收紧了Required Model Properties第二个条件，要求均衡间隙 $s_e(v) = s^*(v,0)$ 在 $v < v_0$时，应该严格等于 $s_0 +vT$.然而，我们希望尽可能保守地实施任何修改，以保留 IDM 的所有其他有意义的特性(尤其是“智能”制动策略)。因此，更改只会影响到：
* 接近稳态均衡时，如果 $z(s,v,\Delta v) = s^*(v,\Delta v)/s \approx 1$
* 当车速为 $v \approx v_0,and\,\, v>v_0$
我们可以通过区分 $z = s^*(v,\Delta v)/s < 1$ (实际间隙大于所需间隙)和 $ z \geq 1$ 的情况来实现这一点。对于满足 $ z = s^*(v,\Delta v)/s < 1 $ 和 $v < v_0$ 的所有输入值，新条件要求 $\tilde a_{mic} = 0.满足 $v < v_0$ 的所有这些条件的最简单的加速函数可能由下式给出:
$$
\begin{equation}
\frac{dv}{dt}|_{v\leq v_0}
\left\{
\begin{aligned}
a(1-z^2) \,\,\,\,\,\,\, z = \frac{s^*(v,\Delta v)}{s} \geq 1 \\
a_{free}(1 - z^{(2a)/a_{free}}) \,\,\,\, otherwise
\end{aligned}
\right.
\end{equation}
$$
对于$v > v_0$,并没有稳定的跟随距离，我们简单地将自由加速度 $a_{free}$ 和相互作用加速度 $a(1-z^2)$ 结合起来，使得相互作用在 $z \leq 1$ 时消失，并且所得的加速度函数连续可微：
$$
\begin{equation}
\frac{dv}{dt}|_{v > v_0}
\left\{
\begin{aligned}
a_{free} + a(1-z^2) \,\,\,\,\,\,\,z(v,\Delta v) \leq 1 \\
a_{free} \,\,\,\, otherwise
\end{aligned}
\right.
\end{equation}
$$
这种改进的智能驾驶员模型 (IIDM) 使用与IDM相同的模型参数集，并产生基本相同的行为，除非车辆在接近所需速度时相互跟随或当车辆快于所需速度时。

### Model for Adaptive Cruise Control
当ACC系统只需要自动控制纵向驾驶任务时，如果传感器的输入变量(间隙s和接近的速度变化率$\delta v),由于一些被动的原因比如旁边车道的车辆插入到自车前方成为leader,或者一些主动的原因比如驾驶员手动改变了车道，系统需要对这些变化做出积极回应。这意味着，必须注意前面中提到的IDM缺陷的第三个——当间隙因外部行为不连续减小时的过度反应。  
反应过度的原因是 IDM（和 IIDM）开发的目的是即使在最坏的情况下也不会发生事故，即领先车辆的驾驶员突然制动至完全静止。然而，在低速或者小间隙的场景情况下，人类驾驶员会依据这个事实，即前车不会突然的全力紧急制动。事实上，他们认为这种情况只是轻微危急。因此，当驾驶员根据恒定加速启发式 (CAH) 采取行动而不是考虑最坏情况时，将会产生更合理、更现实的驾驶行为。CAH基于以下假设：
* 所考虑的领先车辆的加速度在不久的将来（通常是几秒钟）不会改变。
* 在任何给定时刻都不需要安全时间间隙或最小距离。
* 驾驶员（或 ACC 系统）毫不延迟地做出反应，即反应时间为零
对于间隙s、速度v、前车速度$v_l$以及两车恒定加速度$\dot v$和 \dot v_l$的实际值，最大加速度 max(v̇) = CAH 假设下不会导致事故的CAH计算方式如下:
$$
\begin{equation}
a_{CAH}(s,v,v_l,\dot v_l) = 
\begin{cases}
\frac{v^2\tilde a_l}{v^2_l - 2s\tilde a_l} \,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,if\,v_l(v-v_l)\leq -2s\tilde a_l \\
\tilde a_l - \frac{(v-v_l)^2 \Theta(v-v_l)}{2s} \,\,\,\,otherwise
\end{cases}
\end{equation}
$$
effective acceleration $\tilde a_l(\dot v_l) = min (\dot v_l,a)$避免了加速能力较高的领先车辆可能造成形式“拖拽效应”的情况
对于条件 $v_l(v - v_l) = v_l \Delta v \leq -2s\dot v_l$，如果车辆在达到最小间隙s=0前停止下来，则条件为真。Heaviside 阶跃函数$\Theta(x)$（如果 x ≥ 0，则$\Theta(x) = 1$，否则为零）消除了两辆车在最后距离在时间$t^*$下存在的消极的接近速度率 $\Delta v$。otherwise $t^*$ would lie in the past.  
为了保持IDM模型的良好性质，我们将仅使用CAH加速度（11.25）作为判断IDM是否会导致不切实际的高减速度的指标，并且仅在这种情况下修改ACC车辆模型的加速度函数。具体来说，提出的ACC模型是基于以下假设：
* ACC加速度永远不会低于IIDM，受环境推动，IDM和IIDM是无事故的，充满防御性的。
* 如果IIDM和CAH生成了相同的加速度，那ACC加速度也是一样。
* 如果IIDM生成了更大的减速度，同时CAH的加速度处于舒适区间，这种情况下，应该考虑中间状态，最终输出的加速度是处于 $a_{CAH}-b$和$a_{CAH}$中的。仅对于非常小的间隙，减速度应该稍高一些，以避免过于鲁莽的驾驶风格。
* 如果二者的减速度都小于-b,出现这种表明情况紧急，应该选择减速度更大的那个保证刹停。
* ACC 加速度应该是 IIDM 和 CAH 加速度的连续且可微的函数。 此外，还应满足一致性要求。
满足这些标准的最简单的函数形式可能是由下式给出：
$$
\begin{equation}
a_{ACC} = 
\begin{cases}
a_{IIDM}\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,a_{IIDM}\leq a_{CAH} \\
(1-c)a_{IIDM} +c\left[a_{CAH}+b tanh\left(\frac{a_{IIDM}-a_{CAH}}{b}\right)\right]\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,\,otherwise
\end{cases}
\end{equation}
$$
ACC模型仅有一个额外的参数，the "coolness factor"c.c=0表示纯IIDM模型，c = 1表示纯的ACC模型，因为纯的ACC模型可能会在一个非常小的间隙给出一个非常粗鲁的驾驶性为，可以将IIDM的部分影响加入其中。事实证明，1% 的贡献（对应于 c = 0.99）在这种情况下在鲁莽和过于胆怯的行为之间提供了良好的折衷，而在其他情况下它基本上是无关紧要的。
