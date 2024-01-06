# 基于POMDP的决策规划方案调研
## 论文1：MPDM: Multipolicy Decision-Making in Dynamic,Uncertain Environments for Autonomous Driving
### 概述
这篇文献提出了一个 Multipolicy Decision-Making(MPDM)，利用自动驾驶领域的知识为自动驾驶车辆在交通中行驶做出在线决策的一种决策算法。
#### Problem Statement
该问题本质是一个POMDP问题，在一个动态的，不确定的环境下，多个agents行为紧密耦合的决策问题。

定义： 
$v \in V $:表示在局部环境下的N辆产生交互的车  
动作： $ a_t^v \in \Alpha $  
状态： $ x_t^v \in \Chi $  
条件状态转移函数： $ T(x_t,a_t,x_{t+1})= p(x_{t+1}|x_t,a_t) $  
观测不确定性建模：$Z(x_t,z^v_t)=P(z^v_t|x_t)$
其他agent在当前观测到的环境的的行为建模： $ D(x_t,z^v_t,a^v_t) = p(a_t^v|z^v_t,x_t)$

对于该决策的核心问题是为自车求解出最优策略 $ \pi^* $，一种映射关系：由当前状态和观测生成的策略，即$\pi^v : x_t \times z^v_t \rightarrow a^v_t $  
那么问题可以描述为:在给定时域H下,
$$
\begin{equation}
\pi^* = \mathop{\arg\min}\limits_{\pi}
\sum_{t=0}^H \gamma^t \int_{-x_t} R(x_t)p(x_t)\, {\rm d}x_t
\end{equation}
$$
其中，$\gamma^t$表示折扣因子，$R(x_t)$表示reward function
联合概率密度函数$p(x_t)$表示为：  
$$
\begin{equation}
p(x_{t+1})= \mathop{\iiint}\limits_{x_t z_t a_t}p(x_{t+1},x_t,a_t,z_t){\rm d}a_t {\rm d}z_t {\rm d}x_t
\end{equation}
$$
并使用上面的状态转换、观察和驾驶员行为模型递归分解得到
$$
\begin{equation}
p(x_{t+1})= \mathop{\iiint}\limits_{x_t z_t a_t}
p(x_{t+1}|x_t,a_t)p(a_t|z_t,x_t)p(z_t|x_t)p(x_t)
{\rm d}a_t {\rm d}z_t {\rm d}x_t
\end{equation}
$$  
在这里要解决的就是如何公式化出来$p(x_{t+1}|x_t,a_t),p(a_t|z_t,x_t)p(z_t|x_t),p(x_t)$这三个概率函数。  
假设：在多车交互过程中，每辆车的瞬时动作$a^v_t$是独立的，与他车无关的，只与此刻的观测$z_t$和状态$x_t$相关，这样下来，我们考虑单个agent v的joint density:
$$
\begin{equation}
p^v(x^v_t,x^v_{t+1},z^v_t,a^v_t)=
p(x^v_{t+1}|x^v_t,a^v_t)p(a^v_t|z^v_t,x^v_t)p(z^v_t|x^v_t)p(x^v_t)
\end{equation}
$$ 
由于独立性假设，那么
$$
\begin{equation}
p(x_{t+1})=
\prod_{v \in V_{x_t}}
p(x^v_{t+1}|x^v_t,a^v_t)p(a^v_t|z^v_t,x^v_t)p(z^v_t|x^v_t)p(x^v_t)
{\rm d}a^v_t {\rm d}z^v_t {\rm d}x^v_t
\end{equation}
$$ 
上述公式仍然存在大量的不确定性，导致计算量巨复杂，因此需要对部分公式进行近似。
#### Approximate Decision Process
(1) 为自车和他车的动作，在一组有限离散已知的集合中，选择策略。  
(2) 根据策略，对自车和他车，通过一个确定的、闭环前向仿真近似车辆动力学和观测
这样，问题变转化为找到一组高级的车辆行为，即一组最优策略。  
我们令$\Pi$为一组精心构建的策略集合，其中每一个策略代表一个特定的高级行为，比如车道保持，做出变道等等。因此，公式4可以重写为：
$$
\begin{equation}
p^v(x^v_t,x^v_{t+1},z^v_t,a^v_t,\pi^v_t)=
p(x^v_{t+1}|x^v_t,a^v_t)p(a^v_t|z^v_t,x^v_t,\pi^v_t)p(\pi^v_t|x^v_t)p(z^v_t|x^v_t)p(x^v_t)
\end{equation}
$$  
与公式(4)相比，附加项$p(\pi^v_t|x^v_t)$为该车辆选择了给定策略的概率。在本文中，我们假设其他车辆的最似然的策略$\pi^v_t$可以利用路网模型来给出，这样就可以将问题聚焦到我们自车的控制上来。通过公式(6),最终将其他车辆$v\in V$以及在我们控制下的车辆$q \in V$分离开来。
$$
\begin{equation}
p(x_{t+1})= \mathop{\iint}\limits_{x_q z_q}
p^q(x^q_t,x^q_{t+1},z^q_t,a^q_t,\pi^q_t){\rm d}z^q_t {\rm d}x^q_t
\prod_{v \in V| v \neq q}
\{\sum_{\Pi}
\mathop{\iint}\limits_{x_v z_v}
p^v(x^v_t,x^v_{t+1},z^v_t,a^v_t,\pi^v_t){\rm d}z^v_t {\rm d}x^v_t
\}
\end{equation}
$$
通过对闭环系统进行策略建模，我们可以推理得到近似的状态转移概率函数$p(x^v_{t+1}|x^v_t,a^v_t),采用的方法是确定的系统动力学仿真。  
对于每一个可应用的策略$\pi$,从初始状态$x_t$进行演化，即可得到序列化的全部状态$\Phi=(x_0,x_t,...,x_H),接着对序列$\Psi$,利用定义好的cost function进行打分。得到$C=(c_0,c_1,...C_n)$最终，最优策略就是集合$C$中得分最高的那个策略。

A. Policy Design  
策略的设计具体情况具体设计，不同场景下的策略也会不一样。对于一个简单的驾驶场景来说，策略可以表示为:  
* 车道跟车居中：沿当前车道与前车保持一定距离的行驶  
* 向左变道或向右变道
* 停车  

这套政策的构建主要取决于涵盖在给定道路网络中行驶和遵守交通规则所需的一系列行为。  
B. Multi-vehicle Simulation  
通过将正向仿真转换为闭环确定性系统，可以捕获车辆之间必要的交互，从而为自车行为做出合理的选择，我们为环境中每一个车选择一种可能的策略，然后通过状态转移将整个系统向前一步仿真。  
C. Policy Election
构建指标：  
* distance to goal
* lane choice bias
* max yaw rate
* simple policy cost
  
对于每一个指标$m_j$,赋予一个相应的权重$w_j$.该权重既根据用户需求编码了根据经验调整的指标重要性，也编码了指标在给定策略集中的信息量。 我们降低了政策之间变化太小的无信息指标的权重.

