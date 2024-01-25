# 论文3：Automated Driving in Uncertain Enviroments:Planning with Interaction and Uncertain Maneuver Prediction
如图 1 所示，我们定义了其他道路使用者未来轨迹不确定的四个关键原因。
* 其他驾驶员的意图未知
* 它们未来的纵向运动未知
* 与自车交互的可能性
* 传感器的噪音 
![fig1](E:/PnC/pnc_research/Decision/elements/BMW_fig1.png "fig1")    
这篇文章的主要工作就是为自动驾驶展示了一个在线的POMDP算法。
## Related Work
### Belief state planning
belief state规划表示一个规划问题，仅能得到当前状态的概率分布，而非状态本身。允许对不确定行为和不确定的动作建模，通过概率转移模型(probabilistic transition model).与本节中介绍的相关工作相比，我们的目标是提出一个结合多个方面的 POMDP 统一规划框架:
* 它通常适用于任何交叉口几何形状和可变数量的交通参与者
* 它考虑当前甚至预测其他交通参与者意图/运动的未来不确定性，并且不依赖（但可以从中获利）车辆对车辆（V2V）通信
* 它是在线且随时可用的。 给予更多的时间或计算能力，结果将会改善
* 运行在一个连续的状态空间中。

### Problem Statement
这项工作的重点是自我车辆的在线决策，即生成一系列期望的加速度 $a_0 = (a^{t_0}_0,a^{t_1}_0,a^{t_2}_0,...)$,用于穿越一个任意布局和未知其他交通参者的数量以及意图的无信号灯的路口。
假设自车$r_0$的参考路径对静态障碍物是无碰撞的，并且由路径规划器先验地生成或者简单地从给定地图的道路几何形状中检索。第二步，纵向速度是通过我们的算法沿着r0计算出的。这种做法在文献[33]中被称为路径速度分解，并将轨迹规划问题简化为一维工作空间。     
环境有一系列的agents组成， $\mathcal{N} = {N_0,...,N_K}$，自车下标为0.对于其他agent，每个agents假设都有一组参考路径。自车与其他车的路径假设有一个拓扑地图 $\mathcal{R}= {r^{(0)},r^{(1)},...,r^{(I)}}$,其中， $r^{(i)} = \left\{\overrightarrow{q_{i,0}q_{i,1}},...,\overrightarrow{q_{i,J-1}q_{i,J}}\right\}$,$q_{i,j}$表示路径$i$的第$j$个点的位置坐标。每个agent都有一个相应的路径映射，$r_k : N_k$ ,该agent的速度为 $v_k(t) \in [0,v_{max}]$.每条路径都有一组后续路径假设 $\mathcal{M}^{(i)} = succ(r^{(i)})$.agent从当前路径通过未知的概率 $P(r'_k|r_k)$ 转移到下一个路径 $r'_k$. 
<font color=red>
note:一辆车可能有多个潜在的路线，但最终执行的时候只会有一条</font>
由于各个路线元素可能相互交叉，因此交叉函数 $c(r_i,r_j)$ 定义为:
$$
\begin{equation}
c(r^{(i)},r^{(j)}) =
\left\{
\begin{aligned}
1,if\,\,r^{(i)}\cap r^{(j)} \neq \empty \\
0,\,\,\,otherwise
\end{aligned}
\right.
\end{equation}
$$
不同的路径是从道路网络中检索的，因此在下文中被称为路线。该路由定义的示例如图2所示
![fig2](E:/PnC/pnc_research/Decision/elements/BMW_fig2.png "fig2")    
考虑到其他汽车运动的不确定性，自动驾驶汽车必须不断选择最佳加速度a*，以最大化预期的累积贴现未来奖励：
$$
\begin{equation}
a^*_0 := \arg\max_{a_0} E \left[
\sum^{\infty}_{\tau = 0} \gamma^\tau R^{t_{\tau+1}}|a^{t_\tau}_0
\right]
\end{equation}
$$
奖励应该包括惩罚碰撞、总加速度（提供舒适度）以及与交通法和基于曲率的参考速度的偏差。
由于所使用的自适应置信树 (ABT) 算法对多个片段进行采样来近似解决方案，因此不需要明确指定 POMDP 的模型属性（例如概率分布），而是将其作为生成模型。

## APPROACH

本文主要聚焦于在考虑其他车辆未来移动轨迹不确定下，如何优化自车的纵向动作或者说加速度。另外我们的方法不依赖固定的交叉路口(理解为对任意路口都成立),并且假设自车的有效路径存在时，生成对其他agents的预测轨迹。通过对map结构的预处理，车辆的路径可以被编码在一维空间中。这允许将其他车辆的路线作为隐藏变量进行低维情况表示。因此上一节中的问题可以描述为一个POMDP问题。这使得该方法能够表示任意情况，并与最先进的基于点的逼近和适应工具包相结合-实时POMDP解决方案(TAPIR)(参见[34])，在连续状态空间中在线解决问题。其他交通参与者的运动和交互模型允许规划复杂的和交互的机动。此外，我们使用了一个随机观察模型，考虑了周围车辆的潜在未来测量值，以优化一组未来情景下自我车辆的加速度。

由于使用了自适应信念树(ABT)算法对多个事件进行采样来近似解，因此POMDP的模型属性(例如概率分布)不需要明确指定，而是作为生成模型。

正如作者之前在纵向规划方法中所提出的[11]，本文提供的算法在行为层(定义见[3])上以粗糙的方式解决运动规划问题，并将生成的可行子目标提供给轨迹规划层以顺利执行。

### A. Partially Observable Markov Decision Process
POMDP由 $\left<\mathcal{X,A,T,O,Z,R},b_0,\gamma\right>$组成，$x\in \mathcal{X}$ 表示可能的状态，$a\in A$表示可能的动作。$T(x',x,a) = P(x'|x,a)$为概率转移函数。R为奖励函数，$\gamma$为折扣因子。$Z(x',a,o) = P(o|x',a)$表示观测函数。
优化目标为：
$$
\begin{equation}
\pi^* := \mathop{\arg\max}_{\pi}\left(
E\left[
\sum^{\infty}_{\tau = 0} \gamma^{t_{tau}}
R(x^{t_{\tau}},\pi(b^{t_\tau})) | b^{t_0},\pi
\right]
\right)
\end{equation}
$$
### B. State Space

为了在运动模型中对交互行为进行建模，所有场景中的车辆都在状态空间中表示。一个确定的状态$x\in \mathcal{X}$是定义在连续空间
$$
x = (x_0,x_1,x_2,...,x_{\kappa})^T
$$
$x_0$表示自车状态，其他agent的状态为剩余状态。

车辆的位置由位置 $s_k$ 处车辆路线 $r_k$ 上的Frenet公式描述。虽然从车道匹配坐标系L到全局坐标系W，WTL的变换不是双射的，但 $s_k$ 仍然可以从全局坐标计算，因为粒子的路线 $r_k$ 是已知的并且是状态空间的一部分。
自车的状态定义如下：
$$
\begin{equation}
x_0 = \left(
\begin{aligned}
s_0 \\
v_0
\end{aligned}
\right)
\end{equation}
$$
他车的状态为：
$$
\begin{equation}
x_k = \left(
\begin{aligned}
s_k \\
v_k \\
r_k
\end{aligned}
\right)
\end{equation}
$$
其中， $r_k$为隐藏变量，无法被直接观测到，含义为车k可能选的的行驶路线。如下图所示     
![fig3](E:/PnC/pnc_research/Decision/elements/BMW_fig3.png "fig3") 

### C. Actions and Motion Model
他车的转移模型为：
$$
\begin{equation}

\left(
\begin{aligned}
s'_k \\ v'_k \\ r'_k
\end{aligned}
\right)
=
\left(
\begin{matrix}
1 & \Delta t & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{matrix}
\right)
\left(
\begin{aligned}
s_k \\ v_k \\ r_k
\end{aligned}
\right)
+
\left(
\begin{matrix}
\frac{1}{2} (\Delta t)^2 \\ \Delta t \\0
\end{matrix}
\right)
a_k,\, k \in \{1,...,k\}
\end{equation}
$$
如之前提到的，假设代表车辆任何粒子的路径都是恒定的，即$r'_k = r_k$。加速度$a_k$定义为跟随参考速度 $a_{ref}$ 的加速度和基于交互的加速度 $a_{int}$ 的总和：
$$
\begin{equation}
a_{int,k} = 
\begin{cases}
0, &\text{if}\,\,c(r_k,r_0) = 0, \\
-1.5,&\text{if} \,\,c(r_k,r_0) = 1 \land (t_{c,k} - t_{c,0}) \in [1,5]
\end{cases}
\end{equation}
$$
$t_c$表示的是两条路线到达冲突点的时间，假设在恒定速度 $v_k$下,基于交互的加速度是根据经验选择的启发值，但也可以从训练数据中学习或者是概率函数。由于求解器是基于粒子的，因此运动模型生成的加速度还会受到模拟噪声($\sigma$)的干扰。 一方面，这允许在纵向方向上代表各种单独的驾驶风格。尽管如此，我们还是希望有一个尽可能好的运动模型（学习的或调整的）以保持较低的($\sigma$).这种情况是因为高($\sigma$)导致其他车辆的未来位置/速度的高度不确定性，从而导致更保守的策略。在这种case下，随着未来位置分布的增长，这样没有碰撞的潜在状态就会减少。由此产生的运动模型的总加速度现在可以正式写成如下：
$$
\begin{equation}
a_k = \min (a_{ref,k} +a_{int,k},a_{max}) + \mathcal{N}(0,\sigma^2)
\end{equation}
$$
自车的状态转移方程也是如此，只是路线不同而已。  

### D. Reward and Transiton Costs
奖励函数定义为：
$$
\begin{equation}
R(x,a) = R_{crash}(x) +R_v{x} +R_{acc}(a)
\end{equation}
$$
$ R_{crash}(x) $:如果发生了碰撞，则有一个巨大的负值。  
$ R_v(x) $:与参考速度的偏差（定义为没有车辆的道路上的平滑速度）会产生不同的成本，具体取决于偏差的符号。**if**  $v_0 >v_{ref}$，有 $R_v(x)= -K_{v+}(v_{ref}- v_0)^2$,**if** $ v_0 <v_{ref}$，有 $R_v(x)= -K_{v-}(v_{ref}- v_0)$,通过二次惩罚过高的速度，自车不太可能明显超过所需的速度。通过仅以线性方式惩罚过低的速度，规划器尝试以所需的速度行驶，但允许较慢的一些情况(例如，由于临时占用的车道)。 改变加速度需要消耗 $R_{acc}$ 来提供舒适度。

### E. 观测空间
观测空间定义为：
$$
\begin{equation}
\mathbf{o} = (o_0,o_1,...,o_k)^T
\end{equation}
$$
由于算法中不考虑传感器噪声（与离线场景生成相反）并且自动驾驶汽车的路线已知，因此其状态是完全可观察的，并表示为
$$
\begin{equation}
o_0 = 
\left( \begin{matrix}s_0 \\ v_0 \end{matrix} \right)
\end{equation}
$$
他车可能的行驶路线是无法直接观测的，它们的观测方程定义为：
$$
\begin{equation}
o_0 = 
\left( \begin{matrix}v_k \\ x_k \\ y_k \end{matrix} \right)
\end{equation}
$$

### F. Observation Model and Prediction
ABT算法通过对具有不同路径假设的粒子进行采样生成置信树来求解POMDP。因此，观测模型$Z(o,x',a)=P(o|x',a)$不能明确给出，而是必须在动作a之后的新状态 $x'$ 下对潜在观测值进行采样。尽管对于车辆 $N_k$来说，它的路线$r_k$无法被观测，但是是状态 $x_k$ 的一部分，而 $x'_k$可以在任意节点通过状态转移函数获取到。通过明确的变换 WTL可以创建新状态的相应观察。 $(s',v',r') \stackrel{WTL} \longrightarrow (v'_{obs},x'_{obs},y'_{obs})$ 
为了模拟未来时间步长中另一辆车所选择的未知路线的预测不确定性，采用了一个简单的判别分类器。我们提出了一个朴素的贝叶斯分类器，带有二维的特征向量 $\mathbf{f}_k$(基于车速和位置,如图4)，可以从观测空间中获得。
$$
\begin{equation}
\mathbf{f}_k = \left( \begin{matrix}f_{k,1} \\ f_{k,2} \end{matrix}\right) = \left( \begin{matrix}
|v'_k - v_{ref,r^(i)}(s'_k)| \\
\left\|[x'_k,y'_k]^T - [x_{k,pred,r^{(i)}},y_{k,pred,r^{(i)}}]^T \right\|_2
\end{matrix}\right)
\end{equation}
$$
$|v'_k - v_{ref,r^(i)}(s'_k)|$表示车辆k当前的加速度减去当前符合车辆k的预测路径$r(i)$对应的参考加速度。

$\left\|[x'_k,y'_k]^T - [x_{k,pred,r^{(i)}},y_{k,pred,r^{(i)}}]^T \right\|_2$表示车辆k当前x方向和y方向的速度减去$r(i)$上x方向和y方向的预测速度。

$N_k$在一个确定的路线上 $r_k = r^{(i)} \in M^{(i)}$的可能性，由贝叶斯公式推得：
$$
\begin{equation}
P(r_k = r^{(i)}|f_{k,1},f_{k,2}) = 
\frac{P(r^{(i)}) P(f_{k,1},f_{k,2}|r^{(i)})}{P(f_{k,1},f_{k,2})}
\end{equation}
$$
![fig4](E:/PnC/pnc_research/Decision/elements/BMW_fig4.png "fig4")   
假设每条路线有同样的先验概率，$(P(r_k = r^{(1)})=P(r_k = r^{(2)})=P(r_k = r^{(3)})=...)$,且每条概率是独立的，那么上式可以重写写为：
$$
\begin{equation}
P(r_k = r^{(i)}|f_{k,1},f_{k,2}) = 
\frac{P(r^{(i)}) P(f_{k,1},f_{k,2}|r^{(i)})}
{\sum^I_{l=1}P(f_{k,1}|r^{(l)})P(f_{k,2}|r^{(l)})}
\end{equation}
$$

尽管可以从样本数据中学习到 $P(f_{1/2}|r^{(I)})$，但为了模拟预测，我们简单地假设 $P(f_1|r^{(i)}) = \mathcal{N}(0,4.0)$，$P(f_2|r^{(i)}) = \mathcal{N}(0,6.0)$ 为正态分布。现在根据从式(20)中采样的可能的下一条路径$r(i)$为每个粒子生成观测值。

## IMPLEMENTATION

### A. 求解POMDP的理论基础

POMDP问题通常被认为是计算上不可行的，即使是离散的，有限数量的状态$|\mathcal{X}|$也会导致无限数量的信念状态$|\beta|$，因为当前信念的概率分布是连续的。这个问题导致了基于点的POMDP算法，该算法通过使用离散化的信念空间$|\beta|$​来逼近最优解。例如著名的基于值迭代(PBVI)算法[35]就属于这类求解器。

另一个方面是可达的信念空间，仅在此子空间中搜索解将问题缩小到更小的空间，但需要在线解决问题，因为解只从当前信念开始计算。为了在线逼近最优解，我们使用ABT算法[37]。ABT通过采样潜在的未来事件来构造信念树，逼近最优策略。

### B. 通过粒子滤波追踪信念状态

当前的信念状态通过一组包含可能状态实例的粒子来表示，求解器使用简单的拒绝采样，通过非加权粒子滤波跟踪当前的信念状态。跟踪信念状态，而不是在实际观测中使用简单的重采样，不仅可以获得更好的状态估计，而且可以保留先前估计的某个状态的值函数$\hat V(b)$的知识。

### C. Monte Carlo 幕采样

ABT算法的目标就是确定最优策略$\pi^*$，近似的最优策略可以通过进行Q函数来获取。
$$
\hat Q(b,a) \approx R(b,a) + \gamma \sum_{o\in O}\tau(b,a,o)V^*(\tau(b,a,o))
$$
即
$$
\pi(b) := \mathop{\arg\max}_{a\in A}\hat Q(b,a)
$$
其中，$\tau(b,a,o)$为从当前信念状态转移到下一状态的转移函数。$\hat Q(b,a)$是通过从当前信念中随机选择一个粒子，并从粒子的状态实例中采样到规划时域的潜在幕来计算的，接着$\hat Q(b,a)$就是从信念状态$b$到$a$的所有幕的价值的平均值。幕的采样如图5，尽管观测和转变可以从第四节中解释的相应模型中采样，但必须选择确定的行动。首先，该算法尝试当前信念节点中所有可用的动作。在进一步扩展过程中，选择最有希望的操作。因此，要展开的动作a是通过下面的公式来选择的：
$$
a := 
\begin{cases}
\sim A' ,if\ \ \   A'\neq \empty \\
\mathop{\arg\max}_{a\in A} \hat Q(b,a) + c\sqrt{\frac{\log(|H_b|)}{|H_(b,a)|}},\ \ \ otherwise
\end{cases}
$$
$A'$表示在相应信念状态下一直未被选择的所有动作的均匀分布，$|H_b|$从信念空间b开始的所有可能的幕，$H_{(b,a)}$表示从b开始执行动作a的所有次数。一个标量的、成比例的探索系数被定义为c，它允许在探索和开采之间进行权衡。这种类型的操作选择称为上置信度界限(UCB)。这种UCB算法的思想一方面是平衡探索和利用，另一方面允许算法收敛到最优q函数，给定足够的运行时间。作为估计的Q函数是从b开始所有幕的均值，只有当最优策略的动作在UCB动作选择下经常被采用，Q函数收敛于最优函数。虽然当给定的运行时间不够时，这可能导致非最优策略，但可以证明，算法首先在产生更保守策略的方向上搜索。这种情况下，假设的最优动作的分支很可能会扩展，而检测到的碰撞会导致不同的采样焦点。

![BMW_decision_fig5](E:\PnC\pnc_research\elements\BMW_decision_fig5.PNG)

### 确定性上界图搜索启发式算法

如果探索新的信念状态，则将信念状态的初始值设置为启发式估计，以使UCB算法更快地收敛。这个启发式是通过从当前信念执行一个完整的转出来计算的。推出策略应该是当前状态值的上限。这个想法是使用一个不考虑其他车辆状态和过渡模型的不确定性的确定性推出策略。因此，我们使用类似于[11]中使用的图搜索，但没有使用不可避免的碰撞状态(ICS)启发式。这将产生一个Dijkstra图搜索。虽然这可能在计算上过于复杂，但作为启发式方法，我们使用近似h(n)来完成穷举搜索结果，在n步之后终止图搜索，并使用恒速方法直到优化水平。我们定义$C(h(n))$作为最坏情况下节点探索复杂度的启发式，其结果为
$$
C(h(n))= |A|^n \cdot (t_{hor}-n)
$$
如图6所示：

![BMW_decision_fig6](E:\PnC\pnc_research\elements\BMW_decision_fig6.PNG)



### E Sampling timing of episodes

ABT算法是随时随地的，因此可以通过额外的运行时间来改进其解。同时，所使用的粒子过滤器不仅可以跟踪状态，还可以保留已经生成的属于该粒子的片段。虽然更多的计算时间优化了解决方案，但它也引入了近似质量和重新规划频率之间的权衡。由于这项工作的重点是行为生成，我们使用200毫秒来优化策略。尽管如此，我们以1hz重新规划，使算法的步长为1s，从而允许更大的规划范围。因此，我们使用剩余的800毫秒时间来生成进一步的片段，以便已经改进了下一个时间步骤的信念树近似。









