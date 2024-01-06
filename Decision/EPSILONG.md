# 论文2：EPSILON: An Efficient Planning System for Automated Vehicles in Highly Interactive Environments
## PipeLine
该文章的主要架构如下图所示：  

![pipeline](/Decision/elements/EPSILON_pipeline.png "pipeline")    
  
## Problem Formulation
做如下定义：  
$\xi_t$:表示以自车为中心的局部环境,包括道路结构、交通灯以及静态障碍物  
$x^i_t$表示第i辆车在时间t时的状态，不失一般性，i=0表示自车。  
对于时间t的规划周期内，自车接收到的观测为 $z_t$,并且使用该观测来估计真实的状态$x_t$  
$\left<z_t,\xi_t\right>$:表示整个行为规划层的输入  
$ \mathcal{D}_t$:表示行为层输出，由序列化的状态$[x_{t+1},x_{t+2},...,x_{t+H}]$,表示在整个规划时域H下，所有车辆的状态。  
$\left<\mathcal{D}_t,\xi_t\right>$:表示整个运动规划层的输入  
运动规划层的输出为一条光滑的参数化的轨迹。

---

一个POMDP问题可以由以下tuple定义： $\left<\mathcal{X,A,Z,T,O,R}\right>$,其中，$\mathcal{X},\mathcal{A},和\mathcal{Z}$分别表示状态空间，动作空间和观测空间。  
$T(x_{t-1},a_t,x_t) = p(x_t|x_{t-1},a_t)$:表示状态转移概率函数模型
$O(x_t,z_t) = p(z_t|x_t):观测函数模型  

$\mathcal{R}:\mathcal{X} \times \mathcal{A} \rightarrow \mathbb{R}$：表示奖励函数，agent在状态$x_{t-1}$下采取动作$a_t$的奖励。  
由于真实世界中，一些状态无法直接获得，比如隐藏的意图、随机的噪声等等，所以POMDP需要一个信念状态(belief)$b\in \mathcal{B}$,表示$\mathcal{X}$中的状态的概率分布。belief可在agent离开初始belief$b_{t-1}$后被更新，通过采取动作$a_t$，收到$z_t$的观察。belief的状态可以由以下贝叶斯公式推导：

$$
\begin{equation}
b_t(x_t) = p(x_t|z_t,a_t,b_{t-1})= \eta O(x_t,z_t) \int_{x_{t-1}\in \mathcal{X}}T(x_{t-1,a_t,x_t})b_{t-1}(x_{t-1})dx_{t-1}
\end{equation} 
$$
$\eta$表示 a normalizing factor. POMDP就是找个一个最优策略 $\pi^*$ ,将信念状态映射到一个动作 $\pi:  \mathcal{B} \rightarrow \mathcal{A}$,在规划时域最大化总的折扣奖励：

$$
\begin{equation}
\pi^*:=\argmax_{\pi} \mathbb E \left[
    \sum^{t_H}_{t=t_0} \gamma^{t-t_0}R(x_t,\pi(b_t))|b_{t_0}
\right]
\end{equation}
$$
$t_0$ 是当前的规划时间，$\gamma$表示折扣因子。对于在线POMDP来说，开始于初始的信念空间 $b_{t_0}$,在确定的规划时域中，并随着动作空间 $\mathcal{A}$ 和观测空间 $\mathcal{Z}$ 逐步的扩大，一个节点一个节点的构建出信念树。那么，通过在每个内部的节点应用贝尔曼方程，即可找出一条最优策略。
$$
\begin{equation}
V^*(b) = \max_{a \in \mathcal{A}} Q^*(b,a) = \max_{a \in \mathcal{A}} 
\left\{
 \int_{x \in \mathcal{X}} b(x)R(x,a){\rm}dx  +
 \gamma \int_{z\in \mathcal{Z}}p(z|b,a)V^*(\tau(b,a,z)){\rm d}z
\right\}
\end{equation}
$$

其中，$V^*(b) = \int_{s \in S}V^*(s)b(s)ds$ is the optimal utility functin for the belief state
不同于之前的工作仅使用单一的最优动作在初始节点上作为最终输出，这里我们在信念树上得到一条完整的轨迹(trace) $\mathcal{S}_t = [b^*_t,a^*_{t+1},z^*_{t+1},b^*_{t+1},...,b^*_{t+H}] $，通过在动作分支和观测分支上循环构建以下优化目标
$$
\begin{equation}
\begin{aligned}
a^*_t = \argmax_{a_t \in \mathcal{A}}Q^*(b_{t-1},a_t) \\
z^*_t = \argmax_{z_t \in \mathcal{Z}}p(z_t|b_{t-1},a^*_t)
\end{aligned}
\end{equation}
$$
生成的轨迹包含最优动作 $a^*_t$ 在每个信念节点 $b^*_{t-1}$,同时在动作 $a^*_t$ 下得到最有可能的观测 $z^*_t$. 最终的决策 $\mathcal{D}_t$ 可以通过在 $S_t$ 每一步 应用 $x_t = \argmax_x b^*_t(x)$得到。相比于一个单一的优化动作， $\mathcal{D}_t$ 包含更多关于环境和未来预报中完整的信息，这在接下来的运动规划中是必不可少的。
考虑在t时刻有N个agents的驾驶场景，所有的状态可以写为，$x_t={x^0_t,x^1_t,...,x^N_t} \in \mathcal{X}$,其中， $x^i_t \in \mathcal{X}^i$表示第i辆车的状态，包括位置，车速，加速度，朝向以方向盘转角，另外对于周围的车辆，一些隐藏的信息如驾驶意图、驾驶激进程度，虽然无法被直接观测到，但也需要包含到状态中。在这个驾驶场景中，我们仅能控制自车的油门刹车和转向，也就是说无法直接的决定其他agnet的动作。因此原始问题中的动作可以重新表示为 $a_t = a^0_t$ ,同样的动作空间 $\mathcal{A} = \mathcal{A}^0$. 完整的状态转移模型T可以表示为完整状态的联合分布: $p(x_t|x_{t-1},a_t) = p(x^0_t,...,x^N_t|x^0_{t-1},...,x^N_{t-1},a^0_t)$这对于直接建模来说并不简单.  
然后，由于在真实交通环境中，大多数其他的agents，比如vehicle和bicycle，都遵循了交通规则和物理定律，可以简化上面的公式通过对周围agent做出合理的假设，将其转为多智能体交互的模型。我们对周围每个agent赋予一个条件概率转移模型，伴随着相应的动作 $a^i_t \in \mathcal{A}^i,\forall i \neq 0$,并且假设每个agent的瞬时转移是独立的，于是有：

$$
\begin{equation}
\large{p(x_t|x_{t-1},a_t)} \approx 
\begin{matrix} 
\underbrace 
{\large{p(x^0_t|x^0_{t-1},a^0_t)}} \\{\scriptsize{ego\,transition}}
\end{matrix}

\prod^N_{i=1} \int_{\mathcal{A}^i}

\begin{matrix} 
\underbrace 
{\large{p(x^i_t|x^i_{t-1},a^i_t)}}
\\\scriptsize{i-th\,agent's\,transition}
\end{matrix}
\begin{matrix} 
\underbrace 
{\large{p(a^i_t|x_{t-1}) }} \\ \scriptsize{driver model}
\end{matrix}
{\rm d}a^i_t
\end{equation}
$$

该公式将被控车辆和其他agents区别开来，在公式中，$p(x^i_t|x^i_{t-1},a^i_t)$是其他代理的假设转换模型，反映代理的低级运动学， $p(a^i_t|x_{t-1})$是假设的驾驶员模型，代表其他智能体的高级决策过程，根据驾驶环境提供适当的控制信号。驾驶员模型可以是用户定义的，也可以是通过数据驱动来学习的，可以通过一些潜在的状态如意图或者激进度来控制，实现不同的操作或驾驶风格。
注意到，自车的动作 $a^0_t$时通过预定义的自车策略生成的。自车的观测 $z^0_t = z_t$包括感知模块估计的其他车辆的位置和速度。对于周围的agents,对于周围的智能体，我们使用它们的位姿作为原点，并将自我观察 $z^0_t$ 转换到它们的坐标系中作为观察 $z^i_t$。由于 $z^i_t$ 完全取决于自车的观测，假设观测过程是独立的，我们可以得到： $p(z_t|x_t) = \prod^N_{i=1}p(z^i_t|x^i_t)$.于是，式(1)可以写为：

$$
\begin{equation}
b_t(x_t) = \eta \cdot 
\begin{matrix} \scriptsize{belief\,update\,for\,ego\,agent} \\
\overbrace{
p(z^0_t|x^i_t)\int_{\mathcal{X}^0}p(x^0_t|x^0_{t-1},a^0_t)b^0_{t-1}(x^0_{t-1}){\rm d}x^0_{t-1}
}
\end{matrix}\cdot
\,\,
\prod^N_{i=1}
\begin{matrix} \scriptsize{belief\,update\,for\,other\,agent} \\
\overbrace{
p(z^i_t|x^i_t) \iint_{\mathcal{X}^i \mathcal{A}^i}
p(x^i_t|x^i_{t-1},a^i_t)p(a^i_t|x_{t-1})b^i_{t-1} {\rm d}a^i_t {\rm d}x^i_{t-1}
}
\end{matrix}
\end{equation} 
$$
尽管每个agent的状态转移是独立的，假设的驾驶员模型 $p(a^i_t|x_{t-1})$ 和观测模型 $p(z^i_t|x^i_t)$ 利用了所有agnets的状态和观测，所以信念的更新是一个交互的过程。请注意，隐藏状态是在信念更新期间使用观测值逐步估计的，这正是行为预测的作用。因此，POMDP 隐式包含预测，表明规划和预测是自然耦合的。还值得注意的是，将预测和规划分离的方法本质上是原始 POMDP 公式的简化。通过循环应用置信更新，我们可以从初始置信节点开始构建置信树并提取最终决策 $\mathcal{D}_t$.然而，置信树的规模随树深度呈指数增长，这对于实时应用程序来说在计算上是难以处理的。为了克服这个问题，在本文中，我们将领域知识应用到公式中，以进一步简化在快速变化的驾驶环境中实现实时决策的问题，同时保留处理交互和不确定性的能力。给定行为规划器 $\mathcal{D}_t$的输出，运动规划层旨在生成安全且平滑的轨迹，以细粒度的方式实现高层决策。请注意，由于我们的行为层是在多智能体设置中制定的，因此它自然会推理所有智能体的未来状态。因此，运动规划的作用可以归结为局部轨迹优化问题，如图2所示。  
![MotionPlanning](/Decision/elements/Epsilon_mp_ssc.png "SSC")    

## Efficient Behavior Planning
### A. Motivating Example
一个显然的事实是，对于人类驾驶员来说，是无法在大脑中构建颗粒度非常细的网络地图。相反，更可能的是倾向于利用一些公共驾驶常识去推出部分长期的语义级的动作(如车道保持、变道、停车等)，这样做也使得决策更有效率。通过这些语义级动作，我们可以利用动作分支采样出一条长期的最可能的轨迹出来，如图3所示。  
![SemanticAction](/Decision/elements/epsilon_figure3.png "SemanticAction")    
可以看出，即使仅有一些非常粗糙的控制动作(3个横向3个纵向)，利用多步前瞻算法，也会生出如图中灰色区域中的的轨迹空间，对一个车辆来说都是非常巨大的，多车就更糟糕了。但是如果有语义动作的加入(通过不同驾驶激进程度跟随中心线)，利用专业知识，轨迹空间也就变成紫色部分的几条了。  
另一个关键问题在于其他agent的意图是无法直接观测的，POMDP需要对其他智能体的意图进行采样，我们将其称为观察空间中的分支。所需样本的数量相对于agent的数量呈指数级增长，这是效率问题的另一个根源。  
引导分支的动机是模仿人类驾驶员的决策过程并利用领域知识进行有效的探索。在这篇文章中，借用MPDM思路来push我们的行为规划器一步向前仿真，从以下三个方面：
* Flexible driving policy:在动作空间中通过引导分支，减少原始POMDP中的决策空间。相比于MPDM，该方法考虑多个未来决策点，而不是在整个规划范围内使用单个语义动作，从而获得更灵活的运动。
* Efficient policy evaluation:没有直接对agent意图的所有可能组合进行采样，这会导致指数级的复杂性，而是提出了一种机制来挑选以自我策略为条件的潜在风险场景，以减少策略评估期间的计算费用。
* Enhanced risk hanging in real traffic:所提出的规划器提供了一种具有安全机制的新前向模拟模型的实现，可以减少因真实交通参与者与假设模型不一致而造成的风险

### B. Guided Action Branching
什么是Guided Action Branching?
为了在超大空间中探索，我们引入了语义级动作，如图3。本质上，语义动作是可以用人类感官直接解释的高级动作，例如变道和减速。语义级动作包含多个小步骤，可以以闭环方式执行，它根据预定义的控制器在每个步骤生成原始动作（即转向角和纵向加速度）。与原始动作相比，使用语义动作将探索限制在较高似然区域内，并且可以自然地生成类似人类的运动。而且语义级动作的持续时间更长，可以有效地将置信树的高度降低到一个相对较小的数字，同时获得足够大的规划范围。  
在数学上，上述假设在驾驶员模型中引入了一个附加的变量 $\phi^i_t \in \Phi^i $,表示t时刻agent i的语义动作。$\Phi^i$表示不同类型的agent有不同的动作集。例如，自车的动作集可以更大一些，因为更多的动作可以带来更灵活的驾驶行为，而他车的动作集可以小一些让问题复杂度降低。在每一步中，一个语义动作 $\phi^i_t$结合上一刻的状态，利用预定义的控制器 $p(a^i_t|x_{t-1},\phi^i_t)$可以生成一个原始动作 $a^i_t$。这样反映出一个事实：从语义动作到原始动作的的映射，需要将周围驾驶信息考虑进来，也表明了前向仿真的闭环特性。  
为了结合语义动作，信念更新可以表示为 $b_t(x_t) = p(x_t|z_t,\phi_t,b_{t-1})$,将原始动作替换成语义动作，转换模型写为 $T(x_{t-1},\phi_t,x_t)$.这里我们再一次指出唯一可控的元素是自车的语义动作，接着我们写成新的joint state transition:
$$
\begin{equation}
\large{p(x_t|x_{t-1},a_t)} \approx 
\int_{\mathcal{A}^0}
\begin{matrix} 
\underbrace 
{\large{p(x^0_t|x^0_{t-1},a^0_t)}} \\{\scriptsize{ego\,transition}}
\end{matrix}

\begin{matrix} 
\underbrace 
{\large{p(a^0_t|x_{t-1},\phi^0_t)}} \\{\scriptsize{ego\,controller}}
\end{matrix}
{\rm d}a^0_t
\prod^N_{i=1} \int_{\mathcal{A}^i}

\begin{matrix} 
\underbrace 
{\large{p(x^i_t|x^i_{t-1},a^i_t)}}
\\\scriptsize{i-th\,agent's\,transition}
\end{matrix}
\begin{matrix} 
\underbrace 
{\large{p(a^i_t|x_{t-1}) }} \\ \scriptsize{driver model}
\end{matrix}
{\rm d}a^i_t
\end{equation}
$$
这个公式和前面一个公式相比，多了一个用上一时刻的状态和当前的语义动作来推当前原始动作的概率。
自车的语义动作需要规划器在指定的空间中进行探索。同样对驾驶员模型引入语义动作，驾驶员模型更新为：

$$
\begin{equation}
\large{p(a^i_t|x_{t-1})} =
\sum_{\phi^i_t \in \Phi^i}
\begin{matrix} 
\underbrace 
{\large{p(a^i_t|x_{t-1},\phi^i_t)}} \\{\scriptsize{agent\,controller}}
\end{matrix}

\begin{matrix} 
\underbrace 
{\large{p(\phi^i_t|x_{t-1})}} \\{\scriptsize{new\,driver\,model}}
\end{matrix}
\end{equation}
$$
$p(\phi^i_t|x_{t-1})$表示第i个agent的语义级动作依赖驾驶的上下文来实现.  
与MPDM类似，我们通过直接在有限数量的策略内预定义决策空间来修剪动作空间中的信念树。更进一步，我们通过引入特定域的闭环策略树(domain-specific closed-loop policy tree)（DCP-Tree）来扩展时域中的决策点。DCP树的节点是与特定持续时间相关联的预定义语义动作，树的有向边表示时间上的执行顺序。DCP-Tree起始于正在发生的动作 $\hat\phi$,是上一次规划周期给出的最优动作。每当进入到一个新的规划集中，DCP-Tree会以该动作为根节点重新生成。可能的策略序列的数量规模是与树的深度成指数增长。为了避免这样的问题，DCP-Tree的扩展根据一个预定义的策略，就像人类驾驶员那样进行观察，而且在一个决策周期中，不会来回反复的更改策略。因此，从正在进行的行动来看，每个政策序列在一个规划周期内最多会包含一次行动变化，如图4所示，而来回行为则通过重新规划来实现。这样叶子节点的数量大致为 $O[(|A|-1)(h-1)],\forall h > 1$,即随着树的高度线性增长。更多细节看文献[Efficient Uncertainty-aware Decision-making for Automated Driving Using Guided Branching].  
![DCP-Tree](/Decision/elements/epsilon_figure4.png "tree")    
从图中可以很容易看出，因为每次规划只会变更一次动作，因此复杂度是线性的，同样也可以看出，只要策略没有rebuild，那么就依然沿用上一次的最优策略。  
不是一般性，我们定义规划行为策略 $\pi \in \hat\Pi$，表示将要执行的一个又一个语义动作。一旦DCP-Tree被构建出来，自车的整个决策空间 $\hat\Pi$ 也被构建出来。通过在DCP-Tree上遍历所有的路径，即可得到候选的轨迹。因此,行为规划被简化为从候选策略集中选择效用最大的最佳策略，注意，由于每个策略的评估是相互独立的，因此我们可以以并行方式实现规划器，而无需额外的工作。

### C. Guided Observation Branching
为了评估策略的效用，一个公共的方法是使用MonteCarlo方法。基本思想是对初始信念中的起始状态进行采样，并使用模拟转变和观察的黑盒模拟器来生成以下状态。接着，只要有足够的采样，每个节点的信念可以用一个粒子滤波方法近似。然而，在高维空间上采样缺乏效率，并且仿真模型中的建模随机性使情况变得更糟。在本文工作中，为了实时性，我们采用黑盒仿真并且更进一步的简化belief的更新过程。我们假设自车的状态是完全客观的，观测是没有噪音的，只要自车的传感器足够精确。然后，我们通过假设语义动作完全由隐藏状态（例如意图）决定，让agent的策略 $p(x^i_t|x^i_{t-1},a^i_t)$（从联合状态到代理语义动作的映射）具有确定性,从而将智能体行为的随机性转化为信念状态。对于转换模型 $p(x^i_t|x^i_{t-1},a^i_t)$ ,如果计算预算足够，我们还使用确定性运动学模型，同时可以注入控制噪声来反映假设模型的不准确性。在上述简化之后，唯一不确定性的来源就是他车的隐藏状态，以及他们收到观测后的状态更新。基于对动作分支如何引导和实现的理解，剩下的问题是估计他车的语义动作，或者更进一步，估计其他智能体的隐藏状态，这些状态在观察模型中部分建模 $p(z^i_t|x^i_t)$.不同于之前的工作，对于其他交通参与者，我们假设在前向模拟阶段内，其隐藏的状态不会更新，意味着其他agent的语义动作是固定的。原因之一是一旦允许在前向模拟阶段内允许变更隐藏状态，那么计算复杂度会指数上升$O(|Z|^d),显然在实时应用不合适。而实际上这种假设得到的结果也令人足够满意。最终，我们对他车的隐藏状态一开始就进行估计，并用该信念状态进行预测。我们通过对初始信念进行采样来获得场景，并通过进行确定性前向模拟来实现它：
$$
\begin{equation}
\large{b_{tH}(x_{t_H})} =
b_{t_0}(x_{t_0})
\prod^{t_H}_{t=t_0}
\left\{
\begin{matrix} 
\underbrace 
{\large{p(x^0_t|x^0_{t-1},a^0_t)p(a^0_t|x_{t-1},\phi^0_t)}} \\{\scriptsize{ego\,state\,propagation}}
\end{matrix}

\prod^{N}_{i=1}
\begin{matrix} 
\underbrace 
{\large{p(x^i_t|x^i_{t-1},a^i_t)p(a^i_t|x_{t-1},\phi^i_0)}} \\{\scriptsize{other\,agent's\,state\,propagation}}
\end{matrix}
\right\}
\end{equation}
$$
通过这个公式可以看出，自车的语义动作依赖与当前的策略，他车的语义动作建立在初始信念上，并一直固定。注意到，尽管我们在前向仿真中用了多种简化，但其仍然是一个闭环策略，因为他车的动作依赖于上一步的状态 $x_{t-1}$,该状态保留了交互意识。因此，策略的总效用是采样场景的加权和，而权重正是初始信念的值。  
在本文工作中，他车的语义动作表现为跟随特定中心线的动作。对语义动作 $\phi^{i\neq 0}_t$的估计等价于对候选中心线的分布估计。在高速场景中，由于简单的道路结构，这个task是非常直接的，我们采用了一个基于规则的预测器去实现。在urbun环境中面对更复杂的道路结构，这个task更具有挑战性。本质上，在城市环境中，根据场景的不同，我们面临着不同数量的候选中心线。为了解决这个问题，我们提出了一种轻量级分类网络，用于估计概率，同时支持不同数量的类别。所提议的基于学习的意图估计器的细节在后面详细阐述。  
在得到初始信念状态后，接下来的问题就是根据估计的概率确定$\phi^{i\neq 0}_t$,因为每个agent存在多种潜在的可能(比如有多条候选的车道中线)，我们为所有交通参与者的某一种意图组合定义为一种场景。直观上，场景的数量呈指数级增长，每个场景的概率可以从每个参与者估计概率的联合分布中得出。一旦agnet过多，计算出所有场景的代价将会是巨大的。在文献21中，提出了使用条件关注分支(CFB)来解决这个问题。本质上，CFB的目标是用尽可能少的分支来找到附近车辆可能导致危险结果的意图。<font color=red>note：啥意思？为什么要找出导致危险的意图？这里找到的是可能与自车发生交互的车辆的意图，不然肯定会导致危险，关于CFB详细解释见pre-epsilon笔记</font>

### D. Multi-agent Forward Simulation
如上所述，场景是通过前向模拟实现的，它以交互的方式同时传播环境中的所有agent。在多个agent前向传播的过程中，自车和他车的所有轨迹都会被生成出来。当前许多方案采用的是利用预测模块先生成一条预测轨迹出来。然而，在预测和规划解耦之后，考虑自我车辆未来运动对其他智能体的影响是有问题的。相反，我们的方案中将预测耦合在行为规划中。本质上，在规划周期的每一步中，每一个agent都会观测上一时刻的其他agent的状态，并利用预定义的驾驶模型 $p(a^i_t|x_{t-1})$ 得到基于模型的控制律。基于这些控制律，agent就可以通过状态转移 $p(x^i_t|x^i_{t-1},a^i_t)$ 得到下一时刻的状态。
不同于之前的工作，我们将被控车辆和其他交通参与者区分出来。对于被控车辆，我们设计一个context-aware控制器，保证自车行为的灵活性和安全性，同时其他车辆还是采用简单的基于模型的控制器。相比于简单的控制器，context-aware控制器可以生成更真实的操作，特别是在变道过程中。我们使用上下文感知控制器的原因是，在实现语义级动作的策略设计中具有相当大的潜力，这可以促进在动作分支期间实施高级驾驶风格。图6提供了一个直观的示例，其中我们展示了如何进行车道变换。
![Context-aware controller](/Decision/elements/epsilon_figure6.png "controller")   
通过引入新的控制器，解空间扩大，并可以在不同交通环境下得到更为灵活的轨迹。受益于context-aware控制器，所需的动作序列数量下降，而仿真的成功率显著上升，也进一步增强了当前分支的性能和效率。实际上，此类控制器的实现在交通仿真文献中已经得到了很好的研究，我们在第 V-D 节中详细介绍了我们的实现。  
尽管前向仿真模型看起来十分直接，但是实际上对未来预测有很高的准确性。甚至和一些高级的预测模块相比起来也不逊色。
### E. Safety Mechanism
上面的讨论并没有回答一个问题：如果真实世界的驾驶员行为与预定义的不一样该怎么办？在实践中，我们观察到这个问题往往会导致决策风险的低估，因为现实世界的驱动因素往往是随机的且有噪音的理性。在传统方法中，这个问题是通过留下足够大的安全边际来解决的[15,33]，这限制了高度交互环境中的灵活性。针对这种问题，我们提出了一种双层的安全机制。  
首先，我们通过在上下文感知控制器中嵌入安全模块来增强正向仿真模型的安全性，该模块可以在一定程度上自动确保控制输出安全或无故障。在责任和安全方面，Shai等人[48]定义了责任敏感安全（RSS），它提供了各种驾驶情况下的安全横向和纵向距离以及危险情况下的正确响应的数学模型.(即当安全距离不满足).在每一步仿真中，我们会检测被控车辆的仿真状态是否满足RSS-安全。如果不是，我们检查上下文感知控制器提供的控制信号是否遵循 RSS 模型定义的正确响应标准。如果仍然不是，我们生成一个遵守适当响应的安全控制信号并覆盖控制输出。
其次，我们通过在策略选择中设置安全准则来强化决策层的鲁棒性。我们的动机是人类驾驶员可能会执行激进的切入操作：一方面，驾驶员会意识到其他车辆会对他的插入产生反应，另一方面，有经验的驾驶员会在他们脑海中产生一个安全的评估过程，即使其他车辆不肯合作，他们无法完成变道，他们仍然可以灵活地取消变道。从这一点出发设计我们的策略，这种特性我们可以通过备份规划(backup plan)来实现。由于我们的行为规划器在一个规划周期中评估多个策略，因此我们可以采用成功完成前向模拟和安全检查的任何其他策略作为后备计划。为了更有针对性，我们手工制定了备份策略选择的优先级。例如对于"lane change"策略的备份计划应该是"lane change cancelled".我们可以找到相应的备份策略，并检查是否有至少一种可行的方案。 如果是这样，计划好的政策就可以执行。如果根本没有安全策略，意味着行为规划失败，则会显示警告信号，并触发低级别的主动保护，例如自动紧急制动（AEB）。
### F.Policy Selection
在行动空间和观察空间的引导分支以及转换和观察的近似之后，行为规划器可以简化为有限数量的政策评估问题。对于每个策略，我们通过评估计划的行为和模拟的轨迹来计算每个场景的奖励的加权和。具体算法流程如下：
---

Algorithm 1:Process of behavior planning layer
1. Input:Current states of ego and other vehicels s;ongoing aciton $\hat a$;Pre-defined semantic action set $\mathcal{A}$;Planning horizon $t_h$;
2. $\Re \leftarrow \emptyset$ //set of rewards for each policy
3. $\Psi \leftarrow UpdateDCPTree(\mathcal{A},\hat a) $//DCP-Tree $Psi$
4. $\hat \Pi \leftarrow ExtractPolicySequences(\Psi)$
5. foreach $pi \in \hat \Pi$ do:
6. ------$\Gamma^{\pi} \leftarrow \emptyset;$ //set of simulated trajecotries;
7. ------$\Omega \leftarrow CFB(s,\pi);$//set of critical scenarios
8. ------for each $\omega \in \Omega$ do:
9. -----------$\Gamma^\pi \leftarrow \Gamma^\pi \cup SimulateForward(\omega,\pi,t_h)$ 
10. -----end
11. ------$\Re \leftarrow \Re \cup EvaluatePolicy(\pi,\Gamma^\pi)$
12. end
13. $\pi^*,\hat a \leftarrow SelectPolicy(\Re)$
---
发现算法流程和EUDM里面一模一样。

## Trajectory Generation with spatio-temporal semantic corrdior
在上一节中，得到了最终的决策结果为 $\mathcal{D}_t = [x_{t=1},x_{t+2},...,x_{t+H}]$。接下来的问题就是如何生成一条光滑的、安全的动态可行的轨迹，并遵循最终的决策。我们使用SSC算法来实现。由于我们规划系统的层次结构，行为层和运动层的制定和目标不同，导致两层之间可能存在差异，导致规划不一致。与[1]不同的是，除了最小化沿轨迹的加加速度外，我们还将轨迹在相应时间的参考状态的均方误差纳入其中，这表示轨迹与 $\mathcal{D}_t$ 中的自我决策之间的相似度。最终，在 $\sigma$维度上第j段的代价$J^\sigma_j$写为：

$$
\begin{equation}
J^\sigma_j = \omega^\sigma_s \cdot 
\int^{t_j}_{t_{j-1}} 
\left(
\frac{d^3 f^\sigma_j(t)}{dt^3}
\right)^2 dt +
\omega^\sigma_f \cdot \frac{1}{n_j}
\sum^{n_j}_{k=0}(f^\sigma_j(t_k)-r^\sigma_{jk})^2
\end{equation}
$$
$r^\sigma_{jk}$表示由对应于第j段的自车模拟状态生成的维度σ上的第k个参考状态。

## IMPLEMENTATION DETAILS
### A. Semantic-level Actions and DCP-Tree
我们考虑横向和纵向行动以获得多样化的驾驶策略。对于高速公路和城市环境，横向动作集可以定义为相邻车道的目标中心线$c_i$，因为车辆最常见的横向运动是可用车道之间的车道间变换。注意横向动作集 $N^{lat}_a$ 需要根据实际上下文来确定，因为毗邻的车道可能不可用(因为交规)或根本不存在。对于纵向动作来说，为了确保连续性和安全性，我们使用一个预定义的速度控制器而不是直接应用纵向速度信号。因此，我们将语义动作定义为获得不同纵向机动的速度控制器的一组参数。例如，在IDM模型中，我们可以通过增加所需速度同时减少车头时距和最小间距来实现更激进的驾驶风格。这里。我们定义纵向语义动作集为:
$N^{lon}_a = 3$表示 {aggressive,moderate,conservative},最终，我们得到$N^{lat}_a \times N^{lon}_a$个语义动作。对于DCP-Tree来说，设置树的深度为5，每个语义级的动作持续时间设置为1s,而闭环模拟以0.2秒的分辨率进行，以保持保真度。因此整个规划时域为5s.

### B. Context-aware Forward Simulation Model
闭环仿真的目标是在多agent系统中推动所有状态更新，同时考虑潜在的交互问题。对于上面提出的context-aware 驾驶员模型，我们利用不同的横向和纵向控制器来实现不同的语义动作组合。具体来说，对于车道保持，横向控制器遵循纯追踪控制器，而纵向运动则由 IDM 控制。<font color=yellow>对于变道操作，纵向和横向控制是耦合的，因为需要考虑当前车道和相邻车道中的所有车辆，我们将横向控制器扩展为反应式纯追踪，将纵向控制器扩展为间隙通知速度控制器。</font>  
为了更好地说明所提出的正向仿真模型，我们提供了两个示例场景，如图7所示。  
![forward simulation model](/Decision/elements/epsilon_figure7.png "forward simulation model")    
如图a所示，反应式横向控制器用于目标车道上有障碍物的变道过程。原始的和可选的参考轨迹如橙色虚线所示，车道线与障碍物之间的最小间隙由红色区域表示。如果目标点 p_target确定，那么前视距离和方向角误差也可快速确定。图b表示变道时纵向上的间隙通知控制器。假设在变道时有三辆车存在影响，自车前方的头车 $C_l$,旁边车道的前后车，$C_f,C_r$,目标间隙表示为如图的黄色区域中，期望的纵向状态标记为绿色星星。  
注意到，驾驶场景是表示在曲线坐标系上的，s(.),d(.)分别表示横纵向位置。我们在 Tab 中简要介绍了相关变量。而其中重要的部分将在下面的内容中进一步介绍。  
|  Variables   | Description  |
|  ----  | ----  |
| $C_l,C_r,C_f$  | 图7中相关车辆 |
| $W_{ego},L  | 自车的宽度和长度 |
| $\gamma_1,d_{\gamma_1}$| 目标中心线和它的横向位置 |
| $\gamma_2,d_{\gamma_2}$| 可选中心线和它的横向位置 |
| $d_c$ | 车道线与当前车道中线的距离 |
| $l_{oc}$ | 障碍物与车道线的最小距离 |
| $l_{safe}$ | 用户定义的自车与障碍物车辆横向最小间隙 |
| $p_{target}$ | 纯跟踪控制器的预瞄点 |
| $l_d,\alpha$ | 预瞄距离和误差角 |
| $ s_{des},\dot s_{des} | 并道时的纵向期望位置和期望速度 |
| $ s_{ego},\dot s_{ego} | 自车的纵向位置和速度 |
| $\dot s_{pref}$ | 用户和交规的首选纵向速度 |
| $ s_r $ | 预期后车 $C_r$ 的前保险杠纵向位置 |
| $ s_f $ | 预期前车 $C_r$ 的后保险杠纵向位置 |
| $\dot s_r,\dot s_f $ |观测到的 $C_r,C_f$的纵向速度 |
| $ s_{tr},s_{tf}$ | 从安全车头时距得出的阈值纵向位置 |
| $l_{min},T_{safe}$ |  纵向最小间距和安全车头时距 |
  
   
对于横向控制来说，我们首先检查目标车道是否有足够的空间可以并入，如果毗邻的车道是空闲的，那么目标车道 $\gamma_1$作为中心线赋予到参考线中，用于纯跟踪控制，否则的话，选择备用的无碰撞的路径$\gamma_2$作为参考线。$\gamma_r$的横向位置计算方式为：
$$
d_{\gamma_2} = d_c - \frac{W_{ego}}{2} - max(0,l_{safe}-l_{oc})
$$
通过跟踪备用的参考路径 $\gamma_2$,受控车辆可以在不越过车道标线的情况下向目标车道轻微移动，同时与障碍物保持适当的距离。相应的方向角计算为$\delta_{ctrl} = arctan \frac{2Lsin\alpha}{l_d}$.  
对于纵向控制来说，选择目标车道中的间隙，并根据新的前车 $C_f$ 和跟随者 $C_r$ 的状态生成期望的纵向状态。那么期望的纵向状态计算方式为：
$$
s_{des} = min(max(s_{tr},s_{ego}),s_{tf})
$$
$$
\dot s_{des} = min(max(\dot s_{tr},\dot s_{ego}),\dot s_{tf})
$$
$s_{tr}$和$s_{tf}$的计算方式为：
$$
\begin{bmatrix}
s_{tr} \\ s_{tf}
\end{bmatrix}=
\begin{bmatrix}
s_r+l_{min} + T_{safe}\dot s_r \\ s_f-l_{min} + T_{safe}\dot s_{ego}
\end{bmatrix}
$$
所呈现的期望状态为我们提供了变道操作期间纵向控制的目标，使整个过程舒适且安全。即使保险杠到保险杠的距离很小时，期望的状态仍然存在，并且自我纵向状态将逐渐收敛到目标并与间隙对齐。<font color=yellow>这里意思我理解应该是即使如图7所示，自车与后车的位置可能有重叠，我们照样也可以并入到目标车道中去</font>进一步的，即使没有前车或者后车，这个公式依然能够成立。  
一旦期望的纵向状态生成，就可以使用简单的反馈控制器去让自车跟踪到期望位置。期望的纵向加速度为：
$$
a_{track} = K_v(\dot s_{des} +K_s(s_{des}-s_{ego})-\dot s_{ego})
$$
另外，我们还是要来保证当前车道里与前车的安全距离。使用IDM模型来获取另一个加速度$a_{idm}$用于自车的距离控制。最终，加速度输出为 $a_{ctrl} := min(a_{track},a_{idm})$.  
为了进一步保证安全，我们继续使用RSS准则来保证。一旦自车在当前仿真步骤中是RSS危险的，根据[48]中引入的规则，这种情况不是RSS安全的，提供了可行的纵向和横向加速度限制。我们将控制约束与输出信号($\delta_{ctrl},a_{ctrl}$ 进行比较，并检查输出是否满足正确的响应。 如果没有，我们会生成由正确响应引起的安全控制信号并覆盖输出。对于车辆模型，一律采用自行车模型来描述，为了平衡计算效率和仿真正确性。

### C. Intention Estimator
为了得到更好的预测性能，我们提出了一种基于神经网络的方法，利用观察历史和环境信息来估计周围智能体的意图。神经网络结构如图8所示：
![network](/Decision/elements/epsilon_figure8.png "network")    
在顶层，网络将候选中心线的位置样本和目标车辆的历史轨迹作为输入特征，并使用编码器-解码器结构来预测每条候选中心线的标量分数。然后通过应用 softmax 函数对分数进行归一化。由于网络预测每个中心线实例的标量分数，并且不依赖于场景中中心线的数量，因此该网络可以在各种城市环境中工作，如图9所示。  
![Intention estimation](/Decision/elements/epsilon_figure9.png "Intention estimation")  
如图8所示，意图预测网络包含三个主要模块。车道编码器以目标车辆周围车道的中心线作为输入。我们获取中心线上的采样点并使用位置坐标作为输入。对于每条中心线来说，我们总共采样70个点。首先使用内核大小等于3的多个一维卷积层对中心线的特征进行编码，接着在其尾部采样平均池化的方法。对于目标车辆的轨迹，我们使用LSTM对二维位置进行循环编码并生成一个潜在向量，它是运动历史的紧凑表示。我们将 LSTM 的输入大小（输入嵌入后）设置为$n_x = 4$，并将LSTM中隐藏状态的长度设置为 $n_h = 64$。  
处理车道和轨迹信息后，我们将轨迹嵌入与每个车道嵌入连接起来，然后使用具有 ReLU 激活函数的多层感知器（MLP）来估计候选车道的分数。最后的概率分布由softmax函数获得。 Fig8展示了上面介绍的模块，由于意图预测本质上是一个分类问题，因此我们使用负对数似然（NLL）作为损失来最小化。

### D. Policy Evaluation
策略的总体奖励是通过CFB选择的每个场景的奖励的加权和来计算的。奖励函数组成有：自定义的效率指标、安全指标和导航指标；
$$
F_{total}=\lambda_1F_e + \lambda_2F_s + \lambda_3F_n
$$
为了评估效率成本，在语义动作完成后考虑两个来源。其中一项是当前模拟速度与自我车辆的首选速度之间的速度差 $\Delta v_p = |v_{ego} - v_{pref}|$.另一项通过检查当前车道前车辆的速度超调部分代表目标车道的效率  $\Delta v_o = max(v_{ego} - v_{lead},0)$,并且计算前车速度与首选速度之间的差。 $\delta v_l = |v_{lead} - v_{pred}|$,效率指标可以写为：
$$
F_e = \sum^{N_a}_{i=0}F^i_e = \sum^{N_a}_{i=0}(\lambda^p_e \Delta v_p + \lambda^0_e \Delta v_o+\lambda^l_e \Delta v_l)
$$
Na表示当前策略下的所有语义动作的数量。  
对于安全性指标，我们评估自我模拟轨迹的所有离散状态以及附近的所有其他轨迹，如果发生碰撞，对应的场景将直接被标记为失败，并受到巨大成本的惩罚。此外，我们还检查自我轨迹是否包是RSS危险状态。如状态是RSS-dangerous，我们得到当前环境下安全速度区间 $[v^{lb}_{rss},v^{ub}_{rss}]$ ，并对安全速度区间进行惩罚。有：
$$
F_s = \lambda^c_s b_c + \sum^{N_s}_{i=0} b_r F^i_s = \lambda^c_s b_c +
\sum^{N_s}_{i=0} b_r v_{ego} \lambda^{r1}_s e^{\lambda^{r2}_s |v_{ego}- min(max(v_{ego},v^{lb}_{rss}),v^{ub}_{rss})}
$$
导航成本由用户偏好和决策上下文决定。我们对与人机界面（HMI）提供的用户命令相匹配的策略实施奖励（负成本），这可以实现驾驶员触发的变道。此外，为了提高决策的一致性，我们还奖励与上一个规划周期做出的决策具有相似含义的政策。例如，如果上一个周期的最优计划是“换到左车道”，我们将为实现相同操作的策略分配奖励。 因此，成本项表示为:
$$
F_n = \lambda^{user}_{n}b_{navi} +\lambda^{consist}b_{consist}
$$
注意到,安全成本 $F_s$和效率成本 $F_e$ 是针对每个语义级操作生成的。引入折扣因子γ来调整远期奖励的权重，我们将其设置为γ=0.7。