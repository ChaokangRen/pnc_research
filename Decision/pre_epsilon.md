
#论文4：Efficient Uncertainy-aware Decision-making for automated driving using guided branching

## Conditional Foucsed Branching
什么是CFB呢？翻译过来就是条件聚焦分支。具体分析如下：
算法动机：假如有n个agents，每个agent有m个可能的意图，那么总的意图组合就有 $m^n$个，显然组合会随着agent的数量增加而成指数增长，这对于实时计算来说负担是巨大的，因此需要剪去一些不关注的agnet或者其意图。
算法的指导思想：通过观察人类驾驶员的操作，会发现当行驶过程准备切换操作时，只会关注会对当前操作产生影响的相关车辆和相关意图。比如说，当准备执行向左变道时，驾驶员会更关注于左侧的道路信息而非右侧。因此，通过聚焦于自车的策略序列，我们可以选出一组与自车未来动作产生影响的相关车辆。挑选的过程是基于专家系统的规则算法，基于学习的算法将在后续的工作中持续进行。  
有了自车的策略序列后，就可以挑选出需要进一步检测的车辆了，我们不枚举该车辆子集的所有可能意图，而是引入初步安全检查来挑选出我们应该特别注意的车辆。初步安全评估是使用基于多种假设的开环正向仿真进行的。例如，如果有辆车它的意图是不确定的，我们会分别预测这辆车是车道保持还是变道的场景会是什么样子。对于没有通过安全检测的车辆，不同的场景将会进一步的在闭环仿真中进行检测。对于通过评估的车辆，我们使用初始信念的最大后验概率（MAP）。最终，意图空间的分支被引导进潜在的风险场景，实际上尽管它的设计简单，但是通过这种初级的安全检测可以识别出许多危险的场景。算法流程如下：

---

Algorithm 1:Process of EUDM
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
## Implementation Details
### A. Semantic-level Action
语义级动作分成横向动作和纵向动作:  
横向动作包括{LK,LCL,LCR}  
纵向动作包括{acc,maintain,dec}  
请注意，这些纵向动作不是离散控制信号，例如[10, 12]中的加速度命令，而是应用于正向仿真模型的连续期望速度。每个语义级动作的持续时间为2s，而闭环模拟以0.4秒的分辨率进行，以保持模拟保真度。注意，对于每个规划周期，正在进行的操作的持续时间会被每个规划周期的重新规划分辨率（0.05秒）扣除。DCP-Tree的深度是4，规划的总体时间是8s.
### B. Forward Simulation
采用intelligent driving model和pursuit controller分别作为纵向和横向的仿真模型。注入控制噪声以反映驾驶行为的随机特性。
### C.Beilef Update
在本文中，隐藏的意图包括横向行为如{LK,LCL,LCR}。这些agent的意图的belief是通过如图2所示的前向仿真中进行更新。
![EUDM Framework](/Decision/elements/pre_epsilon_figure2.png "Framework")   
在本文中，我们采用了基于规则的轻量级信念跟踪模块，该模块采用一组特征和指标，包括速度差、当前和相邻车道上领先和跟随代理的距离、责任敏感安全（RSS）[ 31]和换道模型[32]作为输入.信念跟踪器给以下意图(LK,LCL,LCR)生成概率分布。概率作为策略评估时的重要性权重。
### D. CFB Mechanism
CFB的第一步是关键车辆的选择，对于当前车道和相邻车道，我们根据本车当前的速度，沿着车道向前和向后搜索一定距离，并将该范围内的车辆标记为关键车辆。第二步：根据初始信念确定车辆意图选择。具体来说，我们挑选出三种意图的概率彼此接近的车辆作为不确定车辆。请注意，对于具有置信预测的车辆，我们选择MAP(maximum a posteriori)意图并使用 MAP 选择结果边缘化意图概率。第三步，使用开环仿真进行安全评估。对于无法通过评估的车辆，我们列举出它们所有的意图组合。每一种组合作为CFB选择的场景，并且计算该场景的概率。第四步是根据用户偏好挑选出前k个场景，我们进一步边缘化前k个场景中的概率，边际概率成为评估时 CFB 选择场景的权重。
### E. Polocy Evaluation
一条策略的总体奖励是CFB下所有场景奖励的权重和。奖励函数是用户定义的多个指标组成，包括效率(当前车速和期望速度之间的差),安全(自车与周围车辆的距离),连续性(避免频繁的更换策略)。