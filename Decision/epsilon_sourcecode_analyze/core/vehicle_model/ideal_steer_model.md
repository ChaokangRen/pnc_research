类：simulator::IdealSteerModel
该类的模型部分与vehicel_model一样，使用的带前轮转角的自行车模型。区别在与多了对前轮转角的精确控制。
主要函数为：
void IdealSteerModel::TruncateControl(const decimal_t &dt);
1. 计算期望纵向加速度 $a_{des} = v_{ctrl} - v_{state}$
2. 计算纵向的jerk： $jerk = (a_{des} - a_{state})/dt$
3. 卡一下jerk的阈值
4. 重新计算期望的加速度：$jerk*dt + a_{state}$
5. 卡一下加速度的阈值
6. 重新计算控制速度 $v_{ctrl} = max(v_{state}+a_{des} *dt ,0)$
7. 计算期望横向加速度：$a_{latdes} = \frac{v^2_{ctrl}  tan(\delta)}{l}$
8. 计算当前的横向加速度: $a_{latori} = v^2_{state} * \kappa$
9. 计算横向jerk: $jerk_{lat} = (a_{latdes} - a_{latori})/dt
10. 卡一下jerk
11. 利用jerk在重新计算横向加速度 $a_{latdes}$
12. 计算控制转角： $\delta_{ctrl}=arctan\frac{(a_{latdes} l)}{max(v^2_{ctrl},0.1 \epsilon_{big})}$
13. 计算控制转角微分： $\dot \delta_{ctrl} = \delta_{ctrl} - \delta_{state})/dt
14. 卡一下微分
15. 重新计算控制转角。

void IdealSteerModel::Step(double dt):
该函数首先通过TruncateControl()函数来重新计算横向和纵向的期望加速度，主要避免jerk值过大。然后在通过微分方程求解带转角的动力学方程。