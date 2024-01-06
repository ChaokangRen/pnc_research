类 simulator::IntelligentDriverModel:
成员变量：该类用来common下面的IDM类的成员函数（好奇的是为怎么不用继承来实现）
即Param和State
typedef boost::array<double, 4> InternalState;
InternalState internal_state_ //这样应该是更好的用于计算。

私有成员函数：
1. UpdateInternalState():更新internal_state_;
2. Linear(InternalState &x,double dt,InternalState *x_out):
该函数目的是利用IDM中的IIDM模型根据当前状态获取期望加速度，并线性递推出dt时间后的状态。

公有成员函数：
1. 默认构造函数
2. 参数构造函数，接受参数param
3. 析构函数
4. 获取状态函数
5. 设置状态函数
6. Setp(dt)函数：看着从当前状态在给定dt后的步进状态
7. operator():实现方式和Linear()一样。