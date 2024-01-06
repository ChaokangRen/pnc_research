类 simulator::ContextIntelligentDriverModel  
基于上下文语义的IDM模型，也是epsionlon利用给自车用的加速度模型。  
成员变量：同样有Parm和State,新增变量：
```
struct CtxParam{
    decimal_t k_s = 0.5;
    decimal_t k_v = 2.0 * k_s;
    CtxParam() = default;
    CtxParam(const decimal_t &_k_s, const decimal_t &_k_v)
        : k_s(_k_s), k_v(_k_v) {}
}
```
```
  struct CtxIdmState {
    decimal_t s{0.0};        // longitudinal distance
    decimal_t v{0.0};        // longitudinal speed
    decimal_t s_front{0.0};  // leading vehicle
    decimal_t v_front{0.0};
    decimal_t s_target{0.0};
    decimal_t v_target{0.0};
  };
```
主要成员函数：  
operator()函数：  
1. 首先利用ACCIDM函数获取IMD下的期望加速度。
2. 计算 acc_track: $a_{track} = K_v(\dot s_{des} +K_s(s_{des}-s_{ego}) - \dot s_{ego})$
3. 这里 $a_{track} = min(max(a_{track,-1}),1.0)$并没有向文献中提到的用 $a_{track} = min(a_{track},a_{idm})$
4. 接下来对状态变量进行复制。

step(dt)函数：  
这个函数就是利用odeint求解operator()里面的模型。