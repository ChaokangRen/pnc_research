# 类 IntelligentDriverModel
该类的所有的成员包括成员函数都是公共的。

成员变量：
1. Param  
```
  struct Param {
    decimal_t kDesiredVelocity = 0.0;
    decimal_t kVehicleLength = 5.0;                   // l_alpha-1
    decimal_t kMinimumSpacing = 2.0;                  // s0
    decimal_t kDesiredHeadwayTime = 1.0;              // T
    decimal_t kAcceleration = 2.0;                    // a
    decimal_t kComfortableBrakingDeceleration = 3.0;  // b
    decimal_t kHardBrakingDeceleration = 5.0;
    int kExponent = 4;  // delta
  };
```
分别是 期望速度、车长、最小空间、标准加速度、舒适的减速度、硬的减速度
2. State
```
  struct State {
    decimal_t s{0.0};        // longitudinal distance
    decimal_t v{0.0};        // longitudinal speed
    decimal_t s_front{0.0};  // leading vehicle
    decimal_t v_front{0.0};

    State() {}
    State(const decimal_t &s_, const decimal_t &v_, const decimal_t &s_front_,
          const decimal_t &v_front_)
        : s(s_), v(v_), s_front(s_front_), v_front(v_front_) {}
  };
```
状态分别是自车和前车横纵向速度

成员函数：该模块有三个成员函数，均用于得到IDM模型下的期望加速度，不同的是一个用IDM模型、一个用IIDM模型，还有一个用ACC模型。该公式均可以在书Traffic Flow Dynamics中找出。也可以直接看IMD.md
