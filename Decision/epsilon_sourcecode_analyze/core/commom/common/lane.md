类lane:
成员变量：
```C++
1. SplineType position_spline_; //样条曲线
2. bool is_vaild_ = false;
```

关于 SplineType:
```C++
typedef Spline<LaneDegree,LaneDim> SplineType
#define LaneDegree 5
#define LaneDim 2
//可见Spline定义的是二维的5次多项式
```

构造函数：
默认构造函数：啥都不干，默认处理
```C 
Lane(const SplineType& position_spline) : position_spline_(position_spline), is_valid_(true) {} 
``` 

```C++
//根据给定s计算曲率
//arc_length指的是当前样条曲线上的某一点到起点的长度
ErrorType GetCurvatureByArcLength(const decimal_t &arc_length,decimal_t* curvature)const{
    //1. 判断输入的arc_length是否合法
    //2. 计算当前点的速度
    //3. 计算当前点的加速度
    //4. 计算曲率
}
//计算曲率和曲率微分
ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature,
                                    decimal_t* curvature_derivative) const;
```