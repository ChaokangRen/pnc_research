样条函数的基类为：
Polynomial
成员变量为：
五次多项式的通式写为：
$$
f(s) = a_0 + a_1*s + a_2*s^2 +a_3*s^3 + a_4*s^4 + a_5*s^5
$$
另一方面有泰勒公式可得：
$$
f(s) = f(0) +f'(0)s + \frac{f^{(2)}(0)}{2!}s^2 + \frac{f^{(3)}(0)}{3!}s^3+\frac{f^{(4)}(0)}{4!}s^4 + \frac{f^{(5)}(0)}{5!}s^5
$$
我们记：
$c_n = f^{(5 - n)}(0)$
有：
$$
f(s) = c_5 +c_4s + \frac{c_3}{2!}s^2 + \frac{c_2}{3!}s^3+\frac{c_1}{4!}s^4 + \frac{c_0}{5!}s^5
$$
```C++
1. VecNf coeff_; //系数
2. VecNf coeff_normal_order_;//相应阶次
3. EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign);
//构造函数
Polynomial(){set_zero();}//默认全部构造为0
Polynomial(const VecNf& coeff):coeff_(coeff){update();}

void update(){
    for(int i = 0;i < N_DEG+1;i++){
        coeff_normal_order_[i] = coeff_[N_DEG -i]/fac(i);
        // coeff_normal_order[i] = a[5 - i] * i!;
    }
}
void set_zero(){
    coeff_.setZero();
    coeff_normal_order_.setZero();
}
//该函数主要作用求任意阶次的多项式函数的值
inline decimal_t evaluate(const decimal_t &s,const int &d)const{
    //use horner's rule for quick evaluation
    decimal_t p = coeff_(0) / fac(N_DEG - d);
    for(int i = i;i < N_DEG-d;i++){
        p = (p * s +coeff(i)/fac(N_DEG-i-d));
    }
    return p;
}

inline decimal_t J(decimal_t s,int d)const{
//该函数主要功能是计算Jerk函数平方后的积分或者加速度函数平方后的积分   
}

void GetJerkOptimalConnection(p1,dp1,ddp1,p2,dp2,ddp2,s){
    //就是通过两点来确定多项式方程
}
```

类：PolynomialND
成员变量：
`std::array<Polynomial<N_DEG>,N_DIM> polys_;`:表示二维的多项式曲线。
该函数主要通过使用Polynomial类来实现二维的操作。

类Spline:
`typedef PolynomialND<N_DEG,N_DIM> PolynomialType`:这里构造成xy平面的多项式

私有成员：
```C++
vec_E<PolynomialType> poly_; //一个样条曲线可能有多段多项式组成
std::vector<decimal_t> vec_domain_;//应该理解为s被分成了多少段，如总长120m,20m一段，[0,20,40,60,80,100,120]
```
成员函数:就是对一系列对样条曲线的操作，比如获取某点的值、微分、加速度等等