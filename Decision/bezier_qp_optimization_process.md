# 贝塞尔曲线的二次规划过程推导

考虑第$i$段的贝塞尔曲线的表达式：
$$
f^\sigma_j(t) = \alpha_j \cdot \sum^m_{i=0}p^2_i \cdot b^i_m(\frac{t-t_{j-1}}{\alpha_j})
$$
令$b^i_m(t)=\left(\matrix{m\\i}\right)t^i\cdot(1-t)^{m-i}$

令$u=\frac{t-t_{j-1}}{\alpha_j}$

**Note:** 首先我们考虑一个普通贝塞尔曲线的求导
$$
C(u)=\sum^n_{i=0}B_{n,i}(u)P_i
$$
有
$$
B_{n,i}(u)=\frac{n!}{i!(n-i)!}u^i(1-u)^{n-i}
$$
求导有：
$$
\frac{d}{du}B_{n,i}(u)=i\frac{n!}{i!(n-i)!}u^{i-1}(1-u)^{n-i} - 
(n-i)\frac{n!}{i!(n-i)!}u^{i}(1-u)^{n-i-1} = \\
n\frac{(n-1)!}{(i-1)!(n-i)!}u^{i-1}(1-u)^{n-i} -lk 
n\frac{(n-1)!}{i!(n-i-1)!}u^{i}(1-u)^{n-i-1}  = \\
n(B_{n-1,i-1(u)} - B_{n-1,i}(u))
$$
进一步有
$$
\frac{d}{du}C(u)=B'_{n,i}(u)=\sum^{n-1}_{i=0}B_{n-1,i}(u)\{n(P_{i+1}-P_i)\}
$$
接下来，我们推到五次贝塞尔曲线的三阶导。

已知，$C(u)=\sum^5_{i=0}B_{5,i}P_i$，先求一阶导数
$$
C'(u)=\sum^{4}_{i=0}B_{4,i}(u)\{5(P_{i+1}-P_i)\}
$$
接着令$\alpha^1_i=5(P_{i+1}-P_i)$，那么上式可以写成：
$$
C'(u)=\sum^{4}_{i=0}B_{4,i}(u)\alpha^1_i
$$
接着令$\alpha^2_i = 4(\alpha^1_{i+1}-\alpha^1_i)$,二阶导数为：
$$
C''(u)=\sum^{3}_{i=0}B_{3,i}(u)\alpha^2_i
$$
接着令$\alpha^3_i = 3(\alpha^2_{i+1}-\alpha^2_i)$,二阶导数为：
$$
C'''(u)=\sum^{2}_{i=0}B_{2,i}(u)\alpha^3_i
$$
令$a_i = \alpha^3_i$,有：
$$
C'''(u)=a_0 B_{2,0}+a_1 B_{2,1}+a_2 B_{2,2} = a_0(1-u)^2 + 2a_1(1-u)u + a_2u^2
$$
展开为
$$
C'''(u)=(a_0-2a_1+a_2)u^2 +2(-a_0+a_1)t+a_0=b_0u^2 + b_1u + b_0
$$
显然经过一系列简化后，三阶导数为简单的二次型，后面进一步的将代价函数转为二次型即可，后续工作较为简单，且之前已做过多次，这里不在进一步推导。