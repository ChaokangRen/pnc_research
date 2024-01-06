## Vehicle类：
1. 成员变量
```C
int id_{kInvaildAgentId};
std::string subcalss_;
std::string type_;
VehicleParam param_;
State state_;
```
2. 成员函数
Vec3f RetDofState()  
功能： 获取车辆中心点的位姿x,y,yaw  
  
该类的主要功能是提供一个车辆的位姿、四个角的位置、前后保险杠的位置等等基本参数。

## GridMapND
```C
template <typename T,int N_DIM>
class GridMapND{
public:
    enum ValType{
        OCCUPIED = 70,
        FREE = 0,
        SCANNED_OCCUPIED = 128,
        UNKNOWN = 0
    }

    GridMapND();
}
```
成员函数：
```C++
ErrorType GridMap<T,N_DIM>::CheckIfEqualUsingGlobalPosition(const std::array<decimal_t,N_DIM> &p_w,const T &val_in,bool *res)const{
    std::array<int,N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
    T val;
    if(GetValueUsingCoordinate(coord,&val) != kSuccess){
        *res = false;
    }else{
        *res = (val == val_in);
    }
    return kSuccess;
}

template <typename T,int N_DIM>
std::array<int,N_DIM> GridMapND<T,N_DIM>::GetCoordUsingGlobalPosition(const std::array<decimal_t,N_DIM> &p_w)const{
    std::array<int,N_DIM> coord = {};
    for(int i = 0; i < N_DIM;++i){
        coord[i] = std::round((p_w[i]-origin_[i])/dims_resolution_[i]);
    }
    return coord;
}

```