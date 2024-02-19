# C++11 多线程(std::thread)详解

## 线程、进程、多线程

### 什么是多线程

>多线程（multithreading），是指从软件或者硬件上实现多个线程并发执行的技术。具有多线程能力的计算机因有硬件支持而能够在同一时间执行多于一个线程，进而提升整体处理性能。
>在一个程序中，这些独立运行的程序片段叫作“线程”（Thread），利用它编程的概念就叫作“多线程处理”。

### 进程与线程的区别

定义：

> 进程是正在运行的程序的实例，而线程是进程中的实际运作的单位

区别：

- 一个程序有且只有一个进程，但可以拥有至少一个线程
- 不同进程有不同的地址空间，互不相干，而不同线程共同拥有相同进程的地址空间

![20210717195132759](E:\PnC\pnc_research\code\20210717195132759.png)

## C++11的std::thread

在C中已经有一个叫做pthread的东西来进行多线程编程，但是并不好用 （如果你认为句柄、回调式编程很实用，那请当我没说），所以c++11标准库中出现了一个叫作std::thread的东西。

### std::thread 常用成员函数

#### 构造&析构函数

|                             函数                             |      类别      |                        作用                        |
| :----------------------------------------------------------: | :------------: | :------------------------------------------------: |
|                      thread() noexcept                       |  默认构造函数  |              创建一个线程，什么也不做              |
| template <class Fn, class… Args><br/>explicit thread(Fn&& fn, Args&&… args) | 初始化构造函数 | 创建一个线程，<br/>以`args`为参数<br/>执行`fn`函数 |
|                thread(const thread&) = delete                |  复制构造函数  |                     （已删除）                     |
|                 thread(thread&& x) noexcept                  |  移动构造函数  |     构造一个与`x`<br/>相同的对象,会破坏`x`对象     |
|                          ~thread()                           |    析构函数    |                      析构对象                      |

#### 常用成员函数

|              函数               |                             作用                             |
| :-----------------------------: | :----------------------------------------------------------: |
|           void join()           |               等待线程结束并清理资源（会阻塞）               |
|         bool joinable()         |                 返回线程是否可以执行join函数                 |
|          void detach()          | 将线程与调用其的线程分离，彼此独立执行（此函数必须在线程创建时立即调用，且调用此函数会使其不能被join） |
|    std::thread::id get_id()     |                          获取线程id                          |
| thread& operator=(thread &&rhs) | 见移动构造函数 （如果对象是joinable的，那么会调用`std::terminate()`结果程序） |

### 例子

#### thread的基本使用

```C++
#include<iostream>
#include<thread>

using namespace std;
void doit(){
    cout<< "world"<<endl;
}
int main(){
    
    thread ({}{
        cout <<"hello, "<<flush;
    }),b(doit);
    a.join();
    b.join();
    return 0;
}
```

输出结果：

```plain
Hello, World!
1
```

或者是

```plain
World!
Hello,
```

那么，为什么会有不同的结果呢？
这就是多线程的特色！

多线程运行时是以异步方式执行的，与我们平时写的同步方式不同。异步方式可以同时执行多条语句。

在上面的例子中，我们定义了2个thread，这2个thread在执行时并不会按照一定的顺序。打个比方，2个thread执行时，就好比赛跑，谁先跑到终点，谁就先执行完毕。



### 深入剖析 std::thread

在`g++`中，`thread`是基于`pthread`实现的。本次主要从以下三个方面分`std::thread`

- `std::thread`对象不可复制，只具有移动属性
- 每个线程具有唯一的标志，即线程id
- 创建子线程

#### 移动属性

有很多书籍说，`std::thread`对象的所有权只能传递不能复制。实际上，就`std::thread`对象，只具有移动属性，不具有复制属性。`std::thread`的构造函数如下：

```c++
class thread{
    private:
    	id 			_M_id;
    public:
    	thread() noexcept = default;
    
    	template<typename _Callable,
    			 typename... _Args,
    			 typename = _Require<_not_same<_Callable>>>
        explicit thread(_Callable&& __f,_Args&&... __args){
                     //...
        }
    ~thread() {
      if (joinable())
        std::terminate();
    }
    // 禁止复制
    thread(const thread&) = delete;
    thread& operator=(const thread&) = delete;

    // std::thread 只具有移动属性
    thread(thread&& __t) noexcept
    { swap(__t); }

    thread& operator=(thread&& __t) noexcept {
      if (joinable())
          std::terminate();
      swap(__t);
      return *this;
    }
    //...
}
```

可以发现，`std::thread`禁止了复制构造函数、复制赋值表达式，只留下了移动构造函数、赋值，使得`std::thread`对象只能移动，不能复制。这就是本文开篇demo中使用`emplace_back`函数添加`std::thread`对象的原因，防止触发复制构造函数。

向`threadList`中添加`std::thread`对象，有如下三种方式：

```c++
  threadList.emplace_back(std::thread{do_some_work, idx});  // 1) ok 

  std::thread trd{do_some_work, idx};
  threadList.push_back(trd);               // 2) error
  threadList.push_back(std::move(td));     // 3) ok
  threadList.emplace_back(std::move(td));  // 4) ok
```

注意：当`push_back`接受的是右值时，底层调用的还是`emplace_back`函数，因此，`3)`和`4)`算是等价。



### std::thread::id

观察可发现，在`std::thread`对象中，只有一个成员变量`_M_id`：

```cpp
   id             _M_id;
```

这个类`id`全称是`std::thread::id`，实现如下：

```cpp
  typedef pthread_t native_handle_type;

  class id { 

    native_handle_type  _M_thread;  // _M_thread 即 pthread_t 对象，线程的唯一辨识标志
  public:
    id() noexcept : _M_thread() { }  // _M_thread 默认值是 0
    explicit id(native_handle_type __id) : _M_thread(__id) { }
  private:
    friend class thread;
    friend class hash<thread::id>;

    // 为 std::thread::id 对象重载了 == 运算
    friend bool operator==(thread::id __x, thread::id __y) noexcept;
    friend bool operator<(thread::id __x,  thread::id __y) noexcept;
    // 为 std::thread::id 对象重载了 << 操作
    template<class _CharT, class _Traits>
    friend basic_ostream<_CharT, _Traits>&
    operator<<(basic_ostream<_CharT, _Traits>& __out, thread::id __id);
  };
```

因此，这个`std::thread::id`实际上，就是封装了`pthread_t`对象，用作每个线程标志。

- 在构造`std::thread`对象的时候，如果没有设置线程入口函数，则线程`_M_id._M_thread`的值是0。

比如下面的demo中，`trd`没有设置线程入口函数，`trd`调用默认构造函数时，`trd`的`_M_id._M_thread`会被初始化为0。

```cpp
    int main(int argc, char const *argv[]) {

      std::thread trd;
      std::cout<<trd.get_id()<<std::endl;
      return 0;
    }
```

但是，打印线程标志`trd.get_id()`，输出的是却不是0。这仅仅是`std::thread::id`在重载`<<`操作符时的设定，用于提示调用者线程没有启动。

```cpp
    $ g++  thread_.cc -o thread_ && ./thread_
    thread::id of a non-executing thread
```















