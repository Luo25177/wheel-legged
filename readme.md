# wheelfoot code

## 通讯

### 下层通讯

can1 作为关节电机 **Tmotor** 的通讯

can2 作为足端轮子电机 **DJmotor** 的通讯

### 上层通讯
  
usart1 作为通讯的接口，使用蓝牙通讯

可以使用摇杆来控制速度，松手之后速度立即置零

打算是利用unity来做上位机，并且用游戏手柄做比赛的控制器，先试一试吧

每次发送的数据不能太多，要分开发，所以可以分别发送多种类型的参数，其中用 id 来判断信号类型

#### 规定

|0-1| 2 |3-n|n+1-n+2|
|---|---|---|-------|
|head|id|data|tail|

- head: 0xff 0xfe
- id:
  - 1 控制参数 ControlParam
  - 2 摇杆数据 HandleParam
  - 3 机器人状态参数 RobotState
  - 4
- data: 利用memcpy来实现复制
- tail: 0x0a 0x0d

## 控制

### 腿部的控制

![](./leg.png)

设定从右侧看机器人为正视角，并且以向右为机器人正方向

所以右方向的腿被认为是正方向的腿，右侧的腿与右侧的腿完全对称就可以直接反过来了，所以只需要一个方向就可以直接规定所有不一致的方向了，所以对于设置电机的力矩之类的，都需要乘以一个方向值了

#### 规定

- angle1为前方电机的角度

- angle4为后方电机的角度

- TF 即为前部电机的力矩，对应于angle1

- TB 即为前部电机的力矩，对应于angle4

- Tp 为关节处扭矩，是一个虚拟出来的力，定义逆时针为正

- 一般会把 angle1 初始化为180度，也就是在angle=180的地方作为零点，同样以 angle4 初始化为0度

### 电机控制

电机的正方向是可以人为改变的，我这里打算把电机的正方向设置成一致的，也就是正向轴端逆时针为正，这样对于解算来说，角度的正方向与电机的正方向一致，直接位控的话很方便，但是肯定不做位控()。电机最后只需要腿部的方向作为控制的方向了，对于需要反向的电机，需要在初始化的时候就设置反向

实际上，由于条件限制，电机正方向是逆时针为负，所以对于右侧电机，角度变化是负的，对于左侧电机，角度变化就是正的了

- **DJmotor**: 因为是通过PID来对输入进行解算的，所以只需要把输入做一个反向就好了，后面的输出都会是反向的

  id号规定
  - 0 右侧轮子电机
  - 1 左侧轮子电机

- **Tmotor**: 数据直接作用于电机，所以输出的正负直接影响电机的表现

  id号规定
  - 右前 1
  - 右后 2
  - 左前 3
  - 左后 4

### 机体控制

- lqr 控制器，很经典，但是控制上也会有很多缺陷，例如响应不够迅速，而且有些情况下就不适合使用了(腾空状态)，但是可以腾空之后只控制腿部的位姿

- 腾空检测 对地支持力的检测

- 机器人初始化的时候腿部需要一个限位块来确定电机的初始位置

- adrc 之后研究一下

## 参考

- [RoboMaster平衡步兵机器人控制系统设计](https://zhuanlan.zhihu.com/p/563048952)

- [RoboMaster平衡步兵机器人控制系统设计视频](https://www.bilibili.com/video/BV15W4y1i7Sf/?spm_id_from=333.337.search-card.all.click&vd_source=51da4e9c6aef240de92028196d31f7e6)

- [Balance Control of a Novel Wheel-legged Robot: Design and Experiments](https://www.researchgate.net/publication/355430284_Balance_Control_of_a_Novel_Wheel-legged_Robot_Design_and_Experiments)

- [A Two-Wheeled Jumping Robot](https://www.researchgate.net/publication/335144663_Ascento_A_Two-Wheeled_Jumping_Robot)

# 注意事项

## C语言结构体中数据的对齐

### 对齐方式

会以结构体中最大的数据类型来进行对齐，会造成内存的浪费

### 规定

- 使用 `#pragma pack(n)` 来进行定义，但是会导致使用内存读取速度降低
- 或者定义的时候要注意，将小的数据类型放在前面

## 踩过的坑 T_T

### 代码的优化等级需要注意

1. 优化等级一般选用 -O0，如果使用较高的优化等级 (-O2) 会导致调试的时候出问题，而且代码的执行顺序也不一定按照自己的需求来，也就是说代码怎么运行的只有上帝才知道了。。。。
2. 优化等级可能会导致板子下不进去代码，一定记得改好优化等级，建议 -O0，你要是nb也可以试试 -O2，毕竟可以优化代码运行效率

### 代码上的一些问题

1. can消息定义一定不要忘记，要把所有该定义的的全部定义完成，有可能导致CAN消息的id号错误和消息错误(这很致命)
2. 对于pid的计算，一定要注意，每次运行只进行一次运算，多次运算会导致一些问题
3. M3508电机控制频率问题，频率太低控制不是很好

### 代码运行速率的优化

1. 注意尽量写代码时规划好，别在同一时刻重复去做一件事
2. 对于整数的乘法，尽量使用移位运算代替，减少运行消耗的时间
3. 尽量别使用 `if-else if-else`，使用 `switch`
4. 尽量使用移位操作来代替乘法，除法

# README

写代码前提前规划好所有要写的和一些代码中需要的规定是个人习惯，并且在写代码的时候严格按照readme里面规定的架构来写，主要还是因为本人比较菜，又需要保证代码的结构和质量 T_T

这套代码里有之前写的模板类，所以感觉还是挺不错的，但是就是很难去debug，所以我是提前把功能调好了之后才把这段代码复制过来的，但是使用的时候要注意，并且记得在头文件里声明需要露出来的接口（感觉这点又有点繁琐，但是能够隐藏接口还是感觉挺不错的）。并且写这些模板类或者模板函数很好的一点就是能够减少很多重复的实现，使代码看起来更简洁（前提是写的没有bug）

代码更新中.....

# 调试寄录

## 12.4

第一次上车，连腿子都没搞好，寄T_T

## 12.5

下午调试，很有问题，还是之前的问题，还好我坚定代码流程没问题，最后是电机的底层代码写的有问题，蓝瘦。。。。。。改完就没问题了，就剩下调节PID了，nice!