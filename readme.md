# wheelfoot code

## 通讯

### 下层通讯

can1 作为关节电机 **Tmotor** 的通讯

can2 作为足端轮子电机 **DJmotor** 的通讯

### 上层通讯
  
usart1 作为通讯的接口，使用蓝牙通讯

可以使用摇杆来控制速度，松手之后速度立即置零

打算是利用unity来做上位机，并且用游戏手柄做比赛的控制器，先试一试吧

## 控制

### 腿部的控制

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

电机的正方向是可以人为改变的，我这里打算把电机的正方向设置成一致的，也就是正向轴端逆时针为正，这样对于解算来说，角度的正方向与电机的正方向一致，直接位控的话很方便，但是肯定不做位控()。电机最后只需要腿部的方向作为控制的方向了

对于需要反向的电机，需要在初始化的时候就设置反向:

- **DJmotor**: 因为是通过PID来对输入进行解算的，所以只需要把输入做一个反向就好了，后面的输出都会是反向的

- **Tmotor**: 数据直接作用于电机，所以输出的正负直接影响电机的表现

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
