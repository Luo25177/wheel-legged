# wheelfoot code

## 通讯

### 下层通讯

can1 作为关节电机 **Tmotor** 的通讯
can2 作为足端轮子电机 **DJmotor** 的通讯

### 上层通讯
  
usart1 作为通讯的接口，使用蓝牙通讯
可以使用摇杆来控制速度

## 控制

### 电机控制

对于需要反向的电机，需要在初始化的时候就设置反向:

- **DJmotor**: 因为是通过PID来对输入进行解算的，所以只需要把输入做一个反向就好了，后面的输出都会是反向的

- **Tmotor**: 数据直接作用于电机，所以输出的正负直接影响电机的表现

### 机体控制

## 参考

- [RoboMaster平衡步兵机器人控制系统设计](https://zhuanlan.zhihu.com/p/563048952)

- [RoboMaster平衡步兵机器人控制系统设计视频](https://www.bilibili.com/video/BV15W4y1i7Sf/?spm_id_from=333.337.search-card.all.click&vd_source=51da4e9c6aef240de92028196d31f7e6)

- [Balance Control of a Novel Wheel-legged Robot: Design and Experiments](https://www.researchgate.net/publication/355430284_Balance_Control_of_a_Novel_Wheel-legged_Robot_Design_and_Experiments)

- [A Two-Wheeled Jumping Robot](https://www.researchgate.net/publication/335144663_Ascento_A_Two-Wheeled_Jumping_Robot)
