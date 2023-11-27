using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class masterparam
{
  //----
  // @brief 摇杆数据
  // 
  //----
  public struct HandleParam
  {
    short run;
    short turn;
    short height;
    short tilt;
  };

  //----
  // @brief 控制参数
  // 
  //----
  public struct ControlParam
  {
    bool begin; // 开始运行
    bool stop; // 急停 打算做一个功能就是为了防止程序疯跑，再怎么说没有急停开关好用
    byte robotmode;
  };

  //----
  // @brief 主控参数
  // 
  //----
  public struct Master
  {
    HandleParam handle;
    ControlParam control;
  };

  //----
  // @brief 机器人状态参数，供主控查看
  // 
  //----
  public struct RobotState
  {
    byte deviceState; // 电机状态
    float v;
    float height;
    float pitch;
    float yaw;
    float roll;
  };
}