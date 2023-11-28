# wheelfoot手机蓝牙控制器

## 通讯规定

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
