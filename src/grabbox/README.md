# grabbox

无 ROS 依赖的 Galbot 机械臂状态控制包。它直接使用官方 `galbot_sdk.g1`
控制关节、夹爪和双臂同步轨迹。

## 安装

先加载官方 SDK 环境：

```bash
source /home/lin/galbot/sdk/galbot_sdk/linux-x86_64-gcc940/setup.sh
pip install -e /home/lin/galbot/src/grabbox
```

## 使用

`GalbotRobot` 和 `GalbotMotion` 在包顶层导出。先由调用方创建并初始化机器人，
再传入 `GrabBox`：

```python
from grabbox import GalbotRobot, GrabBox

robot = GalbotRobot()
if not robot.init():
    raise RuntimeError("GalbotRobot init failed")

try:
    box = GrabBox(robot)
    box.default()
    box.grab()
    box.drop()
finally:
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
```

四个状态函数：

- `default()`：默认抬升站姿、头部归中、双臂放下、夹爪全开 `0.12 m`；
  左右臂通过 `GalbotMotion.traj_plan()` 单路点协调规划。该姿态与官方
  G1 home 示例一致，因此同样关闭轨迹碰撞校验
- `grab()`：Grab 姿态，双手 TCP 间距约 `0.21 m`，夹爪半开 `0.06 m`
- `drop()`：不调用 Grab；确认双手 TCP 和身体高度处于 Grab 目标容差内后，
  降腿并沿 21 个同步路点下降到离地约 `0.08 m`
- `drop_over()`：完整执行 Drop 后，将左右 TCP 间距扩大到 `0.30 m`

`drop()` 默认要求左右 TCP 位置距离 Grab 目标不超过 `0.03 m`，
`head_base_link` 高度误差不超过 `0.03 m`。任一条件不满足时会在下发第一条
运动指令前报错。容差可通过 `drop_arm_tolerance_m` 和
`drop_body_tolerance_m` 调整。

抓取和下放姿态已与本地 URDF 限位核对，并把贴近 `left/right_arm_joint6`
机械限位的路点收进安全余量，避免控制器因关节限位锁存故障。

`drop()` 使用 `Trajectory` 和 `execute_joint_trajectory()` 同步控制左右臂，
避免两个独立关节目标之间的时序偏差。

`GrabBox` 默认会在内部创建并初始化 `GalbotMotion`。如果需要复用已有实例，
可以显式传入：

```python
from grabbox import GalbotMotion, GalbotRobot, GrabBox

robot = GalbotRobot()
motion = GalbotMotion()
robot.init()
if not motion.init():
    raise RuntimeError("GalbotMotion init failed")
box = GrabBox(robot, motion)
```

## 真机测试

```bash
source /home/lin/galbot/sdk/galbot_sdk/linux-x86_64-gcc940/setup.sh

python -m grabbox.real_test --state default
python -m grabbox.real_test --state grab
python -m grabbox.real_test --state drop
```

默认必须手动输入 `YES` 才会下发动作。`--yes` 只用于受控自动化环境。
程序发生异常时会调用 `stop_trajectory_execution()` 和 `stop_base()`，
最后释放 SDK 资源。
