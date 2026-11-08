# ctrl

Galbot G1 真机控制与 RViz 可视化共用的 ROS 2 话题接口。

## 架构

真机后端和仿真后端订阅完全相同的话题：

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/ctrl/wheels` | `ctrl_msgs/WheelCommand` | 四个全向轮角速度 |
| `/ctrl/leg` | `ctrl_msgs/JointGroupCommand` | 腿部关节位置 |
| `/ctrl/head` | `ctrl_msgs/JointGroupCommand` | 头部关节位置 |
| `/ctrl/left_hand` | `ctrl_msgs/JointGroupCommand` | 左臂关节位置 |
| `/ctrl/right_hand` | `ctrl_msgs/JointGroupCommand` | 右臂关节位置 |
| `/ctrl/gripper` | `ctrl_msgs/GripperCommand` | 左右夹爪开口宽度 |

两种运行模式：

- `real`：启动 `real_robot_bridge`，通过 `galbot_sdk.g1` 控制真机。
- `sim`：启动 `rviz_joint_bridge`，将相同话题转换为 `/joint_states`。

应用层只需要发布上述共享话题，不需要关心当前使用真机还是仿真后端。

统一状态与对比话题：

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/ctrl/state/real` | `ctrl_msgs/RobotStateSnapshot` | 真机关节角与左右末端位姿 |
| `/ctrl/state/sim` | `ctrl_msgs/RobotStateSnapshot` | 仿真关节角与左右末端位姿 |
| `/ctrl/state/error` | `ctrl_msgs/RobotStateError` | 真机与仿真的逐关节及末端误差 |

末端位姿统一为 `base_link -> left_gripper_tcp_link` 和
`base_link -> right_gripper_tcp_link`。

## 构建

```bash
cd /home/lin/galbot
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src \
  --packages-select ctrl_msgs ctrl
source install/setup.bash
```

ROS 2 Humble 的接口生成必须使用系统 Python 3.10。如果当前终端已激活
Anaconda，请先切换上面的 `PATH`，或者指定：

```bash
-DPython3_EXECUTABLE=/usr/bin/python3
```

## 仿真与 RViz

```bash
ros2 launch ctrl ctrl.launch.py mode:=sim
```

该命令启动 `rviz_joint_bridge`、`robot_state_publisher` 和 RViz。
`rviz_joint_bridge` 是 `/joint_states` 的唯一发布者，不要同时启动
`joint_state_publisher`。

头部控制示例：

```bash
ros2 topic pub --once /ctrl/head ctrl_msgs/msg/JointGroupCommand \
"{name: [head_joint1, head_joint2], position: [0.2, 0.1], velocity: [], effort: []}"
```

设置默认位姿的测试：

```bash
ros2 run ctrl raise_body_lower_arms_test \
  --ros-args -p confirm:=true
```

该测试先将腿部移动到 Grab 站姿 `[0.172022, 0.841768, 0.569746, 0.0, 0.0]`，
等待后发布头部归中和默认双臂姿态。默认双臂保持 Grab 的 TCP 前后位置、
高度和朝向，双手 TCP 间距加大到 `0.30 m`（Grab 为 `0.21 m`）；测试开始时
还会将左右夹爪张到全开 `0.12 m`。仿真模式会直接显示在 RViz；真机模式
需要 `real_robot_bridge` 已使用 `allow_motion:=true` 启动。

Grab 测试：

```bash
ros2 run ctrl grab \
  --ros-args -p confirm:=true
```

该命令同时发布左右机械臂：

```text
right_arm: [-0.722343, 1.438776, 0.045585, 1.720401,
             1.632508, 0.574619, 0.510512]
left_arm:  [ 0.722343, -1.438776, -0.045585, -1.720401,
            -1.632508, -0.574619, -0.510512]
```

同时会将左右夹爪闭合到半开 `0.06 m`。
Grab 姿态左右 TCP 间距为 `0.21 m`。当前 Grab 目标在上一版基础上沿机体
前方再前移 `0.20 m`，即 `base_link` 系 x 从 `0.49 m` 变为 `0.69 m`。
左右 TCP 还绕各自局部 X 轴旋转 `180°`，用于避免深度相机与底盘碰撞。
腿部会同时发布 Grab 站姿
`[0.172022, 0.841768, 0.569746, 0.0, 0.0]`，比上一版抬高
`head_base_link` 约 `0.05 m`；Grab 末端 TCP 随身体一起抬高，
`base_link` 系 z 约为 `0.85 m`。

同样需要通过 `mode:=sim` 或 `mode:=real` 先启动控制后端。真机模式下必须
使用 `allow_motion:=true`。

Grab2 测试（更窄的双手间距）：

```bash
ros2 run ctrl grab2 \
  --ros-args -p confirm:=true
```

动作与 Grab 相同：腿部发布 Grab 站姿、夹爪半开 `0.06 m`，仅左右手目标
改为双手 TCP 间距 `0.18 m`（Grab 为 `0.21 m`）：

```text
right_arm: [-0.707379, 1.455216, 0.013305, 1.704717,
             1.618452, 0.603934, 0.514830]
left_arm:  [ 0.707379, -1.455216, -0.013305, -1.704717,
            -1.618452, -0.603934, -0.514830]
```

同样需要通过 `mode:=sim` 或 `mode:=real` 先启动控制后端。

Drop 测试：

```bash
ros2 run ctrl drop \
  --ros-args -p confirm:=true
```

执行顺序与 grabbox `drop()` 一致，需先执行 Grab：

1. 使用 21 个同步路点将双臂从 Grab 目标下降，腿部保持 Grab 站姿
2. 将腿部落到 `[0, 0, 0.3, 0, 0]`
3. 保持夹爪半开 `0.06 m`

双臂路点基于官方 URDF 模型生成，从当前 Grab 目标出发，沿 X-Z 平面斜线
向前移动 `0.052 m`、下降 `0.201 m`，左右 TCP 间距从 `0.21 m` 收窄到
`0.19 m`。腿部放下后末端 TCP 高度约 `0.072 m`（`base_link` 系）。
Drop 全程保持与 Grab 相同的局部 X 轴 `180°` 末端朝向。由于 Grab 站姿
已抬高 `0.05 m`，第 1 步手臂路径整体比上一版高 `0.05 m`，第 2 步腿部
下降行程相应增加 `0.05 m`；下放终点位置不变。

Drop 后扩手测试：

```bash
ros2 run ctrl drop_over \
  --ros-args -p confirm:=true
```

该状态先完整执行 Drop，再将左右 TCP 间距扩大到 `0.30 m`。

轮速顺序为 `wheel1_joint` 至 `wheel4_joint`。仿真后端会对轮速积分，得到
主动轮关节位置，同时发布每个轮子的 10 个被动滚子关节，确保 RViz 中轮子
转子 TF 完整。RViz 只显示轮子旋转；如果需要显示底盘平移，还需要额外的
里程计或 TF 数据源。

## 真机控制

```bash
source /home/lin/galbot/sdk/galbot_sdk/linux-x86_64-gcc940/setup.sh
ros2 launch ctrl ctrl.launch.py mode:=real allow_motion:=true rviz:=true
```

`allow_motion` 默认是 `false`。未显式设置为 `true` 时，真机后端会接收并校验
话题消息，但不会向机器人发送控制命令。

真机模式下 `rviz:=true` 会同时启动话题镜像节点、`robot_state_publisher`
和 RViz。镜像节点直接使用收到的控制话题发布 `/joint_states`，因此 RViz
显示的是控制目标状态，不是真机关节反馈。机器人端无图形界面时使用：

```bash
ros2 launch ctrl ctrl.launch.py mode:=real allow_motion:=true rviz:=false
```

启用真机运动前必须确认：

- 急停按钮已释放。
- 机器人处于正常工作模式。
- 机器人运动范围内无人和障碍物。
- 操作者位于急停按钮附近。
- 首次验证使用低速、小幅度，并保持急停可达。

真机后端使用非阻塞关节和夹爪命令。轮速后端使用官方 URDF 的四轮几何关系
转换为底盘速度，并在轮速话题超时后调用 `stop_base()`。

真机与仿真状态对比：

```bash
ros2 launch ctrl ctrl.launch.py \
  mode:=real allow_motion:=false rviz:=false compare_states:=true
```

该模式同时启动真机 SDK 状态读取、仿真状态镜像和 `state_comparator`。
`allow_motion:=false` 表示只读取和对比，不发送运动命令。查看误差：

```bash
ros2 topic echo /ctrl/state/error
```

## 消息规则

- `JointGroupCommand.name` 和 `position` 必须非空且长度一致。
- 关节名必须属于对应话题分组，例如 `/ctrl/left_hand` 只接受
  `left_arm_joint*`。
- 关节位置单位为弧度。
- 轮子角速度单位为弧度/秒。
- 夹爪宽度单位为米，范围为 `[0.0, 0.12]`。
- `JointGroupCommand.velocity` 和 `effort` 用于 RViz 显示；真机后端当前只
  通过 `set_joint_positions` 下发关节位置。
- 真机控制优先使用 SDK 中的 `G1JointGroup` 关节组名称。

## 测试

```bash
python3 -m pytest src/ctrl/test
```
