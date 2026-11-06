# Galbot G1 Python 开发 Agent 指南

本文件面向在本仓库中编写、修改或审查 Galbot G1 Python 程序的编码 Agent。

## 1. 范围与事实来源

- 机器人：Galbot G1。
- SDK：Galbot SDK 1.10.0，Python API。
- 兼容机器人系统：GBS 1.18.x。
- 仅讨论 Python；不要主动引入 C++、交叉编译或 CMake 方案。
- 在线文档：<https://developer.galbot.com/docs/SDK/1.10.0/g1/zh/>
- Python API：<https://developer.galbot.com/docs/SDK/1.10.0/g1/zh/api_python_reference/>
- 本地文档：`GalbotSDK/docs/g1/zh/`
- 当前机器安装目录：`/home/lin/galbot/sdk/`
- 类型定义优先参考安装目录中的 `galbot_sdk.pyi`。
- 可运行示例位于 `GalbotSDK/examples/g1/python/`。
- 文档、`.pyi` 与示例冲突时，以版本化在线文档和 `.pyi` 为准；示例主要用于参考调用顺序。

## 2. 导入与环境

Python 程序统一从 G1 命名空间导入：

```python
import galbot_sdk.g1 as gm
from galbot_sdk.g1 import (
    ControlStatus,
    G1ControllerName,
    G1JointGroup,
    GalbotMotion,
    GalbotNavigation,
    GalbotRobot,
    MotionStatus,
    SensorType,
)
```

G1 包装类会自动绑定 `MachineType.G1`，不要自行传入机型：

```python
robot = GalbotRobot()
motion = GalbotMotion()
navigation = GalbotNavigation()
```

当前本机运行前：

```bash
source /home/lin/galbot/sdk/galbot_sdk/linux-x86_64-gcc940/setup.sh
```

必须使用 `source`，不能用 `bash setup.sh`，否则环境变量不会进入当前进程。

版本检查：

```bash
galbot_sdk check-version
galbot_sdk check-version --list
```

常见 Python 依赖：`numpy`、`opencv-python`、`scipy`。应用代码不要像教程示例一样在导入失败时执行 `pip install`；依赖应由部署环境预先安装。

## 3. 必须先遵守的安全契约

真实机器人动作具有物理风险。除非用户明确要求并确认现场安全，否则 Agent 不得自动运行会驱动机器人的程序。

运行任何真实动作前必须确认：

1. 急停按钮已释放，机器人处于正常工作模式。
2. 机器人周围至少 2 米无人、无障碍，机械臂运动范围内无遮挡。
3. 操作者靠近急停按钮，并明确批准本次动作。
4. 首次验证使用小幅度、低速、阻塞模式；确认方向和坐标后再扩大范围。
5. 所有运动、导航、夹爪命令都检查返回状态，禁止忽略失败后继续执行。
6. 异常时必须停止当前任务，并尽可能恢复安全姿态或释放控制。

推荐异常清理顺序：

```python
try:
    ...
finally:
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
```

运动或导航异常时，根据当前任务优先调用：

```python
robot.stop_trajectory_execution()
robot.stop_base()
navigation.stop_navigation()
```

软件急停与恢复：

```python
robot.emergency_stop()
# 仅在确认现场安全后
robot.resume_from_emergency_stop()
```

不要默认关闭碰撞检查。只有用户明确要求、且已说明风险和替代保护措施时，才允许：

```python
enable_collision_check=False
params.set_check_collision(False)
```

## 4. 生命周期与单例约束

`GalbotRobot`、`GalbotMotion`、`GalbotNavigation`、`GalbotPerception` 都是单例。

- `robot.init()` 每个进程只应执行一次。
- `motion.init()` 必须在任何运动规划调用前执行；失败后不要继续调用运动 API。
- `navigation.init()` 必须在导航 API 前执行，且只执行一次。
- `robot.destroy()` 是最终清理；调用后同一进程不能重新初始化 SDK。
- 需要重新初始化时，退出并启动新进程。
- `wait_for_shutdown()` 会阻塞到 `is_running()` 变为 false。

标准入口模板：

```python
from __future__ import annotations

import time

from galbot_sdk.g1 import ControlStatus, GalbotRobot


def main() -> int:
    robot = GalbotRobot()
    if not robot.init():
        print("GalbotRobot init failed")
        robot.destroy()
        return 1

    try:
        time.sleep(1.0)
        # Read-only checks first.
        print(robot.get_joint_names())
        print(robot.get_joint_positions(["head"]))
        return 0
    finally:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()


if __name__ == "__main__":
    raise SystemExit(main())
```

## 5. 单位、坐标与数据结构

- 角度：弧度。
- 长度：米。
- 时间：秒。
- 时间戳：纳秒。
- 位姿：`[x, y, z, qx, qy, qz, qw]`。
- 四元数顺序：`x, y, z, w`，使用单位四元数。
- 速度：`linear=[vx, vy, vz]`，`angular=[wx, wy, wz]`。
- 机器人默认平面姿态为 `[x, y, z, 0, 0, 0, 1]`。

常用 G1 关节组：

```python
G1JointGroup.chassis
G1JointGroup.head
G1JointGroup.left_arm
G1JointGroup.right_arm
G1JointGroup.leg
G1JointGroup.left_gripper
G1JointGroup.right_gripper
G1JointGroup.left_dexhand
G1JointGroup.right_dexhand
G1JointGroup.left_suction_cup
G1JointGroup.right_suction_cup
```

常用关节命名：

- 腿：`leg_joint1` 至 `leg_joint5`
- 头：`head_joint1` 至 `head_joint2`
- 左臂：`left_arm_joint1` 至 `left_arm_joint7`
- 右臂：`right_arm_joint1` 至 `right_arm_joint7`
- 夹爪：`left_gripper_joint1`、`right_gripper_joint1`

不要假设所有机型的关节数量或顺序相同。涉及精确顺序时：

```python
joint_names = robot.get_joint_names()
positions = robot.get_joint_positions([], joint_names)
```

`joint_names` 优先于 `joint_groups`。指定 `joint_names` 后，返回值和命令按该列表的精确顺序解释。

## 6. Python API 地图

### 6.1 GalbotRobot

初始化和生命周期：

- `init(enable_sensor_set=..., enable_sync_mode=False) -> bool`
- `is_running() -> bool`
- `request_shutdown()`
- `wait_for_shutdown()`
- `destroy()`

关节：

- `get_joint_group_names()`
- `get_joint_names(only_active_joint=True, joint_groups=[])`
- `get_joint_positions(joint_groups=[], joint_names=[])`
- `get_joint_states(joint_groups=[], joint_names=[])`
- `set_joint_positions(...)`: 低频关键帧/姿态控制。
- `set_joint_commands(...)`: 高频逐帧控制，不插值。
- `execute_joint_trajectory(trajectory, is_blocking=True)`
- `check_trajectory_execution_status(...)`
- `stop_trajectory_execution()`

末端与底盘：

- `set_gripper_command(end_effector, width_m, velocity_mps, effort, is_blocking)`
- `set_suction_cup_command(end_effector, activate)`
- `set_dexhand_command(...)`
- `set_base_velocity(linear_velocity, angular_velocity, duration_s=0.0)`
- `set_base_pose(...)`
- `stop_base()`

传感器：

- `get_rgb_data(camera_id, RgbOutputFormat.JPEG, once=True)`
- `get_depth_data(camera_id)`
- `get_ir_data(camera_id)`
- `get_synced_observation(cameras, with_joint_state=True)`
- `get_imu_data(sensor_id)`
- `get_odom()`
- `get_lidar_data(sensor_id)`
- `get_ultrasonic_data(ultrasonic_type)`
- `get_force_sensor_data(sensor_type, calibrated=False, ref_frame="")`
- `get_camera_intrinsic(camera_id)`
- `get_sensor_extrinsic(sensor_id, reference_frame="base_link")`
- `subscribe_video_data(camera_id, callback)`
- `unsubscribe_video_data(camera_id)`

状态、配置与控制权：

- `get_device_information()`
- `get_bms_information()`
- `get_config(service, keys, use_default=False)`
- `set_config(service, fields)`
- `acquire_controller()`、`release_controller()`
- `switch_controller()`、`start_controller()`、`stop_controller()`
- `reload_controller()`

### 6.2 GalbotMotion

所有运动规划前必须调用 `motion.init()`。

运动学：

- `forward_kinematics(...)`
- `inverse_kinematics(...)`
- `inverse_kinematics_general(...)`
- `get_end_effector_pose_on_chain(...)`

运动执行：

- `set_end_effector_pose(...)`: 笛卡尔目标的高级接口。
- `motion_plan(...)`: 采样路径搜索；末端路径形状不保证为直线。
- `traj_plan(...)`: 关节空间插值；末端路径不可预测。
- `move_line(...)`: 笛卡尔直线插值，适合精确接近/撤退。
- `motion_plan_multi_waypoints(...)`
- `combine_plan(...)`

规划模型：

- `attach_tool(chain, tool)`
- `detach_tool(chain)`
- `add_obstacle(...)`、`remove_obstacle(...)`、`clear_obstacle(...)`
- `attach_target_object(...)`、`detach_target_object(...)`
- `get_supported_chains()`、`get_supported_links()`
- `get_supported_ee_frames()`、`get_supported_tool_list()`
- `get_motion_plan_config()`、`set_motion_plan_config(...)`

重要规则：

- `end_effector_frame` 表示运动链，通常是 `"left_arm"` 或 `"right_arm"`，不是任意连杆名。
- `attach_tool()` 只更新运动学和碰撞模型；若目标是工具 TCP，必须显式 `params.set_tool_pose(True)`。
- 笛卡尔规划中不要把 `"leg"` 作为主动链或辅助链；腿部关键帧使用 `set_joint_positions`。
- 运动规划碰撞检查只考虑自碰撞和显式添加的障碍物，不自动读取实时点云或导航地图。
- 直接执行时不要传与真实状态冲突的 `start_state`/`reference_robot_states`，通常保留 `None`。
- `move_line` 可能因奇异点、关节限位、不可达中间点或 IK 不连续失败；必须处理 `MotionStatus`。

### 6.3 GalbotNavigation

典型流程：

```python
if not navigation.init():
    raise RuntimeError("navigation init failed")

if not navigation.is_localized():
    navigation.relocalize(init_pose)

current = navigation.get_current_pose()
if not navigation.check_path_reachability(goal_pose, current):
    raise RuntimeError("goal unreachable")

status = navigation.navigate_to_goal(
    goal_pose,
    enable_collision_check=True,
    is_blocking=True,
    timeout=30.0,
)
if not navigation.check_goal_arrival():
    raise RuntimeError(f"navigation failed: {status}")
```

规则：

- 导航前必须确认 `is_localized() == True`。
- `navigate_to_goal()` 默认非阻塞，返回状态表示命令是否被接受，不表示已到达。
- 非阻塞导航必须轮询 `check_goal_arrival()` 或 `get_navigation_status()`，并设置总超时。
- `navigate_to_goal_v2()` 的 `max_vel=[vx, vy, vyaw]` 每项范围为 `[0.05, 1.5]`。
- `stop_navigation()` 会减速停车，但停止位置可能不同于发起位置。
- 调用 `relocalize()` 时机器人应保持静止；之后用 `is_localized()` 验证。
- 部分导航示例会切换到 `G1ControllerName.CHASSIS_POSE_CTRL`；仅在对应流程需要时使用，并检查返回值。

### 6.4 GalbotPerception

```python
perception = GalbotPerception()
if not perception.init({gm.PerceptionModule.LIGHT_STEREO}):
    raise RuntimeError("perception init failed")

time.sleep(10)  # 文档要求：模型加载后等待约 10 秒
if not perception.run_once(gm.PerceptionModule.LIGHT_STEREO):
    raise RuntimeError("perception inference failed")
if not perception.wait_for_new_result(gm.PerceptionModule.LIGHT_STEREO, 5.0):
    raise RuntimeError("perception timeout")

success, result = perception.get_latest_result(gm.PerceptionModule.LIGHT_STEREO)
```

可用模块：`LIGHT_STEREO`、`FOUNDATION_STEREO`。

GBS 1.15.16 及以上若感知服务异常，先检查 HPU：

```text
/data/galbot/config/galbot_perception_moduleConfig.yaml
```

按文档将 `sensor_data_sync_max_diff_tolerance` 调整为 `500000` 纳秒并重启 `launcher`，不要由应用代码自动修改该配置。

## 7. 常用实现范式

### 7.1 低速关节关键帧

```python
status = robot.set_joint_positions(
    joint_positions=[0.0, 0.0],
    joint_groups=["head"],
    joint_names=[],
    is_blocking=True,
    speed_rad_s=0.1,
    timeout_s=10.0,
)
if status != ControlStatus.SUCCESS:
    raise RuntimeError(f"set_joint_positions failed: {status}")
```

优先使用语义组或明确关节名，避免依赖全关节默认顺序。

### 7.2 末端位姿运动

```python
params = gm.Parameter()
params.set_direct_execute(True)
params.set_blocking(True)
params.set_timeout(20.0)
params.set_check_collision(True)
params.set_reference_frame("base_link")
params.set_actuate("with_chain_only")
params.set_tool_pose(False)

status = motion.set_end_effector_pose(
    target_pose=target_pose,
    end_effector_frame="left_arm",
    reference_frame="base_link",
    enable_collision_check=True,
    is_blocking=True,
    timeout=20.0,
    params=params,
)
```

需要笛卡尔直线接近时，使用 `motion.move_line()` 或 `params.set_move_line(True)`，不要使用 `traj_plan` 代替。

### 7.3 关节轨迹

Python 绑定对象的 list 属性返回副本，对副本 `append()` 不会写回 SDK 对象。必须先在普通 Python list 中构造，再整体赋值：

```python
trajectory = gm.Trajectory()
trajectory.joint_groups = ["head"]
trajectory.joint_names = []

points = []
for timestamp_s, positions in samples:
    point = gm.TrajectoryPoint()
    point.time_from_start_second = timestamp_s

    commands = []
    for position in positions:
        command = gm.JointCommand()
        command.position = position
        commands.append(command)

    point.joint_command_vec = commands
    points.append(point)

trajectory.points = points
```

该规则适用于所有 SDK 绑定对象的 list 属性，包括 `joint_groups`、`joint_names`、`joint_commands`、`group_commands`、`subtask_commands` 等。

### 7.4 高频逐帧控制

`set_joint_commands()` 适合 VLA/RL 等逐帧绝对关节命令：

- 它不会从当前位姿插值到首帧目标。
- 首帧与当前关节角差异过大可能造成快速突变。
- 标准关节当前只使用 `position`；速度和力矩字段被忽略。
- 夹爪支持位置、速度、力矩字段。
- 控制循环必须固定频率、校验维度与 `NaN/Inf`，并提供停止条件。
- 同一关节组不要并发发送其他运动命令。

### 7.5 RGB、深度和同步观测

初始化时只启用实际使用的传感器，以缩短启动时间：

```python
robot.init(
    {
        SensorType.LEFT_ARM_CAMERA,
        SensorType.LEFT_ARM_DEPTH_CAMERA,
    }
)
```

同步模式必须显式开启：

```python
robot.init(
    {
        SensorType.HEAD_LEFT_CAMERA,
        SensorType.LEFT_ARM_CAMERA,
        SensorType.RIGHT_ARM_CAMERA,
    },
    True,
)
```

RGB：

```python
frame = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA, gm.RgbOutputFormat.JPEG)
if not frame:
    raise RuntimeError("camera frame unavailable")
image = cv2.imdecode(np.frombuffer(frame["data"], np.uint8), cv2.IMREAD_COLOR)
```

深度：

```python
depth = robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)
if not depth:
    raise RuntimeError("depth frame unavailable")
depth_m = (
    np.frombuffer(depth["data"], dtype=np.uint16)
    .reshape(depth["height"], depth["width"])
    .astype(np.float32)
    / depth["depth_scale"]
)
```

同步观测规则：

- `cameras[0]` 是锚点相机，必须已启用。
- 同步 RGB 是 CPU 持有的紧凑 NV12，不是 JPEG。
- `joint_state.joint_state_vec` 中每项包含 `joint_name`，应使用名称建立映射。
- 同步是软件时间戳最近邻匹配，不保证最大时间差；严格同步场景必须自行检查时间戳。

### 7.6 坐标变换

`robot.get_transform(target_frame, source_frame)` 返回 `(pose, timestamp_ns)`，`pose` 为目标坐标系下的 7 维位姿。

```python
pose, stamp_ns = robot.get_transform("base_link", "left_arm_camera_color_optical_frame")
if len(pose) != 7 or stamp_ns == 0:
    raise RuntimeError("transform unavailable")
```

不要只写 `if not pose[0]` 判断失败；有效的 `x` 可以等于 0。

### 7.7 夹爪与工具

G1 夹爪命令范围：

- 宽度：`0.0` 至 `0.12` m。
- 速度：`(0.0, 0.2]` m/s。
- 力矩：`(0.0, 100]` N·m。

抓取前根据实物确认末端是夹爪、吸盘还是灵巧手，不要仅按默认配置假设。抓取规划模型与真实硬件状态不一致时，先停止任务并修正配置。

## 8. 异步、超时与重试

- 阻塞调用必须设置合理超时，不能无限等待。
- 非阻塞调用只代表任务已提交。必须监控状态并设置总超时。
- 重试必须有次数上限，并在两次运动尝试之间重新读取机器人状态。
- `set_base_velocity(duration_s > 0)` 在窗口结束时不会自动发送停止命令；若应用需要确定停车，应使用单次发送并显式调用 `stop_base()` 或相应停止逻辑。
- 回调函数必须短小、非阻塞；耗时工作放入队列由业务线程处理。
- 视频订阅和麦克风流在退出前应取消订阅/停止流。

## 9. 状态处理

控制类 API 通常返回 `ControlStatus`，运动类 API 通常返回 `MotionStatus`。只把 `SUCCESS` 视为成功：

```python
if status != gm.ControlStatus.SUCCESS:
    raise RuntimeError(f"control failed: {status.name}")

if status != gm.MotionStatus.SUCCESS:
    raise RuntimeError(f"motion failed: {status.name}")
```

常见非成功状态包括：

- `TIMEOUT`
- `FAULT`
- `INVALID_INPUT`
- `INIT_FAILED`
- `IN_PROGRESS`
- `STOPPED_UNREACHED`
- `DATA_FETCH_FAILED`
- `PUBLISH_FAIL`
- `COMM_DISCONNECTED`

导航非阻塞流程优先轮询 `NavigationTaskStatus.SUCCESS`、`FAILED`、`INTERRUPTED`、`COLLISION` 等终态，不要只看一次返回值。

## 10. Agent 编码规则

- 新代码使用 `from __future__ import annotations` 和明确类型标注。
- 使用 `G1JointGroup`、`G1ControllerName`、`SensorType` 等常量，不散落魔法字符串。
- 将“参数校验”“运动规划”“硬件执行”“状态检查”拆成小函数。
- 所有位姿和关节向量在调用 SDK 前检查长度、有限值和四元数范数。
- 不在库代码或生产入口中调用 `os.system("pip install ...")`。
- 不吞掉 SDK 返回码，不只打印错误后继续运动。
- 不默认复制教程中的固定姿态到新场景；这些姿态只适用于对应示例。
- 不在未确认硬件型号、末端工具和地图状态时假设夹爪/吸盘、相机或导航可用。
- 不引入 ROS 或其他框架作为 Python SDK 的替代层，除非用户明确要求。
- 修改现有示例时保持原有导入方式和 G1 命名空间。
- 只改与任务相关的文件，不重构无关示例或文档。

## 11. 验证顺序

Agent 完成代码后按以下顺序验证：

1. 静态检查：`python -m py_compile <file>`。
2. 导入检查：`python -c "import galbot_sdk.g1"`。
3. 离线逻辑测试：测试坐标变换、四元数归一化、数组维度、状态分支和异常清理。
4. 真实机器人只读验证：初始化、读取关节、相机、TF、设备信息。
5. 真实机器人无负载小幅动作：低速、阻塞、短超时，并有人守急停。
6. 完整任务验证：确认地图、定位、工具、障碍物和恢复逻辑。

没有真实机器人或明确授权时，不要执行第 4 至第 6 步。可以运行静态检查和纯软件测试，并明确说明硬件验证未执行。

## 12. 诊断信息

- SDK 日志：`~/galbot_sdk_log/`
- 急停释放且模式正常仍无响应：检查 XCU 上的 `singorix` 和 `/userdata/log/SingoriX/info/singorix_wbcs_main.INFO`
- 底盘不动：检查 `galbot_chassis_service`；SOC 低于 25% 会触发底盘保护。
- 导航、定位、规划、感知异常：优先检查 HPU 服务。
- 运动控制、末端控制、模式异常：优先检查 XCU。
- PC 远程连接异常时，同时检查 PC 与机器人两侧的 `embosa_ip_config.json`。

## 13. 常用参考文件

- API 类型：`/home/lin/galbot/sdk/galbot_sdk/linux-x86_64-gcc940/lib/python/galbot_sdk/galbot_sdk.pyi`
- G1 包装：`/home/lin/galbot/sdk/galbot_sdk/linux-x86_64-gcc940/lib/python/galbot_sdk/g1/__init__.py`
- 基础控制：`GalbotSDK/examples/g1/python/tutorials/example1_galbot_control_started.py`
- 手臂控制：`GalbotSDK/examples/g1/python/tutorials/example2_arm_manipulation.py`
- 导航：`GalbotSDK/examples/g1/python/tutorials/example3_robot_navigation.py`
- 传感器：`GalbotSDK/examples/g1/python/tutorials/example4_sensor_data_collect.py`
- VLA/高频控制：`GalbotSDK/examples/g1/python/tutorials/example5_execute_vla.py`
- 综合抓放：`GalbotSDK/examples/g1/python/tutorials/example6_pick_and_place.py`
- 单功能示例：`GalbotSDK/examples/g1/python/galbot_robot/`
- 运动示例：`GalbotSDK/examples/g1/python/galbot_motion/`
- 导航示例：`GalbotSDK/examples/g1/python/galbot_navigation/`
- 感知示例：`GalbotSDK/examples/g1/python/galbot_perception/`

## 14. 统一真机 / RViz 控制接口

工作区中的 `src/ctrl` 和 `src/ctrl_msgs` 提供同一套 ROS 2 控制话题：

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/ctrl/wheels` | `ctrl_msgs/WheelCommand` | 四轮角速度 |
| `/ctrl/leg` | `ctrl_msgs/JointGroupCommand` | 腿部关节位置 |
| `/ctrl/head` | `ctrl_msgs/JointGroupCommand` | 头部关节位置 |
| `/ctrl/left_hand` | `ctrl_msgs/JointGroupCommand` | 左臂关节位置 |
| `/ctrl/right_hand` | `ctrl_msgs/JointGroupCommand` | 右臂关节位置 |
| `/ctrl/gripper` | `ctrl_msgs/GripperCommand` | 左右夹爪开口宽度 |

运行模式：

- `mode:=real` 启动 `real_robot_bridge`，通过 `galbot_sdk.g1` 控制真机。
- `mode:=sim` 启动 `rviz_joint_bridge`，将相同话题转换为 `/joint_states`。
- `mode:=real rviz:=true` 同时启动 SDK 控制、控制话题镜像、RViz 和 TF。
- 真机模式默认 `allow_motion:=false`，禁止 Agent 擅自改为 true。
- 仿真模式由 `rviz_joint_bridge` 独占 `/joint_states`，不要再启动
  `joint_state_publisher`。
- `JointGroupCommand` 的关节名必须属于对应话题分组。
- 真机轮速通过官方 URDF 的四轮几何转换为底盘速度，并带命令超时停车保护。
- 仿真轮速用于 RViz 主动轮旋转积分，并发布每轮 10 个被动滚子关节以补齐
  轮子 TF；底盘平移仍需要独立 odom/TF。
- `ros2 run ctrl raise_body_lower_arms_test` 是共享话题测试程序，先放下双臂，
  打开夹爪，再使用官方站姿抬升身体；必须显式传入 `-p confirm:=true`。
- `ros2 run ctrl grab` 发布指定 Grab 姿态：右臂使用配置值，左臂逐项取反，
  双手 TCP 间距 `0.19 m` 并前移约 `0.05 m`，夹爪闭合到 `0.06 m`，
  末端绕局部 X 轴翻转 `180°`，腿部回到默认姿态
  `[0.5, 1.5, 1.0, 0, 0]`。
- `ros2 run ctrl drop` 将腿部落到 `[0, 0, 0.3, 0, 0]`，然后用 21 个同步
  IK 路点沿前下方斜线将双臂 TCP 降至离地 `0.10 m`，终点前移 `0.20 m`，
  保持双手间距 `0.19 m` 和局部 X 轴 `180°` 翻转姿态。
- `/ctrl/state/real`、`/ctrl/state/sim` 分别发布真机与仿真状态；
  `/ctrl/state/error` 发布关节角、左右末端位置和姿态误差。
- 末端位姿统一使用 `base_link -> left/right_gripper_tcp_link`。
- `src/grabbox` 是独立的无 ROS Python 包。顶层导出 `GalbotRobot` 和
  `GrabBox`，由调用方初始化 `GalbotRobot` 后传入 `GrabBox`，提供
  `default()`、`grab()`、`drop()` 三个直接控制真机的状态函数。
- `grabbox.default()` 的左右臂使用 `GalbotMotion.motion_plan()` 单路点协调
  规划；腿部和头部仍使用直接关节设置。
- `python -m grabbox.real_test` 是真机安全测试入口，默认执行
  `default -> grab -> drop`，动作前要求输入 `YES`。
