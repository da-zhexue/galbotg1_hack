import time
from galbot_sdk.g1 import GalbotRobot, G1JointGroup, ControlStatus

# 获取并初始化 GalbotRobot 单例
robot = GalbotRobot()
robot.init()
print('Initialization succeeded')
# Program started, waiting for data
time.sleep(2)

gripper_state = robot.get_gripper_state(G1JointGroup.left_gripper)
print(
        f"初始夹爪宽度： {gripper_state.width:.3f}m "
        f"初始夹爪速度： {gripper_state.velocity:.3f}m/s "
        f"初始夹爪力矩： {gripper_state.effort}N "
        f"夹爪是否在移动： {gripper_state.is_moving}"
    )

# 设置左夹爪宽度为 0.02m，速度0.05m/s，力矩10N，阻塞直至夹爪到达设定位置
status = robot.set_gripper_command(
    G1JointGroup.left_gripper, 0.02, 0.05, 10, True
)
if status != ControlStatus.SUCCESS:
    print("Set gripper failed")
else:
    print('Set gripper success')

gripper_state = robot.get_gripper_state(G1JointGroup.left_gripper)
print(
        f"当前夹爪宽度：{gripper_state.width:.3f}m "
        f"当前夹爪速度：{gripper_state.velocity:.3f}m/s "
        f"当前夹爪力矩：{gripper_state.effort}N "
        f"夹爪是否在移动：{gripper_state.is_moving}"
    )

# 设置左夹爪宽度为 0.1m，速度0.05m/s，力矩10N，阻塞直至夹爪到达设定位置
status = robot.set_gripper_command(
    G1JointGroup.left_gripper, 0.1, 0.05, 10, True
)

gripper_state = robot.get_gripper_state(G1JointGroup.left_gripper)
print(
        f"当前夹爪宽度： {gripper_state.width:.3f}m "
        f"当前夹爪速度： {gripper_state.velocity:.3f}m/s "
        f"当前夹爪力矩： {gripper_state.effort}N "
        f"夹爪是否在移动： {gripper_state.is_moving}"
    )

if status != ControlStatus.SUCCESS:
    print("Set gripper failed")
else:
    print('Set gripper success')

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')
