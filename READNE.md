# 银河通用G1开发

## SDK下载
```bash
git clone https://github.com/GalaxyGeneralRobotics/GalbotSDK.git
mkdir src && cd src
git clone https://github.com/GalaxyGeneralRobotics/galbot_one_golf_description.git
```
按照官方SDK的README进行sdk安装

## 构建
```bash
cd /home/lin/galbot
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ctrl_msgs ctrl
source install/setup.bash
```

## 控制
```bash
# 仿真/RViz
ros2 launch ctrl ctrl.launch.py mode:=sim

# 真机，仅接收但不运动
ros2 launch ctrl ctrl.launch.py mode:=real allow_motion:=false

# 确认安全后再启用运动
ros2 launch ctrl ctrl.launch.py mode:=real allow_motion:=true

# 回到标准位姿
ros2 run ctrl raise_body_lower_arms_test
```