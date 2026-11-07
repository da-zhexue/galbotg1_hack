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
仿真实机同步版本
```bash
# 仿真/RViz
ros2 launch ctrl ctrl.launch.py mode:=sim

# 真机，仅接收但不运动
ros2 launch ctrl ctrl.launch.py mode:=real allow_motion:=false

# 确认安全后再启用运动
ros2 launch ctrl ctrl.launch.py mode:=real allow_motion:=true

# 回到标准位姿
ros2 run ctrl raise_body_lower_arms_test --ros-args -p confirm:=true
ros2 run ctrl grab --ros-args -p confirm:=true
ros2 run ctrl drop --ros-args -p confirm:=true
```

仅实机，无ROS版（仅把src/grabbox放入机器人内电脑即可）
```bash
source /home/lin/galbot/sdk/galbot_sdk/linux-x86_64-gcc940/setup.sh

python -m grabbox.real_test --state default
python -m grabbox.real_test --state grab
python -m grabbox.real_test --state drop
```
必须执行完grab再执行drop，否则手会被底盘卡住。  
如果机器某关节卡死了（判断：按住手腕处的按钮进入重力补偿模式臂无法拖动），将机器完整断电重启（身前屏幕要灭掉），再按住手腕处的按钮进入重力补偿模式将臂拖离卡死状态。  


```
参考现有grabbox和nav封装，将语音模块修改为一个类由main.py调用，在移动前、扫描完第三个取件码后（扫描函数还留TODO）、夹取快递后、到达导航目标点、放下快递后分别播放02～06音频
```