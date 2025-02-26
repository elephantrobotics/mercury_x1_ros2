# mercury_x1_ros2
Use of Mercury X1 mobile chassis ROS2

## Package Download and Install

```bash
$ git clone https://github.com/elephantrobotics/mercury_x1_ros2.git
$ cd ~/mercury_x1_ros2
$ colcon build    # Compile all function packages
$ source install/setup.bash

# Compile the function package separately:
If you only compile “turn_on_mercury_robot“, you need to execute the command:
$ colcon build --packages-select turn_on_mercury_robot
```
# 一、Mercury Seven Axis

>> **Note**:  Make sure the machine is powered on and enabled before use

## Bottom car keyboard control

**1. First start the car's underlying communication program and map building:**

```bash
$ ros2 launch slam_gmapping slam_gmapping.launch.py
```

**2. Then start the keyboard control program:**

```bash
$ ros2 run mercury_x1_control mercury_keyboard
```

## Chassis car + upper body robotic arm control

**1. First start the car's underlying communication program and map building:**

```bash
$ ros2 launch slam_gmapping slam_gmapping.launch.py
```

**2. Then Turn on upper body robotic arm control:**

```bash
$ ros2 run mercury_x1_control slider_control
```

**3. Finally start the keyboard control program:**

```bash
$ ros2 run mercury_x1_control mercury_keyboard
```

# 二、Mercury Turing Six Axis

>> **Note**:  Make sure the machine is powered on and enabled before use

## Bottom car keyboard control

**1. First start the car's underlying communication program and map building:**

```bash
$ ros2 launch slam_gmapping slam_gmapping_turing.launch.py
```

**2. Then start the keyboard control program:**

```bash
$ ros2 run mercury_x1_control mercury_keyboard
```

## Chassis car + upper body robotic arm control

**1. First start the car's underlying communication program and map building:**

```bash
$ ros2 launch slam_gmapping slam_gmapping_turing.launch.py
```

**2. Then Turn on upper body robotic arm control:**

```bash
$ ros2 run mercury_x1_control slider_control_turing
```

**3. Finally start the keyboard control program:**

```bash
$ ros2 run mercury_x1_control mercury_keyboard
```

# ROS2 常用指令汇总：

```bash
单独编译功能包：
如只编译 turn_on_mercury_robot
colcon build --packages-select turn_on_mercury_robot
编译全部功能包:
colcon build
注意：用户修改launch文件内容后需要编译才能生效。

NFS挂载
sudo mount -t nfs 192.168.0.100:/home/er/mercury_x1_ros2/ /mnt

1、打开机器人底盘
# Mercury Seven Axis
ros2 launch turn_on_mercury_robot turn_on_mercury_robot.launch.py
# Mercury Six Axis
ros2 launch turn_on_mercury_robot turn_on_mercury_robot_turing.launch.py

2、打开底盘控制
# Mercury Seven Axis
ros2 launch turn_on_mercury_robot turn_on_mercury_robot.launch.py
# Mercury Seven Axis
ros2 launch turn_on_mercury_robot turn_on_mercury_robot_turing.launch.py

3、打开相机
ros2 launch turn_on_mercury_robot mercury_camera.launch.py

4、打开雷达
ros2 launch turn_on_mercury_robot mercury_lidar.launch.py

5、打开底盘小车键盘控制
ros2 run mercury_x1_control mercury_keyboard 

6、简单跟随功能
① 雷达跟随
ros2 launch simple_follower_ros2 laser_follower.launch.py

② 视觉巡线
ros2 launch simple_follower_ros2 line_follower.launch.py

③ 视觉跟踪
ros2 launch simple_follower_ros2 visual_follower.launch.py

7、2D建图
①使用gmapping建图
# Mercury Seven Axis
ros2 launch slam_gmapping slam_gmapping.launch.py
# Mercury Six Axis
ros2 launch slam_gmapping slam_gmapping_turing.launch.py

②使用slam_toolbox建图
ros2 launch mercury_slam_toolbox online_async.launch.py

③使用cartographer建图
ros2 launch mercury_cartographer cartographer.launch.py

保存地图
ros2 launch mercury_nav2 save_map.launch.py

8、2D导航
ros2 launch mercury_nav2 mercury_nav2.launch.py

9、RRT 自主探索建图[顺时针/逆时针发布四个点，最后一个点发布在已知地图中]
step1：ros2 launch mercury_slam_toolbox online_sync.launch.py
step2：ros2 launch mercury_robot_rrt mercury_rrt_slam.launch.py

10、RTAB-MAP建图
ros2 launch mercury_robot_rtab mercury_slam_rtab.launch.py

11、RTAB-MAP导航
ros2 launch mercury_robot_rtab mercury_nav2_rtab.launch.py

12、USB手柄控制
ros2 launch mercury_joy mercury_joy.launch.py

13、打开rviz2
注意：使用虚拟机打开rviz2

14、底盘小车+上半身机械臂控制
①使用gmapping建图
# Mercury Seven Axis
ros2 launch slam_gmapping slam_gmapping.launch.py
# Mercury Six Axis
ros2 launch slam_gmapping slam_gmapping_turing.launch.py

②打开上半身机械臂控制
# Mercury Seven Axis
ros2 run mercury_x1_control slider_control
# Mercury Six Axis
ros2 run mercury_x1_control slider_control_turing

③打开底盘小车键盘控制
ros2 run mercury_x1_control mercury_keyboard

```