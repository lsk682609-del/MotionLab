# MotionLab Navigation Runbook

## Environment

- Windows + WSL2
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim
- RViz2

---

## Terminal 1 — Navigation Backend

后台导航全部自动启动：

```bash
cd ~/motionlab_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch motionlab_simulation navigation_demo.launch.py \
  use_rviz:=false
```

Automatically starts:

- Gazebo Sim
- robot_state_publisher
- ros_gz_bridge
- static `map -> odom` TF
- map_server
- lifecycle manager
- A* planner
- Pure Pursuit controller
- LiDAR Safety Stop

Navigation chain:

```text
/map
 ↓
A* + Inflation
 ↓
/planned_path
 ↓
Pure Pursuit
 ↓
/cmd_vel_raw
 ↓
LiDAR Safety Stop
 ↓
/cmd_vel
 ↓
Gazebo
```

---

## Terminal 2 — RViz

手动启动 RViz：

```bash
cd ~/motionlab_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

QT_QPA_PLATFORM=xcb \
GALLIUM_DRIVER=d3d12 \
rviz2
```

当前 WSL2 环境下不要使用：

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

否则 RViz 会退回 llvmpipe CPU 软件渲染，容易出现明显卡顿。

### RViz Setup

设置：

```text
Fixed Frame = map
```

添加：

```text
Add -> Grid
```

添加 RobotModel：

```text
Add -> RobotModel

Description Source = Topic
Description Topic  = /robot_description
```

添加静态地图：

```text
Add -> Map

Topic       = /map
Reliability = Reliable
Durability  = Transient Local
```

添加 A* 路径：

```text
Add -> Path

Topic = /planned_path
```

正常导航阶段建议只显示：

```text
Grid
RobotModel
Map /map
Path /planned_path
```

以下内容仅在调试时开启：

```text
/grid_map
LaserScan
PointCloud2
TF
```

发送导航目标：

```text
2D Goal Pose
    ↓
/goal_pose
    ↓
A*
```

---

## Terminal 3 — Debug / Map Refresh

打开第三个 Terminal：

```bash
cd ~/motionlab_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

检查 map_server：

```bash
ros2 lifecycle get /map_server
```

正常输出：

```text
active [3]
```

如果 RViz 后启动后出现：

```text
No map received

Resolution = 0
Width = 0
Height = 0
```

执行：

```bash
ros2 lifecycle set /map_server deactivate
ros2 lifecycle set /map_server activate
```

随后 RViz 中的静态地图应重新出现。

---

## Useful Diagnostics

检查节点：

```bash
ros2 node list
```

检查静态地图：

```bash
ros2 topic info /map -v
```

检查规划路径：

```bash
ros2 topic info /planned_path -v
```

检查 Pure Pursuit 输出：

```bash
ros2 topic info /cmd_vel_raw -v
```

检查 Safety Stop 输出：

```bash
ros2 topic info /cmd_vel -v
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

检查 LiDAR：

```bash
ros2 topic hz /scan
```

检查 Odometry：

```bash
ros2 topic hz /odom
```

---

## Current Navigation Parameters

### A*

```text
inflation_radius = 0.25 m
```

### Pure Pursuit

```text
lookahead_distance = 0.40 m
nominal_speed      = 0.15 m/s
max_angular_speed  = 0.80 rad/s
goal_tolerance     = 0.12 m
```

### LiDAR Safety Stop

```text
stop_distance   = 0.35 m
release_margin  = 0.10 m
front_angle_deg = 30 deg
scan_timeout    = 0.5 s
cmd_timeout     = 0.5 s
```

---

## Navigation Pipeline

当前导航链路：

```text
Static Map
    ↓
map_server
    ↓
/map
    ↓
A* Planner
    ↓
Obstacle Inflation
    ↓
/planned_path
    ↓
Pure Pursuit
    ↓
/cmd_vel_raw
    ↓
LiDAR Safety Stop
    ↓
/cmd_vel
    ↓
Gazebo Differential Drive
```

传感器反馈：

```text
Gazebo
 ├── /odom
 │     ↓
 │ Pure Pursuit
 │
 └── /scan
       ↓
   LiDAR Safety Stop
```

---

# Normal Startup Sequence

## Step 1 — Start Navigation Backend

Terminal 1：

```bash
cd ~/motionlab_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch motionlab_simulation navigation_demo.launch.py \
  use_rviz:=false
```

等待后台节点全部启动。

---

## Step 2 — Start RViz

Terminal 2：

```bash
cd ~/motionlab_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

QT_QPA_PLATFORM=xcb \
GALLIUM_DRIVER=d3d12 \
rviz2
```

手动配置：

```text
Fixed Frame = map

Grid

RobotModel
Topic = /robot_description

Map
Topic = /map
Reliability = Reliable
Durability = Transient Local

Path
Topic = /planned_path
```

---

## Step 3 — Refresh Map If Needed

如果 RViz 没有收到地图：

Terminal 3：

```bash
cd ~/motionlab_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 lifecycle set /map_server deactivate
ros2 lifecycle set /map_server activate
```

然后回到 RViz。

---

# Navigation Test

在 RViz 中使用：

```text
2D Goal Pose
```

选择地图中的自由区域。

完整运行过程：

```text
2D Goal Pose
    ↓
/goal_pose
    ↓
A* Planner
    ↓
/planned_path
    ↓
Pure Pursuit
    ↓
/cmd_vel_raw
    ↓
LiDAR Safety Stop
    ↓
/cmd_vel
    ↓
Gazebo
```

---

# Shutdown

先关闭 RViz：

```text
Terminal 2
Ctrl+C
```

然后关闭 Navigation Backend：

```text
Terminal 1
Ctrl+C
```

只按一次 `Ctrl+C`，等待 ROS 2 launch 正常关闭子进程。

如果需要检查残留节点：

```bash
ros2 node list
```

避免在 launch 正在 shutdown 时连续多次按 `Ctrl+C`。
