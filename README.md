# MotionLab ROS 2 Navigation

基于 ROS 2 Jazzy 的差速移动机器人规划与控制项目。

## 已完成功能

- 差速小车 URDF/Xacro 建模
- Gazebo Sim 仿真
- 激光雷达与里程计
- A* 全局路径规划
- Pure Pursuit 路径跟踪
- RViz 地图、路径和机器人可视化
- ROS 2 一键启动文件

## 技术栈

- Ubuntu 24.04 / WSL2
- ROS 2 Jazzy
- C++ / Python
- Gazebo Sim
- RViz2
- Git

## 构建

cd ~/motionlab_ws
colcon build --symlink-install
source install/setup.bash

## 一键运行

ros2 launch motionlab_simulation navigation_demo.launch.py

## 系统架构

```text
A* Planner
  ├── /grid_map      ──> RViz
  └── /planned_path  ──> Pure Pursuit Controller ──> /cmd_vel_raw ──> Safety Stop ──> /cmd_vel

Gazebo Sim
  ├── /odom ──────────> Pure Pursuit Controller
  └── /scan

/cmd_vel ─────────────> Differential Drive Robot
```

`navigation_demo.launch.py` 一键启动 Gazebo、静态 TF、A* 规划器、Pure Pursuit 控制器和 RViz。

## 核心节点与话题

| 节点 | 订阅 | 发布 | 功能 |
|---|---|---|---|
| `astar_planner` | — | `/grid_map`、`/planned_path` | A* 全局路径规划 |
| `pure_pursuit_controller` | `/planned_path`、`/odom` | `/cmd_vel_raw` | 路径跟踪控制 |
| `lidar_safety_stop` | `/cmd_vel_raw`、`/scan` | `/cmd_vel` | 前方障碍检测与紧急停车 |
| Gazebo 差速驱动 | `/cmd_vel` | `/odom` | 执行运动并发布里程计 |
| Gazebo 激光雷达 | — | `/scan` | 发布激光雷达数据 |
| RViz2 | 地图、路径和 TF | — | 导航过程可视化 |

## 核心算法

### A* 全局路径规划

A* 使用 `f(n) = g(n) + h(n)` 评估候选节点，其中 `g(n)` 是从起点到当前节点的实际代价，`h(n)` 是到目标点的启发式估计。生成的路径经过简化和加密，为控制器提供连续参考点。

默认规划范围：

| 参数 | 默认值 |
|---|---:|
| `start_x` / `start_y` | 5.5 / 5.5 |
| `goal_x` / `goal_y` | 9.5 / 5.5 |

### Pure Pursuit 路径跟踪

控制器根据 `/odom` 获取车辆位姿，在 `/planned_path` 上选择前视点，并计算 `/cmd_vel_raw`。前视距离较小时跟踪灵敏，但更容易振荡；前视距离较大时运动平滑，但转弯误差可能增大。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `lookahead_distance` | 0.6 m | 前视距离 |
| `nominal_speed` | 1.0 m/s | 标称线速度 |
| `max_angular_speed` | 3.0 rad/s | 最大角速度 |
| `goal_tolerance` | 0.12 m | 到达目标的判定距离 |

## 激光雷达安全停车

安全节点检查机器人前方 30 度范围内的激光雷达数据。当前方最近障碍距离小于 0.40 m 时，节点将线速度和角速度置零；障碍消失后恢复传递控制指令。

速度链路：

```text
Pure Pursuit -> /cmd_vel_raw -> Lidar Safety Stop -> /cmd_vel -> Gazebo
```

## WSL2 注意事项

Gazebo 和 RViz 同时运行时可能占用较多 CPU/GPU。若 RViz 出现 OpenGL、GLSL 报错或卡顿，可使用轻量 RViz 配置，或尝试软件渲染：

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```
