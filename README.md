# STAR-Searcher 探索系统使用指南

## 快速启动

```bash
# 基本启动（默认使用 Open3D 仿真模式）
roslaunch semantic_exploration exploration_new.launch

# 指定仿真模式
roslaunch semantic_exploration exploration_new.launch sim_mode:=0  # Gazebo
roslaunch semantic_exploration exploration_new.launch sim_mode:=1  # Open3D
roslaunch semantic_exploration exploration_new.launch sim_mode:=2  # 实机

# 自动开始探索
roslaunch semantic_exploration exploration_new.launch auto_start:=true
```

---

## 必须配置的接口

以下是接入新系统时**必须修改**的接口配置，位于 `config/exploration_new.yaml`：

### 1. 传感器话题 ⚠️ 必须修改

```yaml
rog_map:
  lidar_topic: "/cloud_registered"      # LiDAR 点云话题
  odom_topic: "/ground_truth/odom"      # 里程计话题
  camera_pose_topic: "/ground_truth/camera_pose"  # 相机位姿话题 (可选)
  camera_rgb_topic: "/camera/color/image_raw"     # RGB 图像话题 (可选)
```

| 参数 | 说明 | 常见值 |
|------|------|--------|
| `lidar_topic` | 配准后的点云 | `/cloud_registered`, `/livox/lidar` |
| `odom_topic` | 融合后的位姿 | `/Odometry`, `/mavros/local_position/odom` |

---

## 常用参数调整

### 动力学约束

```yaml
manager:
  max_vel: 1.0          # 最大速度 [m/s]
  max_acc: 0.8          # 最大加速度 [m/s²]
  max_yaw_rate: 0.25    # 最大偏航角速度 [rad/s]
```

### 地图分辨率与范围

```yaml
rog_map:
  resolution: 0.2       # 栅格分辨率 [m]
  
  local_map:
    half_size: [10.0, 10.0, 2.0]  # 局部地图半尺寸 [m]
  
  global_map:
    map_size: [50, 50, 2.0]       # 全局地图尺寸 [m]
```

### 传感器参数

```yaml
rog_map:
  sensor:
    lidar_max_range: 25.0         # LiDAR 最大量程 [m]
    lidar_min_range: 0.2          # LiDAR 最小量程 [m]
    camera_max_range: 5.0         # 相机最大深度 [m]
    camera_fov_h_deg: 90.0        # 水平 FOV [deg]
    camera_fov_v_deg: 60.0        # 垂直 FOV [deg]
```

### 安全参数

```yaml
viewpoint_manager:
  uav_radius: 0.3         # 无人机机身半径 [m]
  safety_margin: 0.25     # 额外安全边距 [m]

optimization:
  obstacle_clearance: 0.5      # 硬约束安全距离 [m]
  obstacle_clearance_soft: 0.8 # 软约束安全距离 [m]
```

---

## 仿真模式说明

| sim_mode | 模式 | 说明 |
|----------|------|------|
| 0 | Gazebo | 自动解锁起飞，需要 MAVROS |
| 1 | Open3D | 纯规划仿真，无需 FCU |
| 2 | Real | 手动解锁，OFFBOARD 后自动起飞 |

---

## 输出话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 位置控制指令 |
| `/planning/trajectory` | `semantic_exploration/PolyTraj` | 多项式轨迹 |
| `/exploration/current_goal` | `geometry_msgs/PoseStamped` | 当前探索目标 |

---

## 文件结构

```
exploration/
├── launch/
│   └── exploration_new.launch    # 主启动文件
├── config/
│   ├── exploration_new.yaml      # 主参数配置
│   └── exploration_open3d.rviz   # RViz 配置
├── src/
│   ├── exploration_core/         # 探索核心 (FSM, ViewpointManager...)
│   ├── ego_planner/              # 轨迹规划器
│   └── map/                      # STAR-MVF 地图系统
└── include/
    ├── exploration/              # 探索模块头文件
    ├── map/                      # 地图模块头文件
    └── ego_planner/              # 规划器头文件
```

---

## 常见问题

### 1. 无人机不起飞
- 检查 `sim_mode` 是否正确设置
- Gazebo 模式：确认 MAVROS 连接正常
- 实机模式：需要先手动解锁并切换至 OFFBOARD

### 2. 地图无更新
- 检查 `lidar_topic` 和 `odom_topic` 是否有数据
- 使用 `rostopic hz /your_lidar_topic` 验证

### 3. 规划失败
- 调低 `max_vel` 和 `max_acc`
- 增大 `obstacle_clearance`
- 检查地图是否正确构建

---

## 参考

- [STAR-MVF 地图系统](doc/STAR_MVF_MAP_SYSTEM.md)
- [探索状态机设计](doc/EXPLORATION_FSM_NEW.md)
- [EGO Planner 技术文档](doc/EGO_Planner_Technical_Documentation.md)
