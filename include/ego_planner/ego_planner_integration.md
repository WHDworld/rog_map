# EGO-Planner (MINCO) 集成方案

## 概述

将 formation_ws 中的 EGO-Planner（使用 MINCO 轨迹）集成到 rog_map 中，与现有的 fast_planner 并存。

## 文件结构

```
rog_map/
├── include/
│   └── ego_planner/
│       ├── plan_container.hpp      # 轨迹数据容器
│       ├── poly_traj_optimizer.h   # 多项式轨迹优化器
│       ├── ego_planner_manager.h   # 规划管理器
│       ├── ego_replan_fsm.h        # 状态机
│       ├── rog_astar.h             # A*搜索（适配rog_map）
│       └── optimizer/
│           ├── poly_traj_utils.hpp # MINCO轨迹工具（从formation_ws复制）
│           ├── lbfgs.hpp           # L-BFGS优化器
│           ├── root_finder.hpp     # 根求解器
│           └── gcopter.hpp         # 优化辅助
├── src/
│   └── ego_planner/
│       ├── poly_traj_optimizer.cpp
│       ├── ego_planner_manager.cpp
│       ├── ego_replan_fsm.cpp
│       ├── rog_astar.cpp
│       └── ego_planner_node.cpp
└── config/
    └── ego_planner_config.yaml
```

## 关键修改点

### 1. 地图接口适配
- 原始: `GridMap::Ptr grid_map_`
- 适配: `rog_map::ROGMap::Ptr rog_map_`
- 方法映射:
  - `getInflateOccupancy(pos)` -> `rog_map_->isOccupiedInflate(pos)`
  - `getResolution()` -> `rog_map_->getResolution()`

### 2. A*搜索适配
- 使用 rog_map 的碰撞检测接口
- 保持与 formation_ws 相同的搜索逻辑

### 3. 轨迹优化
- 保持 MINCO 轨迹优化核心算法不变
- 移除 formation (编队) 相关功能（单机使用）
- 保留障碍物避障和动力学约束

### 4. 状态机
- 保持与 formation_ws 相似的状态转换逻辑
- 添加 PX4 控制支持（参考现有 fast_planner）
- 保持目标点处理逻辑

## 参数配置（基于无人机质量 1.99kg）

```yaml
# 动力学约束
manager:
  max_vel: 2.0           # 最大速度 m/s
  max_acc: 2.0           # 最大加速度 m/s²
  max_jerk: 4.0          # 最大加加速度 m/s³

# 优化权重
optimization:
  weight_obstacle: 10000.0
  weight_feasibility: 10000.0
  weight_sqrvariance: 10000.0
  weight_time: 10.0
  obstacle_clearance: 0.3        # 安全距离
  obstacle_clearance_soft: 0.5   # 软约束距离
  constraint_points_perPiece: 5

# 状态机
fsm:
  thresh_replan_time: 1.0
  thresh_no_replan_meter: 1.0
  planning_horizon: 7.5
  emergency_time: 1.0
```

## 与 fast_planner 的区别

| 特性 | fast_planner (B-spline) | ego_planner (MINCO) |
|-----|------------------------|---------------------|
| 轨迹表示 | B-spline | 多项式 (MinJerk) |
| 优化方法 | NLopt | L-BFGS |
| 碰撞检测 | 控制点距离 | 约束点方向向量 |
| 时间优化 | 段时间分配 | 虚拟时间映射 |

## 使用方法

启动 ego_planner:
```bash
roslaunch rog_map ego_planner.launch
```

启动 fast_planner:
```bash
roslaunch rog_map fast_planner.launch
```

## 注意事项

1. 两个规划器不应同时运行
2. 发布的轨迹话题格式保持一致
3. 配置文件分开管理

