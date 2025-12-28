# ROG-Map 探索系统架构文档

## 📁 项目目录结构

```
rog_map/
├── include/rog_map/               # 头文件
│   │
│   │   ==================== 🗺️ 地图管理模块 ====================
│   ├── rog_map.h                  # ROG-Map 主类 (继承自 ProbMap)
│   ├── prob_map.h                 # 概率地图 (融合多传感器更新)
│   ├── global_map.h               # 全局地图 (存储 GlobalVoxelState)
│   ├── sliding_map.h              # 滑动窗口地图基类
│   ├── inf_map.h                  # 膨胀地图 (障碍物膨胀)
│   ├── esdf_map.h                 # ESDF地图 (欧几里得符号距离场)
│   ├── free_cnt_map.h             # 自由计数地图
│   │
│   │   ==================== 🔍 探索管理模块 ====================
│   ├── exploration_fsm.h          # 探索状态机
│   ├── frontier_viewpoint_generator.h  # 前沿视点生成器
│   │
│   └── rog_map_core/              # 核心工具
│       ├── common.h               # 通用定义
│       └── ...
│
├── src/
│   │
│   │   ==================== 🗺️ 地图管理模块 ====================
│   ├── rog_map/
│   │   ├── rog_map.cpp            # ROG-Map 实现
│   │   ├── prob_map.cpp           # 概率地图实现
│   │   ├── global_map.cpp         # 全局地图实现
│   │   ├── sliding_map.cpp        # 滑动窗口地图实现
│   │   ├── inf_map.cpp            # 膨胀地图实现
│   │   ├── esdf_map.cpp           # ESDF地图实现
│   │   ├── counter_map.cpp        # 计数地图实现
│   │   │
│   │   │   ==================== 🔍 探索管理模块 ====================
│   │   ├── exploration_fsm.cpp          # 探索状态机实现
│   │   ├── exploration_node.cpp         # 探索测试节点
│   │   └── frontier_viewpoint_generator.cpp  # 前沿视点生成器实现
│   │
│   │   ==================== 🚀 规划器管理模块 ====================
│   ├── ego_planner/               # EGO-Planner (MINCO轨迹)
│   │   ├── ego_planner_node.cpp   # 规划器主节点
│   │   ├── ego_planner_manager.cpp # 规划管理器
│   │   ├── ego_replan_fsm.cpp     # 重规划状态机
│   │   ├── ego_traj_server.cpp    # 轨迹服务器
│   │   ├── poly_traj_optimizer.cpp # 多项式轨迹优化器
│   │   └── rog_astar.cpp          # A*路径搜索
│   │
│   ├── fast_planner/              # Fast-Planner (B-Spline轨迹)
│   │   ├── fast_planner_node.cpp  # 规划器主节点
│   │   ├── planner_manager.cpp    # 规划管理器
│   │   ├── planning_fsm.cpp       # 规划状态机
│   │   ├── traj_server.cpp        # 轨迹服务器
│   │   ├── bspline/               # B-Spline相关
│   │   └── polynomial/            # 多项式相关
│   │
│   └── test_explore/              # 测试模块
│       └── test_planner.cpp
│
├── config/                        # 配置文件
│   ├── ego_planner_config.yaml    # EGO-Planner 配置
│   ├── exploration_config.yaml    # 探索模块配置
│   └── exploration_test.rviz      # RViz 可视化配置
│
├── launch/                        # Launch 文件
│   ├── ego_planner.launch         # EGO-Planner 启动
│   ├── exploration_test.launch    # 探索测试启动
│   └── ...
│
└── msg/                           # 自定义消息
    └── PolyTraj.msg               # 多项式轨迹消息
```

---

## 🏗️ 系统架构概述

### 整体框架图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           无人机探索系统                                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐         │
│  │   传感器输入      │  │   里程计输入      │  │   用户触发       │         │
│  │ (LiDAR点云)      │  │ (nav_msgs/Odom)  │  │ (RViz 2D Goal)  │         │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘         │
│           │                    │                    │                   │
│           ▼                    ▼                    ▼                   │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │                    🗺️ 地图管理模块 (ROG-Map)                    │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐            │    │
│  │  │  ProbMap    │  │  GlobalMap  │  │   InfMap    │            │    │
│  │  │ (概率更新)   │→ │ (全局状态)   │→ │ (障碍膨胀)   │            │    │
│  │  └─────────────┘  └──────┬──────┘  └─────────────┘            │    │
│  │                          │                                     │    │
│  │                          ▼                                     │    │
│  │              ┌─────────────────────┐                          │    │
│  │              │     SubGrid         │ ← 子网格管理              │    │
│  │              │   (前沿提取/统计)    │                          │    │
│  │              └─────────────────────┘                          │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                          │                                              │
│                          ▼                                              │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │                    🔍 探索管理模块                               │    │
│  │  ┌───────────────────────────────────────────────────────────┐ │    │
│  │  │              ExplorationFSM (探索状态机)                   │ │    │
│  │  │  ┌─────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │ │    │
│  │  │  │INIT │→ │WAIT_TRIG │→ │UPDATE_FR │→ │PLAN_VP   │       │ │    │
│  │  │  └─────┘  └──────────┘  └──────────┘  └────┬─────┘       │ │    │
│  │  │                                            │              │ │    │
│  │  │                                            ▼              │ │    │
│  │  │  ┌──────────┐  ┌──────────┐        ┌──────────┐          │ │    │
│  │  │  │EMERGENCY │← │ FINISH   │←────── │EXEC_EXPL │          │ │    │
│  │  │  └──────────┘  └──────────┘        └──────────┘          │ │    │
│  │  └───────────────────────────────────────────────────────────┘ │    │
│  │                          │                                     │    │
│  │  ┌───────────────────────────────────────────────────────────┐ │    │
│  │  │       FrontierViewpointGenerator (视点生成器)              │ │    │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │ │    │
│  │  │  │ 前沿聚类     │→ │ 区域划分     │→ │ 视点生成     │       │ │    │
│  │  │  │ (DBSCAN)    │  │ (层次聚类)   │  │ (几何采样)   │       │ │    │
│  │  │  └─────────────┘  └─────────────┘  └─────────────┘       │ │    │
│  │  │                                            │              │ │    │
│  │  │  ┌─────────────┐  ┌─────────────┐         │              │ │    │
│  │  │  │ 视点评估     │← │ TSP路由      │←────────┘              │ │    │
│  │  │  │ (收益计算)   │  │ (MST优化)    │                        │ │    │
│  │  │  └─────────────┘  └─────────────┘                        │ │    │
│  │  └───────────────────────────────────────────────────────────┘ │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                          │                                              │
│                          ▼                                              │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │                    🚀 规划器管理模块                             │    │
│  │  ┌───────────────────────────────────────────────────────────┐ │    │
│  │  │            EGO-Planner (MINCO轨迹优化)                     │ │    │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │ │    │
│  │  │  │ A*路径搜索   │→ │ 轨迹初始化   │→ │ 轨迹优化     │       │ │    │
│  │  │  └─────────────┘  └─────────────┘  └─────────────┘       │ │    │
│  │  │                                            │              │ │    │
│  │  │  ┌─────────────┐  ┌─────────────┐         │              │ │    │
│  │  │  │ 位置命令发布 │← │ 轨迹服务器   │←────────┘              │ │    │
│  │  │  └─────────────┘  └─────────────┘                        │ │    │
│  │  └───────────────────────────────────────────────────────────┘ │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                          │                                              │
│                          ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                     控制器 / 飞控                                 │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 🗺️ 地图管理模块

### 1. 类继承关系

```
SlidingMap (滑动窗口地图基类)
    ↓
ProbMap (概率地图)
    ├── inf_map_        → InfMap (膨胀地图)
    ├── fcnt_map_       → FreeCntMap (自由计数地图)
    ├── esdf_map_       → ESDFMap (ESDF地图)
    └── global_map_     → GlobalMap (全局地图)
            ↓
ROGMap (ROG-Map 主类)
```

### 2. GlobalMap 核心功能

GlobalMap 是探索系统的核心地图，采用**几何-语义双层状态**设计：

```cpp
// 体素状态位定义
struct VoxelStateBits {
    // 几何层 (2位)
    static constexpr uint8_t GEO_UNKNOWN  = 0x00;  // 未知
    static constexpr uint8_t GEO_FREE     = 0x01;  // 自由
    static constexpr uint8_t GEO_OCCUPIED = 0x02;  // 占据
    
    // 视觉层 (1位)
    static constexpr uint8_t FLAG_VISITED = 0x04;  // 已视觉覆盖
    
    // 语义层 (1位)
    static constexpr uint8_t FLAG_TARGET  = 0x08;  // 目标标记
};
```

### 3. SubGrid (子网格) 设计

将全局地图划分为多个子网格，用于高效的前沿管理：

```cpp
struct SubGrid {
    GlobalVoxelIndex global_min_idx, global_max_idx;  // 体素范围
    Eigen::Vector3f world_min, world_max;             // 世界坐标范围
    SubGridIndex grid_idx;                            // 子网格索引
    
    // 统计信息
    int total_voxels, unknown_count, occupied_count, free_count;
    
    // 前沿管理
    std::unordered_set<GlobalVoxelIndex, GlobalVoxelIndexHash> 
        sub_grid_frontier_voxels_;
    
    // 前沿提取 (6邻域检查)
    void extractFrontier(GlobalMap& gmap);
};
```

---

## 🔍 探索管理模块

### 1. 探索策略概述

本系统采用**层级式探索策略**，融合了 EDEN、FLARE、Star-Searcher 等论文的核心思想：

| 层级 | 名称 | 功能 | 参考算法 |
|-----|------|------|---------|
| L1 | 前沿提取 | 从体素图提取几何前沿 | FLARE |
| L2 | 前沿聚类 | 将稀疏前沿聚合为簇 | Star-Searcher |
| L3 | 区域划分 | 基于方向/距离划分区域 | EDEN |
| L4 | 视点生成 | 为每个簇生成候选视点 | FC-Planner |
| L5 | TSP路由 | 优化区域/视点访问顺序 | EDEN EOHDT |
| L6 | 视点评估 | 综合评分选择最佳视点 | Star-Searcher |

### 2. 前沿视点生成器 (FrontierViewpointGenerator)

#### 2.1 前沿聚类 (DBSCAN)

```
输入: 前沿体素集合 F = {f₁, f₂, ..., fₙ}
参数: 邻域半径 ε, 最小点数 minPts

算法:
1. 对每个未访问的前沿点 fᵢ:
   a. 查找 ε-邻域内的所有点
   b. 如果邻域点数 ≥ minPts:
      - 创建新簇 C
      - 递归扩展簇
   c. 否则标记为噪声

输出: 簇集合 C = {C₁, C₂, ..., Cₖ}
```

#### 2.2 区域划分 (EDEN风格)

基于探索方向和距离的层次聚类：

```
核心公式:

1. 方向差异:
   θ_diff = arccos(d_i · d_j)
   其中 d_i = (centroid_i - UAV_pos).normalized()

2. 距离差异:
   dist_diff = |d_i - d_j|

3. 区域合并条件:
   merge_if: θ_diff < θ_max AND dist_diff < dist_max
```

#### 2.3 视点生成

采用**球面坐标采样**策略：

```
视点位置计算:
P_vp = P_centroid + R · [cos(pitch)cos(yaw), cos(pitch)sin(yaw), -sin(pitch)]

其中:
- R: 最优观测距离 (默认3m)
- pitch: 俯仰角 (从-30°到30°采样)
- yaw: 偏航角 (360°均匀采样)

约束条件:
1. 安全约束: ESDF(P_vp) > r_uav + margin
2. 可达约束: 射线追踪无碰撞
3. 覆盖约束: 视锥体覆盖前沿
```

#### 2.4 TSP路由优化 (EDEN EOHDT)

```
EOHDT算法流程:

1. 构建区域距离图:
   Weight(Ri, Rj) = path_distance(centroid_i, centroid_j)

2. Prim算法构建MST:
   - 从UAV位置最近的区域开始
   - 每次添加距离最近的未访问区域

3. 最远分支后置:
   - 识别最远叶节点所在分支
   - 将该分支移至路由末尾
   - 节省返回起点的距离

4. 多子节点选择:
   R_next = argmin_{Rj ∈ Children(Ri)} d(Ri, Rj)
```

#### 2.5 视点评估函数

```
综合评分:
Score(vp) = w_cov · S_coverage + w_dist · S_distance + w_safe · S_safety

各项计算:
1. 覆盖得分: S_coverage = num_covered_frontiers / total_cluster_frontiers
2. 距离得分: S_distance = exp(-λ_d · distance_to_uav)
3. 安全得分: S_safety = min(1, ESDF(vp) / safety_margin)

默认权重: w_cov=0.6, w_dist=0.3, w_safe=0.1
```

---

## 📊 探索状态机 (ExplorationFSM)

### 状态定义

```cpp
enum class ExplorationState {
    INIT = 0,           // 初始化状态
    WAIT_TRIGGER,       // 等待触发
    UPDATE_FRONTIER,    // 更新前沿信息
    PLAN_VIEWPOINTS,    // 规划探索视点
    EXEC_EXPLORATION,   // 执行探索
    FINISH,             // 探索完成
    EMERGENCY_STOP      // 紧急停止
};
```

### 状态转换图

```
                    ┌──────────────────────────────────────┐
                    │                                      │
                    ▼                                      │
    ┌──────┐    ┌─────────────┐    ┌─────────────────┐    │
    │ INIT │───▶│ WAIT_TRIGGER │───▶│ UPDATE_FRONTIER │    │
    └──────┘    └─────────────┘    └────────┬────────┘    │
       │           ▲                        │             │
       │           │                        ▼             │
       │           │              ┌─────────────────┐     │
       │           │              │ PLAN_VIEWPOINTS │     │
       │           │              └────────┬────────┘     │
       │           │                       │              │
       │           │                       ▼              │
       │           │              ┌─────────────────┐     │
       │           └──────────────│ EXEC_EXPLORATION│─────┘
       │                          └────────┬────────┘
       │                                   │
       │                                   ▼
       │                          ┌─────────────────┐
       │                          │     FINISH      │
       │                          └────────┬────────┘
       │                                   │
       │           ┌───────────────────────┘
       │           ▼
       │    ┌──────────────┐
       └───▶│EMERGENCY_STOP│
            └──────────────┘
```

### 各状态详细说明

#### 1. INIT (初始化)
```
功能: 等待系统就绪
退出条件:
  - 收到里程计消息 (have_odom_ = true)
  - 等待时间 > init_wait_time (默认2秒)
转换: → WAIT_TRIGGER
```

#### 2. WAIT_TRIGGER (等待触发)
```
功能: 等待用户触发探索
触发方式:
  - RViz 中点击 "2D Nav Goal"
  - 设置 auto_start = true 自动开始
  - 调用 triggerExploration() 接口
转换条件:
  - triggered_ = true AND frontier_ready_ AND has_frontier_
转换: → UPDATE_FRONTIER
```

#### 3. UPDATE_FRONTIER (更新前沿)
```
功能: 更新前沿信息并生成区域划分
处理:
  1. 获取包含前沿的子网格列表
  2. 更新UAV状态给视点生成器
  3. 检查是否有足够的前沿点
转换:
  - 有前沿: → PLAN_VIEWPOINTS
  - 无前沿: → FINISH
```

#### 4. PLAN_VIEWPOINTS (规划视点)
```
功能: 生成探索视点和访问路由
处理:
  1. 调用 generateExplorationPlan()
  2. 前沿聚类 → 区域划分 → 视点生成 → TSP路由
  3. 选择第一个目标视点
转换:
  - 规划成功: → EXEC_EXPLORATION
  - 规划失败: → FINISH
```

#### 5. EXEC_EXPLORATION (执行探索)
```
功能: 执行探索任务，发布目标视点
处理:
  1. 发布当前目标视点给规划器
  2. 检查是否到达视点
  3. 检查前沿是否被覆盖
  4. 周期性重规划检查
转换:
  - 到达视点 && 有下一个: 保持 EXEC_EXPLORATION
  - 到达视点 && 无下一个: → UPDATE_FRONTIER
  - 前沿被覆盖: → UPDATE_FRONTIER
  - 周期性重规划: → UPDATE_FRONTIER
```

#### 6. FINISH (完成)
```
功能: 探索任务完成
处理:
  1. 输出探索统计信息
  2. 清除可视化
  3. 5秒后返回等待状态
转换: → WAIT_TRIGGER (5秒后)
```

#### 7. EMERGENCY_STOP (紧急停止)
```
功能: 紧急情况处理
触发: 检测到碰撞风险或用户手动触发
处理: 停止所有运动，等待手动恢复
```

### 被动可视化模式

**特殊功能**: 即使没有触发探索，系统也会在手动飞行时实时显示前沿区域和探索视点。

```cpp
// 在 frontierCallback 中实现
if (!triggered_ && config_.enable_visualization && has_frontier_) {
    // 更新UAV状态
    viewpoint_generator_->updateUAVState(uav_state);
    
    // 生成探索规划（仅用于可视化，不执行）
    viewpoint_generator_->generateExplorationPlan(
        frontier_subgrids, regions, region_route);
    
    // 更新可视化数据
    // ...
}
```

---

## 🚀 规划器管理模块

### EGO-Planner 架构

```
┌─────────────────────────────────────────────────────────────┐
│                     EGO-Planner 架构                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐      ┌─────────────────┐              │
│  │   目标点输入     │      │   里程计输入     │              │
│  │ /waypoints      │      │ ~odom            │              │
│  └────────┬────────┘      └────────┬────────┘              │
│           │                        │                        │
│           ▼                        ▼                        │
│  ┌────────────────────────────────────────────────┐        │
│  │            EgoReplanFSM (重规划状态机)          │        │
│  │  状态: INIT → WAIT_TARGET → GEN_NEW_TRAJ →    │        │
│  │        REPLAN_TRAJ → EXEC_TRAJ                 │        │
│  └────────────────────────────────────────────────┘        │
│                          │                                  │
│                          ▼                                  │
│  ┌────────────────────────────────────────────────┐        │
│  │          EgoPlannerManager (规划管理器)         │        │
│  │  ┌──────────────┐  ┌──────────────┐           │        │
│  │  │ ROG A* 搜索  │→ │ 轨迹初始化   │           │        │
│  │  └──────────────┘  └──────────────┘           │        │
│  │                          │                     │        │
│  │                          ▼                     │        │
│  │  ┌──────────────────────────────────────┐     │        │
│  │  │     PolyTrajOptimizer (MINCO优化)    │     │        │
│  │  │  目标: min J_smooth + J_collision +  │     │        │
│  │  │         J_dynamics + J_time          │     │        │
│  │  └──────────────────────────────────────┘     │        │
│  └────────────────────────────────────────────────┘        │
│                          │                                  │
│                          ▼                                  │
│  ┌────────────────────────────────────────────────┐        │
│  │          EgoTrajServer (轨迹服务器)            │        │
│  │  - 接收 MINCO 多项式轨迹                       │        │
│  │  - 按时间采样生成位置命令                       │        │
│  │  - 发布 PositionCommand 给控制器               │        │
│  └────────────────────────────────────────────────┘        │
│                          │                                  │
│                          ▼                                  │
│  ┌─────────────────────────────────────────────────┐       │
│  │            /planning/pos_cmd                     │       │
│  │        (quadrotor_msgs/PositionCommand)          │       │
│  └─────────────────────────────────────────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

---

## 📡 ROS话题接口

### 输入话题

| 话题 | 消息类型 | 说明 |
|-----|---------|------|
| `~odom` | `nav_msgs/Odometry` | 无人机里程计 |
| `~cloud` | `sensor_msgs/PointCloud2` | LiDAR点云 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz目标点/探索触发 |

### 输出话题 - 探索模块

| 话题 | 消息类型 | 说明 |
|-----|---------|------|
| `/exploration/viewpoint` | `geometry_msgs/PoseStamped` | 当前目标视点 |
| `/exploration/frontier_cloud` | `sensor_msgs/PointCloud2` | 前沿点云 |
| `/exploration/frontier_clusters` | `visualization_msgs/MarkerArray` | 前沿簇可视化 |
| `/exploration/viewpoints` | `visualization_msgs/MarkerArray` | 视点可视化 |
| `/exploration/frustums` | `visualization_msgs/MarkerArray` | 视锥体可视化 |
| `/exploration/region_route` | `visualization_msgs/MarkerArray` | 区域路由可视化 |
| `/exploration/state` | `std_msgs/Int8` | 当前探索状态 |

### 输出话题 - 规划模块

| 话题 | 消息类型 | 说明 |
|-----|---------|------|
| `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 位置控制命令 |
| `/planning/trajectory` | `rog_map/PolyTraj` | MINCO多项式轨迹 |

---

## 🎮 使用方法

### 1. 启动探索测试

```bash
# 终端1: 启动仿真环境
roslaunch your_simulation simulation.launch

# 终端2: 启动探索系统
roslaunch rog_map exploration_test.launch

# 终端3 (可选): 启动EGO-Planner进行轨迹规划
roslaunch rog_map ego_planner.launch
```

### 2. 手动飞行 + 实时可视化

系统会自动在RViz中显示：
- **彩色立方体**: 前沿簇 (不同颜色表示不同簇)
- **蓝色球体**: 候选探索视点
- **绿色球体**: 当前目标视点
- **黄色线框**: 视锥体范围
- **绿色连线**: 区域访问路由

### 3. 触发自动探索

在RViz中使用 **"2D Nav Goal"** 工具点击任意位置即可触发探索。

---

## ⚙️ 配置参数

### 统一配置文件: exploration_config.yaml

所有参数都集中在一个配置文件中，方便管理和修改：

**文件位置**: `config/exploration_config.yaml`

**包含内容**:
1. **rog_map**: ROG-Map 地图配置
2. **fsm**: EGO-Planner 状态机配置
3. **manager**: 规划管理器配置
4. **optimization**: MINCO轨迹优化配置
5. **traj_server**: 轨迹服务器配置
6. **exploration**: 探索状态机配置
7. **visualization**: 可视化颜色配置

### 关键参数说明

```yaml
# ========== ROG-Map 地图参数 ==========
rog_map:
  resolution: 0.2                   # 体素分辨率 [m]
  map_size: [20, 20, 6]             # 局部地图尺寸 [m]
  virtual_ceil_height: 2.5          # 虚拟天花板高度 [m]
  virtual_ground_height: -0.3       # 虚拟地面高度 [m]
  
  global_map:
    map_size: [100, 100, 3.0]       # 全局地图尺寸 [m]

# ========== EGO-Planner 规划参数 ==========
fsm:
  flight_type: 2                    # 1=手动目标, 2=预设路径点
  takeoff_height: 1.0               # 起飞高度 [m]
  auto_start_preset: true           # 自动开始预设路径
  
  # 预设路径点（flight_type=2 时使用）
  waypoint_num: 4
  waypoint0_x: 1.0
  waypoint0_y: 30.0
  waypoint0_z: 1.0
  # ...更多路径点...

manager:
  max_vel: 1.5                      # 最大速度 [m/s]
  max_acc: 3.0                      # 最大加速度 [m/s²]

optimization:
  weight_obstacle: 10000.0          # 障碍物代价权重
  obstacle_clearance: 0.2           # 安全距离 [m]
  enable_height_keeping: true       # 启用高度保持

# ========== 探索状态机参数 ==========
exploration:
  # FSM频率
  fsm_frequency: 100.0              # 状态机频率 [Hz]
  frontier_update_frequency: 3.0    # 前沿更新频率 [Hz]
  visualization_frequency: 10.0     # 可视化频率 [Hz]
  
  # 探索参数
  auto_start: false                 # 被动模式（仅可视化）
  enable_visualization: true        # 启用可视化
  
  # 传感器参数
  sensor_range_max: 5.0             # 最大感知距离 [m]
  sensor_fov_h: 1.57                # 水平FOV [rad] (90°)
  sensor_fov_v: 1.22                # 垂直FOV [rad] (70°)
  
  # 视点生成参数
  optimal_viewing_distance: 3.0     # 最优观测距离 [m]
  uav_radius: 0.35                  # 无人机半径 [m]
  min_cluster_size: 10              # 最小簇大小
  max_viewpoints_per_region: 20     # 每区域最大视点数
  
  # 可视化参数
  viewpoint_sphere_radius: 0.2      # 视点球体半径 [m]
  frustum_length: 2.5               # 视锥体长度 [m]
```

---

## 📚 参考文献

1. **EDEN**: Efficient Dual-Layer Exploration Planning (IEEE TIE 2025)
   - EOHDT全局路由算法
   - MST优化与最远分支后置策略

2. **FLARE**: Fast Large-Scale Autonomous Exploration
   - 未知区域划分方法
   - 增量式前沿更新

3. **Star-Searcher**: Aerial System for Autonomous Target Search
   - 可见性聚类方法
   - 视点评估函数

4. **FC-Planner**: Fast Coverage Planning
   - 骨架引导视点采样
   - 高效覆盖规划

5. **EGO-Planner**: Gradient-based Local Planner
   - MINCO轨迹优化
   - 实时避障规划

