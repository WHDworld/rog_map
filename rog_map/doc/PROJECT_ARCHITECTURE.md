# STAR-Searcher 探索系统架构文档

> **最后更新**: 2026-01-09  
> **版本**: v2.0 - 基于 STAR-MVF 地图系统 + EDEN 历史节点 + Falcon 重规划策略

---

## 📋 目录

1. [系统概述](#系统概述)
2. [时序架构](#时序架构)
3. [地图管理模块](#地图管理模块)
4. [探索状态机](#探索状态机)
5. [规划状态机](#规划状态机)
6. [视点生成机制](#视点生成机制)
7. [ROS接口](#ros接口)
8. [参数配置](#参数配置)

---

## 🎯 系统概述

STAR-Searcher 是一个完整的无人机自主探索系统，采用**双状态机协同**设计：

- **探索状态机** (ExplorationFSM): 负责探索策略、前沿提取、视点生成
- **规划状态机** (EGOReplanFSM): 负责轨迹规划、碰撞检测、执行控制

### 核心特性

| 特性 | 说明 |
|------|------|
| **地图系统** | STAR-MVF 双层地图（LocalMap滑动窗口 + GlobalMap子网格） |
| **探索策略** | EDEN历史节点 + 前沿聚类 + 圆周采样 |
| **重规划策略** | Falcon周期性重规划 + 前沿覆盖检测 |
| **轨迹优化** | EGO-Planner MINCO优化 + 实时避障 |
| **仿真支持** | Open3D/Gazebo/Real三种模式 |

### 系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                      STAR-Searcher 系统架构                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐       │
│  │ LiDAR点云   │     │  里程计信息  │     │  用户触发    │       │
│  │ /livox/lidar│     │  /odom      │     │  /goal       │       │
│  └──────┬──────┘     └──────┬──────┘     └──────┬──────┘       │
│         │                   │                   │               │
│         └───────────────────┼───────────────────┘               │
│                             ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │            🗺️ 地图管理模块 (MapManager)                  │   │
│  │  ┌──────────────────────┬──────────────────────┐        │   │
│  │  │   LocalMap (局部)     │   GlobalMap (全局)    │        │   │
│  │  │  - 滑动窗口圆形数组   │  - 子网格管理          │        │   │
│  │  │  - Log-odds更新      │  - 前沿提取            │        │   │
│  │  │  - 射线投射          │  - 膨胀+ESDF          │        │   │
│  │  └──────────┬───────────┴──────────┬───────────┘        │   │
│  │             │                      │                    │   │
│  │     lidarCallback()      syncTimerCallback(10Hz)       │   │
│  │             │                      │                    │   │
│  │             └──────────────────────┘                    │   │
│  └─────────────────────────┬───────────────────────────────┘   │
│                            │                                   │
│                            ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │        🔍 探索状态机 (ExplorationFSM) - 100Hz            │   │
│  │                                                         │   │
│  │  fsmCallback(100Hz)     ┌─────────────────────┐        │   │
│  │  ───────────────────►   │  主状态机循环        │        │   │
│  │                         │  - INIT             │        │   │
│  │  frontierCallback(2Hz)  │  - WAIT_TRIGGER     │        │   │
│  │  ───────────────────►   │  - UPDATE_FRONTIER  │        │   │
│  │                         │  - PLAN_VIEWPOINTS  │        │   │
│  │  safetyCallback(20Hz)   │  - EXEC_EXPLORATION │        │   │
│  │  ───────────────────►   │  - FINISH           │        │   │
│  │                         │  - EMERGENCY_STOP   │        │   │
│  │  visCallback(10Hz)      └─────────────────────┘        │   │
│  │  ───────────────────► 可视化发布                        │   │
│  └─────────────────────────┬───────────────────────────────┘   │
│                            │ /exploration/viewpoint           │
│                            │ (PositionCommand)                │
│                            ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │        🚀 规划状态机 (EGOReplanFSM) - 100Hz             │   │
│  │                                                         │   │
│  │  execFSMCallback(100Hz) ┌─────────────────────┐        │   │
│  │  ───────────────────►   │  主状态机循环        │        │   │
│  │                         │  - INIT             │        │   │
│  │  safetyCallback(20Hz)   │  - PRE_ARM          │        │   │
│  │  ───────────────────►   │  - TAKEOFF          │        │   │
│  │                         │  - WAIT_TARGET      │        │   │
│  │                         │  - GEN_NEW_TRAJ     │        │   │
│  │                         │  - REPLAN_TRAJ      │        │   │
│  │                         │  - EXEC_TRAJ        │        │   │
│  │                         │  - EMERGENCY_STOP   │        │   │
│  │                         └─────────────────────┘        │   │
│  └─────────────────────────┬───────────────────────────────┘   │
│                            │ /planning/pos_cmd                │
│                            │ (PositionCommand)                │
│                            ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │               飞控/控制器 (PX4/SO3)                       │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## ⏰ 时序架构

系统采用**多定时器并行**架构，不同模块独立运行，通过ROS话题通信。

### 🗺️ 地图管理模块 (MapManager)

**节点**: `exploration_planner_node`  
**文件**: `src/star_mvf/map/map_manager.cpp`

| 定时器 | 频率 | 功能 | 实现 |
|-------|------|------|------|
| **Lidar回调** | 点云频率 | 接收LiDAR点云 | `lidarCallback()` |
| **Odom回调** | 里程计频率 | 接收里程计数据 | `odomCallback()` |
| **同步定时器** | **10Hz** | 局部→全局地图同步 | `syncTimerCallback()` |
| **可视化定时器** | **5Hz** | 发布地图可视化 | `vizTimerCallback()` |

#### 数据流

```
LiDAR点云 ──►  lidarCallback()
                   │
                   ▼
            insertPointCloud()
                   │
                   ├─► 阶段1: 收集候选占据体素
                   │
                   ├─► 阶段2: 射线更新（遇占据终止）
                   │
                   └─► 阶段3: 更新真正的占据体素
                   
Odom ──► odomCallback()
              │
              ▼
         updateCenter()
              │
              └─► 滑动窗口更新（圆形数组）

syncTimerCallback(10Hz)
    │
    ├─► getCachedUpdates()  // 获取LocalMap缓存
    │
    ├─► updateOccupiedVoxels()  // 更新GlobalMap
    │
    ├─► updateInflation()  // 膨胀地图
    │
    ├─► updateESDF()  // 距离场
    │
    └─► extractFrontier()  // 前沿提取

vizTimerCallback(5Hz)
    │
    ├─► local_map_->visualize()  // 局部地图可视化
    │   ├─► 占据点云
    │   └─► 局部范围框
    │
    └─► global_map_->visualize()  // 全局地图可视化
        ├─► 占据/自由点云
        ├─► 前沿点云
        ├─► 子网格范围
        ├─► 全局地图范围
        └─► 访问体素
```

#### 关键性能优化

1. **降采样**: 点云降采样参数 `lidar_downsample_filter` (每N个点取1个)
2. **射线终止**: 遇到占据体素立即终止，避免覆盖近处障碍物
3. **缓存机制**: LocalMap缓存更新，批量同步到GlobalMap
4. **订阅发布**: 可视化话题按需发布（`getNumSubscribers() > 0`）

---

### 🔍 探索状态机 (ExplorationFSM)

**节点**: `exploration_planner_node`  
**文件**: `src/rog_map/exploration_fsm.cpp`

| 定时器 | 频率 | 功能 | 实现 |
|-------|------|------|------|
| **FSM主循环** | **100Hz** | 状态机主逻辑 | `fsmCallback()` |
| **前沿更新** | **2Hz** | 更新前沿信息 | `frontierCallback()` |
| **安全检查** | **20Hz** | 碰撞检测 | `safetyCallback()` |
| **可视化** | **10Hz** | 发布可视化 | `visualizationCallback()` |

#### 状态转换图

```
                    探索状态机 (ExplorationFSM)
                    
    INIT (初始化)
      │  等待里程计 (have_odom_)
      │  等待时间 > 2s
      ▼
    WAIT_TRIGGER (等待触发)
      │  用户触发 (RViz 2D Goal)
      │  或 auto_start=true
      ▼
    UPDATE_FRONTIER (更新前沿)
      │  调用 updateFrontierInfo()
      │  获取包含前沿的子网格
      ├──► 无前沿 ─────► FINISH
      │
      ▼ 有前沿
    PLAN_VIEWPOINTS (规划视点)
      │  调用 planExplorationViewpoints()
      │  ├─► 生成历史节点
      │  ├─► 前沿聚类
      │  ├─► 视点采样
      │  └─► 贪心排序
      ├──► 规划失败 ───► FINISH
      │
      ▼ 规划成功
    EXEC_EXPLORATION (执行探索) ◄───────┐
      │  发布目标视点                   │
      │  检查到达/覆盖/超时              │
      │                                │
      ├──► 到达 && 有下一个 ────────────┤
      ├──► 前沿覆盖 > 25% ─────► UPDATE_FRONTIER
      ├──► 超时 > 3s ──────────► UPDATE_FRONTIER
      │
      ▼ 完成所有视点
    FINISH (探索完成)
      │  等待5秒
      ▼
    返回 WAIT_TRIGGER
```

#### 状态详细说明

##### 1. INIT (初始化)

**触发条件**:
- 系统启动

**处理逻辑**:
```cpp
// fsmCallback() 中处理
void ExplorationFSM::processInit() {
    if (!have_odom_) return;  // 等待里程计
    
    double elapsed = (ros::Time::now() - fsm_init_time_).toSec();
    if (elapsed > config_.init_wait_time) {  // 默认2秒
        transitState(ExplorationState::WAIT_TRIGGER, "Init complete");
    }
}
```

**退出条件**:
- `have_odom_ = true` (收到里程计)
- 等待时间 > `init_wait_time` (默认2s)

##### 2. WAIT_TRIGGER (等待触发)

**触发方式**:
1. **RViz手动触发**: 使用"2D Nav Goal"工具点击任意位置
2. **自动触发**: 设置`auto_start=true`

**处理逻辑**:
```cpp
void ExplorationFSM::processWaitTrigger() {
    if (config_.auto_start) {
        triggered_ = true;
    }
    
    if (triggered_ && frontier_ready_ && has_frontier_) {
        transitState(ExplorationState::UPDATE_FRONTIER, "Triggered");
    }
}
```

**被动可视化模式**:
即使未触发，系统仍会在`frontierCallback()`中实时显示前沿和视点：
```cpp
// frontierCallback() 中
if (!triggered_ && config_.enable_visualization) {
    // 生成探索规划（仅可视化，不执行）
    viewpoint_generator_->generate(clusters, viewpoints);
    // 更新可视化数据
    visualize();
}
```

##### 3. UPDATE_FRONTIER (更新前沿)

**功能**: 从GlobalMap获取最新前沿信息

**处理逻辑**:
```cpp
void ExplorationFSM::processUpdateFrontier() {
    if (!updateFrontierInfo()) {
        // 无前沿，探索完成
        transitState(ExplorationState::FINISH, "No frontier");
        return;
    }
    
    // 有前沿，进入视点规划
    transitState(ExplorationState::PLAN_VIEWPOINTS, "Frontier updated");
}

bool ExplorationFSM::updateFrontierInfo() {
    // 从GlobalMap获取前沿子网格
    std::vector<int> frontier_subgrids;
    global_map_->getFrontierSubGridIndices(frontier_subgrids);
    
    // 统计前沿点数量
    int total_frontiers = countTotalFrontiers();
    
    if (total_frontiers < config_.min_frontier_cluster_size) {
        has_frontier_ = false;
        return false;
    }
    
    has_frontier_ = true;
    frontier_ready_ = true;
    return true;
}
```

##### 4. PLAN_VIEWPOINTS (规划视点)

**功能**: 调用视点生成器生成探索视点

**处理逻辑**:
```cpp
void ExplorationFSM::processPlanViewpoints() {
    if (!planExplorationViewpoints()) {
        transitState(ExplorationState::FINISH, "Plan failed");
        return;
    }
    
    transitState(ExplorationState::EXEC_EXPLORATION, "Plan success");
}

bool ExplorationFSM::planExplorationViewpoints() {
    // 更新UAV状态
    UAVState uav_state{odom_pos_.cast<float>(), odom_vel_.cast<float>(), 
                       static_cast<float>(odom_yaw_)};
    viewpoint_generator_->updateUAVState(uav_state);
    
    // 生成视点
    std::vector<FrontierCluster> clusters;
    std::vector<ExplorationViewpoint> viewpoints;
    
    if (!viewpoint_generator_->generate(clusters, viewpoints)) {
        return false;
    }
    
    // 贪心排序（从近到远）
    sortViewpointsByGreedy();
    
    // 保存到探索数据
    expl_data_.current_viewpoints = viewpoints;
    expl_data_.current_viewpoint_idx = 0;
    
    return !viewpoints.empty();
}
```

**贪心排序策略** (参考EDEN算法):
```cpp
void ExplorationFSM::sortViewpointsByGreedy() {
    std::vector<bool> visited(viewpoints.size(), false);
    std::vector<ExplorationViewpoint> sorted;
    sorted.reserve(viewpoints.size());
    
    Eigen::Vector3f current_pos = odom_pos_.cast<float>();
    
    for (size_t i = 0; i < viewpoints.size(); ++i) {
        float min_dist = std::numeric_limits<float>::max();
        int min_idx = -1;
        
        // 找最近的未访问视点
        for (size_t j = 0; j < viewpoints.size(); ++j) {
            if (visited[j]) continue;
            
            float dist = (viewpoints[j].position - current_pos).norm();
            
            // 避免近距离大转向：如果距离 < 2m 且 yaw变化 > 90°，增加惩罚
            float yaw_diff = std::abs(getYawDiff(viewpoints[j].yaw, current_yaw));
            if (dist < config_.min_viewpoint_distance && yaw_diff > config_.large_yaw_threshold) {
                dist += config_.large_yaw_penalty;  // 增加3m惩罚
            }
            
            // 考虑yaw变化代价
            dist += yaw_diff * config_.yaw_change_weight;  // 转1rad = 多飞0.5m
            
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = j;
            }
        }
        
        if (min_idx >= 0) {
            sorted.push_back(viewpoints[min_idx]);
            visited[min_idx] = true;
            current_pos = viewpoints[min_idx].position;
        }
    }
    
    viewpoints = sorted;
}
```

##### 5. EXEC_EXPLORATION (执行探索)

**功能**: 发布视点目标，检查到达、覆盖、超时

**处理逻辑**:
```cpp
void ExplorationFSM::processExecExploration() {
    // 1. 发布当前目标视点（如果尚未发布）
    if (!expl_data_.viewpoint_published) {
        ExplorationViewpoint& vp = expl_data_.current_viewpoints[expl_data_.current_viewpoint_idx];
        
        if (!publishViewpointTarget(vp)) {
            // 发布失败，尝试下一个视点
            if (!selectNextViewpoint()) {
                // 无下一个，返回更新前沿
                transitState(ExplorationState::UPDATE_FRONTIER, "No next viewpoint");
                return;
            }
            return;  // 继续尝试发布下一个
        }
        
        expl_data_.viewpoint_published = true;
        traj_start_time_ = ros::Time::now();
    }
    
    // 2. 检查到达
    if (isNearViewpoint(current_viewpoint)) {
        viewpoint_reached_ = true;
        expl_data_.total_viewpoints_visited++;
        
        // 选择下一个视点
        if (selectNextViewpoint()) {
            expl_data_.viewpoint_published = false;
            return;  // 继续执行
        }
        
        // 无下一个视点，返回更新前沿
        transitState(ExplorationState::UPDATE_FRONTIER, "All viewpoints visited");
        return;
    }
    
    // 3. 检查前沿覆盖（Falcon策略）
    double exec_time = (ros::Time::now() - traj_start_time_).toSec();
    if (exec_time > config_.replan_thresh_frontier) {  // 执行时间 > 0.5s
        if (checkFrontierCovered()) {
            transitState(ExplorationState::UPDATE_FRONTIER, "Frontier covered");
            return;
        }
    }
    
    // 4. 周期性重规划（Falcon策略）
    double since_last_replan = (ros::Time::now() - last_replan_time_).toSec();
    if (since_last_replan > config_.replan_thresh_time) {  // 超过3s
        last_replan_time_ = ros::Time::now();
        transitState(ExplorationState::UPDATE_FRONTIER, "Periodic replan");
    }
}
```

**前沿覆盖检测** (Falcon策略):
```cpp
bool ExplorationFSM::checkFrontierCovered() {
    int current_unknown = global_map_->getUnknownVoxelCount();
    
    if (initial_unknown_count_ == 0) {
        initial_unknown_count_ = current_unknown;
        return false;
    }
    
    float covered_ratio = 1.0f - static_cast<float>(current_unknown) / initial_unknown_count_;
    
    // 如果覆盖比例 > 25%，触发重规划
    if (covered_ratio > config_.frontier_covered_ratio) {
        PERF_INFO_FMT("FSM", "FRONTIER_COVERED", 
                      "Covered %.1f%% (unknown: %d -> %d)",
                      covered_ratio * 100.0f, initial_unknown_count_, current_unknown);
        return true;
    }
    
    return false;
}
```

**视点发布**:
```cpp
bool ExplorationFSM::publishViewpointTarget(const ExplorationViewpoint& vp) {
    Eigen::Vector3f current_pos = odom_pos_.cast<float>();
    
    // 检查是否有障碍物
    bool obstacle_detected = viewpoint_generator_->hasObstacleBetween(current_pos, vp.position);
    
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "map";
    
    cmd.position.x = vp.position.x();
    cmd.position.y = vp.position.y();
    cmd.position.z = vp.position.z();
    
    cmd.yaw = vp.yaw;
    cmd.velocity = {0.0, 0.0, 0.0};  // 终端速度为0（悬停）
    
    // 如果有障碍物，尝试找历史节点路径
    if (obstacle_detected) {
        std::vector<Eigen::Vector3f> intermediate_waypoints;
        
        if (viewpoint_generator_->findPathViaHistoryNodes(
                current_pos, vp.position, intermediate_waypoints)) {
            // 发布带中间航点的路径
            nav_msgs::Path path_msg;
            path_msg.header = cmd.header;
            
            // 添加中间航点
            for (const auto& wp : intermediate_waypoints) {
                geometry_msgs::PoseStamped pose;
                pose.header = cmd.header;
                pose.pose.position.x = wp.x();
                pose.pose.position.y = wp.y();
                pose.pose.position.z = wp.z();
                path_msg.poses.push_back(pose);
            }
            
            // 添加最终目标
            geometry_msgs::PoseStamped final_pose;
            final_pose.header = cmd.header;
            final_pose.pose.position = cmd.position;
            path_msg.poses.push_back(final_pose);
            
            viewpoint_path_pub_.publish(path_msg);
            PERF_INFO_FMT("FSM", "PATH_PUBLISH", "Published path with %zu waypoints",
                         intermediate_waypoints.size());
        } else {
            // 无路径但仍发布单点，让EGO-Planner尝试直接规划
            PERF_WARN("FSM", "Obstacle detected but no path found, EGO-Planner will attempt direct path.");
        }
    }
    
    // 无论如何都发布目标点
    viewpoint_pub_.publish(cmd);
    
    PERF_INFO_FMT("FSM", "VIEWPOINT_PUBLISH", "pos=(%.2f,%.2f,%.2f), yaw=%.2f",
                 vp.position.x(), vp.position.y(), vp.position.z(), vp.yaw);
    
    return true;
}
```

##### 6. FINISH (探索完成)

**功能**: 探索完成，输出统计信息

**处理逻辑**:
```cpp
void ExplorationFSM::processFinish() {
    static bool finish_handled = false;
    static ros::Time finish_time;
    
    if (!finish_handled) {
        finish_time = ros::Time::now();
        finish_handled = true;
        
        // 输出统计信息
        double total_time = (finish_time - expl_data_.start_time).toSec();
        
        PERF_INFO_FMT("FSM", "EXPLORATION_FINISH",
                     "Total time: %.1fs, Viewpoints visited: %d, Distance: %.1fm",
                     total_time, expl_data_.total_viewpoints_visited,
                     expl_data_.total_distance);
        
        // 清除可视化
        clearVisualization();
    }
    
    // 5秒后返回等待状态
    if ((ros::Time::now() - finish_time).toSec() > 5.0) {
        finish_handled = false;
        transitState(ExplorationState::WAIT_TRIGGER, "Finish timeout");
    }
}
```

##### 7. EMERGENCY_STOP (紧急停止)

**触发条件**:
- 检测到碰撞风险 (safetyCallback)
- 用户手动触发

**处理逻辑**:
```cpp
void ExplorationFSM::processEmergencyStop() {
    // 停止所有运动，发布hover命令
    // 等待手动恢复
}
```

---


## 🚀 规划状态机 (EGOReplanFSM)

**节点**: `exploration_planner_node`  
**文件**: `src/ego_planner/ego_replan_fsm.cpp`

| 定时器 | 频率 | 功能 | 实现 |
|-------|------|------|------|
| **FSM主循环** | **100Hz** | 状态机主逻辑 | `execFSMCallback()` |
| **安全检查** | **20Hz** | 碰撞检测 | `checkCollisionCallback()` |

### 状态转换图

```
                    规划状态机 (EGOReplanFSM)
                    
    INIT (初始化)
      │  等待里程计 (have_odom_)
      ▼
    ┌─────────────────────────────────────┐
    │ 根据 sim_mode 选择不同的启动流程     │
    ├─────────────────────────────────────┤
    │ SIM_OPEN3D (1): 跳过 PRE_ARM        │
    │ SIM_GAZEBO (0): 自动解锁起飞         │
    │ SIM_REAL (2):   等待手动解锁         │
    └─────────────────────────────────────┘
      │
      ▼
    PRE_ARM (解锁准备)  ←── 仅 Gazebo/Real
      │  SIM_GAZEBO: 自动请求 OFFBOARD + ARM
      │  SIM_REAL:   等待手动 OFFBOARD + ARM
      ▼
    TAKEOFF (起飞)
      │  发布起飞命令，匀加速到目标高度
      │  t ∈ [0, takeoff_accel_time]
      │  z(t) = z0 + 0.5 * a * t²
      ├──► 起飞完成 (z > takeoff_height - 0.1)
      ▼
    WAIT_TARGET (等待目标)
      │  等待探索视点 (/exploration/viewpoint)
      │  或预设路径点 (waypoint_num > 0)
      ├──► 收到目标 ────────────────┐
      │                            │
      ▼                            ▼
    GEN_NEW_TRAJ (生成新轨迹)      │
      │  planFromGlobalTraj()      │
      │  ├─► A* 搜索               │
      │  ├─► 轨迹初始化             │
      │  └─► MINCO 优化            │
      ├──► 规划成功 ────────────────┤
      │                            │
      ▼                            │
    EXEC_TRAJ (执行轨迹) ◄──────────┘
      │  发布 PositionCommand       │
      │  监控轨迹执行                │
      │                            │
      ├──► 碰撞风险 ───────► EMERGENCY_STOP
      │                            │
      ├──► 接近终点 ────────────────┤
      │    && 有下一个目标           │
      │                            │
      ├──► 到达终点 ────────► WAIT_TARGET
      │                            │
      ▼                            │
    REPLAN_TRAJ (重规划) ◄──────────┘
      │  从当前位置重新规划          │
      │  planFromLocalTraj()        │
      ├──► 规划成功 ───────► EXEC_TRAJ
      └──► 规划失败 ───────► EMERGENCY_STOP
      
    EMERGENCY_STOP (紧急停止)
      │  悬停在当前位置
      │  等待安全恢复 (1s检查 + 5s超时)
      ├──► 安全恢复 ───────► REPLAN_TRAJ
      └──► 超时 ──────────► WAIT_TARGET
```

### 状态详细说明

#### 1. INIT → PRE_ARM → TAKEOFF (启动流程)

**三种仿真模式**:

| 模式 | sim_mode | FCU | 启动流程 |
|------|----------|-----|----------|
| **Open3D** | 1 | 无 | INIT → TAKEOFF (跳过PRE_ARM) |
| **Gazebo** | 0 | PX4 SITL | INIT → PRE_ARM (自动解锁) → TAKEOFF |
| **Real** | 2 | 真实PX4 | INIT → PRE_ARM (等待手动) → TAKEOFF |

**Open3D模式** (无FCU):
```cpp
// INIT状态中
if (sim_mode_ == SIM_OPEN3D) {
    offboard_ready_ = true;
    takeoff_start_time_ = ros::Time::now();
    hover_pos_ = Eigen::Vector3d(initial_pos_.x(), initial_pos_.y(), 
                                  initial_pos_.z() + takeoff_height_);
    hover_yaw_ = odom_yaw_;
    hover_pos_set_ = true;
    changeFSMExecState(TAKEOFF, "FSM");
}
```

**Gazebo模式** (自动解锁):
```cpp
// PRE_ARM状态中
if (sim_mode_ == SIM_GAZEBO) {
    // 等待连接
    if (!current_state_.connected) goto force_return;
    
    // 持续发送hover命令（PX4要求）
    publishHoverCmd();
    
    // 自动请求OFFBOARD模式
    if (current_state_.mode != "OFFBOARD") {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client_.call(offb_set_mode);
    }
    
    // 自动请求解锁
    if (!current_state_.armed) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        arming_client_.call(arm_cmd);
    }
    
    // 解锁成功
    if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
        offboard_ready_ = true;
        changeFSMExecState(TAKEOFF, "FSM");
    }
}
```

**Real模式** (等待手动):
```cpp
// PRE_ARM状态中
if (sim_mode_ == SIM_REAL) {
    // 持续发送hover命令
    publishHoverCmd();
    
    // 等待飞手手动解锁和切换OFFBOARD
    if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
        offboard_ready_ = true;
        changeFSMExecState(TAKEOFF, "FSM");
        ROS_INFO("[EGOReplanFSM] REAL mode: Manual arming detected");
    }
}
```

**起飞命令** (匀加速):
```cpp
void EGOReplanFSM::publishTakeoffCmd() {
    double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
    double t = std::min(elapsed, takeoff_accel_time_);  // 限制在加速时间内
    
    // 匀加速运动: z(t) = z0 + 0.5 * a * t²
    // a = 2 * h / T²
    double accel = 2.0 * takeoff_height_ / (takeoff_accel_time_ * takeoff_accel_time_);
    double z_offset = 0.5 * accel * t * t;
    
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.position.x = hover_pos_.x();
    cmd.position.y = hover_pos_.y();
    cmd.position.z = initial_pos_.z() + z_offset;
    cmd.velocity.z = accel * t;  // v(t) = a * t
    cmd.yaw = hover_yaw_;
    
    pos_cmd_pub_.publish(cmd);
}
```

#### 2. WAIT_TARGET (等待目标)

**接收探索视点**:
```cpp
void EGOReplanFSM::viewpointCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
    // 保存完整的视点信息（位置、yaw、速度）
    end_pt_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    end_yaw_ = msg->yaw;
    end_vel_ = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    
    have_target_ = true;
    have_new_target_ = true;
    
    // 如果在EMERGENCY_STOP状态，尝试恢复
    if (exec_state_ == EMERGENCY_STOP) {
        changeFSMExecState(REPLAN_TRAJ, "new target in emergency");
    }
}
```

**接收带中间航点的路径**:
```cpp
void EGOReplanFSM::viewpointPathCallback(const nav_msgs::PathConstPtr &msg) {
    if (msg->poses.empty()) return;
    
    wps_.clear();
    
    // 解析中间航点
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const auto& pose = msg->poses[i];
        Eigen::Vector3d pos(pose.pose.position.x, 
                            pose.pose.position.y, 
                            pose.pose.position.z);
        
        // 从四元数提取yaw
        double yaw = tf::getYaw(pose.pose.orientation);
        
        // 终点期望速度为0，中间点可以有速度
        Eigen::Vector3d vel = (i == msg->poses.size() - 1) ? 
                              Eigen::Vector3d::Zero() : Eigen::Vector3d(0.5, 0, 0);
        
        wps_.emplace_back(pos, yaw, vel);
    }
    
    have_target_ = true;
    have_new_target_ = true;
}
```

#### 3. GEN_NEW_TRAJ / REPLAN_TRAJ (轨迹规划)

**关键修复：重规划起点**

```cpp
// planFromLocalTraj 和 planFromGlobalTraj 中
if (exec_state_ == REPLAN_TRAJ || exec_state_ == EMERGENCY_STOP) {
    // 紧急重规划或从急停恢复，从当前里程计位置开始
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_ = Eigen::Vector3d::Zero();
    planner_manager_->setReplanStartTime(ros::Time::now().toSec() + replan_duration_);
    
} else if (planner_manager_->traj_.local_traj.traj_id > 0) {
    // 正常执行中，从轨迹上的预测点开始
    double t_cur = ros::Time::now().toSec() - planner_manager_->traj_.local_traj.start_time;
    t_cur = std::max(0.0, t_cur);
    t_cur += replan_duration_;  // 预测 replan_duration_ 秒后的位置
    
    start_pt_ = planner_manager_->traj_.local_traj.traj.getPos(t_cur);
    start_vel_ = planner_manager_->traj_.local_traj.traj.getVel(t_cur);
    start_acc_ = planner_manager_->traj_.local_traj.traj.getAcc(t_cur);
    
} else {
    // 无历史轨迹，从里程计开始
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_ = Eigen::Vector3d::Zero();
}
```

**规划流程**:
```cpp
bool EGOReplanFSM::planFromGlobalTraj(const int trial_times) {
    // 1. A* 搜索
    std::vector<Eigen::Vector3d> grid_path;
    bool success = planner_manager_->gridPathSearch(start_pt_, end_pt_, grid_path);
    if (!success) return false;
    
    // 2. 轨迹初始化
    planner_manager_->initTrajectory(grid_path, start_vel_, end_vel_, end_yaw_);
    
    // 3. MINCO优化
    success = planner_manager_->reboundOptimize(trial_times);
    if (!success) return false;
    
    // 4. 发布轨迹
    rog_map::PolyTraj poly_msg;
    polyTraj2ROSMsg(poly_msg);
    poly_traj_pub_.publish(poly_msg);
    
    return true;
}
```

#### 4. EXEC_TRAJ (执行轨迹)

**发布位置命令**:
```cpp
// execFSMCallback 中 EXEC_TRAJ 状态
double t_cur = ros::Time::now().toSec() - planner_manager_->traj_.local_traj.start_time;

// 安全检查：时间是否在有效范围内
if (t_cur < 0.0) t_cur = 0.0;
if (t_cur > planner_manager_->traj_.local_traj.duration) {
    t_cur = planner_manager_->traj_.local_traj.duration;
}

// 从轨迹采样
Eigen::Vector3d pos = planner_manager_->traj_.local_traj.traj.getPos(t_cur);
Eigen::Vector3d vel = planner_manager_->traj_.local_traj.traj.getVel(t_cur);
Eigen::Vector3d acc = planner_manager_->traj_.local_traj.traj.getAcc(t_cur);
double yaw = planner_manager_->traj_.local_traj.traj.getYaw(t_cur);

// 发布命令
quadrotor_msgs::PositionCommand cmd;
cmd.header.stamp = ros::Time::now();
cmd.position.x = pos.x();
cmd.position.y = pos.y();
cmd.position.z = pos.z();
cmd.velocity.x = vel.x();
cmd.velocity.y = vel.y();
cmd.velocity.z = vel.z();
cmd.acceleration.x = acc.x();
cmd.acceleration.y = acc.y();
cmd.acceleration.z = acc.z();
cmd.yaw = yaw;
cmd.yaw_dot = 0.0;

pos_cmd_pub_.publish(cmd);
```

**到达检测**:
```cpp
// 检查是否接近终点
double t_cur = ros::Time::now().toSec() - planner_manager_->traj_.local_traj.start_time;
double duration = planner_manager_->traj_.local_traj.duration;

if (t_cur > duration - 1e-2) {  // 到达终点
    have_target_ = false;
    changeFSMExecState(WAIT_TARGET, "FSM");
    
    // 如果有下一个航点
    if (!wps_.empty()) {
        planNextWaypoint(wps_.front(), previous_wp);
        wps_.erase(wps_.begin());
    }
}
```

#### 5. EMERGENCY_STOP (紧急停止)

**触发条件** (checkCollisionCallback):
```cpp
void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e) {
    if (exec_state_ != EXEC_TRAJ) return;
    
    // 获取当前轨迹上的位置
    double t_cur = ros::Time::now().toSec() - planner_manager_->traj_.local_traj.start_time;
    if (t_cur < 0.0 || t_cur > planner_manager_->traj_.local_traj.duration) return;
    
    Eigen::Vector3d traj_pos = planner_manager_->traj_.local_traj.traj.getPos(t_cur);
    
    // 检查碰撞
    if (map_manager_->getGlobalMap()->isOccupiedInflate(traj_pos.cast<float>())) {
        PERF_ERROR_FMT("PLAN", "COLLISION_DETECTED",
                      "Collision at traj_pos=(%.2f,%.2f,%.2f), odom_pos=(%.2f,%.2f,%.2f), t_cur=%.3f",
                      traj_pos.x(), traj_pos.y(), traj_pos.z(),
                      odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), t_cur);
        
        callEmergencyStop(odom_pos_);
    }
}

bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos) {
    changeFSMExecState(EMERGENCY_STOP, "safety");
    
    // 设置悬停位置
    hover_pos_ = stop_pos;
    hover_yaw_ = odom_yaw_;
    hover_pos_set_ = true;
    
    emergency_start_time_ = ros::Time::now();
    
    return true;
}
```

**恢复机制**:
```cpp
// execFSMCallback 中 EMERGENCY_STOP 状态
double elapsed = (ros::Time::now() - emergency_start_time_).toSec();

// 持续发布悬停命令
publishHoverCmd();

// 1s后开始检查是否可以恢复
if (elapsed > 1.0) {
    // 检查周围是否安全
    bool is_safe = !map_manager_->getGlobalMap()->isOccupiedInflate(odom_pos_.cast<float>());
    
    if (is_safe && have_target_) {
        // 安全且有目标，尝试重新规划
        changeFSMExecState(REPLAN_TRAJ, "emergency recovery");
        return;
    }
}

// 5s超时，返回等待目标
if (elapsed > 5.0) {
    changeFSMExecState(WAIT_TARGET, "emergency timeout");
}
```

---

## 🎯 视点生成机制

**文件**: `src/rog_map/frontier_viewpoint_generator.cpp`

视点生成采用**EDEN历史节点**策略 + **前沿聚类** + **圆周采样**。

### 完整流程

```
generate() 函数流程
    │
    ├─► Step 0: 更新历史节点 (updateHistoryNodes)
    │   ├─► 检查UAV移动距离
    │   └─► 创建新历史节点（间隔3m）
    │
    ├─► Step 1: 获取前沿点 (getFrontierPositionsBySubGrid)
    │   └─► 从GlobalMap按子网格分组
    │
    ├─► Step 2: 过滤前沿点 (filterFrontierPoints)
    │   ├─► 地面噪声过滤 (z < ground_height + 0.1)
    │   ├─► 无人机周围过滤 (头顶区域)
    │   └─► 天花板过滤 (z > ceil_height)
    │
    ├─► Step 3: 关联历史节点 (findNearestHistoryNode)
    │   └─► 每个子网格关联到最近的历史节点
    │
    ├─► Step 4: 前沿聚类 (clusterFrontierPoints)
    │   ├─► BFS区域增长
    │   └─► 同一历史节点的前沿点一起聚类
    │
    ├─► Step 5: 簇分裂 (splitLargeClusters)
    │   └─► 超过阈值（3m）的大簇二分
    │
    ├─► Step 6: 视点采样 (sampleViewpoints)
    │   ├─► 圆周采样（多半径+多角度）
    │   ├─► 快速安全检查
    │   └─► Top-K候选点
    │
    └─► Step 7: 去重滤波 (filterViewpoints)
        └─► 距离过滤（min_distance=0.8m）
```

### 历史节点系统 (EDEN算法)

**设计动机**:
- 原始子网格分区会导致相邻边界上的前沿被错误分割
- 历史节点沿UAV轨迹采集，自然地关联空间上相邻的前沿区域

**核心数据结构**:
```cpp
struct HistoryNode {
    int node_id;                              // 节点ID
    Eigen::Vector3f position;                 // 节点位置
    float timestamp;                          // 时间戳
    std::vector<int> associated_subgrid_ids;  // 关联的子网格
    std::vector<int> neighbors;               // 邻居节点ID（用于路径搜索）
    int parent_node_id;                       // 创建时的父节点
    
    void addNeighbor(int id);                 // 添加邻居（双向）
    bool hasNeighbor(int id) const;           // 检查是否已有邻居
};
```

**创建策略** (增量连通性):
```cpp
void FrontierViewpointGenerator::updateHistoryNodes() {
    Eigen::Vector3f current_pos = uav_state_.position;
    
    // 检查是否需要创建新节点
    float min_dist_to_existing = std::numeric_limits<float>::max();
    int nearest_node_id = -1;
    
    for (const auto& node : history_nodes_) {
        float dist = (node.position - current_pos).norm();
        if (dist < min_dist_to_existing) {
            min_dist_to_existing = dist;
            nearest_node_id = node.node_id;
        }
    }
    
    // 距离足够远，创建新节点
    if (min_dist_to_existing > config_.history_node_distance) {  // 默认3m
        HistoryNode new_node;
        new_node.node_id = history_nodes_.size();
        new_node.position = current_pos;
        new_node.timestamp = ros::Time::now().toSec();
        new_node.parent_node_id = nearest_node_id;
        
        // 关键：增量连通性构建
        // 新节点连接到最近的可达历史节点
        if (nearest_node_id >= 0) {
            if (!hasObstacleBetween(current_pos, history_nodes_[nearest_node_id].position)) {
                // 双向添加邻居
                new_node.addNeighbor(nearest_node_id);
                history_nodes_[nearest_node_id].addNeighbor(new_node.node_id);
            }
        }
        
        history_nodes_.push_back(new_node);
    }
}
```

**路径搜索** (BFS on预计算图):
```cpp
bool FrontierViewpointGenerator::findPathViaHistoryNodes(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end,
    std::vector<Eigen::Vector3f>& waypoints)
{
    waypoints.clear();
    
    // 1. 找到起点和终点最近的历史节点
    int start_idx = findNearestReachableHistoryNode(start);
    int end_idx = findNearestReachableHistoryNode(end);
    
    if (start_idx < 0 || end_idx < 0) return false;
    
    // 2. BFS搜索（使用预计算的neighbors，不重新检测障碍物）
    std::queue<int> q;
    std::vector<bool> visited(history_nodes_.size(), false);
    std::vector<int> parent(history_nodes_.size(), -1);
    
    q.push(start_idx);
    visited[start_idx] = true;
    
    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        
        if (curr == end_idx) break;
        
        // 遍历预计算的邻居（不重新检测障碍物！）
        for (int neighbor_id : history_nodes_[curr].neighbors) {
            if (!visited[neighbor_id]) {
                visited[neighbor_id] = true;
                parent[neighbor_id] = curr;
                q.push(neighbor_id);
            }
        }
    }
    
    if (!visited[end_idx]) return false;
    
    // 3. 回溯路径
    std::vector<int> path;
    int curr = end_idx;
    while (curr != -1) {
        path.push_back(curr);
        curr = parent[curr];
    }
    std::reverse(path.begin(), path.end());
    
    // 4. 简化路径（直线可达则跳过中间节点）
    simplifyPath(path);
    
    // 5. 转换为航点
    for (int idx : path) {
        waypoints.push_back(history_nodes_[idx].position);
    }
    
    return true;
}
```

**障碍物检测** (用于连通性判断):
```cpp
bool FrontierViewpointGenerator::hasObstacleBetween(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end)
{
    Eigen::Vector3f dir = end - start;
    float dist = dir.norm();
    if (dist < 1e-6f) return false;
    
    dir.normalize();
    float step = config_.raycast_step;  // 0.1m
    int num_steps = static_cast<int>(dist / step);
    
    // 关键修改：需要连续两个占据体素才判定为障碍物
    // 这使得算法对稀疏点云和未知区域更鲁棒
    bool last_occupied = false;
    
    for (int i = 0; i < num_steps; ++i) {
        Eigen::Vector3f pos = start + dir * (i * step);
        
        bool curr_occupied = global_map_->isOccupied(pos);
        
        if (curr_occupied && last_occupied) {
            // 连续两个占据体素，确认有障碍物
            return true;
        }
        
        last_occupied = curr_occupied;
    }
    
    // 检查终点
    bool end_occupied = global_map_->isOccupied(end);
    if (end_occupied && last_occupied) {
        return true;
    }
    
    return false;
}
```

### 前沿点过滤

**地面噪声过滤**:
```cpp
void FrontierViewpointGenerator::filterFrontierPoints(
    std::vector<Eigen::Vector3f>& points)
{
    auto it = std::remove_if(points.begin(), points.end(),
        [this](const Eigen::Vector3f& pt) {
            // 过滤地面附近
            if (pt.z() < config_.ground_height + config_.ground_offset) {
                return true;  // 移除
            }
            return false;
        });
    
    points.erase(it, points.end());
}
```

**头顶区域过滤** (簇级别):
```cpp
// clusterFrontierPoints 中
for (const auto& cluster : raw_clusters) {
    // 检查簇是否在UAV头顶
    Eigen::Vector3f uav_to_cluster = cluster.centroid - uav_state_.position;
    float horizontal_dist = std::sqrt(uav_to_cluster.x() * uav_to_cluster.x() +
                                      uav_to_cluster.y() * uav_to_cluster.y());
    float vertical_dist = std::abs(uav_to_cluster.z());
    
    // 过滤条件：水平距离 < 2m 且垂直距离 > 0.5m
    if (horizontal_dist < config_.overhead_distance_threshold &&
        vertical_dist > config_.overhead_height_threshold) {
        // 跳过头顶区域的簇
        continue;
    }
    
    filtered_clusters.push_back(cluster);
}
```

### 前沿聚类 (BFS区域增长)

```cpp
std::vector<FrontierCluster> FrontierViewpointGenerator::clusterFrontierPoints(
    const std::vector<Eigen::Vector3f>& points)
{
    std::vector<FrontierCluster> clusters;
    std::vector<bool> visited(points.size(), false);
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;
        
        // 创建新簇
        std::queue<size_t> q;
        std::vector<Eigen::Vector3f> cluster_points;
        
        q.push(i);
        visited[i] = true;
        
        // BFS扩展
        while (!q.empty()) {
            size_t curr_idx = q.front();
            q.pop();
            
            cluster_points.push_back(points[curr_idx]);
            
            // 查找邻居
            for (size_t j = 0; j < points.size(); ++j) {
                if (visited[j]) continue;
                
                float dist = (points[j] - points[curr_idx]).norm();
                if (dist <= config_.cluster_eps) {  // 默认1.0m
                    visited[j] = true;
                    q.push(j);
                }
            }
        }
        
        // 过滤小簇
        if (cluster_points.size() >= static_cast<size_t>(config_.min_cluster_size)) {
            FrontierCluster cluster;
            cluster.points = cluster_points;
            cluster.computeCentroidAndExtent();
            clusters.push_back(cluster);
        }
    }
    
    return clusters;
}
```

### 簇分裂

```cpp
void FrontierViewpointGenerator::splitLargeClusters(
    std::vector<FrontierCluster>& clusters)
{
    std::vector<FrontierCluster> result;
    
    for (const auto& cluster : clusters) {
        if (cluster.extent > config_.cluster_split_threshold) {  // 默认3.0m
            // 沿X轴二分
            Eigen::Vector3f center = cluster.centroid;
            float extent = cluster.extent;
            
            FrontierCluster c1, c2;
            c1.centroid = center - Eigen::Vector3f(0.25f * extent, 0, 0);
            c2.centroid = center + Eigen::Vector3f(0.25f * extent, 0, 0);
            c1.extent = c2.extent = extent * 0.5f;
            
            result.push_back(c1);
            result.push_back(c2);
        } else {
            result.push_back(cluster);
        }
    }
    
    clusters = result;
}
```

### 视点采样 (圆周采样)

```cpp
std::vector<ExplorationViewpoint> FrontierViewpointGenerator::sampleViewpoints(
    const FrontierCluster& cluster)
{
    std::vector<ExplorationViewpoint> candidates;
    
    Eigen::Vector3f center = cluster.centroid;
    
    // 多半径采样
    for (int r_idx = 0; r_idx < config_.candidate_rnum; ++r_idx) {
        float r = config_.candidate_rmin + 
                  (config_.candidate_rmax - config_.candidate_rmin) * 
                  r_idx / std::max(1, config_.candidate_rnum - 1);
        
        // 圆周角度采样
        int num_samples = static_cast<int>(2.0f * M_PI / config_.candidate_dphi);
        
        for (int i = 0; i < num_samples; ++i) {
            float phi = i * config_.candidate_dphi;
            
            // 生成候选位置
            Eigen::Vector3f pos(
                center.x() + r * std::cos(phi),
                center.y() + r * std::sin(phi),
                config_.viewpoint_height  // 固定高度
            );
            
            // 快速安全检查
            if (!isPositionSafeForExploration(pos)) continue;
            
            // 计算yaw（朝向簇中心）
            float yaw = std::atan2(center.y() - pos.y(), center.x() - pos.x());
            
            // 快速评分（距离+角度）
            float dist = (pos - center).norm();
            float dist_score = 1.0f - std::abs(dist - config_.optimal_distance) / 
                                      config_.sensor_range_max;
            
            float score = 0.6f * dist_score + 0.4f * 1.0f;  // 角度先给1.0
            
            ExplorationViewpoint vp;
            vp.position = pos;
            vp.yaw = yaw;
            vp.pitch = 0.0f;
            vp.score = score;
            vp.cluster_id = cluster.cluster_id;
            
            candidates.push_back(vp);
        }
    }
    
    // Top-K筛选（默认K=10）
    std::sort(candidates.begin(), candidates.end(),
        [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
            return a.score > b.score;
        });
    
    if (candidates.size() > 10) {
        candidates.resize(10);
    }
    
    return candidates;
}
```

### 安全性检查

```cpp
bool FrontierViewpointGenerator::isPositionSafeForExploration(
    const Eigen::Vector3f& pos)
{
    // 1. 地图范围检查
    if (!global_map_->isInMapRange(pos)) return false;
    
    // 2. 位置自身检查
    if (global_map_->isOccupiedInflate(pos)) return false;
    if (global_map_->isUnknownInflate(pos)) return false;
    
    // 3. 高度检查
    if (pos.z() < config_.viewpoint_height_min ||
        pos.z() > config_.viewpoint_height_max) {
        return false;
    }
    
    // 4. 安全半径检查（UAV半径 + 安全边距）
    float safety_radius = config_.uav_radius + config_.safety_margin;
    int num_checks = 8;  // 8方向
    
    for (int i = 0; i < num_checks; ++i) {
        float angle = i * 2.0f * M_PI / num_checks;
        Eigen::Vector3f offset(
            safety_radius * std::cos(angle),
            safety_radius * std::sin(angle),
            0.0f
        );
        
        Eigen::Vector3f check_pos = pos + offset;
        
        if (global_map_->isOccupiedInflate(check_pos)) return false;
        if (global_map_->isUnknownInflate(check_pos)) return false;
    }
    
    return true;
}
```

### 去重滤波

```cpp
void FrontierViewpointGenerator::filterViewpoints(
    std::vector<ExplorationViewpoint>& viewpoints)
{
    // 按评分降序排序
    std::sort(viewpoints.begin(), viewpoints.end(),
        [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
            return a.score > b.score;
        });
    
    std::vector<ExplorationViewpoint> filtered;
    
    for (const auto& vp : viewpoints) {
        bool too_close = false;
        
        for (const auto& existing : filtered) {
            float dist = (vp.position - existing.position).norm();
            if (dist < config_.min_viewpoint_distance) {  // 默认0.8m
                too_close = true;
                break;
            }
        }
        
        if (!too_close) {
            filtered.push_back(vp);
        }
    }
    
    viewpoints = filtered;
}
```

---

## 📡 ROS接口

### 输入话题

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/odom` | `nav_msgs/Odometry` | 50-100Hz | 无人机里程计 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10Hz | LiDAR点云 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 事件 | 探索触发/手动目标 |

### 输出话题 - 地图模块

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/exploration_planner_node/local_map/occupied` | `sensor_msgs/PointCloud2` | 5Hz | 局部占据点云 |
| `/exploration_planner_node/local_map/range` | `visualization_msgs/Marker` | 5Hz | 局部地图范围 |
| `/exploration_planner_node/global_map/occupied` | `sensor_msgs/PointCloud2` | 5Hz | 全局占据点云 |
| `/exploration_planner_node/global_map/free` | `sensor_msgs/PointCloud2` | 5Hz | 全局自由点云 |
| `/exploration_planner_node/global_map/frontier` | `sensor_msgs/PointCloud2` | 5Hz | 前沿点云 |
| `/exploration_planner_node/global_map/range` | `visualization_msgs/Marker` | 5Hz | 全局地图范围 |
| `/exploration_planner_node/global_map/sub_grid_range` | `visualization_msgs/MarkerArray` | 5Hz | 子网格范围 |

### 输出话题 - 探索模块

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/exploration/viewpoint` | `quadrotor_msgs::PositionCommand` | 事件 | 目标视点（完整状态） |
| `/exploration/viewpoint_path` | `nav_msgs::Path` | 事件 | 带中间航点的路径 |
| `/exploration/frontier_cloud` | `sensor_msgs/PointCloud2` | 10Hz | 前沿点云 |
| `/exploration/frontier_clusters` | `visualization_msgs/MarkerArray` | 10Hz | 前沿簇可视化 |
| `/exploration/viewpoints` | `visualization_msgs/MarkerArray` | 10Hz | 视点可视化 |
| `/exploration/frustums` | `visualization_msgs/MarkerArray` | 10Hz | 视锥体可视化 |
| `/exploration/state` | `std_msgs/Int8` | 10Hz | 探索状态 |

### 输出话题 - 规划模块

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 100Hz | 位置控制命令 |
| `/planning/trajectory` | `rog_map/PolyTraj` | 事件 | MINCO多项式轨迹 |
| `/planning/replan` | `std_msgs::Int8` | 事件 | 重规划触发信号 |

---

## ⚙️ 参数配置

### 地图参数 (`exploration_open3d.yaml`)

```yaml
# ========== 分辨率 ==========
resolution: 0.1              # 体素分辨率 [m]

# ========== 局部地图 ==========
local_map:
  half_size: [10.0, 10.0, 3.0]  # 局部地图半尺寸 [m]
  log_odds_hit: 0.85          # 占据更新值
  log_odds_miss: -0.4         # 自由更新值
  log_odds_min: -2.0          # 最小log-odds
  log_odds_max: 3.5           # 最大log-odds
  log_odds_occ_thresh: 0.0    # 占据阈值
  use_raycast: true           # 是否使用射线投射
  raycast_step: 0.1           # 射线步长 [m]

# ========== 传感器 ==========
sensor:
  lidar_max_range: 20.0       # LiDAR最大距离 [m]
  lidar_min_range: 0.2        # LiDAR最小距离 [m]
  lidar_downsample_filter: 2  # 降采样因子（每N个点取1个）
  camera_max_range: 5.0       # 相机最大距离 [m]
  camera_min_range: 0.5       # 相机最小距离 [m]

# ========== 全局地图 ==========
global_map:
  map_size: [50, 50, 2.0]     # 全局地图尺寸 [m]
  ground_height: 0.40         # 地面高度 [m]
  ceil_height: 9990           # 天花板高度 [m]
  sub_grid_xy: 0.03           # 子网格比例因子
  frontier_height_min: 0.7    # 前沿点最低高度 [m]
  frontier_height_max: 2.5    # 前沿点最高高度 [m]

# ========== 膨胀地图 ==========
inflation_resolution: 0.2     # 膨胀分辨率 [m]
inflation_radius: 0.3         # 膨胀半径 [m]
enable_inflation: true        # 是否启用膨胀

# ========== 同步与可视化 ==========
sync_rate: 10.0               # 地图同步频率 [Hz]
viz_rate: 5.0                 # 可视化频率 [Hz]
visualization_en: true        # 是否启用可视化
visualization_range: [20.0, 20.0, 6.0]  # 可视化范围 [m]
```

### 探索参数 (`exploration_open3d.yaml`)

```yaml
exploration:
  # ========== FSM频率 ==========
  fsm_frequency: 100.0            # FSM主循环 [Hz]
  frontier_update_frequency: 2.0  # 前沿更新 [Hz]
  safety_check_frequency: 20.0    # 安全检查 [Hz]
  visualization_frequency: 10.0   # 可视化 [Hz]
  
  # ========== Falcon重规划策略 ==========
  replan_thresh_reached: 0.5         # 到达阈值 [m]
  replan_thresh_yaw: 0.1             # 偏航阈值 [rad]
  replan_thresh_time: 3.0            # 周期性重规划时间 [s]
  replan_thresh_frontier: 0.5        # 最小执行时间 [s]
  replan_thresh_traj_remain: 0.5     # 轨迹剩余时间 [s]
  frontier_covered_ratio: 0.25       # 前沿覆盖比例 (25%)
  
  # ========== 探索控制 ==========
  auto_start: false               # 是否自动开始
  init_wait_time: 2.0             # 初始化等待 [s]
  min_frontier_cluster_size: 50   # 最小前沿簇大小
  
  # ========== 视点选择（避免近距离大转向）==========
  min_viewpoint_distance: 2.0     # 近距离阈值 [m]
  yaw_change_weight: 0.5          # yaw代价权重 [m/rad]
  large_yaw_threshold: 1.57       # 大转向阈值 [rad] (90°)
  large_yaw_penalty: 3.0          # 近距离大转向惩罚 [m]
  
  # ========== 传感器参数 ==========
  sensor_range_min: 0.5           # 最小感知距离 [m]
  sensor_range_max: 5.0           # 最大感知距离 [m]
  sensor_fov_h: 1.57              # 水平FOV [rad] (~90°)
  sensor_fov_v: 1.22              # 垂直FOV [rad] (~70°)
  
  # ========== 视点采样 ==========
  use_spherical_sampling: false   # 是否使用球扇形采样
  viewpoint_height: 1.0           # 视点基准高度 [m]
  viewpoint_height_min: 0.5       # 最小采样高度 [m]
  viewpoint_height_max: 2.0       # 最大采样高度 [m]
  candidate_rmin: 2.0             # 最小采样半径 [m]
  candidate_rmax: 5.0             # 最大采样半径 [m]
  candidate_rnum: 3               # 半径采样数量
  candidate_dphi: 0.5             # 角度采样间隔 [rad]
  optimal_distance: 3.0           # 最优观测距离 [m]
  
  # ========== 前沿聚类 ==========
  cluster_eps: 1.0                # 聚类半径 [m]
  min_cluster_size: 10            # 最小簇大小
  cluster_split_threshold: 3.0    # 簇分裂阈值 [m]
  max_clusters: 20                # 最大簇数量
  
  # ========== 安全参数 ==========
  uav_radius: 0.35                # 无人机半径 [m]
  safety_margin: 0.2              # 安全边距 [m]
  
  # ========== 滤波参数 ==========
  min_viewpoint_distance: 0.8     # 视点最小间距 [m]
  min_coverage_score: 0.3         # 最小覆盖评分
  min_visibility_ratio: 0.3       # 最小可见性比例
  
  # ========== EDEN历史节点 ==========
  history_node_distance: 3.0      # 历史节点间隔 [m]
  node_frontier_radius: 5.0       # 节点关联前沿半径 [m]
  
  # ========== 前沿点过滤 ==========
  ground_offset: 0.1              # 地面过滤偏移 [m]
  overhead_height_threshold: 0.5  # 头顶高度阈值 [m]
  overhead_distance_threshold: 2.0 # 头顶水平距离阈值 [m]
```

### 规划参数 (`exploration_open3d.yaml`)

```yaml
# ========== EGO-Planner FSM ==========
fsm:
  flight_type: 1                # 1=手动目标, 2=预设路径
  takeoff_height: 1.0           # 起飞高度 [m]
  takeoff_accel_time: 1.0       # 起飞加速时间 [s]
  replan_duration: 0.05         # 重规划偏移时间 [s]
  auto_start_preset: true       # 自动开始预设路径
  sim_mode: 1                   # 0=Gazebo, 1=Open3D, 2=Real
  thresh_replan_time: 1.0       # 重规划时间阈值 [s]
  thresh_no_replan_meter: 1.0   # 不重规划距离阈值 [m]
  planning_horizon: 7.5         # 规划视野 [m]
  emergency_time: 1.0           # 紧急停止时间 [s]
  fail_safe: true               # 是否启用故障安全

# ========== 规划管理器 ==========
manager:
  max_vel: 1.5                  # 最大速度 [m/s]
  max_acc: 3.0                  # 最大加速度 [m/s²]
  max_jerk: 4.0                 # 最大加加速度 [m/s³]
  
# ========== MINCO优化 ==========
optimization:
  weight_obstacle: 10000.0      # 障碍物代价权重
  weight_smoothness: 1.0        # 平滑度代价权重
  weight_feasibility: 1.0       # 可行性代价权重
  weight_time: 1.0              # 时间代价权重
  obstacle_clearance: 0.2       # 安全距离 [m]
  max_iteration: 30             # 最大迭代次数
  enable_height_keeping: true   # 启用高度保持
```

---

## 🎮 使用方法

### 1. 启动系统

```bash
# 终端1: 启动仿真环境
roslaunch simulation_standalone uav_simulation.launch

# 终端2: 启动探索规划系统
roslaunch rog_map exploration_planner_open3d.launch

# 终端3 (可选): 启动RViz
rviz -d $(rospack find rog_map)/config/exploration_open3d.rviz
```

### 2. 触发探索

- **手动触发**: 在RViz中使用 "2D Nav Goal" 工具点击任意位置
- **自动触发**: 设置 `auto_start: true`

### 3. 监控系统

查看性能日志：
```bash
tail -f ~/experiment/STAR-Searcher/src/ROG-Map/rog_map/log/exploration_perf_*.log
```

---

## 📚 参考文献

1. **EDEN**: Efficient Dual-Layer Exploration Planning (IEEE TIE 2025)
   - 历史节点采集
   - EOHDT全局路由

2. **Falcon-Planner**: Fast and Continuous-Time Trajectory Planning (RAL 2024)
   - 周期性重规划策略
   - 前沿覆盖检测

3. **STAR-MVF**: Semantic Terrain-Aware Representation (ICRA 2024)
   - 双层地图系统
   - 子网格管理

4. **EGO-Planner**: Gradient-based Local Planner (ICRA 2021)
   - MINCO轨迹优化
   - 实时避障规划

---

**文档更新日期**: 2026-01-09  
**维护者**: STAR-Searcher Team

