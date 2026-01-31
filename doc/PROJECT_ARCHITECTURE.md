# STAR-Searcher 新架构系统文档

> **最后更新**: 2026-01-10  
> **版本**: v3.0 - 全新架构：单状态机 + 解耦的时序模块

---

## 📋 目录

1. [系统概述](#系统概述)
2. [架构设计理念](#架构设计理念)
3. [时序架构](#时序架构)
4. [核心模块详解](#核心模块详解)
5. [状态机设计](#状态机设计)
6. [视点生成机制](#视点生成机制)
7. [历史节点图](#历史节点图)
8. [ROS接口](#ros接口)
9. [参数配置](#参数配置)

---

## 🎯 系统概述

STAR-Searcher 采用**单状态机 + 多定时器并行**的全新架构设计：

- **探索状态机** (ExplorationFSMNew): 4状态简化设计，负责探索逻辑和规划调度
- **地图管理** (MapManager): 在 LiDAR 回调中直接更新地图和前沿
- **视点管理** (ViewpointManager): 独立定时器 (5Hz) 负责视点生成和聚类
- **历史节点图** (HistoryGraph): 独立定时器 (2Hz) 维护全局节点连通图

### 核心特性

| 特性 | 说明 |
|------|------|
| **地图系统** | STAR-MVF 双层地图（LocalMap滑动窗口 + GlobalMap子网格） |
| **探索策略** | EDEN历史节点 + 前沿聚类 + 圆周采样 |
| **时序解耦** | 地图、视点、历史节点、状态机各自独立运行 |
| **轨迹优化** | EGO-Planner MINCO优化 + 实时避障 |
| **仿真支持** | Open3D/Gazebo/Real三种模式 |

### 系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                  STAR-Searcher 新架构系统                        │
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
│  │    🗺️ 地图管理模块 (MapManager) - 事件驱动             │   │
│  │                                                         │   │
│  │  lidarCallback(~10Hz)                                  │   │
│  │    │                                                   │   │
│  │    ├─► LocalMap::insertPointCloud()                    │   │
│  │    │   └─► 阶段1: 收集占据体素                         │   │
│  │    │   └─► 阶段2: 射线更新（遇占据终止）               │   │
│  │    │   └─► 阶段3: 更新占据体素                         │   │
│  │    │                                                   │   │
│  │    ├─► syncLocalToGlobal()                             │   │
│  │    │   └─► GlobalMap::updateOccupiedVoxels()           │   │
│  │    │                                                   │   │
│  │    └─► GlobalMap::extractGlobalFrontierIncremental()   │   │
│  │        └─► 增量前沿提取                                │   │
│  │                                                         │   │
│  │  vizTimerCallback(5Hz)                                 │   │
│  │    └─► 发布地图可视化                                  │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │    🌐 历史节点图 (HistoryGraph) - 2Hz                   │   │
│  │                                                         │   │
│  │  updateTimer(2Hz)                                      │   │
│  │    │                                                   │   │
│  │    ├─► updateHistoryNodes()                            │   │
│  │    │   └─► 增量创建历史节点（间隔3m）                  │   │
│  │    │   └─► 连接到最近可达节点                          │   │
│  │    │                                                   │   │
│  │    └─► updateHistoryNodeConnections()                  │   │
│  │        └─► 维护节点间双向边（BFS图构建）               │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │    🎯 视点管理器 (ViewpointManager) - 5Hz              │   │
│  │                                                         │   │
│  │  updateTimer(5Hz)                                      │   │
│  │    │                                                   │   │
│  │    ├─► getFrontierPositions()                          │   │
│  │    │   └─► 从GlobalMap获取前沿点                       │   │
│  │    │                                                   │   │
│  │    ├─► clusterFrontierPoints()                         │   │
│  │    │   └─► BFS区域增长聚类                             │   │
│  │    │                                                   │   │
│  │    ├─► sampleViewpoints()                              │   │
│  │    │   └─► 圆周采样（多半径+多角度）                   │   │
│  │    │                                                   │   │
│  │    └─► filterViewpoints()                              │   │
│  │        └─► 安全检查+去重                               │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │    🔍 探索状态机 (ExplorationFSMNew) - 100Hz           │   │
│  │                                                         │   │
│  │  fsmCallback(100Hz)                                    │   │
│  │    │                                                   │   │
│  │    ├─► INIT: 初始化等待                                │   │
│  │    │                                                   │   │
│  │    ├─► WAIT_TRIGGER: 等待触发                          │   │
│  │    │                                                   │   │
│  │    ├─► LOCALPLAN: 选择视点+生成路径+调用规划器         │   │
│  │    │   └─► selectBestViewpoint()                       │   │
│  │    │   └─► findPath via HistoryGraph                  │   │
│  │    │   └─► callPlanner(target_pos, target_yaw)        │   │
│  │    │                                                   │   │
│  │    └─► EXEC_EXPLORATION: 监控执行+碰撞检查+覆盖检查    │   │
│  │        └─► checkCollision()                            │   │
│  │        └─► checkCoverage()                             │   │
│  │        └─► isNearTarget()                              │   │
│  │                                                         │   │
│  │  collisionCheckCallback(20Hz)                          │   │
│  │    └─► 检测轨迹前方碰撞                                │   │
│  │                                                         │   │
│  │  visualizationCallback(10Hz)                           │   │
│  │    └─► 发布视点、路径可视化                            │   │
│  └─────────────────────────────────────────────────────────┘   │
│                             │                                   │
│                             │ /planning/pos_cmd                │
│                             │ (PositionCommand)                │
│                             ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │            飞控/控制器 (PX4/SO3)                         │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 💡 架构设计理念

### 旧架构的问题

1. **状态机耦合过重**: 探索状态机和规划状态机双FSM设计，通信复杂
2. **高频率的计算密集任务**: 在100Hz的状态机中执行视点生成和排序，严重影响性能
3. **地图同步延迟**: 使用单独的10Hz定时器同步地图，导致前沿信息延迟

### 新架构的优势

1. **时序解耦**: 地图更新、视点生成、历史节点维护、状态机各自独立运行
2. **单状态机设计**: 探索状态机直接调用规划器，简化通信路径
3. **实时地图更新**: 在LiDAR回调中直接更新地图和前沿，无延迟
4. **性能优化**: 高频状态机仅负责轻量级决策，重计算任务在低频定时器中完成

### 设计原则

| 原则 | 说明 |
|------|------|
| **单一职责** | 每个模块职责明确，MapManager负责地图，ViewpointManager负责视点 |
| **频率分层** | 高频任务轻量化，重计算任务低频化 |
| **数据驱动** | 状态机基于ViewpointManager和HistoryGraph提供的数据做决策 |
| **增量更新** | 地图、前沿、历史节点均采用增量更新策略 |

---

## ⏰ 时序架构

系统采用**多定时器并行**架构，不同模块独立运行，通过ROS话题和直接接口通信。

### 频率分层设计

| 模块 | 频率 | 计算复杂度 | 功能 |
|------|------|-----------|------|
| **LiDAR回调** | ~10Hz | 中 | 点云处理、地图更新、前沿提取 |
| **HistoryGraph** | 2Hz | 低 | 历史节点增量创建、连接维护 |
| **ViewpointManager** | 5Hz | 高 | 前沿聚类、视点采样、安全检查 |
| **ExplorationFSM** | 100Hz | 低 | 状态转换、决策逻辑 |
| **碰撞检查** | 20Hz | 低 | 轨迹前方碰撞检测 |
| **可视化** | 5-10Hz | 低 | 发布RViz可视化 |

### 数据流图

```
┌─────────────────────────────────────────────────────────┐
│                   数据流 (Data Flow)                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  LiDAR点云 (10Hz)                                       │
│       │                                                 │
│       ▼                                                 │
│  MapManager::lidarCallback()                            │
│       │                                                 │
│       ├─► LocalMap::insertPointCloud()                  │
│       ├─► syncLocalToGlobal()                           │
│       └─► GlobalMap::extractGlobalFrontierIncremental() │
│            │                                            │
│            ▼                                            │
│       GlobalMap (前沿点、占据体素)                       │
│            │                                            │
│            ├────────────────────────┐                   │
│            │                        │                   │
│            ▼                        ▼                   │
│   ViewpointManager (5Hz)   HistoryGraph (2Hz)          │
│       │                            │                   │
│       ├─► 聚类前沿点               ├─► 创建历史节点     │
│       ├─► 采样视点                 └─► 构建连通图       │
│       └─► 安全检查                                      │
│            │                        │                   │
│            └───────┬────────────────┘                   │
│                    │                                    │
│                    ▼                                    │
│          ExplorationFSMNew (100Hz)                      │
│                    │                                    │
│                    ├─► WAIT_TRIGGER                     │
│                    ├─► LOCALPLAN                        │
│                    │   └─► selectBestViewpoint()        │
│                    │   └─► findPath()                   │
│                    │   └─► callPlanner()                │
│                    │                                    │
│                    └─► EXEC_EXPLORATION                 │
│                        └─► checkCollision()             │
│                        └─► checkCoverage()              │
│                        └─► publishPositionCommand()     │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## 🧩 核心模块详解

### 1. MapManager (地图管理器)

**文件**: `src/star_mvf/map/map_manager.cpp`

**主要职责**:
- 接收和处理LiDAR点云
- 维护LocalMap（滑动窗口）和GlobalMap（全局子网格）
- 在LiDAR回调中直接完成地图同步和前沿提取

**关键修改**:
- ❌ **删除**: `syncTimerCallback()` - 不再使用单独的同步定时器
- ✅ **新增**: 在 `lidarCallback()` 中直接调用 `syncLocalToGlobal()` 和 `extractGlobalFrontierIncremental()`

**核心流程** (`lidarCallback`):

```cpp
void MapManager::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1. 点云处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // 2. 降采样
    downsamplePointCloud(cloud, config_.lidar_downsample_filter);
    
    // 3. 更新LocalMap
    local_map_->insertPointCloud(cloud, current_center_);
    
    // 4. 同步到GlobalMap（实时）
    syncLocalToGlobal();
    
    // 5. 增量前沿提取（实时）
    global_map_->extractGlobalFrontierIncremental();
}

void MapManager::syncLocalToGlobal() {
    std::vector<VoxelUpdate> updates = local_map_->getCachedUpdates();
    for (const auto& update : updates) {
        global_map_->updateOccupiedVoxel(update.position, update.log_odds);
    }
    global_map_->updateInflation();
    global_map_->updateESDF();
}
```

**性能优化**:
- **降采样**: `lidar_downsample_filter` 参数控制点云降采样（例如2表示每2个点取1个）
- **射线终止**: 射线遇到占据体素立即终止，避免覆盖近处障碍物
- **缓存机制**: LocalMap缓存更新，批量同步到GlobalMap
- **增量前沿提取**: 只更新变化的子网格，避免全局扫描

---

### 2. ViewpointManager (视点管理器)

**文件**: `src/rog_map/viewpoint_manager.cpp`

**主要职责**:
- 定时生成探索视点（5Hz）
- 前沿点聚类
- 视点采样和安全检查
- 维护探索视点列表

**核心流程** (`updateTimer` 回调):

```cpp
void ViewpointManager::update() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Step 1: 从GlobalMap获取前沿点
    std::vector<Eigen::Vector3f> frontier_points;
    getFrontierPositions(frontier_points);
    
    if (frontier_points.empty()) {
        exploration_viewpoints_.clear();
        return;
    }
    
    // Step 2: 过滤前沿点
    filterFrontierPoints(frontier_points);
    
    // Step 3: 聚类前沿点
    std::vector<FrontierCluster> clusters;
    clusterFrontierPoints(frontier_points, clusters);
    
    // Step 4: 分裂大簇
    splitLargeClusters(clusters);
    
    // Step 5: 为每个簇采样视点
    exploration_viewpoints_.clear();
    for (auto& cluster : clusters) {
        std::vector<ExplorationViewpoint> vps;
        sampleViewpoints(cluster, vps);
        
        // Step 6: 安全检查和评分
        filterViewpoints(vps);
        
        // 保留最佳视点
        if (!vps.empty()) {
            vps[0].id = next_viewpoint_id_++;
            vps[0].cluster_id = cluster.cluster_id;
            exploration_viewpoints_.push_back(vps[0]);
        }
    }
    
    // 保存簇信息用于可视化
    frontier_clusters_ = clusters;
}
```

**视点采样** (圆周采样):

```cpp
void ViewpointManager::sampleViewpoints(
    const FrontierCluster& cluster,
    std::vector<ExplorationViewpoint>& viewpoints)
{
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
                config_.viewpoint_height
            );
            
            // 快速安全检查
            if (!isPositionSafe(pos)) continue;
            
            // 计算yaw（朝向簇中心）
            float yaw = std::atan2(center.y() - pos.y(), 
                                   center.x() - pos.x());
            
            // 计算覆盖和可见性评分
            float coverage_score = computeCoverageScore(pos, yaw, cluster);
            float visibility_score = computeVisibilityScore(pos, cluster);
            float score = 0.6f * coverage_score + 0.4f * visibility_score;
            
            ExplorationViewpoint vp;
            vp.position = pos;
            vp.yaw = yaw;
            vp.score = score;
            viewpoints.push_back(vp);
        }
    }
    
    // 按评分降序排序
    std::sort(viewpoints.begin(), viewpoints.end(),
        [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
            return a.score > b.score;
        });
}
```

**安全检查**:

```cpp
bool ViewpointManager::isPositionSafe(const Eigen::Vector3f& pos) {
    // 1. 地图范围检查
    if (!global_map_->isInGlobalMapRange(pos)) return false;
    
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
    int num_checks = 8;
    
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

---

### 3. HistoryGraph (历史节点图)

**文件**: `src/rog_map/history_graph.cpp`

**主要职责**:
- 定时维护历史节点（2Hz）
- 增量构建全局连通图
- 提供路径查询服务

**核心数据结构**:

```cpp
struct HistoryNode {
    int node_id;
    Eigen::Vector3f position;
    float timestamp;
    std::vector<int> neighbors;  // 邻居节点ID列表
    int parent_node_id;          // 创建时的父节点ID
    
    void addNeighbor(int id);    // 双向添加邻居
    bool hasNeighbor(int id) const;
};
```

**增量连通性构建** (`updateHistoryNodes`):

```cpp
void HistoryGraph::updateHistoryNodes() {
    // 检查距离上一个节点的距离
    if (!first_node_created_) {
        // 创建第一个节点
        HistoryNode new_node(next_node_id_++, current_uav_pos_, 
                             ros::Time::now().toSec());
        history_nodes_.push_back(new_node);
        last_node_position_ = current_uav_pos_;
        first_node_created_ = true;
        return;
    }
    
    float dist = (current_uav_pos_ - last_node_position_).norm();
    
    if (dist > config_.history_node_distance) {
        // 找到最近的可达历史节点
        int nearest_reachable = findNearestReachableHistoryNode(
            current_uav_pos_, config_.max_connection_distance);
        
        // 创建新节点
        HistoryNode new_node(next_node_id_++, current_uav_pos_, 
                             ros::Time::now().toSec());
        new_node.parent_node_id = nearest_reachable;
        
        // 连接到最近可达节点（双向边）
        if (nearest_reachable >= 0) {
            new_node.addNeighbor(nearest_reachable);
            history_nodes_[nearest_reachable].addNeighbor(new_node.node_id);
        }
        
        history_nodes_.push_back(new_node);
        last_node_position_ = current_uav_pos_;
    }
}
```

**连接更新** (`updateHistoryNodeConnections`):

```cpp
void HistoryGraph::updateHistoryNodeConnections() {
    // 为每个节点检查是否可以添加新的连接
    for (size_t i = 0; i < history_nodes_.size(); ++i) {
        for (size_t j = i + 1; j < history_nodes_.size(); ++j) {
            // 已有连接，跳过
            if (history_nodes_[i].hasNeighbor(history_nodes_[j].node_id))
                continue;
            
            float dist = (history_nodes_[i].position - 
                          history_nodes_[j].position).norm();
            
            // 距离超过阈值，跳过
            if (dist > config_.max_connection_distance)
                continue;
            
            // 检查是否有障碍物（注意：已有连接的边不重新检测）
            if (!hasObstacleBetween(history_nodes_[i].position, 
                                    history_nodes_[j].position)) {
                // 双向添加邻居
                history_nodes_[i].addNeighbor(history_nodes_[j].node_id);
                history_nodes_[j].addNeighbor(history_nodes_[i].node_id);
            }
        }
    }
}
```

**路径查询** (`findPath`):

```cpp
bool HistoryGraph::findPath(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end,
    std::vector<Eigen::Vector3f>& waypoints)
{
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    waypoints.clear();
    
    // 1. 找到起点和终点最近的历史节点
    int start_idx = findNearestReachableHistoryNode(start, 
                        config_.max_connection_distance);
    int end_idx = findNearestReachableHistoryNode(end, 
                        config_.max_connection_distance);
    
    if (start_idx < 0 || end_idx < 0) return false;
    
    // 2. BFS搜索（使用预计算的neighbors）
    std::queue<int> q;
    std::vector<bool> visited(history_nodes_.size(), false);
    std::vector<int> parent(history_nodes_.size(), -1);
    
    q.push(start_idx);
    visited[start_idx] = true;
    
    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        
        if (curr == end_idx) break;
        
        // 遍历预计算的邻居（不重新检测障碍物）
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

**障碍物检测** (`hasObstacleBetween`):

```cpp
bool HistoryGraph::hasObstacleBetween(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end)
{
    Eigen::Vector3f dir = end - start;
    float dist = dir.norm();
    if (dist < 1e-6f) return false;
    
    dir.normalize();
    int num_steps = static_cast<int>(dist / config_.raycast_step);
    
    // 需要连续两个占据体素才判定为障碍物（鲁棒性优化）
    int consecutive_occupied = 0;
    
    for (int i = 0; i < num_steps; ++i) {
        Eigen::Vector3f pos = start + dir * (i * config_.raycast_step);
        
        if (global_map_->isOccupied(pos)) {
            consecutive_occupied++;
            if (consecutive_occupied >= config_.consecutive_occupied_threshold) {
                return true;
            }
        } else {
            consecutive_occupied = 0;
        }
    }
    
    return false;
}
```

---

## 🔄 状态机设计

### ExplorationFSMNew (4状态探索状态机)

**文件**: `src/rog_map/exploration_fsm_new.cpp`

**设计理念**:
- ❌ **删除**: 原有的6状态设计（UPDATE_FRONTIER、PLAN_VIEWPOINTS等）
- ❌ **删除**: 独立的规划状态机（EGOReplanFSM）
- ✅ **新增**: 4状态简化设计 + 直接调用规划器

**状态转换图**:

```
                    ExplorationFSMNew (4状态设计)
                    
    INIT (初始化)
      │  等待里程计 (has_odom_)
      │  等待时间 > init_wait_time (2s)
      ▼
    WAIT_TRIGGER (等待触发)
      │  用户触发 (RViz 2D Goal)
      │  或 auto_start=true
      ▼
    LOCALPLAN (局部规划)
      │  1. selectBestViewpoint()
      │  2. findPath() via HistoryGraph
      │  3. callPlanner(target_pos, target_yaw)
      │
      ├──► 规划失败 ────────► WAIT_TRIGGER
      │
      ▼ 规划成功
    EXEC_EXPLORATION (执行探索) ◄───────┐
      │  1. publishPositionCommand()      │
      │  2. checkCollision()              │
      │  3. checkCoverage()               │
      │  4. isNearTarget()                │
      │                                   │
      ├──► 碰撞 ────────────► LOCALPLAN   │
      ├──► 覆盖 > 30% ──────► LOCALPLAN   │
      ├──► 到达当前航点 ─────────────────┤ (规划下一段)
      │                                   │
      └──► 到达最终视点 ────► WAIT_TRIGGER
```

### 状态详细说明

#### 1. INIT (初始化)

**功能**: 系统初始化，等待里程计和地图准备

**处理逻辑**:

```cpp
void ExplorationFSMNew::processInit() {
    if (!has_odom_) {
        return;  // 等待里程计
    }
    
    double elapsed = (ros::Time::now() - fsm_init_time_).toSec();
    if (elapsed > config_.init_wait_time) {
        transitState(ExplorationState::WAIT_TRIGGER, "Init complete");
    }
}
```

#### 2. WAIT_TRIGGER (等待触发)

**功能**: 等待用户触发或自动开始

**触发方式**:
1. **RViz手动触发**: 使用"2D Nav Goal"工具点击任意位置
2. **自动触发**: 设置`auto_start=true`

**处理逻辑**:

```cpp
void ExplorationFSMNew::processWaitTrigger() {
    if (config_.auto_start || triggered_) {
        transitState(ExplorationState::LOCALPLAN, "Triggered");
    }
}

void ExplorationFSMNew::triggerCallback(
    const geometry_msgs::PoseStampedConstPtr& msg)
{
    triggered_ = true;
    PERF_INFO("FSM", "Exploration triggered by user");
}
```

#### 3. LOCALPLAN (局部规划)

**功能**: 选择最佳视点、生成路径、调用规划器

**核心流程**:

```cpp
void ExplorationFSMNew::processLocalPlan() {
    // Step 1: 选择最佳视点
    int best_viewpoint_id = -1;
    if (!selectBestViewpoint(best_viewpoint_id)) {
        PERF_WARN("FSM", "No valid viewpoint, waiting...");
        transitState(ExplorationState::WAIT_TRIGGER, "No viewpoint");
        return;
    }
    
    // Step 2: 规划到视点的路径
    if (!planToViewpoint(best_viewpoint_id)) {
        PERF_ERROR("FSM", "Failed to plan to viewpoint");
        // 标记该视点为已访问（避免重复尝试）
        viewpoint_manager_->markViewpointAsVisited(best_viewpoint_id);
        return;  // 下次循环重新选择
    }
    
    // Step 3: 规划成功，进入执行状态
    current_viewpoint_id_ = best_viewpoint_id;
    trajectory_start_time_ = ros::Time::now();
    transitState(ExplorationState::EXEC_EXPLORATION, "Plan success");
}
```

**视点选择** (`selectBestViewpoint`):

```cpp
bool ExplorationFSMNew::selectBestViewpoint(int& best_viewpoint_id) {
    const auto& viewpoints = viewpoint_manager_->getExplorationViewpoints();
    
    if (viewpoints.empty()) {
        return false;
    }
    
    // 贪心策略：选择距离最近且分数最高的视点
    float min_cost = std::numeric_limits<float>::max();
    best_viewpoint_id = -1;
    
    for (const auto& vp : viewpoints) {
        if (vp.is_visited) continue;
        
        // 计算代价：距离 + (1 - 分数) * 权重
        float dist = (vp.position - odom_pos_.cast<float>()).norm();
        float cost = dist + (1.0f - vp.score) * 5.0f;
        
        // 避免近距离大转向
        float yaw_diff = std::abs(vp.yaw - odom_yaw_);
        if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;
        
        if (dist < config_.close_viewpoint_threshold && 
            yaw_diff > 1.2f) {
            cost += 10.0f;  // 惩罚近距离大转向
        }
        
        if (cost < min_cost) {
            min_cost = cost;
            best_viewpoint_id = vp.id;
        }
    }
    
    return best_viewpoint_id >= 0;
}
```

**路径规划** (`planToViewpoint`):

```cpp
bool ExplorationFSMNew::planToViewpoint(int viewpoint_id) {
    // 找到视点
    const auto& viewpoints = viewpoint_manager_->getExplorationViewpoints();
    auto it = std::find_if(viewpoints.begin(), viewpoints.end(),
        [viewpoint_id](const ExplorationViewpoint& vp) {
            return vp.id == viewpoint_id;
        });
    
    if (it == viewpoints.end()) return false;
    
    ExplorationViewpoint vp = *it;
    Eigen::Vector3f current_pos = odom_pos_.cast<float>();
    float dist = (vp.position - current_pos).norm();
    
    // 如果距离较近，直接规划
    if (dist < config_.close_viewpoint_threshold) {
        Eigen::Vector3d target_pos = vp.position.cast<double>();
        double target_yaw = vp.yaw;
        Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
        
        return callPlanner(target_pos, target_yaw, target_vel);
    }
    
    // 距离较远，通过历史节点图查找路径
    std::vector<Eigen::Vector3f> waypoints;
    if (history_graph_->findPath(current_pos, vp.position, waypoints)) {
        // 构建航点队列
        waypoint_queue_ = std::queue<Waypoint>();  // 清空
        
        for (size_t i = 0; i < waypoints.size(); ++i) {
            Eigen::Vector3d pos = waypoints[i].cast<double>();
            double yaw = (i + 1 < waypoints.size()) ?
                std::atan2(waypoints[i+1].y() - waypoints[i].y(),
                           waypoints[i+1].x() - waypoints[i].x()) : vp.yaw;
            
            waypoint_queue_.emplace(pos, yaw, Eigen::Vector3d::Zero(), -1);
        }
        
        // 添加最终视点
        waypoint_queue_.emplace(vp.position.cast<double>(), 
                                vp.yaw, Eigen::Vector3d::Zero(), vp.id);
        
        // 规划到第一个航点
        Waypoint first_wp = waypoint_queue_.front();
        waypoint_queue_.pop();
        
        return callPlanner(first_wp.position, first_wp.yaw, first_wp.velocity);
    }
    
    // 无法找到路径，尝试直接规划
    Eigen::Vector3d target_pos = vp.position.cast<double>();
    return callPlanner(target_pos, vp.yaw, Eigen::Vector3d::Zero());
}
```

**调用规划器** (`callPlanner`):

```cpp
bool ExplorationFSMNew::callPlanner(
    const Eigen::Vector3d& target_pos,
    double target_yaw,
    const Eigen::Vector3d& target_vel)
{
    // 设置目标
    planner_manager_->setTarget(target_pos, target_vel, target_yaw);
    
    // 调用规划器
    bool success = false;
    
    // 如果已有轨迹，从轨迹上重新规划
    if (planner_manager_->haveTraj()) {
        success = planner_manager_->reboundReplan(
            odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
            target_pos, target_vel, true, false, false);
    } else {
        // 从当前位置规划新轨迹
        success = planner_manager_->planGlobalTraj(odom_pos_);
    }
    
    if (!success) {
        PERF_ERROR_FMT("FSM", "Planner failed for target (%.2f,%.2f,%.2f)",
                      target_pos.x(), target_pos.y(), target_pos.z());
        return false;
    }
    
    PERF_INFO_FMT("FSM", "Planner success for target (%.2f,%.2f,%.2f)",
                 target_pos.x(), target_pos.y(), target_pos.z());
    return true;
}
```

#### 4. EXEC_EXPLORATION (执行探索)

**功能**: 监控轨迹执行、检查碰撞、覆盖、到达

**处理逻辑**:

```cpp
void ExplorationFSMNew::processExecExploration() {
    // 1. 检查碰撞
    if (checkCollision()) {
        PERF_WARN("FSM", "Collision detected, replanning...");
        transitState(ExplorationState::LOCALPLAN, "Collision");
        return;
    }
    
    // 2. 检查覆盖（前沿减少>30%触发重规划）
    double elapsed = (ros::Time::now() - trajectory_start_time_).toSec();
    if (elapsed > config_.coverage_check_interval) {
        if (checkCoverage()) {
            PERF_INFO("FSM", "Coverage threshold reached, replanning...");
            transitState(ExplorationState::LOCALPLAN, "Coverage");
            return;
        }
    }
    
    // 3. 检查是否接近当前目标航点
    Eigen::Vector3d current_target;
    if (!waypoint_queue_.empty()) {
        current_target = waypoint_queue_.front().position;
    } else {
        // 当前目标是最终视点
        const auto& viewpoints = viewpoint_manager_->getExplorationViewpoints();
        auto it = std::find_if(viewpoints.begin(), viewpoints.end(),
            [this](const ExplorationViewpoint& vp) {
                return vp.id == current_viewpoint_id_;
            });
        
        if (it == viewpoints.end()) {
            // 视点已被删除，回到等待状态
            transitState(ExplorationState::WAIT_TRIGGER, "Viewpoint gone");
            return;
        }
        
        current_target = it->position.cast<double>();
    }
    
    // 4. 检查到达
    if (isNearTarget(current_target, 0.0)) {
        if (!waypoint_queue_.empty()) {
            // 还有中间航点，继续规划
            Waypoint next_wp = waypoint_queue_.front();
            waypoint_queue_.pop();
            
            if (callPlanner(next_wp.position, next_wp.yaw, next_wp.velocity)) {
                PERF_INFO("FSM", "Replanning to next waypoint");
                return;
            }
        } else {
            // 到达最终视点
            viewpoint_manager_->markViewpointAsVisited(current_viewpoint_id_);
            PERF_INFO_FMT("FSM", "Reached viewpoint %d", current_viewpoint_id_);
            
            // 清空航点队列
            waypoint_queue_ = std::queue<Waypoint>();
            
            // 返回等待状态（会自动选择下一个视点）
            transitState(ExplorationState::WAIT_TRIGGER, "Viewpoint reached");
            return;
        }
    }
    
    // 5. 发布位置命令（从轨迹采样）
    if (planner_manager_->haveTraj()) {
        double t_cur = ros::Time::now().toSec() - 
                       planner_manager_->getTrajStartTime();
        
        Eigen::Vector3d pos = planner_manager_->getPos(t_cur);
        Eigen::Vector3d vel = planner_manager_->getVel(t_cur);
        Eigen::Vector3d acc = planner_manager_->getAcc(t_cur);
        double yaw = planner_manager_->getYaw(t_cur);
        double yaw_dot = planner_manager_->getYawdot(t_cur);
        
        publishPositionCommand(pos, vel, acc, yaw, yaw_dot);
    }
}
```

**碰撞检测** (`checkCollision`):

```cpp
bool ExplorationFSMNew::checkCollision() {
    if (!planner_manager_->haveTraj()) return false;
    
    double t_cur = ros::Time::now().toSec() - 
                   planner_manager_->getTrajStartTime();
    double t_check = t_cur + config_.collision_lookahead;
    
    Eigen::Vector3d check_pos = planner_manager_->getPos(t_check);
    
    return map_manager_->getGlobalMap()->isOccupiedInflate(
        check_pos.cast<float>());
}
```

**覆盖检测** (`checkCoverage`):

```cpp
bool ExplorationFSMNew::checkCoverage() {
    int current_unknown = map_manager_->getGlobalMap()->getUnknownVoxelCount();
    
    if (initial_unknown_count_ == 0) {
        initial_unknown_count_ = current_unknown;
        return false;
    }
    
    float covered_ratio = 1.0f - 
        static_cast<float>(current_unknown) / initial_unknown_count_;
    
    return covered_ratio > config_.coverage_trigger_ratio;
}
```

---

## 🎯 视点生成机制

**完整流程** (在ViewpointManager中):

```
update() 定时器回调 (5Hz)
    │
    ├─► Step 1: getFrontierPositions()
    │   └─► 从GlobalMap按子网格分组获取前沿点
    │
    ├─► Step 2: filterFrontierPoints()
    │   ├─► 地面噪声过滤 (z < ground_height + 0.1)
    │   ├─► 头顶区域过滤
    │   └─► 天花板过滤 (z > ceil_height)
    │
    ├─► Step 3: clusterFrontierPoints()
    │   └─► BFS区域增长聚类
    │
    ├─► Step 4: splitLargeClusters()
    │   └─► 超过阈值（4m）的大簇二分
    │
    ├─► Step 5: sampleViewpoints()
    │   ├─► 圆周采样（多半径+多角度）
    │   ├─► 快速安全检查
    │   └─► 评分排序
    │
    └─► Step 6: filterViewpoints()
        └─► 距离去重（min_distance=1.5m）
```

### 前沿点聚类 (BFS区域增长)

```cpp
void ViewpointManager::clusterFrontierPoints(
    const std::vector<Eigen::Vector3f>& points,
    std::vector<FrontierCluster>& clusters)
{
    clusters.clear();
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
                if (dist <= config_.cluster_eps) {
                    visited[j] = true;
                    q.push(j);
                }
            }
        }
        
        // 过滤小簇
        if (cluster_points.size() >= 
            static_cast<size_t>(config_.min_cluster_size)) {
            FrontierCluster cluster;
            cluster.points = cluster_points;
            cluster.computeCentroidAndExtent();
            cluster.cluster_id = clusters.size();
            clusters.push_back(cluster);
        }
    }
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
| `/exploration_planner_node/global_map/frontier` | `sensor_msgs/PointCloud2` | 5Hz | 前沿点云 |

### 输出话题 - 探索模块

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 100Hz | 位置控制命令 |
| `/exploration/frontier_clusters` | `visualization_msgs/MarkerArray` | 5Hz | 前沿簇可视化 |
| `/exploration/viewpoints` | `visualization_msgs/MarkerArray` | 5Hz | 视点可视化 |
| `/exploration/state` | `std_msgs/Int8` | 10Hz | 探索状态 |

---

## ⚙️ 参数配置

### 关键参数说明

```yaml
# ========== 地图更新（在lidarCallback中）==========
rog_map:
  resolution: 0.2                    # 体素分辨率 [m]
  sensor:
    lidar_downsample_filter: 2       # 降采样因子（2=每2个点取1个）
  local_map:
    half_size: [10.0, 10.0, 3.0]     # 局部地图半尺寸 [m]
    raycast_step: 0.1                # 射线步长 [m]
  global_map:
    frontier_height_min: 0.7         # 前沿点最低高度 [m]
    frontier_height_max: 2.5         # 前沿点最高高度 [m]

# ========== 历史节点图（2Hz）==========
history_graph:
  update_rate: 2.0                   # Hz
  history_node_distance: 3.0         # 历史节点间隔 [m]
  max_connection_distance: 20.0      # 最大连接距离 [m]
  consecutive_occupied_threshold: 2  # 连续占据阈值

# ========== 视点管理器（5Hz）==========
viewpoint_manager:
  update_rate: 5.0                   # Hz
  candidate_rmin: 2.0                # 最小采样半径 [m]
  candidate_rmax: 5.0                # 最大采样半径 [m]
  candidate_rnum: 6                  # 半径采样数量
  candidate_dphi: 0.25               # 角度采样间隔 [rad]
  cluster_eps: 0.25                  # 聚类半径 [m]
  min_cluster_size: 30               # 最小簇大小
  uav_radius: 0.3                    # 无人机半径 [m]
  safety_margin: 0.5                 # 安全边距 [m]

# ========== 探索状态机（100Hz）==========
fsm:
  fsm_frequency: 100.0               # FSM主循环频率 [Hz]
  reach_threshold: 0.5               # 到达阈值 [m]
  collision_check_interval: 0.05     # 碰撞检查间隔 [s]
  coverage_trigger_ratio: 0.3        # 覆盖触发比例
  close_viewpoint_threshold: 5.0     # 近距离视点阈值 [m]
  auto_start: false                  # 自动开始探索

# ========== 规划器==========
manager:
  max_vel: 1.0                       # 最大速度 [m/s]
  max_acc: 1.0                       # 最大加速度 [m/s²]
  
optimization:
  weight_obstacle: 50000.0           # 障碍物代价权重
  obstacle_clearance: 1.25           # 安全距离 [m]
  enable_height_keeping: true        # 启用高度保持
```

---

## 🎮 使用方法

### 1. 启动系统

```bash
# 终端1: 启动仿真环境
roslaunch simulation_standalone uav_simulation.launch

# 终端2: 启动探索规划系统（新架构）
roslaunch rog_map exploration_new.launch

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

2. **STAR-MVF**: Semantic Terrain-Aware Representation (ICRA 2024)
   - 双层地图系统
   - 子网格管理

3. **EGO-Planner**: Gradient-based Local Planner (ICRA 2021)
   - MINCO轨迹优化
   - 实时避障规划

---

**文档更新日期**: 2026-01-10  
**维护者**: STAR-Searcher Team
