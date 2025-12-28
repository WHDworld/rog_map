# 全局地图更新逻辑修复说明

## 问题描述

用户发现在`hitPointUpdate`和`missPointUpdate`中直接更新全局地图状态时，**地面在全局地图中全部显示为自由状态（FREE）**，但在实时的局部地图中地面正确显示为占据状态（OCCUPIED）。

## 问题根源分析

### 1. Raycasting机制

在概率占据地图（ProbMap）中，使用raycasting更新占据概率：

```cpp
// 伪代码流程
for each point p in point_cloud:
    hitPointUpdate(p)  // 击中点：概率增加
    
    raycast from (sensor + raycast_range_min * direction) to p:
        for each voxel on ray:
            missPointUpdate(voxel)  // 射线路径：概率减少
```

### 2. 地面点的特殊情况

地面点同时受到多种更新：
- **被LiDAR直接击中** → `hitPointUpdate` → `prob += l_hit (≈0.619)`
- **被其他射线穿过** → `missPointUpdate` → `prob -= l_miss (≈0.619)`

如果地面点被多条射线穿过，其概率可能降到中间值，既不是完全OCCUPIED也不是UNKNOWN，而是处于**KNOWN_FREE区域**。

### 3. 概率阈值

```cpp
// 状态判定（config.hpp）
if (prob < l_free(0.499))      → KNOWN_FREE
if (l_free <= prob < l_occ)    → UNKNOWN  
if (prob >= l_occ(0.80))       → OCCUPIED
```

### 4. 原始代码的问题

#### hitPointUpdate (第606-629行)
```cpp
if (to_type == GridType::OCCUPIED) {
    global_map_->updateVoxelStateFromLiDAR(global_idx, true);  // ✅ 正确
}
else if (to_type == GridType::KNOWN_FREE) {
    global_map_->updateVoxelStateFromLiDAR(global_idx, false); // ❌ 错误！
}
```

**问题**：当地面点从UNKNOWN变为KNOWN_FREE（受raycasting影响）时，会错误地更新全局地图为自由状态。

#### missPointUpdate (第673-696行)
```cpp
if (to_type == GridType::OCCUPIED) {
    global_map_->updateVoxelStateFromLiDAR(global_idx, true);
}
else if (to_type == GridType::KNOWN_FREE) {
    global_map_->updateVoxelStateFromLiDAR(global_idx, false); // 可能过于激进
}
```

**问题**：任何状态变为KNOWN_FREE都会更新全局地图，包括KNOWN_FREE到KNOWN_FREE的情况（虽然有`from_type != to_type`检查，但仍可能有问题）。

## 修复方案

### 修复1：hitPointUpdate - 只更新明确的OCCUPIED状态

```cpp
// 🔥 修复：只在明确为OCCUPIED时更新全局地图
// KNOWN_FREE状态可能是暂时的（受raycasting影响），不应更新到全局地图
if (to_type == GridType::OCCUPIED) 
{
    global_map_->updateVoxelStateFromLiDAR(global_idx, true);
    if(global_map_->isPointInFrustum(global_idx))
    {
        global_map_->updateVoxelStateFromCamera(global_idx, true);
    }
}
// 删除了 else if (to_type == GridType::KNOWN_FREE) 分支
```

**原理**：
- Hit操作应该使体素变为OCCUPIED
- 如果hit后体素仍是KNOWN_FREE，说明它被多次miss（可能是其他射线），这是局部概率融合的暂时状态
- 全局地图应该只记录**确定性强的信息**（OCCUPIED），而不是中间状态

### 修复2：missPointUpdate - 限制FREE更新的前置条件

```cpp
// 🔥 修复：miss更新只处理特定状态转换
// 1. OCCUPIED → FREE: 可能是误判或动态障碍物移除，更新到全局地图
// 2. UNKNOWN → FREE: 正常的空间探索，更新到全局地图
// 3. 其他情况：不更新全局地图
if (to_type == GridType::KNOWN_FREE) 
{
    // 只有从UNKNOWN或OCCUPIED转换为FREE时才更新全局地图
    if(from_type == GridType::UNKNOWN || from_type == GridType::OCCUPIED)
    {
        global_map_->updateVoxelStateFromLiDAR(global_idx, false);
    }
}
else if (to_type == GridType::OCCUPIED) 
{
    // miss操作不太可能使体素变为OCCUPIED，但为了完整性保留
    global_map_->updateVoxelStateFromLiDAR(global_idx, true);
}
```

**原理**：
- UNKNOWN → FREE：正常的射线探索，应该更新
- OCCUPIED → FREE：障碍物消失或误判修正，应该更新
- KNOWN_FREE → KNOWN_FREE：已经是自由的继续miss，不需要重复更新全局地图

## 设计理念

### 局部地图 vs 全局地图

| 特性 | 局部地图（ProbMap） | 全局地图（GlobalMap） |
|------|---------------------|----------------------|
| 更新频率 | 高（每帧） | 低（状态变化时） |
| 精度 | 高（概率值连续） | 低（离散状态） |
| 目的 | 实时避障、轨迹规划 | 长期探索、frontier提取 |
| 敏感度 | 敏感（快速响应） | 保守（稳定可靠） |

### 更新策略

**局部地图**：
- 使用概率融合，快速响应传感器数据
- 允许中间状态（KNOWN_FREE, UNKNOWN, OCCUPIED）
- 可以在OCCUPIED和FREE之间振荡

**全局地图**：
- 只记录**高置信度**的状态变化
- OCCUPIED：确认的障碍物
- FREE：明确的空闲空间（从UNKNOWN探索得到）
- 避免因局部概率融合的中间状态而频繁更新

## 预期效果

修复后的行为：

1. **地面点正常处理**：
   - 地面被LiDAR击中 → 局部地图：OCCUPIED
   - 即使被其他射线miss，只要概率仍>= l_occ → 全局地图保持OCCUPIED
   - 只有确实变为FREE（从其他传感器确认）时才更新全局地图

2. **真实障碍物**：
   - 首次检测到 → 局部和全局都标记为OCCUPIED
   - 持续观测 → 保持OCCUPIED

3. **空闲空间**：
   - 射线穿过未知区域 → 局部地图：FREE → 全局地图：FREE（从UNKNOWN转换）
   - 持续穿过 → 全局地图保持FREE

4. **动态障碍物消失**：
   - OCCUPIED → FREE（多次miss）→ 全局地图更新为FREE（从OCCUPIED转换）

## 相关文件

- **修改文件1**：`src/ROG-Map/rog_map/src/rog_map/prob_map.cpp`
  - `hitPointUpdate()` (第566-637行)
  - `missPointUpdate()` (第639-705行)

- **相关配置**：`src/ROG-Map/rog_map/config/ego_planner_config.yaml`
  - `raycasting.p_hit: 0.65`
  - `raycasting.p_occ: 0.80`
  - `raycasting.p_free: 0.499`
  - `raycasting.p_miss: 0.35`

## 测试建议

1. **地面测试**：
   - 在平坦地面上飞行，观察全局地图中地面是否正确标记为OCCUPIED
   
2. **障碍物测试**：
   - 飞过墙壁等障碍物，确认全局地图正确标记为OCCUPIED
   
3. **空闲空间测试**：
   - 飞过开阔空间，确认全局地图正确标记为FREE
   
4. **Frontier测试**：
   - 确认frontier点（已探索与未探索的边界）正确提取

## 注意事项

1. 此修复使全局地图更新**更加保守**，减少误更新
2. 全局地图的FREE状态更新可能**略有延迟**（需要明确的状态转换）
3. 这是**预期行为**，因为全局地图用于长期规划，应优先保证准确性而非实时性
4. 局部地图仍然保持高频率更新，实时避障不受影响

---

# 前沿簇与探索视点生成策略

## 架构设计

### 统一节点架构

为保证数据同步，采用统一的探索规划节点 (`exploration_planner_node`)：

```
exploration_planner_node
├── ROG-Map (唯一实例)
│   ├── ProbMap (概率占据地图)
│   └── GlobalMap (全局地图 + 前沿提取)
├── EGO-Planner FSM (轨迹规划)
└── Exploration FSM (探索策略)
    └── FrontierViewpointGenerator (视点生成)
```

**优势**：
- 规划器与探索器共享同一个地图实例
- 数据同步，无延迟
- 减少内存占用

## 前沿簇生成策略

### 问题：360度LiDAR导致前沿连成一片

使用360度LiDAR时，无人机起飞后前沿会围成一圈。如果直接对全局前沿点聚类，
所有前沿点会因为距离较近而连成一个大簇，无法有效分割不同方向的探索区域。

### 解决方案：基于角度的扇区分区

将360度空间分成8个扇区（每个45°），在每个扇区内独立聚类：

```
                    N (90°)
                     ↑
           NW (135°) | NE (45°)
                  \  |  /
         W (180°) ─ UAV ─ E (0°)
                  /  |  \
           SW (225°) | SE (315°)
                     ↓
                   S (270°)
```

### 数据流

```
GlobalMap.frontier_voxels_  →  getFrontierPositions()  →  前沿点云
                                       ↓
                            partitionBySector()  →  8个扇区的点集
                                       ↓
                    for each sector: clusterFrontierPoints()  →  扇区内簇
                                       ↓
                        sampleViewpointsForClusterSimple()  →  视点列表
```

### 扇区分区算法

```cpp
void partitionBySector(points, sector_points):
    num_sectors = 8  // 每个扇区45°
    sector_angle = 2π / num_sectors
    
    for each point:
        // 计算相对于无人机的方向
        diff = point_2d - uav_pos_2d
        angle = atan2(diff.y, diff.x)  // [-π, π)
        
        // 映射到 [0, 2π) 并确定扇区
        if angle < 0: angle += 2π
        sector_idx = int(angle / sector_angle)
        
        sector_points[sector_idx].push_back(point)
```

### 聚类算法（Region Growing）

在每个扇区内独立进行区域增长聚类：

```cpp
// 参数
cluster_eps = min(config_.cluster_split_threshold, 1.0f)  // 聚类半径 [m]
min_pts = max(5, config_.min_cluster_size / 10)           // 最小簇大小

// BFS 区域增长
for each unvisited point in sector:
    1. 创建新簇，将当前点加入队列
    2. BFS 搜索：
       - 对于队列中每个点，查找 eps 范围内的邻居
       - 将未访问的邻居加入队列和簇
    3. 如果簇大小 >= min_pts，保留该簇
```

### 优势

1. **避免连成一片**：即使前沿点在空间上连通，不同方向的前沿会被分到不同扇区
2. **均匀覆盖**：每个方向都会生成独立的前沿簇和视点
3. **适合360°LiDAR**：自然地将环绕无人机的前沿分成多个区域

### 簇属性

```cpp
struct FrontierCluster {
    int cluster_id;           // 簇ID
    Eigen::Vector3f centroid; // 簇中心（用于采样视点）
    Eigen::Vector3f normal;   // 法向量（默认Z轴向上）
    int size;                 // 簇包含的前沿点数量
};
```

## 探索视点生成策略

### 圆周采样（参考 falcon_planner）

```
                  ← candidate_rmax →
                  ← candidate_rmin →
                       ____
                   ___/    \___
                  /      C      \    ← 簇中心
                 |    .*   *.    |   ← 采样点
                  \___      ___/
                      \____/
```

**采样参数**：
- `viewpoint_height`: 视点固定高度（= 起飞高度）
- `candidate_rmin`: 最小采样半径（距簇中心）
- `candidate_rmax`: 最大采样半径
- `candidate_rnum`: 半径采样数量
- `candidate_dphi`: 角度采样间隔 [rad]

### 采样流程

```cpp
for each cluster:
    1. 在 [rmin, rmax] 范围内选取 rnum 个半径
    2. 对于每个半径 rc:
       - 在 [-π, π) 以 dphi 间隔采样角度
       - 计算候选位置: (cx + rc*cos(φ), cy + rc*sin(φ), viewpoint_height)
    
    3. 对每个候选位置检查:
       - 是否在地图范围内
       - 是否为自由空间（FREE）
       - 周围是否有障碍物（安全检查）
    
    4. 计算视点朝向: yaw = atan2(cy - py, cx - px)  // 朝向簇中心
       pitch = 0.0  // 无人机水平飞行
    
    5. 评分并选择最佳视点
```

### 安全检查

```cpp
bool isPositionSafeSimple(pos):
    1. 检查 pos 是否在地图范围内
    2. 检查 pos 是否为 FREE 状态
    3. 检查 pos 周围 (uav_radius + safety_margin) 范围内是否有障碍物
```

### 视点属性

```cpp
struct ExplorationViewpoint {
    Eigen::Vector3f position;  // 3D位置
    float yaw;                 // 偏航角（朝向簇中心）
    float pitch;               // 俯仰角（固定为0）
    int cluster_id;            // 对应的簇ID
    float total_score;         // 评分
    bool is_valid;             // 是否有效
};
```

## 视点生成策略（基于子网格分区）

### 核心流程

```
GlobalMap                           FrontierViewpointGenerator
    │                                          │
    │  getFrontierPositionsBySubGrid()         │
    │  ────────────────────────────────────►   │
    │  返回: [(sg_id, [点位置列表]), ...]       │
    │                                          │
    │                          ┌───────────────┼───────────────┐
    │                          │  Step 1       │  Step 2       │
    │                          │ 子网格内聚类   │  分裂大簇     │
    │                          │ clusterIn     │  splitLarge   │
    │                          │ Subgrid()     │  Clusters()   │
    │                          └───────────────┴───────────────┘
    │                                          │
    │                          ┌───────────────┼───────────────┐
    │                          │  Step 3       │  Step 4       │
    │                          │ 圆周采样视点   │  去重滤波     │
    │                          │ sampleView    │  filterView   │
    │                          │ points()      │  points()     │
    │                          └───────────────┴───────────────┘
    │                                          │
    │                                          ▼
    │                               最终视点列表
```

### 基于子网格的优势

1. **空间隔离**：利用GlobalMap已有的SubGrid空间划分，自然分离远距离的前沿点
2. **避免连片**：即使360度LiDAR初期前沿形成环状，不同SubGrid的前沿也不会聚成一个簇
3. **局部聚类**：在每个SubGrid内独立进行BFS区域增长聚类

### 聚类算法（BFS区域增长）

```
for each 子网格 sg in subgrid_frontiers:
    if sg.points.size < min_cluster_size:
        continue
    
    for each 未访问点 p in sg.points:
        BFS扩展:
            queue.push(p)
            while queue非空:
                curr = queue.pop()
                cluster.add(curr)
                for each 点 q:
                    if ||q - curr|| <= cluster_eps && q未访问:
                        queue.push(q)
        
        if cluster.size >= min_cluster_size:
            计算中心、范围
            保存簇
```

## 配置参数

```yaml
exploration:
  # 视点采样参数
  viewpoint_height: 1.0       # 固定高度（起飞高度）[m]
  candidate_rmin: 1.5         # 最小采样半径 [m]
  candidate_rmax: 4.5         # 最大采样半径 [m]
  candidate_rnum: 3           # 半径采样数量
  candidate_dphi: 0.5         # 角度间隔 [rad]
  
  # 前沿簇参数
  cluster_eps: 1.0            # 聚类半径 [m]
  min_cluster_size: 10        # 最小簇大小
  cluster_split_threshold: 3.0 # 簇分裂阈值 [m]
  max_clusters: 20            # 最大簇数
  
  # 安全参数
  uav_radius: 0.35            # 无人机半径 [m]
  safety_margin: 0.2          # 安全边距 [m]
  sensor_range_max: 5.0       # 最大感知距离 [m]
  optimal_distance: 3.0       # 最优观测距离 [m]
  
  # 滤波参数
  min_viewpoint_distance: 0.8 # 视点最小间距 [m]
```

## 调试信息

启动 `exploration_planner.launch` 后，可观察以下日志：

```
[ViewpointGen] Processing 5 subgrids with frontiers
[ViewpointGen] Formed 8 clusters
[ViewpointGen] After split: 10 clusters
[ViewpointGen] Generated 10 candidate viewpoints
[ViewpointGen] Final: 8 viewpoints from 10 clusters
```

## RViz 可视化

- `/exploration/frontier_clusters`: 前沿簇（球体 + 标签）
- `/exploration/viewpoints`: 探索视点（球体 + 方向箭头）
- `/exploration/frustums`: 视点视锥体

---
**修复日期**：2025-01-XX  
**影响范围**：全局地图更新逻辑 + 探索视点生成  
**策略**：基于SubGrid空间分区 + BFS聚类 + 圆周采样 + 去重滤波

