# STAR-MVF 地图系统架构文档

## 1. 概述

STAR-MVF (STAR Multi-View Fusion) 地图系统是一个高效的三维体素地图框架，专为无人机自主探索任务设计。系统采用**局部-全局双层架构**，实现实时传感器数据处理与长期环境建模的解耦。

### 1.1 设计目标

- **实时性**：支持高频率（>10Hz）点云输入
- **内存效率**：稀疏存储，仅保存已观测区域
- **探索支持**：增量式前沿提取，子网格管理
- **规划支持**：集成膨胀地图和ESDF，支持路径规划
- **向后兼容**：保留 `rog_map` 命名空间接口，提供 `UnifiedMap` 兼容层

### 1.2 借鉴项目

| 项目 | 借鉴内容 |
|------|----------|
| **EDEN** | 子网格（SubGrid）前沿管理、历史节点关联机制 |
| **FALCON** | 局部-全局地图同步策略、相机视锥检测 |
| **ROG-Map** | 概率占据更新、射线投射、膨胀地图、ESDF |
| **Voxblox** | 稀疏哈希体素存储思想 |

---

## 2. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        MapManager                                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    ROS Interface                          │   │
│  │  • LiDAR Subscriber    • Odom Subscriber                  │   │
│  │  • Sync Timer          • Visualization Timer              │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│          ┌───────────────────┼───────────────────┐              │
│          ▼                                       ▼              │
│  ┌───────────────────┐               ┌───────────────────────┐  │
│  │    LocalMap       │   ──sync──►   │      GlobalMap        │  │
│  │  (滑动窗口地图)    │               │    (全局稀疏地图)      │  │
│  │                   │               │                       │  │
│  │  • 概率更新        │               │  • 稀疏哈希存储        │  │
│  │  • 射线投射        │               │  • 子网格管理          │  │
│  │  • 实时处理        │               │  • 前沿提取            │  │
│  └───────────────────┘               │  • 膨胀地图            │  │
│                                       │  • ESDF距离场          │  │
│                                       └───────────────────────┘  │
│                                                 │                │
│                                                 ▼                │
│                                       ┌───────────────────────┐  │
│                                       │    UnifiedMap         │  │
│                                       │  (兼容层，包装GlobalMap)│  │
│                                       └───────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. 核心数据结构

### 3.1 体素状态 (VoxelState)

采用**位掩码设计**，支持多维度状态组合：

```cpp
// 8位状态编码
// Bit 0-1: 几何状态 (UNKNOWN=0, FREE=1, OCCUPIED=2)
// Bit 2:   视觉状态 (VISITED=4)
// Bit 3:   语义状态 (TARGET=8)
// Bit 4:   前沿状态 (FRONTIER=16)

namespace VoxelStateBits {
    constexpr VoxelState GEO_UNKNOWN  = 0b00000000;  // 0
    constexpr VoxelState GEO_FREE     = 0b00000001;  // 1
    constexpr VoxelState GEO_OCCUPIED = 0b00000010;  // 2
    constexpr VoxelState VISITED      = 0b00000100;  // 4
    constexpr VoxelState TARGET       = 0b00001000;  // 8
    constexpr VoxelState FRONTIER     = 0b00010000;  // 16
    
    // 辅助函数
    inline VoxelState getGeoState(VoxelState state);
    inline bool isUnknown(VoxelState state);
    inline bool isFree(VoxelState state);
    inline bool isOccupied(VoxelState state);
    inline bool isVisited(VoxelState state);
    inline bool isTarget(VoxelState state);
    inline bool isFrontier(VoxelState state);
}
```

**优势**：
- 单字节存储多维状态
- 位运算高效查询
- 支持状态组合（如 `OCCUPIED | VISITED`）

### 3.2 体素索引 (VoxelIndex)

```cpp
struct VoxelIndex {
    int64_t x, y, z;
    
    // 支持算术运算
    VoxelIndex operator+(const VoxelIndex& other) const;
    VoxelIndex operator-(const VoxelIndex& other) const;
};

// 高效哈希函数（黄金比例哈希）
struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex& idx) const {
        constexpr std::size_t prime1 = 73856093ULL;
        constexpr std::size_t prime2 = 19349663ULL;
        constexpr std::size_t prime3 = 83492791ULL;
        return (idx.x * prime1) ^ (idx.y * prime2) ^ (idx.z * prime3);
    }
};
```

### 3.3 子网格 (SubGrid)

子网格是全局地图的空间划分单元，用于：
- 高效的前沿聚类
- 局部统计信息维护
- 增量更新优化

```cpp
struct SubGrid {
    // 边界信息
    VoxelIndex global_min_idx, global_max_idx;
    Position world_min, world_max, center;
    SubGridIndex grid_idx;
    
    // 统计信息
    int total_voxels, unknown_count, occupied_count, free_count;
    float exploration_rate, unknown_rate;
    
    // 前沿管理
    bool has_frontier;
    int frontier_count;
    bool frontier_dirty;  // 脏标记，用于增量更新
    bool was_frontier_last_time;
    
    // 体素集合
    std::unordered_set<VoxelIndex> explored_voxels_;
    std::unordered_set<VoxelIndex> unexplored_voxels_;
    std::unordered_set<VoxelIndex> sub_grid_frontier_voxels_;
    bool unexplored_initialized_;
};
```

### 3.4 膨胀地图配置 (InflationConfig)

```cpp
struct InflationConfig {
    float inflation_radius = 0.3f;      // 膨胀半径 (m)
    float inflation_resolution = 0.1f;  // 膨胀地图分辨率
    int inflation_grid_radius = 3;       // 膨胀格子数
    bool enable_inflation = true;
};
```

### 3.5 ESDF配置 (ESDFConfig)

```cpp
struct ESDFConfig {
    bool enable_esdf = false;
    float esdf_resolution = 0.1f;
    float esdf_max_distance = 3.0f;
    float local_update_range_x = 10.0f;
    float local_update_range_y = 10.0f;
    float local_update_range_z = 5.0f;
};
```

---

## 4. 模块详解

### 4.1 LocalMap (局部地图)

**职责**：实时处理传感器数据，维护以UAV为中心的滑动窗口地图

**核心特性**：
- **概率更新**：基于 log-odds 的占据概率模型
- **射线投射**：沿传感器射线清除自由空间
- **滑动窗口**：跟随UAV移动，自动重置超出范围的区域

**参数配置**：
```yaml
local_map:
  size: [20.0, 20.0, 6.0]        # 局部地图尺寸 (m)
  resolution: 0.1               # 分辨率 (m)
  log_odds_hit: 0.85            # 命中更新量
  log_odds_miss: -0.4           # 穿透更新量
  log_odds_min: -2.0            # 最小log-odds
  log_odds_max: 3.5             # 最大log-odds
  log_odds_occ_thresh: 0.0      # 占据判定阈值
  use_raycast: true             # 启用射线投射
  raycast_step: 0.1             # 射线步长 (m)
  sensor_max_range: 5.0         # 传感器最大距离
  sensor_min_range: 0.1         # 传感器最小距离
  sliding_thresh: 1.0           # 滑动阈值
  sliding_enabled: true         # 启用滑动窗口
```

**更新流程**：
```
点云输入
    │
    ▼
┌─────────────────┐
│ 距离过滤         │  排除过近/过远点
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 射线投射         │  sensor_pos → end_point
│ (清除自由空间)   │  沿途体素 log_odds -= miss
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 终点占据更新     │  end_point log_odds += hit
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 记录变化体素     │  updated_occupied_, updated_free_
└─────────────────┘
```

### 4.2 GlobalMap (全局地图)

**职责**：长期环境建模，前沿管理，探索规划支持

**核心特性**：
- **稀疏哈希存储**：`unordered_map<VoxelIndex, VoxelState>`
- **子网格管理**：将全局地图划分为 N×N 个子网格
- **增量前沿提取**：仅更新脏子网格
- **膨胀地图**：支持障碍物膨胀，用于路径规划
- **ESDF距离场**：支持欧氏符号距离场，用于轨迹优化

**存储结构**：
```cpp
class GlobalMap {
    // 主要存储
    std::unordered_map<VoxelIndex, VoxelState, VoxelIndexHash> voxels_;
    
    // 子网格
    std::vector<SubGrid> sub_grids_;
    
    // 前沿缓存
    std::unordered_set<VoxelIndex> frontier_voxels_;
    std::unordered_set<int> current_frontier_subgrids_;
    std::unordered_set<int> dirty_subgrids_;
    
    // 膨胀地图数据
    std::unordered_map<VoxelIndex, int16_t, VoxelIndexHash> inflation_count_;
    std::vector<VoxelIndex> inflation_neighbor_offsets_;
    
    // ESDF数据
    std::unordered_map<VoxelIndex, double, VoxelIndexHash> esdf_distance_;
    Position esdf_center_;
};
```

**前沿定义**：
前沿体素是满足以下条件的**未知体素**：
1. 至少有一个6-邻域是**自由体素**
2. 高度在 `[frontier_height_min, frontier_height_max]` 范围内
3. 不在地图边界

**膨胀地图**：
- 使用计数机制：每个体素记录有多少个占据体素对其产生膨胀影响
- 支持动态更新：当体素状态改变时，增量更新膨胀区域
- 查询接口：`isOccupiedInflate()`, `isUnknownInflate()`, `isKnownFreeInflate()`

**ESDF距离场**：
- 稀疏存储：只存储局部更新范围内的距离值
- 局部更新：以UAV位置为中心，在配置范围内更新ESDF
- 查询接口：`getESDFDistance()`, `getESDFGradient()`

### 4.3 MapManager (地图管理器)

**职责**：协调局部和全局地图，提供统一接口

**核心功能**：
- 接收传感器数据并更新局部地图
- 定时将局部地图同步到全局地图
- 提供探索规划器所需的接口
- 管理可视化发布

**同步机制** (借鉴 FALCON)：
```
┌─────────────────────────────────────────────────────────────┐
│                   Sync Timer (10 Hz)                         │
│                                                              │
│  1. 获取局部地图更新缓存                                      │
│     LocalMap::getUpdatedVoxels(occupied, free)              │
│                                                              │
│  2. 批量更新全局地图                                          │
│     GlobalMap::updateOccupiedVoxels(occupied, free)         │
│                                                              │
│  3. 增量提取前沿                                              │
│     GlobalMap::extractGlobalFrontierIncremental()           │
│                                                              │
│  4. 更新膨胀地图（如果启用）                                  │
│     GlobalMap::updateInflation()                            │
│                                                              │
│  5. 更新ESDF（如果启用）                                      │
│     GlobalMap::updateESDF(uav_position)                     │
│                                                              │
│  6. 清除局部地图缓存                                          │
│     LocalMap::clearUpdateCache()                            │
└─────────────────────────────────────────────────────────────┘
```

### 4.4 UnifiedMap (统一地图兼容层)

**职责**：提供与旧 `ROGMap` 接口兼容的包装类

**核心功能**：
- 包装 `star_mvf::GlobalMap`，提供旧接口
- 兼容 `common_lib.hpp` 中定义的类型（`Vec3f`, `Vec3i`, `GridType` 等）
- 支持旧规划器和探索器的直接使用

**接口映射**：
```cpp
// 旧接口 → 新接口
bool isOccupied(const Vec3f& pos) 
    → global_map_->isOccupied(pos.cast<float>())

bool isOccupiedInflate(const Vec3f& pos) 
    → global_map_->isOccupiedInflate(pos.cast<float>())

double getESDFDistance(const Vec3f& pos) 
    → global_map_->getESDFDistance(pos.cast<float>())

bool isLineFree(const Vec3f& start, const Vec3f& end) 
    → global_map_->isLineFree(start.cast<float>(), end.cast<float>())
```

---

## 5. 增量前沿提取算法

### 5.1 算法流程 (借鉴 EDEN)

```
extractGlobalFrontierIncremental()
    │
    ▼
┌─────────────────────────────────────┐
│ 遍历脏子网格 (dirty_subgrids_)      │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 清除该子网格的旧前沿                 │
│ frontier_voxels_.erase(old)         │
│ sg.sub_grid_frontier_voxels_.clear()│
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 遍历已探索体素 (explored_voxels_)   │
│ 检查每个FREE体素的6邻域             │
│ 如果邻域有UNKNOWN → 标记为前沿      │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 高度过滤 + 边界过滤                  │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 更新子网格前沿缓存                   │
│ updateFrontierSubGridCache()        │
└─────────────────────────────────────┘
```

### 5.2 优化策略

1. **脏标记机制**：只处理有更新的子网格
2. **探索率自适应**：根据 `exploration_rate` 选择遍历方向
   - `< 0.5`：遍历已探索体素（更少）
   - `≥ 0.5`：遍历未探索体素（更少）
3. **缓存复用**：`current_frontier_subgrids_` 避免重复计算

---

## 6. 膨胀地图算法

### 6.1 膨胀更新机制

```
updateInflation(changed_idx, is_occupied)
    │
    ▼
┌─────────────────────────────────────┐
│ 清理旧的膨胀影响                     │
│ 遍历膨胀邻域，减少计数               │
│ inflation_count_[neighbor]--        │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 如果is_occupied，添加新的膨胀影响     │
│ 遍历膨胀邻域，增加计数               │
│ inflation_count_[neighbor]++        │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 清理计数为0的体素                    │
│ inflation_count_.erase(if count==0) │
└─────────────────────────────────────┘
```

### 6.2 膨胀查询

- `isOccupiedInflate(pos)`：检查位置是否在膨胀后被占据
- `isUnknownInflate(pos)`：检查位置是否在膨胀后未知
- `isKnownFreeInflate(pos)`：检查位置是否在膨胀后已知自由

---

## 7. ESDF距离场算法

### 7.1 ESDF更新机制

```
updateESDF(uav_position)
    │
    ▼
┌─────────────────────────────────────┐
│ 确定局部更新范围                     │
│ 以uav_position为中心，配置范围       │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 初始化ESDF缓冲区                    │
│ 障碍物：distance = 0                │
│ 其他：distance = max_distance       │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ BFS计算距离场                        │
│ 从障碍物开始，6邻域传播              │
│ 更新每个体素到最近障碍物的距离        │
└─────────────────────────────────────┘
```

### 7.2 ESDF查询

- `getESDFDistance(pos)`：获取到最近障碍物的距离
- `getESDFGradient(pos)`：获取ESDF梯度（用于轨迹优化）

---

## 8. 运行效率分析

### 8.1 时间复杂度

| 操作 | 复杂度 | 说明 |
|------|--------|------|
| 体素查询 | O(1) | 哈希表查找 |
| 体素更新 | O(1) | 哈希表插入 |
| 局部→全局同步 | O(k) | k = 变化体素数 |
| 前沿提取 | O(m·n) | m = 脏子网格数, n = 平均已探索体素数 |
| 子网格查找 | O(1) | 直接索引 |
| 膨胀更新 | O(r³) | r = 膨胀半径（体素数） |
| ESDF更新 | O(L³) | L = 局部更新范围（体素数） |

### 8.2 空间复杂度

| 组件 | 复杂度 | 说明 |
|------|--------|------|
| LocalMap | O(L³/r³) | L = 局部地图边长, r = 分辨率 |
| GlobalMap | O(n) | n = 已观测体素数（稀疏） |
| SubGrid | O(N²) | N = 子网格单边数量 |
| 前沿缓存 | O(f) | f = 前沿体素数 |
| 膨胀地图 | O(n·r³) | n = 占据体素数, r = 膨胀半径 |
| ESDF | O(L³/r³) | L = 局部更新范围, r = 分辨率 |

### 8.3 典型性能指标

在典型室内探索场景（50m×50m×5m，分辨率0.1m）：

| 指标 | 数值 |
|------|------|
| 点云处理频率 | > 20 Hz |
| 单帧处理时间 | < 30 ms |
| 前沿提取时间 | < 10 ms |
| 膨胀更新时间 | < 5 ms |
| ESDF更新时间 | < 20 ms |
| 内存占用（10分钟探索后） | ~ 200 MB |

---

## 9. 接口说明

### 9.1 基础查询

```cpp
// 获取地图范围
Position getGlobalMapMin() const;
Position getGlobalMapMax() const;
float getResolution() const;
float getInfResolution() const;  // 膨胀地图分辨率

// 状态查询
bool isOccupied(const Position& pos);
bool isFree(const Position& pos);
bool isUnknown(const Position& pos);
VoxelState getVoxelState(const Position& pos);

// 坐标转换
void posToIndex(const Position& pos, VoxelIndex& idx);
Position indexToPos(const VoxelIndex& idx);
```

### 9.2 前沿接口

```cpp
// 前沿提取
void extractGlobalFrontierIncremental();

// 前沿查询
void getFrontierSubGrids(std::vector<int>& subgrid_ids);
void getFrontierPositions(std::vector<Position>& positions);
void getFrontierPositionsBySubGrid(std::vector<SubGridFrontierInfo>& info);
void getFrontierPointCloud(sensor_msgs::PointCloud2& cloud);
size_t getFrontierCount() const;
```

### 9.3 子网格接口

```cpp
// 子网格访问
SubGrid& getSubGrid(int linear_idx);
void posToSubGrid(const Position& pos, SubGridIndex& sg_idx);
int subGridToLinearIndex(const SubGridIndex& idx);
```

### 9.4 膨胀地图接口

```cpp
// 膨胀查询
bool isOccupiedInflate(const Position& pos);
bool isUnknownInflate(const Position& pos);
bool isKnownFreeInflate(const Position& pos);

// 膨胀更新（内部自动调用）
void updateInflation(const VoxelIndex& idx, bool is_occupied);
```

### 9.5 ESDF接口

```cpp
// ESDF查询
double getESDFDistance(const Position& pos);
void getESDFGradient(const Position& pos, Position& gradient);
bool isESDFEnabled() const;

// ESDF更新
void updateESDF(const Position& center);
```

### 9.6 路径检测接口

```cpp
// 路径安全检查
bool isPathSafe(const Position& start, const Position& end, float margin = 0.0f);

// 区域安全检查
bool isSphereSafe(const Position& center, float radius);

// 直线检测
bool isLineFree(const Position& start, const Position& end, double max_dist = 999999.0);
bool isLineFree(const Position& start, const Position& end, Position& collision_point, double max_dist = 999999.0);
bool isLineFreeInflate(const Position& start, const Position& end, bool use_unknown_as_occ = false);
```

### 9.7 网格类型查询（兼容旧代码）

```cpp
GridType getGridType(const Position& pos) const;
GridType getInfGridType(const Position& pos) const;
double getMapValue(const Position& pos) const;  // 占据概率
```

### 9.8 包围盒搜索

```cpp
void boxSearch(const Position& box_min, const Position& box_max,
               GridType type, std::vector<Position>& out_points);
void boxSearchInflate(const Position& box_min, const Position& box_max,
                      GridType type, std::vector<Position>& out_points);
```

---

## 10. 命名空间兼容性

为保持向后兼容，`star_mvf.h` 提供 `rog_map` 命名空间别名：

```cpp
namespace rog_map {
    // 类型别名
    using GlobalMap = star_mvf::GlobalMap;
    using LocalMap = star_mvf::LocalMap;
    using MapManager = star_mvf::MapManager;
    using VoxelIndex = star_mvf::VoxelIndex;
    using VoxelState = star_mvf::VoxelState;
    using SubGrid = star_mvf::SubGrid;
    using SubGridFrontierInfo = star_mvf::SubGridFrontierInfo;
    
    // 配置类型别名
    using StarMvfInflationConfig = star_mvf::InflationConfig;
    using StarMvfESDFConfig = star_mvf::ESDFConfig;
    using StarMvfRobotState = star_mvf::RobotState;
    using StarMvfGridType = star_mvf::GridType;
    
    // VoxelStateBits 命名空间
    namespace VoxelStateBits = star_mvf::VoxelStateBits;
}
```

**注意**：基础类型（`Vec3f`, `Vec3i`, `GridType`, `Pose` 等）已在 `common_lib.hpp` 中定义，`star_mvf.h` 不重复定义以避免冲突。

现有代码可无缝迁移：
```cpp
// 旧代码
rog_map::GlobalMap::Ptr map;

// 新代码（等价）
star_mvf::GlobalMap::Ptr map;
```

---

## 11. 文件结构

```
include/star_mvf/
├── star_mvf.h              # 统一头文件 + rog_map 兼容别名
└── map/
    ├── map_types.h         # 核心数据类型（VoxelIndex, VoxelState, SubGrid等）
    ├── local_map.h         # 局部地图头文件
    ├── global_map.h        # 全局地图头文件（包含膨胀和ESDF）
    └── map_manager.h       # 地图管理器头文件

include/rog_map/
└── unified_map.h           # 统一地图兼容层（包装star_mvf::GlobalMap）

src/star_mvf/
└── map/
    ├── local_map.cpp       # 局部地图实现
    ├── global_map.cpp      # 全局地图实现（包含膨胀和ESDF实现）
    └── map_manager.cpp     # 地图管理器实现

src/rog_map/
└── unified_map.cpp         # 统一地图兼容层实现
```

---

## 12. 配置示例

```yaml
star_mvf:
  resolution: 0.1
  frame_id: "map"
  sync_rate: 10.0
  viz_rate: 5.0
  
  global_map:
    map_size: [100.0, 100.0, 5.0]
    map_model: 0  # 0: center, 1: right_bottom, 2: right_middle, 3: left_bottom
    ground_height: -0.1
    sub_grid_xy: 0.1
    frontier_height_min: 0.3
    frontier_height_max: 2.5
  
  local_map:
    size: [20.0, 20.0, 6.0]
    resolution: 0.1
    log_odds_hit: 0.85
    log_odds_miss: -0.4
    log_odds_min: -2.0
    log_odds_max: 3.5
    log_odds_occ_thresh: 0.0
    use_raycast: true
    raycast_step: 0.1
    sensor_max_range: 5.0
    sensor_min_range: 0.1
    sliding_thresh: 1.0
    sliding_enabled: true
  
  camera_map:
    enable: false
    fov_h_deg: 90.0
    fov_v_deg: 60.0
    max_dist: 5.0
    min_dist: 0.3
  
  inflation_map:
    enable_inflation: true
    inflation_radius: 0.3
    inflation_resolution: 0.1
    inflation_grid_radius: 3
  
  esdf_map:
    enable_esdf: false
    esdf_resolution: 0.1
    esdf_max_distance: 3.0
    local_update_range_x: 10.0
    local_update_range_y: 10.0
    local_update_range_z: 5.0
```

---

## 13. 可视化话题

系统发布以下ROS话题用于可视化（`frame_id = "map"`）：

| 话题名称 | 类型 | 说明 |
|---------|------|------|
| `global_map/occupied` | `sensor_msgs/PointCloud2` | 全局占据点云 |
| `global_map/free` | `sensor_msgs/PointCloud2` | 全局自由点云 |
| `global_map/frontier` | `sensor_msgs/PointCloud2` | 前沿点云 |
| `global_map/range` | `visualization_msgs/MarkerArray` | 地图范围标记 |
| `global_map/visited_voxels` | `sensor_msgs/PointCloud2` | 已访问体素点云 |
| `global_map/inflation` | `sensor_msgs/PointCloud2` | 膨胀地图点云（如果启用） |
| `global_map/esdf` | `sensor_msgs/PointCloud2` | ESDF可视化（如果启用） |

---

## 14. 与旧系统对比

| 特性 | 旧系统 (global_map.h) | 新系统 (star_mvf) |
|------|----------------------|-------------------|
| 存储方式 | 稀疏哈希 | 局部数组 + 全局稀疏哈希 |
| 更新策略 | 直接更新 | 局部缓冲 → 批量同步 |
| 前沿提取 | 全量扫描 | 增量脏标记 |
| 膨胀地图 | 集成在ROGMap中 | 集成在GlobalMap中 |
| ESDF | 集成在ROGMap中 | 集成在GlobalMap中 |
| 代码结构 | 单文件 700+ 行 | 模块化，4个类 |
| 可维护性 | 较低 | 较高 |
| 扩展性 | 有限 | 易于扩展 |
| 兼容性 | 无 | 提供UnifiedMap兼容层 |

---

## 15. 使用示例

### 15.1 使用MapManager（推荐）

```cpp
#include "star_mvf/star_mvf.h"

ros::NodeHandle nh, nh_private;
star_mvf::MapManager::Ptr map_manager = 
    std::make_shared<star_mvf::MapManager>(nh, nh_private);

map_manager->initFromROS("star_mvf");
map_manager->start();

// 获取全局地图
auto global_map = map_manager->getGlobalMap();

// 查询接口
bool occupied = global_map->isOccupied(Position(1.0, 2.0, 0.5));
bool inflated = global_map->isOccupiedInflate(Position(1.0, 2.0, 0.5));
double esdf_dist = global_map->getESDFDistance(Position(1.0, 2.0, 0.5));
```

### 15.2 使用UnifiedMap（兼容旧代码）

```cpp
#include "rog_map/unified_map.h"

ros::NodeHandle nh;
rog_map::UnifiedMap::Ptr map = 
    std::make_shared<rog_map::UnifiedMap>(nh);

// 更新地图
PointCloud cloud;
Pose pose;
map->updateMap(cloud, pose);

// 使用旧接口
Vec3f pos(1.0, 2.0, 0.5);
bool occupied = map->isOccupied(pos);
bool inflated = map->isOccupiedInflate(pos);
double esdf_dist = map->getESDFDistance(pos);
```

---

## 16. 未来优化方向

1. **GPU加速**：射线投射并行化
2. **多分辨率**：支持 octree 自适应分辨率
3. **增量ESDF优化**：使用更高效的增量更新算法（如Voxblox的ESDF）
4. **语义融合**：支持目标检测结果融合
5. **动态障碍物**：支持动态障碍物检测和跟踪

---

*文档版本：2.0*  
*最后更新：2026-01-07*
