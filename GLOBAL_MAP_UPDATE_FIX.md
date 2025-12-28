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
**修复日期**：2025-01-XX  
**影响范围**：全局地图更新逻辑  
**向后兼容性**：完全兼容，仅改进更新策略  

