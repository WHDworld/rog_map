/**
 * @file map_types.h
 * @brief STAR-MVF 地图系统核心数据类型定义
 * 
 * 参考项目：EDEN, FALCON
 */

#ifndef STAR_MVF_MAP_TYPES_H
#define STAR_MVF_MAP_TYPES_H

#include <Eigen/Dense>
#include <cstdint>
#include <functional>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <queue>
#include <mutex>

namespace star_mvf {

// ==================== 基础类型别名 ====================
using Position = Eigen::Vector3f;
using Positiond = Eigen::Vector3d;
using Quaternion = Eigen::Quaternionf;

// 兼容旧ROG-Map类型
using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Vec3i = Eigen::Vector3i;

// ==================== 体素索引 ====================
struct VoxelIndex {
    int64_t x, y, z;
    
    VoxelIndex() : x(0), y(0), z(0) {}
    VoxelIndex(int64_t x_, int64_t y_, int64_t z_) : x(x_), y(y_), z(z_) {}
    
    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    bool operator!=(const VoxelIndex& other) const {
        return !(*this == other);
    }
    
    VoxelIndex operator+(const VoxelIndex& other) const {
        return VoxelIndex(x + other.x, y + other.y, z + other.z);
    }
    
    VoxelIndex operator-(const VoxelIndex& other) const {
        return VoxelIndex(x - other.x, y - other.y, z - other.z);
    }
};

// 兼容旧名称
using GlobalVoxelIndex = VoxelIndex;

// 体素索引哈希函数
struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex& idx) const {
        // 使用黄金比例哈希
        constexpr std::size_t prime1 = 73856093ULL;
        constexpr std::size_t prime2 = 19349663ULL;
        constexpr std::size_t prime3 = 83492791ULL;
        return (static_cast<std::size_t>(idx.x) * prime1) ^
               (static_cast<std::size_t>(idx.y) * prime2) ^
               (static_cast<std::size_t>(idx.z) * prime3);
    }
};

using GlobalVoxelIndexHash = VoxelIndexHash;

// ==================== 体素状态（位掩码设计） ====================
using VoxelState = uint8_t;
using GlobalVoxelState = VoxelState;

/**
 * @brief 体素状态位定义
 * 
 * 使用位掩码设计，支持多维度状态组合：
 * - 几何层（Bit 0-1）：UNKNOWN=0, FREE=1, OCCUPIED=2
 * - 视觉层（Bit 2）：VISITED=4 （是否被相机观测过）
 * - 语义层（Bit 3）：TARGET=8 （是否为感兴趣目标）
 * - 前沿层（Bit 4）：FRONTIER=16 （是否为前沿体素）
 */
namespace VoxelStateBits {
    // 几何状态（2位）
    constexpr VoxelState GEO_UNKNOWN  = 0b00000000;  // 0: 未知
    constexpr VoxelState GEO_FREE     = 0b00000001;  // 1: 自由
    constexpr VoxelState GEO_OCCUPIED = 0b00000010;  // 2: 占据
    constexpr VoxelState MASK_GEO     = 0b00000011;  // 几何层掩码
    
    // 视觉状态（1位）
    constexpr VoxelState VISITED      = 0b00000100;  // 4: 已访问
    constexpr VoxelState MASK_VISUAL  = 0b00000100;
    
    // 语义状态（1位）
    constexpr VoxelState TARGET       = 0b00001000;  // 8: 目标
    constexpr VoxelState MASK_SEMANTIC = 0b00001000;
    
    // 前沿状态（1位）
    constexpr VoxelState FRONTIER     = 0b00010000;  // 16: 前沿
    constexpr VoxelState MASK_FRONTIER = 0b00010000;
    
    // 辅助函数
    inline VoxelState getGeoState(VoxelState state) { return state & MASK_GEO; }
    inline bool isUnknown(VoxelState state) { return getGeoState(state) == GEO_UNKNOWN; }
    inline bool isFree(VoxelState state) { return getGeoState(state) == GEO_FREE; }
    inline bool isOccupied(VoxelState state) { return getGeoState(state) == GEO_OCCUPIED; }
    inline bool isVisited(VoxelState state) { return (state & VISITED) != 0; }
    inline bool isTarget(VoxelState state) { return (state & TARGET) != 0; }
    inline bool isFrontier(VoxelState state) { return (state & FRONTIER) != 0; }
    // 状态修改
    inline void setGeoState(VoxelState& state, VoxelState geo) {
        state = (state & ~MASK_GEO) | (geo & MASK_GEO);
    }
    inline void setVisited(VoxelState& state, bool visited) {
        if (visited) state |= VISITED;
        else state &= ~VISITED;
    }
    inline void setFrontier(VoxelState& state, bool frontier) {
        if (frontier) state |= FRONTIER;
        else state &= ~FRONTIER;
    }
}

// ==================== 子网格索引 ====================
struct SubGridIndex {
    int x, y;
    
    SubGridIndex() : x(0), y(0) {}
    SubGridIndex(int x_, int y_) : x(x_), y(y_) {}
    
    int toLinearIndex(int grid_x_num) const {
        return y * grid_x_num + x;
    }
    
    bool operator==(const SubGridIndex& other) const {
        return x == other.x && y == other.y;
    }
};

// ==================== 语义目标元数据 (用于高价值目标缓存) ====================
/**
 * @brief 语义目标元数据
 * 
 * 存储从几何地图中提取的高价值目标信息
 * 设计为轻量级结构，便于 SubGrid 级缓存
 */
struct SemanticTargetMetadata {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int id = -1;                    // 目标唯一 ID
    Position centroid;              // 质心位置
    Position bbox_min;              // 包围盒最小点
    Position bbox_max;              // 包围盒最大点
    VoxelIndex seed_voxel;          // 种子体素索引 (用于惰性重生成)
    float score = 0.0f;             // 目标评分
    double timestamp = 0.0;         // 发现时间戳
    
    // ========== 探索视点信息 ==========
    Position viewpoint_position;    // 探索视点位置
    float viewpoint_yaw = 0.0f;     // 视点朝向（弧度，朝向目标质心）
    bool has_viewpoint = false;     // 是否生成了有效视点
    bool is_observed = false;       // 是否已被相机观测（由状态机更新）
    
    SemanticTargetMetadata() {
        centroid.setZero();
        bbox_min.setZero();
        bbox_max.setZero();
        viewpoint_position.setZero();
    }
    
    // 尺寸计算辅助函数
    float getHeight() const { return bbox_max.z() - bbox_min.z(); }
    float getWidth() const { 
        return std::max(bbox_max.x() - bbox_min.x(), 
                        bbox_max.y() - bbox_min.y()); 
    }
};

// ==================== 子网格结构 ====================
struct SubGrid {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // 边界信息
    VoxelIndex global_min_idx, global_max_idx;
    Position world_min, world_max;
    Position center;
    SubGridIndex grid_idx;
    
    // 统计信息
    int total_voxels = 0;
    int unknown_count = 0;
    int occupied_count = 0;
    int free_count = 0;
    float exploration_rate = 0.0f;
    float unknown_rate = 1.0f;
    
    // 前沿信息
    bool has_frontier = false;
    int frontier_count = 0;
    bool frontier_dirty = true;
    bool was_frontier_last_time = false;
    
    // ========== 优化的体素存储结构 ==========
    // FREE体素列表（用于前沿检测）
    std::vector<VoxelIndex> free_voxels_list_;
    // 映射表：VoxelIndex -> free_voxels_list_中的位置（-1表示不在列表中）
    std::unordered_map<VoxelIndex, int, VoxelIndexHash> free_to_list_pos_;
    
    // UNKNOWN体素列表（可选，用于其他用途）
    std::vector<VoxelIndex> unknown_voxels_list_;
    std::unordered_map<VoxelIndex, int, VoxelIndexHash> unknown_to_list_pos_;
    
    // ==================== 3. OCCUPIED 体素结构 (视觉规划专用) ====================
    // 未被视觉观测的占据体素 (语义目标提取的数据来源)
    std::vector<VoxelIndex> occ_unobserved_list_;
    std::unordered_map<VoxelIndex, int, VoxelIndexHash> occ_unobserved_map_;
    
    /**
     * @brief 语义分析状态
     * UNKNOWN: 未探索
     * ACCUMULATING: 正在建图（有体素但探索未完成）
     * READY: 已达到探索率阈值，等待分析
     * ANALYZED: 已分析完成（永不再处理）
     */
    enum class SemanticStatus {
        UNKNOWN,       // 未探索
        ACCUMULATING,  // 正在建图
        READY,         // 等待分析
        ANALYZED       // 已分析完成
    };
    
    SemanticStatus semantic_status_ = SemanticStatus::UNKNOWN;
    std::vector<SemanticTargetMetadata> cached_targets_;  // 提取的高价值目标 (ANALYZED 后不再修改)

    // 前沿体素集合（保持unordered_set，因为需要快速查找和去重）
    std::unordered_set<VoxelIndex, VoxelIndexHash> sub_grid_frontier_voxels_;
        
    /**
     * @brief 添加FREE体素到列表（O(1)）
     */
     void addFreeVoxel(const VoxelIndex& idx) {
        if (free_to_list_pos_.count(idx)) return;
        
        free_voxels_list_.push_back(idx);
        free_to_list_pos_[idx] = static_cast<int>(free_voxels_list_.size() - 1);
        free_count++;
    }
    
    /**
     * @brief 从FREE列表中移除体素（O(1) Swap-and-Pop）
     */
    void removeFreeVoxel(const VoxelIndex& idx) {
        auto it = free_to_list_pos_.find(idx);
        if (it == free_to_list_pos_.end()) return;  // 不在列表中
        
        int pos = it->second;
        int last_pos = static_cast<int>(free_voxels_list_.size() - 1);
        
        // Swap-and-Pop优化
        if (pos != last_pos) {
            VoxelIndex last_idx = free_voxels_list_[last_pos];
            free_voxels_list_[pos] = last_idx;
            free_to_list_pos_[last_idx] = pos;  // 更新被搬运元素的映射
        }
        
        free_voxels_list_.pop_back();
        free_to_list_pos_.erase(it);
        free_count--;
    }
        /**
     * @brief 添加UNKNOWN体素到列表（O(1)）
     */
     void addUnknownVoxel(const VoxelIndex& idx) {
        auto it = unknown_to_list_pos_.find(idx);
        if (it != unknown_to_list_pos_.end()) return;
        
        unknown_voxels_list_.push_back(idx);
        unknown_to_list_pos_[idx] = static_cast<int>(unknown_voxels_list_.size() - 1);
        unknown_count++;
    }
    
    /**
     * @brief 从UNKNOWN列表中移除体素（O(1) Swap-and-Pop）
     */
    void removeUnknownVoxel(const VoxelIndex& idx) {
        auto it = unknown_to_list_pos_.find(idx);
        if (it == unknown_to_list_pos_.end()) return;
        
        int pos = it->second;
        int last_pos = static_cast<int>(unknown_voxels_list_.size() - 1);
        
        if (pos != last_pos) {
            VoxelIndex last_idx = unknown_voxels_list_[last_pos];
            unknown_voxels_list_[pos] = last_idx;
            unknown_to_list_pos_[last_idx] = pos;
        }
        
        unknown_voxels_list_.pop_back();
        unknown_to_list_pos_.erase(it);
        unknown_count--;
    }
    
    // ==================== OCC_UNOBSERVED 体素操作 (语义目标提取专用) ====================
    
    /**
     * @brief 添加未观测占据体素（O(1)）
     * 当 LiDAR 发现新的 OCCUPIED 体素且尚未被相机观测时调用
     */
    void addOccUnobserved(const VoxelIndex& idx) {
        if (occ_unobserved_map_.count(idx)) return;
        
        occ_unobserved_list_.push_back(idx);
        occ_unobserved_map_[idx] = static_cast<int>(occ_unobserved_list_.size() - 1);
        
        // 状态转换：首次发现体素时进入 ACCUMULATING
        if (semantic_status_ == SemanticStatus::UNKNOWN) {
            semantic_status_ = SemanticStatus::ACCUMULATING;
        }
    }
    
    /**
     * @brief 移除未观测占据体素（O(1) Swap-and-Pop）
     * 当体素被相机观测后调用，将其移动到 occ_observed_list_
     */
    void removeOccUnobserved(const VoxelIndex& idx) {
        auto it = occ_unobserved_map_.find(idx);
        if (it == occ_unobserved_map_.end()) return;
        
        int pos = it->second;
        int last_pos = static_cast<int>(occ_unobserved_list_.size() - 1);
        
        if (pos != last_pos) {
            VoxelIndex last_idx = occ_unobserved_list_[last_pos];
            occ_unobserved_list_[pos] = last_idx;
            occ_unobserved_map_[last_idx] = pos;
        }
        
        occ_unobserved_list_.pop_back();
        occ_unobserved_map_.erase(it);
        // 注意：移除体素不影响状态，ANALYZED 后不再变化
    }
    
    /**
     * @brief 标记体素为已观测（从未观测列表移除）
     * 当相机观测到体素时调用
     */
    void markOccObserved(const VoxelIndex& idx) {
        removeOccUnobserved(idx);
    }
    
    float getExplorationRate() const {
        if (total_voxels == 0) return 0.0f;
        int explored = free_count + occupied_count;
        return static_cast<float>(explored) / total_voxels;
    }

    float getUnknownRate() const {
        if (total_voxels == 0) return 0.0f;
        return static_cast<float>(unknown_count) / total_voxels;
    }
    
    bool should_iterate_explored() {
        return exploration_rate < 0.5f;
    }
};

// ==================== 子网格前沿信息（用于视点生成） ====================
struct SubGridFrontierInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int subgrid_id;
    Position subgrid_center;
    Position subgrid_min;
    Position subgrid_max;
    std::vector<Position> frontier_points;
};

// ==================== 6邻域偏移 ====================
const VoxelIndex neighbor_offsets_6[6] = {
    VoxelIndex(1, 0, 0), VoxelIndex(-1, 0, 0),
    VoxelIndex(0, 1, 0), VoxelIndex(0, -1, 0),
    VoxelIndex(0, 0, 1), VoxelIndex(0, 0, -1)
};

// ==================== 26邻域偏移 ====================
const VoxelIndex neighbor_offsets_26[26] = {
    // 6 面邻居
    VoxelIndex(1, 0, 0), VoxelIndex(-1, 0, 0),
    VoxelIndex(0, 1, 0), VoxelIndex(0, -1, 0),
    VoxelIndex(0, 0, 1), VoxelIndex(0, 0, -1),
    // 12 边邻居
    VoxelIndex(1, 1, 0), VoxelIndex(1, -1, 0),
    VoxelIndex(-1, 1, 0), VoxelIndex(-1, -1, 0),
    VoxelIndex(1, 0, 1), VoxelIndex(1, 0, -1),
    VoxelIndex(-1, 0, 1), VoxelIndex(-1, 0, -1),
    VoxelIndex(0, 1, 1), VoxelIndex(0, 1, -1),
    VoxelIndex(0, -1, 1), VoxelIndex(0, -1, -1),
    // 8 角邻居
    VoxelIndex(1, 1, 1), VoxelIndex(1, 1, -1),
    VoxelIndex(1, -1, 1), VoxelIndex(1, -1, -1),
    VoxelIndex(-1, 1, 1), VoxelIndex(-1, 1, -1),
    VoxelIndex(-1, -1, 1), VoxelIndex(-1, -1, -1)
};

// ==================== 膨胀地图配置 ====================
struct InflationConfig {
    float inflation_radius = 0.3f;      // 膨胀半径 (m)
    float inflation_resolution = 0.1f;  // 膨胀地图分辨率
    int inflation_grid_radius = 3;      // 膨胀格子数
    bool enable_inflation = true;
};

// ==================== ESDF配置 ====================
struct ESDFConfig {
    bool enable_esdf = false;
    float esdf_resolution = 0.1f;
    float esdf_max_distance = 3.0f;
    float local_update_range_x = 10.0f;
    float local_update_range_y = 10.0f;
    float local_update_range_z = 5.0f;
};

// ==================== 机器人状态 ====================
struct RobotState {
    Position position = Position::Zero();
    Position velocity = Position::Zero();
    Quaternion orientation = Quaternion::Identity();
    float yaw = 0.0f;
    double timestamp = 0.0;
};

// ==================== 网格类型（兼容旧代码）====================
enum class GridType {
    UNKNOWN = 0,
    KNOWN_FREE = 1,
    OCCUPIED = 2,
    OUT_OF_MAP = 3
};

}  // namespace star_mvf

#endif  // STAR_MVF_MAP_TYPES_H

