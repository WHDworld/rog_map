/**
 * @file global_map.cpp
 * @brief STAR-MVF 全局地图实现
 */

#include "map/global_map.h"
#include <cmath>

// 性能监控
#include "utils/perf_logger.hpp"

namespace star_mvf {

// ==================== 构造与初始化 ====================

GlobalMap::GlobalMap(const ros::NodeHandle& nh, const std::string& name_space)
    : nh_(nh) {
    initFromROS(name_space);
}

void GlobalMap::initFromROS(const std::string& name_space) {
    // 分辨率
    nh_.param(name_space + "/resolution", config_.resolution, 0.2f);
    config_.resolution_inv = 1.0f / config_.resolution;
    
    // 地图大小
    std::vector<float> map_size;
    nh_.param(name_space + "/global_map/map_size", map_size, std::vector<float>{200.0f, 200.0f, 5.0f});
    if (map_size.size() != 3) {
        ROS_ERROR("[GlobalMap] map_size dimension is not 3!");
        map_size = {200.0f, 200.0f, 5.0f};
    }
    
    int map_model;
    float ground_height;
    nh_.param(name_space + "/global_map/map_model", map_model, 0);
    nh_.param(name_space + "/global_map/ground_height", ground_height, -0.1f);
    config_.ground_height = ground_height;  // 保存到配置，用于语义分析过滤
    
    // 根据map_model计算地图边界
    if (map_model == 0) {  // center
        config_.global_min = Position(-map_size[0]/2, -map_size[1]/2, ground_height);
        config_.global_max = Position(map_size[0]/2, map_size[1]/2, ground_height + map_size[2]);
    } else if (map_model == 1) {  // right_bottom
        config_.global_min = Position(0.0f, -map_size[1], ground_height);
        config_.global_max = Position(map_size[0], 0.0f, ground_height + map_size[2]);
    } else if (map_model == 2) {  // right_middle
        config_.global_min = Position(0.0f, -map_size[1]/2, ground_height);
        config_.global_max = Position(map_size[0], map_size[1]/2, ground_height + map_size[2]);
    } else {  // left_bottom or default
        config_.global_min = Position(0.0f, 0.0f, ground_height);
        config_.global_max = Position(map_size[0], map_size[1], ground_height + map_size[2]);
    }
    
    posToIndex(config_.global_min, global_min_idx_);
    posToIndex(config_.global_max, global_max_idx_);
    ROS_INFO("[GlobalMap] Global Map Size: [%f, %f, %f] - [%f, %f, %f]", config_.global_max.x(), config_.global_max.y(), config_.global_max.z(), config_.global_min.x(), config_.global_min.y(), config_.global_min.z());
    // 子网格配置
    nh_.param(name_space + "/global_map/sub_grid_xy", config_.sub_grid_xy, 0.1f);
    sub_grid_x_ = (config_.global_max.x() - config_.global_min.x()) * config_.sub_grid_xy;
    sub_grid_y_ = (config_.global_max.y() - config_.global_min.y()) * config_.sub_grid_xy;
    sub_grid_size_x_ = static_cast<int>(sub_grid_x_ * config_.resolution_inv);
    sub_grid_size_y_ = static_cast<int>(sub_grid_y_ * config_.resolution_inv);
    
    // 相机参数
    nh_.param(name_space + "/sensor/camera_fov_h_deg", config_.fov_h_deg, 90.0f);
    nh_.param(name_space + "/sensor/camera_fov_v_deg", config_.fov_v_deg, 60.0f);
    nh_.param(name_space + "/sensor/camera_max_range", config_.camera_max_dist, 5.0f);
    nh_.param(name_space + "/sensor/camera_min_range", config_.camera_min_dist, 0.3f);
    
    constexpr float DEG_TO_RAD = M_PI / 180.0f;
    fov_h_rad_ = config_.fov_h_deg * DEG_TO_RAD;
    fov_v_rad_ = config_.fov_v_deg * DEG_TO_RAD;
    tan_half_fov_h_ = std::tan(fov_h_rad_ / 2.0f);
    tan_half_fov_v_ = std::tan(fov_v_rad_ / 2.0f);
    
    // 前沿参数
    nh_.param(name_space + "/global_map/frontier_height_min", config_.frontier_height_min, 0.9f);
    nh_.param(name_space + "/global_map/frontier_height_max", config_.frontier_height_max, 2.3f);
    
    // 膨胀配置
    nh_.param(name_space + "/inflation/enable", config_.inflation_config.enable_inflation, true);
    int inflation_step;
    nh_.param(name_space + "/inflation/inflation_step", inflation_step, 2);
    config_.inflation_config.inflation_grid_radius = inflation_step;
    config_.inflation_config.inflation_radius = config_.resolution * inflation_step;
    
    ROS_INFO("[GlobalMap] Inflation config: enable=%d, radius=%d grids (%.2fm)",
             config_.inflation_config.enable_inflation,
             config_.inflation_config.inflation_grid_radius,
             config_.inflation_config.inflation_radius);
    
    // 初始化子网格
    initializeSubGrids();
    
    // 初始化膨胀偏移量
    if (config_.inflation_config.enable_inflation) {
        initInflationOffsets();
    }
    
    // 初始化可视化发布器（话题名与 rviz 配置一致）
    global_map_occupied_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/occupied", 1);
    global_map_free_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/free", 1);
    global_map_frontier_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/frontier", 1);
    global_map_range_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("global_map/range", 1);
    global_map_voxels_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/voxels", 1);
    visited_voxels_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/visited_voxels", 1);
    sub_grid_range_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("global_map/sub_grid_range", 1);
    camera_frustum_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("global_map/camera_frustum", 1);
    
    ROS_INFO("[GlobalMap] Initialized: resolution=%.3f, range=[(%.1f,%.1f,%.1f), (%.1f,%.1f,%.1f)]",
             config_.resolution, 
             config_.global_min.x(), config_.global_min.y(), config_.global_min.z(),
             config_.global_max.x(), config_.global_max.y(), config_.global_max.z());
}

// ==================== 子网格初始化核心逻辑 ====================

void GlobalMap::initializeSubGrids() {
    std::lock_guard<std::mutex> lock(map_mutex_);

    // 1. 计算全局体素索引范围 (Global Voxel Range)
    // global_min_idx 通常为 (0,0,0)
    // global_max_idx 是开区间边界 (Total Size)
    VoxelIndex global_min_idx, global_max_idx;
    posToIndex(config_.global_min, global_min_idx);
    posToIndex(config_.global_max, global_max_idx);

    // 计算总体素数量
    int total_voxels_x = static_cast<int>(global_max_idx.x - global_min_idx.x);
    int total_voxels_y = static_cast<int>(global_max_idx.y - global_min_idx.y);
    int total_voxels_z = static_cast<int>(global_max_idx.z - global_min_idx.z);

    // 2. 计算子网格行列数量 (基于比例 sub_grid_xy)
    // 例如 ratio=0.1 -> num=10
    sub_grid_num_x_ = static_cast<int>(std::round(1.0f / config_.sub_grid_xy));
    sub_grid_num_y_ = static_cast<int>(std::round(1.0f / config_.sub_grid_xy));
    
    // 安全检查
    if (sub_grid_num_x_ < 1) sub_grid_num_x_ = 1;
    if (sub_grid_num_y_ < 1) sub_grid_num_y_ = 1;

    // 3. 计算每个子网格分配的体素步长 (Integer Division)
    // 这确保了我们是基于整数索引切分的，绝对没有缝隙
    int voxels_per_sub_x = total_voxels_x / sub_grid_num_x_;
    int voxels_per_sub_y = total_voxels_y / sub_grid_num_y_;

    int total_subgrids = sub_grid_num_x_ * sub_grid_num_y_;
    sub_grids_.clear();
    sub_grids_.resize(total_subgrids);

    ROS_INFO("[GlobalMap] Map Voxels: [%d x %d x %d]", total_voxels_x, total_voxels_y, total_voxels_z);
    ROS_INFO("[GlobalMap] SubGrids: %d x %d (Base Size: %d x %d voxels)", 
             sub_grid_num_x_, sub_grid_num_y_, voxels_per_sub_x, voxels_per_sub_y);

    // 4. 遍历并初始化每个子网格
    for (int y = 0; y < sub_grid_num_y_; y++) {
        for (int x = 0; x < sub_grid_num_x_; x++) {
            int linear_idx = y * sub_grid_num_x_ + x;
            SubGrid& sg = sub_grids_[linear_idx];
            
            // ========== A. 设定索引范围 (Index Bounds) ==========
            sg.grid_idx = SubGridIndex(x, y);

            // 计算 X 轴起止偏移 (Offset)
            int start_offset_x = x * voxels_per_sub_x;
            int end_offset_x = (x + 1) * voxels_per_sub_x;
            // 修正：最后一列要把剩余的余数全部吃掉，直接对齐到 total_voxels_x
            if (x == sub_grid_num_x_ - 1) end_offset_x = total_voxels_x;

            // 计算 Y 轴起止偏移
            int start_offset_y = y * voxels_per_sub_y;
            int end_offset_y = (y + 1) * voxels_per_sub_y;
            if (y == sub_grid_num_y_ - 1) end_offset_y = total_voxels_y;

            // 设置子网格的全局索引范围 (闭区间 [min, max])
            sg.global_min_idx.x = global_min_idx.x + start_offset_x;
            sg.global_max_idx.x = global_min_idx.x + end_offset_x - 1;

            sg.global_min_idx.y = global_min_idx.y + start_offset_y;
            sg.global_max_idx.y = global_min_idx.y + end_offset_y - 1;

            // Z 轴通常不切分，直接继承全局高度范围
            sg.global_min_idx.z = global_min_idx.z;
            sg.global_max_idx.z = global_max_idx.z - 1; // max_idx 是开区间，所以减 1 变闭区间

            // ========== B. 反向计算物理边界 (Snap to Grid) ==========
            // 物理 Min = min_idx * resolution (左边缘)
            // 物理 Max = (max_idx + 1) * resolution (右边缘)
            // 这样计算出的 world_min/max 是完美贴合体素网格线的
            
            sg.world_min.x() = config_.global_min.x() + (sg.global_min_idx.x - global_min_idx.x) * config_.resolution;
            sg.world_min.y() = config_.global_min.y() + (sg.global_min_idx.y - global_min_idx.y) * config_.resolution;
            sg.world_min.z() = config_.global_min.z(); // Z轴同理

            sg.world_max.x() = config_.global_min.x() + (sg.global_max_idx.x - global_min_idx.x + 1) * config_.resolution;
            sg.world_max.y() = config_.global_min.y() + (sg.global_max_idx.y - global_min_idx.y + 1) * config_.resolution;
            sg.world_max.z() = config_.global_max.z(); // global_max 已经是对齐的

            sg.center = (sg.world_min + sg.world_max) * 0.5f;

            // ========== C. 统计与内存初始化 ==========
            // 计算该网格内的体素总数 (闭区间长度 = max - min + 1)
            int count_x = static_cast<int>(sg.global_max_idx.x - sg.global_min_idx.x + 1);
            int count_y = static_cast<int>(sg.global_max_idx.y - sg.global_min_idx.y + 1);
            int count_z = static_cast<int>(sg.global_max_idx.z - sg.global_min_idx.z + 1);
            
            sg.total_voxels = count_x * count_y * count_z;
            
            // 初始状态：全部未知
            sg.unknown_count = sg.total_voxels;
            sg.occupied_count = 0;
            sg.free_count = 0;
            sg.exploration_rate = 0.0f;
            sg.unknown_rate = 1.0f;

            sg.has_frontier = false;
            sg.frontier_count = 0;
            sg.frontier_dirty = true;
            sg.was_frontier_last_time = false;

            // 清空并预留容器
            sg.free_voxels_list_.clear();
            sg.free_voxels_list_.reserve(sg.total_voxels / 20); // 预留 5%
            sg.free_to_list_pos_.clear();
            sg.sub_grid_frontier_voxels_.clear();

            // ========== D. 全量填充 UNKNOWN 列表 (关键修改) ==========
            // 直接将所有体素塞入 unknown 列表，避免懒惰加载的复杂性
            sg.unknown_voxels_list_.clear();
            sg.unknown_voxels_list_.reserve(sg.total_voxels);
            sg.unknown_to_list_pos_.clear();
            sg.unknown_to_list_pos_.reserve(sg.total_voxels);

            // 三层循环遍历该子网格范围内的所有体素索引
            // 注意：这里使用的是闭区间 [min, max]
            for (int k = sg.global_min_idx.z; k <= sg.global_max_idx.z; ++k) {
                for (int j = sg.global_min_idx.y; j <= sg.global_max_idx.y; ++j) {
                    for (int i = sg.global_min_idx.x; i <= sg.global_max_idx.x; ++i) {
                        VoxelIndex idx(i, j, k);
                        
                        // 存入列表
                        sg.unknown_voxels_list_.push_back(idx);
                        // 建立映射：Index -> Vector下标
                        // 此时下标就是 size - 1
                        sg.unknown_to_list_pos_[idx] = static_cast<int>(sg.unknown_voxels_list_.size() - 1);
                    }
                }
            }        
        }
    }

    ROS_INFO("[GlobalMap] Initialization Done. Created %d subgrids.", total_subgrids);
}

// ==================== 基础查询 ====================

bool GlobalMap::isInGlobalMapRange(const Position& pos) const {
    return pos.x() >= config_.global_min.x() && pos.x() < config_.global_max.x() &&
           pos.y() >= config_.global_min.y() && pos.y() < config_.global_max.y() &&
           pos.z() >= config_.global_min.z() && pos.z() < config_.global_max.z();
}

bool GlobalMap::isInGlobalMapRange(const VoxelIndex& idx) const {
    return idx.x >= global_min_idx_.x && idx.x <= global_max_idx_.x &&
           idx.y >= global_min_idx_.y && idx.y <= global_max_idx_.y &&
           idx.z >= global_min_idx_.z && idx.z <= global_max_idx_.z;
}

void GlobalMap::posToIndex(const Position& pos, VoxelIndex& idx) const {
    idx.x = static_cast<int64_t>(std::floor((pos.x() - config_.global_min.x()) * config_.resolution_inv));
    idx.y = static_cast<int64_t>(std::floor((pos.y() - config_.global_min.y()) * config_.resolution_inv));
    idx.z = static_cast<int64_t>(std::floor((pos.z() - config_.global_min.z()) * config_.resolution_inv));
}

VoxelIndex GlobalMap::posToIndex(const Position& pos) const {
    VoxelIndex idx;
    posToIndex(pos, idx);
    return idx;
}

void GlobalMap::indexToPos(const VoxelIndex& idx, Position& pos) const {
    pos.x() = (idx.x + 0.5f) * config_.resolution + config_.global_min.x();
    pos.y() = (idx.y + 0.5f) * config_.resolution + config_.global_min.y();
    pos.z() = (idx.z + 0.5f) * config_.resolution + config_.global_min.z();
}

Position GlobalMap::indexToPos(const VoxelIndex& idx) const {
    Position pos;
    indexToPos(idx, pos);
    return pos;
}

VoxelState GlobalMap::getVoxelState(const Position& pos) {
    VoxelIndex idx = posToIndex(pos);
    return getVoxelState(idx);
}

VoxelState GlobalMap::getVoxelState(const VoxelIndex& idx) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = voxels_.find(idx);
    if (it == voxels_.end()) {
        return VoxelStateBits::GEO_UNKNOWN;
    }
    return it->second;
}

bool GlobalMap::isOccupied(const Position& pos) {
    return VoxelStateBits::isOccupied(getVoxelState(pos));
}

bool GlobalMap::isOccupied(const VoxelIndex& idx) {
    return VoxelStateBits::isOccupied(getVoxelState(idx));
}

bool GlobalMap::isFree(const Position& pos) {
    return VoxelStateBits::isFree(getVoxelState(pos));
}

bool GlobalMap::isFree(const VoxelIndex& idx) {
    return VoxelStateBits::isFree(getVoxelState(idx));
}

bool GlobalMap::isUnknown(const Position& pos) {
    return VoxelStateBits::isUnknown(getVoxelState(pos));
}

bool GlobalMap::isUnknown(const VoxelIndex& idx) {
    return VoxelStateBits::isUnknown(getVoxelState(idx));
}

bool GlobalMap::isFrontier(const Position& pos) {
    VoxelIndex idx = posToIndex(pos);
    return isFrontier(idx);
}

bool GlobalMap::isFrontier(const VoxelIndex& idx) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return frontier_voxels_.count(idx) > 0;
}

// ==================== 状态更新 ====================
void GlobalMap::updateVoxelStateFromLiDAR(const VoxelIndex& idx, bool is_occupied) {
    if (!isInGlobalMapRange(idx)) return;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    VoxelState& state = voxels_[idx];
    VoxelState old_geo = VoxelStateBits::getGeoState(state);
    VoxelState new_geo = is_occupied ? VoxelStateBits::GEO_OCCUPIED : VoxelStateBits::GEO_FREE;
    
    // 如果状态未改变，无需更新
    if (old_geo == new_geo) return;
    
    VoxelStateBits::setGeoState(state, new_geo);
    
    // 标记子网格脏
    SubGridIndex sg_idx;
    voxelToSubGrid(idx, sg_idx);
    int linear_idx = subGridToLinearIndex(sg_idx);
    if (linear_idx >= 0 && linear_idx < static_cast<int>(sub_grids_.size())) {
        SubGrid& sg = sub_grids_[linear_idx];
        dirty_subgrids_.insert(linear_idx);
        sg.frontier_dirty = true;
        
        // 更新子网格体素列表
        // 处理旧状态
        if (old_geo == VoxelStateBits::GEO_UNKNOWN) {
            sg.removeUnknownVoxel(idx);  // 内部已执行 unknown_count--
        } else if (old_geo == VoxelStateBits::GEO_FREE) {
            sg.removeFreeVoxel(idx);  // 内部已执行 free_count--
        } else if (old_geo == VoxelStateBits::GEO_OCCUPIED) {
            sg.occupied_count--;  // OCCUPIED需要手动更新
        }
        
        // 处理新状态
        if (new_geo == VoxelStateBits::GEO_FREE) {
            sg.addFreeVoxel(idx);  // 内部已执行 free_count++
        } else if (new_geo == VoxelStateBits::GEO_OCCUPIED) {
            sg.occupied_count++;  // OCCUPIED需要手动更新
        }
        // UNKNOWN状态不需要添加到列表（默认就是UNKNOWN）
    }
}

// ==================== 增量更新接口实现 ====================

int GlobalMap::getAndClearFrontierChangedSubGrids(std::vector<int>& out_changed_ids) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    out_changed_ids.clear();
    out_changed_ids.reserve(frontier_changed_subgrids_.size());
    
    for (int sg_id : frontier_changed_subgrids_) {
        out_changed_ids.push_back(sg_id);
    }
    
    int count = static_cast<int>(frontier_changed_subgrids_.size());
    frontier_changed_subgrids_.clear();  // 由 ViewpointManager 清空
    
    return count;
}

void GlobalMap::updateVoxelStateFromCamera(const VoxelIndex& encoded_pos, bool is_visited) {
    // 方案B修正：解码世界坐标（除以 10000）
    Position pos;
    pos.x() = static_cast<float>(encoded_pos.x) / 10000.0f;
    pos.y() = static_cast<float>(encoded_pos.y) / 10000.0f;
    pos.z() = static_cast<float>(encoded_pos.z) / 10000.0f;
    
    // 使用 GlobalMap 的 posToIndex 正确计算全局索引
    VoxelIndex idx;
    posToIndex(pos, idx);
    
    if (!isInGlobalMapRange(idx)) return;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    auto it = voxels_.find(idx);
    if (it != voxels_.end()) {
        VoxelState& state = it->second;
        bool was_visited = VoxelStateBits::isVisited(state);
        VoxelStateBits::setVisited(state, is_visited);
        
        // 如果是 OCCUPIED 体素且刚被标记为 VISITED，从未观测列表移除
        if (is_visited && !was_visited && VoxelStateBits::isOccupied(state)) {
            SubGridIndex sg_idx;
            voxelToSubGrid(idx, sg_idx);
            int linear_idx = subGridToLinearIndex(sg_idx);
            if (linear_idx >= 0 && linear_idx < static_cast<int>(sub_grids_.size())) {
                sub_grids_[linear_idx].markOccObserved(idx);
            }
        }
    }
}

void GlobalMap::updateOccupiedVoxels(const std::vector<VoxelIndex>& occupied_indices,
    const std::vector<VoxelIndex>& free_indices) {
    
    // 性能监控：仅当有大量更新时记录
    auto t_start = ros::WallTime::now();
    
    // 收集需要更新膨胀的体素
    std::vector<std::pair<VoxelIndex, bool>> inflation_updates;
    
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        // 方案B修正：传入的是用 10000 倍编码的世界坐标（0.1mm 精度）
        // 解码后使用 GlobalMap 的 posToIndex 计算正确的全局索引

        for (const auto& encoded_pos : occupied_indices) 
        {
            // 解码世界坐标（除以 10000）
            Position pos;
            pos.x() = static_cast<float>(encoded_pos.x) / 10000.0f;
            pos.y() = static_cast<float>(encoded_pos.y) / 10000.0f;
            pos.z() = static_cast<float>(encoded_pos.z) / 10000.0f;
            
            // 使用 GlobalMap 的 posToIndex 正确计算全局索引
            VoxelIndex idx;
            posToIndex(pos, idx);
            
            if (!isInGlobalMapRange(idx)) continue;

            VoxelState& state = voxels_[idx];
            VoxelState old_geo = VoxelStateBits::getGeoState(state);

            if (old_geo == VoxelStateBits::GEO_OCCUPIED) continue;  // 已经是OCCUPIED

            VoxelStateBits::setGeoState(state, VoxelStateBits::GEO_OCCUPIED);
            
            // 记录需要更新膨胀的体素（新增为占据）
            inflation_updates.emplace_back(idx, true);

            SubGridIndex sg_idx;
            voxelToSubGrid(idx, sg_idx);
            int linear_idx = subGridToLinearIndex(sg_idx);
            if (linear_idx >= 0 && linear_idx < static_cast<int>(sub_grids_.size())) {
                SubGrid& sg = sub_grids_[linear_idx];
                dirty_subgrids_.insert(linear_idx);
                sg.frontier_dirty = true;

                // 更新体素列表
                if (old_geo == VoxelStateBits::GEO_UNKNOWN) {
                    sg.removeUnknownVoxel(idx);
                } else if (old_geo == VoxelStateBits::GEO_FREE) {
                    sg.removeFreeVoxel(idx);
                }
                sg.occupied_count++;
                
                // 新增 OCCUPIED 体素默认为未观测状态
                // 过滤地面噪声：只有高于 ground_height + 0.2m 的体素才参与语义分析
                Position voxel_pos = indexToPos(idx);
                if (voxel_pos.z() > config_.ground_height + 0.2f) {
                    sg.addOccUnobserved(idx);
                }
                
                // 检查探索率是否达标，加入待分析索引
                using Status = SubGrid::SemanticStatus;
                if (sg.semantic_status_ == Status::ACCUMULATING &&
                    sg.getExplorationRate() > semantic_exploration_threshold_) {
                    ready_for_analysis_subgrids_.insert(linear_idx);
                }
            }
        }

        for (const auto& encoded_pos : free_indices) 
        {
            // 解码世界坐标（除以 10000）
            Position pos;
            pos.x() = static_cast<float>(encoded_pos.x) / 10000.0f;
            pos.y() = static_cast<float>(encoded_pos.y) / 10000.0f;
            pos.z() = static_cast<float>(encoded_pos.z) / 10000.0f;
            
            // 使用 GlobalMap 的 posToIndex 正确计算全局索引
            VoxelIndex idx;
            posToIndex(pos, idx);
            
            if (!isInGlobalMapRange(idx)) continue;

            VoxelState& state = voxels_[idx];
            VoxelState old_geo = VoxelStateBits::getGeoState(state);

            if (old_geo == VoxelStateBits::GEO_FREE) continue;  // 已经是FREE

            // 如果之前是占据，需要移除膨胀
            if (old_geo == VoxelStateBits::GEO_OCCUPIED) {
                inflation_updates.emplace_back(idx, false);
            }

            VoxelStateBits::setGeoState(state, VoxelStateBits::GEO_FREE);

            SubGridIndex sg_idx;
            voxelToSubGrid(idx, sg_idx);

            int linear_idx = subGridToLinearIndex(sg_idx);
            if (linear_idx >= 0 && linear_idx < static_cast<int>(sub_grids_.size())) {
                SubGrid& sg = sub_grids_[linear_idx];
                dirty_subgrids_.insert(linear_idx);
                sg.frontier_dirty = true;

                // 更新体素列表
                if (old_geo == VoxelStateBits::GEO_UNKNOWN) {
                    sg.removeUnknownVoxel(idx);
                } else if (old_geo == VoxelStateBits::GEO_OCCUPIED) {
                    sg.occupied_count--;
                }
                sg.addFreeVoxel(idx);  // 添加到FREE列表
            }
        }
    }
    
    // 在 map_mutex_ 锁外更新膨胀（使用独立的 inflation_mutex_）
    for (const auto& update : inflation_updates) {
        updateInflation(update.first, update.second);
    }
    
    // 性能监控：记录更新统计（仅当有更新时）
    double elapsed_ms = (ros::WallTime::now() - t_start).toSec() * 1000.0;
    if (!occupied_indices.empty() || !free_indices.empty()) {
        // 限制日志频率，避免刷屏
        static int log_counter = 0;
        if (++log_counter % 10 == 0 && PERF_LOG.isInitialized()) {
            PERF_LOG.logf("MAP", "UPDATE_VOXELS", "occ=%lu, free=%lu, inflate=%lu, time=%.2fms",
                          occupied_indices.size(), free_indices.size(), 
                          inflation_updates.size(), elapsed_ms);
        }
    }
}

// ==================== 子网格操作 ====================

void GlobalMap::posToSubGrid(const Position& pos, SubGridIndex& sg_idx) const {
    sg_idx.x = static_cast<int>((pos.x() - config_.global_min.x()) / sub_grid_x_);
    sg_idx.y = static_cast<int>((pos.y() - config_.global_min.y()) / sub_grid_y_);
    sg_idx.x = std::max(0, std::min(sg_idx.x, sub_grid_num_x_ - 1));
    sg_idx.y = std::max(0, std::min(sg_idx.y, sub_grid_num_y_ - 1));
}

void GlobalMap::voxelToSubGrid(const VoxelIndex& voxel_idx, SubGridIndex& sg_idx) const {
    Position pos = indexToPos(voxel_idx);
    posToSubGrid(pos, sg_idx);
}

SubGrid& GlobalMap::getSubGrid(const SubGridIndex& idx) {
    int linear_idx = subGridToLinearIndex(idx);
    return sub_grids_[linear_idx];
}

SubGrid& GlobalMap::getSubGrid(int linear_idx) {
    return sub_grids_[linear_idx];
}

const SubGrid& GlobalMap::getSubGrid(const SubGridIndex& idx) const {
    int linear_idx = subGridToLinearIndex(idx);
    return sub_grids_[linear_idx];
}

const SubGrid& GlobalMap::getSubGrid(int linear_idx) const {
    return sub_grids_[linear_idx];
}

int GlobalMap::subGridToLinearIndex(const SubGridIndex& idx) const {
    return idx.y * sub_grid_num_x_ + idx.x;
}

void GlobalMap::markSubGridDirty(int linear_idx) {
    if (linear_idx >= 0 && linear_idx < static_cast<int>(sub_grids_.size())) {
        dirty_subgrids_.insert(linear_idx);
        sub_grids_[linear_idx].frontier_dirty = true;
    }
}

void GlobalMap::getNeighborSubGrids4(const SubGridIndex& idx, std::vector<SubGridIndex>& neighbors) {
    neighbors.clear();
    const int dx[] = {1, -1, 0, 0};
    const int dy[] = {0, 0, 1, -1};
    
    for (int i = 0; i < 4; i++) {
        int nx = idx.x + dx[i];
        int ny = idx.y + dy[i];
        if (nx >= 0 && nx < sub_grid_num_x_ && ny >= 0 && ny < sub_grid_num_y_) {
            neighbors.push_back(SubGridIndex(nx, ny));
        }
    }
}

void GlobalMap::extractGlobalFrontierIncremental() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 只处理脏子网格
    for (int sg_id : dirty_subgrids_) {
        if (sg_id < 0 || sg_id >= static_cast<int>(sub_grids_.size())) continue;
        
        SubGrid& sg = sub_grids_[sg_id];
        if (!sg.frontier_dirty) continue;
        
        // 1. 清除旧前沿 (逻辑不变)
        for (const auto& f_idx : sg.sub_grid_frontier_voxels_) {
            frontier_voxels_.erase(f_idx);
        }
        sg.sub_grid_frontier_voxels_.clear();

        // 获取当前探索率
        float exploration_rate = sg.getExplorationRate();
        // ==================== 策略分支 ====================
        
        if (exploration_rate < 0.5f) {
            // ==================== 策略 A: 正向查找 (遍历 Free -> 找 Unknown) ====================
            // 适用场景：已探索区域少，列表短，遍历 Free 更快
            
            for (const auto& free_idx : sg.free_voxels_list_) {
                // 安全检查
                auto it = voxels_.find(free_idx);
                if (it == voxels_.end() || !VoxelStateBits::isFree(it->second)) continue;

                // 检查 6 邻域
                for (int d = 0; d < 6; d++) {
                    VoxelIndex nei_idx = free_idx + neighbor_offsets_6[d];
                    
                    if (!isInGlobalMapRange(nei_idx)) continue;
                    
                    // 检查邻居是否为 Unknown
                    auto nei_it = voxels_.find(nei_idx);
                    bool is_neighbor_unknown = (nei_it == voxels_.end()) || 
                                               VoxelStateBits::isUnknown(nei_it->second);
                    
                    if (is_neighbor_unknown) {
                        // [注意] 此时目标前沿点是【邻居】(nei_idx)
                        // 所以高度检查要查 nei_idx
                        Position nei_pos = indexToPos(nei_idx);
                        if (nei_pos.z() < config_.frontier_height_min || 
                            nei_pos.z() > config_.frontier_height_max) continue;
                        
                        sg.sub_grid_frontier_voxels_.insert(nei_idx);
                        frontier_voxels_.insert(nei_idx);
                    }
                }
            }
        } 
        else {
            // ==================== 策略 B: 反向查找 (遍历 Unknown -> 找 Free) ====================
            // 适用场景：未探索区域少 (如残留的死角)，列表短，遍历 Unknown 更快
            
            for (const auto& unk_idx : sg.unknown_voxels_list_) {
                // [优化 1]：先检查自身高度
                // 因为在策略 B 中，"当前遍历的点" (unk_idx) 就是潜在的前沿点
                // 如果自身高度都不对，就不用去查 6 个邻居了，省了大量哈希查找
                Position unk_pos = indexToPos(unk_idx);
                if (unk_pos.z() < config_.frontier_height_min || 
                    unk_pos.z() > config_.frontier_height_max) continue;

                // [安全检查] 确保它确实是 Unknown (虽然列表通常是同步的，但为了稳健)
                // 如果它在 voxels_ map 里且状态不是 unknown，说明列表没同步好，跳过
                auto it = voxels_.find(unk_idx);
                if (it != voxels_.end() && !VoxelStateBits::isUnknown(it->second)) continue;

                // 检查 6 邻域，看有没有 Free
                bool has_free_neighbor = false;
                for (int d = 0; d < 6; d++) {
                    VoxelIndex nei_idx = unk_idx + neighbor_offsets_6[d];
                    
                    if (!isInGlobalMapRange(nei_idx)) continue;
                    
                    auto nei_it = voxels_.find(nei_idx);
                    // 只有在 map 中且标记为 Free 才是 Free
                    if (nei_it != voxels_.end() && VoxelStateBits::isFree(nei_it->second)) {
                        has_free_neighbor = true;
                        break; // [优化 2]：只要找到 1 个 Free 邻居，它就是前沿，无需再查其他 5 个
                    }
                }

                if (has_free_neighbor) {
                    // [注意] 此时目标前沿点是【自身】(unk_idx)
                    sg.sub_grid_frontier_voxels_.insert(unk_idx);
                    frontier_voxels_.insert(unk_idx);
                }
            }
        }
        
        // 更新统计信息
        sg.has_frontier = !sg.sub_grid_frontier_voxels_.empty();
        sg.frontier_count = static_cast<int>(sg.sub_grid_frontier_voxels_.size());
        sg.frontier_dirty = false;
        
        // [新增] 记录此子网格的前沿发生了变化（供 ViewpointManager 使用）
        frontier_changed_subgrids_.insert(sg_id);
    }
    
    dirty_subgrids_.clear();
    updateFrontierSubGridCache();
}

void GlobalMap::updateFrontierSubGridCache() {
    current_frontier_subgrids_.clear();
    for (int i = 0; i < static_cast<int>(sub_grids_.size()); i++) {
        if (sub_grids_[i].has_frontier) {
            current_frontier_subgrids_.insert(i);
        }
    }
}

void GlobalMap::getFrontierSubGrids(std::vector<int>& frontier_subgrid_ids) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    frontier_subgrid_ids.clear();
    frontier_subgrid_ids.reserve(current_frontier_subgrids_.size());
    for (int sg_id : current_frontier_subgrids_) {
        frontier_subgrid_ids.push_back(sg_id);
    }
}

void GlobalMap::getFrontierPositions(std::vector<Position>& positions) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    positions.clear();
    positions.reserve(frontier_voxels_.size());
    for (const auto& f_idx : frontier_voxels_) {
        positions.push_back(indexToPos(f_idx));
    }
}

void GlobalMap::getFrontierPositionsBySubGrid(std::vector<SubGridFrontierInfo>& subgrid_frontiers) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    subgrid_frontiers.clear();
    
    for (int sg_id : current_frontier_subgrids_) {
        if (sg_id < 0 || sg_id >= static_cast<int>(sub_grids_.size())) continue;
        
        const SubGrid& sg = sub_grids_[sg_id];
        if (sg.sub_grid_frontier_voxels_.empty()) continue;
        
        SubGridFrontierInfo info;
        info.subgrid_id = sg_id;
        info.subgrid_center = sg.center;
        info.subgrid_min = sg.world_min;
        info.subgrid_max = sg.world_max;
        info.frontier_points.reserve(sg.sub_grid_frontier_voxels_.size());
        
        for (const auto& f_idx : sg.sub_grid_frontier_voxels_) {
            info.frontier_points.push_back(indexToPos(f_idx));
        }
        
        subgrid_frontiers.push_back(info);
    }
}

bool GlobalMap::getSubGridFrontierPoints(int sg_id, std::vector<Eigen::Vector3f>& out_points) const {
    std::lock_guard<std::mutex> lock(map_mutex_); // 必须加锁，防止并行读写冲突
    out_points.clear();

    // 1. 边界检查
    if (sg_id < 0 || sg_id >= static_cast<int>(sub_grids_.size())) {
        return false;
    }

    const SubGrid& sg = sub_grids_[sg_id];

    // 2. 检查该子网格是否活跃（有前沿点）
    if (sg.sub_grid_frontier_voxels_.empty()) {
        return false; // 返回 false 方便外部判断是否需要 releaseSubGrid
    }

    // 3. 提取点并转换坐标
    out_points.reserve(sg.sub_grid_frontier_voxels_.size());
    for (const auto& f_idx : sg.sub_grid_frontier_voxels_) 
    {
        // indexToPos 返回的是 Position 类型，根据你的定义可能是 Eigen::Vector3d 或 Vector3f
        // 这里显式转为 float 以匹配 ViewpointManager 的需求
        out_points.push_back(indexToPos(f_idx).cast<float>()); 
    }
    return true;
}

void GlobalMap::getFrontierPointCloud(sensor_msgs::PointCloud2& pc_frontier) const {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    std::lock_guard<std::mutex> lock(map_mutex_);
    pcl_cloud.reserve(frontier_voxels_.size());
    for (const auto& f_idx : frontier_voxels_) 
    {
        Position pos = indexToPos(f_idx);
        pcl_cloud.push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
    }
    pcl::toROSMsg(pcl_cloud, pc_frontier);
    pc_frontier.header.frame_id = config_.frame_id;
    pc_frontier.header.stamp = ros::Time::now();
}

size_t GlobalMap::getFrontierCount() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return frontier_voxels_.size();
}

// ==================== 安全性检查 ====================

bool GlobalMap::isPathSafe(const Position& start, const Position& end, float safety_margin) const {
    Position dir = end - start;
    float length = dir.norm();
    if (length < 1e-6f) return true;
    
    dir.normalize();
    float step = config_.resolution;
    int num_steps = static_cast<int>(length / step) + 1;
    
    for (int i = 0; i <= num_steps; i++) {
        Position pos = start + dir * (i * step);
        
        // 检查当前位置
        VoxelIndex idx;
        const_cast<GlobalMap*>(this)->posToIndex(pos, idx);
        
        std::lock_guard<std::mutex> lock(map_mutex_);
        auto it = voxels_.find(idx);
        if (it != voxels_.end() && VoxelStateBits::isOccupied(it->second)) {
            return false;
        }
        
        // 检查安全余量
        if (safety_margin > 0.0f) {
            int margin_voxels = static_cast<int>(std::ceil(safety_margin * config_.resolution_inv));
            for (int dx = -margin_voxels; dx <= margin_voxels; dx++) {
                for (int dy = -margin_voxels; dy <= margin_voxels; dy++) {
                    for (int dz = -margin_voxels; dz <= margin_voxels; dz++) {
                        VoxelIndex check_idx(idx.x + dx, idx.y + dy, idx.z + dz);
                        auto check_it = voxels_.find(check_idx);
                        if (check_it != voxels_.end() && VoxelStateBits::isOccupied(check_it->second)) {
                            return false;
                        }
                    }
                }
            }
        }
    }
    
    return true;
}

bool GlobalMap::isSphereSafe(const Position& center, float radius) const {
    int radius_voxels = static_cast<int>(std::ceil(radius * config_.resolution_inv));
    VoxelIndex center_idx;
    const_cast<GlobalMap*>(this)->posToIndex(center, center_idx);
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (int dx = -radius_voxels; dx <= radius_voxels; dx++) {
        for (int dy = -radius_voxels; dy <= radius_voxels; dy++) {
            for (int dz = -radius_voxels; dz <= radius_voxels; dz++) {
                if (dx*dx + dy*dy + dz*dz > radius_voxels*radius_voxels) continue;
                
                VoxelIndex check_idx(center_idx.x + dx, center_idx.y + dy, center_idx.z + dz);
                auto it = voxels_.find(check_idx);
                if (it != voxels_.end() && VoxelStateBits::isOccupied(it->second)) {
                    return false;
                }
            }
        }
    }
    
    return true;
}

// ==================== 可视化 ====================
void GlobalMap::visualize() {
    // 统一入口，调用所有可视化方法
    visualizeGlobalMap();           // 占据+空闲+已访问+前沿点云（单次遍历优化）
    visualizeGlobalMapRange();      // 全局地图范围框
    visualizeSubGridRange();        // 子网格范围
    visualizeCameraFrustum();       // 相机视锥体
}

void GlobalMap::visualizeGlobalMap() {
    // 检查是否有任何订阅者需要遍历 voxels_
    bool need_occ = global_map_occupied_pub_.getNumSubscribers() > 0;
    bool need_free = global_map_free_pub_.getNumSubscribers() > 0;
    bool need_visited = visited_voxels_pub_.getNumSubscribers() > 0;
    bool need_frontier = global_map_frontier_pub_.getNumSubscribers() > 0;
    
    // 如果没有任何订阅者，直接返回
    if (!need_occ && !need_free && !need_visited && !need_frontier) {
        return;
    }
    
    // 分类点云缓存
    pcl::PointCloud<pcl::PointXYZRGB> occ_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> free_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> visited_cloud;
    
    // 单次遍历，分类缓存（仅当需要 occ/free/visited 时遍历）
    if (need_occ || need_free || need_visited) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        for (const auto& kv : voxels_) {
            VoxelState geo_state = VoxelStateBits::getGeoState(kv.second);
            bool is_visited = VoxelStateBits::isVisited(kv.second);
            Position pos = indexToPos(kv.first);
            
            pcl::PointXYZRGB pt;
            pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
            
            if (geo_state == VoxelStateBits::GEO_OCCUPIED) {
                if (need_occ) {
                    // 颜色：已访问为绿色，未访问为红色
                    if (is_visited) {
                        pt.r = 0; pt.g = 255; pt.b = 0;
                    } else {
                        pt.r = 255; pt.g = 0; pt.b = 0;
                    }
                    occ_cloud.push_back(pt);
                }
                // 已访问的占据体素也加入 visited_cloud
                if (need_visited && is_visited) {
                    pt.r = 150; pt.g = 255; pt.b = 50;  // 黄绿色
                    visited_cloud.push_back(pt);
                }
            } else if (geo_state == VoxelStateBits::GEO_FREE) {
                if (need_free) {
                    // 颜色：已访问为浅绿色，未访问为浅蓝色
                    if (is_visited) {
                        pt.r = 100; pt.g = 255; pt.b = 100;
                    } else {
                        pt.r = 100; pt.g = 100; pt.b = 255;
                    }
                    free_cloud.push_back(pt);
                }
                // 已访问的空闲体素也加入 visited_cloud
                // if (need_visited && is_visited) {
                //     pt.r = 100; pt.g = 200; pt.b = 255;  // 浅蓝色
                //     visited_cloud.push_back(pt);
                // }
            }
        }
    }
    
    // 按需发布
    ros::Time now = ros::Time::now();
    
    if (need_occ && !occ_cloud.empty()) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(occ_cloud, msg);
        msg.header.frame_id = config_.frame_id;
        msg.header.stamp = now;
        global_map_occupied_pub_.publish(msg);
    }
    
    if (need_free && !free_cloud.empty()) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(free_cloud, msg);
        msg.header.frame_id = config_.frame_id;
        msg.header.stamp = now;
        global_map_free_pub_.publish(msg);
    }
    
    if (need_visited && !visited_cloud.empty()) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(visited_cloud, msg);
        msg.header.frame_id = config_.frame_id;
        msg.header.stamp = now;
        visited_voxels_pub_.publish(msg);
    }
    
    // 前沿点云单独处理（有自己的数据结构）
    if (need_frontier) {
        sensor_msgs::PointCloud2 frontier_msg;
        getFrontierPointCloud(frontier_msg);
        global_map_frontier_pub_.publish(frontier_msg);
    }
}

void GlobalMap::getAllKnownPointCloud(sensor_msgs::PointCloud2& pc_all_known) const {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        for (const auto& kv : voxels_) {
            if (!VoxelStateBits::isUnknown(kv.second)) {
                Position pos = indexToPos(kv.first);
                pcl_cloud.push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
            }
        }
    }
    
    pcl::toROSMsg(pcl_cloud, pc_all_known);
    pc_all_known.header.frame_id = config_.frame_id;
    pc_all_known.header.stamp = ros::Time::now();
}

bool GlobalMap::isInGlobalRange(const VoxelIndex& idx, 
                                const VoxelIndex& min_idx, 
                                const VoxelIndex& max_idx) const {
    return idx.x >= min_idx.x && idx.x <= max_idx.x &&
           idx.y >= min_idx.y && idx.y <= max_idx.y &&
           idx.z >= min_idx.z && idx.z <= max_idx.z;
}

void GlobalMap::visualizeGlobalMapRange() {
    if (global_map_range_pub_.getNumSubscribers() == 0) return;
    
    // 发布地图范围标记（完整的3D立方体线框）
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = config_.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "global_map_range";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    // 立方体的8个顶点
    Position min_pt = config_.global_min;
    Position max_pt = config_.global_max;
    
    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    p1.x = min_pt.x(); p1.y = min_pt.y(); p1.z = min_pt.z();
    p2.x = max_pt.x(); p2.y = min_pt.y(); p2.z = min_pt.z();
    p3.x = max_pt.x(); p3.y = max_pt.y(); p3.z = min_pt.z();
    p4.x = min_pt.x(); p4.y = max_pt.y(); p4.z = min_pt.z();
    p5.x = min_pt.x(); p5.y = min_pt.y(); p5.z = max_pt.z();
    p6.x = max_pt.x(); p6.y = min_pt.y(); p6.z = max_pt.z();
    p7.x = max_pt.x(); p7.y = max_pt.y(); p7.z = max_pt.z();
    p8.x = min_pt.x(); p8.y = max_pt.y(); p8.z = max_pt.z();
    
    // 底面4条边
    marker.points.push_back(p1); marker.points.push_back(p2);
    marker.points.push_back(p2); marker.points.push_back(p3);
    marker.points.push_back(p3); marker.points.push_back(p4);
    marker.points.push_back(p4); marker.points.push_back(p1);
    // 顶面4条边
    marker.points.push_back(p5); marker.points.push_back(p6);
    marker.points.push_back(p6); marker.points.push_back(p7);
    marker.points.push_back(p7); marker.points.push_back(p8);
    marker.points.push_back(p8); marker.points.push_back(p5);
    // 4条竖边
    marker.points.push_back(p1); marker.points.push_back(p5);
    marker.points.push_back(p2); marker.points.push_back(p6);
    marker.points.push_back(p3); marker.points.push_back(p7);
    marker.points.push_back(p4); marker.points.push_back(p8);
    
    markers.markers.push_back(marker);
    global_map_range_pub_.publish(markers);
}

void GlobalMap::visualizeSubGridRange() {
    if (sub_grid_range_pub_.getNumSubscribers() == 0) return;
    
    visualization_msgs::MarkerArray markers;
    
    // 首先发送删除所有旧marker的命令
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = config_.frame_id;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "sub_grid_range";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    int marker_id = 0;
    for (const auto& sg : sub_grids_) {
        // 只显示已探索的子网格（完全探索或正在探索，不包括完全未探索的）
        // 判断条件：探索率 > 0 或者有前沿（正在探索）
        float exploration_rate = sg.getExplorationRate();
        if (exploration_rate <= 0.0f && !sg.has_frontier) {
            continue;  // 跳过完全未探索的子网格
        }
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = config_.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "sub_grid_range";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.03;
        
        // 根据探索状态设置颜色
        if (sg.has_frontier) {
            // 有前沿（正在探索）：绿色
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f;
        } else if (exploration_rate > 0.8f) {
            // 几乎完全探索：青色
            marker.color.r = 0.0f;
            marker.color.g = 0.8f;
            marker.color.b = 1.0f;
            marker.color.a = 0.6f;
        } else {
            // 部分探索：黄色
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
        }
        
        // 绘制子网格边界框
        Position min_pt = sg.world_min;
        Position max_pt = sg.world_max;
        
        geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
        p1.x = min_pt.x(); p1.y = min_pt.y(); p1.z = min_pt.z();
        p2.x = max_pt.x(); p2.y = min_pt.y(); p2.z = min_pt.z();
        p3.x = max_pt.x(); p3.y = max_pt.y(); p3.z = min_pt.z();
        p4.x = min_pt.x(); p4.y = max_pt.y(); p4.z = min_pt.z();
        p5.x = min_pt.x(); p5.y = min_pt.y(); p5.z = max_pt.z();
        p6.x = max_pt.x(); p6.y = min_pt.y(); p6.z = max_pt.z();
        p7.x = max_pt.x(); p7.y = max_pt.y(); p7.z = max_pt.z();
        p8.x = min_pt.x(); p8.y = max_pt.y(); p8.z = max_pt.z();
        
        // 底面
        marker.points.push_back(p1); marker.points.push_back(p2);
        marker.points.push_back(p2); marker.points.push_back(p3);
        marker.points.push_back(p3); marker.points.push_back(p4);
        marker.points.push_back(p4); marker.points.push_back(p1);
        // 顶面
        marker.points.push_back(p5); marker.points.push_back(p6);
        marker.points.push_back(p6); marker.points.push_back(p7);
        marker.points.push_back(p7); marker.points.push_back(p8);
        marker.points.push_back(p8); marker.points.push_back(p5);
        // 四条竖边
        marker.points.push_back(p1); marker.points.push_back(p5);
        marker.points.push_back(p2); marker.points.push_back(p6);
        marker.points.push_back(p3); marker.points.push_back(p7);
        marker.points.push_back(p4); marker.points.push_back(p8);
        
        markers.markers.push_back(marker);
    }
    
    sub_grid_range_pub_.publish(markers);
}

// visualizeVisitedVoxels 已合并到 visualizeGlobalMap，删除冗余函数


void GlobalMap::visualizeCameraFrustum() {
    if (camera_frustum_pub_.getNumSubscribers() == 0) return;
    
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = config_.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "camera_frustum";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.03;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.9f;
    
    // 使用无人机位姿（current_cam_origin_ 和 current_cam_rot_ 由 MapManager 从 odom 更新）
    Position cam_pos = current_cam_origin_;
    Eigen::Matrix3f rot = current_cam_rot_.toRotationMatrix();
    
    // 使用配置的FOV参数（水平半角和垂直半角）
    float fov_h = config_.fov_h_deg * 0.5f * M_PI / 180.0f;  // 转换为弧度的半角
    float fov_v = config_.fov_v_deg * 0.5f * M_PI / 180.0f;
    float range = config_.camera_max_dist;  // 使用配置的最大感知距离
    
    float tan_h = std::tan(fov_h);
    float tan_v = std::tan(fov_v);
    
    // 远平面四个角点（相机坐标系，X轴为前向）
    Eigen::Vector3f corners_cam[4];
    corners_cam[0] = Eigen::Vector3f(range, -range * tan_h, -range * tan_v);  // 左下
    corners_cam[1] = Eigen::Vector3f(range,  range * tan_h, -range * tan_v);  // 右下
    corners_cam[2] = Eigen::Vector3f(range,  range * tan_h,  range * tan_v);  // 右上
    corners_cam[3] = Eigen::Vector3f(range, -range * tan_h,  range * tan_v);  // 左上
    
    // 转换到世界坐标系
    Eigen::Vector3f corners_world[4];
    for (int i = 0; i < 4; i++) {
        corners_world[i] = rot * corners_cam[i] + cam_pos;
    }
    
    // 绘制从相机原点到四个角的线
    geometry_msgs::Point origin;
    origin.x = cam_pos.x();
    origin.y = cam_pos.y();
    origin.z = cam_pos.z();
    
    for (int i = 0; i < 4; i++) {
        geometry_msgs::Point corner;
        corner.x = corners_world[i].x();
        corner.y = corners_world[i].y();
        corner.z = corners_world[i].z();
        marker.points.push_back(origin);
        marker.points.push_back(corner);
    }
    
    // 绘制远平面边框
    for (int i = 0; i < 4; i++) {
        geometry_msgs::Point p1, p2;
        p1.x = corners_world[i].x();
        p1.y = corners_world[i].y();
        p1.z = corners_world[i].z();
        p2.x = corners_world[(i + 1) % 4].x();
        p2.y = corners_world[(i + 1) % 4].y();
        p2.z = corners_world[(i + 1) % 4].z();
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
    
    markers.markers.push_back(marker);
    camera_frustum_pub_.publish(markers);
}

// ==================== 相机视锥相关 ====================

bool GlobalMap::isPointInFrustum(const VoxelIndex& idx) const {
    Position pos = indexToPos(idx);
    return isPointInFrustum(pos);
}

bool GlobalMap::isPointInFrustum(const Position& pos) const {
    if (!enable_camera_map_) return false;
    
    // 将点转换到相机坐标系
    Eigen::Vector3f cam_pos = current_cam_rot_.inverse() * (pos - current_cam_origin_);
    
    // 检查是否在前方
    if (cam_pos.z() < config_.camera_min_dist || cam_pos.z() > config_.camera_max_dist) {
        return false;
    }
    
    // 检查是否在视锥内
    float half_width = cam_pos.z() * tan_half_fov_h_;
    float half_height = cam_pos.z() * tan_half_fov_v_;
    
    return std::abs(cam_pos.x()) < half_width && std::abs(cam_pos.y()) < half_height;
}

void GlobalMap::updateCameraPose(const Position& origin, const Quaternion& rotation) {
    current_cam_origin_ = origin;
    current_cam_rot_ = rotation;
}

// ==================== 膨胀地图实现 ====================

void GlobalMap::initInflationOffsets() {
    inflation_neighbor_offsets_.clear();
    int r = config_.inflation_config.inflation_grid_radius;
    
    for (int dx = -r; dx <= r; dx++) {
        for (int dy = -r; dy <= r; dy++) {
            for (int dz = -r; dz <= r; dz++) {
                // 使用球形膨胀
                if (dx*dx + dy*dy + dz*dz <= r*r) {
                    inflation_neighbor_offsets_.push_back(VoxelIndex(dx, dy, dz));
                }
            }
        }
    }
    
    ROS_INFO("[GlobalMap] Initialized inflation offsets: radius=%d, neighbors=%zu", 
             r, inflation_neighbor_offsets_.size());
}

void GlobalMap::updateInflation(const VoxelIndex& idx, bool is_occupied) {
    if (!config_.inflation_config.enable_inflation) return;
    
    std::lock_guard<std::mutex> lock(inflation_mutex_);
    
    int delta = is_occupied ? 1 : -1;
    
    for (const auto& offset : inflation_neighbor_offsets_) {
        VoxelIndex nei_idx = idx + offset;
        if (!isInGlobalMapRange(nei_idx)) continue;
        
        auto it = inflation_count_.find(nei_idx);
        if (it == inflation_count_.end()) {
            if (is_occupied) {
                inflation_count_[nei_idx] = 1;
            }
        } else {
            it->second += delta;
            if (it->second <= 0) {
                inflation_count_.erase(it);
            }
        }
    }
}

bool GlobalMap::isOccupiedInflate(const Position& pos) const {
    VoxelIndex idx = posToIndex(pos);
    return isOccupiedInflate(idx);
}

bool GlobalMap::isOccupiedInflate(const VoxelIndex& idx) const {
    if (!isInGlobalMapRange(idx)) {
        return true;  // 超出范围视为占据
    }
    
    if (!config_.inflation_config.enable_inflation) {
        // 没有启用膨胀，直接检查原始占据
        std::lock_guard<std::mutex> lock(map_mutex_);
        auto it = voxels_.find(idx);
        return it != voxels_.end() && VoxelStateBits::isOccupied(it->second);
    }
    
    std::lock_guard<std::mutex> lock(inflation_mutex_);
    auto it = inflation_count_.find(idx);
    return it != inflation_count_.end() && it->second > 0;
}

bool GlobalMap::isUnknownInflate(const Position& pos) const {
    if (!isInGlobalMapRange(pos)) return false;
    
    // 检查膨胀区域内是否有未知
    VoxelIndex center_idx = posToIndex(pos);
    int r = config_.inflation_config.inflation_grid_radius;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (int dx = -r; dx <= r; dx++) {
        for (int dy = -r; dy <= r; dy++) {
            for (int dz = -r; dz <= r; dz++) {
                if (dx*dx + dy*dy + dz*dz > r*r) continue;
                
                VoxelIndex check_idx(center_idx.x + dx, center_idx.y + dy, center_idx.z + dz);
                auto it = voxels_.find(check_idx);
                if (it == voxels_.end() || VoxelStateBits::isUnknown(it->second)) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool GlobalMap::isKnownFreeInflate(const Position& pos) const {
    return !isOccupiedInflate(pos) && !isUnknownInflate(pos);
}

// ==================== ESDF实现 ====================

double GlobalMap::getESDFDistance(const Position& pos) const {
    if (!config_.esdf_config.enable_esdf) {
        // ESDF未启用，使用简化的距离计算
        return isOccupiedInflate(pos) ? -config_.resolution : config_.esdf_config.esdf_max_distance;
    }
    
    VoxelIndex idx = posToIndex(pos);
    
    std::lock_guard<std::mutex> lock(esdf_mutex_);
    auto it = esdf_distance_.find(idx);
    if (it != esdf_distance_.end()) {
        return it->second;
    }
    
    // 未计算的体素，返回最大距离或计算近似值
    return config_.esdf_config.esdf_max_distance;
}

void GlobalMap::getESDFGradient(const Position& pos, Position& gradient) const {
    gradient = Position::Zero();
    
    if (!config_.esdf_config.enable_esdf) return;
    
    // 使用数值差分计算梯度
    float eps = config_.resolution * 0.5f;
    
    gradient.x() = (getESDFDistance(pos + Position(eps, 0, 0)) - 
                    getESDFDistance(pos - Position(eps, 0, 0))) / (2.0f * eps);
    gradient.y() = (getESDFDistance(pos + Position(0, eps, 0)) - 
                    getESDFDistance(pos - Position(0, eps, 0))) / (2.0f * eps);
    gradient.z() = (getESDFDistance(pos + Position(0, 0, eps)) - 
                    getESDFDistance(pos - Position(0, 0, eps))) / (2.0f * eps);
}

void GlobalMap::updateESDF(const Position& center) {
    if (!config_.esdf_config.enable_esdf) return;
    
    esdf_center_ = center;
    
    // 计算更新范围
    Position range(config_.esdf_config.local_update_range_x,
                   config_.esdf_config.local_update_range_y,
                   config_.esdf_config.local_update_range_z);
    Position update_min = center - range * 0.5f;
    Position update_max = center + range * 0.5f;
    
    VoxelIndex min_idx = posToIndex(update_min);
    VoxelIndex max_idx = posToIndex(update_max);
    
    std::lock_guard<std::mutex> lock(esdf_mutex_);
    
    // 简化的ESDF计算：BFS波前传播
    std::queue<std::pair<VoxelIndex, double>> bfs_queue;
    std::unordered_set<VoxelIndex, VoxelIndexHash> visited;
    
    // 初始化：将所有占据体素加入队列，距离为0
    {
        std::lock_guard<std::mutex> map_lock(map_mutex_);
        for (const auto& kv : voxels_) {
            if (VoxelStateBits::isOccupied(kv.second)) {
                if (isInGlobalRange(kv.first, min_idx, max_idx)) {
                    bfs_queue.push({kv.first, 0.0});
                    esdf_distance_[kv.first] = 0.0;
                    visited.insert(kv.first);
                }
            }
        }
    }
    
    // BFS传播
    while (!bfs_queue.empty()) {
        auto [idx, dist] = bfs_queue.front();
        bfs_queue.pop();
        
        if (dist >= config_.esdf_config.esdf_max_distance) continue;
        
        for (int d = 0; d < 6; d++) {
            VoxelIndex nei_idx = idx + neighbor_offsets_6[d];
            
            if (!isInGlobalRange(nei_idx, min_idx, max_idx)) continue;
            if (visited.count(nei_idx) > 0) continue;
            
            double new_dist = dist + config_.resolution;
            esdf_distance_[nei_idx] = new_dist;
            visited.insert(nei_idx);
            bfs_queue.push({nei_idx, new_dist});
        }
    }
}

void GlobalMap::updateESDFVoxel(const VoxelIndex& idx) {
    // 单体素ESDF更新（简化版）
    // 实际使用中建议调用 updateESDF
}

// ==================== 射线/路径检测 ====================

bool GlobalMap::isLineFree(const Position& start, const Position& end, double max_dist) const {
    Position collision;
    return isLineFree(start, end, collision, max_dist);
}

bool GlobalMap::isLineFree(const Position& start, const Position& end,
                           Position& collision_point, double max_dist) const {
    Position dir = end - start;
    double length = dir.norm();
    
    if (length < 1e-6) return true;
    if (length > max_dist) length = max_dist;
    
    dir.normalize();
    double step = config_.resolution * 0.5;
    int num_steps = static_cast<int>(length / step) + 1;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (int i = 0; i <= num_steps; i++) {
        Position pos = start + dir * (i * step);
        VoxelIndex idx = posToIndex(pos);
        
        auto it = voxels_.find(idx);
        if (it != voxels_.end() && !VoxelStateBits::isFree(it->second)) {
            collision_point = pos;
            return false;
        }
    }
    
    return true;
}

bool GlobalMap::isLineFreeInflate(const Position& start, const Position& end,
                                   bool use_unknown_as_occ) const {
    Position dir = end - start;
    double length = dir.norm();
    
    if (length < 1e-6) return true;
    
    dir.normalize();
    double step = config_.resolution * 0.5; // [Fix] Use finer step size for safety
    int num_steps = static_cast<int>(length / step) + 1;
    
    for (int i = 0; i <= num_steps; i++) {
        Position pos = start + dir * (i * step);
        
        if (isOccupiedInflate(pos)) {
            return false;
        }
        
        if (use_unknown_as_occ && isUnknownInflate(pos)) {
            return false;
        }
    }
    
    return true;
}

// ==================== 网格类型查询 ====================

GridType GlobalMap::getGridType(const Position& pos) const {
    if (!isInGlobalMapRange(pos)) {
        return GridType::OUT_OF_MAP;
    }
    
    VoxelIndex idx = posToIndex(pos);
    return getGridType(idx);
}

GridType GlobalMap::getGridType(const VoxelIndex& idx) const {
    if (!isInGlobalMapRange(idx)) {
        return GridType::OUT_OF_MAP;
    }
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = voxels_.find(idx);
    
    if (it == voxels_.end()) {
        return GridType::UNKNOWN;
    }
    
    VoxelState geo = VoxelStateBits::getGeoState(it->second);
    if (geo == VoxelStateBits::GEO_OCCUPIED) {
        return GridType::OCCUPIED;
    } else if (geo == VoxelStateBits::GEO_FREE) {
        return GridType::KNOWN_FREE;
    }
    
    return GridType::UNKNOWN;
}

GridType GlobalMap::getInfGridType(const Position& pos) const {
    if (!isInGlobalMapRange(pos)) {
        return GridType::OUT_OF_MAP;
    }
    
    if (isOccupiedInflate(pos)) {
        return GridType::OCCUPIED;
    } else if (isUnknownInflate(pos)) {
        return GridType::UNKNOWN;
    }
    
    return GridType::KNOWN_FREE;
}

double GlobalMap::getMapValue(const Position& pos) const {
    // 返回概率值（简化版：0为未知，1为占据）
    GridType type = getGridType(pos);
    switch (type) {
        case GridType::OCCUPIED: return 1.0;
        case GridType::KNOWN_FREE: return 0.0;
        default: return 0.5;
    }
}

// ==================== 包围盒搜索 ====================

void GlobalMap::boxSearch(const Position& box_min, const Position& box_max,
                          GridType type, std::vector<Position>& out_points) const {
    out_points.clear();
    
    VoxelIndex min_idx = posToIndex(box_min);
    VoxelIndex max_idx = posToIndex(box_max);
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (const auto& kv : voxels_) {
        if (!isInGlobalRange(kv.first, min_idx, max_idx)) continue;
        
        GridType voxel_type;
        VoxelState geo = VoxelStateBits::getGeoState(kv.second);
        if (geo == VoxelStateBits::GEO_OCCUPIED) voxel_type = GridType::OCCUPIED;
        else if (geo == VoxelStateBits::GEO_FREE) voxel_type = GridType::KNOWN_FREE;
        else voxel_type = GridType::UNKNOWN;
        
        if (voxel_type == type) {
            out_points.push_back(indexToPos(kv.first));
        }
    }
}

void GlobalMap::boxSearchInflate(const Position& box_min, const Position& box_max,
                                  GridType type, std::vector<Position>& out_points) const {
    out_points.clear();
    
    // 简化实现：遍历体素
    VoxelIndex min_idx = posToIndex(box_min);
    VoxelIndex max_idx = posToIndex(box_max);
    
    for (int64_t x = min_idx.x; x <= max_idx.x; x++) {
        for (int64_t y = min_idx.y; y <= max_idx.y; y++) {
            for (int64_t z = min_idx.z; z <= max_idx.z; z++) {
                Position pos = indexToPos(VoxelIndex(x, y, z));
                
                GridType voxel_type = getInfGridType(pos);
                if (voxel_type == type) {
                    out_points.push_back(pos);
                }
            }
        }
    }
}

void GlobalMap::boundBoxByLocalMap(Position& box_min, Position& box_max) const {
    // 限制到局部地图范围
    Position local_min = robot_state_.position - local_map_size_ * 0.5f;
    Position local_max = robot_state_.position + local_map_size_ * 0.5f;
    
    box_min.x() = std::max(box_min.x(), local_min.x());
    box_min.y() = std::max(box_min.y(), local_min.y());
    box_min.z() = std::max(box_min.z(), local_min.z());
    
    box_max.x() = std::min(box_max.x(), local_max.x());
    box_max.y() = std::min(box_max.y(), local_max.y());
    box_max.z() = std::min(box_max.z(), local_max.z());
}

Position GlobalMap::getLocalMapOrigin() const {
    return robot_state_.position - local_map_size_ * 0.5f;
}

Position GlobalMap::getLocalMapSize() const {
    return local_map_size_;
}

void GlobalMap::markSubGridAnalyzed(int linear_idx) {
    ready_for_analysis_subgrids_.erase(linear_idx);
    
    if (linear_idx >= 0 && linear_idx < static_cast<int>(sub_grids_.size())) {
        sub_grids_[linear_idx].semantic_status_ = SubGrid::SemanticStatus::ANALYZED;
    }
}

}  // namespace star_mvf

