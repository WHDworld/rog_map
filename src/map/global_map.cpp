/**
 * @file global_map.cpp
 * @brief STAR-MVF 全局地图实现
 */

#include "map/global_map.h"
#include "utils/perf_logger.hpp"
#include <cmath>

// 性能监控
#include "utils/perf_logger.hpp"

namespace star_mvf {

namespace {
template <typename Map>
size_t estimateHashBytes(const Map& map) {
    // libstdc++ node containers: bucket pointer array plus value, next pointer,
    // cached hash/alignment overhead. This is an estimate, not allocator truth.
    return map.bucket_count() * sizeof(void*) +
           map.size() * (sizeof(typename Map::value_type) + 2 * sizeof(void*));
}

template <typename T, typename Alloc>
size_t estimateVectorBytes(const std::vector<T, Alloc>& vec) {
    return vec.capacity() * sizeof(T);
}
}  // namespace

void GlobalMap::logMemoryStats(const std::string& tag) const {
    if (!PERF_LOG.isInitialized()) return;
    std::lock_guard<std::mutex> lock(map_mutex_);

    size_t unknown_size = 0, unknown_capacity = 0, unknown_hash_bytes = 0;
    size_t free_size = 0, free_capacity = 0, free_hash_bytes = 0;
    size_t occ_size = 0, occ_capacity = 0, occ_hash_bytes = 0;
    size_t frontier_size = 0, frontier_hash_bytes = 0, target_capacity = 0;
    for (const auto& sg : sub_grids_) {
        unknown_size += sg.unknown_voxels_list_.size();
        unknown_capacity += sg.unknown_voxels_list_.capacity();
        unknown_hash_bytes += estimateHashBytes(sg.unknown_to_list_pos_);
        free_size += sg.free_voxels_list_.size();
        free_capacity += sg.free_voxels_list_.capacity();
        free_hash_bytes += estimateHashBytes(sg.free_to_list_pos_);
        occ_size += sg.occ_unobserved_list_.size();
        occ_capacity += sg.occ_unobserved_list_.capacity();
        occ_hash_bytes += estimateHashBytes(sg.occ_unobserved_map_);
        frontier_size += sg.sub_grid_frontier_voxels_.size();
        frontier_hash_bytes += estimateHashBytes(sg.sub_grid_frontier_voxels_);
        target_capacity += sg.cached_targets_.capacity();
    }

    const size_t vectors_bytes =
        (unknown_capacity + free_capacity + occ_capacity) * sizeof(VoxelIndex) +
        target_capacity * sizeof(SemanticTargetMetadata) +
        ray_endpoint_stamp_.capacity() * sizeof(uint32_t) +
        frame_voxel_stamp_.capacity() * sizeof(uint32_t) +
        frame_voxel_state_.capacity() * sizeof(int8_t) +
        frame_touched_voxels_.capacity() * sizeof(VoxelIndex);
    const size_t hash_bytes = estimateHashBytes(voxels_) + estimateHashBytes(occupancy_buffer_) +
        estimateHashBytes(inflation_count_) +
        estimateHashBytes(esdf_distance_) + unknown_hash_bytes + free_hash_bytes +
        occ_hash_bytes + frontier_hash_bytes;
    const double estimate_mb = static_cast<double>(vectors_bytes + hash_bytes) / (1024.0 * 1024.0);

    PERF_LOG.logf("MAP", "MEMORY",
        "tag=%s estimate_mb=%.2f voxels=%zu/%zu occupancy=%zu/%zu frame=%zu/%zu "
        "inflation=%zu/%zu esdf=%zu/%zu subgrids=%zu unknown=%zu/%zu free=%zu/%zu "
        "occ_unobserved=%zu/%zu frontier=%zu dirty=%zu changed=%zu active_frontier_sg=%zu",
        tag.c_str(), estimate_mb, voxels_.size(), voxels_.bucket_count(),
        occupancy_buffer_.size(), occupancy_buffer_.bucket_count(), frame_touched_voxels_.size(),
        frame_touched_voxels_.capacity(), inflation_count_.size(), inflation_count_.bucket_count(),
        esdf_distance_.size(), esdf_distance_.bucket_count(), sub_grids_.size(),
        unknown_size, unknown_capacity, free_size, free_capacity, occ_size, occ_capacity,
        frontier_size, dirty_subgrids_.size(), frontier_changed_subgrids_.size(),
        current_frontier_subgrids_.size());
}

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
    ray_endpoint_stamp_.assign(getTotalVoxelCount(), 0);
    frame_voxel_stamp_.assign(getTotalVoxelCount(), 0);
    frame_voxel_state_.assign(getTotalVoxelCount(), 0);
    frame_touched_voxels_.reserve(65536);
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
    nh_.param(name_space + "/sensor/camera_ray_step", config_.camera_ray_step, 0.1f);
    nh_.param(name_space + "/sensor/camera_angular_resolution_deg", config_.camera_angular_res_deg, 3.0f);
    
    constexpr float DEG_TO_RAD = M_PI / 180.0f;
    fov_h_rad_ = config_.fov_h_deg * DEG_TO_RAD;
    fov_v_rad_ = config_.fov_v_deg * DEG_TO_RAD;
    tan_half_fov_h_ = std::tan(fov_h_rad_ / 2.0f);
    tan_half_fov_v_ = std::tan(fov_v_rad_ / 2.0f);
    
    // 前沿参数
    nh_.param(name_space + "/global_map/frontier_height_min", config_.frontier_height_min, 0.2f);
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
    
    // LiDAR 射线投射参数（从 LocalMap 移植）
    nh_.param(name_space + "/local_map/raycast_step", raycast_step_, 0.1f);
    nh_.param(name_space + "/sensor/lidar_downsample_filter", lidar_downsample_filter_, 3);
    ROS_INFO("[GlobalMap] LiDAR raycast: step=%.2f, downsample=%d", 
             raycast_step_, lidar_downsample_filter_);
    
    // Log-Odds 概率更新参数
    nh_.param(name_space + "/global_map/p_hit", config_.p_hit, 0.75f);
    nh_.param(name_space + "/global_map/p_miss", config_.p_miss, 0.45f);
    nh_.param(name_space + "/global_map/p_min", config_.p_min, 0.12f);
    nh_.param(name_space + "/global_map/p_max", config_.p_max, 0.97f);
    nh_.param(name_space + "/global_map/p_occ", config_.p_occ, 0.80f);
    
    // Log-Odds 转换
    auto logit = [](float p) { return std::log(p / (1.0f - p)); };
    prob_hit_log_ = logit(config_.p_hit);
    prob_miss_log_ = logit(config_.p_miss);
    clamp_min_log_ = logit(config_.p_min);
    clamp_max_log_ = logit(config_.p_max);
    min_occupancy_log_ = logit(config_.p_occ);
    unknown_flag_ = 0.01f;
    
    ROS_INFO("[GlobalMap] Log-Odds: hit=%.2f, miss=%.2f, clamp=[%.2f, %.2f], occ_thresh=%.2f", 
             prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_);
    
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

    // 保存到成员变量（用于 voxelToSubGrid 整数计算）
    global_min_idx_ = global_min_idx;
    global_max_idx_ = global_max_idx;

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
    // 存储到成员变量供 voxelToSubGrid 使用
    sub_grid_size_x_ = total_voxels_x / sub_grid_num_x_;
    sub_grid_size_y_ = total_voxels_y / sub_grid_num_y_;

    int total_subgrids = sub_grid_num_x_ * sub_grid_num_y_;
    sub_grids_.clear();
    sub_grids_.resize(total_subgrids);

    ROS_INFO("[GlobalMap] Map Voxels: [%d x %d x %d]", total_voxels_x, total_voxels_y, total_voxels_z);
    ROS_INFO("[GlobalMap] SubGrids: %d x %d (Base Size: %d x %d voxels)", 
             sub_grid_num_x_, sub_grid_num_y_, sub_grid_size_x_, sub_grid_size_y_);

    // 4. 遍历并初始化每个子网格
    for (int y = 0; y < sub_grid_num_y_; y++) {
        for (int x = 0; x < sub_grid_num_x_; x++) {
            int linear_idx = y * sub_grid_num_x_ + x;
            SubGrid& sg = sub_grids_[linear_idx];
            
            // ========== A. 设定索引范围 (Index Bounds) ==========
            sg.grid_idx = SubGridIndex(x, y);

            // 计算 X 轴起止偏移 (Offset) - 使用成员变量
            int start_offset_x = x * sub_grid_size_x_;
            int end_offset_x = (x + 1) * sub_grid_size_x_;
            // 修正：最后一列要把剩余的余数全部吃掉，直接对齐到 total_voxels_x
            if (x == sub_grid_num_x_ - 1) end_offset_x = total_voxels_x;

            // 计算 Y 轴起止偏移 - 使用成员变量
            int start_offset_y = y * sub_grid_size_y_;
            int end_offset_y = (y + 1) * sub_grid_size_y_;
            if (y == sub_grid_num_y_ - 1) end_offset_y = total_voxels_y;

            // 设置子网格的全局索引范围 (闭区间 [min, max])
            sg.global_min_idx.x = global_min_idx_.x + start_offset_x;
            sg.global_max_idx.x = global_min_idx_.x + end_offset_x - 1;

            sg.global_min_idx.y = global_min_idx_.y + start_offset_y;
            sg.global_max_idx.y = global_min_idx_.y + end_offset_y - 1;

            // Z 轴通常不切分，直接继承全局高度范围
            sg.global_min_idx.z = global_min_idx_.z;
            sg.global_max_idx.z = global_max_idx_.z - 1; // max_idx 是开区间，所以减 1 变闭区间

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

size_t GlobalMap::getTotalVoxelCount() const {
    const int64_t nx = global_max_idx_.x - global_min_idx_.x;
    const int64_t ny = global_max_idx_.y - global_min_idx_.y;
    const int64_t nz = global_max_idx_.z - global_min_idx_.z;
    if (nx <= 0 || ny <= 0 || nz <= 0) return 0;
    return static_cast<size_t>(nx * ny * nz);
}

bool GlobalMap::voxelToLinearAddress(const VoxelIndex& idx, size_t& address) const {
    const int64_t nx = global_max_idx_.x - global_min_idx_.x;
    const int64_t ny = global_max_idx_.y - global_min_idx_.y;
    const int64_t nz = global_max_idx_.z - global_min_idx_.z;
    const int64_t x = idx.x - global_min_idx_.x;
    const int64_t y = idx.y - global_min_idx_.y;
    const int64_t z = idx.z - global_min_idx_.z;
    if (x < 0 || y < 0 || z < 0 || x >= nx || y >= ny || z >= nz) return false;
    address = static_cast<size_t>((z * ny + y) * nx + x);
    return true;
}

bool GlobalMap::linearAddressToVoxel(size_t address, VoxelIndex& idx) const {
    // 严格逆变换：voxelToLinearAddress 使用 address = (z * ny + y) * nx + x
    const int64_t nx = global_max_idx_.x - global_min_idx_.x;
    const int64_t ny = global_max_idx_.y - global_min_idx_.y;
    const int64_t nz = global_max_idx_.z - global_min_idx_.z;
    if (nx <= 0 || ny <= 0 || nz <= 0) return false;
    const size_t total = static_cast<size_t>(nx * ny * nz);
    if (address >= total) return false;

    const size_t ux = static_cast<size_t>(nx);
    const size_t uy = static_cast<size_t>(ny);
    const int64_t x = static_cast<int64_t>(address % ux);
    const size_t t = address / ux;
    const int64_t y = static_cast<int64_t>(t % uy);
    const int64_t z = static_cast<int64_t>(t / uy);

    idx.x = global_min_idx_.x + x;
    idx.y = global_min_idx_.y + y;
    idx.z = global_min_idx_.z + z;
    return true;
}

Position GlobalMap::clipToMapBoundary(const Position& pt, const Position& sensor_pos) const {
    // 参考 FUEL sdf_map.cpp closetPointInMap
    // 找到从 sensor_pos 到 pt 的射线与地图边界的交点
    Position diff = pt - sensor_pos;
    Position max_tc = config_.global_max - sensor_pos;
    Position min_tc = config_.global_min - sensor_pos;
    
    float min_t = 1000000.0f;
    
    for (int i = 0; i < 3; ++i) {
        if (std::abs(diff[i]) > 1e-6f) {
            float t1 = max_tc[i] / diff[i];
            if (t1 > 0 && t1 < min_t) min_t = t1;
            float t2 = min_tc[i] / diff[i];
            if (t2 > 0 && t2 < min_t) min_t = t2;
        }
    }
    
    // 返回边界上的点（稍微缩进避免边界问题）
    return sensor_pos + (min_t - 1e-3f) * diff;
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

void GlobalMap::setVoxelVisitedDirect(const VoxelIndex& idx, bool is_visited) {
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

// ==================== 子网格操作 ====================

void GlobalMap::posToSubGrid(const Position& pos, SubGridIndex& sg_idx) const {
    sg_idx.x = static_cast<int>((pos.x() - config_.global_min.x()) / sub_grid_x_);
    sg_idx.y = static_cast<int>((pos.y() - config_.global_min.y()) / sub_grid_y_);
    sg_idx.x = std::max(0, std::min(sg_idx.x, sub_grid_num_x_ - 1));
    sg_idx.y = std::max(0, std::min(sg_idx.y, sub_grid_num_y_ - 1));
}

void GlobalMap::voxelToSubGrid(const VoxelIndex& voxel_idx, SubGridIndex& sg_idx) const {
    // 直接使用整数索引计算，避免浮点精度问题
    // 公式：sg_idx = (voxel_idx - global_min_idx_) / sub_grid_size_
    // 注意：最后一列/行可能包含余数，需要 clamp 到有效范围
    
    int offset_x = static_cast<int>(voxel_idx.x - global_min_idx_.x);
    int offset_y = static_cast<int>(voxel_idx.y - global_min_idx_.y);
    
    // 整数除法计算子网格索引
    sg_idx.x = offset_x / sub_grid_size_x_;
    sg_idx.y = offset_y / sub_grid_size_y_;
    
    // 安全 clamp（最后一列/行可能因为余数超出）
    sg_idx.x = std::max(0, std::min(sg_idx.x, sub_grid_num_x_ - 1));
    sg_idx.y = std::max(0, std::min(sg_idx.y, sub_grid_num_y_ - 1));
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

void GlobalMap::extractGlobalFrontierIncremental() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 只处理脏子网格
    for (int sg_id : dirty_subgrids_) 
    {
        if (sg_id < 0 || sg_id >= static_cast<int>(sub_grids_.size())) continue;
        
        SubGrid& sg = sub_grids_[sg_id];
        if (!sg.frontier_dirty) continue;

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
                        // [Target = Free Voxel] 
                        // neighbor is Unknown, so 'free_idx' is the boundary free voxel.
                        
                        // Height check on self (Free Voxel)
                        Position free_pos = indexToPos(free_idx);
                        if (free_pos.z() < config_.frontier_height_min || 
                            free_pos.z() > config_.frontier_height_max) break; // Self is invalid, next free
                        
                        sg.sub_grid_frontier_voxels_.insert(free_idx);
                        break; // Found one unknown neighbor, so this free voxel is a frontier. Done.
                    }
                }
            }
        } 
        else {
            // ==================== 策略 B: 反向查找 (遍历 Unknown -> 找 Free) ====================
            // 适用场景：未探索区域少 (如残留的死角)，列表短，遍历 Unknown 更快
            
            for (const auto& unk_idx : sg.unknown_voxels_list_) {
                // Safety check
                auto it = voxels_.find(unk_idx);
                if (it != voxels_.end() && !VoxelStateBits::isUnknown(it->second)) continue;

                // Check 6 neighbors for Free
                VoxelIndex best_free_idx;
                bool has_free_neighbor = false;
                
                for (int d = 0; d < 6; d++) {
                    VoxelIndex nei_idx = unk_idx + neighbor_offsets_6[d];
                    
                    if (!isInGlobalMapRange(nei_idx)) continue;
                    
                    auto nei_it = voxels_.find(nei_idx);
                    if (nei_it != voxels_.end() && VoxelStateBits::isFree(nei_it->second)) {
                        // Found a Free neighbor!
                        // Check Height of THIS Free neighbor
                        Position nei_pos = indexToPos(nei_idx);
                        if (nei_pos.z() >= config_.frontier_height_min && 
                            nei_pos.z() <= config_.frontier_height_max) {
                            
                            best_free_idx = nei_idx;
                            has_free_neighbor = true;
                            break; // Found valid free neighbor
                        }
                    }
                }

                if (has_free_neighbor) {
                    // [Target = Free Voxel]
                    sg.sub_grid_frontier_voxels_.insert(best_free_idx);
                }
            }
        }
        
        // 更新统计信息
        sg.has_frontier = !sg.sub_grid_frontier_voxels_.empty();
        sg.frontier_count = static_cast<int>(sg.sub_grid_frontier_voxels_.size());
        sg.frontier_dirty = false;
        
        // [新增] 记录此子网格的前沿发生了变化（供 ViewpointManager 使用）
        frontier_changed_subgrids_.insert(sg_id);
        
        // [新增] 语义分析触发：探索率达到阈值且未分析过
        if (exploration_rate >= semantic_exploration_threshold_ && 
            (sg.semantic_status_ == SubGrid::SemanticStatus::UNKNOWN ||
             sg.semantic_status_ == SubGrid::SemanticStatus::ACCUMULATING)) {
            ready_for_analysis_subgrids_.insert(sg_id);
            sg.semantic_status_ = SubGrid::SemanticStatus::READY;
        }
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

// ==================== 安全性检查 ====================

bool GlobalMap::isPathSafe(const Position& start, const Position& end, float safety_margin,
                           bool unknown_as_occupied) const {
    Position dir = end - start;
    float length = dir.norm();
    if (length < 1e-6f) return true;
    
    dir.normalize();
    float step = config_.resolution;
    int num_steps = static_cast<int>(length / step) + 1;

    // 在函数级持锁，避免逐体素加锁/解锁的开销（posToIndex 不访问 voxels_）
    std::lock_guard<std::mutex> lock(map_mutex_);

    for (int i = 0; i <= num_steps; i++) {
        Position pos = start + dir * (i * step);

        // 检查当前位置
        VoxelIndex idx;
        const_cast<GlobalMap*>(this)->posToIndex(pos, idx);

        auto it = voxels_.find(idx);
        if (it != voxels_.end() && VoxelStateBits::isOccupied(it->second)) {
            return false;
        }
        if (unknown_as_occupied &&
            (it == voxels_.end() || VoxelStateBits::isUnknown(it->second))) return false;
        
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
                        if (unknown_as_occupied &&
                            (check_it == voxels_.end() || VoxelStateBits::isUnknown(check_it->second))) {
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
            if(pos.z() < config_.ground_height + 0.2f) continue;
            pcl::PointXYZRGB pt;
            pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
            
            if (geo_state == VoxelStateBits::GEO_OCCUPIED) {
                if (need_occ) {
                    // 颜色：已访问为绿色，未访问为红色
                    if (need_visited && is_visited) {
                        pt.r = 0; pt.g = 255; pt.b = 0;
                        visited_cloud.push_back(pt);
                    } else {
                        pt.r = 255; pt.g = 0; pt.b = 0;
                    }
                    pt.r = 255; pt.g = 0; pt.b = 0;
                    occ_cloud.push_back(pt);
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
        global_map_frontier_pub_.publish(frontier_msg);
    }
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
    
    // 复制位姿避免竞态条件
    Position cam_pos;
    Quaternion cam_rot;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        cam_pos = current_cam_origin_;
        cam_rot = current_cam_rot_;
    }
    
    // 检查四元数是否已初始化（单位四元数 norm ≈ 1）
    if (cam_rot.coeffs().norm() < 0.5f) return;
    
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
    
    // 使用复制的位姿数据
    Eigen::Matrix3f rot = cam_rot.toRotationMatrix();
    
    // 使用配置的FOV参数（水平半角和垂直半角）
    float fov_h = config_.fov_h_deg * 0.5f * M_PI / 180.0f;  // 转换为弧度的半角
    float fov_v = config_.fov_v_deg * 0.5f * M_PI / 180.0f;
    float range = config_.camera_max_dist;  // 使用配置的最大感知距离
    
    float tan_h = std::tan(fov_h);
    float tan_v = std::tan(fov_v);
    
    // 远平面四个角点（光学坐标系：Z轴为光轴，X向右，Y向下）
    Eigen::Vector3f corners_cam[4];
    corners_cam[0] = Eigen::Vector3f(-range * tan_h, -range * tan_v, range);  // 左上
    corners_cam[1] = Eigen::Vector3f( range * tan_h, -range * tan_v, range);  // 右上
    corners_cam[2] = Eigen::Vector3f( range * tan_h,  range * tan_v, range);  // 右下
    corners_cam[3] = Eigen::Vector3f(-range * tan_h,  range * tan_v, range);  // 左下
    
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

void GlobalMap::updateCameraPose(const Position& origin, const Quaternion& rotation) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    current_cam_origin_ = origin;
    current_cam_rot_ = rotation;
    
    // 相机视域射线投射，标记占据体素为 VISITED
    if (!enable_camera_map_) return;
    
    // 计算射线数量（使用配置的角分辨率）
    float angular_res_rad = config_.camera_angular_res_deg * M_PI / 180.0f;
    int num_rays_h = std::max(1, static_cast<int>(config_.fov_h_deg * M_PI / 180.0f / angular_res_rad));
    int num_rays_v = std::max(1, static_cast<int>(config_.fov_v_deg * M_PI / 180.0f / angular_res_rad));
    
    float fov_h_rad = config_.fov_h_deg * M_PI / 180.0f;
    float fov_v_rad = config_.fov_v_deg * M_PI / 180.0f;
    float fov_h_start = -fov_h_rad / 2.0f;
    float fov_v_start = -fov_v_rad / 2.0f;
    
    float ray_step = config_.camera_ray_step;
    float max_range = config_.camera_max_dist;
    float min_range = config_.camera_min_dist;
    int max_steps = static_cast<int>(max_range / ray_step);
    
    Eigen::Matrix3f rot_mat = rotation.toRotationMatrix();
    
    for (int i = 0; i < num_rays_h; ++i) {
        float angle_h = fov_h_start + fov_h_rad * (i + 0.5f) / num_rays_h;
        for (int j = 0; j < num_rays_v; ++j) {
            float angle_v = fov_v_start + fov_v_rad * (j + 0.5f) / num_rays_v;
            
            // 光学坐标系：Z轴为光轴，X向右，Y向下
            float cos_v = std::cos(angle_v);
            Eigen::Vector3f dir_cam(
                cos_v * std::sin(angle_h),
                -std::sin(angle_v),
                cos_v * std::cos(angle_h)
            );
            
            Eigen::Vector3f dir_world = rot_mat * dir_cam;
            dir_world.normalize();
            
            // 沿光线步进
            for (int s = 0; s < max_steps; ++s) {
                float dist = s * ray_step;
                if (dist < min_range) continue;
                
                Position pos = origin + dir_world * dist;
                if (!isInGlobalMapRange(pos)) break;
                
                VoxelIndex idx = posToIndex(pos);
                auto it = voxels_.find(idx);
                if (it == voxels_.end()) continue;
                
                VoxelState geo = VoxelStateBits::getGeoState(it->second);
                if (geo == VoxelStateBits::GEO_OCCUPIED) 
                {
                    // [修复] 标记为 VISITED，同时从 occ_unobserved_list_ 移除
                    if (!VoxelStateBits::isVisited(it->second)) {
                        VoxelStateBits::setVisited(it->second, true);
                        
                        // 从子网格的未观测列表移除
                        SubGridIndex sg_idx;
                        voxelToSubGrid(idx, sg_idx);
                        int linear_idx = subGridToLinearIndex(sg_idx);
                        if (linear_idx >= 0 && linear_idx < static_cast<int>(sub_grids_.size())) {
                            sub_grids_[linear_idx].markOccObserved(idx);
                        }
                    }
                    break;  // 遇到占据体素停止
                }
            }
        }
    }
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

void GlobalMap::markSubGridAnalyzed(int linear_idx) {
    ready_for_analysis_subgrids_.erase(linear_idx);
    
    if (linear_idx >= 0 && linear_idx < static_cast<int>(sub_grids_.size())) {
        sub_grids_[linear_idx].semantic_status_ = SubGrid::SemanticStatus::ANALYZED;
    }
}

// ==================== LiDAR 点云处理（从 LocalMap 移植）====================

void GlobalMap::insertPointCloud(const sensor_msgs::PointCloud2& cloud, 
                                   const Position& sensor_pos) {
    const auto total_start = ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    const auto converted_at = ros::WallTime::now();
    
    if (pcl_cloud.empty()) return;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 新帧开始，清空帧内缓存
    if (++raycast_frame_id_ == 0) {
        std::fill(ray_endpoint_stamp_.begin(), ray_endpoint_stamp_.end(), 0);
        std::fill(frame_voxel_stamp_.begin(), frame_voxel_stamp_.end(), 0);
        raycast_frame_id_ = 1;
    }
    frame_touched_voxels_.clear();
    
    // 阶段1：处理每个点，标记终点为 hit，沿途标记 miss
    int point_counter = 0;
    size_t accepted_points = 0;
    size_t clipped_points = 0;
    size_t unique_rays = 0;
    
    for (const auto& pt : pcl_cloud) {
        if (lidar_downsample_filter_ > 1) {
            if ((point_counter++ % lidar_downsample_filter_) != 0) continue;
        }
        
        Position end_pos(pt.x, pt.y, pt.z);
        bool is_hit = true;  // 是否标记终点为占据
        
        // [修复] 处理超出地图范围的点云（参考 FUEL closetPointInMap）
        if (!isInGlobalMapRange(end_pos)) {
            // 将终点裁剪到地图边界
            end_pos = clipToMapBoundary(end_pos, sensor_pos);
            is_hit = false;  // 边界点不标记为 hit，只标记沿途 miss
            ++clipped_points;
            
            // 如果裁剪后仍然无效，跳过
            if (!isInGlobalMapRange(end_pos)) continue;
        }
        
        VoxelIndex end_idx = posToIndex(end_pos);
        ++accepted_points;
        
        // 标记终点（如果是真实命中点才标记 hit）
        if (is_hit) {
            setCacheOccupancy(end_idx, 1);
        }
        
        // FUEL flag_rayend_ pattern: dense generation stamps avoid allocating and
        // hashing an unordered_set for every point-cloud frame.
        size_t endpoint_address = 0;
        if (voxelToLinearAddress(end_idx, endpoint_address)) {
            if (ray_endpoint_stamp_[endpoint_address] == raycast_frame_id_) continue;
            ray_endpoint_stamp_[endpoint_address] = raycast_frame_id_;
        }
        ++unique_rays;
        
        // 射线投射：沿途标记 miss
        raycastAndMarkMiss(sensor_pos, end_pos, end_idx);
    }
    const auto raycast_at = ros::WallTime::now();
    const size_t frame_voxels = frame_touched_voxels_.size();
    
    // 阶段2：应用缓存更新到概率缓冲区
    applyFrameCache();
    const auto applied_at = ros::WallTime::now();
    PERF_LOG.logf("MAP", "INSERT",
        "input=%zu accepted=%zu clipped=%zu unique_rays=%zu frame_voxels=%zu "
        "convert_ms=%.3f raycast_ms=%.3f apply_ms=%.3f total_ms=%.3f",
        pcl_cloud.size(), accepted_points, clipped_points, unique_rays, frame_voxels,
        (converted_at - total_start).toSec() * 1000.0,
        (raycast_at - converted_at).toSec() * 1000.0,
        (applied_at - raycast_at).toSec() * 1000.0,
        (applied_at - total_start).toSec() * 1000.0);
}

void GlobalMap::setCacheOccupancy(const VoxelIndex& idx, int8_t occ) {
    size_t address = 0;
    if (!voxelToLinearAddress(idx, address)) return;
    if (frame_voxel_stamp_[address] != raycast_frame_id_) {
        frame_voxel_stamp_[address] = raycast_frame_id_;
        frame_voxel_state_[address] = occ;
        frame_touched_voxels_.push_back(idx);
    } else if (occ == 1) {
        frame_voxel_state_[address] = 1;  // hit wins over miss in one frame
    }
}

void GlobalMap::raycastAndMarkMiss(const Position& start, const Position& end,
                                     const VoxelIndex& end_idx) {
    Position dir = end - start;
    float length = dir.norm();
    if (length < 1e-6f) return;
    
    dir.normalize();
    int num_steps = static_cast<int>(length / raycast_step_);
    
    for (int i = 1; i < num_steps; i++) {  // 从1开始，跳过传感器位置
        Position pos = start + dir * (i * raycast_step_);
        if (!isInGlobalMapRange(pos)) continue;
        
        VoxelIndex idx = posToIndex(pos);
        if (idx == end_idx) break;  // 到达终点前停止
        
        setCacheOccupancy(idx, 0);  // miss
    }
}

void GlobalMap::applyFrameCache() {
    std::vector<std::pair<VoxelIndex, bool>> inflation_updates;
    
    for (const VoxelIndex& idx : frame_touched_voxels_) {
        size_t address = 0;
        if (!voxelToLinearAddress(idx, address)) continue;
        const int8_t occ = frame_voxel_state_[address];
        
        // 获取或初始化概率
        float& log_odds = occupancy_buffer_[idx];
        if (std::abs(log_odds) < 1e-6f) {
            log_odds = clamp_min_log_ - unknown_flag_;  // 初始为 unknown
        }
        
        // 概率累积更新
        float update = (occ == 1) ? prob_hit_log_ : prob_miss_log_;
        log_odds = std::clamp(log_odds + update, clamp_min_log_, clamp_max_log_);
        
        // 更新几何状态（参考 FUEL：只有 OCCUPIED/FREE 两种状态，无 UNKNOWN 回退）
        VoxelState& state = voxels_[idx];
        VoxelState old_geo = VoxelStateBits::getGeoState(state);
        VoxelState new_geo;
        
        // FUEL 逻辑：大于 min_occupancy_log_ 为占据，否则为自由
        if (log_odds > min_occupancy_log_) {
            new_geo = VoxelStateBits::GEO_OCCUPIED;
        } else {
            new_geo = VoxelStateBits::GEO_FREE;
        }
        
        if (new_geo != old_geo) {
            VoxelStateBits::setGeoState(state, new_geo);
            updateSubGridOnStateChange(idx, old_geo, new_geo);
            
            // 记录膨胀更新
            if (new_geo == VoxelStateBits::GEO_OCCUPIED) {
                inflation_updates.emplace_back(idx, true);
            } else if (old_geo == VoxelStateBits::GEO_OCCUPIED) {
                inflation_updates.emplace_back(idx, false);
            }
        }
    }
    
    // 更新膨胀
    for (const auto& update : inflation_updates) {
        updateInflation(update.first, update.second);
    }
}

void GlobalMap::updateSubGridOnStateChange(const VoxelIndex& idx, VoxelState old_geo, VoxelState new_geo) {
    SubGridIndex sg_idx;
    voxelToSubGrid(idx, sg_idx);
    int linear_idx = subGridToLinearIndex(sg_idx);
    
    if (linear_idx < 0 || linear_idx >= static_cast<int>(sub_grids_.size())) return;
    
    SubGrid& sg = sub_grids_[linear_idx];
    dirty_subgrids_.insert(linear_idx);
    sg.frontier_dirty = true;
    
    // [关键修复] 通知增量更新机制此子网格发生变化
    frontier_changed_subgrids_.insert(linear_idx);
    
    // 移除旧状态
    if (old_geo == VoxelStateBits::GEO_UNKNOWN) {
        sg.removeUnknownVoxel(idx);
    } else if (old_geo == VoxelStateBits::GEO_FREE) {
        sg.removeFreeVoxel(idx);
    } else if (old_geo == VoxelStateBits::GEO_OCCUPIED) {
        sg.occupied_count--;
    }
    
    // 添加新状态
    if (new_geo == VoxelStateBits::GEO_FREE) {
        sg.addFreeVoxel(idx);
    } else if (new_geo == VoxelStateBits::GEO_OCCUPIED) {
        sg.occupied_count++;
        
        // 语义检测：高于地面的新占据体素，加入未观测列表
        Position voxel_pos = indexToPos(idx);
        float height_thresh = config_.ground_height + 0.2f;
        
        if (voxel_pos.z() > height_thresh) 
        {
            auto it = voxels_.find(idx);
            bool is_visited = (it != voxels_.end()) && VoxelStateBits::isVisited(it->second);
            if(!is_visited)
            {
                sg.addOccUnobserved(idx);
            }
        } 
    }
}

}  // namespace star_mvf
