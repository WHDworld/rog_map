/**
 * @file local_map.cpp
 * @brief STAR-MVF 局部地图实现 - 基于循环数组的滑动窗口
 * 
 * 参考项目：ROG-Map (HKU-MaRS Lab)
 * https://github.com/hku-mars/ROG-Map
 */

#include "star_mvf/map/local_map.h"
#include <cmath>
#include <geometry_msgs/Point.h>

namespace star_mvf {

// 辅助函数：归一化索引到 [a, b] 范围（循环）
static int normalizeIndex(int x, int a, int b) {
    int range = b - a + 1;
    int y = (x - a) % range;
    return (y < 0 ? y + range : y) + a;
}

LocalMap::LocalMap() {
    origin_d_ = Position::Zero();
    origin_i_ = Eigen::Vector3i::Zero();
    bound_min_d_ = Position::Zero();
    bound_max_d_ = Position::Zero();
}

void LocalMap::init(const LocalMapConfig& config) {
    config_ = config;
    config_.resolution_inv = 1.0f / config_.resolution;
    
    // 计算半尺寸（体素数）
    half_map_size_i_.x() = static_cast<int>(config_.local_half_size.x() * config_.resolution_inv);
    half_map_size_i_.y() = static_cast<int>(config_.local_half_size.y() * config_.resolution_inv);
    half_map_size_i_.z() = static_cast<int>(config_.local_half_size.z() * config_.resolution_inv);
    
    // 总尺寸 = 2 * 半尺寸 + 1
    map_size_i_ = 2 * half_map_size_i_ + Eigen::Vector3i::Constant(1);
    map_vox_num_ = map_size_i_.x() * map_size_i_.y() * map_size_i_.z();
    
    // 初始化占据缓冲区
    occupancy_buffer_.resize(map_vox_num_, 0.0f);
    
    // 相机参数
    float DEG_TO_RAD = M_PI / 180.0f;
    camera_fov_h_rad_ = config_.camera_fov_h_deg * DEG_TO_RAD;
    camera_fov_v_rad_ = config_.camera_fov_v_deg * DEG_TO_RAD;
    tan_half_fov_h_ = std::tan(camera_fov_h_rad_ / 2.0f);
    tan_half_fov_v_ = std::tan(camera_fov_v_rad_ / 2.0f);
    
    // 计算最优角分辨率
    float optimal_angle_step = std::atan(config_.resolution / config_.camera_max_range);
    camera_angular_resolution_deg_ = optimal_angle_step * 180.0f / M_PI;
    
    initialized_ = true;
    
    ROS_INFO("[LocalMap] Initialized: half_size_i=(%d,%d,%d), map_size_i=(%d,%d,%d), total=%d voxels",
             half_map_size_i_.x(), half_map_size_i_.y(), half_map_size_i_.z(),
             map_size_i_.x(), map_size_i_.y(), map_size_i_.z(), map_vox_num_);
}

// ==================== 核心坐标转换函数 ====================

void LocalMap::posToGlobalIndex(const Position& pos, Eigen::Vector3i& global_idx) const {
    // ORIGIN_AT_CORNER 策略：floor
    global_idx.x() = static_cast<int>(std::floor(pos.x() * config_.resolution_inv));
    global_idx.y() = static_cast<int>(std::floor(pos.y() * config_.resolution_inv));
    global_idx.z() = static_cast<int>(std::floor(pos.z() * config_.resolution_inv));
}

void LocalMap::globalIndexToPos(const Eigen::Vector3i& global_idx, Position& pos) const {
    // ORIGIN_AT_CORNER 策略：体素中心在 (idx + 0.5) * resolution
    pos.x() = (global_idx.x() + 0.5f) * config_.resolution;
    pos.y() = (global_idx.y() + 0.5f) * config_.resolution;
    pos.z() = (global_idx.z() + 0.5f) * config_.resolution;
}

void LocalMap::globalIndexToLocalIndex(const Eigen::Vector3i& global_idx, Eigen::Vector3i& local_idx) const {
    // 核心循环映射算法（参考 ROG-Map 论文公式 7-8）
    for (int i = 0; i < 3; ++i) {
        // 计算 i_k = id_g(i) mod map_size(i)
        local_idx(i) = global_idx(i) % map_size_i_(i);
        
        // 归一化到 [-half_size, half_size] 范围
        if (local_idx(i) > half_map_size_i_(i)) {
            local_idx(i) -= map_size_i_(i);
        } else if (local_idx(i) < -half_map_size_i_(i)) {
            local_idx(i) += map_size_i_(i);
        }
    }
}

void LocalMap::localIndexToGlobalIndex(const Eigen::Vector3i& local_idx, Eigen::Vector3i& global_idx) const {
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -half_map_size_i_(i) + origin_i_(i);
        int min_id_l = min_id_g % map_size_i_(i);
        
        // 归一化 min_id_l
        if (min_id_l > half_map_size_i_(i)) {
            min_id_l -= map_size_i_(i);
        } else if (min_id_l < -half_map_size_i_(i)) {
            min_id_l += map_size_i_(i);
        }
        
        int cur_dis_to_min_id = local_idx(i) - min_id_l;
        if (cur_dis_to_min_id < 0) {
            cur_dis_to_min_id += map_size_i_(i);
        }
        
        global_idx(i) = cur_dis_to_min_id + min_id_g;
    }
}

int LocalMap::localIndexToHash(const Eigen::Vector3i& local_idx) const {
    Eigen::Vector3i idx = local_idx + half_map_size_i_;
    return idx.x() * map_size_i_.y() * map_size_i_.z() +
           idx.y() * map_size_i_.z() +
           idx.z();
}

void LocalMap::hashToLocalIndex(int hash_id, Eigen::Vector3i& local_idx) const {
    local_idx.x() = hash_id / (map_size_i_.y() * map_size_i_.z());
    int remainder = hash_id % (map_size_i_.y() * map_size_i_.z());
    local_idx.y() = remainder / map_size_i_.z();
    local_idx.z() = remainder % map_size_i_.z();
    local_idx -= half_map_size_i_;
}

void LocalMap::hashToGlobalIndex(int hash_id, Eigen::Vector3i& global_idx) const {
    Eigen::Vector3i local_idx;
    hashToLocalIndex(hash_id, local_idx);
    localIndexToGlobalIndex(local_idx, global_idx);
}

void LocalMap::hashToPos(int hash_id, Position& pos) const {
    Eigen::Vector3i global_idx;
    hashToGlobalIndex(hash_id, global_idx);
    globalIndexToPos(global_idx, pos);
}

int LocalMap::posToHash(const Position& pos) const {
    Eigen::Vector3i global_idx, local_idx;
    posToGlobalIndex(pos, global_idx);
    globalIndexToLocalIndex(global_idx, local_idx);
    return localIndexToHash(local_idx);
}

int LocalMap::globalIndexToHash(const Eigen::Vector3i& global_idx) const {
    Eigen::Vector3i local_idx;
    globalIndexToLocalIndex(global_idx, local_idx);
    return localIndexToHash(local_idx);
}

// ==================== 滑动地图核心函数 ====================

void LocalMap::updateOriginAndBound(const Position& new_origin_d, const Eigen::Vector3i& new_origin_i) {
    origin_i_ = new_origin_i;
    origin_d_ = new_origin_d;
    
    bound_max_i_ = origin_i_ + half_map_size_i_;
    bound_min_i_ = origin_i_ - half_map_size_i_;
    
    globalIndexToPos(bound_min_i_, bound_min_d_);
    globalIndexToPos(bound_max_i_, bound_max_d_);
    
    // 调整边界为体素边缘而非中心
    bound_min_d_ -= Position(0.5f, 0.5f, 0.5f) * config_.resolution;
    bound_max_d_ += Position(0.5f, 0.5f, 0.5f) * config_.resolution;
}

void LocalMap::clearMemoryOutOfMap(const std::vector<int>& clear_ids, int axis) {
    // 计算另外两个轴
    int axis1 = (axis + 1) % 3;
    int axis2 = (axis + 2) % 3;
    
    for (int id : clear_ids) {
        for (int x = -half_map_size_i_(axis1); x <= half_map_size_i_(axis1); x++) {
            for (int y = -half_map_size_i_(axis2); y <= half_map_size_i_(axis2); y++) {
                Eigen::Vector3i local_idx;
                local_idx(axis) = id;
                local_idx(axis1) = x;
                local_idx(axis2) = y;
                resetCell(localIndexToHash(local_idx));
            }
        }
    }
}

void LocalMap::resetCell(int hash_id) {
    if (hash_id >= 0 && hash_id < map_vox_num_) {
        occupancy_buffer_[hash_id] = 0.0f;
    }
}

void LocalMap::resetLocalMap() {
    std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0f);
    updated_occupied_.clear();
    updated_free_.clear();
    updated_observed_.clear();
}

void LocalMap::mapSliding(const Position& odom) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!config_.sliding_enabled) {
        return;
    }
    
    // 计算新原点
    Eigen::Vector3i new_origin_i;
    posToGlobalIndex(odom, new_origin_i);
    Position new_origin_d = Position(
        new_origin_i.x() * config_.resolution,
        new_origin_i.y() * config_.resolution,
        new_origin_i.z() * config_.resolution
    );
    
    // 计算位移
    Eigen::Vector3i shift_num = new_origin_i - origin_i_;
    
    // 如果位移太大，直接重置整个地图
    for (int i = 0; i < 3; i++) {
        if (std::abs(shift_num(i)) > map_size_i_(i)) {
            resetLocalMap();
            updateOriginAndBound(new_origin_d, new_origin_i);
            ROS_DEBUG("[LocalMap] Large shift, reset entire map");
            return;
        }
    }
    
    // 清除超出范围的内存
    for (int i = 0; i < 3; i++) {
        if (shift_num(i) == 0) {
            continue;
        }
        
        int min_id_g = -half_map_size_i_(i) + origin_i_(i);
        int min_id_l = min_id_g % map_size_i_(i);
        
        std::vector<int> clear_ids;
        
        if (shift_num(i) > 0) {
            // 正向滑动：清除最小端
            for (int k = 0; k < shift_num(i); k++) {
                int temp_id = min_id_l + k;
                temp_id = normalizeIndex(temp_id, -half_map_size_i_(i), half_map_size_i_(i));
                clear_ids.push_back(temp_id);
            }
        } else {
            // 负向滑动：清除最大端
            for (int k = -1; k >= shift_num(i); k--) {
                int temp_id = min_id_l + k;
                temp_id = normalizeIndex(temp_id, -half_map_size_i_(i), half_map_size_i_(i));
                clear_ids.push_back(temp_id);
            }
        }
        
        if (!clear_ids.empty()) {
            clearMemoryOutOfMap(clear_ids, i);
        }
    }
    
    updateOriginAndBound(new_origin_d, new_origin_i);
}

bool LocalMap::isInLocalMap(const Position& pos) const {
    Eigen::Vector3i global_idx;
    posToGlobalIndex(pos, global_idx);
    return isInLocalMap(global_idx);
}

bool LocalMap::isInLocalMap(const Eigen::Vector3i& global_idx) const {
    Eigen::Vector3i diff = (global_idx - origin_i_).cwiseAbs();
    return diff.x() <= half_map_size_i_.x() &&
           diff.y() <= half_map_size_i_.y() &&
           diff.z() <= half_map_size_i_.z();
}

// ==================== 点云处理 ====================
void LocalMap::insertPointCloud(const sensor_msgs::PointCloud2& cloud, const Position& sensor_pos) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // ========== 三阶段处理，解决射线覆盖和杂点过滤问题 ==========
    // 问题1：两个点几乎同方向但距离不同时，远处点的射线会覆盖近处点的占据状态
    // 问题2：远处的点可能是杂点（被近处障碍物遮挡），不应该被标记为占据
    // 解决：
    //   阶段1：收集所有候选占据体素（不更新概率）
    //   阶段2：射线更新，遇到候选占据体素时终止，并标记为"真正占据"
    //   阶段3：只对"真正占据"的体素更新概率
    
    // 收集有效点云和候选占据体素
    std::vector<std::pair<Position, int>> valid_endpoints;  // (终点位置, 体素ID)
    std::unordered_set<int> candidate_occupied_voxels;  // 候选占据体素（用于射线终止判断）
    valid_endpoints.reserve(pcl_cloud.size() / std::max(1, config_.lidar_downsample_filter));
    
    int point_counter = 0;
    
    // 阶段1：预处理 - 收集有效终点和候选占据体素（不更新概率）
    for (const auto& pt : pcl_cloud) {
        // 简单降采样：每 N 个点取一个
        if (config_.lidar_downsample_filter > 1) {
            if ((point_counter++ % config_.lidar_downsample_filter) != 0) {
                continue;
            }
        }
        
        Position end_pos(pt.x, pt.y, pt.z);
        float dist = (end_pos - sensor_pos).norm();
        
        // 距离过滤
        if (dist < config_.lidar_min_range || dist > config_.lidar_max_range) {
            continue;
        }
        
        // 收集候选占据体素（但不更新概率）
        if (isInLocalMap(end_pos)) {
            int hash_id = posToHash(end_pos);
            candidate_occupied_voxels.insert(hash_id);
            valid_endpoints.emplace_back(end_pos, hash_id);
        }
    }
    
    // 阶段2：射线更新，同时确定真正的占据体素
    std::unordered_set<int> confirmed_occupied_voxels;  // 真正的占据体素
    
    if (config_.use_raycast) {
        for (const auto& endpoint : valid_endpoints) {
            const Position& end_pos = endpoint.first;
            int end_hash_id = endpoint.second;
            
            // 射线更新，返回是否到达终点（没有被其他占据点截断）
            bool reached_endpoint = raycastWithConfirm(sensor_pos, end_pos, 
                                                        candidate_occupied_voxels,
                                                        confirmed_occupied_voxels);
            
            // 如果射线到达终点，确认该终点为真正的占据
            if (reached_endpoint) {
                confirmed_occupied_voxels.insert(end_hash_id);
            }
        }
    } else {
        // 不使用射线时，所有候选都是真正的占据
        confirmed_occupied_voxels = candidate_occupied_voxels;
    }
    
    // 阶段3：只对真正的占据体素更新概率
    for (int hash_id : confirmed_occupied_voxels) {
        updateVoxel(hash_id, config_.log_odds_hit);
    }
}

void LocalMap::raycast(const Position& start, const Position& end, 
                       const std::unordered_set<int>& occupied_voxels) {
    Position dir = end - start;
    float length = dir.norm();
    if (length < 1e-6f) return;
    
    dir.normalize();
    int num_steps = static_cast<int>(length / config_.raycast_step);
    
    for (int i = 0; i < num_steps; i++) {
        Position pos = start + dir * (i * config_.raycast_step);
        if (isInLocalMap(pos)) {
            int hash_id = posToHash(pos);
            
            // 关键优化：遇到本帧已被标记为占据的体素时，直接终止射线
            // 因为该方向已有障碍物，后面的体素要么在障碍物后面，要么是杂点
            if (occupied_voxels.find(hash_id) != occupied_voxels.end()) {
                break;  // 终止射线，不是 continue
            }
            
            updateVoxel(hash_id, config_.log_odds_miss);
        }
    }
}

bool LocalMap::raycastWithConfirm(const Position& start, const Position& end,
                                   const std::unordered_set<int>& candidate_occupied,
                                   std::unordered_set<int>& confirmed_occupied) {
    Position dir = end - start;
    float length = dir.norm();
    if (length < 1e-6f) return true;
    
    dir.normalize();
    int num_steps = static_cast<int>(length / config_.raycast_step);
    int end_hash_id = posToHash(end);
    
    for (int i = 0; i < num_steps; i++) {
        Position pos = start + dir * (i * config_.raycast_step);
        if (isInLocalMap(pos)) {
            int hash_id = posToHash(pos);
            
            // 遇到候选占据体素
            if (candidate_occupied.find(hash_id) != candidate_occupied.end()) {
                // 如果这个体素就是终点，说明射线成功到达
                if (hash_id == end_hash_id) {
                    return true;
                }
                // 如果是其他占据体素，说明射线被截断
                // 确认这个中间的占据点为真正的占据
                confirmed_occupied.insert(hash_id);
                return false;  // 射线被截断，终点是杂点
            }
            
            // 更新为 free
            updateVoxel(hash_id, config_.log_odds_miss);
        }
    }
    
    // 射线没有被截断，成功到达终点
    return true;
}

// 保留原始接口以兼容其他调用
void LocalMap::raycast(const Position& start, const Position& end) {
    std::unordered_set<int> empty_set;
    raycast(start, end, empty_set);
}

void LocalMap::updateVoxel(int hash_id, float log_odds_update) {
    if (hash_id < 0 || hash_id >= map_vox_num_) return;
    
    float& log_odds = occupancy_buffer_[hash_id];
    float old_log_odds = log_odds;
    
    log_odds += log_odds_update;
    log_odds = std::max(config_.log_odds_min, std::min(config_.log_odds_max, log_odds));
    
    // 记录状态变化
    bool was_occupied = old_log_odds > config_.log_odds_occ_thresh;
    bool is_occupied = log_odds > config_.log_odds_occ_thresh;
    bool was_free = old_log_odds < -config_.log_odds_occ_thresh;
    bool is_free = log_odds < -config_.log_odds_occ_thresh;
    
    if (!was_occupied && is_occupied) {
        updated_occupied_.insert(hash_id);
        updated_free_.erase(hash_id);
    } else if (!was_free && is_free) {
        updated_free_.insert(hash_id);
        updated_occupied_.erase(hash_id);
    }
}

// ==================== 数据获取 ====================

void LocalMap::getOccupiedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    cloud->clear();
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (int i = 0; i < map_vox_num_; i++) {
        if (occupancy_buffer_[i] > config_.log_odds_occ_thresh) {
            Position pos;
            hashToPos(i, pos);
            cloud->push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
        }
    }
}

void LocalMap::getFreeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    cloud->clear();
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (int i = 0; i < map_vox_num_; i++) {
        if (occupancy_buffer_[i] < -config_.log_odds_occ_thresh) {
            Position pos;
            hashToPos(i, pos);
            cloud->push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
        }
    }
}

void LocalMap::getUpdatedVoxels(std::vector<VoxelIndex>& occupied,
                                std::vector<VoxelIndex>& free_voxels) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    occupied.clear();
    free_voxels.clear();
    
    for (int hash_id : updated_occupied_) {
        Position pos;
        hashToPos(hash_id, pos);
        
        // 使用固定点数表示位置（乘以 1000 保留精度）
        VoxelIndex global_idx;
        global_idx.x = static_cast<int64_t>(pos.x() * 1000);
        global_idx.y = static_cast<int64_t>(pos.y() * 1000);
        global_idx.z = static_cast<int64_t>(pos.z() * 1000);
        occupied.push_back(global_idx);
    }
    
    for (int hash_id : updated_free_) {
        Position pos;
        hashToPos(hash_id, pos);
        
        VoxelIndex global_idx;
        global_idx.x = static_cast<int64_t>(pos.x() * 1000);
        global_idx.y = static_cast<int64_t>(pos.y() * 1000);
        global_idx.z = static_cast<int64_t>(pos.z() * 1000);
        free_voxels.push_back(global_idx);
    }
}

void LocalMap::clearUpdateCache() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    updated_occupied_.clear();
    updated_free_.clear();
    updated_observed_.clear();
}

float LocalMap::getOccupancy(const Position& pos) const {
    if (!isInLocalMap(pos)) return 0.0f;
    
    int hash_id = posToHash(pos);
    if (hash_id < 0 || hash_id >= map_vox_num_) return 0.0f;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    return occupancy_buffer_[hash_id];
}

bool LocalMap::isOccupied(const Position& pos) const {
    return getOccupancy(pos) > config_.log_odds_occ_thresh;
}

// ==================== 相机可视域 ====================
void LocalMap::updateCameraFrustum(const Position& camera_origin, 
                                    const Quaternion& camera_rotation) {
    std::lock_guard<std::mutex> lock(map_mutex_);

    int num_rays_h, num_rays_v;
    computeCameraRayCount(num_rays_h, num_rays_v);

    float fov_h_start = -camera_fov_h_rad_ / 2.0f;
    float fov_h_end = camera_fov_h_rad_ / 2.0f;
    float fov_v_start = -camera_fov_v_rad_ / 2.0f;
    float fov_v_end = camera_fov_v_rad_ / 2.0f;

    for (int i = 0; i < num_rays_h; ++i) {
        float angle_h = fov_h_start + (fov_h_end - fov_h_start) * (i + 0.5f) / num_rays_h;
        for (int j = 0; j < num_rays_v; ++j) {
            float angle_v = fov_v_start + (fov_v_end - fov_v_start) * (j + 0.5f) / num_rays_v;

            float cos_v = std::cos(angle_v);
            Position dir_cam(
                cos_v * std::cos(angle_h),
                cos_v * std::sin(angle_h),
                std::sin(angle_v)
            );

            Position dir_world = camera_rotation * dir_cam;
            dir_world.normalize();

            castCameraRay(camera_origin, dir_world, config_.camera_max_range);
        }
    }
}

void LocalMap::computeCameraRayCount(int& num_rays_h, int& num_rays_v) const {
    constexpr float DEG_TO_RAD = M_PI / 180.0f;
    float angle_step_rad = camera_angular_resolution_deg_ * DEG_TO_RAD;
    
    num_rays_h = static_cast<int>(std::ceil(camera_fov_h_rad_ / angle_step_rad));
    num_rays_v = static_cast<int>(std::ceil(camera_fov_v_rad_ / angle_step_rad));
    
    num_rays_h = std::max(1, num_rays_h);
    num_rays_v = std::max(1, num_rays_v);
}

bool LocalMap::castCameraRay(const Position& origin, const Position& direction, float max_range) {
    float step = config_.camera_ray_step;
    int max_steps = static_cast<int>(max_range / step);
    
    int last_hash_id = -1;
    
    for (int i = 0; i < max_steps; ++i) {
        float dist = i * step;
        if (dist < config_.camera_min_range) continue;
        
        Position current_pos = origin + direction * dist;
        
        if (!isInLocalMap(current_pos)) {
            break;
        }
        
        int hash_id = posToHash(current_pos);
        if (hash_id < 0 || hash_id >= map_vox_num_) continue;
        
        if (hash_id == last_hash_id) continue;
        last_hash_id = hash_id;
        
        float log_odds = occupancy_buffer_[hash_id];
        bool is_occupied = log_odds > config_.log_odds_occ_thresh;
        bool is_free = log_odds < -config_.log_odds_occ_thresh;
        
        if (is_free || is_occupied) {
            updated_observed_.insert(hash_id);
        }
        
        if (is_occupied) {
            return true;
        }
    }
    
    return false;
}

void LocalMap::getObservedVoxels(std::vector<VoxelIndex>& observed_voxels) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    observed_voxels.clear();
    
    for (int hash_id : updated_observed_) {
        Position pos;
        hashToPos(hash_id, pos);
        
        VoxelIndex global_idx;
        global_idx.x = static_cast<int64_t>(pos.x() * 1000);
        global_idx.y = static_cast<int64_t>(pos.y() * 1000);
        global_idx.z = static_cast<int64_t>(pos.z() * 1000);
        observed_voxels.push_back(global_idx);
    }
}

void LocalMap::clearObservedCache() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    updated_observed_.clear();
}

// ==================== 可视化实现 ====================

void LocalMap::initVisualization(ros::NodeHandle& nh, const std::string& frame_id) {
    viz_frame_id_ = frame_id;
    local_map_bound_pub_ = nh.advertise<visualization_msgs::MarkerArray>("local_map/bound", 1);
    local_map_occ_pub_ = nh.advertise<sensor_msgs::PointCloud2>("local_map/occ", 1);
}

void LocalMap::visualize() {
    // 1. 局部地图范围框
    if (local_map_bound_pub_.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;
        marker.header.frame_id = viz_frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "local_map_bound";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.08;
        marker.color.r = 0.2;
        marker.color.g = 0.6;
        marker.color.b = 1.0;
        marker.color.a = 0.9;
        
        Position min_pt, max_pt;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            min_pt = bound_min_d_;
            max_pt = bound_max_d_;
        }
        
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
        // 竖边
        marker.points.push_back(p1); marker.points.push_back(p5);
        marker.points.push_back(p2); marker.points.push_back(p6);
        marker.points.push_back(p3); marker.points.push_back(p7);
        marker.points.push_back(p4); marker.points.push_back(p8);
        
        markers.markers.push_back(marker);
        local_map_bound_pub_.publish(markers);
    }
    
    // 2. 局部地图占据体素
    if (local_map_occ_pub_.getNumSubscribers() > 0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr occ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        getOccupiedCloud(occ_cloud);
        
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*occ_cloud, msg);
        msg.header.frame_id = viz_frame_id_;
        msg.header.stamp = ros::Time::now();
        local_map_occ_pub_.publish(msg);
    }
}

}  // namespace star_mvf
