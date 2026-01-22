#include "rog_map/target_voxel_grid.h"
#include <cmath>
#include <algorithm>

namespace rog_map {

TargetVoxelGrid::TargetVoxelGrid(const Eigen::Vector3d& center, 
                                   const Eigen::Vector3d& size, 
                                   double resolution)
    : center_(center), size_(size), resolution_(resolution)
{
    // 添加边距确保点云不会超出边界
    double margin = 0.1;
    Eigen::Vector3d padded_size = size + Eigen::Vector3d(margin, margin, margin) * 2.0;
    
    // 计算各维度体素数量
    nx_ = std::max(1, static_cast<int>(std::ceil(padded_size.x() / resolution_)));
    ny_ = std::max(1, static_cast<int>(std::ceil(padded_size.y() / resolution_)));
    nz_ = std::max(1, static_cast<int>(std::ceil(padded_size.z() / resolution_)));
    total_voxels_ = nx_ * ny_ * nz_;
    
    // 计算网格原点（左下角）
    origin_ = center_ - Eigen::Vector3d(nx_ * resolution_ / 2.0, 
                                         ny_ * resolution_ / 2.0, 
                                         nz_ * resolution_ / 2.0);
    
    // 初始化体素数组
    voxels_.resize(total_voxels_);
    
    // [新增] 估算理论表面体素数（基于 BBox 六个面）
    // 使用原始目标尺寸（不含边距）计算
    int sx = std::max(1, static_cast<int>(std::ceil(size.x() / resolution_)));
    int sy = std::max(1, static_cast<int>(std::ceil(size.y() / resolution_)));
    int sz = std::max(1, static_cast<int>(std::ceil(size.z() / resolution_)));
    estimated_surface_count_ = 2 * (sx * sy + sx * sz + sy * sz);
    
    // ROS_INFO("[TargetVoxelGrid] Created: size=(%d,%d,%d), estimated_surface=%d",
    //          sx, sy, sz, estimated_surface_count_);
}

void TargetVoxelGrid::update(const pcl::PointCloud<pcl::PointXYZ>& cloud, 
                              const Eigen::Vector3d& sensor_pose) 
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (cloud.empty()) return;
    
    for (const auto& pt : cloud.points) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        
        // ========== Raycasting: 标记射线路径为 FREE ==========
        raycastAndMarkFree(sensor_pose, p);
        
        // ========== 终点标记为 OCCUPIED ==========
        int idx = posToIndex(p);
        if (idx < 0) continue;
        
        TargetVoxelState& voxel = voxels_[idx];
        
        // 状态转换：UNKNOWN/FREE -> OCCUPIED
        if (voxel.state != TargetVoxelType::OCCUPIED) {
            if (voxel.state == TargetVoxelType::FREE) {
                free_count_--;  // 从 FREE 转为 OCCUPIED
            }
            voxel.state = TargetVoxelType::OCCUPIED;
            occupied_count_++;
        }
        
        // [改进] 统计高质量扫描体素：hit_count 从 2 变为 3 时触发
        int old_hit = voxel.hit_count;
        voxel.hit_count++;
        if (old_hit < 6 && voxel.hit_count >= 6) {
            well_scanned_count_++;
        }
    }
}

void TargetVoxelGrid::raycastAndMarkFree(const Eigen::Vector3d& from, 
                                          const Eigen::Vector3d& to)
{
    Eigen::Vector3d dir = to - from;
    double total_dist = dir.norm();
    if (total_dist < 1e-6) return;
    
    double step_size = resolution_ * 0.5;  // 半个体素步长
    int num_steps = static_cast<int>(total_dist / step_size);
    Eigen::Vector3d step = dir.normalized() * step_size;
    
    for (int i = 1; i < num_steps - 1; ++i) {  // 跳过起点和终点
        Eigen::Vector3d sample_pt = from + step * i;
        int idx = posToIndex(sample_pt);
        if (idx < 0) continue;
        
        TargetVoxelState& voxel = voxels_[idx];
        
        // 只有 UNKNOWN 才能转为 FREE
        if (voxel.state == TargetVoxelType::UNKNOWN) {
            voxel.state = TargetVoxelType::FREE;
            free_count_++;
        }
    }
}

double TargetVoxelGrid::getCoverageRatio() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // [修正] 覆盖率 = 实际发现的表面体素 / 理论表面体素
    // - 分子：occupied_count_（累积扫描发现的表面体素数）
    // - 分母：estimated_surface_count_（基于 BBox 估算的理论表面数）
    if (estimated_surface_count_ == 0) return 0.0;
    
    // 使用 min 防止超过 100%（实际表面可能比 BBox 估算更复杂）
    return std::min(1.0, static_cast<double>(occupied_count_) / estimated_surface_count_);
}

std::unordered_set<int> TargetVoxelGrid::computeNewCoverage(
    const Eigen::Vector3d& vp_pos,
    double vp_yaw,
    double camera_pitch,
    double hfov,
    double vfov,
    double max_range,
    const std::unordered_set<int>& already_covered) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::unordered_set<int> new_coverage;
    
    // [改进] 基于射线采样的覆盖评估，同时考虑 UNKNOWN 和 OCCUPIED 体素
    // 在 FOV 内采样 8×8 = 64 条射线
    auto ray_dirs = sampleRayDirections(vp_yaw, camera_pitch, hfov, vfov, 8, 8);
    
    for (const auto& dir : ray_dirs) {
        // 计算射线与目标 BBox 的交点
        Eigen::Vector3d entry, exit;
        if (!rayBBoxIntersection(vp_pos, dir, entry, exit)) continue;
        
        // 距离检查
        double dist_to_entry = (entry - vp_pos).norm();
        if (dist_to_entry > max_range) continue;
        
        // 沿射线遍历 BBox 内的体素
        double step = resolution_ * 0.5;
        double ray_length = (exit - entry).norm();
        Eigen::Vector3d ray_dir = (exit - entry).normalized();
        
        for (double t = 0; t < ray_length; t += step) {
            Eigen::Vector3d pt = entry + ray_dir * t;
            int idx = posToIndex(pt);
            if (idx < 0) continue;
            
            const auto& voxel = voxels_[idx];
            
            // 遇到 UNKNOWN 或 OCCUPIED 体素 → 有覆盖价值
            if (voxel.state == TargetVoxelType::UNKNOWN || 
                voxel.state == TargetVoxelType::OCCUPIED) {
                if (!already_covered.count(idx)) {
                    new_coverage.insert(idx);
                }
                break;  // 射线被阻挡，停止遍历这条射线
            }
        }
    }
    
    return new_coverage;
}

double TargetVoxelGrid::computeWeightedCoverageScore(
    const Eigen::Vector3d& vp_pos,
    double vp_yaw,
    double camera_pitch,
    double hfov,
    double vfov,
    double max_range) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    double total_score = 0.0;
    
    // 在 FOV 内采样 8×8 = 64 条射线
    auto ray_dirs = sampleRayDirections(vp_yaw, camera_pitch, hfov, vfov, 8, 8);
    
    for (const auto& dir : ray_dirs) {
        // 计算射线与目标 BBox 的交点
        Eigen::Vector3d entry, exit;
        if (!rayBBoxIntersection(vp_pos, dir, entry, exit)) continue;
        
        // 距离检查
        double dist_to_entry = (entry - vp_pos).norm();
        if (dist_to_entry > max_range) continue;
        
        // 沿射线遍历 BBox 内的体素
        double step = resolution_ * 0.5;
        double ray_length = (exit - entry).norm();
        Eigen::Vector3d ray_dir = (exit - entry).normalized();
        
        for (double t = 0; t < ray_length; t += step) {
            Eigen::Vector3d pt = entry + ray_dir * t;
            int idx = posToIndex(pt);
            if (idx < 0) continue;
            
            const auto& voxel = voxels_[idx];
            
            // [加权评分策略]
            if (voxel.state == TargetVoxelType::UNKNOWN) {
                // UNKNOWN 体素：最高信息增益
                total_score += 1.0;
                break;
            } else if (voxel.state == TargetVoxelType::OCCUPIED) {
                if (!voxel.isWellScanned()) {
                    // OCCUPIED 但未充分扫描 (hit_count < 3)：需要更多观测
                    total_score += 0.5;
                } else {
                    // OCCUPIED 且已充分扫描：可以再观测但优先级低
                    total_score += 0.25;
                }
                break;
            }
            // FREE 体素：继续遍历
        }
    }
    
    return total_score;
}

std::vector<int> TargetVoxelGrid::getSurfaceVoxelIndices() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<int> indices;
    indices.reserve(occupied_count_);
    
    for (int idx = 0; idx < total_voxels_; ++idx) {
        if (voxels_[idx].state == TargetVoxelType::OCCUPIED) {
            indices.push_back(idx);
        }
    }
    return indices;
}

std::vector<int> TargetVoxelGrid::getUnobservedVoxelIndices() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<int> indices;
    
    for (int idx = 0; idx < total_voxels_; ++idx) {
        // [改进] 未充分扫描的体素（hit_count < 3）
        if (voxels_[idx].state == TargetVoxelType::OCCUPIED && !voxels_[idx].isWellScanned()) {
            indices.push_back(idx);
        }
    }
    return indices;
}

std::vector<int> TargetVoxelGrid::getOccupiedVoxelIndices() const {
    // 直接复用 getSurfaceVoxelIndices 的逻辑
    return getSurfaceVoxelIndices();
}

bool TargetVoxelGrid::isOccluded(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
    // 射线追踪：DDA算法
    Eigen::Vector3d dir = to - from;
    double total_dist = dir.norm();
    if (total_dist < 1e-6) return false;
    
    // 使用较粗的步长加速
    double step_size = resolution_ * 0.8;
    int num_steps = static_cast<int>(total_dist / step_size);
    Eigen::Vector3d step = dir.normalized() * step_size;
    
    // 从起点向终点逐步检测
    for (int i = 1; i < num_steps; ++i) {
        Eigen::Vector3d sample_pt = from + step * i;
        
        // 接近终点时跳过
        if ((sample_pt - to).norm() < resolution_ * 1.5) break;
        
        int idx = posToIndex(sample_pt);
        if (idx >= 0 && voxels_[idx].state == TargetVoxelType::OCCUPIED) {
            // 中途遇到表面体素，被遮挡
            return true;
        }
    }
    
    return false;  // 可见
}

bool TargetVoxelGrid::isSurface(const Eigen::Vector3d& point) const {
    int idx = posToIndex(point);
    if (idx < 0) return false;
    return voxels_[idx].state == TargetVoxelType::OCCUPIED;
}

Eigen::Vector3d TargetVoxelGrid::getVoxelCenter(int idx) const {
    return indexToPos(idx);
}

int TargetVoxelGrid::posToIndex(const Eigen::Vector3d& pos) const {
    // 计算在网格中的相对位置
    Eigen::Vector3d rel = pos - origin_;
    
    int ix = static_cast<int>(std::floor(rel.x() / resolution_));
    int iy = static_cast<int>(std::floor(rel.y() / resolution_));
    int iz = static_cast<int>(std::floor(rel.z() / resolution_));
    
    // 边界检查
    if (ix < 0 || ix >= nx_ || iy < 0 || iy >= ny_ || iz < 0 || iz >= nz_) {
        return -1;
    }
    
    return ix + iy * nx_ + iz * nx_ * ny_;
}

Eigen::Vector3d TargetVoxelGrid::indexToPos(int idx) const {
    if (idx < 0 || idx >= total_voxels_) {
        return Eigen::Vector3d::Zero();
    }
    
    int iz = idx / (nx_ * ny_);
    int rem = idx % (nx_ * ny_);
    int iy = rem / nx_;
    int ix = rem % nx_;
    
    // 返回体素中心
    return origin_ + Eigen::Vector3d(
        (ix + 0.5) * resolution_,
        (iy + 0.5) * resolution_,
        (iz + 0.5) * resolution_
    );
}

bool TargetVoxelGrid::isInBounds(const Eigen::Vector3d& pos) const {
    Eigen::Vector3d rel = pos - origin_;
    return rel.x() >= 0 && rel.x() < nx_ * resolution_ &&
           rel.y() >= 0 && rel.y() < ny_ * resolution_ &&
           rel.z() >= 0 && rel.z() < nz_ * resolution_;
}

bool TargetVoxelGrid::isInFOV(const Eigen::Vector3d& vp_pos, double vp_yaw, double camera_pitch,
                               double hfov, double vfov, const Eigen::Vector3d& target_point) const 
{
    // 计算视点到目标点的方向
    Eigen::Vector3d dir_to_target = (target_point - vp_pos).normalized();
    
    // 相机朝向（由 yaw 和固定 pitch 决定）
    Eigen::Vector3d cam_forward(
        cos(camera_pitch) * cos(vp_yaw),
        cos(camera_pitch) * sin(vp_yaw),
        sin(camera_pitch)
    );
    
    // 计算总夹角
    double cos_angle = cam_forward.dot(dir_to_target);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double angle = std::acos(cos_angle);
    
    // 使用最大FOV的一半作为阈值（简化的圆锥检查）
    double max_half_fov = std::max(hfov, vfov) / 2.0;
    
    return angle < max_half_fov;
}

bool TargetVoxelGrid::isInteriorUnknown(int idx) const {
    // 判断 UNKNOWN 体素是否为内部空心（被 OCCUPIED 包围）
    // 方法：检查 6 个主方向是否都被 OCCUPIED 阻挡
    
    if (idx < 0 || idx >= total_voxels_) return false;
    if (voxels_[idx].state != TargetVoxelType::UNKNOWN) return false;
    
    Eigen::Vector3d center = indexToPos(idx);
    
    // 6 个主方向
    std::vector<Eigen::Vector3d> directions = {
        {1, 0, 0}, {-1, 0, 0},
        {0, 1, 0}, {0, -1, 0},
        {0, 0, 1}, {0, 0, -1}
    };
    
    int blocked_count = 0;
    double max_probe_dist = size_.maxCoeff();
    
    for (const auto& dir : directions) {
        bool hit_occupied = false;
        
        // 沿该方向投射，检查是否先碰到 OCCUPIED 还是先出界
        for (double d = resolution_; d < max_probe_dist; d += resolution_) {
            Eigen::Vector3d probe = center + dir * d;
            int probe_idx = posToIndex(probe);
            
            if (probe_idx < 0) {
                // 出界了，说明这个方向没有被 OCCUPIED 阻挡
                break;
            }
            
            if (voxels_[probe_idx].state == TargetVoxelType::OCCUPIED) {
                hit_occupied = true;
                break;
            }
        }
        
        if (hit_occupied) blocked_count++;
    }
    
    // 如果 6 个方向都被阻挡 → 是内部空心
    return blocked_count == 6;
}

std::vector<Eigen::Vector3d> TargetVoxelGrid::sampleRayDirections(
    double yaw, double pitch, double hfov, double vfov,
    int num_h, int num_v) const 
{
    std::vector<Eigen::Vector3d> dirs;
    dirs.reserve(num_h * num_v);
    
    // 相机朝向的基准向量
    Eigen::Vector3d forward(cos(yaw) * cos(pitch), sin(yaw) * cos(pitch), sin(pitch));
    
    // 构建局部坐标系
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d right = forward.cross(up).normalized();
    if (right.norm() < 0.01) {
        right = Eigen::Vector3d::UnitX();
    }
    Eigen::Vector3d local_up = right.cross(forward).normalized();
    
    // 在 FOV 内均匀采样
    for (int i = 0; i < num_h; ++i) {
        double h_angle = -hfov / 2.0 + hfov * i / std::max(1, num_h - 1);
        for (int j = 0; j < num_v; ++j) {
            double v_angle = -vfov / 2.0 + vfov * j / std::max(1, num_v - 1);
            
            // 旋转基准向量
            Eigen::Vector3d dir = forward * cos(h_angle) * cos(v_angle)
                                + right * sin(h_angle) * cos(v_angle)
                                + local_up * sin(v_angle);
            dirs.push_back(dir.normalized());
        }
    }
    
    return dirs;
}

bool TargetVoxelGrid::rayBBoxIntersection(
    const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
    Eigen::Vector3d& entry_point, Eigen::Vector3d& exit_point) const 
{
    // 计算目标 BBox 边界
    Eigen::Vector3d bbox_min = center_ - size_ / 2.0;
    Eigen::Vector3d bbox_max = center_ + size_ / 2.0;
    
    double t_min = -1e9;
    double t_max = 1e9;
    
    for (int i = 0; i < 3; ++i) {
        if (std::abs(dir[i]) > 1e-6) {
            double t1 = (bbox_min[i] - origin[i]) / dir[i];
            double t2 = (bbox_max[i] - origin[i]) / dir[i];
            if (t1 > t2) std::swap(t1, t2);
            t_min = std::max(t_min, t1);
            t_max = std::min(t_max, t2);
        } else {
            // 射线与该轴平行
            if (origin[i] < bbox_min[i] || origin[i] > bbox_max[i]) {
                return false;  // 射线不穿过 BBox
            }
        }
    }
    
    if (t_min > t_max || t_max < 0) {
        return false;  // 无交点或在射线反方向
    }
    
    // 确保 t_min >= 0（从射线起点开始）
    if (t_min < 0) t_min = 0;
    
    entry_point = origin + dir * t_min;
    exit_point = origin + dir * t_max;
    
    return true;
}

} // namespace rog_map
