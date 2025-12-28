/**
 * @file frontier_viewpoint_generator.cpp
 * @brief 前沿视点生成器实现
 * 
 * 核心流程：
 * 1. 基于子网格分区获取前沿点
 * 2. 在每个子网格内聚类生成前沿簇
 * 3. 对大簇进行分裂
 * 4. 圆周采样生成探索视点
 * 5. 去重滤波得到最终视点集
 */

#include "rog_map/frontier_viewpoint_generator.h"
#include <queue>

namespace rog_map {

// ==================== 构造函数 ====================

FrontierViewpointGenerator::FrontierViewpointGenerator(
    GlobalMap::Ptr global_map,
    const ViewpointGeneratorConfig& config)
    : global_map_(global_map)
    , config_(config)
{
}

void FrontierViewpointGenerator::updateUAVState(const UAVState& state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    uav_state_ = state;
}

// ==================== 主接口 ====================

bool FrontierViewpointGenerator::generate(
    std::vector<FrontierCluster>& clusters,
    std::vector<ExplorationViewpoint>& viewpoints)
{
    clusters.clear();
    viewpoints.clear();
    
    if (!global_map_) {
        ROS_WARN("[ViewpointGen] No global_map!");
        return false;
    }
    
    // Step 1: 获取按子网格分组的前沿点
    std::vector<std::pair<int, std::vector<Eigen::Vector3f>>> subgrid_frontiers;
    global_map_->getFrontierPositionsBySubGrid(subgrid_frontiers);
    
    if (subgrid_frontiers.empty()) {
        ROS_DEBUG("[ViewpointGen] No frontier subgrids");
        return false;
    }
    
    ROS_INFO("[ViewpointGen] Processing %lu subgrids with frontiers", subgrid_frontiers.size());
    
    // Step 2: 在每个子网格内聚类
    int global_cluster_id = 0;
    for (const auto& sg_pair : subgrid_frontiers) {
        int sg_id = sg_pair.first;
        const auto& points = sg_pair.second;
        
        if (points.size() < static_cast<size_t>(config_.min_cluster_size)) {
            continue;
        }
        
        std::vector<FrontierCluster> sg_clusters;
        clusterInSubgrid(points, sg_id, sg_clusters);
        
        for (auto& c : sg_clusters) {
            c.cluster_id = global_cluster_id++;
            clusters.push_back(c);
        }
    }
    
    if (clusters.empty()) {
        ROS_DEBUG("[ViewpointGen] No clusters formed");
        return false;
    }
    
    ROS_INFO("[ViewpointGen] Formed %lu clusters", clusters.size());
    
    // Step 3: 分裂大簇
    splitLargeClusters(clusters);
    ROS_INFO("[ViewpointGen] After split: %lu clusters", clusters.size());
    
    // 限制最大簇数
    if (static_cast<int>(clusters.size()) > config_.max_clusters) {
        // 按大小排序，保留最大的
        std::sort(clusters.begin(), clusters.end(),
            [](const FrontierCluster& a, const FrontierCluster& b) {
                return a.size() > b.size();
            });
        clusters.resize(config_.max_clusters);
    }
    
    // Step 4: 为每个簇采样视点
    for (const auto& cluster : clusters) {
        sampleViewpoints(cluster, viewpoints);
    }
    
    ROS_INFO("[ViewpointGen] Generated %lu candidate viewpoints", viewpoints.size());
    
    // Step 5: 去重滤波
    filterViewpoints(viewpoints);
    
    ROS_INFO("[ViewpointGen] Final: %lu viewpoints from %lu clusters", 
             viewpoints.size(), clusters.size());
    
    return !viewpoints.empty();
}

// ==================== Step 1: 子网格内聚类 ====================

void FrontierViewpointGenerator::clusterInSubgrid(
    const std::vector<Eigen::Vector3f>& points,
    int subgrid_id,
    std::vector<FrontierCluster>& clusters)
{
    clusters.clear();
    if (points.empty()) return;
    
    const float eps = config_.cluster_eps;
    const int min_pts = config_.min_cluster_size;
    
    // BFS 区域增长聚类
    std::vector<bool> visited(points.size(), false);
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;
        
        // BFS
        std::vector<size_t> cluster_indices;
        std::queue<size_t> queue;
        queue.push(i);
        visited[i] = true;
        
        while (!queue.empty()) {
            size_t current = queue.front();
            queue.pop();
            cluster_indices.push_back(current);
            
            // 查找邻居
            for (size_t j = 0; j < points.size(); ++j) {
                if (visited[j]) continue;
                if ((points[j] - points[current]).norm() <= eps) {
                    visited[j] = true;
                    queue.push(j);
                }
            }
        }
        
        // 检查簇大小
        if (static_cast<int>(cluster_indices.size()) >= min_pts) {
            FrontierCluster cluster;
            cluster.subgrid_id = subgrid_id;
            cluster.point_count = static_cast<int>(cluster_indices.size());
            
            // 保存前沿点位置（用于可视化）
            cluster.points.reserve(cluster_indices.size());
            for (size_t idx : cluster_indices) {
                cluster.points.push_back(points[idx]);
            }
            
            // 计算中心
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (const auto& pt : cluster.points) {
                sum += pt;
            }
            cluster.centroid = sum / static_cast<float>(cluster.points.size());
            
            // 计算范围（用于判断是否需要分裂）
            float max_dist = 0.0f;
            for (const auto& pt : cluster.points) {
                float d = (pt - cluster.centroid).norm();
                max_dist = std::max(max_dist, d);
            }
            cluster.extent = max_dist;
            
            clusters.push_back(cluster);
        }
    }
}

// ==================== Step 2: 分裂大簇 ====================

void FrontierViewpointGenerator::splitLargeClusters(
    std::vector<FrontierCluster>& clusters)
{
    std::vector<FrontierCluster> result;
    
    for (const auto& cluster : clusters) {
        // 判断是否需要分裂
        if (cluster.extent > config_.cluster_split_threshold && !cluster.points.empty()) {
            // 按X坐标分裂：将点分成两组
            float split_x = cluster.centroid.x();
            
            FrontierCluster c1, c2;
            c1.subgrid_id = cluster.subgrid_id;
            c2.subgrid_id = cluster.subgrid_id;
            
            // 按X坐标分配点
            for (const auto& pt : cluster.points) {
                if (pt.x() < split_x) {
                    c1.points.push_back(pt);
                } else {
                    c2.points.push_back(pt);
                }
            }
            
            // 重新计算c1的属性
            if (!c1.points.empty()) {
                c1.point_count = static_cast<int>(c1.points.size());
                Eigen::Vector3f sum = Eigen::Vector3f::Zero();
                for (const auto& pt : c1.points) sum += pt;
                c1.centroid = sum / static_cast<float>(c1.points.size());
                float max_dist = 0.0f;
                for (const auto& pt : c1.points) {
                    max_dist = std::max(max_dist, (pt - c1.centroid).norm());
                }
                c1.extent = max_dist;
                result.push_back(c1);
            }
            
            // 重新计算c2的属性
            if (!c2.points.empty()) {
                c2.point_count = static_cast<int>(c2.points.size());
                Eigen::Vector3f sum = Eigen::Vector3f::Zero();
                for (const auto& pt : c2.points) sum += pt;
                c2.centroid = sum / static_cast<float>(c2.points.size());
                float max_dist = 0.0f;
                for (const auto& pt : c2.points) {
                    max_dist = std::max(max_dist, (pt - c2.centroid).norm());
                }
                c2.extent = max_dist;
                result.push_back(c2);
            }
        } else {
            result.push_back(cluster);
        }
    }
    
    clusters = std::move(result);
    
    // 重新分配ID
    for (size_t i = 0; i < clusters.size(); ++i) {
        clusters[i].cluster_id = static_cast<int>(i);
    }
}

// ==================== Step 3: 圆周采样视点（分层过滤） ====================

void FrontierViewpointGenerator::sampleViewpoints(
    const FrontierCluster& cluster,
    std::vector<ExplorationViewpoint>& viewpoints)
{
    // 固定视点高度
    float viewpoint_z = config_.viewpoint_height;
    
    // 采样半径范围
    float dr = (config_.candidate_rmax - config_.candidate_rmin) / 
               std::max(1.0f, static_cast<float>(config_.candidate_rnum - 1));
    
    std::vector<ExplorationViewpoint> layer1_passed;  // 第1层过滤后
    std::vector<ExplorationViewpoint> layer2_passed;  // 第2层过滤后
    
    // ========== 第1层：快速过滤（计算量小） ==========
    // 检查：位置安全、距离范围、FOV角度
    for (float rc = config_.candidate_rmin; rc <= config_.candidate_rmax + 1e-3f; rc += dr) {
        for (float phi = -PI; phi < PI; phi += config_.candidate_dphi) {
            Eigen::Vector3f sample_pos(
                cluster.centroid.x() + rc * std::cos(phi),
                cluster.centroid.y() + rc * std::sin(phi),
                viewpoint_z
            );
            
            // 1.1 位置安全检查
            if (!isPositionSafe(sample_pos)) {
                continue;
            }
            
            // 计算朝向（朝向簇中心）
            Eigen::Vector3f to_cluster = cluster.centroid - sample_pos;
            float dist = to_cluster.norm();
            float yaw = std::atan2(to_cluster.y(), to_cluster.x());
            
            // 1.2 距离范围检查
            if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) {
                continue;
            }
            
            // 1.3 FOV角度检查（簇中心是否在视锥体内）
            float half_fov_h = config_.sensor_fov_h * 0.5f;
            float half_fov_v = config_.sensor_fov_v * 0.5f;
            
            float vertical_angle = std::atan2(to_cluster.z(), 
                std::sqrt(to_cluster.x()*to_cluster.x() + to_cluster.y()*to_cluster.y()));
            
            if (std::abs(vertical_angle) > half_fov_v) {
                continue;  // 超出垂直FOV
            }
            
            // 1.4 快速评分（基于距离和角度）
            float dist_score = std::exp(-0.5f * std::pow((dist - config_.optimal_distance) / 
                                                          (config_.sensor_range_max * 0.3f), 2));
            float angle_score = 1.0f - (std::abs(vertical_angle) / half_fov_v);
            float quick_score = 0.6f * dist_score + 0.4f * angle_score;
            
            ExplorationViewpoint vp;
            vp.position = sample_pos;
            vp.yaw = yaw;
            vp.pitch = 0.0f;
            vp.cluster_id = cluster.cluster_id;
            vp.score = quick_score;
            vp.is_valid = true;
            
            layer1_passed.push_back(vp);
        }
    }
    
    if (layer1_passed.empty()) {
        return;  // 没有通过第1层的视点
    }
    
    // 按快速评分排序，只保留 top-K 进入第2层
    std::sort(layer1_passed.begin(), layer1_passed.end(),
        [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
            return a.score > b.score;
        });
    
    int top_k_layer2 = std::min(10, static_cast<int>(layer1_passed.size()));
    
    // ========== 第2层：可见性检查（中等计算量） ==========
    for (int i = 0; i < top_k_layer2; ++i) {
        auto& vp = layer1_passed[i];
        
        // 视锥体多射线可见性检查
        float visibility = computeVisibilityScore(vp.position, cluster);
        
        if (visibility < 0.3f) {
            continue;  // 可见性太低，跳过
        }
        
        // 更新评分（加入可见性）
        vp.score = vp.score * 0.7f + visibility * 0.3f;
        layer2_passed.push_back(vp);
    }
    
    if (layer2_passed.empty()) {
        return;  // 没有通过第2层的视点
    }
    
    // 按评分排序
    std::sort(layer2_passed.begin(), layer2_passed.end(),
        [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
            return a.score > b.score;
        });
    
    int top_k_layer3 = std::min(3, static_cast<int>(layer2_passed.size()));
    
    // ========== 第3层：轨迹可达验证（高计算量） ==========
    for (int i = 0; i < top_k_layer3; ++i) {
        auto& vp = layer2_passed[i];
        
        // 快速A*搜索验证可达性
        if (checkPathReachable(uav_state_.position, vp.position)) {
            // 通过所有验证，添加到最终结果
            viewpoints.push_back(vp);
            break;  // 只需要一个最佳视点
        }
    }
}

// ==================== 覆盖评分 ====================

float FrontierViewpointGenerator::computeCoverageScore(
    const Eigen::Vector3f& viewpoint,
    float yaw,
    const FrontierCluster& cluster)
{
    // 1. 距离评分 - 评估视点到簇中心的距离是否在最优范围内
    Eigen::Vector3f to_cluster = cluster.centroid - viewpoint;
    float dist = to_cluster.norm();
    
    // 距离在 [sensor_range_min, sensor_range_max] 范围内
    if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) {
        return 0.0f;  // 超出感知范围
    }
    
    // ========== 2. 可见性检查（射线遮挡） ==========
    // 从视点向簇中心发射射线，检查是否有障碍物遮挡
    float visibility_score = computeVisibilityScore(viewpoint, cluster);
    if (visibility_score < 0.1f) {
        return 0.0f;  // 严重遮挡，不可见
    }
    
    // 最优距离评分 (高斯分布)
    float dist_score = std::exp(-0.5f * std::pow((dist - config_.optimal_distance) / 
                                                  (config_.sensor_range_max * 0.3f), 2));
    
    // 3. 角度评分 - 评估簇中心是否在视锥体内
    float angle_to_cluster = std::atan2(to_cluster.y(), to_cluster.x());
    float angle_diff = std::abs(angle_to_cluster - yaw);
    
    // 归一化角度差到 [0, PI]
    while (angle_diff > PI) angle_diff -= 2 * PI;
    angle_diff = std::abs(angle_diff);
    
    // 检查是否在水平FOV内
    float half_fov_h = config_.sensor_fov_h * 0.5f;
    if (angle_diff > half_fov_h) {
        return 0.0f;  // 簇中心不在水平FOV内
    }
    
    // 角度评分 (中心更高)
    float angle_score = 1.0f - (angle_diff / half_fov_h);
    
    // 4. 垂直角度评分 - 评估簇是否在垂直FOV内
    float vertical_angle = std::atan2(to_cluster.z(), 
                                      std::sqrt(to_cluster.x()*to_cluster.x() + 
                                                to_cluster.y()*to_cluster.y()));
    float half_fov_v = config_.sensor_fov_v * 0.5f;
    
    if (std::abs(vertical_angle) > half_fov_v) {
        return 0.0f;  // 簇中心不在垂直FOV内
    }
    
    float vertical_score = 1.0f - (std::abs(vertical_angle) / half_fov_v);
    
    // 5. 簇覆盖评分 - 评估簇的范围是否能被视锥体覆盖
    // 估算在观测距离处FOV能覆盖的范围
    float coverage_width = 2.0f * dist * std::tan(half_fov_h);
    float coverage_height = 2.0f * dist * std::tan(half_fov_v);
    
    // 如果簇的范围小于覆盖范围，则得分更高
    float extent_ratio = cluster.extent / std::min(coverage_width, coverage_height);
    float extent_score = std::exp(-extent_ratio);  // 簇越小相对于覆盖范围，评分越高
    
    // 综合评分（包含可见性）
    float total_score = 0.25f * dist_score + 
                        0.25f * angle_score + 
                        0.15f * vertical_score + 
                        0.15f * extent_score +
                        0.20f * visibility_score;  // 可见性权重
    
    return total_score;
}

// ==================== 可见性评分（视锥体多射线） ====================

float FrontierViewpointGenerator::computeVisibilityScore(
    const Eigen::Vector3f& viewpoint,
    const FrontierCluster& cluster)
{
    if (!global_map_) return 1.0f;
    
    Eigen::Vector3f to_cluster = cluster.centroid - viewpoint;
    float dist = to_cluster.norm();
    if (dist < 0.1f) return 1.0f;
    
    // 主方向（视点到簇中心）
    Eigen::Vector3f main_dir = to_cluster.normalized();
    float main_yaw = std::atan2(main_dir.y(), main_dir.x());
    float main_pitch = std::atan2(main_dir.z(), 
                                   std::sqrt(main_dir.x()*main_dir.x() + main_dir.y()*main_dir.y()));
    
    // 视锥体参数
    float half_fov_h = config_.sensor_fov_h * 0.5f;
    float half_fov_v = config_.sensor_fov_v * 0.5f;
    
    // 在视锥体内采样射线
    // 采样密度：水平5个，垂直3个 = 15根射线
    const int num_h_samples = 5;
    const int num_v_samples = 3;
    float h_step = config_.sensor_fov_h / (num_h_samples - 1);
    float v_step = config_.sensor_fov_v / (num_v_samples - 1);
    
    int visible_rays = 0;
    int total_rays = 0;
    
    for (int ih = 0; ih < num_h_samples; ++ih) {
        float yaw_offset = -half_fov_h + ih * h_step;
        
        for (int iv = 0; iv < num_v_samples; ++iv) {
            float pitch_offset = -half_fov_v + iv * v_step;
            
            // 计算当前射线方向
            float ray_yaw = main_yaw + yaw_offset;
            float ray_pitch = main_pitch + pitch_offset;
            
            Eigen::Vector3f ray_dir(
                std::cos(ray_pitch) * std::cos(ray_yaw),
                std::cos(ray_pitch) * std::sin(ray_yaw),
                std::sin(ray_pitch)
            );
            
            // 计算射线与簇区域的交点距离
            // 使用簇的 extent 作为目标距离
            float ray_dist = std::min(dist + cluster.extent, config_.sensor_range_max);
            
            // 检查这根射线是否被遮挡
            bool ray_visible = checkRayClear(viewpoint, ray_dir, ray_dist);
            
            total_rays++;
            if (ray_visible) {
                visible_rays++;
            }
        }
    }
    
    if (total_rays == 0) return 1.0f;
    
    // 可见性评分 = 可见射线比例
    float visibility = static_cast<float>(visible_rays) / total_rays;
    
    return visibility;
}

bool FrontierViewpointGenerator::checkRayClear(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& direction,
    float max_dist)
{
    if (!global_map_) return true;
    
    float step = 0.15f;  // 15cm步长
    int num_steps = static_cast<int>(max_dist / step);
    
    // 从起点沿方向发射射线
    for (int i = 1; i < num_steps; ++i) {
        Eigen::Vector3f check_pos = start + direction * (i * step);
        
        if (!global_map_->isInGlobalMapRange(check_pos)) {
            continue;
        }
        
        // 如果遇到占据点，射线被阻挡
        if (global_map_->isOccupied(check_pos)) {
            return false;
        }
    }
    
    return true;
}

bool FrontierViewpointGenerator::checkRayVisibility(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end)
{
    if (!global_map_) return true;
    
    Eigen::Vector3f dir = end - start;
    float dist = dir.norm();
    if (dist < 0.1f) return true;
    
    dir.normalize();
    float step = 0.1f;
    int num_steps = static_cast<int>(dist / step);
    
    // 从起点向终点发射射线（不包括终点本身，因为终点可能是前沿）
    for (int i = 1; i < num_steps; ++i) {
        Eigen::Vector3f check_pos = start + dir * (i * step);
        
        if (!global_map_->isInGlobalMapRange(check_pos)) {
            continue;
        }
        
        // 如果遇到占据点，视线被阻挡
        if (global_map_->isOccupied(check_pos)) {
            return false;
        }
    }
    
    return true;
}

// ==================== Step 4: 去重滤波 ====================

void FrontierViewpointGenerator::filterViewpoints(
    std::vector<ExplorationViewpoint>& viewpoints)
{
    if (viewpoints.size() <= 1) return;
    
    // 按评分排序
    std::sort(viewpoints.begin(), viewpoints.end(),
        [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
            return a.score > b.score;
        });
    
    std::vector<ExplorationViewpoint> filtered;
    filtered.reserve(viewpoints.size());
    
    for (const auto& vp : viewpoints) {
        bool too_close = false;
        for (const auto& kept : filtered) {
            if ((vp.position - kept.position).norm() < config_.min_viewpoint_distance) {
                too_close = true;
                break;
            }
        }
        if (!too_close) {
            filtered.push_back(vp);
        }
    }
    
    viewpoints = std::move(filtered);
}

// ==================== 辅助函数 ====================

bool FrontierViewpointGenerator::isPositionSafe(const Eigen::Vector3f& pos)
{
    if (!global_map_) return true;
    
    // 检查是否在地图范围内
    if (!global_map_->isInGlobalMapRange(pos)) {
        return false;
    }
    
    // 检查是否为自由空间
    if (!isPositionFree(pos)) {
        return false;
    }
    
    // 检查周围安全距离
    float check_radius = config_.uav_radius + config_.safety_margin;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            Eigen::Vector3f check_pos = pos + Eigen::Vector3f(
                dx * check_radius, dy * check_radius, 0.0f);
            if (global_map_->isInGlobalMapRange(check_pos) && 
                global_map_->isOccupied(check_pos)) {
                return false;
            }
        }
    }
    
    return true;
}

bool FrontierViewpointGenerator::isPositionFree(const Eigen::Vector3f& pos)
{
    if (!global_map_) return true;
    GlobalVoxelState state = global_map_->getVoxelState(pos);
    return VoxelStateBits::isFree(state);
}

} // namespace rog_map


