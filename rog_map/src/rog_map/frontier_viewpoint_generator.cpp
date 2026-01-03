/**
 * @file frontier_viewpoint_generator.cpp
 * @brief 前沿视点生成器实现
 * 
 * 核心流程（借鉴EDEN算法）：
 * 1. 沿无人机轨迹采集历史节点
 * 2. 获取前沿点并过滤（地面噪声+无人机周围）
 * 3. 将前沿区域关联到历史节点
 * 4. 同一历史节点关联的前沿区域合并聚类
 * 5. 对大簇进行分裂
 * 6. 圆周采样生成探索视点
 * 7. 去重滤波得到最终视点集
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
    
    // Step 0: 更新历史节点（沿无人机轨迹采集）
    updateHistoryNodes();
    
    // Step 1: 获取按子网格分组的前沿点
    std::vector<std::pair<int, std::vector<Eigen::Vector3f>>> subgrid_frontiers;
    global_map_->getFrontierPositionsBySubGrid(subgrid_frontiers);
    
    if (subgrid_frontiers.empty()) {
        ROS_DEBUG("[ViewpointGen] No frontier subgrids");
        return false;
    }
    
    // Step 2: 过滤前沿点（地面噪声 + 无人机周围）并关联到历史节点
    // 按历史节点分组前沿点
    std::map<int, std::vector<Eigen::Vector3f>> node_frontiers;
    size_t total_points_before = 0;
    size_t total_points_after = 0;
    
    // 清除旧的子网格关联信息
    associated_subgrids_.clear();
    
    for (auto& sg_pair : subgrid_frontiers) {
        int sg_id = sg_pair.first;
        auto& points = sg_pair.second;
        total_points_before += points.size();
        
        // 过滤前沿点
        filterFrontierPoints(points);
        total_points_after += points.size();
        
        if (points.empty()) continue;
        
        // 计算子网格中心和边界框
        Eigen::Vector3f sg_center = Eigen::Vector3f::Zero();
        Eigen::Vector3f bbox_min = points[0];
        Eigen::Vector3f bbox_max = points[0];
        for (const auto& p : points) {
            sg_center += p;
            bbox_min = bbox_min.cwiseMin(p);
            bbox_max = bbox_max.cwiseMax(p);
        }
        sg_center /= static_cast<float>(points.size());
        
        // 关联到最近的历史节点
        int node_id = findNearestHistoryNode(sg_center);
        if (node_id >= 0) {
            // 将前沿点添加到该节点
            node_frontiers[node_id].insert(
                node_frontiers[node_id].end(), 
                points.begin(), points.end());
            
            // 保存子网格关联信息（用于可视化）
            AssociatedSubgrid asg;
            asg.subgrid_id = sg_id;
            asg.history_node_id = node_id;
            asg.center = sg_center;
            asg.bbox_min = bbox_min;
            asg.bbox_max = bbox_max;
            asg.point_count = static_cast<int>(points.size());
            associated_subgrids_.push_back(asg);
        } else {
            // 如果没有历史节点，使用子网格ID作为临时分组
            node_frontiers[-sg_id - 1].insert(
                node_frontiers[-sg_id - 1].end(),
                points.begin(), points.end());
            
            // 也保存这些子网格的信息
            AssociatedSubgrid asg;
            asg.subgrid_id = sg_id;
            asg.history_node_id = -1;  // 未关联
            asg.center = sg_center;
            asg.bbox_min = bbox_min;
            asg.bbox_max = bbox_max;
            asg.point_count = static_cast<int>(points.size());
            associated_subgrids_.push_back(asg);
        }
    }
    
    ROS_INFO("[ViewpointGen] Filtered frontier points: %lu -> %lu (removed %lu)",
             total_points_before, total_points_after, 
             total_points_before - total_points_after);
    ROS_INFO("[ViewpointGen] History nodes: %lu, Frontier groups: %lu", 
             history_nodes_.size(), node_frontiers.size());
    
    // Step 3: 在每个历史节点关联的前沿区域内聚类
    int global_cluster_id = 0;
    for (const auto& nf_pair : node_frontiers) {
        int node_id = nf_pair.first;
        const auto& points = nf_pair.second;
        
        if (points.size() < static_cast<size_t>(config_.min_cluster_size)) {
            continue;
        }
        
        std::vector<FrontierCluster> node_clusters;
        clusterByHistoryNode(points, node_id, node_clusters);
        
        for (auto& c : node_clusters) {
            c.cluster_id = global_cluster_id++;
            c.history_node_id = node_id;
            clusters.push_back(c);
        }
    }
    
    if (clusters.empty()) {
        ROS_DEBUG("[ViewpointGen] No clusters formed");
    return false;
}

    ROS_INFO("[ViewpointGen] Formed %lu clusters from %lu history node groups", 
             clusters.size(), node_frontiers.size());
    
    // Step 4: 分裂大簇
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
    
    // Step 5: 为每个簇采样视点（L1快速过滤 + L2可见性检查）
    for (const auto& cluster : clusters) {
        sampleViewpoints(cluster, viewpoints);
    }
    
    ROS_INFO("[ViewpointGen] After L1+L2: %lu candidate viewpoints", viewpoints.size());
    
    // Step 6: 去重滤波
    filterViewpoints(viewpoints);
    
    ROS_INFO("[ViewpointGen] After dedup: %lu viewpoints", viewpoints.size());
    
    // Step 7: 视点聚类与Yaw优化
    // 对相邻视点进行聚类，然后优化每个簇内视点的yaw角，避免对射情况
    optimizeViewpointYaw(viewpoints);
    
    ROS_INFO("[ViewpointGen] Final: %lu viewpoints from %lu clusters", 
             viewpoints.size(), clusters.size());
    
    return !viewpoints.empty();
}

// ==================== 历史节点管理 ====================

void FrontierViewpointGenerator::updateHistoryNodes()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    Eigen::Vector3f current_pos = uav_state_.position;
    
    // 第一次创建历史节点
    if (!first_node_created_) {
        HistoryNode node(next_node_id_++, current_pos, ros::Time::now().toSec());
        history_nodes_.push_back(node);
        last_node_position_ = current_pos;
        first_node_created_ = true;
        ROS_INFO("[ViewpointGen] Created first history node at (%.2f, %.2f, %.2f)",
                 current_pos.x(), current_pos.y(), current_pos.z());
        return;
    }
    
    // 检查是否需要添加新节点（距离超过阈值）
    float dist = (current_pos - last_node_position_).norm();
    if (dist >= config_.history_node_distance) {
        HistoryNode node(next_node_id_++, current_pos, ros::Time::now().toSec());
        history_nodes_.push_back(node);
        last_node_position_ = current_pos;
        ROS_DEBUG("[ViewpointGen] Created history node %d at (%.2f, %.2f, %.2f)",
                  node.node_id, current_pos.x(), current_pos.y(), current_pos.z());
    }
}

int FrontierViewpointGenerator::findNearestHistoryNode(const Eigen::Vector3f& subgrid_center)
{
    if (history_nodes_.empty()) return -1;
    
    // 关键设计：每个子网格只会关联到距离最近的唯一一个历史节点
    // 即使多个节点都在 node_frontier_radius 范围内，也只选择最近的那个
    int nearest_id = -1;
    float min_dist = config_.node_frontier_radius;  // 初始阈值：只关联在此半径内的节点
    
    for (const auto& node : history_nodes_) {
        // 只考虑XY距离（高度差不影响关联）
        float dist = (node.position - subgrid_center).head<2>().norm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest_id = node.node_id;  // 更新为更近的节点
        }
    }
    
    // 如果 nearest_id >= 0，说明找到了一个在半径内且距离最近的节点
    // 该子网格将唯一关联到这个节点
    return nearest_id;
}

// ==================== 前沿点过滤 ====================

void FrontierViewpointGenerator::filterFrontierPoints(std::vector<Eigen::Vector3f>& points)
{
    if (points.empty()) return;
    
    float ground_threshold = config_.ground_height + config_.ground_offset;
    
    std::vector<Eigen::Vector3f> filtered;
    filtered.reserve(points.size());
    
    for (const auto& p : points) {
        // 过滤1：地面噪声（高度太低）
        if (p.z() < ground_threshold) {
            continue;
        }
        
        // 过滤2：无人机周围安全包围盒
        if (isInUAVSafetyBox(p)) {
            continue;
        }
        
        filtered.push_back(p);
    }
    
    points = std::move(filtered);
}

bool FrontierViewpointGenerator::isInUAVSafetyBox(const Eigen::Vector3f& point)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    Eigen::Vector3f uav_pos = uav_state_.position;
    float box_half_xy = config_.uav_filter_box_xy;
    
    // XY平面检查
    if (std::abs(point.x() - uav_pos.x()) > box_half_xy) return false;
    if (std::abs(point.y() - uav_pos.y()) > box_half_xy) return false;
    
    // Z轴：从地面到全局地图顶部
    float z_min = config_.ground_height;
    float z_max = config_.ground_height + config_.global_map_height;
    if (point.z() < z_min || point.z() > z_max) return false;
    
    // 在包围盒内
    return true;
}

// ==================== 历史节点聚类 ====================

void FrontierViewpointGenerator::clusterByHistoryNode(
    const std::vector<Eigen::Vector3f>& points,
    int node_id,
    std::vector<FrontierCluster>& clusters)
{
    // 使用与clusterInSubgrid相同的聚类逻辑
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
                
                float dist = (points[j] - points[current]).norm();
                if (dist < eps) {
                    visited[j] = true;
                    queue.push(j);
                }
            }
        }
        
        // 检查簇大小
        if (static_cast<int>(cluster_indices.size()) >= min_pts) {
            FrontierCluster cluster;
            cluster.subgrid_id = -1;  // 不再使用子网格ID
            cluster.history_node_id = node_id;
            
            // 计算簇属性
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (size_t idx : cluster_indices) {
                sum += points[idx];
                cluster.points.push_back(points[idx]);
            }
            cluster.centroid = sum / static_cast<float>(cluster_indices.size());
            cluster.point_count = static_cast<int>(cluster_indices.size());
            
            // 计算范围（extent）
            float max_dist = 0.0f;
            for (size_t idx : cluster_indices) {
                float dist = (points[idx] - cluster.centroid).norm();
                if (dist > max_dist) max_dist = dist;
            }
            cluster.extent = max_dist;
            
            // 估算法向量（简化：使用水平方向到质心的向量）
            Eigen::Vector3f to_centroid = cluster.centroid - uav_state_.position;
            to_centroid.z() = 0.0f;
            if (to_centroid.norm() > 0.1f) {
                cluster.normal = to_centroid.normalized();
            } else {
                cluster.normal = Eigen::Vector3f::UnitX();
            }
            
            clusters.push_back(cluster);
        }
    }
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
        
        if (visibility < config_.min_visibility_ratio) {
            continue;  // 可见性太低，跳过
        }
        
        // 更新评分（加入可见性，权重30%）
        vp.score = vp.score * 0.7f + visibility * 0.3f;
        layer2_passed.push_back(vp);
    }
    
    if (layer2_passed.empty()) {
        return;  // 没有通过第2层的视点
    }
    
    // 按评分排序，选择最佳视点
    std::sort(layer2_passed.begin(), layer2_passed.end(),
        [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
            return a.score > b.score;
        });
    
    // 为该簇选择最佳视点（L3可达性验证在后续统一进行）
    if (!layer2_passed.empty()) {
        viewpoints.push_back(layer2_passed[0]);
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

// ==================== Step 6: 视点Yaw优化 ====================

void FrontierViewpointGenerator::optimizeViewpointYaw(
    std::vector<ExplorationViewpoint>& viewpoints)
{
    if (viewpoints.size() < 2) return;
    
    // 视点聚类阈值（相邻视点距离小于此值则归为一簇）
    const float cluster_dist = config_.min_viewpoint_distance * 3.0f;
    
    // 1. 对视点进行聚类（简单的贪心聚类）
    std::vector<std::vector<int>> vp_clusters;
    std::vector<bool> assigned(viewpoints.size(), false);
    
    for (size_t i = 0; i < viewpoints.size(); ++i) {
        if (assigned[i]) continue;
        
        std::vector<int> cluster;
        cluster.push_back(static_cast<int>(i));
        assigned[i] = true;
        
        // 找到所有与当前视点相邻的视点
        for (size_t j = i + 1; j < viewpoints.size(); ++j) {
            if (assigned[j]) continue;
            
            float dist = (viewpoints[j].position - viewpoints[i].position).norm();
            if (dist < cluster_dist) {
                cluster.push_back(static_cast<int>(j));
                assigned[j] = true;
            }
        }
        
        if (cluster.size() >= 2) {
            vp_clusters.push_back(cluster);
        }
    }
    
    // 2. 对每个簇内的视点优化yaw
    for (const auto& cluster : vp_clusters) {
        if (cluster.size() < 2) continue;
        
        // 计算簇的中心位置
        Eigen::Vector3f cluster_center = Eigen::Vector3f::Zero();
        for (int idx : cluster) {
            cluster_center += viewpoints[idx].position;
        }
        cluster_center /= static_cast<float>(cluster.size());
        
        // 收集每个视点需要观测的目标簇中心
        std::vector<Eigen::Vector3f> target_centers;
        std::vector<float> original_yaws;
        
        for (int idx : cluster) {
            // 根据cluster_id找到对应的前沿簇中心
            // 这里简单使用原始yaw指向的方向推算目标位置
            float yaw = viewpoints[idx].yaw;
            float target_dist = config_.optimal_distance;
            Eigen::Vector3f target = viewpoints[idx].position + 
                Eigen::Vector3f(std::cos(yaw) * target_dist, 
                               std::sin(yaw) * target_dist, 0.0f);
            target_centers.push_back(target);
            original_yaws.push_back(yaw);
        }
        
        // 计算需要覆盖的yaw角范围
        std::vector<float> yaw_to_targets;
        for (const auto& target : target_centers) {
            Eigen::Vector3f dir = target - cluster_center;
            float yaw = std::atan2(dir.y(), dir.x());
            yaw_to_targets.push_back(yaw);
        }
        
        // 对yaw进行排序，找出需要覆盖的角度范围
        std::vector<float> sorted_yaws = yaw_to_targets;
        std::sort(sorted_yaws.begin(), sorted_yaws.end());
        
        // 计算相邻yaw之间的gap，找到最大gap（作为起始点）
        float max_gap = 0.0f;
        int gap_start_idx = 0;
        for (size_t i = 0; i < sorted_yaws.size(); ++i) {
            size_t next = (i + 1) % sorted_yaws.size();
            float gap = sorted_yaws[next] - sorted_yaws[i];
            if (next == 0) gap += 2 * PI;  // 跨越-PI/PI边界
            if (gap > max_gap) {
                max_gap = gap;
                gap_start_idx = static_cast<int>(next);
            }
        }
        
        // 3. 重新分配yaw，使相邻视点的yaw平滑过渡
        // 按照视点在簇中的位置（按角度排序）分配yaw
        std::vector<std::pair<float, int>> angle_idx_pairs;
        for (int idx : cluster) {
            Eigen::Vector3f dir = viewpoints[idx].position - cluster_center;
            float angle = std::atan2(dir.y(), dir.x());
            angle_idx_pairs.push_back({angle, idx});
        }
        std::sort(angle_idx_pairs.begin(), angle_idx_pairs.end());
        
        // 根据视点位置的角度顺序，平滑分配yaw
        // 确保相邻视点的yaw变化最小
        for (size_t i = 0; i < angle_idx_pairs.size(); ++i) {
            int idx = angle_idx_pairs[i].second;
            
            // 保持原始yaw指向目标，但检查是否需要调整
            // 如果与前一个视点的yaw差距太大，进行平滑
            if (i > 0) {
                int prev_idx = angle_idx_pairs[i-1].second;
                float prev_yaw = viewpoints[prev_idx].yaw;
                float curr_yaw = viewpoints[idx].yaw;
                
                // 计算yaw差值（考虑环绕）
                float yaw_diff = curr_yaw - prev_yaw;
                while (yaw_diff > PI) yaw_diff -= 2 * PI;
                while (yaw_diff < -PI) yaw_diff += 2 * PI;
                
                // 如果yaw差距超过90度（对射情况），尝试调整
                if (std::abs(yaw_diff) > PI * 0.5f) {
                    // 检查是否可以将yaw调整到同侧
                    // 方法：找到两个目标的中间方向
                    float mid_yaw = prev_yaw + yaw_diff * 0.5f;
                    
                    // 检查从当前位置看，是否可以用mid_yaw观测到目标
                    Eigen::Vector3f to_original_target = target_centers[i] - viewpoints[idx].position;
                    float original_yaw_to_target = std::atan2(to_original_target.y(), to_original_target.x());
                    
                    // 如果调整后的yaw与目标方向偏差不超过FOV一半，则调整
                    float fov_tolerance = config_.sensor_fov_h * 0.4f;
                    float adjusted_diff = mid_yaw - original_yaw_to_target;
                    while (adjusted_diff > PI) adjusted_diff -= 2 * PI;
                    while (adjusted_diff < -PI) adjusted_diff += 2 * PI;
                    
                    if (std::abs(adjusted_diff) < fov_tolerance) {
                        viewpoints[idx].yaw = mid_yaw;
                        ROS_DEBUG("[ViewpointGen] Adjusted yaw for vp %d: %.2f -> %.2f (smooth transition)",
                                  idx, original_yaws[i], mid_yaw);
                    }
                }
            }
        }
    }
    
    ROS_INFO("[ViewpointGen] Yaw optimization: processed %lu viewpoint clusters", vp_clusters.size());
}

// ==================== 辅助函数 ====================

bool FrontierViewpointGenerator::isPositionSafe(const Eigen::Vector3f& pos)
{
    if (!global_map_) return true;
    
    // 检查是否在地图范围内
    if (!global_map_->isInGlobalMapRange(pos)) {
        return false;
    }
    
    // 检查是否为自由空间（必须是已知的自由区域）
    if (!isPositionFree(pos)) {
        return false;
    }
    
    // 安全半径：无人机半径 + 安全边距
    float check_radius = config_.uav_radius + config_.safety_margin;
    float step = 0.2f;  // 地图分辨率（固定值，与global_map配置一致）
    int num_steps = static_cast<int>(check_radius / step) + 1;
    
    // 在安全半径内密集检查（XY平面 + 上下一点高度）
    for (int dx = -num_steps; dx <= num_steps; ++dx) {
        for (int dy = -num_steps; dy <= num_steps; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                Eigen::Vector3f check_pos = pos + Eigen::Vector3f(
                    dx * step, dy * step, dz * step * 0.5f);
                
                // 检查距离是否在安全半径内
                float dist_xy = std::sqrt(dx*dx + dy*dy) * step;
                if (dist_xy > check_radius) continue;
                
                if (!global_map_->isInGlobalMapRange(check_pos)) {
                    continue;  // 超出地图范围的点跳过
                }
                
                // 检查是否有障碍物
                if (global_map_->isOccupied(check_pos)) {
                    return false;  // 有障碍物，不安全
                }
                
                // 检查是否为未知区域（未知区域也不安全）
                if (global_map_->isUnknown(check_pos)) {
                    return false;  // 未知区域，不安全
                }
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


