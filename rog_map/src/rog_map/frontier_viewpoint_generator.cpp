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
    
    // Step 1: 获取按子网格分组的前沿点（包含子网格几何信息）
    std::vector<rog_map::GlobalMap::SubGridFrontierInfo> subgrid_frontiers;
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
    
    for (auto& sg_info : subgrid_frontiers) 
    {
        int sg_id = sg_info.subgrid_id;
        auto& points = sg_info.frontier_points;
        total_points_before += points.size();
        
        // 过滤前沿点
        filterFrontierPoints(points);
        total_points_after += points.size();
        
        if (points.empty()) continue;
        
        // 使用子网格的几何中心（而不是前沿点中心）来关联历史节点
        // 这样可视化时历史节点指向的是子网格区域中心
        Eigen::Vector3f sg_center = sg_info.subgrid_center;
        
        // 关联到最近的历史节点
        int node_id = findNearestHistoryNode(sg_center);
        if (node_id >= 0) {
            // 将前沿点添加到该节点
            node_frontiers[node_id].insert(
                node_frontiers[node_id].end(), 
                points.begin(), points.end());
            
            // 保存子网格关联信息（用于可视化）
            // 使用子网格的几何中心和边界，而不是前沿点的范围
            AssociatedSubgrid asg;
            asg.subgrid_id = sg_id;
            asg.history_node_id = node_id;
            asg.center = sg_info.subgrid_center;    // 子网格几何中心
            asg.bbox_min = sg_info.subgrid_min;     // 子网格边界
            asg.bbox_max = sg_info.subgrid_max;
            asg.point_count = static_cast<int>(points.size());
            associated_subgrids_.push_back(asg);
        }
        // 注意：未关联到历史节点的子网格不再生成探索视点
        // 这些区域会在无人机移动并创建新历史节点后被关联
    }
    
    ROS_INFO("[ViewpointGen] Filtered frontier points: %lu -> %lu (removed %lu)",
             total_points_before, total_points_after, 
             total_points_before - total_points_after);
    ROS_INFO("[ViewpointGen] History nodes: %lu, Frontier groups: %lu", 
             history_nodes_.size(), node_frontiers.size());
    
    // Step 3: 在每个历史节点关联的前沿区域内聚类
    int global_cluster_id = 0;
    for (const auto& nf_pair : node_frontiers) 
    {
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
    
    if (clusters.empty()) 
    {
        ROS_DEBUG("[ViewpointGen] No clusters formed");
    return false;
}

    ROS_INFO("[ViewpointGen] Formed %lu clusters from %lu history node groups", 
             clusters.size(), node_frontiers.size());
    
    // Step 4: 分裂大簇
    splitLargeClusters(clusters);
    ROS_INFO("[ViewpointGen] After split: %lu clusters", clusters.size());
    
    // Step 4.5: 过滤头顶区域簇
    // 策略：如果簇中心比无人机高且水平距离很近，认为是头顶区域，不生成探索视点
    {
        Eigen::Vector3f uav_pos = uav_state_.position;
        size_t before_count = clusters.size();
        
        clusters.erase(
            std::remove_if(clusters.begin(), clusters.end(),
                [&](const FrontierCluster& c) {
                    // 计算簇中心与无人机的高度差
                    float height_diff = c.centroid.z() - uav_pos.z();
                    
                    // 计算簇中心与无人机的水平距离
                    float horiz_dist = (c.centroid.head<2>() - uav_pos.head<2>()).norm();
                    
                    // 头顶区域判定：高度差超过阈值 且 水平距离小于阈值
                    bool is_overhead = (height_diff > config_.overhead_height_threshold) &&
                                       (horiz_dist < config_.overhead_distance_threshold);
                    
                    if (is_overhead) {
                        ROS_DEBUG("[ViewpointGen] Filtered overhead cluster %d: "
                                  "height_diff=%.2f (>%.2f), horiz_dist=%.2f (<%.2f)",
                                  c.cluster_id, height_diff, config_.overhead_height_threshold,
                                  horiz_dist, config_.overhead_distance_threshold);
                    }
                    return is_overhead;
                }),
            clusters.end());
        
        if (clusters.size() < before_count) {
            ROS_INFO("[ViewpointGen] Filtered %lu overhead clusters, remaining: %lu",
                     before_count - clusters.size(), clusters.size());
        }
    }
    
    // 限制最大簇数
    // 改进：按距离无人机的远近排序（近的优先），而不是按大小
    // 这样可以确保探索最近的区域，同时保留各个方向的簇
    if (static_cast<int>(clusters.size()) > config_.max_clusters) {
        Eigen::Vector3f uav_pos;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            uav_pos = uav_state_.position;
        }
        
        // 按距离排序，保留最近的
        std::sort(clusters.begin(), clusters.end(),
            [&uav_pos](const FrontierCluster& a, const FrontierCluster& b) {
                float dist_a = (a.centroid - uav_pos).head<2>().norm();
                float dist_b = (b.centroid - uav_pos).head<2>().norm();
                return dist_a < dist_b;
            });
        
        ROS_INFO("[ViewpointGen] Limiting clusters: %lu -> %d (keeping nearest)",
                 clusters.size(), config_.max_clusters);
        clusters.resize(config_.max_clusters);
    }
    
    // Step 5: 过滤已被覆盖的簇，然后为每个簇采样视点
    size_t covered_count = 0;
    size_t no_viewpoint_count = 0;
    for (const auto& cluster : clusters) {
        // ★★★ 关键：检查簇中心附近是否还有足够的未知体素 ★★★
        // 如果未知体素太少，说明这个区域已经被探索过了，跳过
        if (isClusterCovered(cluster)) {
            covered_count++;
            continue;
        }
        
        size_t before = viewpoints.size();
        sampleViewpoints(cluster, viewpoints);
        
        // 调试：记录没有产生视点的簇
        if (viewpoints.size() == before) {
            no_viewpoint_count++;
            ROS_DEBUG("[ViewpointGen] Cluster %d at (%.2f,%.2f,%.2f) produced no viewpoints",
                      cluster.cluster_id, cluster.centroid.x(), 
                      cluster.centroid.y(), cluster.centroid.z());
        }
    }
    
    if (covered_count > 0 || no_viewpoint_count > 0) {
        ROS_INFO("[ViewpointGen] Cluster stats: %lu covered, %lu no-viewpoint, %lu produced viewpoints",
                 covered_count, no_viewpoint_count, 
                 clusters.size() - covered_count - no_viewpoint_count);
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
    
    // 检查无人机是否已完成起飞（高度达到起飞高度的90%）
    float takeoff_threshold = config_.viewpoint_height * 0.9f;
    if (current_pos.z() < takeoff_threshold) {
        // 无人机还在起飞过程中，不创建历史节点
        ROS_DEBUG_THROTTLE(2.0, "[ViewpointGen] UAV still taking off (z=%.2f < %.2f), waiting...",
                           current_pos.z(), takeoff_threshold);
        return;
    }
    
    // 第一次创建历史节点（无人机已完成起飞）
    if (!first_node_created_) {
        HistoryNode node(next_node_id_++, current_pos, ros::Time::now().toSec());
        history_nodes_.push_back(node);
        first_node_created_ = true;
        ROS_INFO("[ViewpointGen] Created first history node at (%.2f, %.2f, %.2f) after takeoff",
                 current_pos.x(), current_pos.y(), current_pos.z());
        return;
    }
    
    // 检查是否需要添加新节点
    // 改进：与所有已采样历史节点的最短距离超过阈值时才采样
    // 而不是只与上一个节点比较
    float min_dist_to_existing = std::numeric_limits<float>::max();
    for (const auto& node : history_nodes_) {
        float dist = (current_pos - node.position).head<2>().norm();  // 只考虑XY距离
        if (dist < min_dist_to_existing) {
            min_dist_to_existing = dist;
        }
    }
    
    // 只有当与所有现有节点的距离都超过阈值时，才创建新节点
    if (min_dist_to_existing >= config_.history_node_distance) {
        HistoryNode node(next_node_id_++, current_pos, ros::Time::now().toSec());
        history_nodes_.push_back(node);
        ROS_INFO("[ViewpointGen] Created history node %d at (%.2f, %.2f, %.2f), min_dist=%.2f",
                 node.node_id, current_pos.x(), current_pos.y(), current_pos.z(),
                 min_dist_to_existing);
    }
}

int FrontierViewpointGenerator::findNearestHistoryNode(const Eigen::Vector3f& subgrid_center)
{
    if (history_nodes_.empty()) return -1;
    
    // 关键设计：每个子网格关联到距离最近的历史节点
    // 优先选择在 node_frontier_radius 范围内的节点
    // 如果没有在范围内的节点，则选择最近的节点（确保所有前沿区域都能被关联）
    int nearest_id = -1;
    float min_dist = std::numeric_limits<float>::max();
    int nearest_in_radius_id = -1;
    float min_dist_in_radius = config_.node_frontier_radius;
    
    for (const auto& node : history_nodes_) {
        // 只考虑XY距离（高度差不影响关联）
        float dist = (node.position - subgrid_center).head<2>().norm();
        
        // 记录最近的节点（不管距离）
        if (dist < min_dist) {
            min_dist = dist;
            nearest_id = node.node_id;
        }
        
        // 记录在半径内的最近节点
        if (dist < min_dist_in_radius) {
            min_dist_in_radius = dist;
            nearest_in_radius_id = node.node_id;
        }
    }
    
    // 优先返回在半径内的节点，如果没有则返回最近的节点
    // 这样确保所有前沿区域都能被关联到某个历史节点
    if (nearest_in_radius_id >= 0) {
        return nearest_in_radius_id;
    }
    
    // 备选：返回最近的节点（即使超出半径）
    // 这确保了远处的前沿区域也能被探索
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
        // 过滤：地面噪声（高度太低）
        if (p.z() < ground_threshold) {
            continue;
        }
        
        // 注意：无人机头顶区域的过滤已移至簇级别（overhead_height_threshold）
        // 这样可以更精确地过滤近处头顶区域，同时保留远处高处区域
        
        filtered.push_back(p);
    }
    
    points = std::move(filtered);
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

// ==================== 簇覆盖检查 ====================

bool FrontierViewpointGenerator::isClusterCovered(const FrontierCluster& cluster)
{
    // 检查簇中心附近是否还有足够的未知体素
    // 如果未知体素太少，说明这个区域已经被探索过了
    
    Eigen::Vector3f target_pos = cluster.centroid;
    const float check_radius = 2.0f;
    const int min_unknown_threshold = 10;  // 与 exploration_fsm 中的阈值一致
    
    int unknown_count = 0;
    for (float dx = -check_radius; dx <= check_radius; dx += 0.5f) {
        for (float dy = -check_radius; dy <= check_radius; dy += 0.5f) {
            for (float dz = -check_radius; dz <= check_radius; dz += 0.5f) {
                Eigen::Vector3f check_pos = target_pos + Eigen::Vector3f(dx, dy, dz);
                if (!global_map_->isInGlobalMapRange(check_pos)) continue;
                
                GlobalVoxelState state = global_map_->getVoxelState(check_pos);
                if (VoxelStateBits::isUnknown(state)) {
                    unknown_count++;
                    // 快速返回：一旦找到足够的未知体素就认为未被覆盖
                    if (unknown_count >= min_unknown_threshold) {
                        return false;
                    }
                }
            }
        }
    }
    
    // 未知体素太少，认为已被覆盖
    ROS_DEBUG("[ViewpointGen] Cluster %d at (%.2f,%.2f,%.2f) covered: unknown=%d",
              cluster.cluster_id, target_pos.x(), target_pos.y(), target_pos.z(), unknown_count);
    return true;
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
    // 递归分割，直到所有簇的 extent 都小于阈值
    const int max_iterations = 10;  // 防止无限循环
    int iteration = 0;
    
    bool changed = true;
    while (changed && iteration < max_iterations) {
        changed = false;
        iteration++;
        
        std::vector<FrontierCluster> result;
        
        for (const auto& cluster : clusters) {
            // 判断是否需要分裂
            if (cluster.extent > config_.cluster_split_threshold && 
                !cluster.points.empty() && 
                cluster.points.size() >= 4) {  // 至少4个点才能分裂
                
                // 交替使用X和Y坐标分裂，避免狭长簇分裂不充分
                bool use_x = (iteration % 2 == 1);
                float split_val = use_x ? cluster.centroid.x() : cluster.centroid.y();
                
                FrontierCluster c1, c2;
                c1.subgrid_id = cluster.subgrid_id;
                c2.subgrid_id = cluster.subgrid_id;
                
                // 按坐标分配点
                for (const auto& pt : cluster.points) {
                    float val = use_x ? pt.x() : pt.y();
                    if (val < split_val) {
                        c1.points.push_back(pt);
                    } else {
                        c2.points.push_back(pt);
                    }
                }
                
                // 重新计算c1的属性
                if (c1.points.size() >= 2) {
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
                    changed = true;
                }
                
                // 重新计算c2的属性
                if (c2.points.size() >= 2) {
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
                    changed = true;
                }
                
                // 如果分裂失败（点太少），保留原簇
                if (c1.points.size() < 2 && c2.points.size() < 2) {
                    result.push_back(cluster);
                    changed = false;
                }
            } else {
                result.push_back(cluster);
            }
        }
        
        clusters = std::move(result);
    }
    
    if (iteration > 1) {
        ROS_DEBUG("[ViewpointGen] Cluster splitting took %d iterations", iteration);
    }
    
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
            
            // 注意：不再检查从无人机到视点的直线可达性
            // 原因：EGO-Planner 会自动绕过障碍物，直线不可达不代表无法到达
            // 墙后面区域的问题由 L2 可见性检查（视点到簇）解决
            
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
            vp.target_center = cluster.centroid;  // 存储目标簇中心
            vp.yaw = yaw;
            vp.pitch = 0.0f;
            vp.cluster_id = cluster.cluster_id;
            vp.score = quick_score;
            vp.is_valid = true;
            
            layer1_passed.push_back(vp);
        }
    }
    
    if (layer1_passed.empty()) {
        ROS_DEBUG("[ViewpointGen] Cluster %d: no L1 candidates (centroid=%.2f,%.2f,%.2f)",
                  cluster.cluster_id, cluster.centroid.x(), cluster.centroid.y(), cluster.centroid.z());
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
        ROS_DEBUG("[ViewpointGen] Cluster %d: L1=%lu but no L2 candidates (visibility < %.2f)",
                  cluster.cluster_id, layer1_passed.size(), config_.min_visibility_ratio);
        return;  // 没有通过第2层的视点
    }
    
    // 按评分排序
    std::sort(layer2_passed.begin(), layer2_passed.end(),
        [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
            return a.score > b.score;
        });
    
    // ★★★ 移除 L3 可达性检查 ★★★
    // 原因：
    // 1. 视点本身已经通过 L1 安全检查，是一个安全的位置
    // 2. EGO-Planner 可以自动绑过障碍物，不需要直线可达
    // 3. 墙另一边的视点是合理的（能观察那边的前沿区域）
    // 直接选择评分最高的视点
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

// ==================== 可见性评分（直接射线到簇内点） ====================

float FrontierViewpointGenerator::computeVisibilityScore(
    const Eigen::Vector3f& viewpoint,
    const FrontierCluster& cluster)
{
    if (!global_map_) return 1.0f;
    
    // 首先检查到簇中心的直线是否被遮挡（最重要的检查）
    bool center_visible = checkRayVisibility(viewpoint, cluster.centroid);
    if (!center_visible) {
        // 簇中心被墙遮挡，可见性为0
        return 0.0f;
    }
    
    // 如果簇没有存储具体点，只检查中心
    if (cluster.points.empty()) {
        return 1.0f;
    }
    
    // 采样簇内的点进行可见性检查
    // 最多采样 20 个点以控制计算量
    const size_t max_samples = 20;
    size_t step = std::max(size_t(1), cluster.points.size() / max_samples);
    
    int visible_rays = 0;
    int total_rays = 0;
    
    for (size_t i = 0; i < cluster.points.size(); i += step) {
        const Eigen::Vector3f& point = cluster.points[i];
        
        // 检查该点是否在传感器范围内
        float dist = (point - viewpoint).norm();
                if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) {
                    continue;
                }
                
        // 检查到这个点的射线是否被遮挡
        bool ray_visible = checkRayVisibility(viewpoint, point);
        
        total_rays++;
        if (ray_visible) {
            visible_rays++;
        }
    }
    
    if (total_rays == 0) return 1.0f;  // 簇中心可见但没有其他点
    
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
    
    // 简化的 Yaw 优化：确保相邻视点的 yaw 变化平滑
    // 对于按顺序访问的视点，如果 yaw 差异超过 PI，调整 yaw 方向
    
    Eigen::Vector3f uav_pos;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        uav_pos = uav_state_.position;
    }
    
    // 按距离无人机的远近排序（greedy 访问顺序）
    std::vector<std::pair<float, size_t>> dist_idx;
    for (size_t i = 0; i < viewpoints.size(); ++i) {
        float dist = (viewpoints[i].position - uav_pos).head<2>().norm();
        dist_idx.push_back({dist, i});
    }
    std::sort(dist_idx.begin(), dist_idx.end());
    
    // 对排序后的视点序列进行 yaw 平滑
    int optimized_count = 0;
    for (size_t i = 1; i < dist_idx.size(); ++i) {
        size_t curr_idx = dist_idx[i].second;
        size_t prev_idx = dist_idx[i-1].second;
        
        float prev_yaw = viewpoints[prev_idx].yaw;
        float curr_yaw = viewpoints[curr_idx].yaw;
        
        // 计算 yaw 差异
        float yaw_diff = curr_yaw - prev_yaw;
        while (yaw_diff > PI) yaw_diff -= 2 * PI;
        while (yaw_diff < -PI) yaw_diff += 2 * PI;
        
        // 如果 yaw 差异超过 120 度（约 2.09 rad），记录需要优化的情况
        // 注意：当前保持原有 yaw，因为调整可能会使视点无法看到目标
        if (std::abs(yaw_diff) > 2.09f) {
            optimized_count++;
        }
    }
    
    ROS_INFO("[ViewpointGen] Yaw optimization: checked %lu viewpoint pairs, %d needed adjustment",
             viewpoints.size() > 1 ? viewpoints.size() - 1 : 0, optimized_count);
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


