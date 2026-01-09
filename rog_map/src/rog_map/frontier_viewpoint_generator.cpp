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
#include <unordered_map>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 性能监控
#include "utils/perf_logger.hpp"

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
    // 性能监控：整体计时
    perf::ScopeTimer gen_timer("GEN", "TOTAL_GENERATE");
    PERF_INFO("GEN", "========== Starting viewpoint generation cycle ==========");
    
    clusters.clear();
    viewpoints.clear();
    
    if (!global_map_) {
        ROS_WARN("[ViewpointGen] No global_map!");
        PERF_WARN("GEN", "No global_map available!");
        return false;
    }
    
    // Step 0: 更新历史节点（沿无人机轨迹采集）
    updateHistoryNodes();
    
    // Step 1: 获取按子网格分组的前沿点（包含子网格几何信息）
    std::vector<SubGridFrontierInfo> subgrid_frontiers;
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
    
    // 性能监控：前沿点统计（输出到日志文件，减少终端输出）
    PERF_LOG.logf("GEN", "FILTER_POINTS", "Raw=%lu, After=%lu, Removed=%lu",
                  total_points_before, total_points_after, 
                  total_points_before - total_points_after);
    PERF_LOG.logf("GEN", "INFO", "HistoryNodes=%lu, FrontierGroups=%lu",
                  history_nodes_.size(), node_frontiers.size());
    
    ROS_DEBUG("[ViewpointGen] Filtered frontier points: %lu -> %lu (removed %lu)",
             total_points_before, total_points_after, 
             total_points_before - total_points_after);
    ROS_DEBUG("[ViewpointGen] History nodes: %lu, Frontier groups: %lu", 
             history_nodes_.size(), node_frontiers.size());
    
    // Step 3: 在每个历史节点关联的前沿区域内聚类
    int global_cluster_id = 0;
    
    if (config_.use_surface_frontier) {
        // ==================== SFExplorer 策略：表面前沿聚类 ====================
        // 表面前沿：同时存在 occupied 和 free 邻居的 unknown 体素
        // 基于法向量相似性进行区域生长聚类
        
        ROS_DEBUG("[ViewpointGen] Using SFExplorer surface frontier strategy");
        
        for (const auto& nf_pair : node_frontiers) 
        {
            int node_id = nf_pair.first;
            const auto& points = nf_pair.second;
            
            if (points.size() < static_cast<size_t>(config_.min_cluster_size)) {
                continue;
            }
            
            // 步骤1：将普通前沿点转换为表面前沿点（带法向量）
            std::vector<SurfaceFrontierPoint> surface_points;
            for (const auto& pt : points) {
                SurfaceFrontierPoint sfp(pt);
                sfp.has_normal = computeSurfaceNormal(pt, sfp.normal);
                surface_points.push_back(sfp);
            }
            
            // 步骤2：基于法向量相似性聚类
            std::vector<FrontierCluster> node_clusters;
            clusterByNormalSimilarity(surface_points, node_clusters);
            
            for (auto& c : node_clusters) {
                c.cluster_id = global_cluster_id++;
                c.history_node_id = node_id;
                clusters.push_back(c);
            }
        }
        
        ROS_INFO("[ViewpointGen-SF] Surface frontier clustering: %lu clusters", clusters.size());
    } else {
        // ==================== 原有策略：空间距离聚类 ====================
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
    }
    
    if (clusters.empty()) 
    {
        ROS_DEBUG("[ViewpointGen] No clusters formed");
    return false;
}

    PERF_LOG.logf("GEN", "CLUSTERS", "formed=%lu from %lu groups", 
                  clusters.size(), node_frontiers.size());
    
    // Step 4: 分裂大簇
    splitLargeClusters(clusters);
    PERF_LOG.logf("GEN", "SPLIT", "after_split=%lu clusters", clusters.size());
    
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
            PERF_LOG.logf("GEN", "OVERHEAD_FILTER", "removed=%lu, remaining=%lu",
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
        
        PERF_LOG.logf("GEN", "LIMIT", "clusters %lu -> %d (keeping nearest)",
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
        
        // 根据配置选择采样策略
        if (config_.use_spherical_sampling) {
            // SFExplorer 策略：球扇形均匀采样
            sampleViewpointsSpherical(cluster, viewpoints);
        } else {
            // 原有策略：圆周采样
            sampleViewpoints(cluster, viewpoints);
        }
        
        // 调试：记录没有产生视点的簇
        if (viewpoints.size() == before) {
            no_viewpoint_count++;
            ROS_DEBUG("[ViewpointGen] Cluster %d at (%.2f,%.2f,%.2f) produced no viewpoints",
                      cluster.cluster_id, cluster.centroid.x(), 
                      cluster.centroid.y(), cluster.centroid.z());
        }
    }
    
    // 性能监控：簇处理统计
    PERF_LOG.logf("GEN", "CLUSTER_STATS", "total=%lu, covered=%lu, no_viewpoint=%lu, produced=%lu",
                  clusters.size(), covered_count, no_viewpoint_count,
                  clusters.size() - covered_count - no_viewpoint_count);
    
    // 性能监控：簇统计（输出到日志文件）
    PERF_LOG.logf("GEN", "SAMPLING", "covered=%lu, no_vp=%lu, produced=%lu, L1L2_candidates=%lu",
                  covered_count, no_viewpoint_count, 
                  clusters.size() - covered_count - no_viewpoint_count, viewpoints.size());
    
    // Step 6: 去重滤波
    filterViewpoints(viewpoints);
    
    PERF_LOG.logf("GEN", "DEDUP", "After_dedup=%lu", viewpoints.size());
    
    // Step 7: 视点聚类与Yaw优化
    // 对相邻视点进行聚类，然后优化每个簇内视点的yaw角，避免对射情况
    optimizeViewpointYaw(viewpoints);
    
    // Step 8: 增量视点聚类（SFExplorer策略，可选）
    // 对远距离视点聚类，用锚点替代，降低路径规划复杂度
    if (config_.use_viewpoint_clustering && viewpoints.size() > 5) {
        std::vector<ExplorationViewpoint> unclustered;
        std::vector<ViewpointCluster> vp_clusters;
        
        clusterViewpointsIncremental(viewpoints, unclustered, vp_clusters);
        
        // 将锚点转换为视点加入结果
        for (const auto& vpc : vp_clusters) {
            // 入口锚点视点
            ExplorationViewpoint anchor_vp;
            anchor_vp.position = vpc.anchor_entry;
            anchor_vp.target_center = vpc.centroid;
            Eigen::Vector3f to_center = vpc.centroid - vpc.anchor_entry;
            anchor_vp.yaw = std::atan2(to_center.y(), to_center.x());
            anchor_vp.pitch = 0.0f;
            anchor_vp.cluster_id = -1;  // 锚点标记
            anchor_vp.score = 1.0f;     // 高优先级
            anchor_vp.is_valid = true;
            unclustered.push_back(anchor_vp);
            
            // 出口锚点视点（可选，根据路径规划需要）
            // 暂时不添加，让路径规划自行决定
        }
        
        PERF_LOG.logf("GEN", "VP_CLUSTER", "%zu -> %zu unclustered + %zu clusters",
                      viewpoints.size(), unclustered.size(), vp_clusters.size());
        
        viewpoints = std::move(unclustered);
        viewpoint_clusters_ = std::move(vp_clusters);
    }
    
    // 性能监控：最终结果
    PERF_LOG.logf("GEN", "FINAL", "viewpoints=%zu, clusters=%lu, gen_time=%.2fms",
                  viewpoints.size(), clusters.size(), gen_timer.elapsed());
    
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
        node.parent_node_id = -1;  // 第一个节点没有父节点
        history_nodes_.push_back(node);
        first_node_created_ = true;
        last_connected_node_idx_ = 0;  // 初始化最后连接的节点索引
        
        // 性能监控：记录首个历史节点
        PERF_LOG.logf("HIST", "CREATE_FIRST", "id=%d, pos=(%.2f,%.2f,%.2f)",
                      node.node_id, current_pos.x(), current_pos.y(), current_pos.z());
        ROS_DEBUG("[ViewpointGen] Created first history node at (%.2f, %.2f, %.2f) after takeoff",
                 current_pos.x(), current_pos.y(), current_pos.z());
        return;
    }
    
    // 找到距离当前位置最近的现有节点（这就是 UAV 从哪个节点飞来的）
    float min_dist_to_existing = std::numeric_limits<float>::max();
    int nearest_node_idx = -1;
    for (size_t i = 0; i < history_nodes_.size(); ++i) {
        float dist = (current_pos - history_nodes_[i].position).norm();  // 3D距离
        if (dist < min_dist_to_existing) {
            min_dist_to_existing = dist;
            nearest_node_idx = static_cast<int>(i);
        }
    }
    
    // 更新 last_connected_node_idx_：UAV 当前最近的节点
    // 这用于追踪 UAV 的实际路径
    if (nearest_node_idx >= 0 && min_dist_to_existing < config_.history_node_distance * 1.5f) {
        last_connected_node_idx_ = nearest_node_idx;
    }
    
    // 只有当与所有现有节点的距离都超过阈值时，才创建新节点
    if (min_dist_to_existing >= config_.history_node_distance) {
        HistoryNode node(next_node_id_++, current_pos, ros::Time::now().toSec());
        
        // 关键：建立与最近节点的连接（因为 UAV 确实从那里飞来）
        node.parent_node_id = nearest_node_idx >= 0 ? history_nodes_[nearest_node_idx].node_id : -1;
        
        // 双向连接
        if (nearest_node_idx >= 0) {
            node.addNeighbor(history_nodes_[nearest_node_idx].node_id);
            history_nodes_[nearest_node_idx].addNeighbor(node.node_id);
        }
        
        // 如果最近节点不是 last_connected_node_idx_，也与 last_connected 建立连接
        // 这处理回溯情况：UAV 从节点 A 飞到 B，再飞回 A 附近并创建新节点 C
        // 此时 C 应该连接到 A（最近的）
        if (last_connected_node_idx_ >= 0 && last_connected_node_idx_ != nearest_node_idx) {
            // 检查是否已经连接
            if (!node.hasNeighbor(history_nodes_[last_connected_node_idx_].node_id)) {
                node.addNeighbor(history_nodes_[last_connected_node_idx_].node_id);
                history_nodes_[last_connected_node_idx_].addNeighbor(node.node_id);
            }
        }
        
        history_nodes_.push_back(node);
        last_connected_node_idx_ = history_nodes_.size() - 1;  // 更新为新创建的节点
        
        // 性能监控：记录新历史节点及其连接
        PERF_LOG.logf("HIST", "CREATE_NODE", "id=%d, pos=(%.2f,%.2f,%.2f), parent=%d, neighbors=%zu, total=%zu",
                      node.node_id, current_pos.x(), current_pos.y(), current_pos.z(),
                      node.parent_node_id, node.neighbors.size(), history_nodes_.size());
        ROS_DEBUG("[ViewpointGen] Created history node %d at (%.2f, %.2f, %.2f), connected to node %d",
                 node.node_id, current_pos.x(), current_pos.y(), current_pos.z(),
                 node.parent_node_id);
    }
    
    // 周期性统计：每10秒输出一次 History Node 网络状态摘要
    static double last_summary_time = 0;
    double current_time = ros::Time::now().toSec();
    if (history_nodes_.size() >= 3 && (current_time - last_summary_time) >= 10.0) {
        last_summary_time = current_time;
        
        // 统计邻居数量
        int total_edges = 0;
        int isolated_nodes = 0;
        for (const auto& node : history_nodes_) {
            total_edges += node.neighbors.size();
            if (node.neighbors.empty() && node.node_id != 0) {  // 第一个节点可以没有邻居
                isolated_nodes++;
            }
        }
        
        PERF_LOG.logf("HIST", "NETWORK_SUMMARY", "nodes=%zu, edges=%d, isolated=%d",
                      history_nodes_.size(), total_edges / 2, isolated_nodes);
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
    // 高度采样范围
    float h_min = config_.viewpoint_height_min;
    float h_max = config_.viewpoint_height_max;
    int h_num = std::max(1, config_.viewpoint_height_num);
    float dh = (h_num > 1) ? (h_max - h_min) / static_cast<float>(h_num - 1) : 0.0f;
    
    // 采样半径范围
    float dr = (config_.candidate_rmax - config_.candidate_rmin) / 
               std::max(1.0f, static_cast<float>(config_.candidate_rnum - 1));
    
    std::vector<ExplorationViewpoint> layer1_passed;  // 第1层过滤后
    std::vector<ExplorationViewpoint> layer2_passed;  // 第2层过滤后
    
    // ========== 第1层：快速过滤（计算量小） ==========
    // 检查：位置安全、距离范围、FOV角度
    // 三重循环：高度 × 半径 × 角度
    for (int ih = 0; ih < h_num; ++ih) {
        float viewpoint_z = (h_num > 1) ? (h_min + ih * dh) : config_.viewpoint_height;
        
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
    
    PERF_LOG.logf("GEN", "YAW_OPT", "checked=%lu pairs, adjusted=%d",
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

bool FrontierViewpointGenerator::isPositionSafeForExploration(const Eigen::Vector3f& pos)
{
    // 用于球扇形采样的宽松安全检查：
    // - 允许 FREE 和 UNKNOWN 区域
    // - 只拒绝 OCCUPIED 区域
    // 这是因为探索视点可能需要放置在已知自由区域的边界附近
    
    if (!global_map_) return true;
    
    // 调试计数器（静态，用于统计拒绝原因）
    static int reject_range = 0, reject_occupied = 0, reject_height = 0, reject_neighbor = 0;
    static int accept_count = 0;
    static int total_checks = 0;
    total_checks++;
    
    // 检查是否在地图范围内
    if (!global_map_->isInGlobalMapRange(pos)) {
        reject_range++;
        return false;
    }
    
    // 检查中心点不是障碍物
    GlobalVoxelState state = global_map_->getVoxelState(pos);
    if (VoxelStateBits::isOccupied(state)) {
        reject_occupied++;
        return false;
    }
    
    // 高度检查
    if (pos.z() < config_.viewpoint_height_min || pos.z() > config_.viewpoint_height_max) {
        reject_height++;
        return false;
    }
    
    // 简化的障碍物距离检查（只检查正下方和周围一圈）
    float check_radius = config_.uav_radius;
    
    // 检查周围几个关键点是否有障碍物
    const Eigen::Vector3f offsets[] = {
        {check_radius, 0, 0}, {-check_radius, 0, 0},
        {0, check_radius, 0}, {0, -check_radius, 0},
        {0, 0, -0.3f}  // 下方
    };
    
    for (const auto& offset : offsets) {
        Eigen::Vector3f check_pos = pos + offset;
        if (!global_map_->isInGlobalMapRange(check_pos)) continue;
        
        GlobalVoxelState check_state = global_map_->getVoxelState(check_pos);
        if (VoxelStateBits::isOccupied(check_state)) {
            reject_neighbor++;
            return false;
        }
    }
    
    accept_count++;
    
    // 每100次检查输出一次统计
    if (total_checks % 100 == 0) {
        ROS_INFO_THROTTLE(2.0, "[SafetyCheck] total=%d, accept=%d, reject: range=%d, occ=%d, height=%d, neighbor=%d | height_range=[%.1f,%.1f]",
            total_checks, accept_count, reject_range, reject_occupied, reject_height, reject_neighbor,
            config_.viewpoint_height_min, config_.viewpoint_height_max);
    }
    
    return true;
}

// ==================== 路径规划函数 ====================

bool FrontierViewpointGenerator::hasObstacleBetween(const Eigen::Vector3f& start, 
                                                     const Eigen::Vector3f& end)
{
    if (!global_map_) return false;
    
    Eigen::Vector3f dir = end - start;
    float dist = dir.norm();
    if (dist < 0.1f) return false;  // 距离太近，无需检查
    
    dir.normalize();
    float step = 0.3f;  // 放宽步长，减少过度检测
    
    int obstacle_count = 0;
    int unknown_count = 0;
    int total_checks = 0;
    
    // 沿射线检查障碍物
    for (float t = step; t < dist - step; t += step) {
        Eigen::Vector3f check_pos = start + dir * t;
        total_checks++;
        
        if (!global_map_->isInGlobalMapRange(check_pos)) {
            unknown_count++;
                    continue;
                }
                
        // 使用膨胀后的检测，更准确地反映实际碰撞风险
        if (global_map_->isOccupiedInflate(check_pos)) {
            obstacle_count++;
            // 关键：不要只检测到一个点就返回，累计检测
            // 如果连续多个点都是障碍物，才认为有障碍物阻挡
            if (obstacle_count >= 2) {
                return true;  // 确认有障碍物阻挡
            }
        } else {
            obstacle_count = 0;  // 重置计数，不是连续的
        }
    }
    
    // 如果大部分区域是未知的（超过60%），允许通过
    // 因为这可能是待探索区域，不应该阻止探索
    if (total_checks > 0 && (float)unknown_count / total_checks > 0.6f) {
        return false;
    }
    
    return false;  // 无障碍物
}

bool FrontierViewpointGenerator::findPathViaHistoryNodes(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end,
    std::vector<Eigen::Vector3f>& waypoints)
{
    waypoints.clear();
    
    if (history_nodes_.empty()) {
        PERF_WARN("HIST", "No history nodes available for path finding");
        return false;
    }
    
    PERF_LOG.logf("HIST", "PATH_FIND_START", "from=(%.2f,%.2f,%.2f) to=(%.2f,%.2f,%.2f) nodes=%zu",
             start.x(), start.y(), start.z(), end.x(), end.y(), end.z(), history_nodes_.size());
    
    // 步骤1：找到距离起点最近的历史节点（优先使用距离近的，不再检查障碍物）
    // 因为 UAV 当前就在起点附近，最近的节点一定可达
    int start_idx = -1;
    float min_start_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < history_nodes_.size(); ++i) {
        float dist = (history_nodes_[i].position - start).norm();
        if (dist < min_start_dist) {
            min_start_dist = dist;
            start_idx = static_cast<int>(i);
        }
    }
    
    // 步骤2：找到距离终点最近的历史节点
    int end_idx = -1;
    float min_end_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < history_nodes_.size(); ++i) {
        float dist = (history_nodes_[i].position - end).norm();
        if (dist < min_end_dist) {
            min_end_dist = dist;
            end_idx = static_cast<int>(i);
        }
    }
    
    if (start_idx < 0 || end_idx < 0) {
        PERF_LOG.logf("HIST", "PATH_FAIL", "reason=no_nodes, start_idx=%d, end_idx=%d",
                      start_idx, end_idx);
        return false;
    }
    
    PERF_LOG.logf("HIST", "PATH_NODES", "start_idx=%d(%.2f,%.2f), end_idx=%d(%.2f,%.2f), dist_to_start=%.2f, dist_to_end=%.2f",
             start_idx, history_nodes_[start_idx].position.x(), history_nodes_[start_idx].position.y(),
             end_idx, history_nodes_[end_idx].position.x(), history_nodes_[end_idx].position.y(),
             min_start_dist, min_end_dist);
    
    // 如果起点和终点是同一个节点，直接通过该节点
    if (start_idx == end_idx) {
        waypoints.push_back(history_nodes_[start_idx].position);
        PERF_LOG.logf("HIST", "PATH_SAME_NODE", "node_idx=%d", start_idx);
        return true;
    }
    
    // 步骤3：使用存储的邻接关系构建索引映射
    // 将 node_id 映射到数组索引
    std::unordered_map<int, int> id_to_idx;
    for (size_t i = 0; i < history_nodes_.size(); ++i) {
        id_to_idx[history_nodes_[i].node_id] = static_cast<int>(i);
    }
    
    // 步骤4：BFS 搜索最短路径（使用存储的邻接关系）
    std::vector<int> parent(history_nodes_.size(), -1);
    std::vector<bool> visited(history_nodes_.size(), false);
    std::queue<int> queue;
    
    queue.push(start_idx);
    visited[start_idx] = true;
    
    while (!queue.empty()) {
        int curr_idx = queue.front();
        queue.pop();
        
        if (curr_idx == end_idx) break;  // 找到目标
        
        // 使用存储的邻居列表
        for (int neighbor_id : history_nodes_[curr_idx].neighbors) {
            auto it = id_to_idx.find(neighbor_id);
            if (it != id_to_idx.end()) {
                int neighbor_idx = it->second;
                if (!visited[neighbor_idx]) {
                    visited[neighbor_idx] = true;
                    parent[neighbor_idx] = curr_idx;
                    queue.push(neighbor_idx);
                }
            }
        }
    }
    
    // 性能监控：连通性统计
    int total_edges = 0;
    int isolated_nodes = 0;
    for (const auto& node : history_nodes_) {
        total_edges += node.neighbors.size();
        if (node.neighbors.empty() && node.node_id != 0) isolated_nodes++;
    }
    PERF_LOG.logf("HIST", "CONNECTIVITY", "nodes=%zu, edges=%d, isolated=%d",
                  history_nodes_.size(), total_edges / 2, isolated_nodes);
    
    // 步骤5：检查是否找到路径
    if (!visited[end_idx]) {
        PERF_LOG.logf("HIST", "PATH_FAIL", "reason=no_graph_path, start_idx=%d, end_idx=%d, "
                      "start_neighbors=%zu, end_neighbors=%zu",
                      start_idx, end_idx, 
                      history_nodes_[start_idx].neighbors.size(), 
                      history_nodes_[end_idx].neighbors.size());
        return false;
    }
    
    // 步骤6：回溯构建路径
    std::vector<int> path_indices;
    int curr = end_idx;
    while (curr != -1) {
        path_indices.push_back(curr);
        curr = parent[curr];
    }
    std::reverse(path_indices.begin(), path_indices.end());
    
    PERF_LOG.logf("HIST", "RAW_PATH", "nodes=%zu", path_indices.size());
    
    // 步骤7：路径简化 - 移除不必要的中间点
    // 如果从节点 A 可以直接到节点 C（无障碍），则移除中间的 B
    std::vector<int> simplified;
    simplified.push_back(path_indices[0]);
    
    size_t i = 0;
    while (i < path_indices.size() - 1) {
        // 尝试跳过尽可能多的中间节点
        size_t j = path_indices.size() - 1;
        while (j > i + 1) {
            if (!hasObstacleBetween(history_nodes_[path_indices[i]].position,
                                    history_nodes_[path_indices[j]].position)) {
                // 可以从 i 直接到 j
            break;
            }
            j--;
        }
        simplified.push_back(path_indices[j]);
        i = j;
    }
    
    PERF_LOG.logf("HIST", "SIMPLIFIED", "%zu -> %zu nodes", path_indices.size(), simplified.size());
    
    // 步骤8：转换为航点（跳过起点节点，因为会从当前位置直接到第一个中间航点）
    for (size_t k = 1; k < simplified.size(); ++k) {
        waypoints.push_back(history_nodes_[simplified[k]].position);
    }
    
    // 记录路径到日志文件
    std::stringstream path_ss;
    path_ss << "waypoints=" << waypoints.size() << ": ";
    for (size_t k = 0; k < waypoints.size(); ++k) {
        path_ss << "(" << std::fixed << std::setprecision(1) 
                << waypoints[k].x() << "," << waypoints[k].y() << ")";
        if (k < waypoints.size() - 1) path_ss << "->";
    }
    PERF_LOG.log("HIST", "PATH_FOUND", path_ss.str());
    
    return true;
}

float FrontierViewpointGenerator::estimatePathLength(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end)
{
    float direct_dist = (end - start).norm();
    
    // 情况1：直线无障碍，直接返回直线距离
    if (!hasObstacleBetween(start, end)) {
        return direct_dist;
    }
    
    // 情况2：直线有障碍，判断是否需要回溯
    // 关键：检查终点附近是否有历史节点
    // - 如果终点附近没有历史节点 → 新探索区域，不需要回溯，EGO-Planner可以规划
    // - 如果终点附近有历史节点 → 已探索区域，可能需要回溯
    
    float min_dist_to_end = std::numeric_limits<float>::max();
    int nearest_end_node = -1;
    for (size_t i = 0; i < history_nodes_.size(); ++i) {
        float dist = (history_nodes_[i].position - end).norm();
        if (dist < min_dist_to_end) {
            min_dist_to_end = dist;
            nearest_end_node = static_cast<int>(i);
        }
    }
    
    // 如果终点附近没有历史节点（距离 > 阈值），说明是新探索区域
    // EGO-Planner可以直接规划过去（绕开拐角），返回稍微增加的距离
    float history_threshold = config_.history_node_distance * 3.0f;
    if (min_dist_to_end > history_threshold) {
        // 新探索区域，返回直线距离的1.3倍（考虑可能需要绕开拐角）
        ROS_DEBUG("[ViewpointGen] estimatePathLength: target is new area (dist_to_nearest_node=%.2f > %.2f), "
                  "returning %.2f (1.3x direct)",
                  min_dist_to_end, history_threshold, direct_dist * 1.3f);
        return direct_dist * 1.3f;
    }
    
    // 情况3：终点附近有历史节点，尝试通过历史节点回溯
    std::vector<Eigen::Vector3f> waypoints;
    if (!findPathViaHistoryNodes(start, end, waypoints)) {
        // 找不到路径，但这不一定意味着无法到达
        // 可能是新探索区域的边缘，EGO-Planner仍可能规划成功
        ROS_DEBUG("[ViewpointGen] estimatePathLength: no path via history nodes, "
                  "but target may still be reachable, returning %.2f (2x direct)",
                  direct_dist * 2.0f);
        return direct_dist * 2.0f;  // 给一个适中的代价
    }
    
    // 计算实际路径长度
    float path_length = 0.0f;
    Eigen::Vector3f prev = start;
    for (const auto& wp : waypoints) {
        path_length += (wp - prev).norm();
        prev = wp;
    }
    path_length += (end - prev).norm();
    
    ROS_DEBUG("[ViewpointGen] estimatePathLength: via %zu waypoints, "
              "direct=%.2f, actual=%.2f",
              waypoints.size(), direct_dist, path_length);
    
    return path_length;
}

// ==================== SFExplorer 策略实现 ====================

void FrontierViewpointGenerator::extractSurfaceFrontiers(
    std::vector<SurfaceFrontierPoint>& surface_frontiers)
{
    surface_frontiers.clear();
    
    if (!global_map_) return;
    
    // 获取所有前沿体素
    std::vector<GlobalVoxelIndex> frontier_voxels;
    global_map_->extractGlobalFrontierIncremental();
    
    // 遍历前沿体素，检查是否为表面前沿
    // 表面前沿：同时存在 occupied 和 free 邻居的 unknown 体素
    const GlobalVoxelIndex neighbor_offsets[6] = {
        {1, 0, 0}, {-1, 0, 0},
        {0, 1, 0}, {0, -1, 0},
        {0, 0, 1}, {0, 0, -1}
    };
    
    // 获取全局地图的 frontier voxels
    sensor_msgs::PointCloud2 pc_frontier;
    global_map_->getFrontierPointCloud(pc_frontier);
    
    // 将 PointCloud2 转换为点列表
    pcl::PointCloud<pcl::PointXYZ> pcl_frontier;
    pcl::fromROSMsg(pc_frontier, pcl_frontier);
    
    for (const auto& pcl_pt : pcl_frontier.points) {
        Eigen::Vector3f pos(pcl_pt.x, pcl_pt.y, pcl_pt.z);
        GlobalVoxelIndex idx;
        global_map_->posToIndex(pos, idx);
        
        // 检查是否同时有 occupied 和 free 邻居
        bool has_occupied_neighbor = false;
        bool has_free_neighbor = false;
        
        for (int d = 0; d < 6; ++d) {
            GlobalVoxelIndex nei_idx = idx + neighbor_offsets[d];
            Eigen::Vector3f nei_pos;
            global_map_->indexToPos(nei_idx, nei_pos);
            
            GlobalVoxelState state = global_map_->getVoxelState(nei_pos);
            if (VoxelStateBits::isOccupied(state)) {
                has_occupied_neighbor = true;
            } else if (VoxelStateBits::isFree(state)) {
                has_free_neighbor = true;
            }
        }
        
        // 只有同时有 occupied 和 free 邻居的才是表面前沿
        if (has_occupied_neighbor && has_free_neighbor) {
            SurfaceFrontierPoint sfp(pos);
            
            // 计算法向量
            sfp.has_normal = computeSurfaceNormal(pos, sfp.normal);
            
            surface_frontiers.push_back(sfp);
        }
    }
    
    ROS_DEBUG("[ViewpointGen-SF] Extracted %zu surface frontiers from %zu total frontiers",
              surface_frontiers.size(), pcl_frontier.size());
}

bool FrontierViewpointGenerator::computeSurfaceNormal(
    const Eigen::Vector3f& point, Eigen::Vector3f& normal)
{
    if (!global_map_) return false;
    
    // 收集邻域内的 occupied 点（表面点云）
    std::vector<Eigen::Vector3f> neighbor_points;
    float search_radius = config_.cluster_eps;  // 搜索半径
    // config_.normal_pca_k 用于控制采样密度，但这里我们采用半径搜索
    
    // 在搜索半径内查找 occupied 体素
    GlobalVoxelIndex center_idx;
    global_map_->posToIndex(point, center_idx);
    
    float voxel_size = global_map_->getResolution();
    int search_range = static_cast<int>(std::ceil(search_radius / voxel_size));
    
    for (int dx = -search_range; dx <= search_range; ++dx) {
        for (int dy = -search_range; dy <= search_range; ++dy) {
            for (int dz = -search_range; dz <= search_range; ++dz) {
                GlobalVoxelIndex nei_idx(center_idx.x + dx, center_idx.y + dy, center_idx.z + dz);
                Eigen::Vector3f nei_pos;
                global_map_->indexToPos(nei_idx, nei_pos);
                
                // 检查距离
                if ((nei_pos - point).norm() > search_radius) continue;
                
                // 检查是否是 occupied
                if (global_map_->isOccupied(nei_pos)) {
                    neighbor_points.push_back(nei_pos);
                }
            }
        }
    }
    
    // 需要至少 3 个点才能计算法向量
    if (neighbor_points.size() < 3) {
        // 没有足够的邻居点，使用默认法向量（从UAV指向前沿点）
        Eigen::Vector3f to_point = point - uav_state_.position;
        to_point.z() = 0.0f;
        if (to_point.norm() > 0.1f) {
            normal = to_point.normalized();
        } else {
            normal = Eigen::Vector3f::UnitX();
        }
        return false;
    }
    
    // PCA 计算法向量
    // 1. 计算均值
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& p : neighbor_points) {
        mean += p;
    }
    mean /= static_cast<float>(neighbor_points.size());
    
    // 2. 构建协方差矩阵
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& p : neighbor_points) {
        Eigen::Vector3f diff = p - mean;
        cov += diff * diff.transpose();
    }
    cov /= static_cast<float>(neighbor_points.size());
    
    // 3. 特征值分解，最小特征值对应的特征向量就是法向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    
    // 最小特征值对应的特征向量（第一列）
    normal = solver.eigenvectors().col(0);
    
    // 4. 确保法向量指向未知区域（从 occupied 指向 unknown）
    // 法向量应该从表面指向外部（前沿点方向）
    Eigen::Vector3f to_frontier = point - mean;
    if (normal.dot(to_frontier) < 0) {
        normal = -normal;
    }
    
    return true;
}

void FrontierViewpointGenerator::clusterByNormalSimilarity(
    const std::vector<SurfaceFrontierPoint>& surface_points,
    std::vector<FrontierCluster>& clusters)
{
    clusters.clear();
    if (surface_points.empty()) return;
    
    const float angle_threshold = config_.normal_angle_threshold;
    const float eps = config_.cluster_eps;
    const int min_pts = config_.min_cluster_size;
    
    std::vector<bool> visited(surface_points.size(), false);
    
    for (size_t i = 0; i < surface_points.size(); ++i) {
        if (visited[i]) continue;
        
        // 区域生长
        std::vector<size_t> cluster_indices;
        std::queue<size_t> queue;
        queue.push(i);
        visited[i] = true;
        
        // 使用第一个点的法向量作为初始主法线
        Eigen::Vector3f cluster_normal = surface_points[i].normal;
        
        while (!queue.empty()) {
            size_t current = queue.front();
            queue.pop();
            cluster_indices.push_back(current);
            
            const auto& curr_point = surface_points[current];
            
            // 查找邻居
            for (size_t j = 0; j < surface_points.size(); ++j) {
                if (visited[j]) continue;
                
                const auto& other = surface_points[j];
                
                // 距离检查
                float dist = (other.position - curr_point.position).norm();
                if (dist > eps) continue;
                
                // 法向量相似性检查（仅当两个点都有有效法向量时）
                if (curr_point.has_normal && other.has_normal) {
                    float cos_angle = curr_point.normal.dot(other.normal);
                    cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
                    float angle = std::acos(cos_angle);
                    
                    if (angle > angle_threshold) continue;
                }
                
                visited[j] = true;
                queue.push(j);
            }
        }
        
        // 检查簇大小
        if (static_cast<int>(cluster_indices.size()) >= min_pts) {
            FrontierCluster cluster;
            
            // 计算属性
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();
            
            for (size_t idx : cluster_indices) {
                sum += surface_points[idx].position;
                cluster.points.push_back(surface_points[idx].position);
                cluster.surface_points.push_back(surface_points[idx]);
                
                if (surface_points[idx].has_normal) {
                    normal_sum += surface_points[idx].normal;
                }
            }
            
            cluster.centroid = sum / static_cast<float>(cluster_indices.size());
            cluster.point_count = static_cast<int>(cluster_indices.size());
            
            // 计算主法线
            if (normal_sum.norm() > 0.1f) {
                cluster.normal = normal_sum.normalized();
            }
            
            // 计算范围
            float max_dist = 0.0f;
            for (const auto& pt : cluster.points) {
                max_dist = std::max(max_dist, (pt - cluster.centroid).norm());
            }
            cluster.extent = max_dist;
            
            // 计算主成分方向（用于锚点生成）
            computeClusterPrincipalNormal(cluster);
            
            clusters.push_back(cluster);
        }
    }
}

void FrontierViewpointGenerator::computeClusterPrincipalNormal(FrontierCluster& cluster)
{
    if (cluster.surface_points.size() < 3) {
        cluster.principal_direction = Eigen::Vector3f::UnitX();
        return;
    }
    
    // 收集所有法向量
    std::vector<Eigen::Vector3f> normals;
    for (const auto& sp : cluster.surface_points) {
        if (sp.has_normal) {
            normals.push_back(sp.normal);
        }
    }
    
    if (normals.size() < 3) {
        cluster.principal_direction = cluster.normal;
        return;
    }
    
    // PCA 计算主法线
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& n : normals) mean += n;
    mean /= static_cast<float>(normals.size());
    
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& n : normals) {
        Eigen::Vector3f diff = n - mean;
        cov += diff * diff.transpose();
    }
    cov /= static_cast<float>(normals.size());
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() == Eigen::Success) {
        // 最大特征值对应的特征向量（最后一列）
        cluster.principal_direction = solver.eigenvectors().col(2);
    } else {
        cluster.principal_direction = cluster.normal;
    }
}

void FrontierViewpointGenerator::sampleViewpointsSpherical(
    const FrontierCluster& cluster,
    std::vector<ExplorationViewpoint>& viewpoints)
{
    // 球扇形采样参数
    // 注意：采样半径应该大于等于最小传感器距离
    float r_min = config_.sensor_range_min;
    float r_max = std::max(config_.sphere_sample_radius, config_.sensor_range_min + 1.0f);
    float theta_max = config_.sphere_theta_max;
    int sample_num = config_.sphere_sample_num * 3;  // 增加采样数量
    
    // 构建局部坐标系：z 轴沿主法线的反方向（指向自由空间）
    // 因为 cluster.normal 指向未知区域，我们需要在相反方向（自由空间）采样视点
    Eigen::Vector3f z_axis = -cluster.normal.normalized();
    
    // 找一个不平行于 z 的向量来构建 x 轴
    Eigen::Vector3f temp = (std::abs(z_axis.z()) < 0.9f) ? 
                           Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    Eigen::Vector3f x_axis = z_axis.cross(temp).normalized();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    
    // 旋转矩阵：从局部坐标系到世界坐标系
    Eigen::Matrix3f R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
    
    // 随机数生成器
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    std::vector<ExplorationViewpoint> candidates;
    
    // 调试计数器
    int rejected_safety = 0;
    int rejected_distance = 0;
    int rejected_utility = 0;
    
    for (int i = 0; i < sample_num; ++i) {
        // 均匀球扇形采样（SFExplorer 公式 4）
        float u1 = dist(gen);
        float u2 = dist(gen);
        float u3 = dist(gen);
        
        // r 在 [r_min, r_max] 范围内均匀采样（体积均匀）
        // r = (r_min^3 + u1 * (r_max^3 - r_min^3))^(1/3)
        float r_min_cubed = r_min * r_min * r_min;
        float r_max_cubed = r_max * r_max * r_max;
        float r = std::cbrt(r_min_cubed + u1 * (r_max_cubed - r_min_cubed));
        
        // θ = arccos(1 - u2*(1 - cos(θ_max)))
        float cos_theta = 1.0f - u2 * (1.0f - std::cos(theta_max));
        float theta = std::acos(std::max(-1.0f, std::min(1.0f, cos_theta)));
        
        // φ = 2π * u3
        float phi = 2.0f * PI * u3;
        
        // 球坐标转笛卡尔坐标（局部坐标系）
        float sin_theta = std::sin(theta);
        Eigen::Vector3f local_pos(
            r * sin_theta * std::cos(phi),
            r * sin_theta * std::sin(phi),
            r * std::cos(theta)
        );
        
        // 转换到世界坐标系
        Eigen::Vector3f world_pos = cluster.centroid + R * local_pos;
        
        // 将视点高度钳制到允许范围内（重要：球扇形采样后高度可能超出范围）
        world_pos.z() = std::max(config_.viewpoint_height_min + 0.1f,
                                 std::min(config_.viewpoint_height_max - 0.1f, world_pos.z()));
        
        // 安全检查 - 对于球扇形采样，允许UNKNOWN区域
        // 因为我们是朝未知方向探索，视点可能在边界附近
        bool safe = isPositionSafeForExploration(world_pos);
        if (!safe) {
            rejected_safety++;
            continue;
        }
        
        // 计算朝向（朝向簇中心）
        Eigen::Vector3f to_cluster = cluster.centroid - world_pos;
        float yaw = std::atan2(to_cluster.y(), to_cluster.x());
        
        // 距离检查
        float dist_to_cluster = to_cluster.norm();
        if (dist_to_cluster < config_.sensor_range_min || 
            dist_to_cluster > config_.sensor_range_max) {
            rejected_distance++;
            continue;
        }
        
        // 计算效用评分
        float utility = computeUtilityScore(world_pos, cluster);
        
        if (utility > 0.01f) {  // 降低效用阈值
            ExplorationViewpoint vp;
            vp.position = world_pos;
            vp.target_center = cluster.centroid;
            vp.yaw = yaw;
            vp.pitch = 0.0f;
            vp.cluster_id = cluster.cluster_id;
            vp.score = utility;
            vp.is_valid = true;
            
            candidates.push_back(vp);
        } else {
            rejected_utility++;
        }
    }
    
    // 输出调试信息
    ROS_INFO_THROTTLE(1.0, "[ViewpointGen-SF] Cluster %d (center=[%.1f,%.1f,%.1f]): candidates=%zu, "
              "reject(safety=%d, distance=%d, utility=%d) / total=%d",
              cluster.cluster_id, cluster.centroid.x(), cluster.centroid.y(), cluster.centroid.z(),
              candidates.size(), rejected_safety, rejected_distance, rejected_utility, sample_num);
    
    // 可见性检查，选择最佳视点
    if (!candidates.empty()) {
        // 按效用排序
        std::sort(candidates.begin(), candidates.end(),
            [](const ExplorationViewpoint& a, const ExplorationViewpoint& b) {
                return a.score > b.score;
            });
        
        // 对 top-K 进行可见性检查
        int top_k = std::min(5, static_cast<int>(candidates.size()));
        bool found_good_visibility = false;
        for (int i = 0; i < top_k; ++i) {
            float visibility = computeVisibilityScore(candidates[i].position, cluster);
            if (visibility >= config_.min_visibility_ratio) {
                candidates[i].score = candidates[i].score * 0.7f + visibility * 0.3f;
                viewpoints.push_back(candidates[i]);
                found_good_visibility = true;
                break;  // 只选择一个最佳视点
            }
        }
        
        // 如果所有候选点可见性都不够，选择效用最高的那个作为回退
        if (!found_good_visibility && !candidates.empty()) {
            ROS_DEBUG("[ViewpointGen-SF] Cluster %d: no good visibility, using best utility candidate",
                     cluster.cluster_id);
            viewpoints.push_back(candidates[0]);
        }
    }
}

float FrontierViewpointGenerator::computeUtilityScore(
    const Eigen::Vector3f& viewpoint,
    const FrontierCluster& cluster)
{
    // SFExplorer 效用函数：U(vp) = Σ (n_k · (vp - f_k)) / (||n_k|| * ||vp - f_k||)
    // 综合考虑覆盖度和观测视角质量
    
    float utility = 0.0f;
    int visible_count = 0;
    
    // 遍历簇内所有带法向量的前沿点
    for (const auto& sp : cluster.surface_points) {
        if (!sp.has_normal) continue;
        
        Eigen::Vector3f vp_to_frontier = sp.position - viewpoint;
        float dist = vp_to_frontier.norm();
        
        // 距离检查
        if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) continue;
        
        // 射线可见性检查（简化：只检查采样点）
        if (!checkRayVisibility(viewpoint, sp.position)) continue;
        
        visible_count++;
        
        // 计算 cos(θ) = |n · d| / (||n|| * ||d||)
        // 其中 d 是从前沿点到视点的方向
        // 注意：法向量可能指向任意方向，我们取绝对值确保正贡献
        Eigen::Vector3f view_dir = -vp_to_frontier.normalized();
        float cos_theta = std::abs(sp.normal.dot(view_dir));
        
        // cos_theta 越大（越接近 1），观测视角越垂直于表面，质量越高
        utility += cos_theta;
    }
    
    // 如果没有表面点，使用簇的法向量和质心
    if (cluster.surface_points.empty() || visible_count == 0) {
        Eigen::Vector3f vp_to_centroid = cluster.centroid - viewpoint;
        float centroid_dist = vp_to_centroid.norm();
        
        if (centroid_dist >= config_.sensor_range_min && centroid_dist <= config_.sensor_range_max) {
            Eigen::Vector3f view_dir = -vp_to_centroid.normalized();
            // 取绝对值，因为法向量方向可能不确定
            float cos_theta = std::abs(cluster.normal.dot(view_dir));
            utility = cos_theta;
        }
    }
    
    // 返回平均效用（如果有多个可见点）
    if (visible_count > 0) {
        utility /= visible_count;
    }
    
    return utility;
}

void FrontierViewpointGenerator::clusterViewpointsIncremental(
    const std::vector<ExplorationViewpoint>& viewpoints,
    std::vector<ExplorationViewpoint>& unclustered,
    std::vector<ViewpointCluster>& clusters)
{
    unclustered.clear();
    clusters.clear();
    
    if (viewpoints.empty()) return;
    
    // 获取 UAV 位置
    Eigen::Vector3f uav_pos = uav_state_.position;
    float cluster_min_dist = config_.vp_cluster_min_dist;
    float cluster_dist = config_.vp_cluster_distance;
    float cluster_radius = config_.vp_cluster_radius;
    
    // 分离近处和远处视点
    std::vector<ExplorationViewpoint> far_viewpoints;
    
    for (const auto& vp : viewpoints) {
        float dist = (vp.position - uav_pos).norm();
        if (dist < cluster_min_dist) {
            // 近处视点不聚类
            unclustered.push_back(vp);
        } else {
            far_viewpoints.push_back(vp);
        }
    }
    
    if (far_viewpoints.empty()) return;
    
    // 增量聚类远处视点
    std::vector<bool> clustered(far_viewpoints.size(), false);
    
    for (size_t i = 0; i < far_viewpoints.size(); ++i) {
        if (clustered[i]) continue;
        
        ViewpointCluster cluster;
        cluster.cluster_id = next_vp_cluster_id_++;
        cluster.viewpoints.push_back(far_viewpoints[i]);
        clustered[i] = true;
        
        // 尝试将其他视点加入聚类
        bool changed = true;
        while (changed) {
            changed = false;
            
            for (size_t j = 0; j < far_viewpoints.size(); ++j) {
                if (clustered[j]) continue;
                
                const auto& vp = far_viewpoints[j];
                
                // 条件1：与聚类内所有视点的距离 < d_c
                bool dist_ok = true;
                for (const auto& cvp : cluster.viewpoints) {
                    if ((vp.position - cvp.position).norm() > cluster_dist) {
                        dist_ok = false;
                        break;
                    }
                }
                if (!dist_ok) continue;
                
                // 条件2：加入后外接圆半径 < r_c
                // 计算当前质心
                Eigen::Vector3f sum = Eigen::Vector3f::Zero();
                for (const auto& cvp : cluster.viewpoints) sum += cvp.position;
                sum += vp.position;
                Eigen::Vector3f new_centroid = sum / static_cast<float>(cluster.viewpoints.size() + 1);
                
                float max_radius = 0.0f;
                for (const auto& cvp : cluster.viewpoints) {
                    max_radius = std::max(max_radius, (cvp.position - new_centroid).norm());
                }
                max_radius = std::max(max_radius, (vp.position - new_centroid).norm());
                
                if (max_radius > cluster_radius) continue;
                
                // 条件3：与聚类内视点无障碍
                bool no_obstacle = true;
                for (const auto& cvp : cluster.viewpoints) {
                    if (hasObstacleBetween(vp.position, cvp.position)) {
                        no_obstacle = false;
                        break;
                    }
                }
                if (!no_obstacle) continue;
                
                // 满足所有条件，加入聚类
                cluster.viewpoints.push_back(vp);
                clustered[j] = true;
                changed = true;
            }
        }
        
        // 处理聚类结果
        if (cluster.viewpoints.size() >= 2) {
            // 计算聚类属性并生成锚点
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (const auto& cvp : cluster.viewpoints) sum += cvp.position;
            cluster.centroid = sum / static_cast<float>(cluster.viewpoints.size());
            
            float max_r = 0.0f;
            for (const auto& cvp : cluster.viewpoints) {
                max_r = std::max(max_r, (cvp.position - cluster.centroid).norm());
            }
            cluster.radius = max_r;
            
            generateAnchorPoints(cluster);
            clusters.push_back(cluster);
        } else {
            // 单个视点不形成聚类
            unclustered.push_back(cluster.viewpoints[0]);
        }
    }
}

void FrontierViewpointGenerator::generateAnchorPoints(ViewpointCluster& cluster)
{
    if (cluster.viewpoints.size() < 2) {
        cluster.anchor_entry = cluster.centroid;
        cluster.anchor_exit = cluster.centroid;
        return;
    }
    
    // PCA 计算主方向
    Eigen::Vector3f mean = cluster.centroid;
    
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& vp : cluster.viewpoints) {
        Eigen::Vector3f diff = vp.position - mean;
        cov += diff * diff.transpose();
    }
    cov /= static_cast<float>(cluster.viewpoints.size());
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() == Eigen::Success) {
        // 最大特征值对应的特征向量（主方向）
        cluster.principal_direction = solver.eigenvectors().col(2);
        
        // 第一主成分方向的标准差
        float sigma = std::sqrt(solver.eigenvalues()(2));
        float l_1 = sigma;  // 锚点距离
        
        // 生成锚点：ap_a = p_c - l_1*v_1, ap_b = p_c + l_1*v_1
        cluster.anchor_entry = cluster.centroid - l_1 * cluster.principal_direction;
        cluster.anchor_exit = cluster.centroid + l_1 * cluster.principal_direction;
    } else {
        cluster.anchor_entry = cluster.centroid;
        cluster.anchor_exit = cluster.centroid;
    }
}

} // namespace rog_map


