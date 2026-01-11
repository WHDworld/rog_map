/**
 * @file viewpoint_manager.cpp
 * @brief 视点生成管理器实现
 * 
 * 完整流程（来自frontier_viewpoint_generator.cpp）：
 * 1. 获取前沿点并过滤
 * 2. 按历史节点分组
 * 3. 聚类（支持空间距离聚类和法向量相似性聚类）
 * 4. 分裂大簇
 * 5. 采样视点（支持圆周采样和球扇形采样）
 * 6. 分层过滤（L1快速检查 + L2可见性检查）
 * 7. 去重和Yaw优化
 * 8. 可选的增量视点聚类
 */

#include "rog_map/viewpoint_manager.h"
#include <queue>
#include <algorithm>
#include <cmath>

// 简化日志宏
#define VP_INFO(fmt, ...) ROS_INFO("[VP_MGR] " fmt, ##__VA_ARGS__)
#define VP_WARN(fmt, ...) ROS_WARN("[VP_MGR] " fmt, ##__VA_ARGS__)
#define VP_DEBUG(fmt, ...) ROS_DEBUG("[VP_MGR] " fmt, ##__VA_ARGS__)
#define VP_INFO_THROTTLE(rate, fmt, ...) ROS_INFO_THROTTLE(rate, "[VP_MGR] " fmt, ##__VA_ARGS__)

namespace rog_map {

ViewpointManager::ViewpointManager() {}

void ViewpointManager::init(ros::NodeHandle& nh, GlobalMap::Ptr global_map, 
                            HistoryGraph::Ptr history_graph) {
    nh_ = nh;
    global_map_ = global_map;
    history_graph_ = history_graph;
    
    // ==================== 加载配置 ====================
    
    // 更新频率
    nh_.param("viewpoint_manager/update_rate", config_.update_rate, 5.0f);
    
    // 传感器参数
    nh_.param("viewpoint_manager/sensor_range_min", config_.sensor_range_min, 2.0f);
    nh_.param("viewpoint_manager/sensor_range_max", config_.sensor_range_max, 5.0f);
    nh_.param("viewpoint_manager/sensor_fov_h", config_.sensor_fov_h, 1.57f);
    nh_.param("viewpoint_manager/sensor_fov_v", config_.sensor_fov_v, 1.15f);
    
    // 视点采样参数
    nh_.param("viewpoint_manager/viewpoint_height", config_.viewpoint_height, 1.5f);
    nh_.param("viewpoint_manager/viewpoint_height_min", config_.viewpoint_height_min, 1.0f);
    nh_.param("viewpoint_manager/viewpoint_height_max", config_.viewpoint_height_max, 2.0f);
    nh_.param("viewpoint_manager/viewpoint_height_num", config_.viewpoint_height_num, 1);
    nh_.param("viewpoint_manager/candidate_rmin", config_.candidate_rmin, 2.0f);
    nh_.param("viewpoint_manager/candidate_rmax", config_.candidate_rmax, 5.0f);
    nh_.param("viewpoint_manager/candidate_rnum", config_.candidate_rnum, 6);
    nh_.param("viewpoint_manager/candidate_dphi", config_.candidate_dphi, 0.25f);
    nh_.param("viewpoint_manager/optimal_distance", config_.optimal_distance, 3.5f);
    
    // 前沿聚类参数
    nh_.param("viewpoint_manager/cluster_eps", config_.cluster_eps, 0.25f);
    nh_.param("viewpoint_manager/min_cluster_size", config_.min_cluster_size, 30);
    nh_.param("viewpoint_manager/cluster_split_threshold", config_.cluster_split_threshold, 4.0f);
    nh_.param("viewpoint_manager/max_clusters", config_.max_clusters, 30);
    
    // 安全参数
    nh_.param("viewpoint_manager/uav_radius", config_.uav_radius, 0.3f);
    nh_.param("viewpoint_manager/safety_margin", config_.safety_margin, 0.5f);
    
    // 滤波参数
    nh_.param("viewpoint_manager/min_viewpoint_distance", config_.min_viewpoint_distance, 1.5f);
    nh_.param("viewpoint_manager/min_coverage_score", config_.min_coverage_score, 0.3f);
    nh_.param("viewpoint_manager/min_visibility_ratio", config_.min_visibility_ratio, 0.70f);
    
    // 前沿点过滤参数
    nh_.param("rog_map/global_map/frontier_height_min", config_.frontier_height_min, 0.7f);
    nh_.param("rog_map/global_map/frontier_height_max", config_.frontier_height_max, 2.5f);
    
    // 头顶区域过滤参数
    nh_.param("viewpoint_manager/overhead_height_threshold", config_.overhead_height_threshold, 0.3f);
    nh_.param("viewpoint_manager/overhead_distance_threshold", config_.overhead_distance_threshold, 3.0f);
    
    // 策略开关
    nh_.param("viewpoint_manager/use_surface_frontier", config_.use_surface_frontier, false);
    nh_.param("viewpoint_manager/use_spherical_sampling", config_.use_spherical_sampling, false);
    nh_.param("viewpoint_manager/use_viewpoint_clustering", config_.use_viewpoint_clustering, false);
    
    // 表面前沿聚类参数
    nh_.param("viewpoint_manager/normal_angle_threshold", config_.normal_angle_threshold, 0.785f);
    nh_.param("viewpoint_manager/normal_pca_k", config_.normal_pca_k, 10);
    
    // 球扇形采样参数
    nh_.param("viewpoint_manager/sphere_sample_radius", config_.sphere_sample_radius, 2.5f);
    nh_.param("viewpoint_manager/sphere_theta_max", config_.sphere_theta_max, 0.785f);
    nh_.param("viewpoint_manager/sphere_sample_num", config_.sphere_sample_num, 20);
    
    // 增量视点聚类参数
    nh_.param("viewpoint_manager/vp_cluster_distance", config_.vp_cluster_distance, 3.0f);
    nh_.param("viewpoint_manager/vp_cluster_radius", config_.vp_cluster_radius, 5.0f);
    nh_.param("viewpoint_manager/vp_cluster_min_dist", config_.vp_cluster_min_dist, 6.0f);
      
    VP_INFO("Initialized: rate=%.1fHz, r=[%.1f,%.1f], strategies: surface=%d, spherical=%d, vp_cluster=%d",
            config_.update_rate, config_.candidate_rmin, config_.candidate_rmax,
            config_.use_surface_frontier, config_.use_spherical_sampling, config_.use_viewpoint_clustering);
}

void ViewpointManager::start() {

    timer_ = nh_.createTimer(ros::Duration(1.0 / config_.update_rate),
                              &ViewpointManager::timerCallback, this);
}

void ViewpointManager::stop() {
    timer_.stop();
}

void ViewpointManager::updateUAVState(const Eigen::Vector3f& position, float yaw) {
    std::lock_guard<std::mutex> lock(uav_mutex_);
    uav_position_ = position;
    uav_yaw_ = yaw;
    has_uav_state_ = true;
}

void ViewpointManager::timerCallback(const ros::TimerEvent& e) {
    if (!has_uav_state_) return;
    update();
}

void ViewpointManager::triggerUpdate() {
    update();
}

// ==================== 主更新函数 ====================

void ViewpointManager::update() {
    history_graph_->refreshSubGridOwnership();
    ros::Time t_start = ros::Time::now();
    
    if (!global_map_ || !history_graph_) {
        VP_WARN("Global map or history graph not available");
        return;
    }
    
    // 获取UAV位置
    Eigen::Vector3f uav_pos;
    {
        std::lock_guard<std::mutex> lock(uav_mutex_);
        uav_pos = uav_position_;
    }
    
    // Step 0: 刷新 HistoryGraph 的子网格权属（确保新生成的节点接管附近的子网格）
    history_graph_->refreshSubGridOwnership();

    // Step 1 & 2: 从 HistoryGraph 获取活跃节点及其关联的前沿点
    // 替代了原来的全量 getFrontierPositionsBySubGrid
    ros::Time t_step1 = ros::Time::now();
    
    std::map<int, std::vector<Eigen::Vector3f>> node_frontiers;
    std::map<int, std::vector<SurfaceFrontierPoint>> node_surface_frontiers;
    
    std::vector<int> active_node_ids = history_graph_->getAllNodeIds();

    size_t total_points = 0;

    for (int node_id : active_node_ids) {
        // 获取该节点管辖的子网格 ID
        std::vector<int> associated_sg_ids = history_graph_->getAssociatedSubGrids(node_id);
        if (associated_sg_ids.empty()) continue;

        for (int sg_id : associated_sg_ids) {
            std::vector<Eigen::Vector3f> points;

            // B. [核心修改] 调用 GlobalMap 的新接口获取局部前沿数据
            bool has_frontiers = global_map_->getSubGridFrontierPoints(sg_id, points);
            // C. 闭环管理：如果 GlobalMap 说这个子网格没前沿了，通知 HistoryGraph 释放权属
            if (!has_frontiers) 
            {
                history_graph_->releaseSubGrid(sg_id);
                continue;
            }

            if (points.empty()) continue;
            
            total_points += points.size();

            // 存入 map
            if (config_.use_surface_frontier) {
                for (const auto& pt : points) {
                    SurfaceFrontierPoint sfp(pt);
                    sfp.has_normal = computeSurfaceNormal(pt, sfp.normal);
                    node_surface_frontiers[node_id].push_back(sfp);
                }
            } else {
                node_frontiers[node_id].insert(node_frontiers[node_id].end(),
                                               points.begin(), points.end());
            }
        }
    }
    
    double t_get_frontier = (ros::Time::now() - t_step1).toSec() * 1000.0;
    
    if (total_points == 0) {
        // ROS_INFO_THROTTLE(2.0, "[VP_MGR] No active frontiers (took %.1fms)", t_get_frontier);
        std::lock_guard<std::mutex> lock(mutex_);
        current_clusters_.clear();
        return;
    }    
    // Step 3: 聚类
    ros::Time t_step3 = ros::Time::now();
    std::vector<FrontierCluster> all_clusters;
    
    if (config_.use_surface_frontier) {
        // ==================== SFExplorer策略：法向量相似性聚类 ====================
        for (const auto& nf : node_surface_frontiers) {
            int node_id = nf.first;
            const auto& surface_points = nf.second;
            
            if (surface_points.size() < static_cast<size_t>(config_.min_cluster_size)) continue;
            
            std::vector<FrontierCluster> node_clusters;
            clusterByNormalSimilarity(surface_points, node_clusters);
            
            for (auto& c : node_clusters) {
                c.cluster_id = next_cluster_id_++;
                c.history_node_id = node_id;
                all_clusters.push_back(c);
            }
        }
        VP_DEBUG("Surface frontier clustering: %lu clusters", all_clusters.size());
    } else {
        // ==================== 原有策略：空间距离聚类 ====================
        for (const auto& nf : node_frontiers) {
            int node_id = nf.first;
            const auto& points = nf.second;
            
            if (points.size() < static_cast<size_t>(config_.min_cluster_size)) continue;
            
            std::vector<FrontierCluster> node_clusters;
            clusterByHistoryNode(points, node_id, node_clusters);
            
            for (auto& c : node_clusters) {
                c.cluster_id = next_cluster_id_++;
                c.history_node_id = node_id;
                all_clusters.push_back(c);
            }
        }
    }
    
    double t_cluster = (ros::Time::now() - t_step3).toSec() * 1000.0;
    
    if (all_clusters.empty()) {
        ROS_INFO("[VP_MGR] No clusters formed (frontier:%.1f, group:%.1f, cluster:%.1fms)",
                 t_get_frontier, t_group, t_cluster);
        std::lock_guard<std::mutex> lock(mutex_);
        current_clusters_.clear();
        return;
    }
    
    // Step 4: 分裂大簇
    ros::Time t_step4 = ros::Time::now();
    splitLargeClusters(all_clusters);
    
    // Step 5: 过滤头顶区域簇
    filterOverheadClusters(all_clusters);
    
    // Step 6: 限制簇数量（按距离排序保留最近的）
    if (all_clusters.size() > static_cast<size_t>(config_.max_clusters)) {
        std::sort(all_clusters.begin(), all_clusters.end(),
                  [&uav_pos](const FrontierCluster& a, const FrontierCluster& b) {
                      return (a.centroid - uav_pos).head<2>().norm() < 
                             (b.centroid - uav_pos).head<2>().norm();
                  });
        all_clusters.resize(config_.max_clusters);
    }
    
    double t_filter = (ros::Time::now() - t_step4).toSec() * 1000.0;
    
    // Step 7: 为每个簇生成视点
    ros::Time t_step7 = ros::Time::now();
    int generated_count = 0;
    
    for (auto& cluster : all_clusters) {
        if (generateBestViewpoint(cluster)) {
            generated_count++;
        }
    }
    
    // Step 8: Yaw优化
    // optimizeViewpointYaw(all_clusters);
    
    // Step 9: 可选的增量视点聚类
    if (config_.use_viewpoint_clustering && generated_count > 5) {
        std::vector<FrontierCluster> unclustered;
        std::vector<ViewpointClusterInfo> vp_clusters;
        
        clusterViewpointsIncremental(all_clusters, unclustered, vp_clusters);
        
        // 将锚点作为视点添加
        for (const auto& vpc : vp_clusters) {
            FrontierCluster anchor_cluster;
            anchor_cluster.cluster_id = next_cluster_id_++;
            anchor_cluster.history_node_id = -1;  // 锚点标记
            anchor_cluster.centroid = vpc.centroid;
            anchor_cluster.has_viewpoint = true;
            anchor_cluster.viewpoint_position = vpc.anchor_entry;
            
            Eigen::Vector3f to_center = vpc.centroid - vpc.anchor_entry;
            anchor_cluster.viewpoint_yaw = std::atan2(to_center.y(), to_center.x());
            anchor_cluster.viewpoint_score = 1.0f;  // 高优先级
            
            unclustered.push_back(anchor_cluster);
        }
        
        VP_DEBUG("VP clustering: %lu -> %lu unclustered + %lu clusters",
                 all_clusters.size(), unclustered.size(), vp_clusters.size());
        
        all_clusters = std::move(unclustered);
        viewpoint_clusters_ = std::move(vp_clusters);
    }
    
    // Step 10: 更新当前数据并设置更新标志
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_clusters_ = all_clusters;
        total_clusters_ = static_cast<int>(all_clusters.size());
        total_viewpoints_ = generated_count;
        
        // 设置更新标志，供HistoryGraph轮询
        if (generated_count > 0) {
            viewpoints_updated_ = true;
        }
    }
    
    double t_gen_vp = (ros::Time::now() - t_step7).toSec() * 1000.0;
    double t_total = (ros::Time::now() - t_start).toSec() * 1000.0;
    
    // 找到最近的视点用于显示
    Eigen::Vector3f best_vp_pos(0, 0, 0);
    float best_vp_score = -1.0f;
    for (const auto& c : all_clusters) {
        if (c.has_viewpoint && c.viewpoint_score > best_vp_score) {
            best_vp_score = c.viewpoint_score;
            best_vp_pos = c.viewpoint_position;
        }
    }
    
    ROS_INFO("[VP_MGR] Update: clusters=%zu, vp=%d | Time: frontier=%.1f, group=%.1f, cluster=%.1f, filter=%.1f, gen=%.1fms, total=%.1fms",
             all_clusters.size(), generated_count, 
             t_get_frontier, t_group, t_cluster, t_filter, t_gen_vp, t_total);
    
    // if (generated_count > 0) {
    //     ROS_INFO("[VP_MGR] Best viewpoint: pos=(%.2f,%.2f,%.2f), score=%.3f",
    //              best_vp_pos.x(), best_vp_pos.y(), best_vp_pos.z(), best_vp_score);
    // }
}

// ==================== 空间距离聚类 ====================
void ViewpointManager::clusterByHistoryNode(const std::vector<Eigen::Vector3f>& points,
                                            int node_id,
                                            std::vector<FrontierCluster>& clusters) {
    clusters.clear();
    
    if (points.size() < static_cast<size_t>(config_.min_cluster_size)) return;
    
    std::vector<bool> visited(points.size(), false);
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;
        
        // BFS聚类
        std::queue<size_t> q;
        std::vector<Eigen::Vector3f> cluster_points;
        
        q.push(i);
        visited[i] = true;
        
        while (!q.empty()) {
            size_t curr = q.front();
            q.pop();
            
            cluster_points.push_back(points[curr]);
            
            // 查找邻居
            for (size_t j = 0; j < points.size(); ++j) {
                if (visited[j]) continue;
                
                float dist = (points[j] - points[curr]).norm();
                if (dist <= config_.cluster_eps) {
                    visited[j] = true;
                    q.push(j);
                }
            }
        }
        
        // 检查簇大小
        if (cluster_points.size() >= static_cast<size_t>(config_.min_cluster_size)) {
            FrontierCluster cluster;
            cluster.history_node_id = node_id;
            cluster.points = cluster_points;
            cluster.point_count = static_cast<int>(cluster_points.size());
            
            // 计算质心
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (const auto& pt : cluster_points) {
                sum += pt;
            }
            cluster.centroid = sum / static_cast<float>(cluster_points.size());
            
            // 计算范围
            float max_dist = 0.0f;
            for (const auto& pt : cluster_points) {
                float d = (pt - cluster.centroid).norm();
                if (d > max_dist) max_dist = d;
            }
            cluster.extent = max_dist * 2.0f;
            
            // 估算法向量
            Eigen::Vector3f uav_pos;
            {
                std::lock_guard<std::mutex> lock(uav_mutex_);
                uav_pos = uav_position_;
            }
            Eigen::Vector3f to_centroid = cluster.centroid - uav_pos;
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

// ==================== 法向量相似性聚类 ====================
void ViewpointManager::clusterByNormalSimilarity(
    const std::vector<SurfaceFrontierPoint>& surface_points,
    std::vector<FrontierCluster>& clusters) {
    
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
                
                // 法向量相似性检查
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
            cluster.extent = max_dist * 2.0f;
            
            // 计算主成分方向
            computeClusterPrincipalNormal(cluster);
            
            clusters.push_back(cluster);
        }
    }
}

// ==================== 计算表面法向量 ====================
bool ViewpointManager::computeSurfaceNormal(const Eigen::Vector3f& point, Eigen::Vector3f& normal) {
    if (!global_map_) return false;
    
    // 收集邻域内的 occupied 点
    std::vector<Eigen::Vector3f> neighbor_points;
    float search_radius = config_.cluster_eps;
    float voxel_size = global_map_->getResolution();
    int search_range = static_cast<int>(std::ceil(search_radius / voxel_size));
    
    star_mvf::GlobalVoxelIndex center_idx;
    global_map_->posToIndex(point, center_idx);
    
    for (int dx = -search_range; dx <= search_range; ++dx) {
        for (int dy = -search_range; dy <= search_range; ++dy) {
            for (int dz = -search_range; dz <= search_range; ++dz) {
                star_mvf::GlobalVoxelIndex nei_idx(center_idx.x + dx, center_idx.y + dy, center_idx.z + dz);
                Eigen::Vector3f nei_pos;
                global_map_->indexToPos(nei_idx, nei_pos);
                
                if ((nei_pos - point).norm() > search_radius) continue;
                
                if (global_map_->isOccupied(nei_pos)) {
                    neighbor_points.push_back(nei_pos);
                }
            }
        }
    }
    
    // 需要至少3个点才能计算法向量
    if (neighbor_points.size() < 3) {
        Eigen::Vector3f uav_pos;
        {
            std::lock_guard<std::mutex> lock(uav_mutex_);
            uav_pos = uav_position_;
        }
        Eigen::Vector3f to_point = point - uav_pos;
        to_point.z() = 0.0f;
        if (to_point.norm() > 0.1f) {
            normal = to_point.normalized();
        } else {
            normal = Eigen::Vector3f::UnitX();
        }
        return false;
    }
    
    // PCA 计算法向量
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& p : neighbor_points) mean += p;
    mean /= static_cast<float>(neighbor_points.size());
    
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& p : neighbor_points) {
        Eigen::Vector3f diff = p - mean;
        cov += diff * diff.transpose();
    }
    cov /= static_cast<float>(neighbor_points.size());
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    
    // 最小特征值对应的特征向量（第一列）
    normal = solver.eigenvectors().col(0);
    
    // 确保法向量指向未知区域
    Eigen::Vector3f to_frontier = point - mean;
    if (normal.dot(to_frontier) < 0) {
        normal = -normal;
    }
    
    return true;
}

void ViewpointManager::computeClusterPrincipalNormal(FrontierCluster& cluster) {
    if (cluster.surface_points.size() < 3) {
        cluster.principal_direction = Eigen::Vector3f::UnitX();
        return;
    }
    
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
        cluster.principal_direction = solver.eigenvectors().col(2);
    } else {
        cluster.principal_direction = cluster.normal;
    }
}

// ==================== 分裂大簇 ====================
void ViewpointManager::splitLargeClusters(std::vector<FrontierCluster>& clusters) {
    const int max_iterations = 10;
    int iteration = 0;
    bool changed = true;
    
    while (changed && iteration < max_iterations) {
        changed = false;
        iteration++;
        
        std::vector<FrontierCluster> result;
        
        for (const auto& cluster : clusters) {
            if (cluster.extent > config_.cluster_split_threshold && 
                !cluster.points.empty() && 
                cluster.points.size() >= 4) {
                
                // 交替使用X和Y坐标分裂
                bool use_x = (iteration % 2 == 1);
                float split_val = use_x ? cluster.centroid.x() : cluster.centroid.y();
                
                FrontierCluster c1, c2;
                c1.history_node_id = cluster.history_node_id;
                c2.history_node_id = cluster.history_node_id;
                
                for (const auto& pt : cluster.points) {
                    float val = use_x ? pt.x() : pt.y();
                    if (val < split_val) {
                        c1.points.push_back(pt);
                    } else {
                        c2.points.push_back(pt);
                    }
                }
                
                // 重新计算属性
                auto updateCluster = [](FrontierCluster& c) {
                    if (c.points.size() < 2) return false;
                    c.point_count = static_cast<int>(c.points.size());
                    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
                    for (const auto& pt : c.points) sum += pt;
                    c.centroid = sum / static_cast<float>(c.points.size());
                    float max_dist = 0.0f;
                    for (const auto& pt : c.points) {
                        max_dist = std::max(max_dist, (pt - c.centroid).norm());
                    }
                    c.extent = max_dist * 2.0f;
                    return true;
                };
                
                if (updateCluster(c1)) {
                    c1.cluster_id = next_cluster_id_++;
                    result.push_back(c1);
                    changed = true;
                }
                if (updateCluster(c2)) {
                    c2.cluster_id = next_cluster_id_++;
                    result.push_back(c2);
                    changed = true;
                }
                
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
    
    // 重新分配ID
    for (size_t i = 0; i < clusters.size(); ++i) {
        clusters[i].cluster_id = static_cast<int>(i);
    }
}

// ==================== 过滤头顶区域簇 ====================
void ViewpointManager::filterOverheadClusters(std::vector<FrontierCluster>& clusters) {
    Eigen::Vector3f uav_pos;
    {
        std::lock_guard<std::mutex> lock(uav_mutex_);
        uav_pos = uav_position_;
    }
    
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
            [this, &uav_pos](const FrontierCluster& c) {
                float height_diff = c.centroid.z() - uav_pos.z();
                float horiz_dist = (c.centroid.head<2>() - uav_pos.head<2>()).norm();
                
                return (height_diff > config_.overhead_height_threshold) &&
                       (horiz_dist < config_.overhead_distance_threshold);
            }),
        clusters.end());
}

// ==================== 检查簇覆盖 ====================
bool ViewpointManager::isClusterCovered(const FrontierCluster& cluster) {
    Eigen::Vector3f target_pos = cluster.centroid;
    const float check_radius = 2.0f;
    const int min_unknown_threshold = 10;
    
    int unknown_count = 0;
    for (float dx = -check_radius; dx <= check_radius; dx += 0.5f) {
        for (float dy = -check_radius; dy <= check_radius; dy += 0.5f) {
            for (float dz = -check_radius; dz <= check_radius; dz += 0.5f) {
                Eigen::Vector3f check_pos = target_pos + Eigen::Vector3f(dx, dy, dz);
                if (!global_map_->isInGlobalMapRange(check_pos)) continue;
                
                if (global_map_->isUnknown(check_pos)) {
                    unknown_count++;
                    if (unknown_count >= min_unknown_threshold) {
                        return false;  // 还有未知体素，未被覆盖
                    }
                }
            }
        }
    }
    
    return true;  // 未知体素太少，已被覆盖
}

// ==================== 生成视点 ====================

bool ViewpointManager::generateBestViewpoint(FrontierCluster& cluster) {
    std::vector<Eigen::Vector3f> positions;
    std::vector<float> yaws;
    std::vector<float> scores;
    
    // 根据策略选择采样方法
    if (config_.use_spherical_sampling) {
        sampleViewpointsSpherical(cluster, positions, yaws, scores);
    } else {
        sampleViewpointsCircular(cluster, positions, yaws, scores);
    }
    
    if (positions.empty()) {
        cluster.has_viewpoint = false;
        return false;
    }
    
    // 找最佳视点
    size_t best_idx = 0;
    float best_score = scores[0];
    for (size_t i = 1; i < scores.size(); ++i) {
        if (scores[i] > best_score) {
            best_score = scores[i];
            best_idx = i;
        }
    }
    
    cluster.has_viewpoint = true;
    cluster.viewpoint_position = positions[best_idx];
    cluster.viewpoint_yaw = yaws[best_idx];
    cluster.viewpoint_score = best_score;
    
    return true;
}

// ==================== 圆周采样（优化过滤策略） ====================
// 过滤流程：1.安全性检查 -> 2.可见性检查(FOV方向射线) -> 3.覆盖性检查(FOV边界内) -> 4.综合评分

void ViewpointManager::sampleViewpointsCircular(
    const FrontierCluster& cluster,
    std::vector<Eigen::Vector3f>& positions,
    std::vector<float>& yaws,
    std::vector<float>& scores) {
    
    positions.clear();
    yaws.clear();
    scores.clear();
    
    // 高度采样范围
    float h_min = config_.viewpoint_height_min;
    float h_max = config_.viewpoint_height_max;
    int h_num = std::max(1, config_.viewpoint_height_num);
    float dh = (h_num > 1) ? (h_max - h_min) / static_cast<float>(h_num - 1) : 0.0f;
    
    // 采样半径范围
    float dr = (config_.candidate_rmax - config_.candidate_rmin) / 
               std::max(1.0f, static_cast<float>(config_.candidate_rnum - 1));
    
    // 候选视点结构
    struct Candidate {
        Eigen::Vector3f position;
        float yaw;
        float safety_score;     // 安全性评分
        float visibility_score; // 可见性评分
        float coverage_score;   // 覆盖性评分
        float total_score;      // 总评分
    };
    std::vector<Candidate> valid_candidates;
    
    // 统计信息
    int total_samples = 0;
    int rejected_safety = 0;
    int rejected_visibility = 0;
    int rejected_coverage = 0;
    
    // ==================== 三重循环采样：高度 × 半径 × 角度 ====================
    for (int ih = 0; ih < h_num; ++ih) {
        float viewpoint_z = (h_num > 1) ? (h_min + ih * dh) : config_.viewpoint_height;
        
        for (float rc = config_.candidate_rmin; rc <= config_.candidate_rmax + 1e-3f; rc += dr) {
            for (float phi = -PI; phi < PI; phi += config_.candidate_dphi) {
                total_samples++;
                
                Eigen::Vector3f sample_pos(
                    cluster.centroid.x() + rc * std::cos(phi),
                    cluster.centroid.y() + rc * std::sin(phi),
                    viewpoint_z
                );
                
                // 计算朝向（朝向簇中心）
                Eigen::Vector3f to_cluster = cluster.centroid - sample_pos;
                float yaw = std::atan2(to_cluster.y(), to_cluster.x());
                
                // ========== Step 1: 安全性检查 ==========
                if (!isPositionSafe(sample_pos)) {
                    rejected_safety++;
                    continue;
                }
                
                // ========== Step 2: 可见性检查（从视点沿yaw方向发射射线） ==========
                // 检查从视点到簇中心的射线是否被障碍物遮挡
                float visibility = 0.0f;
                {
                    // 首先检查到簇中心的可见性
                    if (!checkRayVisibility(sample_pos, cluster.centroid)) {
                        rejected_visibility++;
                        continue;  // 簇中心完全被遮挡，直接跳过
                    }
                    
                    // 计算对簇内点的可见性评分
                    visibility = computeVisibilityScore(sample_pos, cluster);
                    
                    if (visibility < config_.min_visibility_ratio) {
                        rejected_visibility++;
                        continue;  // 可见性太低，跳过
                    }
                }
                
                // ========== Step 3: 覆盖性检查（考虑FOV边界） ==========
                float coverage = computeCoverageScore(sample_pos, yaw, cluster);
                
                if (coverage < config_.min_coverage_score) {
                    rejected_coverage++;
                    continue;  // 覆盖性太低，跳过
                }
                
                // ========== Step 4: 计算综合评分 ==========
                // 距离评分：与最优距离越接近评分越高
                float dist = to_cluster.norm();
                float dist_score = std::exp(-0.5f * std::pow((dist - config_.optimal_distance) / 
                                                              (config_.sensor_range_max * 0.3f), 2));
                
                
                // 综合评分 = 距离(20%) + 可见性(30%) + 覆盖性(40%)
                float total_score = 0.30f * dist_score + 
                                   0.30f * visibility + 
                                   0.40f * coverage;
                
                Candidate cand;
                cand.position = sample_pos;
                cand.yaw = yaw;
                cand.safety_score = 1.0f;  // 已通过安全检查
                cand.visibility_score = visibility;
                cand.coverage_score = coverage;
                cand.total_score = total_score;
                
                valid_candidates.push_back(cand);
            }
        }
    }
    
    VP_DEBUG("Cluster %d: samples=%d, passed=%zu, rejected(safety=%d, visibility=%d, coverage=%d)",
             cluster.cluster_id, total_samples, valid_candidates.size(),
             rejected_safety, rejected_visibility, rejected_coverage);
    
    if (valid_candidates.empty()) {
        VP_DEBUG("Cluster %d: no valid candidates", cluster.cluster_id);
        return;
    }
    
    // ========== 选择最佳视点 ==========
    std::sort(valid_candidates.begin(), valid_candidates.end(),
              [](const Candidate& a, const Candidate& b) { 
                  return a.total_score > b.total_score; 
              });
    
    // 返回最佳视点（只返回一个）
    const auto& best = valid_candidates[0];
    positions.push_back(best.position);
    yaws.push_back(best.yaw);
    scores.push_back(best.total_score);
    
    VP_DEBUG("Cluster %d: best viewpoint score=%.3f (vis=%.2f, cov=%.2f)",
             cluster.cluster_id, best.total_score, best.visibility_score, best.coverage_score);
}

// ==================== 球扇形采样 ====================

void ViewpointManager::sampleViewpointsSpherical(
    const FrontierCluster& cluster,
    std::vector<Eigen::Vector3f>& positions,
    std::vector<float>& yaws,
    std::vector<float>& scores) {
    
    positions.clear();
    yaws.clear();
    scores.clear();
    
    float r_min = config_.sensor_range_min;
    float r_max = std::max(config_.sphere_sample_radius, config_.sensor_range_min + 1.0f);
    float theta_max = config_.sphere_theta_max;
    int sample_num = config_.sphere_sample_num * 3;
    
    // 构建局部坐标系：z轴沿主法线的反方向
    Eigen::Vector3f z_axis = -cluster.normal.normalized();
    
    Eigen::Vector3f temp = (std::abs(z_axis.z()) < 0.9f) ? 
                           Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    Eigen::Vector3f x_axis = z_axis.cross(temp).normalized();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    
    Eigen::Matrix3f R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
    
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    struct Candidate {
        Eigen::Vector3f position;
        float yaw;
        float score;
    };
    std::vector<Candidate> candidates;
    
    for (int i = 0; i < sample_num; ++i) {
        float u1 = dist(rng_);
        float u2 = dist(rng_);
        float u3 = dist(rng_);
        
        // r 在 [r_min, r_max] 范围内均匀采样（体积均匀）
        float r_min_cubed = r_min * r_min * r_min;
        float r_max_cubed = r_max * r_max * r_max;
        float r = std::cbrt(r_min_cubed + u1 * (r_max_cubed - r_min_cubed));
        
        // θ = arccos(1 - u2*(1 - cos(θ_max)))
        float cos_theta = 1.0f - u2 * (1.0f - std::cos(theta_max));
        float theta = std::acos(std::max(-1.0f, std::min(1.0f, cos_theta)));
        
        // φ = 2π * u3
        float phi = 2.0f * PI * u3;
        
        // 球坐标转笛卡尔坐标
        float sin_theta = std::sin(theta);
        Eigen::Vector3f local_pos(
            r * sin_theta * std::cos(phi),
            r * sin_theta * std::sin(phi),
            r * std::cos(theta)
        );
        
        // 转换到世界坐标系
        Eigen::Vector3f world_pos = cluster.centroid + R * local_pos;
        
        // 钳制高度
        world_pos.z() = std::max(config_.viewpoint_height_min + 0.1f,
                                 std::min(config_.viewpoint_height_max - 0.1f, world_pos.z()));
        
        // 安全检查
        if (!isPositionSafeForExploration(world_pos)) continue;
        
        // 计算朝向
        Eigen::Vector3f to_cluster = cluster.centroid - world_pos;
        float yaw = std::atan2(to_cluster.y(), to_cluster.x());
        
        // 距离检查
        float dist_to_cluster = to_cluster.norm();
        if (dist_to_cluster < config_.sensor_range_min || 
            dist_to_cluster > config_.sensor_range_max) continue;
        
        // 计算效用评分
        float utility = computeUtilityScore(world_pos, cluster);
        if (utility > 0.01f) {
            candidates.push_back({world_pos, yaw, utility});
        }
    }
    
    if (candidates.empty()) return;
    
    // 按效用排序
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.score > b.score; });
    
    // 对 top-K 进行可见性检查
    int top_k = std::min(5, static_cast<int>(candidates.size()));
    for (int i = 0; i < top_k; ++i) {
        float visibility = computeVisibilityScore(candidates[i].position, cluster);
        if (visibility >= config_.min_visibility_ratio) {
            candidates[i].score = candidates[i].score * 0.7f + visibility * 0.3f;
            positions.push_back(candidates[i].position);
            yaws.push_back(candidates[i].yaw);
            scores.push_back(candidates[i].score);
            break;
        }
    }
    
    // 如果所有候选点可见性都不够，选择效用最高的
    if (positions.empty() && !candidates.empty()) {
        positions.push_back(candidates[0].position);
        yaws.push_back(candidates[0].yaw);
        scores.push_back(candidates[0].score);
    }
}

// ==================== 效用评分 ====================

float ViewpointManager::computeUtilityScore(const Eigen::Vector3f& viewpoint, 
                                            const FrontierCluster& cluster) {
    float utility = 0.0f;
    int visible_count = 0;
    
    // 遍历簇内所有带法向量的前沿点
    for (const auto& sp : cluster.surface_points) {
        if (!sp.has_normal) continue;
        
        Eigen::Vector3f vp_to_frontier = sp.position - viewpoint;
        float dist = vp_to_frontier.norm();
        
        if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) continue;
        
        if (!checkRayVisibility(viewpoint, sp.position)) continue;
        
        visible_count++;
        
        Eigen::Vector3f view_dir = -vp_to_frontier.normalized();
        float cos_theta = std::abs(sp.normal.dot(view_dir));
        utility += cos_theta;
    }
    
    // 如果没有表面点，使用簇的法向量和质心
    if (cluster.surface_points.empty() || visible_count == 0) {
        Eigen::Vector3f vp_to_centroid = cluster.centroid - viewpoint;
        float centroid_dist = vp_to_centroid.norm();
        
        if (centroid_dist >= config_.sensor_range_min && centroid_dist <= config_.sensor_range_max) {
            Eigen::Vector3f view_dir = -vp_to_centroid.normalized();
            float cos_theta = std::abs(cluster.normal.dot(view_dir));
            utility = cos_theta;
        }
    }
    
    if (visible_count > 0) {
        utility /= visible_count;
    }
    
    return utility;
}

// ==================== 可见性评分 ====================

float ViewpointManager::computeVisibilityScore(const Eigen::Vector3f& viewpoint, 
                                               const FrontierCluster& cluster) {
    if (!global_map_) return 1.0f;
    
    // 首先检查到簇中心的直线
    if (!checkRayVisibility(viewpoint, cluster.centroid)) {
        return 0.0f;
    }
    
    if (cluster.points.empty()) {
        return 1.0f;
    }
    
    // 采样簇内的点
    const size_t max_samples = 20;
    size_t step = std::max(size_t(1), cluster.points.size() / max_samples);
    
    int visible_rays = 0;
    int total_rays = 0;
    
    for (size_t i = 0; i < cluster.points.size(); i += step) {
        const auto& point = cluster.points[i];
        
        float dist = (point - viewpoint).norm();
        if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) continue;
        
        total_rays++;
        if (checkRayVisibility(viewpoint, point)) {
            visible_rays++;
        }
    }
    
    if (total_rays == 0) return 1.0f;
    
    return static_cast<float>(visible_rays) / total_rays;
}

bool ViewpointManager::checkRayVisibility(const Eigen::Vector3f& start, const Eigen::Vector3f& end) {
    if (!global_map_) return true;
    
    Eigen::Vector3f dir = end - start;
    float dist = dir.norm();
    if (dist < 0.1f) return true;
    
    dir.normalize();
    float step = 0.1f;
    int num_steps = static_cast<int>(dist / step);
    
    for (int i = 1; i < num_steps; ++i) {
        Eigen::Vector3f check_pos = start + dir * (i * step);
        
        if (!global_map_->isInGlobalMapRange(check_pos)) continue;
        
        if (global_map_->isOccupied(check_pos)) {
            return false;
        }
    }
    
    return true;
}

// ==================== 安全检查 ====================

bool ViewpointManager::isPositionSafe(const Eigen::Vector3f& pos) {
    if (!global_map_) return false;
    
    if (!global_map_->isInGlobalMapRange(pos)) return false;
    if (global_map_->isOccupiedInflate(pos)) return false;
    if (global_map_->isUnknownInflate(pos)) return false;
    
    if (pos.z() < config_.viewpoint_height_min ||
        pos.z() > config_.viewpoint_height_max) {
        return false;
    }
    
    // UAV安全半径检查
    float check_radius = config_.uav_radius + config_.safety_margin;
    float step = 0.2f;
    int num_steps = static_cast<int>(check_radius / step) + 1;
    
    for (int dx = -num_steps; dx <= num_steps; ++dx) {
        for (int dy = -num_steps; dy <= num_steps; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                Eigen::Vector3f check_pos = pos + Eigen::Vector3f(
                    dx * step, dy * step, dz * step * 0.5f);
                
                float dist_xy = std::sqrt(dx*dx + dy*dy) * step;
                if (dist_xy > check_radius) continue;
                
                if (!global_map_->isInGlobalMapRange(check_pos)) continue;
                
                if (global_map_->isOccupied(check_pos)) return false;
                if (global_map_->isUnknown(check_pos)) return false;
            }
        }
    }
    
    return true;
}

bool ViewpointManager::isPositionSafeForExploration(const Eigen::Vector3f& pos) {
    // 宽松检查：允许 UNKNOWN 区域
    if (!global_map_) return true;
    
    if (!global_map_->isInGlobalMapRange(pos)) return false;
    if (global_map_->isOccupied(pos)) return false;
    
    if (pos.z() < config_.viewpoint_height_min || pos.z() > config_.viewpoint_height_max) {
        return false;
    }
    
    // 检查周围关键点
    float check_radius = config_.uav_radius;
    const Eigen::Vector3f offsets[] = {
        {check_radius, 0, 0}, {-check_radius, 0, 0},
        {0, check_radius, 0}, {0, -check_radius, 0},
        {0, 0, -0.3f}
    };
    
    for (const auto& offset : offsets) {
        Eigen::Vector3f check_pos = pos + offset;
        if (!global_map_->isInGlobalMapRange(check_pos)) continue;
        if (global_map_->isOccupied(check_pos)) return false;
    }
    
    return true;
}

// ==================== 评估视点 ====================

float ViewpointManager::evaluateViewpoint(const Eigen::Vector3f& position, 
                                          const FrontierCluster& cluster) {
    if (!isPositionSafe(position)) return -1.0f;
    
    float dist = (position - cluster.centroid).norm();
    if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) {
        return -1.0f;
    }
    
    float dist_score = 1.0f - std::abs(dist - config_.optimal_distance) / config_.sensor_range_max;
    dist_score = std::max(0.0f, dist_score);
    
    float visibility_score = computeVisibilityScore(position, cluster);
    if (visibility_score < 0.5f) return -1.0f;
    
    return 0.5f * dist_score + 0.5f * visibility_score;
}

// ==================== Yaw优化 ====================

void ViewpointManager::optimizeViewpointYaw(std::vector<FrontierCluster>& clusters) {
    // 简化的 Yaw 优化：确保相邻视点的 yaw 变化平滑
    if (clusters.size() < 2) return;
    
    Eigen::Vector3f uav_pos;
    {
        std::lock_guard<std::mutex> lock(uav_mutex_);
        uav_pos = uav_position_;
    }
    
    // 按距离排序
    std::vector<std::pair<float, size_t>> dist_idx;
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (!clusters[i].has_viewpoint) continue;
        float dist = (clusters[i].viewpoint_position - uav_pos).head<2>().norm();
        dist_idx.push_back({dist, i});
    }
    std::sort(dist_idx.begin(), dist_idx.end());
    
    // 记录需要优化的情况
    int optimized_count = 0;
    for (size_t i = 1; i < dist_idx.size(); ++i) {
        size_t curr_idx = dist_idx[i].second;
        size_t prev_idx = dist_idx[i-1].second;
        
        float prev_yaw = clusters[prev_idx].viewpoint_yaw;
        float curr_yaw = clusters[curr_idx].viewpoint_yaw;
        
        float yaw_diff = curr_yaw - prev_yaw;
        while (yaw_diff > PI) yaw_diff -= 2 * PI;
        while (yaw_diff < -PI) yaw_diff += 2 * PI;
        
        if (std::abs(yaw_diff) > 2.09f) {
            optimized_count++;
        }
    }
    
    VP_DEBUG("Yaw optimization: checked %lu pairs, large_diff=%d", 
             dist_idx.size() > 1 ? dist_idx.size() - 1 : 0, optimized_count);
}

// ==================== 增量视点聚类 ====================
void ViewpointManager::clusterViewpointsIncremental(
    const std::vector<FrontierCluster>& input_clusters,
    std::vector<FrontierCluster>& unclustered,
    std::vector<ViewpointClusterInfo>& vp_clusters) {
    
    unclustered.clear();
    vp_clusters.clear();
    
    Eigen::Vector3f uav_pos;
    {
        std::lock_guard<std::mutex> lock(uav_mutex_);
        uav_pos = uav_position_;
    }
    
    // 分离近处和远处视点
    std::vector<FrontierCluster> far_clusters;
    
    for (const auto& cluster : input_clusters) {
        if (!cluster.has_viewpoint) continue;
        
        float dist = (cluster.viewpoint_position - uav_pos).norm();
        if (dist < config_.vp_cluster_min_dist) {
            unclustered.push_back(cluster);
        } else {
            far_clusters.push_back(cluster);
        }
    }
    
    if (far_clusters.empty()) return;
    
    // 增量聚类
    std::vector<bool> clustered(far_clusters.size(), false);
    
    for (size_t i = 0; i < far_clusters.size(); ++i) {
        if (clustered[i]) continue;
        
        ViewpointClusterInfo vpc;
        vpc.cluster_id = next_vp_cluster_id_++;
        vpc.viewpoint_positions.push_back(far_clusters[i].viewpoint_position);
        clustered[i] = true;
        
        // 尝试将其他视点加入聚类
        bool changed = true;
        while (changed) {
            changed = false;
            
            for (size_t j = 0; j < far_clusters.size(); ++j) {
                if (clustered[j]) continue;
                
                const auto& vp_pos = far_clusters[j].viewpoint_position;
                
                // 条件1：与聚类内所有视点的距离 < d_c
                bool dist_ok = true;
                for (const auto& pos : vpc.viewpoint_positions) {
                    if ((vp_pos - pos).norm() > config_.vp_cluster_distance) {
                        dist_ok = false;
                        break;
                    }
                }
                if (!dist_ok) continue;
                
                // 条件2：加入后外接圆半径 < r_c
                Eigen::Vector3f sum = Eigen::Vector3f::Zero();
                for (const auto& pos : vpc.viewpoint_positions) sum += pos;
                sum += vp_pos;
                Eigen::Vector3f new_centroid = sum / static_cast<float>(vpc.viewpoint_positions.size() + 1);
                
                float max_radius = 0.0f;
                for (const auto& pos : vpc.viewpoint_positions) {
                    max_radius = std::max(max_radius, (pos - new_centroid).norm());
                }
                max_radius = std::max(max_radius, (vp_pos - new_centroid).norm());
                
                if (max_radius > config_.vp_cluster_radius) continue;
                
                // 条件3：与聚类内视点无障碍
                bool no_obstacle = true;
                for (const auto& pos : vpc.viewpoint_positions) {
                    // if (history_graph_ && history_graph_->hasObstacleBetween(vp_pos, pos)) {
                    if (history_graph_) {
                        no_obstacle = false;
                        break;
                    }
                }
                if (!no_obstacle) continue;
                
                vpc.viewpoint_positions.push_back(vp_pos);
                clustered[j] = true;
                changed = true;
            }
        }
        
        // 处理聚类结果
        if (vpc.viewpoint_positions.size() >= 2) {
            // 计算质心和半径
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (const auto& pos : vpc.viewpoint_positions) sum += pos;
            vpc.centroid = sum / static_cast<float>(vpc.viewpoint_positions.size());
            
            float max_r = 0.0f;
            for (const auto& pos : vpc.viewpoint_positions) {
                max_r = std::max(max_r, (pos - vpc.centroid).norm());
            }
            vpc.radius = max_r;
            
            generateAnchorPoints(vpc);
            vp_clusters.push_back(vpc);
        } else {
            // 单个视点不形成聚类
            for (size_t k = 0; k < far_clusters.size(); ++k) {
                if (clustered[k] && &far_clusters[k].viewpoint_position == &vpc.viewpoint_positions[0]) {
                    unclustered.push_back(far_clusters[k]);
                    break;
                }
            }
        }
    }
}

void ViewpointManager::generateAnchorPoints(ViewpointClusterInfo& cluster) {
    if (cluster.viewpoint_positions.size() < 2) {
        cluster.anchor_entry = cluster.centroid;
        cluster.anchor_exit = cluster.centroid;
        return;
    }
    
    // PCA 计算主方向
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& pos : cluster.viewpoint_positions) {
        Eigen::Vector3f diff = pos - cluster.centroid;
        cov += diff * diff.transpose();
    }
    cov /= static_cast<float>(cluster.viewpoint_positions.size());
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() == Eigen::Success) {
        cluster.principal_direction = solver.eigenvectors().col(2);
        float sigma = std::sqrt(solver.eigenvalues()(2));
        
        cluster.anchor_entry = cluster.centroid - sigma * cluster.principal_direction;
        cluster.anchor_exit = cluster.centroid + sigma * cluster.principal_direction;
    } else {
        cluster.anchor_entry = cluster.centroid;
        cluster.anchor_exit = cluster.centroid;
    }
}

// ==================== 覆盖评分（考虑FOV边界） ====================
float ViewpointManager::computeCoverageScore(const Eigen::Vector3f& viewpoint, 
                                             float yaw, 
                                             const FrontierCluster& cluster) {
    if (cluster.points.empty()) return 0.0f;
    
    float half_fov_h = config_.sensor_fov_h * 0.5f;
    float half_fov_v = config_.sensor_fov_v * 0.5f;
    
    // 计算当前视点的全局水平视角范围
    // 如果yaw=90°(PI/2), FOV=60°(PI/3), 则视角范围为 [60°, 120°] 即 [PI/3, 2PI/3]
    // float fov_h_min = yaw - half_fov_h;
    // float fov_h_max = yaw + half_fov_h;
    
    int total_points = 0;
    int covered_points = 0;
    
    // 采样簇内的点（最多采样50个点以控制计算量）
    const size_t max_samples = 50;
    size_t step = std::max(size_t(1), cluster.points.size() / max_samples);
    
    for (size_t i = 0; i < cluster.points.size(); i += step) {
        const auto& point = cluster.points[i];
        
        // 计算视点到前沿点的方向
        Eigen::Vector3f to_point = point - viewpoint;
        // float dist = to_point.norm();
        
        // 距离检查
        // if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) {
        //     continue;
        // }
        
        total_points++;
        
        // 计算水平角度（全局坐标系）
        float point_h_angle = std::atan2(to_point.y(), to_point.x());
        
        // 归一化角度差到 [-PI, PI]
        float h_diff = point_h_angle - yaw;
        while (h_diff > PI) h_diff -= 2.0f * PI;
        while (h_diff < -PI) h_diff += 2.0f * PI;
        
        // 检查是否在水平FOV内
        if (std::abs(h_diff) > half_fov_h) {
            continue;
        }
        
        // 计算垂直角度
        float horiz_dist = std::sqrt(to_point.x() * to_point.x() + to_point.y() * to_point.y());
        float point_v_angle = std::atan2(to_point.z(), horiz_dist);
        
        // 检查是否在垂直FOV内
        if (std::abs(point_v_angle) > half_fov_v) {
            continue;
        }
        
        // 该点在FOV范围内
        covered_points++;
    }
    
    if (total_points == 0) return 0.0f;
    
    return static_cast<float>(covered_points) / static_cast<float>(total_points);
}

// ==================== 数据访问 ====================

std::vector<FrontierCluster> ViewpointManager::getClusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_clusters_;
}

int ViewpointManager::getViewpointCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_viewpoints_;
}

bool ViewpointManager::hasValidViewpoints() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_viewpoints_ > 0;
}

FrontierCluster* ViewpointManager::getClusterById(int id) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& cluster : current_clusters_) {
        if (cluster.cluster_id == id) return &cluster;
    }
    return nullptr;
}

} // namespace rog_map
