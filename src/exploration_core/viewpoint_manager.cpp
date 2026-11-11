/**
 * @file viewpoint_manager.cpp
 * @brief 全局前沿聚类与视点生成实现（FUEL 风格，无历史节点依赖）
 *
 * 流程：
 * 1. 获取全部前沿子网格；仅对前沿发生变化的子网格重新提取点云
 * 2. 合并前沿点，按体素去重后建立 VoxelIndex -> point_index 映射
 * 3. 26 邻域体素区域生长（每个体素只处理一次）
 * 4. 分裂过大簇、过滤头顶区域
 * 5. 探索增益、NMS、top-K 过滤
 * 6. 圆周/多高度采样 + 安全/可见性/覆盖过滤 + 评分，输出 top-K 候选视点
 * 7. 全局 ID 继承匹配（质心距离 + 包围盒重叠 + 点数比例）
 */

#include "exploration/viewpoint_manager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

#include "utils/perf_logger.hpp"

// 简化日志宏
#define VP_INFO(fmt, ...) ROS_INFO("[VP_MGR] " fmt, ##__VA_ARGS__)
#define VP_WARN(fmt, ...) ROS_WARN("[VP_MGR] " fmt, ##__VA_ARGS__)
#define VP_DEBUG(fmt, ...) ROS_DEBUG("[VP_MGR] " fmt, ##__VA_ARGS__)
#define VP_INFO_THROTTLE(rate, fmt, ...) ROS_INFO_THROTTLE(rate, "[VP_MGR] " fmt, ##__VA_ARGS__)

namespace rog_map {

namespace {

bool bboxOverlaps(const FrontierCluster& a, const FrontierCluster& b, float eps) {
    return a.bbox_min.x() <= b.bbox_max.x() + eps &&
           b.bbox_min.x() <= a.bbox_max.x() + eps &&
           a.bbox_min.y() <= b.bbox_max.y() + eps &&
           b.bbox_min.y() <= a.bbox_max.y() + eps &&
           a.bbox_min.z() <= b.bbox_max.z() + eps &&
           b.bbox_min.z() <= a.bbox_max.z() + eps;
}

}  // namespace

ViewpointManager::ViewpointManager() = default;

// ==================== 初始化 ====================

void ViewpointManager::init(ros::NodeHandle& nh, GlobalMap::Ptr global_map) {
    nh_ = nh;
    global_map_ = global_map;
    if (!global_map_) {
        VP_WARN("Global map is null, viewpoint manager will stay idle");
        return;
    }

    // 更新频率
    nh_.param("viewpoint_manager/update_rate", config_.update_rate, 5.0f);

    // 传感器参数
    nh_.param("viewpoint_manager/sensor_range_min", config_.sensor_range_min, 0.5f);
    nh_.param("viewpoint_manager/sensor_range_max", config_.sensor_range_max, 5.0f);
    nh_.param("viewpoint_manager/sensor_fov_h", config_.sensor_fov_h, 1.57f);
    nh_.param("viewpoint_manager/sensor_fov_v", config_.sensor_fov_v, 1.22f);

    // 视点采样参数
    nh_.param("viewpoint_manager/viewpoint_height", config_.viewpoint_height, 1.0f);
    nh_.param("viewpoint_manager/viewpoint_height_min", config_.viewpoint_height_min, 0.5f);
    nh_.param("viewpoint_manager/viewpoint_height_max", config_.viewpoint_height_max, 2.0f);
    nh_.param("viewpoint_manager/viewpoint_height_num", config_.viewpoint_height_num, 1);
    nh_.param("viewpoint_manager/candidate_rmin", config_.candidate_rmin, 2.0f);
    nh_.param("viewpoint_manager/candidate_rmax", config_.candidate_rmax, 5.0f);
    nh_.param("viewpoint_manager/candidate_rnum", config_.candidate_rnum, 3);
    nh_.param("viewpoint_manager/candidate_dphi", config_.candidate_dphi, 0.5f);
    nh_.param("viewpoint_manager/optimal_distance", config_.optimal_distance, 3.0f);

    // 前沿聚类参数
    nh_.param("viewpoint_manager/min_cluster_size", config_.min_cluster_size, 10);
    nh_.param("viewpoint_manager/cluster_split_threshold", config_.cluster_split_threshold, 3.0f);
    nh_.param("viewpoint_manager/cluster_match_dist", config_.cluster_match_dist, 1.0f);
    nh_.param("viewpoint_manager/max_clusters", config_.max_clusters, 30);
    nh_.param("viewpoint_manager/gain_max_expand_step", config_.gain_max_expand_step, 6);

    // 安全参数
    nh_.param("viewpoint_manager/uav_radius", config_.uav_radius, 0.35f);
    nh_.param("viewpoint_manager/safety_margin", config_.safety_margin, 0.2f);

    // 滤波参数
    nh_.param("viewpoint_manager/min_viewpoint_distance", config_.min_viewpoint_distance, 0.8f);
    nh_.param("viewpoint_manager/min_coverage_score", config_.min_coverage_score, 0.3f);
    nh_.param("viewpoint_manager/min_visibility_ratio", config_.min_visibility_ratio, 0.3f);

    // 前沿点过滤参数
    nh_.param("rog_map/global_map/frontier_height_min", config_.frontier_height_min, 0.5f);
    nh_.param("rog_map/global_map/frontier_height_max", config_.frontier_height_max, 2.5f);

    // 头顶区域过滤参数
    nh_.param("viewpoint_manager/overhead_height_threshold",
              config_.overhead_height_threshold, 0.3f);
    nh_.param("viewpoint_manager/overhead_distance_threshold",
              config_.overhead_distance_threshold, 3.0f);

    // 候选视点数量由全局调度统一控制
    nh_.param("global_scheduler/top_view_num", config_.top_view_num, 5);
    nh_.param("global_scheduler/max_decay", config_.max_decay, 0.8f);

    gain_visit_stamp_.assign(global_map_->getTotalVoxelCount(), 0);

    VP_INFO("Initialized: rate=%.1fHz, r=[%.1f,%.1f], top_view=%d, max_decay=%.2f",
            config_.update_rate, config_.candidate_rmin, config_.candidate_rmax,
            config_.top_view_num, config_.max_decay);
}

void ViewpointManager::start() {
    if (config_.update_rate <= 0.0f) {
        VP_WARN("Invalid update_rate %.2f, timer not started", config_.update_rate);
        return;
    }
    timer_ = nh_.createTimer(ros::Duration(1.0 / config_.update_rate),
                             &ViewpointManager::timerCallback, this);
}

void ViewpointManager::updateUAVState(const Eigen::Vector3f& position, float yaw) {
    std::lock_guard<std::mutex> lock(uav_mutex_);
    uav_position_ = position;
    uav_yaw_ = yaw;
    has_uav_state_ = true;
}

void ViewpointManager::timerCallback(const ros::TimerEvent& /*e*/) {
    if (!has_uav_state_) return;
    update();
}

void ViewpointManager::triggerUpdate() {
    update();
}

// ==================== 主更新函数 ====================

void ViewpointManager::update() {
    std::lock_guard<std::mutex> update_lock(update_mutex_);
    const ros::Time t_start = ros::Time::now();

    if (!global_map_) {
        VP_WARN("Global map not available");
        return;
    }

    // ---------- Step 1: 前沿子网格与变化集合 ----------
    const ros::Time t_step1 = ros::Time::now();
    std::vector<int> frontier_subgrids;
    global_map_->getFrontierSubGrids(frontier_subgrids);

    std::vector<int> changed_subgrids;
    global_map_->getAndClearFrontierChangedSubGrids(changed_subgrids);

    // ---------- Step 2: 只刷新变化子网格的点云缓存 ----------
    for (int sg_id : changed_subgrids) {
        std::vector<Eigen::Vector3f> points;
        if (global_map_->getSubGridFrontierPoints(sg_id, points) && !points.empty()) {
            subgrid_points_cache_[sg_id] = std::move(points);
        } else {
            subgrid_points_cache_.erase(sg_id);
        }
    }

    // ---------- Step 3: 合并活跃子网格点云 ----------
    std::vector<Eigen::Vector3f> all_points;
    std::vector<int> active_ids;
    active_ids.reserve(frontier_subgrids.size());
    for (int sg_id : frontier_subgrids) {
        auto it = subgrid_points_cache_.find(sg_id);
        if (it == subgrid_points_cache_.end()) {
            std::vector<Eigen::Vector3f> points;
            if (!global_map_->getSubGridFrontierPoints(sg_id, points) || points.empty()) {
                continue;
            }
            it = subgrid_points_cache_.emplace(sg_id, std::move(points)).first;
        }
        if (it->second.empty()) continue;
        active_ids.push_back(sg_id);
        all_points.insert(all_points.end(), it->second.begin(), it->second.end());
    }

    // 清理不再活跃的缓存
    const std::unordered_set<int> active_set(active_ids.begin(), active_ids.end());
    for (auto it = subgrid_points_cache_.begin(); it != subgrid_points_cache_.end();) {
        if (active_set.count(it->first) == 0) {
            it = subgrid_points_cache_.erase(it);
        } else {
            ++it;
        }
    }

    const size_t total_points = all_points.size();
    const double t_frontier = (ros::Time::now() - t_step1).toSec() * 1000.0;

    if (all_points.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_clusters_.clear();
        total_clusters_ = 0;
        total_viewpoints_ = 0;
        ROS_INFO_THROTTLE(2.0, "[SCHED][SNAPSHOT] frontier_points=0 changed_sg=%zu active_sg=0",
                          changed_subgrids.size());
        return;
    }

    // ---------- Step 4: 全局体素区域生长聚类 ----------
    const ros::Time t_step4 = ros::Time::now();
    std::vector<FrontierCluster> clusters;
    clusterFrontiers(all_points, clusters);
    splitLargeClusters(clusters);
    filterOverheadClusters(clusters);
    const size_t raw_cluster_count = clusters.size();
    const double t_cluster = (ros::Time::now() - t_step4).toSec() * 1000.0;

    if (clusters.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_clusters_.clear();
        total_clusters_ = 0;
        total_viewpoints_ = 0;
        ROS_INFO_THROTTLE(2.0,
                          "[SCHED][SNAPSHOT] frontier_points=%zu changed_sg=%zu active_sg=%zu "
                          "raw_clusters=0",
                          total_points, changed_subgrids.size(), active_ids.size());
        return;
    }

    // ---------- Step 5: 增益 + NMS + top-K ----------
    const ros::Time t_step5 = ros::Time::now();
    for (auto& cluster : clusters) {
        cluster.exploration_gain = calculateClusterGain(cluster);
    }
    std::stable_sort(clusters.begin(), clusters.end(),
                     [](const FrontierCluster& a, const FrontierCluster& b) {
                         return a.exploration_gain > b.exploration_gain;
                     });

    std::vector<FrontierCluster> kept;
    kept.reserve(std::min<size_t>(clusters.size(), static_cast<size_t>(config_.max_clusters)));
    constexpr float kNmsRadius = 2.5f;
    for (auto& cluster : clusters) {
        bool suppressed = false;
        for (const auto& selected : kept) {
            if ((cluster.centroid - selected.centroid).norm() < kNmsRadius) {
                suppressed = true;
                break;
            }
        }
        if (suppressed) continue;
        kept.push_back(std::move(cluster));
        if (static_cast<int>(kept.size()) >= config_.max_clusters) break;
    }
    const double t_filter = (ros::Time::now() - t_step5).toSec() * 1000.0;

    // ---------- Step 6: 为每个簇生成 top-K 候选视点 ----------
    const ros::Time t_step6 = ros::Time::now();
    int generated_count = 0;
    for (auto& cluster : kept) {
        if (generateBestViewpoint(cluster)) ++generated_count;
    }
    const double t_generate = (ros::Time::now() - t_step6).toSec() * 1000.0;

    // ---------- Step 7: 全局 ID 继承 ----------
    std::vector<FrontierCluster> previous_clusters;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        previous_clusters = current_clusters_;
    }
    matchClustersGlobal(kept, previous_clusters);

    // ---------- Step 8: 更新输出 ----------
    int output_clusters = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_clusters_ = std::move(kept);
        total_clusters_ = static_cast<int>(current_clusters_.size());
        total_viewpoints_ = generated_count;
        output_clusters = total_clusters_;
    }

    const double t_total = (ros::Time::now() - t_start).toSec() * 1000.0;
    ROS_INFO_THROTTLE(2.0,
                      "[SCHED][SNAPSHOT] frontier_points=%zu changed_sg=%zu active_sg=%zu "
                      "raw_clusters=%zu output_clusters=%d viewpoints=%d total_ms=%.2f",
                      total_points, changed_subgrids.size(), active_ids.size(),
                      raw_cluster_count, output_clusters, generated_count, t_total);

    size_t cache_point_capacity = 0;
    for (const auto& kv : subgrid_points_cache_) cache_point_capacity += kv.second.capacity();
    PERF_LOG.logf("VIEWPOINT", "UPDATE",
                  "frontier_points=%zu changed_sg=%zu active_sg=%zu raw_clusters=%zu "
                  "output_clusters=%d generated=%d frontier_ms=%.3f cluster_ms=%.3f "
                  "filter_ms=%.3f generate_ms=%.3f total_ms=%.3f cache_sg=%zu "
                  "cache_point_capacity=%zu",
                  total_points, changed_subgrids.size(), active_ids.size(), raw_cluster_count,
                  output_clusters, generated_count, t_frontier, t_cluster, t_filter,
                  t_generate, t_total, subgrid_points_cache_.size(), cache_point_capacity);
}

// ==================== 全局体素区域生长聚类 ====================

void ViewpointManager::clusterFrontiers(const std::vector<Eigen::Vector3f>& points,
                                        std::vector<FrontierCluster>& clusters) {
    clusters.clear();
    if (points.empty()) return;

    // 同一体素可能被多个子网格重复提供，先按体素去重
    std::unordered_map<star_mvf::VoxelIndex, size_t, star_mvf::VoxelIndexHash> point_index;
    point_index.reserve(points.size() * 2);
    std::vector<Eigen::Vector3f> unique_points;
    unique_points.reserve(points.size());
    std::vector<star_mvf::VoxelIndex> voxels;
    voxels.reserve(points.size());
    for (const auto& point : points) {
        const star_mvf::VoxelIndex voxel = global_map_->posToIndex(point);
        const auto inserted = point_index.emplace(voxel, unique_points.size());
        if (!inserted.second) continue;
        unique_points.push_back(point);
        voxels.push_back(voxel);
    }

    if (unique_points.size() < static_cast<size_t>(std::max(1, config_.min_cluster_size))) {
        return;
    }

    std::vector<uint8_t> visited(unique_points.size(), 0);
    std::queue<size_t> queue;
    std::vector<size_t> component;

    for (size_t seed = 0; seed < unique_points.size(); ++seed) {
        if (visited[seed]) continue;
        visited[seed] = 1;
        queue.push(seed);
        component.clear();

        while (!queue.empty()) {
            const size_t current = queue.front();
            queue.pop();
            component.push_back(current);

            const auto& voxel = voxels[current];
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        const star_mvf::VoxelIndex neighbor(voxel.x + dx, voxel.y + dy,
                                                            voxel.z + dz);
                        const auto it = point_index.find(neighbor);
                        if (it == point_index.end() || visited[it->second]) continue;
                        visited[it->second] = 1;
                        queue.push(it->second);
                    }
                }
            }
        }

        if (component.size() < static_cast<size_t>(std::max(1, config_.min_cluster_size))) {
            continue;
        }

        FrontierCluster cluster;
        cluster.points.reserve(component.size());
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for (size_t index : component) {
            const auto& point = unique_points[index];
            cluster.points.push_back(point);
            sum += point;
            cluster.bbox_min = cluster.bbox_min.cwiseMin(point);
            cluster.bbox_max = cluster.bbox_max.cwiseMax(point);
        }
        cluster.point_count = static_cast<int>(cluster.points.size());
        cluster.centroid = sum / static_cast<float>(cluster.points.size());

        float max_dist = 0.0f;
        for (const auto& point : cluster.points) {
            max_dist = std::max(max_dist, (point - cluster.centroid).norm());
        }
        cluster.extent = max_dist * 2.0f;
        clusters.push_back(std::move(cluster));
    }
}

// ==================== 分裂大簇 ====================

void ViewpointManager::splitLargeClusters(std::vector<FrontierCluster>& clusters) {
    constexpr int kMaxIterations = 10;

    for (int iteration = 1; iteration <= kMaxIterations; ++iteration) {
        bool changed = false;
        std::vector<FrontierCluster> result;
        result.reserve(clusters.size() * 2);

        auto refreshCluster = [](FrontierCluster& cluster) -> bool {
            if (cluster.points.empty()) return false;
            cluster.point_count = static_cast<int>(cluster.points.size());
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (const auto& point : cluster.points) {
                sum += point;
                cluster.bbox_min = cluster.bbox_min.cwiseMin(point);
                cluster.bbox_max = cluster.bbox_max.cwiseMax(point);
            }
            cluster.centroid = sum / static_cast<float>(cluster.points.size());
            float max_dist = 0.0f;
            for (const auto& point : cluster.points) {
                max_dist = std::max(max_dist, (point - cluster.centroid).norm());
            }
            cluster.extent = max_dist * 2.0f;
            return true;
        };

        for (auto& cluster : clusters) {
            if (cluster.extent <= config_.cluster_split_threshold ||
                cluster.points.size() < 4) {
                result.push_back(std::move(cluster));
                continue;
            }

            const bool use_x = (iteration % 2 == 1);
            const float split_val = use_x ? cluster.centroid.x() : cluster.centroid.y();

            FrontierCluster c1;
            FrontierCluster c2;
            c1.points.reserve(cluster.points.size());
            c2.points.reserve(cluster.points.size());
            for (const auto& point : cluster.points) {
                const float value = use_x ? point.x() : point.y();
                if (value < split_val) {
                    c1.points.push_back(point);
                } else {
                    c2.points.push_back(point);
                }
            }

            if (!refreshCluster(c1) || !refreshCluster(c2) ||
                c1.points.size() < 2 || c2.points.size() < 2) {
                result.push_back(std::move(cluster));
                continue;
            }

            result.push_back(std::move(c1));
            result.push_back(std::move(c2));
            changed = true;
        }

        clusters = std::move(result);
        if (!changed) break;
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
                       [this, &uav_pos](const FrontierCluster& cluster) {
                           const float height_diff = cluster.centroid.z() - uav_pos.z();
                           const float horiz_dist =
                               (cluster.centroid.head<2>() - uav_pos.head<2>()).norm();
                           return (height_diff > config_.overhead_height_threshold) &&
                                  (horiz_dist < config_.overhead_distance_threshold);
                       }),
        clusters.end());
}

// ==================== 全局 ID 继承 ====================

void ViewpointManager::matchClustersGlobal(std::vector<FrontierCluster>& new_clusters,
                                           const std::vector<FrontierCluster>& old_clusters) {
    if (new_clusters.empty()) return;

    if (old_clusters.empty()) {
        for (auto& cluster : new_clusters) {
            cluster.cluster_id = global_cluster_id_++;
        }
        return;
    }

    struct Match {
        float distance;
        size_t new_index;
        size_t old_index;
    };

    std::vector<Match> matches;
    constexpr float kBboxEps = 1e-4f;
    for (size_t ni = 0; ni < new_clusters.size(); ++ni) {
        for (size_t oi = 0; oi < old_clusters.size(); ++oi) {
            const auto& nc = new_clusters[ni];
            const auto& oc = old_clusters[oi];
            const float distance = (nc.centroid - oc.centroid).norm();
            if (distance > config_.cluster_match_dist) continue;
            if (!bboxOverlaps(nc, oc, kBboxEps)) continue;

            const int min_count = std::min(nc.point_count, oc.point_count);
            const int max_count = std::max(nc.point_count, oc.point_count);
            if (max_count <= 0 ||
                static_cast<float>(min_count) / static_cast<float>(max_count) < 0.5f) {
                continue;
            }
            matches.push_back({distance, ni, oi});
        }
    }

    std::sort(matches.begin(), matches.end(), [](const Match& a, const Match& b) {
        return a.distance < b.distance;
    });

    std::vector<uint8_t> used_new(new_clusters.size(), 0);
    std::vector<uint8_t> used_old(old_clusters.size(), 0);
    for (const auto& match : matches) {
        if (used_new[match.new_index] || used_old[match.old_index]) continue;
        new_clusters[match.new_index].cluster_id = old_clusters[match.old_index].cluster_id;
        used_new[match.new_index] = 1;
        used_old[match.old_index] = 1;
    }

    for (size_t ni = 0; ni < new_clusters.size(); ++ni) {
        if (!used_new[ni]) {
            new_clusters[ni].cluster_id = global_cluster_id_++;
        }
    }
}

// ==================== 生成 top-K 候选视点 ====================

bool ViewpointManager::generateBestViewpoint(FrontierCluster& cluster) {
    std::vector<Eigen::Vector3f> positions;
    std::vector<float> yaws;
    std::vector<float> scores;
    sampleViewpointsCircular(cluster, positions, yaws, scores);

    cluster.candidates.clear();
    if (positions.empty() || yaws.size() != positions.size() ||
        scores.size() != positions.size()) {
        cluster.has_viewpoint = false;
        return false;
    }

    const float best_score = scores.front();
    const float threshold = std::max(0.0f, config_.max_decay) * best_score;
    const int max_candidates = std::max(1, config_.top_view_num);

    std::vector<size_t> selected;
    selected.reserve(static_cast<size_t>(max_candidates));
    for (size_t i = 0; i < positions.size() && static_cast<int>(selected.size()) < max_candidates;
         ++i) {
        if (scores[i] < threshold) continue;
        bool too_close = false;
        for (size_t kept_index : selected) {
            if ((positions[i] - positions[kept_index]).norm() < config_.min_viewpoint_distance) {
                too_close = true;
                break;
            }
        }
        if (too_close) continue;
        selected.push_back(i);
    }
    if (selected.empty()) selected.push_back(0);  // 至少保留最佳视点

    cluster.candidates.reserve(selected.size());
    for (size_t index : selected) {
        ExplorationViewpoint viewpoint;
        viewpoint.position = positions[index].cast<double>();
        viewpoint.yaw = static_cast<double>(yaws[index]);
        viewpoint.view_score = static_cast<double>(scores[index]);
        cluster.candidates.push_back(viewpoint);
    }

    const size_t best_index = selected.front();
    cluster.has_viewpoint = true;
    cluster.viewpoint_position = positions[best_index];
    cluster.viewpoint_yaw = yaws[best_index];
    cluster.viewpoint_score = scores[best_index];
    return true;
}

// ==================== 圆周/多高度采样 ====================
// 过滤流程：安全性 -> 可见性(FOV方向射线) -> 覆盖率(FOV边界内) -> 综合评分

void ViewpointManager::sampleViewpointsCircular(const FrontierCluster& cluster,
                                                std::vector<Eigen::Vector3f>& positions,
                                                std::vector<float>& yaws,
                                                std::vector<float>& scores) {
    positions.clear();
    yaws.clear();
    scores.clear();

    Eigen::Vector3f uav_pos;
    {
        std::lock_guard<std::mutex> lock(uav_mutex_);
        uav_pos = uav_position_;
    }

    const float h_min = config_.viewpoint_height_min;
    const float h_max = config_.viewpoint_height_max;
    const int h_num = std::max(1, config_.viewpoint_height_num);
    const float dh = (h_num > 1) ? (h_max - h_min) / static_cast<float>(h_num - 1) : 0.0f;

    const float dr = (config_.candidate_rmax - config_.candidate_rmin) /
                     std::max(1.0f, static_cast<float>(config_.candidate_rnum - 1));

    struct Candidate {
        Eigen::Vector3f position;
        float yaw = 0.0f;
        float visibility_score = 0.0f;
        float coverage_score = 0.0f;
        float total_score = 0.0f;
    };
    std::vector<Candidate> valid_candidates;

    int total_samples = 0;
    int rejected_safety = 0;
    int rejected_visibility = 0;
    int rejected_coverage = 0;

    for (int ih = 0; ih < h_num; ++ih) {
        const float viewpoint_z =
            (h_num > 1) ? (h_min + ih * dh) : config_.viewpoint_height;

        for (float radius = config_.candidate_rmin;
             radius <= config_.candidate_rmax + 1e-3f; radius += dr) {
            for (float phi = -PI; phi < PI; phi += config_.candidate_dphi) {
                ++total_samples;

                const Eigen::Vector3f sample_pos(
                    cluster.centroid.x() + radius * std::cos(phi),
                    cluster.centroid.y() + radius * std::sin(phi), viewpoint_z);

                const Eigen::Vector3f to_cluster = cluster.centroid - sample_pos;
                const float yaw = std::atan2(to_cluster.y(), to_cluster.x());

                // Step 1: 安全性
                if (!isPositionSafe(sample_pos)) {
                    ++rejected_safety;
                    continue;
                }
                if ((sample_pos - uav_pos).norm() < 1.0f) {
                    ++rejected_visibility;
                    continue;
                }

                // Step 2: 可见性
                if (!checkRayVisibility(sample_pos, cluster.centroid)) {
                    ++rejected_visibility;
                    continue;
                }
                const float visibility = computeVisibilityScore(sample_pos, cluster);
                if (visibility < config_.min_visibility_ratio) {
                    ++rejected_visibility;
                    continue;
                }

                // Step 3: 覆盖率
                const float coverage = computeCoverageScore(sample_pos, yaw, cluster);
                if (coverage < config_.min_coverage_score) {
                    ++rejected_coverage;
                    continue;
                }

                // Step 4: 综合评分
                const float dist = to_cluster.norm();
                const float dist_score =
                    std::exp(-0.5f * std::pow((dist - config_.optimal_distance) /
                                                  (config_.sensor_range_max * 0.3f),
                                              2));
                const float total_score =
                    0.30f * dist_score + 0.30f * visibility + 0.40f * coverage;

                Candidate candidate;
                candidate.position = sample_pos;
                candidate.yaw = yaw;
                candidate.visibility_score = visibility;
                candidate.coverage_score = coverage;
                candidate.total_score = total_score;
                valid_candidates.push_back(candidate);
            }
        }
    }

    VP_DEBUG("Cluster %d: samples=%d, passed=%zu, rejected(safety=%d, visibility=%d, "
             "coverage=%d)",
             cluster.cluster_id, total_samples, valid_candidates.size(), rejected_safety,
             rejected_visibility, rejected_coverage);

    if (valid_candidates.empty()) {
        VP_DEBUG("Cluster %d: no valid candidates", cluster.cluster_id);
        return;
    }

    std::sort(valid_candidates.begin(), valid_candidates.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.total_score > b.total_score;
              });

    positions.reserve(valid_candidates.size());
    yaws.reserve(valid_candidates.size());
    scores.reserve(valid_candidates.size());
    for (const auto& candidate : valid_candidates) {
        positions.push_back(candidate.position);
        yaws.push_back(candidate.yaw);
        scores.push_back(candidate.total_score);
    }

    VP_DEBUG("Cluster %d: best viewpoint score=%.3f (vis=%.2f, cov=%.2f)",
             cluster.cluster_id, valid_candidates.front().total_score,
             valid_candidates.front().visibility_score,
             valid_candidates.front().coverage_score);
}

// ==================== 可见性评分 ====================

float ViewpointManager::computeVisibilityScore(const Eigen::Vector3f& viewpoint,
                                               const FrontierCluster& cluster) {
    if (!global_map_) return 1.0f;

    if (!checkRayVisibility(viewpoint, cluster.centroid)) {
        return 0.0f;
    }
    if (cluster.points.empty()) {
        return 1.0f;
    }

    const size_t max_samples = 20;
    const size_t step = std::max(size_t(1), cluster.points.size() / max_samples);

    int visible_rays = 0;
    int total_rays = 0;
    for (size_t i = 0; i < cluster.points.size(); i += step) {
        const auto& point = cluster.points[i];
        const float dist = (point - viewpoint).norm();
        if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) continue;

        ++total_rays;
        if (checkRayVisibility(viewpoint, point)) ++visible_rays;
    }

    if (total_rays == 0) return 1.0f;
    return static_cast<float>(visible_rays) / static_cast<float>(total_rays);
}

bool ViewpointManager::checkRayVisibility(const Eigen::Vector3f& start,
                                          const Eigen::Vector3f& end) {
    if (!global_map_) return true;

    Eigen::Vector3f dir = end - start;
    const float dist = dir.norm();
    if (dist < 0.1f) return true;

    dir.normalize();
    const float step = 0.1f;
    const int num_steps = static_cast<int>(dist / step);

    for (int i = 1; i < num_steps; ++i) {
        const Eigen::Vector3f check_pos = start + dir * (i * step);
        if (!global_map_->isInGlobalMapRange(check_pos)) continue;
        if (global_map_->isOccupied(check_pos)) return false;
    }
    return true;
}

// ==================== 安全检查 ====================

bool ViewpointManager::isPositionSafe(const Eigen::Vector3f& pos) {
    if (!global_map_) return false;
    if (!global_map_->isInGlobalMapRange(pos)) return false;
    if (global_map_->isOccupiedInflate(pos)) return false;
    if (global_map_->isUnknown(pos)) return false;

    if (pos.z() < config_.viewpoint_height_min || pos.z() > config_.viewpoint_height_max) {
        return false;
    }

    const float check_radius = config_.uav_radius + config_.safety_margin;
    const float step = 0.2f;
    const int num_steps = static_cast<int>(check_radius / step) + 1;

    for (int dx = -num_steps; dx <= num_steps; ++dx) {
        for (int dy = -num_steps; dy <= num_steps; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const Eigen::Vector3f check_pos =
                    pos + Eigen::Vector3f(dx * step, dy * step, dz * step * 0.5f);
                const float dist_xy = std::sqrt(dx * dx + dy * dy) * step;
                if (dist_xy > check_radius) continue;
                if (!global_map_->isInGlobalMapRange(check_pos)) continue;
                if (global_map_->isOccupied(check_pos)) return false;
                if (global_map_->isUnknown(check_pos)) return false;
            }
        }
    }
    return true;
}

// ==================== 覆盖评分（考虑FOV边界） ====================

float ViewpointManager::computeCoverageScore(const Eigen::Vector3f& viewpoint, float yaw,
                                             const FrontierCluster& cluster) {
    if (cluster.points.empty()) return 0.0f;

    const float half_fov_h = config_.sensor_fov_h * 0.5f;
    const float half_fov_v = config_.sensor_fov_v * 0.5f;

    int total_points = 0;
    int covered_points = 0;

    const size_t max_samples = 50;
    const size_t step = std::max(size_t(1), cluster.points.size() / max_samples);

    for (size_t i = 0; i < cluster.points.size(); i += step) {
        const auto& point = cluster.points[i];
        const Eigen::Vector3f to_point = point - viewpoint;
        ++total_points;

        const float point_h_angle = std::atan2(to_point.y(), to_point.x());
        float h_diff = point_h_angle - yaw;
        while (h_diff > PI) h_diff -= 2.0f * PI;
        while (h_diff < -PI) h_diff += 2.0f * PI;
        if (std::abs(h_diff) > half_fov_h) continue;

        const float horiz_dist =
            std::sqrt(to_point.x() * to_point.x() + to_point.y() * to_point.y());
        const float point_v_angle = std::atan2(to_point.z(), horiz_dist);
        if (std::abs(point_v_angle) > half_fov_v) continue;

        ++covered_points;
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

// ==================== 探索增益 ====================

float ViewpointManager::calculateClusterGain(const FrontierCluster& cluster) {
    if (cluster.points.empty() || !global_map_) return 0.0f;

    if (gain_visit_stamp_.size() != global_map_->getTotalVoxelCount()) {
        gain_visit_stamp_.assign(global_map_->getTotalVoxelCount(), 0);
    }
    if (++gain_generation_ == 0) {
        std::fill(gain_visit_stamp_.begin(), gain_visit_stamp_.end(), 0);
        gain_generation_ = 1;
    }

    auto markFirstVisit = [&](const star_mvf::VoxelIndex& index) {
        size_t address = 0;
        if (!global_map_->voxelToLinearAddress(index, address)) return false;
        if (gain_visit_stamp_[address] == gain_generation_) return false;
        gain_visit_stamp_[address] = gain_generation_;
        return true;
    };

    std::queue<std::pair<star_mvf::VoxelIndex, int>> queue;
    for (const auto& point : cluster.points) {
        const star_mvf::VoxelIndex index = global_map_->posToIndex(point);
        if (markFirstVisit(index)) queue.push({index, 0});
    }

    int unknown_count = 0;
    static constexpr int kDx[6] = {1, -1, 0, 0, 0, 0};
    static constexpr int kDy[6] = {0, 0, 1, -1, 0, 0};
    static constexpr int kDz[6] = {0, 0, 0, 0, 1, -1};
    while (!queue.empty()) {
        const auto current = queue.front();
        queue.pop();
        if (current.second >= config_.gain_max_expand_step) continue;
        for (int i = 0; i < 6; ++i) {
            star_mvf::VoxelIndex neighbor = current.first;
            neighbor.x += kDx[i];
            neighbor.y += kDy[i];
            neighbor.z += kDz[i];
            if (!markFirstVisit(neighbor)) continue;
            if (!global_map_->isUnknown(neighbor)) continue;
            ++unknown_count;
            queue.push({neighbor, current.second + 1});
        }
    }
    return static_cast<float>(unknown_count);
}

}  // namespace rog_map
