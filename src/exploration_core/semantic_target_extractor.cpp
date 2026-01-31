/**
 * @file semantic_target_extractor.cpp
 * @brief 语义目标提取器实现 - 基于探索率触发的增量式几何显著性检测
 */

#include "exploration/semantic_target_extractor.h"
#include <unordered_set>

namespace rog_map {

// ==================== 初始化 ====================
void SemanticTargetExtractor::init(ros::NodeHandle& nh, star_mvf::GlobalMap* global_map) {
    nh_ = nh;
    global_map_ = global_map;
    analyzed_count_ = 0;
    pruned_wall_count_ = 0;
    
    // 加载配置（从 semantic_extractor 命名空间）
    loadConfig(nh_, "semantic_extractor");
    
    // 设置 GlobalMap 的探索率阈值
    if (global_map_) {
        global_map_->setSemanticExplorationThreshold(config_.exploration_threshold);
    }
    
    // 初始化发布器
    target_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/semantic_targets_vis", 1, true);
    
    // 初始化定时器
    process_timer_ = nh_.createTimer(
        ros::Duration(1.0 / config_.process_rate),
        &SemanticTargetExtractor::processTimerCallback, this);
    
    vis_timer_ = nh_.createTimer(
        ros::Duration(1.0 / config_.vis_rate),
        &SemanticTargetExtractor::visualizationTimerCallback, this);
    
    // ROS_INFO("[SemanticTargetExtractor] Initialized: process_rate=%.1fHz, vis_rate=%.1fHz, exploration_threshold=%.2f",
    //          config_.process_rate, config_.vis_rate, config_.exploration_threshold);
}

void SemanticTargetExtractor::loadConfig(const ros::NodeHandle& nh, const std::string& ns) {
    std::string prefix = ns.empty() ? "" : ns + "/";
    
    nh.param(prefix + "exploration_threshold", config_.exploration_threshold, 0.85f);
    nh.param(prefix + "pca_check_interval", config_.pca_check_interval, 20);
    nh.param(prefix + "planarity_threshold", config_.planarity_threshold, 0.85f);
    nh.param(prefix + "wall_size_threshold", config_.wall_size_threshold, 1.0f);
    
    // 混合 PCA 参数
    nh.param(prefix + "local_pca_radius", config_.local_pca_radius, 0.3f);
    nh.param(prefix + "local_planarity_threshold", config_.local_planarity_threshold, 0.7f);
    nh.param(prefix + "hybrid_switch_threshold", config_.hybrid_switch_threshold, 0.6f);
    
    nh.param(prefix + "min_voxel_count", config_.min_voxel_count, 20);
    nh.param(prefix + "max_target_height", config_.max_target_height, 2.5f);
    nh.param(prefix + "min_target_height", config_.min_target_height, 0.3f);
    nh.param(prefix + "max_target_width", config_.max_target_width, 1.5f);
    
    // 相机观测排除参数
    nh.param(prefix + "observed_reject_ratio", config_.observed_reject_ratio, 0.15f);
    
    nh.param(prefix + "enable_self_cleaning", config_.enable_self_cleaning, true);
    nh.param(prefix + "process_rate", config_.process_rate, 2.0f);
    nh.param(prefix + "vis_rate", config_.vis_rate, 1.0f);
    nh.param<std::string>("frame_id", frame_id_, "map");
    
    ROS_INFO("[SemanticTargetExtractor] Config: exploration_thresh=%.2f, planarity_thresh=%.2f, hybrid_thresh=%.2f",
             config_.exploration_threshold, config_.planarity_threshold, config_.hybrid_switch_threshold);
}

// ==================== 定时器回调 ====================
void SemanticTargetExtractor::processTimerCallback(const ros::TimerEvent& event) {
    update();
}

void SemanticTargetExtractor::visualizationTimerCallback(const ros::TimerEvent& event) {
    publishTargetMarkers();
}

// ==================== 主更新函数 ====================
void SemanticTargetExtractor::update() {
    if (!global_map_) {
        return;
    }
    
    // 从 GlobalMap 获取待分析的子网格索引（已达到探索率阈值）
    const auto& ready_subgrids = global_map_->getReadyForAnalysisSubGrids();
    
    if (ready_subgrids.empty()) {
        return;  // 没有需要分析的子网格
    }
    
    // 复制一份索引，因为分析过程中会修改原集合
    std::vector<int> to_analyze(ready_subgrids.begin(), ready_subgrids.end());
    
    // 不再重置访问标记！
    // 让标记持久化，确保跨子网格边界的目标不会在不同批次被分成两个簇
    // resetVisitTags(); // 删除
    double start_time = ros::Time::now().toSec();
    for (int linear_idx : to_analyze) {
        star_mvf::SubGrid& sg = global_map_->getSubGrid(linear_idx);
        
        // 一次性分析
        analyzeSubGrid(sg);
        
        // 标记为已分析（由 GlobalMap 统一管理状态）
        global_map_->markSubGridAnalyzed(linear_idx);
        analyzed_count_++;
        
        // ROS_INFO("[SemanticTargetExtractor] SubGrid %d analyzed, found %zu targets",
        //          linear_idx, sg.cached_targets_.size());
    }
    double end_time = ros::Time::now().toSec();
    ROS_INFO("[SemanticTargetExtractor] Analyzed %zu subgrids in %.2f ms", to_analyze.size(), (end_time - start_time) * 1000);
}

// ==================== 可视化 ====================
void SemanticTargetExtractor::publishTargetMarkers() {
    if (target_marker_pub_.getNumSubscribers() == 0) {
        return;
    }
    
    std::vector<TargetMetadata> all_targets;
    getAllTargets(all_targets);
    
    visualization_msgs::MarkerArray marker_array;
    
    // 先清除旧的 markers
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = frame_id_;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "semantic_targets";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    int marker_id = 0;
    for (size_t i = 0; i < all_targets.size(); ++i) {
        const auto& target = all_targets[i];
        
        // 1. BBox 立方体
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "semantic_targets";
        bbox_marker.id = marker_id++;
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        
        Eigen::Vector3f center = (target.bbox_min + target.bbox_max) * 0.5f;
        Eigen::Vector3f size = target.bbox_max - target.bbox_min;
        
        bbox_marker.pose.position.x = center.x();
        bbox_marker.pose.position.y = center.y();
        bbox_marker.pose.position.z = center.z();
        bbox_marker.pose.orientation.w = 1.0;
        
        bbox_marker.scale.x = std::max(size.x(), 0.1f);
        bbox_marker.scale.y = std::max(size.y(), 0.1f);
        bbox_marker.scale.z = std::max(size.z(), 0.1f);
        
        bbox_marker.color = getColorForTarget(static_cast<int>(i));
        bbox_marker.color.a = 0.6f;  // 加深透明度
        
        marker_array.markers.push_back(bbox_marker);
        
        // 2. 质心球体
        visualization_msgs::Marker centroid_marker;
        centroid_marker.header.frame_id = frame_id_;
        centroid_marker.header.stamp = ros::Time::now();
        centroid_marker.ns = "semantic_targets";
        centroid_marker.id = marker_id++;
        centroid_marker.type = visualization_msgs::Marker::SPHERE;
        centroid_marker.action = visualization_msgs::Marker::ADD;
        
        centroid_marker.pose.position.x = target.centroid.x();
        centroid_marker.pose.position.y = target.centroid.y();
        centroid_marker.pose.position.z = target.centroid.z();
        centroid_marker.pose.orientation.w = 1.0;
        
        centroid_marker.scale.x = 0.4;  // 加大球体
        centroid_marker.scale.y = 0.4;
        centroid_marker.scale.z = 0.4;
        
        centroid_marker.color = getColorForTarget(static_cast<int>(i));
        centroid_marker.color.a = 1.0f;  // 不透明
        
        marker_array.markers.push_back(centroid_marker);
        
        // 3. ID 文本
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id_;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "semantic_targets";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = target.centroid.x();
        text_marker.pose.position.y = target.centroid.y();
        text_marker.pose.position.z = target.bbox_max.z() + 0.3;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.3;  // 文字高度
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        text_marker.text = "T" + std::to_string(i);
        
        marker_array.markers.push_back(text_marker);
    }
    
    target_marker_pub_.publish(marker_array);
}

std_msgs::ColorRGBA SemanticTargetExtractor::getColorForTarget(int target_id) const {
    // 使用 HSV 色轮生成不同颜色
    std_msgs::ColorRGBA color;
    
    float hue = fmod(target_id * 0.618033988749895f, 1.0f);  // 黄金分割
    float sat = 0.9f;
    float val = 0.9f;
    
    // HSV to RGB
    int h_i = static_cast<int>(hue * 6);
    float f = hue * 6 - h_i;
    float p = val * (1 - sat);
    float q = val * (1 - f * sat);
    float t = val * (1 - (1 - f) * sat);
    
    switch (h_i % 6) {
        case 0: color.r = val; color.g = t;   color.b = p;   break;
        case 1: color.r = q;   color.g = val; color.b = p;   break;
        case 2: color.r = p;   color.g = val; color.b = t;   break;
        case 3: color.r = p;   color.g = q;   color.b = val; break;
        case 4: color.r = t;   color.g = p;   color.b = val; break;
        case 5: color.r = val; color.g = p;   color.b = q;   break;
    }
    color.a = 1.0f;
    
    return color;
}

// ==================== 子网格一次性分析 ====================
void SemanticTargetExtractor::analyzeSubGrid(star_mvf::SubGrid& sg) {
    sg.cached_targets_.clear();
    
    int seed_count = 0;
    int skipped_assigned = 0;
    int wall_count = 0;
    int new_target_count = 0;
    int merged_count = 0;
    int invalid_count = 0;
    
    // 遍历该子网格内所有未观测占据体素
    for (const auto& seed : sg.occ_unobserved_list_) {
        seed_count++;
        
        // 已分配（属于某目标或已标记为墙）则跳过
        int existing_id = getVoxelTargetId(seed);
        if (existing_id != -2) {  // -2 表示未分配
            skipped_assigned++;
            continue;
        }
        
        // 动态生长，同时检测相邻的目标
        SemanticCluster cluster;
        std::set<int> adjacent_target_ids;
        bool is_target = growRegion(seed, cluster, adjacent_target_ids);
        
        // 标记所有体素的目标 ID
        int assigned_id = -2;  // 默认保持未分配，等待后续重新聚类
        
        if (cluster.is_wall && config_.enable_self_cleaning) {
            // 自清洁：将墙壁体素标记
            pruneWallVoxels(cluster);
            pruned_wall_count_++;
            wall_count++;
            assigned_id = -1;  // 墙
        } else if (is_target && cluster.is_valid) {
            if (!adjacent_target_ids.empty()) {
                // 至少有一个相邻目标
                int first_target_id = *adjacent_target_ids.begin();
                
                // 如果有多个相邻目标，先合并它们
                if (adjacent_target_ids.size() > 1) {
                    mergeTargets(adjacent_target_ids);
                }
                
                // 合并当前簇到第一个目标
                mergeToTarget(first_target_id, cluster);
                assigned_id = first_target_id;
                merged_count++;
            } else {
                // 创建新目标
                assigned_id = createNewTarget(cluster);
                new_target_count++;
                
                // 详细打印新目标
                // ROS_INFO("  [New Target %d] voxels=%zu, centroid=(%.2f,%.2f,%.2f), "
                //          "height=%.2f, width=%.2f",
                //          assigned_id,
                //          cluster.voxels.size(),
                //          cluster.centroid.x(), cluster.centroid.y(), cluster.centroid.z(),
                //          cluster.getHeight(), cluster.getWidth());
            }
        } else {
            invalid_count++;
            // 注意：小簇体素保持 assigned_id = -2（未分配），等待后续重新聚类
        }
        
        // 只有确定分配了目标 ID 才标记体素
        if (assigned_id != -2) {
            for (const auto& v : cluster.voxels) {
                setVoxelTargetId(v, assigned_id);
            }
        }
    }
    
    // ROS_INFO("  SubGrid stats: seeds=%d, skipped=%d, walls=%d, new=%d, merged=%d, invalid=%d",
    //          seed_count, skipped_assigned, wall_count, new_target_count, merged_count, invalid_count);
}

// ==================== 增量更新函数 ====================
void SemanticTargetExtractor::mergeToTarget(int target_id, const SemanticCluster& cluster) {
    if (target_id < 0 || target_id >= static_cast<int>(all_targets_.size())) {
        return;
    }
    
    TargetMetadata& target = all_targets_[target_id];
    
    // 扩展 bbox
    target.bbox_min = target.bbox_min.cwiseMin(cluster.bbox_min);
    target.bbox_max = target.bbox_max.cwiseMax(cluster.bbox_max);
    
    // 加权平均 centroid
    float old_score = target.score;
    float new_score = static_cast<float>(cluster.voxels.size());
    target.centroid = (target.centroid * old_score + cluster.centroid * new_score) 
                       / (old_score + new_score);
    target.score = old_score + new_score;
    target.timestamp = ros::Time::now().toSec();
    
    // ROS_INFO("  [Merged to Target %d] added %zu voxels, total_score=%.0f",
    //          target_id, cluster.voxels.size(), target.score);
}

int SemanticTargetExtractor::createNewTarget(const SemanticCluster& cluster) {
    TargetMetadata meta;
    meta.id = next_target_id_++;
    meta.centroid = cluster.centroid;
    meta.bbox_min = cluster.bbox_min;
    meta.bbox_max = cluster.bbox_max;
    meta.timestamp = ros::Time::now().toSec();
    meta.score = static_cast<float>(cluster.voxels.size());
    
    all_targets_.push_back(meta);
    return meta.id;
}

void SemanticTargetExtractor::mergeTargets(const std::set<int>& target_ids) {
    if (target_ids.size() < 2) return;
    
    // 第一个目标作为主目标
    int primary_id = *target_ids.begin();
    
    // 找到主目标在 all_targets_ 中的索引
    int primary_idx = -1;
    for (size_t i = 0; i < all_targets_.size(); ++i) {
        if (all_targets_[i].id == primary_id) {
            primary_idx = static_cast<int>(i);
            break;
        }
    }
    if (primary_idx < 0) return;
    
    TargetMetadata& primary = all_targets_[primary_idx];
    
    // 合并其他目标到主目标
    for (int tid : target_ids) {
        if (tid == primary_id) continue;
        
        // 找到要合并的目标
        for (size_t i = 0; i < all_targets_.size(); ++i) {
            if (all_targets_[i].id == tid) {
                TargetMetadata& other = all_targets_[i];
                
                // 扩展 bbox
                primary.bbox_min = primary.bbox_min.cwiseMin(other.bbox_min);
                primary.bbox_max = primary.bbox_max.cwiseMax(other.bbox_max);
                
                // 加权平均 centroid
                float w_p = primary.score;
                float w_o = other.score;
                primary.centroid = (primary.centroid * w_p + other.centroid * w_o) / (w_p + w_o);
                primary.score = w_p + w_o;
                
                // 标记被合并的目标为无效
                other.id = -1;  // 标记为已删除
                
                // ROS_INFO("  [Merged Target %d -> %d] combined score=%.0f", tid, primary_id, primary.score);
                break;
            }
        }
    }
    
    primary.timestamp = ros::Time::now().toSec();
}

// ==================== 目标获取 ====================
void SemanticTargetExtractor::getAllTargets(std::vector<TargetMetadata>& out_targets) const {
    out_targets.clear();
    // 过滤掉已删除的目标（id == -1）
    for (const auto& target : all_targets_) {
        if (target.id >= 0) {
            out_targets.push_back(target);
        }
    }
}

void SemanticTargetExtractor::getTargetsInRange(const Eigen::Vector3f& center, float radius,
                                                 std::vector<TargetMetadata>& out_targets) const {
    out_targets.clear();
    if (!global_map_) return;
    
    float radius_sq = radius * radius;
    
    int sg_num_x = global_map_->getSubGridNumX();
    int sg_num_y = global_map_->getSubGridNumY();
    
    for (int y = 0; y < sg_num_y; ++y) {
        for (int x = 0; x < sg_num_x; ++x) {
            star_mvf::SubGridIndex sg_idx(x, y);
            const star_mvf::SubGrid& sg = global_map_->getSubGrid(sg_idx);
            
            for (const auto& meta : sg.cached_targets_) {
                float dist_sq = (meta.centroid - center).squaredNorm();
                if (dist_sq <= radius_sq) {
                    out_targets.push_back(meta);
                }
            }
        }
    }
}

int SemanticTargetExtractor::getTotalTargetCount() const {
    int count = 0;
    if (!global_map_) return count;
    
    int sg_num_x = global_map_->getSubGridNumX();
    int sg_num_y = global_map_->getSubGridNumY();
    
    for (int y = 0; y < sg_num_y; ++y) {
        for (int x = 0; x < sg_num_x; ++x) {
            star_mvf::SubGridIndex sg_idx(x, y);
            const star_mvf::SubGrid& sg = global_map_->getSubGrid(sg_idx);
            count += static_cast<int>(sg.cached_targets_.size());
        }
    }
    return count;
}

// ==================== Cluster 重生成 ====================
bool SemanticTargetExtractor::regenerateCluster(const TargetMetadata& meta, 
                                                 SemanticCluster& out_cluster) {
    if (!global_map_) return false;
    
    std::set<int> dummy_adjacent_ids;
    return growRegion(meta.seed_voxel, out_cluster, dummy_adjacent_ids);
}

// ==================== 核心算法：动态区域生长 ====================
bool SemanticTargetExtractor::growRegion(const star_mvf::VoxelIndex& seed, 
                                          SemanticCluster& cluster,
                                          std::set<int>& adjacent_target_ids) {
    cluster = SemanticCluster();
    adjacent_target_ids.clear();  // 收集所有相邻的目标 ID
    
    if (!isOccupiedUnobserved(seed)) {
        return false;
    }
    
    std::queue<star_mvf::VoxelIndex> q;
    q.push(seed);
    
    // 使用临时集合记录本次生长访问的体素
    std::unordered_set<star_mvf::VoxelIndex, star_mvf::VoxelIndexHash> local_visited;
    local_visited.insert(seed);
    
    Eigen::Vector3f sum_pos = Eigen::Vector3f::Zero();
    Eigen::Matrix3f sum_sq = Eigen::Matrix3f::Zero();
    int count = 0;
    int adjacent_observed_count = 0;  // 相邻的 occ_observed 体素计数
    
    std::vector<star_mvf::VoxelIndex> neighbors;
    neighbors.reserve(6);
    
    while (!q.empty()) {
        star_mvf::VoxelIndex curr = q.front();
        q.pop();
        
        Eigen::Vector3f pos = global_map_->indexToPos(curr);
        cluster.voxels.push_back(curr);
        
        cluster.bbox_min = cluster.bbox_min.cwiseMin(pos);
        cluster.bbox_max = cluster.bbox_max.cwiseMax(pos);
        
        updatePCAStats(pos, sum_pos, sum_sq, count);
        
        // Early Rejection 检查
        if (count > 0 && count % config_.pca_check_interval == 0) {
            computePCAFeatures(sum_pos, sum_sq, count, cluster.pca_features);
            
            if (isWallOrFloor(cluster.pca_features, cluster)) {
                cluster.is_wall = true;
                cluster.is_valid = false;
                
                // 将队列中剩余的体素也加入 cluster，确保它们都被标记为墙
                while (!q.empty()) {
                    cluster.voxels.push_back(q.front());
                    q.pop();
                }
                return false;
            }
        }
        
        getNeighbors6(curr, neighbors);
        
        for (const auto& nei : neighbors) {
            // 检查邻居是否已分配给某个目标
            int nei_target_id = getVoxelTargetId(nei);
            if (nei_target_id >= 0) {
                // 发现邻居属于某目标，添加到相邻目标集合
                adjacent_target_ids.insert(nei_target_id);
            }
            
            // 检查邻居是否是相机已观测的占据体素
            star_mvf::VoxelState nei_state = global_map_->getVoxelState(nei);
            bool is_occ = star_mvf::VoxelStateBits::isOccupied(nei_state);
            bool is_vis = star_mvf::VoxelStateBits::isVisited(nei_state);
            if (is_occ && is_vis) {
                adjacent_observed_count++;
            }
            
            // 未分配的占据未观测体素加入生长
            if (local_visited.find(nei) == local_visited.end() && 
                nei_target_id == -2 &&  // -2 表示未分配
                isOccupiedUnobserved(nei)) {
                local_visited.insert(nei);
                q.push(nei);
            }
        }
    }
    
    if (count >= config_.min_voxel_count) {
        cluster.centroid = sum_pos / static_cast<float>(count);
        computePCAFeatures(sum_pos, sum_sq, count, cluster.pca_features);
        
        // 检查是否被相机部分观测（排除树/柱子等）
        float ratio = static_cast<float>(adjacent_observed_count) / static_cast<float>(count);
        if (ratio > config_.observed_reject_ratio) {
            cluster.is_valid = false;  // 排除：相机已部分观测
            return false;
        }
        
        cluster.is_valid = isValidTarget(cluster);
    }
    
    return cluster.is_valid;
}

// ==================== PCA 计算 ====================
void SemanticTargetExtractor::updatePCAStats(const Eigen::Vector3f& pos,
                                              Eigen::Vector3f& sum_pos,
                                              Eigen::Matrix3f& sum_sq,
                                              int& count) {
    sum_pos += pos;
    sum_sq += pos * pos.transpose();
    count++;
}

void SemanticTargetExtractor::computePCAFeatures(const Eigen::Vector3f& sum_pos,
                                                  const Eigen::Matrix3f& sum_sq,
                                                  int count,
                                                  ClusterPCAFeatures& features) {
    if (count < 3) return;
    
    float n = static_cast<float>(count);
    Eigen::Vector3f mean = sum_pos / n;
    
    Eigen::Matrix3f cov = sum_sq / n - mean * mean.transpose();
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        ROS_WARN_THROTTLE(1.0, "[SemanticTargetExtractor] Eigen decomposition failed!");
        return;
    }
    
    Eigen::Vector3f evals = solver.eigenvalues().reverse();
    Eigen::Matrix3f evecs = solver.eigenvectors().rowwise().reverse();
    
    features.eigenvalues = evals;
    features.eigenvectors = evecs;
    
    float sum_evals = evals.sum() + 1e-6f;
    float lambda1 = evals(0) + 1e-6f;
    
    features.planarity = 2.0f * (evals(1) - evals(2)) / sum_evals;
    features.sphericity = evals(2) / lambda1;
    features.linearity = (evals(0) - evals(1)) / lambda1;
    features.normal = evecs.col(2);
}

// ==================== 判定函数 ====================
bool SemanticTargetExtractor::isWallOrFloor(const ClusterPCAFeatures& features, 
                                             const SemanticCluster& cluster) {
    float max_extent = cluster.getMaxExtent();
    
    if (features.planarity > config_.planarity_threshold && 
        max_extent > config_.wall_size_threshold) {
        return true;
    }
    return false;
}

bool SemanticTargetExtractor::isValidTarget(const SemanticCluster& cluster) {
    if (static_cast<int>(cluster.voxels.size()) < config_.min_voxel_count) {
        return false;
    }
    
    float height = cluster.getHeight();
    if (height < config_.min_target_height || height > config_.max_target_height) {
        return false;
    }
    
    float width = cluster.getWidth();
    if (width > config_.max_target_width) {
        if (cluster.pca_features.planarity > 0.5f) {
            return false;
        }
    }
    
    return true;
}

// ==================== 自清洁 ====================
void SemanticTargetExtractor::pruneWallVoxels(const SemanticCluster& cluster) {
    if (!global_map_) return;
    
    for (const auto& idx : cluster.voxels) {
        global_map_->updateVoxelStateFromCamera(idx, true);
    }
}

// ==================== 体素-目标映射 ====================
int SemanticTargetExtractor::getVoxelTargetId(const star_mvf::VoxelIndex& idx) const {
    auto it = voxel_target_ids_.find(idx);
    if (it != voxel_target_ids_.end()) {
        return it->second;
    }
    return -2;  // 未分配
}

void SemanticTargetExtractor::setVoxelTargetId(const star_mvf::VoxelIndex& idx, int target_id) {
    voxel_target_ids_[idx] = target_id;
}

// ==================== 辅助函数 ====================
// 6 邻域生长
void SemanticTargetExtractor::getNeighbors6(const star_mvf::VoxelIndex& idx, 
                                             std::vector<star_mvf::VoxelIndex>& neighbors) const {
    neighbors.clear();
    neighbors.reserve(6);
    
    // 6 邻域：上下前后左右
    neighbors.emplace_back(idx.x + 1, idx.y, idx.z);
    neighbors.emplace_back(idx.x - 1, idx.y, idx.z);
    neighbors.emplace_back(idx.x, idx.y + 1, idx.z);
    neighbors.emplace_back(idx.x, idx.y - 1, idx.z);
    neighbors.emplace_back(idx.x, idx.y, idx.z + 1);
    neighbors.emplace_back(idx.x, idx.y, idx.z - 1);
}

bool SemanticTargetExtractor::isOccupiedUnobserved(const star_mvf::VoxelIndex& idx) const {
    if (!global_map_) return false;
    
    star_mvf::VoxelState state = global_map_->getVoxelState(idx);
    
    bool is_occupied = star_mvf::VoxelStateBits::isOccupied(state);
    bool is_visited = star_mvf::VoxelStateBits::isVisited(state);
    
    return is_occupied && !is_visited;
}

// ==================== 混合 PCA 分析 ====================
bool SemanticTargetExtractor::hybridPCAAnalysis(const SemanticCluster& cluster, 
                                                 std::vector<star_mvf::VoxelIndex>& anomaly_voxels) {
    anomaly_voxels.clear();
    
    // 检查全局平面度
    float global_planarity = cluster.pca_features.planarity;
    
    // 高平面度 → 纯墙壁，不做局部分析
    if (global_planarity > config_.planarity_threshold) {
        return false;
    }
    
    // 低平面度 → 大部分点都是异常，不需要逐点分析
    if (global_planarity < config_.hybrid_switch_threshold) {
        anomaly_voxels = cluster.voxels;
        return static_cast<int>(anomaly_voxels.size()) >= config_.min_voxel_count;
    }
    
    // 中等平面度 → 混合分析，逐点计算局部 PCA
    for (const auto& voxel : cluster.voxels) {
        ClusterPCAFeatures local_features;
        
        if (computeLocalPCA(voxel, config_.local_pca_radius, local_features)) {
            // 局部平面度低于阈值 → 该点是异常点（非平面）
            if (local_features.planarity < config_.local_planarity_threshold) {
                anomaly_voxels.push_back(voxel);
            }
        } else {
            // 邻域点太少，保守起见认为是异常
            anomaly_voxels.push_back(voxel);
        }
    }
    
    return static_cast<int>(anomaly_voxels.size()) >= config_.min_voxel_count;
}

bool SemanticTargetExtractor::computeLocalPCA(const star_mvf::VoxelIndex& center, 
                                               float radius, 
                                               ClusterPCAFeatures& out_features) {
    if (!global_map_) return false;
    
    float resolution = global_map_->getResolution();
    int voxel_radius = static_cast<int>(std::ceil(radius / resolution));
    
    Eigen::Vector3f sum_pos = Eigen::Vector3f::Zero();
    Eigen::Matrix3f sum_sq = Eigen::Matrix3f::Zero();
    int count = 0;
    
    // 遍历立方体邻域
    for (int dz = -voxel_radius; dz <= voxel_radius; ++dz) {
        for (int dy = -voxel_radius; dy <= voxel_radius; ++dy) {
            for (int dx = -voxel_radius; dx <= voxel_radius; ++dx) {
                star_mvf::VoxelIndex nei(center.x + dx, center.y + dy, center.z + dz);
                
                // 检查是否是占据体素
                star_mvf::VoxelState state = global_map_->getVoxelState(nei);
                if (star_mvf::VoxelStateBits::isOccupied(state)) {
                    Eigen::Vector3f pos = global_map_->indexToPos(nei);
                    sum_pos += pos;
                    sum_sq += pos * pos.transpose();
                    count++;
                }
            }
        }
    }
    
    // 至少需要 5 个点才能计算有意义的 PCA
    if (count < 5) {
        return false;
    }
    
    computePCAFeatures(sum_pos, sum_sq, count, out_features);
    return true;
}

}  // namespace rog_map

