// frontier_viewpoint_generator.cpp
// 前端视点生成器实现

#include "rog_map/frontier_viewpoint_generator.h"
#include <numeric>

namespace rog_map {

// ==================== 构造函数 ====================

FrontierViewpointGenerator::FrontierViewpointGenerator(
    GlobalMap::Ptr global_map,
    const ViewpointGeneratorConfig& config)
    : global_map_(global_map)
    , config_(config)
{
    regions_.reserve(20);
    route_.reserve(20);
}

// ==================== 核心接口 ====================

void FrontierViewpointGenerator::updateUAVState(const UAVState& state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    uav_state_ = state;
}

bool FrontierViewpointGenerator::generateExplorationPlan(
    const std::vector<int>& subgrid_ids,
    std::vector<ExplorationRegion>& regions,
    std::vector<int>& route)
{
    if (subgrid_ids.empty()) {
        return false;
    }
    
    // 清空缓存
    regions_.clear();
    route_.clear();
    
    // Step 1: 将子区域划分为探索区域块
    partitionIntoRegions(subgrid_ids, regions_);
    
    if (regions_.empty()) {
        return false;
    }
    
    // Step 2-4: 对每个区域进行处理
    for (auto& region : regions_) {
        // Step 2: 聚类前视体素
        clusterFrontiersInRegion(region);
        
        // Step 3: 分裂大簇
        if (!region.clusters.empty()) {
            splitLargeClusters(region.clusters);
        }
        
        // Step 4: 生成视点
        if (!region.clusters.empty()) {
            generateViewpointsForRegion(region);
        }
    }
    
    // 过滤没有有效视点的区域
    regions_.erase(
        std::remove_if(regions_.begin(), regions_.end(),
            [](const ExplorationRegion& r) { return r.viewpoints.empty(); }),
        regions_.end());
    
    if (regions_.empty()) {
        return false;
    }
    
    // Step 5: 规划区域访问顺序
    planRegionRoute(regions_, route_);
    
    // 输出结果
    regions = regions_;
    route = route_;
    
    return !route_.empty();
}

bool FrontierViewpointGenerator::getFirstRegionViewpoints(
    std::vector<ExplorationViewpoint>& viewpoints)
{
    viewpoints.clear();
    
    if (route_.empty() || regions_.empty()) {
        return false;
    }
    
    int first_region_id = route_[0];
    
    // 查找第一个区域
    for (const auto& region : regions_) {
        if (region.region_id == first_region_id) {
            viewpoints = region.viewpoints;
            return !viewpoints.empty();
        }
    }
    
    return false;
}

// ==================== Step 1: 区域划分 ====================

void FrontierViewpointGenerator::partitionIntoRegions(
    const std::vector<int>& subgrid_ids,
    std::vector<ExplorationRegion>& regions)
{
    regions.clear();
    
    if (subgrid_ids.empty()) return;
    
    // 简单策略：每个子网格作为一个独立的探索区域
    // 后续可以改进为基于连通性的合并策略
    
    int region_id = 0;
    for (int sg_id : subgrid_ids) {
        SubGrid& sg = global_map_->getSubGrid(sg_id);
        
        // 检查该子网格是否有前视体素
        if (sg.sub_grid_frontier_voxels_.empty()) {
            continue;
        }
        
        // 检查探索率
        float exp_rate = sg.getExplorationRate();
        if (exp_rate <= 0.0f || exp_rate >= 1.0f) {
            continue;
        }
        
        ExplorationRegion region;
        region.region_id = region_id++;
        region.subgrid_ids.push_back(sg_id);
        region.centroid = sg.center;
        region.total_frontier_area = static_cast<float>(sg.sub_grid_frontier_voxels_.size());
        
        regions.push_back(region);
    }
    
    // 计算探索优先级（基于前视面积和距离）
    UAVState current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = uav_state_;
    }
    
    for (auto& region : regions) {
        float dist = distance(current_state.position, region.centroid);
        // 优先级 = 前视面积 / (距离 + 1)，避免除零
        region.exploration_priority = region.total_frontier_area / (dist + 1.0f);
    }
}

// ==================== Step 2: 前视簇聚类 ====================

void FrontierViewpointGenerator::clusterFrontiersInRegion(ExplorationRegion& region)
{
    region.clusters.clear();
    
    // 收集区域内所有子网格的前视体素
    std::unordered_set<GlobalVoxelIndex, GlobalVoxelIndexHash> all_frontiers;
    
    for (int sg_id : region.subgrid_ids) {
        SubGrid& sg = global_map_->getSubGrid(sg_id);
        all_frontiers.insert(
            sg.sub_grid_frontier_voxels_.begin(),
            sg.sub_grid_frontier_voxels_.end());
    }
    
    if (all_frontiers.empty()) return;
    
    // 复制用于"破坏性读取"
    std::unordered_set<GlobalVoxelIndex, GlobalVoxelIndexHash> remaining = all_frontiers;
    std::vector<FrontierCluster> residual_clusters;
    std::vector<GlobalVoxelIndex> voxels_to_mark_occupied;
    
    int cluster_id = 0;
    
    // BFS聚类
    while (!remaining.empty() && 
           cluster_id < config_.max_clusters_per_region) 
    {
        // 选择种子点
        auto seed_it = remaining.begin();
        GlobalVoxelIndex seed_idx = *seed_it;
        remaining.erase(seed_it);
        
        FrontierCluster cluster;
        cluster.cluster_id = cluster_id;
        cluster.parent_region_id = region.region_id;
        cluster.voxel_indices.push_back(seed_idx);
        
        // BFS扩展
        std::queue<GlobalVoxelIndex> bfs_queue;
        bfs_queue.push(seed_idx);
        
        int neighbor_free_count = 0;
        int neighbor_occupied_count = 0;
        
        while (!bfs_queue.empty()) {
            GlobalVoxelIndex current = bfs_queue.front();
            bfs_queue.pop();
            
            // 遍历6邻域
            for (const auto& offset : neighbor_offsets_6) {
                GlobalVoxelIndex neighbor = current + offset;
                
                // 聚类逻辑
                auto it = remaining.find(neighbor);
                if (it != remaining.end()) {
                    cluster.voxel_indices.push_back(neighbor);
                    bfs_queue.push(neighbor);
                    remaining.erase(it);
                }
                
                // 环境统计
                if (cluster.size() < config_.min_cluster_size * 2) {
                    auto state = global_map_->getVoxelState(neighbor);
                    if (VoxelStateBits::isOccupied(state)) {
                        neighbor_occupied_count++;
                    } else if (VoxelStateBits::isFree(state)) {
                        neighbor_free_count++;
                    }
                }
            }
        }
        
        // 分级处理
        if (cluster.size() >= config_.min_cluster_size) {
            // 正常簇：计算PCA
            computeClusterPCA(cluster);
            cluster.needs_split = checkClusterNeedsSplit(cluster);
            region.clusters.push_back(cluster);
            cluster_id++;
        } else {
            // 小簇处理
            int total_neighbors = neighbor_free_count + neighbor_occupied_count;
            float obstacle_ratio = 0.0f;
            if (total_neighbors > 0) {
                obstacle_ratio = static_cast<float>(neighbor_occupied_count) / total_neighbors;
            }
            
            bool is_enclosed = (obstacle_ratio > config_.obstacle_ratio_threshold) || 
                               (neighbor_free_count == 0);
            
            if (is_enclosed) {
                // 死角：标记为占据
                voxels_to_mark_occupied.insert(
                    voxels_to_mark_occupied.end(),
                    cluster.voxel_indices.begin(),
                    cluster.voxel_indices.end());
            } else if (cluster.size() > config_.absolute_noise_threshold) {
                // 残留簇
                residual_clusters.push_back(cluster);
            }
        }
    }
    
    // 清理死角
    for (const auto& idx : voxels_to_mark_occupied) 
    {
        global_map_->updateVoxelStateFromLiDAR(idx, true);
    }
    
    // 兜底机制
    if (region.clusters.empty() && !residual_clusters.empty()) {
        std::sort(residual_clusters.begin(), residual_clusters.end(),
            [](const FrontierCluster& a, const FrontierCluster& b) {
                return a.size() > b.size();
            });
        
        int fallback_count = std::min(static_cast<int>(residual_clusters.size()), 2);
        for (int i = 0; i < fallback_count; ++i) {
            FrontierCluster& rescue = residual_clusters[i];
            rescue.cluster_id = static_cast<int>(region.clusters.size());
            rescue.parent_region_id = region.region_id;
            computeClusterPCA(rescue);
            region.clusters.push_back(rescue);
        }
    }
}

// ==================== Step 3: 大簇分裂 ====================

bool FrontierViewpointGenerator::splitLargeClusters(std::vector<FrontierCluster>& clusters)
{
    std::list<FrontierCluster> cluster_list(clusters.begin(), clusters.end());
    bool any_split = false;
    
    auto it = cluster_list.begin();
    while (it != cluster_list.end()) {
        if (it->needs_split) {
            std::list<FrontierCluster> splits;
            if (splitClusterHorizontally(*it, splits)) {
                // 替换当前簇为分裂后的簇
                it = cluster_list.erase(it);
                cluster_list.insert(it, splits.begin(), splits.end());
                any_split = true;
                // 不递增it，继续检查新插入的簇
                continue;
            }
        }
        ++it;
    }
    
    // 转换回vector并重新编号
    clusters.clear();
    clusters.reserve(cluster_list.size());
    int new_id = 0;
    for (auto& c : cluster_list) {
        c.cluster_id = new_id++;
        clusters.push_back(std::move(c));
    }
    
    return any_split;
}

bool FrontierViewpointGenerator::splitClusterHorizontally(
    const FrontierCluster& cluster,
    std::list<FrontierCluster>& splits)
{
    splits.clear();
    
    if (cluster.size() < config_.min_cluster_size * 2) {
        return false;
    }
    
    // 计算水平方向的中心
    Eigen::Vector2f mean_2d = cluster.centroid.head<2>();
    
    // 检查是否需要分裂
    bool need_split = false;
    for (const auto& idx : cluster.voxel_indices) {
        Eigen::Vector3f pos;
        global_map_->indexToPos(idx, pos);
        if ((pos.head<2>() - mean_2d).norm() > config_.cluster_split_threshold) {
            need_split = true;
            break;
        }
    }
    
    if (!need_split) {
        return false;
    }
    
    // 计算2D协方差矩阵
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& idx : cluster.voxel_indices) {
        Eigen::Vector3f pos;
        global_map_->indexToPos(idx, pos);
        Eigen::Vector2f diff = pos.head<2>() - mean_2d;
        cov += diff * diff.transpose();
    }
    cov /= static_cast<float>(cluster.size());
    
    // 特征值分解找主成分方向
    Eigen::EigenSolver<Eigen::Matrix2f> solver(cov);
    Eigen::Vector2f eigenvalues = solver.eigenvalues().real();
    Eigen::Matrix2f eigenvectors = solver.eigenvectors().real();
    
    // 找最大特征值对应的特征向量
    int max_idx = 0;
    if (eigenvalues(1) > eigenvalues(0)) {
        max_idx = 1;
    }
    Eigen::Vector2f first_pc = eigenvectors.col(max_idx);
    
    // 沿主成分方向分裂
    FrontierCluster cluster1, cluster2;
    cluster1.parent_region_id = cluster.parent_region_id;
    cluster2.parent_region_id = cluster.parent_region_id;
    
    for (const auto& idx : cluster.voxel_indices) {
        Eigen::Vector3f pos;
        global_map_->indexToPos(idx, pos);
        Eigen::Vector2f diff = pos.head<2>() - mean_2d;
        
        if (diff.dot(first_pc) >= 0) {
            cluster1.voxel_indices.push_back(idx);
        } else {
            cluster2.voxel_indices.push_back(idx);
        }
    }
    
    // 检查分裂后的簇是否有效
    if (cluster1.size() < config_.min_cluster_size || 
        cluster2.size() < config_.min_cluster_size) {
        return false;
    }
    
    // 计算分裂后簇的PCA
    computeClusterPCA(cluster1);
    computeClusterPCA(cluster2);
    
    // 检查是否需要继续分裂
    cluster1.needs_split = checkClusterNeedsSplit(cluster1);
    cluster2.needs_split = checkClusterNeedsSplit(cluster2);
    
    splits.push_back(cluster1);
    splits.push_back(cluster2);
    
    // 递归分裂
    if (cluster1.needs_split) {
        std::list<FrontierCluster> sub_splits;
        if (splitClusterHorizontally(cluster1, sub_splits)) {
            splits.pop_front();
            splits.insert(splits.begin(), sub_splits.begin(), sub_splits.end());
        }
    }
    
    if (cluster2.needs_split) {
        std::list<FrontierCluster> sub_splits;
        if (splitClusterHorizontally(cluster2, sub_splits)) {
            splits.pop_back();
            splits.insert(splits.end(), sub_splits.begin(), sub_splits.end());
        }
    }
    
    return !splits.empty();
}

bool FrontierViewpointGenerator::checkClusterNeedsSplit(const FrontierCluster& cluster)
{
    if (cluster.size() < config_.min_cluster_size * 2) {
        return false;
    }
    
    Eigen::Vector2f mean_2d = cluster.centroid.head<2>();
    
    for (const auto& idx : cluster.voxel_indices) {
        Eigen::Vector3f pos;
        global_map_->indexToPos(idx, pos);
        if ((pos.head<2>() - mean_2d).norm() > config_.cluster_split_threshold) {
            return true;
        }
    }
    
    return false;
}

void FrontierViewpointGenerator::computeClusterPCA(FrontierCluster& cluster)
{
    if (cluster.voxel_indices.empty()) {
        return;
    }
    
    int n = cluster.size();
    
    // 计算重心
    cluster.centroid = Eigen::Vector3f::Zero();
    for (const auto& idx : cluster.voxel_indices) {
        Eigen::Vector3f pos;
        global_map_->indexToPos(idx, pos);
        cluster.centroid += pos;
    }
    cluster.centroid /= static_cast<float>(n);
    
    if (n < 3) {
        cluster.normal = Eigen::Vector3f(0, 0, 1);
        cluster.principal_axis = Eigen::Vector3f(1, 0, 0);
        cluster.extent = 0.0f;
        return;
    }
    
    // 构建协方差矩阵
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& idx : cluster.voxel_indices) {
        Eigen::Vector3f pos;
        global_map_->indexToPos(idx, pos);
        Eigen::Vector3f diff = pos - cluster.centroid;
        covariance += diff * diff.transpose();
    }
    covariance /= static_cast<float>(n - 1);
    
    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    cluster.normal = solver.eigenvectors().col(0);
    cluster.principal_axis = solver.eigenvectors().col(2);
    cluster.extent = std::sqrt(solver.eigenvalues()(2));
    
    // 确保法向量指向自由空间
    Eigen::Vector3f test_pos = cluster.centroid + 
        cluster.normal * global_map_->global_resolution_ * 2.0f;
    GlobalVoxelState state = global_map_->getVoxelState(test_pos);
    if (VoxelStateBits::isUnknown(state)) {
        cluster.normal = -cluster.normal;
    }
}

// ==================== Step 4: 视点生成 ====================

void FrontierViewpointGenerator::generateViewpointsForRegion(ExplorationRegion& region)
{
    region.viewpoints.clear();
    
    if (region.clusters.empty()) {
        return;
    }
    
    // Step 4.1: 在区域内采样候选位置
    std::vector<Eigen::Vector3f> candidates;
    sampleFreeSpaceInRegion(region, candidates);
    
    if (candidates.empty()) {
        return;
    }
    
    // Step 4.2: 安全性滤波
    std::vector<Eigen::Vector3f> safe_candidates;
    safe_candidates.reserve(candidates.size());
    
    for (const auto& pos : candidates) {
        if (!global_map_->isInGlobalMapRange(pos)) continue;
        if (!isPositionSafe(pos)) continue;
        safe_candidates.push_back(pos);
    }
    
    if (safe_candidates.empty()) {
        return;
    }
    
    // Step 4.3: 贪心选择视点
    greedySelectViewpoints(safe_candidates, region.clusters, region.viewpoints);
    
    // 设置区域ID
    for (auto& vp : region.viewpoints) {
        vp.region_id = region.region_id;
    }
}

void FrontierViewpointGenerator::sampleFreeSpaceInRegion(
    const ExplorationRegion& region,
    std::vector<Eigen::Vector3f>& samples)
{
    samples.clear();
    
    int step = 0;
    
    for (int sg_id : region.subgrid_ids) {
        SubGrid& sg = global_map_->getSubGrid(sg_id);
        
        for (const auto& voxel_idx : sg.explored_voxels_) {
            GlobalVoxelState state = global_map_->getVoxelState(voxel_idx);
            if (!VoxelStateBits::isFree(state)) continue;
            
            step++;
            if (step > config_.sampling_step) {
                Eigen::Vector3f pos;
                global_map_->indexToPos(voxel_idx, pos);
                samples.push_back(pos);
                step = 0;
            }
        }
    }
    
    // 随机下采样
    if (samples.size() > static_cast<size_t>(config_.max_viewpoints_per_region * 5)) {
        std::random_shuffle(samples.begin(), samples.end());
        samples.resize(config_.max_viewpoints_per_region * 5);
    }
}

void FrontierViewpointGenerator::greedySelectViewpoints(
    const std::vector<Eigen::Vector3f>& candidates,
    const std::vector<FrontierCluster>& clusters,
    std::vector<ExplorationViewpoint>& selected)
{
    selected.clear();
    
    if (candidates.empty() || clusters.empty()) {
        return;
    }
    
    int num_clusters = static_cast<int>(clusters.size());
    
    // 构建候选视点列表（每个候选位置对每个簇的覆盖评分）
    struct ViewpointCandidate {
        Eigen::Vector3f position;
        float yaw, pitch;
        int target_cluster;
        float score;
        std::vector<int> covered_clusters;
        bool selected;
    };
    
    std::vector<ViewpointCandidate> all_candidates;
    
    for (size_t pos_idx = 0; pos_idx < candidates.size(); ++pos_idx) {
        const Eigen::Vector3f& pos = candidates[pos_idx];
        
        for (int c_idx = 0; c_idx < num_clusters; ++c_idx) {
            float yaw, pitch;
            float score = computeCoverageScore(pos, clusters[c_idx], yaw, pitch);
            
            if (score < config_.min_coverage_score) continue;
            
            ViewpointCandidate vc;
            vc.position = pos;
            vc.yaw = yaw;
            vc.pitch = pitch;
            vc.target_cluster = c_idx;
            vc.score = score;
            vc.selected = false;
            
            // 添加主目标簇
            vc.covered_clusters.push_back(c_idx);
            
            // 检查FOV内能否覆盖其他簇
            Eigen::Vector3f view_dir(
                std::cos(pitch) * std::cos(yaw),
                std::cos(pitch) * std::sin(yaw),
                -std::sin(pitch));
            
            float fov_cos = std::cos(std::max(config_.sensor_fov_h, config_.sensor_fov_v) / 2.0f);
            
            for (int other_idx = 0; other_idx < num_clusters; ++other_idx) {
                if (other_idx == c_idx) continue;
                
                Eigen::Vector3f to_cluster = clusters[other_idx].centroid - pos;
                float dist = to_cluster.norm();
                
                if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) {
                    continue;
                }
                
                Eigen::Vector3f d_v = to_cluster / dist;
                
                if (view_dir.dot(d_v) < fov_cos) {
                    continue;
                }
                
                // 可见性检查
                if (isLineVisible(pos, clusters[other_idx].centroid)) {
                    vc.covered_clusters.push_back(other_idx);
                }
            }
            
            all_candidates.push_back(vc);
        }
    }
    
    if (all_candidates.empty()) {
        return;
    }
    
    // 贪心选择
    std::unordered_set<int> uncovered;
    for (int i = 0; i < num_clusters; ++i) {
        uncovered.insert(i);
    }
    
    while (!uncovered.empty() && 
           static_cast<int>(selected.size()) < config_.max_viewpoints_per_region) 
    {
        int best_idx = -1;
        float best_gain = 0.0f;
        int best_coverage = 0;
        
        for (size_t i = 0; i < all_candidates.size(); ++i) {
            auto& vc = all_candidates[i];
            if (vc.selected) continue;
            
            // 计算当前增益
            int coverage = 0;
            for (int cluster_id : vc.covered_clusters) {
                if (uncovered.count(cluster_id) > 0) {
                    ++coverage;
                }
            }
            
            float gain = static_cast<float>(coverage) * vc.score;
            
            if (gain > best_gain || 
                (gain == best_gain && coverage > best_coverage)) {
                best_gain = gain;
                best_idx = static_cast<int>(i);
                best_coverage = coverage;
            }
        }
        
        if (best_idx < 0 || best_coverage == 0) {
            break;
        }
        
        // 选中
        auto& chosen = all_candidates[best_idx];
        chosen.selected = true;
        
        // 更新未覆盖集合
        for (int cluster_id : chosen.covered_clusters) {
            uncovered.erase(cluster_id);
        }
        
        // 检查与已选视点的距离
        bool too_close = false;
        for (const auto& existing : selected) {
            if (distance(chosen.position, existing.position) < config_.min_viewpoint_distance) {
                too_close = true;
                break;
            }
        }
        
        if (too_close) {
            continue;
        }
        
        // 创建视点
        ExplorationViewpoint vp;
        vp.position = chosen.position;
        vp.yaw = chosen.yaw;
        vp.pitch = chosen.pitch;
        vp.cluster_id = chosen.target_cluster;
        vp.total_score = chosen.score;
        vp.covered_cluster_ids = chosen.covered_clusters;
        vp.visible_frontier_count = countVisibleFrontierVoxels(
            vp.position, vp.yaw, vp.pitch, clusters[chosen.target_cluster]);
        vp.is_valid = true;
        
        selected.push_back(vp);
    }
}

float FrontierViewpointGenerator::computeCoverageScore(
    const Eigen::Vector3f& position,
    const FrontierCluster& cluster,
    float& best_yaw, float& best_pitch)
{
    Eigen::Vector3f to_cluster = cluster.centroid - position;
    float dist = to_cluster.norm();
    
    // 距离检查
    if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) {
        return 0.0f;
    }
    
    Eigen::Vector3f d_v = to_cluster / dist;
    
    // 计算视角
    float alignment_angle = std::acos(std::abs(cluster.normal.dot(d_v)));
    if (alignment_angle > config_.max_viewing_angle) {
        return 0.0f;
    }
    
    // 计算朝向
    best_yaw = std::atan2(d_v.y(), d_v.x());
    float horizontal_dist = std::sqrt(d_v.x() * d_v.x() + d_v.y() * d_v.y());
    best_pitch = std::atan2(-d_v.z(), horizontal_dist);
    best_pitch = std::max(-PI / 2.0f, std::min(PI / 2.0f, best_pitch));
    
    // 可见性检查
    if (!isLineVisible(position, cluster.centroid)) {
        return 0.0f;
    }
    
    // 计算评分
    // Gain_distance: 距离增益
    float dist_ratio = dist / config_.optimal_viewing_distance;
    float gain_distance = std::exp(-0.5f * (dist_ratio - 1.0f) * (dist_ratio - 1.0f));
    
    // Gain_angle: 角度增益
    float gain_angle = std::abs(cluster.normal.dot(d_v));
    
    // Gain_visibility: 可见性增益（简化）
    float gain_visibility = static_cast<float>(cluster.size()) / 100.0f;
    
    return gain_distance * gain_angle * (1.0f + gain_visibility);
}

int FrontierViewpointGenerator::countVisibleFrontierVoxels(
    const Eigen::Vector3f& pos, float yaw, float pitch,
    const FrontierCluster& cluster)
{
    int count = 0;
    
    Eigen::Vector3f view_dir(
        std::cos(pitch) * std::cos(yaw),
        std::cos(pitch) * std::sin(yaw),
        -std::sin(pitch));
    
    float half_fov = std::max(config_.sensor_fov_h, config_.sensor_fov_v) / 2.0f;
    float fov_cos = std::cos(half_fov);
    
    for (const auto& idx : cluster.voxel_indices) {
        Eigen::Vector3f voxel_pos;
        global_map_->indexToPos(idx, voxel_pos);
        
        Eigen::Vector3f to_voxel = voxel_pos - pos;
        float dist = to_voxel.norm();
        
        if (dist < config_.sensor_range_min || dist > config_.sensor_range_max) {
            continue;
        }
        
        to_voxel /= dist;
        
        if (view_dir.dot(to_voxel) < fov_cos) {
            continue;
        }
        
        // 简单可见性检查
        if (isLineVisible(pos, voxel_pos)) {
            ++count;
        }
    }
    
    return count;
}

// ==================== Step 5: 区域路由规划 ====================

void FrontierViewpointGenerator::planRegionRoute(
    std::vector<ExplorationRegion>& regions,
    std::vector<int>& route)
{
    route.clear();
    
    if (regions.empty()) {
        return;
    }
    
    UAVState current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = uav_state_;
    }
    
    // 计算每个区域的路由评分
    std::vector<RegionRouteNode> nodes(regions.size());
    
    for (size_t i = 0; i < regions.size(); ++i) {
        nodes[i].region_id = regions[i].region_id;
        nodes[i].distance_from_uav = distance(current_state.position, regions[i].centroid);
        nodes[i].exploration_gain = regions[i].total_frontier_area;
        
        // 路由评分 = 探索收益 / 距离
        if (nodes[i].distance_from_uav > 0.1f) {
            nodes[i].route_score = config_.exploration_weight * nodes[i].exploration_gain / 
                                   (config_.distance_weight * nodes[i].distance_from_uav);
        } else {
            nodes[i].route_score = nodes[i].exploration_gain * 100.0f;
        }
    }
    
    // 贪心选择：每次选择未访问区域中评分最高的
    // 并更新后续区域的距离（从当前区域出发）
    std::vector<bool> visited(regions.size(), false);
    Eigen::Vector3f current_pos = current_state.position;
    
    for (size_t round = 0; round < regions.size(); ++round) {
        int best_idx = -1;
        float best_score = -std::numeric_limits<float>::max();
        
        for (size_t i = 0; i < regions.size(); ++i) {
            if (visited[i]) continue;
            
            // 更新距离
            float dist = distance(current_pos, regions[i].centroid);
            float score = config_.exploration_weight * nodes[i].exploration_gain /
                         (config_.distance_weight * dist + 0.1f);
            
            if (score > best_score) {
                best_score = score;
                best_idx = static_cast<int>(i);
            }
        }
        
        if (best_idx < 0) {
            break;
        }
        
        visited[best_idx] = true;
        route.push_back(regions[best_idx].region_id);
        current_pos = regions[best_idx].centroid;
    }
}

// ==================== 辅助函数 ====================

bool FrontierViewpointGenerator::isPositionSafe(const Eigen::Vector3f& pos)
{
    GlobalVoxelIndex center_idx;
    global_map_->posToIndex(pos, center_idx);
    
    float total_radius = config_.uav_radius + config_.safety_margin;
    int radius_idx_cnt = std::ceil(total_radius / global_map_->global_resolution_);
    
    // 注意类型匹配，为了保险起见用 int64_t 防止溢出（虽然这里半径通常很小）
    int64_t radius_sq = static_cast<int64_t>(radius_idx_cnt) * radius_idx_cnt;
    
    // 1. 将 offset 定义改为 GlobalVoxelIndex
    GlobalVoxelIndex offset;

    // 2. 循环访问成员时，去掉括号 (Eigen是 .x()，结构体是 .x)
    for (offset.x = -radius_idx_cnt; offset.x <= radius_idx_cnt; offset.x++) {
        for (offset.y = -radius_idx_cnt; offset.y <= radius_idx_cnt; offset.y++) {
            for (offset.z = -radius_idx_cnt; offset.z <= radius_idx_cnt; offset.z++) {
                
                // 3. 手动计算平方范数 (替代 Eigen 的 squaredNorm())
                int64_t current_sq = offset.x * offset.x + 
                                     offset.y * offset.y + 
                                     offset.z * offset.z;

                if (current_sq > radius_sq) continue;
                
                // 4. 现在可以直接相加了，因为类型一致，调用你写的 operator+
                GlobalVoxelIndex check_idx = center_idx + offset;
                
                if (!global_map_->isInGlobalMapRange(check_idx)) continue;
                
                // 这里可能需要根据你的代码调整 getVoxelState 的返回值类型
                // 如果是 uint8_t 或者是 struct Voxel
                auto state = global_map_->getVoxelState(check_idx); 
                
                if (rog_map::VoxelStateBits::isOccupied(state)) {
                    return false;
                }
            }
        }
    }
    
    return true;
}

bool FrontierViewpointGenerator::isLineVisible(
    const Eigen::Vector3f& from, const Eigen::Vector3f& to)
{
    Eigen::Vector3f direction = to - from;
    float length = direction.norm();
    
    if (length < 1e-6f) {
        return true;
    }
    
    direction /= length;
    float step = global_map_->global_resolution_;
    
    for (float t = step; t < length - step; t += step) {
        Eigen::Vector3f check_pos = from + t * direction;
        if (global_map_->isOccupied(check_pos)) {
            return false;
        }
    }
    
    return true;
}

float FrontierViewpointGenerator::normalizeAngle(float angle)
{
    while (angle >= PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

float FrontierViewpointGenerator::distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
    return (a - b).norm();
}

} // namespace rog_map
