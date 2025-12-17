// frontier_viewpoint_generator.cpp
#include "frontier_viewpoint_generator.h"
#include <numeric>

namespace rog_map {

// ==================== 构造函数 ====================

FrontierViewpointGenerator::FrontierViewpointGenerator(
    rog_map::GlobalMap::Ptr global_map,
    const ViewpointGeneratorConfig& config)
    : global_map_(global_map)
    , config_(config)
{
    candidate_viewpoints_.reserve(1000);
    optimized_viewpoints_.reserve(500);
}

// ==================== 核心接口实现 ====================

void FrontierViewpointGenerator::updateUAVState(const UAVState& state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    uav_state_ = state;
}

void FrontierViewpointGenerator::generateCandidateViewpoints(
    const std::unordered_set<int>& subgrid_indices,
    std::vector<ExplorationViewpoint>& candidate_viewpoints)
{
    candidate_viewpoints.clear();
    candidate_viewpoints.reserve(subgrid_indices.size() * config_.max_viewpoints_per_subgrid);
    
    for (int subgrid_id : subgrid_indices) {
        std::vector<ExplorationViewpoint> subgrid_viewpoints;
        generateViewpointsFromSubGrid(subgrid_id, subgrid_viewpoints);
        
        candidate_viewpoints.insert(
            candidate_viewpoints.end(),
            subgrid_viewpoints.begin(),
            subgrid_viewpoints.end()
        );
    }
    
    // 更新内部缓存
    candidate_viewpoints_ = candidate_viewpoints;
}

void FrontierViewpointGenerator::generateCandidateViewpointsForRegion(
    int first_region_subgrid_id,
    std::vector<ExplorationViewpoint>& candidate_viewpoints)
{
    // 获取首个区域及其邻域子网格
    std::unordered_set<int> region_subgrids;
    region_subgrids.insert(first_region_subgrid_id);
    
    // 获取4-连通邻域
    rog_map::SubGrid& sg = global_map_->getSubGrid(first_region_subgrid_id);
    std::vector<rog_map::SubGridIndex> neighbors;
    global_map_->getNeighborSubGrids4(sg.grid_idx, neighbors);
    
    for (const auto& nei_idx : neighbors) {
        int nei_linear = global_map_->subGridToLinearIndex(nei_idx);
        region_subgrids.insert(nei_linear);
    }
    
    generateCandidateViewpoints(region_subgrids, candidate_viewpoints);
}

void FrontierViewpointGenerator::evaluateViewpoints(
    std::vector<ExplorationViewpoint>& candidate_viewpoints)
{
    UAVState current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = uav_state_;
    }
    
    for (auto& vp : candidate_viewpoints) {
        if (!vp.is_valid) continue;
        
        // 1. 计算曲率和虚拟速度
        vp.curvature = computeCurvatureToViewpoint(vp);
        vp.virtual_velocity = computeVirtualVelocity(vp.curvature);
        
        // 2. 计算运动成本
        vp.motion_cost = computeExplorationCost(vp);
        
        // 3. 计算探索信息增益
        computeExplorationGain(vp);
        
        // 4. 计算综合评分
        // 使用EDEN风格的指数评分（只考虑未知体素，不依赖frontier）
        float cost_term = std::exp(-config_.exploration_weight_lambda * vp.motion_cost);
        float gain_term = static_cast<float>(vp.visible_unknown_count);
        
        vp.total_score = cost_term * (1.0f + config_.gain_weight * gain_term);
    }
}

void FrontierViewpointGenerator::applyGravitationModel(
    std::vector<ExplorationViewpoint>& viewpoints)
{
    if (viewpoints.empty()) return;
    
    // 计算引力半径
    float gravitation_radius = computeGravitationRadius();
    
    // 按覆盖体素数降序排序
    std::vector<size_t> sorted_indices(viewpoints.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
        [&viewpoints](size_t a, size_t b) {
            return viewpoints[a].covered_voxels_count > viewpoints[b].covered_voxels_count;
        });
    
    // 迭代应用引力模型
    for (int iter = 0; iter < config_.max_gravitation_iterations; ++iter) {
        bool any_update = false;
        
        for (size_t idx : sorted_indices) {
            ExplorationViewpoint& vp_query = viewpoints[idx];
            if (!vp_query.is_active || !vp_query.is_valid) continue;
            
            // 查找邻域内的活跃视点
            std::vector<ExplorationViewpoint> neighbors;
            for (size_t other_idx = 0; other_idx < viewpoints.size(); ++other_idx) {
                if (other_idx == idx) continue;
                const ExplorationViewpoint& vp_other = viewpoints[other_idx];
                if (!vp_other.is_active || !vp_other.is_valid) continue;
                
                float dist = distance(vp_query.position, vp_other.position);
                if (dist < gravitation_radius) {
                    neighbors.push_back(vp_other);
                }
            }
            
            if (!neighbors.empty()) {
                // 应用引力更新
                Eigen::Vector3f old_pos = vp_query.position;
                applyGravitationUpdate(vp_query, neighbors);
                
                // 将邻域视点设为非活跃
                for (size_t other_idx = 0; other_idx < viewpoints.size(); ++other_idx) {
                    if (other_idx == idx) continue;
                    ExplorationViewpoint& vp_other = viewpoints[other_idx];
                    if (!vp_other.is_active) continue;
                    
                    float dist = distance(old_pos, vp_other.position);
                    if (dist < gravitation_radius) {
                        vp_other.is_active = false;
                        any_update = true;
                    }
                }
            }
        }
        
        if (!any_update) break;
    }
    
    // 合并过近的视点
    mergeCloseViewpoints(viewpoints);
    
    // 提取活跃视点到优化结果
    optimized_viewpoints_.clear();
    for (const auto& vp : viewpoints) {
        if (vp.is_active && vp.is_valid) {
            optimized_viewpoints_.push_back(vp);
        }
    }
}

bool FrontierViewpointGenerator::selectBestViewpointPair(
    const std::vector<ExplorationViewpoint>& candidate_viewpoints,
    ViewpointPair& best_pair)
{
    if (candidate_viewpoints.size() < 2) {
        return false;
    }
    
    float best_score = -std::numeric_limits<float>::max();
    bool found = false;
    
    // 遍历所有视点对
    for (size_t i = 0; i < candidate_viewpoints.size(); ++i) {
        const ExplorationViewpoint& vp_e = candidate_viewpoints[i];
        if (!vp_e.is_valid) continue;
        
        for (size_t j = 0; j < candidate_viewpoints.size(); ++j) {
            if (i == j) continue;
            const ExplorationViewpoint& vp_c = candidate_viewpoints[j];
            if (!vp_c.is_valid) continue;
            
            // 检查视点对可行性（EDEN约束）
            if (!isViewpointPairFeasible(vp_e, vp_c)) continue;
            
            // 计算视点对评分
            float pair_score = computeViewpointPairScore(vp_e, vp_c);
            
            if (pair_score > best_score) {
                best_score = pair_score;
                best_pair.vp_explore = vp_e;
                best_pair.vp_continuous = vp_c;
                best_pair.explore_cost = computeExplorationCost(vp_e);
                best_pair.continuous_cost = computeContinuousCost(vp_e, vp_c);
                best_pair.pair_score = pair_score;
                best_pair.is_feasible = true;
                found = true;
            }
        }
    }
    
    return found;
}

bool FrontierViewpointGenerator::selectBestViewpointTriplet(
    const std::vector<ExplorationViewpoint>& candidate_viewpoints,
    ViewpointTriplet& best_triplet)
{
    // 首先选择最优视点对
    ViewpointPair best_pair;
    if (!selectBestViewpointPair(candidate_viewpoints, best_pair)) {
        return false;
    }
    
    best_triplet.vp_e = best_pair.vp_explore;
    best_triplet.vp_c = best_pair.vp_continuous;
    
    // 寻找安全视点
    ExplorationViewpoint vp_safety;
    if (!findSafetyViewpoint(best_triplet.vp_e, candidate_viewpoints, vp_safety)) {
        // 如果找不到安全视点，使用探索视点作为安全视点
        vp_safety = best_triplet.vp_e;
    }
    best_triplet.vp_s = vp_safety;
    
    // 计算三视点组合评分
    best_triplet.triplet_score = best_pair.pair_score;
    
    return true;
}

bool FrontierViewpointGenerator::generateAndSelectViewpoints(
    const std::unordered_set<int>& frontier_subgrids,
    ViewpointTriplet& best_triplet)
{
    if (frontier_subgrids.empty()) {
        return false;
    }
    
    // Step 1: 生成候选视点
    std::vector<ExplorationViewpoint> candidates;
    generateCandidateViewpoints(frontier_subgrids, candidates);
    
    if (candidates.empty()) {
        return false;
    }
    
    // Step 2: 评估视点
    evaluateViewpoints(candidates);
    
    // Step 3: 应用引力模型优化（可选）
    // applyGravitationModel(candidates);
    
    // Step 4: 选择最优三视点组合
    return selectBestViewpointTriplet(candidates, best_triplet);
}

// ==================== 视点采样实现（基于EDEN的EROI自由空间采样）====================

/**
 * @brief 从单个子网格(EROI)的自由空间生成候选视点
 * 
 * EDEN核心策略：
 * 1. 不从frontier体素采样，而是在EROI的自由空间内直接采样
 * 2. 使用Dijkstra验证可达性
 * 3. 视点朝向指向未知区域重心
 */
void FrontierViewpointGenerator::generateViewpointsFromSubGrid(
    int subgrid_id,
    std::vector<ExplorationViewpoint>& viewpoints)
{
    viewpoints.clear();
    
    rog_map::SubGrid& sg = global_map_->getSubGrid(subgrid_id);
    
    // EDEN策略：检查子网格的探索率，只处理部分探索的区域
    float exp_rate = sg.getExplorationRate();
    if (exp_rate <= 0.0f || exp_rate >= 1.0f) {
        // 完全未探索或完全探索的区域不生成视点
        return;
    }
    
    viewpoints.reserve(config_.max_viewpoints_per_subgrid);
    
    // Step 1: 双策略采样 - 均匀采样 + 骨架引导采样
    std::vector<Eigen::Vector3f> uniform_samples;
    std::vector<Eigen::Vector3f> skeleton_samples;
    std::vector<Eigen::Vector3f> sample_positions;
    
    // 1.1 均匀采样
    sampleFreeSpaceInEROI(sg, uniform_samples);
    
    // 1.2 骨架引导采样（基于FC-Planner思想）
    if (config_.enable_skeleton_sampling) {
        sampleFromSkeleton(sg, skeleton_samples);
    }
    
    // 1.3 融合两种采样结果
    mergeSamplingResults(uniform_samples, skeleton_samples, sample_positions);
    
    if (sample_positions.empty()) {
        return;
    }
    
    int sampled_count = 0;
    
    // Step 2: 对每个采样位置验证并创建视点
    for (const auto& sample_pos : sample_positions) {
        if (sampled_count >= config_.max_viewpoints_per_subgrid) break;
        
        // 检查位置安全性
        if (!isPositionSafe(sample_pos)) {
            continue;
        }
        
        // EDEN核心：使用Dijkstra验证可达性
        float path_length = 0.0f;
        if (!verifyReachabilityDijkstra(sample_pos, path_length)) {
            continue;
        }
        
        // 检查该位置能否观测到未知区域
        float best_yaw = 0.0f, best_pitch = 0.0f;
        computeOptimalOrientation(sample_pos, sg, best_yaw, best_pitch);
        
        int visible_unknown = countVisibleUnknownVoxels(sample_pos, best_yaw, best_pitch);
        if (visible_unknown == 0) {
            // 该位置无法观测到未知区域，跳过
            continue;
        }
        
        // 检查与已有视点的最小距离
        bool too_close = false;
        for (const auto& existing_vp : viewpoints) {
            if (distance(sample_pos, existing_vp.position) < config_.min_viewpoint_distance) {
                too_close = true;
                break;
            }
        }
        if (too_close) continue;
        
        // 创建视点
        ExplorationViewpoint vp;
        vp.position = sample_pos;
        vp.yaw = best_yaw;
        vp.pitch = best_pitch;
        vp.subgrid_id = subgrid_id;
        vp.is_valid = true;
        vp.is_active = true;
        vp.visible_unknown_count = visible_unknown;
        
        viewpoints.push_back(vp);
        ++sampled_count;
    }
}

/**
 * @brief 在EROI的自由空间内均匀采样候选位置
 * 
 * EDEN策略：候选视点位于EROI内部或边界的自由空间
 */
void FrontierViewpointGenerator::sampleFreeSpaceInEROI(
    const rog_map::SubGrid& subgrid,
    std::vector<Eigen::Vector3f>& sample_positions)
{
    sample_positions.clear();
    
    // 计算采样步长
    float step = config_.sampling_resolution;
    
    // 在EROI范围内均匀采样自由空间
    for (float x = subgrid.world_min.x() + step/2; x <= subgrid.world_max.x(); x += step) {
        for (float y = subgrid.world_min.y() + step/2; y <= subgrid.world_max.y(); y += step) {
            for (float z = subgrid.world_min.z() + step/2; z <= subgrid.world_max.z(); z += step) {
                Eigen::Vector3f pos(x, y, z);
                
                // 检查是否在地图范围内
                if (!global_map_->isInGlobalMapRange(pos)) {
                    continue;
                }
                
                // EDEN策略：只在自由空间采样（不是frontier）
                rog_map::GlobalVoxelState state = global_map_->getVoxelState(pos);
                if (state == rog_map::GlobalVoxelState::FREE) {
                    sample_positions.push_back(pos);
                }
            }
        }
    }
    
    // 如果自由空间采样点过多，进行随机下采样
    if (sample_positions.size() > static_cast<size_t>(config_.max_viewpoints_per_subgrid * 3)) {
        std::random_shuffle(sample_positions.begin(), sample_positions.end());
        sample_positions.resize(config_.max_viewpoints_per_subgrid * 3);
    }
}

// ==================== 骨架引导采样实现（基于FC-Planner思想）====================

/**
 * @brief 计算未知体素到已知边界的距离场
 * 
 * 类比FC-Planner：从"目标"（未知区域）内部计算距离场
 * 使用BFS计算每个未知体素到最近已知区域（FREE/OCCUPIED）的距离
 */
void FrontierViewpointGenerator::computeDistanceField(
    const rog_map::SubGrid& subgrid,
    std::unordered_map<rog_map::GlobalVoxelIndex, float, 
                      rog_map::GlobalVoxelIndexHash>& distance_field)
{
    distance_field.clear();
    
    std::queue<rog_map::GlobalVoxelIndex> bfs_queue;
    std::unordered_set<rog_map::GlobalVoxelIndex, rog_map::GlobalVoxelIndexHash> visited;
    
    // 初始化：找到未知区域边界（与已知区域相邻的未知体素）
    for (const auto& unknown_idx : subgrid.unexplored_voxels_) {
        // 检查是否为边界（相邻已知区域：FREE或OCCUPIED）
        bool is_boundary_adjacent = false;
        for (int d = 0; d < 6; ++d) {
            rog_map::GlobalVoxelIndex nei_idx = unknown_idx + rog_map::neighbor_offsets_6[d];
            auto nei_state = global_map_->getVoxelState(nei_idx);
            // 相邻体素是已知的（FREE或OCCUPIED）
            if (!rog_map::VoxelStateBits::isUnknown(nei_state)) {
                is_boundary_adjacent = true;
                break;
            }
        }
        if (is_boundary_adjacent) {
            // 边界未知体素，距离为1个分辨率
            distance_field[unknown_idx] = global_map_->global_resolution_;
            bfs_queue.push(unknown_idx);
            visited.insert(unknown_idx);
        }
    }
    
    // BFS向未知区域内部扩展，计算到已知边界的距离
    float resolution = global_map_->global_resolution_;
    while (!bfs_queue.empty()) {
        rog_map::GlobalVoxelIndex current = bfs_queue.front();
        bfs_queue.pop();
        
        float current_dist = distance_field[current];
        
        for (int d = 0; d < 6; ++d) {
            rog_map::GlobalVoxelIndex nei_idx = current + rog_map::neighbor_offsets_6[d];
            
            if (visited.find(nei_idx) != visited.end()) continue;
            if (!subgrid.isInRange(nei_idx)) continue;
            
            // 只在未知区域内扩展
            auto nei_state = global_map_->getVoxelState(nei_idx);
            if (rog_map::VoxelStateBits::isUnknown(nei_state)) {
                float new_dist = current_dist + resolution;
                if (distance_field.find(nei_idx) == distance_field.end() ||
                    new_dist > distance_field[nei_idx]) {
                    distance_field[nei_idx] = new_dist;
                }
                visited.insert(nei_idx);
                bfs_queue.push(nei_idx);
            }
        }
    }
}

/**
 * @brief 从EROI的未知区域提取拓扑骨架
 * 
 * 类比FC-Planner：骨架在"目标"（未知区域）内部
 * 骨架点 = 未知区域中距离已知边界最远的点（即未知区域的"深处/中心"）
 */
void FrontierViewpointGenerator::extractFreeSpaceSkeleton(
    const rog_map::SubGrid& subgrid,
    FreeSpaceSkeleton& skeleton)
{
    skeleton.clear();
    
    // 计算未知区域的距离场（到已知边界的距离）
    std::unordered_map<rog_map::GlobalVoxelIndex, float, 
                      rog_map::GlobalVoxelIndexHash> distance_field;
    computeDistanceField(subgrid, distance_field);
    
    if (distance_field.empty()) {
        return;
    }
    
    // 提取局部距离最大点作为骨架点（未知区域的"深处"）
    std::unordered_map<rog_map::GlobalVoxelIndex, int, 
                      rog_map::GlobalVoxelIndexHash> idx_to_skeleton;
    
    for (const auto& [voxel_idx, dist] : distance_field) {
        // 只保留距离足够大的点（深入未知区域的点）
        if (dist < config_.skeleton_distance_threshold) continue;
        
        bool is_local_max = true;
        int higher_neighbor_count = 0;
        
        for (int d = 0; d < 6; ++d) {
            rog_map::GlobalVoxelIndex nei_idx = voxel_idx + rog_map::neighbor_offsets_6[d];
            auto it = distance_field.find(nei_idx);
            if (it != distance_field.end() && it->second > dist) {
                is_local_max = false;
                ++higher_neighbor_count;
            }
        }
        
        // 局部最大点 = 未知区域的"深处/中心"
        if (is_local_max || dist > config_.skeleton_distance_threshold * 2.0f) {
            SkeletonPoint sp;
            global_map_->indexToPos(voxel_idx, sp.position);
            sp.distance_to_boundary = dist;
            sp.is_junction = (higher_neighbor_count >= 2);
            
            idx_to_skeleton[voxel_idx] = skeleton.points.size();
            skeleton.points.push_back(sp);
        }
    }
    
    // 构建骨架边（连接相邻骨架点）
    for (const auto& [voxel_idx, skeleton_idx] : idx_to_skeleton) {
        for (int d = 0; d < 6; ++d) {
            rog_map::GlobalVoxelIndex nei_idx = voxel_idx + rog_map::neighbor_offsets_6[d];
            auto it = idx_to_skeleton.find(nei_idx);
            if (it != idx_to_skeleton.end() && it->second > skeleton_idx) {
                skeleton.edges.push_back({skeleton_idx, it->second});
                skeleton.points[skeleton_idx].neighbor_indices.push_back(it->second);
                skeleton.points[it->second].neighbor_indices.push_back(skeleton_idx);
            }
        }
    }
    
    skeleton.is_valid = (skeleton.points.size() >= config_.min_skeleton_points);
}

/**
 * @brief 从骨架点反向采样视点
 * 
 * 类比FC-Planner：骨架在目标（未知区域）内部，视点在外部（自由空间）
 * 从自由空间向骨架点方向采样视点，视点朝向骨架点（未知区域深处）
 */
void FrontierViewpointGenerator::generateSkeletonSamplingRays(
    const SkeletonPoint& skeleton_point,
    const rog_map::SubGrid& subgrid,
    std::vector<Eigen::Vector3f>& sample_positions)
{
    // 骨架点位于未知区域内部，需要在自由空间中找采样位置
    // 策略：从骨架点向已知自由空间方向"退后"采样
    
    // 找到最近的自由空间体素
    Eigen::Vector3f nearest_free = skeleton_point.position;
    float min_dist = std::numeric_limits<float>::max();
    
    for (const auto& explored_idx : subgrid.explored_voxels_) {
        auto state = global_map_->getVoxelState(explored_idx);
        if (rog_map::VoxelStateBits::isFree(state)) {
            Eigen::Vector3f free_pos;
            global_map_->indexToPos(explored_idx, free_pos);
            
            float dist = (free_pos - skeleton_point.position).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest_free = free_pos;
            }
        }
    }
    
    // 计算从骨架点指向自由空间的方向（视点采样方向）
    Eigen::Vector3f to_free_direction = nearest_free - skeleton_point.position;
    if (to_free_direction.norm() < 1e-6f) {
        return;
    }
    to_free_direction.normalize();
    
    // 在自由空间内，保持最佳观测距离，朝向骨架点采样
    // 视点位置 = 骨架点 + 观测距离 * 指向自由空间的方向
    // 即视点在自由空间内，距离骨架点optimal_viewing_distance
    
    // 采样点1：最佳观测距离位置
    Eigen::Vector3f optimal_pos = skeleton_point.position + 
        config_.optimal_viewing_distance * to_free_direction;
    if (isPositionSafe(optimal_pos) && global_map_->isFree(optimal_pos)) {
        sample_positions.push_back(optimal_pos);
    }
    
    // 采样点2：较近距离位置（更激进的探索）
    Eigen::Vector3f close_pos = skeleton_point.position + 
        (config_.optimal_viewing_distance * 0.6f) * to_free_direction;
    if (isPositionSafe(close_pos) && global_map_->isFree(close_pos)) {
        sample_positions.push_back(close_pos);
    }
    
    // 采样点3：较远距离位置（更保守的探索）
    Eigen::Vector3f far_pos = skeleton_point.position + 
        (config_.optimal_viewing_distance * 1.5f) * to_free_direction;
    if (isPositionSafe(far_pos) && global_map_->isFree(far_pos)) {
        sample_positions.push_back(far_pos);
    }
    
    // 为每个采样点设置朝向：指向骨架点（未知区域深处）
    // 朝向在创建视点时计算，这里只返回位置
}

/**
 * @brief 使用骨架引导采样生成候选位置
 */
void FrontierViewpointGenerator::sampleFromSkeleton(
    const rog_map::SubGrid& subgrid,
    std::vector<Eigen::Vector3f>& sample_positions)
{
    sample_positions.clear();
    
    int subgrid_id = subgrid.grid_idx.toLinearIndex(global_map_->sub_grid_x_num_);
    
    // 检查骨架缓存
    FreeSpaceSkeleton skeleton;
    auto cache_it = skeleton_cache_.find(subgrid_id);
    if (cache_it != skeleton_cache_.end() && cache_it->second.is_valid) {
        skeleton = cache_it->second;
    } else {
        // 提取新骨架
        extractFreeSpaceSkeleton(subgrid, skeleton);
        skeleton_cache_[subgrid_id] = skeleton;
    }
    
    if (!skeleton.is_valid) {
        return;
    }
    
    // 从每个骨架点生成采样射线
    for (const auto& sp : skeleton.points) {
        generateSkeletonSamplingRays(sp, subgrid, sample_positions);
    }
}

/**
 * @brief 融合均匀采样和骨架采样的结果
 * 
 * 去除重复点，优先保留骨架采样的点
 */
void FrontierViewpointGenerator::mergeSamplingResults(
    const std::vector<Eigen::Vector3f>& uniform_samples,
    const std::vector<Eigen::Vector3f>& skeleton_samples,
    std::vector<Eigen::Vector3f>& merged_samples)
{
    merged_samples.clear();
    
    // 首先添加骨架采样点（优先级高）
    for (const auto& pos : skeleton_samples) {
        merged_samples.push_back(pos);
    }
    
    // 添加均匀采样点（排除与骨架点距离过近的）
    float min_dist_sq = config_.min_viewpoint_distance * config_.min_viewpoint_distance;
    
    for (const auto& uniform_pos : uniform_samples) {
        bool too_close = false;
        for (const auto& existing_pos : merged_samples) {
            if ((uniform_pos - existing_pos).squaredNorm() < min_dist_sq) {
                too_close = true;
                break;
            }
        }
        if (!too_close) {
            merged_samples.push_back(uniform_pos);
        }
    }
}

/**
 * @brief 使用Dijkstra搜索验证视点的可达性
 * 
 * EDEN论文原文：
 * "A Dijkstra search, which starts from the current UAV position in the explored space, 
 *  is conducted to find the path to each candidate viewpoint in the first region of the routing."
 */
bool FrontierViewpointGenerator::verifyReachabilityDijkstra(
    const Eigen::Vector3f& viewpoint_pos,
    float& path_length)
{
    UAVState current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = uav_state_;
    }
    
    // 简化实现：使用直线可行性检查 + 欧氏距离
    // 完整实现应使用Dijkstra在自由空间图上搜索
    
    // 检查直线路径是否可行（只穿过自由空间）
    Eigen::Vector3f direction = viewpoint_pos - current_state.position;
    float length = direction.norm();
    
    if (length < 1e-6f) {
        path_length = 0.0f;
        return true;
    }
    
    direction /= length;
    float step = global_map_->global_resolution_ * 0.5f;
    
    for (float t = 0; t <= length; t += step) {
        Eigen::Vector3f check_pos = current_state.position + t * direction;
        
        rog_map::GlobalVoxelState state = global_map_->getVoxelState(check_pos);
        
        // EDEN约束：路径必须在已探索的自由空间内
        // 不能穿过占用区域，也不能穿过未知区域（可达性要求）
        if (state != rog_map::GlobalVoxelState::FREE) {
            return false;
        }
    }
    
    path_length = length;
    return true;
}

/**
 * @brief 计算视点在给定方向的可见未知体素数量
 */
int FrontierViewpointGenerator::countVisibleUnknownVoxels(
    const Eigen::Vector3f& viewpoint_pos,
    float yaw,
    float pitch)
{
    int count = 0;
    
    // 相机FOV内的射线采样
    const int yaw_samples = 10;
    const int pitch_samples = 8;
    
    float half_fov_h = config_.sensor_fov_h / 2.0f;
    float half_fov_v = config_.sensor_fov_v / 2.0f;
    
    float yaw_step = config_.sensor_fov_h / static_cast<float>(yaw_samples);
    float pitch_step = config_.sensor_fov_v / static_cast<float>(pitch_samples);
    
    for (int i = 0; i < yaw_samples; ++i) {
        float ray_yaw = yaw - half_fov_h + i * yaw_step;
        
        for (int j = 0; j < pitch_samples; ++j) {
            float ray_pitch = pitch - half_fov_v + j * pitch_step;
            
            // 计算射线方向
            Eigen::Vector3f ray_dir(
                std::cos(ray_pitch) * std::cos(ray_yaw),
                std::cos(ray_pitch) * std::sin(ray_yaw),
                -std::sin(ray_pitch)
            );
            
            // 沿射线步进
            float step = global_map_->global_resolution_;
            for (float t = config_.sensor_range_min; t <= config_.sensor_range_max; t += step) {
                Eigen::Vector3f sample_pos = viewpoint_pos + t * ray_dir;
                
                if (!global_map_->isInGlobalMapRange(sample_pos)) {
                    break;
                }
                
                rog_map::GlobalVoxelState state = global_map_->getVoxelState(sample_pos);
                
                if (state == rog_map::GlobalVoxelState::OCCUPIED) {
                    break;  // 射线被阻挡
                } else if (state == rog_map::GlobalVoxelState::UNKNOWN) {
                    ++count;
                }
            }
        }
    }
    
    return count;
}

/**
 * @brief 计算视点的最优朝向（指向未知区域重心）
 * 
 * 策略：计算EROI内未探索体素的重心方向
 */
void FrontierViewpointGenerator::computeOptimalOrientation(
    const Eigen::Vector3f& viewpoint_pos,
    const rog_map::SubGrid& subgrid,
    float& yaw,
    float& pitch)
{
    // 计算EROI内未探索区域的重心
    Eigen::Vector3f unknown_centroid = Eigen::Vector3f::Zero();
    int unknown_count = 0;
    
    // 遍历EROI内的未探索体素
    for (const auto& unexplored_idx : subgrid.unexplored_voxels_) {
        Eigen::Vector3f pos;
        global_map_->indexToPos(unexplored_idx, pos);
        unknown_centroid += pos;
        ++unknown_count;
    }
    
    if (unknown_count == 0) {
        // 如果没有未探索体素，使用子网格中心
        unknown_centroid = subgrid.center;
    } else {
        unknown_centroid /= static_cast<float>(unknown_count);
    }
    
    // 计算朝向
    Eigen::Vector3f direction = unknown_centroid - viewpoint_pos;
    
    if (direction.norm() < 1e-6f) {
        yaw = 0.0f;
        pitch = 0.0f;
        return;
    }
    
    direction.normalize();
    
    // 偏航角
    yaw = std::atan2(direction.y(), direction.x());
    
    // 俯仰角
    float horizontal_dist = std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());
    pitch = std::atan2(-direction.z(), horizontal_dist);
    
    // 限制俯仰角范围
    pitch = std::max(-static_cast<float>(PI / 2.0), std::min(static_cast<float>(PI / 2.0), pitch));
}

// ==================== 曲率与成本计算实现 ====================

float FrontierViewpointGenerator::computeCurvatureToViewpoint(
    const ExplorationViewpoint& viewpoint)
{
    UAVState current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = uav_state_;
    }
    
    // 计算到视点的方向
    Eigen::Vector3f to_viewpoint = viewpoint.position - current_state.position;
    float dist_to_viewpoint = to_viewpoint.norm();
    
    if (dist_to_viewpoint < 1e-6f) {
        return 0.0f;  // 已在目标位置
    }
    
    // 计算锚点位置（沿路径前进 anchor_time * max_velocity 的距离）
    float anchor_distance = config_.anchor_time * config_.max_velocity;
    anchor_distance = std::min(anchor_distance, dist_to_viewpoint);
    
    Eigen::Vector3f anchor_pos = current_state.position + 
        (anchor_distance / dist_to_viewpoint) * to_viewpoint;
    
    // 计算当前速度方向与到锚点方向的夹角
    float speed = current_state.velocity.norm();
    if (speed < 0.1f) {
        // 速度很小，使用当前朝向
        Eigen::Vector3f forward(std::cos(current_state.yaw), std::sin(current_state.yaw), 0);
        speed = 1.0f;  // 避免除零
        
        Eigen::Vector3f to_anchor = anchor_pos - current_state.position;
        float to_anchor_dist = to_anchor.norm();
        
        if (to_anchor_dist < 1e-6f) {
            return 0.0f;
        }
        
        float cos_theta = forward.dot(to_anchor) / to_anchor_dist;
        cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
        float theta = std::acos(cos_theta);
        
        // 曲率公式: K = 2*sin(θ) / |p_anchor - p_current|
        return 2.0f * std::sin(theta) / to_anchor_dist;
    }
    
    // 使用当前速度方向
    Eigen::Vector3f velocity_dir = current_state.velocity / speed;
    Eigen::Vector3f to_anchor = anchor_pos - current_state.position;
    float to_anchor_dist = to_anchor.norm();
    
    if (to_anchor_dist < 1e-6f) {
        return 0.0f;
    }
    
    float cos_theta = velocity_dir.dot(to_anchor) / to_anchor_dist;
    cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
    float theta = std::acos(cos_theta);
    
    // 曲率公式
    return 2.0f * std::sin(theta) / to_anchor_dist;
}

float FrontierViewpointGenerator::computeVirtualVelocity(float curvature)
{
    if (curvature < 1e-6f) {
        return config_.max_velocity;
    }
    
    // 方法1: 基于圆周运动模型 v = sqrt(λ_a * a_m / K)
    float v_from_curvature = std::sqrt(
        config_.curvature_penalty_lambda * config_.max_acceleration / curvature
    );
    
    // 方法2: 指数惩罚 v = v_m * exp(-λ_a * K)
    // float v_from_curvature = config_.max_velocity * 
    //     std::exp(-config_.curvature_penalty_lambda * curvature);
    
    return std::min(v_from_curvature, config_.max_velocity);
}

float FrontierViewpointGenerator::computeExplorationCost(
    const ExplorationViewpoint& viewpoint)
{
    UAVState current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = uav_state_;
    }
    
    // 路径长度
    float path_len = pathLength(current_state.position, viewpoint.position);
    
    // 虚拟速度（已考虑曲率惩罚）
    float virtual_vel = viewpoint.virtual_velocity > 0 ? 
        viewpoint.virtual_velocity : config_.max_velocity;
    
    // 位置运动时间
    float position_time = path_len / virtual_vel;
    
    // 偏航运动时间
    float yaw_diff = std::abs(yawDifference(viewpoint.yaw, current_state.yaw));
    float yaw_time = yaw_diff / config_.max_yaw_rate;
    
    // 取最大值（EDEN公式）
    return std::max(position_time, yaw_time);
}

float FrontierViewpointGenerator::computeContinuousCost(
    const ExplorationViewpoint& vp_explore,
    const ExplorationViewpoint& vp_continuous)
{
    // 路径长度
    float path_len = pathLength(vp_explore.position, vp_continuous.position);
    
    // 使用探索视点的虚拟速度
    float virtual_vel = vp_explore.virtual_velocity > 0 ? 
        vp_explore.virtual_velocity : config_.max_velocity;
    
    // 位置运动时间
    float position_time = path_len / virtual_vel;
    
    // 偏航运动时间
    float yaw_diff = std::abs(yawDifference(vp_continuous.yaw, vp_explore.yaw));
    float yaw_time = yaw_diff / config_.max_yaw_rate;
    
    return std::max(position_time, yaw_time);
}

float FrontierViewpointGenerator::computeViewpointPairScore(
    const ExplorationViewpoint& vp_explore,
    const ExplorationViewpoint& vp_continuous)
{
    float cost_e = computeExplorationCost(vp_explore);
    float cost_c = computeContinuousCost(vp_explore, vp_continuous);
    
    // EDEN评分公式: G(vp_k, vp_j) = exp(-λ_e * CostE) * (1 + exp(-λ_e * CostC))
    float score = std::exp(-config_.exploration_weight_lambda * cost_e) *
                  (1.0f + std::exp(-config_.exploration_weight_lambda * cost_c));
    
    // 可选：加入信息增益权重
    float gain_bonus = config_.gain_weight * 
        (vp_explore.visible_unknown_count + vp_continuous.visible_unknown_count);
    
    return score * (1.0f + gain_bonus / 1000.0f);  // 归一化增益
}

// ==================== 信息增益计算实现 ====================

/**
 * @brief 计算视点的探索信息增益
 * @note EDEN策略：只统计未知体素，不依赖frontier
 */
void FrontierViewpointGenerator::computeExplorationGain(ExplorationViewpoint& viewpoint)
{
    int visible_unknown = 0;
    
    performVisibilityRaycasting(
        viewpoint.position,
        viewpoint.yaw,
        viewpoint.pitch,
        visible_unknown
    );
    
    viewpoint.visible_unknown_count = visible_unknown;
    viewpoint.covered_voxels_count = visible_unknown;
    viewpoint.exploration_gain = static_cast<float>(visible_unknown);
}

/**
 * @brief 射线投射可见性检查
 * @note EDEN策略：只统计未知体素的可见性，不依赖frontier
 */
void FrontierViewpointGenerator::performVisibilityRaycasting(
    const Eigen::Vector3f& viewpoint_pos,
    float viewpoint_yaw,
    float viewpoint_pitch,
    int& visible_unknown_count)
{
    visible_unknown_count = 0;
    
    // 相机FOV内的射线采样
    const int yaw_samples = 20;
    const int pitch_samples = 15;
    
    float half_fov_h = config_.sensor_fov_h / 2.0f;
    float half_fov_v = config_.sensor_fov_v / 2.0f;
    
    float yaw_step = config_.sensor_fov_h / static_cast<float>(yaw_samples);
    float pitch_step = config_.sensor_fov_v / static_cast<float>(pitch_samples);
    
    std::unordered_set<rog_map::GlobalVoxelIndex, rog_map::GlobalVoxelIndexHash> counted_voxels;
    
    for (int i = 0; i < yaw_samples; ++i) {
        float ray_yaw = viewpoint_yaw - half_fov_h + i * yaw_step;
        
        for (int j = 0; j < pitch_samples; ++j) {
            float ray_pitch = viewpoint_pitch - half_fov_v + j * pitch_step;
            
            // 计算射线方向
            Eigen::Vector3f ray_dir(
                std::cos(ray_pitch) * std::cos(ray_yaw),
                std::cos(ray_pitch) * std::sin(ray_yaw),
                -std::sin(ray_pitch)
            );
            
            // 沿射线步进
            float step = global_map_->global_resolution_ * 0.5f;
            for (float t = config_.sensor_range_min; t <= config_.sensor_range_max; t += step) {
                Eigen::Vector3f sample_pos = viewpoint_pos + t * ray_dir;
                
                if (!global_map_->isInGlobalMapRange(sample_pos)) {
                    break;
                }
                
                rog_map::GlobalVoxelIndex sample_idx;
                global_map_->posToIndex(sample_pos, sample_idx);
                
                // 避免重复计数
                if (counted_voxels.find(sample_idx) != counted_voxels.end()) {
                    continue;
                }
                counted_voxels.insert(sample_idx);
                
                rog_map::GlobalVoxelState state = global_map_->getVoxelState(sample_idx);
                
                if (state == rog_map::GlobalVoxelState::OCCUPIED) {
                    // 遇到障碍物，射线终止
                    break;
                } else if (state == rog_map::GlobalVoxelState::UNKNOWN) {
                    // EDEN策略：只统计未知体素，不依赖frontier
                    ++visible_unknown_count;
                }
            }
        }
    }
}

// ==================== 引力模型实现 ====================

float FrontierViewpointGenerator::computeGravitationRadius()
{
    // FC-Planner公式: r_q = 2 * d_v * sin(max(fov_h, fov_w) / 2)
    float max_fov = std::max(config_.sensor_fov_h, config_.sensor_fov_v);
    return config_.gravitation_radius_factor * config_.sensor_range_max * 
           std::sin(max_fov / 2.0f);
}

void FrontierViewpointGenerator::applyGravitationUpdate(
    ExplorationViewpoint& query_vp,
    const std::vector<ExplorationViewpoint>& neighbor_vps)
{
    if (neighbor_vps.empty() || query_vp.covered_voxels_count == 0) {
        return;
    }
    
    Eigen::Vector3f position_update = Eigen::Vector3f::Zero();
    float yaw_update = 0.0f;
    float pitch_update = 0.0f;
    
    float c_q = static_cast<float>(query_vp.covered_voxels_count);
    
    for (const auto& neighbor : neighbor_vps) {
        float c_a = static_cast<float>(neighbor.covered_voxels_count);
        float weight = c_a / c_q;
        
        // 位置更新
        position_update += weight * (neighbor.position - query_vp.position);
        
        // 姿态更新
        yaw_update += weight * yawDifference(neighbor.yaw, query_vp.yaw);
        pitch_update += weight * (neighbor.pitch - query_vp.pitch);
    }
    
    // 应用更新
    query_vp.position += position_update;
    query_vp.yaw = normalizeAngle(query_vp.yaw + yaw_update);
    query_vp.pitch += pitch_update;
    query_vp.pitch = std::max(-static_cast<float>(PI / 2.0), 
                              std::min(static_cast<float>(PI / 2.0), query_vp.pitch));
    
    // 验证更新后的位置安全性
    if (!isPositionSafe(query_vp.position)) {
        // 回退到原位置
        query_vp.position -= position_update;
    }
}

void FrontierViewpointGenerator::mergeCloseViewpoints(
    std::vector<ExplorationViewpoint>& viewpoints)
{
    for (size_t i = 0; i < viewpoints.size(); ++i) {
        if (!viewpoints[i].is_active) continue;
        
        for (size_t j = i + 1; j < viewpoints.size(); ++j) {
            if (!viewpoints[j].is_active) continue;
            
            float dist = distance(viewpoints[i].position, viewpoints[j].position);
            if (dist < config_.merge_distance_threshold) {
                // 保留覆盖体素多的视点
                if (viewpoints[i].covered_voxels_count >= viewpoints[j].covered_voxels_count) {
                    viewpoints[j].is_active = false;
                } else {
                    viewpoints[i].is_active = false;
                    break;  // 当前视点被合并，跳出内层循环
                }
            }
        }
    }
}

// ==================== 安全视点选择实现 ====================

bool FrontierViewpointGenerator::findSafetyViewpoint(
    const ExplorationViewpoint& vp_explore,
    const std::vector<ExplorationViewpoint>& candidate_viewpoints,
    ExplorationViewpoint& vp_safety)
{
    float min_distance = std::numeric_limits<float>::max();
    bool found = false;
    
    for (const auto& vp : candidate_viewpoints) {
        if (!vp.is_valid) continue;
        
        // 安全视点必须完全在已探索的自由空间中
        // 简化检查：确保到探索视点有无障碍路径
        if (!isLineFeasible(vp_explore.position, vp.position)) {
            continue;
        }
        
        float dist = distance(vp_explore.position, vp.position);
        
        // 排除过近的视点（需要有一定的安全距离）
        if (dist < config_.uav_radius * 2.0f) {
            continue;
        }
        
        if (dist < min_distance) {
            min_distance = dist;
            vp_safety = vp;
            found = true;
        }
    }
    
    return found;
}

// ==================== 碰撞检测实现 ====================

bool FrontierViewpointGenerator::isPositionSafe(const Eigen::Vector3f& pos)
{
    // 检查位置是否在地图范围内
    if (!global_map_->isInGlobalMapRange(pos)) {
        return false;
    }
    
    // 检查位置周围是否有足够的安全距离（简化版ESDF检查）
    float check_radius = config_.uav_radius + config_.safety_margin;
    float step = global_map_->global_resolution_;
    
    for (float dx = -check_radius; dx <= check_radius; dx += step) {
        for (float dy = -check_radius; dy <= check_radius; dy += step) {
            for (float dz = -check_radius; dz <= check_radius; dz += step) {
                if (dx * dx + dy * dy + dz * dz > check_radius * check_radius) {
                    continue;
                }
                
                Eigen::Vector3f check_pos = pos + Eigen::Vector3f(dx, dy, dz);
                if (global_map_->isOccupied(check_pos)) {
                    return false;
                }
            }
        }
    }
    
    return true;
}

bool FrontierViewpointGenerator::isLineFeasible(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end)
{
    Eigen::Vector3f direction = end - start;
    float length = direction.norm();
    
    if (length < 1e-6f) {
        return true;
    }
    
    direction /= length;
    float step = global_map_->global_resolution_ * 0.5f;
    
    for (float t = 0; t <= length; t += step) {
        Eigen::Vector3f check_pos = start + t * direction;
        
        if (global_map_->isOccupied(check_pos)) {
            return false;
        }
    }
    
    return true;
}

bool FrontierViewpointGenerator::isViewpointPairFeasible(
    const ExplorationViewpoint& vp_explore,
    const ExplorationViewpoint& vp_continuous)
{
    // EDEN约束：连线可以穿过未知区域，但不能穿过占用区域
    Eigen::Vector3f direction = vp_continuous.position - vp_explore.position;
    float length = direction.norm();
    
    if (length < 1e-6f) {
        return false;  // 两点重合
    }
    
    direction /= length;
    float step = global_map_->global_resolution_ * 0.5f;
    
    for (float t = 0; t <= length; t += step) {
        Eigen::Vector3f check_pos = vp_explore.position + t * direction;
        
        // 只检查占用状态，未知状态是允许的
        if (global_map_->isOccupied(check_pos)) {
            return false;
        }
    }
    
    return true;
}

// ==================== 工具函数实现 ====================

float FrontierViewpointGenerator::yawDifference(float yaw_target, float yaw_current)
{
    float diff = yaw_target - yaw_current;
    
    // 归一化到 [-π, π)
    while (diff >= PI) diff -= TWO_PI;
    while (diff < -PI) diff += TWO_PI;
    
    return diff;
}

float FrontierViewpointGenerator::normalizeAngle(float angle)
{
    while (angle >= PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

float FrontierViewpointGenerator::distance(
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2)
{
    return (p1 - p2).norm();
}

float FrontierViewpointGenerator::pathLength(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end)
{
    // 简化实现：直接使用欧氏距离
    // TODO: 可扩展为A*路径长度
    return distance(start, end);
}

} // namespace rog_map