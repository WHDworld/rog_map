// frontier_viewpoint_generator.cpp
// 前端视点生成器实现

#include "rog_map/frontier_viewpoint_generator.h"
#include <numeric>
#include <functional>

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

// Union-Find 数据结构，用于连通性合并
class UnionFind {
public:
    UnionFind(int n) : parent_(n), rank_(n, 0) {
        for (int i = 0; i < n; ++i) parent_[i] = i;
    }
    
    int find(int x) {
        if (parent_[x] != x) {
            parent_[x] = find(parent_[x]);  // 路径压缩
        }
        return parent_[x];
    }
    
    void unite(int x, int y) {
        int px = find(x), py = find(y);
        if (px == py) return;
        
        // 按秩合并
        if (rank_[px] < rank_[py]) std::swap(px, py);
        parent_[py] = px;
        if (rank_[px] == rank_[py]) rank_[px]++;
    }
    
    bool connected(int x, int y) {
        return find(x) == find(y);
    }
    
private:
    std::vector<int> parent_;
    std::vector<int> rank_;
};

void FrontierViewpointGenerator::partitionIntoRegions(
    const std::vector<int>& subgrid_ids,
    std::vector<ExplorationRegion>& regions)
{
    regions.clear();
    
    if (subgrid_ids.empty()) return;
    
    // ========== 阶段1：过滤有效子网格 ==========
    // 时间复杂度：O(m)，m = subgrid_ids.size()
    std::vector<int> valid_sg_ids;
    std::unordered_map<int, int> sg_id_to_idx;  // subgrid_id -> 在valid_sg_ids中的索引
    
    for (int sg_id : subgrid_ids) {
        SubGrid& sg = global_map_->getSubGrid(sg_id);
        
        // 过滤条件：有前沿体素且未完全探索
        if (sg.sub_grid_frontier_voxels_.empty()) continue;
        
        float exp_rate = sg.getExplorationRate();
        if (exp_rate <= 0.0f || exp_rate >= 1.0f) continue;
        
        sg_id_to_idx[sg_id] = static_cast<int>(valid_sg_ids.size());
        valid_sg_ids.push_back(sg_id);
    }
    
    if (valid_sg_ids.empty()) return;
    
    // 只有一个有效子网格，直接返回
    if (valid_sg_ids.size() == 1) {
        int sg_id = valid_sg_ids[0];
        SubGrid& sg = global_map_->getSubGrid(sg_id);
        
        ExplorationRegion region;
        region.region_id = 0;
        region.subgrid_ids.push_back(sg_id);
        region.centroid = sg.center;
        region.total_frontier_area = static_cast<float>(sg.sub_grid_frontier_voxels_.size());
        regions.push_back(region);
        
        computeRegionPriorities(regions);
        return;
    }
    
    // ========== 阶段2：受限连通性合并（优化版） ==========
    // 合并约束参数（从配置读取）
    const float max_region_diameter = config_.planning_horizon * config_.max_region_diameter_factor;
    const float max_direction_diff = config_.max_direction_diff;
    const float max_region_area = config_.max_region_area;
    
    int n = static_cast<int>(valid_sg_ids.size());
    UnionFind uf(n);
    
    // 预计算每个子网格的信息
    // 时间复杂度：O(n * k_sample)，k_sample = min(体素数, MAX_SAMPLE)
    struct SubGridInfo {
        Eigen::Vector3f center;
        Eigen::Vector3f main_frontier_direction;  // 主要前沿方向
        float frontier_area;
        SubGridIndex grid_idx;  // 空间索引，用于快速邻居查找
    };
    std::vector<SubGridInfo> sg_infos(n);
    
    constexpr int MAX_SAMPLE_VOXELS = 20;  // 采样上限，避免遍历所有体素
    
    for (int i = 0; i < n; ++i) {
        int sg_id = valid_sg_ids[i];
        SubGrid& sg = global_map_->getSubGrid(sg_id);
        
        sg_infos[i].center = sg.center;
        sg_infos[i].frontier_area = static_cast<float>(sg.sub_grid_frontier_voxels_.size());
        sg_infos[i].grid_idx = sg.grid_idx;  // 保存空间索引
        
        // 【优化】采样计算前沿方向，避免遍历所有体素
        Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
        int sample_count = 0;
        int step = std::max(1, static_cast<int>(sg.sub_grid_frontier_voxels_.size()) / MAX_SAMPLE_VOXELS);
        int idx = 0;
        
        for (const auto& fv : sg.sub_grid_frontier_voxels_) {
            if (idx % step == 0) {
                Eigen::Vector3f pos;
                global_map_->indexToPos(fv, pos);
                avg_normal += (sg.center - pos).normalized();
                sample_count++;
                if (sample_count >= MAX_SAMPLE_VOXELS) break;
            }
            idx++;
        }
        
        if (avg_normal.norm() > 1e-6f) {
            sg_infos[i].main_frontier_direction = avg_normal.normalized();
        } else {
            sg_infos[i].main_frontier_direction = Eigen::Vector3f::UnitZ();
        }
    }
    
    // 【优化】使用空间索引检测真正相邻的子网格对
    // 时间复杂度：O(n * 4) = O(n)，而非之前的 O(n²)
    struct MergePair {
        int i, j;
        float distance;
        bool operator>(const MergePair& other) const {
            return distance > other.distance;
        }
    };
    std::priority_queue<MergePair, std::vector<MergePair>, std::greater<MergePair>> merge_queue;
    
    // 使用集合避免重复添加邻居对（i,j）和（j,i）
    std::unordered_set<uint64_t> processed_pairs;
    auto makePairKey = [](int a, int b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
    };
    
    for (int i = 0; i < n; ++i) {
        // 获取4邻域子网格
        std::vector<SubGridIndex> neighbors;
        global_map_->getNeighborSubGrids4(sg_infos[i].grid_idx, neighbors);
        
        for (const auto& nei_idx : neighbors) {
            int nei_sg_id = global_map_->subGridToLinearIndex(nei_idx);
            
            // 检查邻居是否在有效列表中
            auto it = sg_id_to_idx.find(nei_sg_id);
            if (it == sg_id_to_idx.end()) continue;
            
            int j = it->second;
            
            // 使用哈希集合避免重复处理（i,j）和（j,i）
            uint64_t pair_key = makePairKey(i, j);
            if (processed_pairs.count(pair_key) > 0) continue;
            processed_pairs.insert(pair_key);
            
            float dist = (sg_infos[i].center - sg_infos[j].center).norm();
            merge_queue.push({i, j, dist});
        }
    }
    
    // 维护每个连通分量的信息
    struct ComponentInfo {
        Eigen::Vector3f bbox_min, bbox_max;
        float total_area;
        Eigen::Vector3f avg_direction;
    };
    std::unordered_map<int, ComponentInfo> component_infos;
    
    // 初始化每个组件
    for (int i = 0; i < n; ++i) {
        ComponentInfo info;
        info.bbox_min = sg_infos[i].center;
        info.bbox_max = sg_infos[i].center;
        info.total_area = sg_infos[i].frontier_area;
        info.avg_direction = sg_infos[i].main_frontier_direction;
        component_infos[i] = info;
    }
    
    // Lambda: 获取组件信息（通过Union-Find根节点）
    auto getComponentInfo = [&](int idx) -> ComponentInfo& {
        int root = uf.find(idx);
        return component_infos[root];
    };
    
    // Lambda: 计算合并后的直径
    auto computeMergedDiameter = [](const ComponentInfo& c1, const ComponentInfo& c2) -> float {
        Eigen::Vector3f new_min = c1.bbox_min.cwiseMin(c2.bbox_min);
        Eigen::Vector3f new_max = c1.bbox_max.cwiseMax(c2.bbox_max);
        return (new_max - new_min).norm();
    };
    
    // Lambda: 计算方向差异
    auto computeDirectionDiff = [](const Eigen::Vector3f& d1, const Eigen::Vector3f& d2) -> float {
        float dot = d1.dot(d2);
        dot = std::max(-1.0f, std::min(1.0f, dot));  // 替代 std::clamp (C++17)
        return std::acos(std::abs(dot));  // 使用绝对值，因为方向可能相反
    };
    
    // 按距离从近到远尝试合并
    // 时间复杂度：O(p * log(p))，p = 实际相邻对数量（通常 << n²）
    while (!merge_queue.empty()) {
        MergePair pair = merge_queue.top();
        merge_queue.pop();
        
        // 【关键检查】已经在同一组件中，跳过（避免重复处理）
        if (uf.connected(pair.i, pair.j)) continue;
        
        // 获取两个组件的当前信息
        ComponentInfo& comp_i = getComponentInfo(pair.i);
        ComponentInfo& comp_j = getComponentInfo(pair.j);
        
        // 检查约束1：合并后直径
        float merged_diameter = computeMergedDiameter(comp_i, comp_j);
        if (merged_diameter > max_region_diameter) continue;
        
        // 检查约束2：方向差异
        float dir_diff = computeDirectionDiff(comp_i.avg_direction, comp_j.avg_direction);
        if (dir_diff > max_direction_diff) continue;
        
        // 检查约束3：合并后面积
        float merged_area = comp_i.total_area + comp_j.total_area;
        if (merged_area > max_region_area) continue;
        
        // 满足所有约束，执行合并
        int old_root_i = uf.find(pair.i);
        int old_root_j = uf.find(pair.j);
        uf.unite(pair.i, pair.j);
        int new_root = uf.find(pair.i);
        
        // 更新合并后组件的信息
        ComponentInfo& merged = component_infos[new_root];
        merged.bbox_min = comp_i.bbox_min.cwiseMin(comp_j.bbox_min);
        merged.bbox_max = comp_i.bbox_max.cwiseMax(comp_j.bbox_max);
        merged.total_area = merged_area;
        
        // 加权平均方向
        merged.avg_direction = (comp_i.avg_direction * comp_i.total_area + 
                               comp_j.avg_direction * comp_j.total_area).normalized();
        
        // 清理旧的组件信息（可选，节省内存）
        if (old_root_i != new_root) component_infos.erase(old_root_i);
        if (old_root_j != new_root) component_infos.erase(old_root_j);
    }
    
    // ========== 阶段3：提取连通分量作为探索区域 ==========
    // 时间复杂度：O(n)
    std::unordered_map<int, ExplorationRegion> root_to_region;
    
    for (int i = 0; i < n; ++i) {
        int root = uf.find(i);
        int sg_id = valid_sg_ids[i];
        SubGrid& sg = global_map_->getSubGrid(sg_id);
        
        if (root_to_region.find(root) == root_to_region.end()) {
            ExplorationRegion region;
            region.region_id = static_cast<int>(root_to_region.size());
            region.centroid = Eigen::Vector3f::Zero();
            region.total_frontier_area = 0.0f;
            root_to_region[root] = region;
        }
        
        ExplorationRegion& region = root_to_region[root];
        region.subgrid_ids.push_back(sg_id);
        region.centroid += sg.center * static_cast<float>(sg.sub_grid_frontier_voxels_.size());
        region.total_frontier_area += static_cast<float>(sg.sub_grid_frontier_voxels_.size());
    }
    
    // 归一化中心点（按前沿面积加权平均）
    for (auto& kv : root_to_region) {
        ExplorationRegion& region = kv.second;
        if (region.total_frontier_area > 0) {
            region.centroid /= region.total_frontier_area;
        }
    }
    
    // 转换为向量
    regions.reserve(root_to_region.size());
    for (auto& kv : root_to_region) {
        regions.push_back(std::move(kv.second));
    }
    
    // 重新分配区域ID
    for (size_t i = 0; i < regions.size(); ++i) {
        regions[i].region_id = static_cast<int>(i);
    }
    
    // ========== 阶段4：计算探索优先级 ==========
    computeRegionPriorities(regions);
    
    ROS_DEBUG("[FrontierViewpointGenerator] Partitioned %lu subgrids into %lu regions (processed %lu pairs)",
              valid_sg_ids.size(), regions.size(), processed_pairs.size());
}

void FrontierViewpointGenerator::computeRegionPriorities(std::vector<ExplorationRegion>& regions)
{
    UAVState current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = uav_state_;
    }
    
    for (auto& region : regions) {
        float dist = distance(current_state.position, region.centroid);
        // 优先级 = 前视面积 / (距离 + 1)，避免除零
        // 可以考虑添加更多因素：方向一致性、区域紧凑度等
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
    
    // 确定性下采样（按空间位置排序，避免随机性导致视点不稳定）
    if (samples.size() > static_cast<size_t>(config_.max_viewpoints_per_region * 5)) {
        // 按照空间位置排序，保证结果稳定
        std::sort(samples.begin(), samples.end(), 
            [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                if (std::abs(a.x() - b.x()) > 0.01f) return a.x() < b.x();
                if (std::abs(a.y() - b.y()) > 0.01f) return a.y() < b.y();
                return a.z() < b.z();
            });
        // 均匀下采样，而不是随机选择
        size_t target_size = config_.max_viewpoints_per_region * 5;
        size_t step = samples.size() / target_size;
        std::vector<Eigen::Vector3f> downsampled;
        downsampled.reserve(target_size);
        for (size_t i = 0; i < samples.size() && downsampled.size() < target_size; i += step) {
            downsampled.push_back(samples[i]);
        }
        samples = std::move(downsampled);
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

// ==================== Step 5: 区域路由规划 (EDEN风格MST) ====================

/**
 * @brief MST边结构
 */
struct MSTEdge {
    int from, to;
    float weight;
    
    bool operator<(const MSTEdge& other) const {
        return weight < other.weight;
    }
};

/**
 * @brief MST树节点结构
 */
struct MSTNode {
    int region_idx;
    int parent;
    std::vector<int> children;
    float distance_to_root;
    bool is_leaf;
    
    MSTNode() : region_idx(-1), parent(-1), distance_to_root(0.0f), is_leaf(true) {}
};

void FrontierViewpointGenerator::planRegionRoute(
    std::vector<ExplorationRegion>& regions,
    std::vector<int>& route)
{
    route.clear();
    
    if (regions.empty()) {
        return;
    }
    
    int n = static_cast<int>(regions.size());
    
    // 只有一个区域，直接返回
    if (n == 1) {
        route.push_back(regions[0].region_id);
        return;
    }
    
    UAVState current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = uav_state_;
    }
    
    // ========== Step 1: 计算距离矩阵 ==========
    std::vector<std::vector<float>> dist_matrix(n, std::vector<float>(n, 0.0f));
    
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            float d = distance(regions[i].centroid, regions[j].centroid);
            dist_matrix[i][j] = d;
            dist_matrix[j][i] = d;
        }
    }
    
    // ========== Step 2: 找到离UAV最近的区域作为根节点 ==========
    int root_idx = 0;
    float min_dist = std::numeric_limits<float>::max();
    
    for (int i = 0; i < n; ++i) {
        float d = distance(current_state.position, regions[i].centroid);
        if (d < min_dist) {
            min_dist = d;
            root_idx = i;
        }
    }
    
    // ========== Step 3: Prim算法构建MST ==========
    std::vector<MSTNode> mst_nodes(n);
    std::vector<bool> in_mst(n, false);
    std::vector<float> min_edge(n, std::numeric_limits<float>::max());
    std::vector<int> parent(n, -1);
    
    // 从根节点开始
    min_edge[root_idx] = 0.0f;
    
    for (int count = 0; count < n; ++count) {
        // 找到不在MST中且边权最小的节点
        int u = -1;
        float min_w = std::numeric_limits<float>::max();
        
        for (int i = 0; i < n; ++i) {
            if (!in_mst[i] && min_edge[i] < min_w) {
                min_w = min_edge[i];
                u = i;
            }
        }
        
        if (u == -1) break;
        
        in_mst[u] = true;
        mst_nodes[u].region_idx = u;
        mst_nodes[u].parent = parent[u];
        
        if (parent[u] >= 0) {
            mst_nodes[parent[u]].children.push_back(u);
            mst_nodes[parent[u]].is_leaf = false;
            mst_nodes[u].distance_to_root = mst_nodes[parent[u]].distance_to_root + 
                                            dist_matrix[parent[u]][u];
        }
        
        // 更新相邻节点的边权
        for (int v = 0; v < n; ++v) {
            if (!in_mst[v] && dist_matrix[u][v] < min_edge[v]) {
                min_edge[v] = dist_matrix[u][v];
                parent[v] = u;
            }
        }
    }
    
    // 更新叶节点标记
    for (int i = 0; i < n; ++i) {
        mst_nodes[i].is_leaf = mst_nodes[i].children.empty();
    }
    
    // ========== Step 4: EOHDT优化 - 找到最远叶节点分支 ==========
    int farthest_leaf = -1;
    float max_dist_to_root = -1.0f;
    
    for (int i = 0; i < n; ++i) {
        if (mst_nodes[i].is_leaf && mst_nodes[i].distance_to_root > max_dist_to_root) {
            max_dist_to_root = mst_nodes[i].distance_to_root;
            farthest_leaf = i;
        }
    }
    
    // 找到最远分支的起点（从根到最远叶节点路径上的第一个分叉点的子节点）
    int farthest_branch_start = -1;
    if (farthest_leaf >= 0) {
        int current = farthest_leaf;
        while (current >= 0) {
            int p = mst_nodes[current].parent;
            if (p == root_idx) {
                farthest_branch_start = current;
                break;
            }
            current = p;
        }
    }
    
    // ========== Step 5: DFS遍历生成路由（最远分支后置） ==========
    std::vector<int> traversal_order;
    std::vector<bool> dfs_visited(n, false);
    
    // DFS遍历函数（lambda）
    std::function<void(int)> dfs = [&](int node) {
        dfs_visited[node] = true;
        traversal_order.push_back(node);
        
        // 对子节点按距离排序（最近优先）
        std::vector<std::pair<float, int>> sorted_children;
        for (int child : mst_nodes[node].children) {
            // 最远分支的起点最后处理
            if (child == farthest_branch_start) {
                sorted_children.push_back({std::numeric_limits<float>::max(), child});
        } else {
                sorted_children.push_back({dist_matrix[node][child], child});
            }
        }
        std::sort(sorted_children.begin(), sorted_children.end());
        
        for (const auto& p : sorted_children) {
            if (!dfs_visited[p.second]) {
                dfs(p.second);
            }
        }
    };
    
    dfs(root_idx);
    
    // ========== Step 6: 转换为区域ID ==========
    for (int idx : traversal_order) {
        route.push_back(regions[idx].region_id);
    }
    
    // ========== 可达性检查与不可达区域后置 ==========
    // 检查每个区域是否可达（通过检查是否有安全路径）
    std::vector<int> reachable_route;
    std::vector<int> unreachable_route;
    
    Eigen::Vector3f check_pos = current_state.position;
    
    for (int region_id : route) {
        // 查找对应的区域
        bool is_reachable = true;
        Eigen::Vector3f target_pos;
        
        for (const auto& region : regions) {
            if (region.region_id == region_id) {
                target_pos = region.centroid;
                
                // 简单可达性检查：检查直线路径是否被完全阻挡
                // （完整的可达性检查应使用A*或其他路径规划）
                int obstacle_count = 0;
                int check_count = 0;
                float step = global_map_->global_resolution_ * 2;
                Eigen::Vector3f dir = target_pos - check_pos;
                float length = dir.norm();
            
                if (length > step) {
                    dir /= length;
                    for (float t = step; t < length - step; t += step) {
                        Eigen::Vector3f p = check_pos + t * dir;
                        if (global_map_->isOccupied(p)) {
                            obstacle_count++;
                        }
                        check_count++;
                        if (check_count > 50) break;  // 限制检查数量
                    }
                    
                    // 如果超过30%的检查点被占据，认为不可达
                    if (check_count > 0 && 
                        static_cast<float>(obstacle_count) / check_count > 0.3f) {
                        is_reachable = false;
                    }
                }
            break;
        }
        }
        
        if (is_reachable) {
            reachable_route.push_back(region_id);
            check_pos = target_pos;
        } else {
            unreachable_route.push_back(region_id);
        }
    }
    
    // 合并路由：可达区域在前，不可达区域在后
    route.clear();
    route.insert(route.end(), reachable_route.begin(), reachable_route.end());
    route.insert(route.end(), unreachable_route.begin(), unreachable_route.end());
    
    ROS_DEBUG("[FrontierViewpointGenerator] MST route: %lu regions (%lu reachable, %lu deferred)",
              route.size(), reachable_route.size(), unreachable_route.size());
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
