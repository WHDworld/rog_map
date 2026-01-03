#include "rog_map/global_map.h"

namespace rog_map {

GlobalVoxelState GlobalMap::getVoxelState(const Eigen::Vector3f& pos)
{
    GlobalVoxelIndex idx;
    posToIndex(pos, idx);
    return getVoxelState(idx);
}

GlobalVoxelState GlobalMap::getVoxelState(const GlobalVoxelIndex& idx)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = voxels_.find(idx);
    return (it != voxels_.end()) ? it->second : VoxelStateBits::GEO_UNKNOWN;
}

bool GlobalMap::isInGlobalMapRange(const GlobalVoxelIndex& idx)
{
    return idx.x >= global_min_idx_.x && idx.x < global_max_idx_.x &&
           idx.y >= global_min_idx_.y && idx.y < global_max_idx_.y &&
           idx.z >= global_min_idx_.z && idx.z < global_max_idx_.z;
}

bool GlobalMap::isInGlobalMapRange(const Eigen::Vector3f& pos)
{
    return pos.x() >= global_map_min_.x() && pos.x() < global_map_max_.x() &&
           pos.y() >= global_map_min_.y() && pos.y() < global_map_max_.y() &&
           pos.z() >= global_map_min_.z() && pos.z() < global_map_max_.z();
}

bool GlobalMap::isOccupied(const Eigen::Vector3f& pos)
{
    return VoxelStateBits::isOccupied(getVoxelState(pos));
}

bool GlobalMap::isFree(const Eigen::Vector3f& pos)
{
    return VoxelStateBits::isFree(getVoxelState(pos));
}

bool GlobalMap::isUnknown(const Eigen::Vector3f& pos)
{
    return VoxelStateBits::isUnknown(getVoxelState(pos));
}

void GlobalMap::updateVoxelStateFromLiDAR(const GlobalVoxelIndex& idx, bool is_occupied)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    if(!isInGlobalMapRange(idx)) return;
    auto it = voxels_.find(idx);
    GlobalVoxelState old_state;
    GlobalVoxelState new_state;
    if(it == voxels_.end())
    {
        old_state = VoxelStateBits::GEO_UNKNOWN;
        voxels_[idx] = old_state;
        VoxelStateBits::updateFromLiDAR(voxels_[idx], is_occupied);
    }
    else
    {
        old_state = it->second;
        // if(new_state == GlobalVoxelState::UNKNOWN && old_state != GlobalVoxelState::UNKNOWN) 
        // {
        //     current_problem_idx_count_++;
        //     return;
        // }
        VoxelStateBits::updateFromLiDAR(it->second, is_occupied);
    }

    new_state = voxels_[idx];
    if(old_state == new_state) return;

    // 1. 更新 voxels_ 状态
    SubGridIndex sg_idx;
    voxelToSubGrid(idx, sg_idx);
    SubGrid& sg = getSubGrid(sg_idx);

    if(VoxelStateBits::isFree(new_state) && VoxelStateBits::isUnknown(old_state))
    {
        sg.free_count++;
        sg.explored_voxels_.insert(idx);
        sg.unexplored_voxels_.erase(idx);
        // 标记当前子网格及其邻居为 dirty（因为前沿检测使用全局边界）
        markSubGridDirty(subGridToLinearIndex(sg_idx));
        updateExploredBounds(idx);
    }
    else if(VoxelStateBits::isOccupied(new_state) && VoxelStateBits::isUnknown(old_state))
    {
        sg.occupied_count++;
        sg.explored_voxels_.insert(idx);
        sg.unexplored_voxels_.erase(idx);
        // 标记当前子网格及其邻居为 dirty（因为前沿检测使用全局边界）
        markSubGridDirty(subGridToLinearIndex(sg_idx));
        updateExploredBounds(idx);
    }
    else if(VoxelStateBits::isFree(new_state) && VoxelStateBits::isOccupied(old_state))
    {
        sg.free_count--;
        sg.occupied_count++;
    }
    else if(VoxelStateBits::isOccupied(new_state) && VoxelStateBits::isFree(old_state))
    {
        sg.occupied_count--;
        sg.free_count++;
    }

}

void GlobalMap::updateVoxelStateFromCamera(const GlobalVoxelIndex& idx, bool is_visited)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = voxels_.find(idx);
    if(it == voxels_.end() || !VoxelStateBits::isOccupied(it->second)) return;
    if(VoxelStateBits::isVisited(it->second)) return;
    visited_voxels_.push_back(idx);
    VoxelStateBits::updateFromCamera(it->second, is_visited);
}

bool GlobalMap::isFrontier(const Eigen::Vector3f& pos)
{
    GlobalVoxelIndex idx;
    posToIndex(pos, idx);
    return isFrontier(idx);
}

bool GlobalMap::isFrontier(const GlobalVoxelIndex& idx)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = voxels_.find(idx);
    if(it != voxels_.end())
    {
        return false;
    }

    std::vector<GlobalVoxelIndex> neighbors;
    getNeighbors(idx, neighbors);
    for(const auto& nei : neighbors)
    {
        auto nei_it = voxels_.find(nei);
        if(nei_it != voxels_.end() && !VoxelStateBits::isUnknown(nei_it->second))
        {
            return true;
        }
    }
    return false;
}

void GlobalMap::initializeSubGrids()
{
    int64_t x_voxels = global_max_idx_.x - global_min_idx_.x + 1;
    int64_t y_voxels = global_max_idx_.y - global_min_idx_.y + 1;
    // int64_t z_voxels = global_max_idx_.z - global_min_idx_.z + 1;

    // sub_grid_size_x_ 和 sub_grid_size_y_ 是体素数量，不是米
    sub_grid_x_num_ = static_cast<int>(std::ceil(static_cast<double>(x_voxels) / sub_grid_size_x_));
    sub_grid_y_num_ = static_cast<int>(std::ceil(static_cast<double>(y_voxels) / sub_grid_size_y_));
    sub_grid_total_ = sub_grid_x_num_ * sub_grid_y_num_;

    sub_grids_.resize(sub_grid_total_);
    for(int i = 0; i < sub_grid_x_num_; i++)
    {
        for(int j = 0; j < sub_grid_y_num_; j++)
        {
            int linear_idx = j * sub_grid_x_num_ + i;
            sub_grids_[linear_idx].grid_idx.x = i;
            sub_grids_[linear_idx].grid_idx.y = j;
            sub_grids_[linear_idx].global_min_idx.x = global_min_idx_.x + i * sub_grid_size_x_;
            sub_grids_[linear_idx].global_min_idx.y = global_min_idx_.y + j * sub_grid_size_y_;
            sub_grids_[linear_idx].global_min_idx.z = global_min_idx_.z;
            sub_grids_[linear_idx].global_max_idx.x = std::min(sub_grids_[linear_idx].global_min_idx.x + sub_grid_size_x_ - 1, global_max_idx_.x);
            sub_grids_[linear_idx].global_max_idx.y = std::min(sub_grids_[linear_idx].global_min_idx.y + sub_grid_size_y_ - 1, global_max_idx_.y);
            sub_grids_[linear_idx].global_max_idx.z = global_max_idx_.z;
            sub_grids_[linear_idx].world_min.x() = sub_grids_[linear_idx].global_min_idx.x * global_resolution_ + global_map_min_.x();
            sub_grids_[linear_idx].world_min.y() = sub_grids_[linear_idx].global_min_idx.y * global_resolution_ + global_map_min_.y();
            sub_grids_[linear_idx].world_min.z() = global_map_min_.z();
            sub_grids_[linear_idx].world_max.x() = sub_grids_[linear_idx].global_max_idx.x * global_resolution_ + global_map_min_.x();
            sub_grids_[linear_idx].world_max.y() = sub_grids_[linear_idx].global_max_idx.y * global_resolution_ + global_map_min_.y();
            sub_grids_[linear_idx].world_max.z() = global_map_max_.z();
            sub_grids_[linear_idx].center.x() = (sub_grids_[linear_idx].world_min.x() + sub_grids_[linear_idx].world_max.x()) / 2;
            sub_grids_[linear_idx].center.y() = (sub_grids_[linear_idx].world_min.y() + sub_grids_[linear_idx].world_max.y()) / 2;
            sub_grids_[linear_idx].center.z() = (sub_grids_[linear_idx].world_min.z() + sub_grids_[linear_idx].world_max.z()) / 2;
            sub_grids_[linear_idx].total_voxels = (sub_grids_[linear_idx].global_max_idx.x - sub_grids_[linear_idx].global_min_idx.x + 1) * (sub_grids_[linear_idx].global_max_idx.y - sub_grids_[linear_idx].global_min_idx.y + 1) * (sub_grids_[linear_idx].global_max_idx.z - sub_grids_[linear_idx].global_min_idx.z + 1);
            sub_grids_[linear_idx].unknown_count = sub_grids_[linear_idx].total_voxels;
            sub_grids_[linear_idx].occupied_count = 0;
            sub_grids_[linear_idx].free_count = 0;
            sub_grids_[linear_idx].exploration_rate = 0.0f;
            sub_grids_[linear_idx].unknown_rate = 1.0;
            sub_grids_[linear_idx].has_frontier = false;
            sub_grids_[linear_idx].frontier_count = 0;
            sub_grids_[linear_idx].frontier_dirty = true;
            sub_grids_[linear_idx].was_frontier_last_time = false;
            // 懒加载，暂不初始化 unexplored_voxels_，只在需要时初始化
            sub_grids_[linear_idx].ensureUnexploredInitialized();
        }
    }
    last_frontier_subgrids_.clear();
    current_frontier_subgrids_.clear();
    candidate_subgrids_.clear();
    frontier_cache_valid_ = false;
}

void GlobalMap::voxelToSubGrid(const GlobalVoxelIndex& voxel_idx, SubGridIndex& subgrid_idx)
{
    // 计算相对于global_min_idx_的偏移
    int64_t relative_x = voxel_idx.x - global_min_idx_.x;
    int64_t relative_y = voxel_idx.y - global_min_idx_.y;
    
    subgrid_idx.x = static_cast<int>(relative_x / sub_grid_size_x_);
    subgrid_idx.y = static_cast<int>(relative_y / sub_grid_size_y_);
    
    // 边界限制
    subgrid_idx.x = std::max(0, std::min(subgrid_idx.x, sub_grid_x_num_ - 1));
    subgrid_idx.y = std::max(0, std::min(subgrid_idx.y, sub_grid_y_num_ - 1));
}

void GlobalMap::posToSubGrid(const Eigen::Vector3f& pos, SubGridIndex& subgrid_idx)
{
    GlobalVoxelIndex voxel_idx;
    posToIndex(pos, voxel_idx);
    voxelToSubGrid(voxel_idx, subgrid_idx);
}

void GlobalMap::updateSubGridStatistics(const SubGridIndex& idx)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    sub_grids_[idx.toLinearIndex(sub_grid_x_num_)].unknown_count = 0;
    sub_grids_[idx.toLinearIndex(sub_grid_x_num_)].occupied_count = 0;
    sub_grids_[idx.toLinearIndex(sub_grid_x_num_)].free_count = 0;
}

void GlobalMap::getNeighborSubGrids4(const SubGridIndex& idx, std::vector<SubGridIndex>& neighbors)
{
    neighbors.clear();
    neighbors.reserve(4);
    const int dx[] = {1, -1, 0, 0};
    const int dy[] = {0, 0, 1, -1};
    for(int d = 0; d < 4; d++)
    {
        int nx = idx.x + dx[d];
        int ny = idx.y + dy[d];
        if(nx >= 0 && nx < sub_grid_x_num_ && ny >= 0 && ny < sub_grid_y_num_)
        {
            neighbors.push_back(SubGridIndex(nx, ny));
        }
    }
}

void GlobalMap::extractGlobalFrontierIncremental()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    frontier_voxels_.clear();
    current_frontier_subgrids_.clear();  // 清空当前帧的 frontier 子网格集合

    if(!frontier_cache_valid_ || last_frontier_subgrids_.empty())
    {
        // 🔥 首次提取或缓存失效：检查所有可能有frontier的子网格
        // std::cout << YELLOW << " -- [GlobalOccMap] Full frontier scan (first time or cache invalid)" 
        //           << RESET << std::endl;

        for(int i = 0; i < sub_grid_total_; i++)
        {
            SubGrid& sg = sub_grids_[i];
            if(sg.getExplorationRate() > 0.0f && sg.getExplorationRate() < 1.0f)
            {
                sg.has_frontier = true;
                candidate_subgrids_.insert(i);
            }
            else
            {
                sg.has_frontier = false;
            }
        }
    }
    else
    {
        // 🔥 增量提取：基于上次frontier子网格+膨胀策略
        // std::cout << BLUE << " -- [GlobalOccMap] Incremental frontier scan from " 
        //           << last_frontier_subgrids_.size() << " previous subgrids" << RESET << std::endl;

        std::unordered_set<int> next_candidates;  // 下一轮候选
        std::unordered_set<int> checked_subgrids;
        // Step 1: 从上次的frontier子网格开始
        for(int linear_idx : last_frontier_subgrids_)
        {
            SubGrid& sg = sub_grids_[linear_idx];
            if(sg.getExplorationRate() > 0.0f && sg.getExplorationRate() < 1.0f)
            {
                sg.has_frontier = true;
                next_candidates.insert(linear_idx);
            }
            else
            {
                sg.has_frontier = false;
            }

            std::vector<SubGridIndex> neighbors;
            getNeighborSubGrids4(sg.grid_idx, neighbors);
            for(const auto& nei_idx : neighbors)    
            {
                int nei_linear = subGridToLinearIndex(nei_idx);

                if(checked_subgrids.find(nei_linear) != checked_subgrids.end()) continue;
                checked_subgrids.insert(nei_linear);

                SubGrid& nei_sg = sub_grids_[nei_linear];
                float nei_exp_rate = nei_sg.getExplorationRate();
                if((nei_exp_rate > 0.0f && nei_exp_rate < 1.0f))
                {
                    nei_sg.has_frontier = true;
                    next_candidates.insert(nei_linear);
                }
                else
                {
                    nei_sg.has_frontier = false;
                }
            }
        }
        candidate_subgrids_ = next_candidates;
    }

    for(int linear_idx : candidate_subgrids_)
    {
        SubGrid& sg = sub_grids_[linear_idx];
        if(sg.frontier_dirty)
        {
            // 传入全局地图边界，确保能正确检测跨子网格边界的前沿点
            sg.extractFrontier(voxels_, frontier_voxels_, global_min_idx_, global_max_idx_);
        }
        else
        {
            sg.extractFrontier(frontier_voxels_);
        }
        
        current_frontier_subgrids_.insert(linear_idx);
    
    }

    // ROS_INFO("current_problem_idx_count_: %d", current_problem_idx_count_);

    current_problem_idx_count_ = 0;
    last_frontier_subgrids_ = current_frontier_subgrids_;
    if(current_frontier_subgrids_.size() > 0)
    {
        frontier_cache_valid_ = true;
    }
    else
    {
        ROS_WARN("No frontier subgrids found");
        frontier_cache_valid_ = false;
    }
    // std::cout << GREEN << " -- [GlobalOccMap] Frontier extraction complete:" << RESET << std::endl;
    // std::cout << "    - Candidate subgrids: " << candidate_subgrids_.size() << std::endl;
    // std::cout << "    - Dirty subgrids: " << dirty_subgrids.size() << std::endl;
    // std::cout << "    - Frontier subgrids: " << current_frontier_subgrids_.size() << std::endl;
    // std::cout << "    - Total frontier voxels: " << frontier_voxels.size() << std::endl;

}

void GlobalMap::updateFrontierSubGridCache()
{

}

void GlobalMap::markSubGridDirty(int linear_idx)
{
    if (linear_idx < 0 || linear_idx >= sub_grid_total_) return;
    
    SubGrid& sg = sub_grids_[linear_idx];
    sg.frontier_dirty = true;
    
    // 同时标记邻居子网格为 dirty
    // 这是因为前沿检测使用全局边界，邻居子网格边界处的前沿可能受到影响
    std::vector<SubGridIndex> neighbors;
    getNeighborSubGrids4(sg.grid_idx, neighbors);
    for (const auto& nei_idx : neighbors) {
        int nei_linear = subGridToLinearIndex(nei_idx);
        if (nei_linear >= 0 && nei_linear < sub_grid_total_) {
            sub_grids_[nei_linear].frontier_dirty = true;
        }
    }
}

int GlobalMap::subGridToLinearIndex(const SubGridIndex& idx)
{
    return idx.y * sub_grid_x_num_ + idx.x;
}

SubGrid& GlobalMap::getSubGrid(const SubGridIndex& idx)
{
    int linear_idx = subGridToLinearIndex(idx);
    return sub_grids_[linear_idx];
}

SubGrid& GlobalMap::getSubGrid(int linear_idx)
{
    return sub_grids_[linear_idx];
}

void GlobalMap::getFrontierPointCloud(sensor_msgs::PointCloud2& pc_frontier)
{
    // 1. 先构建PCL点云容器（可直接操作点）
    pcl::PointCloud<pcl::PointXYZ> pcl_pc;
    pcl_pc.header.frame_id = "map";
    pcl_pc.is_dense = true;
    pcl_pc.points.clear();

    // 2. 遍历前沿体素，填充PCL点云
    for(const auto& f_idx : frontier_voxels_)
    {
        Eigen::Vector3f pos;
        indexToPos(f_idx, pos);
        
        // 正确声明PCL的PointXYZ类型
        pcl::PointXYZ p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = pos.z();
        
        // 向PCL点云添加点（push_back是PCL点云的方法）
        pcl_pc.points.push_back(p);
    }

    // 3. 将PCL点云转换为ROS的PointCloud2消息（核心步骤）
    pcl::toROSMsg(pcl_pc, pc_frontier);
    // 补充ROS消息的时间戳（PCL转ROS后header.stamp需手动设置）
    pc_frontier.header.stamp = ros::Time::now();
}

void GlobalMap::getAllKnownPointCloud(sensor_msgs::PointCloud2& pc_all_known)
{
    // 1. 构建PCL点云容器
    pcl::PointCloud<pcl::PointXYZ> pcl_pc;
    pcl_pc.header.frame_id = "map";
    pcl_pc.is_dense = true;
    pcl_pc.points.clear();

    // 2. 遍历已知体素，填充PCL点云
    for(const auto& v_idx : voxels_)
    {
        if(!VoxelStateBits::isUnknown(v_idx.second))
        {
            Eigen::Vector3f pos;
            indexToPos(v_idx.first, pos);
            
            // 正确声明PCL的PointXYZ类型
            pcl::PointXYZ p;
            p.x = pos.x();
            p.y = pos.y();
            p.z = pos.z();
            
            pcl_pc.points.push_back(p);
        }
    }

    // 3. PCL点云转ROS PointCloud2消息
    pcl::toROSMsg(pcl_pc, pc_all_known);
    pc_all_known.header.stamp = ros::Time::now();
}

void GlobalMap::posToIndex(const Eigen::Vector3f& pos, GlobalVoxelIndex& idx)
{
    idx.x = static_cast<int64_t>(std::floor((pos.x() - global_map_min_.x()) * global_resolution_inv_));
    idx.y = static_cast<int64_t>(std::floor((pos.y() - global_map_min_.y()) * global_resolution_inv_));
    idx.z = static_cast<int64_t>(std::floor((pos.z() - global_map_min_.z()) * global_resolution_inv_));
}

void GlobalMap::indexToPos(const GlobalVoxelIndex& idx, Eigen::Vector3f& pos)
{
    pos.x() = (idx.x + 0.5f) * global_resolution_ + global_map_min_.x();
    pos.y() = (idx.y + 0.5f) * global_resolution_ + global_map_min_.y();
    pos.z() = (idx.z + 0.5f) * global_resolution_ + global_map_min_.z();
}

void GlobalMap::getNeighbors(const GlobalVoxelIndex& idx, std::vector<GlobalVoxelIndex>& neighbors)
{
    neighbors.clear();
    neighbors.reserve(26);
    for(int dx = -1; dx <= 1; dx++)
    {
        for(int dy = -1; dy <= 1; dy++)
        {
            for(int dz = -1; dz <= 1; dz++)
            {
                if(dx == 0 && dy == 0 && dz == 0) continue;
                neighbors.push_back(GlobalVoxelIndex(idx.x + dx, idx.y + dy, idx.z + dz));
            }
        }
    }
}

void GlobalMap::updateExploredBounds(const GlobalVoxelIndex& idx)
{
    Eigen::Vector3f global_pos;
    indexToPos(idx, global_pos);
    if(!is_bound_initialized_)
    {
        explored_min_ = global_pos;
        explored_max_ = global_pos;
        explored_min_idx_ = idx;
        explored_max_idx_ = idx;
        is_bound_initialized_ = true;
    }
    else
    {
        explored_min_ = explored_min_.cwiseMin(global_pos);
        explored_max_ = explored_max_.cwiseMax(global_pos);
        posToIndex(explored_min_, explored_min_idx_);
        posToIndex(explored_max_, explored_max_idx_);
    }
}

bool GlobalMap::isPointInFrustum(const GlobalVoxelIndex& target_idx)
{
    // 1. 获取目标点的物理坐标 (世界坐标系)
    Eigen::Vector3f target_pos;
    indexToPos(target_idx, target_pos);
    
    return isPointInFrustum(target_pos);
}

bool GlobalMap::isPointInFrustum(const Eigen::Vector3f& target_pos)
{
    // 将点从世界坐标系转换到相机/机体坐标系
    // P_body = R_world_to_body * (P_world - t_cam)
    // 其中 R_world_to_body = R_body_to_world.inverse() = current_cam_rot_.inverse()
    Eigen::Vector3f p_body = current_cam_rot_.inverse() * (target_pos - current_cam_origin_);
    
    // 机体坐标系约定 (FLU - Front Left Up):
    //   x: 前向 (forward)
    //   y: 左向 (left)  
    //   z: 上向 (up)
    // 相机假设固定安装在机体上，朝向机体 x 轴方向
    float forward_dist = p_body.x(); // 前向距离（深度）
    float horiz_dev    = p_body.y(); // 水平偏差（左为正）
    float vert_dev     = p_body.z(); // 垂直偏差（上为正）

    // 1. 必须在相机前方（forward_dist > 0）
    if (forward_dist <= 0) {
        return false;
    }
    
    // 2. 距离检查 (Depth Check)
    if (forward_dist < min_dist_ || forward_dist > max_dist_) {
        return false;
    }

    // 3. 水平角度检查 (Horizontal FOV Check)
    // 利用相似三角形原理: |y| <= x * tan(fov_h / 2)
    float max_horiz_dev = forward_dist * tan_half_fov_h_;
    if (std::abs(horiz_dev) > max_horiz_dev) {
        return false;
    }

    // 4. 垂直角度检查 (Vertical FOV Check)
    // |z| <= x * tan(fov_v / 2)
    float max_vert_dev = forward_dist * tan_half_fov_v_;
    if (std::abs(vert_dev) > max_vert_dev) {
        return false;
    }

    return true;
}
void GlobalMap::visualizeGlobalMapVoxels()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if(!is_bound_initialized_)
    {
        return;
    }
    
    // 检查是否有任何一个发布器有订阅者
    if (global_map_voxels_pub_.getNumSubscribers() == 0 && 
        global_map_occupied_pub_.getNumSubscribers() == 0 && 
        global_map_free_pub_.getNumSubscribers() == 0) {
        return;
    }
    
    // 使用带颜色的点云来区分不同状态的体素（参考 rog_map 的占据栅格可视化）
    pcl::PointCloud<pcl::PointXYZRGB> free_cloud, occupied_cloud;
    
    // 只遍历已知的体素（存储在 voxels_ 中）
    for(const auto& voxel_pair : voxels_)
    {
        const GlobalVoxelIndex& idx = voxel_pair.first;
        const GlobalVoxelState& state = voxel_pair.second;
        
        Eigen::Vector3f pos;
        indexToPos(idx, pos);
        
        pcl::PointXYZRGB point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        
        // 根据状态设置颜色并添加到对应的点云
        switch(state)
        {
            case VoxelStateBits::GEO_UNKNOWN:
                // 不显示 UNKNOWN 状态（因为默认就是未知）
                break;
            case VoxelStateBits::GEO_FREE:
                // 白色
                point.r = 255;
                point.g = 255;
                point.b = 255;
                free_cloud.push_back(point);
                break;
            case VoxelStateBits::GEO_OCCUPIED:
                // 红色
                point.r = 255;
                point.g = 0;
                point.b = 0;
                occupied_cloud.push_back(point);
                break;
        }
    }
    
    
    // 转换为 ROS PointCloud2 消息并发布（只发布有订阅者的话题）
    if (!free_cloud.empty() || !occupied_cloud.empty()) {
        // 单独发布 OCCUPIED 体素（如果有订阅者）
        if (global_map_occupied_pub_.getNumSubscribers() > 0 && !occupied_cloud.empty()) {
            sensor_msgs::PointCloud2 occupied_cloud_msg;
            pcl::toROSMsg(occupied_cloud, occupied_cloud_msg);
            occupied_cloud_msg.header.stamp = ros::Time::now();
            occupied_cloud_msg.header.frame_id = "map";
            global_map_occupied_pub_.publish(occupied_cloud_msg);
        }
        
        // 单独发布 FREE 体素（如果有订阅者）
        if (global_map_free_pub_.getNumSubscribers() > 0 && !free_cloud.empty()) {
            sensor_msgs::PointCloud2 free_cloud_msg;
            pcl::toROSMsg(free_cloud, free_cloud_msg);
            free_cloud_msg.header.stamp = ros::Time::now();
            free_cloud_msg.header.frame_id = "map";
            global_map_free_pub_.publish(free_cloud_msg);
        }

        // 合并发布所有体素（如果有订阅者）
        if (global_map_voxels_pub_.getNumSubscribers() > 0) {
            pcl::PointCloud<pcl::PointXYZRGB> combined_cloud;
            combined_cloud = free_cloud + occupied_cloud;
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(combined_cloud, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            cloud_msg.header.frame_id = "map";
            global_map_voxels_pub_.publish(cloud_msg);
        }
    }
    // 定期打印统计信息
    // static size_t last_total_voxels = 0;
    // size_t current_total_voxels = free_cloud.size() + occupied_cloud.size();
    // if (current_total_voxels > 0 && (current_total_voxels - last_total_voxels > 100 || last_total_voxels == 0)) 
    // {
    //     ROS_INFO_THROTTLE(5.0, "[Global Map] Voxels - FREE: %lu, OCCUPIED: %lu, Total stored: %lu",
    //              free_cloud.size(), 
    //              occupied_cloud.size(),
    //              voxels_.size());
    //     last_total_voxels = current_total_voxels;
    // }
}

void GlobalMap::visualizeGlobalMapRange()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    visualization_msgs::MarkerArray marker_array;

    // 创建边界框的线段 Marker
    visualization_msgs::Marker range_marker;
    range_marker.header.frame_id = "map";
    range_marker.header.stamp = ros::Time::now();
    range_marker.ns = "global_map_range";
    range_marker.id = 0;
    range_marker.type = visualization_msgs::Marker::LINE_LIST;
    range_marker.action = visualization_msgs::Marker::ADD;
    range_marker.pose.orientation.w = 1.0;
    
    // 设置线宽
    range_marker.scale.x = 0.05f;  // 线宽
    
    // 设置颜色：蓝色边界框
    range_marker.color.r = 0.0f;
    range_marker.color.g = 0.5f;
    range_marker.color.b = 1.0f;
    range_marker.color.a = 1.0f;
    
    range_marker.lifetime = ros::Duration(0);  // 持久显示
    
    // 定义8个顶点
    geometry_msgs::Point p[8];
    // 底面4个顶点
    p[0].x = global_map_min_.x(); p[0].y = global_map_min_.y(); p[0].z = global_map_min_.z();
    p[1].x = global_map_max_.x(); p[1].y = global_map_min_.y(); p[1].z = global_map_min_.z();
    p[2].x = global_map_max_.x(); p[2].y = global_map_max_.y(); p[2].z = global_map_min_.z();
    p[3].x = global_map_min_.x(); p[3].y = global_map_max_.y(); p[3].z = global_map_min_.z();
    // 顶面4个顶点
    p[4].x = global_map_min_.x(); p[4].y = global_map_min_.y(); p[4].z = global_map_max_.z();
    p[5].x = global_map_max_.x(); p[5].y = global_map_min_.y(); p[5].z = global_map_max_.z();
    p[6].x = global_map_max_.x(); p[6].y = global_map_max_.y(); p[6].z = global_map_max_.z();
    p[7].x = global_map_min_.x(); p[7].y = global_map_max_.y(); p[7].z = global_map_max_.z();
    
    // 底面4条边
    range_marker.points.push_back(p[0]); range_marker.points.push_back(p[1]);
    range_marker.points.push_back(p[1]); range_marker.points.push_back(p[2]);
    range_marker.points.push_back(p[2]); range_marker.points.push_back(p[3]);
    range_marker.points.push_back(p[3]); range_marker.points.push_back(p[0]);
    
    // 顶面4条边
    range_marker.points.push_back(p[4]); range_marker.points.push_back(p[5]);
    range_marker.points.push_back(p[5]); range_marker.points.push_back(p[6]);
    range_marker.points.push_back(p[6]); range_marker.points.push_back(p[7]);
    range_marker.points.push_back(p[7]); range_marker.points.push_back(p[4]);
    
    // 4条竖边
    range_marker.points.push_back(p[0]); range_marker.points.push_back(p[4]);
    range_marker.points.push_back(p[1]); range_marker.points.push_back(p[5]);
    range_marker.points.push_back(p[2]); range_marker.points.push_back(p[6]);
    range_marker.points.push_back(p[3]); range_marker.points.push_back(p[7]);
    
    marker_array.markers.push_back(range_marker);
    
    // 添加文本标签 - 全局地图范围（参考局部地图的文本位置设置）
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "global_map_range_text";
    text_marker.id = 1;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    
    // 文本位置：全局地图边界框右上角上方（与局部地图文本位置一致）
    text_marker.pose.position.x = global_map_max_.x();
    text_marker.pose.position.y = global_map_max_.y();
    text_marker.pose.position.z = global_map_max_.z() + 1.0f;
    text_marker.pose.orientation.w = 1.0;
    
    text_marker.scale.z = 0.6f;  // 与局部地图文本大小一致
    
    text_marker.color.r = 0.0f;
    text_marker.color.g = 0.5f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;
    
    text_marker.text = "Global Map Range";
    text_marker.lifetime = ros::Duration(0);
    
    marker_array.markers.push_back(text_marker);
    
    // 如果已经有探索数据，添加探索范围的文本
    if (is_bound_initialized_) {
        visualization_msgs::Marker explored_text_marker;
        explored_text_marker.header.frame_id = "map";
        explored_text_marker.header.stamp = ros::Time::now();
        explored_text_marker.ns = "global_map_explored_text";
        explored_text_marker.id = 2;
        explored_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        explored_text_marker.action = visualization_msgs::Marker::ADD;
        
        // 文本位置：探索区域右上角上方
        explored_text_marker.pose.position.x = explored_max_.x();
        explored_text_marker.pose.position.y = explored_max_.y();
        explored_text_marker.pose.position.z = explored_max_.z() + 0.5f;
        explored_text_marker.pose.orientation.w = 1.0;
        
        explored_text_marker.scale.z = 0.5f;
        
        explored_text_marker.color.r = 1.0f;
        explored_text_marker.color.g = 1.0f;
        explored_text_marker.color.b = 0.0f;
        explored_text_marker.color.a = 1.0f;
        
        explored_text_marker.text = "Explored Area";
        explored_text_marker.lifetime = ros::Duration(0);
        
        marker_array.markers.push_back(explored_text_marker);
    }
    
    // 发布
    global_map_range_pub_.publish(marker_array);
    
    // 定期打印探索范围信息
    if (!is_bound_initialized_) 
    {
        ROS_INFO_THROTTLE(10.0, "Global map explored range: [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
            explored_min_.x(), explored_min_.y(), explored_min_.z(),
            explored_max_.x(), explored_max_.y(), explored_max_.z());
    }
}

void GlobalMap::visualizeGlobalMapFrontier()
{
    if(!is_bound_initialized_)
    {
        return;
    }

    if(global_map_frontier_pub_.getNumSubscribers() == 0 || frontier_voxels_.empty())
    {
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB> frontier_cloud;
    for(const auto& f_idx : frontier_voxels_)
    {
        Eigen::Vector3f pos;
        indexToPos(f_idx, pos);
        pcl::PointXYZRGB point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        point.r = 0;
        point.g = 255;
        point.b = 0;
        frontier_cloud.push_back(point);
    }
    // ROS_INFO("Frontier voxels: %d", frontier_cloud.size());
    sensor_msgs::PointCloud2 frontier_cloud_msg;
    pcl::toROSMsg(frontier_cloud, frontier_cloud_msg);
    frontier_cloud_msg.header.stamp = ros::Time::now();
    frontier_cloud_msg.header.frame_id = "map";
    global_map_frontier_pub_.publish(frontier_cloud_msg);
}

void GlobalMap::visualizeSubGridRange()
{
    if(!is_bound_initialized_)
    {
        return;
    }

    if(frontier_range_pub_.getNumSubscribers() == 0)
    {
        return;
    }
    
    visualization_msgs::MarkerArray marker_array;
    
    // 先发送一个 DELETEALL marker 来清除所有旧的 marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "frontier_subgrid_range";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // 遍历所有子网格，根据状态用不同颜色显示边界框
    int marker_id = 1;  // 从 1 开始，因为 0 用于 DELETEALL
    for(auto& last_frontier_subgrids : last_frontier_subgrids_ )
    {
        SubGrid& sg = sub_grids_[last_frontier_subgrids];
        
        // 创建子网格边界框的线段 Marker
        visualization_msgs::Marker grid_marker;
        grid_marker.header.frame_id = "map";
        grid_marker.header.stamp = ros::Time::now();
        grid_marker.ns = "frontier_subgrid_range";
        grid_marker.id = marker_id++;
        grid_marker.type = visualization_msgs::Marker::LINE_LIST;
        grid_marker.action = visualization_msgs::Marker::ADD;
        grid_marker.pose.orientation.w = 1.0;
        
        // 设置线宽
        grid_marker.scale.x = 0.03f;
        
        // 根据子网格状态设置颜色
        float exp_rate = sg.getExplorationRate();
        if(sg.has_frontier)
        {
            // 包含前视点的子网格：绿色
            grid_marker.color.r = 0.0f;
            grid_marker.color.g = 1.0f;
            grid_marker.color.b = 0.0f;
            grid_marker.color.a = 1.0f;
        }
        else if(exp_rate < 0.01f)
        {
            // 未探索的子网格：灰色，半透明
            grid_marker.color.r = 0.5f;
            grid_marker.color.g = 0.5f;
            grid_marker.color.b = 0.5f;
            grid_marker.color.a = 0.3f;
        }
        else if(exp_rate > 0.999f)
        {
            // 完全探索的子网格：蓝色
            grid_marker.color.r = 0.0f;
            grid_marker.color.g = 0.0f;
            grid_marker.color.b = 1.0f;
            grid_marker.color.a = 0.5f;
        }
        else
        {
            // 部分探索的子网格：黄色
            grid_marker.color.r = 1.0f;
            grid_marker.color.g = 1.0f;
            grid_marker.color.b = 0.0f;
            grid_marker.color.a = 0.7f;
        }
        
        grid_marker.lifetime = ros::Duration(0);
        
        // 定义8个顶点
        geometry_msgs::Point p[8];
        // 底面4个顶点
        p[0].x = sg.world_min.x(); p[0].y = sg.world_min.y(); p[0].z = sg.world_min.z();
        p[1].x = sg.world_max.x(); p[1].y = sg.world_min.y(); p[1].z = sg.world_min.z();
        p[2].x = sg.world_max.x(); p[2].y = sg.world_max.y(); p[2].z = sg.world_min.z();
        p[3].x = sg.world_min.x(); p[3].y = sg.world_max.y(); p[3].z = sg.world_min.z();
        // 顶面4个顶点
        p[4].x = sg.world_min.x(); p[4].y = sg.world_min.y(); p[4].z = sg.world_max.z();
        p[5].x = sg.world_max.x(); p[5].y = sg.world_min.y(); p[5].z = sg.world_max.z();
        p[6].x = sg.world_max.x(); p[6].y = sg.world_max.y(); p[6].z = sg.world_max.z();
        p[7].x = sg.world_min.x(); p[7].y = sg.world_max.y(); p[7].z = sg.world_max.z();
        // ROS_INFO("SubGrid[%d] frontier_count: %d, exploration_rate: %f, unknown_rate: %f, sub_grid_range: [%.2f, %.2f, %.2f] m to [%.2f, %.2f, %.2f] m", 
        // last_frontier_subgrids, sg.frontier_count, sg.getExplorationRate(), sg.getUnknownRate(), 
        // sg.world_min.x(), sg.world_min.y(), sg.world_min.z(), sg.world_max.x(), sg.world_max.y(), sg.world_max.z());
        // 底面4条边
        grid_marker.points.push_back(p[0]); grid_marker.points.push_back(p[1]);
        grid_marker.points.push_back(p[1]); grid_marker.points.push_back(p[2]);
        grid_marker.points.push_back(p[2]); grid_marker.points.push_back(p[3]);
        grid_marker.points.push_back(p[3]); grid_marker.points.push_back(p[0]);
        
        // 顶面4条边
        grid_marker.points.push_back(p[4]); grid_marker.points.push_back(p[5]);
        grid_marker.points.push_back(p[5]); grid_marker.points.push_back(p[6]);
        grid_marker.points.push_back(p[6]); grid_marker.points.push_back(p[7]);
        grid_marker.points.push_back(p[7]); grid_marker.points.push_back(p[4]);
        
        // 4条竖边
        grid_marker.points.push_back(p[0]); grid_marker.points.push_back(p[4]);
        grid_marker.points.push_back(p[1]); grid_marker.points.push_back(p[5]);
        grid_marker.points.push_back(p[2]); grid_marker.points.push_back(p[6]);
        grid_marker.points.push_back(p[3]); grid_marker.points.push_back(p[7]);
        
        marker_array.markers.push_back(grid_marker);
    }
    // ROS_INFO("SubGrid range markers: %d", marker_array.markers.size());
    // 发布所有子网格边界框
    frontier_range_pub_.publish(marker_array);
}

void GlobalMap::visualizeVisitedVoxels()
{
    if(visited_voxels_pub_.getNumSubscribers() == 0 || visited_voxels_.empty())
    {
        return;
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
    pcl::PointCloud<pcl::PointXYZRGB> visited_cloud;
    for(const auto& v_idx : visited_voxels_)
    {
        Eigen::Vector3f pos;
        indexToPos(v_idx, pos);
        pcl::PointXYZRGB point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        point.r = 0;
        point.g = 0;
        point.b = 255;
        visited_cloud.push_back(point);
    }

    sensor_msgs::PointCloud2 visited_cloud_msg;
    pcl::toROSMsg(visited_cloud, visited_cloud_msg);
    visited_cloud_msg.header.stamp = ros::Time::now();
    visited_cloud_msg.header.frame_id = "map";
    visited_voxels_pub_.publish(visited_cloud_msg);

}

void GlobalMap::visualizeCameraFrustum()
{
    // 1. 基础检查：如果没有订阅者，直接返回节省算力
    if(camera_frustum_pub_.getNumSubscribers() == 0)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);

    // 2. 直接使用成员变量计算局部尺寸 (无需重复计算 tan)
    // 假设成员变量命名为: tan_half_fov_h_, tan_half_fov_v_, min_dist_, max_dist_
    float h_near = min_dist_ * tan_half_fov_h_; // 近平面半宽
    float v_near = min_dist_ * tan_half_fov_v_; // 近平面半高
    float h_far  = max_dist_ * tan_half_fov_h_; // 远平面半宽
    float v_far  = max_dist_ * tan_half_fov_v_; // 远平面半高

    // 3. 定义局部坐标系下的关键点 (8个角点)
    // 坐标系假设: X向前(Forward), Y向左(Left), Z向上(Up)
    std::vector<Eigen::Vector3f> local_pts = {
        // 近平面 (Near Plane) - 0,1,2,3
        {min_dist_,  h_near,  v_near}, // 左上
        {min_dist_, -h_near,  v_near}, // 右上
        {min_dist_, -h_near, -v_near}, // 右下
        {min_dist_,  h_near, -v_near}, // 左下
        // 远平面 (Far Plane) - 4,5,6,7
        {max_dist_,  h_far,  v_far},   // 左上
        {max_dist_, -h_far,  v_far},   // 右上
        {max_dist_, -h_far, -v_far},   // 右下
        {max_dist_,  h_far, -v_far}    // 左下
    };

    // 4. 坐标变换 (Local -> World) 并转换为 ROS Point
    std::vector<geometry_msgs::Point> world_pts(8);
    geometry_msgs::Point cam_origin_pt;
    
    // 相机原点
    cam_origin_pt.x = current_cam_origin_.x();
    cam_origin_pt.y = current_cam_origin_.y();
    cam_origin_pt.z = current_cam_origin_.z();

    for(int i=0; i<8; ++i) {
        // P_world = T_world_cam * P_local
        // 利用成员变量 current_cam_origin_ 和 current_cam_rot_
        Eigen::Vector3f p_w = current_cam_origin_ + current_cam_rot_ * local_pts[i];
        world_pts[i].x = p_w.x();
        world_pts[i].y = p_w.y();
        world_pts[i].z = p_w.z();
    }

    // 5. 构建 Visualization Marker
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker frustum_marker;
    
    frustum_marker.header.frame_id = "map";
    frustum_marker.header.stamp = ros::Time::now();
    frustum_marker.ns = "camera_frustum";
    frustum_marker.id = 0;
    frustum_marker.type = visualization_msgs::Marker::LINE_LIST;
    frustum_marker.action = visualization_msgs::Marker::ADD;
    
    // Pose 设为单位阵，因为点已经是世界坐标
    frustum_marker.pose.orientation.w = 1.0;
    frustum_marker.scale.x = 0.1f; // 线宽
    
    // 颜色设置 (青色，半透明)
    frustum_marker.color.r = 0.0f;
    frustum_marker.color.g = 0.8f;
    frustum_marker.color.b = 1.0f;
    frustum_marker.color.a = 0.8f;
    frustum_marker.lifetime = ros::Duration(0);

    // 6. 添加连线 (LINE_LIST 模式)
    auto add_line = [&](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        frustum_marker.points.push_back(p1);
        frustum_marker.points.push_back(p2);
    };

    // A. 近平面矩形 (0-1-2-3-0)
    for(int i=0; i<4; ++i) add_line(world_pts[i], world_pts[(i+1)%4]);

    // B. 远平面矩形 (4-5-6-7-4)
    for(int i=0; i<4; ++i) add_line(world_pts[4+i], world_pts[4 + (i+1)%4]);

    // C. 连接棱 (0-4, 1-5, 2-6, 3-7)
    for(int i=0; i<4; ++i) add_line(world_pts[i], world_pts[i+4]);

    // D. 原点连线 (Origin -> Near Plane)
    for(int i=0; i<4; ++i) add_line(cam_origin_pt, world_pts[i]);

    marker_array.markers.push_back(frustum_marker);
    camera_frustum_pub_.publish(marker_array);
}
}