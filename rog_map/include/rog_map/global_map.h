#ifndef GLOBAL_MAP_H
#define GLOBAL_MAP_H

#include <unordered_map>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <mutex>
#include <string>
#include <vector>
#include <limits>  // 必须包含，用于数值极限
#include <ros/ros.h>
#include <unordered_set>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
namespace rog_map {

// 全局体素状态（紧凑存储，1字节）
// enum class GlobalVoxelState : uint8_t {
//     // --- 初始状态 ---
//     UNKNOWN = 0,            // 0: 完全未探索（几何、语义均未知）

//     // --- 几何已探索，无需视觉介入 ---
//     FREE = 1,               // 1: 已知自由（LiDAR确认空闲，无需相机探索）
    
//     // --- 几何已探索，等待视觉确认 (关键的 "Camera Frontier") ---
//     OCCUPIED_UNSEEN = 2,    // 2: 几何占据 + 视觉未覆盖 (这是相机探索的候选目标)
    
//     // --- 几何已探索 + 视觉已确认/已排除 ---
//     OCCUPIED_TARGET = 3,    // 3: 几何占据 + 视觉确认为目标 (True Positive)
//       = 4     // 4: 几何占据 + (视觉确认为背景 OR 几何先验判定为平面/墙体)
// };
typedef uint8_t GlobalVoxelState;
struct VoxelStateBits {

    // --- 1. 几何层 (占用低 2 位: 0000 0011) ---
    static constexpr uint8_t MASK_GEO       = 0x03; // 用于提取几何状态的掩码

    static constexpr uint8_t GEO_UNKNOWN    = 0x00; // 0: 未知

    static constexpr uint8_t GEO_FREE       = 0x01; // 1: 自由

    static constexpr uint8_t GEO_OCCUPIED   = 0x02; // 2: 占据
    // --- 2. 视觉层 (占用第 2 位: 0000 0100) ---
    // 标记该体素是否被相机的视锥体有效覆盖过（且未被遮挡）
    static constexpr uint8_t FLAG_VISITED   = 0x04; 
    // --- 3. 语义层 (占用第 3 位: 0000 1000) ---
    // 标记该体素是否被确认为目标
    static constexpr uint8_t FLAG_TARGET    = 0x08; 
    // --- 辅助判断函数 ---
    static bool isUnknown(uint8_t s)  { return (s & MASK_GEO) == GEO_UNKNOWN; }

    static bool isFree(uint8_t s)     { return (s & MASK_GEO) == GEO_FREE; }

    static bool isOccupied(uint8_t s) { return (s & MASK_GEO) == GEO_OCCUPIED; }

    static bool isVisited(uint8_t s)  { return (s & FLAG_VISITED) != 0; }

    static bool isTarget(uint8_t s)   { return (s & FLAG_TARGET) != 0; }
};

// 扩展辅助函数
struct VoxelHelper : public VoxelStateBits {
    
    // --- 1. LiDAR 更新流 (只修改几何层) ---
    // 输入：当前状态引用，观测结果(是否占据)
    static void updateFromLiDAR(uint8_t& s, bool is_occupied) {
        // 清除旧的几何状态 (保留视觉和语义位)
        s &= ~MASK_GEO; 
        
        if (is_occupied) {
            s |= GEO_OCCUPIED;
            // 注意：几何变占据，保持之前的 Visited/Target 状态不变
            // 因为之前可能由几何先验判断过，或者LiDAR再次确认了该位置
        } else {
            s |= GEO_FREE;
            // 【关键逻辑】: 如果物理上变为了 FREE，那么它一定不是 TARGET
            // 强制清除 TARGET 位，但保留 VISITED 位（表示我们看过了，这里确实是空的）
            s &= ~FLAG_TARGET; 
        }
    }
    // --- 2. 相机/几何先验 更新流 (修改视觉/语义层) ---
    // 输入：当前状态引用，是否是目标，是否仅标记为已看(Ignored)
    static void updateFromCamera(uint8_t& s, bool is_target) {
        // 前置检查：如果几何是 UNKNOWN，通常相机无法单独更新它 (依赖深度/LiDAR)
        // 但如果你的策略允许纯视觉建图，可以去掉这个检查
        if (isUnknown(s)) return; 

        // 1. 标记为“已探索” (Visited)
        s |= FLAG_VISITED;

        // 2. 更新语义信息
        if (is_target) {
            // 只有几何占据时，才能标记为目标 (避免空中出现目标)
            if (isOccupied(s)) {
                s |= FLAG_TARGET;
            }
        } else {
            // 确认为背景，或者几何先验排除了 -> 清除 Target 位
            // 此时状态变为：GEO_OCCUPIED | FLAG_VISITED (即 Ignored)
            s &= ~FLAG_TARGET;
        }
    }
    
    // --- 3. 几何先验流 (比如平面检测) ---
    static void updateFromGeometricPrior(uint8_t& s) {
        if (isOccupied(s)) {
            // 标记为 Visited (表示不需要再去看了)
            s |= FLAG_VISITED;
            // 确保 Target 为 0 (表示这是背景/墙面)
            s &= ~FLAG_TARGET;
        }
    }
};

struct GlobalVoxelIndex
{
    int64_t x, y, z;
    GlobalVoxelIndex() : x(0), y(0), z(0) {}
    GlobalVoxelIndex(int64_t x_, int64_t y_, int64_t z_) : x(x_), y(y_), z(z_) {}

    GlobalVoxelIndex operator+(const GlobalVoxelIndex& other) const
    {
        return GlobalVoxelIndex(x + other.x, y + other.y, z + other.z);
    }
    bool operator==(const GlobalVoxelIndex& other) const 
    { 
        return x == other.x && y == other.y && z == other.z; 
    }

    int64_t operator[](int idx) const
    {
        if (idx == 0) return x;
        if (idx == 1) return y;
        return z;
    }
};

struct GlobalVoxelIndexHash
{
    std::size_t operator()(const GlobalVoxelIndex& idx) const
    {
        std::size_t h1 = std::hash<int64_t>{}(idx.x);
        std::size_t h2 = std::hash<int64_t>{}(idx.y);
        std::size_t h3 = std::hash<int64_t>{}(idx.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

const GlobalVoxelIndex neighbor_offsets_6[6] = {
    GlobalVoxelIndex(1, 0, 0),   // +X
    GlobalVoxelIndex(-1, 0, 0),  // -X
    GlobalVoxelIndex(0, 1, 0),   // +Y
    GlobalVoxelIndex(0, -1, 0),  // -Y
    GlobalVoxelIndex(0, 0, 1),   // +Z
    GlobalVoxelIndex(0, 0, -1)   // -Z
};

struct SubGridIndex 
{
    int x, y;
    SubGridIndex() : x(0), y(0) {}
    SubGridIndex(int x_, int y_) : x(x_), y(y_) {}
    int toLinearIndex(int grid_x_num) const {
        return y * grid_x_num + x;
    }
};

struct SubGrid
{
    GlobalVoxelIndex global_min_idx, global_max_idx;
    Eigen::Vector3f world_min, world_max;
    Eigen::Vector3f center;
    SubGridIndex grid_idx; // 自身在全局的索引

    // 统计信息
    int total_voxels, unknown_count, occupied_count, free_count;
    float exploration_rate = 0.0, unknown_rate = 1.0;
    bool has_frontier;
    int frontier_count; // frontier点数估计

    std::unordered_set<GlobalVoxelIndex, GlobalVoxelIndexHash> explored_voxels_;
    std::unordered_set<GlobalVoxelIndex, GlobalVoxelIndexHash> unexplored_voxels_;
    std::unordered_set<GlobalVoxelIndex, GlobalVoxelIndexHash> sub_grid_frontier_voxels_;

    // Frontier增量更新标记
    bool frontier_dirty = true; // 是否需要重新检查frontier
    bool was_frontier_last_time; // 上次是否包含frontier
    bool unexplored_initialized_ = false; // 未探索体素是否已初始化
    
    float getExplorationRate()
    {
        exploration_rate = static_cast<float>(explored_voxels_.size()) / total_voxels;
        return exploration_rate;
    }

    float getUnknownRate()
    {
        unknown_rate = static_cast<float>(unexplored_voxels_.size()) / total_voxels;
        return unknown_rate;
    }

    void ensureUnexploredInitialized()
    {
        if(unexplored_initialized_) return;
        for(int64_t x = global_min_idx.x; x <= global_max_idx.x; x++)
        {
            for(int64_t y = global_min_idx.y; y <= global_max_idx.y; y++)
            {
                for(int64_t z = global_min_idx.z; z <= global_max_idx.z; z++)
                {
                    unexplored_voxels_.insert(GlobalVoxelIndex(x, y, z));
                }
            }
        }
        unexplored_initialized_ = true;
    }

    bool should_iterate_explored()
    {
        return exploration_rate < 0.5f;
    }

    void extractFrontier(std::vector<GlobalVoxelIndex>& frontier_voxels)
    {
        for(const auto& sub_grid_frontier_voxel : sub_grid_frontier_voxels_)
        {
            frontier_voxels.push_back(sub_grid_frontier_voxel);
        }
    }

    void extractFrontier(const std::unordered_map<GlobalVoxelIndex, GlobalVoxelState, GlobalVoxelIndexHash>& voxels,
    std::vector<GlobalVoxelIndex>& frontier_voxels)
    {
        sub_grid_frontier_voxels_.clear();

        if(should_iterate_explored())
        {
            for(const auto& explored_idx : explored_voxels_)
            {
                auto it = voxels.find(explored_idx);
                if(it == voxels.end() || !VoxelStateBits::isFree(it->second))
                {
                    continue;
                }

                bool is_frontier = false;
                for(int d = 0; d < 6; d++)
                {
                    GlobalVoxelIndex nei_idx = explored_idx + neighbor_offsets_6[d];
                    if(!isInRange(nei_idx)) continue;

                    auto nei_it = voxels.find(nei_idx);
                    if(nei_it == voxels.end() || VoxelStateBits::isUnknown(nei_it->second))
                    {
                        sub_grid_frontier_voxels_.insert(nei_idx);
                        frontier_voxels.push_back(nei_idx);
                    }
                }
            }
        }
        else
        {
            for(const auto& unexplored_idx : unexplored_voxels_)
            {
                bool is_frontier = false;
                for(int d = 0; d < 6; d++)
                {
                    GlobalVoxelIndex nei_idx = unexplored_idx + neighbor_offsets_6[d];
                    if(!isInRange(nei_idx)) continue;
                    auto nei_it = voxels.find(nei_idx);
                    if(nei_it != voxels.end() && !VoxelStateBits::isUnknown(nei_it->second))
                    {
                        is_frontier = true;
                        break;
                    }
                }
                if(is_frontier)
                {
                    sub_grid_frontier_voxels_.insert(unexplored_idx);
                    frontier_voxels.push_back(unexplored_idx);
                }
            }
        }

        frontier_count = sub_grid_frontier_voxels_.size();
        frontier_dirty = false;

        // std::cout << "SubGrid[" << grid_idx.x << "," << grid_idx.y << "] "
        //           << "explored:" << explored_voxels_.size() 
        //           << " unexplored:" << unexplored_voxels_.size() 
        //           << " → frontier:" << frontier_count 
        //           << " (strategy: " << (should_iterate_explored() ? "explored" : "unexplored") << ")" << std::endl;
    }

    void globalToLocal(const GlobalVoxelIndex& global_idx, GlobalVoxelIndex& local_idx)
    {
        local_idx.x = global_idx.x - global_min_idx.x;
        local_idx.y = global_idx.y - global_min_idx.y;
        local_idx.z = global_idx.z - global_min_idx.z;
    }

    void localToGlobal(const GlobalVoxelIndex& local_idx, GlobalVoxelIndex& global_idx)
    {
        global_idx.x = local_idx.x + global_min_idx.x;
        global_idx.y = local_idx.y + global_min_idx.y;
        global_idx.z = local_idx.z + global_min_idx.z;
    }

    bool isInRange(const GlobalVoxelIndex& idx)
    {
        return idx.x >= global_min_idx.x && idx.x < global_max_idx.x &&
               idx.y >= global_min_idx.y && idx.y < global_max_idx.y &&
               idx.z >= global_min_idx.z && idx.z < global_max_idx.z;
    }

};


class GlobalMap 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<GlobalMap> Ptr;

    GlobalMap(const ros::NodeHandle& nh, const std::string& name_space = "rog_map") : nh_(nh)
    {
        LoadParam(name_space + "/global_map/resolution", global_resolution_, 0.2f);
        global_resolution_inv_ = 1.0f / global_resolution_;
        std::vector<float> temp_global_map_size;
        LoadParam(name_space + "/global_map/map_size", temp_global_map_size, std::vector<float>{200.0, 200.0, 5.0});
        if(temp_global_map_size.size() != 3)
        {
            ROS_ERROR("Global map size dimension is not 3!");
            exit(-1);
        }
        global_map_size_ = Eigen::Vector3f(temp_global_map_size[0], temp_global_map_size[1], temp_global_map_size[2]);
        LoadParam(name_space + "/global_map/map_model", global_map_model_, 0); // 0: center, 1: right_bottom, 2: right_middle, 3: left_bottom
        LoadParam(name_space + "/global_map/ground_height", global_map_ground_height_, -0.1f);
        if(global_map_model_ == 0)
        {
            global_map_min_ = Eigen::Vector3f(-global_map_size_[0] / 2, -global_map_size_[1] / 2, global_map_ground_height_);
            global_map_max_ = Eigen::Vector3f(global_map_size_[0] / 2, global_map_size_[1] / 2, global_map_ground_height_ + global_map_size_[2]);
        }
        else if(global_map_model_ == 1)
        {
            global_map_min_ = Eigen::Vector3f(0.0, -global_map_size_[1], global_map_ground_height_);
            global_map_max_ = Eigen::Vector3f(global_map_size_[0], 0, global_map_ground_height_ + global_map_size_[2]);
        }
        else if(global_map_model_ == 2)
        {
            global_map_min_ = Eigen::Vector3f(0.0, -global_map_size_[1] / 2, global_map_ground_height_);
            global_map_max_ = Eigen::Vector3f(global_map_size_[0], global_map_size_[1] / 2, global_map_ground_height_ + global_map_size_[2]);
        }
        else if(global_map_model_ == 3)
        {
            global_map_min_ = Eigen::Vector3f(0.0, 0.0, global_map_ground_height_);
            global_map_max_ = Eigen::Vector3f(global_map_size_[0], global_map_size_[1], global_map_ground_height_ + global_map_size_[2]);
        }
        else
        {
            global_map_min_ = Eigen::Vector3f(0.0, -global_map_size_[1] / 2, global_map_ground_height_);
            global_map_max_ = Eigen::Vector3f(global_map_size_[0], global_map_size_[1] / 2, global_map_ground_height_ + global_map_size_[2]); 
        }
        posToIndex(global_map_min_, global_min_idx_);
        posToIndex(global_map_max_, global_max_idx_);

        LoadParam(name_space + "/global_map/sub_grid_xy", sub_grid_xy_, 0.1f);
        sub_grid_x_ = (global_map_max_.x() - global_map_min_.x()) * sub_grid_xy_;
        sub_grid_y_ = (global_map_max_.y() - global_map_min_.y()) * sub_grid_xy_;

        sub_grid_size_x_ = static_cast<int>(sub_grid_x_ * global_resolution_inv_);
        sub_grid_size_y_ = static_cast<int>(sub_grid_y_ * global_resolution_inv_);

        // 初始化子网格
        initializeSubGrids();

        // 初始化可视化发布器（参考 rog_map 的占据栅格可视化方式）
        global_map_voxels_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/voxels", 1);
        global_map_occupied_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/occupied", 1);
        global_map_free_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/free", 1);
        global_map_frontier_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map/frontier", 1);
        global_map_range_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("global_map/range", 1);
        frontier_range_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("global_map/sub_grid_range", 1);
        ROS_INFO("GlobalMap initialized with visualization publishers");
    }
    ~GlobalMap() = default;

    GlobalVoxelState getVoxelState(const Eigen::Vector3f& pos);
    GlobalVoxelState getVoxelState(const GlobalVoxelIndex& idx);
    bool isInGlobalMapRange(const GlobalVoxelIndex& idx);
    bool isInGlobalMapRange(const Eigen::Vector3f& pos);

    bool isOccupied(const Eigen::Vector3f& pos);
    bool isFree(const Eigen::Vector3f& pos);
    bool isUnknown(const Eigen::Vector3f& pos);
    
    void updateVoxelStateFromLiDAR(const GlobalVoxelIndex& idx, bool is_occupied);

    bool isFrontier(const Eigen::Vector3f& pos);
    bool isFrontier(const GlobalVoxelIndex& idx);

    void initializeSubGrids();
    void voxelToSubGrid(const GlobalVoxelIndex& voxel_idx, SubGridIndex& subgrid_idx);
    void posToSubGrid(const Eigen::Vector3f& pos, SubGridIndex& subgrid_idx);
    void updateSubGridStatistics(const SubGridIndex& idx);
    void getNeighborSubGrids4(const SubGridIndex& idx, std::vector<SubGridIndex>& neighbors);
    void extractGlobalFrontierIncremental();
    void updateFrontierSubGridCache();
    void markSubGridDirty(int linear_idx);
    int subGridToLinearIndex(const SubGridIndex& idx);
    SubGrid& getSubGrid(const SubGridIndex& idx);
    SubGrid& getSubGrid(int linear_idx);

    void getFrontierPointCloud(sensor_msgs::PointCloud2& pc_frontier);
    void getAllKnownPointCloud(sensor_msgs::PointCloud2& pc_all_known);

    void clearMap();
    void posToIndex(const Eigen::Vector3f& pos, GlobalVoxelIndex& idx);
    void indexToPos(const GlobalVoxelIndex& idx, Eigen::Vector3f& pos);
    void getNeighbors(const GlobalVoxelIndex& idx, std::vector<GlobalVoxelIndex>& neighbors);
    void updateExploredBounds(const GlobalVoxelIndex& idx);

    void visualizeGlobalMapVoxels();
    void visualizeGlobalMapRange();
    void visualizeGlobalMapFrontier();
    void visualizeSubGridRange();
    std::unordered_map<GlobalVoxelIndex, GlobalVoxelState, GlobalVoxelIndexHash> voxels_;
    std::vector<GlobalVoxelIndex> frontier_voxels_;
private:
    ros::NodeHandle nh_;

    ros::Publisher global_map_voxels_pub_;  // 体素化可视化发布器
    ros::Publisher global_map_occupied_pub_;  // 占用体素可视化发布器
    ros::Publisher global_map_free_pub_;  // 自由体素可视化发布器
    ros::Publisher global_map_frontier_pub_;  // 前沿体素可视化发布器
    ros::Publisher global_map_range_pub_;  // 范围可视化发布器
    ros::Publisher frontier_range_pub_;  // 前沿范围可视化发布器
    std::vector<SubGrid> sub_grids_;
    int sub_grid_x_num_, sub_grid_y_num_, sub_grid_total_;
    int sub_grid_size_x_, sub_grid_size_y_;
    float sub_grid_xy_, sub_grid_x_, sub_grid_y_;

    std::unordered_set<int> last_frontier_subgrids_, current_frontier_subgrids_;
    std::unordered_set<int> candidate_subgrids_; // 本次需要检查的子网格
    bool frontier_cache_valid_ = false;

    mutable std::mutex map_mutex_;

    float global_resolution_;
    float global_resolution_inv_;
    float global_map_ground_height_;

    int global_map_model_;
    bool is_bound_initialized_ = false;

    Eigen::Vector3f global_map_size_;
    Eigen::Vector3f global_map_min_;
    Eigen::Vector3f global_map_max_;
    GlobalVoxelIndex global_min_idx_, global_max_idx_;
    Eigen::Vector3f explored_min_ = Eigen::Vector3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector3f explored_max_ = Eigen::Vector3f(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
    GlobalVoxelIndex explored_min_idx_ = GlobalVoxelIndex(std::numeric_limits<int64_t>::max(), std::numeric_limits<int64_t>::max(), std::numeric_limits<int64_t>::max());
    GlobalVoxelIndex explored_max_idx_ = GlobalVoxelIndex(std::numeric_limits<int64_t>::min(), std::numeric_limits<int64_t>::min(), std::numeric_limits<int64_t>::min());

    int current_problem_idx_count_ = 0;
    template<class T>
    bool LoadParam(std::string param_name, T &param_value, T default_value = T{}, bool required = false) {
        if (nh_.getParam(param_name, param_value)) {
            printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
            std::cout << param_value << std::endl;
            return true;
        } else {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                   (nh_.getNamespace() + "/" + param_name).c_str());
            param_value = default_value;
            std::cout << param_value << std::endl;
            if (required) {
                throw std::invalid_argument(
                        std::string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
            }
            return false;
        }
    }

    template<class T>
    bool LoadParam(std::string param_name, std::vector<T> &param_value, std::vector<T> default_value = std::vector<T>{},
                   bool required = false) {
        if (nh_.getParam(param_name, param_value)) {
            printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
            for (size_t i = 0; i < param_value.size(); i++) {
                std::cout << param_value[i] << " ";
            }
            std::cout << std::endl;
            return true;
        } else {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                   (nh_.getNamespace() + "/" + param_name).c_str());
            param_value = default_value;
            for (size_t i = 0; i < param_value.size(); i++) {
                std::cout << param_value[i] << " ";
            }
            std::cout << std::endl;
            if (required) {
                throw std::invalid_argument(
                        std::string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
            }
            return false;
        }
    }
};

}

#endif