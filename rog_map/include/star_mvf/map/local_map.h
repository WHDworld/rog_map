/**
 * @file local_map.h
 * @brief STAR-MVF 局部地图 - 滑动窗口概率地图
 * 
 * 设计特点：
 * 1. 滑动窗口：使用循环数组（cyclic buffer）实现高效滑动
 * 2. 概率更新：基于log-odds的占据概率
 * 3. 射线投射：高效的传感器模型
 * 
 * 参考项目：ROG-Map (HKU-MaRS Lab)
 */

#ifndef STAR_MVF_LOCAL_MAP_H
#define STAR_MVF_LOCAL_MAP_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <mutex>
#include <memory>
#include <unordered_set>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "star_mvf/map/map_types.h"

namespace star_mvf {

/**
 * @brief 局部地图配置
 */
struct LocalMapConfig {
    // 地图尺寸（半尺寸，单位：米）
    Position local_half_size = Position(10.0f, 10.0f, 3.0f);
    float resolution = 0.1f;
    float resolution_inv = 10.0f;
    
    // 概率更新参数
    float log_odds_hit = 0.85f;
    float log_odds_miss = -0.4f;
    float log_odds_min = -2.0f;
    float log_odds_max = 3.5f;
    float log_odds_occ_thresh = 0.0f;
    
    // 传感器参数
    float lidar_max_range = 20.0f;
    float lidar_min_range = 0.2f;
    int lidar_downsample_filter = 1;  // 点云降采样系数，每N个点取1个，1表示不降采样
    float camera_max_range = 5.0f;
    float camera_min_range = 0.5f;
    float camera_fov_h_deg = 90.0f;
    float camera_fov_v_deg = 60.0f;
    float camera_ray_step = 0.1f;
    float camera_angular_resolution_deg = 1.0f;
    
    // 射线投射
    bool use_raycast = true;
    float raycast_step = 0.1f;
    
    // 地图滑动
    float sliding_thresh = 1.0f;
    bool sliding_enabled = true;
};

/**
 * @brief STAR-MVF 局部地图类 - 基于循环数组的滑动窗口
 * 
 * 核心概念：
 * - 全局索引 (global_idx): 世界坐标对应的整数索引
 * - 局部索引 (local_idx): 循环数组中的索引 [-half_size_i, half_size_i]
 * - 通过取模运算将全局索引映射到局部索引
 * - 地图滑动时只清除超出范围的单元格，保留重叠区域
 */
class LocalMap {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<LocalMap>;
    using ConstPtr = std::shared_ptr<const LocalMap>;
    
    LocalMap();
    ~LocalMap() = default;
    
    /**
     * @brief 初始化
     */
    void init(const LocalMapConfig& config);
    
    /**
     * @brief 地图滑动（参考 ROG-Map 的 mapSliding）
     * @param odom 当前位置
     */
    void mapSliding(const Position& odom);
    
    /**
     * @brief 插入点云
     */
    void insertPointCloud(const sensor_msgs::PointCloud2& cloud, const Position& sensor_pos);
    
    /**
     * @brief 插入清除点云 (仅清除free space, 不标记占据)
     * 用于处理深度图的天空/无穷远点
     */
    void insertClearingCloud(const sensor_msgs::PointCloud2& cloud, const Position& sensor_pos);
    
    /**
     * @brief 获取占据点云
     */
    void getOccupiedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
    
    /**
     * @brief 获取自由点云
     */
    void getFreeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
    
    /**
     * @brief 获取更新的体素（用于同步到全局地图）
     */
    void getUpdatedVoxels(std::vector<VoxelIndex>& occupied,
                          std::vector<VoxelIndex>& free_voxels);
    
    /**
     * @brief 清除更新缓存
     */
    void clearUpdateCache();
    
    /**
     * @brief 检查位置是否在局部地图内
     */
    bool isInLocalMap(const Position& pos) const;
    
    /**
     * @brief 检查全局索引是否在局部地图内
     */
    bool isInLocalMap(const Eigen::Vector3i& global_idx) const;
    
    /**
     * @brief 获取体素占据概率
     */
    float getOccupancy(const Position& pos) const;
    
    /**
     * @brief 检查是否占据
     */
    bool isOccupied(const Position& pos) const;
    
    /**
     * @brief 获取地图中心
     */
    Position getCenter() const { return origin_d_; }

    /**
     * @brief 更新相机可视域
     */
    void updateCameraFrustum(const Position& camera_origin, 
                             const Quaternion& camera_rotation);

    /**
     * @brief 获取已观测的体素
     */
    void getObservedVoxels(std::vector<VoxelIndex>& observed_voxels);

    /**
     * @brief 清除观测缓存
     */
    void clearObservedCache();
    
    // ==================== 可视化接口 ====================
    
    /**
     * @brief 初始化可视化
     */
    void initVisualization(ros::NodeHandle& nh, const std::string& frame_id);
    
    /**
     * @brief 执行所有可视化
     */
    void visualize();
    
    /**
     * @brief 获取地图边界
     */
    Position getMapMin() const { return bound_min_d_; }
    Position getMapMax() const { return bound_max_d_; }

protected:
    LocalMapConfig config_;
    
    // 滑动地图配置
    Eigen::Vector3i half_map_size_i_;   // 半尺寸（体素数）
    Eigen::Vector3i map_size_i_;        // 总尺寸（体素数）
    int map_vox_num_ = 0;               // 体素总数
    
    // 地图原点和边界（世界坐标系）
    Position origin_d_;                  // 当前原点（米）
    Eigen::Vector3i origin_i_;          // 当前原点（体素索引）
    Position bound_min_d_, bound_max_d_; // 边界（米）
    Eigen::Vector3i bound_min_i_, bound_max_i_; // 边界（体素索引）
    
    // 概率存储（log-odds形式）- 循环数组
    std::vector<float> occupancy_buffer_;
    mutable std::mutex map_mutex_;
    
    // 相机参数
    float camera_fov_h_rad_ = 0.0f;
    float camera_fov_v_rad_ = 0.0f;
    float tan_half_fov_h_ = 0.0f;
    float tan_half_fov_v_ = 0.0f;
    float camera_angular_resolution_deg_ = 0.0f;
    
    // 更新缓存（用于同步到全局地图）
    std::unordered_set<int> updated_occupied_;
    std::unordered_set<int> updated_free_;
    std::unordered_set<int> updated_observed_;
    
    bool initialized_ = false;
    
    // ==================== 核心坐标转换函数（参考 ROG-Map） ====================
    
    /**
     * @brief 位置转全局索引
     */
    void posToGlobalIndex(const Position& pos, Eigen::Vector3i& global_idx) const;
    
    /**
     * @brief 全局索引转位置（体素中心）
     */
    void globalIndexToPos(const Eigen::Vector3i& global_idx, Position& pos) const;
    
    /**
     * @brief 全局索引转局部索引（循环映射）
     * @note 这是滑动地图的核心：使用取模运算将全局索引映射到循环数组
     */
    void globalIndexToLocalIndex(const Eigen::Vector3i& global_idx, Eigen::Vector3i& local_idx) const;
    
    /**
     * @brief 局部索引转全局索引
     */
    void localIndexToGlobalIndex(const Eigen::Vector3i& local_idx, Eigen::Vector3i& global_idx) const;
    
    /**
     * @brief 局部索引转哈希值（线性索引）
     */
    int localIndexToHash(const Eigen::Vector3i& local_idx) const;
    
    /**
     * @brief 哈希值转局部索引
     */
    void hashToLocalIndex(int hash_id, Eigen::Vector3i& local_idx) const;
    
    /**
     * @brief 哈希值转全局索引
     */
    void hashToGlobalIndex(int hash_id, Eigen::Vector3i& global_idx) const;
    
    /**
     * @brief 哈希值转位置
     */
    void hashToPos(int hash_id, Position& pos) const;
    
    /**
     * @brief 位置转哈希值
     */
    int posToHash(const Position& pos) const;
    
    /**
     * @brief 全局索引转哈希值
     */
    int globalIndexToHash(const Eigen::Vector3i& global_idx) const;
    
    // ==================== 滑动地图核心函数 ====================
    
    /**
     * @brief 更新原点和边界
     */
    void updateOriginAndBound(const Position& new_origin_d, const Eigen::Vector3i& new_origin_i);
    
    /**
     * @brief 清除超出范围的内存
     * @param clear_ids 需要清除的索引列表
     * @param axis 清除的轴（0=x, 1=y, 2=z）
     */
    void clearMemoryOutOfMap(const std::vector<int>& clear_ids, int axis);
    
    /**
     * @brief 重置单个单元格
     */
    void resetCell(int hash_id);
    
    /**
     * @brief 重置整个地图
     */
    void resetLocalMap();
    
    // ==================== 其他辅助函数 ====================
    
    void raycast(const Position& start, const Position& end);
    void raycast(const Position& start, const Position& end, 
                 const std::unordered_set<int>& occupied_voxels);
    // 射线投射并确认占据：返回 true 表示射线到达终点，false 表示被中途截断
    bool raycastWithConfirm(const Position& start, const Position& end,
                            const std::unordered_set<int>& candidate_occupied,
                            std::unordered_set<int>& confirmed_occupied);
    void updateVoxel(int hash_id, float log_odds_update);
    void computeCameraRayCount(int& num_rays_h, int& num_rays_v) const;
    bool castCameraRay(const Position& origin, const Position& direction, float max_range);
    
    // 可视化相关
    std::string viz_frame_id_ = "map";
    ros::Publisher local_map_bound_pub_;
    ros::Publisher local_map_occ_pub_;
};

}  // namespace star_mvf

#endif  // STAR_MVF_LOCAL_MAP_H
