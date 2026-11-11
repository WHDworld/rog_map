/**
 * @file global_map.h
 * @brief STAR-MVF 全局地图 - 统一的稀疏体素地图实现
 * 
 * 设计特点：
 * 1. 稀疏哈希存储：只存储已观测体素
 * 2. 子网格管理：支持高效的前沿聚类
 * 3. 增量前沿提取：仅更新变化区域
 * 4. 多状态融合：几何+视觉+语义
 * 
 * 参考项目：EDEN, FALCON, ROG-Map
 */

#ifndef STAR_MVF_GLOBAL_MAP_H
#define STAR_MVF_GLOBAL_MAP_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <memory>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "map/map_types.h"

namespace star_mvf {

/**
 * @brief 全局地图配置
 */
struct GlobalMapConfig {
    // 地图范围
    Position global_min = Position(-50, -50, -2);
    Position global_max = Position(50, 50, 5);
    float resolution = 0.1f;
    float resolution_inv = 10.0f;
    
    // 子网格配置
    float sub_grid_xy = 0.1f;  // 子网格占全局地图的比例
    
    // 相机参数（用于视觉状态更新）
    float fov_h_deg = 90.0f;
    float fov_v_deg = 60.0f;
    float camera_max_dist = 5.0f;
    float camera_min_dist = 0.3f;
    float camera_ray_step = 0.1f;          // 射线步进距离
    float camera_angular_res_deg = 3.0f;   // 角分辨率（度）
    
    // 前沿参数
    float frontier_height_min = 0.2f;
    float frontier_height_max = 2.3f;
    float ground_height = 0.0f;  // 地面高度，低于此高度+0.2m的体素不参与语义分析
    
    // 话题名称
    std::string frame_id = "map";
    
    // 膨胀地图配置
    InflationConfig inflation_config;
    
    // Log-Odds 概率更新参数（参考 FUEL sdf_map）
    float p_hit = 0.70f;       // 命中概率
    float p_miss = 0.35f;      // 未命中概率
    float p_min = 0.12f;       // 概率下限
    float p_max = 0.97f;       // 概率上限
    float p_occ = 0.80f;       // 占据判定阈值
    
    // ESDF配置
    ESDFConfig esdf_config;
};

/**
 * @brief STAR-MVF 全局地图类
 */
class GlobalMap {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<GlobalMap>;
    using ConstPtr = std::shared_ptr<const GlobalMap>;
    
    /**
     * @brief 构造函数
     */
    GlobalMap(const ros::NodeHandle& nh, const std::string& name_space = "star_mvf");
    ~GlobalMap() = default;
    
    // ==================== 初始化 ====================
    
    /**
     * @brief 从ROS参数初始化
     */
    void initFromROS(const std::string& name_space);
    
    /**
     * @brief 初始化子网格
     */
    void initializeSubGrids();
    
    // ==================== 基础查询接口 ====================
    
    /**
     * @brief 获取地图范围
     */
    Position getGlobalMapMin() const { return config_.global_min; }
    Position getGlobalMapMax() const { return config_.global_max; }
    Position getGlobalMin() const { return config_.global_min; }
    Position getGlobalMax() const { return config_.global_max; }
    float getResolution() const { return config_.resolution; }
    size_t getTotalVoxelCount() const;
    bool voxelToLinearAddress(const VoxelIndex& idx, size_t& address) const;
    bool linearAddressToVoxel(size_t address, VoxelIndex& idx) const;
    
    // 子网格信息 (SemanticTargetExtractor 需要)
    float getSubGridSize() const { return sub_grid_x_; }  // 子网格世界尺寸
    int getSubGridNumX() const { return sub_grid_num_x_; }
    int getSubGridNumY() const { return sub_grid_num_y_; }
    
    // 语义目标分析索引接口
    const std::unordered_set<int>& getReadyForAnalysisSubGrids() const { 
        return ready_for_analysis_subgrids_; 
    }
    void markSubGridAnalyzed(int linear_idx);
    void setSemanticExplorationThreshold(float exploration_threshold)
    {
        semantic_exploration_threshold_ = exploration_threshold;
    }
    /**
     * @brief 检查位置是否在地图范围内
     */
    bool isInGlobalMapRange(const Position& pos) const;
    bool isInGlobalMapRange(const VoxelIndex& idx) const;
    
    /**
     * @brief 将超出范围的点裁剪到地图边界（参考 FUEL closetPointInMap）
     */
    Position clipToMapBoundary(const Position& pt, const Position& sensor_pos) const;
    
    /**
     * @brief 位置与索引转换
     */
    void posToIndex(const Position& pos, VoxelIndex& idx) const;
    VoxelIndex posToIndex(const Position& pos) const;
    void indexToPos(const VoxelIndex& idx, Position& pos) const;
    Position indexToPos(const VoxelIndex& idx) const;
    
    /**
     * @brief 获取体素状态
     */
    VoxelState getVoxelState(const Position& pos);
    VoxelState getVoxelState(const VoxelIndex& idx);
    
    /**
     * @brief 状态查询
     */
    bool isOccupied(const Position& pos);
    bool isOccupied(const VoxelIndex& idx);
    bool isFree(const Position& pos);
    bool isFree(const VoxelIndex& idx);
    bool isUnknown(const Position& pos);
    bool isUnknown(const VoxelIndex& idx);
        
    /**
     * @brief 从相机更新视觉状态
     */
    void updateVoxelStateFromCamera(const VoxelIndex& idx, bool is_visited);

    /**
     * @brief 直接设置体素 VISITED 状态 (输入为 Grid Index)
     */
    void setVoxelVisitedDirect(const VoxelIndex& idx, bool is_visited);
        
    // ==================== LiDAR 点云处理（从 LocalMap 移植）====================
    
    /**
     * @brief 插入点云并更新地图（直接射线投射）
     * @param cloud LiDAR 点云
     * @param sensor_pos 传感器位置
     */
    void insertPointCloud(const sensor_msgs::PointCloud2& cloud, const Position& sensor_pos);
            
    /**
     * @brief 位置到子网格转换
     */
    void posToSubGrid(const Position& pos, SubGridIndex& sg_idx) const;
    void voxelToSubGrid(const VoxelIndex& voxel_idx, SubGridIndex& sg_idx) const;
    
    /**
     * @brief 获取子网格
     */
    SubGrid& getSubGrid(const SubGridIndex& idx);
    SubGrid& getSubGrid(int linear_idx);
    const SubGrid& getSubGrid(const SubGridIndex& idx) const;
    const SubGrid& getSubGrid(int linear_idx) const;
    int subGridToLinearIndex(const SubGridIndex& idx) const;
        
    // ==================== 前沿接口 ====================

    // ==================== 增量更新接口 ====================
    /**
     * @brief 获取并清空前沿变化的子网格列表（专供 ViewpointManager）
     * @param out_changed_ids 输出：前沿发生变化的子网格 ID 列表
     * @return 变化的子网格数量
     * 
     * 注意：此接口与 getAndClearDirtySubGrids 不同
     * - dirty_subgrids_ 在每帧 extractGlobalFrontierIncremental() 结束时被清空
     * - frontier_changed_subgrids_ 由 ViewpointManager 调用此函数时清空
     */
    int getAndClearFrontierChangedSubGrids(std::vector<int>& out_changed_ids);
    
    /**
     * @brief 增量式前沿提取
     */
    void extractGlobalFrontierIncremental();
    
    /**
     * @brief 更新前沿子网格缓存
     */
    void updateFrontierSubGridCache();
    
    /**
     * @brief 获取前沿子网格ID列表
     */
    void getFrontierSubGrids(std::vector<int>& frontier_subgrid_ids) const;
            
    /**
    * @brief 获取指定子网格内的所有前沿点（供全局前沿聚类使用）
    * @param sg_id 子网格ID
    * @param out_points 输出的前沿点列表
    * @return 如果子网格存在且有前沿点返回 true，否则返回 false
    */
    bool getSubGridFrontierPoints(int sg_id, std::vector<Eigen::Vector3f>& out_points) const;    /**
    
    // ==================== 安全性检查 ====================
    
    /**
     * @brief 检查路径是否安全
     */
    bool isPathSafe(const Position& start, const Position& end, float safety_margin = 0.0f,
                    bool unknown_as_occupied = false) const;
    
    /**
     * @brief 检查球形区域是否安全
     */
    bool isSphereSafe(const Position& center, float radius) const;
    
    // ==================== 膨胀地图接口（规划器需要）====================
    
    /**
     * @brief 检查位置是否在膨胀后被占据
     */
    bool isOccupiedInflate(const Position& pos) const;
    bool isOccupiedInflate(const VoxelIndex& idx) const;
    
    /**
     * @brief 检查位置是否在膨胀后未知
     */
    bool isUnknownInflate(const Position& pos) const;
        
    // ==================== ESDF接口 ====================
    
    /**
     * @brief 获取ESDF距离
     */
    double getESDFDistance(const Position& pos) const;
        
    /**
     * @brief 检查ESDF是否启用
     */
    bool isESDFEnabled() const { return config_.esdf_config.enable_esdf; }
    
    /**
     * @brief 更新ESDF（局部更新）
     */
    void updateESDF(const Position& center);
    
    // ==================== 射线/路径检测 ====================
    
    /**
     * @brief 检查路径是否自由（无障碍）
     */
    bool isLineFree(const Position& start, const Position& end, 
                    double max_dist = 999999.0) const;
    
    /**
     * @brief 检查路径是否自由，返回碰撞点
     */
    bool isLineFree(const Position& start, const Position& end,
                    Position& collision_point, double max_dist = 999999.0) const;
    
    /**
     * @brief 检查路径是否自由（使用膨胀地图）
     */
    bool isLineFreeInflate(const Position& start, const Position& end, 
                           bool use_unknown_as_occ = false) const;
    
    // ==================== 可视化 ====================
    
    /**
     * @brief 执行所有可视化（统一入口，内部检查订阅者）
     */
    void visualize();
    
    /**
     * @brief 单独的可视化方法（内部会检查订阅者）
     */
    void visualizeGlobalMap();          // 占据+空闲+已访问+前沿点云（单次遍历优化）
    void visualizeGlobalMapRange();     // 全局地图范围框
    void visualizeSubGridRange();       // 子网格范围
    void visualizeCameraFrustum();      // 相机视锥体

    /**
     * @brief 更新相机位姿
     */
    void updateCameraPose(const Position& origin, const Quaternion& rotation);
    
    // 相机状态（公开访问以兼容旧代码）
    Position current_cam_origin_ = Position::Zero();
    Quaternion current_cam_rot_ = Quaternion::Identity();
    bool enable_camera_map_ = true;
    
    float getGroundHeight() const { return config_.ground_height; }

    /** Log logical sizes, capacities and a conservative container-memory estimate. */
    void logMemoryStats(const std::string& tag = "periodic") const;
protected:
    // 配置
    GlobalMapConfig config_;
    ros::NodeHandle nh_;
    
    // 体素存储（稀疏哈希）
    std::unordered_map<VoxelIndex, VoxelState, VoxelIndexHash> voxels_;
    mutable std::mutex map_mutex_;
    
    // 子网格
    std::vector<SubGrid> sub_grids_;
    int sub_grid_num_x_ = 0;
    int sub_grid_num_y_ = 0;
    int sub_grid_size_x_ = 0;  // 每个子网格的体素数
    int sub_grid_size_y_ = 0;
    float sub_grid_x_ = 0.0f;  // 子网格世界尺寸
    float sub_grid_y_ = 0.0f;
    
    // 前沿缓存
    std::unordered_set<int> current_frontier_subgrids_;
    std::unordered_set<int> dirty_subgrids_;
    
    // [新增] 前沿变化子网格（专供 ViewpointManager 使用）
    // 在 extractGlobalFrontierIncremental() 中记录哪些子网格的前沿发生了变化
    // 与 dirty_subgrids_ 不同，此集合由 ViewpointManager 调用 getAndClearFrontierChangedSubGrids() 时清空
    std::unordered_set<int> frontier_changed_subgrids_;
    
    // 语义目标提取索引 (探索率达标但尚未分析的子网格)
    std::unordered_set<int> ready_for_analysis_subgrids_;
    float semantic_exploration_threshold_ = 0.8f;  // 探索率阈值
    
    // 地图索引范围
    VoxelIndex global_min_idx_;
    VoxelIndex global_max_idx_;
    
    // 相机参数
    float fov_h_rad_ = 0.0f;
    float fov_v_rad_ = 0.0f;
    float tan_half_fov_h_ = 0.0f;
    float tan_half_fov_v_ = 0.0f;
    
    // 膨胀地图数据
    std::unordered_map<VoxelIndex, int16_t, VoxelIndexHash> inflation_count_;  // 每个体素的膨胀计数
    std::vector<VoxelIndex> inflation_neighbor_offsets_;  // 膨胀邻域偏移
    mutable std::mutex inflation_mutex_;
    
    // ESDF数据
    std::unordered_map<VoxelIndex, double, VoxelIndexHash> esdf_distance_;
    Position esdf_center_;  // ESDF更新中心
    mutable std::mutex esdf_mutex_;
    
    // 局部地图范围（用于概率更新）
    Position local_map_origin_;
    Position local_map_size_ = Position(20.0f, 20.0f, 6.0f);
    
    // LiDAR 射线投射参数（从 LocalMap 移植）
    float raycast_step_ = 0.15f;
    int lidar_downsample_filter_ = 3;
    
    // 机器人状态
    RobotState robot_state_;
    
    // 可视化发布器
    ros::Publisher global_map_occupied_pub_;
    ros::Publisher global_map_free_pub_;
    ros::Publisher global_map_frontier_pub_;
    ros::Publisher global_map_range_pub_;
    ros::Publisher global_map_voxels_pub_;
    ros::Publisher visited_voxels_pub_;
    ros::Publisher sub_grid_range_pub_;
    ros::Publisher camera_frustum_pub_;
    ros::Publisher inflation_pub_;
    ros::Publisher esdf_pub_;
    
    // ==================== Log-Odds 概率更新 ====================
    std::unordered_map<VoxelIndex, float, VoxelIndexHash> occupancy_buffer_;  // Log-Odds 概率缓冲
    float prob_hit_log_ = 0.0f;      // logit(p_hit)
    float prob_miss_log_ = 0.0f;     // logit(p_miss)
    float clamp_min_log_ = 0.0f;     // logit(p_min)
    float clamp_max_log_ = 0.0f;     // logit(p_max)
    float min_occupancy_log_ = 0.0f; // logit(p_occ)
    float unknown_flag_ = 0.01f;     // 未知标志
    
    // 帧内缓存（避免重复处理）
    uint32_t raycast_frame_id_ = 0;  // 当前帧 generation ID
    std::vector<uint32_t> ray_endpoint_stamp_;  // FUEL-style dense endpoint dedup flags
    std::vector<uint32_t> frame_voxel_stamp_;   // touched-voxel generation flags
    std::vector<int8_t> frame_voxel_state_;     // hit/miss for current generation
    std::vector<VoxelIndex> frame_touched_voxels_;
    
    /**
     * @brief 设置帧内缓存占据状态
     */
    void setCacheOccupancy(const VoxelIndex& idx, int8_t occ);
    
    /**
     * @brief 射线投射标记 miss（沿途所有体素）
     */
    void raycastAndMarkMiss(const Position& start, const Position& end, const VoxelIndex& end_idx);
    
    /**
     * @brief 应用帧内缓存到概率缓冲区
     */
    void applyFrameCache();
    
    /**
     * @brief 状态变化时更新子网格统计
     */
    void updateSubGridOnStateChange(const VoxelIndex& idx, VoxelState old_geo, VoxelState new_geo);
    
    // 辅助函数
    bool isInGlobalRange(const VoxelIndex& idx, 
                         const VoxelIndex& min_idx, 
                         const VoxelIndex& max_idx) const;
    
    /**
     * @brief 初始化膨胀邻域偏移
     */
    void initInflationOffsets();
    
    /**
     * @brief 更新膨胀计数
     */
    void updateInflation(const VoxelIndex& idx, bool is_occupied);
    
    /**
     * @brief 更新单个体素的ESDF
     */
    void updateESDFVoxel(const VoxelIndex& idx);
};

}  // namespace star_mvf

#endif  // STAR_MVF_GLOBAL_MAP_H
