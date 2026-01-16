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

#include "star_mvf/map/map_types.h"

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
    
    // 前沿参数
    float frontier_height_min = 0.9f;
    float frontier_height_max = 2.3f;
    
    // 话题名称
    std::string frame_id = "map";
    
    // 膨胀地图配置
    InflationConfig inflation_config;
    
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
    
    /**
     * @brief 检查位置是否在地图范围内
     */
    bool isInGlobalMapRange(const Position& pos) const;
    bool isInGlobalMapRange(const VoxelIndex& idx) const;
    
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
    bool isFrontier(const Position& pos);
    bool isFrontier(const VoxelIndex& idx);
    
    // ==================== 状态更新接口 ====================
    
    /**
     * @brief 从LiDAR更新体素状态
     */
    void updateVoxelStateFromLiDAR(const VoxelIndex& idx, bool is_occupied);
    
    /**
     * @brief 从相机更新视觉状态
     */
    void updateVoxelStateFromCamera(const VoxelIndex& idx, bool is_visited);
    
    /**
     * @brief 批量更新占据状态
     */
    void updateOccupiedVoxels(const std::vector<VoxelIndex>& occupied_indices,
                              const std::vector<VoxelIndex>& free_indices);
    
    // ==================== 子网格接口 ====================
    
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
    
    /**
     * @brief 标记子网格脏
     */
    void markSubGridDirty(int linear_idx);
    
    /**
     * @brief 获取4邻域子网格
     */
    void getNeighborSubGrids4(const SubGridIndex& idx, std::vector<SubGridIndex>& neighbors);
    
    // ==================== 前沿接口 ====================
    
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
     * @brief 获取所有前沿点位置
     */
    void getFrontierPositions(std::vector<Position>& positions) const;
    
    /**
     * @brief 获取按子网格分组的前沿点
     */
    void getFrontierPositionsBySubGrid(std::vector<SubGridFrontierInfo>& subgrid_frontiers) const;
    
    /**
    * @brief 获取指定子网格内的所有前沿点 (用于 HistoryGraph 局部查询)
    * @param sg_id 子网格ID
    * @param out_points 输出的前沿点列表
    * @return 如果子网格存在且有前沿点返回 true，否则返回 false
    */
    bool getSubGridFrontierPoints(int sg_id, std::vector<Eigen::Vector3f>& out_points) const;    /**
    
    * @brief 获取前沿点云
     */
    void getFrontierPointCloud(sensor_msgs::PointCloud2& pc_frontier) const;
    
    /**
     * @brief 获取前沿数量
     */
    size_t getFrontierCount() const;
    int getTotalFrontierCount() const { return static_cast<int>(getFrontierCount()); }
    size_t getVoxelCount() const { return voxels_.size(); }
    
    // ==================== 安全性检查 ====================
    
    /**
     * @brief 检查路径是否安全
     */
    bool isPathSafe(const Position& start, const Position& end, float safety_margin = 0.0f) const;
    
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
    
    /**
     * @brief 检查位置是否在膨胀后自由
     */
    bool isKnownFreeInflate(const Position& pos) const;
    
    /**
     * @brief 获取膨胀分辨率
     */
    float getInfResolution() const { return config_.inflation_config.inflation_resolution; }
    
    // ==================== ESDF接口 ====================
    
    /**
     * @brief 获取ESDF距离
     */
    double getESDFDistance(const Position& pos) const;
    
    /**
     * @brief 获取ESDF梯度
     */
    void getESDFGradient(const Position& pos, Position& gradient) const;
    
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
    
    // ==================== 网格类型查询（兼容旧代码）====================
    
    /**
     * @brief 获取网格类型
     */
    GridType getGridType(const Position& pos) const;
    GridType getGridType(const VoxelIndex& idx) const;
    GridType getInfGridType(const Position& pos) const;
    
    /**
     * @brief 获取概率值（占据概率）
     */
    double getMapValue(const Position& pos) const;
    
    // ==================== 包围盒搜索 ====================
    
    /**
     * @brief 在包围盒内搜索特定类型的点
     */
    void boxSearch(const Position& box_min, const Position& box_max,
                   GridType type, std::vector<Position>& out_points) const;
    
    /**
     * @brief 在包围盒内搜索特定类型的点（膨胀版本）
     */
    void boxSearchInflate(const Position& box_min, const Position& box_max,
                          GridType type, std::vector<Position>& out_points) const;
    
    /**
     * @brief 限制包围盒到局部地图范围
     */
    void boundBoxByLocalMap(Position& box_min, Position& box_max) const;
    
    /**
     * @brief 获取局部地图原点和尺寸
     */
    Position getLocalMapOrigin() const;
    Position getLocalMapSize() const;
    
    // ==================== 可视化 ====================
    
    /**
     * @brief 执行所有可视化（统一入口，内部检查订阅者）
     */
    void visualize();
    
    /**
     * @brief 单独的可视化方法（内部会检查订阅者）
     */
    void visualizeGlobalMap();          // 占据+空闲+前沿点云
    void visualizeGlobalMapRange();     // 全局地图范围框
    void visualizeSubGridRange();       // 子网格范围
    void visualizeVisitedVoxels();      // 已访问体素
    void visualizeCameraFrustum();      // 相机视锥体
    
    /**
     * @brief 获取已知体素点云
     */
    void getAllKnownPointCloud(sensor_msgs::PointCloud2& pc_all_known) const;
    
    // ==================== 相机视锥相关 ====================
    
    /**
     * @brief 检查点是否在相机视锥内
     */
    bool isPointInFrustum(const VoxelIndex& idx) const;
    bool isPointInFrustum(const Position& pos) const;
    
    /**
     * @brief 更新相机位姿
     */
    void updateCameraPose(const Position& origin, const Quaternion& rotation);
    
    // 相机状态（公开访问以兼容旧代码）
    Position current_cam_origin_ = Position::Zero();
    Quaternion current_cam_rot_ = Quaternion::Identity();
    bool enable_camera_map_ = true;
    
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
    std::unordered_set<VoxelIndex, VoxelIndexHash> frontier_voxels_;
    std::unordered_set<int> current_frontier_subgrids_;
    std::unordered_set<int> dirty_subgrids_;
    
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

