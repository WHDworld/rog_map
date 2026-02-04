/**
 * @file map_manager.h
 * @brief STAR-MVF 地图管理器 - 统一的地图接口（替代 ROGMap）
 * 
 * 职责：
 * 1. 接收传感器数据并更新局部地图
 * 2. 定期将局部地图同步到全局地图
 * 3. 提供统一的地图查询接口（兼容旧的 ROGMap 接口）
 * 4. 管理可视化发布
 * 5. 支持探索器和规划器的所有功能
 * 
 * 参考：EDEN, FALCON, ROG-Map 的设计
 */

#ifndef STAR_MVF_MAP_MANAGER_H
#define STAR_MVF_MAP_MANAGER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>
#include <memory>
#include <atomic>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
// LocalMap 已移除：射线投射直接在 GlobalMap 中处理
#include "map/global_map.h"

namespace star_mvf {

/**
 * @brief 机器人状态（兼容旧代码）
 */
struct RobotStateData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Position p = Position::Zero();
    Quaternion q = Quaternion::Identity();
    double yaw = 0.0;
    double rcv_time = 0.0;
    bool rcv = false;
};

/**
 * @brief 地图管理器配置
 */
struct MapManagerConfig {
    // 话题
    std::string lidar_topic = "/cloud_registered";
    std::string odom_topic = "/odom";
    std::string camera_pose_topic = "/camera/pose";
    std::string camera_rgb_topic = "/camera/color/image_raw";
    std::string depth_topic = "/camera/depth_image"; // [Added] Depth topic
    std::string frame_id = "map";
    
    // Camera Intrinsics [Added]
    double fx = 320.0;
    double fy = 320.0;
    double cx = 320.0;
    double cy = 240.0;
    int depth_filter_margin = 0;
    int skip_pixel = 2;
    double depth_scale = 1.0; // [Added]
    
    // Extrinsics: Rotation from Camera Frame to Body Frame (FLU)
    // Default: Standard Optical to FLU (Z forward -> X forward, X right -> Y left (-y), Y down -> Z down (-z))
    Eigen::Matrix3f body_R_cam = Eigen::Matrix3f::Identity();
    
    // 更新频率
    float viz_rate = 5.0f;        // 可视化频率
    
    // 可视化配置
    bool visualization_en = true;
    Position visualization_range = Position(20.0f, 20.0f, 6.0f);
    
    // 子模块配置（LocalMapConfig 已移除）
    GlobalMapConfig global_config;
};

/**
 * @brief STAR-MVF 地图管理器
 */
class MapManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<MapManager>;
    using ConstPtr = std::shared_ptr<const MapManager>;
    
    MapManager(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~MapManager() = default;
    
    /**
     * @brief 从ROS参数初始化
     */
    void initFromROS(const std::string& name_space = "star_mvf");
    
    /**
     * @brief 启动地图更新
     */
    void start();
    
    /**
     * @brief 停止地图更新
     */
    void stop();
    
    // ==================== 地图访问接口 ====================
    
    /**
     * @brief 获取全局地图
     */
    GlobalMap::Ptr getGlobalMap() { return global_map_; }
    GlobalMap::ConstPtr getGlobalMap() const { return global_map_; }
    
    // LocalMap 接口已移除：射线投射直接在 GlobalMap 中处理
    
    // ==================== 统一查询接口（委托给GlobalMap） ====================
    
    Position getGlobalMapMin() const { return global_map_->getGlobalMapMin(); }
    Position getGlobalMapMax() const { return global_map_->getGlobalMapMax(); }
    float getResolution() const { return global_map_->getResolution(); }
    
    bool isInGlobalMapRange(const Position& pos) const { return global_map_->isInGlobalMapRange(pos); }
    bool isOccupied(const Position& pos) { return global_map_->isOccupied(pos); }
    bool isFree(const Position& pos) { return global_map_->isFree(pos); }
    bool isUnknown(const Position& pos) { return global_map_->isUnknown(pos); }
    
    VoxelState getVoxelState(const Position& pos) { return global_map_->getVoxelState(pos); }
    void posToIndex(const Position& pos, VoxelIndex& idx) const { global_map_->posToIndex(pos, idx); }
    Position indexToPos(const VoxelIndex& idx) const { return global_map_->indexToPos(idx); }
    
    // 前沿接口
    void extractGlobalFrontierIncremental() { global_map_->extractGlobalFrontierIncremental(); }
    void getFrontierSubGrids(std::vector<int>& ids) const { global_map_->getFrontierSubGrids(ids); }
    void getFrontierPositionsBySubGrid(std::vector<SubGridFrontierInfo>& info) const {
        global_map_->getFrontierPositionsBySubGrid(info);
    }
    void getFrontierPointCloud(sensor_msgs::PointCloud2& cloud) const {
        global_map_->getFrontierPointCloud(cloud);
    }
    size_t getFrontierCount() const { return global_map_->getFrontierCount(); }
    int getTotalFrontierCount() const { return global_map_->getTotalFrontierCount(); }
    
    // 子网格接口
    SubGrid& getSubGrid(int linear_idx) { return global_map_->getSubGrid(linear_idx); }
    const SubGrid& getSubGrid(int linear_idx) const { return global_map_->getSubGrid(linear_idx); }
    
    // 安全性检查
    bool isPathSafe(const Position& start, const Position& end, float margin = 0.0f) const {
        return global_map_->isPathSafe(start, end, margin);
    }
    bool isSphereSafe(const Position& center, float radius) const {
        return global_map_->isSphereSafe(center, radius);
    }
    
    // ==================== 膨胀地图接口（规划器需要）====================
    
    bool isOccupiedInflate(const Position& pos) const { 
        return global_map_->isOccupiedInflate(pos); 
    }
    bool isUnknownInflate(const Position& pos) const { 
        return global_map_->isUnknownInflate(pos); 
    }
    bool isKnownFreeInflate(const Position& pos) const { 
        return global_map_->isKnownFreeInflate(pos); 
    }
    float getInfResolution() const { 
        return global_map_->getInfResolution(); 
    }
    
    // ==================== 射线/路径检测（规划器需要）====================
    /**
     * @brief 检查路径是否自由（使用膨胀地图）
     * @param start 起点
     * @param end 终点
     * @param use_inf_map 是否使用膨胀地图
     * @param use_unk_as_occ 是否将未知视为障碍
     */
    bool isLineFree(const Position& start, const Position& end,
                    bool use_inf_map = false, bool use_unk_as_occ = false) const {
        if (use_inf_map) {
            return global_map_->isLineFreeInflate(start, end, use_unk_as_occ);
        }
        return global_map_->isLineFree(start, end);
    }
    
    /**
     * @brief 检查路径是否自由（带碰撞点输出）
     */
    bool isLineFree(const Position& start, const Position& end,
                    Position& collision_point, double max_dist = 999999.0) const {
        return global_map_->isLineFree(start, end, collision_point, max_dist);
    }
    
    /**
     * @brief 检查路径是否自由（带最大距离）
     */
    bool isLineFree(const Position& start, const Position& end,
                    double max_dist) const {
        return global_map_->isLineFree(start, end, max_dist);
    }
    
    // ==================== 网格类型查询 ====================
    
    GridType getGridType(const Position& pos) const {
        return global_map_->getGridType(pos);
    }
    GridType getInfGridType(const Position& pos) const {
        return global_map_->getInfGridType(pos);
    }
    double getMapValue(const Position& pos) const {
        return global_map_->getMapValue(pos);
    }
    
    // ==================== 包围盒搜索 ====================
    
    void boxSearch(const Position& box_min, const Position& box_max,
                   GridType type, std::vector<Position>& out_points) const {
        global_map_->boxSearch(box_min, box_max, type, out_points);
    }
    void boxSearchInflate(const Position& box_min, const Position& box_max,
                          GridType type, std::vector<Position>& out_points) const {
        global_map_->boxSearchInflate(box_min, box_max, type, out_points);
    }
    void boundBoxByLocalMap(Position& box_min, Position& box_max) const {
        global_map_->boundBoxByLocalMap(box_min, box_max);
    }
    Position getLocalMapOrigin() const {
        return global_map_->getLocalMapOrigin();
    }
    Position getLocalMapSize() const {
        return global_map_->getLocalMapSize();
    }
    
    // ==================== 状态查询 ====================
    
    bool hasOdom() const { return has_odom_; }
    Position getCurrentPosition() const {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        return current_position_;
    }
    
    /**
     * @brief 获取机器人状态（兼容旧代码）
     */
    RobotStateData getRobotState() const {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        return robot_state_;
    }
    
    /**
     * @brief 获取全局地图范围（兼容旧代码）
     */
    Eigen::Vector3f getGlobalMapMinF() const { 
        return config_.global_config.global_min.cast<float>(); 
    }
    Eigen::Vector3f getGlobalMapMaxF() const { 
        return config_.global_config.global_max.cast<float>(); 
    }
    
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    MapManagerConfig config_;
    
    // 子模块（LocalMap 已移除）
    GlobalMap::Ptr global_map_;
    
    // ROS接口
    ros::Subscriber lidar_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber camera_pose_sub_;
    ros::Subscriber camera_rgb_sub_;
    ros::Subscriber depth_sub_; // [Added]
    ros::Timer sync_timer_;
    ros::Timer viz_timer_;
    
    // 状态
    std::atomic<bool> running_{false};
    std::atomic<bool> has_odom_{false};
    Position current_position_;
    Quaternion current_rotation_;
    RobotStateData robot_state_;
    mutable std::mutex odom_mutex_;
    
    Position current_camera_position_;
    Quaternion current_camera_rotation_;
    mutable std::mutex camera_mutex_;
    bool has_camera_pose_ = false;
    
    // 可视化由 LocalMap 和 GlobalMap 各自管理
    
    // 回调函数
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void cameraOdomCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void cameraRgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg); // [Added]
    void vizTimerCallback(const ros::TimerEvent& event);
    
    // syncLocalToGlobal 已移除：射线投射直接在 GlobalMap 中处理
    
};

}  // namespace star_mvf

#endif  // STAR_MVF_MAP_MANAGER_H

