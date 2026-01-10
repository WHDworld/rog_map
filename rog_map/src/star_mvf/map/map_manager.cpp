/**
 * @file map_manager.cpp
 * @brief STAR-MVF 地图管理器实现
 */

#include "star_mvf/map/map_manager.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace star_mvf {

MapManager::MapManager(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
    local_map_ = std::make_shared<LocalMap>();
    // GlobalMap 稍后在 initFromROS 中初始化
}

void MapManager::initFromROS(const std::string& name_space) {
    // 读取话题配置
    nh_private_.param<std::string>(name_space + "/lidar_topic", config_.lidar_topic, "/cloud_registered");
    nh_private_.param<std::string>(name_space + "/odom_topic", config_.odom_topic, "/odom");
    nh_private_.param<std::string>(name_space + "/camera_pose_topic", config_.camera_pose_topic, "/camera/pose");
    nh_private_.param<std::string>(name_space + "/camera_rgb_topic", config_.camera_rgb_topic, "/camera/color/image_raw");
    nh_private_.param<std::string>(name_space + "/frame_id", config_.frame_id, "map");
    
    // 读取更新频率
    double viz_rate;
    nh_private_.param(name_space + "/viz_rate", viz_rate, 5.0);
    config_.viz_rate = static_cast<float>(viz_rate);
    
    // 读取可视化配置
    nh_private_.param(name_space + "/visualization_en", config_.visualization_en, true);
    std::vector<float> viz_range;
    nh_private_.param(name_space + "/visualization_range", viz_range, std::vector<float>{20.0f, 20.0f, 6.0f});
    if (viz_range.size() == 3) {
        config_.visualization_range = Position(viz_range[0], viz_range[1], viz_range[2]);
    }
    
    // 读取分辨率
    float resolution;
    nh_private_.param(name_space + "/resolution", resolution, 0.1f);
    config_.local_config.resolution = resolution;
    config_.local_config.resolution_inv = 1.0f / resolution;
    config_.global_config.resolution = resolution;
    config_.global_config.resolution_inv = 1.0f / resolution;
    
    // 读取局部地图配置（半尺寸）
    std::vector<float> local_half_size;
    nh_private_.param(name_space + "/local_map/half_size", local_half_size, std::vector<float>{10.0f, 10.0f, 3.0f});
    if (local_half_size.size() == 3) {
        config_.local_config.local_half_size = Position(local_half_size[0], local_half_size[1], local_half_size[2]);
    }
    
    double log_hit, log_miss, log_min, log_max, occ_thresh;
    nh_private_.param(name_space + "/local_map/log_odds_hit", log_hit, 0.85);
    nh_private_.param(name_space + "/local_map/log_odds_miss", log_miss, -0.4);
    nh_private_.param(name_space + "/local_map/log_odds_min", log_min, -2.0);
    nh_private_.param(name_space + "/local_map/log_odds_max", log_max, 3.5);
    nh_private_.param(name_space + "/local_map/log_odds_occ_thresh", occ_thresh, 0.0);
    config_.local_config.log_odds_hit = static_cast<float>(log_hit);
    config_.local_config.log_odds_miss = static_cast<float>(log_miss);
    config_.local_config.log_odds_min = static_cast<float>(log_min);
    config_.local_config.log_odds_max = static_cast<float>(log_max);
    config_.local_config.log_odds_occ_thresh = static_cast<float>(occ_thresh);

    bool use_raycast;
    double raycast_step;
    nh_private_.param(name_space + "/local_map/use_raycast", use_raycast, true);
    nh_private_.param(name_space + "/local_map/raycast_step", raycast_step, 0.1);
    config_.local_config.use_raycast = use_raycast;
    config_.local_config.raycast_step = static_cast<float>(raycast_step);

    double lidar_sensor_max, lidar_sensor_min;
    double camera_sensor_max, camera_sensor_min;
    nh_private_.param(name_space + "/sensor/lidar_max_range", lidar_sensor_max, 20.0);
    nh_private_.param(name_space + "/sensor/lidar_min_range", lidar_sensor_min, 0.2);
    int lidar_downsample_filter;
    nh_private_.param(name_space + "/sensor/lidar_downsample_filter", lidar_downsample_filter, 1);

    nh_private_.param(name_space + "/sensor/camera_max_range", camera_sensor_max, 5.0);
    nh_private_.param(name_space + "/sensor/camera_min_range", camera_sensor_min, 0.5);
    double camera_fov_h_deg, camera_fov_v_deg;
    nh_private_.param(name_space + "/sensor/camera_fov_h_deg", camera_fov_h_deg, 90.0);
    nh_private_.param(name_space + "/sensor/camera_fov_v_deg", camera_fov_v_deg, 60.0);
    double camera_ray_step, camera_angular_resolution_deg;
    nh_private_.param(name_space + "/sensor/camera_ray_step", camera_ray_step, 0.1);
    nh_private_.param(name_space + "/sensor/camera_angular_resolution_deg", camera_angular_resolution_deg, 1.0);
    config_.local_config.lidar_max_range = static_cast<float>(lidar_sensor_max);
    config_.local_config.lidar_min_range = static_cast<float>(lidar_sensor_min);
    config_.local_config.lidar_downsample_filter = lidar_downsample_filter;
    config_.local_config.camera_max_range = static_cast<float>(camera_sensor_max);
    config_.local_config.camera_min_range = static_cast<float>(camera_sensor_min);
    config_.local_config.camera_fov_h_deg = static_cast<float>(camera_fov_h_deg);
    config_.local_config.camera_fov_v_deg = static_cast<float>(camera_fov_v_deg);
    config_.local_config.camera_ray_step = static_cast<float>(raycast_step);
    config_.local_config.camera_angular_resolution_deg = static_cast<float>(camera_angular_resolution_deg);

    
    // 初始化局部地图
    local_map_->init(config_.local_config);
    
    // 初始化全局地图（使用私有NodeHandle确保话题带节点前缀）
    global_map_ = std::make_shared<GlobalMap>(nh_private_, name_space);
    
    // 设置订阅器
    lidar_sub_ = nh_.subscribe(config_.lidar_topic, 10, &MapManager::lidarCallback, this);
    odom_sub_ = nh_.subscribe(config_.odom_topic, 100, &MapManager::odomCallback, this);
    camera_pose_sub_ = nh_.subscribe(config_.camera_pose_topic, 10, &MapManager::cameraOdomCallback, this);
    camera_rgb_sub_ = nh_.subscribe(config_.camera_rgb_topic, 10, &MapManager::cameraRgbCallback, this);
    
    // 初始化局部地图可视化（由 LocalMap 自己管理发布器）
    if (config_.visualization_en) {
        local_map_->initVisualization(nh_private_, config_.frame_id);
    }
    
    ROS_INFO("[MapManager] Initialized with lidar_topic=%s, odom_topic=%s",
             config_.lidar_topic.c_str(), config_.odom_topic.c_str());
}

void MapManager::start() {
    if (running_) return;
    running_ = true;
    ROS_INFO("[MapManager] Creating viz_timer with rate=%.1f Hz", config_.viz_rate);
    viz_timer_ = nh_.createTimer(
        ros::Duration(1.0 / config_.viz_rate),
        &MapManager::vizTimerCallback, this);
    ROS_INFO("[MapManager] Started (new architecture: sync in lidarCallback)");
}

void MapManager::stop() {
    if (!running_) return;
    running_ = false;
    
    // sync_timer_.stop();  // 新架构：不再使用 syncTimer
    viz_timer_.stop();
    
    ROS_INFO("[MapManager] Stopped");
}

void MapManager::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    static int lidar_count = 0;
    lidar_count++;
    
    if (!running_) return;
    if (!has_odom_) return;
    
    Position sensor_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        sensor_pos = current_position_;
    }
    
    // === 新架构：在lidarCallback中完成所有地图更新 ===
    
    // Step 1: 更新局部地图
    auto t1 = ros::Time::now();
    local_map_->insertPointCloud(*msg, sensor_pos);
    auto t2 = ros::Time::now();
    
    // Step 2: 同步到全局地图
    syncLocalToGlobal();
    auto t3 = ros::Time::now();
    
    // Step 3: 提取前沿
    global_map_->extractGlobalFrontierIncremental();
    auto t4 = ros::Time::now();
    
    // 性能日志（每10帧输出一次）
    // if (lidar_count % 10 == 0) {
    //     ROS_INFO("[MapManager] LiDAR #%d: insert=%.1fms, sync=%.1fms, frontier=%.1fms",
    //              lidar_count,
    //              (t2 - t1).toSec() * 1000.0,
    //              (t3 - t2).toSec() * 1000.0,
    //              (t4 - t3).toSec() * 1000.0);
    // }
}

void MapManager::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    static int odom_count = 0;
    odom_count++;
    
    // 第一次打印
    if (odom_count == 1) {
        ROS_INFO("[MapManager] First odom received!");
    }
    
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    current_position_.x() = static_cast<float>(msg->pose.pose.position.x);
    current_position_.y() = static_cast<float>(msg->pose.pose.position.y);
    current_position_.z() = static_cast<float>(msg->pose.pose.position.z);
    current_rotation_ = Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    has_odom_ = true;
    
    // 更新机器人状态（兼容旧代码）
    robot_state_.p = current_position_;
    robot_state_.q = current_rotation_;
    robot_state_.rcv_time = ros::Time::now().toSec();
    robot_state_.rcv = true;
    
    // 从四元数提取yaw角
    Eigen::Matrix3f rot_mat = current_rotation_.toRotationMatrix();
    robot_state_.yaw = std::atan2(rot_mat(1, 0), rot_mat(0, 0));
    
    // 地图滑动（使用循环数组，只清除超出范围的体素）
    local_map_->mapSliding(current_position_);
    
    // 如果启用相机地图，更新全局地图的相机位姿
    if (global_map_->enable_camera_map_) {
        global_map_->current_cam_origin_ = current_position_;
        global_map_->current_cam_rot_ = current_rotation_;
    }
}

// 添加相机位姿回调函数（在 odomCallback 后添加，第138行后）
void MapManager::cameraOdomCallback(const nav_msgs::OdometryConstPtr& msg) {    
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    // 提取相机位姿
    current_camera_position_.x() = static_cast<float>(msg->pose.pose.position.x);
    current_camera_position_.y() = static_cast<float>(msg->pose.pose.position.y);
    current_camera_position_.z() = static_cast<float>(msg->pose.pose.position.z);
    
    current_camera_rotation_ = Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    has_camera_pose_ = true;
    
    // 更新局部地图的可视域
    local_map_->updateCameraFrustum(current_camera_position_, current_camera_rotation_);
}

void MapManager::cameraRgbCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 相机RGB图像回调（预留用于语义分割等）
    // 目前不做处理
}

void MapManager::vizTimerCallback(const ros::TimerEvent& event) {
    static int viz_count = 0;
    viz_count++;
    
    if (!has_odom_ || !config_.visualization_en) return;
    
    auto t_start = ros::Time::now();
    
    // 局部地图可视化（由 LocalMap 管理，内部检查订阅者）
    local_map_->visualize();
    
    // 全局地图可视化（由 GlobalMap 管理，内部检查订阅者）
    global_map_->visualize();
    
    auto t_end = ros::Time::now();
    
    // ROS_INFO_THROTTLE(5.0, "[MapManager] vizTimer #%d: global_voxels=%zu, time=%.1fms", 
    //          viz_count, global_map_->getVoxelCount(), (t_end - t_start).toSec() * 1000.0);
}

void MapManager::syncLocalToGlobal() {
    // 获取局部地图更新
    std::vector<VoxelIndex> occupied, free_voxels;
    local_map_->getUpdatedVoxels(occupied, free_voxels);

    // 获取相机观测的体素（新增）
    std::vector<VoxelIndex> observed_voxels;
    if (has_camera_pose_) {
        local_map_->getObservedVoxels(observed_voxels);
    }
    
    if (occupied.empty() && free_voxels.empty()) return;
    
    // 批量更新全局地图
    global_map_->updateOccupiedVoxels(occupied, free_voxels);
    
    // 批量更新全局地图的语义状态（VISITED）（新增）
    if (!observed_voxels.empty()) {
        for (const auto& idx : observed_voxels) {
            global_map_->updateVoxelStateFromCamera(idx, true);
        }
    }
    // 清除局部地图缓存
    local_map_->clearUpdateCache();
}

}  // namespace star_mvf

