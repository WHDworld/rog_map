/**
 * @file map_manager.cpp
 * @brief STAR-MVF 地图管理器实现
 */

#include "map/map_manager.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

    // [Added] Load Intrinsics
    nh_private_.param(name_space + "/sensor/camera_intrinsics/fx", config_.fx, 320.0);
    nh_private_.param(name_space + "/sensor/camera_intrinsics/fy", config_.fy, 320.0);
    nh_private_.param(name_space + "/sensor/camera_intrinsics/cx", config_.cx, 320.0);
    nh_private_.param(name_space + "/sensor/camera_intrinsics/cy", config_.cy, 240.0);
    nh_private_.param(name_space + "/sensor/depth_filter_margin", config_.depth_filter_margin, 0);
    nh_private_.param(name_space + "/sensor/skip_pixel", config_.skip_pixel, 8);
    nh_private_.param(name_space + "/sensor/depth_scale", config_.depth_scale, 1.0); // [Added]
    nh_private_.param<std::string>(name_space + "/depth_topic", config_.depth_topic, "/uav_simulator/depth_image");

    // Load Extrinsics (Rotation Matrix)
    std::vector<float> rot_vec;
    // Default to Standard Optical -> FLU: [0 0 1; -1 0 0; 0 -1 0]
    std::vector<float> default_rot = {0.0, 0.0, 1.0, 
                                     -1.0, 0.0, 0.0, 
                                      0.0, -1.0, 0.0};
    nh_private_.param(name_space + "/sensor/camera_extrinsics/rotation", rot_vec, default_rot);
    if (rot_vec.size() == 9) {
        config_.body_R_cam << rot_vec[0], rot_vec[1], rot_vec[2],
                              rot_vec[3], rot_vec[4], rot_vec[5],
                              rot_vec[6], rot_vec[7], rot_vec[8];
    } else {
        ROS_WARN("[MapManager] Invalid camera extrinsics rotation size (%lu). Using default.", rot_vec.size());
        config_.body_R_cam << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    }



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
    nh_private_.param(name_space + "/sensor/camera_angular_resolution_deg", camera_angular_resolution_deg, 3.0);  // 推荐值
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
    depth_sub_ = nh_.subscribe(config_.depth_topic, 10, &MapManager::depthCallback, this); // [Added]
    
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
    
    // Step 1: 先更新局部地图（LiDAR 点云 → 占据体素）
    auto t1 = ros::Time::now();
    local_map_->insertPointCloud(*msg, sensor_pos);
    auto t2 = ros::Time::now();
    
    // Step 2: 再更新相机可视域（标记已观测的占据体素）
    // 必须在 insertPointCloud 之后，因为要先有占据体素才能标记观测
    if (has_camera_pose_) {
        Position cam_pos;
        Quaternion cam_rot;
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            cam_pos = current_camera_position_;
            cam_rot = current_camera_rotation_;
        }
        local_map_->updateCameraFrustum(cam_pos, cam_rot);
    }
    
    // Step 3: 同步到全局地图
    syncLocalToGlobal();
    auto t3 = ros::Time::now();
    
    // Step 4: 提取前沿
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

// 添加相机位姿回调函数（仅存储位姿，视域在lidarCallback中与点云同频更新）
void MapManager::cameraOdomCallback(const geometry_msgs::PoseStampedConstPtr& msg) {    
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    // 提取相机位姿
    current_camera_position_.x() = static_cast<float>(msg->pose.position.x);
    current_camera_position_.y() = static_cast<float>(msg->pose.position.y);
    current_camera_position_.z() = static_cast<float>(msg->pose.position.z);
    
    current_camera_rotation_ = Quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    has_camera_pose_ = true;
    // 注意: 视域更新已移至 lidarCallback 中，与点云处理同频（~10-20Hz）
}

void MapManager::cameraRgbCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 相机RGB图像回调（预留用于语义分割等）
    // 目前不做处理
}

void MapManager::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!running_) return;

    // Check if we have camera pose
    if (!has_odom_) return;

    Position cam_pos;
    Quaternion cam_rot;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        cam_pos = current_position_;
        cam_rot = current_rotation_;
    }

    // Convert ROS image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Parameters
    const double fx = config_.fx;
    const double fy = config_.fy;
    const double cx = config_.cx;
    const double cy = config_.cy;
    const int margin = config_.depth_filter_margin;
    const int skip = config_.skip_pixel;
    const double max_range = config_.local_config.camera_max_range;
    const double scale = config_.depth_scale;

    pcl::PointCloud<pcl::PointXYZ> cloud_obstacle;
    pcl::PointCloud<pcl::PointXYZ> cloud_clearing;
    cloud_obstacle.reserve(msg->width * msg->height / (skip * skip));
    cloud_clearing.reserve(msg->width * msg->height / (skip * skip));

    // Debugging info (Once)
    static bool first_depth = true;
    if (first_depth) {
        float min_val = 1e6, max_val = -1e6;
        for (int v = 0; v < msg->height; v += skip) {
            for (int u = 0; u < msg->width; u += skip) {
                float d = cv_ptr->image.at<float>(v, u);
                if (std::isnan(d) || d < 0.001) continue;
                if (d < min_val) min_val = d;
                if (d > max_val) max_val = d;
            }
        }
        ROS_WARN("[MapManager] First depth image stats: Min=%.2f, Max=%.2f. Configured scale: %.2f.", min_val, max_val, scale);
        ROS_WARN("[MapManager] Interpretation:");
        if (max_val > 100.0 && min_val > 100.0 && scale > 0.1) 
            ROS_WARN("  -> Values seem large (likely mm). Recommend setting depth_scale = 0.001.");
        else if (max_val > 100.0 && min_val < 10.0 && scale > 0.1)
            ROS_WARN("  -> Values seem to be Meters using a large value (%.2f) for sky. Keep depth_scale = 1.0.", max_val);
        else if (max_val > 0.0 && max_val < 20.0 && scale > 0.1)
            ROS_WARN("  -> Values seem normal (likely meters). Keep depth_scale = 1.0.");
        else if (max_val > 240.0 && max_val < 260.0)
            ROS_WARN("  -> Values could be 0-255 pixels? Verify if max saturates at 255.");
            
        first_depth = false;
    }

    // Iterate over pixels
    for (int v = margin; v < msg->height - margin; v += skip) {
        for (int u = margin; u < msg->width - margin; u += skip) {
            float depth = cv_ptr->image.at<float>(v, u) * static_cast<float>(scale);

            if (std::isnan(depth) || depth < 0.1) continue;

            bool is_clearing = false;
            double final_depth = depth;
            
            // Sky detection: if depth > max_range, treat as clearing point at max_range
            if (depth > max_range) {
                final_depth = max_range;
                is_clearing = true;
            }

            // Project to Camera Frame (x right, y down, z forward)
            float x_c = (u - cx) * final_depth / fx;
            float y_c = (v - cy) * final_depth / fy;
            float z_c = final_depth;

            // Transform to Body Frame first, then to World
            // p_body = R_cam_body * p_c
            // p_world = R_body_world * p_body + pos_world
            
            // Note: cam_rot is R_body_world (from odom)
            // config_.body_R_cam is R_cam_body
            
            Eigen::Vector3f p_c(x_c, y_c, z_c);
            Eigen::Vector3f p_b = config_.body_R_cam * p_c;
            Eigen::Vector3f p_w = cam_rot * p_b + cam_pos;

            if (is_clearing) {
                cloud_clearing.push_back(pcl::PointXYZ(p_w.x(), p_w.y(), p_w.z()));
            } else {
                cloud_obstacle.push_back(pcl::PointXYZ(p_w.x(), p_w.y(), p_w.z()));
            }
        }
    }

    // ROS_INFO("depth cloud size: %zu, clearing size: %zu", cloud_obstacle.size(), cloud_clearing.size());

    // 1. Process Obstacles (Occupancy + Clearing)
    if (!cloud_obstacle.empty()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_obstacle, cloud_msg);
        cloud_msg.header = msg->header;
        cloud_msg.header.frame_id = config_.frame_id;
        local_map_->insertPointCloud(cloud_msg, cam_pos);
    }
    
    // 2. Process Clearing Points (Clearing Only, No Occupancy at Endpoint)
    if (!cloud_clearing.empty()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_clearing, cloud_msg);
        cloud_msg.header = msg->header;
        cloud_msg.header.frame_id = config_.frame_id;
        local_map_->insertClearingCloud(cloud_msg, cam_pos);
    }

    // Sync to global
    syncLocalToGlobal();
    
    // Extract Frontier
    global_map_->extractGlobalFrontierIncremental();
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

    // 获取相机观测的体素
    std::vector<VoxelIndex> observed_voxels;
    if (has_camera_pose_) {
        local_map_->getObservedVoxels(observed_voxels);
    }
    
    if (occupied.empty() && free_voxels.empty()) return;
    
    // Step 1: 先批量更新全局地图（创建占据体素）
    global_map_->updateOccupiedVoxels(occupied, free_voxels);
    
    // Step 2: 再更新语义状态（标记占据体素为 VISITED）
    // 必须在 updateOccupiedVoxels 之后，因为要先有占据体素
    if (!observed_voxels.empty()) {
        for (const auto& idx : observed_voxels) {
            global_map_->updateVoxelStateFromCamera(idx, true);
        }
    }

    // 清除局部地图缓存（包括观测缓存）
    local_map_->clearUpdateCache();
    local_map_->clearObservedCache();  // 新增：清除观测缓存
}

}  // namespace star_mvf

