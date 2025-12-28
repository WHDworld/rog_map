/**
 * @file exploration_fsm.cpp
 * @brief 探索状态机实现
 */

#include "rog_map/exploration_fsm.h"
#include <tf/transform_datatypes.h>

namespace rog_map {

// ==================== 析构函数 ====================
ExplorationFSM::~ExplorationFSM() {
    fsm_timer_.stop();
    frontier_timer_.stop();
    safety_timer_.stop();
    vis_timer_.stop();
}

// ==================== 初始化 ====================
void ExplorationFSM::init(ros::NodeHandle& nh, ROGMap::Ptr rog_map, GlobalMap::Ptr global_map) {
    nh_ = nh;
    rog_map_ = rog_map;
    global_map_ = global_map;
    
    // 初始化状态
    state_ = ExplorationState::INIT;
    have_odom_ = false;
    triggered_ = false;
    has_frontier_ = false;
    frontier_ready_ = false;
    viewpoint_reached_ = false;
    
    // 加载参数
    nh_.param("exploration/fsm_frequency", config_.fsm_frequency, 100.0);
    nh_.param("exploration/frontier_update_frequency", config_.frontier_update_frequency, 2.0);
    nh_.param("exploration/safety_check_frequency", config_.safety_check_frequency, 20.0);
    nh_.param("exploration/visualization_frequency", config_.visualization_frequency, 10.0);
    
    nh_.param("exploration/replan_thresh_reached", config_.replan_thresh_reached, 0.5);
    nh_.param("exploration/replan_thresh_yaw", config_.replan_thresh_yaw, 0.1);
    nh_.param("exploration/replan_thresh_time", config_.replan_thresh_time, 2.0);
    nh_.param("exploration/replan_thresh_frontier", config_.replan_thresh_frontier, 1.0);
    
    nh_.param("exploration/auto_start", config_.auto_start, false);
    nh_.param("exploration/init_wait_time", config_.init_wait_time, 2.0);
    nh_.param("exploration/min_frontier_cluster_size", config_.min_frontier_cluster_size, 50);
    
    nh_.param("exploration/enable_visualization", config_.enable_visualization, true);
    nh_.param("exploration/viewpoint_sphere_radius", config_.viewpoint_sphere_radius, 0.2);
    nh_.param("exploration/frustum_length", config_.frustum_length, 2.0);
    nh_.param("exploration/frustum_fov_h", config_.frustum_fov_h, 1.57);
    nh_.param("exploration/frustum_fov_v", config_.frustum_fov_v, 1.22);
    
    // 创建视点生成器
    ViewpointGeneratorConfig vp_config;
    nh_.param("exploration/sensor_range_min", vp_config.sensor_range_min, 0.3f);
    nh_.param("exploration/sensor_range_max", vp_config.sensor_range_max, 5.0f);
    nh_.param("exploration/sensor_fov_h", vp_config.sensor_fov_h, 1.57f);
    nh_.param("exploration/sensor_fov_v", vp_config.sensor_fov_v, 1.22f);
    nh_.param("exploration/optimal_viewing_distance", vp_config.optimal_viewing_distance, 3.0f);
    nh_.param("exploration/uav_radius", vp_config.uav_radius, 0.35f);
    nh_.param("exploration/safety_margin", vp_config.safety_margin, 0.2f);
    nh_.param("exploration/min_cluster_size", vp_config.min_cluster_size, 10);
    nh_.param("exploration/max_viewpoints_per_region", vp_config.max_viewpoints_per_region, 20);
    nh_.param("exploration/planning_horizon", vp_config.planning_horizon, 7.5f);
    
    viewpoint_generator_ = std::make_shared<FrontierViewpointGenerator>(global_map_, vp_config);
    
    // 订阅者
    odom_sub_ = nh_.subscribe("odom", 1, &ExplorationFSM::odometryCallback, this);
    trigger_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ExplorationFSM::triggerCallback, this);
    
    // 发布者
    viewpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/exploration/viewpoint", 1);
    frontier_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/exploration/frontier_cloud", 1);
    frontier_cluster_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frontier_clusters", 1);
    viewpoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/viewpoints", 1);
    frustum_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frustums", 1);
    route_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/region_route", 1);
    info_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/info", 1);
    exploration_state_pub_ = nh_.advertise<std_msgs::Int8>("/exploration/state", 1);
    
    // 定时器
    fsm_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.fsm_frequency), 
                                  &ExplorationFSM::fsmCallback, this);
    frontier_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.frontier_update_frequency),
                                       &ExplorationFSM::frontierCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.visualization_frequency),
                                  &ExplorationFSM::visualizationCallback, this);
    
    fsm_init_time_ = ros::Time::now();
    last_replan_time_ = ros::Time::now();
    
    ROS_INFO("[ExplorationFSM] Initialized successfully");
}

// ==================== 状态转换 ====================
void ExplorationFSM::transitState(ExplorationState new_state, const std::string& reason) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    ExplorationState old_state = state_;
    state_ = new_state;
    
    ROS_INFO("[ExplorationFSM] State: %s -> %s (%s)",
             state_strings_[static_cast<int>(old_state)].c_str(),
             state_strings_[static_cast<int>(new_state)].c_str(),
             reason.c_str());
    
    // 发布状态
    std_msgs::Int8 state_msg;
    state_msg.data = static_cast<int8_t>(new_state);
    exploration_state_pub_.publish(state_msg);
}

std::string ExplorationFSM::getStateString() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_strings_[static_cast<int>(state_)];
}

// ==================== FSM主回调 ====================
void ExplorationFSM::fsmCallback(const ros::TimerEvent& e) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    switch (state_) {
        case ExplorationState::INIT:
            processInit();
            break;
        case ExplorationState::WAIT_TRIGGER:
            processWaitTrigger();
            break;
        case ExplorationState::UPDATE_FRONTIER:
            processUpdateFrontier();
            break;
        case ExplorationState::PLAN_VIEWPOINTS:
            processPlanViewpoints();
            break;
        case ExplorationState::EXEC_EXPLORATION:
            processExecExploration();
            break;
        case ExplorationState::FINISH:
            processFinish();
            break;
        case ExplorationState::EMERGENCY_STOP:
            processEmergencyStop();
            break;
    }
}

// ==================== 状态处理函数 ====================

void ExplorationFSM::processInit() {
    // 等待里程计
    if (!have_odom_) {
        ROS_INFO_THROTTLE(1.0, "[ExplorationFSM] Waiting for odometry...");
        return;
    }
    
    // 等待初始化时间
    if ((ros::Time::now() - fsm_init_time_).toSec() < config_.init_wait_time) {
        ROS_INFO_THROTTLE(1.0, "[ExplorationFSM] Initializing (%.1f/%.1f s)...",
                          (ros::Time::now() - fsm_init_time_).toSec(), config_.init_wait_time);
        return;
    }
    
    state_ = ExplorationState::WAIT_TRIGGER;
    ROS_INFO("[ExplorationFSM] Initialized, waiting for trigger");
}

void ExplorationFSM::processWaitTrigger() {
    // 自动开始
    if (config_.auto_start && frontier_ready_ && has_frontier_) {
        triggered_ = true;
    }
    
    // 检查触发
    if (triggered_) {
        expl_data_.start_time = ros::Time::now();
        expl_data_.reset();
        state_ = ExplorationState::UPDATE_FRONTIER;
        ROS_INFO("[ExplorationFSM] Exploration triggered");
    }
}

void ExplorationFSM::processUpdateFrontier() {
    // 更新前沿信息
    if (updateFrontierInfo()) {
        state_ = ExplorationState::PLAN_VIEWPOINTS;
    } else {
        // 没有前沿，探索完成
        state_ = ExplorationState::FINISH;
        ROS_WARN("[ExplorationFSM] No frontiers found, exploration finished");
    }
}

void ExplorationFSM::processPlanViewpoints() {
    ros::Time plan_start = ros::Time::now();
    
    if (planExplorationViewpoints()) {
        double plan_time = (ros::Time::now() - plan_start).toSec();
        ROS_INFO("[ExplorationFSM] Viewpoint planning took %.2f ms", plan_time * 1000.0);
        
        // 选择下一个视点
        if (selectNextViewpoint()) {
            state_ = ExplorationState::EXEC_EXPLORATION;
            last_replan_time_ = ros::Time::now();
        } else {
            state_ = ExplorationState::UPDATE_FRONTIER;
            ROS_WARN("[ExplorationFSM] No valid viewpoint selected, re-updating frontiers");
        }
    } else {
        state_ = ExplorationState::FINISH;
        ROS_WARN("[ExplorationFSM] Viewpoint planning failed, exploration finished");
    }
}

void ExplorationFSM::processExecExploration() {
    // 检查是否到达当前视点
    if (isNearViewpoint(expl_data_.next_viewpoint)) {
        expl_data_.total_viewpoints_visited++;
        expl_data_.current_viewpoint_idx++;
        
        ROS_INFO("[ExplorationFSM] Reached viewpoint %d/%lu",
                 expl_data_.current_viewpoint_idx,
                 expl_data_.current_viewpoints.size());
        
        // 尝试选择下一个视点
        if (selectNextViewpoint()) {
            last_replan_time_ = ros::Time::now();
        } else {
            // 当前区域探索完成，更新前沿
            state_ = ExplorationState::UPDATE_FRONTIER;
        }
        return;
    }
    
    // 检查前沿是否被覆盖
    if (checkFrontierCovered()) {
        ROS_INFO("[ExplorationFSM] Current frontier covered, replanning...");
        state_ = ExplorationState::UPDATE_FRONTIER;
        return;
    }
    
    // 周期性重规划检查
    double elapsed = (ros::Time::now() - last_replan_time_).toSec();
    if (elapsed > config_.replan_thresh_time) {
        ROS_INFO("[ExplorationFSM] Periodic replan (%.1f s elapsed)", elapsed);
        state_ = ExplorationState::UPDATE_FRONTIER;
        return;
    }
    
    // 持续发布目标视点
    publishViewpointTarget(expl_data_.next_viewpoint);
}

void ExplorationFSM::processFinish() {
    static ros::Time finish_time = ros::Time::now();
    static bool logged = false;
    
    if (!logged) {
        expl_data_.finish_time = ros::Time::now();
        double duration = (expl_data_.finish_time - expl_data_.start_time).toSec();
        
        ROS_INFO("====================================");
        ROS_INFO("[ExplorationFSM] Exploration Finished!");
        ROS_INFO("  Duration: %.2f s", duration);
        ROS_INFO("  Viewpoints visited: %d", expl_data_.total_viewpoints_visited);
        ROS_INFO("  Total distance: %.2f m", expl_data_.total_distance);
        ROS_INFO("====================================");
        
        logged = true;
        finish_time = ros::Time::now();
    }
    
    // 清理可视化
    clearVisualization();
    
    // 5秒后返回等待状态
    if ((ros::Time::now() - finish_time).toSec() > 5.0) {
        triggered_ = false;
        logged = false;
        state_ = ExplorationState::WAIT_TRIGGER;
        ROS_INFO("[ExplorationFSM] Ready for new exploration");
    }
}

void ExplorationFSM::processEmergencyStop() {
    // 紧急停止状态，等待手动恢复
    ROS_WARN_THROTTLE(1.0, "[ExplorationFSM] Emergency stop! Waiting for manual recovery...");
}

// ==================== 前沿回调 ====================
void ExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
    if (!have_odom_) return;
    
    // 更新全局地图的前沿信息
    std::vector<int> frontier_subgrids;
    global_map_->getFrontierSubGrids(frontier_subgrids);
    
    int total_frontiers = countTotalFrontiers();
    has_frontier_ = (total_frontiers > config_.min_frontier_cluster_size);
    frontier_ready_ = true;
    
    last_frontier_update_time_ = ros::Time::now();
    
    // ========== 检测前沿是否有变化 ==========
    // 只有当前沿有实际变化时才重新生成视点，避免无人机静止时视点不稳定
    frontier_changed_ = false;
    
    // 检查1：前沿子网格列表变化
    if (frontier_subgrids.size() != last_frontier_subgrids_.size()) {
        frontier_changed_ = true;
    } else {
        // 排序后比较（因为顺序可能不同）
        std::vector<int> sorted_current = frontier_subgrids;
        std::vector<int> sorted_last = last_frontier_subgrids_;
        std::sort(sorted_current.begin(), sorted_current.end());
        std::sort(sorted_last.begin(), sorted_last.end());
        if (sorted_current != sorted_last) {
            frontier_changed_ = true;
        }
    }
    
    // 检查2：前沿体素总数显著变化（超过5%）
    if (!frontier_changed_ && last_total_frontiers_ > 0) {
        float change_ratio = std::abs(total_frontiers - last_total_frontiers_) 
                            / static_cast<float>(last_total_frontiers_);
        if (change_ratio > 0.05f) {
            frontier_changed_ = true;
        }
    }
    
    // 首次运行时强制更新
    if (last_frontier_subgrids_.empty() && !frontier_subgrids.empty()) {
        frontier_changed_ = true;
    }
    
    ROS_DEBUG_THROTTLE(2.0, "[ExplorationFSM] Frontier update: %lu subgrids, %d voxels, changed=%s",
                       frontier_subgrids.size(), total_frontiers, 
                       frontier_changed_ ? "true" : "false");
    
    // ========== 被动可视化模式 ==========
    // 即使没有触发探索，也更新视点信息用于可视化
    // 只有在前沿有变化时才重新生成视点，保持视点稳定性
    if (!triggered_ && config_.enable_visualization && has_frontier_ && frontier_changed_) {
        // 更新UAV状态
        UAVState uav_state;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            uav_state.position = odom_pos_.cast<float>();
            uav_state.velocity = odom_vel_.cast<float>();
            uav_state.yaw = static_cast<float>(odom_yaw_);
        }
        viewpoint_generator_->updateUAVState(uav_state);
        
        // 生成探索规划（仅用于可视化，不执行）
        std::lock_guard<std::mutex> lock(data_mutex_);
        bool success = viewpoint_generator_->generateExplorationPlan(
            frontier_subgrids,
            expl_data_.regions,
            expl_data_.region_route);
        
        if (success && !expl_data_.regions.empty()) {
            // 收集所有区域的视点
            expl_data_.current_viewpoints.clear();
            for (const auto& region : expl_data_.regions) {
                for (const auto& vp : region.viewpoints) {
                    if (vp.is_valid) {
                        expl_data_.current_viewpoints.push_back(vp);
                    }
                }
            }
            
            // 选择第一个视点作为下一个目标（仅用于高亮显示）
            if (!expl_data_.current_viewpoints.empty()) {
                expl_data_.next_viewpoint = expl_data_.current_viewpoints[0];
            }
            
            ROS_INFO_THROTTLE(5.0, "[ExplorationFSM] Passive mode: %lu regions, %lu viewpoints generated",
                             expl_data_.regions.size(), expl_data_.current_viewpoints.size());
        }
        
        // 更新上次状态
        last_frontier_subgrids_ = frontier_subgrids;
        last_total_frontiers_ = total_frontiers;
    }
}

// ==================== 前沿处理 ====================
bool ExplorationFSM::updateFrontierInfo() {
    std::vector<int> frontier_subgrids;
    global_map_->getFrontierSubGrids(frontier_subgrids);
    
    if (frontier_subgrids.empty()) {
        has_frontier_ = false;
        return false;
    }
    
    // 更新UAV状态
    UAVState uav_state;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        uav_state.position = odom_pos_.cast<float>();
        uav_state.velocity = odom_vel_.cast<float>();
        uav_state.yaw = static_cast<float>(odom_yaw_);
    }
    viewpoint_generator_->updateUAVState(uav_state);
    
    has_frontier_ = true;
    return true;
}

bool ExplorationFSM::checkFrontierCovered() {
    // 检查当前目标簇是否已被覆盖
    if (expl_data_.current_viewpoints.empty()) {
        return true;
    }
    
    const ExplorationViewpoint& vp = expl_data_.next_viewpoint;
    
    // 检查目标簇是否还有足够的前沿体素
    // 这里简化处理：检查目标位置附近是否还有未知体素
    Eigen::Vector3f target_pos = vp.position;
    float check_radius = 2.0f;
    
    int unknown_count = 0;
    for (float dx = -check_radius; dx <= check_radius; dx += 0.5f) {
        for (float dy = -check_radius; dy <= check_radius; dy += 0.5f) {
            for (float dz = -check_radius; dz <= check_radius; dz += 0.5f) {
                Eigen::Vector3f check_pos = target_pos + Eigen::Vector3f(dx, dy, dz);
                if (!global_map_->isInGlobalMapRange(check_pos)) continue;
                
                GlobalVoxelState state = global_map_->getVoxelState(check_pos);
                if (VoxelStateBits::isUnknown(state)) {
                    unknown_count++;
                }
            }
        }
    }
    
    // 如果未知体素太少，认为已被覆盖
    return unknown_count < 10;
}

int ExplorationFSM::countTotalFrontiers() const {
    std::vector<int> frontier_subgrids;
    global_map_->getFrontierSubGrids(frontier_subgrids);
    
    int total = 0;
    for (int sg_id : frontier_subgrids) {
        const SubGrid& sg = global_map_->getSubGrid(sg_id);
        total += static_cast<int>(sg.sub_grid_frontier_voxels_.size());
    }
    return total;
}

// ==================== 视点规划 ====================
bool ExplorationFSM::planExplorationViewpoints() {
    std::vector<int> frontier_subgrids;
    global_map_->getFrontierSubGrids(frontier_subgrids);
    
    if (frontier_subgrids.empty()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 调用视点生成器
    bool success = viewpoint_generator_->generateExplorationPlan(
        frontier_subgrids,
        expl_data_.regions,
        expl_data_.region_route);
    
    if (!success || expl_data_.regions.empty()) {
        return false;
    }
    
    // 重置索引
    expl_data_.current_region_idx = 0;
    expl_data_.current_viewpoint_idx = 0;
    
    // 获取当前区域的视点
    if (!expl_data_.region_route.empty()) {
        int first_region_id = expl_data_.region_route[0];
        for (const auto& region : expl_data_.regions) {
            if (region.region_id == first_region_id) {
                expl_data_.current_viewpoints = region.viewpoints;
                break;
            }
        }
    }
    
    ROS_INFO("[ExplorationFSM] Planned %lu regions, %lu viewpoints in first region",
             expl_data_.regions.size(), expl_data_.current_viewpoints.size());
    
    return !expl_data_.current_viewpoints.empty();
}

bool ExplorationFSM::selectNextViewpoint() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 检查当前区域是否还有视点
    while (expl_data_.current_viewpoint_idx >= static_cast<int>(expl_data_.current_viewpoints.size())) {
        // 切换到下一个区域
        expl_data_.current_region_idx++;
        expl_data_.current_viewpoint_idx = 0;
        
        if (expl_data_.current_region_idx >= static_cast<int>(expl_data_.region_route.size())) {
            // 所有区域探索完成
            return false;
        }
        
        // 获取下一个区域的视点
        int next_region_id = expl_data_.region_route[expl_data_.current_region_idx];
        expl_data_.current_viewpoints.clear();
        
        for (const auto& region : expl_data_.regions) {
            if (region.region_id == next_region_id) {
                expl_data_.current_viewpoints = region.viewpoints;
                break;
            }
        }
        
        ROS_INFO("[ExplorationFSM] Switched to region %d with %lu viewpoints",
                 next_region_id, expl_data_.current_viewpoints.size());
    }
    
    // 选择当前视点
    expl_data_.next_viewpoint = expl_data_.current_viewpoints[expl_data_.current_viewpoint_idx];
    
    ROS_INFO("[ExplorationFSM] Selected viewpoint %d: (%.2f, %.2f, %.2f) yaw=%.2f",
             expl_data_.current_viewpoint_idx,
             expl_data_.next_viewpoint.position.x(),
             expl_data_.next_viewpoint.position.y(),
             expl_data_.next_viewpoint.position.z(),
             expl_data_.next_viewpoint.yaw);
    
    return true;
}

bool ExplorationFSM::getNextViewpoint(ExplorationViewpoint& viewpoint) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (state_ != ExplorationState::EXEC_EXPLORATION) {
        return false;
    }
    
    viewpoint = expl_data_.next_viewpoint;
    return viewpoint.is_valid;
}

// ==================== 回调函数 ====================
void ExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    odom_pos_.x() = msg->pose.pose.position.x;
    odom_pos_.y() = msg->pose.pose.position.y;
    odom_pos_.z() = msg->pose.pose.position.z;
    
    odom_vel_.x() = msg->twist.twist.linear.x;
    odom_vel_.y() = msg->twist.twist.linear.y;
    odom_vel_.z() = msg->twist.twist.linear.z;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_yaw_ = yaw;
    
    have_odom_ = true;
}

void ExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (state_ != ExplorationState::WAIT_TRIGGER) {
        ROS_WARN("[ExplorationFSM] Trigger received but not in WAIT_TRIGGER state");
        return;
    }
    
    if (!frontier_ready_ || !has_frontier_) {
        ROS_WARN("[ExplorationFSM] Trigger received but frontiers not ready");
        return;
    }
    
    triggered_ = true;
    ROS_INFO("[ExplorationFSM] Exploration triggered by user");
}

void ExplorationFSM::triggerExploration() {
    triggered_ = true;
}

void ExplorationFSM::stopExploration() {
    triggered_ = false;
    state_ = ExplorationState::WAIT_TRIGGER;
    clearVisualization();
}

// ==================== 可视化回调 ====================
void ExplorationFSM::visualizationCallback(const ros::TimerEvent& e) {
    if (!config_.enable_visualization) return;
    visualize();
}

void ExplorationFSM::visualize() {
    visualizeFrontierClusters();
    visualizeViewpoints();
    visualizeFrustums();
    visualizeRegionRoute();
    visualizeExplorationInfo();
}

void ExplorationFSM::visualizeFrontierClusters() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    // 为每个区域的每个簇创建不同颜色的marker
    std::vector<std::array<float, 4>> colors = {
        {1.0f, 0.0f, 0.0f, 0.6f},  // Red
        {0.0f, 1.0f, 0.0f, 0.6f},  // Green
        {0.0f, 0.0f, 1.0f, 0.6f},  // Blue
        {1.0f, 1.0f, 0.0f, 0.6f},  // Yellow
        {1.0f, 0.0f, 1.0f, 0.6f},  // Magenta
        {0.0f, 1.0f, 1.0f, 0.6f},  // Cyan
        {1.0f, 0.5f, 0.0f, 0.6f},  // Orange
        {0.5f, 0.0f, 1.0f, 0.6f},  // Purple
    };
    
    for (const auto& region : expl_data_.regions) {
        for (const auto& cluster : region.clusters) {
            if (cluster.voxel_indices.empty()) continue;
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "frontier_clusters";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::CUBE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            
            float voxel_size = global_map_->global_resolution_;
            marker.scale.x = voxel_size * 0.8;
            marker.scale.y = voxel_size * 0.8;
            marker.scale.z = voxel_size * 0.8;
            
            auto& color = colors[cluster.cluster_id % colors.size()];
            marker.color.r = color[0];
            marker.color.g = color[1];
            marker.color.b = color[2];
            marker.color.a = color[3];
            
            for (const auto& idx : cluster.voxel_indices) {
                Eigen::Vector3f pos;
                global_map_->indexToPos(idx, pos);
                
                geometry_msgs::Point p;
                p.x = pos.x();
                p.y = pos.y();
                p.z = pos.z();
                marker.points.push_back(p);
            }
            
            markers.markers.push_back(marker);
            
            // 添加簇中心和法向量
            visualization_msgs::Marker center_marker;
            center_marker.header.frame_id = "map";
            center_marker.header.stamp = ros::Time::now();
            center_marker.ns = "cluster_centers";
            center_marker.id = marker_id++;
            center_marker.type = visualization_msgs::Marker::SPHERE;
            center_marker.action = visualization_msgs::Marker::ADD;
            center_marker.pose.position.x = cluster.centroid.x();
            center_marker.pose.position.y = cluster.centroid.y();
            center_marker.pose.position.z = cluster.centroid.z();
            center_marker.scale.x = 0.3;
            center_marker.scale.y = 0.3;
            center_marker.scale.z = 0.3;
            center_marker.color.r = 1.0;
            center_marker.color.g = 1.0;
            center_marker.color.b = 1.0;
            center_marker.color.a = 1.0;
            markers.markers.push_back(center_marker);
        }
    }
    
    // 清除旧marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    
    frontier_cluster_pub_.publish(markers);
}

void ExplorationFSM::visualizeViewpoints() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    for (const auto& region : expl_data_.regions) {
        for (const auto& vp : region.viewpoints) {
            if (!vp.is_valid) continue;
            
            // 视点位置（球体）
            visualization_msgs::Marker sphere;
            sphere.header.frame_id = "map";
            sphere.header.stamp = ros::Time::now();
            sphere.ns = "viewpoints";
            sphere.id = marker_id++;
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position.x = vp.position.x();
            sphere.pose.position.y = vp.position.y();
            sphere.pose.position.z = vp.position.z();
            sphere.scale.x = config_.viewpoint_sphere_radius * 2;
            sphere.scale.y = config_.viewpoint_sphere_radius * 2;
            sphere.scale.z = config_.viewpoint_sphere_radius * 2;
            
            // 当前目标视点用绿色，其他用蓝色
            bool is_current = (vp.region_id == expl_data_.next_viewpoint.region_id &&
                              vp.cluster_id == expl_data_.next_viewpoint.cluster_id);
            if (is_current) {
                sphere.color.r = 0.0;
                sphere.color.g = 1.0;
                sphere.color.b = 0.0;
            } else {
                sphere.color.r = 0.2;
                sphere.color.g = 0.4;
                sphere.color.b = 1.0;
            }
            sphere.color.a = 0.8;
            
            markers.markers.push_back(sphere);
            
            // 视线方向（箭头）
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "view_directions";
            arrow.id = marker_id++;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            
            geometry_msgs::Point start, end;
            start.x = vp.position.x();
            start.y = vp.position.y();
            start.z = vp.position.z();
            
            float arrow_length = 1.0f;
            end.x = start.x + arrow_length * std::cos(vp.pitch) * std::cos(vp.yaw);
            end.y = start.y + arrow_length * std::cos(vp.pitch) * std::sin(vp.yaw);
            end.z = start.z - arrow_length * std::sin(vp.pitch);
            
            arrow.points.push_back(start);
            arrow.points.push_back(end);
            arrow.scale.x = 0.05;
            arrow.scale.y = 0.1;
            arrow.scale.z = 0.1;
            arrow.color.r = 1.0;
            arrow.color.g = 1.0;
            arrow.color.b = 0.0;
            arrow.color.a = 1.0;
            
            markers.markers.push_back(arrow);
        }
    }
    
    viewpoint_marker_pub_.publish(markers);
}

void ExplorationFSM::visualizeFrustums() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    for (const auto& region : expl_data_.regions) {
        for (const auto& vp : region.viewpoints) {
            if (!vp.is_valid) continue;
            
            bool is_current = (vp.region_id == expl_data_.next_viewpoint.region_id &&
                              vp.cluster_id == expl_data_.next_viewpoint.cluster_id);
            
            std::array<float, 4> color;
            if (is_current) {
                color = {0.0f, 1.0f, 0.0f, 1.0f};
            } else {
                color = {0.0f, 0.5f, 1.0f, 1.0f};
            }
            
            createFrustumMarker(vp.position, vp.yaw, vp.pitch,
                               config_.frustum_length,
                               config_.frustum_fov_h, config_.frustum_fov_v,
                               marker_id, color, markers);
            marker_id += 5;  // 每个视锥体使用5个marker
        }
    }
    
    frustum_marker_pub_.publish(markers);
}

void ExplorationFSM::createFrustumMarker(
    const Eigen::Vector3f& position,
    float yaw, float pitch,
    float length, float fov_h, float fov_v,
    int id,
    const std::array<float, 4>& color,
    visualization_msgs::MarkerArray& markers)
{
    // 计算视锥体的4个角点
    float half_h = fov_h / 2.0f;
    float half_v = fov_v / 2.0f;
    
    // 视线方向
    Eigen::Vector3f view_dir(
        std::cos(pitch) * std::cos(yaw),
        std::cos(pitch) * std::sin(yaw),
        -std::sin(pitch));
    
    // 计算up和right向量
    Eigen::Vector3f world_up(0, 0, 1);
    Eigen::Vector3f right = view_dir.cross(world_up).normalized();
    Eigen::Vector3f up = right.cross(view_dir).normalized();
    
    // 计算4个角点方向
    std::vector<Eigen::Vector3f> corners;
    corners.push_back(view_dir * std::cos(half_h) * std::cos(half_v) + 
                     right * std::sin(half_h) + up * std::sin(half_v));
    corners.push_back(view_dir * std::cos(half_h) * std::cos(half_v) - 
                     right * std::sin(half_h) + up * std::sin(half_v));
    corners.push_back(view_dir * std::cos(half_h) * std::cos(half_v) - 
                     right * std::sin(half_h) - up * std::sin(half_v));
    corners.push_back(view_dir * std::cos(half_h) * std::cos(half_v) + 
                     right * std::sin(half_h) - up * std::sin(half_v));
    
    for (auto& c : corners) {
        c = position + c.normalized() * length;
    }
    
    // 创建线框
    visualization_msgs::Marker lines;
    lines.header.frame_id = "map";
    lines.header.stamp = ros::Time::now();
    lines.ns = "frustum_lines";
    lines.id = id;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.scale.x = 0.02;
    lines.color.r = color[0];
    lines.color.g = color[1];
    lines.color.b = color[2];
    lines.color.a = color[3] * 2;  // 线框更明显
    
    geometry_msgs::Point origin;
    origin.x = position.x();
    origin.y = position.y();
    origin.z = position.z();
    
    // 从原点到4个角
    for (const auto& c : corners) {
        lines.points.push_back(origin);
        geometry_msgs::Point p;
        p.x = c.x(); p.y = c.y(); p.z = c.z();
        lines.points.push_back(p);
    }
    
    // 角之间的连线
    for (int i = 0; i < 4; ++i) {
        geometry_msgs::Point p1, p2;
        p1.x = corners[i].x(); p1.y = corners[i].y(); p1.z = corners[i].z();
        p2.x = corners[(i+1)%4].x(); p2.y = corners[(i+1)%4].y(); p2.z = corners[(i+1)%4].z();
        lines.points.push_back(p1);
        lines.points.push_back(p2);
    }
    
    markers.markers.push_back(lines);
}

void ExplorationFSM::visualizeRegionRoute() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    visualization_msgs::MarkerArray markers;
    
    if (expl_data_.region_route.size() < 2) {
        route_marker_pub_.publish(markers);
        return;
    }
    
    // 路由线
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "region_route";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.1;
    line.color.r = 1.0;
    line.color.g = 0.5;
    line.color.b = 0.0;
    line.color.a = 0.8;
    
    for (int region_id : expl_data_.region_route) {
        for (const auto& region : expl_data_.regions) {
            if (region.region_id == region_id) {
                geometry_msgs::Point p;
                p.x = region.centroid.x();
                p.y = region.centroid.y();
                p.z = region.centroid.z();
                line.points.push_back(p);
                break;
            }
        }
    }
    
    markers.markers.push_back(line);
    
    // 区域编号
    int marker_id = 1;
    for (size_t i = 0; i < expl_data_.region_route.size(); ++i) {
        int region_id = expl_data_.region_route[i];
        for (const auto& region : expl_data_.regions) {
            if (region.region_id == region_id) {
                visualization_msgs::Marker text;
                text.header.frame_id = "map";
                text.header.stamp = ros::Time::now();
                text.ns = "region_ids";
                text.id = marker_id++;
                text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text.action = visualization_msgs::Marker::ADD;
                text.pose.position.x = region.centroid.x();
                text.pose.position.y = region.centroid.y();
                text.pose.position.z = region.centroid.z() + 0.5;
                text.scale.z = 0.5;
                text.color.r = 1.0;
                text.color.g = 1.0;
                text.color.b = 1.0;
                text.color.a = 1.0;
                text.text = std::to_string(i + 1);
                markers.markers.push_back(text);
                break;
            }
        }
    }
    
    route_marker_pub_.publish(markers);
}

void ExplorationFSM::visualizeExplorationInfo() {
    visualization_msgs::MarkerArray markers;
    
    visualization_msgs::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = ros::Time::now();
    text.ns = "exploration_info";
    text.id = 0;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    
    // 在固定位置显示
    text.pose.position.x = 0;
    text.pose.position.y = 0;
    text.pose.position.z = 3.0;
    text.scale.z = 0.4;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    
    std::stringstream ss;
    ss << "State: " << getStateString() << "\n";
    ss << "Regions: " << expl_data_.regions.size() << "\n";
    ss << "Current Region: " << expl_data_.current_region_idx + 1 << "/" << expl_data_.region_route.size() << "\n";
    ss << "Viewpoints: " << expl_data_.current_viewpoint_idx + 1 << "/" << expl_data_.current_viewpoints.size() << "\n";
    ss << "Total Visited: " << expl_data_.total_viewpoints_visited;
    
    text.text = ss.str();
    markers.markers.push_back(text);
    
    info_marker_pub_.publish(markers);
}

void ExplorationFSM::clearVisualization() {
    visualization_msgs::MarkerArray empty;
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    empty.markers.push_back(delete_all);
    
    frontier_cluster_pub_.publish(empty);
    viewpoint_marker_pub_.publish(empty);
    frustum_marker_pub_.publish(empty);
    route_marker_pub_.publish(empty);
    info_marker_pub_.publish(empty);
}

// ==================== 辅助函数 ====================
void ExplorationFSM::publishViewpointTarget(const ExplorationViewpoint& vp) {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = vp.position.x();
    msg.pose.position.y = vp.position.y();
    msg.pose.position.z = vp.position.z();
    
    // 从yaw创建四元数
    tf::Quaternion q;
    q.setRPY(0, 0, vp.yaw);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    
    viewpoint_pub_.publish(msg);
}

bool ExplorationFSM::isNearViewpoint(const ExplorationViewpoint& vp) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    double dist = (odom_pos_ - vp.position.cast<double>()).norm();
    double yaw_diff = std::abs(getYawDiff(odom_yaw_, vp.yaw));
    
    return dist < config_.replan_thresh_reached && 
           yaw_diff < config_.replan_thresh_yaw;
}

double ExplorationFSM::getYawDiff(double yaw1, double yaw2) {
    double diff = yaw1 - yaw2;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

} // namespace rog_map

