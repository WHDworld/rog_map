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
    nh_.param("exploration/replan_thresh_time", config_.replan_thresh_time, 3.0);           // Falcon: 3.0s
    nh_.param("exploration/replan_thresh_frontier", config_.replan_thresh_frontier, 0.5);   // Falcon: 0.2s
    nh_.param("exploration/replan_thresh_traj_remain", config_.replan_thresh_traj_remain, 0.5);  // Falcon: 0.05s
    nh_.param("exploration/frontier_covered_ratio", config_.frontier_covered_ratio, 0.25);  // Falcon: 0.25
    
    nh_.param("exploration/auto_start", config_.auto_start, false);
    nh_.param("exploration/init_wait_time", config_.init_wait_time, 2.0);
    nh_.param("exploration/min_frontier_cluster_size", config_.min_frontier_cluster_size, 50);
    
    // 起飞高度 - 从 EGO-Planner 配置读取，用于判断起飞是否完成
    nh_.param("fsm/takeoff_height", config_.takeoff_height, 1.0);
    
    nh_.param("exploration/enable_visualization", config_.enable_visualization, true);
    nh_.param("exploration/viewpoint_sphere_radius", config_.viewpoint_sphere_radius, 0.2);
    nh_.param("exploration/frustum_length", config_.frustum_length, 2.0);
    nh_.param("exploration/frustum_fov_h", config_.frustum_fov_h, 1.57);
    nh_.param("exploration/frustum_fov_v", config_.frustum_fov_v, 1.22);
    
    // 创建视点生成器
    ViewpointGeneratorConfig vp_config;
    
    // 传感器参数（用于评估视点覆盖能力）
    nh_.param("exploration/sensor_range_min", vp_config.sensor_range_min, 0.5f);
    nh_.param("exploration/sensor_range_max", vp_config.sensor_range_max, 5.0f);
    nh_.param("exploration/sensor_fov_h", vp_config.sensor_fov_h, 1.57f);  // ~90度
    nh_.param("exploration/sensor_fov_v", vp_config.sensor_fov_v, 1.22f);  // ~70度
    
    // 视点采样参数
    float takeoff_height = 1.0f;
    nh_.param("fsm/takeoff_height", takeoff_height, 1.0f);
    nh_.param("exploration/viewpoint_height", vp_config.viewpoint_height, takeoff_height);
    nh_.param("exploration/candidate_rmin", vp_config.candidate_rmin, 1.5f);
    nh_.param("exploration/candidate_rmax", vp_config.candidate_rmax, 4.5f);
    nh_.param("exploration/candidate_rnum", vp_config.candidate_rnum, 3);
    nh_.param("exploration/candidate_dphi", vp_config.candidate_dphi, 0.5f);
    nh_.param("exploration/optimal_distance", vp_config.optimal_distance, 3.0f);
    
    // 前沿簇参数
    nh_.param("exploration/cluster_eps", vp_config.cluster_eps, 1.0f);
    nh_.param("exploration/min_cluster_size", vp_config.min_cluster_size, 10);
    nh_.param("exploration/cluster_split_threshold", vp_config.cluster_split_threshold, 3.0f);
    nh_.param("exploration/max_clusters", vp_config.max_clusters, 20);
    
    // 安全参数
    nh_.param("exploration/uav_radius", vp_config.uav_radius, 0.35f);
    nh_.param("exploration/safety_margin", vp_config.safety_margin, 0.2f);
    
    // 滤波参数
    nh_.param("exploration/min_viewpoint_distance", vp_config.min_viewpoint_distance, 0.8f);
    nh_.param("exploration/min_coverage_score", vp_config.min_coverage_score, 0.3f);
    nh_.param("exploration/min_visibility_ratio", vp_config.min_visibility_ratio, 0.3f);
    
    // 历史节点参数（借鉴EDEN算法）
    nh_.param("exploration/history_node_distance", vp_config.history_node_distance, 3.0f);
    nh_.param("exploration/node_frontier_radius", vp_config.node_frontier_radius, 5.0f);
    
    // 前沿点过滤参数
    float ground_height = 0.0f;
    nh_.param("rog_map/global_map/ground_height", ground_height, 0.0f);
    vp_config.ground_height = ground_height;
    nh_.param("exploration/ground_offset", vp_config.ground_offset, 0.1f);
    
    // 头顶区域过滤参数（在簇级别过滤）
    nh_.param("exploration/overhead_height_threshold", vp_config.overhead_height_threshold, 0.5f);
    nh_.param("exploration/overhead_distance_threshold", vp_config.overhead_distance_threshold, 2.0f);
    
    ROS_INFO("[ExplorationFSM] Viewpoint config: height=%.2f, r=[%.1f, %.1f], sensor=[%.1f, %.1f], fov_h=%.2f",
             vp_config.viewpoint_height, vp_config.candidate_rmin, vp_config.candidate_rmax,
             vp_config.sensor_range_min, vp_config.sensor_range_max, vp_config.sensor_fov_h);
    ROS_INFO("[ExplorationFSM] History node: dist=%.1f, radius=%.1f | Ground filter: %.2f+%.2f",
             vp_config.history_node_distance, vp_config.node_frontier_radius,
             vp_config.ground_height, vp_config.ground_offset);
    ROS_INFO("[ExplorationFSM] Overhead filter: height_th=%.2f, dist_th=%.2f",
             vp_config.overhead_height_threshold, vp_config.overhead_distance_threshold);
    
    viewpoint_generator_ = std::make_shared<FrontierViewpointGenerator>(global_map_, vp_config);
    
    // 订阅者
    odom_sub_ = nh_.subscribe("odom", 1, &ExplorationFSM::odometryCallback, this);
    trigger_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ExplorationFSM::triggerCallback, this);
    
    // 发布者
    // 使用 quadrotor_msgs::PositionCommand 发送视点给 EGO-Planner
    // 包含位置、yaw 角和期望终端速度
    viewpoint_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/exploration/viewpoint", 1);
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
    
    // 等待无人机起飞完成（高度达到起飞高度的90%）
    double takeoff_threshold = config_.takeoff_height * 0.9;
    if (odom_pos_.z() < takeoff_threshold) {
        ROS_INFO_THROTTLE(1.0, "[ExplorationFSM] Waiting for takeoff (z=%.2f < %.2f)...",
                          odom_pos_.z(), takeoff_threshold);
        return;
    }
    
    state_ = ExplorationState::WAIT_TRIGGER;
    ROS_INFO("[ExplorationFSM] Takeoff complete (z=%.2f), waiting for trigger (use 2D Nav Goal in RViz)",
             odom_pos_.z());
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
            expl_data_.viewpoint_published = false;  // 重置标志，确保新视点会被发布
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
    double t_cur = (ros::Time::now() - traj_start_time_).toSec();
    double elapsed_since_replan = (ros::Time::now() - last_replan_time_).toSec();
    
    // ==================== Falcon 风格重规划逻辑 ====================
    // 三种重规划触发条件：
    // 1. 目标前视簇被覆盖（执行一定时间后）
    // 2. 轨迹即将执行完毕（剩余时间不足）
    // 3. 周期性重规划（适应环境变化）
    
    bool need_replan = false;
    int replan_type = 0;
    
    // 检查是否到达当前视点
    if (isNearViewpoint(expl_data_.next_viewpoint)) {
        expl_data_.total_viewpoints_visited++;
        expl_data_.current_viewpoint_idx++;
        
        ROS_INFO("[ExplorationFSM] Reached viewpoint %d/%lu",
                 expl_data_.current_viewpoint_idx,
                 expl_data_.current_viewpoints.size());
        
        // 尝试选择下一个视点
        if (selectNextViewpoint()) {
            // 成功选择下一个视点，重置轨迹计时
            traj_start_time_ = ros::Time::now();
            last_replan_time_ = ros::Time::now();
            expl_data_.viewpoint_published = false;
            // 估算轨迹时间（简单估算：距离 / 速度）
            Eigen::Vector3f target_pos = expl_data_.next_viewpoint.position;
            Eigen::Vector3d odom;
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                odom = odom_pos_;
            }
            double dist = (target_pos.cast<double>() - odom).norm();
            traj_duration_ = dist / 1.0 + 2.0;  // 假设1m/s + 2s缓冲
        } else {
            // 当前区域探索完成，更新前沿
            state_ = ExplorationState::UPDATE_FRONTIER;
        }
        return;
    }
    
    // ========== 只有在视点已发布后才检查重规划条件 ==========
    // 避免在视点还未发布时就触发重规划
    if (expl_data_.viewpoint_published) {
        // ========== 触发条件1：目标前视簇被覆盖（Falcon: t > replan_thresh2_） ==========
        if (t_cur > config_.replan_thresh_frontier && checkFrontierCovered()) {
            ROS_INFO("[ExplorationFSM] Replan: cluster covered (t=%.2fs)", t_cur);
            need_replan = true;
            replan_type = 1;
        }
        // ========== 触发条件2：轨迹即将执行完毕（Falcon: remaining < replan_thresh1_） ==========
        // 只在 traj_duration_ > 0 且 t_cur > 0 时才检查（确保已正确初始化）
        else if (traj_duration_ > 0.1 && t_cur > 0.1 && 
                 (traj_duration_ - t_cur) < config_.replan_thresh_traj_remain) {
            ROS_INFO("[ExplorationFSM] Replan: trajectory almost done (remaining=%.2fs)", 
                     traj_duration_ - t_cur);
            need_replan = true;
            replan_type = 2;
        }
        // ========== 触发条件3：周期性重规划（Falcon: t > replan_thresh3_） ==========
        else if (elapsed_since_replan > config_.replan_thresh_time) {
            ROS_INFO("[ExplorationFSM] Replan: periodic call (%.1fs elapsed)", elapsed_since_replan);
            need_replan = true;
            replan_type = 3;
        }
    }
    
    if (need_replan) {
        // 重规划：更新前沿并重新规划视点
        last_replan_time_ = ros::Time::now();
        state_ = ExplorationState::UPDATE_FRONTIER;
        return;
    }
    
    // 只在视点尚未发布时发布一次（避免频繁触发EGO-Planner重规划）
    if (!expl_data_.viewpoint_published) {
        publishViewpointTarget(expl_data_.next_viewpoint);
        expl_data_.viewpoint_published = true;
        
        // 记录轨迹开始时间
        traj_start_time_ = ros::Time::now();
        
        // 重置覆盖检测的初始未知体素计数（用于 Falcon 风格的比例检测）
        initial_unknown_count_ = 0;
        
        // 估算轨迹执行时间
        Eigen::Vector3f target_pos = expl_data_.next_viewpoint.position;
        Eigen::Vector3d odom;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom = odom_pos_;
        }
        double dist = (target_pos.cast<double>() - odom).norm();
        traj_duration_ = dist / 1.0 + 2.0;  // 假设1m/s + 2s缓冲
        
        ROS_INFO("[ExplorationFSM] Published viewpoint: (%.2f, %.2f, %.2f) yaw=%.2f, target=(%.2f, %.2f, %.2f), est_time=%.1fs",
                 expl_data_.next_viewpoint.position.x(),
                 expl_data_.next_viewpoint.position.y(),
                 expl_data_.next_viewpoint.position.z(),
                 expl_data_.next_viewpoint.yaw,
                 expl_data_.next_viewpoint.target_center.x(),
                 expl_data_.next_viewpoint.target_center.y(),
                 expl_data_.next_viewpoint.target_center.z(),
                 traj_duration_);
    }
}

void ExplorationFSM::processFinish() {
    static ros::Time finish_time = ros::Time::now();
    static bool logged = false;
    
    if (!logged) {
        expl_data_.finish_time = ros::Time::now();
        double duration = (expl_data_.finish_time - expl_data_.start_time).toSec();
        
        ROS_INFO("====================================");
        ROS_INFO("[ExplorationFSM] Exploration paused (no viewpoints)");
        ROS_INFO("  Duration: %.2f s", duration);
        ROS_INFO("  Viewpoints visited: %d", expl_data_.total_viewpoints_visited);
        ROS_INFO("  Total distance: %.2f m", expl_data_.total_distance);
        ROS_INFO("====================================");
        
        logged = true;
        finish_time = ros::Time::now();
    }
    
    // 2秒后主动更新视点并检查是否有新的前沿可以继续探索
    if ((ros::Time::now() - finish_time).toSec() > 2.0) {
        // 主动更新视点（因为 triggered_=true 时 frontierCallback 不会更新）
        UAVState uav_state;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            uav_state.position = odom_pos_.cast<float>();
            uav_state.yaw = static_cast<float>(odom_yaw_);
        }
        viewpoint_generator_->updateUAVState(uav_state);
        
        std::vector<FrontierCluster> clusters;
        std::vector<ExplorationViewpoint> viewpoints;
        bool success = viewpoint_generator_->generate(clusters, viewpoints);
        
        if (success && !viewpoints.empty()) {
            ROS_INFO("[ExplorationFSM] Found %lu new viewpoints, resuming exploration", viewpoints.size());
            logged = false;
            expl_data_.current_viewpoints = viewpoints;
            expl_data_.viewpoint_published = false;
            state_ = ExplorationState::PLAN_VIEWPOINTS;
            return;
        }
        
        // 5秒后返回等待状态
        if ((ros::Time::now() - finish_time).toSec() > 5.0) {
            triggered_ = false;
            logged = false;
            state_ = ExplorationState::WAIT_TRIGGER;
            ROS_INFO("[ExplorationFSM] Ready for new exploration");
        }
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
            uav_state.yaw = static_cast<float>(odom_yaw_);
        }
        viewpoint_generator_->updateUAVState(uav_state);
        
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 使用新的简化接口生成视点
        bool success = viewpoint_generator_->generate(
            frontier_clusters_,
            expl_data_.current_viewpoints);
        
        if (success) {
            // 选择第一个视点作为下一个目标（仅用于高亮显示）
            if (!expl_data_.current_viewpoints.empty()) {
                expl_data_.next_viewpoint = expl_data_.current_viewpoints[0];
            }
            
            ROS_INFO_THROTTLE(2.0, "[ExplorationFSM] Passive mode: %lu clusters, %lu viewpoints",
                             frontier_clusters_.size(), expl_data_.current_viewpoints.size());
        } else {
            ROS_DEBUG_THROTTLE(2.0, "[ExplorationFSM] No viewpoints generated");
        }
        
        // 更新上次状态
        last_frontier_subgrids_ = frontier_subgrids;
        last_total_frontiers_ = total_frontiers;
    }
    
    // 首次无前沿时也打印信息
    if (!has_frontier_ && frontier_subgrids.empty()) {
        ROS_INFO_THROTTLE(5.0, "[ExplorationFSM] No frontiers detected yet");
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
        uav_state.yaw = static_cast<float>(odom_yaw_);
    }
    viewpoint_generator_->updateUAVState(uav_state);
    
    has_frontier_ = true;
    return true;
}

bool ExplorationFSM::checkFrontierCovered() {
    // ★★★ Falcon 风格覆盖检测 ★★★
    // 检查当前目标簇是否已被覆盖（基于覆盖比例）
    // 如果覆盖比例超过阈值（默认25%），则触发重规划
    
    if (expl_data_.current_viewpoints.empty()) {
        return true;
    }
    
    const ExplorationViewpoint& vp = expl_data_.next_viewpoint;
    
    // 检查目标簇中心附近是否还有未知体素
    // 注意：使用 target_center（前沿簇中心）而不是 position（视点位置）
    Eigen::Vector3f target_pos = vp.target_center;
    
    // 如果 target_center 未设置（零向量），使用视点方向估计
    bool use_estimated = false;
    if (target_pos.norm() < 0.01f) {
        // 沿视点方向 3m 处估计目标位置
        float yaw = vp.yaw;
        target_pos = vp.position + 3.0f * Eigen::Vector3f(cos(yaw), sin(yaw), 0.0f);
        use_estimated = true;
    }
    
    float check_radius = 2.0f;
    
    // 统计当前未知体素数量
    int current_unknown_count = 0;
    int total_checked = 0;
    for (float dx = -check_radius; dx <= check_radius; dx += 0.5f) {
        for (float dy = -check_radius; dy <= check_radius; dy += 0.5f) {
            for (float dz = -check_radius; dz <= check_radius; dz += 0.5f) {
                Eigen::Vector3f check_pos = target_pos + Eigen::Vector3f(dx, dy, dz);
                if (!global_map_->isInGlobalMapRange(check_pos)) continue;
                
                total_checked++;
                GlobalVoxelState state = global_map_->getVoxelState(check_pos);
                if (VoxelStateBits::isUnknown(state)) {
                    current_unknown_count++;
                }
            }
        }
    }
    
    // ★★★ Falcon 风格：基于覆盖比例判断 ★★★
    // 如果是第一次检查，记录初始未知体素数量
    if (initial_unknown_count_ == 0) {
        initial_unknown_count_ = std::max(current_unknown_count, 1);  // 避免除零
    }
    
    // 计算变化数量（已被覆盖的体素）
    int covered_count = initial_unknown_count_ - current_unknown_count;
    float covered_ratio = static_cast<float>(covered_count) / initial_unknown_count_;
    
    // 判断是否已覆盖（变化比例超过阈值）
    // Falcon 默认使用 25% 的阈值
    bool is_covered = covered_ratio >= config_.frontier_covered_ratio;
    
    // 备选逻辑：如果剩余未知体素太少，也认为已覆盖
    const int min_remaining_threshold = 5;
    if (current_unknown_count < min_remaining_threshold) {
        is_covered = true;
    }
    
    ROS_DEBUG_THROTTLE(1.0, "[ExplorationFSM] checkFrontierCovered: target=(%.2f,%.2f,%.2f) %s, "
              "unknown=%d->%d (initial), covered_ratio=%.1f%%, threshold=%.1f%%, covered=%s",
              target_pos.x(), target_pos.y(), target_pos.z(),
              use_estimated ? "(estimated)" : "(from cluster)",
              initial_unknown_count_, current_unknown_count,
              covered_ratio * 100.0f, config_.frontier_covered_ratio * 100.0f,
              is_covered ? "YES" : "NO");
    
    return is_covered;
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
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 使用新的简化接口生成视点
    bool success = viewpoint_generator_->generate(
        frontier_clusters_,
        expl_data_.current_viewpoints);
    
    if (!success || expl_data_.current_viewpoints.empty()) {
        return false;
    }
    
    // 贪心排序策略（参考 EDEN 算法）：
    // 从当前无人机位置开始，每次选择距离最近的未访问视点
    // 这样可以减少飞行距离，提高探索效率
    sortViewpointsByGreedy();
    
    // 重置索引
    expl_data_.current_viewpoint_idx = 0;
    
    ROS_INFO("[ExplorationFSM] Planned %lu clusters, %lu viewpoints (greedy sorted)",
             frontier_clusters_.size(), expl_data_.current_viewpoints.size());
    
    return true;
}

bool ExplorationFSM::selectNextViewpoint() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 检查是否还有视点
    if (expl_data_.current_viewpoint_idx >= static_cast<int>(expl_data_.current_viewpoints.size())) {
        // 所有视点探索完成，需要重新规划
        return false;
    }
    
    // 选择当前视点
    expl_data_.next_viewpoint = expl_data_.current_viewpoints[expl_data_.current_viewpoint_idx];
    
    ROS_INFO("[ExplorationFSM] Selected viewpoint %d/%lu: (%.2f, %.2f, %.2f) yaw=%.2f",
             expl_data_.current_viewpoint_idx + 1,
             expl_data_.current_viewpoints.size(),
             expl_data_.next_viewpoint.position.x(),
             expl_data_.next_viewpoint.position.y(),
             expl_data_.next_viewpoint.position.z(),
             expl_data_.next_viewpoint.yaw);
    
    return true;
}

void ExplorationFSM::sortViewpointsByGreedy() {
    // 贪心排序策略（参考 EDEN 算法）：
    // 从当前无人机位置开始，每次选择距离最近的未访问视点
    
    auto& viewpoints = expl_data_.current_viewpoints;
    if (viewpoints.size() <= 1) {
        return;
    }
    
    // 获取当前无人机位置
    Eigen::Vector3d current_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
    }
    
    std::vector<ExplorationViewpoint> sorted;
    sorted.reserve(viewpoints.size());
    
    std::vector<bool> visited(viewpoints.size(), false);
    
    // 第一个视点：选择距离无人机当前位置最近的
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < viewpoints.size(); i++) {
        double dist = (viewpoints[i].position.cast<double>() - current_pos).norm();
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = static_cast<int>(i);
        }
    }
    sorted.push_back(viewpoints[best_idx]);
    visited[best_idx] = true;
    
    // 后续视点：每次选择距离上一个视点最近的未访问视点
    while (sorted.size() < viewpoints.size()) {
        Eigen::Vector3f last_pos = sorted.back().position;
        best_idx = -1;
        best_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < viewpoints.size(); i++) {
            if (visited[i]) continue;
            
            double dist = (viewpoints[i].position - last_pos).norm();
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = static_cast<int>(i);
            }
        }
        
        if (best_idx >= 0) {
            sorted.push_back(viewpoints[best_idx]);
            visited[best_idx] = true;
        } else {
            break;  // 防止无限循环
        }
    }
    
    // 更新视点列表
    viewpoints = std::move(sorted);
    
    ROS_DEBUG("[ExplorationFSM] Viewpoints sorted by greedy (from UAV pos: %.2f, %.2f, %.2f)",
              current_pos.x(), current_pos.y(), current_pos.z());
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
    visualizeHistoryNodes();
}

void ExplorationFSM::visualizeFrontierClusters() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    // 先清除旧marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 颜色列表
    std::vector<std::array<float, 4>> colors = {
        {1.0f, 0.0f, 0.0f, 0.8f},  // Red
        {0.0f, 1.0f, 0.0f, 0.8f},  // Green
        {0.0f, 0.0f, 1.0f, 0.8f},  // Blue
        {1.0f, 1.0f, 0.0f, 0.8f},  // Yellow
        {1.0f, 0.0f, 1.0f, 0.8f},  // Magenta
        {0.0f, 1.0f, 1.0f, 0.8f},  // Cyan
        {1.0f, 0.5f, 0.0f, 0.8f},  // Orange
        {0.5f, 0.0f, 1.0f, 0.8f},  // Purple
    };
    
    // 使用直接聚类的前沿簇（与前沿点云一致）
    for (size_t i = 0; i < frontier_clusters_.size(); ++i) {
        const auto& cluster = frontier_clusters_[i];
        auto& color = colors[i % colors.size()];
        
        // ========== 1. 簇范围（透明球体，表示覆盖区域） ==========
        visualization_msgs::Marker extent_marker;
        extent_marker.header.frame_id = "map";
        extent_marker.header.stamp = ros::Time::now();
        extent_marker.ns = "cluster_extent";
        extent_marker.id = marker_id++;
        extent_marker.type = visualization_msgs::Marker::SPHERE;
        extent_marker.action = visualization_msgs::Marker::ADD;
        extent_marker.pose.position.x = cluster.centroid.x();
        extent_marker.pose.position.y = cluster.centroid.y();
        extent_marker.pose.position.z = cluster.centroid.z();
        // 使用 extent 作为直径（如果 extent 太小则设为最小值）
        float diameter = std::max(cluster.extent * 2.0f, 0.5f);
        extent_marker.scale.x = diameter;
        extent_marker.scale.y = diameter;
        extent_marker.scale.z = diameter;
        extent_marker.color.r = color[0];
        extent_marker.color.g = color[1];
        extent_marker.color.b = color[2];
        extent_marker.color.a = 0.25f;  // 透明度较低，方便看到内部
        markers.markers.push_back(extent_marker);
        
        // ========== 2. 簇中心（小实心球体） ==========
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
        center_marker.color.r = color[0];
        center_marker.color.g = color[1];
        center_marker.color.b = color[2];
        center_marker.color.a = 1.0f;  // 完全不透明
        markers.markers.push_back(center_marker);
        
        // ========== 3. 簇ID文字标签 ==========
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "cluster_labels";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = cluster.centroid.x();
        text_marker.pose.position.y = cluster.centroid.y();
        text_marker.pose.position.z = cluster.centroid.z() + diameter * 0.5f + 0.3f;
        text_marker.scale.z = 0.4;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        // 显示簇ID和点数
        text_marker.text = "C" + std::to_string(i) + " (" + std::to_string(cluster.point_count) + ")";
        markers.markers.push_back(text_marker);
        
        // ========== 4. 前沿点（小立方体，不同簇不同颜色） ==========
        if (!cluster.points.empty()) {
            visualization_msgs::Marker points_marker;
            points_marker.header.frame_id = "map";
            points_marker.header.stamp = ros::Time::now();
            points_marker.ns = "frontier_points";
            points_marker.id = marker_id++;
            points_marker.type = visualization_msgs::Marker::CUBE_LIST;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.scale.x = 0.1;  // 10cm 立方体
            points_marker.scale.y = 0.1;
            points_marker.scale.z = 0.1;
            points_marker.color.r = color[0];
            points_marker.color.g = color[1];
            points_marker.color.b = color[2];
            points_marker.color.a = 0.9f;  // 较高不透明度
            
            points_marker.points.reserve(cluster.points.size());
            for (const auto& pt : cluster.points) {
                geometry_msgs::Point p;
                p.x = pt.x();
                p.y = pt.y();
                p.z = pt.z();
                points_marker.points.push_back(p);
            }
            
            markers.markers.push_back(points_marker);
        }
    }
    
    frontier_cluster_pub_.publish(markers);
}

void ExplorationFSM::visualizeViewpoints() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    // 先清除旧marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 使用直接生成的视点，按探索顺序添加序号标签
    int viewpoint_order = 1;  // 序号从1开始
    for (size_t i = 0; i < expl_data_.current_viewpoints.size(); ++i) {
            const auto& vp = expl_data_.current_viewpoints[i];
            if (!vp.is_valid) continue;
            
            // 判断视点状态
            bool is_current = (static_cast<int>(i) == expl_data_.current_viewpoint_idx);
            bool is_visited = (static_cast<int>(i) < expl_data_.current_viewpoint_idx);
            
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
            
            // 颜色：当前目标=绿色，已访问=灰色，待访问=蓝色
            if (is_current) {
                sphere.color.r = 0.0;
                sphere.color.g = 1.0;
                sphere.color.b = 0.0;
                sphere.color.a = 0.9;
            } else if (is_visited) {
                sphere.color.r = 0.5;
                sphere.color.g = 0.5;
                sphere.color.b = 0.5;
                sphere.color.a = 0.5;
            } else {
                sphere.color.r = 0.2;
                sphere.color.g = 0.4;
                sphere.color.b = 1.0;
                sphere.color.a = 0.8;
            }
            
            markers.markers.push_back(sphere);
            
            // 添加序号标签（显示探索顺序）
            visualization_msgs::Marker text;
            text.header.frame_id = "map";
            text.header.stamp = ros::Time::now();
            text.ns = "viewpoint_labels";
            text.id = marker_id++;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position.x = vp.position.x();
            text.pose.position.y = vp.position.y();
            text.pose.position.z = vp.position.z() + 0.5;  // 在视点上方显示
            text.scale.z = 0.4;  // 文字高度
            text.text = std::to_string(viewpoint_order);
            
            // 文字颜色
            if (is_current) {
                text.color.r = 0.0;
                text.color.g = 1.0;
                text.color.b = 0.0;
            } else if (is_visited) {
                text.color.r = 0.6;
                text.color.g = 0.6;
                text.color.b = 0.6;
            } else {
                text.color.r = 1.0;
                text.color.g = 1.0;
                text.color.b = 1.0;
            }
            text.color.a = 1.0;
            
            markers.markers.push_back(text);
            viewpoint_order++;
            
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
            
            // 箭头颜色与视点一致
            if (is_current) {
                arrow.color.r = 0.0;
                arrow.color.g = 1.0;
                arrow.color.b = 0.0;
            } else if (is_visited) {
                arrow.color.r = 0.5;
                arrow.color.g = 0.5;
                arrow.color.b = 0.5;
            } else {
                arrow.color.r = 1.0;
                arrow.color.g = 1.0;
                arrow.color.b = 0.0;
            }
            arrow.color.a = is_visited ? 0.5 : 1.0;
            
            markers.markers.push_back(arrow);
    }
    
    // 添加探索路径连线（显示探索顺序）
    if (expl_data_.current_viewpoints.size() > 1) {
        visualization_msgs::Marker route_line;
        route_line.header.frame_id = "map";
        route_line.header.stamp = ros::Time::now();
        route_line.ns = "exploration_route";
        route_line.id = marker_id++;
        route_line.type = visualization_msgs::Marker::LINE_STRIP;
        route_line.action = visualization_msgs::Marker::ADD;
        route_line.scale.x = 0.05;
        route_line.color.r = 1.0;
        route_line.color.g = 0.5;
        route_line.color.b = 0.0;
        route_line.color.a = 0.8;
        
        for (const auto& vp : expl_data_.current_viewpoints) {
            if (!vp.is_valid) continue;
            geometry_msgs::Point p;
            p.x = vp.position.x();
            p.y = vp.position.y();
            p.z = vp.position.z();
            route_line.points.push_back(p);
        }
        
        markers.markers.push_back(route_line);
    }
    
    viewpoint_marker_pub_.publish(markers);
}

void ExplorationFSM::visualizeFrustums() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    // 使用直接生成的视点
    for (size_t i = 0; i < expl_data_.current_viewpoints.size(); ++i) {
        const auto& vp = expl_data_.current_viewpoints[i];
        if (!vp.is_valid) continue;
        
        // 第一个视点用绿色，其他用蓝色
        std::array<float, 4> color;
        if (i == 0) {
            color = {0.0f, 1.0f, 0.0f, 0.6f};
        } else {
            color = {0.0f, 0.5f, 1.0f, 0.4f};
        }
        
        createFrustumMarker(vp.position, vp.yaw, vp.pitch,
                           config_.frustum_length,
                           config_.frustum_fov_h, config_.frustum_fov_v,
                           marker_id, color, markers);
        marker_id += 5;  // 每个视锥体使用5个marker
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
    // 简化后不再使用区域路由，发布空marker
    visualization_msgs::MarkerArray markers;
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
    ss << "Clusters: " << frontier_clusters_.size() << "\n";
    ss << "Viewpoints: " << expl_data_.current_viewpoint_idx + 1 << "/" << expl_data_.current_viewpoints.size() << "\n";
    ss << "Total Visited: " << expl_data_.total_viewpoints_visited;
    
    text.text = ss.str();
    markers.markers.push_back(text);
    
    info_marker_pub_.publish(markers);
}

void ExplorationFSM::visualizeHistoryNodes() {
    if (!viewpoint_generator_) return;
    
    const auto& history_nodes = viewpoint_generator_->getHistoryNodes();
    const auto& associated_subgrids = viewpoint_generator_->getAssociatedSubgrids();
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    // 清除旧marker
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.ns = "history_nodes";
    markers.markers.push_back(delete_marker);
    
    delete_marker.ns = "history_node_ids";
    markers.markers.push_back(delete_marker);
    
    delete_marker.ns = "history_path";
    markers.markers.push_back(delete_marker);
    
    delete_marker.ns = "associated_subgrids";
    markers.markers.push_back(delete_marker);
    
    delete_marker.ns = "node_subgrid_links";
    markers.markers.push_back(delete_marker);
    
    // 颜色表（不同历史节点使用不同颜色）
    static const std::vector<std::array<float, 4>> node_colors = {
        {1.0f, 0.2f, 0.2f, 0.4f},   // 红色
        {0.2f, 1.0f, 0.2f, 0.4f},   // 绿色
        {0.2f, 0.2f, 1.0f, 0.4f},   // 蓝色
        {1.0f, 1.0f, 0.2f, 0.4f},   // 黄色
        {1.0f, 0.2f, 1.0f, 0.4f},   // 品红
        {0.2f, 1.0f, 1.0f, 0.4f},   // 青色
        {1.0f, 0.6f, 0.2f, 0.4f},   // 橙色
        {0.6f, 0.2f, 1.0f, 0.4f},   // 紫色
    };
    
    // 绘制关联的子网格区域（立方体线框）
    for (const auto& asg : associated_subgrids) {
        if (asg.history_node_id < 0) continue;  // 跳过未关联的
        
        // 获取该节点对应的颜色
        int color_idx = asg.history_node_id % static_cast<int>(node_colors.size());
        const auto& color = node_colors[color_idx];
        
        // 绘制子网格边界框（使用LINE_LIST绘制立方体线框）
        visualization_msgs::Marker cube;
        cube.header.frame_id = "map";
        cube.header.stamp = ros::Time::now();
        cube.ns = "associated_subgrids";
        cube.id = marker_id++;
        cube.type = visualization_msgs::Marker::LINE_LIST;
        cube.action = visualization_msgs::Marker::ADD;
        
        cube.scale.x = 0.03;  // 线宽
        cube.color.r = color[0];
        cube.color.g = color[1];
        cube.color.b = color[2];
        cube.color.a = 0.8f;
        
        // 立方体的8个顶点
        Eigen::Vector3f bmin = asg.bbox_min;
        Eigen::Vector3f bmax = asg.bbox_max;
        
        // 稍微扩展边界框以便可视化
        bmin -= Eigen::Vector3f(0.02f, 0.02f, 0.02f);
        bmax += Eigen::Vector3f(0.02f, 0.02f, 0.02f);
        
        std::vector<geometry_msgs::Point> vertices(8);
        vertices[0].x = bmin.x(); vertices[0].y = bmin.y(); vertices[0].z = bmin.z();
        vertices[1].x = bmax.x(); vertices[1].y = bmin.y(); vertices[1].z = bmin.z();
        vertices[2].x = bmax.x(); vertices[2].y = bmax.y(); vertices[2].z = bmin.z();
        vertices[3].x = bmin.x(); vertices[3].y = bmax.y(); vertices[3].z = bmin.z();
        vertices[4].x = bmin.x(); vertices[4].y = bmin.y(); vertices[4].z = bmax.z();
        vertices[5].x = bmax.x(); vertices[5].y = bmin.y(); vertices[5].z = bmax.z();
        vertices[6].x = bmax.x(); vertices[6].y = bmax.y(); vertices[6].z = bmax.z();
        vertices[7].x = bmin.x(); vertices[7].y = bmax.y(); vertices[7].z = bmax.z();
        
        // 12条边
        int edges[12][2] = {
            {0,1}, {1,2}, {2,3}, {3,0},  // 底面
            {4,5}, {5,6}, {6,7}, {7,4},  // 顶面
            {0,4}, {1,5}, {2,6}, {3,7}   // 侧面
        };
        
        for (int i = 0; i < 12; ++i) {
            cube.points.push_back(vertices[edges[i][0]]);
            cube.points.push_back(vertices[edges[i][1]]);
        }
        
        markers.markers.push_back(cube);
        
        // 绘制从历史节点到子网格中心的连线
        visualization_msgs::Marker link;
        link.header.frame_id = "map";
        link.header.stamp = ros::Time::now();
        link.ns = "node_subgrid_links";
        link.id = marker_id++;
        link.type = visualization_msgs::Marker::LINE_STRIP;
        link.action = visualization_msgs::Marker::ADD;
        
        link.scale.x = 0.02;
        link.color.r = color[0];
        link.color.g = color[1];
        link.color.b = color[2];
        link.color.a = 0.3f;
        
        // 找到对应的历史节点位置
        for (const auto& node : history_nodes) {
            if (node.node_id == asg.history_node_id) {
                geometry_msgs::Point p1, p2;
                p1.x = node.position.x();
                p1.y = node.position.y();
                p1.z = node.position.z();
                p2.x = asg.center.x();
                p2.y = asg.center.y();
                p2.z = asg.center.z();
                link.points.push_back(p1);
                link.points.push_back(p2);
                break;
            }
        }
        
        if (link.points.size() == 2) {
            markers.markers.push_back(link);
        }
    }
    
    // 绘制历史节点（球体，使用与子网格相同的颜色）
    for (const auto& node : history_nodes) {
        int color_idx = node.node_id % static_cast<int>(node_colors.size());
        const auto& color = node_colors[color_idx];
        
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = ros::Time::now();
        sphere.ns = "history_nodes";
        sphere.id = marker_id++;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        
        sphere.pose.position.x = node.position.x();
        sphere.pose.position.y = node.position.y();
        sphere.pose.position.z = node.position.z();
        sphere.pose.orientation.w = 1.0;
        
        sphere.scale.x = 0.4;
        sphere.scale.y = 0.4;
        sphere.scale.z = 0.4;
        
        // 使用与子网格相同的颜色，但更不透明
        sphere.color.r = color[0];
        sphere.color.g = color[1];
        sphere.color.b = color[2];
        sphere.color.a = 0.9f;
        
        markers.markers.push_back(sphere);
        
        // 节点ID文本
        visualization_msgs::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = ros::Time::now();
        text.ns = "history_node_ids";
        text.id = marker_id++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        
        text.pose.position.x = node.position.x();
        text.pose.position.y = node.position.y();
        text.pose.position.z = node.position.z() + 0.5;
        
        text.scale.z = 0.3;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        text.text = "N" + std::to_string(node.node_id);
        
        markers.markers.push_back(text);
    }
    
    // 绘制历史节点之间的连线（轨迹）
    if (history_nodes.size() > 1) {
        visualization_msgs::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = ros::Time::now();
        line.ns = "history_path";
        line.id = marker_id++;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        
        line.scale.x = 0.08;
        line.color.r = 0.8f;
        line.color.g = 0.8f;
        line.color.b = 0.8f;
        line.color.a = 0.6f;
        
        for (const auto& node : history_nodes) {
            geometry_msgs::Point p;
            p.x = node.position.x();
            p.y = node.position.y();
            p.z = node.position.z();
            line.points.push_back(p);
        }
        
        markers.markers.push_back(line);
    }
    
    route_marker_pub_.publish(markers);  // 复用route_marker_pub
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
    // 使用 quadrotor_msgs::PositionCommand 发送给 EGO-Planner
    // 包含位置、yaw 角和期望速度
    quadrotor_msgs::PositionCommand msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    
    // 位置
    msg.position.x = vp.position.x();
    msg.position.y = vp.position.y();
    msg.position.z = vp.position.z();
    
    // yaw 角 - EGO-Planner 会使用这个值规划偏航角
    msg.yaw = vp.yaw;
    msg.yaw_dot = 0.0;
    
    // 期望终端速度（设为0，即到达视点后悬停）
    msg.velocity.x = 0.0;
    msg.velocity.y = 0.0;
    msg.velocity.z = 0.0;
    
    // 加速度设为0
    msg.acceleration.x = 0.0;
    msg.acceleration.y = 0.0;
    msg.acceleration.z = 0.0;
    
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

