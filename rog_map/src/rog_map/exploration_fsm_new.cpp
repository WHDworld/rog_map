/**
 * @file exploration_fsm_new.cpp
 * @brief 探索状态机实现 - 重构版（4状态 + 集成规划器）
 */

#include "rog_map/exploration_fsm_new.h"
#include <tf/transform_datatypes.h>

// 简化日志宏
#define LOG_INFO(msg) ROS_INFO_STREAM("[FSM_NEW] " << msg)
#define LOG_WARN(msg) ROS_WARN_STREAM("[FSM_NEW] " << msg)
#define LOG_DEBUG(msg) ROS_DEBUG_STREAM("[FSM_NEW] " << msg)

namespace rog_map {

ExplorationFSMNew::ExplorationFSMNew() {}

ExplorationFSMNew::~ExplorationFSMNew() {
    fsm_timer_.stop();
}

void ExplorationFSMNew::init(ros::NodeHandle& nh, MapManager::Ptr map_manager) {
    nh_ = nh;
    map_manager_ = map_manager;
    global_map_ = map_manager_->getGlobalMap();
    
    // 加载配置（使用 fsm/ 命名空间，与 yaml 文件一致）
    nh_.param("fsm/fsm_frequency", config_.fsm_frequency, 100.0);
    nh_.param("fsm/takeoff_timeout", config_.takeoff_timeout, 10.0);
    nh_.param("fsm/init_wait_time", config_.init_wait_time, 2.0);
    nh_.param("fsm/reach_threshold", config_.reach_threshold, 0.5);
    nh_.param("fsm/yaw_threshold", config_.yaw_threshold, 0.3);
    nh_.param("fsm/replan_check_interval", config_.replan_check_interval, 0.5);
    nh_.param("fsm/waypoint_approach_dist", config_.waypoint_approach_dist, 1.5);
    nh_.param("fsm/trajectory_timeout", config_.trajectory_timeout, 10.0);
    nh_.param("fsm/coverage_check_interval", config_.coverage_check_interval, 1.0);
    nh_.param("fsm/coverage_trigger_ratio", config_.coverage_trigger_ratio, 0.3);
    nh_.param("fsm/collision_check_interval", config_.collision_check_interval, 0.05);
    nh_.param("fsm/collision_lookahead", config_.collision_lookahead, 0.5);
    nh_.param("fsm/max_viewpoint_distance", config_.max_viewpoint_distance, 15.0);
    nh_.param("fsm/close_viewpoint_threshold", config_.close_viewpoint_threshold, 5.0);
    nh_.param("fsm/auto_start", config_.auto_start, false);
    nh_.param("fsm/enable_visualization", config_.enable_visualization, true);

    nh_.param("fsm/planning_horizon", config_.planning_horizon, 7.5);
    nh_.param("fsm/takeoff_height", config_.takeoff_height, 1.0);

    nh_.param("manager/max_vel", config_.max_vel, 1.5);
    nh_.param("manager/max_acc", config_.max_acc, 3.0);
    nh_.param("manager/max_yaw_rate", config_.max_yaw_rate, 1.0);
    nh_.param("manager/max_yaw_acc", config_.max_yaw_acc, 1.0);
    nh_.param<std::string>("rog_map/lidar_topic", config_.lidar_topic, "/cloud_registered");
    nh_.param<std::string>("rog_map/odom_topic", config_.odom_topic, "/uav_simulator/odometry");

    
    // 创建模块
    history_graph_ = std::make_shared<HistoryGraph>();
    history_graph_->init(nh_, global_map_);
    
    viewpoint_manager_ = std::make_shared<ViewpointManager>();
    viewpoint_manager_->init(nh_, global_map_, history_graph_);
    
    // 设置HistoryGraph的ViewpointManager指针（用于轮询视点更新）
    history_graph_->setViewpointManager(viewpoint_manager_.get());
    
    // 创建规划器管理器
    planner_manager_.reset(new ego_planner::EGOPlannerManager);
    planner_manager_->initPlanModules(nh_, map_manager_);
    
    // 订阅者
    odom_sub_ = nh_.subscribe(config_.odom_topic, 1, &ExplorationFSMNew::odometryCallback, this);
    trigger_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ExplorationFSMNew::triggerCallback, this);
    
    // 发布者
    pos_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
    exploration_state_pub_ = nh_.advertise<std_msgs::Int8>("/exploration/state", 10);
    
    // 可视化发布者（使用与老架构相同的话题名称）
    frontier_cluster_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frontier_clusters", 1);
    viewpoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/viewpoints", 1);
    frustum_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frustums", 1);
    info_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/info", 1);
    path_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/path", 1);
    viewpoint_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/viewpoints", 1);
    
    // 定时器
    fsm_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.fsm_frequency),
                                  &ExplorationFSMNew::fsmCallback, this);
    
    // 初始化状态
    state_ = ExplorationState::INIT;
    fsm_init_time_ = ros::Time::now();
    
    LOG_INFO("ExplorationFSMNew initialized with 4-state design");
}

std::string ExplorationFSMNew::getStateString() const {
    switch (state_.load()) {
        case ExplorationState::INIT: return "INIT";
        case ExplorationState::WAIT_TRIGGER: return "WAIT_TRIGGER";
        case ExplorationState::LOCALPLAN: return "LOCALPLAN";
        case ExplorationState::EXEC_EXPLORATION: return "EXEC_EXPLORATION";
        default: return "UNKNOWN";
    }
}

void ExplorationFSMNew::transitState(ExplorationState new_state, const std::string& reason) {
    std::string old_state_str = getStateString();
    state_ = new_state;
    
    std::string new_state_str;
    switch (new_state) {
        case ExplorationState::INIT: new_state_str = "INIT"; break;
        case ExplorationState::WAIT_TRIGGER: new_state_str = "WAIT_TRIGGER"; break;
        case ExplorationState::LOCALPLAN: new_state_str = "LOCALPLAN"; break;
        case ExplorationState::EXEC_EXPLORATION: new_state_str = "EXEC_EXPLORATION"; break;
        default: new_state_str = "UNKNOWN"; break;
    }
    
    ROS_INFO("[FSM_NEW] TRANSIT: %s -> %s (%s)", 
             old_state_str.c_str(), new_state_str.c_str(), reason.c_str());
    
    // 发布状态
    std_msgs::Int8 state_msg;
    state_msg.data = static_cast<int8_t>(new_state);
    exploration_state_pub_.publish(state_msg);
}

void ExplorationFSMNew::triggerExploration() {
    triggered_ = true;
}

void ExplorationFSMNew::stopExploration() {
    triggered_ = false;
    transitState(ExplorationState::WAIT_TRIGGER, "Manual stop");
}

// ==================== 回调函数 ====================

void ExplorationFSMNew::fsmCallback(const ros::TimerEvent& e) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    switch (state_.load()) {
        case ExplorationState::INIT:
            processInit();
            break;
        case ExplorationState::WAIT_TRIGGER:
            processWaitTrigger();
            break;
        case ExplorationState::LOCALPLAN:
            processLocalPlan();
            break;
        case ExplorationState::EXEC_EXPLORATION:
            processExecExploration();
            break;
    }
    
    // 可视化
    if (config_.enable_visualization) {
        visualize();
    }
}

void ExplorationFSMNew::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    odom_pos_.x() = msg->pose.pose.position.x;
    odom_pos_.y() = msg->pose.pose.position.y;
    odom_pos_.z() = msg->pose.pose.position.z;
    
    odom_vel_.x() = msg->twist.twist.linear.x;
    odom_vel_.y() = msg->twist.twist.linear.y;
    odom_vel_.z() = msg->twist.twist.linear.z;
    
    odom_yaw_ = tf::getYaw(msg->pose.pose.orientation);
    
    if (!have_odom_) {
        initial_pos_ = odom_pos_;
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        have_odom_ = true;
        ROS_INFO("[FSM_NEW] FIRST_ODOM: pos=(%.2f,%.2f,%.2f)",
                 odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
    }
    
    // 更新模块的UAV状态
    Eigen::Vector3f pos_f = odom_pos_.cast<float>();
    history_graph_->updateUAVPosition(pos_f);
    viewpoint_manager_->updateUAVState(pos_f, static_cast<float>(odom_yaw_));
}

void ExplorationFSMNew::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    LOG_INFO("Received exploration trigger");
    triggered_ = true;
}

// ==================== 状态处理函数 ====================

void ExplorationFSMNew::processInit() {
    if (!have_odom_) {
        ROS_INFO_THROTTLE(2.0, "[FSM_NEW] INIT: Waiting for odometry...");
        return;
    }
    
    double elapsed = (ros::Time::now() - fsm_init_time_).toSec();
    
    // 检查起飞状态
    if (!takeoff_complete_) {
        // 发布起飞命令
        if (elapsed < 0.5) {
            takeoff_start_time_ = ros::Time::now();
            ROS_INFO("[FSM_NEW] INIT: Starting takeoff to height %.2f", config_.takeoff_height);
        }
        
        publishTakeoffCmd();
        
        // 检查是否到达起飞高度
        double height_diff = odom_pos_.z() - initial_pos_.z();
        ROS_INFO_THROTTLE(1.0, "[FSM_NEW] TAKEOFF: z=%.2f, initial_z=%.2f, diff=%.2f, target=%.2f",
                          odom_pos_.z(), initial_pos_.z(), height_diff, config_.takeoff_height);
        
        if (height_diff > config_.takeoff_height - 0.1) {
            takeoff_complete_ = true;
            hover_pos_ = odom_pos_;
            hover_yaw_ = odom_yaw_;
            ROS_INFO("[FSM_NEW] TAKEOFF_DONE: Reached height %.2f", odom_pos_.z());
        }
        
        // 起飞超时
        double takeoff_elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
        if (takeoff_elapsed > config_.takeoff_timeout) {
            LOG_WARN("Takeoff timeout, proceeding anyway");
            takeoff_complete_ = true;
            hover_pos_ = odom_pos_;
            hover_yaw_ = odom_yaw_;
        }
        
        return;
    }
    
    // 等待初始化时间
    if (elapsed < config_.init_wait_time) {
        publishHoverCmd();
        return;
    }
    
    // 启动模块
    history_graph_->start();
    viewpoint_manager_->start();
    
    transitState(ExplorationState::WAIT_TRIGGER, "Init complete");
}

void ExplorationFSMNew::processWaitTrigger() {
    publishHoverCmd();
    
    static bool first_wait = true;
    if (first_wait) {
        ROS_INFO("[FSM_NEW] WAIT_TRIGGER: Hovering at (%.2f, %.2f, %.2f), waiting for trigger...",
                 hover_pos_.x(), hover_pos_.y(), hover_pos_.z());
        ROS_INFO("[FSM_NEW] Send goal to /move_base_simple/goal to start exploration, or set auto_start=true");
        first_wait = false;
    }
    
    if (config_.auto_start) {
        triggered_ = true;
    }
    
    if (triggered_) {
        // 检查是否有有效视点
        int vp_count = viewpoint_manager_->getViewpointCount();
        ROS_INFO_THROTTLE(2.0, "[FSM_NEW] WAIT_TRIGGER: Triggered, checking viewpoints (%d available)...", vp_count);
        
        if (viewpoint_manager_->hasValidViewpoints()) {
            transitState(ExplorationState::LOCALPLAN, "Triggered with viewpoints");
        } else {
            // 没有视点，等待视点生成
            LOG_DEBUG("Waiting for viewpoints...");
        }
    }
}

void ExplorationFSMNew::processLocalPlan() {
    ros::Time t_start = ros::Time::now();
    
    // Step 1: 选择最佳视点
    ros::Time t_select_start = ros::Time::now();
    int selected_vp_id = -1;
    if (!selectBestViewpoint(selected_vp_id)) {
        double t_select = (ros::Time::now() - t_select_start).toSec() * 1000.0;
        ROS_WARN_THROTTLE(2.0, "[FSM_NEW] LOCALPLAN: No valid viewpoint found (select took %.1fms)", t_select);
        
        // 检查是否还有前沿
        if (!viewpoint_manager_->hasValidViewpoints()) {
            LOG_INFO("EXPLORE_DONE: No more viewpoints, exploration complete!");
            transitState(ExplorationState::WAIT_TRIGGER, "Exploration complete");
        }
        
        publishHoverCmd();
        return;
    }
    double t_select = (ros::Time::now() - t_select_start).toSec() * 1000.0;
    
    // 获取视点信息用于打印
    Eigen::Vector3f vp_pos;
    float vp_yaw = 0;
    if (history_graph_->getViewpointInfo(selected_vp_id, vp_pos, vp_yaw)) {
        ROS_INFO("[FSM_NEW] LOCALPLAN: Selected viewpoint %d at (%.2f,%.2f,%.2f), yaw=%.2f",
                 selected_vp_id, vp_pos.x(), vp_pos.y(), vp_pos.z(), vp_yaw);
    }
    
    // Step 2: 规划到视点的路径
    ros::Time t_plan_start = ros::Time::now();
    if (!planToViewpoint(selected_vp_id)) {
        double t_plan = (ros::Time::now() - t_plan_start).toSec() * 1000.0;
        ROS_WARN("[FSM_NEW] LOCALPLAN: Failed to plan to viewpoint %d (plan took %.1fms)", 
                 selected_vp_id, t_plan);
        
        // 标记视点失效，尝试其他视点
        history_graph_->markViewpointInvalid(selected_vp_id);
        
        publishHoverCmd();
        return;
    }
    double t_plan = (ros::Time::now() - t_plan_start).toSec() * 1000.0;
    
    // Step 3: 设置当前目标并进入执行状态
    current_viewpoint_id_ = selected_vp_id;
    current_waypoint_idx_ = 0;
    trajectory_start_time_ = ros::Time::now();
    last_replan_check_time_ = ros::Time::now();
    last_coverage_check_time_ = ros::Time::now();
    last_collision_check_time_ = ros::Time::now();
    has_trajectory_ = true;
    
    double t_total = (ros::Time::now() - t_start).toSec() * 1000.0;
    ROS_INFO("[FSM_NEW] LOCALPLAN: Success! select=%.1fms, plan=%.1fms, total=%.1fms | waypoints=%zu",
             t_select, t_plan, t_total, waypoint_queue_.size());
    
    transitState(ExplorationState::EXEC_EXPLORATION, "Plan success");
}

void ExplorationFSMNew::processExecExploration() {
    if (!has_trajectory_) {
        transitState(ExplorationState::LOCALPLAN, "No trajectory");
        return;
    }
    
    ros::Time now = ros::Time::now();
    
    // 定期打印执行状态
    static ros::Time last_status_time = ros::Time(0);
    if ((now - last_status_time).toSec() > 2.0) {
        last_status_time = now;
        double traj_elapsed = (now - trajectory_start_time_).toSec();
        
        Eigen::Vector3d target_pos(0, 0, 0);
        if (!waypoint_queue_.empty() && current_waypoint_idx_ < static_cast<int>(waypoint_queue_.size())) {
            target_pos = waypoint_queue_[current_waypoint_idx_].position;
        }
        
        double dist_to_target = (target_pos - odom_pos_).norm();
        
        ROS_INFO("[FSM_NEW] EXEC: vp=%d, wp=%d/%zu, dist=%.2f, elapsed=%.1fs | pos=(%.2f,%.2f,%.2f)",
                 current_viewpoint_id_, current_waypoint_idx_, waypoint_queue_.size(),
                 dist_to_target, traj_elapsed,
                 odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
    }
    
    // 1. 碰撞检测
    if ((now - last_collision_check_time_).toSec() > config_.collision_check_interval) {
        last_collision_check_time_ = now;
        if (checkCollision()) {
            LOG_WARN("Collision detected, replanning");
            has_trajectory_ = false;
            transitState(ExplorationState::LOCALPLAN, "Collision");
            return;
        }
    }
    
    // 2. 检查到达当前航点
    if (checkReachWaypoint()) {
        current_waypoint_idx_++;
        
        // 检查是否到达最终视点
        if (current_waypoint_idx_ >= static_cast<int>(waypoint_queue_.size())) {
            // 到达视点
            ROS_INFO("[FSM_NEW] REACHED_VP: Viewpoint %d reached", current_viewpoint_id_);
            total_viewpoints_visited_++;
            
            // 标记视点已探索
            history_graph_->markViewpointExplored(current_viewpoint_id_);
            
            // 返回规划下一个视点
            current_viewpoint_id_ = -1;
            waypoint_queue_.clear();
            has_trajectory_ = false;
            transitState(ExplorationState::LOCALPLAN, "Viewpoint reached");
            return;
        }
        
        // 规划到下一个航点
        if (!planNextWaypoint()) {
            LOG_WARN("Failed to plan next waypoint");
            has_trajectory_ = false;
            transitState(ExplorationState::LOCALPLAN, "Waypoint plan failed");
            return;
        }
        
        trajectory_start_time_ = ros::Time::now();
    }
    
    // 3. 检查接近当前航点（提前规划下一段）
    if (checkApproachingWaypoint()) {
        // 可以在这里提前启动下一段规划
    }
    
    // 4. 检查覆盖（目标视点是否已被探索）
    if ((now - last_coverage_check_time_).toSec() > config_.coverage_check_interval) {
        last_coverage_check_time_ = now;
        if (checkCoverage()) {
            LOG_INFO("COVERAGE: Viewpoint covered, replanning");
            history_graph_->markViewpointExplored(current_viewpoint_id_);
            has_trajectory_ = false;
            transitState(ExplorationState::LOCALPLAN, "Covered");
            return;
        }
    }
    
    // 5. 检查超时
    if (checkTimeout()) {
        LOG_WARN("Trajectory timeout, replanning");
        has_trajectory_ = false;
        transitState(ExplorationState::LOCALPLAN, "Timeout");
        return;
    }
    
    // 6. 发布轨迹命令
    publishTrajectoryCmd();
}

// ==================== LOCALPLAN 核心方法 ====================

bool ExplorationFSMNew::selectBestViewpoint(int& selected_viewpoint_id) {
    ros::Time t_start = ros::Time::now();
    
    std::vector<ViewpointNode> viewpoints = history_graph_->getValidViewpoints();
    
    if (viewpoints.empty()) {
        return false;
    }
    
    Eigen::Vector3d current_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
    }
    
    Eigen::Vector3f current_pos_f = current_pos.cast<float>();
    
    // 按路径距离排序（考虑历史节点绕行）
    float best_cost = std::numeric_limits<float>::max();
    int best_id = -1;
    int checked = 0, reachable = 0;
    
    for (const auto& vp : viewpoints) {
        checked++;
        // 估算路径长度（使用直线距离，避免在选择阶段调用耗时的寻路）
        float direct_dist = (vp.position - current_pos_f).norm();
        
        if (direct_dist > config_.max_viewpoint_distance) {
            continue;
        }
        
        reachable++;
        
        // 综合代价：直线距离 - 评分加成
        float cost = direct_dist - vp.score * 2.0f;
        
        if (cost < best_cost) {
            best_cost = cost;
            best_id = vp.viewpoint_id;
        }
    }
    
    double t_elapsed = (ros::Time::now() - t_start).toSec() * 1000.0;
    
    if (best_id < 0) {
        ROS_WARN_THROTTLE(1.0, "[SELECT_VP] No valid viewpoint: checked=%d, reachable=%d, time=%.1fms",
                          checked, reachable, t_elapsed);
        return false;
    }
    
    selected_viewpoint_id = best_id;
    ROS_INFO("[FSM_NEW] SELECT_VP: Selected viewpoint %d, cost=%.2f (checked=%d in %.1fms)", 
             best_id, best_cost, checked, t_elapsed);
    
    return true;
}

bool ExplorationFSMNew::planToViewpoint(int viewpoint_id) {
    ros::Time t_start = ros::Time::now();
    
    const ViewpointNode* vp = history_graph_->getViewpoint(viewpoint_id);
    if (!vp) {
        return false;
    }
    
    Eigen::Vector3d current_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
    }
    
    Eigen::Vector3f current_pos_f = current_pos.cast<float>();
    
    // 清空航点队列
    waypoint_queue_.clear();
    
    // 查找路径（直接使用视点位置，简化路径）
    ros::Time t_path_start = ros::Time::now();
    
    // 简化：直接规划到视点，不经过历史节点
    // 这避免了复杂的路径查找
    WaypointInfo wp;
    wp.position = vp->position.cast<double>();
    wp.yaw = vp->yaw;
    wp.is_viewpoint = true;
    wp.id = viewpoint_id;
    waypoint_queue_.push_back(wp);
    
    double t_path = (ros::Time::now() - t_path_start).toSec() * 1000.0;
    
    ROS_INFO("[FSM_NEW] PATH_BUILT: Path with %zu waypoints", waypoint_queue_.size());
    
    // 重置航点索引并规划到第一个航点
    current_waypoint_idx_ = 0;
    
    ros::Time t_plan_start = ros::Time::now();
    bool success = planNextWaypoint();
    double t_plan = (ros::Time::now() - t_plan_start).toSec() * 1000.0;
    
    if (!success) {
        ROS_WARN_THROTTLE(0.5, "[PLAN_PERF] planNextWaypoint FAILED: path=%.1fms, plan=%.1fms", t_path, t_plan);
    }
    
    return success;
}

bool ExplorationFSMNew::planNextWaypoint() {
    if (current_waypoint_idx_ >= static_cast<int>(waypoint_queue_.size())) {
        return false;
    }
    
    const WaypointInfo& wp = waypoint_queue_[current_waypoint_idx_];
    
    // 确定目标速度
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
    
    // 如果不是最后一个航点，可以设置飞越速度
    if (current_waypoint_idx_ + 1 < static_cast<int>(waypoint_queue_.size()) && !wp.is_viewpoint) {
        // 计算飞越方向
        const WaypointInfo& next_wp = waypoint_queue_[current_waypoint_idx_ + 1];
        Eigen::Vector3d dir = next_wp.position - wp.position;
        dir.normalize();
        target_vel = dir * config_.max_vel * 0.3;  // 30%速度飞越
    }
    
    return callPlanner(wp.position, wp.yaw, target_vel);
}

bool ExplorationFSMNew::callPlanner(const Eigen::Vector3d& target, double target_yaw,
                                     const Eigen::Vector3d& target_vel) {
    Eigen::Vector3d start_pos, start_vel;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        start_pos = odom_pos_;
        start_vel = odom_vel_;
    }
    
    // 检查规划器是否初始化
    if (!planner_manager_) {
        ROS_ERROR("[PLAN_DEBUG] planner_manager_ is NULL!");
        return false;
    }
    
    // 调用规划（使用正确的接口签名）
    // reboundReplan(start_pt, start_vel, start_acc, end_pt, end_vel, 
    //               flag_polyInit, flag_randomPolyTraj, touch_goal)
    bool success = planner_manager_->reboundReplan(
        start_pos, start_vel, Eigen::Vector3d::Zero(),  // start
        target, target_vel,                              // end
        true,    // flag_polyInit
        false,   // flag_randomPolyTraj
        true);   // touch_goal
    
    return success;
}

// ==================== EXEC_EXPLORATION 检测方法 ====================

bool ExplorationFSMNew::checkCollision() {
    if (!planner_manager_->traj_.local_traj.traj_id) {
        return false;
    }
    
    double t_cur = ros::Time::now().toSec() - planner_manager_->traj_.local_traj.start_time;
    double duration = planner_manager_->traj_.local_traj.duration;
    
    if (t_cur < 0 || t_cur > duration) {
        return false;
    }
    
    // 检查当前位置 + 前瞻时间
    for (double dt = 0; dt <= config_.collision_lookahead; dt += 0.1) {
        double check_t = t_cur + dt;
        if (check_t > duration) break;
        
        Eigen::Vector3d pos = planner_manager_->traj_.local_traj.traj.getPos(check_t);
        
        if (global_map_->isOccupiedInflate(pos.cast<float>())) {
            ROS_ERROR("[FSM_NEW] COLLISION: at t=%.2f, pos=(%.2f,%.2f,%.2f)",
                      check_t, pos.x(), pos.y(), pos.z());
            return true;
        }
    }
    
    return false;
}

bool ExplorationFSMNew::checkReachWaypoint() {
    if (current_waypoint_idx_ >= static_cast<int>(waypoint_queue_.size())) {
        return false;
    }
    
    const WaypointInfo& wp = waypoint_queue_[current_waypoint_idx_];
    
    Eigen::Vector3d current_pos;
    double current_yaw;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
        current_yaw = odom_yaw_;
    }
    
    double dist = (current_pos - wp.position).norm();
    double yaw_diff = std::abs(getYawDiff(current_yaw, wp.yaw));
    
    // 对于中间节点，只检查位置
    if (!wp.is_viewpoint) {
        return dist < config_.reach_threshold;
    }
    
    // 对于视点，检查位置和朝向
    return dist < config_.reach_threshold && yaw_diff < config_.yaw_threshold;
}

bool ExplorationFSMNew::checkCoverage() {
    if (current_viewpoint_id_ < 0) {
        return false;
    }
    
    const ViewpointNode* vp = history_graph_->getViewpoint(current_viewpoint_id_);
    if (!vp) {
        return true;  // 视点不存在，视为已覆盖
    }
    
    // 检查视点目标区域的未知体素是否减少
    // 简化实现：检查视点周围是否仍有足够的前沿
    // TODO: 更精确的覆盖检测
    
    return false;
}

bool ExplorationFSMNew::checkTimeout() {
    double elapsed = (ros::Time::now() - trajectory_start_time_).toSec();
    return elapsed > config_.trajectory_timeout;
}

bool ExplorationFSMNew::checkApproachingWaypoint() {
    if (current_waypoint_idx_ >= static_cast<int>(waypoint_queue_.size())) {
        return false;
    }
    
    const WaypointInfo& wp = waypoint_queue_[current_waypoint_idx_];
    
    Eigen::Vector3d current_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
    }
    
    double dist = (current_pos - wp.position).norm();
    return dist < config_.waypoint_approach_dist;
}

// ==================== 辅助函数 ====================

void ExplorationFSMNew::publishTakeoffCmd() {
    double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
    double takeoff_time = 2.0;  // 2秒起飞
    
    double t = std::min(elapsed, takeoff_time);
    double progress = t / takeoff_time;
    
    // 平滑起飞
    double z_offset = config_.takeoff_height * (3 * progress * progress - 2 * progress * progress * progress);
    
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "map";
    
    cmd.position.x = initial_pos_.x();
    cmd.position.y = initial_pos_.y();
    cmd.position.z = initial_pos_.z() + z_offset;
    
    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = (elapsed < takeoff_time) ? config_.takeoff_height / takeoff_time * 0.5 : 0;
    
    cmd.yaw = hover_yaw_;
    
    pos_cmd_pub_.publish(cmd);
}

void ExplorationFSMNew::publishHoverCmd() {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "map";
    
    cmd.position.x = hover_pos_.x();
    cmd.position.y = hover_pos_.y();
    cmd.position.z = hover_pos_.z();
    
    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = 0;
    
    cmd.yaw = hover_yaw_;
    
    pos_cmd_pub_.publish(cmd);
}

void ExplorationFSMNew::publishTrajectoryCmd() {
    if (!planner_manager_->traj_.local_traj.traj_id) {
        publishHoverCmd();
        return;
    }
    
    double t_cur = ros::Time::now().toSec() - planner_manager_->traj_.local_traj.start_time;
    double duration = planner_manager_->traj_.local_traj.duration;
    
    if (t_cur < 0) t_cur = 0;
    if (t_cur > duration) t_cur = duration;
    
    Eigen::Vector3d pos = planner_manager_->traj_.local_traj.traj.getPos(t_cur);
    Eigen::Vector3d vel = planner_manager_->traj_.local_traj.traj.getVel(t_cur);
    Eigen::Vector3d acc = planner_manager_->traj_.local_traj.traj.getAcc(t_cur);
    
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "map";
    
    cmd.position.x = pos.x();
    cmd.position.y = pos.y();
    cmd.position.z = pos.z();
    
    cmd.velocity.x = vel.x();
    cmd.velocity.y = vel.y();
    cmd.velocity.z = vel.z();
    
    cmd.acceleration.x = acc.x();
    cmd.acceleration.y = acc.y();
    cmd.acceleration.z = acc.z();
    
    // 计算yaw（指向当前航点）
    if (current_waypoint_idx_ < static_cast<int>(waypoint_queue_.size())) {
        const WaypointInfo& wp = waypoint_queue_[current_waypoint_idx_];
        cmd.yaw = wp.yaw;
    } else {
        cmd.yaw = hover_yaw_;
    }
    
    pos_cmd_pub_.publish(cmd);
    
    // 更新悬停位置
    hover_pos_ = pos;
    hover_yaw_ = cmd.yaw;
}

double ExplorationFSMNew::getYawDiff(double yaw1, double yaw2) {
    double diff = yaw1 - yaw2;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

// ==================== 颜色表（与老架构一致） ====================
static const std::vector<std::array<float, 4>> kClusterColors = {
    {1.0f, 0.2f, 0.2f, 0.8f},   // 红色
    {0.2f, 1.0f, 0.2f, 0.8f},   // 绿色
    {0.2f, 0.4f, 1.0f, 0.8f},   // 蓝色
    {1.0f, 1.0f, 0.2f, 0.8f},   // 黄色
    {1.0f, 0.2f, 1.0f, 0.8f},   // 品红
    {0.2f, 1.0f, 1.0f, 0.8f},   // 青色
    {1.0f, 0.6f, 0.2f, 0.8f},   // 橙色
    {0.6f, 0.2f, 1.0f, 0.8f},   // 紫色
};

static std::array<float, 4> getClusterColor(int cluster_id) {
    if (cluster_id < 0) cluster_id = 0;
    return kClusterColors[cluster_id % kClusterColors.size()];
}

void ExplorationFSMNew::visualize() {
    if (!config_.enable_visualization) return;
    
    visualizeFrontierClusters();
    visualizeViewpoints();
    visualizeFrustums();
    visualizeExplorationInfo();
}

void ExplorationFSMNew::visualizeFrontierClusters() {
    if (frontier_cluster_pub_.getNumSubscribers() == 0) return;
    if (!viewpoint_manager_) return;
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    // 清除旧marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 获取当前簇数据
    std::vector<FrontierCluster> clusters = viewpoint_manager_->getClusters();
    
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        int color_idx = cluster.cluster_id >= 0 ? cluster.cluster_id : static_cast<int>(i);
        auto color = getClusterColor(color_idx);
        
        // 1. 簇范围（透明球体）
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
        float diameter = std::max(cluster.extent, 0.5f);
        extent_marker.scale.x = diameter;
        extent_marker.scale.y = diameter;
        extent_marker.scale.z = diameter;
        extent_marker.color.r = color[0];
        extent_marker.color.g = color[1];
        extent_marker.color.b = color[2];
        extent_marker.color.a = 0.25f;
        markers.markers.push_back(extent_marker);
        
        // 2. 簇中心（小实心球）
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
        center_marker.color.a = 1.0f;
        markers.markers.push_back(center_marker);
        
        // 3. 簇ID文字标签
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
        text_marker.text = "C" + std::to_string(cluster.cluster_id) + " (" + std::to_string(cluster.point_count) + ")";
        markers.markers.push_back(text_marker);
        
        // 4. 前沿点（小立方体）
        if (!cluster.points.empty()) {
            visualization_msgs::Marker points_marker;
            points_marker.header.frame_id = "map";
            points_marker.header.stamp = ros::Time::now();
            points_marker.ns = "frontier_points";
            points_marker.id = marker_id++;
            points_marker.type = visualization_msgs::Marker::CUBE_LIST;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.scale.x = 0.1;
            points_marker.scale.y = 0.1;
            points_marker.scale.z = 0.1;
            points_marker.color.r = color[0];
            points_marker.color.g = color[1];
            points_marker.color.b = color[2];
            points_marker.color.a = 0.9f;
            
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

void ExplorationFSMNew::visualizeViewpoints() {
    if (viewpoint_marker_pub_.getNumSubscribers() == 0) return;
    if (!viewpoint_manager_) return;
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    // 清除旧marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 获取当前簇数据（包含视点）
    std::vector<FrontierCluster> clusters = viewpoint_manager_->getClusters();
    
    int viewpoint_order = 1;
    for (const auto& cluster : clusters) {
        if (!cluster.has_viewpoint) continue;
        
        // 判断是否为当前目标
        bool is_current = (cluster.cluster_id == current_viewpoint_id_);
        auto cluster_color = getClusterColor(cluster.cluster_id);
        
        // 1. 视点位置（球体）
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = ros::Time::now();
        sphere.ns = "viewpoints";
        sphere.id = marker_id++;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose.position.x = cluster.viewpoint_position.x();
        sphere.pose.position.y = cluster.viewpoint_position.y();
        sphere.pose.position.z = cluster.viewpoint_position.z();
        sphere.scale.x = 0.4;
        sphere.scale.y = 0.4;
        sphere.scale.z = 0.4;
        sphere.color.r = cluster_color[0];
        sphere.color.g = cluster_color[1];
        sphere.color.b = cluster_color[2];
        sphere.color.a = is_current ? 1.0f : 0.8f;
        markers.markers.push_back(sphere);
        
        // 2. 序号标签
        visualization_msgs::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = ros::Time::now();
        text.ns = "viewpoint_labels";
        text.id = marker_id++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position.x = cluster.viewpoint_position.x();
        text.pose.position.y = cluster.viewpoint_position.y();
        text.pose.position.z = cluster.viewpoint_position.z() + 0.5;
        text.scale.z = 0.4;
        text.text = std::to_string(viewpoint_order);
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        markers.markers.push_back(text);
        viewpoint_order++;
        
        // 3. 视线方向（箭头）
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "view_directions";
        arrow.id = marker_id++;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point start, end;
        start.x = cluster.viewpoint_position.x();
        start.y = cluster.viewpoint_position.y();
        start.z = cluster.viewpoint_position.z();
        
        float arrow_length = 1.0f;
        end.x = start.x + arrow_length * std::cos(cluster.viewpoint_yaw);
        end.y = start.y + arrow_length * std::sin(cluster.viewpoint_yaw);
        end.z = start.z;
        
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.r = cluster_color[0];
        arrow.color.g = cluster_color[1];
        arrow.color.b = cluster_color[2];
        arrow.color.a = 0.9f;
        markers.markers.push_back(arrow);
    }
    
    viewpoint_marker_pub_.publish(markers);
}

void ExplorationFSMNew::visualizeFrustums() {
    if (frustum_marker_pub_.getNumSubscribers() == 0) return;
    if (!viewpoint_manager_) return;
    
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    
    // 清除旧marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 获取当前簇数据
    std::vector<FrontierCluster> clusters = viewpoint_manager_->getClusters();
    const auto& config = viewpoint_manager_->getConfig();
    
    for (const auto& cluster : clusters) {
        if (!cluster.has_viewpoint) continue;
        
        bool is_current = (cluster.cluster_id == current_viewpoint_id_);
        auto cluster_color = getClusterColor(cluster.cluster_id);
        
        std::array<float, 4> color;
        color[0] = cluster_color[0];
        color[1] = cluster_color[1];
        color[2] = cluster_color[2];
        color[3] = is_current ? 0.7f : 0.4f;
        
        createFrustumMarker(cluster.viewpoint_position, cluster.viewpoint_yaw, 0.0f,
                           config.sensor_range_max * 0.5f,
                           config.sensor_fov_h, config.sensor_fov_v,
                           marker_id, color, markers);
        marker_id += 5;
    }
    
    frustum_marker_pub_.publish(markers);
}

void ExplorationFSMNew::createFrustumMarker(
    const Eigen::Vector3f& position,
    float yaw, float pitch,
    float length, float fov_h, float fov_v,
    int id,
    const std::array<float, 4>& color,
    visualization_msgs::MarkerArray& markers)
{
    float half_h = fov_h / 2.0f;
    float half_v = fov_v / 2.0f;
    
    // 视线方向
    Eigen::Vector3f view_dir(
        std::cos(pitch) * std::cos(yaw),
        std::cos(pitch) * std::sin(yaw),
        -std::sin(pitch));
    
    // up和right向量
    Eigen::Vector3f world_up(0, 0, 1);
    Eigen::Vector3f right = view_dir.cross(world_up).normalized();
    Eigen::Vector3f up = right.cross(view_dir).normalized();
    
    // 4个角点方向
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
    lines.color.a = color[3] * 2;
    
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

void ExplorationFSMNew::visualizeExplorationInfo() {
    if (info_marker_pub_.getNumSubscribers() == 0) return;
    
    visualization_msgs::MarkerArray markers;
    
    visualization_msgs::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = ros::Time::now();
    text.ns = "exploration_info";
    text.id = 0;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    
    // 在UAV上方显示信息
    std::lock_guard<std::mutex> lock(odom_mutex_);
    text.pose.position.x = odom_pos_.x();
    text.pose.position.y = odom_pos_.y();
    text.pose.position.z = odom_pos_.z() + 1.5;
    
    text.scale.z = 0.5;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    
    std::string state_str = getStateString();
    int vp_count = viewpoint_manager_ ? viewpoint_manager_->getViewpointCount() : 0;
    size_t node_count = history_graph_ ? history_graph_->getHistoryNodeCount() : 0;
    
    std::stringstream ss;
    ss << "State: " << state_str << "\n";
    ss << "Viewpoints: " << vp_count << "\n";
    ss << "History Nodes: " << node_count << "\n";
    ss << "Visited: " << total_viewpoints_visited_;
    
    text.text = ss.str();
    markers.markers.push_back(text);
    
    info_marker_pub_.publish(markers);
}

void ExplorationFSMNew::clearVisualization() {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.header.frame_id = "map";
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    
    clear_marker.ns = "exploration_path";
    markers.markers.push_back(clear_marker);
    
    path_vis_pub_.publish(markers);
    frontier_cluster_pub_.publish(markers);
    viewpoint_marker_pub_.publish(markers);
    frustum_marker_pub_.publish(markers);
    info_marker_pub_.publish(markers);
}

} // namespace rog_map
