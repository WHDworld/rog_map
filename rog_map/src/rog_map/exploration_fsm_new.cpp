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
    nh_.param("fsm/coverage_trigger_ratio", config_.coverage_trigger_ratio, 0.3);
    nh_.param("fsm/collision_check_interval", config_.collision_check_interval, 0.05);
    nh_.param("fsm/collision_lookahead", config_.collision_lookahead, 0.5);
    nh_.param("fsm/max_viewpoint_distance", config_.max_viewpoint_distance, 20.0);
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
        
    // 创建规划器管理器
    planner_manager_.reset(new ego_planner::EGOPlannerManager);
    planner_manager_->initPlanModules(nh_, map_manager_);
    
    // 订阅者
    odom_sub_ = nh_.subscribe(config_.odom_topic, 1, &ExplorationFSMNew::odometryCallback, this);
    trigger_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ExplorationFSMNew::triggerCallback, this);
    
    // 发布者
    exploration_state_pub_ = nh_.advertise<std_msgs::Int8>("/exploration/state", 10);
    
    // 可视化发布者（使用与老架构相同的话题名称）
    frontier_cluster_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frontier_clusters", 1);
    viewpoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/viewpoints", 1);
    frustum_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frustums", 1);
    info_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/info", 1);
    path_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/path", 1);
    viewpoint_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/viewpoints", 1);
    
    // 初始化轨迹发布
    poly_traj_pub_ = nh.advertise<rog_map::PolyTraj>("/planning/trajectory", 10);
    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
    replan_pub_ = nh.advertise<std_msgs::Int8>("/planning/replan", 10);

    // 定时器
    fsm_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.fsm_frequency),
                                  &ExplorationFSMNew::fsmCallback, this);
    execution_timer_ = nh_.createTimer(ros::Duration(0.01), &ExplorationFSMNew::executionCallback, this);
    traj_vis_pub = nh_.advertise<visualization_msgs::Marker>("/ego_planner/trajectory_vis", 10);

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

void ExplorationFSMNew::visualizeTrajectory()
{
    if (!planner_manager_ || !planner_manager_->traj_.local_traj.traj_id) return;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ego_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
  
    auto& traj = planner_manager_->traj_.local_traj.traj;
    double duration = planner_manager_->traj_.local_traj.duration;
    
    for (double t = 0; t < duration; t += 0.1) {
        Eigen::Vector3d pos = traj.getPos(t);
        geometry_msgs::Point pt;
        pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
        marker.points.push_back(pt);
    }
  
    traj_vis_pub.publish(marker);
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
    viewpoint_manager_->updateUAVState(pos_f, static_cast<float>(odom_yaw_));
    history_graph_->updateUAVState(odom_pos_);
}

void ExplorationFSMNew::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    LOG_INFO("Received exploration trigger");
    triggered_ = true;
}

// ==================== 状态处理函数 ====================

void ExplorationFSMNew::processInit() 
{
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
        
        if (height_diff > config_.takeoff_height - 0.02) {
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
    
    // static bool first_wait = true;
    // if (first_wait) {
    //     ROS_INFO("[FSM_NEW] WAIT_TRIGGER: Hovering at (%.2f, %.2f, %.2f), waiting for trigger...",
    //              hover_pos_.x(), hover_pos_.y(), hover_pos_.z());
    //     ROS_INFO("[FSM_NEW] Send goal to /move_base_simple/goal to start exploration, or set auto_start=true");
    //     first_wait = false;
    // }
    
    if (config_.auto_start) {
        triggered_ = true;
    }
    
    if (triggered_) {
        // 检查是否有有效视点        
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
    
    // 1. 选择最佳视点 (两阶段 + 缓存)
    int selected_vp_id = -1;
    if (!selectBestViewpoint(selected_vp_id)) {
        // 真的没视点了，检查是否任务完成
        if (!viewpoint_manager_->hasValidViewpoints()) {
            LOG_INFO("Exploration Complete! No more viewpoints.");
            transitState(ExplorationState::WAIT_TRIGGER, "Done");
        } else {
            // 可能只是暂时没算出来，悬停等待
            ROS_WARN_THROTTLE(1.0, "[FSM] No reachable viewpoint found, retrying...");
        }
        publishHoverCmd();
        return;
    }
    ros::Time t_select_vp = ros::Time::now();
    // 2. 生成导航路径 (复用缓存)
    if (!planToViewpoint(selected_vp_id)) {
        ROS_WARN("[FSM] Failed to generate path to VP %d.", selected_vp_id);
        
        // 规划失败直接放弃本次决策，悬停，等待下一轮 FSM 循环重新选择
        // (不需要标记 Invalid，因为可能是暂时的局部规划器失败)
        publishHoverCmd();
        return;
    }
    ros::Time t_plan_to_vp = ros::Time::now();
    // 3. 锁定目标，准备起飞
    current_viewpoint_id_ = selected_vp_id;
    current_waypoint_idx_ = 0;
    
    // 重置计时器
    trajectory_start_time_ = ros::Time::now();
    last_collision_check_time_ = ros::Time::now();
    last_replan_time_ = ros::Time::now();         // [New]
    last_coverage_check_time_ = ros::Time::now();
    
    has_prev_traj_ = false;  // [New] 清空旧轨迹缓存
    has_trajectory_ = true;
    ROS_INFO("[FSM] LOCALPLAN: processLocalPlan success, current_viewpoint_id_: %d, select_vp_time: %.2fms, plan_to_vp_time: %.2fms, plan all_time: %.2fms", 
        current_viewpoint_id_, (t_select_vp - t_start).toSec() * 1000.0, (t_plan_to_vp - t_select_vp).toSec() * 1000.0, (ros::Time::now() - t_start).toSec() * 1000.0);
    transitState(ExplorationState::EXEC_EXPLORATION, "Plan success");
}

void ExplorationFSMNew::processExecExploration() {
    // 0. 基础状态检查
    if (!has_trajectory_) {
        transitState(ExplorationState::LOCALPLAN, "No trajectory");
        return;
    }
    
    ros::Time now = ros::Time::now();
    
    // 1. 碰撞检测 (安全第一)
    // 如果发现路径上有障碍物，立即中止并重规划
    if ((now - last_collision_check_time_).toSec() > config_.collision_check_interval) {
        last_collision_check_time_ = now;
        if (checkCollision()) {
            LOG_WARN("Collision detected ahead, stopping current task.");
            has_trajectory_ = false;
            transitState(ExplorationState::LOCALPLAN, "Collision detected");
            return;
        }
    }
    
    // ==================== [新增] Receding Horizon Replan Loop ====================
    double time_since_replan = (now - last_replan_time_).toSec();
    double dist_to_target = (waypoint_queue_[current_waypoint_idx_].position - odom_pos_).norm();
    
    // 触发条件: 间隔 > 0.5s 且 距离目标 > 1.0m
    bool need_replan = (time_since_replan > config_.replan_check_interval) && (dist_to_target > 2.0);
    
    if (need_replan) 
    {
        double t_start = ros::Time::now().toSec();
         Eigen::Vector3d target = waypoint_queue_[current_waypoint_idx_].position;
         Eigen::Vector3d next_vel = Eigen::Vector3d::Zero();
         
         if (current_waypoint_idx_ < waypoint_queue_.size() - 1) {
             next_vel = (waypoint_queue_[current_waypoint_idx_+1].position - target).normalized() * config_.max_vel;
         }

         // 调用带预测的规划接口 (use_latest_traj = true)
         if (planTrajectory(target, 0.0, next_vel, true)) {
             last_replan_time_ = now;
             // 更新这一段轨迹的开始时间
             trajectory_start_time_ = now;
            ROS_INFO("[FSM] EXEC_EXPLORATION: Receding replan success, plan time: %f ms", (ros::Time::now().toSec() - t_start) * 1000.0);
         } else {
             ROS_WARN_THROTTLE(1.0, "[FSM] EXEC_EXPLORATION: Receding replan failed");
         }
    }
    
    // 2. 航点追踪与切换 (核心执行逻辑)
    if (checkReachWaypoint()) {
        // 到达当前航点，索引+1
        current_waypoint_idx_++;
        
        // 2.1 检查是否到达所有航点 (即到达了最终视点)
        if (current_waypoint_idx_ >= static_cast<int>(waypoint_queue_.size())) {
            ROS_INFO("[FSM] EXEC_EXPLORATION: Reached Final Viewpoint %d. Task Complete.", current_viewpoint_id_);
            total_viewpoints_visited_++;
            
            // 任务成功：直接切回 LOCALPLAN 进行下一轮决策
            // (不需要手动标记已探索，ViewpointManager 会根据传感器覆盖情况自动移除无效视点)
            has_trajectory_ = false;
            transitState(ExplorationState::LOCALPLAN, "Reached Goal");
            return;
        }
        double t_start_next_waypoint = ros::Time::now().toSec();
        // 2.2 还没到终点，规划到下一个航点
        if (!planNextWaypoint()) {
            ROS_WARN("Failed to plan path to next waypoint (idx %d).", current_waypoint_idx_);
            has_trajectory_ = false;
            transitState(ExplorationState::LOCALPLAN, "Plan Failed");
            return;
        }    
        ROS_INFO("[FSM] EXEC_EXPLORATION: Plan next waypoint success, plan time: %f ms", (ros::Time::now().toSec() - t_start_next_waypoint) * 1000.0);
    }
    
    // 3. 超时检测
    if (checkTimeout()) {
        ROS_WARN("[FSM] EXEC_EXPLORATION: Trajectory execution timeout. Aborting.");
        
        // [Step 1] 更新悬停点为当前位置 (防止 hover_pos_ 过期)
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        
        // [Step 2] 主动发送一次刹车指令 (覆盖掉可能的旧轨迹指令)
        publishHoverCmd();

        // [Step 3] 禁用轨迹执行器 (停止 executionCallback)
        // 合理性：必须置为 false，因为当前轨迹已超时（可能死锁或参数错误），
        // 继续运行只会执行错误的旧指令。
        has_trajectory_ = false;
        
        // Variable Reset? 不需要。因为 transitState -> LOCALPLAN 会在下一帧
        // 立即触发 processLocalPlan，那里有全套的变量重置逻辑。
        
        transitState(ExplorationState::LOCALPLAN, "Timeout");
    }
    
    // 4. 发布控制指令给底层控制器
    // publishTrajectoryCmd();
}

// ==================== LOCALPLAN 核心方法 ====================
bool ExplorationFSMNew::selectBestViewpoint(int& selected_viewpoint_id) {
    // 1. 直接从 VM 获取最新 Clusters
    std::vector<FrontierCluster> clusters = viewpoint_manager_->getClusters();
    if (clusters.empty()) return false;
    
    Eigen::Vector3d current_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
    }

    // 2. 预计算 Dijkstra (Caching)
    int start_node_id = history_graph_->findNearestNode(current_pos);

    // 收集所有关联的 Anchor Nodes
    std::unordered_set<int> target_anchors;
    for (const auto& c : clusters) {
        if (c.has_viewpoint && c.history_node_id != -1) {
            // Eigen::Vector3d node_pos = history_graph_->getHistoryNodePos(c.history_node_id);
            target_anchors.insert(c.history_node_id);
            // ROS_INFO("HistoryNodeID: %d, Node_position: (%.2f,%.2f,%.2f), Viewpoint_position: (%.2f,%.2f,%.2f)", c.history_node_id, node_pos.x(), node_pos.y(), node_pos.z(), c.viewpoint_position.x(), c.viewpoint_position.y(), c.viewpoint_position.z());
        }
    }
    cached_dijkstra_result_ = DijkstraResult(); // Clear old cache
    // 运行优化版 Dijkstra (只计算必要的 targets)
    if (start_node_id != -1 && !target_anchors.empty()) {
        history_graph_->computeDijkstraToTargets(start_node_id, target_anchors, cached_dijkstra_result_);
    }

    // 定义两阶段评估 Lambda
    auto runSelection = [&](double max_range, bool is_global_search) -> int {
        int local_best_id = -1;
        float local_best_score = -1.0f;

        for (const auto& cluster : clusters) {
            if (!cluster.has_viewpoint) continue;

            // 距离初筛
            double direct_dist = (cluster.viewpoint_position.cast<double>() - current_pos).norm();
            if (direct_dist > max_range) continue;

            // [级联评估] RayCast -> Local A* -> Graph Look-up
            // 传入 cached_dijkstra_result_ 进行 O(1) 查表
            double path_cost = history_graph_->evaluateViewpointCost(
                current_pos, 
                cluster.history_node_id, 
                cluster.viewpoint_position.cast<double>(), 
                cached_dijkstra_result_, 
                nullptr // 不需要生成路径
            );

            if (path_cost >= std::numeric_limits<double>::max()) continue;

            // 评分: 加大距离惩罚防止远距离抖动
            // Score = Gain * 10 / (Cost + 1)
            float dist_weight = is_global_search ? 1.5f : 1.0f;
            float cost_metric = static_cast<float>(path_cost) * dist_weight + 1.0f;
            float score = cluster.viewpoint_score * 10.0f / cost_metric;

            if (score > local_best_score) {
                local_best_score = score;
                local_best_id = cluster.cluster_id;
            }
        }
        return local_best_id;
    };

    // 第一轮：局部优先 (< max_dist)
    int best_id = runSelection(config_.max_viewpoint_distance, false);

    // 第二轮：全图兜底 (Global Fallback)
    if (best_id == -1) {
        ROS_WARN_THROTTLE(2.0, "[SELECT] No local VP within %.1fm. Trying GLOBAL.", config_.max_viewpoint_distance);
        best_id = runSelection(std::numeric_limits<double>::max(), true);
    }

    if (best_id < 0) return false;

    selected_viewpoint_id = best_id;
    return true;
}

bool ExplorationFSMNew::planToViewpoint(int viewpoint_id) {
    // 1. 查找 Cluster 信息
    const FrontierCluster* target_cluster = nullptr;
    std::vector<FrontierCluster> clusters = viewpoint_manager_->getClusters();
    for (const auto& c : clusters) {
        if (c.cluster_id == viewpoint_id) {
            target_cluster = &c;
            break;
        }
    }

    if (!target_cluster) {
        ROS_ERROR("Cluster %d not found!", viewpoint_id);
        return false;
    }

    Eigen::Vector3d vp_pos = target_cluster->viewpoint_position.cast<double>();
    double vp_yaw = target_cluster->viewpoint_yaw;

    Eigen::Vector3d current_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
    }
    ROS_INFO("PlanToViewpoint: current_pos: (%.2f,%.2f,%.2f), best_vp_pos: (%.2f,%.2f,%.2f) best_vp_yaw: %.2f", current_pos.x(), current_pos.y(), current_pos.z(), vp_pos.x(), vp_pos.y(), vp_pos.z(), vp_yaw);
    // 2. 路径生成 (Path Generation)
    std::vector<Eigen::Vector3d> global_path;
    double path_len = 0.0;
    bool direct_success = false;

    // A. 尝试极速近场连接 (<8m)
    double dist_direct = (current_pos - vp_pos).norm();
    if (dist_direct < 8.0) {
        auto local_path = history_graph_->runLocalAStar(current_pos, vp_pos);
        if (!local_path.empty()) {
            double len = history_graph_->calculatePathLength(local_path);
            if (len < dist_direct * 2.0) { // 绕路不严重
                global_path = history_graph_->shortcutPath(current_pos, local_path);
                path_len = len;
                direct_success = true;
            }
        }
    }

    // B. 拓扑路径重建 (如果近场失败)
    if (!direct_success) {
        std::vector<Eigen::Vector3d> topo_path;
        bool graph_ok = false;
        
        if (target_cluster->history_node_id != -1) {
            graph_ok = history_graph_->reconstructPathFromCache(
                cached_dijkstra_result_, 
                target_cluster->history_node_id, 
                topo_path
            );
        }

        if (!graph_ok || topo_path.empty()) {
            // 如果拓扑路径重建失败，尝试最后的挣扎：直接跑一次 Local A*
            // 这在距离稍远但还在 A* 搜索范围内时有用
            auto fallback_path = history_graph_->runLocalAStar(current_pos, vp_pos);
            if (!fallback_path.empty()) {
                global_path = history_graph_->shortcutPath(current_pos, fallback_path);
                path_len = history_graph_->calculatePathLength(global_path);
            } else {
                ROS_ERROR("PlanToViewpoint: fallback_path is empty");
                return false;
            }
        } else {
            // 拼接: Entry + Graph + Exit
            Eigen::Vector3d start_node_pos = topo_path.front();
            Eigen::Vector3d anchor_node_pos = topo_path.back();

            auto entry_path = history_graph_->runLocalAStar(current_pos, start_node_pos);
            auto exit_path = history_graph_->runLocalAStar(anchor_node_pos, vp_pos);

            if (entry_path.empty() || exit_path.empty()) 
            {
                ROS_ERROR("PlanToViewpoint: entry_path or exit_path is empty");
                return false;
            }

            global_path.insert(global_path.end(), entry_path.begin(), entry_path.end());
            // 中间的拓扑点
            if (topo_path.size() > 1) {
                // 跳过首尾避免重复
                global_path.insert(global_path.end(), topo_path.begin() + 1, topo_path.end());
            }
            global_path.insert(global_path.end(), exit_path.begin(), exit_path.end());
            
            // 确保终点精确
            if ((global_path.back() - vp_pos).norm() > 0.01) global_path.push_back(vp_pos);

            // 再次拉直优化
            global_path = history_graph_->shortcutPath(current_pos, global_path);
            path_len = history_graph_->calculatePathLength(global_path);
        }
    }

    // 3. 构建航点队列 & 分级策略
    waypoint_queue_.clear();
    current_global_path_ = global_path; // 保存用于可视化

    // if (config_.enable_visualization) visualizeGlobalPath();

    // 【策略 A】短途/直连模式 -> 直接交给 EGO-Planner
    if (path_len < 5.0 || global_path.size() <= 2) {
        ROS_INFO("[FSM] Short path (%.1fm). DIRECT mode.", path_len);
        WaypointInfo wp;
        wp.position = vp_pos;
        wp.yaw = vp_yaw;
        wp.is_viewpoint = true;
        wp.id = viewpoint_id;
        waypoint_queue_.push_back(wp);
    } 
    // 【策略 B】长途拓扑模式 -> 跟随航点
    else {
        ROS_INFO("[FSM] Long path (%.1fm). GRAPH mode (%zu pts).", path_len, global_path.size());
        
        // 跳过起点 (当前位置)
        for (size_t i = 1; i < global_path.size(); ++i) {
            WaypointInfo wp;
            wp.position = global_path[i];
            
            // 计算期望朝向
            if (i < global_path.size() - 1) {
                // 中间点：朝向下一个路径点，方便飞越
                Eigen::Vector3d dir = global_path[i+1] - global_path[i];
                wp.yaw = std::atan2(dir.y(), dir.x());
                wp.is_viewpoint = false;
            } else {
                // 终点：使用视点期望朝向
                wp.yaw = vp_yaw;
                wp.is_viewpoint = true;
                wp.id = viewpoint_id;
            }
            waypoint_queue_.push_back(wp);
        }
    }

    // 4. 启动执行
    current_waypoint_idx_ = 0;
    if(!planNextWaypoint()) {
        ROS_ERROR("PlanToViewpoint: planNextWaypoint failed");
        return false;
    }
    return true;
}

bool ExplorationFSMNew::planNextWaypoint() {
    if (current_waypoint_idx_ >= static_cast<int>(waypoint_queue_.size())) return false;
    
    const WaypointInfo& wp = waypoint_queue_[current_waypoint_idx_];
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
    
    // [优化] 飞越中间航点 (Continuous Flight)
    if (current_waypoint_idx_ + 1 < static_cast<int>(waypoint_queue_.size())) {
        const WaypointInfo& next_wp = waypoint_queue_[current_waypoint_idx_ + 1];
        Eigen::Vector3d dir = next_wp.position - wp.position;
        
        if (dir.norm() > 0.5) {
            dir.normalize();
            // 设定 80% 速度飞过中间点
            target_vel = dir * config_.max_vel * 0.8; 
        }
    }
    
    return planTrajectory(wp.position, wp.yaw, target_vel, true);
}

bool ExplorationFSMNew::planTrajectory(const Eigen::Vector3d& target, double target_yaw, 
    const Eigen::Vector3d& target_vel, bool use_latest_traj) 
{
    Eigen::Vector3d start_pos, start_vel, start_acc;
    double t_now = ros::Time::now().toSec();

    // === 预测逻辑 ===
    if (use_latest_traj && has_trajectory_) {
        has_prev_traj_ = true;
        prev_traj_start_time_ = traj_start_time_;
        prev_traj_duration_ = traj_duration_;

        double lookahead = 0.05; 
        double t_cur = t_now - traj_start_time_;
        double t_predict = t_cur + lookahead; 

        if (t_predict > traj_duration_) t_predict = traj_duration_;

        auto& traj = planner_manager_->traj_.local_traj.traj;
        start_pos = traj.getPos(t_predict);
        start_vel = traj.getVel(t_predict);
        start_acc = traj.getAcc(t_predict);

        planner_manager_->setReplanStartTime(traj_start_time_ + t_predict);
    } else {
        start_pos = odom_pos_;
        start_vel = odom_vel_;
        start_acc = Eigen::Vector3d::Zero();
        planner_manager_->setReplanStartTime(t_now);
    }

    end_yaw_ = target_yaw; 
    
    // Set desired height for height retention (using start height)
    planner_manager_->setDesiredHeight(start_pos.z());
    // 调用 Planner
    bool success = planner_manager_->reboundReplan(
    start_pos, start_vel, start_acc, target, target_vel, 
    true, false, true); 

    if (success) {
        traj_start_time_ = planner_manager_->traj_.local_traj.start_time;
        traj_duration_ = planner_manager_->traj_.local_traj.duration;
        has_trajectory_ = true;
        last_replan_time_ = ros::Time::now();
        
        // 更新超时计时器的基准时间，防止误报超时
        trajectory_start_time_ = ros::Time(traj_start_time_);
    }

    return success;
}

std::pair<double, double> ExplorationFSMNew::calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
    constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
    constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
    std::pair<double, double> yaw_yawdot(0, 0);

    // 1. 计算速度方向 Yaw (Velocity Yaw)
    double time_forward = 0.5; 
    Eigen::Vector3d dir;
    
    // 辅助lambda：从多项式采样位置
    auto getPos = [&](double t, const rog_map::PolyTraj& traj_data) -> Eigen::Vector3d {
        return planner_manager_->traj_.local_traj.traj.getPos(t); 
    };

    // 为简化工程，建议直接使用 planner_manager_ 中的 traj 数据
    poly_traj::Trajectory* active_traj_ptr = &planner_manager_->traj_.local_traj.traj;
    
    if (t_cur + time_forward <= traj_duration_) {
        dir = active_traj_ptr->getPos(t_cur + time_forward) - pos;
    } else {
        dir = active_traj_ptr->getPos(traj_duration_) - pos;
    }

    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;

    // 2. 动态限制 (Dynamic Limits)
    double d_yaw = yaw_temp - last_yaw_;
    // 归一化到 [-PI, PI]
    while (d_yaw >= M_PI) d_yaw -= 2 * M_PI;
    while (d_yaw < -M_PI) d_yaw += 2 * M_PI;

    const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
    double d_yaw_max;
    
    if (fabs(last_yaw_dot_ + dt * YDDM) <= fabs(YDM)) {
        d_yaw_max = last_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
    } else {
        double t1 = (YDM - last_yaw_dot_) / YDDM;
        d_yaw_max = ((dt - t1) + dt) * (YDM - last_yaw_dot_) / 2.0;
    }

    if (fabs(d_yaw) > fabs(d_yaw_max)) {
        d_yaw = d_yaw_max;
    }
    
    double yaw_dot = d_yaw / dt;
    double yaw = last_yaw_ + d_yaw;

    // 更新状态
    last_yaw_ = yaw;
    last_yaw_dot_ = yaw_dot;

    return {yaw, yaw_dot};
}

void ExplorationFSMNew::executionCallback(const ros::TimerEvent& e) {
    if (state_ != ExplorationState::EXEC_EXPLORATION || !has_trajectory_) {
        return;
    }

    ros::Time now = ros::Time::now();
    double t_cur = (now.toSec() - traj_start_time_);
    static ros::Time last_time = now;
    double dt = (now - last_time).toSec();
    if (dt <= 0) dt = 0.01;

    Eigen::Vector3d pos, vel, acc;
    std::pair<double, double> yaw_state;

    // Case 1: 新轨迹还没开始 (预测期)
    if (t_cur < 0) {
        if (has_prev_traj_) {
            double t_prev = (now.toSec() - prev_traj_start_time_);
            if (t_prev < prev_traj_duration_) {
                publishCmd(last_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
            } else {
                publishCmd(last_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
            }
        } else {
             publishCmd(last_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
        }
        last_time = now;
        return;
    }

    // Case 2: 执行当前轨迹
    if (t_cur < traj_duration_) {
        auto& traj = planner_manager_->traj_.local_traj.traj;
        pos = traj.getPos(t_cur);
        vel = traj.getVel(t_cur);
        acc = traj.getAcc(t_cur);

        yaw_state = calculate_yaw(t_cur, pos, dt);
        last_pos_ = pos;
        
        publishCmd(pos, vel, acc, yaw_state.first, yaw_state.second);
    } 
    // Case 3: 轨迹结束
    else {
        auto& traj = planner_manager_->traj_.local_traj.traj;
        pos = traj.getPos(traj_duration_);
        publishCmd(pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
    }

    static int vis_counter = 0;
    if (++vis_counter >= 20) { // 100Hz下每20次就是5Hz，足够流畅且不卡顿
        vis_counter = 0;
        visualizeTrajectory();
    }
    last_time = now;
}

void ExplorationFSMNew::publishCmd(const Eigen::Vector3d& p, const Eigen::Vector3d& v, 
                                   const Eigen::Vector3d& a, double yaw, double yaw_dot) {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "map";
    
    // 如果规划器已初始化，关联轨迹ID
    if (planner_manager_) {
        cmd.trajectory_id = planner_manager_->traj_.local_traj.traj_id;
    }
    
    cmd.position.x = p.x();
    cmd.position.y = p.y();
    cmd.position.z = p.z();
    
    cmd.velocity.x = v.x();
    cmd.velocity.y = v.y();
    cmd.velocity.z = v.z();
    
    cmd.acceleration.x = a.x();
    cmd.acceleration.y = a.y();
    cmd.acceleration.z = a.z();
    
    cmd.yaw = yaw;
    cmd.yaw_dot = yaw_dot;
    
    pos_cmd_pub_.publish(cmd);
}

void ExplorationFSMNew::polyTraj2ROSMsg(rog_map::PolyTraj &poly_msg)
{
  poly_traj::Trajectory &traj = planner_manager_->traj_.local_traj.traj;
  
  poly_msg.traj_id = planner_manager_->traj_.local_traj.traj_id;
  poly_msg.start_time = ros::Time(planner_manager_->traj_.local_traj.start_time);
  poly_msg.order = 5;  // MINCO uses order 5
  
  // Yaw planning: 
  // If end_yaw_ is NaN, it means "follow velocity direction" (preset waypoint mode)
  // Otherwise, interpolate from current yaw to target yaw (exploration mode)
  poly_msg.start_yaw = odom_yaw_;
  poly_msg.end_yaw = end_yaw_;  // May be NaN for preset waypoint mode

  int piece_num = traj.getPieceNum();
  for (int i = 0; i < piece_num; i++)
  {
    poly_msg.duration.push_back(traj.getPiece(i).getDuration());
    
    Eigen::MatrixXd coef = traj.getPiece(i).getCoeffMat();
    for (int j = 0; j < 6; j++)
    {
      poly_msg.coef_x.push_back(coef(0, j));
      poly_msg.coef_y.push_back(coef(1, j));
      poly_msg.coef_z.push_back(coef(2, j));
    }
  }
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
    
    // A. 如果是中间路径点 (Intermediate Waypoint)
    // 只要距离足够近，就算通过，允许飞越 (Fly-through)
    if (!wp.is_viewpoint) {
        // 使用稍大的阈值 (e.g., 0.8m ~ 1.5m) 保证流畅性
        // 如果 waypoint_approach_dist 设为了 1.5，这里可以用 0.8 作为切换阈值
        return dist < std::max(config_.reach_threshold, 0.8); 
    }
    
    // B. 如果是最终视点 (Target Viewpoint)
    // 必须严格检查位置和朝向 (Stop and Align)
    double yaw_diff = std::abs(getYawDiff(current_yaw, wp.yaw));
    
    bool pos_reached = dist < config_.reach_threshold;
    bool yaw_reached = yaw_diff < config_.yaw_threshold;

    if (pos_reached && yaw_reached) {
        return true;
    }
    
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
