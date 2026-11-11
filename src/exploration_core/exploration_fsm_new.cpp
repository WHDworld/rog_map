/**
 * @file exploration_fsm_new.cpp
 * @brief 探索状态机实现 - 重构版（4状态 + 集成规划器）
 */

#include "exploration/exploration_fsm_new.h"
#include "ego_planner/yaw_optimizer.h"
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <ros/package.h>
#include "utils/perf_logger.hpp"

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
    nh_.param("fsm/trajectory_switch_delay", config_.trajectory_switch_delay, 0.20);
    nh_.param("fsm/plan_retry_initial", config_.plan_retry_initial, 0.10);
    nh_.param("fsm/plan_retry_max", config_.plan_retry_max, 1.0);
    nh_.param("fsm/trajectory_bounds_margin", config_.trajectory_bounds_margin, 0.05);
    nh_.param("fsm/waypoint_approach_dist", config_.waypoint_approach_dist, 1.5);
    nh_.param("fsm/trajectory_timeout", config_.trajectory_timeout, 10.0);
    nh_.param("fsm/coverage_trigger_ratio", config_.coverage_trigger_ratio, 0.3);
    nh_.param("fsm/minimum_task_execution_time", config_.minimum_task_execution_time, 0.5);
    nh_.param("fsm/collision_check_interval", config_.collision_check_interval, 0.05);
    nh_.param("fsm/collision_lookahead", config_.collision_lookahead, 0.5);
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
    viewpoint_manager_ = std::make_shared<ViewpointManager>();
    viewpoint_manager_->init(nh_, global_map_);

    // 全局调度器（ATSP + 局部精化 + 已知自由路径）
    global_tour_planner_ = std::make_shared<GlobalTourPlanner>();
    global_tour_planner_->init(nh_, global_map_);
        
    // 创建规划器管理器
    planner_manager_.reset(new ego_planner::EGOPlannerManager);
    planner_manager_->initPlanModules(nh_, map_manager_);
    
    // 创建语义目标提取模块
    semantic_target_extractor_ = std::make_unique<SemanticTargetExtractor>();
    semantic_target_extractor_->init(nh_, global_map_.get());
    
    // 订阅者
    odom_sub_ = nh_.subscribe(config_.odom_topic, 1, &ExplorationFSMNew::odometryCallback, this);
    trigger_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ExplorationFSMNew::triggerCallback, this);
    
    // [新增] 仿真模式参数
    nh_.param("fsm/sim_mode", sim_mode_, 1);  // 默认 SIM_OPEN3D
    const char* sim_mode_str[] = {"GAZEBO", "OPEN3D", "REAL"};
    ROS_INFO("[FSM_NEW] Simulation mode: %s", sim_mode_str[sim_mode_]);
    
    // [新增] MAVROS (仅 Gazebo/Real 模式)
    if (sim_mode_ != SIM_OPEN3D) {
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &ExplorationFSMNew::stateCallback, this);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        ROS_INFO("[FSM_NEW] MAVROS clients initialized");
    }
    
    // 发布者
    exploration_state_pub_ = nh_.advertise<std_msgs::Int8>("/exploration/state", 10);
    
    // 可视化发布者（使用与老架构相同的话题名称）
    frontier_cluster_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frontier_clusters", 1);
    global_tour_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/global_tour", 1);   // [Visual] 全局巡游顺序
    refined_tour_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/refined_tour", 1); // [Visual] 局部精化视点
    viewpoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/viewpoints", 1);
    frustum_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/frustums", 1);
    info_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/info", 1);
    path_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/path", 1);
    viewpoint_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/viewpoints", 1);
    
    // 初始化轨迹发布
    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
    replan_pub_ = nh.advertise<std_msgs::Int8>("/planning/replan", 10);

    // 定时器
    fsm_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.fsm_frequency),
                                  &ExplorationFSMNew::fsmCallback, this);
    execution_timer_ = nh_.createTimer(ros::Duration(0.01), &ExplorationFSMNew::executionCallback, this);
    traj_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/ego_planner/trajectory_vis", 10);

    // [新增] 初始化轨迹日志文件
    if (enable_traj_logging_) {
        // 自动获取 rog_map 包路径
        std::string package_path = ros::package::getPath("semantic_exploration");
        if (package_path.empty()) {
            ROS_WARN("[FSM] Failed to find rog_map package path, trajectory logging disabled");
            enable_traj_logging_ = false;
        } else {
            std::string log_dir = package_path + "/log";
            const auto run_id = static_cast<unsigned long long>(
                ros::WallTime::now().toNSec());
            std::string log_file = log_dir + "/trajectory_log_" +
                                   std::to_string(run_id) + ".csv";
            
            // 创建目录（如果不存在）
            int ret = system(("mkdir -p " + log_dir).c_str());
            (void)ret;  // 忽略返回值警告
            
            traj_log_file_.open(log_file, std::ios::out | std::ios::trunc);
            if (traj_log_file_.is_open()) {
                // 写入 CSV 头
                traj_log_file_ << "timestamp,cmd_x,cmd_y,cmd_z,cmd_vx,cmd_vy,cmd_vz,cmd_yaw,cmd_yaw_dot,"
                              << "odom_x,odom_y,odom_z,odom_vx,odom_vy,odom_vz,odom_yaw,"
                              << "traj_end_x,traj_end_y,traj_end_z,plan_time,plan_target_yaw\n";
                traj_log_file_.flush();
                ROS_INFO("[FSM] Trajectory logging enabled: %s", log_file.c_str());
            } else {
                ROS_WARN("[FSM] Failed to open trajectory log file: %s", log_file.c_str());
                enable_traj_logging_ = false;
            }
        }
    }

    // 初始化状态
    state_ = ExplorationState::INIT;
    fsm_init_time_ = ros::Time::now();
    
    LOG_INFO("ExplorationFSMNew initialized with 4-state design");
}

std::string ExplorationFSMNew::getStateString() const {
    switch (state_.load()) {
        case ExplorationState::INIT: return "INIT";
        case ExplorationState::PRE_ARM: return "PRE_ARM";
        case ExplorationState::TAKEOFF: return "TAKEOFF";
        case ExplorationState::WAIT_TRIGGER: return "WAIT_TRIGGER";
        case ExplorationState::LOCALPLAN: return "LOCALPLAN";
        case ExplorationState::EXEC_EXPLORATION: return "EXEC_EXPLORATION";
        case ExplorationState::FINISH: return "FINISH";
        default: return "UNKNOWN";
    }
}

void ExplorationFSMNew::visualizeTrajectory()
{
    if (!planner_manager_ || !has_trajectory_) return;
    
    auto& traj = planner_manager_->traj_.local_traj.traj;
    double duration = planner_manager_->traj_.local_traj.duration;
    
    // 安全检查：确保轨迹有效
    if (traj.getPieceNum() == 0 || duration <= 0) return;
    
    visualization_msgs::MarkerArray marker_array;
    
    // ========== 1. 轨迹线 ==========
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.ns = "trajectory";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = 0.08;
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 0.5;
    traj_marker.color.b = 1.0;
    traj_marker.color.a = 1.0;
    
    for (double t = 0; t < duration; t += 0.1) {
        Eigen::Vector3d pos = traj.getPos(t);
        if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z())) continue;
        geometry_msgs::Point pt;
        pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
        traj_marker.points.push_back(pt);
    }
    if (!traj_marker.points.empty())
        marker_array.markers.push_back(traj_marker);
    
    // ========== 2. 终点Yaw箭头（橙色） ==========
    if (!std::isnan(current_vp_yaw_))
    {
        visualization_msgs::Marker goal_yaw_marker;
        goal_yaw_marker.header.frame_id = "map";
        goal_yaw_marker.header.stamp = ros::Time::now();
        goal_yaw_marker.ns = "goal_yaw";
        goal_yaw_marker.id = 1;
        goal_yaw_marker.type = visualization_msgs::Marker::ARROW;
        goal_yaw_marker.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point start, end;
        start.x = current_vp_pos_.x();
        start.y = current_vp_pos_.y();
        start.z = current_vp_pos_.z();
        double arrow_len = 1.0;
        end.x = current_vp_pos_.x() + arrow_len * cos(current_vp_yaw_);
        end.y = current_vp_pos_.y() + arrow_len * sin(current_vp_yaw_);
        end.z = current_vp_pos_.z();
        goal_yaw_marker.points.push_back(start);
        goal_yaw_marker.points.push_back(end);
        
        goal_yaw_marker.scale.x = 0.15;
        goal_yaw_marker.scale.y = 0.25;
        goal_yaw_marker.scale.z = 0.0;
        goal_yaw_marker.color.r = 1.0;
        goal_yaw_marker.color.g = 0.3;
        goal_yaw_marker.color.b = 0.0;
        goal_yaw_marker.color.a = 1.0;
        marker_array.markers.push_back(goal_yaw_marker);
    }
    
    traj_vis_pub.publish(marker_array);
}

void ExplorationFSMNew::transitState(ExplorationState new_state, const std::string& reason) {
    std::string old_state_str = getStateString();
    state_ = new_state;

    std::string new_state_str;
    switch (new_state) {
        case ExplorationState::INIT: new_state_str = "INIT"; break;
        case ExplorationState::PRE_ARM: new_state_str = "PRE_ARM"; break;
        case ExplorationState::TAKEOFF: new_state_str = "TAKEOFF"; break;
        case ExplorationState::WAIT_TRIGGER: new_state_str = "WAIT_TRIGGER"; break;
        case ExplorationState::LOCALPLAN: new_state_str = "LOCALPLAN"; break;
        case ExplorationState::EXEC_EXPLORATION: new_state_str = "EXEC_EXPLORATION"; break;
        case ExplorationState::FINISH: new_state_str = "FINISH"; break;
        default: new_state_str = "UNKNOWN"; break;
    }
    
    if (old_state_str != new_state_str) {
        ROS_INFO("[FSM_NEW] TRANSIT: %s -> %s (%s)",
                 old_state_str.c_str(), new_state_str.c_str(), reason.c_str());
    }
    
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
    // [调试] 入口日志
    switch (state_.load()) {
        case ExplorationState::INIT:
            processInit();
            break;
        case ExplorationState::PRE_ARM:
            processPreArm();
            break;
        case ExplorationState::TAKEOFF:
            processTakeoff();
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
        case ExplorationState::FINISH:
            publishHoverCmd();
            break;
    }
    
    // 可视化 (暂时禁用以调试)
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
    
    // 记录初始位置
    initial_pos_ = odom_pos_;
    
    if (sim_mode_ == SIM_OPEN3D) {
        // Open3D 模式：无 FCU，直接起飞
        offboard_ready_ = true;
        takeoff_start_time_ = ros::Time::now();
        hover_pos_ = Eigen::Vector3d(initial_pos_.x(), initial_pos_.y(), 
                                      initial_pos_.z() + config_.takeoff_height);
        hover_yaw_ = odom_yaw_;
        ROS_INFO("[FSM_NEW] Open3D mode: skipping PRE_ARM, starting takeoff to z=%.2f", 
                 initial_pos_.z() + config_.takeoff_height);
        transitState(ExplorationState::TAKEOFF, "Open3D: direct takeoff");
    } else {
        // Gazebo/Real 模式：需要 PRE_ARM
        transitState(ExplorationState::PRE_ARM, "Need FCU arming");
    }
}

void ExplorationFSMNew::processPreArm() 
{
    if (sim_mode_ == SIM_REAL) {
        // 真机模式：等待手动解锁
        static bool waiting_printed = false;
        
        if (!current_state_.connected) {
            ROS_INFO_THROTTLE(1.0, "[FSM_NEW] REAL mode: Waiting for FCU connection...");
            return;
        }
        
        if (!waiting_printed) {
            ROS_INFO("[FSM_NEW] REAL mode: Waiting for manual arming and OFFBOARD mode...");
            waiting_printed = true;
        }
        
        // 发送悬停命令保持 setpoint 活跃
        publishHoverCmd();
        
        // 检测手动解锁
        if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
            offboard_ready_ = true;
            takeoff_start_time_ = ros::Time::now();
            hover_pos_ = odom_pos_;
            hover_yaw_ = odom_yaw_;
            waiting_printed = false;
            ROS_INFO("[FSM_NEW] REAL mode: Manual arming detected, starting takeoff");
            transitState(ExplorationState::TAKEOFF, "Manual arm OK");
        } else {
            ROS_INFO_THROTTLE(2.0, "[FSM_NEW] REAL mode: armed=%d, mode=%s (waiting for OFFBOARD+armed)", 
                             current_state_.armed, current_state_.mode.c_str());
        }
    } else {
        // Gazebo 模式：自动请求 OFFBOARD + ARM
        static ros::Time last_request = ros::Time::now();
        static ros::Time pre_arm_start = ros::Time(0);
        static int setpoint_count = 0;
        
        if (!current_state_.connected) {
            ROS_INFO_THROTTLE(1.0, "[FSM_NEW] GAZEBO mode: Waiting for FCU connection...");
            return;
        }
        
        // 记录进入PRE_ARM的时间
        if (pre_arm_start == ros::Time(0)) {
            pre_arm_start = ros::Time::now();
            hover_pos_ = odom_pos_;  // 固定悬停位置
            hover_yaw_ = odom_yaw_;
            ROS_INFO("[FSM_NEW] GAZEBO mode: Pre-arming started, sending setpoints...");
        }
        
        // 持续发送悬停命令（这是关键！）
        publishHoverCmd();
        setpoint_count++;
        
        // 等待发送足够的setpoint（至少2秒 @ 100Hz = 200个命令）
        double pre_arm_elapsed = (ros::Time::now() - pre_arm_start).toSec();
        if (pre_arm_elapsed < 2.5) {
            ROS_INFO_THROTTLE(1.0, "[FSM_NEW] GAZEBO mode: Pre-arming (%.1f/2.5s, %d setpoints)", 
                             pre_arm_elapsed, setpoint_count);
            return;
        }
        
        // 请求OFFBOARD模式
        if (current_state_.mode != "OFFBOARD" && 
            ros::Time::now() - last_request > ros::Duration(2.0)) {
            mavros_msgs::SetMode mode;
            mode.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(mode) && mode.response.mode_sent) {
                ROS_INFO("[FSM_NEW] GAZEBO mode: OFFBOARD mode requested");
            } else {
                ROS_WARN("[FSM_NEW] GAZEBO mode: OFFBOARD request FAILED");
            }
            last_request = ros::Time::now();
        } 
        // 请求ARM
        else if (current_state_.mode == "OFFBOARD" && !current_state_.armed && 
                   ros::Time::now() - last_request > ros::Duration(1.0)) {
            mavros_msgs::CommandBool arm;
            arm.request.value = true;
            if (arming_client_.call(arm) && arm.response.success) {
                ROS_INFO("[FSM_NEW] GAZEBO mode: Vehicle armed");
            } else {
                ROS_WARN("[FSM_NEW] GAZEBO mode: Arming request FAILED");
            }
            last_request = ros::Time::now();
        }
        
        if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
            offboard_ready_ = true;
            takeoff_start_time_ = ros::Time::now();
            hover_pos_ = odom_pos_;
            hover_yaw_ = odom_yaw_;
            // 重置静态变量
            pre_arm_start = ros::Time(0);
            setpoint_count = 0;
            ROS_INFO("[FSM_NEW] GAZEBO mode: OFFBOARD enabled, starting takeoff");
            transitState(ExplorationState::TAKEOFF, "Auto arm OK");
        }
    }
}

void ExplorationFSMNew::processTakeoff() 
{
    double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
    double target_height = initial_pos_.z() + config_.takeoff_height;
    
    publishTakeoffCmd();
    
    // 检查是否完成起飞
    if (odom_pos_.z() > target_height - 0.1 && elapsed > 2.0) {
        takeoff_complete_ = true;
        hover_pos_ = Eigen::Vector3d(initial_pos_.x(), initial_pos_.y(), target_height);
        hover_yaw_ = odom_yaw_;
        
        // 启动模块
        viewpoint_manager_->start();
        
        if (config_.auto_start) {
            triggered_ = true;
            ROS_INFO("[FSM_NEW] Takeoff complete (z=%.2f), auto starting exploration", odom_pos_.z());
            transitState(ExplorationState::LOCALPLAN, "Takeoff done, auto start");
        } else {
            ROS_INFO("[FSM_NEW] Takeoff complete (z=%.2f), waiting for trigger", odom_pos_.z());
            transitState(ExplorationState::WAIT_TRIGGER, "Takeoff done");
        }
    } else if (elapsed > config_.takeoff_timeout) {
        ROS_WARN("[FSM_NEW] Takeoff timeout!");
        takeoff_complete_ = true;
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        
        viewpoint_manager_->start();

        transitState(ExplorationState::WAIT_TRIGGER, "Takeoff timeout");
    } else {
        ROS_INFO_THROTTLE(1.0, "[FSM_NEW] TAKEOFF: z=%.2f, target=%.2f, elapsed=%.1fs",
                          odom_pos_.z(), target_height, elapsed);
    }
}

void ExplorationFSMNew::stateCallback(const mavros_msgs::State::ConstPtr& msg) 
{
    current_state_ = *msg;
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
        // [新增] 非阻塞延迟 3 秒启动
        constexpr double TRIGGER_DELAY_SEC = 5.0;
        
        if (!trigger_delay_started_) {
            // 首次触发，记录时间并开始计时
            trigger_time_ = ros::Time::now();
            trigger_delay_started_ = true;
            ROS_INFO("[FSM] Triggered! Waiting %.1f seconds before starting...", TRIGGER_DELAY_SEC);
        }
        
        double elapsed = (ros::Time::now() - trigger_time_).toSec();
        if (elapsed < TRIGGER_DELAY_SEC) {
            // 延迟期间，定期打印倒计时（每秒一次）
            ROS_INFO_THROTTLE(1.0, "[FSM] Starting in %.1f s...", TRIGGER_DELAY_SEC - elapsed);
            return;  // 继续等待，不进入后续状态
        }
        
        // 延迟结束，正式开始
        // 检查是否有可调度的探索目标（前沿簇或未观测语义目标）
        if (hasActiveExplorationTargets()) {
            transitState(ExplorationState::LOCALPLAN, "Triggered with exploration targets");
        } else {
            // 没有目标，等待地图/语义模块生成
            LOG_DEBUG("Waiting for exploration targets...");
        }
    }
}

void ExplorationFSMNew::processLocalPlan() {
    const ros::WallTime t_start = ros::WallTime::now();
    const ros::Time now = ros::Time::now();
    if (!next_plan_allowed_time_.isZero() && now < next_plan_allowed_time_) {
        publishHoverCmd();
        return;
    }

    auto defer_retry = [this](const char* reason) {
        ++consecutive_plan_failures_;
        const int exponent = std::min(consecutive_plan_failures_ - 1, 10);
        const double delay = std::min(
            config_.plan_retry_max,
            config_.plan_retry_initial * std::pow(2.0, static_cast<double>(exponent)));
        next_plan_allowed_time_ = ros::Time::now() + ros::Duration(delay);
        ROS_WARN_THROTTLE(1.0,
            "[FSM] Planning retry deferred %.2fs after %d consecutive failures (%s)",
            delay, consecutive_plan_failures_, reason);
    };

    // 1. 机器人状态快照
    const RobotPlanningState robot = snapshotRobotState();
    if (!robot.valid) {
        ROS_WARN_THROTTLE(1.0, "[FSM] No odometry yet, cannot plan.");
        publishHoverCmd();
        defer_retry("odometry unavailable");
        return;
    }

    // 2. 采集统一任务（前沿簇 + 未观测语义目标，过滤黑名单）
    std::vector<ExplorationTask> tasks = collectExplorationTasks();
    if (tasks.empty()) {
        if (!hasActiveExplorationTargets()) {
            LOG_INFO("Exploration Complete! No more frontier or semantic targets.");
            transitState(ExplorationState::WAIT_TRIGGER, "Done");
        } else {
            // 已完成/失败任务保持抑制，直到其稳定 ID 从活动集合消失。
            // 立即清空会重新选择同一前沿，形成“到达-拉黑-重选”的活锁。
            ROS_WARN_THROTTLE(1.0, "[FSM] No schedulable task; waiting for the active task set to change.");
            defer_retry("no schedulable task");
        }
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        publishHoverCmd();
        return;
    }

    // 记录本轮活动任务集（EXEC 阶段用于检测任务集变化）
    last_active_task_keys_ = activeTaskKeys();

    // 3. 全局调度（ATSP + 局部精化 + 首视点已知自由路径）
    GlobalTourPlan plan = global_tour_planner_->plan(robot, tasks, current_task_key_);
    current_tour_plan_ = plan;
    const ros::WallTime t_sched = ros::WallTime::now();
    if (!plan.success) {
        ROS_WARN_THROTTLE(1.0, "[FSM] Global tour planning failed: %s", plan.failure_reason.c_str());
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        publishHoverCmd();
        defer_retry("global tour failed");
        return;
    }

    // 4. 重验证调度路径（规划期间地图可能已更新）
    if (!global_tour_planner_->pathFinder().validatePath(
            plan.next_goal.path, UnknownSpacePolicy::BLOCK)) {
        ROS_WARN("[FSM] Scheduled path no longer known-free, invalidating task=%s#%d",
                 taskTypeName(plan.next_goal.task_key.type), plan.next_goal.task_key.id);
        global_tour_planner_->invalidateTask(plan.next_goal.task_key);
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        publishHoverCmd();
        defer_retry("scheduled path invalid");
        return;
    }

    // 5. 生成航点队列并启动轨迹
    if (!planToScheduledGoal(plan.next_goal)) {
        ROS_WARN("[FSM] Failed to plan trajectory to task=%s#%d. Blacklisting it.",
                 taskTypeName(plan.next_goal.task_key.type), plan.next_goal.task_key.id);
        task_blacklist_.insert(plan.next_goal.task_key);
        global_tour_planner_->invalidateTask(plan.next_goal.task_key);
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        publishHoverCmd();
        defer_retry("local trajectory failed");
        return;
    }
    const ros::WallTime t_goal = ros::WallTime::now();

    // 6. 锁定任务与可视化状态
    current_task_key_ = plan.next_goal.task_key;
    consecutive_plan_failures_ = 0;
    next_plan_allowed_time_ = ros::Time(0);
    task_execution_start_time_ = ros::Time::now();
    global_tour_dirty_ = false;
    current_vp_pos_ = plan.next_goal.viewpoint;
    current_vp_yaw_ = plan.next_goal.yaw;
    current_frontier_.clear();
    current_viewpoint_centroid_ = plan.next_goal.viewpoint;
    if (current_task_key_->type == ExplorationTaskType::FRONTIER) {
        for (const auto& cluster : viewpoint_manager_->getClusters()) {
            if (cluster.cluster_id == current_task_key_->id) {
                current_viewpoint_centroid_ = cluster.centroid.cast<double>();
                current_frontier_ = cluster.points;
                break;
            }
        }
    } else {
        for (const auto& task : tasks) {
            if (task.key == *current_task_key_) {
                current_viewpoint_centroid_ = task.centroid;
                break;
            }
        }
    }

    // 重置计时器
    last_collision_check_time_ = ros::Time::now();
    last_coverage_check_time_ = ros::Time::now();
    has_prev_traj_ = false;  // 清空旧轨迹缓存

    const double sched_ms = (t_sched - t_start).toSec() * 1000.0;
    const double goal_ms = (t_goal - t_sched).toSec() * 1000.0;
    const double total_ms = (ros::WallTime::now() - t_start).toSec() * 1000.0;
    ROS_INFO("[FSM] LOCALPLAN: task=%s#%d sched_ms=%.2f goal_ms=%.2f total_ms=%.2f waypoints=%zu",
             taskTypeName(current_task_key_->type), current_task_key_->id,
             sched_ms, goal_ms, total_ms, waypoint_queue_.size());
    PERF_LOG.logf("SCHED", "LOCAL_PLAN",
        "task=%s#%d sched_ms=%.3f goal_ms=%.3f total_ms=%.3f atsp_ms=%.3f refine_ms=%.3f "
        "path_ms=%.3f waypoints=%zu",
        taskTypeName(current_task_key_->type), current_task_key_->id,
        sched_ms, goal_ms, total_ms, plan.timings.atsp_ms, plan.timings.refine_ms,
        plan.timings.path_ms, waypoint_queue_.size());
    transitState(ExplorationState::EXEC_EXPLORATION, "Plan success");
}

// ==================== 统一任务采集与调度目标执行 ====================

RobotPlanningState ExplorationFSMNew::snapshotRobotState() const {
    RobotPlanningState state;
    std::lock_guard<std::mutex> lock(odom_mutex_);
    state.position = odom_pos_;
    state.velocity = odom_vel_;
    state.yaw = odom_yaw_;
    state.valid = have_odom_;
    return state;
}

std::vector<ExplorationTask> ExplorationFSMNew::collectExplorationTasks() {
    std::vector<ExplorationTask> tasks;

    // ---- 前沿簇任务 ----
    if (viewpoint_manager_) {
        for (const auto& cluster : viewpoint_manager_->getClusters()) {
            if (!cluster.has_viewpoint || cluster.candidates.empty()) continue;

            ExplorationTask task;
            task.key.type = ExplorationTaskType::FRONTIER;
            task.key.id = cluster.cluster_id;
            task.centroid = cluster.centroid.cast<double>();
            task.raw_gain = 0.5 * static_cast<double>(cluster.point_count) +
                            static_cast<double>(cluster.exploration_gain);
            task.semantic_value = 0.0;
            task.viewpoints = cluster.candidates;
            tasks.push_back(std::move(task));
        }
    }

    // ---- 未观测语义目标任务 ----
    if (semantic_target_extractor_) {
        std::vector<TargetMetadata> targets;
        semantic_target_extractor_->getUnobservedTargetsSnapshot(targets);
        for (const auto& target : targets) {
            if (!target.has_viewpoint && target.candidate_viewpoints.empty()) continue;

            ExplorationTask task;
            task.key.type = ExplorationTaskType::SEMANTIC;
            task.key.id = target.id;
            task.centroid = target.centroid.cast<double>();
            task.raw_gain = static_cast<double>(target.score);
            task.semantic_value = 1.0;
            if (!target.candidate_viewpoints.empty()) {
                task.viewpoints.reserve(target.candidate_viewpoints.size());
                for (const auto& candidate : target.candidate_viewpoints) {
                    ExplorationViewpoint viewpoint;
                    viewpoint.position = candidate.position.cast<double>();
                    viewpoint.yaw = static_cast<double>(candidate.yaw);
                    viewpoint.view_score = static_cast<double>(candidate.score);
                    task.viewpoints.push_back(viewpoint);
                }
            } else {
                ExplorationViewpoint viewpoint;
                viewpoint.position = target.viewpoint_position.cast<double>();
                viewpoint.yaw = static_cast<double>(target.viewpoint_yaw);
                viewpoint.view_score = 1.0;
                task.viewpoints.push_back(viewpoint);
            }
            tasks.push_back(std::move(task));
        }
    }

    // 清理已消失任务的黑名单项（避免长期误过滤）
    auto still_exists = [&tasks](const ExplorationTaskKey& key) {
        for (const auto& task : tasks) {
            if (task.key == key) return true;
        }
        return false;
    };
    for (auto it = task_blacklist_.begin(); it != task_blacklist_.end();) {
        if (!still_exists(*it)) {
            it = task_blacklist_.erase(it);
        } else {
            ++it;
        }
    }

    // 过滤黑名单
    tasks.erase(std::remove_if(tasks.begin(), tasks.end(),
                               [this](const ExplorationTask& task) {
                                   return task_blacklist_.count(task.key) > 0;
                               }),
                tasks.end());
    return tasks;
}

bool ExplorationFSMNew::planToScheduledGoal(const ScheduledGoal& goal) {
    waypoint_queue_.clear();
    current_global_path_ = goal.path;

    double path_length = 0.0;
    for (size_t i = 1; i < goal.path.size(); ++i) {
        path_length += (goal.path[i] - goal.path[i - 1]).norm();
    }

    // 短途/直连模式：直接把最终视点交给局部规划器
    const bool direct_mode = goal.path.size() <= 2 || path_length < 6.0;
    if (!direct_mode) {
        Eigen::Vector3d last_added = goal.path.front();
        for (size_t i = 1; i + 1 < goal.path.size(); ++i) {
            const Eigen::Vector3d& point = goal.path[i];
            if ((point - last_added).norm() < 1.5) continue;

            WaypointInfo waypoint;
            waypoint.position = point;
            const Eigen::Vector3d dir = goal.path[i + 1] - point;
            waypoint.yaw = dir.norm() > 1e-6 ? std::atan2(dir.y(), dir.x()) : goal.yaw;
            waypoint.is_viewpoint = false;
            waypoint.task_key = goal.task_key;
            waypoint_queue_.push_back(waypoint);
            last_added = point;
        }
    }

    WaypointInfo final_waypoint;
    final_waypoint.position = goal.viewpoint;
    final_waypoint.yaw = goal.yaw;
    final_waypoint.is_viewpoint = true;
    final_waypoint.id = goal.task_key.id;
    final_waypoint.task_key = goal.task_key;
    waypoint_queue_.push_back(final_waypoint);

    current_waypoint_idx_ = 0;
    if (!planNextWaypoint()) {
        ROS_ERROR("[FSM] planNextWaypoint failed at start (task=%s#%d, %zu waypoints).",
                  taskTypeName(goal.task_key.type), goal.task_key.id,
                  waypoint_queue_.size());
        return false;
    }

    ROS_INFO("[FSM] Mode: %s (len=%.1f, path_points=%zu, waypoints=%zu)",
             direct_mode ? "DIRECT" : "PATH", path_length, goal.path.size(),
             waypoint_queue_.size());
    return true;
}

bool ExplorationFSMNew::hasActiveExplorationTargets() const {
    if (viewpoint_manager_ && viewpoint_manager_->hasValidViewpoints()) return true;

    if (semantic_target_extractor_) {
        std::vector<TargetMetadata> targets;
        semantic_target_extractor_->getUnobservedTargetsSnapshot(targets);
        if (!targets.empty()) return true;
    }
    return false;
}

bool ExplorationFSMNew::currentTaskExists() const {
    if (!current_task_key_.has_value()) return false;
    const ExplorationTaskKey& key = *current_task_key_;

    if (key.type == ExplorationTaskType::FRONTIER) {
        if (!viewpoint_manager_) return false;
        for (const auto& cluster : viewpoint_manager_->getClusters()) {
            if (cluster.cluster_id == key.id && cluster.has_viewpoint &&
                !cluster.candidates.empty()) {
                return true;
            }
        }
        return false;
    }

    if (!semantic_target_extractor_) return false;
    std::vector<TargetMetadata> targets;
    semantic_target_extractor_->getUnobservedTargetsSnapshot(targets);
    for (const auto& target : targets) {
        if (target.id == key.id) return true;
    }
    return false;
}

std::unordered_set<ExplorationTaskKey, ExplorationTaskKeyHash>
ExplorationFSMNew::activeTaskKeys() const {
    std::unordered_set<ExplorationTaskKey, ExplorationTaskKeyHash> keys;

    if (viewpoint_manager_) {
        for (const auto& cluster : viewpoint_manager_->getClusters()) {
            if (cluster.has_viewpoint && !cluster.candidates.empty()) {
                keys.insert(ExplorationTaskKey{ExplorationTaskType::FRONTIER,
                                               cluster.cluster_id});
            }
        }
    }

    if (semantic_target_extractor_) {
        std::vector<TargetMetadata> targets;
        semantic_target_extractor_->getUnobservedTargetsSnapshot(targets);
        for (const auto& target : targets) {
            keys.insert(ExplorationTaskKey{ExplorationTaskType::SEMANTIC, target.id});
        }
    }
    return keys;
}

void ExplorationFSMNew::processExecExploration() {
    // 0. 基础状态检查
    if (!has_trajectory_) {
        transitState(ExplorationState::LOCALPLAN, "No trajectory");
        return;
    }
    if (waypoint_queue_.empty() ||
        current_waypoint_idx_ >= static_cast<int>(waypoint_queue_.size())) {
        has_trajectory_ = false;
        transitState(ExplorationState::LOCALPLAN, "Invalid waypoint queue");
        return;
    }

    ros::Time now = ros::Time::now();

        // ==================== [新增] 覆盖/失效检测与无缝切换 ====================
    // 频率控制: 2Hz (每0.5秒检查一次)
    if ((now - last_coverage_check_time_).toSec() > 0.5) {
        last_coverage_check_time_ = now;

        const auto active_keys = activeTaskKeys();
        const bool active_set_changed = active_keys != last_active_task_keys_;
        if (active_set_changed) {
            // FUEL-style behavior: frontier changes update the pending tour, but do not
            // interrupt a safe trajectory. Re-solve at the next meaningful trigger.
            global_tour_dirty_ = true;
            last_active_task_keys_ = active_keys;
            ROS_DEBUG("[Coverage] Active task set changed; mark global tour dirty.");
        }

        const double task_elapsed = task_execution_start_time_.isZero()
                                        ? std::numeric_limits<double>::infinity()
                                        : (now - task_execution_start_time_).toSec();
        const bool allow_frontier_switch =
            task_elapsed >= config_.minimum_task_execution_time;
        const bool covered = allow_frontier_switch && checkCurrentGoalCovered();
        const bool task_missing = allow_frontier_switch && !currentTaskExists();

        // Only validate the unexecuted portion. Validating current_global_path_ from
        // its historical start makes map changes behind the UAV trigger false replans.
        std::vector<Eigen::Vector3d> remaining_path;
        remaining_path.reserve(waypoint_queue_.size() - current_waypoint_idx_ + 1);
        remaining_path.push_back(odom_pos_);
        for (size_t i = static_cast<size_t>(current_waypoint_idx_);
             i < waypoint_queue_.size(); ++i) {
            remaining_path.push_back(waypoint_queue_[i].position);
        }
        const bool path_invalid = !global_tour_planner_->pathFinder().validatePath(
            remaining_path, UnknownSpacePolicy::BLOCK);

        if (covered || task_missing || path_invalid) {
            if (covered && current_task_key_.has_value() &&
                current_task_key_->type == ExplorationTaskType::FRONTIER) {
                // 前沿已覆盖：本轮内不再选择该任务
                task_blacklist_.insert(*current_task_key_);
            }
            if (path_invalid && current_task_key_.has_value()) {
                global_tour_planner_->invalidateTask(*current_task_key_);
            }

            const char* reason = covered ? "goal covered"
                                 : task_missing ? "task disappeared"
                                 : "scheduled path invalid";
            ROS_INFO("[Coverage] Replan trigger: %s. Attempting seamless switch...", reason);

            // 尝试触发无缝重规划
            if (triggerSeamlessReplan(path_invalid)) {
                // 如果成功，直接 return，下一帧就会开始执行新轨迹
                // 无需 transitState，保持在 EXEC_EXPLORATION 状态
                return;
            } else {
                // [新增] 无缝切换失败，降级到 LOCALPLAN 重新调度
                ROS_WARN("[Coverage] Seamless replan failed. Falling back to LOCALPLAN.");
                has_trajectory_ = false;
                transitState(ExplorationState::LOCALPLAN, "Seamless failed");
                return;
            }
        }
    }

    // 1. 碰撞检测 (安全第一)
    // 如果发现路径上有障碍物，立即中止并重规划
    if ((now - last_collision_check_time_).toSec() > config_.collision_check_interval) {
        last_collision_check_time_ = now;
        if (checkCollision()) {
            LOG_WARN("Collision detected ahead, stopping current task.");
            if (current_task_key_.has_value()) {
                global_tour_planner_->invalidateTask(*current_task_key_);
            }
            has_trajectory_ = false;
            // [Fix] Update hover state to prevent stale yaw
            transitState(ExplorationState::LOCALPLAN, "Collision detected");
            return;
        }
    }

    // ==================== [新增] Receding Horizon Replan Loop ====================
    const WaypointInfo& current_wp = waypoint_queue_[current_waypoint_idx_];
    double time_since_replan = (now - last_replan_time_).toSec();
    double dist_to_target = (current_wp.position - odom_pos_).norm();

    // 触发条件: 间隔 > 0.5s 且 距离目标 > 1.0m (太近了规划器会报错 Solver Error)
    bool need_replan = (time_since_replan > config_.replan_check_interval) && (dist_to_target > 1.0);

    if (need_replan)
    {
        Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();

         if (current_waypoint_idx_ + 1 < static_cast<int>(waypoint_queue_.size()))
         {
            const WaypointInfo& next_wp = waypoint_queue_[current_waypoint_idx_ + 1];
            Eigen::Vector3d dir = next_wp.position - current_wp.position;
            double dist = dir.norm();
            if (dist > 2.0) {
                target_vel = dir.normalized() * config_.max_vel * 0.8;
            }
         }

         // 调用带预测的规划接口 (use_latest_traj = true)
         // [Fix] 传入正确的 Yaw，否则会导致最后时刻 Yaw 对不准而卡死
         if (!planTrajectory(current_wp.position, current_wp.yaw, target_vel)) {
             ROS_WARN_THROTTLE(1.0, "[FSM] EXEC_EXPLORATION: Receding replan failed");
             // EGO 失败：使当前任务的缓存路径失效，下一轮重新求解
             if (current_task_key_.has_value()) {
                 global_tour_planner_->invalidateTask(*current_task_key_);
             }
         }
    }

    // 2. 航点追踪与切换 (核心执行逻辑)
    if (checkReachWaypoint())
    {
        // [Safe Switch] 尝试切换到下一个航点
        int next_idx = current_waypoint_idx_ + 1;

        // 2.1 抵达最终视点，任务完成
        if (next_idx >= static_cast<int>(waypoint_queue_.size())) {
            const WaypointInfo& final_wp = waypoint_queue_[current_waypoint_idx_];
            current_waypoint_idx_ = next_idx;
            ROS_INFO("[FSM] EXEC_EXPLORATION: Reached final viewpoint of task=%s#%d.",
                     taskTypeName(final_wp.task_key.type), final_wp.task_key.id);
            total_viewpoints_visited_++;

            // [Check] 再次确认标记（防止中途通过但未触发阈值的情况?）
            if (final_wp.task_key.type == ExplorationTaskType::SEMANTIC &&
                semantic_target_extractor_) {
                semantic_target_extractor_->markTargetObserved(final_wp.task_key.id);
                ROS_INFO("[FSM] Semantic Target %d Observation Confirmed.", final_wp.task_key.id);
            } else if (final_wp.task_key.type == ExplorationTaskType::FRONTIER) {
                // [修复] 对于前沿簇视点，到达终点后强制加入黑名单，防止原地由于覆盖率判断延迟导致的死循环
                task_blacklist_.insert(final_wp.task_key);
                ROS_INFO("[FSM] Frontier cluster %d added to blacklist to enforce progress.",
                         final_wp.task_key.id);
            }

            has_trajectory_ = false;
            transitState(ExplorationState::LOCALPLAN, "Reached Goal");
            return;
        }
        // 2.2 如果是中间点，先尝试规划，只有规划成功才真的切换
        int prev_idx = current_waypoint_idx_;
        current_waypoint_idx_ = next_idx; // 暂时切换，供 planNextWaypoint 使用

        if (planNextWaypoint()) {
             // 成功！确认切换
             ROS_INFO("[FSM] EXEC_EXPLORATION: Safe switch to WP%d success (dist<thresh).", next_idx);
        } else {
             // 失败！说明虽然距离满足阈值 (e.g. 2.0m)，但位置不好无法过弯
             // 回退索引，继续执行当前轨迹，飞得更近一点再试
             current_waypoint_idx_ = prev_idx;

             // 如果距离已经非常近 (<0.5m) 且还是规划失败，那只能报错重置了
             if ((waypoint_queue_[current_waypoint_idx_].position - odom_pos_).norm() < 0.5) {
                ROS_WARN("Failed to plan path to next waypoint (idx %d) even at close range.", next_idx);
                has_trajectory_ = false;
                transitState(ExplorationState::LOCALPLAN, "Plan Failed");
                return;
             }
             // 否则，保持沉默或低频警告，继续接近
             ROS_DEBUG_THROTTLE(1.0, "[FSM] Early switch to WP%d failed. Continuing approach to WP%d.", next_idx, prev_idx);
        }
    }

    // 3. 超时检测
    if (checkTimeout())
    {
        // [Step 1] 更新悬停点为当前位置 (防止 hover_pos_ 过期)
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;

        // [Step 2] 主动发送一次刹车指令 (覆盖掉可能的旧轨迹指令)
        // publishHoverCmd();

        // 继续运行只会执行错误的旧指令。
        has_trajectory_ = false;

        transitState(ExplorationState::LOCALPLAN, "Timeout");
    }
}

bool ExplorationFSMNew::checkCurrentGoalCovered() {
    if (!current_task_key_.has_value()) return false;

    // ========== 语义目标：使用视觉覆盖率检测 ==========
    if (current_task_key_->type == ExplorationTaskType::SEMANTIC) {
        if (!semantic_target_extractor_) return false;
        const int target_id = current_task_key_->id;

        // 先检查是否已被标记为观测，避免重复处理
        std::vector<TargetMetadata> unobserved_targets;
        semantic_target_extractor_->getUnobservedTargetsSnapshot(unobserved_targets);
        bool still_unobserved = false;
        for (const auto& target : unobserved_targets) {
            if (target.id == target_id) {
                still_unobserved = true;
                break;
            }
        }
        if (!still_unobserved) {
            // 目标已被标记为观测，直接返回 true 触发切换
            return true;
        }

        // 检查视觉覆盖率
        if (semantic_target_extractor_->isTargetVisualCovered(target_id, 0.8f)) {
            ROS_INFO("[FSM] Semantic Target %d Covered. Marking OBSERVED.", target_id);
            semantic_target_extractor_->markTargetObserved(target_id);
            return true;
        }
        return false; // 尚未覆盖
    }

    // ========== 前沿簇：未知体素活跃比例检测 ==========
    if (current_frontier_.empty()) return true; // 空数据默认已覆盖

    int active_anchors = 0;
    int total_anchors = current_frontier_.size();

    const float check_radius = 0.4f;
    const float check_sq = check_radius * check_radius;

    // 遍历所有锚点
    for (const auto& anchor : current_frontier_) {
        bool is_anchor_active = false;

        // 检查该锚点周围局部范围是否还有未知体素
        // 稀疏体素查询 (步长 0.2m)
        for (float dx = -check_radius; dx <= check_radius; dx += 0.2f) {
            for (float dy = -check_radius; dy <= check_radius; dy += 0.2f) {
                for (float dz = -check_radius; dz <= check_radius; dz += 0.2f) {
                    if (dx*dx+dy*dy+dz*dz > check_sq) continue;

                    Eigen::Vector3f p = anchor + Eigen::Vector3f(dx, dy, dz);
                    // 只要发现任意一个未知体素，该锚点就算"活跃"
                    if (global_map_->isUnknown(p)) {
                        is_anchor_active = true;
                        goto check_next_anchor; // 跳出内层循环
                    }
                }
            }
        }

        check_next_anchor:
        if (is_anchor_active) {
            active_anchors++;
        }
    }

    // 计算活跃比例
    double active_ratio = static_cast<double>(active_anchors) / total_anchors;

    // 如果活跃锚点比例低于 30% (即 >70% 已覆盖)，则认为目标已完成
    if (active_ratio < 0.3) {
        ROS_INFO("[Coverage] Covered! Active ratio: %.2f (%d/%d)", active_ratio, active_anchors, total_anchors);
        return true;
    }

    return false;
}

// ==================== LOCALPLAN 核心方法 ====================
// 旧单点视点选择/路径规划已移除（由 GlobalTourPlanner 统一调度）

// 切换到下一个航点
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
    
    return planTrajectory(wp.position, wp.yaw, target_vel);
}

// [重构] 无缝重规划触发器：统一任务 + 全局调度
bool ExplorationFSMNew::triggerSeamlessReplan(bool force_replace_current)
{
    double t_now = ros::Time::now().toSec();
    double t_cur = t_now - traj_start_time_;
    double t_predict = t_cur + config_.trajectory_switch_delay;

    // 快到终点了就不切换了
    if (t_predict > traj_duration_ - 0.5) return false;

    auto& traj = planner_manager_->traj_.local_traj.traj;
    if (traj.getPieceNum() == 0) return false;

    // 1. 用预测位置构造机器人状态（选点与路径生成使用同一起点）
    RobotPlanningState robot = snapshotRobotState();
    robot.position = traj.getPos(t_predict);
    robot.velocity = traj.getVel(t_predict);
    robot.valid = true;

    // 2. 采集统一任务并求解全局巡游
    std::vector<ExplorationTask> tasks = collectExplorationTasks();
    last_active_task_keys_ = activeTaskKeys();
    if (tasks.empty()) return false;

    GlobalTourPlan plan = global_tour_planner_->plan(robot, tasks, current_task_key_);
    current_tour_plan_ = plan;
    if (!plan.success) {
        ROS_WARN("[Seamless] Global tour planning failed: %s", plan.failure_reason.c_str());
        return false;
    }

    // 3. 目标未变化时保持当前轨迹，避免无意义切换
    if (!force_replace_current && current_task_key_.has_value() &&
        plan.next_goal.task_key == *current_task_key_) {
        ROS_DEBUG("[Seamless] Global scheduler keeps current task=%s#%d.",
                  taskTypeName(plan.next_goal.task_key.type), plan.next_goal.task_key.id);
        // The old trajectory remains valid. This is a successful no-op, not a
        // planning failure that should force the FSM back to LOCALPLAN.
        global_tour_dirty_ = false;
        return true;
    }

    // 4. 重验证新路径
    if (!global_tour_planner_->pathFinder().validatePath(
            plan.next_goal.path, UnknownSpacePolicy::BLOCK)) {
        ROS_WARN("[Seamless] New scheduled path is not known-free.");
        return false;
    }

    // 5. 生成航点队列并启动轨迹
    if (!planToScheduledGoal(plan.next_goal)) {
        task_blacklist_.insert(plan.next_goal.task_key);
        global_tour_planner_->invalidateTask(plan.next_goal.task_key);
        return false;
    }

    // 6. 锁定任务与可视化状态
    current_task_key_ = plan.next_goal.task_key;
    task_execution_start_time_ = ros::Time::now();
    global_tour_dirty_ = false;
    current_vp_pos_ = plan.next_goal.viewpoint;
    current_vp_yaw_ = plan.next_goal.yaw;
    current_frontier_.clear();
    current_viewpoint_centroid_ = plan.next_goal.viewpoint;
    if (current_task_key_->type == ExplorationTaskType::FRONTIER) {
        for (const auto& cluster : viewpoint_manager_->getClusters()) {
            if (cluster.cluster_id == current_task_key_->id) {
                current_viewpoint_centroid_ = cluster.centroid.cast<double>();
                current_frontier_ = cluster.points;
                break;
            }
        }
    } else {
        for (const auto& task : tasks) {
            if (task.key == *current_task_key_) {
                current_viewpoint_centroid_ = task.centroid;
                break;
            }
        }
    }

    ROS_WARN("[Seamless] Replan Success -> task=%s#%d",
             taskTypeName(current_task_key_->type), current_task_key_->id);
    return true;
}

bool ExplorationFSMNew::planTrajectory(const Eigen::Vector3d& target, double target_yaw, 
    const Eigen::Vector3d& target_vel) 
{
    Eigen::Vector3d start_pos, start_vel, start_acc;
    double start_yaw;
    double start_yaw_rate = 0.0;
    double start_yaw_acc = 0.0;
    double t_now = ros::Time::now().toSec();
    const auto previous_local_traj = planner_manager_->traj_.local_traj;

    // === 统一预测逻辑 ===
    if (has_trajectory_) {
        // [模式A] 预测模式：从当前轨迹的未来点开始，实现平滑衔接
        has_prev_traj_ = true;
        prev_traj_start_time_ = traj_start_time_;
        prev_traj_duration_ = traj_duration_;
        prev_traj_ = planner_manager_->traj_.local_traj.traj;
        
        // [新增] 保存旧Yaw轨迹控制点，供过渡期直接采样
        auto& old_local_data = planner_manager_->traj_.local_traj;
        prev_use_yaw_traj_ = old_local_data.use_yaw_traj;
        prev_last_yaw_ = last_yaw_;  // 保存当前last_yaw_用于后备分支
        if (old_local_data.use_yaw_traj && !old_local_data.yaw_ctrl_pts.empty()) {
            prev_yaw_ctrl_pts_ = old_local_data.yaw_ctrl_pts;
        } else {
            prev_yaw_ctrl_pts_.clear();
        }

        const double lookahead = config_.trajectory_switch_delay;
        double t_cur = t_now - traj_start_time_;
        double t_predict = std::min(t_cur + lookahead, traj_duration_);

        auto& traj = planner_manager_->traj_.local_traj.traj;
        auto& local_data = planner_manager_->traj_.local_traj;
        
        start_pos = traj.getPos(t_predict);
        start_vel = traj.getVel(t_predict);
        start_acc = traj.getAcc(t_predict);

        // [关键修复] 预测 Yaw (从 Yaw 轨迹采样或从速度方向推断)
        if (local_data.use_yaw_traj && !local_data.yaw_ctrl_pts.empty()) {
            start_yaw = ego_planner::YawOptimizer::evaluateYaw(
                local_data.yaw_ctrl_pts, t_predict, local_data.duration);
            start_yaw_rate = ego_planner::YawOptimizer::evaluateYawDot(
                local_data.yaw_ctrl_pts, t_predict, local_data.duration);
            start_yaw_acc = ego_planner::YawOptimizer::evaluateYawDDot(
                local_data.yaw_ctrl_pts, t_predict, local_data.duration);
        } else {
            // Fallback: 从速度方向推断
            // 注意：atan2 返回 [-π, π]，可能导致不连续
            // 使用 last_yaw_ 的展开版本来保持连续
            double vel_yaw = (start_vel.norm() > 0.1) ? atan2(start_vel.y(), start_vel.x()) : 0.0;
            // 选择最接近 last_yaw_ 的等价角度
            double diff = vel_yaw - last_yaw_;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            start_yaw = last_yaw_ + diff;
        }

        // [诊断] 检查预测Yaw与里程计Yaw的偏差
        // 注意：不再修改start_yaw，因为这会导致360°跳变
        // YawOptimizer和calculate_yaw已有动力学限制，确保yaw变化平滑
        double yaw_diff = start_yaw - odom_yaw_;
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

        if (std::abs(yaw_diff) > 45.0 * M_PI / 180.0) { // 阈值 45度
            ROS_DEBUG("[FSM] Yaw tracking lag: %.2f deg (start_yaw=%.2f, odom_yaw=%.2f)", 
                     yaw_diff * 180.0 / M_PI, start_yaw * 180.0 / M_PI, odom_yaw_ * 180.0 / M_PI);
            // 不再重置start_yaw，保持展开空间连续性
        }

        planner_manager_->setReplanStartTime(traj_start_time_ + t_predict);
    } 
    else {
        // [模式B] 冷启动模式：从当前位置开始 (初始规划)
        start_pos = odom_pos_;
        start_vel = odom_vel_;
        start_acc = Eigen::Vector3d::Zero();
        start_yaw = odom_yaw_;
        start_yaw_rate = 0.0;
        start_yaw_acc = 0.0;
        planner_manager_->setReplanStartTime(t_now);
    }

    end_yaw_ = target_yaw; 
    // The height term is a soft cost, so point it at the actual local goal rather
    // than perpetuating any transient altitude error at the replan start.
    planner_manager_->setDesiredHeight(target.z());
    
    // 调用 Planner (传入预测的 start_yaw)
    bool success = planner_manager_->reboundReplan(
        start_pos, start_vel, start_acc, target, target_vel, 
        true, false, true, start_yaw, target_yaw,
        start_yaw_rate, start_yaw_acc);

    if (success) {
        // The optimizer's height term is not a hard constraint. Reject a
        // trajectory that leaves the only map used by scheduling and safety.
        const auto& candidate = planner_manager_->traj_.local_traj.traj;
        const double candidate_duration = planner_manager_->traj_.local_traj.duration;
        const double min_z = static_cast<double>(global_map_->getGlobalMin().z()) +
                             config_.trajectory_bounds_margin;
        const double max_z = static_cast<double>(global_map_->getGlobalMax().z()) -
                             config_.trajectory_bounds_margin;
        bool in_bounds = candidate.getPieceNum() > 0 && min_z < max_z;
        for (double t = 0.0; in_bounds && t <= candidate_duration + 1e-6; t += 0.05) {
            const Eigen::Vector3d p = candidate.getPos(std::min(t, candidate_duration));
            in_bounds = p.allFinite() && p.z() >= min_z && p.z() <= max_z &&
                        global_map_->isInGlobalMapRange(p.cast<float>());
        }
        if (!in_bounds) {
            ROS_WARN("[FSM] Rejecting local trajectory outside map bounds z=[%.2f, %.2f]",
                     min_z, max_z);
            planner_manager_->traj_.local_traj = previous_local_traj;
            has_prev_traj_ = false;
            return false;
        }

        traj_start_time_ = planner_manager_->traj_.local_traj.start_time;
        traj_duration_ = planner_manager_->traj_.local_traj.duration;
        has_trajectory_ = true;
        last_replan_time_ = ros::Time::now();
        trajectory_start_time_ = ros::Time(traj_start_time_);
        
    } 
    else {
        // 确保 executionCallback 继续执行当前轨迹，而不是尝试切换到不存在的新轨迹
        has_prev_traj_ = false;
    }

    return success;
}

// ============ 探索轨迹 Yaw 计算 ============
std::pair<double, double> ExplorationFSMNew::calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
    constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
    constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    
    auto& local_data = planner_manager_->traj_.local_traj;

    // ============ [优先] 使用YawOptimizer生成的B样条轨迹 ============
    // 优先使用规划器生成的 yaw 轨迹。
    if (local_data.use_yaw_traj && !local_data.yaw_ctrl_pts.empty()) {
        double t_eval = t_cur; 
        if (t_eval < 0) t_eval = 0;
        if (t_eval > local_data.duration) t_eval = local_data.duration;

        double yaw = ego_planner::YawOptimizer::evaluateYaw(
            local_data.yaw_ctrl_pts, t_eval, local_data.duration);
        double yaw_dot = ego_planner::YawOptimizer::evaluateYawDot(
            local_data.yaw_ctrl_pts, t_eval, local_data.duration);

        last_yaw_ = yaw;
        last_yaw_dot_ = yaw_dot;
        return {yaw, yaw_dot};
    }

    // 1. 计算速度方向 Yaw (Velocity Yaw)
    double time_forward = 2.0; 
    Eigen::Vector3d dir;
    
    // Helper lambda for angle normalization
    auto normalizeAngle = [](double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };

    auto angleDiff = [&](double target, double current) {
        double diff = target - current;
        return normalizeAngle(diff);
    };

    poly_traj::Trajectory* active_traj_ptr = &planner_manager_->traj_.local_traj.traj;
    // local_data已在函数开头定义
    if (t_cur + time_forward <= traj_duration_) {
        dir = active_traj_ptr->getPos(t_cur + time_forward) - pos;
    } else {
        dir = active_traj_ptr->getPos(traj_duration_) - pos;
    }

    double velocity_yaw = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double yaw_temp = velocity_yaw; // Default to velocity yaw

    // 2. 混合策略 (Fly-and-Turn)
    // - 前70%：跟随速度方向（看路）
    // - 后30%：平滑过渡到目标Yaw（看目标）
    // 当前提是 target_yaw 有效（在这个函数里 end_yaw_ 应该是有效的）
    
    double progress = std::min(1.0, t_cur / traj_duration_);
    const double transition_start = 0.7;  // 70% 时开始过渡

    if (progress > transition_start) {
        // 后 30%：从当前速度方向平滑过渡到目标 yaw
        double transition_progress = (progress - transition_start) / (1.0 - transition_start);
        // 使用平滑插值函数: 3*t^2 - 2*t^3
        double smooth_progress = transition_progress * transition_progress * (3.0 - 2.0 * transition_progress);
        
        // 从速度方向过渡到目标 yaw
        double yaw_diff = angleDiff(end_yaw_, velocity_yaw);
        yaw_temp = normalizeAngle(velocity_yaw + smooth_progress * yaw_diff);
    }

    // 3. 动态限制 (Dynamic Limits)
    double d_yaw = angleDiff(yaw_temp, last_yaw_);

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
    double yaw = normalizeAngle(last_yaw_ + d_yaw);

    // 更新状态
    last_yaw_ = yaw;
    last_yaw_dot_ = yaw_dot;

    return {yaw, yaw_dot};
}

void ExplorationFSMNew::executionCallback(const ros::TimerEvent& e) {
    const bool in_exec_state = state_ == ExplorationState::EXEC_EXPLORATION;
    
    if (!in_exec_state || !has_trajectory_) {
        return;
    }

    ros::Time now = ros::Time::now();
    double t_cur = (now.toSec() - traj_start_time_);
    static ros::Time last_time = now;
    double dt = (now - last_time).toSec();
    if (dt <= 0) dt = 0.01;

    // [关键修复] 首次进入时初始化状态
    if (!exec_initialized_) {
        last_pos_ = odom_pos_;
        last_yaw_ = odom_yaw_;
        last_yaw_dot_ = 0.0;
        exec_initialized_ = true;
        ROS_INFO("[Exec] Initialized: pos=(%.2f,%.2f,%.2f), yaw=%.2f", 
                 last_pos_.x(), last_pos_.y(), last_pos_.z(), last_yaw_);
    }

    Eigen::Vector3d pos, vel, acc;
    std::pair<double, double> yaw_state;

    // Case 1: 新轨迹还没开始 (预测期) -> 执行旧轨迹 (Seamless Stitching)
    if (t_cur < 0) 
    {
        if (has_prev_traj_) 
        {
            double t_prev = (now.toSec() - prev_traj_start_time_);
            // [Fix] Add safety check for empty trajectory to prevent SEGFAULT
            if (prev_traj_.getPieceNum() > 0 && t_prev < prev_traj_duration_) 
            {
                // [Fix] Execute previous trajectory instead of holding
                // Clamp time to total duration to be safe (though < prev_traj_duration_ check helps)
                double t_eval = std::min(t_prev, prev_traj_.getTotalDuration());
                pos = prev_traj_.getPos(t_eval);
                vel = prev_traj_.getVel(t_eval);
                acc = prev_traj_.getAcc(t_eval);

                double yaw, yaw_dot;
                
                // [优化] 直接从旧Yaw轨迹采样，无需繁琐的速度方向计算
                if (prev_use_yaw_traj_ && !prev_yaw_ctrl_pts_.empty()) 
                {
                    yaw = ego_planner::YawOptimizer::evaluateYaw(
                        prev_yaw_ctrl_pts_, t_eval, prev_traj_duration_);
                    yaw_dot = ego_planner::YawOptimizer::evaluateYawDot(
                        prev_yaw_ctrl_pts_, t_eval, prev_traj_duration_);
                } else {
                    // 后备：使用速度方向，展开到旧轨迹的last_yaw
                    Eigen::Vector3d dir = vel;
                    double velocity_yaw = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : prev_last_yaw_;
                    
                    // 展开到prev_last_yaw_附近（保证连续性）
                    double diff = velocity_yaw - prev_last_yaw_;
                    while (diff > M_PI) diff -= 2 * M_PI;
                    while (diff < -M_PI) diff += 2 * M_PI;
                    yaw = prev_last_yaw_ + diff;
                    yaw_dot = 0.0;
                }
                
                last_yaw_ = yaw;
                last_yaw_dot_ = yaw_dot;
                last_pos_ = pos;
                publishCmd(pos, vel, acc, yaw, yaw_dot);
            } 
            else 
            {
                // [关键修复] Old traj finished, hold at last known position
                // 更新 last_pos_ 为当前里程计位置，避免悬停在错误位置
                last_pos_ = odom_pos_;
                publishCmd(last_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
            }
        } 
        else 
        {
             // [关键修复] 无 prev_traj，使用当前里程计位置悬停
             last_pos_ = odom_pos_;
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
        
        double t_clamped = std::min(t_cur, traj_duration_);
        yaw_state = calculate_yaw(t_clamped, pos, dt);
        last_pos_ = pos;
        
        publishCmd(pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), yaw_state.first, yaw_state.second);
    }

    static int vis_counter = 0;
    if (++vis_counter >= 10) { // 100Hz下每20次就是5Hz，足够流畅且不卡顿
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
    
    cmd.position.x =p.x();
    cmd.position.y = p.y();
    cmd.position.z = p.z();
    
    cmd.velocity.x = v.x();
    cmd.velocity.y = v.y();
    cmd.velocity.z = v.z();
    
    cmd.acceleration.x = a.x();
    cmd.acceleration.y = a.y();
    cmd.acceleration.z = a.z();
    
    // [关键修复] 不归一化 Yaw！直接使用展开角度
    // 原因：SO3Control 使用 cos(yaw) 和 sin(yaw)，天然处理任意角度
    // 归一化会在跨越 ±π 边界时产生跳变（如 3.15 -> -3.13）
    // 保持展开空间的连续性，避免控制器看到突变指令
    cmd.yaw = yaw;
    cmd.yaw_dot = yaw_dot;
    
    // [新增] 记录轨迹日志
    if (enable_traj_logging_ && traj_log_file_.is_open()) {
        double timestamp = cmd.header.stamp.toSec();
        
        // 获取当前里程计状态
        Eigen::Vector3d odom_p, odom_v;
        double odom_y;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_p = odom_pos_;
            odom_v = odom_vel_;
            odom_y = odom_yaw_;
        }
        
        // 获取轨迹终点位置
        Eigen::Vector3d traj_end = p;  // 默认当前位置
        if (has_trajectory_ && planner_manager_) {
            auto& traj = planner_manager_->traj_.local_traj.traj;
            if (traj.getPieceNum() > 0) {
                traj_end = traj.getPos(traj_duration_);
            }
        }
        
        // 写入 CSV
        traj_log_file_ << std::fixed << std::setprecision(6)
                       << timestamp << ","
                       << p.x() << "," << p.y() << "," << p.z() << ","
                       << v.x() << "," << v.y() << "," << v.z() << ","
                       << yaw << "," << yaw_dot << ","
                       << odom_p.x() << "," << odom_p.y() << "," << odom_p.z() << ","
                       << odom_v.x() << "," << odom_v.y() << "," << odom_v.z() << ","
                       << odom_y << ","
                       << traj_end.x() << "," << traj_end.y() << "," << traj_end.z() << ","
                       << traj_start_time_ << "," << end_yaw_ << "\n";
    }
    
    pos_cmd_pub_.publish(cmd);
}

// ==================== EXEC_EXPLORATION 检测方法 ====================
bool ExplorationFSMNew::checkCollision() {
    // 如果没有有效轨迹，不检测碰撞
    if (!has_trajectory_) {
        return false;
    }
    
    // 使用成员变量的轨迹时间（与重建轨迹一致）
    double t_cur = ros::Time::now().toSec() - traj_start_time_;
    
    if (t_cur < 0 || t_cur > traj_duration_) {
        return false;
    }
    
    // 检查轨迹是否有效
    auto& traj = planner_manager_->traj_.local_traj.traj;
    if (traj.getPieceNum() == 0) {
        return false;
    }
    
    // 检查当前位置 + 前瞻时间
    for (double dt = 0; dt <= config_.collision_lookahead; dt += 0.1) {
        double check_t = t_cur + dt;
        if (check_t > traj_duration_) break;
        
        Eigen::Vector3d pos = traj.getPos(check_t);
        
        // 检查NaN
        if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z())) {
            continue;
        }
        
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
        // 使用稍大的阈值 (e.g., 0.4m) 保证流畅性，但不能太大否则会切墙
        return dist < std::max(config_.reach_threshold, 2.0); 
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
    // 动态超时判定：基于实际规划时长 + 5秒缓冲
    double timeout_thresh = std::max(config_.trajectory_timeout, traj_duration_ + 5.0);
    if(elapsed > timeout_thresh)
    {
        if (waypoint_queue_.empty() ||
            current_waypoint_idx_ >= static_cast<int>(waypoint_queue_.size())) {
            ROS_WARN("[FSM] EXEC_EXPLORATION: Trajectory execution timeout (waypoint queue empty).");
            return true;
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
        ROS_WARN("[FSM] EXEC_EXPLORATION: Trajectory execution timeout. Aborting. dist: %.2f, yaw_diff: %.2f", dist, yaw_diff);
    }
    return elapsed > timeout_thresh;
}

// ==================== 辅助函数 ====================
double ExplorationFSMNew::getYawDiff(double yaw1, double yaw2) {
    double diff = yaw1 - yaw2;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

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
    visualizeGlobalTour();
    visualizeRefinedTour();
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

// [New] 可视化全局巡游顺序（局部精化窗口）
void ExplorationFSMNew::visualizeGlobalTour() {
    if (global_tour_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);

    const auto& plan = current_tour_plan_;
    if (!plan.success) {
        global_tour_pub_.publish(markers);
        return;
    }

    int marker_id = 0;
    for (size_t i = 0; i < plan.refined_viewpoints.size(); ++i) {
        const Eigen::Vector3d& point = plan.refined_viewpoints[i];

        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = ros::Time::now();
        sphere.ns = "global_tour_nodes";
        sphere.id = marker_id++;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose.position.x = point.x();
        sphere.pose.position.y = point.y();
        sphere.pose.position.z = point.z();
        sphere.scale.x = 0.35;
        sphere.scale.y = 0.35;
        sphere.scale.z = 0.35;
        sphere.color.r = 0.1f;
        sphere.color.g = 0.9f;
        sphere.color.b = 0.3f;
        sphere.color.a = 0.9f;
        markers.markers.push_back(sphere);

        visualization_msgs::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = ros::Time::now();
        text.ns = "global_tour_order";
        text.id = marker_id++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position.x = point.x();
        text.pose.position.y = point.y();
        text.pose.position.z = point.z() + 0.6;
        text.scale.z = 0.5;
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 1.0f;
        text.text = std::to_string(i + 1);
        markers.markers.push_back(text);
    }

    global_tour_pub_.publish(markers);
}

// [New] 可视化局部精化巡游路径
void ExplorationFSMNew::visualizeRefinedTour() {
    if (refined_tour_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);

    const auto& plan = current_tour_plan_;
    if (!plan.success || plan.refined_viewpoints.empty()) {
        refined_tour_pub_.publish(markers);
        return;
    }

    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "refined_tour";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.08;
    line.color.r = 1.0f;
    line.color.g = 0.5f;
    line.color.b = 0.0f;
    line.color.a = 0.9f;

    geometry_msgs::Point start;
    start.x = odom_pos_.x();
    start.y = odom_pos_.y();
    start.z = odom_pos_.z();
    line.points.push_back(start);
    for (const auto& point : plan.refined_viewpoints) {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        line.points.push_back(p);
    }
    markers.markers.push_back(line);

    refined_tour_pub_.publish(markers);
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
        bool is_current = current_task_key_.has_value() &&
                          current_task_key_->type == ExplorationTaskType::FRONTIER &&
                          cluster.cluster_id == current_task_key_->id;
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
        
        bool is_current = current_task_key_.has_value() &&
                          current_task_key_->type == ExplorationTaskType::FRONTIER &&
                          cluster.cluster_id == current_task_key_->id;
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
    global_tour_pub_.publish(markers);
    refined_tour_pub_.publish(markers);
}



} // namespace rog_map
