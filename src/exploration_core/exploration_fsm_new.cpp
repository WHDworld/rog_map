/**
 * @file exploration_fsm_new.cpp
 * @brief 探索状态机实现 - 重构版（4状态 + 集成规划器）
 */

#include "exploration/exploration_fsm_new.h"
#include "ego_planner/yaw_optimizer.h"
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iomanip>
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

    // ==================== [新增] 视点选择稳定性参数 ====================
    nh_.param("exploration/hysteresis_gain", vp_selection_config_.hysteresis_gain, 1.2);
    nh_.param("exploration/heading_penalty_weight", vp_selection_config_.heading_penalty_weight, 0.5);
    nh_.param("exploration/min_switch_interval", vp_selection_config_.min_switch_interval, 0.5);
    nh_.param("exploration/anchor_lock_distance", vp_selection_config_.anchor_lock_distance, 2.0);
    nh_.param("exploration/velocity_threshold", vp_selection_config_.velocity_threshold, 0.3);
    last_viewpoint_switch_time_ = ros::Time::now();
    // 创建模块
    history_graph_ = std::make_shared<HistoryGraph>();
    history_graph_->init(nh_, global_map_);
    
    viewpoint_manager_ = std::make_shared<ViewpointManager>();
    viewpoint_manager_->init(nh_, global_map_, history_graph_);
        
    // 创建规划器管理器
    planner_manager_.reset(new ego_planner::EGOPlannerManager);
    planner_manager_->initPlanModules(nh_, map_manager_);
    
    // 创建目标重建模块
    target_reconstructor_ = std::make_shared<TargetReconstructor>();
    target_reconstructor_->init(nh_, global_map_);  // 传入全局地图用于遮挡检测
    
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
    region_route_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/region_route", 1); // [Visual] 历史节点网络
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
            std::string log_file = log_dir + "/trajectory_log.csv";
            
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
        case ExplorationState::RECON_PLAN: return "RECON_PLAN";
        case ExplorationState::EXEC_RECON: return "EXEC_RECON";
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
    ExplorationState old_state = state_.load();  // 保存旧状态用于速度缩放判断
    state_ = new_state;

    std::string new_state_str;
    switch (new_state) {
        case ExplorationState::INIT: new_state_str = "INIT"; break;
        case ExplorationState::PRE_ARM: new_state_str = "PRE_ARM"; break;
        case ExplorationState::TAKEOFF: new_state_str = "TAKEOFF"; break;
        case ExplorationState::WAIT_TRIGGER: new_state_str = "WAIT_TRIGGER"; break;
        case ExplorationState::LOCALPLAN: new_state_str = "LOCALPLAN"; break;
        case ExplorationState::EXEC_EXPLORATION: new_state_str = "EXEC_EXPLORATION"; break;
        case ExplorationState::RECON_PLAN: new_state_str = "RECON_PLAN"; break;
        case ExplorationState::EXEC_RECON: new_state_str = "EXEC_RECON"; break;
        case ExplorationState::FINISH: new_state_str = "FINISH"; break;
        default: new_state_str = "UNKNOWN"; break;
    }
    
    // ROS_INFO("[FSM_NEW] TRANSIT: %s -> %s (%s)", 
    //          old_state_str.c_str(), new_state_str.c_str(), reason.c_str());
    
    // [新增] 速度缩放处理：重建任务使用 0.5 倍速，探索任务使用 1.0 倍速
    bool is_recon_state = (new_state == ExplorationState::RECON_PLAN || 
                           new_state == ExplorationState::EXEC_RECON);
    bool was_recon_state = (old_state == ExplorationState::RECON_PLAN || 
                            old_state == ExplorationState::EXEC_RECON);
    
    if (is_recon_state && !was_recon_state) {
        // 刚进入重建状态，降低速度
        planner_manager_->setVelocityScale(0.5);
    } else if (!is_recon_state && was_recon_state) {
        // 刚离开重建状态，恢复正常速度
        planner_manager_->setVelocityScale(1.0);
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
        case ExplorationState::RECON_PLAN:
            processReconPlan(e);
            break;
        case ExplorationState::EXEC_RECON:
            processExecRecon(e);
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
    history_graph_->updateUAVState(odom_pos_);
    target_reconstructor_->updateDronePosition(odom_pos_);
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
        history_graph_->start();
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
        
        history_graph_->start();
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
        // [优化] 优先检查重建任务，实现起飞即重建
        // if (target_reconstructor_->hasReconstructionTask()) {
        //     ROS_INFO("[FSM] Detected reconstruction task at startup, switching to RECON_PLAN");
        //     transitState(ExplorationState::RECON_PLAN, "recon_task_at_startup");
        //     return;
        // }
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
    ros::WallTime t_start = ros::WallTime::now();
    
    // 1. 选择最佳视点 (两阶段 + 缓存)
    int selected_vp_id = -1;
    if (!selectBestViewpoint(selected_vp_id)) {
        // 真的没视点了，检查是否任务完成
        if (!viewpoint_manager_->hasValidViewpoints()) {
            LOG_INFO("Exploration Complete! No more viewpoints.");
            transitState(ExplorationState::WAIT_TRIGGER, "Done");
        } else {
            // 可能全被拉黑了，重置黑名单重试
            if (!blacklisted_clusters_.empty()) {
                 ROS_WARN("[FSM] All VPs blacklisted. Clearing blacklist to retry.");
                 blacklisted_clusters_.clear();
            }
            // 可能只是暂时没算出来，悬停等待
            ROS_WARN_THROTTLE(1.0, "[FSM] No reachable viewpoint found, retrying...");
        }
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        publishHoverCmd();
        return;
    }
    ros::WallTime t_select_vp = ros::WallTime::now();
    // 2. 生成导航路径 (复用缓存)
    if (!planToViewpoint(selected_vp_id)) {
        ROS_WARN("[FSM] Failed to generate path to VP %d. Blacklisting it.", selected_vp_id);
        
        // [New] 加入黑名单，强制下一次选择其他视点
        blacklisted_clusters_.insert(selected_vp_id);
        
        // 规划失败直接放弃本次决策，悬停，等待下一轮 FSM 循环重新选择
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        publishHoverCmd();
        return;
    }
    else {
        // [New] 规划成功，清空黑名单
        blacklisted_clusters_.clear();
    }
    ros::WallTime t_plan_to_vp = ros::WallTime::now();
    // 3. 锁定目标，准备起飞
    current_viewpoint_id_ = selected_vp_id;
    
    // 重置计时器
    last_collision_check_time_ = ros::Time::now();
    last_coverage_check_time_ = ros::Time::now();
    has_prev_traj_ = false;  // [New] 清空旧轨迹缓存
    
    ROS_INFO("[FSM] LOCALPLAN: processLocalPlan success, current_viewpoint_id_: %d, select_vp_time: %.2fms, plan_to_vp_time: %.2fms, plan all_time: %.2fms", 
        current_viewpoint_id_, (t_select_vp - t_start).toSec() * 1000.0, (t_plan_to_vp - t_select_vp).toSec() * 1000.0, (ros::WallTime::now() - t_start).toSec() * 1000.0);
    PERF_LOG.logf("SCHED", "LOCAL_PLAN",
        "viewpoint=%d select_ms=%.3f route_and_traj_ms=%.3f total_ms=%.3f waypoints=%zu",
        current_viewpoint_id_, (t_select_vp - t_start).toSec() * 1000.0,
        (t_plan_to_vp - t_select_vp).toSec() * 1000.0,
        (ros::WallTime::now() - t_start).toSec() * 1000.0, waypoint_queue_.size());
    transitState(ExplorationState::EXEC_EXPLORATION, "Plan success");
}

void ExplorationFSMNew::processExecExploration() {
    // [新增] 检查重建任务 (每0.5秒)
    if ((ros::Time::now() - last_recon_check_time_).toSec() > 0.5) {
        last_recon_check_time_ = ros::Time::now();
        
        // if (target_reconstructor_->hasReconstructionTask()) {
        //     ROS_INFO("[FSM] Detected reconstruction task, from ExecExploration switching to RECON_PLAN");
        //     transitState(ExplorationState::RECON_PLAN, "recon_task_detected");
        //     return;
        // }
    }
    // 0. 基础状态检查
    if (!has_trajectory_) {
        transitState(ExplorationState::LOCALPLAN, "No trajectory");
        return;
    }
    
    ros::Time now = ros::Time::now();

        // ==================== [新增] 覆盖检测与无缝切换 ====================
    // 频率控制: 2Hz (每0.5秒检查一次)
    if ((now - last_coverage_check_time_).toSec() > 0.5) {
        last_coverage_check_time_ = now;
        
        // Step 1: 检查当前目标是否已满足覆盖条件 (>80% 锚点覆盖)
        if (checkCurrentGoalCovered()) {
            ROS_INFO("[Coverage] Target covered early. Attempting seamless switch...");
            
            // Step 2: 尝试触发无缝重规划
            if (triggerSeamlessReplan()) {
                // 如果成功，直接 return，下一帧就会开始执行新轨迹
                // 无需 transitState，保持在 EXEC_EXPLORATION 状态
                return; 
            } else {
                // [新增] 无缝切换失败，降级到 LOCALPLAN 重新选择视点
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
            has_trajectory_ = false;
            // [Fix] Update hover state to prevent stale yaw
            transitState(ExplorationState::LOCALPLAN, "Collision detected");
            return;
        }
    }
    
    // ==================== [新增] Receding Horizon Replan Loop ====================
    double time_since_replan = (now - last_replan_time_).toSec();
    double dist_to_target = (waypoint_queue_[current_waypoint_idx_].position - odom_pos_).norm();
    
    // 触发条件: 间隔 > 0.5s 且 距离目标 > 1.5m (太近了规划器会报错 Solver Error)
    bool need_replan = (time_since_replan > config_.replan_check_interval) && (dist_to_target > 1.0);
    
    if (need_replan) 
    {
        const WaypointInfo& wp = waypoint_queue_[current_waypoint_idx_];
        Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
         
         if (current_waypoint_idx_ + 1 < waypoint_queue_.size()) 
         {
            const WaypointInfo& next_wp = waypoint_queue_[current_waypoint_idx_ + 1];
            Eigen::Vector3d dir = next_wp.position - wp.position;
            double dist = dir.norm();
            if (dist > 2.0) {
                target_vel = dir.normalized() * config_.max_vel * 0.8;
            }
         }

         // 调用带预测的规划接口 (use_latest_traj = true)
         // [Fix] 传入正确的 Yaw，否则会导致最后时刻 Yaw 对不准而卡死
         if (planTrajectory(wp.position, wp.yaw, target_vel)) {
            // ROS_INFO("[FSM] EXEC_EXPLORATION: Receding replan success, plan time: %f ms", (ros::Time::now().toSec() - t_start) * 1000.0);
         } else {
             ROS_WARN_THROTTLE(1.0, "[FSM] EXEC_EXPLORATION: Receding replan failed");
         }
    }
    
    // 2. 航点追踪与切换 (核心执行逻辑)
    if (checkReachWaypoint()) 
    {
        // [Safe Switch] 尝试切换到下一个航点
        int next_idx = current_waypoint_idx_ + 1;
        
        // 2.1 抵达最终视点，任务完成
        if (next_idx >= static_cast<int>(waypoint_queue_.size())) {
            current_waypoint_idx_ = next_idx;
            ROS_INFO("[FSM] EXEC_EXPLORATION: Reached Final Viewpoint %d. Task Complete.", current_viewpoint_id_);
            total_viewpoints_visited_++;
            
            // [Check] 再次确认标记（防止中途通过但未触发阈值的情况?）
            if (current_viewpoint_id_ < -1 && semantic_target_extractor_) {
                int target_id = -(current_viewpoint_id_ + 100);
                semantic_target_extractor_->markTargetObserved(target_id);
                ROS_INFO("[FSM] Semantic Target %d Observation Confirmed.", target_id);
            } else if (current_viewpoint_id_ >= 0) {
                // [修复] 对于普通Cluster视点，到达终点后强制加入黑名单，防止原地由于覆盖率判断延迟导致的死循环
                blacklisted_clusters_.insert(current_viewpoint_id_);
                ROS_INFO("[FSM] Cluster %d added to blacklist to enforce progress.", current_viewpoint_id_);
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

void ExplorationFSMNew::processReconPlan(const ros::TimerEvent& e) {
    ros::Time t_start = ros::Time::now();
    
    // 1. 获取当前无人机位置
    Eigen::Vector3d current_pos = odom_pos_;
    
    // ==================== [新增] Step 0: 获取目标中心并判断距离/Yaw ====================
    Eigen::Vector3d target_center;
    if (!target_reconstructor_->getActiveTargetCenter(target_center)) {
        ROS_WARN("[RECON] No active target center, returning to LOCALPLAN");
        recon_approaching_ = false;
        planner_manager_->setEnableHeightKeeping(true);
        transitState(ExplorationState::LOCALPLAN, "no_active_target");
        return;
    }
    
    // 缓存目标中心用于后续使用
    recon_target_center_ = target_center;
    
    // 计算到目标的距离
    double dist_to_target = (current_pos - target_center).norm();
    
    // ==================== Step 0.1: 优先检查是否正在接近阶段 ====================
    if (recon_approaching_) {
        if (has_trajectory_) {
            // 检查轨迹是否仍在执行
            double t_cur = ros::Time::now().toSec() - traj_start_time_;
            if (t_cur < traj_duration_ - 0.3) {
                // 轨迹仍在执行中，继续等待
                ROS_INFO_THROTTLE(1.0, "[RECON] Approaching target... dist=%.2fm, traj_progress=%.1f/%.1fs",
                                  dist_to_target, t_cur, traj_duration_);
                return;
            }
        }
        // 轨迹接近完成或无轨迹，清除标志
        recon_approaching_ = false;
        has_trajectory_ = false;
        ROS_INFO("[RECON] Approach complete, dist_to_target=%.2fm", dist_to_target);
        // 继续后续逻辑（检查Yaw或生成视点）
    }
    
    // ==================== Step 0.2: 距离判断 - 若距离过远，先规划接近轨迹 ====================
    if (dist_to_target > config_.recon_approach_threshold) {
        ROS_INFO("[RECON] Too far from target (%.2fm > %.2fm), approaching first",
                 dist_to_target, config_.recon_approach_threshold);
        
        // 计算接近点：目标方向上距离目标 approach_distance 的位置
        Eigen::Vector3d approach_dir = (target_center - current_pos).normalized();
        Eigen::Vector3d approach_pos = target_center - approach_dir * config_.recon_approach_distance;
        
        // 保持当前飞行高度，避免过大高度变化
        approach_pos.z() = std::max(approach_pos.z(), current_pos.z() - 1.0);
        approach_pos.z() = std::min(approach_pos.z(), current_pos.z() + 1.0);
        
        // 接近时 Yaw 看向目标
        double approach_yaw = atan2(approach_dir.y(), approach_dir.x());
        
        // 规划接近轨迹
        if (planTrajectory(approach_pos, approach_yaw, Eigen::Vector3d::Zero())) {
            recon_approaching_ = true;  // 标记正在接近阶段
            // planTrajectory 内部已设置 has_trajectory_ = true
            ROS_INFO("[RECON] Approach trajectory planned: target=(%.2f,%.2f,%.2f), approach_pos=(%.2f,%.2f,%.2f), duration=%.2fs",
                     target_center.x(), target_center.y(), target_center.z(),
                     approach_pos.x(), approach_pos.y(), approach_pos.z(), traj_duration_);
            // 保持在 RECON_PLAN 状态，等待接近完成后重入
            return;
        } else {
            ROS_WARN("[RECON] Approach trajectory planning failed");
            recon_approaching_ = false;
            planner_manager_->setEnableHeightKeeping(true);
            transitState(ExplorationState::LOCALPLAN, "approach_plan_failed");
            return;
        }
    }
    
    // ==================== Step 0.3: 距离足够近，检查 Yaw 是否对准目标 ====================
    Eigen::Vector3d to_target = target_center - current_pos;
    double desired_yaw = atan2(to_target.y(), to_target.x());
    double yaw_diff = fabs(odom_yaw_ - desired_yaw);
    if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;
    
    if (yaw_diff > config_.recon_yaw_threshold) {
        // Yaw 未对准，原地悬停调整
        ROS_INFO("[RECON] Adjusting yaw to face target (current=%.2f, desired=%.2f, diff=%.2f rad)",
                 odom_yaw_, desired_yaw, yaw_diff);
        
        // 发布悬停+yaw调整指令
        hover_pos_ = current_pos;
        hover_yaw_ = desired_yaw;
        publishHoverCmd();
        return;  // 等待 yaw 调整完成
    }
    
    // ==================== 原有逻辑: 距离和Yaw都满足条件，开始正式的视点生成和规划 ====================
    ROS_INFO("[RECON] Distance (%.2fm) and Yaw (diff=%.2f rad) OK, generating viewpoints...",
             dist_to_target, yaw_diff);
    
    // 2. 启动重建 (传入当前位置用于计算TSP最优切入点)
    if (!target_reconstructor_->startReconstruction(current_pos, -1)) {
        ROS_WARN("[%.3f][RECON] Start failed, pos=(%.2f,%.2f,%.2f)", 
                 t_start.toSec(), current_pos.x(), current_pos.y(), current_pos.z());
        planner_manager_->setEnableHeightKeeping(true);
        transitState(ExplorationState::LOCALPLAN, "recon_start_failed");
        return;
    }
    
    // 3. 获取完整重建路径
    std::vector<ReconViewpoint> recon_path;
    int num_viewpoints = target_reconstructor_->getReconstructionPath(recon_path);
    double viewpoint_time = (ros::Time::now() - t_start).toSec() * 1000.0;
    
    if (num_viewpoints == 0) {
        ROS_WARN("[%.3f][RECON] No viewpoints generated (%.1fms)", t_start.toSec(), viewpoint_time);
        planner_manager_->setEnableHeightKeeping(true);
        transitState(ExplorationState::LOCALPLAN, "no_recon_viewpoints");
        return;
    }
    
    // 4. 批量规划整条重建轨迹
    if (planReconstructionTrajectory(recon_path, 0)) 
    {
        current_recon_viewpoint_ = recon_path[0];
        double total_time = (ros::Time::now() - t_start).toSec() * 1000.0;
        ROS_INFO("[planReconstructionTrajectory] Started: %d vps, first=(%.2f,%.2f,%.2f), gen=%.1fms, total=%.1fms",
                 num_viewpoints, 
                 recon_path[0].position.x(), recon_path[0].position.y(), recon_path[0].position.z(),
                 viewpoint_time, total_time);
        
        // 打印所有视点的位置和yaw信息
        ROS_INFO("[RECON] ========== Reconstruction Viewpoints ==========");
        for (size_t i = 0; i < recon_path.size(); ++i) {
            const auto& vp = recon_path[i];
            ROS_INFO("[RECON] VP[%lu]: pos=(%.2f, %.2f, %.2f), yaw=%.2f deg",
                     i, vp.position.x(), vp.position.y(), vp.position.z(),
                     vp.yaw * 180.0 / M_PI);
        }
        ROS_INFO("[RECON] ================================================");
        
        recon_approaching_ = false;  // 清除接近标志
        transitState(ExplorationState::EXEC_RECON, "recon_batch_plan_success");
    } else {
        ROS_WARN("[%.3f][RECON] Batch planning failed, %d viewpoints", t_start.toSec(), num_viewpoints);
        planner_manager_->setEnableHeightKeeping(true);
        transitState(ExplorationState::LOCALPLAN, "recon_batch_plan_failed");
    }
    has_prev_traj_ = false;  // [New] 清空旧轨迹缓存
}

void ExplorationFSMNew::processExecRecon(const ros::TimerEvent& e) {
    
    // ==================== 1. 检查当前视点抵达情况 ====================
    // 使用 getCurrentReconViewpoint 获取当前应该到达的视点
    ReconViewpoint current_vp;
    bool has_current_vp = target_reconstructor_->getCurrentReconViewpoint(current_vp);
    
    if (!has_current_vp) {
        // 没有当前视点，可能已完成或出错
        ROS_INFO("[ProcessExecRecon] No current viewpoint, checking completion...");
        
        // 调用 optimizeRemainingViewpoints 检查是否触发第二阶段
        bool phase2_triggered = target_reconstructor_->optimizeRemainingViewpoints();
        
        if (phase2_triggered) {
            // 第二阶段生成了新视点，获取新路径并规划
            std::vector<ReconViewpoint> new_path;
            int num_remaining = target_reconstructor_->getReconstructionPath(new_path);
            
            if (num_remaining > 0 && planReconstructionTrajectory(new_path, 0)) {
                ROS_INFO("[ProcessExecRecon] Phase2 trajectory planned with %d viewpoints", num_remaining);
                return;  // 继续执行新轨迹
            }
        }
        
        // 真的没有视点了，重建完成
        ROS_INFO("[ProcessExecRecon] Reconstruction complete!");
        planner_manager_->setEnableHeightKeeping(true);
        recon_path_.clear();
        transitState(ExplorationState::LOCALPLAN, "recon_complete");
        return;
    }
    
    // 检查当前视点到达情况
    const double reach_threshold = 0.1;  // 10cm到达阈值
    const double yaw_threshold = 10.0 * M_PI / 180.0;  // 10°yaw阈值
    
    double dist_to_current_vp = (odom_pos_ - current_vp.position).norm();
    double yaw_diff = fabs(odom_yaw_ - current_vp.yaw);
    if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;
    
    if (dist_to_current_vp < reach_threshold && yaw_diff < yaw_threshold) {
        // 当前视点到达，标记完成
        target_reconstructor_->markCurrentViewpointComplete();
        ROS_INFO("[ProcessExecRecon] Viewpoint %d reached (dist=%.2fm, yaw_diff=%.1f°)", 
                 current_vp.id, dist_to_current_vp, yaw_diff * 180.0 / M_PI);
    }
    
    // ==================== 2. 调用视点优化（检查安全性、触发第二阶段） ====================
    bool viewpoints_changed = target_reconstructor_->optimizeRemainingViewpoints();
    
    // ==================== 3. 定时重规划检查 ====================
    // 复用 replan_check_interval 参数，定时从当前位置重规划到剩余视点
    bool need_periodic_replan = false;
    double time_since_last_replan = (ros::Time::now() - last_recon_update_time_).toSec();
    
    // 触发条件：
    // 1. 有正在执行的轨迹
    // 2. 距上次规划已超过 replan_check_interval
    // 3. 距离当前目标视点足够远（>1.0m），避免太近时规划失败
    if (has_trajectory_ && 
        time_since_last_replan >= config_.replan_check_interval &&
        dist_to_current_vp > 1.0) {
        need_periodic_replan = true;
    }
    
    // ==================== 4. 执行重规划（视点变化或定时触发） ====================
    if (viewpoints_changed || need_periodic_replan) {
        if (need_periodic_replan && !viewpoints_changed) {
            ROS_INFO("[ProcessExecRecon] Periodic replan triggered (%.1fs elapsed, dist=%.2fm)", 
                     time_since_last_replan, dist_to_current_vp);
        } else {
            ROS_INFO("[ProcessExecRecon] Viewpoints changed, replanning trajectory...");
        }
        
        // 获取剩余未完成的视点路径
        std::vector<ReconViewpoint> remaining_path;
        int num_remaining = target_reconstructor_->getReconstructionPath(remaining_path);
        
        if (num_remaining == 0) {
            // 所有视点已完成，重建结束
            ROS_INFO("[ProcessExecRecon] No remaining viewpoints, reconstruction complete!");
            planner_manager_->setEnableHeightKeeping(true);
            recon_path_.clear();
            transitState(ExplorationState::LOCALPLAN, "recon_complete");
            return;
        }
        
        // 从当前位置规划到剩余视点
        // - 从当前轨迹预测位置开始，实现平滑衔接
        // - 更新 traj_start_time_, traj_duration_, has_trajectory_
        // - 更新 last_recon_update_time_
        if (planReconstructionTrajectory(remaining_path, 0)) {
            ROS_INFO("[ProcessExecRecon] Replanned trajectory with %d remaining viewpoints", 
                     num_remaining);
        } else {
            ROS_WARN("[ProcessExecRecon] Failed to replan, continuing with current trajectory");
            // 规划失败时不退出，继续执行当前轨迹
            // 更新时间戳避免频繁重试
            last_recon_update_time_ = ros::Time::now();
        }
    }
}

bool ExplorationFSMNew::planReconstructionTrajectory(
    const std::vector<ReconViewpoint>& viewpoints, int from_idx, int window_size) 
{
    if (viewpoints.empty() || from_idx >= (int)viewpoints.size()) {
        ROS_WARN("[FSM] Invalid reconstruction path for sliding window planning");
        return false;
    }

    Eigen::Vector3d start_pos, start_vel, start_acc;
    double start_yaw;
    double t_now = ros::Time::now().toSec();

    // 1. 禁用高度保持，允许垂直方向自由移动
    planner_manager_->setEnableHeightKeeping(false);
    
    // 2. [修改] 全程规划：使用所有视点，不再使用滑动窗口
    int end_idx = (int)viewpoints.size();  // 使用全部视点
    
    // 3. 构建waypoints序列
    std::vector<Eigen::Vector3d> waypoints;
    if(has_trajectory_)
    {
        // [模式A] 预测模式：从当前轨迹的未来点开始，实现平滑衔接
        has_prev_traj_ = true;
        prev_traj_start_time_ = traj_start_time_;
        prev_traj_duration_ = traj_duration_;
        prev_traj_ = planner_manager_->traj_.local_traj.traj;

        double lookahead = 0.05;  // 50ms 前瞻
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
        } else {
            double vel_yaw = (start_vel.norm() > 0.1) ? atan2(start_vel.y(), start_vel.x()) : 0.0;

            double diff = vel_yaw - last_yaw_;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            start_yaw = last_yaw_ + diff;
        }
        // [诊断] 检查预测Yaw与里程计Yaw的偏差
        // 注意：不再修改start_yaw，因为这会导致360°跳变
        double yaw_diff = start_yaw - odom_yaw_;
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        if (std::abs(yaw_diff) > 45.0 * M_PI / 180.0) {
            ROS_DEBUG("[FSM] Yaw tracking lag in recon: %.2f deg", yaw_diff * 180.0 / M_PI);
            // 不再重置start_yaw，保持展开空间连续性
        }
        planner_manager_->setReplanStartTime(traj_start_time_ + t_predict);
    }
    else
    {
        // 添加当前位置作为起点
        start_pos = odom_pos_;
        start_vel = odom_vel_;
        start_acc = Eigen::Vector3d::Zero();
        start_yaw = last_yaw_;
        planner_manager_->setReplanStartTime(t_now);
    }
    // [FIX] 不要在waypoints中包含起点！
    // ego_planner_manager.cpp 会自动将 start_pos 加入 all_pts
    // 如果这里也加，会导致 all_pts 重复起点，造成 yaw 索引偏移45°
    // 只添加窗口内的视点位置
    for (int i = from_idx; i < end_idx; ++i) {
        waypoints.push_back(viewpoints[i].position);
    }
    
    if (waypoints.empty()) {
        ROS_WARN("[FSM] No waypoints in window");
        return false;
    }
    
    // 4. 使用多点规划接口
    // 如果窗口未到终点，终点速度非零以便衔接
    bool is_last_window = (end_idx >= (int)viewpoints.size());
    Eigen::Vector3d end_vel = is_last_window ? Eigen::Vector3d::Zero() : Eigen::Vector3d(0.0, 0.3, 0);
    Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();
    
    // 获取起始yaw和终点yaw用于轨迹优化
    double end_yaw = viewpoints[end_idx - 1].yaw;  // 窗口最后一个视点的yaw
    
    // [新增] 收集每个视点的yaw用于中间航点约束
    std::vector<double> waypoint_yaws;
    for (int i = from_idx; i < end_idx; ++i) {
        waypoint_yaws.push_back(viewpoints[i].yaw);
    }
    
    // 调用带航点yaw约束的规划接口
    bool plan_success = planner_manager_->planGlobalTrajWaypoints(
        start_pos, start_vel, start_acc, waypoints, end_vel, end_acc,
        start_yaw, end_yaw, waypoint_yaws);
    
    if (plan_success) {
        // 保存完整路径(首次规划时)
        if (recon_path_.empty() || from_idx == 0) {
            recon_path_ = viewpoints;
            recon_path_idx_ = 0;  // 只在首次规划时重置索引
        }
        recon_window_start_ = from_idx;
        // [Fix] 滑窗切换时不重置recon_path_idx_，该索引由processExecRecon的到达检测管理
        
        // 设置轨迹参数
        traj_start_time_ = ros::Time::now().toSec();
        traj_duration_ = planner_manager_->traj_.local_traj.duration;
        has_trajectory_ = true;
        last_recon_update_time_ = ros::Time::now();
        
        // ============ [新增] 生成分段yaw信息 ============
        recon_segments_.clear();
        int num_segments = (int)waypoints.size() - 1;  // waypoints包含起点+视点
        if (num_segments > 0) {
            double segment_duration = traj_duration_ / num_segments;
            
            for (int i = 0; i < num_segments; ++i) {
                ReconSegmentInfo seg;
                seg.start_time = i * segment_duration;
                seg.end_time = (i + 1) * segment_duration;
                
                // 第一段：从当前yaw过渡到第一个视点yaw
                if (i == 0) {
                    seg.start_yaw = start_yaw;
                    seg.end_yaw = viewpoints[from_idx].yaw;
                } else {
                    // 后续段：从上一个视点yaw过渡到当前视点yaw
                    seg.start_yaw = viewpoints[from_idx + i - 1].yaw;
                    seg.end_yaw = viewpoints[from_idx + i].yaw;
                }
                
                recon_segments_.push_back(seg);
            }
            ROS_INFO("[FSM] Generated %d yaw segments for smooth transition", (int)recon_segments_.size());
        }
        
        // [修复] 安全访问waypoints，避免越界
        Eigen::Vector3d first_goal = waypoints.size() > 0 ? waypoints[0] : Eigen::Vector3d::Zero();
        ROS_INFO("[FSM] Sliding window recon trajectory: window[%d-%d], %lu waypoints, %.2fs, current_pos: %f %f %f, first_goal: %f %f %f",
                 from_idx, end_idx - 1, waypoints.size(), traj_duration_, odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), first_goal.x(), first_goal.y(), first_goal.z());
        return true;
    } else {
        ROS_WARN("[FSM] Failed to plan sliding window trajectory");
        has_prev_traj_ = false;
        planner_manager_->setEnableHeightKeeping(true);
        return false;
    }
}

// [新增] 稀疏快照覆盖检测
bool ExplorationFSMNew::checkCurrentGoalCovered() {
    // [修改] 语义目标（ID < -1）使用视觉覆盖率检测
    // 如果视觉覆盖率满足要求，则认为目标已探索完成，并标记状态
    if (current_viewpoint_id_ < -1) {
         if (semantic_target_extractor_) {
             int target_id = -(current_viewpoint_id_ + 100);
             
             // [新增] 先检查是否已被标记为观测，避免重复处理
             std::vector<TargetMetadata*> unobserved_targets;
             semantic_target_extractor_->getUnobservedTargets(unobserved_targets);
             bool is_already_observed = true;
             for (const auto* t : unobserved_targets) {
                 if (t->id == target_id) {
                     is_already_observed = false;
                     break;
                 }
             }
             if (is_already_observed) {
                 // 目标已被标记为观测，直接返回 true 触发切换
                 return true;
             }
             
             // 检查视觉覆盖率
             if (semantic_target_extractor_->isTargetVisualCovered(target_id, 0.8f)) {
                 ROS_INFO("[FSM] Semantic Target %d Covered. Marking OBSERVED.", target_id);
                 semantic_target_extractor_->markTargetObserved(target_id);
                 return true;
             }
         }
         return false; // 尚未覆盖
    }

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
                    
                    Eigen::Vector3f p = anchor.cast<float>() + Eigen::Vector3f(dx, dy, dz);
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
    double active_ratio = (double)active_anchors / total_anchors;
    
    // [DEBUG] 打印覆盖率详细信息 (帮助定位为何人眼看着覆盖了但代码觉得没覆盖)
    if (active_ratio > 0.7 && !current_frontier_.empty()) {
        Eigen::Vector3f p0 = current_frontier_[0].cast<float>();
        // ROS_INFO_THROTTLE(2.0, "[Coverage DEBUG] Ratio: %.2f. Sample Pt0: (%.2f, %.2f, %.2f) is Unknown? %d. (GlobalMap check)", 
        //                   active_ratio, p0.x(), p0.y(), p0.z(), global_map_->isUnknown(p0));
    } else {
        // ROS_INFO_THROTTLE(0.5, "[Coverage] Ratio: %.2f (%d/%d), FrontierSize: %zu", 
        //                   active_ratio, active_anchors, total_anchors, current_frontier_.size());
    }

    // 如果活跃锚点比例低于 70% (即 >30% 已覆盖)，则认为目标已完成
    if (active_ratio < 0.3) {
        ROS_INFO("[Coverage] Covered! Active ratio: %.2f (%d/%d)", active_ratio, active_anchors, total_anchors);
        return true;
    }
    
    return false;
}

// ==================== LOCALPLAN 核心方法 ====================
bool ExplorationFSMNew::selectBestViewpoint(int& selected_viewpoint_id, const Eigen::Vector3d* from_pos) {
    const auto profile_start = ros::WallTime::now();
    std::vector<FrontierCluster> clusters = viewpoint_manager_->getClusters();
    // 深拷贝语义目标，防止多线程问题
    std::vector<TargetMetadata> semantic_targets_copy; 
    if (semantic_target_extractor_) {
        std::vector<TargetMetadata*> raw_targets;
        semantic_target_extractor_->getUnobservedTargets(raw_targets);
        for(auto* t : raw_targets) semantic_targets_copy.push_back(*t);
    }
    if (clusters.empty() && semantic_targets_copy.empty()) return false;

    // 获取当前机器人状态
    Eigen::Vector3d current_pos;
    double current_yaw;
    Eigen::Vector3d current_vel;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = from_pos ? *from_pos : odom_pos_;
        current_yaw = odom_yaw_;
        current_vel = odom_vel_;
    }

    // 计算航向向量
    Eigen::Vector3d current_heading = (current_vel.norm() > vp_selection_config_.velocity_threshold) ? 
                                      current_vel.normalized() : 
                                      Eigen::Vector3d(cos(current_yaw), sin(current_yaw), 0.0);
    // ================= 2. 构建候选列表 & 收集所有 Anchor =================
    struct Candidate {
        int id; // >0: Cluster, <0: Semantic (mapped)
        int type; // 0: Cluster, 1: Semantic
        Eigen::Vector3d pos;
        float gain_value;
        std::vector<int> potential_anchors;
    };
    std::vector<Candidate> all_candidates;
    // 供 Dijkstra 使用
    std::unordered_set<int> distinct_anchors_to_solve; 

    // --- Helper Lambda: 填充候选者 ---
    auto addCandidate = [&](int id, int type, const Eigen::Vector3d& pos, float gain) {
        Candidate cand;
        cand.id = id;
        cand.type = type;
        cand.pos = pos;
        cand.gain_value = gain;
        
        // 关键：找周围 15m 内所有节点，不仅仅是最近的一个
        // 建议 HistoryGraph 增加 findNodesInRange 返回 vector<int>
        std::vector<int> nearby_nodes = history_graph_->findNodesInRange(pos, 35.0);
        
        // 简单的排序，优先取距离近的 Top 3，减少计算量
        std::sort(nearby_nodes.begin(), nearby_nodes.end(), [&](int a, int b){
            return (history_graph_->getNodePos(a) - pos).squaredNorm() < (history_graph_->getNodePos(b) - pos).squaredNorm();
        });
        
        int count = 0;
        for(int node_id : nearby_nodes) {
            cand.potential_anchors.push_back(node_id);
            distinct_anchors_to_solve.insert(node_id);
            if(++count >= 3) break; // 只取最近的3个作为候选锚点
        }
        // 如果周围没有节点（比如全新区域），尝试找最近的一个
        if(cand.potential_anchors.empty()) {
            int nearest = history_graph_->findNearestNode(pos);
            if(nearest != -1) {
                cand.potential_anchors.push_back(nearest);
                distinct_anchors_to_solve.insert(nearest);
            }
        }
        all_candidates.push_back(cand);
    };

    // -> 处理 Clusters
    for (const auto& c : clusters) {
        if (!c.has_viewpoint || blacklisted_clusters_.count(c.cluster_id)) continue;
        // [优化] 直接从未 ViewpointManager 计算好的结果中读取增益
        float exp_gain = c.exploration_gain;
        float total_gain = (c.points.size() * 0.5f) + exp_gain;
        addCandidate(c.cluster_id, 0, c.viewpoint_position.cast<double>(), total_gain);
    }

    // -> 处理 Semantic Targets (ID 映射: -(id + 100))
    for (const auto& t : semantic_targets_copy) {
        if (!t.has_viewpoint) continue;
        float fixed_gain = 3000.0f; // 语义目标给予高优先级
        addCandidate(-(t.id + 100), 1, t.viewpoint_position.cast<double>(), fixed_gain);
    }

    // ================= 3. 运行全局 Dijkstra =================
    int start_node_id = history_graph_->findNearestNode(current_pos);
    cached_dijkstra_result_ = DijkstraResult(); // Clear
    
    if (start_node_id != -1 && !distinct_anchors_to_solve.empty()) {
        history_graph_->computeDijkstraToTargets(start_node_id, distinct_anchors_to_solve, cached_dijkstra_result_);
    }

    // ================= 4. 全量评估与择优 =================
    int best_id = -1;
    float best_score = -1.0f;
    int best_anchor_final = -1;
    double best_path_cost = -1.0;

    for (auto& cand : all_candidates) {
        double min_path_cost = std::numeric_limits<double>::max();
        int local_best_anchor = -1;

        // --- 多锚点路径择优 ---
        for (int anchor_id : cand.potential_anchors) {
            // 1. 检查拓扑连通性
            if (cached_dijkstra_result_.dists.find(anchor_id) == cached_dijkstra_result_.dists.end() ||
                cached_dijkstra_result_.dists[anchor_id] >= 1e9) {
                continue; 
            }
            
            double global_dist = cached_dijkstra_result_.dists[anchor_id];
            double local_dist = std::numeric_limits<double>::max();
            Eigen::Vector3d anchor_pos = history_graph_->getNodePos(anchor_id);

            // 2. 几何连接性检查 (Raycast 优先，A* 兜底)
            // 假设 global_map_ 有 isPathSafe 接口
            if (global_map_->isLineFreeInflate(anchor_pos.cast<float>(), cand.pos.cast<float>())) {
                local_dist = (anchor_pos - cand.pos).norm();
            } else {
                // 只有当射线被挡住时，才跑 A*，步数限制 2000
                auto path = history_graph_->runLocalAStar(anchor_pos, cand.pos, 200000);
                if (!path.empty()) {
                    local_dist = history_graph_->calculatePathLength(path);
                }
            }

            if (local_dist < 1e9) {
                double total = global_dist + local_dist;
                if (total < min_path_cost) {
                    min_path_cost = total;
                    local_best_anchor = anchor_id;
                }
            }
        } // End Anchor Loop

        if (min_path_cost >= 1e9) continue; // 该候选点不可达

        // --- 航向代价 ---
        Eigen::Vector3d dir_to_vp = (cand.pos - current_pos).normalized();
        double dot = current_heading.dot(dir_to_vp);
        double heading_penalty = (1.0 - dot) * vp_selection_config_.heading_penalty_weight;
        
        // --- 最终打分 ---
        float cost_metric = static_cast<float>(min_path_cost * (1.0 + heading_penalty)) + 1.0f;
        float score = cand.gain_value / cost_metric;

        // --- 迟滞处理 ---
        if (last_selected_cluster_id_ != -1 && cand.id == last_selected_cluster_id_) {
            score *= static_cast<float>(vp_selection_config_.hysteresis_gain);
        }

        if (score > best_score) {
            best_score = score;
            best_id = cand.id;
            best_anchor_final = local_best_anchor;
            best_path_cost = min_path_cost;
        }
    }

    if (best_id == -1) return false;

    // ================= 5. 状态更新与输出 =================
    // 处理切换逻辑 (简化版)
    if (best_id != last_selected_cluster_id_) {
         // ... (保留原有的时间间隔检查逻辑) ...
         last_selected_cluster_id_ = best_id;
         last_viewpoint_switch_time_ = ros::Time::now();
         last_selected_anchor_id_ = best_anchor_final;
    }

    // 填充输出数据
    if (best_id > 0) 
    { // Cluster
        for (const auto& c : clusters) 
        {
            if (c.cluster_id == best_id) {
                current_vp_pos_ = c.viewpoint_position.cast<double>();
                current_vp_yaw_ = c.viewpoint_yaw;
                current_frontier_ = c.points; //用于可视化
                current_frontier_history_id_ = best_anchor_final; // 重要：更新为实际选中的最佳锚点
                break;
            }
        }
    } 
    else 
    { // Semantic Target
        // 直接从之前深拷贝的列表中取值，无需再次查询 extractor
        int original_id = -(best_id + 100);
        for(const auto& t : semantic_targets_copy) {
            if (t.id == original_id) {
                current_vp_pos_ = t.viewpoint_position.cast<double>();
                current_vp_yaw_ = t.viewpoint_yaw;
                current_frontier_.clear(); 
                current_frontier_.push_back(t.centroid); // 伪造一个点用于显示
                current_frontier_history_id_ = best_anchor_final;
                break;
            }
        }
    }

    selected_viewpoint_id = best_id;
    PERF_LOG.logf("SCHED", "SELECT",
        "clusters=%zu semantic=%zu candidates=%zu anchors=%zu selected=%d anchor=%d "
        "score=%.6f path_cost=%.3f elapsed_ms=%.3f",
        clusters.size(), semantic_targets_copy.size(), all_candidates.size(),
        distinct_anchors_to_solve.size(), best_id, best_anchor_final, best_score,
        best_path_cost,
        (ros::WallTime::now() - profile_start).toSec() * 1000.0);
    return true;
}

bool ExplorationFSMNew::planToViewpoint(int viewpoint_id, const Eigen::Vector3d* from_pos) {
    if (current_frontier_.empty()) {
        ROS_ERROR("[FSM] PlanToViewpoint: Cluster %d not found!", viewpoint_id);
        return false;
    }
    
    // 1. 获取当前状态
    Eigen::Vector3d current_pos;
    if (from_pos) {
        current_pos = *from_pos;
    } else {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
    }

    Eigen::Vector3d vp_pos = current_vp_pos_;
    double vp_yaw = current_vp_yaw_;

    // 2. 路径生成 (Path Generation)
    std::vector<Eigen::Vector3d> global_path;
    double final_path_len = 0.0;
    
    // 尝试从缓存恢复拓扑路径 (包含: StartNode -> ... -> EndAnchor)
    // 注意: topo_nodes 不包含 CurrentPos 和 VP_Pos，只包含历史节点坐标
    std::vector<Eigen::Vector3d> topo_nodes;
    bool graph_ok = false;
    
    if (current_frontier_history_id_ != -1) {
        graph_ok = history_graph_->reconstructPathFromCache(
            cached_dijkstra_result_, 
            current_frontier_history_id_, 
            topo_nodes
        );
    }

    if (graph_ok && !topo_nodes.empty()) {
        bool skip_intermediate_node = false;

        // ================= [Optimization] 单节点回溯检查 =================
        // 场景: Current -> MidNode -> Viewpoint
        // 如果无人机当前位置去视点更顺路，就不要折返去 MidNode
        if (topo_nodes.size() == 1) {
            Eigen::Vector3d mid_node = topo_nodes[0];
            
            // --- 计算 A: 间接路径代价 (Indirect Cost) ---
            // Cost = Path(Current->Mid) + Path(Mid->VP)
            double cost_indirect = 0.0;
            
            // 1. Current -> Mid
            double dist_c_m = (mid_node - current_pos).norm();
            // 如果直线不通，跑 A* 获取真实距离
            if (!global_map_->isLineFreeInflate(current_pos.cast<float>(), mid_node.cast<float>())) {
                auto path_cm = history_graph_->runLocalAStar(current_pos, mid_node, 200000);
                if (!path_cm.empty()) dist_c_m = history_graph_->calculatePathLength(path_cm);
                else dist_c_m = 1e9; // 不可达
            }
            
            // 2. Mid -> VP (Anchor 到视点通常是近距离的，且在SelectVP阶段验证过)
            // 这里为了效率可以用直线估算，或者严谨点也跑A*
            double dist_m_v = (vp_pos - mid_node).norm(); 
            // 如果你在SelectVP里保证了可见性，这里直接用norm即可；否则建议同上做检查
            
            cost_indirect = dist_c_m + dist_m_v;

            // --- 计算 B: 直接路径代价 (Direct Cost) ---
            // Cost = Path(Current->VP)
            double cost_direct = 1e9;
            
            // 先看直线是否通 (极速)
            if (global_map_->isLineFreeInflate(current_pos.cast<float>(), vp_pos.cast<float>())) {
                cost_direct = (vp_pos - current_pos).norm();
            } else {
                // 直线不通，尝试绕过 (增加步数限制，允许一定程度的绕行)
                auto path_cv = history_graph_->runLocalAStar(current_pos, vp_pos, 200000);
                if (!path_cv.empty()) {
                    cost_direct = history_graph_->calculatePathLength(path_cv);
                }
            }

            // --- 决策 ---
            // 如果直接走代价更小，或者代价差不多(1.2倍内)但直接走更顺畅，则跳过
            // 这里设置 1.0 意味着只要直接走更短就直接走
            if (cost_direct < cost_indirect) {
                ROS_INFO("[FSM] Shortcuts: Skip node. Direct(%.2f) < Indirect(%.2f)", cost_direct, cost_indirect);
                skip_intermediate_node = true;
            }
        }
        // ===============================================================

        // 3. 组装初始路径点
        global_path.push_back(current_pos);
        
        if (!skip_intermediate_node) {
            // 如果不跳过，加入拓扑节点
            for (const auto& p : topo_nodes) {
                // 简单的去重：如果节点离上一个点极近，忽略
                if ((p - global_path.back()).norm() > 0.5) {
                    global_path.push_back(p);
                }
            }
        }
        
        // 加入终点
        if ((vp_pos - global_path.back()).norm() > 0.1) {
            global_path.push_back(vp_pos);
        } else {
            global_path.back() = vp_pos; // 修正最后一点
        }

        // 4. 通用路径裁减 (General Shortcut)
        // 这一步非常重要，它利用直线可视性(Raycast)再次简化路径
        // 即使 topo_nodes 有多个点，这一步也能把能拉直的都拉直
        global_path = history_graph_->shortcutPath(current_pos, global_path);

        // 5. 最终去重 (防止 EGO-Planner 报错)
        std::vector<Eigen::Vector3d> dedup_path;
        if (!global_path.empty()) {
            dedup_path.push_back(global_path[0]);
            for (size_t i = 1; i < global_path.size(); ++i) {
                // 阈值设为 0.2m，避免航点过于密集
                if ((global_path[i] - dedup_path.back()).norm() > 0.2) { 
                    dedup_path.push_back(global_path[i]);
                }
            }
            // 确保终点是 VP (去重可能把最后一个很近的点删了)
            if ((dedup_path.back() - vp_pos).norm() > 0.01) {
                dedup_path.push_back(vp_pos);
            } else {
                dedup_path.back() = vp_pos;
            }
            global_path = dedup_path;
        }

        // 计算总长
        final_path_len = history_graph_->calculatePathLength(global_path);

    } else {
        // --- 拓扑失效回退 (Fallback) ---
        // 如果 Dijkstra 没结果，或者 graph_ok=false，直接尝试 A*
        // ROS_WARN("[FSM] Topo path invalid. Fallback to Local A*.");
        auto fallback_path = history_graph_->runLocalAStar(current_pos, vp_pos, 200000); // 给大一点步数
        
        if (!fallback_path.empty()) {
            // A* 返回的是密集点，先简化
            global_path = history_graph_->shortcutPath(current_pos, fallback_path);
            final_path_len = history_graph_->calculatePathLength(global_path);
        } else {
            ROS_ERROR("[FSM] PlanToViewpoint Failed: Both Topo and Direct A* unreachable.");
            return false;
        }
    }

    // 6. 构建航点队列 & 执行策略
    waypoint_queue_.clear();
    current_global_path_ = global_path; // Visualization

    // 【策略 A】短途/直连模式 (Short/Simple)
    // 路径长度短，或者路径点少（只有起点终点），直接把终点给局部规划器
    if (final_path_len < 6.0 || global_path.size() <= 2) {
        ROS_INFO("[FSM] Mode: DIRECT (Len: %.1f, Pts: %zu)", final_path_len, global_path.size());
        WaypointInfo wp;
        wp.position = vp_pos;
        wp.yaw = vp_yaw;
        wp.is_viewpoint = true;
        wp.id = viewpoint_id;
        waypoint_queue_.push_back(wp);
    } 
    // 【策略 B】长途拓扑引导模式 (Graph Guided)
    else {
        ROS_INFO("[FSM] Mode: GRAPH (Len: %.1f, Pts: %zu)", final_path_len, global_path.size());
        
        // 跳过起点 i=0
        for (size_t i = 1; i < global_path.size(); ++i) {
            WaypointInfo wp;
            wp.position = global_path[i];
            
            if (i < global_path.size() - 1) {
                // 中间航点：朝向下一个点
                Eigen::Vector3d dir = global_path[i+1] - global_path[i];
                // 简单的平滑 Yaw，或者保持 current_yaw
                wp.yaw = std::atan2(dir.y(), dir.x()); 
                wp.is_viewpoint = false;
            } else {
                // 最终视点
                wp.yaw = vp_yaw;
                wp.is_viewpoint = true;
                wp.id = viewpoint_id;
            }
            waypoint_queue_.push_back(wp);
        }
    }

    // 7. 启动执行
    current_waypoint_idx_ = 0;
    if(!planNextWaypoint()) {
        ROS_ERROR("[FSM] planNextWaypoint failed at start.");
        return false;
    }
    return true;
}
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

// [重构] 无缝重规划触发器 - 简化版
bool ExplorationFSMNew::triggerSeamlessReplan() 
{
    double t_now = ros::Time::now().toSec();
    double t_cur = t_now - traj_start_time_;
    double t_predict = t_cur + 0.05;  // 50ms 前瞻 (与 planTrajectory 保持一致)
    
    // 快到终点了就不切换了
    if (t_predict > traj_duration_ - 0.5) return false;

    // 1. 计算预测位置 (用于选点)
    Eigen::Vector3d pred_pos = planner_manager_->traj_.local_traj.traj.getPos(t_predict);

    // 2. 用预测位置选点 [关键修复：选点和规划使用相同的预测起点]
    int best_id = -1;
    if (!selectBestViewpoint(best_id, &pred_pos) || best_id == -1) {
        return false;
    }
    
    // 3. 规划到新视点 (路径生成和轨迹规划都使用预测位置)
    if (planToViewpoint(best_id, &pred_pos)) {
        current_viewpoint_id_ = best_id;
        ROS_WARN("[Seamless] Replan Success -> VP %d", best_id);
        return true;
    }
    
    ROS_WARN("[Seamless] Replan Failed -> VP %d", best_id);
    return false;
}

bool ExplorationFSMNew::planTrajectory(const Eigen::Vector3d& target, double target_yaw, 
    const Eigen::Vector3d& target_vel) 
{
    Eigen::Vector3d start_pos, start_vel, start_acc;
    double start_yaw;
    double t_now = ros::Time::now().toSec();

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

        double lookahead = 0.05;  // 50ms 前瞻
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
        start_yaw = last_yaw_;
        planner_manager_->setReplanStartTime(t_now);
    }

    end_yaw_ = target_yaw; 
    planner_manager_->setDesiredHeight(start_pos.z());
    
    // 调用 Planner (传入预测的 start_yaw)
    bool success = planner_manager_->reboundReplan(
        start_pos, start_vel, start_acc, target, target_vel, 
        true, false, true, start_yaw, target_yaw); 

    if (success) {
        traj_start_time_ = planner_manager_->traj_.local_traj.start_time;
        traj_duration_ = planner_manager_->traj_.local_traj.duration;
        has_trajectory_ = true;
        last_replan_time_ = ros::Time::now();
        trajectory_start_time_ = ros::Time(traj_start_time_);
        
        // [关键修复] 同步 last_yaw_ 到新轨迹的起点值
        // 防止 calculate_yaw 在新轨迹第一帧使用旧的 last_yaw_ 导致360°跳变
        auto& local_data = planner_manager_->traj_.local_traj;
        if (local_data.use_yaw_traj && !local_data.yaw_ctrl_pts.empty()) {
            double new_start_yaw = ego_planner::YawOptimizer::evaluateYaw(
                local_data.yaw_ctrl_pts, 0.0, local_data.duration);
            last_yaw_ = new_start_yaw;
            last_yaw_dot_ = 0.0;  // 重置角速度
        } else {
            last_yaw_ = start_yaw;  // 使用传入的start_yaw
            last_yaw_dot_ = 0.0;
        }
    } 
    else {
        // 确保 executionCallback 继续执行当前轨迹，而不是尝试切换到不存在的新轨迹
        has_prev_traj_ = false;
    }

    return success;
}

// ============ [新增] 重建模式Yaw计算：视点间平滑过渡 ============
std::pair<double, double> ExplorationFSMNew::calculateReconSegmentYaw(double t_cur, double dt)
{
    // Helper lambda：角度归一化
    auto normalizeAngle = [](double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };
    
    // 找到当前所在的路段
    int seg_idx = 0;
    for (size_t i = 0; i < recon_segments_.size(); ++i) {
        if (t_cur >= recon_segments_[i].start_time && 
            t_cur <= recon_segments_[i].end_time) {
            seg_idx = i;
            break;
        }
        // 如果超出最后一段，使用最后一段
        if (i == recon_segments_.size() - 1) {
            seg_idx = i;
        }
    }
    
    const auto& seg = recon_segments_[seg_idx];
    
    // 计算段内进度 [0, 1]
    double seg_duration = seg.end_time - seg.start_time;
    double progress = (seg_duration > 0.01) ? 
        (t_cur - seg.start_time) / seg_duration : 1.0;
    progress = std::max(0.0, std::min(1.0, progress));  // clamp
    
    // 平滑插值函数: 3t² - 2t³ (S型曲线，起止速度为0)
    double smooth_progress = progress * progress * (3.0 - 2.0 * progress);
    
    // 计算目标yaw
    double yaw_diff = normalizeAngle(seg.end_yaw - seg.start_yaw);
    double target_yaw = normalizeAngle(seg.start_yaw + smooth_progress * yaw_diff);
    
    // 应用动态限制（平滑过渡，避免突变）
    double d_yaw = normalizeAngle(target_yaw - last_yaw_);
    double max_yaw_change = config_.max_yaw_rate * dt;
    if (fabs(d_yaw) > max_yaw_change) {
        d_yaw = (d_yaw > 0) ? max_yaw_change : -max_yaw_change;
    }
    
    double yaw = normalizeAngle(last_yaw_ + d_yaw);
    double yaw_dot = d_yaw / dt;
    
    last_yaw_ = yaw;
    last_yaw_dot_ = yaw_dot;
    return {yaw, yaw_dot};
}

std::pair<double, double> ExplorationFSMNew::calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
    constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
    constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    
    auto& local_data = planner_manager_->traj_.local_traj;

    // ============ [优先] 使用YawOptimizer生成的B样条轨迹 ============
    // 这包括带中间航点约束的重建yaw轨迹
    if (local_data.use_yaw_traj && !local_data.yaw_ctrl_pts.empty()) {
        double t_eval = t_cur; 
        if (t_eval < 0) t_eval = 0;
        if (t_eval > local_data.duration) t_eval = local_data.duration;

        double yaw_raw = ego_planner::YawOptimizer::evaluateYaw(
            local_data.yaw_ctrl_pts, t_eval, local_data.duration);
        double yaw_dot = ego_planner::YawOptimizer::evaluateYawDot(
            local_data.yaw_ctrl_pts, t_eval, local_data.duration);

        // ============ [新增] 轨迹切换平滑过渡 ============
        // 检测新轨迹到来（通过 traj_id 变化检测）
        int current_traj_id = local_data.traj_id;
        if (current_traj_id != yaw_switch_traj_id_) {
            // 新轨迹！计算切换时刻的偏差
            double yaw_at_zero = ego_planner::YawOptimizer::evaluateYaw(
                local_data.yaw_ctrl_pts, 0.0, local_data.duration);
            
            // 计算当前 last_yaw_ 与新轨迹起点的偏差（走最短路径）
            double diff = last_yaw_ - yaw_at_zero;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            
            yaw_offset_at_switch_ = diff;
            yaw_switch_time_ = ros::Time::now().toSec();
            yaw_switch_traj_id_ = current_traj_id;
            
            // // [DEBUG] 详细日志
            // ROS_INFO("[YawBlend] Traj switch: traj_id=%d, last_yaw=%.2f, yaw(0)=%.2f, offset=%.2f deg",
            //          current_traj_id, last_yaw_, yaw_at_zero, diff * 180.0 / M_PI);
        }
        
        // 计算混合因子：从 1.0 线性衰减到 0.0
        double time_since_switch = ros::Time::now().toSec() - yaw_switch_time_;
        double blend_factor = 0.0;
        if (time_since_switch < kYawBlendDuration && time_since_switch >= 0) {
            blend_factor = 1.0 - time_since_switch / kYawBlendDuration;
        }
        
        // 应用偏差补偿
        double yaw = yaw_raw + blend_factor * yaw_offset_at_switch_;

        last_yaw_ = yaw;
        last_yaw_dot_ = yaw_dot;
        return {yaw, yaw_dot};
    }

    // ============ [后备] 重建模式：使用分段线性插值 ============
    // 仅当YawOptimizer失败时使用
    if (state_ == ExplorationState::EXEC_RECON && !recon_segments_.empty()) {
        return calculateReconSegmentYaw(t_cur, dt);
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
    // [修复] 同时支持探索模式和重建模式的轨迹执行
    // [Fix] 增加对 RECON_PLAN 接近阶段的支持，否则接近轨迹不会被执行
    bool in_exec_state = (state_ == ExplorationState::EXEC_EXPLORATION || 
                          state_ == ExplorationState::EXEC_RECON ||
                          (state_ == ExplorationState::RECON_PLAN && recon_approaching_));
    
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
    visualizeHistoryGraph(); // [New] Visualize History Graph
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

// [New] 可视化历史节点图
void ExplorationFSMNew::visualizeHistoryGraph() {
    if (region_route_pub_.getNumSubscribers() == 0) return;
    if (!history_graph_) return;

    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    auto stamp = ros::Time::now();

    // 1. 获取数据（线程安全副本）
    auto edges = history_graph_->getAllEdges();
    auto nodes = history_graph_->getAllNodes();

    // 2. 可视化节点 (Nodes)
    visualization_msgs::Marker nodes_marker;
    nodes_marker.header.frame_id = "map";
    nodes_marker.header.stamp = stamp;
    nodes_marker.ns = "history_nodes";
    nodes_marker.id = marker_id++;
    nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes_marker.action = visualization_msgs::Marker::ADD;
    nodes_marker.scale.x = 0.4;
    nodes_marker.scale.y = 0.4;
    nodes_marker.scale.z = 0.4;
    nodes_marker.color.r = 0.0;
    nodes_marker.color.g = 1.0;
    nodes_marker.color.b = 0.0; 
    nodes_marker.color.a = 0.8;

    for (const auto& node : nodes) {
        geometry_msgs::Point p;
        p.x = node.position.x();
        p.y = node.position.y();
        p.z = node.position.z();
        nodes_marker.points.push_back(p);
    }
    markers.markers.push_back(nodes_marker);

    // 3. 可视化边 (Edges)
    visualization_msgs::Marker edges_marker;
    edges_marker.header.frame_id = "map";
    edges_marker.header.stamp = stamp;
    edges_marker.ns = "history_edges";
    edges_marker.id = marker_id++;
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::Marker::ADD;
    edges_marker.scale.x = 0.1; 
    edges_marker.color.r = 1.0;
    edges_marker.color.g = 1.0;
    edges_marker.color.b = 1.0; 
    edges_marker.color.a = 0.6;

    for (const auto& edge : edges) {
        if (edge.waypoints.size() < 2) {
             // 简单直线
             // Note: finding node position by ID is inefficient here if we iterate edges.
             // Ideally edge stores position or we map ID to pos.
             // But waypoints should be populated!
             continue;
        }
        for (size_t i = 0; i < edge.waypoints.size() - 1; ++i) {
            geometry_msgs::Point p1, p2;
            p1.x = edge.waypoints[i].x();
            p1.y = edge.waypoints[i].y();
            p1.z = edge.waypoints[i].z();
            
            p2.x = edge.waypoints[i+1].x();
            p2.y = edge.waypoints[i+1].y();
            p2.z = edge.waypoints[i+1].z();
            
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }
    }
    markers.markers.push_back(edges_marker);

    // 4. 可视化子网格中心 (SubGrid Centers)
    visualization_msgs::Marker sg_marker;
    sg_marker.header.frame_id = "map";
    sg_marker.header.stamp = stamp;
    sg_marker.ns = "subgrid_centers";
    sg_marker.id = marker_id++;
    sg_marker.type = visualization_msgs::Marker::CUBE_LIST;
    sg_marker.action = visualization_msgs::Marker::ADD;
    sg_marker.scale.x = 0.3;
    sg_marker.scale.y = 0.3;
    sg_marker.scale.z = 0.3;
    sg_marker.color.r = 1.0; // Yellow
    sg_marker.color.g = 1.0;
    sg_marker.color.b = 0.0;
    sg_marker.color.a = 0.5;

    // To visualize associated subgrids, we iterate nodes
    // Using a set to avoid duplicates if needed (though subgrids belong to one node usually)
    for (const auto& node : nodes) {
        for (int sg_id : node.associated_subgrid_ids) {
            Eigen::Vector3d center = history_graph_->getSubGridCenter(sg_id);
            if (center.norm() < 1e-3) continue; // Invalid
            
            geometry_msgs::Point p;
            p.x = center.x();
            p.y = center.y();
            p.z = center.z();
            sg_marker.points.push_back(p);
        }
    }
    markers.markers.push_back(sg_marker);
    
    // 5. 可视化节点与子网格的连接线 (Optional: Lines from Node to SubGrid)
    // To clearly show ownership
    visualization_msgs::Marker owner_line_marker;
    owner_line_marker.header.frame_id = "map";
    owner_line_marker.header.stamp = stamp;
    owner_line_marker.ns = "node_subgrid_links";
    owner_line_marker.id = marker_id++;
    owner_line_marker.type = visualization_msgs::Marker::LINE_LIST;
    owner_line_marker.action = visualization_msgs::Marker::ADD;
    owner_line_marker.scale.x = 0.1;
    owner_line_marker.color.r = 1.0;
    owner_line_marker.color.g = 0.5; // Orange
    owner_line_marker.color.b = 0.0;
    owner_line_marker.color.a = 0.8;
    
    for (const auto& node : nodes) {
        geometry_msgs::Point p_node;
        p_node.x = node.position.x();
        p_node.y = node.position.y();
        p_node.z = node.position.z();
        
        for (int sg_id : node.associated_subgrid_ids) {
            Eigen::Vector3d center = history_graph_->getSubGridCenter(sg_id);
            if (center.norm() < 1e-3) continue;
            
            geometry_msgs::Point p_sg;
            p_sg.x = center.x();
            p_sg.y = center.y();
            p_sg.z = center.z();
            
            owner_line_marker.points.push_back(p_node);
            owner_line_marker.points.push_back(p_sg);
        }
    }
    markers.markers.push_back(owner_line_marker);

    region_route_pub_.publish(markers);
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
