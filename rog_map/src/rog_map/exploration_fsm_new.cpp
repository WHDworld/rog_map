/**
 * @file exploration_fsm_new.cpp
 * @brief 探索状态机实现 - 重构版（4状态 + 集成规划器）
 */

#include "rog_map/exploration_fsm_new.h"
#include "ego_planner/yaw_optimizer.h"
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iomanip>
#include <ros/package.h>

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
    
    // 创建目标重建模块
    target_reconstructor_ = std::make_shared<TargetReconstructor>();
    target_reconstructor_->init(nh_, global_map_);  // 传入全局地图用于遮挡检测
    
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
    poly_traj_pub_ = nh.advertise<rog_map::PolyTraj>("/planning/trajectory", 10);
    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
    replan_pub_ = nh.advertise<std_msgs::Int8>("/planning/replan", 10);

    // 定时器
    fsm_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.fsm_frequency),
                                  &ExplorationFSMNew::fsmCallback, this);
    execution_timer_ = nh_.createTimer(ros::Duration(0.01), &ExplorationFSMNew::executionCallback, this);
    traj_vis_pub = nh_.advertise<visualization_msgs::Marker>("/ego_planner/trajectory_vis", 10);

    // [新增] 初始化轨迹日志文件
    if (enable_traj_logging_) {
        // 自动获取 rog_map 包路径
        std::string package_path = ros::package::getPath("rog_map");
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
    
    for (double t = 0; t < duration; t += 0.1) {
        Eigen::Vector3d pos = traj.getPos(t);
        // 检查NaN
        if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z())) continue;
        geometry_msgs::Point pt;
        pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
        marker.points.push_back(pt);
    }
    
    if (!marker.points.empty()) {
        traj_vis_pub.publish(marker);
    }
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
    
    ROS_INFO("[FSM_NEW] TRANSIT: %s -> %s (%s)", 
             old_state_str.c_str(), new_state_str.c_str(), reason.c_str());
    
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
        if (target_reconstructor_->hasReconstructionTask()) {
            ROS_INFO("[FSM] Detected reconstruction task at startup, switching to RECON_PLAN");
            transitState(ExplorationState::RECON_PLAN, "recon_task_at_startup");
            return;
        }
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
    ros::Time t_select_vp = ros::Time::now();
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
    ros::Time t_plan_to_vp = ros::Time::now();
    // 3. 锁定目标，准备起飞
    current_viewpoint_id_ = selected_vp_id;
    
    // 重置计时器
    last_collision_check_time_ = ros::Time::now();
    last_coverage_check_time_ = ros::Time::now();
    has_prev_traj_ = false;  // [New] 清空旧轨迹缓存
    
    ROS_INFO("[FSM] LOCALPLAN: processLocalPlan success, current_viewpoint_id_: %d, select_vp_time: %.2fms, plan_to_vp_time: %.2fms, plan all_time: %.2fms", 
        current_viewpoint_id_, (t_select_vp - t_start).toSec() * 1000.0, (t_plan_to_vp - t_select_vp).toSec() * 1000.0, (ros::Time::now() - t_start).toSec() * 1000.0);
    transitState(ExplorationState::EXEC_EXPLORATION, "Plan success");
}

void ExplorationFSMNew::processExecExploration() {
    // [新增] 检查重建任务 (每0.5秒)
    if ((ros::Time::now() - last_recon_check_time_).toSec() > 0.5) {
        last_recon_check_time_ = ros::Time::now();
        
        if (target_reconstructor_->hasReconstructionTask()) {
            ROS_INFO("[FSM] Detected reconstruction task, from ExecExploration switching to RECON_PLAN");
            transitState(ExplorationState::RECON_PLAN, "recon_task_detected");
            return;
        }
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
            hover_pos_ = odom_pos_;
            hover_yaw_ = last_yaw_;
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
        double t_start = ros::Time::now().toSec();
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
            ROS_INFO("[FSM] EXEC_EXPLORATION: Receding replan success, plan time: %f ms", (ros::Time::now().toSec() - t_start) * 1000.0);
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
            
            has_trajectory_ = false;
            hover_pos_ = odom_pos_;
            hover_yaw_ = last_yaw_;
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
                hover_pos_ = odom_pos_;
                hover_yaw_ = last_yaw_;
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
        publishHoverCmd();

        // [Step 3] 禁用轨迹执行器 (停止 executionCallback)
        // 合理性：必须置为 false，因为当前轨迹已超时（可能死锁或参数错误），
        // 继续运行只会执行错误的旧指令。
        has_trajectory_ = false;
        
        // Variable Reset? 不需要。因为 transitState -> LOCALPLAN 会在下一帧
        // 立即触发 processLocalPlan，那里有全套的变量重置逻辑。
        
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
    
    double dist = (odom_pos_ - current_vp.position).norm();
    double yaw_diff = fabs(odom_yaw_ - current_vp.yaw);
    if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;
    
    if (dist < reach_threshold && yaw_diff < yaw_threshold) {
        // 当前视点到达，标记完成
        target_reconstructor_->markCurrentViewpointComplete();
        ROS_INFO("[ProcessExecRecon] Viewpoint %d reached (dist=%.2fm, yaw_diff=%.1f°)", 
                 current_vp.id, dist, yaw_diff * 180.0 / M_PI);
    }
    
    // ==================== 2. 调用视点优化（检查安全性、触发第二阶段） ====================
    bool viewpoints_changed = target_reconstructor_->optimizeRemainingViewpoints();
    
    // ==================== 3. 如果视点发生变化，重新获取并规划轨迹 ====================
    if (viewpoints_changed) {
        ROS_INFO("[ProcessExecRecon] Viewpoints changed, replanning trajectory...");
        
        // 获取更新后的视点路径
        std::vector<ReconViewpoint> new_path;
        int num_remaining = target_reconstructor_->getReconstructionPath(new_path);
        
        if (num_remaining == 0) {
            // 第二阶段也没有生成新视点，重建完成
            ROS_INFO("[ProcessExecRecon] No remaining viewpoints, reconstruction complete!");
            planner_manager_->setEnableHeightKeeping(true);
            recon_path_.clear();
            transitState(ExplorationState::LOCALPLAN, "recon_complete");
            return;
        }
        
        // 规划新轨迹
        if (planReconstructionTrajectory(new_path, 0)) {
            ROS_INFO("[ProcessExecRecon] Replanned trajectory with %d viewpoints", num_remaining);
            // recon_path_ 在 planReconstructionTrajectory 内部已更新
        } else {
            ROS_WARN("[ProcessExecRecon] Failed to replan, returning to exploration");
            planner_manager_->setEnableHeightKeeping(true);
            recon_path_.clear();
            transitState(ExplorationState::LOCALPLAN, "recon_replan_failed");
            return;
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
        // [New] 安全检查：如果预测Yaw与当前里程计Yaw偏差过大，强制使用里程计Yaw
        // 防止由于之前的跟踪误差累积，导致新规划基于错误的假设开始
        double yaw_diff = start_yaw - odom_yaw_;
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        if (std::abs(yaw_diff) > 45.0 * M_PI / 180.0) { // 阈值 45度
            ROS_WARN("[FSM] Yaw divergence detected (%.2f deg). Resetting start_yaw to Odom.", 
                     yaw_diff * 180.0 / M_PI);
            // 将 odom_yaw_ 展开到与 start_yaw 最接近的等价角度
            // 这样保持了展开空间的连续性
            double odom_yaw_expanded = start_yaw - yaw_diff;
            start_yaw = odom_yaw_expanded;
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
    
    if (waypoints.size() < 2) {
        ROS_WARN("[FSM] Not enough waypoints in window");
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
        
        ROS_INFO("[FSM] Sliding window recon trajectory: window[%d-%d], %lu waypoints, %.2fs, current_pos: %f %f %f, first_goal: %f %f %f",
                 from_idx, end_idx - 1, waypoints.size(), traj_duration_, odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), waypoints[1].x(), waypoints[1].y(), waypoints[1].z());
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
    // 1. 直接从 VM 获取最新 Clusters
    std::vector<FrontierCluster> clusters = viewpoint_manager_->getClusters();
    if (clusters.empty()) return false;
    
    // 使用传入的 from_pos (预测位置) 或里程计位置
    Eigen::Vector3d current_pos;
    if (from_pos) {
        current_pos = *from_pos;
    } else {
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
    } else {
        ROS_WARN_THROTTLE(1.0, "[SelectVP] Dijkstra Skipped. StartNode: %d. Targets: %zu", start_node_id, target_anchors.size());
    }

    // 定义两阶段评估 Lambda
    auto runSelection = [&](double max_range, bool is_global_search) -> int {
        int local_best_id = -1;
        float local_best_score = -1.0f;

        for (const auto& cluster : clusters) {
            if (!cluster.has_viewpoint) continue;
            
            // Skip blacklisted clusters
            if (blacklisted_clusters_.count(cluster.cluster_id)) continue;

            // 距离初筛
            double direct_dist = (cluster.viewpoint_position.cast<double>() - current_pos).norm();
            if (direct_dist > max_range) continue;

            // [Robustness] Evaluate Multiple Anchors
            // The pre-assigned cluster.history_node_id might be blocked.
            // Search for other nearby nodes that might offer a valid path.
            double min_path_cost = std::numeric_limits<double>::max();
            int best_anchor = -1;

            // Get candidates (search radius 15.0m covers typical gaps)
            // Use cached `target_anchors` if they are in range? 
            // Better to just search spatially.
            auto candidate_anchors = history_graph_->findNodesInRange(cluster.viewpoint_position.cast<double>(), 10.0);
            
            // Ensure the original ID is included if valid
            bool has_orig = false;
            for(int id : candidate_anchors) if(id == cluster.history_node_id) has_orig = true;
            if(!has_orig && cluster.history_node_id != -1) candidate_anchors.push_back(cluster.history_node_id);

            // Evaluate all candidates
            for (int anchor_id : candidate_anchors) {
                // If anchor is not reachable from StartNode, skip
                if (cached_dijkstra_result_.dists.find(anchor_id) == cached_dijkstra_result_.dists.end() ||
                    cached_dijkstra_result_.dists[anchor_id] >= std::numeric_limits<double>::max()) {
                    continue;
                }

                double cost = history_graph_->evaluateViewpointCost(
                    current_pos, 
                    anchor_id, 
                    cluster.viewpoint_position.cast<double>(), 
                    cached_dijkstra_result_, 
                    nullptr
                );
                
                if (cost < min_path_cost) {
                    min_path_cost = cost;
                    best_anchor = anchor_id;
                }
            }

            if (min_path_cost >= std::numeric_limits<double>::max()) {
                ROS_WARN_THROTTLE(1.0, "[SelectVP] Cluster %d blocked (Tried %zu anchors). OrigNode: %d. StartNode: %d.", 
                                  cluster.cluster_id, candidate_anchors.size(), cluster.history_node_id, start_node_id);
                continue;
            }

            // [Important] Update the cluster's history node to the BEST one found
            // This ensures subsequent planning uses the correct anchor
            const_cast<FrontierCluster&>(cluster).history_node_id = best_anchor;

            double path_cost = min_path_cost;

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

        // [新增] 稀疏快照采样逻辑
    if (best_id != -1) {        
        // 找回原始 cluster 数据指针
        for (const auto& c : clusters) {
            if (c.cluster_id == best_id) {
                current_frontier_ = c.points;
                current_viewpoint_centroid_ = c.centroid.cast<double>();
                current_vp_pos_ = c.viewpoint_position.cast<double>();
                current_vp_yaw_ = c.viewpoint_yaw;
                current_frontier_history_id_ = c.history_node_id;
                break;
            }
        }
        
    }
    selected_viewpoint_id = best_id;
    return true;
}

bool ExplorationFSMNew::planToViewpoint(int viewpoint_id, const Eigen::Vector3d* from_pos) {
    if (current_frontier_.empty()) {
        ROS_ERROR("[FSM] PlanToViewpoint: Cluster %d not found!", viewpoint_id);
        return false;
    }
    
    // 使用传入的 from_pos (预测位置) 或里程计位置用于路径生成
    Eigen::Vector3d current_pos;
    if (from_pos) {
        current_pos = *from_pos;
    } else {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pos = odom_pos_;
    }

    Eigen::Vector3d vp_pos = current_vp_pos_;
    double vp_yaw = current_vp_yaw_;

    ROS_INFO("PlanToViewpoint: current_pos: (%.2f,%.2f,%.2f), best_vp_pos: (%.2f,%.2f,%.2f) best_vp_yaw: %.2f", current_pos.x(), current_pos.y(), current_pos.z(), vp_pos.x(), vp_pos.y(), vp_pos.z(), vp_yaw);
    // 2. 路径生成 (Path Generation)
    // 2. 路径生成 (Path Generation) [Optimized]
    std::vector<Eigen::Vector3d> global_path;
    double path_len = 0.0;
    
    double dist_direct = (current_pos - vp_pos).norm();
    
    // 【策略 A】近距离 (< 5.0m) -> 直接交给规划器 (Direct)
    if (dist_direct < 5.0) {
        global_path.push_back(current_pos);
        global_path.push_back(vp_pos);
        path_len = dist_direct;
        // ROS_INFO("[FSM] Plan: Close range (%.1fm). Direct.", dist_direct);
    } 
    // 【策略 B】远距离 (>= 5.0m) -> 拓扑引导 (Topological)
    else 
    {
        std::vector<Eigen::Vector3d> topo_path;
        bool graph_ok = false;
        
        if (current_frontier_history_id_ != -1) {
            graph_ok = history_graph_->reconstructPathFromCache(
                cached_dijkstra_result_, 
                current_frontier_history_id_, 
                topo_path
            );
        }

        if (graph_ok && !topo_path.empty()) {
            std::cout << "\033[1;33m[TopoPath] Backtracking Nodes:\033[0m" << std::endl;
            for (size_t k = 0; k < topo_path.size(); ++k) {
                std::cout << "  Node " << k << ": (" 
                          << topo_path[k].x() << ", " 
                          << topo_path[k].y() << ", " 
                          << topo_path[k].z() << ")" << std::endl;
            }
        }

        if (graph_ok && !topo_path.empty()) {
            // User: "Directly use intermediate history nodes as path points, no A*"
            global_path.push_back(current_pos);
            
            // 添加拓扑节点 (过滤掉离当前位置太近的起始节点)
            for (const auto& p : topo_path) {
                if ((p - global_path.back()).norm() > 1.0) {
                    global_path.push_back(p);
                }
            }
            
            // 添加最终视点
            if ((vp_pos - global_path.back()).norm() > 0.5) {
                global_path.push_back(vp_pos);
            } else {
                global_path.back() = vp_pos; // 修正终点
            }
            
            // [Optimization] 路径剪枝：尝试跳过不必要的中间历史节点
            // 如果能直接飞到下一个节点，就不要去中间的那个（解决V字形回溯问题）
            global_path = history_graph_->shortcutPath(current_pos, global_path);
            
            // [关键修复] 去除重复点
            // shortcutPath 可能返回重复点，导致规划失败
            std::vector<Eigen::Vector3d> dedup_path;
            dedup_path.push_back(global_path[0]);
            for (size_t i = 1; i < global_path.size(); ++i) {
                if ((global_path[i] - dedup_path.back()).norm() > 0.3) {  // 距离>0.3m才保留
                    dedup_path.push_back(global_path[i]);
                }
            }
            global_path = dedup_path;
            
            // [Debug] 打印过滤后的实际路径
            std::cout << "\033[1;32m[GlobalPath] After Filtering & Shortcut:\033[0m" << std::endl;
            for (size_t k = 0; k < global_path.size(); ++k) {
                double dist_to_current = (global_path[k] - current_pos).norm();
                std::cout << "  WP" << k << ": (" 
                          << global_path[k].x() << ", " 
                          << global_path[k].y() << ", " 
                          << global_path[k].z() << ") dist=" << dist_to_current << "m" << std::endl;
            }
            
            // 计算总长
            path_len = 0;
            for(size_t i=0; i<global_path.size()-1; ++i) {
                path_len += (global_path[i+1] - global_path[i]).norm();
            }
            // ROS_INFO("[FSM] Plan: Long range. Using %zu topo nodes.", topo_path.size());
        } else {
            // 拓扑失败的回退：尝试 Local A* (虽然可能也不会成功)
            auto fallback_path = history_graph_->runLocalAStar(current_pos, vp_pos);
            if (!fallback_path.empty()) {
                global_path = history_graph_->shortcutPath(current_pos, fallback_path);
                path_len = history_graph_->calculatePathLength(global_path);
                ROS_WARN("[FSM] Plan: Topo failed. Fallback to Local A*.");
            } else {
                ROS_ERROR("[FSM] Plan: Topo failed and Direct failed.");
                return false;
            }
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
            
            // [重要] 不要归一化！保持展开空间连续性
            // 如果旧轨迹在 3.7 rad 附近，新轨迹也应该从 3.7 开始
            // YawOptimizer 内部会正确处理 end_yaw 的展开
            // 最终归一化在 publishCmd 中进行
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

        // [New] 安全检查：如果预测Yaw与当前里程计Yaw偏差过大，强制使用里程计Yaw
        // 防止由于之前的跟踪误差累积，导致新规划基于错误的假设开始
        double yaw_diff = start_yaw - odom_yaw_;
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

        if (std::abs(yaw_diff) > 45.0 * M_PI / 180.0) { // 阈值 45度
            ROS_WARN("[FSM] Yaw divergence detected (%.2f deg). Resetting start_yaw to Odom.", 
                     yaw_diff * 180.0 / M_PI);
            // 将 odom_yaw_ 展开到与 start_yaw 最接近的等价角度
            // 这样保持了展开空间的连续性
            double odom_yaw_expanded = start_yaw - yaw_diff;
            start_yaw = odom_yaw_expanded;
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

        double yaw = ego_planner::YawOptimizer::evaluateYaw(
            local_data.yaw_ctrl_pts, t_eval, local_data.duration);
        double yaw_dot = ego_planner::YawOptimizer::evaluateYawDot(
            local_data.yaw_ctrl_pts, t_eval, local_data.duration);

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
    double time_forward = 0.5; 
    Eigen::Vector3d dir;
    
    // 辅助lambda：从多项式采样位置
    auto getPos = [&](double t, const rog_map::PolyTraj& traj_data) -> Eigen::Vector3d {
        return planner_manager_->traj_.local_traj.traj.getPos(t); 
    };
    
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
    
    // [调试] 每秒打印一次执行状态
    // static ros::Time last_debug_time = ros::Time(0);
    // if ((ros::Time::now() - last_debug_time).toSec() > 1.0) {
    //     last_debug_time = ros::Time::now();
    //     ROS_INFO_THROTTLE(1.0, "[ExecCallback DEBUG] state=%d, in_exec_state=%d, has_trajectory=%d, has_recon_trajectory=%d",
    //                       static_cast<int>(state_.load()), in_exec_state, has_trajectory_, has_recon_trajectory_);
    // }
    
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
    if (t_cur < 0) {
        if (has_prev_traj_) {
            double t_prev = (now.toSec() - prev_traj_start_time_);
            // [Fix] Add safety check for empty trajectory to prevent SEGFAULT
            if (prev_traj_.getPieceNum() > 0 && t_prev < prev_traj_duration_) {
                // [Fix] Execute previous trajectory instead of holding
                // Clamp time to total duration to be safe (though < prev_traj_duration_ check helps)
                double t_eval = std::min(t_prev, prev_traj_.getTotalDuration());
                pos = prev_traj_.getPos(t_eval);
                vel = prev_traj_.getVel(t_eval);
                acc = prev_traj_.getAcc(t_eval);

                // Manual Yaw Calculation for Prev Traj (Fly-and-Turn)
                // We use local logic here to avoid messing with calculate_yaw's dependency on current manager
                Eigen::Vector3d dir;
                double time_forward = 0.5;
                // Clamp lookahead time
                double t_forward_eval = std::min(t_prev + time_forward, prev_traj_.getTotalDuration());
                dir = prev_traj_.getPos(t_forward_eval) - pos;
                
                double velocity_yaw = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
                
                // Yaw Blending for Prev Traj
                double yaw_target = velocity_yaw;
                double progress = std::min(1.0, t_prev / prev_traj_duration_);
                if (progress > 0.7) {
                     // Note: We don't have the old 'end_yaw' stored easily unless we added it to member.
                     // But usually replan happens before completion. 
                     // Fallback: just follow velocity or hold last blend. 
                     // Given replan usually happens mid-flight, velocity yaw is fine.
                     yaw_target = velocity_yaw; 
                }

                // Apply Dynamic Limits
                // Helper standardizes angle
                 auto normalizeAngle = [](double angle) {
                    while (angle > M_PI) angle -= 2 * M_PI;
                    while (angle < -M_PI) angle += 2 * M_PI;
                    return angle;
                };
                auto angleDiff = [&](double target, double current) {
                    return normalizeAngle(target - current);
                };

                double d_yaw = angleDiff(yaw_target, last_yaw_);
                constexpr double YDM = 2 * M_PI; // Max Yaw Rate
                constexpr double YDDM = 5 * M_PI; // Max Yaw Acc
                double d_yaw_max;
                 if (fabs(last_yaw_dot_ + dt * YDDM) <= fabs(YDM)) {
                    d_yaw_max = last_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
                } else {
                    double t1 = (YDM - last_yaw_dot_) / YDDM;
                    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yaw_dot_) / 2.0;
                }
                if (fabs(d_yaw) > fabs(d_yaw_max)) d_yaw = d_yaw_max;
                
                double yaw_dot = d_yaw / dt;
                double yaw = normalizeAngle(last_yaw_ + d_yaw);
                
                last_yaw_ = yaw;
                last_yaw_dot_ = yaw_dot;
                last_pos_ = pos; // Important for next frame continuity
                
                publishCmd(pos, vel, acc, yaw, yaw_dot);
            } else {
                // [关键修复] Old traj finished, hold at last known position
                // 更新 last_pos_ 为当前里程计位置，避免悬停在错误位置
                last_pos_ = odom_pos_;
                publishCmd(last_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
            }
        } else {
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
        
        // [关键修复] Clamp t_cur 避免 calculate_yaw 处理超出范围的时间
        double t_clamped = std::min(t_cur, traj_duration_);
        yaw_state = calculate_yaw(t_clamped, pos, dt);
        last_pos_ = pos;
        
        publishCmd(pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), yaw_state.first, yaw_state.second);
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
