/**
 * @file gcopter_fsm.cpp
 * @brief GCOPTER 规划状态机实现
 * 
 * 完整功能:
 * 1. PX4 OFFBOARD 模式切换和解锁
 * 2. 平滑起飞到指定高度
 * 3. 等待目标点
 * 4. 轨迹规划和执行
 * 5. 碰撞检测和重规划
 * 6. 紧急停止
 */

#include "gcopter/gcopter_fsm.h"
#include "gcopter/gcopter_traj_server.h"
#include "rog_map/rog_map.h"

#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/transform_datatypes.h>

namespace rog_map {

void GCOPTERReplanFSM::init(ros::NodeHandle& nh, ROGMap* rog_map, GCOPTERTrajServer* traj_server) {
    nh_ = nh;
    rog_map_ = rog_map;
    traj_server_ = traj_server;
    exec_state_ = INIT;
    
    have_odom_ = false;
    have_target_ = false;
    have_trigger_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;
    takeoff_complete_ = false;
    initial_pose_recorded_ = false;
    takeoff_trajectory_initialized_ = false;
    
    // ==================== 加载 PX4 参数 ====================
    nh.param("fsm/real_world_flag", config_.real_world_flag, false);
    nh.param("fsm/takeoff_height", config_.takeoff_height, 1.0);
    nh.param("fsm/takeoff_threshold", config_.takeoff_threshold, 0.1);
    nh.param("fsm/pre_arm_setpoint_count", config_.pre_arm_setpoint_count, 100);
    nh.param("fsm/service_timeout", config_.service_timeout, 1.0);
    
    // 起飞参数
    nh.param("fsm/takeoff_accel_time", config_.takeoff_accel_time, 2.0);
    nh.param("fsm/takeoff_cruise_time", config_.takeoff_cruise_time, 0.5);
    nh.param("fsm/takeoff_decel_time", config_.takeoff_decel_time, 2.0);
    nh.param("fsm/takeoff_max_vel", config_.takeoff_max_vel, 0.5);
    nh.param("fsm/takeoff_max_accel", config_.takeoff_max_accel, 0.25);
    
    // ==================== 加载规划参数 ====================
    nh.param("fsm/target_type", config_.target_type, 1);
    nh.param("fsm/planning_horizon", config_.planning_horizon, 7.5);
    nh.param("fsm/replan_thresh", config_.replan_thresh, 2.0);
    nh.param("fsm/emergency_time", config_.emergency_time, 0.8);
    nh.param("fsm/enable_fail_safe", config_.enable_fail_safe, true);
    nh.param("fsm/heartbeat_timeout", config_.heartbeat_timeout, 0.5);
    nh.param("fsm/max_vel", config_.max_vel, 2.0);
    nh.param("fsm/max_acc", config_.max_acc, 2.0);
    
    // 计算不重规划阈值
    config_.no_replan_thresh = 0.5 * config_.emergency_time * config_.max_vel;
    
    // 加载预设路径点
    int waypoint_num = 0;
    nh.param("fsm/waypoint_num", waypoint_num, 0);
    for (int i = 0; i < waypoint_num; i++) {
        double x, y, z;
        nh.param("fsm/waypoint" + std::to_string(i) + "_x", x, 0.0);
        nh.param("fsm/waypoint" + std::to_string(i) + "_y", y, 0.0);
        nh.param("fsm/waypoint" + std::to_string(i) + "_z", z, 1.0);
        config_.preset_waypoints.push_back(Eigen::Vector3d(x, y, z));
    }
    
    // ==================== 初始化规划器 ====================
    planner_ = std::make_unique<GCOPTERPlanner>(nh, rog_map, "rog_map");
    
    // ==================== MAVROS 服务 ====================
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    
    // ==================== ROS 通信 ====================
    // 定时器
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &GCOPTERReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(config_.collision_check_dt), 
                                    &GCOPTERReplanFSM::checkCollisionCallback, this);
    
    // 订阅
    odom_sub_ = nh.subscribe("/ground_truth/odom", 1, &GCOPTERReplanFSM::odometryCallback, this);
    mavros_state_sub_ = nh.subscribe("/mavros/state", 10, &GCOPTERReplanFSM::mavrosStateCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &GCOPTERReplanFSM::mandatoryStopCallback, this);
    
    // 发布
    traj_pub_ = nh.advertise<visualization_msgs::Marker>("gcopter/traj_msg", 10);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("gcopter/heartbeat", 10);
    goal_vis_pub_ = nh.advertise<visualization_msgs::Marker>("gcopter/goal", 10);
    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
    
    // 根据目标类型订阅
    if (config_.target_type == MANUAL_TARGET) {
        waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, 
                                      &GCOPTERReplanFSM::waypointCallback, this);
        have_trigger_ = true;  // 手动模式不需要触发
    } else if (config_.target_type == PRESET_TARGET) {
        trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, 
                                     &GCOPTERReplanFSM::triggerCallback, this);
    }
    
    last_request_time_ = ros::Time::now();
    
    ROS_INFO("[GCOPTER FSM] Initialized.");
    ROS_INFO("[GCOPTER FSM] real_world_flag: %s, takeoff_height: %.2fm",
             config_.real_world_flag ? "true" : "false", config_.takeoff_height);
}

bool GCOPTERReplanFSM::setGoal(const Eigen::Vector3d& goal_pos, double goal_yaw) {
    if (!have_odom_) {
        ROS_WARN("[GCOPTER FSM] Cannot set goal: no odometry received");
        return false;
    }
    
    final_goal_ = goal_pos;
    final_goal_yaw_ = goal_yaw;
    have_target_ = true;
    have_new_target_ = true;
    
    // 可视化目标点
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "world";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "goal";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = goal_pos.x();
    goal_marker.pose.position.y = goal_pos.y();
    goal_marker.pose.position.z = goal_pos.z();
    goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.5;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 0.8;
    goal_vis_pub_.publish(goal_marker);
    
    ROS_INFO("[GCOPTER FSM] New goal set: (%.2f, %.2f, %.2f)", 
             goal_pos.x(), goal_pos.y(), goal_pos.z());
    
    return true;
}

bool GCOPTERReplanFSM::setWaypoints(const std::vector<Eigen::Vector3d>& waypoints) {
    if (waypoints.empty()) {
        ROS_WARN("[GCOPTER FSM] Empty waypoints");
        return false;
    }
    
    waypoints_ = waypoints;
    current_waypoint_id_ = 0;
    
    return setGoal(waypoints[0]);
}

void GCOPTERReplanFSM::forceStop() {
    ROS_WARN("[GCOPTER FSM] Force stop requested");
    mandatory_stop_ = true;
    changeFSMExecState(EMERGENCY_STOP, "FORCE_STOP");
}

std::string GCOPTERReplanFSM::getStateString() const {
    static const std::string state_str[] = {
        "INIT", "PRE_ARM", "TAKEOFF", "WAIT_TARGET", 
        "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"
    };
    return state_str[static_cast<int>(exec_state_)];
}

// ==================== 状态机核心 ====================

void GCOPTERReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
    exec_timer_.stop();  // 防止阻塞
    
    // 发布心跳
    publishHeartbeat();
    
    // 定期打印状态
    static int fsm_num = 0;
    if (++fsm_num >= 500) {
        fsm_num = 0;
        printFSMExecState();
    }
    
    switch (exec_state_) {
    case INIT: {
        if (!have_odom_) {
            break;
        }
        
        // 记录初始位姿
        if (!initial_pose_recorded_) {
            initial_pos_ = odom_pos_;
            initial_yaw_ = odom_yaw_;
            initial_pose_recorded_ = true;
            ROS_INFO("[GCOPTER FSM] Initial pose recorded: (%.2f, %.2f, %.2f), yaw=%.2f deg",
                     initial_pos_.x(), initial_pos_.y(), initial_pos_.z(),
                     initial_yaw_ * 180.0 / M_PI);
        }
        
        // 发送初始 setpoint（满足 PX4 OFFBOARD 模式要求）
        sendInitialSetpoints();
        
        changeFSMExecState(PRE_ARM, "FSM");
        break;
    }
    
    case PRE_ARM: {
        if (config_.real_world_flag) {
            // 真实环境：等待手动解锁和切换 OFFBOARD
            ROS_INFO_THROTTLE(5.0, "[GCOPTER FSM] Real world mode: waiting for manual arm and OFFBOARD...");
            
            // 持续发送 setpoint 保持 OFFBOARD 模式
            publishSetpoint();
            
            if (is_armed_ && is_offboard_) {
                ROS_INFO("[GCOPTER FSM] Armed and OFFBOARD! Starting takeoff...");
                takeoff_start_time_ = ros::Time::now();
                changeFSMExecState(TAKEOFF, "FSM");
            }
        } else {
            // 仿真环境：自动切换 OFFBOARD 和解锁
            publishSetpoint();  // 持续发送 setpoint
            armAndOffboard();
            
            if (is_armed_ && is_offboard_) {
                ROS_INFO("[GCOPTER FSM] Armed and OFFBOARD! Starting takeoff...");
                takeoff_start_time_ = ros::Time::now();
                changeFSMExecState(TAKEOFF, "FSM");
            }
        }
        break;
    }
    
    case TAKEOFF: {
        if (!is_armed_ || !is_offboard_) {
            // 失去解锁或 OFFBOARD 模式
            ROS_WARN("[GCOPTER FSM] Lost arm or OFFBOARD during takeoff!");
            changeFSMExecState(PRE_ARM, "FSM");
            break;
        }
        
        // 更新并发布起飞轨迹
        updateTakeoffTrajectory();
        publishTakeoffCmd();
        
        // 检查起飞是否完成
        if (checkTakeoffComplete()) {
            ROS_INFO("[GCOPTER FSM] Takeoff complete! Hovering at (%.2f, %.2f, %.2f)",
                     odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
            changeFSMExecState(WAIT_TARGET, "FSM");
        }
        break;
    }
    
    case WAIT_TARGET: {
        // 悬停并等待目标
        publishHoverCmd();
        
        if (!have_target_ || !have_trigger_) {
            break;
        }
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        break;
    }
    
    case GEN_NEW_TRAJ: {
        bool success = planFromGlobalTraj(10);
        if (success) {
            changeFSMExecState(EXEC_TRAJ, "FSM");
            flag_escape_emergency_ = true;
        } else {
            // 规划失败，保持悬停并重试
            publishHoverCmd();
            // 避免频繁重试
            ros::Duration(0.5).sleep();
        }
        break;
    }
    
    case REPLAN_TRAJ: {
        if (planFromLocalTraj(1)) {
            changeFSMExecState(EXEC_TRAJ, "FSM");
        } else {
            // 重规划失败，保持悬停并重试
            publishHoverCmd();
        }
        break;
    }
    
    case EXEC_TRAJ: {
        LocalTrajData* info = &traj_.local_traj;
        double t_cur = ros::Time::now().toSec() - info->start_time;
        t_cur = std::min(info->duration, t_cur);
        
        Eigen::Vector3d pos = info->traj.getPos(t_cur);
        bool touch_the_goal = (local_target_pt_ - final_goal_).norm() < 0.1;
        
        // 检查是否接近轨迹终点
        bool close_to_traj_end = (info->duration - t_cur) < config_.emergency_time;
        
        // 情况1: 目标点在障碍物中
        if (modifyInCollisionGoal()) {
            // 目标已修改，保持执行
        }
        // 情况2: 多路径点模式，到达当前路径点
        else if (config_.target_type == PRESET_TARGET &&
                 current_waypoint_id_ < static_cast<int>(waypoints_.size()) - 1 &&
                 (final_goal_ - pos).norm() < config_.no_replan_thresh) {
            current_waypoint_id_++;
            planNextWaypoint(waypoints_[current_waypoint_id_]);
        }
        // 情况3: 到达最终目标
        else if ((t_cur > info->duration - 1e-2) && touch_the_goal) {
            have_target_ = false;
            have_trigger_ = (config_.target_type == MANUAL_TARGET);  // 手动模式不需要重新触发
            
            ROS_INFO("[GCOPTER FSM] Goal reached! Hovering...");
            
            // 预设模式准备下一轮
            if (config_.target_type == PRESET_TARGET && !waypoints_.empty()) {
                current_waypoint_id_ = 0;
                planNextWaypoint(waypoints_[current_waypoint_id_]);
            }
            
            changeFSMExecState(WAIT_TARGET, "FSM");
        }
        // 情况4: 需要重规划
        else if (t_cur > config_.replan_thresh || (!touch_the_goal && close_to_traj_end)) {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        
        break;
    }
    
    case EMERGENCY_STOP: {
        // 紧急停止：立即悬停
        publishHoverCmd();
        
        if (flag_escape_emergency_) {
            callEmergencyStop(odom_pos_);
        } else {
            // 恢复检查
            if (config_.enable_fail_safe && odom_vel_.norm() < 0.1) {
                ROS_INFO("[GCOPTER FSM] Recovered from emergency, returning to WAIT_TARGET");
                mandatory_stop_ = false;
                changeFSMExecState(WAIT_TARGET, "FSM");
            }
        }
        flag_escape_emergency_ = false;
        break;
    }
    }
    
    exec_timer_.start();
}

void GCOPTERReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
    LocalTrajData* info = &traj_.local_traj;
    
    if (exec_state_ != EXEC_TRAJ || info->traj_id <= 0) {
        return;
    }
    
    double t_cur = ros::Time::now().toSec() - info->start_time;
    
    // 检查轨迹碰撞
    const auto& pts_chk = info->pts_chk;
    if (pts_chk.empty()) return;
    
    // 找到当前时间后的检查点
    bool touch_the_end = (local_target_pt_ - final_goal_).norm() < 0.1;
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    
    for (size_t i = 0; i < i_end; ++i) {
        for (size_t j = 0; j < pts_chk[i].size(); ++j) {
            double t = pts_chk[i][j].first;
            if (t <= t_cur) continue;
            
            const Eigen::Vector3d& p = pts_chk[i][j].second;
            
            // 碰撞检查
            bool dangerous = false;
            if (rog_map_) {
                Vec3f pos_f = p;
                dangerous = rog_map_->isOccupiedInflate(pos_f);
            }
            
            if (dangerous) {
                ROS_WARN("[GCOPTER FSM] Collision detected at t=%.2f, trying to replan", t);
                
                // 尝试重规划
                if (planFromLocalTraj()) {
                    ROS_INFO("[GCOPTER FSM] Replan success after collision detection");
                    changeFSMExecState(EXEC_TRAJ, "SAFETY");
                    return;
                } else {
                    // 判断是否紧急
                    if (t - t_cur < config_.emergency_time) {
                        ROS_ERROR("[GCOPTER FSM] EMERGENCY STOP! Collision in %.2fs", t - t_cur);
                        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
                    } else {
                        ROS_WARN("[GCOPTER FSM] Collision ahead, replanning...");
                        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
                    }
                    return;
                }
            }
        }
    }
}

void GCOPTERReplanFSM::changeFSMExecState(FSMState new_state, const std::string& pos_call) {
    if (new_state == exec_state_) {
        consecutive_state_calls_++;
        return;  // 相同状态不打印
    }
    
    consecutive_state_calls_ = 1;
    
    static const std::string state_str[] = {
        "INIT", "PRE_ARM", "TAKEOFF", "WAIT_TARGET",
        "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"
    };
    
    FSMState prev_state = exec_state_;
    exec_state_ = new_state;
    
    ROS_INFO("[%s] State: %s -> %s", 
             pos_call.c_str(), 
             state_str[static_cast<int>(prev_state)].c_str(),
             state_str[static_cast<int>(new_state)].c_str());
}

void GCOPTERReplanFSM::printFSMExecState() {
    static const std::string state_str[] = {
        "INIT", "PRE_ARM", "TAKEOFF", "WAIT_TARGET",
        "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"
    };
    
    std::string status = "[GCOPTER FSM] State: " + state_str[static_cast<int>(exec_state_)];
    
    if (!have_odom_) status += " | Waiting: odom";
    if (!is_connected_) status += " | Waiting: MAVROS";
    if (!is_armed_) status += " | Not armed";
    if (!is_offboard_) status += " | Not OFFBOARD";
    if (!have_target_) status += " | Waiting: target";
    if (!have_trigger_) status += " | Waiting: trigger";
    
    ROS_INFO_STREAM(status);
}

// ==================== PX4 控制 ====================

void GCOPTERReplanFSM::sendInitialSetpoints() {
    ROS_INFO("[GCOPTER FSM] Sending initial setpoints for OFFBOARD mode...");
    
    for (int i = 0; i < config_.pre_arm_setpoint_count; ++i) {
        geometry_msgs::PoseStamped setpoint;
        setpoint.header.stamp = ros::Time::now();
        setpoint.header.frame_id = "world";
        setpoint.pose.position.x = odom_pos_.x();
        setpoint.pose.position.y = odom_pos_.y();
        setpoint.pose.position.z = odom_pos_.z();
        
        tf::Quaternion q;
        q.setRPY(0, 0, odom_yaw_);
        setpoint.pose.orientation.x = q.x();
        setpoint.pose.orientation.y = q.y();
        setpoint.pose.orientation.z = q.z();
        setpoint.pose.orientation.w = q.w();
        
        setpoint_pub_.publish(setpoint);
        ros::Duration(0.01).sleep();  // 100Hz
    }
    
    ROS_INFO("[GCOPTER FSM] Initial setpoints sent.");
}

void GCOPTERReplanFSM::armAndOffboard() {
    ros::Time now = ros::Time::now();
    
    // 限制请求频率
    if ((now - last_request_time_).toSec() < config_.service_timeout) {
        return;
    }
    
    // 先切换到 OFFBOARD 模式
    if (!is_offboard_) {
        mavros_msgs::SetMode offboard_set_mode;
        offboard_set_mode.request.custom_mode = "OFFBOARD";
        
        if (set_mode_client_.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
            ROS_INFO("[GCOPTER FSM] OFFBOARD mode request sent");
        }
        last_request_time_ = now;
    }
    // 然后解锁
    else if (!is_armed_) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("[GCOPTER FSM] Arming request sent");
        }
        last_request_time_ = now;
    }
}

bool GCOPTERReplanFSM::checkTakeoffComplete() {
    if (!initial_pose_recorded_) return false;
    
    // 检查高度是否到达目标
    double dz = std::abs(odom_pos_.z() - config_.takeoff_height);
    
    // 检查 XY 位置偏移是否过大
    double dx = std::abs(odom_pos_.x() - initial_pos_.x());
    double dy = std::abs(odom_pos_.y() - initial_pos_.y());
    
    // 检查速度是否接近零
    bool vel_small = odom_vel_.norm() < 0.2;
    
    return (dz < config_.takeoff_threshold) && 
           (dx < config_.takeoff_threshold) && 
           (dy < config_.takeoff_threshold) &&
           vel_small;
}

void GCOPTERReplanFSM::updateTakeoffTrajectory() {
    if (!takeoff_trajectory_initialized_) {
        trajectory_z_ = initial_pos_.z();
        trajectory_vel_z_ = 0.0;
        trajectory_acc_z_ = 0.0;
        takeoff_trajectory_initialized_ = true;
        takeoff_start_time_ = ros::Time::now();
        ROS_INFO("[GCOPTER FSM] Takeoff trajectory initialized, target height: %.2fm", 
                 config_.takeoff_height);
        return;
    }
    
    double t = (ros::Time::now() - takeoff_start_time_).toSec();
    double total_t = config_.takeoff_accel_time + config_.takeoff_cruise_time + config_.takeoff_decel_time;
    double accel_t = config_.takeoff_accel_time;
    double cruise_t = config_.takeoff_cruise_time;
    
    double initial_z = initial_pos_.z();
    double target_z = config_.takeoff_height;
    
    if (t <= 0) {
        trajectory_z_ = initial_z;
        trajectory_vel_z_ = 0.0;
        trajectory_acc_z_ = 0.0;
    }
    // 阶段1：加速上升
    else if (t <= accel_t) {
        trajectory_acc_z_ = config_.takeoff_max_accel;
        trajectory_vel_z_ = std::min(trajectory_acc_z_ * t, config_.takeoff_max_vel);
        trajectory_z_ = initial_z + 0.5 * trajectory_acc_z_ * t * t;
    }
    // 阶段2：匀速巡航
    else if (t <= accel_t + cruise_t) {
        trajectory_acc_z_ = 0.0;
        trajectory_vel_z_ = config_.takeoff_max_vel;
        double z_accel_end = initial_z + 0.5 * config_.takeoff_max_accel * accel_t * accel_t;
        double t_cruise = t - accel_t;
        trajectory_z_ = z_accel_end + trajectory_vel_z_ * t_cruise;
    }
    // 阶段3：减速上升
    else if (t <= total_t) {
        trajectory_acc_z_ = -config_.takeoff_max_accel;
        double t_decel = t - (accel_t + cruise_t);
        trajectory_vel_z_ = std::max(config_.takeoff_max_vel + trajectory_acc_z_ * t_decel, 0.0);
        double z_cruise_end = initial_z + 0.5 * config_.takeoff_max_accel * accel_t * accel_t 
                             + config_.takeoff_max_vel * cruise_t;
        trajectory_z_ = z_cruise_end + config_.takeoff_max_vel * t_decel 
                       + 0.5 * trajectory_acc_z_ * t_decel * t_decel;
    }
    // 阶段4：到达目标高度
    else {
        trajectory_z_ = target_z;
        trajectory_vel_z_ = 0.0;
        trajectory_acc_z_ = 0.0;
    }
    
    // 约束不超过目标高度
    trajectory_z_ = std::min(trajectory_z_, target_z);
}

void GCOPTERReplanFSM::publishTakeoffCmd() {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    
    // 保持初始 XY 位置，只改变 Z
    cmd.position.x = initial_pos_.x();
    cmd.position.y = initial_pos_.y();
    cmd.position.z = trajectory_z_;
    
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = trajectory_vel_z_;
    
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = trajectory_acc_z_;
    
    cmd.yaw = initial_yaw_;
    cmd.yaw_dot = 0.0;
    
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    
    pos_cmd_pub_.publish(cmd);
}

void GCOPTERReplanFSM::publishHoverCmd() {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    
    // 悬停在当前位置
    cmd.position.x = odom_pos_.x();
    cmd.position.y = odom_pos_.y();
    cmd.position.z = odom_pos_.z();
    
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;
    
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    
    cmd.yaw = odom_yaw_;
    cmd.yaw_dot = 0.0;
    
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    
    pos_cmd_pub_.publish(cmd);
}

void GCOPTERReplanFSM::publishSetpoint() {
    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "world";
    
    // 使用当前位置作为 setpoint
    setpoint.pose.position.x = odom_pos_.x();
    setpoint.pose.position.y = odom_pos_.y();
    setpoint.pose.position.z = odom_pos_.z();
    
    tf::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    setpoint.pose.orientation.x = q.x();
    setpoint.pose.orientation.y = q.y();
    setpoint.pose.orientation.z = q.z();
    setpoint.pose.orientation.w = q.w();
    
    setpoint_pub_.publish(setpoint);
}

// ==================== 规划接口 ====================

bool GCOPTERReplanFSM::planFromGlobalTraj(int trial_times) {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_ = Eigen::Vector3d::Zero();
    
    // 获取局部目标
    getLocalTarget(start_pt_, final_goal_, local_target_pt_, local_target_vel_, touch_goal_);
    
    for (int i = 0; i < trial_times; i++) {
        auto result = planner_->plan(start_pt_, start_vel_, local_target_pt_, local_target_vel_);
        
        if (result.success) {
            // 存储轨迹
            traj_.local_traj.traj_id = ++traj_id_counter_;
            traj_.local_traj.start_time = ros::Time::now().toSec();
            traj_.local_traj.duration = result.total_time;
            traj_.local_traj.traj = result.trajectory;
            traj_.local_traj.rrt_path = result.rrt_path;
            
            // 生成碰撞检测点
            generateCheckPoints(result.trajectory, traj_.local_traj.pts_chk);
            
            // 发布轨迹
            publishTrajectory();
            
            // 设置到轨迹服务器
            if (traj_server_) {
                traj_server_->setTrajectory(result.trajectory, traj_id_counter_);
                traj_server_->setGoalPosition(final_goal_);
            }
            
            ROS_INFO("[GCOPTER FSM] Plan success! Duration: %.2fs, Pieces: %d", 
                     result.total_time, result.trajectory.getPieceNum());
            
            have_new_target_ = false;
            return true;
        }
        else
        {
            ROS_WARN("[GCOPTER FSM] planFromGlobalTraj failed because of %s", result.error_msg.c_str());
        }
    }
    
    ROS_WARN("[GCOPTER FSM] planFromGlobalTraj failed after %d trials", trial_times);
    return false;
}

bool GCOPTERReplanFSM::planFromLocalTraj(int trial_times) {
    LocalTrajData* info = &traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    t_cur = std::max(0.0, std::min(info->duration, t_cur));
    
    // 从当前轨迹位置开始
    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);
    
    // 获取局部目标
    getLocalTarget(start_pt_, final_goal_, local_target_pt_, local_target_vel_, touch_goal_);
    
    for (int i = 0; i < trial_times; i++) {
        auto result = planner_->plan(start_pt_, start_vel_, local_target_pt_, local_target_vel_);
        
        if (result.success) {
            traj_.local_traj.traj_id = ++traj_id_counter_;
            traj_.local_traj.start_time = ros::Time::now().toSec();
            traj_.local_traj.duration = result.total_time;
            traj_.local_traj.traj = result.trajectory;
            traj_.local_traj.rrt_path = result.rrt_path;
            
            generateCheckPoints(result.trajectory, traj_.local_traj.pts_chk);
            publishTrajectory();
            
            if (traj_server_) {
                traj_server_->setTrajectory(result.trajectory, traj_id_counter_);
            }
            
            return true;
        }
    }
    
    return false;
}

bool GCOPTERReplanFSM::planNextWaypoint(const Eigen::Vector3d& next_wp) {
    final_goal_ = next_wp;
    have_target_ = true;
    have_new_target_ = true;
    
    if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
        changeFSMExecState(REPLAN_TRAJ, "NEW_WP");
    }
    
    return true;
}

bool GCOPTERReplanFSM::callEmergencyStop(const Eigen::Vector3d& stop_pos) {
    ROS_ERROR("[GCOPTER FSM] Emergency stop at (%.2f, %.2f, %.2f)", 
              stop_pos.x(), stop_pos.y(), stop_pos.z());
    
    // 清空轨迹，TrajServer 会自动悬停
    if (traj_server_) {
        traj_server_->emergencyStop();
    }
    
    // 更新轨迹
    traj_.local_traj.traj_id = ++traj_id_counter_;
    traj_.local_traj.start_time = ros::Time::now().toSec();
    traj_.local_traj.duration = 0.0;
    traj_.local_traj.pts_chk.clear();
    
    return true;
}

void GCOPTERReplanFSM::getLocalTarget(const Eigen::Vector3d& start_pt,
                                       const Eigen::Vector3d& global_end_pt,
                                       Eigen::Vector3d& local_target_pos,
                                       Eigen::Vector3d& local_target_vel,
                                       bool& touch_goal) {
    double dist_to_goal = (global_end_pt - start_pt).norm();
    
    if (dist_to_goal <= config_.planning_horizon) {
        // 目标在规划视野内
        local_target_pos = global_end_pt;
        local_target_vel = Eigen::Vector3d::Zero();
        touch_goal = true;
    } else {
        // 目标超出规划视野
        Eigen::Vector3d dir = (global_end_pt - start_pt).normalized();
        local_target_pos = start_pt + dir * config_.planning_horizon;
        local_target_vel = dir * config_.max_vel * 0.5;
        touch_goal = false;
    }
    
    // 如果接近目标，减速
    if (dist_to_goal < (config_.max_vel * config_.max_vel) / (2 * config_.max_acc)) {
        local_target_vel = Eigen::Vector3d::Zero();
    }
}

bool GCOPTERReplanFSM::modifyInCollisionGoal() {
    if (!rog_map_) return false;
    
    Vec3f goal_f = final_goal_;
    if (!rog_map_->isOccupiedInflate(goal_f)) {
        return false;
    }
    
    ROS_WARN("[GCOPTER FSM] Goal in collision, searching for safe goal...");
    
    Eigen::Vector3d dir = (final_goal_ - odom_pos_).normalized();
    double step = 0.2;
    double max_dist = (final_goal_ - odom_pos_).norm();
    
    for (double d = max_dist - step; d > 0; d -= step) {
        Eigen::Vector3d test_pt = odom_pos_ + dir * d;
        Vec3f test_f = test_pt;
        
        if (!rog_map_->isOccupiedInflate(test_f)) {
            ROS_INFO("[GCOPTER FSM] Found safe goal at (%.2f, %.2f, %.2f)", 
                     test_pt.x(), test_pt.y(), test_pt.z());
            final_goal_ = test_pt;
            have_new_target_ = true;
            return true;
        }
    }
    
    ROS_ERROR("[GCOPTER FSM] Cannot find safe goal!");
    return false;
}

void GCOPTERReplanFSM::generateCheckPoints(const Trajectory<5>& traj,
    std::vector<std::vector<std::pair<double, Eigen::Vector3d>>>& pts_chk) {
    pts_chk.clear();
    
    int piece_num = traj.getPieceNum();
    if (piece_num == 0) return;
    
    double total_duration = traj.getTotalDuration();
    double dt = 0.05;
    
    Eigen::VectorXd durations = traj.getDurations();
    
    double piece_start = 0.0;
    for (int i = 0; i < piece_num; ++i) {
        std::vector<std::pair<double, Eigen::Vector3d>> piece_pts;
        
        double piece_duration = durations(i);
        
        for (double t = 0; t < piece_duration; t += dt) {
            double global_t = piece_start + t;
            if (global_t > total_duration) break;
            
            Eigen::Vector3d pos = traj.getPos(global_t);
            piece_pts.emplace_back(global_t, pos);
        }
        
        pts_chk.push_back(piece_pts);
        piece_start += piece_duration;
    }
}

// ==================== ROS 回调 ====================

void GCOPTERReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom_pos_.x() = msg->pose.pose.position.x;
    odom_pos_.y() = msg->pose.pose.position.y;
    odom_pos_.z() = msg->pose.pose.position.z;
    
    odom_vel_.x() = msg->twist.twist.linear.x;
    odom_vel_.y() = msg->twist.twist.linear.y;
    odom_vel_.z() = msg->twist.twist.linear.z;
    
    // 提取 yaw
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odom_yaw_ = yaw;
    
    have_odom_ = true;
}

void GCOPTERReplanFSM::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
    is_connected_ = msg->connected;
    is_armed_ = msg->armed;
    is_offboard_ = (msg->mode == "OFFBOARD");
}

void GCOPTERReplanFSM::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (msg->pose.position.z < -0.1) return;
    
    // 只有在起飞完成后才接受目标
    if (exec_state_ != WAIT_TARGET && exec_state_ != EXEC_TRAJ && exec_state_ != REPLAN_TRAJ) {
        ROS_WARN("[GCOPTER FSM] Cannot accept goal in current state: %s", getStateString().c_str());
        return;
    }
    
    ROS_INFO("[GCOPTER FSM] Received goal from RViz: (%.2f, %.2f, %.2f)", 
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 
                         std::max(msg->pose.position.z, 0.5));  // 最低0.5米
    
    setGoal(goal);
}

void GCOPTERReplanFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    have_trigger_ = true;
    ROS_INFO("[GCOPTER FSM] Triggered!");
    
    if (config_.target_type == PRESET_TARGET && !config_.preset_waypoints.empty()) {
        waypoints_ = config_.preset_waypoints;
        current_waypoint_id_ = 0;
        setGoal(waypoints_[0]);
    }
}

void GCOPTERReplanFSM::mandatoryStopCallback(const std_msgs::Empty& msg) {
    ROS_ERROR("[GCOPTER FSM] Mandatory stop received!");
    forceStop();
}

// ==================== 轨迹发布 ====================

void GCOPTERReplanFSM::publishTrajectory() {
    if (traj_.local_traj.traj.getPieceNum() == 0) return;
    
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "world";
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.ns = "trajectory";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.scale.x = 0.05;
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 1.0;
    traj_marker.color.b = 0.0;
    traj_marker.color.a = 1.0;
    
    double dt = 0.05;
    double duration = traj_.local_traj.duration;
    for (double t = 0; t < duration; t += dt) {
        Eigen::Vector3d pos = traj_.local_traj.traj.getPos(t);
        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = pos.z();
        traj_marker.points.push_back(p);
    }
    
    traj_pub_.publish(traj_marker);
}

void GCOPTERReplanFSM::publishHeartbeat() {
    std_msgs::Empty heartbeat;
    heartbeat_pub_.publish(heartbeat);
    heartbeat_time_ = ros::Time::now();
}

} // namespace rog_map
