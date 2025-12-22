/**
 * @file gcopter_traj_server.cpp
 * @brief GCOPTER 轨迹服务器实现
 * 
 * 核心功能:
 * 1. 以固定频率（默认100Hz）采样轨迹
 * 2. 计算期望位置、速度、加速度、jerk
 * 3. 计算平滑的偏航角控制
 * 4. 发布 quadrotor_msgs::PositionCommand
 */

#include "gcopter/gcopter_traj_server.h"

namespace rog_map {

void GCOPTERTrajServer::init(ros::NodeHandle& nh) {
    nh_ = nh;
    
    // ==================== 加载参数 ====================
    nh.param("traj_server/cmd_rate", config_.cmd_rate, 100.0);
    nh.param("traj_server/time_forward", config_.time_forward, 1.0);
    nh.param("traj_server/yaw_dot_max", config_.yaw_dot_max, 2 * M_PI);
    nh.param("traj_server/yaw_dot_dot_max", config_.yaw_dot_dot_max, 5 * M_PI);
    nh.param("traj_server/heartbeat_timeout", config_.heartbeat_timeout, 0.5);
    nh.param("traj_server/enable_heartbeat_check", config_.enable_heartbeat_check, true);
    nh.param("traj_server/flip_yaw_at_end", config_.flip_yaw_at_end, false);
    nh.param("traj_server/hover_at_end", config_.hover_at_end, true);
    nh.param("traj_server/max_vel", config_.max_vel, 2.0);
    nh.param("traj_server/max_acc", config_.max_acc, 2.0);
    
    int yaw_mode_int;
    nh.param("traj_server/yaw_mode", yaw_mode_int, 0);
    config_.yaw_mode = static_cast<YawMode>(yaw_mode_int);
    
    // ==================== ROS 通信 ====================
    // 发布器 - 使用用户指定的话题 /planning/pos_cmd
    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
    traj_vis_pub_ = nh.advertise<visualization_msgs::Marker>("traj_server/trajectory", 10);
    
    // 订阅器
    heartbeat_sub_ = nh.subscribe("gcopter/heartbeat", 10, 
                                   &GCOPTERTrajServer::heartbeatCallback, this);
    odom_sub_ = nh.subscribe("odom", 1, 
                              &GCOPTERTrajServer::odomCallback, this);
    emergency_stop_sub_ = nh.subscribe("emergency_stop", 1, 
                                        &GCOPTERTrajServer::emergencyStopCallback, this);
    
    // 命令发布定时器
    double dt = 1.0 / config_.cmd_rate;
    cmd_timer_ = nh.createTimer(ros::Duration(dt), &GCOPTERTrajServer::cmdTimerCallback, this);
    
    // 初始化状态
    status_ = IDLE;
    last_heartbeat_time_ = ros::Time::now();
    last_cmd_time_ = ros::Time::now();
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;
    last_pos_ = Eigen::Vector3d::Zero();
    
    ROS_INFO("[TrajServer] Initialized. Rate: %.1fHz, Publishing to /planning/pos_cmd", 
             config_.cmd_rate);
}

void GCOPTERTrajServer::setTrajectory(const Trajectory<5>& traj, int traj_id) {
    if (traj.getPieceNum() == 0) {
        ROS_WARN("[TrajServer] Received empty trajectory");
        return;
    }
    
    traj_data_.traj = traj;
    traj_data_.traj_id = traj_id;
    traj_data_.start_time = ros::Time::now().toSec();
    traj_data_.duration = traj.getTotalDuration();
    traj_data_.valid = true;
    
    status_ = EXECUTING;
    traj_finished_ = false;
    
    ROS_INFO("[TrajServer] New trajectory set. ID: %d, Duration: %.2fs, Pieces: %d",
             traj_id, traj_data_.duration, traj.getPieceNum());
    
    // 可视化轨迹
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "world";
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.ns = "traj_server";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.scale.x = 0.03;
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 0.8;
    traj_marker.color.b = 1.0;
    traj_marker.color.a = 1.0;
    
    double dt = 0.05;
    for (double t = 0; t < traj_data_.duration; t += dt) {
        Eigen::Vector3d pos = traj_data_.traj.getPos(t);
        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = pos.z();
        traj_marker.points.push_back(p);
    }
    
    traj_vis_pub_.publish(traj_marker);
}

void GCOPTERTrajServer::emergencyStop() {
    ROS_ERROR("[TrajServer] Emergency stop!");
    status_ = EMERGENCY;
    traj_data_.valid = false;
    
    // 立即发布悬停命令
    publishHoverCmd();
}

void GCOPTERTrajServer::resume() {
    if (status_ == EMERGENCY) {
        ROS_INFO("[TrajServer] Resuming from emergency stop");
        status_ = IDLE;
    }
}

// ==================== 核心功能 ====================

void GCOPTERTrajServer::cmdTimerCallback(const ros::TimerEvent& e) {
    ros::Time time_now = ros::Time::now();
    double dt = (time_now - last_cmd_time_).toSec();
    last_cmd_time_ = time_now;
    
    // 心跳检查
    if (config_.enable_heartbeat_check) {
        if ((time_now - last_heartbeat_time_).toSec() > config_.heartbeat_timeout) {
            if (!heartbeat_lost_) {
                ROS_ERROR("[TrajServer] Lost heartbeat from planner!");
                heartbeat_lost_ = true;
            }
            
            // 心跳丢失，发布悬停命令
            publishHoverCmd();
            return;
        } else {
            heartbeat_lost_ = false;
        }
    }
    
    // 状态处理
    if (status_ == IDLE || status_ == EMERGENCY) {
        // 空闲或紧急状态，发布悬停命令
        publishHoverCmd();
        return;
    }
    
    if (!traj_data_.valid) {
        publishHoverCmd();
        return;
    }
    
    // 计算当前轨迹时间
    double t_cur = time_now.toSec() - traj_data_.start_time;
    
    // ==================== 轨迹执行中 ====================
    if (t_cur >= 0.0 && t_cur < traj_data_.duration) {
        // 获取期望状态
        Eigen::Vector3d pos = traj_data_.traj.getPos(t_cur);
        Eigen::Vector3d vel = traj_data_.traj.getVel(t_cur);
        Eigen::Vector3d acc = traj_data_.traj.getAcc(t_cur);
        Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
        
        // 尝试获取 jerk（如果轨迹支持）
        if (traj_data_.traj.getPieceNum() > 0) {
            // MINCO 轨迹通常支持 jerk 计算
            // jerk = traj_data_.traj.getJer(t_cur);  // 如果接口存在
        }
        
        // 计算偏航角
        auto yaw_yawdot = calculateYaw(t_cur, pos, dt);
        
        // 发布命令
        publishCmd(pos, vel, acc, jerk, yaw_yawdot.first, yaw_yawdot.second);
        
        // 更新状态
        desired_pos_ = pos;
        desired_vel_ = vel;
        last_pos_ = pos;
        
        // 更新偏航翻转目标
        slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
        slowly_flip_yaw_target_ = normalizeAngle(slowly_flip_yaw_target_);
        
        traj_finished_ = false;
    }
    // ==================== 轨迹结束 ====================
    else if (t_cur >= traj_data_.duration) {
        Eigen::Vector3d pos = traj_data_.traj.getPos(traj_data_.duration);
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
        
        // 轨迹结束后的偏航控制
        double yaw = last_yaw_;
        double yaw_dot = 0.0;
        
        if (config_.flip_yaw_at_end && !traj_finished_) {
            // 缓慢翻转偏航角
            double d_yaw = slowly_flip_yaw_target_ - last_yaw_;
            d_yaw = normalizeAngle(d_yaw);
            
            if (std::abs(d_yaw) > 0.1) {
                double sign = d_yaw > 0 ? 1.0 : -1.0;
                yaw_dot = sign * M_PI / 2;  // 90度/秒
                yaw = last_yaw_ + yaw_dot * dt;
                yaw = normalizeAngle(yaw);
            } else {
                traj_finished_ = true;
            }
        } else {
            traj_finished_ = true;
        }
        
        // 发布悬停命令
        publishCmd(pos, vel, acc, jerk, yaw, yaw_dot);
        
        last_yaw_ = yaw;
        last_yaw_dot_ = yaw_dot;
        
        // 标记完成
        if (traj_finished_ && status_ == EXECUTING) {
            ROS_INFO("[TrajServer] Trajectory completed");
            status_ = COMPLETED;
        }
    }
}

std::pair<double, double> GCOPTERTrajServer::calculateYaw(double t_cur,
                                                           const Eigen::Vector3d& pos,
                                                           double dt) {
    double yaw = 0.0;
    double yaw_dot = 0.0;
    
    // 计算目标偏航角
    double yaw_target = last_yaw_;
    
    switch (config_.yaw_mode) {
    case YAW_FORWARD: {
        // 朝向飞行方向
        double t_forward = std::min(t_cur + config_.time_forward, traj_data_.duration);
        Eigen::Vector3d forward_pos = traj_data_.traj.getPos(t_forward);
        Eigen::Vector3d dir = forward_pos - pos;
        
        if (dir.head<2>().norm() > 0.1) {
            yaw_target = std::atan2(dir.y(), dir.x());
        }
        break;
    }
    case YAW_FIXED:
        // 固定偏航角（保持初始值）
        yaw_target = last_yaw_;
        break;
        
    case YAW_MANUAL:
        // 手动设置
        yaw_target = target_yaw_;
        break;
        
    case YAW_TOWARDS_GOAL: {
        // 朝向目标点
        Eigen::Vector3d dir = goal_pos_ - pos;
        if (dir.head<2>().norm() > 0.1) {
            yaw_target = std::atan2(dir.y(), dir.x());
        }
        break;
    }
    }
    
    // 计算平滑的偏航角变化
    double d_yaw = yaw_target - last_yaw_;
    
    // 归一化到 [-π, π]
    if (d_yaw > M_PI) d_yaw -= 2 * M_PI;
    if (d_yaw < -M_PI) d_yaw += 2 * M_PI;
    
    // 限制角速度和角加速度
    double yaw_dot_max = config_.yaw_dot_max;
    double yaw_dot_dot_max = config_.yaw_dot_dot_max;
    
    double sign = d_yaw >= 0 ? 1.0 : -1.0;
    double max_ddot = sign * yaw_dot_dot_max;
    double max_dot = sign * yaw_dot_max;
    
    // 计算可达的最大角度变化
    double d_yaw_max;
    if (std::abs(last_yaw_dot_ + dt * max_ddot) <= std::abs(max_dot)) {
        // 还在加速阶段
        d_yaw_max = last_yaw_dot_ * dt + 0.5 * max_ddot * dt * dt;
    } else {
        // 已达到最大角速度
        double t1 = (max_dot - last_yaw_dot_) / max_ddot;
        d_yaw_max = ((dt - t1) + dt) * (max_dot - last_yaw_dot_) / 2.0;
    }
    
    // 限制实际角度变化
    if (std::abs(d_yaw) > std::abs(d_yaw_max)) {
        d_yaw = d_yaw_max;
    }
    
    yaw_dot = d_yaw / dt;
    yaw = last_yaw_ + d_yaw;
    yaw = normalizeAngle(yaw);
    
    // 更新状态
    last_yaw_ = yaw;
    last_yaw_dot_ = yaw_dot;
    
    return std::make_pair(yaw, yaw_dot);
}

void GCOPTERTrajServer::publishCmd(const Eigen::Vector3d& pos,
                                    const Eigen::Vector3d& vel,
                                    const Eigen::Vector3d& acc,
                                    const Eigen::Vector3d& jerk,
                                    double yaw, double yaw_dot) {
    quadrotor_msgs::PositionCommand cmd;
    
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.trajectory_id = traj_data_.traj_id;
    
    // 轨迹状态
    if (status_ == EMERGENCY) {
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTROY_STATUS_ABORT;
    } else if (status_ == EXECUTING) {
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    } else if (status_ == COMPLETED) {
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
    } else {
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
    }
    
    // 位置
    cmd.position.x = pos.x();
    cmd.position.y = pos.y();
    cmd.position.z = pos.z();
    
    // 速度
    cmd.velocity.x = vel.x();
    cmd.velocity.y = vel.y();
    cmd.velocity.z = vel.z();
    
    // 加速度
    cmd.acceleration.x = acc.x();
    cmd.acceleration.y = acc.y();
    cmd.acceleration.z = acc.z();
    
    // Jerk
    cmd.jerk.x = jerk.x();
    cmd.jerk.y = jerk.y();
    cmd.jerk.z = jerk.z();
    
    // 偏航
    cmd.yaw = yaw;
    cmd.yaw_dot = yaw_dot;
    
    // 速度和加速度范数（用于监控）
    // cmd.vel_norm = vel.norm();
    // cmd.acc_norm = acc.norm();
    
    pos_cmd_pub_.publish(cmd);
}

void GCOPTERTrajServer::publishHoverCmd() {
    Eigen::Vector3d hover_pos;
    
    if (have_odom_) {
        hover_pos = current_pos_;
    } else if (traj_data_.valid) {
        // 使用轨迹最后位置
        hover_pos = traj_data_.traj.getPos(traj_data_.duration);
    } else {
        hover_pos = last_pos_;
    }
    
    publishCmd(hover_pos, 
               Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero(),
               last_yaw_, 0.0);
}

// ==================== ROS 回调 ====================

void GCOPTERTrajServer::heartbeatCallback(const std_msgs::EmptyConstPtr& msg) {
    last_heartbeat_time_ = ros::Time::now();
}

void GCOPTERTrajServer::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    current_pos_.x() = msg->pose.pose.position.x;
    current_pos_.y() = msg->pose.pose.position.y;
    current_pos_.z() = msg->pose.pose.position.z;
    
    current_vel_.x() = msg->twist.twist.linear.x;
    current_vel_.y() = msg->twist.twist.linear.y;
    current_vel_.z() = msg->twist.twist.linear.z;
    
    have_odom_ = true;
}

void GCOPTERTrajServer::emergencyStopCallback(const std_msgs::EmptyConstPtr& msg) {
    emergencyStop();
}

// ==================== 工具函数 ====================

double GCOPTERTrajServer::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

} // namespace rog_map

