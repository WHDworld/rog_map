/**
 * @file gcopter_traj_server.h
 * @brief GCOPTER 轨迹服务器 - 将轨迹转换为控制命令
 * 
 * 功能:
 * 1. 订阅轨迹消息
 * 2. 定时采样轨迹生成期望状态
 * 3. 计算偏航角和偏航角速度
 * 4. 发布 PositionCommand 给控制器
 * 
 * 安全特性:
 * 1. 心跳检测 - 规划器失联则悬停
 * 2. 轨迹超时 - 轨迹结束后悬停
 * 3. 紧急停止 - 立即悬停
 */

#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <string>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>

#include "gcopter/trajectory.hpp"

namespace rog_map {

/**
 * @brief GCOPTER 轨迹服务器
 * 
 * 将优化后的轨迹转换为控制器可执行的位置命令
 */
class GCOPTERTrajServer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 轨迹状态
     */
    enum TrajStatus {
        IDLE = 0,           // 空闲（悬停）
        EXECUTING,          // 执行中
        COMPLETED,          // 完成
        EMERGENCY           // 紧急停止
    };

    /**
     * @brief 偏航控制模式
     */
    enum YawMode {
        YAW_FORWARD = 0,    // 朝向飞行方向
        YAW_FIXED,          // 固定偏航角
        YAW_MANUAL,         // 手动设置
        YAW_TOWARDS_GOAL    // 朝向目标点
    };

    /**
     * @brief 配置参数
     */
    struct Config {
        // 发布频率
        double cmd_rate = 100.0;           // 命令发布频率 [Hz]
        
        // 偏航控制
        YawMode yaw_mode = YAW_FORWARD;    // 偏航模式
        double yaw_dot_max = 2 * M_PI;     // 最大偏航角速度 [rad/s]
        double yaw_dot_dot_max = 5 * M_PI; // 最大偏航角加速度 [rad/s²]
        double time_forward = 1.0;         // 前向看时间 [s]
        
        // 安全参数
        double heartbeat_timeout = 0.5;    // 心跳超时 [s]
        bool enable_heartbeat_check = true;
        
        // 轨迹结束行为
        bool flip_yaw_at_end = false;      // 结束时翻转偏航角
        bool hover_at_end = true;          // 结束时悬停
        
        // 动力学限制
        double max_vel = 2.0;
        double max_acc = 2.0;
    };

    /**
     * @brief 本地轨迹数据
     */
    struct TrajData {
        int traj_id = 0;
        double start_time = 0.0;
        double duration = 0.0;
        Trajectory<5> traj;
        bool valid = false;
    };

public:
    GCOPTERTrajServer() = default;
    ~GCOPTERTrajServer() = default;

    /**
     * @brief 初始化
     * @param nh ROS 节点句柄
     */
    void init(ros::NodeHandle& nh);

    /**
     * @brief 设置新轨迹
     * @param traj 轨迹
     * @param traj_id 轨迹 ID
     */
    void setTrajectory(const Trajectory<5>& traj, int traj_id);

    /**
     * @brief 设置目标偏航角
     */
    void setTargetYaw(double yaw) { target_yaw_ = yaw; }

    /**
     * @brief 设置目标点（用于 YAW_TOWARDS_GOAL 模式）
     */
    void setGoalPosition(const Eigen::Vector3d& goal) { goal_pos_ = goal; }

    /**
     * @brief 紧急停止
     */
    void emergencyStop();

    /**
     * @brief 恢复执行
     */
    void resume();

    /**
     * @brief 获取当前状态
     */
    TrajStatus getStatus() const { return status_; }

    /**
     * @brief 是否正在执行
     */
    bool isExecuting() const { return status_ == EXECUTING; }

    /**
     * @brief 获取当前位置
     */
    Eigen::Vector3d getCurrentPosition() const { return current_pos_; }

    /**
     * @brief 获取期望位置
     */
    Eigen::Vector3d getDesiredPosition() const { return desired_pos_; }

private:
    // ==================== 核心功能 ====================
    
    /**
     * @brief 命令发布定时器回调
     */
    void cmdTimerCallback(const ros::TimerEvent& e);

    /**
     * @brief 计算偏航角和偏航角速度
     * @param t_cur 当前轨迹时间
     * @param pos 当前位置
     * @param dt 时间间隔
     * @return (yaw, yaw_dot)
     */
    std::pair<double, double> calculateYaw(double t_cur, 
                                            const Eigen::Vector3d& pos, 
                                            double dt);

    /**
     * @brief 发布位置命令
     */
    void publishCmd(const Eigen::Vector3d& pos,
                    const Eigen::Vector3d& vel,
                    const Eigen::Vector3d& acc,
                    const Eigen::Vector3d& jerk,
                    double yaw, double yaw_dot);

    /**
     * @brief 发布悬停命令
     */
    void publishHoverCmd();

    // ==================== ROS 回调 ====================

    void heartbeatCallback(const std_msgs::EmptyConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void emergencyStopCallback(const std_msgs::EmptyConstPtr& msg);

    // ==================== 工具函数 ====================

    /**
     * @brief 归一化角度到 [-π, π]
     */
    double normalizeAngle(double angle) const;

private:
    // ==================== ROS ====================
    ros::NodeHandle nh_;
    ros::Timer cmd_timer_;
    
    ros::Publisher pos_cmd_pub_;
    ros::Publisher traj_vis_pub_;
    
    ros::Subscriber heartbeat_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber emergency_stop_sub_;

    // ==================== 配置 ====================
    Config config_;

    // ==================== 状态 ====================
    TrajStatus status_ = IDLE;
    TrajData traj_data_;
    
    // 心跳
    ros::Time last_heartbeat_time_;
    bool heartbeat_lost_ = false;
    
    // 里程计
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    bool have_odom_ = false;
    
    // 期望状态
    Eigen::Vector3d desired_pos_;
    Eigen::Vector3d desired_vel_;
    Eigen::Vector3d last_pos_;
    
    // 偏航控制
    double last_yaw_ = 0.0;
    double last_yaw_dot_ = 0.0;
    double target_yaw_ = 0.0;
    Eigen::Vector3d goal_pos_;
    
    // 时间
    ros::Time last_cmd_time_;
    
    // 轨迹结束处理
    bool traj_finished_ = false;
    double slowly_flip_yaw_target_ = 0.0;
};

} // namespace rog_map

