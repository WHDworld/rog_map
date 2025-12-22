/**
 * @file gcopter_fsm.h
 * @brief GCOPTER 规划状态机 - 管理 PX4 模式切换、起飞、规划和执行
 * 
 * 完整状态转换图:
 * 
 *    ┌──────┐   have_odom   ┌──────────┐   armed &    ┌─────────┐
 *    │ INIT │ ───────────▶ │ PRE_ARM  │ ──────────▶ │ TAKEOFF │
 *    └──────┘              └──────────┘  offboard   └─────────┘
 *                                                        │
 *                         ┌─────────────┐                │ takeoff_complete
 *                         │ WAIT_TARGET │ ◀──────────────┘
 *                         └─────────────┘
 *                                │
 *                         have_target
 *                                ▼
 *                         ┌─────────────┐
 *                         │ GEN_NEW_TRAJ│
 *                         └─────────────┘
 *                           │      ▲
 *                    success│      │fail
 *                           ▼      │
 *                         ┌─────────────┐
 *           ┌─────────────│  EXEC_TRAJ  │─────────────┐
 *           │             └─────────────┘   reach_goal│
 *           │                  │   ▲                  ▼
 *  collision│         replan_t │   │ success   ┌─────────────┐
 *  detected │                  ▼   │           │ WAIT_TARGET │
 *           │             ┌─────────────┐      └─────────────┘
 *           │             │ REPLAN_TRAJ │
 *           │             └─────────────┘
 *           ▼                    │
 *   ┌───────────────┐            │
 *   │EMERGENCY_STOP │◀───────────┘ fail & critical
 *   └───────────────┘
 */

#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

// MAVROS 相关
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include "gcopter/gcopter_planner.h"
#include "gcopter/trajectory.hpp"

namespace rog_map {

// 前向声明
class ROGMap;
class GCOPTERTrajServer;

/**
 * @brief GCOPTER 规划状态机
 * 
 * 核心功能:
 * 1. 管理 PX4 模式切换和解锁
 * 2. 自动起飞到指定高度
 * 3. 管理规划状态转换
 * 4. 安全监控（碰撞检测、心跳）
 * 5. 自动重规划
 * 6. 紧急停止
 */
class GCOPTERReplanFSM {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 状态机执行状态
     */
    enum FSMState {
        INIT = 0,           // 初始化（等待里程计）
        PRE_ARM,            // 预解锁（发送 setpoint，切换 OFFBOARD，解锁）
        TAKEOFF,            // 起飞
        WAIT_TARGET,        // 等待目标
        GEN_NEW_TRAJ,       // 生成新轨迹
        REPLAN_TRAJ,        // 重规划轨迹
        EXEC_TRAJ,          // 执行轨迹
        EMERGENCY_STOP      // 紧急停止
    };

    /**
     * @brief 目标类型
     */
    enum TargetType {
        MANUAL_TARGET = 1,  // 手动点击（RViz）
        PRESET_TARGET = 2,  // 预设路径点
        API_TARGET = 3      // 通过 API 设置
    };

    /**
     * @brief 本地轨迹数据
     */
    struct LocalTrajData {
        int traj_id = 0;
        double start_time = 0.0;
        double duration = 0.0;
        Trajectory<5> traj;
        std::vector<Eigen::Vector3d> rrt_path;
        
        // 碰撞检测点缓存
        // pts_chk[piece_id][check_id] = (time, position)
        std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> pts_chk;
    };

    /**
     * @brief 全局轨迹数据
     */
    struct GlobalTrajData {
        double global_start_time = 0.0;
        double duration = 0.0;
        double glb_t_of_lc_tgt = 0.0;      // 当前局部目标在全局轨迹上的时间
        double last_glb_t_of_lc_tgt = 0.0; // 上一个局部目标时间
        std::vector<Eigen::Vector3d> waypoints;
        Trajectory<5> traj;
    };

    /**
     * @brief 轨迹容器
     */
    struct TrajContainer {
        LocalTrajData local_traj;
        GlobalTrajData global_traj;
    };

    /**
     * @brief FSM 配置参数
     */
    struct FSMConfig {
        // ==== PX4 控制参数 ====
        bool real_world_flag = false;      // 是否真实环境（true=手动解锁，false=自动）
        double takeoff_height = 1.0;       // 起飞高度 [m]
        double takeoff_threshold = 0.1;    // 起飞完成阈值 [m]
        int pre_arm_setpoint_count = 100;  // 预解锁 setpoint 发送次数
        double service_timeout = 1.0;      // 服务请求间隔 [s]
        
        // ==== 起飞参数 ====
        double takeoff_accel_time = 2.0;   // 加速时间 [s]
        double takeoff_cruise_time = 0.5;  // 巡航时间 [s]
        double takeoff_decel_time = 2.0;   // 减速时间 [s]
        double takeoff_max_vel = 0.5;      // 最大速度 [m/s]
        double takeoff_max_accel = 0.25;   // 最大加速度 [m/s²]
        
        // ==== 目标类型 ====
        int target_type = MANUAL_TARGET;
        
        // ==== 规划参数 ====
        double planning_horizon = 7.5;     // 规划视野 [m]
        double replan_thresh = 2.0;        // 重规划时间阈值 [s]
        double no_replan_thresh = 1.0;     // 不重规划距离阈值 [m]
        double emergency_time = 0.8;       // 紧急时间 [s]
        
        // ==== 安全参数 ====
        bool enable_fail_safe = true;      // 启用失败安全
        double heartbeat_timeout = 0.5;    // 心跳超时 [s]
        double collision_check_dt = 0.05;  // 碰撞检测时间步长 [s]
        
        // ==== 动力学参数 ====
        double max_vel = 2.0;              // 最大速度 [m/s]
        double max_acc = 2.0;              // 最大加速度 [m/s²]
        
        // ==== 预设路径点 ====
        std::vector<Eigen::Vector3d> preset_waypoints;
    };

public:
    GCOPTERReplanFSM() = default;
    ~GCOPTERReplanFSM() = default;

    /**
     * @brief 初始化状态机
     * @param nh ROS 节点句柄
     * @param rog_map 地图指针
     * @param traj_server 轨迹服务器指针
     */
    void init(ros::NodeHandle& nh, ROGMap* rog_map, GCOPTERTrajServer* traj_server);

    /**
     * @brief 设置新的目标点
     */
    bool setGoal(const Eigen::Vector3d& goal_pos, double goal_yaw = 0.0);

    /**
     * @brief 设置多个路径点
     */
    bool setWaypoints(const std::vector<Eigen::Vector3d>& waypoints);

    /**
     * @brief 强制停止
     */
    void forceStop();

    /**
     * @brief 获取当前状态
     */
    FSMState getCurrentState() const { return exec_state_; }

    /**
     * @brief 获取状态字符串
     */
    std::string getStateString() const;

    /**
     * @brief 是否正在执行轨迹
     */
    bool isExecuting() const { return exec_state_ == EXEC_TRAJ; }

    /**
     * @brief 是否空闲等待
     */
    bool isIdle() const { return exec_state_ == WAIT_TARGET; }

    /**
     * @brief 获取当前轨迹
     */
    const TrajContainer& getTraj() const { return traj_; }

    /**
     * @brief 获取当前位置
     */
    Eigen::Vector3d getCurrentPosition() const { return odom_pos_; }

    /**
     * @brief 获取当前速度
     */
    Eigen::Vector3d getCurrentVelocity() const { return odom_vel_; }

    /**
     * @brief 获取目标位置
     */
    Eigen::Vector3d getGoalPosition() const { return final_goal_; }

    /**
     * @brief 无人机是否已解锁并处于 OFFBOARD 模式
     */
    bool isArmedAndOffboard() const { return is_armed_ && is_offboard_; }

private:
    // ==================== 状态机核心 ====================
    
    /**
     * @brief FSM 执行回调（主循环）
     */
    void execFSMCallback(const ros::TimerEvent& e);

    /**
     * @brief 安全检查回调
     */
    void checkCollisionCallback(const ros::TimerEvent& e);

    /**
     * @brief 状态转换
     */
    void changeFSMExecState(FSMState new_state, const std::string& pos_call);

    /**
     * @brief 打印当前状态
     */
    void printFSMExecState();

    // ==================== PX4 控制 ====================

    /**
     * @brief 发送初始 setpoint（满足 PX4 OFFBOARD 模式要求）
     */
    void sendInitialSetpoints();

    /**
     * @brief 切换到 OFFBOARD 模式并解锁
     */
    void armAndOffboard();

    /**
     * @brief 检查起飞是否完成
     */
    bool checkTakeoffComplete();

    /**
     * @brief 更新起飞轨迹（平滑）
     */
    void updateTakeoffTrajectory();

    /**
     * @brief 发布起飞命令
     */
    void publishTakeoffCmd();

    /**
     * @brief 发布悬停命令（保持当前位置）
     */
    void publishHoverCmd();

    /**
     * @brief 发布 setpoint（用于 PX4 模式切换）
     */
    void publishSetpoint();

    // ==================== 规划接口 ====================

    /**
     * @brief 从全局轨迹规划（生成新轨迹）
     */
    bool planFromGlobalTraj(int trial_times = 1);

    /**
     * @brief 从当前轨迹重规划
     */
    bool planFromLocalTraj(int trial_times = 1);

    /**
     * @brief 规划下一个路径点
     */
    bool planNextWaypoint(const Eigen::Vector3d& next_wp);

    /**
     * @brief 紧急停止
     */
    bool callEmergencyStop(const Eigen::Vector3d& stop_pos);

    /**
     * @brief 获取局部目标点
     */
    void getLocalTarget(const Eigen::Vector3d& start_pt, 
                        const Eigen::Vector3d& global_end_pt,
                        Eigen::Vector3d& local_target_pos,
                        Eigen::Vector3d& local_target_vel,
                        bool& touch_goal);

    /**
     * @brief 检查并修正碰撞目标点
     */
    bool modifyInCollisionGoal();

    /**
     * @brief 生成碰撞检测点
     */
    void generateCheckPoints(const Trajectory<5>& traj,
                             std::vector<std::vector<std::pair<double, Eigen::Vector3d>>>& pts_chk);

    // ==================== ROS 回调 ====================

    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void mandatoryStopCallback(const std_msgs::Empty& msg);

    // ==================== 轨迹发布 ====================

    /**
     * @brief 发布轨迹消息
     */
    void publishTrajectory();

    /**
     * @brief 发布心跳
     */
    void publishHeartbeat();

private:
    // ==================== ROS ====================
    ros::NodeHandle nh_;
    ros::Timer exec_timer_;           // FSM 执行定时器
    ros::Timer safety_timer_;         // 安全检查定时器
    
    ros::Subscriber odom_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber trigger_sub_;
    ros::Subscriber mandatory_stop_sub_;
    
    ros::Publisher traj_pub_;         // 轨迹发布（给 TrajServer）
    ros::Publisher heartbeat_pub_;    // 心跳发布
    ros::Publisher goal_vis_pub_;     // 目标可视化
    ros::Publisher setpoint_pub_;     // setpoint 发布（用于 PX4）
    ros::Publisher pos_cmd_pub_;      // 位置命令发布

    // MAVROS 服务客户端
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    // ==================== 规划器 ====================
    std::unique_ptr<GCOPTERPlanner> planner_;
    ROGMap* rog_map_ = nullptr;
    GCOPTERTrajServer* traj_server_ = nullptr;

    // ==================== 配置 ====================
    FSMConfig config_;

    // ==================== 状态 ====================
    FSMState exec_state_ = INIT;
    int consecutive_state_calls_ = 0;
    int traj_id_counter_ = 0;

    // ==================== PX4 状态 ====================
    bool is_armed_ = false;           // 是否解锁
    bool is_offboard_ = false;        // 是否在 OFFBOARD 模式
    bool is_connected_ = false;       // MAVROS 是否连接
    mavros_msgs::State current_state_;
    ros::Time last_request_time_;     // 上次服务请求时间

    // ==================== 起飞状态 ====================
    bool takeoff_complete_ = false;
    bool initial_pose_recorded_ = false;
    Eigen::Vector3d initial_pos_;     // 初始位置
    double initial_yaw_ = 0.0;        // 初始偏航角
    ros::Time takeoff_start_time_;    // 起飞开始时间
    
    // 起飞轨迹变量
    double trajectory_z_ = 0.0;       // 当前目标 Z 位置
    double trajectory_vel_z_ = 0.0;   // 当前目标 Z 速度
    double trajectory_acc_z_ = 0.0;   // 当前目标 Z 加速度
    bool takeoff_trajectory_initialized_ = false;

    // ==================== 标志 ====================
    bool have_odom_ = false;
    bool have_target_ = false;
    bool have_trigger_ = false;
    bool have_new_target_ = false;
    bool mandatory_stop_ = false;
    bool touch_goal_ = false;
    bool flag_escape_emergency_ = true;

    // ==================== 状态数据 ====================
    // 里程计
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d odom_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d odom_acc_ = Eigen::Vector3d::Zero();
    double odom_yaw_ = 0.0;
    
    // 规划起点
    Eigen::Vector3d start_pt_;
    Eigen::Vector3d start_vel_;
    Eigen::Vector3d start_acc_;
    
    // 目标
    Eigen::Vector3d final_goal_;
    double final_goal_yaw_ = 0.0;
    Eigen::Vector3d local_target_pt_;
    Eigen::Vector3d local_target_vel_;
    
    // 路径点
    std::vector<Eigen::Vector3d> waypoints_;
    int current_waypoint_id_ = 0;

    // ==================== 轨迹 ====================
    TrajContainer traj_;
    ros::Time heartbeat_time_;
};

} // namespace rog_map
