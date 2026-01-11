/**
 * @file exploration_fsm_new.h
 * @brief 探索状态机 - 重构版（简化4状态 + 集成规划器）
 * 
 * 状态设计：
 * 1. INIT        - 初始化，等待里程计和起飞
 * 2. WAIT_TRIGGER - 等待探索触发
 * 3. LOCALPLAN   - 本地规划（选择视点、规划轨迹）
 * 4. EXEC_EXPLORATION - 执行探索（碰撞检测、覆盖检测、超时检测）
 * 
 * 设计原则：
 * - 直接调用规划器接口，不再有单独的规划状态机
 * - 视点由 ViewpointManager 生成，本FSM只负责选择和执行
 * - 路径由 HistoryGraph 提供，支持多跳历史节点引导
 */

#ifndef EXPLORATION_FSM_NEW_H
#define EXPLORATION_FSM_NEW_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Int8.h>

#include <vector>
#include <memory>
#include <mutex>
#include <atomic>

// 地图系统
#include "star_mvf/star_mvf.h"

// 新架构模块
#include "rog_map/history_graph.h"
#include "rog_map/viewpoint_manager.h"

// 规划器
#include "ego_planner/ego_planner_manager.h"

namespace rog_map {

using MapManager = star_mvf::MapManager;
using GlobalMap = star_mvf::GlobalMap;

// ==================== 状态枚举 ====================

enum class ExplorationState {
    INIT = 0,           // 初始化（等待里程计、起飞）
    WAIT_TRIGGER,       // 等待触发
    LOCALPLAN,          // 本地规划（选择视点、生成轨迹）
    EXEC_EXPLORATION    // 执行探索（监控轨迹执行）
};

// ==================== 航点信息 ====================

struct WaypointInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3d position;             // 位置
    double yaw;                           // 朝向
    bool is_viewpoint;                    // 是否为探索视点（否则是历史节点）
    int id;                               // 视点ID或历史节点ID
    
    WaypointInfo()
        : position(Eigen::Vector3d::Zero())
        , yaw(0.0)
        , is_viewpoint(false)
        , id(-1) {}
    
    WaypointInfo(const Eigen::Vector3d& pos, double y, bool is_vp, int i)
        : position(pos), yaw(y), is_viewpoint(is_vp), id(i) {}
};

// ==================== 配置参数 ====================

struct ExplorationFSMConfig {
    // FSM频率
    double fsm_frequency = 100.0;         // Hz
    
    // 起飞参数
    double takeoff_height = 1.0;          // 起飞高度 [m]
    double takeoff_timeout = 10.0;        // 起飞超时 [s]
    double init_wait_time = 2.0;          // 初始化等待时间 [s]
    
    // 到达判断
    double reach_threshold = 0.5;         // 到达阈值 [m]
    double yaw_threshold = 0.3;           // 偏航到达阈值 [rad]
    
    // 重规划触发
    double replan_check_interval = 0.5;   // 重规划检查间隔 [s]
    double waypoint_approach_dist = 1.5;  // 接近航点时触发重规划的距离 [m]
    double trajectory_timeout = 10.0;     // 轨迹执行超时 [s]
    double coverage_trigger_ratio = 0.3;  // 触发重规划的覆盖比例
    
    // 碰撞检测
    double collision_check_interval = 0.05; // 碰撞检测间隔 [s]
    double collision_lookahead = 0.5;     // 碰撞预测时间 [s]
    
    // 视点选择
    double max_viewpoint_distance = 15.0; // 最大视点距离 [m]
    double close_viewpoint_threshold = 5.0; // 近距离视点阈值（直接规划）[m]
    
    // 规划参数
    double planning_horizon = 7.5;        // 规划视野 [m]
    double max_vel = 1.5;                 // 最大速度 [m/s]
    double max_acc = 3.0;                 // 最大加速度 [m/s²]
    double max_yaw_rate = 1.0;            // 最大偏航率 [rad/s]
    double max_yaw_acc = 1.0;             // 最大偏航加速度 [rad/s²]
    // 自动开始
    bool auto_start = false;
    
    // 可视化
    bool enable_visualization = true;

    std::string odom_topic;
    std::string lidar_topic;
};

// ==================== 主类 ====================

class ExplorationFSMNew {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ExplorationFSMNew> Ptr;
    
    ExplorationFSMNew();
    ~ExplorationFSMNew();
    
    /**
     * @brief 初始化
     * @param nh ROS节点句柄
     * @param map_manager 地图管理器
     */
    void init(ros::NodeHandle& nh, MapManager::Ptr map_manager);
    
    /**
     * @brief 获取当前状态
     */
    ExplorationState getState() const { return state_; }
    std::string getStateString() const;
    
    /**
     * @brief 手动触发探索
     */
    void triggerExploration();
    
    /**
     * @brief 停止探索
     */
    void stopExploration();
    
private:
    // ==================== 状态转换 ====================
    
    void transitState(ExplorationState new_state, const std::string& reason);
    
    // ==================== 回调函数 ====================
    
    void fsmCallback(const ros::TimerEvent& e);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    
    // ==================== 状态处理函数 ====================
    
    void processInit();
    void processWaitTrigger();
    void processLocalPlan();
    void processExecExploration();
    
    // ==================== LOCALPLAN 核心方法 ====================
    
    /**
     * @brief 选择最佳视点
     * @param selected_viewpoint_id 输出选中的视点ID
     * @return 是否找到有效视点
     */
    bool selectBestViewpoint(int& selected_viewpoint_id);
    
    /**
     * @brief 规划到视点的路径
     * @param viewpoint_id 视点ID
     * @return 是否规划成功
     */
    bool planToViewpoint(int viewpoint_id);
    
    /**
     * @brief 规划到下一个航点的轨迹
     * @return 是否规划成功
     */
    bool planNextWaypoint();
    
    /**
     * @brief 调用规划器生成轨迹
     * @param target 目标位置
     * @param target_yaw 目标朝向
     * @param target_vel 目标速度（通常为0悬停，或飞越时有速度）
     * @return 是否规划成功
     */
    bool callPlanner(const Eigen::Vector3d& target, double target_yaw,
                     const Eigen::Vector3d& target_vel = Eigen::Vector3d::Zero());
    
    // ==================== EXEC_EXPLORATION 检测方法 ====================
    
    /**
     * @brief 检查轨迹碰撞
     * @return true 如果检测到碰撞
     */
    bool checkCollision();
    
    /**
     * @brief 检查是否到达当前航点
     * @return true 如果到达
     */
    bool checkReachWaypoint();
        
    /**
     * @brief 检查轨迹执行是否超时
     * @return true 如果超时
     */
    bool checkTimeout();
    
    /**
     * @brief 检查是否接近当前航点（需要准备下一个）
     * @return true 如果接近
     */
    bool checkApproachingWaypoint();
    
    // ==================== 辅助函数 ====================
    
    /**
     * @brief 发布起飞命令
     */
    void publishTakeoffCmd();
    
    /**
     * @brief 发布悬停命令
     */
    void publishHoverCmd();
    
    /**
     * @brief 从轨迹采样并发布位置命令
     */
    void publishTrajectoryCmd();
    
    /**
     * @brief 计算两个yaw角的差
     */
    double getYawDiff(double yaw1, double yaw2);
    
    // ==================== 可视化 ====================
    
    void visualize();
    void visualizeFrontierClusters();
    void visualizeViewpoints();
    void visualizeFrustums();
    void visualizeExplorationInfo();
    void clearVisualization();
    
    /**
     * @brief 创建视锥体marker
     */
    void createFrustumMarker(
        const Eigen::Vector3f& position,
        float yaw, float pitch,
        float length, float fov_h, float fov_v,
        int id,
        const std::array<float, 4>& color,
        visualization_msgs::MarkerArray& markers);

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Timer fsm_timer_;
    
    ros::Subscriber odom_sub_;
    ros::Subscriber trigger_sub_;
    
    ros::Publisher pos_cmd_pub_;           // 位置命令
    ros::Publisher exploration_state_pub_; // 探索状态
    ros::Publisher frontier_cluster_pub_;  // 前沿簇可视化
    ros::Publisher viewpoint_marker_pub_;  // 视点可视化
    ros::Publisher frustum_marker_pub_;    // 视锥体可视化
    ros::Publisher info_marker_pub_;       // 信息可视化
    ros::Publisher path_vis_pub_;          // 路径可视化（保留兼容）
    ros::Publisher viewpoint_vis_pub_;     // 视点可视化（保留兼容）
    
    // 模块
    MapManager::Ptr map_manager_;
    GlobalMap::Ptr global_map_;
    HistoryGraph::Ptr history_graph_;
    ViewpointManager::Ptr viewpoint_manager_;
    ego_planner::EGOPlannerManager::Ptr planner_manager_;
    
    // 配置
    ExplorationFSMConfig config_;
    
    // 状态
    std::atomic<ExplorationState> state_{ExplorationState::INIT};
    
    // 里程计
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d odom_vel_ = Eigen::Vector3d::Zero();
    double odom_yaw_ = 0.0;
    bool have_odom_ = false;
    
    // 触发标志
    std::atomic<bool> triggered_{false};
    
    // 起飞状态
    Eigen::Vector3d initial_pos_ = Eigen::Vector3d::Zero();
    ros::Time takeoff_start_time_;
    bool takeoff_complete_ = false;
    
    // 当前探索目标
    int current_viewpoint_id_ = -1;       // 当前目标视点ID
    std::vector<WaypointInfo> waypoint_queue_; // 航点队列（历史节点+视点）
    int current_waypoint_idx_ = 0;        // 当前航点索引
    
    std::vector<Eigen::Vector3d> current_global_path_; // 用于长距离导航的拓扑路径
    bool use_global_path_ = false; // 标志位：当前是否在使用全局拓扑路径导航
    // 轨迹执行状态
    ros::Time trajectory_start_time_;
    ros::Time last_replan_check_time_;
    ros::Time last_coverage_check_time_;
    ros::Time last_collision_check_time_;
    bool has_trajectory_ = false;
    
    // 悬停状态
    Eigen::Vector3d hover_pos_ = Eigen::Vector3d::Zero();
    double hover_yaw_ = 0.0;
    
    // 时间戳
    ros::Time fsm_init_time_;
    
    // 统计
    int total_viewpoints_visited_ = 0;
    double total_distance_ = 0.0;
    
    // 线程安全
    mutable std::mutex state_mutex_;
    mutable std::mutex odom_mutex_;
};

} // namespace rog_map

#endif // EXPLORATION_FSM_NEW_H
