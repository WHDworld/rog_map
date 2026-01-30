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
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <vector>
#include <memory>
#include <mutex>
#include <atomic>

// 地图系统
#include "star_mvf/star_mvf.h"
#include "rog_map/PolyTraj.h"
// 新架构模块
#include "rog_map/history_graph.h"
#include "rog_map/viewpoint_manager.h"

// 规划器
#include "ego_planner/ego_planner_manager.h"

// 目标重建
#include "rog_map/target_reconstructor.h"

// 语义目标提取
#include "rog_map/semantic_target_extractor.h"

namespace rog_map {

using MapManager = star_mvf::MapManager;
using GlobalMap = star_mvf::GlobalMap;

// ==================== 仿真模式枚举 ====================

enum SimMode {
    SIM_GAZEBO = 0,   // Gazebo + PX4 SITL (自动请求 OFFBOARD + ARM)
    SIM_OPEN3D = 1,   // Open3D 纯仿真 (无 FCU，直接起飞)
    SIM_REAL = 2      // 真机飞行 (等待手动解锁)
};

// ==================== 状态枚举 ====================

enum class ExplorationState {
    INIT = 0,           // 初始化（等待里程计）
    PRE_ARM,            // 预解锁（Gazebo自动/Real等待手动）
    TAKEOFF,            // 起飞中
    WAIT_TRIGGER,       // 等待触发
    LOCALPLAN,          // 本地规划（选择视点、生成轨迹）
    EXEC_EXPLORATION,   // 执行探索（监控轨迹执行）

    RECON_PLAN,         // 规划前往重建视点的路径
    EXEC_RECON,         // 执行重建任务(在该状态下会逐个访问视点)
    FINISH              // 完成
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

// ==================== 重建轨迹分段信息 ====================

struct ReconSegmentInfo {
    double start_time;   // 本段起始时间（相对traj_start_time_）
    double end_time;     // 本段结束时间
    double start_yaw;    // 起始yaw
    double end_yaw;      // 终止yaw
    
    ReconSegmentInfo()
        : start_time(0.0), end_time(0.0), start_yaw(0.0), end_yaw(0.0) {}
    
    ReconSegmentInfo(double st, double et, double sy, double ey)
        : start_time(st), end_time(et), start_yaw(sy), end_yaw(ey) {}
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

    // 重建接近参数
    double recon_approach_threshold = 4.0;   // 接近目标阈值 [m]，距离超过此值时先接近
    double recon_approach_distance = 3.0;    // 接近停止距离 [m]，飞到距目标此距离处
    double recon_yaw_threshold = 0.5;        // Yaw对准阈值 [rad]，约30度

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
    void processPreArm();           // [新增] 预解锁处理
    void processTakeoff();          // [新增] 起飞处理
    void processWaitTrigger();
    void processLocalPlan();
    void processExecExploration();
    
    // ==================== MAVROS 回调 ====================
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    
    // ==================== LOCALPLAN 核心方法 ====================
    
    /**
     * @brief 选择最佳视点
     * @param selected_viewpoint_id 输出选中的视点ID
     * @return 是否找到有效视点
     */
    bool selectBestViewpoint(int& selected_viewpoint_id, const Eigen::Vector3d* from_pos = nullptr);
    
    /**
      * @brief 规划到视点的路径
      * @param viewpoint_id 视点ID
      * @param from_pos 可选的起始位置 (用于无缝切换时的预测位置)
      * @return 是否规划成功
      */
    bool planToViewpoint(int viewpoint_id, const Eigen::Vector3d* from_pos = nullptr);
    
    /**
     * @brief 规划到下一个航点的轨迹
     * @return 是否规划成功
     */
    bool planNextWaypoint();
        
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
    void visualizeHistoryGraph();
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
    
    bool planTrajectory(const Eigen::Vector3d& target, double target_yaw, 
        const Eigen::Vector3d& target_vel);
private:
    // ROS
    ros::NodeHandle nh_;
    ros::Timer fsm_timer_;
    
    ros::Subscriber odom_sub_;
    ros::Subscriber trigger_sub_;
    
    ros::Publisher exploration_state_pub_; // 探索状态
    ros::Publisher frontier_cluster_pub_;  // 前沿簇可视化
    ros::Publisher region_route_pub_; // [Visual] 历史节点拓扑
    ros::Publisher viewpoint_marker_pub_;  // 视点可视化
    ros::Publisher frustum_marker_pub_;    // 视锥体可视化
    ros::Publisher info_marker_pub_;       // 信息可视化
    ros::Publisher path_vis_pub_;          // 路径可视化（保留兼容）
    ros::Publisher viewpoint_vis_pub_;     // 视点可视化（保留兼容）
    
    // 规划器相关
    ros::Timer execution_timer_;          // 100Hz 执行定时器
    ros::Publisher poly_traj_pub_; // 规划轨迹发布
    ros::Publisher pos_cmd_pub_; // 位置命令发布
    ros::Publisher replan_pub_; // 重规划通知发布
    ros::Publisher traj_vis_pub;  // 轨迹可视化（MarkerArray：轨迹+终点yaw+实时yaw）

    rog_map::PolyTraj current_traj_;       // 当前多项式轨迹数据
    double traj_start_time_;               // 轨迹绝对开始时间
    double traj_duration_;                 // 轨迹总时长
    bool has_trajectory_ = false;
    ros::Time last_replan_time_;
    double end_yaw_;                       // 轨迹末端期望 Yaw

    double last_yaw_ = 0.0, last_yaw_dot_ = 0.0;      // Yaw 平滑专用
    Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();  // 上一帧位置（用于异常处理）
    bool exec_initialized_ = false;  // [新增] 标志：executionCallback 是否已初始化
    
    // 4. 双缓存机制 (处理 Time_Start > Time_Now 的情况)
    // 当收到一条为了平滑而设定在"未来"开始的轨迹时，我们需要保留旧轨迹继续飞，直到新轨迹开始
    poly_traj::Trajectory prev_traj_;      // [Fix] Change to Trajectory class for execution
    bool has_prev_traj_ = false;          // 是否有旧轨迹
    double prev_traj_start_time_;
    double prev_traj_duration_;
    std::vector<double> prev_yaw_ctrl_pts_;  // [新增] 旧Yaw轨迹控制点
    bool prev_use_yaw_traj_ = false;         // [新增] 旧轨迹是否使用Yaw B样条
    double prev_last_yaw_ = 0.0;             // [新增] 保存旧轨迹时的last_yaw_

    // ==================== [新增] 轨迹日志记录 ====================
    std::ofstream traj_log_file_;         // 轨迹日志文件流
    bool enable_traj_logging_ = true;     // 是否启用轨迹日志

    // ==================== [新增] 核心函数声明 ====================
    
    /**
     * @brief 100Hz 高频执行回调 (移植自 ego_traj_server::cmdCallback)
     */
     void executionCallback(const ros::TimerEvent& e);

    /**
     * @brief 偏航角平滑计算 (移植自 ego_traj_server::calculate_yaw)
     * 计算受最大角速度/角加速度限制的平滑 Yaw
     */
     std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt);

     /**
      * @brief 规划指令发布
      */
     void publishCmd(const Eigen::Vector3d& p, const Eigen::Vector3d& v, 
                     const Eigen::Vector3d& a, double yaw, double yaw_dot);
    
    void visualizeTrajectory();
    // 模块
    MapManager::Ptr map_manager_;
    GlobalMap::Ptr global_map_;
    HistoryGraph::Ptr history_graph_;
    ViewpointManager::Ptr viewpoint_manager_;
    ego_planner::EGOPlannerManager::Ptr planner_manager_;

    // ========== 重建任务相关 ==========
    TargetReconstructor::Ptr target_reconstructor_;
    
    // ========== 语义目标提取 ==========
    std::unique_ptr<SemanticTargetExtractor> semantic_target_extractor_;
    
    ReconViewpoint current_recon_viewpoint_;
    ros::Time last_recon_check_time_;
    ros::Time last_recon_update_time_;
    void processReconPlan(const ros::TimerEvent& e);
    void processExecRecon(const ros::TimerEvent& e);
    
    // 重建轨迹滑动窗口规划相关
    std::vector<ReconViewpoint> recon_path_;      // 完整重建路径
    int recon_path_idx_;                          // 当前执行到的路径索引(全局)
    int recon_window_start_;                      // 当前窗口起始索引
    static constexpr int RECON_WINDOW_SIZE = 3;   // 滑动窗口大小
    
    // 重建接近阶段状态
    bool recon_approaching_ = false;              // 是否正在接近目标阶段
    Eigen::Vector3d recon_target_center_;         // 当前重建目标中心缓存
    
    /**
     * @brief 滑动窗口规划重建轨迹
     * @param viewpoints 完整视点序列
     * @param from_idx 窗口起始索引
     * @param window_size 窗口大小(默认3)
     * @return 是否规划成功
     */
    bool planReconstructionTrajectory(const std::vector<ReconViewpoint>& viewpoints, 
                                       int from_idx = 0, 
                                       int window_size = RECON_WINDOW_SIZE);
    
    // 重建轨迹分段yaw信息（用于视点间平滑过渡）
    std::vector<ReconSegmentInfo> recon_segments_;
    
    /**
     * @brief 计算重建模式下的yaw（视点间平滑插值）
     * @param t_cur 当前轨迹时间
     * @param dt 时间步长
     * @return {yaw, yaw_dot}
     */
    std::pair<double, double> calculateReconSegmentYaw(double t_cur, double dt);
    
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
    ros::Time trigger_time_;           // 记录触发时刻，用于延迟启动
    bool trigger_delay_started_{false}; // 是否已开始延迟计时
    
    // 起飞状态
    Eigen::Vector3d initial_pos_ = Eigen::Vector3d::Zero();
    ros::Time takeoff_start_time_;
    bool takeoff_complete_ = false;
    
    // 仿真模式与 MAVROS
    int sim_mode_ = SIM_OPEN3D;              // 默认 Open3D 模式
    mavros_msgs::State current_state_;       // FCU 状态
    ros::Subscriber state_sub_;              // FCU 状态订阅
    ros::ServiceClient arming_client_;       // 解锁服务
    ros::ServiceClient set_mode_client_;     // 模式切换服务
    bool offboard_ready_ = false;            // OFFBOARD 模式是否就绪
    
    // 当前探索目标
    // 当前探索目标
    int current_viewpoint_id_ = -1;       // 当前目标视点ID
    std::vector<WaypointInfo> waypoint_queue_; // 航点队列（历史节点+视点）
    int current_waypoint_idx_ = 0;        // 当前航点索引

    // ==================== [新增] 视点选择稳定性优化 ====================
    
    // 视点选择状态
    int last_selected_cluster_id_ = -1;        // 上一帧选中的簇 ID（用于迟滞）
    int last_selected_anchor_id_ = -1;         // 上一帧绑定的锚点 ID（用于锚点锁定）
    ros::Time last_viewpoint_switch_time_;     // 上次视点切换时间
    
    // 配置参数结构体
    struct ViewpointSelectionConfig {
        double hysteresis_gain = 1.2;          // 迟滞增益（当前目标获得 20% 加分）
        double heading_penalty_weight = 0.5;   // 航向惩罚权重（后方目标代价 ×1.5）
        double min_switch_interval = 0.5;      // 最小切换间隔 [s]
        double anchor_lock_distance = 2.0;     // 锚点锁定距离阈值 [m]
        double velocity_threshold = 0.3;       // 使用速度方向的最低速度 [m/s]
    } vp_selection_config_;
    
    // 状态计时
    ros::Time state_start_time_;
    
    // 黑名单簇 (用于规划失败时的即时切换)
    std::unordered_set<int> blacklisted_clusters_;
    
    // 黑盒/覆盖检测
    std::vector<Eigen::Vector3f> current_frontier_; // 目标簇
    Eigen::Vector3d current_viewpoint_centroid_;            // 目标簇质心(辅助黑名单)
    Eigen::Vector3d current_vp_pos_; // 当前视点位置
    double current_vp_yaw_; // 当前视点朝向
    int current_frontier_history_id_ = -1; // 目标簇历史ID
    bool triggerSeamlessReplan();
    bool checkCurrentGoalCovered();

    // [已删除] use_replan_start_ 相关变量已被 planTrajectory 内部统一预测逻辑替代

    DijkstraResult cached_dijkstra_result_;

    std::vector<Eigen::Vector3d> current_global_path_; // 用于长距离导航的拓扑路径

    bool use_global_path_ = false; // 标志位：当前是否在使用全局拓扑路径导航
    // 轨迹执行状态
    ros::Time trajectory_start_time_;
    ros::Time last_replan_check_time_;
    ros::Time last_coverage_check_time_;
    ros::Time last_collision_check_time_;
    
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
