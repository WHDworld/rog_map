/**
 * @file exploration_fsm.h
 * @brief 探索状态机 - 参考Falcon-Planner设计
 * 
 * 功能：
 * 1. 管理探索过程的状态转换
 * 2. 调用视点生成器生成探索视点
 * 3. 实时可视化探索状态
 * 4. 与EGO-Planner集成进行轨迹规划
 */

#ifndef EXPLORATION_FSM_H
#define EXPLORATION_FSM_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"
#include "rog_map/frontier_viewpoint_generator.h"

namespace rog_map {

// 使用 star_mvf 类型
using MapManager = star_mvf::MapManager;
using GlobalMap = star_mvf::GlobalMap;

// ==================== 探索状态枚举 ====================
enum class ExplorationState {
    INIT = 0,           // 初始化
    WAIT_TRIGGER,       // 等待触发
    UPDATE_FRONTIER,    // 更新前沿
    PLAN_VIEWPOINTS,    // 规划视点
    EXEC_EXPLORATION,   // 执行探索
    FINISH,             // 探索完成
    EMERGENCY_STOP      // 紧急停止
};

// ==================== 探索数据结构 ====================
struct ExplorationData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // 当前探索状态
    std::vector<ExplorationViewpoint> current_viewpoints;
    
    // 当前目标
    int current_viewpoint_idx;
    ExplorationViewpoint next_viewpoint;
    bool viewpoint_published;  // 标志：当前视点是否已发布给规划器
    
    // 统计信息
    ros::Time start_time;
    ros::Time finish_time;
    double total_distance;
    int total_viewpoints_visited;
    int total_frontiers_explored;
    
    // 探索进度
    float exploration_progress;  // [0, 1]
    
    ExplorationData()
        : current_viewpoint_idx(0)
        , viewpoint_published(false)
        , total_distance(0.0)
        , total_viewpoints_visited(0)
        , total_frontiers_explored(0)
        , exploration_progress(0.0f)
    {}
    
    void reset() {
        current_viewpoints.clear();
        current_viewpoint_idx = 0;
        viewpoint_published = false;
        total_distance = 0.0;
        total_viewpoints_visited = 0;
        total_frontiers_explored = 0;
        exploration_progress = 0.0f;
    }
};

// ==================== 探索参数配置 ====================
struct ExplorationFSMConfig {
    // FSM参数
    double fsm_frequency = 100.0;           // FSM主循环频率 (Hz)
    double frontier_update_frequency = 2.0; // 前沿更新频率 (Hz)
    double safety_check_frequency = 20.0;   // 安全检查频率 (Hz)
    double visualization_frequency = 10.0;  // 可视化频率 (Hz)
    
    // 重规划参数（参考 Falcon Planner）
    double replan_thresh_reached = 0.5;         // 到达阈值 (m)
    double replan_thresh_yaw = 0.1;             // 偏航阈值 (rad)
    double replan_thresh_time = 3.0;            // 周期性重规划时间 (s) - Falcon: 3.0s
    double replan_thresh_frontier = 0.5;        // 最小执行时间后才检查覆盖 (s) - Falcon: 0.2s
    double replan_thresh_traj_remain = 0.5;     // 轨迹剩余时间阈值 (s) - Falcon: 0.05s
    double frontier_covered_ratio = 0.25;       // 前沿覆盖比例触发重规划 - Falcon: 0.25
    
    // 探索参数
    bool auto_start = false;                // 自动开始探索
    double init_wait_time = 2.0;            // 初始化等待时间 (s)
    int min_frontier_cluster_size = 50;     // 最小前沿簇大小
    double takeoff_height = 1.0;            // 起飞高度 (m) - 用于判断是否起飞完成
    
    // 视点选择参数（避免近距离大转向）
    double min_viewpoint_distance = 2.0;    // 视点最小距离 (m) - 小于此值考虑 yaw 惩罚
    double yaw_change_weight = 0.5;         // yaw 变化权重 (m/rad) - 转1rad相当于多飞多少米
    double large_yaw_threshold = 1.57;      // 大转向阈值 (rad) - 超过90°视为大转向
    double large_yaw_penalty = 3.0;         // 近距离大转向额外惩罚 (m)
    
    // 可视化参数
    bool enable_visualization = true;
    double viewpoint_sphere_radius = 0.2;
    double frustum_length = 2.0;
    double frustum_fov_h = 1.57;            // 90°
    double frustum_fov_v = 1.22;            // 70°
};

// ==================== 探索状态机主类 ====================
class ExplorationFSM {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ExplorationFSM> Ptr;
    
    ExplorationFSM() = default;
    ~ExplorationFSM();
    
    /**
     * @brief 初始化状态机
     * @param nh 节点句柄
     * @param map_manager 地图管理器（替代旧的 ROGMap）
     */
    void init(ros::NodeHandle& nh, MapManager::Ptr map_manager);
    
    /**
     * @brief 获取当前状态
     */
    ExplorationState getState() const { return state_; }
    std::string getStateString() const;
    
    /**
     * @brief 获取探索数据
     */
    const ExplorationData& getExplorationData() const { return expl_data_; }
    
    /**
     * @brief 手动触发探索
     */
    void triggerExploration();
    
    /**
     * @brief 停止探索
     */
    void stopExploration();
    
    /**
     * @brief 获取下一个目标视点
     */
    bool getNextViewpoint(ExplorationViewpoint& viewpoint);
    
    /**
     * @brief 检查是否有前沿需要探索
     */
    bool hasFrontier() const { return has_frontier_; }

private:
    // ==================== 状态转换 ====================
    void transitState(ExplorationState new_state, const std::string& reason);
    
    // ==================== 回调函数 ====================
    void fsmCallback(const ros::TimerEvent& e);
    void frontierCallback(const ros::TimerEvent& e);
    void safetyCallback(const ros::TimerEvent& e);
    void visualizationCallback(const ros::TimerEvent& e);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    
    
    // ==================== 状态处理函数 ====================
    void processInit();
    void processWaitTrigger();
    void processUpdateFrontier();
    void processPlanViewpoints();
    void processExecExploration();
    void processFinish();
    void processEmergencyStop();
    
    // ==================== 前沿处理 ====================
    bool updateFrontierInfo();
    bool checkFrontierCovered();
    int countTotalFrontiers() const;
    
    // ==================== 视点规划 ====================
    bool planExplorationViewpoints();
    bool selectNextViewpoint();
    
    /**
     * @brief 贪心排序视点（参考 EDEN 算法）
     * 
     * 从当前无人机位置开始，每次选择距离最近的未访问视点
     * 这样可以减少飞行距离，提高探索效率
     */
    void sortViewpointsByGreedy();
    
    // ==================== 可视化 ====================
    void visualize();
    void visualizeFrontierClusters();
    void visualizeViewpoints();
    void visualizeFrustums();
    void visualizeRegionRoute();
    void visualizeExplorationInfo();
    void visualizeHistoryNodes();
    void clearVisualization();
    
    // ==================== 辅助函数 ====================
    // 发布视点目标，如果无法找到有效路径返回 false
    bool publishViewpointTarget(const ExplorationViewpoint& vp);
    bool isNearViewpoint(const ExplorationViewpoint& vp);
    double getYawDiff(double yaw1, double yaw2);
    
    // 创建视锥体可视化
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
    ros::Timer frontier_timer_;
    ros::Timer safety_timer_;
    ros::Timer vis_timer_;
    
    ros::Subscriber odom_sub_;
    ros::Subscriber trigger_sub_;
    
    ros::Publisher viewpoint_pub_;           // 发布目标视点给EGO-Planner
    ros::Publisher viewpoint_path_pub_;      // 发布带中间航点的路径给EGO-Planner
    ros::Publisher frontier_cloud_pub_;      // 前沿点云
    ros::Publisher frontier_cluster_pub_;    // 前沿簇可视化
    ros::Publisher viewpoint_marker_pub_;    // 视点可视化
    ros::Publisher frustum_marker_pub_;      // 视锥体可视化
    ros::Publisher route_marker_pub_;        // 区域路由可视化
    ros::Publisher info_marker_pub_;         // 信息文本可视化
    ros::Publisher exploration_state_pub_;   // 探索状态
    
    // 地图模块
    MapManager::Ptr map_manager_;
    GlobalMap::Ptr global_map_;  // 从 map_manager_ 获取
    FrontierViewpointGenerator::Ptr viewpoint_generator_;
    
    // 状态
    ExplorationState state_;
    ExplorationFSMConfig config_;
    ExplorationData expl_data_;
    
    // 里程计
    Eigen::Vector3d odom_pos_;
    Eigen::Vector3d odom_vel_;
    double odom_yaw_;
    bool have_odom_;
    
    // 标志
    std::atomic<bool> triggered_;
    std::atomic<bool> has_frontier_;
    std::atomic<bool> frontier_ready_;
    std::atomic<bool> viewpoint_reached_;
    
    // 时间戳
    ros::Time fsm_init_time_;
    ros::Time last_replan_time_;
    ros::Time last_frontier_update_time_;
    ros::Time traj_start_time_;              // 轨迹开始执行时间（用于 Falcon 风格重规划）
    double traj_duration_ = 0.0;             // 当前轨迹预期执行时间（秒）
    int initial_unknown_count_ = 0;          // 初始未知体素数量（用于覆盖比例计算）
    
    // 前沿变化检测（用于避免无变化时重复计算视点）
    std::vector<int> last_frontier_subgrids_;
    int last_total_frontiers_ = 0;
    bool frontier_changed_ = false;
    
    // 前沿簇缓存（直接从共享的GlobalMap获取）
    std::vector<FrontierCluster> frontier_clusters_;
    
    // 互斥锁
    mutable std::mutex state_mutex_;
    mutable std::mutex odom_mutex_;
    mutable std::mutex data_mutex_;
    
    // 状态字符串
    std::vector<std::string> state_strings_ = {
        "INIT",
        "WAIT_TRIGGER", 
        "UPDATE_FRONTIER",
        "PLAN_VIEWPOINTS",
        "EXEC_EXPLORATION",
        "FINISH",
        "EMERGENCY_STOP"
    };
};

} // namespace rog_map

#endif // EXPLORATION_FSM_H

