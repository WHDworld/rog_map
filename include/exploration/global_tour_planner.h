/**
 * @file global_tour_planner.h
 * @brief FUEL 风格全局调度：效用归一化 + ATSP（LKH）+ 局部精化 + 路径生成
 *
 * 职责：
 * 1. 将前沿/语义任务统一为 ExplorationTask，并计算 utility
 * 2. 构造 ATSP 代价矩阵（节点 0 = 机器人，任务间为 FUEL 运动时间）
 * 3. 调用 LKH 求解；失败时使用 utility 感知的最近邻 + 有向 2-opt
 * 4. 对前若干任务做分层视点精化（top-K 视点上的最短路）
 * 5. 用 GridPathFinder 生成机器人到首个精化视点的已知自由路径
 */

#ifndef SEMANTIC_EXPLORATION_GLOBAL_TOUR_PLANNER_H
#define SEMANTIC_EXPLORATION_GLOBAL_TOUR_PLANNER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "exploration/exploration_task.h"
#include "exploration/grid_path_finder.h"
#include "map/global_map.h"

namespace rog_map {

struct RobotPlanningState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    bool valid = false;
};

struct ScheduledGoal {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ExplorationTaskKey task_key;
    Eigen::Vector3d viewpoint = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double view_score = 0.0;
    std::vector<Eigen::Vector3d> path;
};

struct GlobalTourPlan {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct Timings {
        double matrix_ms = 0.0;
        double atsp_ms = 0.0;
        double refine_ms = 0.0;
        double path_ms = 0.0;
        double total_ms = 0.0;
    };

    bool success = false;
    bool used_fallback = false;
    std::string failure_reason;
    std::vector<ExplorationTaskKey> task_order;
    std::vector<Eigen::Vector3d> refined_viewpoints;
    std::vector<Eigen::Vector3d> global_tour_path;
    ScheduledGoal next_goal;
    Timings timings;
};

class GlobalTourPlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<GlobalTourPlanner>;

    GlobalTourPlanner() = default;
    ~GlobalTourPlanner() = default;

    /**
     * @brief 初始化，加载 global_scheduler/* 与 manager/* 参数
     */
    void init(ros::NodeHandle& nh, star_mvf::GlobalMap::Ptr global_map);

    /**
     * @brief 求解全局巡游并返回下一个目标
     * @param robot 机器人状态
     * @param tasks 统一任务（调用方已过滤黑名单与无候选任务）
     * @param previous_goal 上一执行目标（用于首跳滞后奖励）
     */
    GlobalTourPlan plan(const RobotPlanningState& robot,
                        const std::vector<ExplorationTask>& tasks,
                        const std::optional<ExplorationTaskKey>& previous_goal);

    /**
     * @brief 使某任务的缓存路径失效（EGO 失败、路径碰撞等）
     */
    void invalidateTask(const ExplorationTaskKey& key);

    /**
     * @brief 清空缓存与当前目标状态
     */
    void clear();

    const GridPathFinder& pathFinder() const { return path_finder_; }
    bool visualizationEnabled() const { return config_.enable_visualization; }

private:
    // 单元测试可访问代价模型/LKH/精化的内部实现
    friend class GlobalTourPlannerTest;

    struct PlannerConfig {
        int max_tasks = 30;
        bool refine_local = true;
        int refined_num = 5;
        double refined_radius = 5.0;
        int top_view_num = 5;
        double max_decay = 0.8;

        double gain_weight = 0.70;
        double semantic_weight = 0.25;
        double viewpoint_weight = 0.05;

        double utility_penalty = 5.0;
        double hysteresis_bonus = 1.0;
        double min_switch_interval = 0.5;
        double direction_weight = 1.5;

        int cost_scale = 100;
        int invalid_cost = 1000000;
        double solver_time_limit = 0.1;

        bool unknown_as_occupied = true;
        int path_max_expansions = 200000;
        double safety_margin = 0.5;
        // Cruise-altitude floor for grid paths [m]; 0 disables the floor.
        double min_flight_height = 0.0;
        bool enable_visualization = true;

        double max_vel = 2.0;
        double max_yaw_rate = 0.25;
    };

    struct CachedGoal {
        Eigen::Vector3d start = Eigen::Vector3d::Zero();
        Eigen::Vector3d viewpoint = Eigen::Vector3d::Zero();
        double yaw = 0.0;
        double view_score = 0.0;
        std::vector<Eigen::Vector3d> path;
    };

    struct TaskEdgeKey {
        ExplorationTaskKey from;
        ExplorationTaskKey to;

        bool operator==(const TaskEdgeKey& other) const {
            return from == other.from && to == other.to;
        }
    };

    struct TaskEdgeKeyHash {
        size_t operator()(const TaskEdgeKey& key) const {
            const ExplorationTaskKeyHash hash;
            const size_t h1 = hash(key.from);
            const size_t h2 = hash(key.to);
            return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
        }
    };

    struct CachedTaskEdge {
        Eigen::Vector3d from_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d to_position = Eigen::Vector3d::Zero();
        double from_yaw = 0.0;
        double to_yaw = 0.0;
        double motion_cost = 0.0;
        std::vector<Eigen::Vector3d> path;
    };

    static double wrapDiff(double a, double b);

    /**
     * @brief FUEL 运动时间：T = max(L/v + w_dir*|Δyaw|, |Δyaw|/yaw_rate)
     */
    double motionTime(const Eigen::Vector3d& from, double from_yaw,
                      const Eigen::Vector3d& to, double to_yaw) const;

    double pathAwareMotionTime(const Eigen::Vector3d& from, double from_yaw,
                               const Eigen::Vector3d& to, double to_yaw,
                               const Eigen::Vector3d& initial_velocity,
                               std::vector<Eigen::Vector3d>* path = nullptr);

    double cachedTaskMotionTime(const ExplorationTask& from,
                                const ExplorationTask& to,
                                size_t& reused_edges,
                                size_t& recomputed_edges);

    /**
     * @brief 归一化增益、语义价值、视点质量并计算 utility
     */
    void normalizeUtilities(std::vector<ExplorationTask>& tasks) const;

    /**
     * @brief 构造 ATSP 代价矩阵（含首跳效用惩罚与滞后奖励）
     */
    void buildCostMatrix(const RobotPlanningState& robot,
                         const std::vector<ExplorationTask>& tasks,
                         const std::optional<ExplorationTaskKey>& previous_goal,
                         std::vector<std::vector<double>>& matrix);

    /**
     * @brief 调用 LKH 求解 ATSP，并校验返回的节点序列
     */
    bool runLkh(const std::vector<std::vector<double>>& matrix,
                std::vector<int>& order, std::string& reason);

    /**
     * @brief utility 感知首跳 + 最近邻 + 有向 2-opt
     */
    std::vector<int> runFallback(const std::vector<std::vector<double>>& matrix) const;

    /**
     * @brief 前 refined_num 个任务（或累计距离 <= refined_radius）上的分层精化
     * @return 每个窗口任务选中的视点下标（与 refined_task_indices 对齐）
     */
    bool refineLocal(const RobotPlanningState& robot,
                     const std::vector<ExplorationTask>& tasks,
                     const std::vector<int>& order,
                     const std::optional<ExplorationTaskKey>& previous_goal,
                     std::vector<int>& refined_task_indices,
                     std::vector<int>& refined_viewpoint_indices);

    /**
     * @brief 生成机器人到目标视点的已知自由路径（带缓存与重验证）
     */
    bool planNextGoalPath(const RobotPlanningState& robot,
                          const ExplorationTask& task,
                          const ExplorationViewpoint& viewpoint,
                          std::vector<Eigen::Vector3d>& path,
                          std::string& reason);

    PlannerConfig config_;
    star_mvf::GlobalMap::Ptr global_map_;
    GridPathFinder path_finder_;
    std::string lkh_dir_;

    std::mutex plan_mutex_;
    std::unordered_map<ExplorationTaskKey, CachedGoal, ExplorationTaskKeyHash> path_cache_;
    std::unordered_map<TaskEdgeKey, CachedTaskEdge, TaskEdgeKeyHash> task_edge_cache_;
    std::optional<ExplorationTaskKey> current_goal_key_;
    ros::Time last_switch_time_;
    bool initialized_ = false;
};

}  // namespace rog_map

#endif  // SEMANTIC_EXPLORATION_GLOBAL_TOUR_PLANNER_H
