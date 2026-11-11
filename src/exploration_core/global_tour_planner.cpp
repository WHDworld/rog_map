/**
 * @file global_tour_planner.cpp
 * @brief FUEL 风格全局调度实现
 */

#include "exploration/global_tour_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numeric>
#include <unordered_set>
#include <vector>

#include <unistd.h>

#include <lkh_tsp_solver/lkh_interface.h>

namespace rog_map {

namespace {

constexpr double kEps = 1e-9;
constexpr double kViewpointReuseDist = 0.3;

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double pathLength(const std::vector<Eigen::Vector3d>& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}

}  // namespace

// ==================== 初始化 ====================

void GlobalTourPlanner::init(ros::NodeHandle& nh, star_mvf::GlobalMap::Ptr global_map) {
    global_map_ = std::move(global_map);

    nh.param("global_scheduler/max_tasks", config_.max_tasks, 30);
    nh.param("global_scheduler/refine_local", config_.refine_local, true);
    nh.param("global_scheduler/refined_num", config_.refined_num, 5);
    nh.param("global_scheduler/refined_radius", config_.refined_radius, 5.0);
    nh.param("global_scheduler/top_view_num", config_.top_view_num, 5);
    nh.param("global_scheduler/max_decay", config_.max_decay, 0.8);

    nh.param("global_scheduler/gain_weight", config_.gain_weight, 0.70);
    nh.param("global_scheduler/semantic_weight", config_.semantic_weight, 0.25);
    nh.param("global_scheduler/viewpoint_weight", config_.viewpoint_weight, 0.05);

    nh.param("global_scheduler/utility_penalty", config_.utility_penalty, 5.0);
    nh.param("global_scheduler/hysteresis_bonus", config_.hysteresis_bonus, 1.0);
    nh.param("global_scheduler/min_switch_interval", config_.min_switch_interval, 0.5);
    nh.param("global_scheduler/direction_weight", config_.direction_weight, 1.5);

    nh.param("global_scheduler/cost_scale", config_.cost_scale, 100);
    nh.param("global_scheduler/invalid_cost", config_.invalid_cost, 1000000);
    nh.param("global_scheduler/solver_time_limit", config_.solver_time_limit, 0.1);

    nh.param("global_scheduler/unknown_as_occupied", config_.unknown_as_occupied, true);
    nh.param("global_scheduler/path_max_expansions", config_.path_max_expansions, 200000);
    nh.param("global_scheduler/safety_margin", config_.safety_margin, 0.5);
    nh.param("global_scheduler/min_flight_height", config_.min_flight_height, 0.0);
    nh.param("global_scheduler/enable_visualization", config_.enable_visualization, true);

    nh.param("manager/max_vel", config_.max_vel, 2.0);
    nh.param("manager/max_yaw_rate", config_.max_yaw_rate, 0.25);

    config_.max_tasks = std::max(1, config_.max_tasks);
    config_.refined_num = std::max(1, config_.refined_num);
    config_.top_view_num = std::max(1, config_.top_view_num);
    config_.cost_scale = std::max(1, config_.cost_scale);
    config_.invalid_cost = std::max(config_.cost_scale, config_.invalid_cost);
    config_.max_vel = std::max(0.1, config_.max_vel);
    config_.max_yaw_rate = std::max(0.01, config_.max_yaw_rate);

    path_finder_.init(global_map_, config_.safety_margin, config_.min_flight_height);

    lkh_dir_ = "/tmp/semantic_exploration_lkh_" + std::to_string(::getpid());
    const std::string command = "mkdir -p '" + lkh_dir_ + "'";
    if (std::system(command.c_str()) != 0) {
        ROS_WARN("[SCHED][ATSP] Failed to create LKH temp dir %s", lkh_dir_.c_str());
    }

    initialized_ = global_map_ != nullptr;
    ROS_INFO("[SCHED][SNAPSHOT] GlobalTourPlanner initialized: max_tasks=%d refined=%d "
             "scale=%d time_limit=%.3f dir=%s",
             config_.max_tasks, config_.refined_num, config_.cost_scale,
             config_.solver_time_limit, lkh_dir_.c_str());
}

// ==================== 工具函数 ====================

double GlobalTourPlanner::wrapDiff(double a, double b) {
    double diff = a - b;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

double GlobalTourPlanner::motionTime(const Eigen::Vector3d& from, double from_yaw,
                                     const Eigen::Vector3d& to, double to_yaw) const {
    const double dist = (to - from).norm();
    const double dyaw = std::abs(wrapDiff(to_yaw, from_yaw));
    const double t_pos = dist / config_.max_vel;
    const double t_yaw = dyaw / config_.max_yaw_rate;
    return std::max(t_pos, t_yaw);
}

double GlobalTourPlanner::pathAwareMotionTime(
    const Eigen::Vector3d& from, double from_yaw, const Eigen::Vector3d& to,
    double to_yaw, const Eigen::Vector3d& initial_velocity,
    std::vector<Eigen::Vector3d>* path) {
    const UnknownSpacePolicy policy = config_.unknown_as_occupied
                                          ? UnknownSpacePolicy::BLOCK
                                          : UnknownSpacePolicy::ALLOW;
    const GridPathResult result =
        path_finder_.search(from, to, policy, config_.path_max_expansions);
    if (!result.success) {
        return static_cast<double>(config_.invalid_cost) / config_.cost_scale;
    }
    if (path) *path = result.points;

    double direction_penalty = 0.0;
    if (initial_velocity.norm() > 1e-3 && result.points.size() >= 2) {
        const Eigen::Vector3d first_direction = result.points[1] - result.points[0];
        if (first_direction.norm() > 1e-3) {
            const double cosine = std::max(-1.0, std::min(
                1.0, initial_velocity.normalized().dot(first_direction.normalized())));
            direction_penalty = config_.direction_weight * std::acos(cosine);
        }
    }
    const double dyaw = std::abs(wrapDiff(to_yaw, from_yaw));
    return std::max(result.length / config_.max_vel + direction_penalty,
                    dyaw / config_.max_yaw_rate);
}

double GlobalTourPlanner::cachedTaskMotionTime(
    const ExplorationTask& from, const ExplorationTask& to,
    size_t& reused_edges, size_t& recomputed_edges) {
    const ExplorationViewpoint& from_view = from.viewpoints.front();
    const ExplorationViewpoint& to_view = to.viewpoints.front();
    const TaskEdgeKey key{from.key, to.key};
    auto cached = task_edge_cache_.find(key);
    if (cached != task_edge_cache_.end()) {
        const CachedTaskEdge& edge = cached->second;
        const bool same_geometry =
            (edge.from_position - from_view.position).norm() <= kViewpointReuseDist &&
            (edge.to_position - to_view.position).norm() <= kViewpointReuseDist &&
            std::abs(wrapDiff(edge.from_yaw, from_view.yaw)) <= 0.15 &&
            std::abs(wrapDiff(edge.to_yaw, to_view.yaw)) <= 0.15;
        if (same_geometry) {
            ++reused_edges;
            return edge.motion_cost;
        }
        task_edge_cache_.erase(cached);
    }

    CachedTaskEdge edge;
    edge.from_position = from_view.position;
    edge.to_position = to_view.position;
    edge.from_yaw = from_view.yaw;
    edge.to_yaw = to_view.yaw;
    edge.motion_cost = pathAwareMotionTime(
        from_view.position, from_view.yaw, to_view.position, to_view.yaw,
        Eigen::Vector3d::Zero(), &edge.path);
    task_edge_cache_[key] = edge;

    // With zero task-to-task initial velocity the FUEL cost is symmetric.
    // Store the reversed path at the same time, as FrontierFinder::costs_/paths_
    // does, so the initial matrix needs only N(N-1)/2 searches.
    CachedTaskEdge reverse_edge;
    reverse_edge.from_position = to_view.position;
    reverse_edge.to_position = from_view.position;
    reverse_edge.from_yaw = to_view.yaw;
    reverse_edge.to_yaw = from_view.yaw;
    reverse_edge.motion_cost = edge.motion_cost;
    reverse_edge.path = edge.path;
    std::reverse(reverse_edge.path.begin(), reverse_edge.path.end());
    task_edge_cache_[TaskEdgeKey{to.key, from.key}] = std::move(reverse_edge);
    ++recomputed_edges;
    return edge.motion_cost;
}

// ==================== 效用归一化 ====================

void GlobalTourPlanner::normalizeUtilities(std::vector<ExplorationTask>& tasks) const {
    if (tasks.empty()) return;

    std::vector<double> log_gains(tasks.size(), 0.0);
    double max_log_gain = 0.0;
    for (size_t i = 0; i < tasks.size(); ++i) {
        const double gain = std::max(0.0, tasks[i].raw_gain);
        log_gains[i] = std::log1p(gain);
        max_log_gain = std::max(max_log_gain, log_gains[i]);
    }

    for (size_t i = 0; i < tasks.size(); ++i) {
        auto& task = tasks[i];
        const double normalized_gain =
            max_log_gain > kEps ? log_gains[i] / (max_log_gain + kEps) : 0.0;
        const double semantic_value =
            task.key.type == ExplorationTaskType::SEMANTIC
                ? clamp01(task.semantic_value)
                : 0.0;
        const double viewpoint_quality =
            task.viewpoints.empty() ? 0.0 : clamp01(task.viewpoints.front().view_score);

        task.normalized_gain = normalized_gain;
        task.semantic_value = semantic_value;
        task.utility = clamp01(config_.gain_weight * normalized_gain +
                               config_.semantic_weight * semantic_value +
                               config_.viewpoint_weight * viewpoint_quality);
    }
}

// ==================== 代价矩阵 ====================

void GlobalTourPlanner::buildCostMatrix(
    const RobotPlanningState& robot, const std::vector<ExplorationTask>& tasks,
    const std::optional<ExplorationTaskKey>& previous_goal,
    std::vector<std::vector<double>>& matrix) {
    const int task_count = static_cast<int>(tasks.size());
    const int dimension = task_count + 1;
    matrix.assign(dimension, std::vector<double>(dimension, 0.0));

    // FUEL-style incremental matrix maintenance: retain old↔old edges, discard
    // edges whose task disappeared, and compute only new/moved task edges.
    std::unordered_set<ExplorationTaskKey, ExplorationTaskKeyHash> active_keys;
    active_keys.reserve(tasks.size());
    for (const auto& task : tasks) active_keys.insert(task.key);
    size_t invalidated_edges = 0;
    for (auto it = task_edge_cache_.begin(); it != task_edge_cache_.end();) {
        if (active_keys.count(it->first.from) == 0 ||
            active_keys.count(it->first.to) == 0) {
            it = task_edge_cache_.erase(it);
            ++invalidated_edges;
        } else {
            ++it;
        }
    }
    size_t reused_edges = 0;
    size_t recomputed_edges = 0;

    // 节点 0 = 机器人：首跳 = 运动时间 + utility 惩罚 - 滞后奖励
    for (int j = 0; j < task_count; ++j) {
        const auto& task = tasks[j];
        const auto& viewpoint = task.viewpoints.front();
        double cost = pathAwareMotionTime(robot.position, robot.yaw, viewpoint.position,
                                          viewpoint.yaw, robot.velocity);
        cost += config_.utility_penalty * (1.0 - task.utility);
        if (previous_goal.has_value() && task.key == *previous_goal) {
            cost -= config_.hysteresis_bonus;
        }
        matrix[0][j + 1] = std::max(0.0, cost);
        matrix[j + 1][0] = 0.0;  // 任务返回机器人无代价
    }

    for (int i = 0; i < task_count; ++i) {
        for (int j = 0; j < task_count; ++j) {
            if (i == j) continue;
            matrix[i + 1][j + 1] = cachedTaskMotionTime(
                tasks[i], tasks[j], reused_edges, recomputed_edges);
        }
    }
    ROS_INFO("[SCHED][CACHE] task_edges=%zu reused=%zu recomputed=%zu invalidated=%zu",
             task_edge_cache_.size(), reused_edges, recomputed_edges, invalidated_edges);
}

// ==================== LKH ====================

bool GlobalTourPlanner::runLkh(const std::vector<std::vector<double>>& matrix,
                               std::vector<int>& order, std::string& reason) {
    const int dimension = static_cast<int>(matrix.size());
    if (dimension < 2) {
        reason = "dimension too small";
        return false;
    }

    const std::string tsp_path = lkh_dir_ + "/single.tsp";
    const std::string par_path = lkh_dir_ + "/single.par";
    const std::string tour_path = lkh_dir_ + "/single.txt";

    std::ofstream tsp_file(tsp_path);
    if (!tsp_file.is_open()) {
        reason = "cannot open tsp file";
        return false;
    }
    tsp_file << "NAME : single\n"
             << "TYPE : ATSP\n"
             << "DIMENSION : " << dimension << "\n"
             << "EDGE_WEIGHT_TYPE : EXPLICIT\n"
             << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
             << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
            int cost = 0;
            if (i == j) {
                cost = config_.invalid_cost;
            } else {
                const double scaled = matrix[i][j] * config_.cost_scale;
                cost = static_cast<int>(std::llround(scaled));
                cost = std::max(0, std::min(cost, config_.invalid_cost));
            }
            tsp_file << cost << ' ';
        }
        tsp_file << '\n';
    }
    tsp_file << "EOF\n";
    tsp_file.close();

    std::ofstream par_file(par_path);
    if (!par_file.is_open()) {
        reason = "cannot open par file";
        return false;
    }
    par_file << "PROBLEM_FILE = " << tsp_path << "\n"
             << "OUTPUT_TOUR_FILE = " << tour_path << "\n"
             << "RUNS = 1\n"
             << "GAIN23 = NO\n"
             << "TRACE_LEVEL = 0\n";
    if (config_.solver_time_limit > 0.0) {
        par_file << "TIME_LIMIT = " << config_.solver_time_limit << "\n";
    }
    par_file.close();

    std::remove(tour_path.c_str());

    // LKH 使用大量全局变量，必须全局串行调用
    static std::mutex lkh_mutex;
    {
        std::lock_guard<std::mutex> lock(lkh_mutex);
        solveTSPLKH(par_path.c_str());
    }

    std::ifstream tour_file(tour_path);
    if (!tour_file.is_open()) {
        reason = "tour file missing";
        return false;
    }

    std::string line;
    bool found_section = false;
    while (std::getline(tour_file, line)) {
        if (line == "TOUR_SECTION") {
            found_section = true;
            break;
        }
    }
    if (!found_section) {
        reason = "TOUR_SECTION missing";
        return false;
    }

    std::vector<int> nodes;
    while (std::getline(tour_file, line)) {
        if (line.empty()) continue;
        const int id = std::atoi(line.c_str());
        if (id == -1) break;
        nodes.push_back(id);
    }
    tour_file.close();

    if (static_cast<int>(nodes.size()) != dimension) {
        reason = "unexpected node count";
        return false;
    }

    std::vector<uint8_t> seen(dimension + 1, 0);
    for (int id : nodes) {
        if (id < 1 || id > dimension || seen[id]) {
            reason = "duplicate or out-of-range node";
            return false;
        }
        seen[id] = 1;
    }

    const auto robot_it = std::find(nodes.begin(), nodes.end(), 1);
    if (robot_it == nodes.end()) {
        reason = "robot node missing";
        return false;
    }
    std::rotate(nodes.begin(), robot_it, nodes.end());
    if (nodes.front() != 1) {
        reason = "robot node not first after rotation";
        return false;
    }

    order.clear();
    order.reserve(dimension - 1);
    std::vector<uint8_t> task_seen(dimension - 1, 0);
    for (size_t i = 1; i < nodes.size(); ++i) {
        const int task_index = nodes[i] - 2;
        if (task_index < 0 || task_index >= dimension - 1 || task_seen[task_index]) {
            reason = "invalid task index";
            order.clear();
            return false;
        }
        task_seen[task_index] = 1;
        order.push_back(task_index);
    }

    if (static_cast<int>(order.size()) != dimension - 1) {
        reason = "task count mismatch";
        order.clear();
        return false;
    }
    return true;
}

// ==================== 回退：最近邻 + 有向 2-opt ====================

std::vector<int> GlobalTourPlanner::runFallback(
    const std::vector<std::vector<double>>& matrix) const {
    const int task_count = static_cast<int>(matrix.size()) - 1;
    std::vector<int> order;
    order.reserve(task_count);
    std::vector<uint8_t> used(task_count, 0);

    // utility 感知首跳：C(0, j) 已包含效用惩罚与滞后奖励
    int first = -1;
    double best = std::numeric_limits<double>::max();
    for (int j = 0; j < task_count; ++j) {
        if (matrix[0][j + 1] < best) {
            best = matrix[0][j + 1];
            first = j;
        }
    }
    if (first < 0) return order;
    order.push_back(first);
    used[first] = 1;

    while (static_cast<int>(order.size()) < task_count) {
        const int current = order.back() + 1;
        int next = -1;
        best = std::numeric_limits<double>::max();
        for (int j = 0; j < task_count; ++j) {
            if (used[j]) continue;
            if (matrix[current][j + 1] < best) {
                best = matrix[current][j + 1];
                next = j;
            }
        }
        if (next < 0) break;
        order.push_back(next);
        used[next] = 1;
    }

    auto tourCost = [&matrix](const std::vector<int>& tour) {
        if (tour.empty()) return 0.0;
        double cost = matrix[0][tour.front() + 1];
        for (size_t i = 1; i < tour.size(); ++i) {
            cost += matrix[tour[i - 1] + 1][tour[i] + 1];
        }
        return cost;
    };

    // 有向 2-opt：反转子段并按方向重新计算代价
    if (order.size() >= 3) {
        double current_cost = tourCost(order);
        for (int iteration = 0; iteration < 10; ++iteration) {
            bool improved = false;
            for (size_t i = 0; i + 1 < order.size(); ++i) {
                for (size_t j = i + 1; j < order.size(); ++j) {
                    std::vector<int> candidate = order;
                    std::reverse(candidate.begin() + i, candidate.begin() + j + 1);
                    const double candidate_cost = tourCost(candidate);
                    if (candidate_cost + kEps < current_cost) {
                        order = std::move(candidate);
                        current_cost = candidate_cost;
                        improved = true;
                    }
                }
            }
            if (!improved) break;
        }
    }

    return order;
}

// ==================== 局部精化 ====================

bool GlobalTourPlanner::refineLocal(
    const RobotPlanningState& robot, const std::vector<ExplorationTask>& tasks,
    const std::vector<int>& order,
    const std::optional<ExplorationTaskKey>& previous_goal,
    std::vector<int>& refined_task_indices, std::vector<int>& refined_viewpoint_indices) {
    refined_task_indices.clear();
    refined_viewpoint_indices.clear();
    if (order.empty()) return false;

    int window = config_.refine_local ? std::min<int>(config_.refined_num, order.size()) : 1;

    // 累计几何距离超过 refined_radius 时截断窗口（至少保留 2 个任务）
    double cumulative = 0.0;
    Eigen::Vector3d previous_position = robot.position;
    int allowed = 0;
    for (int i = 0; i < window; ++i) {
        const auto& task = tasks[order[i]];
        if (task.viewpoints.empty()) break;
        cumulative += (task.viewpoints.front().position - previous_position).norm();
        previous_position = task.viewpoints.front().position;
        ++allowed;
        if (i >= 1 && cumulative > config_.refined_radius) break;
    }
    window = std::max(1, allowed);

    // 若窗口内某层没有候选视点，截断到该层之前
    for (int i = 0; i < window; ++i) {
        if (tasks[order[i]].viewpoints.empty()) {
            window = i;
            break;
        }
    }
    if (window <= 0 || tasks[order[0]].viewpoints.empty()) {
        return false;
    }

    std::vector<std::vector<double>> layer_cost(window);
    std::vector<std::vector<int>> back_pointer(window);

    const auto& first_task = tasks[order[0]];
    layer_cost[0].assign(first_task.viewpoints.size(), 0.0);
    back_pointer[0].assign(first_task.viewpoints.size(), -1);
    for (size_t j = 0; j < first_task.viewpoints.size(); ++j) {
        const auto& viewpoint = first_task.viewpoints[j];
        double cost = pathAwareMotionTime(robot.position, robot.yaw, viewpoint.position,
                                          viewpoint.yaw, robot.velocity);
        cost += config_.utility_penalty * (1.0 - first_task.utility);
        if (previous_goal.has_value() && first_task.key == *previous_goal) {
            cost -= config_.hysteresis_bonus;
        }
        layer_cost[0][j] = std::max(0.0, cost);
    }

    for (int layer = 1; layer < window; ++layer) {
        const auto& previous_task = tasks[order[layer - 1]];
        const auto& current_task = tasks[order[layer]];
        layer_cost[layer].assign(current_task.viewpoints.size(),
                                 std::numeric_limits<double>::max());
        back_pointer[layer].assign(current_task.viewpoints.size(), -1);

        for (size_t j = 0; j < current_task.viewpoints.size(); ++j) {
            const auto& to = current_task.viewpoints[j];
            for (size_t k = 0; k < previous_task.viewpoints.size(); ++k) {
                const auto& from = previous_task.viewpoints[k];
                const double edge = pathAwareMotionTime(
                    from.position, from.yaw, to.position, to.yaw, Eigen::Vector3d::Zero());
                const double cost = layer_cost[layer - 1][k] + edge;
                if (cost < layer_cost[layer][j]) {
                    layer_cost[layer][j] = cost;
                    back_pointer[layer][j] = static_cast<int>(k);
                }
            }
        }
    }

    // 回溯最优路径
    const auto& last_layer_cost = layer_cost.back();
    const auto best_it = std::min_element(last_layer_cost.begin(), last_layer_cost.end());
    if (best_it == last_layer_cost.end() ||
        *best_it >= std::numeric_limits<double>::max()) {
        return false;
    }

    refined_task_indices.resize(window);
    refined_viewpoint_indices.resize(window);
    int viewpoint_index = static_cast<int>(std::distance(last_layer_cost.begin(), best_it));
    for (int layer = window - 1; layer >= 0; --layer) {
        refined_task_indices[layer] = order[layer];
        refined_viewpoint_indices[layer] = viewpoint_index;
        if (layer > 0) viewpoint_index = back_pointer[layer][viewpoint_index];
        if (viewpoint_index < 0 && layer > 0) return false;
    }
    return true;
}

// ==================== 路径生成 ====================

bool GlobalTourPlanner::planNextGoalPath(const RobotPlanningState& robot,
                                         const ExplorationTask& task,
                                         const ExplorationViewpoint& viewpoint,
                                         std::vector<Eigen::Vector3d>& path,
                                         std::string& reason) {
    const UnknownSpacePolicy policy = config_.unknown_as_occupied
                                          ? UnknownSpacePolicy::BLOCK
                                          : UnknownSpacePolicy::ALLOW;

    auto cached = path_cache_.find(task.key);
    if (cached != path_cache_.end() &&
        (cached->second.start - robot.position).norm() <= kViewpointReuseDist &&
        (cached->second.viewpoint - viewpoint.position).norm() <= kViewpointReuseDist) {
        if (path_finder_.validatePath(cached->second.path, policy)) {
            path = cached->second.path;
            ROS_DEBUG("[SCHED][CACHE] hit task=%s#%d points=%zu",
                      taskTypeName(task.key.type), task.key.id, path.size());
            return true;
        }
        ROS_INFO("[SCHED][INVALIDATE] cached path invalid task=%s#%d",
                 taskTypeName(task.key.type), task.key.id);
        path_cache_.erase(cached);
    }

    const GridPathResult result =
        path_finder_.search(robot.position, viewpoint.position, policy,
                            config_.path_max_expansions);
    if (!result.success || result.points.empty()) {
        reason = "GridPathFinder failed";
        ROS_WARN("[SCHED][INVALIDATE] path search failed task=%s#%d goal=(%.2f,%.2f,%.2f)",
                 taskTypeName(task.key.type), task.key.id, viewpoint.position.x(),
                 viewpoint.position.y(), viewpoint.position.z());
        return false;
    }

    CachedGoal entry;
    entry.start = robot.position;
    entry.viewpoint = viewpoint.position;
    entry.yaw = viewpoint.yaw;
    entry.view_score = viewpoint.view_score;
    entry.path = result.points;
    path_cache_[task.key] = std::move(entry);
    path = result.points;
    ROS_DEBUG("[SCHED][CACHE] miss task=%s#%d points=%zu",
              taskTypeName(task.key.type), task.key.id, path.size());
    return true;
}

// ==================== 主入口 ====================

GlobalTourPlan GlobalTourPlanner::plan(const RobotPlanningState& robot,
                                       const std::vector<ExplorationTask>& tasks,
                                       const std::optional<ExplorationTaskKey>& previous_goal) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    const ros::Time t_start = ros::Time::now();

    GlobalTourPlan plan;
    if (!initialized_) {
        plan.failure_reason = "planner not initialized";
        return plan;
    }
    if (!robot.valid) {
        plan.failure_reason = "invalid robot state";
        return plan;
    }

    // 过滤无候选视点的任务
    std::vector<ExplorationTask> candidates;
    candidates.reserve(tasks.size());
    for (const auto& task : tasks) {
        if (!task.viewpoints.empty()) candidates.push_back(task);
    }
    if (candidates.empty()) {
        plan.failure_reason = "no tasks";
        plan.timings.total_ms = (ros::Time::now() - t_start).toSec() * 1000.0;
        ROS_INFO("[SCHED][GOAL] no tasks available");
        return plan;
    }

    normalizeUtilities(candidates);
    if (static_cast<int>(candidates.size()) > config_.max_tasks) {
        std::stable_sort(candidates.begin(), candidates.end(),
                         [](const ExplorationTask& a, const ExplorationTask& b) {
                             return a.utility > b.utility;
                         });
        candidates.resize(static_cast<size_t>(config_.max_tasks));
        normalizeUtilities(candidates);
    }

    std::string last_failure = "unknown";
    const int max_attempts = static_cast<int>(candidates.size());
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        if (candidates.empty()) break;

        // ---- 代价矩阵 ----
        const ros::Time t_matrix = ros::Time::now();
        std::vector<std::vector<double>> matrix;
        buildCostMatrix(robot, candidates, previous_goal, matrix);
        plan.timings.matrix_ms = (ros::Time::now() - t_matrix).toSec() * 1000.0;
        ROS_INFO("[SCHED][MATRIX] tasks=%zu nodes=%zu scale=%d matrix_ms=%.2f",
                 candidates.size(), matrix.size(), config_.cost_scale,
                 plan.timings.matrix_ms);

        // ---- ATSP ----
        const ros::Time t_atsp = ros::Time::now();
        std::vector<int> order;
        std::string lkh_reason;
        bool used_fallback = false;
        if (candidates.size() == 1) {
            order = {0};
        } else if (!runLkh(matrix, order, lkh_reason)) {
            used_fallback = true;
            order = runFallback(matrix);
            ROS_WARN("[SCHED][ATSP] LKH failed (%s), fallback order size=%zu",
                     lkh_reason.c_str(), order.size());
        }
        plan.timings.atsp_ms = (ros::Time::now() - t_atsp).toSec() * 1000.0;
        if (order.size() != candidates.size()) {
            last_failure = "ATSP returned incomplete tour";
            plan.used_fallback = true;
            break;
        }
        plan.used_fallback = used_fallback;
        ROS_INFO("[SCHED][ATSP] tasks=%zu fallback=%d atsp_ms=%.2f",
                 candidates.size(), used_fallback ? 1 : 0, plan.timings.atsp_ms);

        // ---- 首跳去抖：min_switch_interval 内保持当前目标 ----
        if (config_.min_switch_interval > 0.0 && current_goal_key_.has_value()) {
            const double elapsed = (ros::Time::now() - last_switch_time_).toSec();
            if (elapsed < config_.min_switch_interval) {
                int current_index = -1;
                for (size_t i = 0; i < candidates.size(); ++i) {
                    if (candidates[i].key == *current_goal_key_) {
                        current_index = static_cast<int>(i);
                        break;
                    }
                }
                if (current_index >= 0) {
                    const auto order_it = std::find(order.begin(), order.end(), current_index);
                    if (order_it != order.end() && order_it != order.begin()) {
                        std::rotate(order.begin(), order_it, order.end());
                        ROS_DEBUG("[SCHED][ATSP] debounce keeps task=%s#%d first",
                                  taskTypeName(candidates[current_index].key.type),
                                  candidates[current_index].key.id);
                    }
                }
            }
        }

        // ---- 局部精化 ----
        const ros::Time t_refine = ros::Time::now();
        std::vector<int> refined_task_indices;
        std::vector<int> refined_viewpoint_indices;
        const bool refine_ok = refineLocal(robot, candidates, order, previous_goal,
                                           refined_task_indices, refined_viewpoint_indices);
        plan.timings.refine_ms = (ros::Time::now() - t_refine).toSec() * 1000.0;
        if (!refine_ok || refined_task_indices.empty()) {
            last_failure = "local refinement failed";
            // 首个任务不可用：移除后重解
            ROS_WARN("[SCHED][INVALIDATE] refinement failed, removing task=%s#%d",
                     taskTypeName(candidates[order.front()].key.type),
                     candidates[order.front()].key.id);
            path_cache_.erase(candidates[order.front()].key);
            candidates.erase(candidates.begin() + order.front());
            continue;
        }
        ROS_INFO("[SCHED][REFINE] window=%zu refine_ms=%.2f",
                 refined_task_indices.size(), plan.timings.refine_ms);

        const int first_index = refined_task_indices.front();
        const ExplorationTask& first_task = candidates[first_index];
        const ExplorationViewpoint& first_viewpoint =
            first_task.viewpoints[refined_viewpoint_indices.front()];

        // ---- 机器人到首个视点的已知自由路径 ----
        const ros::Time t_path = ros::Time::now();
        std::vector<Eigen::Vector3d> path;
        std::string path_reason;
        const bool path_ok = planNextGoalPath(robot, first_task, first_viewpoint, path,
                                              path_reason);
        plan.timings.path_ms = (ros::Time::now() - t_path).toSec() * 1000.0;

        if (!path_ok) {
            last_failure = path_reason.empty() ? "path planning failed" : path_reason;
            // 持有 plan_mutex_，直接清缓存，避免调用 invalidateTask() 造成死锁
            path_cache_.erase(first_task.key);
            ROS_WARN("[SCHED][INVALIDATE] drop task=%s#%d and replan",
                     taskTypeName(first_task.key.type), first_task.key.id);
            candidates.erase(candidates.begin() + first_index);
            continue;
        }

        // ---- 输出 ----
        plan.success = true;
        plan.task_order.clear();
        plan.task_order.reserve(order.size());
        for (int index : order) plan.task_order.push_back(candidates[index].key);

        plan.refined_viewpoints.clear();
        plan.refined_viewpoints.reserve(refined_task_indices.size());
        for (size_t i = 0; i < refined_task_indices.size(); ++i) {
            const auto& task = candidates[refined_task_indices[i]];
            plan.refined_viewpoints.push_back(
                task.viewpoints[refined_viewpoint_indices[i]].position);
        }
        plan.global_tour_path = path;

        plan.next_goal.task_key = first_task.key;
        plan.next_goal.viewpoint = first_viewpoint.position;
        plan.next_goal.yaw = first_viewpoint.yaw;
        plan.next_goal.view_score = first_viewpoint.view_score;
        plan.next_goal.path = path;

        if (!current_goal_key_.has_value() || !(*current_goal_key_ == first_task.key)) {
            current_goal_key_ = first_task.key;
            last_switch_time_ = ros::Time::now();
        }

        plan.timings.total_ms = (ros::Time::now() - t_start).toSec() * 1000.0;
        ROS_INFO("[SCHED][GOAL] task=%s#%d goal=(%.2f,%.2f,%.2f) yaw=%.2f path_points=%zu "
                 "path_len=%.2f total_ms=%.2f",
                 taskTypeName(first_task.key.type), first_task.key.id,
                 first_viewpoint.position.x(), first_viewpoint.position.y(),
                 first_viewpoint.position.z(), first_viewpoint.yaw, path.size(),
                 pathLength(path), plan.timings.total_ms);
        return plan;
    }

    plan.success = false;
    plan.failure_reason = last_failure;
    plan.timings.total_ms = (ros::Time::now() - t_start).toSec() * 1000.0;
    ROS_WARN("[SCHED][GOAL] planning failed: %s", last_failure.c_str());
    return plan;
}

// ==================== 缓存控制 ====================

void GlobalTourPlanner::invalidateTask(const ExplorationTaskKey& key) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    const size_t removed = path_cache_.erase(key);
    size_t removed_edges = 0;
    for (auto it = task_edge_cache_.begin(); it != task_edge_cache_.end();) {
        if (it->first.from == key || it->first.to == key) {
            it = task_edge_cache_.erase(it);
            ++removed_edges;
        } else {
            ++it;
        }
    }
    ROS_INFO("[SCHED][INVALIDATE] task=%s#%d cache_removed=%zu",
             taskTypeName(key.type), key.id, removed + removed_edges);
}

void GlobalTourPlanner::clear() {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    path_cache_.clear();
    task_edge_cache_.clear();
    current_goal_key_.reset();
    last_switch_time_ = ros::Time(0);
    ROS_INFO("[SCHED][INVALIDATE] planner cache cleared");
}

}  // namespace rog_map
