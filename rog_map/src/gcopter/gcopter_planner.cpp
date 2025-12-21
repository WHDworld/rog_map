/**
 * @file gcopter_planner.cpp
 * @brief GCOPTER 轨迹规划器实现
 */

// 重要：先包含 GCOPTER 头文件，避免命名空间污染
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"

// 然后包含 ROG-Map 头文件
#include "rog_map/rog_map.h"

// 最后包含自己的头文件（它会包含前面已包含的，但不会重复）
#include "gcopter/gcopter_planner.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace rog_map {

void GCOPTERPlanner::setMapBounds(const Eigen::Vector3d& lower, 
                                   const Eigen::Vector3d& upper) {
    map_lower_ = lower;
    map_upper_ = upper;
}

bool GCOPTERPlanner::isPositionFree(const Eigen::Vector3d& pos) const {
    return queryCollision(pos) == 0;
}

double GCOPTERPlanner::getESDFDistance(const Eigen::Vector3d& pos) const {
    if (!rog_map_) return 0.0;
    
    Vec3f pos_f = pos;
    
    // // 优先使用 ESDF 距离（如果启用）
    // if (config_.use_esdf && rog_map_->isESDFEnabled()) {
    //     // 直接获取真正的 ESDF 距离
    //     return rog_map_->getESDFDistance(pos_f);
    // }
    
    // 回退：使用膨胀地图进行简单估计
    if (rog_map_->isKnownFreeInflate(pos_f)) {
        return config_.safety_margin + 0.1;
    } else if (rog_map_->isOccupiedInflate(pos_f)) {
        return -0.1;
    }
    return 0.0;
}

int GCOPTERPlanner::queryCollision(const Eigen::Vector3d& pos) const {
    if (!rog_map_) return 1;
    
    // 边界检查
    if (pos.x() < map_lower_.x() || pos.x() > map_upper_.x() ||
        pos.y() < map_lower_.y() || pos.y() > map_upper_.y() ||
        pos.z() < map_lower_.z() || pos.z() > map_upper_.z()) {
        return 1;  // 超出边界视为占用
    }
    
    Vec3f pos_f = pos;
    
    // ==================== 模式1：使用 ESDF 进行快速碰撞检查 ====================
    // 优点：快速、连续的距离查询，可用于梯度优化
    // ESDF 查询只需 O(1) 时间，无需遍历邻域
    // if (config_.use_esdf && rog_map_->isESDFEnabled()) {
    //     double dist = rog_map_->getESDFDistance(pos_f);
    //     // 距离小于安全裕度则认为碰撞
    //     if (dist < config_.safety_margin) {
    //         return 1;  // 碰撞
    //     }
    //     return 0;  // 自由
    // }
    
    // ==================== 模式2：使用膨胀地图进行碰撞检查 ====================
    // 使用膨胀地图进行碰撞检查（已考虑无人机半径）
    if (rog_map_->isOccupiedInflate(pos_f)) {
        return 1;  // 占用
    }
    
    // 根据配置决定是否将未知区域视为可通行
    if (!config_.treat_unknown_as_free && rog_map_->isUnknown(pos_f)) {
        return 1;  // 未知区域视为不可通行（保守策略）
    }
    
    return 0;  // 自由
}

GCOPTERPlanner::PlanResult GCOPTERPlanner::plan(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& goal_pos,
    const Eigen::Vector3d& goal_vel)
{
    PlanResult result;
    
    if (!initialized_ || !rog_map_) {
        result.error_msg = "Planner not initialized or ROGMap is null";
        return result;
    }
    
    auto t_start = ros::Time::now();
    
    // 更新地图边界
    Vec3f local_origin = rog_map_->getLocalMapOrigin();
    Vec3f local_size = rog_map_->getLocalMapSize();
    map_lower_ = local_origin.cast<double>();
    map_upper_ = (local_origin + local_size).cast<double>();
    
    // 1. 检查起点和终点
    if (queryCollision(start_pos) != 0) {
        result.error_msg = "Start position is in collision or unknown";
        return result;
    }
    if (queryCollision(goal_pos) != 0) {
        result.error_msg = "Goal position is in collision or unknown";
        return result;
    }
    
    // 2. RRT* 路径搜索
    if (!searchPath(start_pos, goal_pos, result.rrt_path)) {
        result.error_msg = "RRT* path search failed";
        return result;
    }
    ROS_INFO("[GCOPTERPlanner] RRT* found path with %zu waypoints", result.rrt_path.size());
    
    // 3. 生成安全飞行走廊
    if (!generateCorridor(result.rrt_path, result.corridors)) {
        result.error_msg = "SFC generation failed";
        return result;
    }
    ROS_INFO("[GCOPTERPlanner] Generated %zu corridors", result.corridors.size());
    
    // 4. 设置起始和终止状态 [pos | vel | acc]
    Eigen::Matrix3d iniState, finState;
    iniState.col(0) = start_pos;
    iniState.col(1) = start_vel;
    iniState.col(2) = Eigen::Vector3d::Zero();
    
    finState.col(0) = goal_pos;
    finState.col(1) = goal_vel;
    finState.col(2) = Eigen::Vector3d::Zero();
    
    // 5. 轨迹优化
    if (!optimizeTrajectory(iniState, finState, result.corridors, result.trajectory)) {
        result.error_msg = "Trajectory optimization failed";
        return result;
    }
    
    result.success = true;
    result.total_time = result.trajectory.getTotalDuration();
    result.cost = result.total_time;
    
    auto t_end = ros::Time::now();
    ROS_INFO("[GCOPTERPlanner] Planning succeeded! Duration: %.2fs, Planning time: %.3fs",
             result.total_time, (t_end - t_start).toSec());
    
    return result;
}

GCOPTERPlanner::PlanResult GCOPTERPlanner::planThroughWaypoints(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& start_vel,
    const std::vector<Eigen::Vector3d>& waypoints)
{
    PlanResult result;
    
    if (waypoints.empty()) {
        result.error_msg = "Empty waypoints";
        return result;
    }
    
    if (!initialized_ || !rog_map_) {
        result.error_msg = "Planner not initialized";
        return result;
    }
    
    // 更新地图边界
    Vec3f local_origin = rog_map_->getLocalMapOrigin();
    Vec3f local_size = rog_map_->getLocalMapSize();
    map_lower_ = local_origin.cast<double>();
    map_upper_ = (local_origin + local_size).cast<double>();
    
    // 构建完整路径：start -> waypoints
    std::vector<Eigen::Vector3d> full_path;
    full_path.push_back(start_pos);
    for (const auto& wp : waypoints) {
        full_path.push_back(wp);
    }
    
    // 逐段搜索 RRT* 路径
    std::vector<Eigen::Vector3d> combined_rrt_path;
    for (size_t i = 0; i < full_path.size() - 1; ++i) {
        std::vector<Eigen::Vector3d> segment_path;
        if (!searchPath(full_path[i], full_path[i+1], segment_path)) {
            result.error_msg = "RRT* failed for segment " + std::to_string(i);
            return result;
        }
        
        // 合并路径（避免重复点）
        if (combined_rrt_path.empty()) {
            combined_rrt_path = segment_path;
        } else {
            combined_rrt_path.insert(combined_rrt_path.end(), 
                                     segment_path.begin() + 1, segment_path.end());
        }
    }
    
    result.rrt_path = combined_rrt_path;
    
    // 生成走廊
    if (!generateCorridor(result.rrt_path, result.corridors)) {
        result.error_msg = "SFC generation failed";
        return result;
    }
    
    // 设置状态
    Eigen::Matrix3d iniState, finState;
    iniState.col(0) = start_pos;
    iniState.col(1) = start_vel;
    iniState.col(2) = Eigen::Vector3d::Zero();
    
    finState.col(0) = waypoints.back();
    finState.col(1) = Eigen::Vector3d::Zero();  // 最终悬停
    finState.col(2) = Eigen::Vector3d::Zero();
    
    // 优化
    if (!optimizeTrajectory(iniState, finState, result.corridors, result.trajectory)) {
        result.error_msg = "Trajectory optimization failed";
        return result;
    }
    
    result.success = true;
    result.total_time = result.trajectory.getTotalDuration();
    
    return result;
}

bool GCOPTERPlanner::searchPath(const Eigen::Vector3d& start,
                                 const Eigen::Vector3d& goal,
                                 std::vector<Eigen::Vector3d>& path)
{
    // 创建一个满足 sfc_gen::planPath 模板要求的查询对象
    // sfc_gen::planPath 需要一个有 query(pos) 方法的对象
    struct QueryWrapper {
        const GCOPTERPlanner* planner;
        int query(const Eigen::Vector3d& pos) const {
            return planner->queryCollision(pos);
        }
    };
    
    QueryWrapper wrapper{this};
    
    double cost = sfc_gen::planPath(start, goal, map_lower_, map_upper_,
                                    &wrapper, config_.timeout_rrt, path);
    
    return !std::isinf(cost) && path.size() >= 2;
}

bool GCOPTERPlanner::generateCorridor(const std::vector<Eigen::Vector3d>& path,
                                       std::vector<Eigen::MatrixX4d>& hPolys)
{
    if (path.size() < 2) return false;
    
    // 生成凸包覆盖
    sfc_gen::convexCover(path, rog_map_->inf_occ_map_pcl,
                         map_lower_, map_upper_,
                         config_.sfc_progress,
                         config_.sfc_range,
                         hPolys);
    
    // 简化走廊
    if (!hPolys.empty()) {
        sfc_gen::shortCut(hPolys);
    }
    
    return !hPolys.empty();
}

bool GCOPTERPlanner::optimizeTrajectory(const Eigen::Matrix3d& iniState,
                                         const Eigen::Matrix3d& finState,
                                         const std::vector<Eigen::MatrixX4d>& hPolys,
                                         Trajectory<5>& traj)
{
    gcopter::GCOPTER_PolytopeSFC optimizer;
    
    // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]
    Eigen::VectorXd magnitudeBounds(5);
    magnitudeBounds << config_.max_vel, config_.max_body_rate, config_.max_tilt_angle,
                       config_.min_thrust, config_.max_thrust;
    
    // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]
    Eigen::VectorXd penaltyWeights(5);
    penaltyWeights << 1.0e4, 1.0e4, 1.0e4, 1.0e4, 1.0e5;
    
    // physicalParams = [mass, g, horiz_drag, vert_drag, paras_drag, speed_eps]
    Eigen::VectorXd physicalParams(6);
    physicalParams << config_.vehicle_mass, config_.grav_acc,
                      config_.horiz_drag, config_.vert_drag,
                      config_.paras_drag, config_.speed_eps;
    
    // 设置优化问题
    if (!optimizer.setup(config_.weight_time,
                         iniState, finState,
                         hPolys, INFINITY,
                         config_.smoothing_eps,
                         config_.integral_intervs,
                         magnitudeBounds,
                         penaltyWeights,
                         physicalParams)) {
        ROS_ERROR("[GCOPTERPlanner] Optimizer setup failed");
        return false;
    }
    
    // 执行优化
    double cost = optimizer.optimize(traj, config_.rel_cost_tol);
    
    if (std::isinf(cost) || traj.getPieceNum() == 0) {
        ROS_ERROR("[GCOPTERPlanner] Optimization failed, cost: %f", cost);
        return false;
    }
    
    return true;
}

} // namespace rog_map

