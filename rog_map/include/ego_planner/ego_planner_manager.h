#ifndef _EGO_PLANNER_MANAGER_H_
#define _EGO_PLANNER_MANAGER_H_

#include <stdlib.h>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include "ego_planner/poly_traj_optimizer.h"
#include "ego_planner/plan_container.hpp"
#include "ego_planner/optimizer/poly_traj_utils.hpp"
#include "ego_planner/yaw_optimizer.h"
// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"

namespace ego_planner
{

  // EGO Planner Manager for STAR-MVF Map
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, star_mvf::MapManager::Ptr map);
    
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
    
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit, 
        const bool flag_randomPolyTraj, const bool touch_goal,
        double start_yaw = 0.0, double end_yaw = 10000.0);
    
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
        double start_yaw = 0.0, double end_yaw = 10000.0);
    
    // [新增] 带中间航点yaw约束的版本（用于重建任务）
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
        double start_yaw, double end_yaw,
        const std::vector<double>& waypoint_yaws);  // 每个航点的期望yaw
    
    
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);
    
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal, 
                               const double time_offset = 0.0);
    
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    
    // 设置期望高度（用于高度保持功能）
    inline void setDesiredHeight(double height) { ploy_traj_opt_->setDesiredHeight(height); }
    
    // 启用/禁用高度保持 (重建时应禁用)
    inline void setEnableHeightKeeping(bool enable) { ploy_traj_opt_->setEnableHeightKeeping(enable); }
    
    // FALCON风格：设置重规划时间偏移（用于轨迹连续性）
    inline void setReplanTimeOffset(double offset) { replan_time_offset_ = offset; }
    
    // 设置精确的重规划开始时间（用于确保 start_time 与预测状态匹配）
    inline void setReplanStartTime(double time) { replan_start_time_ = time; use_precise_start_time_ = true; }
    
    // [新增] 设置速度缩放比例（用于重建任务时降低速度）
    // scale: 1.0 = 使用原始 max_vel，0.5 = 使用 0.5 * max_vel
    inline void setVelocityScale(double scale) {
        velocity_scale_ = std::max(0.1, std::min(1.0, scale));  // 限制在 [0.1, 1.0]
        pp_.max_vel_ = base_max_vel_ * velocity_scale_;
        ROS_INFO("[EGOPlannerManager] Velocity scale set to %.2f, effective max_vel=%.2f", 
                 velocity_scale_, pp_.max_vel_);
    }
    
    // 获取当前速度缩放比例
    inline double getVelocityScale() const { return velocity_scale_; }

    PlanParameters pp_;
    star_mvf::MapManager::Ptr map_manager_;
    TrajContainer traj_;

  private:
    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    int continous_failures_count_{0};
    double replan_time_offset_{0.0};  // 重规划时间偏移，用于延迟新轨迹生效
    double replan_start_time_{0.0};   // 精确的重规划开始时间
    bool use_precise_start_time_{false};  // 是否使用精确的开始时间
    
    // [新增] 速度缩放相关
    double base_max_vel_{2.0};    // 原始最大速度（从参数文件读取）
    double velocity_scale_{1.0};  // 当前速度缩放比例

    std::unique_ptr<YawOptimizer> yaw_optimizer_;

  public:
    typedef std::unique_ptr<EGOPlannerManager> Ptr;
  };
  
} // namespace ego_planner

#endif
