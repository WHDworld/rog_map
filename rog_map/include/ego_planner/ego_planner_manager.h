#ifndef _EGO_PLANNER_MANAGER_H_
#define _EGO_PLANNER_MANAGER_H_

#include <stdlib.h>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include "ego_planner/poly_traj_optimizer.h"
#include "ego_planner/plan_container.hpp"
#include "ego_planner/optimizer/poly_traj_utils.hpp"
#include <rog_map/rog_map.h>

namespace ego_planner
{

  // EGO Planner Manager for ROG-Map
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, rog_map::ROGMap::Ptr map);
    
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
    
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit, 
        const bool flag_randomPolyTraj, const bool touch_goal);
    
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);
    
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal);
    
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    
    // 设置期望高度（用于高度保持功能）
    inline void setDesiredHeight(double height) { ploy_traj_opt_->setDesiredHeight(height); }

    PlanParameters pp_;
    rog_map::ROGMap::Ptr rog_map_;
    TrajContainer traj_;

  private:
    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    int continous_failures_count_{0};

  public:
    typedef std::unique_ptr<EGOPlannerManager> Ptr;
  };
  
} // namespace ego_planner

#endif
