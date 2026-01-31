#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include "ego_planner/optimizer/poly_traj_utils.hpp"

using std::vector;

namespace ego_planner
{
  // Check points list - for collision checking
  typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  struct LocalTrajData
  {
    poly_traj::Trajectory traj; //MINCO轨迹
    PtsChk_t pts_chk; //用于存储碰撞检测或约束检查的相关信息（比如哪些控制点是安全的，哪些是在障碍物附近的）
    int traj_id;
    double duration;
    double start_time; // ros::Time time
    Eigen::Vector3d start_pos;
    
    // [Innovation] Explicit Yaw Trajectory for 4D Planning
    bool use_yaw_traj = false;
    std::vector<double> yaw_ctrl_pts;
    double yaw_start_time = 0.0;
  };

  struct GlobalTrajData
  {
    poly_traj::Trajectory traj; // 局参考轨迹对象。它通常比较平滑，但不一定无碰撞
    double duration; // 轨迹的总持续时间
    double global_start_time; // world time
    double last_glb_t_of_lc_tgt; // 上一次局部目标的时间
    double glb_t_of_lc_tgt; // 当前局部目标在全局轨迹上的时间
  };

  struct TrajContainer
  {
    GlobalTrajData global_traj;
    LocalTrajData local_traj;

    void setGlobalTraj(const poly_traj::Trajectory &traj, const double &world_time)
    {
      global_traj.traj = traj;
      global_traj.duration = traj.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;
      global_traj.last_glb_t_of_lc_tgt = -1.0;

      local_traj.traj_id = 0;
    }

    void setLocalTraj(const poly_traj::Trajectory &traj, const PtsChk_t &pts_chk, const double &world_time)
    {
      local_traj.traj = traj;
      local_traj.pts_chk = pts_chk;
      local_traj.duration = traj.getTotalDuration();
      local_traj.start_time = world_time;
      local_traj.start_pos = traj.getPos(0);
      ++local_traj.traj_id;
    }
  };

  struct PlanParameters
  {
    double max_vel_ = 2.0;
    double max_acc_ = 2.0;
    double max_jerk_ = 4.0;
    double max_yaw_rate_ = 1.0;   // [新增] 最大偏航角速度 [rad/s]
    double max_yaw_acc_ = 1.0;    // [新增] 最大偏航角加速度 [rad/s²]
    double feasibility_tolerance_ = 0.05;
    double polyTraj_piece_length = 1.5;
    double planning_horizen_ = 7.5;
    bool use_distinctive_trajs = false;
  };

} // namespace ego_planner

#endif

