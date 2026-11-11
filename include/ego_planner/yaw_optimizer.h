#ifndef _YAW_OPTIMIZER_H_
#define _YAW_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include "ego_planner/optimizer/poly_traj_utils.hpp"

namespace ego_planner
{

class YawOptimizer
{
public:
  YawOptimizer() {}
  ~YawOptimizer() {}

  /**
   * @brief Analyze XYZ trajectory and optimize Yaw trajectory using B-spline
   * 
   * @param xyz_traj Optimized XYZ trajectory (B-spline)
   * @param start_yaw Robot's current yaw
   * @param end_yaw Goal yaw
   * @param yaw_ctrl_pts Output: B-spline control points for Yaw
   * @param dt Time step for discrete optimization (default 0.05s)
   * @return true 
   * @return false 
   */
  bool optimize(const poly_traj::Trajectory& xyz_traj, 
                double start_yaw, 
                double end_yaw, 
                std::vector<double>& yaw_ctrl_pts,
                double dt = 0.05,
                double start_yaw_rate = 0.0,
                double start_yaw_acc = 0.0);

  /**
   * @brief Yaw轨迹优化（支持中间航点yaw约束）
   * 
   * @param xyz_traj 位置轨迹
   * @param start_yaw 起始yaw
   * @param end_yaw 终点yaw
   * @param waypoint_yaws 中间航点yaw约束 (time, yaw) pairs
   * @param yaw_ctrl_pts 输出: B样条控制点
   * @param max_yaw_rate 最大角速度约束 [rad/s]
   * @param dt 时间步长
   * @return true 优化成功
   */
  bool optimizeWithWaypoints(
      const poly_traj::Trajectory& xyz_traj, 
      double start_yaw, 
      double end_yaw, 
      const std::vector<std::pair<double, double>>& waypoint_yaws,
      std::vector<double>& yaw_ctrl_pts,
      double max_yaw_rate = 1.0,
      double dt = 0.05);

  // Helper functions for B-spline evaluation (Uniform B-spline, order=3 or 4)
  // Assuming Yaw B-spline has same time allocation as XYZ or uniform? 
  // For simplicity and coupling, we can reuse XYZ's time allocation or resample.
  // Here we assume uniform B-spline for simplicity in this standalone class.
  
  static double evaluateYaw(const std::vector<double>& ctrl_pts, double t, double total_dur);
  static double evaluateYawDot(const std::vector<double>& ctrl_pts, double t, double total_dur);
  static double evaluateYawDDot(const std::vector<double>& ctrl_pts, double t, double total_dur);

private:
  // Optimization weights (TV-YC tuned)
  double w_smooth_ = 10.0;    // Smoothness (minimize jerk/acc)
  double w_vel_ = 1000.0;     // Alignment with velocity (巡航阶段强跟随速度方向)
  double w_target_ = 1000.0;  // Alignment with target yaw (接近阶段强收敛到目标)

  // Solve Unconstrained QP: min 0.5 * x'Hx + f'x
  bool solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& f, Eigen::VectorXd& x);
};

} // namespace ego_planner

#endif
