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
                double dt = 0.05);

  // Helper functions for B-spline evaluation (Uniform B-spline, order=3 or 4)
  // Assuming Yaw B-spline has same time allocation as XYZ or uniform? 
  // For simplicity and coupling, we can reuse XYZ's time allocation or resample.
  // Here we assume uniform B-spline for simplicity in this standalone class.
  
  static double evaluateYaw(const std::vector<double>& ctrl_pts, double t, double total_dur);
  static double evaluateYawDot(const std::vector<double>& ctrl_pts, double t, double total_dur);

private:
  // Optimization weights
  double w_smooth_ = 10.0; // Smoothness (minimize jerk/acc)
  double w_vel_ = 10.0;    // Alignment with velocity
  double w_target_ = 10.0; // Alignment with target yaw

  // Solve Unconstrained QP: min 0.5 * x'Hx + f'x
  bool solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& f, Eigen::VectorXd& x);
};

} // namespace ego_planner

#endif
