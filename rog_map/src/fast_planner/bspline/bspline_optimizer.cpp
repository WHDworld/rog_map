#include "fast_planner/bspline/bspline_optimizer.h"

namespace fast_planner {
const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
const int BsplineOptimizer::DISTANCE = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::START = (1 << 3);
const int BsplineOptimizer::END = (1 << 4);
const int BsplineOptimizer::WAYPOINTS = (1 << 5);
const int BsplineOptimizer::MINTIME = (1 << 6);

const int BsplineOptimizer::EXPLORATION_POSITION_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY |
    BsplineOptimizer::START | BsplineOptimizer::END | BsplineOptimizer::MINTIME;
const int BsplineOptimizer::EXPLORATION_YAW_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END |
    BsplineOptimizer::WAYPOINTS;

void BsplineOptimizer::setParam(ros::NodeHandle &nh) {
  nh.param("bspline_opt/pos/smoothness", pos_smooth_, 10.0);
  nh.param("bspline_opt/pos/distance", pos_dist_, 5.0);
  nh.param("bspline_opt/pos/feasibility", pos_feasi_, 1.0);
  nh.param("bspline_opt/pos/start", pos_start_, 100.0);
  nh.param("bspline_opt/pos/end", pos_end_, 1.0);
  nh.param("bspline_opt/pos/time", pos_time_, 10.0);

  nh.param("bspline_opt/yaw/smoothness", yaw_smooth_, 10.0);
  nh.param("bspline_opt/yaw/start", yaw_start_, 100.0);
  nh.param("bspline_opt/yaw/end", yaw_end_, 1.0);
  nh.param("bspline_opt/yaw/waypoint", yaw_waypt_, 1.0);

  nh.param("bspline_opt/bspline_degree", bspline_degree_, 3);
  nh.param("bspline_opt/safe_distance", safe_distance_, 0.3);
  nh.param("bspline_opt/max_iteration_num", max_iteration_num_, 300);
  nh.param("bspline_opt/max_iteration_time", max_iteration_time_, 0.02);

  time_lb_ = -1;
}

void BsplineOptimizer::setMap(const rog_map::ROGMap::Ptr &map) {
  this->rog_map_ = map;
  
  // Get map bounds
  Eigen::Vector3f min_f = rog_map_->getGlobalMapMin();
  Eigen::Vector3f max_f = rog_map_->getGlobalMapMax();
  map_min_ = min_f.cast<double>();
  map_max_ = max_f.cast<double>();
}

void BsplineOptimizer::setCostFunction(const int &cost_code) {
  cost_function_ = cost_code;
}

void BsplineOptimizer::setBoundaryStates(const std::vector<Eigen::Vector3d> &start,
                                         const std::vector<Eigen::Vector3d> &end) {
  start_state_ = start;
  end_state_ = end;
}

void BsplineOptimizer::setTimeLowerBound(const double &lb) { time_lb_ = lb; }

void BsplineOptimizer::setWaypoints(const std::vector<Eigen::Vector3d> &waypts,
                                    const std::vector<int> &waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

void BsplineOptimizer::setPhysicalLimits(const double &max_vel, const double &max_acc) {
  max_vel_ = max_vel;
  max_acc_ = max_acc;
}

void BsplineOptimizer::optimize(Eigen::MatrixXd &points, double &dt, const int &cost_function) {
  if (start_state_.empty()) {
    ROS_ERROR("[BsplineOptimizer] Initial state undefined!");
    return;
  }
  
  control_points_ = points;
  knot_span_ = dt;
  setCostFunction(cost_function);

  dim_ = control_points_.cols();
  if (dim_ == 1)
    order_ = 3;
  else
    order_ = bspline_degree_;
  point_num_ = control_points_.rows();
  
  // Check minimum control points (need at least 4 for cubic B-spline jerk calculation)
  if (point_num_ < 4) {
    ROS_WARN("[BsplineOptimizer] Too few control points (%d), skipping optimization", point_num_);
    return;
  }
  
  optimize_time_ = cost_function_ & MINTIME;
  variable_num_ = optimize_time_ ? dim_ * point_num_ + 1 : dim_ * point_num_;
  if (variable_num_ <= 0) {
    ROS_ERROR("[BsplineOptimizer] Empty variable to optimize.");
    return;
  }

  pt_dist_ = 0.0;
  for (int i = 0; i < control_points_.rows() - 1; ++i) {
    pt_dist_ += (control_points_.row(i + 1) - control_points_.row(i)).norm();
  }
  pt_dist_ /= double(point_num_);
  
  // Avoid division by zero in smoothness cost
  if (pt_dist_ < 1e-6) {
    pt_dist_ = 0.1;
  }

  ROS_DEBUG("[BsplineOptimizer] Optimizing with %d control points, dim=%d, dt=%.3f, pt_dist=%.3f",
            point_num_, dim_, knot_span_, pt_dist_);

  min_cost_ = std::numeric_limits<double>::max();
  best_variable_.clear();  // Clear previous best
  
  g_q_.resize(point_num_);
  g_smoothness_.resize(point_num_);
  g_distance_.resize(point_num_);
  g_feasibility_.resize(point_num_);
  g_start_.resize(point_num_);
  g_end_.resize(point_num_);
  g_waypoints_.resize(point_num_);
  g_time_.resize(point_num_);

  optimize();

  // Only update if optimization found a valid solution
  if (!best_variable_.empty()) {
    points = control_points_;
    dt = knot_span_;
  }
  
  start_state_.clear();
  time_lb_ = -1;
}

void BsplineOptimizer::optimize() {
  nlopt::opt opt(nlopt::algorithm::LD_LBFGS, variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_);
  opt.set_maxtime(max_iteration_time_);
  opt.set_xtol_rel(1e-4);

  // Set bounds - use default large bounds if map bounds are invalid
  Eigen::Vector3d bmin, bmax;
  if (map_min_.norm() > 1e-6 && map_max_.norm() > 1e-6 && 
      (map_max_ - map_min_).minCoeff() > 1.0) {
    bmin = map_min_;
    bmax = map_max_;
    for (int k = 0; k < 3; ++k) {
      bmin[k] += 0.1;
      bmax[k] -= 0.1;
    }
  } else {
    // Use large default bounds
    bmin = Eigen::Vector3d(-1000, -1000, -100);
    bmax = Eigen::Vector3d(1000, 1000, 100);
  }

  std::vector<double> q(variable_num_);
  for (int i = 0; i < point_num_; ++i)
    for (int j = 0; j < dim_; ++j) {
      double cij = control_points_(i, j);
      if (dim_ != 1)
        cij = std::max(std::min(cij, bmax[j % 3]), bmin[j % 3]);
      q[dim_ * i + j] = cij;
    }
  if (optimize_time_)
    q[variable_num_ - 1] = knot_span_;

  // Initialize best_variable with current state
  best_variable_ = q;

  if (dim_ != 1) {
    std::vector<double> lb(variable_num_), ub(variable_num_);
    const double bound = 10.0;
    for (int i = 0; i < 3 * point_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
      lb[i] = std::max(lb[i], bmin[i % 3]);
      ub[i] = std::min(ub[i], bmax[i % 3]);
    }
    if (optimize_time_) {
      lb[variable_num_ - 1] = 0.01;  // Minimum dt to avoid numerical issues
      ub[variable_num_ - 1] = 5.0;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    double final_cost;
    opt.optimize(q, final_cost);
    ROS_DEBUG("[BsplineOptimizer] Optimization converged, final cost: %.4f", final_cost);
  } catch (std::exception &e) {
    ROS_WARN("[BsplineOptimizer] NLopt exception: %s (using best found solution)", e.what());
  }

  // Use best found solution
  if (!best_variable_.empty()) {
    for (int i = 0; i < point_num_; ++i)
      for (int j = 0; j < dim_; ++j)
        control_points_(i, j) = best_variable_[dim_ * i + j];
    if (optimize_time_)
      knot_span_ = best_variable_[variable_num_ - 1];
  }
}

void BsplineOptimizer::calcSmoothnessCost(const std::vector<Eigen::Vector3d> &q, const double &dt,
                                          double &cost, std::vector<Eigen::Vector3d> &gradient_q,
                                          double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (size_t i = 0; i < q.size() - 3; i++) {
    Eigen::Vector3d ji = (q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]) / pt_dist_;
    cost += ji.squaredNorm();
    temp_j = 2 * ji / pt_dist_;

    gradient_q[i + 0] += -temp_j;
    gradient_q[i + 1] += 3.0 * temp_j;
    gradient_q[i + 2] += -3.0 * temp_j;
    gradient_q[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(const std::vector<Eigen::Vector3d> &q, double &cost,
                                        std::vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  if (!rog_map_) return;

  for (size_t i = 0; i < q.size(); i++) {
    const Eigen::Vector3d& pos = q[i];
    
    // Try to use ESDF distance if available
    double dist = rog_map_->getESDFDistance(pos);
    
    if (dist < safe_distance_) {
      // Cost increases as we get closer to obstacles
      cost += std::pow(dist - safe_distance_, 2);
      
      // Compute numerical gradient for ESDF
      Eigen::Vector3d grad = Eigen::Vector3d::Zero();
      const double delta = 0.05;
      for (int j = 0; j < 3; ++j) {
        Eigen::Vector3d pos_plus = pos, pos_minus = pos;
        pos_plus[j] += delta;
        pos_minus[j] -= delta;
        
        double dist_plus = rog_map_->getESDFDistance(pos_plus);
        double dist_minus = rog_map_->getESDFDistance(pos_minus);
        
        // Only compute gradient if both distances are valid (< safe_distance_)
        double cost_plus = (dist_plus < safe_distance_) ? std::pow(dist_plus - safe_distance_, 2) : 0.0;
        double cost_minus = (dist_minus < safe_distance_) ? std::pow(dist_minus - safe_distance_, 2) : 0.0;
        
        grad[j] = (cost_plus - cost_minus) / (2 * delta);
      }
      gradient_q[i] += grad;
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const std::vector<Eigen::Vector3d> &q, const double &dt,
                                           double &cost, std::vector<Eigen::Vector3d> &gradient_q,
                                           double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  gt = 0.0;

  const double dt_inv = 1 / dt;
  const double dt_inv2 = dt_inv * dt_inv;

  // Velocity feasibility
  for (size_t i = 0; i < q.size() - 1; ++i) {
    Eigen::Vector3d vi = (q[i + 1] - q[i]) * dt_inv;
    for (int k = 0; k < 3; ++k) {
      double vd = fabs(vi[k]) - max_vel_;
      if (vd > 0.0) {
        cost += pow(vd, 2);
        double sign = vi[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * vd * sign * dt_inv;
        gradient_q[i][k] += -tmp;
        gradient_q[i + 1][k] += tmp;
        if (optimize_time_)
          gt += tmp * (-vi[k]);
      }
    }
  }

  // Acceleration feasibility
  for (size_t i = 0; i < q.size() - 2; ++i) {
    Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i]) * dt_inv2;
    for (int k = 0; k < 3; ++k) {
      double ad = fabs(ai[k]) - max_acc_;
      if (ad > 0.0) {
        cost += pow(ad, 2);
        double sign = ai[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * ad * sign * dt_inv2;
        gradient_q[i][k] += tmp;
        gradient_q[i + 1][k] += -2 * tmp;
        gradient_q[i + 2][k] += tmp;
        if (optimize_time_)
          gt += tmp * ai[k] * (-2) * dt;
      }
    }
  }
}

void BsplineOptimizer::calcStartCost(const std::vector<Eigen::Vector3d> &q, const double &dt,
                                     double &cost, std::vector<Eigen::Vector3d> &gradient_q,
                                     double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  for (int i = 0; i < 3; ++i)
    gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q1, q2, q3, dq;
  q1 = q[0];
  q2 = q[1];
  q3 = q[2];

  // Start position
  static const double w_pos = 10.0;
  dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - start_state_[0];
  cost += w_pos * dq.squaredNorm();
  gradient_q[0] += w_pos * 2 * dq * (1 / 6.0);
  gradient_q[1] += w_pos * 2 * dq * (4 / 6.0);
  gradient_q[2] += w_pos * 2 * dq * (1 / 6.0);

  // Start velocity
  dq = 1 / (2 * dt) * (q3 - q1) - start_state_[1];
  cost += dq.squaredNorm();
  gradient_q[0] += 2 * dq * (-1.0) / (2 * dt);
  gradient_q[2] += 2 * dq * 1.0 / (2 * dt);
  if (optimize_time_)
    gt += dq.dot(q3 - q1) / (-dt * dt);

  // Start acceleration
  dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - start_state_[2];
  cost += dq.squaredNorm();
  gradient_q[0] += 2 * dq * 1.0 / (dt * dt);
  gradient_q[1] += 2 * dq * (-2.0) / (dt * dt);
  gradient_q[2] += 2 * dq * 1.0 / (dt * dt);
  if (optimize_time_)
    gt += dq.dot(q1 - 2 * q2 + q3) / (-dt * dt * dt);
}

void BsplineOptimizer::calcEndCost(const std::vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                                   std::vector<Eigen::Vector3d> &gradient_q, double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  for (size_t i = q.size() - 3; i < q.size(); ++i)
    gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  // End position
  dq = 1 / 6.0 * (q_1 + 4 * q_2 + q_3) - end_state_[0];
  cost += dq.squaredNorm();
  gradient_q[q.size() - 1] += 2 * dq * (1 / 6.0);
  gradient_q[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient_q[q.size() - 3] += 2 * dq * (1 / 6.0);

  if (end_state_.size() >= 2) {
    // End velocity
    dq = 1 / (2 * dt) * (q_1 - q_3) - end_state_[1];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (2 * dt);
    gradient_q[q.size() - 3] += 2 * dq * (-1.0) / (2 * dt);
    if (optimize_time_)
      gt += dq.dot(q_1 - q_3) / (-dt * dt);
  }
  if (end_state_.size() == 3) {
    // End acceleration
    dq = 1 / (dt * dt) * (q_1 - 2 * q_2 + q_3) - end_state_[2];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (dt * dt);
    gradient_q[q.size() - 2] += 2 * dq * (-2.0) / (dt * dt);
    gradient_q[q.size() - 3] += 2 * dq * 1.0 / (dt * dt);
    if (optimize_time_)
      gt += dq.dot(q_1 - 2 * q_2 + q_3) / (-dt * dt * dt);
  }
}

void BsplineOptimizer::calcWaypointsCost(const std::vector<Eigen::Vector3d> &q, double &cost,
                                         std::vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int idx = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient_q[idx] += dq * (2.0 / 6.0);
    gradient_q[idx + 1] += dq * (8.0 / 6.0);
    gradient_q[idx + 2] += dq * (2.0 / 6.0);
  }
}

void BsplineOptimizer::calcTimeCost(const double &dt, double &cost, double &gt) {
  double duration = (point_num_ - order_) * dt;
  cost = duration;
  gt = double(point_num_ - order_);

  // Time lower bound
  if (time_lb_ > 0 && duration < time_lb_) {
    static const double w_lb = 10;
    cost += w_lb * pow(duration - time_lb_, 2);
    gt += w_lb * 2 * (duration - time_lb_) * (point_num_ - order_);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad,
                                   double &f_combine) {
  for (int i = 0; i < point_num_; ++i) {
    for (int j = 0; j < dim_; ++j)
      g_q_[i][j] = x[dim_ * i + j];
    for (int j = dim_; j < 3; ++j)
      g_q_[i][j] = 0.0;
  }
  const double dt = optimize_time_ ? x[variable_num_ - 1] : knot_span_;

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  double f_smoothness = 0.0, gt_smoothness = 0.0;
  double f_distance = 0.0;
  double f_feasibility = 0.0, gt_feasibility = 0.0;
  double f_start = 0.0, gt_start = 0.0;
  double f_end = 0.0, gt_end = 0.0;
  double f_waypoints = 0.0;
  double f_time = 0.0, gt_time = 0.0;

  double ld_smooth = 0, ld_dist = 0, ld_feasi = 0, ld_start = 0, ld_end = 0, ld_waypt = 0, ld_time = 0;

  if (cost_function_ == EXPLORATION_POSITION_PHASE) {
    ld_smooth = pos_smooth_;
    ld_dist = pos_dist_;
    ld_feasi = pos_feasi_;
    ld_start = pos_start_;
    ld_end = pos_end_;
    ld_time = pos_time_;
  } else if (cost_function_ == EXPLORATION_YAW_PHASE) {
    ld_smooth = yaw_smooth_;
    ld_start = yaw_start_;
    ld_end = yaw_end_;
    ld_waypt = yaw_waypt_;
  } else {
    ROS_ERROR("[BsplineOptimizer] Invalid cost function.");
    return;
  }

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, dt, f_smoothness, g_smoothness_, gt_smoothness);
    f_combine += ld_smooth * f_smoothness;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_smooth * g_smoothness_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_smooth * gt_smoothness;
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += ld_dist * f_distance;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_dist * g_distance_[i](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, dt, f_feasibility, g_feasibility_, gt_feasibility);
    f_combine += ld_feasi * f_feasibility;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_feasi * g_feasibility_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_feasi * gt_feasibility;
  }
  if (cost_function_ & START) {
    calcStartCost(g_q_, dt, f_start, g_start_, gt_start);
    f_combine += ld_start * f_start;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_start * g_start_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_start * gt_start;
  }
  if (cost_function_ & END) {
    calcEndCost(g_q_, dt, f_end, g_end_, gt_end);
    f_combine += ld_end * f_end;
    for (int i = point_num_ - 3; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_end * g_end_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_end * gt_end;
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += ld_waypt * f_waypoints;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_waypt * g_waypoints_[i](j);
  }
  if (cost_function_ & MINTIME) {
    calcTimeCost(dt, f_time, gt_time);
    f_combine += ld_time * f_time;
    grad[variable_num_ - 1] += ld_time * gt_time;
  }

  // Record final costs
  final_costs_.clear();
  if (cost_function_ & SMOOTHNESS)
    final_costs_.push_back(ld_smooth * f_smoothness);
  if (cost_function_ & DISTANCE)
    final_costs_.push_back(ld_dist * f_distance);
  if (cost_function_ & FEASIBILITY)
    final_costs_.push_back(ld_feasi * f_feasibility);
  if (cost_function_ & START)
    final_costs_.push_back(ld_start * f_start);
  if (cost_function_ & END)
    final_costs_.push_back(ld_end * f_end);
  if (cost_function_ & WAYPOINTS)
    final_costs_.push_back(ld_waypt * f_waypoints);
  if (cost_function_ & MINTIME)
    final_costs_.push_back(ld_time * f_time);
}

double BsplineOptimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad,
                                      void *func_data) {
  BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);

  if (cost < opt->min_cost_) {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
  }
  return cost;
}
} // namespace fast_planner

