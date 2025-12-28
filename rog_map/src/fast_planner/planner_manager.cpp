#include "fast_planner/planner_manager.h"
#include <queue>
#include <unordered_map>

namespace fast_planner {

// ==================== Yaw Helper Functions ====================
// Helper function to wrap yaw to [-PI, PI]
static double wrapYaw(double yaw) {
  while (yaw < -M_PI) yaw += 2 * M_PI;
  while (yaw > M_PI) yaw -= 2 * M_PI;
  return yaw;
}

// Helper function to find shortest yaw difference
static double yawDiff(double yaw1, double yaw2) {
  double diff = wrapYaw(yaw2) - wrapYaw(yaw1);
  if (diff > M_PI) diff -= 2 * M_PI;
  if (diff < -M_PI) diff += 2 * M_PI;
  return diff;
}

// ==================== ROGAstar Implementation ====================
ROGAstar::~ROGAstar() {
  for (auto &node : node_pool_) {
    if (node) {
      // Use Eigen aligned delete
      Eigen::aligned_allocator<AstarNode> alloc;
      alloc.deallocate(node, 1);
    }
  }
  node_pool_.clear();
}

void ROGAstar::init(ros::NodeHandle &nh, const rog_map::ROGMap::Ptr &map) {
  rog_map_ = map;
  
  // Load parameters
  nh.param("astar/resolution", config_.resolution, 0.1);
  nh.param("astar/max_search_time", config_.max_search_time, 0.5);
  nh.param("astar/lambda_heuristic", config_.lambda_heuristic, 1.0);
  nh.param("astar/allocate_num", config_.allocate_num, 100000);
  nh.param("astar/unknown_as_free", config_.unknown_as_free, true);
  nh.param("astar/use_inflated_map", config_.use_inflated_map, true);
  nh.param("astar/verbose", config_.verbose, false);
  
  // Get resolution from map if available
  config_.resolution = rog_map_->getInfResolution();
  
  // Allocate node pool with Eigen aligned allocator
  Eigen::aligned_allocator<AstarNode> alloc;
  node_pool_.resize(config_.allocate_num);
  for (int i = 0; i < config_.allocate_num; ++i) {
    node_pool_[i] = alloc.allocate(1);
    new (node_pool_[i]) AstarNode();  // Placement new
  }
  
  use_node_num_ = 0;
  
  ROS_INFO("[ROGAstar] Initialized with resolution: %.2f, max_search_time: %.2f",
           config_.resolution, config_.max_search_time);
}

void ROGAstar::reset() {
  use_node_num_ = 0;
  path_nodes_.clear();
}

void ROGAstar::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx) {
  idx = ((pos / config_.resolution).array().floor()).cast<int>();
}

void ROGAstar::indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3d &pos) {
  pos = (idx.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * config_.resolution;
}

double ROGAstar::getHeuristic(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
  // Use diagonal distance heuristic
  double dx = std::abs(p1.x() - p2.x());
  double dy = std::abs(p1.y() - p2.y());
  double dz = std::abs(p1.z() - p2.z());
  
  double h = 0.0;
  double dmin = std::min({dx, dy, dz});
  double dmax = std::max({dx, dy, dz});
  double dmid = dx + dy + dz - dmin - dmax;
  
  h = (std::sqrt(3.0) - std::sqrt(2.0)) * dmin + (std::sqrt(2.0) - 1.0) * dmid + dmax;
  
  return config_.lambda_heuristic * h;
}

bool ROGAstar::isOccupied(const Eigen::Vector3d &pos) {
  // Note: rog_map::Vec3f is Eigen::Vector3d (decimal_t = double)
  if (config_.use_inflated_map) {
    // Use inflated map for better obstacle avoidance
    if (config_.unknown_as_free) {
      return rog_map_->isOccupiedInflate(pos);
    } else {
      return rog_map_->isOccupiedInflate(pos) || rog_map_->isUnknown(pos);
    }
  } else {
    // Use raw occupancy map
    if (config_.unknown_as_free) {
      return rog_map_->isOccupied(pos);
    } else {
      return rog_map_->isOccupied(pos) || rog_map_->isUnknown(pos);
    }
  }
}

void ROGAstar::backtrack(AstarNode* end_node) {
  path_nodes_.clear();
  AstarNode* node = end_node;
  while (node != nullptr) {
    path_nodes_.push_back(node->position);
    node = node->parent;
  }
  std::reverse(path_nodes_.begin(), path_nodes_.end());
}

int ROGAstar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
  reset();
  
  ros::Time t_start = ros::Time::now();
  
  // Check start and end validity
  if (isOccupied(start_pt)) {
    ROS_WARN("[ROGAstar] Start point is occupied!");
    return NO_PATH;
  }
  if (isOccupied(end_pt)) {
    ROS_WARN("[ROGAstar] End point is occupied!");
    return NO_PATH;
  }
  
  // Initialize start node
  Eigen::Vector3i start_idx, end_idx;
  posToIndex(start_pt, start_idx);
  posToIndex(end_pt, end_idx);
  
  AstarNode* start_node = node_pool_[use_node_num_++];
  start_node->position = start_pt;
  start_node->index = start_idx;
  start_node->g_score = 0.0;
  start_node->f_score = getHeuristic(start_pt, end_pt);
  start_node->parent = nullptr;
  
  // Priority queue and hash maps
  std::priority_queue<AstarNode*, std::vector<AstarNode*>, NodeComparator> open_set;
  std::unordered_map<int64_t, AstarNode*> open_map;
  std::unordered_map<int64_t, bool> close_map;
  
  auto indexToHash = [](const Eigen::Vector3i &idx) -> int64_t {
    return static_cast<int64_t>(idx.x()) * 1000000000LL + 
           static_cast<int64_t>(idx.y()) * 1000000LL + 
           static_cast<int64_t>(idx.z() + 500000);  // offset to handle negative indices
  };
  
  open_set.push(start_node);
  open_map[indexToHash(start_idx)] = start_node;
  
  // Neighbor directions (26 connectivity)
  std::vector<Eigen::Vector3i> neighbors;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        neighbors.push_back(Eigen::Vector3i(dx, dy, dz));
      }
    }
  }
  
  int iter = 0;
  while (!open_set.empty()) {
    // Check timeout
    if ((ros::Time::now() - t_start).toSec() > config_.max_search_time) {
      ROS_WARN("[ROGAstar] Search timeout after %d iterations", iter);
      return TIMEOUT;
    }
    
    // Get current node
    AstarNode* current = open_set.top();
    open_set.pop();
    
    int64_t hash = indexToHash(current->index);
    if (close_map.find(hash) != close_map.end()) {
      continue;  // Already in closed set
    }
    close_map[hash] = true;
    
    // Check if reached goal
    if ((current->index - end_idx).squaredNorm() <= 1) {
      // Add end point
      AstarNode* end_node = node_pool_[use_node_num_++];
      end_node->position = end_pt;
      end_node->index = end_idx;
      end_node->parent = current;
      
      backtrack(end_node);
      
      if (config_.verbose) {
        ROS_INFO("[ROGAstar] Path found with %zu nodes in %d iterations, time: %.3f s",
                 path_nodes_.size(), iter, (ros::Time::now() - t_start).toSec());
      }
      return REACH_END;
    }
    
    // Expand neighbors
    for (const auto &dir : neighbors) {
      Eigen::Vector3i neighbor_idx = current->index + dir;
      Eigen::Vector3d neighbor_pos;
      indexToPos(neighbor_idx, neighbor_pos);
      
      // Check if in closed set
      int64_t neighbor_hash = indexToHash(neighbor_idx);
      if (close_map.find(neighbor_hash) != close_map.end()) {
        continue;
      }
      
      // Check collision
      if (isOccupied(neighbor_pos)) {
        continue;
      }
      
      // Calculate cost
      double move_cost = dir.cast<double>().norm() * config_.resolution;
      double tentative_g = current->g_score + move_cost;
      
      // Check if already in open set with better score
      auto it = open_map.find(neighbor_hash);
      if (it != open_map.end() && tentative_g >= it->second->g_score) {
        continue;
      }
      
      // Add new node
      if (use_node_num_ >= config_.allocate_num) {
        ROS_ERROR("[ROGAstar] Node pool exhausted!");
        return NO_PATH;
      }
      
      AstarNode* neighbor_node = node_pool_[use_node_num_++];
      neighbor_node->position = neighbor_pos;
      neighbor_node->index = neighbor_idx;
      neighbor_node->g_score = tentative_g;
      neighbor_node->f_score = tentative_g + getHeuristic(neighbor_pos, end_pt);
      neighbor_node->parent = current;
      
      open_set.push(neighbor_node);
      open_map[neighbor_hash] = neighbor_node;
    }
    
    ++iter;
  }
  
  ROS_WARN("[ROGAstar] No path found after %d iterations", iter);
  return NO_PATH;
}

int ROGAstar::searchUnknown(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
  bool old_setting = config_.unknown_as_free;
  config_.unknown_as_free = true;
  int result = search(start_pt, end_pt);
  config_.unknown_as_free = old_setting;
  return result;
}

double ROGAstar::getPathLength() {
  if (path_nodes_.size() < 2) return 0.0;
  
  double length = 0.0;
  for (size_t i = 0; i < path_nodes_.size() - 1; ++i) {
    length += (path_nodes_[i + 1] - path_nodes_[i]).norm();
  }
  return length;
}

// ==================== FastPlannerManager Implementation ====================
void FastPlannerManager::init(ros::NodeHandle &nh, const rog_map::ROGMap::Ptr &map) {
  rog_map_ = map;
  
  // Load parameters
  nh.param("planner/max_vel", pp_.max_vel_, 2.0);
  nh.param("planner/max_acc", pp_.max_acc_, 2.0);
  nh.param("planner/max_yaw_vel", pp_.max_yaw_vel_, 1.0472);  // 60 deg/s
  nh.param("planner/max_yaw_acc", pp_.max_yaw_acc_, 1.0472);  // 60 deg/s^2
  nh.param("planner/ctrl_pt_dist", pp_.ctrl_pt_dist_, 0.3);
  nh.param("planner/time_lower_bound", pp_.time_lower_bound_, 0.5);
  nh.param("planner/min_segment_time", pp_.min_segment_time_, 0.3);
  
  // Initialize path finder
  path_finder_.reset(new ROGAstar);
  path_finder_->init(nh, rog_map_);
  
  // Initialize B-spline optimizers (position and yaw)
  bspline_optimizers_.resize(2);
  for (size_t i = 0; i < bspline_optimizers_.size(); ++i) {
    bspline_optimizers_[i].reset(new BsplineOptimizer);
    bspline_optimizers_[i]->setParam(nh);
    bspline_optimizers_[i]->setMap(rog_map_);
    if (i == 0) {
      bspline_optimizers_[i]->setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
    }
  }
  
  local_data_.traj_id_ = 0;
  
  ROS_INFO("[FastPlannerManager] Initialized with max_vel: %.2f, max_acc: %.2f, max_yaw_vel: %.2f",
           pp_.max_vel_, pp_.max_acc_, pp_.max_yaw_vel_);
}

bool FastPlannerManager::planTrajectory(const Eigen::Vector3d &start_pos,
                                        const Eigen::Vector3d &start_vel,
                                        const Eigen::Vector3d &start_acc,
                                        const Eigen::Vector3d &goal_pos,
                                        const double &cur_yaw,
                                        const double &cur_yaw_vel) {
  // Search path using A*
  int search_result = path_finder_->searchUnknown(start_pos, goal_pos);
  ROS_INFO("[FastPlannerManager] Search result: %d", search_result);
  if (search_result != ROGAstar::REACH_END) {
    ROS_WARN("[FastPlannerManager] Path search failed with code: %d", search_result);
    return false;
  }
  
  std::vector<Eigen::Vector3d> path = path_finder_->getPath();
  ROS_INFO("[FastPlannerManager] Path length: %zu", path.size());
  if (path.size() < 2) {
    ROS_WARN("[FastPlannerManager] Path too short");
    return false;
  }
  
  // Shorten path (remove unnecessary waypoints)
  shortenPath(path);
  ROS_INFO("[FastPlannerManager] Shortened path length: %zu", path.size());
  
  // Ensure start and goal are in path
  path.front() = start_pos;
  path.back() = goal_pos;
  
  // Compute goal yaw: direction from start to goal
  Eigen::Vector3d dir_to_goal = goal_pos - start_pos;
  double goal_yaw = cur_yaw;
  if (dir_to_goal.head<2>().norm() > 0.3) {
    goal_yaw = std::atan2(dir_to_goal.y(), dir_to_goal.x());
  }
  
  // Compute yaw time lower bound using shortest rotation path
  // Use yawDiff to get the shortest angle change
  double yaw_change = std::abs(yawDiff(cur_yaw, goal_yaw));
  
  // Calculate yaw time with acceleration consideration (trapezoidal profile)
  // For yaw: t = yd/ydd + diff/yd (acceleration + cruise time)
  double yaw_time_lb;
  double yaw_accel_dist = 0.5 * pp_.max_yaw_vel_ * pp_.max_yaw_vel_ / pp_.max_yaw_acc_;
  if (yaw_change < 2 * yaw_accel_dist) {
    // Triangular profile
    yaw_time_lb = 2.0 * std::sqrt(yaw_change / pp_.max_yaw_acc_);
  } else {
    // Trapezoidal profile
    yaw_time_lb = pp_.max_yaw_vel_ / pp_.max_yaw_acc_ + yaw_change / pp_.max_yaw_vel_;
  }
  
  // Add safety margin (20% extra time for yaw control response)
  yaw_time_lb *= 1.2;
  
  ROS_INFO("[FastPlannerManager] Yaw change: %.2f rad (%.1f deg), yaw_time_lb: %.2f s",
           yaw_change, yaw_change * 180.0 / M_PI, yaw_time_lb);
  
  // Use maximum of position time lower bound and yaw time lower bound
  double time_lb = std::max(pp_.time_lower_bound_, yaw_time_lb);
  
  // Plan position trajectory
  planExplorationPositionTraj(path, start_vel, start_acc, time_lb, false);
  ROS_INFO("[FastPlannerManager] Position trajectory planned, duration: %.2f s", local_data_.duration_);
  
  // Plan yaw trajectory (looking forward along the trajectory)
  planYawLookingForward(cur_yaw, cur_yaw_vel);
  ROS_INFO("[FastPlannerManager] Yaw trajectory planned, start: %.2f, end: %.2f", 
           local_data_.start_yaw_, local_data_.end_yaw_);
  return true;
}

bool FastPlannerManager::planTrajectoryWithYaw(const Eigen::Vector3d &start_pos,
                                               const Eigen::Vector3d &start_vel,
                                               const Eigen::Vector3d &start_acc,
                                               const Eigen::Vector3d &goal_pos,
                                               const double &cur_yaw,
                                               const double &cur_yaw_vel,
                                               const double &goal_yaw) {
  // Search path using A*
  int search_result = path_finder_->searchUnknown(start_pos, goal_pos);
  ROS_INFO("[FastPlannerManager] Search result: %d", search_result);
  if (search_result != ROGAstar::REACH_END) {
    ROS_WARN("[FastPlannerManager] Path search failed with code: %d", search_result);
    return false;
  }
  
  std::vector<Eigen::Vector3d> path = path_finder_->getPath();
  ROS_INFO("[FastPlannerManager] Path length: %zu", path.size());
  if (path.size() < 2) {
    ROS_WARN("[FastPlannerManager] Path too short");
    return false;
  }
  
  // Shorten path
  shortenPath(path);
  ROS_INFO("[FastPlannerManager] Shortened path length: %zu", path.size());
  
  path.front() = start_pos;
  path.back() = goal_pos;
  
  // Compute yaw time lower bound using shortest rotation path
  double yaw_change = std::abs(yawDiff(cur_yaw, goal_yaw));
  
  // Calculate yaw time with acceleration consideration (trapezoidal profile)
  double yaw_time_lb;
  double yaw_accel_dist = 0.5 * pp_.max_yaw_vel_ * pp_.max_yaw_vel_ / pp_.max_yaw_acc_;
  if (yaw_change < 2 * yaw_accel_dist) {
    yaw_time_lb = 2.0 * std::sqrt(yaw_change / pp_.max_yaw_acc_);
  } else {
    yaw_time_lb = pp_.max_yaw_vel_ / pp_.max_yaw_acc_ + yaw_change / pp_.max_yaw_vel_;
  }
  yaw_time_lb *= 1.2;  // 20% safety margin
  
  ROS_INFO("[FastPlannerManager] Yaw change: %.2f rad (%.1f deg), yaw_time_lb: %.2f s",
           yaw_change, yaw_change * 180.0 / M_PI, yaw_time_lb);
  
  // Use maximum of position and yaw time lower bound
  double time_lb = std::max(pp_.time_lower_bound_, yaw_time_lb);
  
  // Plan position trajectory
  planExplorationPositionTraj(path, start_vel, start_acc, time_lb, false);
  ROS_INFO("[FastPlannerManager] Position trajectory planned, duration: %.2f s", local_data_.duration_);
  
  // Plan yaw trajectory to specified goal yaw
  planYawToTarget(cur_yaw, cur_yaw_vel, goal_yaw);
  ROS_INFO("[FastPlannerManager] Yaw trajectory planned to target: %.2f -> %.2f", 
           local_data_.start_yaw_, local_data_.end_yaw_);
  return true;
}

void FastPlannerManager::planYawToTarget(const double &cur_yaw, const double &cur_yaw_vel, 
                                         const double &goal_yaw) {
  if (local_data_.position_traj_.getTimeSum() < 0.1) return;
  
  double duration = local_data_.duration_;
  const int seg_num = 12;
  double dt_yaw = duration / seg_num;
  
  // Normalize goal yaw relative to current yaw (find shortest path)
  double start_yaw = cur_yaw;
  double target_yaw = goal_yaw;
  
  // Use yawDiff to find shortest rotation
  double diff = yawDiff(start_yaw, target_yaw);
  target_yaw = start_yaw + diff;
  
  // Generate rate-limited yaw waypoints
  std::vector<Eigen::Vector3d> yaw_waypts;
  std::vector<int> yaw_waypts_idx;
  
  int yaw_dir = (diff > 0) ? 1 : -1;
  double abs_diff = std::abs(diff);
  bool flag_fast_yaw = (abs_diff > pp_.max_yaw_vel_ * dt_yaw * seg_num);
  
  double accumulated_yaw = start_yaw;
  for (int i = 1; i < seg_num; i += 2) {
    double yaw_waypt;
    
    if (!flag_fast_yaw) {
      // Linear interpolation
      yaw_waypt = start_yaw + abs_diff * static_cast<double>(i) / seg_num * yaw_dir;
    } else {
      // Rate-limited
      yaw_waypt = start_yaw + pp_.max_yaw_vel_ * dt_yaw * i * yaw_dir;
    }
    
    // Clamp to goal
    if ((target_yaw - yaw_waypt) * yaw_dir < 0) {
      yaw_waypt = target_yaw;
    }
    
    accumulated_yaw = yaw_waypt;
    yaw_waypts.push_back(Eigen::Vector3d(yaw_waypt, 0, 0));
    yaw_waypts_idx.push_back(i);
  }
  
  ROS_INFO("[FastPlannerManager] Yaw to target: start=%.2f, goal=%.2f, diff=%.2f, waypts=%zu",
           start_yaw, target_yaw, diff, yaw_waypts.size());
  
  planExplorationYawWaypointsTraj(start_yaw, cur_yaw_vel, target_yaw, yaw_waypts, yaw_waypts_idx);
}

void FastPlannerManager::shortenPath(std::vector<Eigen::Vector3d> &path) {
  if (path.size() <= 2) return;
  
  std::vector<Eigen::Vector3d> shortened;
  shortened.push_back(path.front());
  
  for (size_t i = 1; i < path.size() - 1; ++i) {
    Eigen::Vector3d p1 = shortened.back();
    Eigen::Vector3d p2 = path[i + 1];
    
    // Check if we can skip path[i]
    bool can_skip = true;
    Eigen::Vector3d dir = (p2 - p1).normalized();
    double dist = (p2 - p1).norm();
    
    for (double d = 0.1; d < dist; d += 0.1) {
      Eigen::Vector3d check_pt = p1 + d * dir;
      // Use inflated map for path shortening to maintain safety distance
      if (rog_map_->isOccupiedInflate(check_pt)) {
        can_skip = false;
        break;
      }
    }
    
    if (!can_skip) {
      shortened.push_back(path[i]);
    }
  }
  
  shortened.push_back(path.back());
  path = shortened;
}

void FastPlannerManager::planExplorationPositionTraj(const std::vector<Eigen::Vector3d> &tour,
                                                     const Eigen::Vector3d &cur_vel,
                                                     const Eigen::Vector3d &cur_acc,
                                                     const double &time_lb,
                                                     const bool verbose) {
  if (tour.empty()) {
    ROS_ERROR("[FastPlannerManager] Empty path for trajectory planning");
    return;
  }
  
  if (tour.size() < 2) {
    ROS_ERROR("[FastPlannerManager] Path must have at least 2 points, got %zu", tour.size());
    return;
  }
  
  ROS_INFO("[FastPlannerManager] Planning trajectory with %zu waypoints", tour.size());
  
  ros::Time t1 = ros::Time::now();
  
  // Bound current velocity and acceleration
  Eigen::Vector3d cur_vel_bound = cur_vel;
  Eigen::Vector3d cur_acc_bound = cur_acc;
  for (int i = 0; i < 3; ++i) {
    cur_vel_bound(i) = std::min(pp_.max_vel_, std::max(-pp_.max_vel_, cur_vel(i)));
    cur_acc_bound(i) = std::min(pp_.max_acc_, std::max(-pp_.max_acc_, cur_acc(i)));
  }
  
  // Calculate segment times
  const int pt_num = tour.size();
  const int path_seg_num = pt_num - 1;
  ROS_INFO("[FastPlannerManager] Calculating segment times for %d segments", path_seg_num);
  
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) {
    pos.row(i) = tour[i];
  }
  
  // Calculate segment times - simplified and robust approach
  // Use conservative velocity (50% of max) for all segments
  // This ensures smooth trajectory generation and leaves room for B-spline optimization
  Eigen::VectorXd times(path_seg_num);
  double conservative_vel = pp_.max_vel_ * 0.5;  // Use 50% of max velocity
  
  for (int i = 0; i < path_seg_num; ++i) {
    double dist = (pos.row(i + 1) - pos.row(i)).norm();
    
    // Basic time = distance / velocity
    times(i) = dist / conservative_vel;
    
    // Add extra time for first segment if current velocity is significant
    if (i == 0) {
      double cur_speed = cur_vel_bound.norm();
      // If moving fast, need time to transition smoothly
      if (cur_speed > 0.1) {
        // Add acceleration/deceleration time margin
        times(i) += cur_speed / pp_.max_acc_;
      }
    }
    
    // Add deceleration time for last segment to stop at goal
    if (i == path_seg_num - 1) {
      // Time to decelerate from cruise velocity to zero
      times(i) += conservative_vel / pp_.max_acc_;
    }
    
    // Ensure minimum segment time for numerical stability
    times(i) = std::max(times(i), pp_.min_segment_time_);
    
    ROS_DEBUG("[FastPlannerManager] Segment %d: dist=%.2f, time=%.2f", i, dist, times(i));
  }
  
  // Ensure minimum total time (considering yaw time and time_lb)
  double times_sum = times.sum();
  double min_total_time = std::max(pp_.time_lower_bound_, time_lb);
  if (times_sum < min_total_time) {
    times *= (min_total_time / times_sum);
    times_sum = min_total_time;
  }
  
  ROS_INFO("[FastPlannerManager] Total trajectory time: %.2f s", times_sum);
  
  // Generate polynomial trajectory
  ROS_INFO("[FastPlannerManager] Generating polynomial trajectory with %d segments", path_seg_num);
  PolynomialTraj::waypointsTraj(pos, cur_vel_bound, Eigen::Vector3d::Zero(),
                                cur_acc_bound, Eigen::Vector3d::Zero(),
                                times, local_data_.init_traj_poly_);
  ROS_INFO("[FastPlannerManager] Polynomial trajectory planned successfully");
  PolynomialTraj &init_traj = local_data_.init_traj_poly_;
  
  // Convert to B-spline for optimization
  std::vector<Eigen::Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  
  // Ensure minimum duration to avoid numerical issues
  if (duration < 0.1) {
    ROS_WARN("[FastPlannerManager] Trajectory duration too short (%.3f s), extending to 0.5 s", duration);
    duration = 0.5;
  }
  
  int seg_num = std::max(8, (int)(init_traj.getLength() / pp_.ctrl_pt_dist_));
  double dt = duration / double(seg_num);
  
  ROS_INFO("[FastPlannerManager] Converting to B-spline: duration=%.2f s, seg_num=%d, dt=%.3f",
           duration, seg_num, dt);
  
  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt) {
    points.push_back(init_traj.evaluate(std::min(ts, init_traj.getTotalTime()), 0));
  }
  
  // Boundary derivatives (velocity and acceleration at start and end)
  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));
  
  // Parameterize to B-spline
  Eigen::MatrixXd ctrl_pts;
  int bspline_degree = bspline_optimizers_[0]->getBsplineDegree();
  NonUniformBspline::parameterizeToBspline(dt, points, boundary_deri, bspline_degree, ctrl_pts);
  
  ROS_INFO("[FastPlannerManager] B-spline parameterized: %ld control points, degree=%d",
           ctrl_pts.rows(), bspline_degree);
  
  local_data_.init_traj_bspline_ = NonUniformBspline(ctrl_pts, bspline_degree, dt);
  
  // Optimize B-spline
  int cost_func = BsplineOptimizer::EXPLORATION_POSITION_PHASE;
  
  std::vector<Eigen::Vector3d> start, end;
  local_data_.init_traj_bspline_.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) {
    bspline_optimizers_[0]->setTimeLowerBound(time_lb);
  }
  
  ROS_INFO("[FastPlannerManager] Starting B-spline optimization...");
  
  // Run optimization
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func);
  
  ROS_INFO("[FastPlannerManager] B-spline optimization completed, dt=%.3f", dt);
  
  // Store optimized trajectory
  local_data_.position_traj_.setUniformBspline(ctrl_pts, bspline_degree, dt);
  
  // Set physical limits and reallocate time to ensure feasibility
  local_data_.position_traj_.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, 1.5);
  bool feasible = local_data_.position_traj_.checkFeasibility(false);
  if (!feasible) {
    ROS_WARN("[FastPlannerManager] Trajectory infeasible, reallocating time...");
    local_data_.position_traj_.reallocateTime(false);
  }
  
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();
  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.end_pos_ = local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_);
  
  local_data_.traj_id_++;
  local_data_.tour_ = tour;
  local_data_.cur_vel_ = cur_vel;
  local_data_.cur_acc_ = cur_acc;
  local_data_.time_lb_ = time_lb;
  
  double t_total = (ros::Time::now() - t1).toSec();
  ROS_INFO("[FastPlannerManager] Position trajectory planned in %.1f ms, duration: %.2f s",
           t_total * 1000, local_data_.duration_);
}

void FastPlannerManager::planExplorationYawWaypointsTraj(
    const double &cur_yaw, const double &cur_yaw_vel, const double &goal_yaw,
    const std::vector<Eigen::Vector3d> &yaw_waypts,
    const std::vector<int> &yaw_waypts_idx) {
  ros::Time t1 = ros::Time::now();
  
  const int seg_num = 12;
  double dt_yaw = local_data_.duration_ / seg_num;
  Eigen::Vector3d start_yaw3d(cur_yaw, cur_yaw_vel, 0);
  Eigen::Vector3d end_yaw3d(goal_yaw, 0, 0);
  
  // Yaw trajectory control points
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();
  
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw,
                1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
                1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  
  // Initial and final state
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;
  
  // Optimize yaw trajectory
  int cost_func = BsplineOptimizer::EXPLORATION_YAW_PHASE;
  
  std::vector<Eigen::Vector3d> start = {Eigen::Vector3d(start_yaw3d[0], start_yaw3d[1], start_yaw3d[2]),
                                        Eigen::Vector3d(cur_yaw_vel, 0, 0),
                                        Eigen::Vector3d(0, 0, 0)};
  std::vector<Eigen::Vector3d> end = {Eigen::Vector3d(end_yaw3d[0], 0, 0),
                                      Eigen::Vector3d(0, 0, 0)};
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->setWaypoints(yaw_waypts, yaw_waypts_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func);
  
  // Store yaw trajectory
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  
  local_data_.start_yaw_ = local_data_.yaw_traj_.evaluateDeBoorT(0.0)[0];
  local_data_.end_yaw_ = local_data_.yaw_traj_.evaluateDeBoorT(local_data_.duration_)[0];
  
  local_data_.cur_yaw_ = cur_yaw;
  local_data_.cur_yaw_vel_ = cur_yaw_vel;
  local_data_.goal_yaw_ = goal_yaw;
  local_data_.yaw_waypts_ = yaw_waypts;
  local_data_.yaw_waypts_idx_ = yaw_waypts_idx;
  
  double t_total = (ros::Time::now() - t1).toSec();
  ROS_INFO("[FastPlannerManager] Yaw trajectory planned in %.1f ms", t_total * 1000);
}

void FastPlannerManager::planYawLookingForward(const double &cur_yaw, const double &cur_yaw_vel) {
  // Strategy: Look along the velocity direction at each point
  // Key insight: Complete most of the yaw turn BEFORE significant position movement
  
  if (local_data_.position_traj_.getTimeSum() < 0.1) return;
  
  double duration = local_data_.duration_;
  const int seg_num = 12;
  double dt_yaw = duration / seg_num;
  
  // Step 1: Sample velocity directions along the trajectory
  std::vector<double> sampled_yaws;
  sampled_yaws.push_back(cur_yaw);  // Start with current yaw
  
  for (int i = 1; i <= seg_num; ++i) {
    double t = i * dt_yaw;
    if (t > duration) t = duration;
    
    Eigen::Vector3d vel = local_data_.velocity_traj_.evaluateDeBoorT(t);
    double vel_xy_norm = vel.head<2>().norm();
    
    double target_yaw;
    if (vel_xy_norm > 0.2) {
      // Use velocity direction
      target_yaw = std::atan2(vel.y(), vel.x());
    } else {
      // Low velocity - keep previous yaw
      target_yaw = sampled_yaws.back();
    }
    
    // Apply rate limiting from previous yaw
    double prev_yaw = sampled_yaws.back();
    double diff = yawDiff(prev_yaw, target_yaw);
    double max_change = pp_.max_yaw_vel_ * dt_yaw;
    
    if (std::abs(diff) > max_change) {
      // Rate limit
      target_yaw = prev_yaw + (diff > 0 ? max_change : -max_change);
    } else {
      // Smooth transition
      target_yaw = prev_yaw + diff;
    }
    
    sampled_yaws.push_back(target_yaw);
  }
  
  // Step 2: Compute initial target yaw (direction at trajectory start)
  Eigen::Vector3d init_vel = local_data_.velocity_traj_.evaluateDeBoorT(0.0);
  double init_target_yaw = cur_yaw;
  if (init_vel.head<2>().norm() > 0.2) {
    init_target_yaw = std::atan2(init_vel.y(), init_vel.x());
  } else {
    // Use direction to first waypoint
    Eigen::Vector3d start_pos = local_data_.position_traj_.evaluateDeBoorT(0.0);
    Eigen::Vector3d next_pos = local_data_.position_traj_.evaluateDeBoorT(duration * 0.3);
    Eigen::Vector3d dir = next_pos - start_pos;
    if (dir.head<2>().norm() > 0.3) {
      init_target_yaw = std::atan2(dir.y(), dir.x());
    }
  }
  
  // Step 3: Check if we need a "pre-turn" phase
  double init_yaw_diff = std::abs(yawDiff(cur_yaw, init_target_yaw));
  double pre_turn_ratio = 0.0;  // Portion of trajectory for pre-turning
  
  if (init_yaw_diff > M_PI / 4) {  // More than 45 degrees difference
    // Dedicate first 30% of trajectory for turning
    pre_turn_ratio = 0.3;
    ROS_INFO("[FastPlannerManager] Pre-turn needed: yaw_diff=%.2f rad (%.1f deg)",
             init_yaw_diff, init_yaw_diff * 180.0 / M_PI);
  }
  
  // Step 4: Generate yaw waypoints with pre-turn consideration
  std::vector<Eigen::Vector3d> yaw_waypts;
  std::vector<int> yaw_waypts_idx;
  
  double accumulated_yaw = cur_yaw;
  int pre_turn_segs = static_cast<int>(seg_num * pre_turn_ratio);
  
  for (int i = 1; i < seg_num; i += 2) {
    double target_yaw;
    
    if (i <= pre_turn_segs && pre_turn_segs > 0) {
      // Pre-turn phase: interpolate towards initial velocity direction
      double progress = static_cast<double>(i) / pre_turn_segs;
      double diff = yawDiff(cur_yaw, init_target_yaw);
      target_yaw = cur_yaw + diff * progress;
    } else {
      // Normal phase: use sampled velocity-based yaw
      target_yaw = sampled_yaws[std::min(i, static_cast<int>(sampled_yaws.size()) - 1)];
    }
    
    // Rate limiting
    double diff = yawDiff(accumulated_yaw, target_yaw);
    double max_change = pp_.max_yaw_vel_ * dt_yaw * 2;  // *2 because we skip every other segment
    
    if (std::abs(diff) > max_change) {
      target_yaw = accumulated_yaw + (diff > 0 ? max_change : -max_change);
    } else {
      target_yaw = accumulated_yaw + diff;
    }
    
    accumulated_yaw = target_yaw;
    yaw_waypts.push_back(Eigen::Vector3d(target_yaw, 0, 0));
    yaw_waypts_idx.push_back(i);
  }
  
  // Step 5: Compute goal yaw (final velocity direction)
  Eigen::Vector3d end_vel = local_data_.velocity_traj_.evaluateDeBoorT(duration * 0.9);
  double goal_yaw = accumulated_yaw;
  if (end_vel.head<2>().norm() > 0.2) {
    goal_yaw = std::atan2(end_vel.y(), end_vel.x());
    // Unwrap goal_yaw relative to accumulated_yaw
    double diff = yawDiff(accumulated_yaw, goal_yaw);
    goal_yaw = accumulated_yaw + diff;
  }
  
  // Normalize start_yaw to be continuous
  double start_yaw = cur_yaw;
  
  ROS_INFO("[FastPlannerManager] Yaw plan: start=%.2f->%.2f, waypts=%zu, pre_turn=%d segs",
           start_yaw, goal_yaw, yaw_waypts.size(), pre_turn_segs);
  
  // Step 6: Call B-spline yaw trajectory planner
  planExplorationYawWaypointsTraj(start_yaw, cur_yaw_vel, goal_yaw, yaw_waypts, yaw_waypts_idx);
}

void FastPlannerManager::calcNextYaw(const double &last_yaw, double &yaw) {
  // Round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) round_last += 2 * M_PI;
  while (round_last > M_PI) round_last -= 2 * M_PI;
  
  double diff = yaw - round_last;
  if (std::abs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

bool FastPlannerManager::checkTrajCollision() {
  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();
  
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;
  
  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
    // Use inflated map for collision checking to maintain safety distance
    if (rog_map_->isOccupiedInflate(fut_pt)) {
      ROS_ERROR("[FastPlannerManager] Collision detected at (%.2f, %.2f, %.2f)",
                fut_pt.x(), fut_pt.y(), fut_pt.z());
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }
  
  return true;
}

} // namespace fast_planner


