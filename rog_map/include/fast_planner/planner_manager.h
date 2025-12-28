#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <ros/ros.h>
#include <memory>
#include <vector>

#include <Eigen/Eigen>

#include "fast_planner/bspline/bspline_optimizer.h"
#include "fast_planner/bspline/non_uniform_bspline.h"
#include "fast_planner/polynomial/polynomial_traj.h"
#include "rog_map/rog_map.h"

namespace fast_planner {

// A* path finder adapted for ROGMap
class ROGAstar {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ROGAstar> Ptr;
  
  enum { REACH_END = 1, NO_PATH = 2, TIMEOUT = 3 };
  
  struct Config {
    double resolution = 0.1;
    double max_search_time = 0.5;
    double lambda_heuristic = 1.0;
    int allocate_num = 100000;
    bool unknown_as_free = true;
    bool use_inflated_map = true;  // Use inflated map for collision detection
    bool verbose = false;
  };
  
  ROGAstar() {}
  ~ROGAstar();
  
  void init(ros::NodeHandle &nh, const rog_map::ROGMap::Ptr &map);
  void reset();
  
  // Search path from start to end
  int search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
  int searchUnknown(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
  
  std::vector<Eigen::Vector3d> getPath() { return path_nodes_; }
  double getPathLength();
  
  void setResolution(double res) { config_.resolution = res; }
  void setMaxSearchTime(double t) { config_.max_search_time = t; }
  
private:
  struct AstarNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3d position;
    Eigen::Vector3i index;
    double g_score = 0.0;
    double f_score = 0.0;
    AstarNode* parent = nullptr;
    
    AstarNode() = default;
  };
  
  struct NodeComparator {
    bool operator()(const AstarNode* a, const AstarNode* b) const {
      return a->f_score > b->f_score;
    }
  };
  
  void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx);
  void indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3d &pos);
  double getHeuristic(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);
  bool isOccupied(const Eigen::Vector3d &pos);
  void backtrack(AstarNode* end_node);
  
  Config config_;
  rog_map::ROGMap::Ptr rog_map_;
  
  // Use Eigen aligned allocator for proper memory alignment
  std::vector<AstarNode*, Eigen::aligned_allocator<AstarNode*>> node_pool_;
  int use_node_num_ = 0;
  std::vector<Eigen::Vector3d> path_nodes_;
};

struct PlanParameters {
  double max_vel_ = 2.0;
  double max_acc_ = 2.0;
  double max_yaw_vel_ = 1.0472;  // 60 deg/s
  double max_yaw_acc_ = 1.0472;  // 60 deg/s^2
  double ctrl_pt_dist_ = 0.3;
  double time_lower_bound_ = 0.5;
  double min_segment_time_ = 0.3;
};

struct LocalTrajData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  int traj_id_ = 0;
  double duration_ = 0.0;
  ros::Time start_time_;
  Eigen::Vector3d start_pos_, end_pos_;
  double start_yaw_ = 0.0, end_yaw_ = 0.0;
  
  PolynomialTraj init_traj_poly_;
  NonUniformBspline init_traj_bspline_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  
  NonUniformBspline yaw_traj_, yawdot_traj_, yawdotdot_traj_;
  
  // Bspline optimization input
  std::vector<Eigen::Vector3d> tour_;
  Eigen::Vector3d cur_vel_, cur_acc_;
  double time_lb_ = 0.0;
  double cur_yaw_ = 0.0, cur_yaw_vel_ = 0.0, goal_yaw_ = 0.0;
  std::vector<Eigen::Vector3d> yaw_waypts_;
  std::vector<int> yaw_waypts_idx_;
};

class FastPlannerManager {
public:
  typedef std::shared_ptr<FastPlannerManager> Ptr;
  typedef std::shared_ptr<const FastPlannerManager> ConstPtr;
  
  FastPlannerManager() {}
  ~FastPlannerManager() {}
  
  void init(ros::NodeHandle &nh, const rog_map::ROGMap::Ptr &map);
  
  // Plan trajectory from current state to goal (yaw follows velocity direction)
  bool planTrajectory(const Eigen::Vector3d &start_pos,
                      const Eigen::Vector3d &start_vel,
                      const Eigen::Vector3d &start_acc,
                      const Eigen::Vector3d &goal_pos,
                      const double &cur_yaw = 0.0,
                      const double &cur_yaw_vel = 0.0);
  
  // Plan trajectory with specified goal yaw (for exploration)
  bool planTrajectoryWithYaw(const Eigen::Vector3d &start_pos,
                             const Eigen::Vector3d &start_vel,
                             const Eigen::Vector3d &start_acc,
                             const Eigen::Vector3d &goal_pos,
                             const double &cur_yaw,
                             const double &cur_yaw_vel,
                             const double &goal_yaw);
  
  // Plan yaw to a specific target (rotate in place)
  void planYawToTarget(const double &cur_yaw, const double &cur_yaw_vel, const double &goal_yaw);
  
  // Plan trajectory along a tour of waypoints
  void planExplorationPositionTraj(const std::vector<Eigen::Vector3d> &tour,
                                   const Eigen::Vector3d &cur_vel,
                                   const Eigen::Vector3d &cur_acc,
                                   const double &time_lb,
                                   const bool verbose = false);
  
  // Plan yaw trajectory
  void planExplorationYawWaypointsTraj(const double &cur_yaw, const double &cur_yaw_vel,
                                       const double &goal_yaw,
                                       const std::vector<Eigen::Vector3d> &yaw_waypts,
                                       const std::vector<int> &yaw_waypts_idx);
  
  // Plan yaw trajectory looking forward
  void planYawLookingForward(const double &cur_yaw, const double &cur_yaw_vel);
  
  // Check trajectory collision
  bool checkTrajCollision();
  
  // Get local trajectory data
  LocalTrajData& getLocalData() { return local_data_; }
  
  // Get parameters
  PlanParameters& getParams() { return pp_; }
  
  // Get path finder
  ROGAstar::Ptr getPathFinder() { return path_finder_; }
  
  PlanParameters pp_;
  LocalTrajData local_data_;
  ROGAstar::Ptr path_finder_;
  rog_map::ROGMap::Ptr rog_map_;
  
private:
  void calcNextYaw(const double &last_yaw, double &yaw);
  void shortenPath(std::vector<Eigen::Vector3d> &path);
  
  std::vector<BsplineOptimizer::Ptr> bspline_optimizers_;
};

} // namespace fast_planner

#endif

