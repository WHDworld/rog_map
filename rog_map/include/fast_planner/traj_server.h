#ifndef _TRAJ_SERVER_H_
#define _TRAJ_SERVER_H_

#include <ros/ros.h>
#include <memory>

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

#include "fast_planner/bspline/non_uniform_bspline.h"
#include "rog_map/Bspline.h"

namespace fast_planner {

class TrajServer {
public:
  typedef std::shared_ptr<TrajServer> Ptr;
  
  TrajServer() {}
  ~TrajServer() {}
  
  void init(ros::NodeHandle &nh);
  
  // Set trajectory externally
  void setTrajectory(const std::vector<NonUniformBspline> &traj,
                     const ros::Time &start_time,
                     int traj_id);
  
  // Check if trajectory is valid
  bool hasValidTrajectory() const { return receive_traj_; }
  
  // Get current trajectory time
  double getCurrentTime() const;
  
  // Get trajectory duration
  double getDuration() const { return traj_duration_; }
  
private:
  // Trajectory data
  std::vector<NonUniformBspline> traj_;
  std::vector<NonUniformBspline> prev_traj_;  // Previous trajectory for smooth transition
  double traj_duration_ = 0.0;
  double prev_traj_duration_ = 0.0;
  ros::Time start_time_;
  ros::Time prev_start_time_;
  int traj_id_ = 0;
  bool receive_traj_ = false;
  bool task_finished_ = false;
  bool have_prev_traj_ = false;  // Whether we have a previous trajectory
  
  // Executed trajectory for visualization
  std::vector<Eigen::Vector3d> traj_cmd_;
  std::vector<Eigen::Vector3d> vel_cmd_;
  
  // ROS
  ros::NodeHandle nh_;
  ros::Timer cmd_timer_, vis_timer_;
  ros::Subscriber bspline_sub_, replan_sub_;
  ros::Publisher pos_cmd_pub_, cmd_vis_pub_, traj_pub_, traj_vel_pub_;
  
  // Command msg
  quadrotor_msgs::PositionCommand cmd_;
  
  // Initial position
  Eigen::Vector3d init_pos_;
  double init_yaw_ = 0.0;
  
  // Parameters
  double cmd_rate_ = 100.0;
  
  // Yaw tracking for smooth transitions
  double last_yaw_ = 0.0;
  int last_traj_id_ = -1;
  bool yaw_initialized_ = false;
  
  // Callbacks
  void cmdCallback(const ros::TimerEvent &e);
  void visCallback(const ros::TimerEvent &e);
  void bsplineCallback(const rog_map::BsplineConstPtr &msg);
  void replanCallback(const std_msgs::Int32ConstPtr &msg);
  
  // Visualization
  void displayTraj();
  void drawFOV(const Eigen::Vector3d &pos, double yaw);
};

} // namespace fast_planner

#endif

