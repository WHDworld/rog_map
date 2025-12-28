#ifndef _PLANNING_FSM_H_
#define _PLANNING_FSM_H_

#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "fast_planner/planner_manager.h"
#include "rog_map/Bspline.h"

namespace fast_planner {

enum FSM_STATE {
  INIT,
  PRE_ARM,
  TAKEOFF,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  REPLAN_TRAJ,
  EXEC_TRAJ,
  EMERGENCY_STOP
};

struct FSMParam {
  // Replan thresholds
  double replan_thresh1 = 1.0;   // Remaining time threshold
  double replan_thresh2 = 2.0;   // Distance threshold
  double replan_thresh3 = 3.0;   // Periodic replan time
  double replan_duration = 0.3;  // Replan look-ahead time
  
  // PX4 control
  bool real_world_flag = false;
  double takeoff_height = 1.0;
  double takeoff_threshold = 0.1;
  int pre_arm_setpoint_count = 100;
  double service_timeout = 1.0;
  
  // Takeoff trajectory
  double takeoff_accel_time = 2.0;
  double takeoff_cruise_time = 0.5;
  double takeoff_decel_time = 2.0;
  double takeoff_max_vel = 0.5;
  double takeoff_max_accel = 0.25;
  
  // Target type: 1=manual, 2=preset waypoints, 3=API
  int target_type = 1;
  bool auto_start_preset = false;
  
  // Safety
  bool enable_fail_safe = true;
  double emergency_time = 0.8;
  double heartbeat_timeout = 0.5;
  
  // Dynamics
  double max_vel = 2.0;
  double max_acc = 2.0;
};

struct FSMData {
  bool have_odom = false;
  bool have_target = false;
  bool triggered = false;
  bool static_state = true;
  
  Eigen::Vector3d odom_pos;
  Eigen::Vector3d odom_vel;
  Eigen::Quaterniond odom_orient;
  double odom_yaw = 0.0;
  double odom_yaw_vel = 0.0;  // Yaw angular velocity from odometry
  
  Eigen::Vector3d start_pos, start_vel, start_acc;
  Eigen::Vector3d start_yaw;
  
  Eigen::Vector3d target_pos;
  double target_yaw = 0.0;
  
  std::vector<std::string> state_str = {
    "INIT", "PRE_ARM", "TAKEOFF", "WAIT_TARGET",
    "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"
  };
};

class PlanningFSM {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<PlanningFSM> Ptr;
  
  PlanningFSM() {}
  ~PlanningFSM() {}
  
  void init(ros::NodeHandle &nh, const rog_map::ROGMap::Ptr &map);
  
private:
  // Modules
  FastPlannerManager::Ptr planner_manager_;
  rog_map::ROGMap::Ptr rog_map_;
  
  // Parameters and data
  FSMParam fp_;
  FSMData fd_;
  FSM_STATE state_;
  
  // Preset waypoints
  std::vector<Eigen::Vector3d> preset_waypoints_;
  int current_waypoint_idx_ = 0;
  
  // ROS
  ros::NodeHandle nh_;
  ros::Timer exec_timer_, safety_timer_;
  ros::Subscriber odom_sub_, target_sub_, trigger_sub_, mavros_state_sub_;
  ros::Publisher replan_pub_, bspline_pub_, setpoint_pub_, pos_cmd_pub_;
  ros::ServiceClient arming_client_, set_mode_client_;
  
  ros::Publisher set_target_point_pub_;
  // PX4 control state
  mavros_msgs::State mavros_state_;
  bool is_connected_ = false;
  bool is_armed_ = false;
  bool is_offboard_ = false;
  int setpoint_count_ = 0;
  ros::Time last_arm_request_, last_mode_request_;
  
  // Takeoff state
  Eigen::Vector3d initial_pos_;
  double initial_yaw_ = 0.0;
  bool initial_pose_recorded_ = false;
  bool takeoff_complete_ = false;
  bool takeoff_trajectory_initialized_ = false;
  ros::Time takeoff_start_time_;
  double trajectory_z_ = 0.0;
  double trajectory_vel_z_ = 0.0;
  double trajectory_acc_z_ = 0.0;
  
  // Hover state
  Eigen::Vector3d hover_pos_;
  double hover_yaw_ = 0.0;
  bool hover_pos_set_ = false;
  
  // Planning state
  int plan_fail_count_ = 0;
  ros::Time trajectory_start_time_;
  
  // Callbacks
  void execFSMCallback(const ros::TimerEvent &e);
  void safetyCallback(const ros::TimerEvent &e);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void targetCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void triggerCallback(const std_msgs::EmptyConstPtr &msg);
  void mavrosStateCallback(const mavros_msgs::StateConstPtr &msg);
  
  // FSM functions
  void transitState(FSM_STATE new_state, const std::string &caller);
  void printState();
  
  // Planning functions
  bool callPlanningOnce();
  bool planFromCurrentState();
  
  // PX4 control functions
  void sendInitialSetpoints();
  bool armAndOffboard();
  void updateTakeoffTrajectory();
  bool checkTakeoffComplete();
  
  // Command publishing
  void publishTakeoffCmd();
  void publishHoverCmd();
  void publishSetpoint(const Eigen::Vector3d &pos, double yaw);
  void publishBsplineTraj();
  
  // Helper functions
  void loadPresetWaypoints();
  bool setNextWaypoint();

};

} // namespace fast_planner

#endif

