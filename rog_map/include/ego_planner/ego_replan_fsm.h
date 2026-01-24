#ifndef _EGO_REPLAN_FSM_H_
#define _EGO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <limits>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include "ego_planner/ego_planner_manager.h"
#include "rog_map/PolyTraj.h"
// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    void init(ros::NodeHandle &nh, star_mvf::MapManager::Ptr map);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- FSM states ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      PRE_ARM,
      TAKEOFF,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2
    };
    /* Simulation mode: determines FCU/arming behavior */
    enum SIM_MODE
    {
      SIM_GAZEBO = 0,   // Full FCU/MAVROS integration (Gazebo + PX4)
      SIM_OPEN3D = 1,   // No FCU, direct position command (Open3D simulator)
      SIM_REAL = 2      // Real flight: wait for manual arming, then auto-takeoff
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    star_mvf::MapManager::Ptr map_manager_;

    /* FSM parameters */
    int target_type_;
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    double waypoints_yaw_[50];  // 每个目标点的yaw，NaN表示跟随速度方向
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    bool enable_fail_safe_;
    
    /* PX4 control parameters */
    double takeoff_height_;
    double takeoff_accel_time_;
    double replan_duration_;
    bool auto_start_preset_;
    int sim_mode_;  // SIM_GAZEBO=0, SIM_OPEN3D=1, SIM_REAL=2

    /* state flags */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    /* planning states */
    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d end_pt_, end_vel_;                   // goal state (position and velocity)
    double end_yaw_;                                     // goal yaw
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    Eigen::Quaterniond odom_orient_;
    double odom_yaw_, odom_yaw_vel_;
    
    /* waypoints with full state (pos, yaw, vel) */
    struct Waypoint {
      Eigen::Vector3d pos;
      double yaw;
      Eigen::Vector3d vel;  // desired velocity at this waypoint (typically for end)
      Waypoint() : pos(Eigen::Vector3d::Zero()), yaw(0.0), vel(Eigen::Vector3d::Zero()) {}
      Waypoint(const Eigen::Vector3d& p, double y, const Eigen::Vector3d& v = Eigen::Vector3d::Zero()) 
        : pos(p), yaw(y), vel(v) {}
    };
    std::vector<Waypoint> wps_;
    
    /* hover state */
    Eigen::Vector3d hover_pos_;
    double hover_yaw_;
    bool hover_pos_set_;
    
    /* PX4 state */
    mavros_msgs::State current_state_;
    Eigen::Vector3d initial_pos_;
    bool offboard_ready_;
    ros::Time takeoff_start_time_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, viewpoint_sub_, viewpoint_path_sub_, odom_sub_, trigger_sub_, mandatory_stop_sub_, state_sub_;
    ros::Publisher pos_cmd_pub_, replan_pub_;
    ros::ServiceClient arming_client_, set_mode_client_;
    ros::Publisher set_target_point_pub_;
    ros::Publisher traj_vis_pub_;  // 轨迹可视化（MarkerArray：轨迹+终点yaw+实时yaw）
    
    /* 轨迹可视化 */
    void visualizeTrajectory();
    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMExecState();
    std::pair<int, FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void viewpointCallback(const quadrotor_msgs::PositionCommandConstPtr &msg);  // Full viewpoint with pos, yaw, vel
    void viewpointPathCallback(const nav_msgs::PathConstPtr &msg);  // Path with intermediate waypoints
    void readGivenWpsAndPlan();
    void planNextWaypoint(const Waypoint& next_wp, const Waypoint& previous_wp);

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void polyTraj2ROSMsg(rog_map::PolyTraj &poly_msg);
    
    /* PX4 control */
    void publishTakeoffCmd();
    void publishHoverCmd();
    
    /* 轨迹执行（独立定时器） */
    ros::Timer execution_timer_;
    double traj_start_time_, traj_duration_;
    Eigen::Vector3d last_pos_;
    double last_yaw_, last_yaw_dot_;
    
    /* 旧轨迹保存（用于衔接） */
    bool has_prev_traj_;
    double prev_traj_start_time_, prev_traj_duration_;
    poly_traj::Trajectory prev_traj_;
    std::vector<double> prev_yaw_ctrl_pts_;
    bool prev_use_yaw_traj_;
    double prev_last_yaw_;
    
    void executionCallback(const ros::TimerEvent &e);
    void publishCmd(const Eigen::Vector3d& p, const Eigen::Vector3d& v, 
                    const Eigen::Vector3d& a, double yaw, double yaw_dot);
    void savePrevTrajectory();
  };

} // namespace ego_planner

#endif
