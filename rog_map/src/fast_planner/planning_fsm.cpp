#include "fast_planner/planning_fsm.h"
#include <tf/tf.h>

namespace fast_planner {

void PlanningFSM::init(ros::NodeHandle &nh, const rog_map::ROGMap::Ptr &map) {
  nh_ = nh;
  rog_map_ = map;
  
  // Load FSM parameters
  nh.param("fsm/real_world_flag", fp_.real_world_flag, false);
  nh.param("fsm/takeoff_height", fp_.takeoff_height, 1.0);
  nh.param("fsm/takeoff_threshold", fp_.takeoff_threshold, 0.1);
  nh.param("fsm/pre_arm_setpoint_count", fp_.pre_arm_setpoint_count, 100);
  nh.param("fsm/service_timeout", fp_.service_timeout, 1.0);
  
  nh.param("fsm/takeoff_accel_time", fp_.takeoff_accel_time, 2.0);
  nh.param("fsm/takeoff_cruise_time", fp_.takeoff_cruise_time, 0.5);
  nh.param("fsm/takeoff_decel_time", fp_.takeoff_decel_time, 2.0);
  nh.param("fsm/takeoff_max_vel", fp_.takeoff_max_vel, 0.5);
  nh.param("fsm/takeoff_max_accel", fp_.takeoff_max_accel, 0.25);
  
  nh.param("fsm/target_type", fp_.target_type, 1);
  nh.param("fsm/auto_start_preset", fp_.auto_start_preset, false);
  
  nh.param("fsm/replan_thresh1", fp_.replan_thresh1, 1.0);
  nh.param("fsm/replan_thresh2", fp_.replan_thresh2, 2.0);
  nh.param("fsm/replan_thresh3", fp_.replan_thresh3, 3.0);
  nh.param("fsm/replan_duration", fp_.replan_duration, 0.3);
  
  nh.param("fsm/enable_fail_safe", fp_.enable_fail_safe, true);
  nh.param("fsm/emergency_time", fp_.emergency_time, 0.8);
  nh.param("fsm/heartbeat_timeout", fp_.heartbeat_timeout, 0.5);
  
  nh.param("fsm/max_vel", fp_.max_vel, 2.0);
  nh.param("fsm/max_acc", fp_.max_acc, 2.0);
  
  // Initialize planner manager
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->init(nh, rog_map_);
  
  // Load preset waypoints
  loadPresetWaypoints();
  
  // Initialize state
  state_ = FSM_STATE::INIT;
  
  // ROS subscribers
  odom_sub_ = nh.subscribe("odom", 10, &PlanningFSM::odometryCallback, this);
  target_sub_ = nh.subscribe("/move_base_simple/goal", 1, &PlanningFSM::targetCallback, this);
  trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &PlanningFSM::triggerCallback, this);
  mavros_state_sub_ = nh.subscribe("/mavros/state", 10, &PlanningFSM::mavrosStateCallback, this);
  
  // ROS publishers
  replan_pub_ = nh.advertise<std_msgs::Int32>("/planning/replan", 10);
  bspline_pub_ = nh.advertise<rog_map::Bspline>("/planning/bspline", 10);
  setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
  
  set_target_point_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/set_target_point", 10);
  // ROS service clients
  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  

  // Timers
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &PlanningFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &PlanningFSM::safetyCallback, this);
  
  last_arm_request_ = ros::Time(0);
  last_mode_request_ = ros::Time(0);
  
  ROS_INFO("[PlanningFSM] Initialized. real_world: %s, takeoff_height: %.2f, target_type: %d",
           fp_.real_world_flag ? "true" : "false", fp_.takeoff_height, fp_.target_type);
}

void PlanningFSM::loadPresetWaypoints() {
  preset_waypoints_.clear();
  
  int waypoint_num = 0;
  nh_.param("fsm/waypoint_num", waypoint_num, 0);
  
  for (int i = 0; i < waypoint_num; ++i) {
    double x, y, z;
    nh_.param("fsm/waypoint" + std::to_string(i) + "_x", x, 0.0);
    nh_.param("fsm/waypoint" + std::to_string(i) + "_y", y, 0.0);
    nh_.param("fsm/waypoint" + std::to_string(i) + "_z", z, 1.0);
    preset_waypoints_.push_back(Eigen::Vector3d(x, y, z));
    ROS_INFO("[PlanningFSM] Loaded waypoint %d: (%.2f, %.2f, %.2f)", i, x, y, z);
  }
  
  current_waypoint_idx_ = 0;
}

bool PlanningFSM::setNextWaypoint() {
  if (current_waypoint_idx_ >= preset_waypoints_.size()) {
    ROS_INFO("[PlanningFSM] All preset waypoints completed");
    return false;
  }
  
  fd_.target_pos = preset_waypoints_[current_waypoint_idx_];
  fd_.have_target = true;
  current_waypoint_idx_++;
  // 可视化目标点
  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = ros::Time::now();
  goal_marker.ns = "goal";
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.pose.position.x = fd_.target_pos.x();
  goal_marker.pose.position.y = fd_.target_pos.y();
  goal_marker.pose.position.z = fd_.target_pos.z();
  goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.5;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 0.8;
  set_target_point_pub_.publish(goal_marker);

  ROS_INFO("[PlanningFSM] Set waypoint %d: (%.2f, %.2f, %.2f)",
           current_waypoint_idx_, fd_.target_pos.x(), fd_.target_pos.y(), fd_.target_pos.z());
  return true;
}

void PlanningFSM::transitState(FSM_STATE new_state, const std::string &caller) {
  ROS_INFO("[PlanningFSM] State: %s -> %s (by %s)",
           fd_.state_str[static_cast<int>(state_)].c_str(),
           fd_.state_str[static_cast<int>(new_state)].c_str(),
           caller.c_str());
  state_ = new_state;
}

void PlanningFSM::printState() {
  ROS_INFO_THROTTLE(5.0, "[PlanningFSM] State: %s", fd_.state_str[static_cast<int>(state_)].c_str());
}

void PlanningFSM::execFSMCallback(const ros::TimerEvent &e) {
  switch (state_) {
    case FSM_STATE::INIT: {
      // Wait for odometry
      if (!fd_.have_odom) {
        ROS_INFO_THROTTLE(1.0, "[PlanningFSM] Waiting for odometry...");
        return;
      }
      
      // Record initial pose
      if (!initial_pose_recorded_) {
        initial_pos_ = fd_.odom_pos;
        initial_yaw_ = fd_.odom_yaw;
        initial_pose_recorded_ = true;
        ROS_INFO("[PlanningFSM] Initial pose: (%.2f, %.2f, %.2f), yaw: %.2f deg",
                 initial_pos_.x(), initial_pos_.y(), initial_pos_.z(),
                 initial_yaw_ * 180.0 / M_PI);
      }
      
      if (fp_.real_world_flag) {
        // Real world: wait for manual arm and offboard
        ROS_INFO_THROTTLE(1.0, "[PlanningFSM] Real world mode: waiting for manual arm/offboard");
        if (is_armed_ && is_offboard_) {
          transitState(FSM_STATE::TAKEOFF, "execFSM");
        }
      } else {
        // Simulation: auto arm and offboard
        transitState(FSM_STATE::PRE_ARM, "execFSM");
      }
      break;
    }
    
    case FSM_STATE::PRE_ARM: {
      sendInitialSetpoints();
      
      if (setpoint_count_ >= fp_.pre_arm_setpoint_count) {
        if (armAndOffboard()) {
          transitState(FSM_STATE::TAKEOFF, "execFSM");
        }
      }
      break;
    }
    
    case FSM_STATE::TAKEOFF: {
      if (!takeoff_trajectory_initialized_) {
        takeoff_start_time_ = ros::Time::now();
        trajectory_z_ = initial_pos_.z();
        trajectory_vel_z_ = 0.0;
        trajectory_acc_z_ = 0.0;
        takeoff_trajectory_initialized_ = true;
        ROS_INFO("[PlanningFSM] Takeoff trajectory initialized, target: %.2f m", fp_.takeoff_height);
      }
      
      updateTakeoffTrajectory();
      publishTakeoffCmd();
      
      if (checkTakeoffComplete()) {
        takeoff_complete_ = true;
        
        // Set hover position
        hover_pos_ = Eigen::Vector3d(initial_pos_.x(), initial_pos_.y(), fp_.takeoff_height);
        hover_yaw_ = initial_yaw_;
        hover_pos_set_ = true;
        
        ROS_INFO("[PlanningFSM] Takeoff complete! Hovering at (%.2f, %.2f, %.2f), yaw=%.2f",
                 hover_pos_.x(), hover_pos_.y(), hover_pos_.z(), hover_yaw_);
        
        // Auto start preset waypoints
        if (fp_.target_type == 2 && fp_.auto_start_preset && preset_waypoints_.size() > 0) {
          current_waypoint_idx_ = 0;
          if (setNextWaypoint()) {
            fd_.triggered = true;
            transitState(FSM_STATE::GEN_NEW_TRAJ, "execFSM");
            return;
          }
        }
        
        // Ensure traj_server stops (in case it was running from a previous session)
        std_msgs::Int32 stop_msg;
        stop_msg.data = 2;  // Task finished signal
        replan_pub_.publish(stop_msg);
        
        transitState(FSM_STATE::WAIT_TARGET, "execFSM");
      }
      break;
    }
    
    case FSM_STATE::WAIT_TARGET: {
      printState();
      publishHoverCmd();
      
      if (fd_.have_target && fd_.triggered) {
        transitState(FSM_STATE::GEN_NEW_TRAJ, "execFSM");
      }
      break;
    }
    
    case FSM_STATE::GEN_NEW_TRAJ: {
      bool success = planFromCurrentState();
      
      if (success) {
        fd_.static_state = false;
        trajectory_start_time_ = ros::Time::now() + ros::Duration(fp_.replan_duration);
        planner_manager_->getLocalData().start_time_ = trajectory_start_time_;
        
        publishBsplineTraj();
        transitState(FSM_STATE::EXEC_TRAJ, "execFSM");
        plan_fail_count_ = 0;
      } else {
        ROS_WARN("[PlanningFSM] Planning failed!");
        plan_fail_count_++;
        
        if (plan_fail_count_ > 5) {
          ROS_ERROR("[PlanningFSM] Planning failed too many times, emergency stop!");
          transitState(FSM_STATE::EMERGENCY_STOP, "execFSM");
        } else {
          // Keep trying
          ros::Duration(0.1).sleep();
        }
      }
      break;
    }
    
    case FSM_STATE::REPLAN_TRAJ: {
      // Replan from current trajectory state
      LocalTrajData &info = planner_manager_->getLocalData();
      ros::Time now = ros::Time::now();
      double t_cur = (now - info.start_time_).toSec() + fp_.replan_duration;
      
      Eigen::Vector3d start_pos, start_vel, start_acc;
      double cur_yaw, cur_yaw_vel;
      
      // Check if trajectory has ended or is about to end
      // In this case, use odometry (actual) velocity to ensure continuity
      if (t_cur >= info.duration_ - 0.05) {
        // Trajectory ended or nearly ended - use odometry state
        // This ensures smooth transition when replanning at trajectory end
        start_pos = fd_.odom_pos;
        start_vel = fd_.odom_vel;
        start_acc = Eigen::Vector3d::Zero();
        cur_yaw = fd_.odom_yaw;
        cur_yaw_vel = fd_.odom_yaw_vel;
        
        ROS_DEBUG("[PlanningFSM] Replan at traj end: using odom vel=(%.2f, %.2f, %.2f)",
                  start_vel.x(), start_vel.y(), start_vel.z());
      } else {
        // Trajectory still running - use trajectory state for smooth handoff
        start_pos = info.position_traj_.evaluateDeBoorT(t_cur);
        start_vel = info.velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc = info.acceleration_traj_.evaluateDeBoorT(t_cur);
        
        cur_yaw = fd_.odom_yaw;
        cur_yaw_vel = 0.0;
        if (info.yaw_traj_.getTimeSum() > 0) {
          cur_yaw = info.yaw_traj_.evaluateDeBoorT(t_cur)[0];
          cur_yaw_vel = info.yawdot_traj_.evaluateDeBoorT(t_cur)[0];
        }
      }
      
      bool success = planner_manager_->planTrajectory(start_pos, start_vel, start_acc, fd_.target_pos, cur_yaw, cur_yaw_vel);
      
      if (success) {
        trajectory_start_time_ = now + ros::Duration(fp_.replan_duration);
        planner_manager_->getLocalData().start_time_ = trajectory_start_time_;
        
        publishBsplineTraj();
        transitState(FSM_STATE::EXEC_TRAJ, "execFSM");
        plan_fail_count_ = 0;
      } else {
        ROS_WARN("[PlanningFSM] Replan failed!");
        plan_fail_count_++;
        
        if (plan_fail_count_ > 10) {
          // Try planning from current position
          transitState(FSM_STATE::GEN_NEW_TRAJ, "execFSM");
        }
      }
      break;
    }
    
    case FSM_STATE::EXEC_TRAJ: {
      LocalTrajData &info = planner_manager_->getLocalData();
      ros::Time now = ros::Time::now();
      double t_cur = (now - info.start_time_).toSec();
      
      // First check if reached goal (priority over replan)
      double dist_to_goal = (fd_.odom_pos - fd_.target_pos).norm();
      
      if (dist_to_goal < 0.5) {
        ROS_INFO("[PlanningFSM] Reached goal! (dist=%.2f)", dist_to_goal);
        
        // Set hover position using target position
        hover_pos_ = fd_.target_pos;
        
        // Use the yaw at current trajectory time (not end_yaw_ which may be in the future)
        // This prevents yaw jumps when we detect goal arrival before trajectory completes
        double current_traj_yaw;
        if (t_cur >= 0 && t_cur <= info.duration_) {
          // Evaluate yaw at current time
          current_traj_yaw = info.yaw_traj_.evaluateDeBoorT(t_cur)[0];
        } else {
          // Trajectory completed, use end yaw
          current_traj_yaw = info.end_yaw_;
        }
        hover_yaw_ = current_traj_yaw;
        hover_pos_set_ = true;
        
        ROS_INFO("[PlanningFSM] Hover set: pos=(%.2f, %.2f, %.2f), yaw=%.2f (t_cur=%.2f)",
                 hover_pos_.x(), hover_pos_.y(), hover_pos_.z(), hover_yaw_, t_cur);
        
        // Check for next waypoint
        if (fp_.target_type == 2 && setNextWaypoint()) {
          transitState(FSM_STATE::GEN_NEW_TRAJ, "execFSM");
        } else {
          // All waypoints completed - notify traj_server to stop FIRST
          std_msgs::Int32 stop_msg;
          stop_msg.data = 2;  // Task finished signal
          replan_pub_.publish(stop_msg);
          ROS_INFO("[PlanningFSM] All preset waypoints completed, traj_server stopped");
          
          fd_.have_target = false;
          fd_.triggered = false;
          fd_.static_state = true;
          transitState(FSM_STATE::WAIT_TARGET, "execFSM");
        }
        return;
      }
      
      // Check if trajectory completed
      if (t_cur >= info.duration_) {
        // Trajectory ended but not at goal, replan
        ROS_WARN("[PlanningFSM] Trajectory ended but not at goal (dist=%.2f), replanning...", dist_to_goal);
        transitState(FSM_STATE::GEN_NEW_TRAJ, "execFSM");
        return;
      }
      
      // Check for replan conditions
      bool need_replan = false;
      
      // Condition 1: Near end of trajectory (only if trajectory is long enough)
      double remaining_time = info.duration_ - t_cur;
      if (remaining_time < fp_.replan_thresh1 && remaining_time > 0.1) {
        // Only replan if we're not too close to goal and trajectory is meaningful
        if (dist_to_goal > 1.0) {
          ROS_INFO("[PlanningFSM] Replan: near end of trajectory (remaining=%.2f, dist_to_goal=%.2f)", 
                   remaining_time, dist_to_goal);
          need_replan = true;
        }
      }
      
      // Condition 2: Periodic replan (only for long trajectories)
      if (t_cur > fp_.replan_thresh3 && dist_to_goal > 1.0) {
        ROS_INFO("[PlanningFSM] Replan: periodic");
        need_replan = true;
      }
      
      if (need_replan) {
        transitState(FSM_STATE::REPLAN_TRAJ, "execFSM");
      }
      break;
    }
    
    case FSM_STATE::EMERGENCY_STOP: {
      if (!hover_pos_set_) {
        hover_pos_ = fd_.odom_pos;
        hover_yaw_ = fd_.odom_yaw;
        hover_pos_set_ = true;
      }
      
      publishHoverCmd();
      
      ROS_WARN_THROTTLE(1.0, "[PlanningFSM] EMERGENCY STOP! Hovering at (%.2f, %.2f, %.2f)",
                        hover_pos_.x(), hover_pos_.y(), hover_pos_.z());
      break;
    }
  }
}

void PlanningFSM::safetyCallback(const ros::TimerEvent &e) {
  if (state_ == FSM_STATE::EXEC_TRAJ) {
    // Check trajectory collision
    bool safe = planner_manager_->checkTrajCollision();
    
    if (!safe) {
      ROS_ERROR("[PlanningFSM] Collision detected! Replanning...");
      
      std_msgs::Int32 replan_msg;
      replan_msg.data = 1;
      replan_pub_.publish(replan_msg);
      
      transitState(FSM_STATE::REPLAN_TRAJ, "safetyCallback");
    }
  }
}

void PlanningFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
  fd_.odom_pos.x() = msg->pose.pose.position.x;
  fd_.odom_pos.y() = msg->pose.pose.position.y;
  fd_.odom_pos.z() = msg->pose.pose.position.z;
  
  fd_.odom_vel.x() = msg->twist.twist.linear.x;
  fd_.odom_vel.y() = msg->twist.twist.linear.y;
  fd_.odom_vel.z() = msg->twist.twist.linear.z;
  
  fd_.odom_orient.w() = msg->pose.pose.orientation.w;
  fd_.odom_orient.x() = msg->pose.pose.orientation.x;
  fd_.odom_orient.y() = msg->pose.pose.orientation.y;
  fd_.odom_orient.z() = msg->pose.pose.orientation.z;
  
  // Extract yaw from quaternion
  tf::Quaternion q(fd_.odom_orient.x(), fd_.odom_orient.y(),
                   fd_.odom_orient.z(), fd_.odom_orient.w());
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  fd_.odom_yaw = yaw;
  
  // Extract yaw velocity from angular velocity
  fd_.odom_yaw_vel = msg->twist.twist.angular.z;
  
  fd_.have_odom = true;
}

void PlanningFSM::targetCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
  if (state_ < FSM_STATE::WAIT_TARGET) {
    ROS_WARN("[PlanningFSM] Cannot accept target before takeoff!");
    return;
  }
  
  fd_.target_pos.x() = msg->pose.position.x;
  fd_.target_pos.y() = msg->pose.position.y;
  fd_.target_pos.z() = fp_.takeoff_height;  // Use takeoff height
  
  // Extract target yaw
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                   msg->pose.orientation.z, msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  fd_.target_yaw = yaw;
  
  fd_.have_target = true;
  fd_.triggered = true;
  
  ROS_INFO("[PlanningFSM] Received target: (%.2f, %.2f, %.2f)",
           fd_.target_pos.x(), fd_.target_pos.y(), fd_.target_pos.z());
  
  // If already in trajectory execution, switch to new trajectory
  if (state_ == FSM_STATE::EXEC_TRAJ || state_ == FSM_STATE::REPLAN_TRAJ) {
    transitState(FSM_STATE::GEN_NEW_TRAJ, "targetCallback");
  }
}

void PlanningFSM::triggerCallback(const std_msgs::EmptyConstPtr &msg) {
  if (state_ != FSM_STATE::WAIT_TARGET) return;
  
  if (fp_.target_type == 2) {
    // Preset waypoints mode
    if (setNextWaypoint()) {
      fd_.triggered = true;
      transitState(FSM_STATE::GEN_NEW_TRAJ, "triggerCallback");
    }
  }
}

void PlanningFSM::mavrosStateCallback(const mavros_msgs::StateConstPtr &msg) {
  mavros_state_ = *msg;
  is_connected_ = msg->connected;
  is_armed_ = msg->armed;
  is_offboard_ = (msg->mode == "OFFBOARD");
}

// ==================== Planning Functions ====================
bool PlanningFSM::callPlanningOnce() {
  return planFromCurrentState();
}

bool PlanningFSM::planFromCurrentState() {
  Eigen::Vector3d start_pos, start_vel, start_acc;
  double cur_yaw = fd_.odom_yaw;
  double cur_yaw_vel = fd_.odom_yaw_vel;
  
  if (fd_.static_state) {
    // Static state - use odometry directly
    start_pos = fd_.odom_pos;
    start_vel = fd_.odom_vel;
    start_acc = Eigen::Vector3d::Zero();
  } else {
    // Get state from current trajectory
    LocalTrajData &info = planner_manager_->getLocalData();
    double t_cur = (ros::Time::now() - info.start_time_).toSec() + fp_.replan_duration;
    
    // Check if trajectory has ended or is about to end
    if (t_cur >= info.duration_ - 0.05) {
      // Trajectory ended - use odometry state for continuity
      start_pos = fd_.odom_pos;
      start_vel = fd_.odom_vel;
      start_acc = Eigen::Vector3d::Zero();
      // Keep cur_yaw and cur_yaw_vel from odometry
    } else {
      // Trajectory running - use trajectory state
      start_pos = info.position_traj_.evaluateDeBoorT(t_cur);
      start_vel = info.velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc = info.acceleration_traj_.evaluateDeBoorT(t_cur);
      
      // Get yaw state from trajectory if available
      if (info.yaw_traj_.getTimeSum() > 0) {
        cur_yaw = info.yaw_traj_.evaluateDeBoorT(t_cur)[0];
        cur_yaw_vel = info.yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      }
    }
  }
  
  ROS_INFO("[PlanningFSM] Planning from (%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
           start_pos.x(), start_pos.y(), start_pos.z(),
           start_vel.x(), start_vel.y(), start_vel.z(),
           fd_.target_pos.x(), fd_.target_pos.y(), fd_.target_pos.z());
  return planner_manager_->planTrajectory(start_pos, start_vel, start_acc, fd_.target_pos, cur_yaw, cur_yaw_vel);
}

// ==================== PX4 Control Functions ====================
void PlanningFSM::sendInitialSetpoints() {
  geometry_msgs::PoseStamped setpoint;
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = initial_pos_.x();
  setpoint.pose.position.y = initial_pos_.y();
  setpoint.pose.position.z = initial_pos_.z() + 0.5;  // Slight offset
  
  tf::Quaternion q;
  q.setRPY(0, 0, initial_yaw_);
  setpoint.pose.orientation.x = q.x();
  setpoint.pose.orientation.y = q.y();
  setpoint.pose.orientation.z = q.z();
  setpoint.pose.orientation.w = q.w();
  
  setpoint_pub_.publish(setpoint);
  setpoint_count_++;
  
  if (setpoint_count_ % 20 == 0) {
    ROS_INFO("[PlanningFSM] Sending setpoints: %d/%d",
             setpoint_count_, fp_.pre_arm_setpoint_count);
  }
}

bool PlanningFSM::armAndOffboard() {
  ros::Time now = ros::Time::now();
  
  // Request offboard mode
  if (!is_offboard_ && (now - last_mode_request_).toSec() > fp_.service_timeout) {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
      ROS_INFO("[PlanningFSM] OFFBOARD mode request sent");
    }
    last_mode_request_ = now;
  }
  
  // Request arming
  if (is_offboard_ && !is_armed_ && (now - last_arm_request_).toSec() > fp_.service_timeout) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("[PlanningFSM] Arming request sent");
    }
    last_arm_request_ = now;
  }
  
  return is_armed_ && is_offboard_;
}

void PlanningFSM::updateTakeoffTrajectory() {
  double t = (ros::Time::now() - takeoff_start_time_).toSec();
  double target_height = fp_.takeoff_height;
  double height_diff = target_height - initial_pos_.z();
  
  // Simple trapezoidal velocity profile
  double t_accel = fp_.takeoff_accel_time;
  double t_decel = fp_.takeoff_decel_time;
  double v_max = fp_.takeoff_max_vel;
  
  double accel_dist = 0.5 * v_max * t_accel;
  double decel_dist = 0.5 * v_max * t_decel;
  double cruise_dist = height_diff - accel_dist - decel_dist;
  
  if (cruise_dist < 0) {
    // No cruise phase, adjust
    cruise_dist = 0;
    double total_dist = height_diff;
    v_max = std::sqrt(2 * total_dist * fp_.takeoff_max_accel);
    t_accel = v_max / fp_.takeoff_max_accel;
    t_decel = t_accel;
    accel_dist = 0.5 * v_max * t_accel;
    decel_dist = accel_dist;
  }
  
  double t_cruise = cruise_dist / v_max;
  
  if (t < t_accel) {
    // Acceleration phase
    trajectory_acc_z_ = fp_.takeoff_max_accel;
    trajectory_vel_z_ = trajectory_acc_z_ * t;
    trajectory_z_ = initial_pos_.z() + 0.5 * trajectory_acc_z_ * t * t;
  } else if (t < t_accel + t_cruise) {
    // Cruise phase
    double t_in_cruise = t - t_accel;
    trajectory_acc_z_ = 0.0;
    trajectory_vel_z_ = v_max;
    trajectory_z_ = initial_pos_.z() + accel_dist + v_max * t_in_cruise;
  } else if (t < t_accel + t_cruise + t_decel) {
    // Deceleration phase
    double t_in_decel = t - t_accel - t_cruise;
    trajectory_acc_z_ = -fp_.takeoff_max_accel;
    trajectory_vel_z_ = v_max - fp_.takeoff_max_accel * t_in_decel;
    trajectory_z_ = initial_pos_.z() + accel_dist + cruise_dist +
                    v_max * t_in_decel - 0.5 * fp_.takeoff_max_accel * t_in_decel * t_in_decel;
  } else {
    // Done
    trajectory_acc_z_ = 0.0;
    trajectory_vel_z_ = 0.0;
    trajectory_z_ = target_height;
  }
}

bool PlanningFSM::checkTakeoffComplete() {
  double height_error = std::abs(fd_.odom_pos.z() - fp_.takeoff_height);
  double vel_error = std::abs(fd_.odom_vel.z());
  
  return (height_error < fp_.takeoff_threshold && vel_error < 0.1);
}

// ==================== Command Publishing ====================
void PlanningFSM::publishTakeoffCmd() {
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  
  cmd.position.x = initial_pos_.x();
  cmd.position.y = initial_pos_.y();
  cmd.position.z = trajectory_z_;
  
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = trajectory_vel_z_;
  
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = trajectory_acc_z_;
  
  cmd.yaw = initial_yaw_;
  cmd.yaw_dot = 0.0;
  
  pos_cmd_pub_.publish(cmd);
  
  // Also publish to mavros
  publishSetpoint(Eigen::Vector3d(cmd.position.x, cmd.position.y, cmd.position.z), initial_yaw_);
}

void PlanningFSM::publishHoverCmd() {
  if (!hover_pos_set_) {
    hover_pos_ = fd_.odom_pos;
    hover_yaw_ = fd_.odom_yaw;
    hover_pos_set_ = true;
    ROS_INFO("[PlanningFSM] Hover position initialized: (%.2f, %.2f, %.2f), yaw=%.2f",
             hover_pos_.x(), hover_pos_.y(), hover_pos_.z(), hover_yaw_);
  }
  
  // Normalize hover_yaw_ to [-PI, PI]
  double yaw_normalized = hover_yaw_;
  while (yaw_normalized < -M_PI) yaw_normalized += 2 * M_PI;
  while (yaw_normalized > M_PI) yaw_normalized -= 2 * M_PI;
  
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  
  cmd.position.x = hover_pos_.x();
  cmd.position.y = hover_pos_.y();
  cmd.position.z = hover_pos_.z();
  
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  
  cmd.yaw = yaw_normalized;
  cmd.yaw_dot = 0.0;
  
  pos_cmd_pub_.publish(cmd);
  
  // Also publish to mavros
  publishSetpoint(hover_pos_, yaw_normalized);
}

void PlanningFSM::publishSetpoint(const Eigen::Vector3d &pos, double yaw) {
  geometry_msgs::PoseStamped setpoint;
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = pos.x();
  setpoint.pose.position.y = pos.y();
  setpoint.pose.position.z = pos.z();
  
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  setpoint.pose.orientation.x = q.x();
  setpoint.pose.orientation.y = q.y();
  setpoint.pose.orientation.z = q.z();
  setpoint.pose.orientation.w = q.w();
  
  setpoint_pub_.publish(setpoint);
}

void PlanningFSM::publishBsplineTraj() {
  LocalTrajData &info = planner_manager_->getLocalData();
  
  rog_map::Bspline bspline;
  bspline.order = 3;
  bspline.traj_id = info.traj_id_;
  bspline.start_time = info.start_time_;
  
  // Position control points
  Eigen::MatrixXd pos_pts = info.position_traj_.getControlPoint();
  for (int i = 0; i < pos_pts.rows(); ++i) {
    geometry_msgs::Point pt;
    pt.x = pos_pts(i, 0);
    pt.y = pos_pts(i, 1);
    pt.z = pos_pts(i, 2);
    bspline.pos_pts.push_back(pt);
  }
  
  // Knots
  Eigen::VectorXd knots = info.position_traj_.getKnot();
  for (int i = 0; i < knots.rows(); ++i) {
    bspline.knots.push_back(knots(i));
  }
  
  // Yaw control points
  Eigen::MatrixXd yaw_pts = info.yaw_traj_.getControlPoint();
  for (int i = 0; i < yaw_pts.rows(); ++i) {
    bspline.yaw_pts.push_back(yaw_pts(i, 0));
  }
  bspline.yaw_dt = info.yaw_traj_.getKnotSpan();
  
  bspline_pub_.publish(bspline);
  
  ROS_INFO("[PlanningFSM] Published Bspline trajectory, id: %ld, duration: %.2f s",
           bspline.traj_id, info.duration_);
}

} // namespace fast_planner

