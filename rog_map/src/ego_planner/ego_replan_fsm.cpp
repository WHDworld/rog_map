#include "ego_planner/ego_replan_fsm.h"
#include <cmath>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh, rog_map::ROGMap::Ptr map)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_trigger_ = false;
    have_new_target_ = false;
    mandatory_stop_ = false;
    hover_pos_set_ = false;
    offboard_ready_ = false;
    
    // Initialize target state
    end_vel_ = Eigen::Vector3d::Zero();
    end_yaw_ = 0.0;

    rog_map_ = map;

    /* initialize main modules */
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, rog_map_);

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, 1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, 1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, 1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, 7.5);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    
    /* PX4 control parameters */
    nh.param("fsm/takeoff_height", takeoff_height_, 1.0);
    nh.param("fsm/takeoff_accel_time", takeoff_accel_time_, 1.0);
    nh.param("fsm/replan_duration", replan_duration_, 0.05);
    nh.param("fsm/auto_start_preset", auto_start_preset_, true);

    /* waypoints */
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* callbacks */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);
    state_sub_ = nh.subscribe("/mavros/state", 10, &EGOReplanFSM::stateCallback, this);

    poly_traj_pub_ = nh.advertise<rog_map::PolyTraj>("/planning/trajectory", 10);
    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
    replan_pub_ = nh.advertise<std_msgs::Int8>("/planning/replan", 10);
    set_target_point_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/set_target_point", 10);
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
      // Exploration viewpoint with full state (pos, yaw, vel)
      viewpoint_sub_ = nh.subscribe("/exploration/viewpoint", 1, &EGOReplanFSM::viewpointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
    }

    ROS_INFO("[EGOReplanFSM] Initialized with target_type=%d", target_type_);
    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ROS_INFO("[EGOReplanFSM] Preset waypoints (%d):", waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        ROS_INFO("  [%d] (%.2f, %.2f, %.2f)", i, waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
      }
    }
  }

  void EGOReplanFSM::stateCallback(const mavros_msgs::State::ConstPtr &msg)
  {
    current_state_ = *msg;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      initial_pos_ = odom_pos_;
      changeFSMExecState(PRE_ARM, "FSM");
      break;
    }

    case PRE_ARM:
    {
      // Wait for connection and request OFFBOARD mode
      static ros::Time last_request = ros::Time::now();
      static int arm_attempts = 0;
      
      if (!current_state_.connected)
      {
        ROS_INFO_THROTTLE(1.0, "[EGOReplanFSM] Waiting for FCU connection...");
        goto force_return;
      }

      // First send setpoints before switching to OFFBOARD
      publishHoverCmd();
      
      if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0)))
      {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("[EGOReplanFSM] OFFBOARD mode requested");
        }
        last_request = ros::Time::now();
      }
      else if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
      {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("[EGOReplanFSM] Vehicle armed");
        }
        last_request = ros::Time::now();
        arm_attempts++;
      }

      if (current_state_.mode == "OFFBOARD" && current_state_.armed)
      {
        offboard_ready_ = true;
        takeoff_start_time_ = ros::Time::now();
        hover_pos_ = odom_pos_;
        hover_yaw_ = odom_yaw_;
        hover_pos_set_ = true;
        changeFSMExecState(TAKEOFF, "FSM");
        ROS_INFO("[EGOReplanFSM] OFFBOARD mode enabled, starting takeoff");
      }
      break;
    }

    case TAKEOFF:
    {
      double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
      double target_height = initial_pos_.z() + takeoff_height_;
      
      publishTakeoffCmd();
      
      if (odom_pos_.z() > target_height - 0.1 && elapsed > takeoff_accel_time_)
      {
        hover_pos_ = Eigen::Vector3d(initial_pos_.x(), initial_pos_.y(), target_height);
        hover_yaw_ = odom_yaw_;
        hover_pos_set_ = true;
        
        if (target_type_ == TARGET_TYPE::PRESET_TARGET && auto_start_preset_)
        {
          readGivenWpsAndPlan();
          have_trigger_ = true;
          changeFSMExecState(WAIT_TARGET, "FSM");
          ROS_INFO("[EGOReplanFSM] Takeoff complete, starting preset path");
        }
        else
        {
          changeFSMExecState(WAIT_TARGET, "FSM");
          ROS_INFO("[EGOReplanFSM] Takeoff complete, waiting for target");
        }
      }
      break;
    }

    case WAIT_TARGET:
    {
      publishHoverCmd();
      
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      bool success = planFromGlobalTraj(10);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        ROS_WARN("[EGOReplanFSM] Failed to generate trajectory, retrying...");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (planFromLocalTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ:
    {
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;

      // Check if reached goal
      if (touch_goal_)
      {
        if (t_cur > info->duration - 1e-2)
        {
          // Update hover position to goal
          hover_pos_ = end_pt_;
          // Use target yaw if specified (not NaN), otherwise keep current yaw
          hover_yaw_ = std::isnan(end_yaw_) ? odom_yaw_ : end_yaw_;
          hover_pos_set_ = true;
          have_target_ = false;
          have_trigger_ = false;
          
          // Check for next waypoint
          if (wpt_id_ < (int)wps_.size() - 1)
          {
            wpt_id_++;
            const Waypoint& next_wp = wps_[wpt_id_];
            const Waypoint& prev_wp = wps_[wpt_id_-1];
            end_pt_ = next_wp.pos;
            end_yaw_ = next_wp.yaw;
            end_vel_ = next_wp.vel;
            planNextWaypoint(next_wp, prev_wp);
            have_target_ = true;
            have_trigger_ = true;
            if (std::isnan(end_yaw_))
              ROS_INFO("[EGOReplanFSM] Next waypoint: pos=(%.2f, %.2f, %.2f), yaw=auto", 
                       end_pt_.x(), end_pt_.y(), end_pt_.z());
            else
              ROS_INFO("[EGOReplanFSM] Next waypoint: pos=(%.2f, %.2f, %.2f), yaw=%.2f", 
                       end_pt_.x(), end_pt_.y(), end_pt_.z(), end_yaw_);
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          }
          else
          {
            ROS_INFO("[EGOReplanFSM] All waypoints completed, waiting for new target");
            // Signal trajectory server to stop
            std_msgs::Int8 stop_msg;
            stop_msg.data = 2;
            replan_pub_.publish(stop_msg);
            changeFSMExecState(WAIT_TARGET, "FSM");
          }
          goto force_return;
        }
      }

      // Check replan conditions
      if (t_cur > replan_thresh_ || (!touch_goal_ && t_cur > info->duration - 1e-2))
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      publishHoverCmd();
      ROS_WARN_THROTTLE(1.0, "[EGOReplanFSM] EMERGENCY STOP! Hovering at (%.2f, %.2f, %.2f)",
                        hover_pos_.x(), hover_pos_.y(), hover_pos_.z());
      break;
    }
    }

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call)
  {
    static std::string state_str[8] = {"INIT", "PRE_ARM", "TAKEOFF", "WAIT_TARGET",
                                       "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    
    // int pre_s = static_cast<int>(exec_state_);
    exec_state_ = new_state;
    
    // ROS_INFO("[EGOReplanFSM] State: %s -> %s (by %s)",
    //          state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str(), pos_call.c_str());
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static std::string state_str[8] = {"INIT", "PRE_ARM", "TAKEOFF", "WAIT_TARGET",
                                       "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    ROS_INFO("[EGOReplanFSM] Current state: %s", state_str[static_cast<int>(exec_state_)].c_str());
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // Simplified collision check
    if (exec_state_ == EXEC_TRAJ)
    {
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      
      // Check future trajectory for collisions
      for (double t = t_cur; t < std::min(info->duration, t_cur + emergency_time_); t += 0.05)
      {
        Eigen::Vector3d pos = info->traj.getPos(t);
        if (rog_map_->isOccupiedInflate(pos))
        {
          if (enable_fail_safe_)
          {
            ROS_ERROR("[EGOReplanFSM] Collision detected at t=%.2f, emergency stop!", t);
            callEmergencyStop(odom_pos_);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          return;
        }
      }
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    hover_pos_ = stop_pos;
    hover_yaw_ = odom_yaw_;
    hover_pos_set_ = true;
    
    planner_manager_->EmergencyStop(stop_pos);
    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    planner_manager_->getLocalTarget(planning_horizen_, start_pt_, end_pt_,
                                     local_target_pt_, local_target_vel_, touch_goal_);

    // If touch_goal and user specified non-zero end velocity, use it
    // This supports exploration tasks where continuous motion is needed
    if (touch_goal_ && end_vel_.norm() > 1e-3)
    {
      local_target_vel_ = end_vel_;
      ROS_DEBUG("[EGOReplanFSM] Using specified end velocity: (%.2f, %.2f, %.2f)",
                end_vel_.x(), end_vel_.y(), end_vel_.z());
    }

    // 设置期望高度（高度保持功能）：使用起点高度
    planner_manager_->setDesiredHeight(start_pt_(2));

    bool success = planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_,
                                                   local_target_pt_, local_target_vel_,
                                                   flag_use_poly_init, flag_randomPolyTraj, touch_goal_);
    
    if (success)
    {
      // Publish trajectory with yaw info
      rog_map::PolyTraj poly_msg;
      polyTraj2ROSMsg(poly_msg);
      poly_traj_pub_.publish(poly_msg);
      
      // Notify trajectory server
      std_msgs::Int8 replan_msg;
      replan_msg.data = 1;
      replan_pub_.publish(replan_msg);
    }
    
    return success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times)
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_ = Eigen::Vector3d::Zero();

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, false))
        return true;
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times)
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(false, false))
        return true;
    }
    return false;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    end_pt_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    // Extract yaw from quaternion orientation using stable method
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
    end_yaw_ = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    
    // Default: zero end velocity for waypoint goals
    end_vel_ = Eigen::Vector3d::Zero();
    
    wps_.clear();
    wps_.push_back(Waypoint(end_pt_, end_yaw_, end_vel_));
    wpt_id_ = 0;

    // Plan global trajectory
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(end_pt_);
    planner_manager_->planGlobalTrajWaypoints(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
                                               waypoints, end_vel_, Eigen::Vector3d::Zero());

    have_target_ = true;
    have_trigger_ = true;
    have_new_target_ = true;

    ROS_INFO("[EGOReplanFSM] Received waypoint: pos=(%.2f, %.2f, %.2f), yaw=%.2f", 
             end_pt_.x(), end_pt_.y(), end_pt_.z(), end_yaw_);

    if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ)
    {
      changeFSMExecState(GEN_NEW_TRAJ, "WAYPOINT");
    }
  }

  void EGOReplanFSM::viewpointCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
  {
    // Full exploration viewpoint with position, yaw, and velocity
    end_pt_ << msg->position.x, msg->position.y, msg->position.z;
    end_yaw_ = msg->yaw;
    end_vel_ << msg->velocity.x, msg->velocity.y, msg->velocity.z;
    
    wps_.clear();
    wps_.push_back(Waypoint(end_pt_, end_yaw_, end_vel_));
    wpt_id_ = 0;

    // Plan global trajectory with end velocity
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(end_pt_);
    planner_manager_->planGlobalTrajWaypoints(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
                                               waypoints, end_vel_, Eigen::Vector3d::Zero());

    have_target_ = true;
    have_trigger_ = true;
    have_new_target_ = true;

    ROS_INFO("[EGOReplanFSM] Received viewpoint: pos=(%.2f, %.2f, %.2f), yaw=%.2f, vel=(%.2f, %.2f, %.2f)", 
             end_pt_.x(), end_pt_.y(), end_pt_.z(), end_yaw_,
             end_vel_.x(), end_vel_.y(), end_vel_.z());

    if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ)
    {
      changeFSMExecState(GEN_NEW_TRAJ, "VIEWPOINT");
    }
  }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    wps_.clear();
    for (int i = 0; i < waypoint_num_; i++)
    {
      // For preset waypoints, use NaN for yaw (means "follow velocity direction")
      // and zero velocity (will stop at each waypoint)
      Eigen::Vector3d pos(waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
      wps_.push_back(Waypoint(pos, std::numeric_limits<double>::quiet_NaN(), Eigen::Vector3d::Zero()));
      
      // Visualize target point
      visualization_msgs::Marker goal_marker;
      goal_marker.header.frame_id = "map";
      goal_marker.header.stamp = ros::Time::now();
      goal_marker.ns = "goal";
      goal_marker.id = i;
      goal_marker.type = visualization_msgs::Marker::SPHERE;
      goal_marker.action = visualization_msgs::Marker::ADD;
      goal_marker.pose.position.x = waypoints_[i][0];
      goal_marker.pose.position.y = waypoints_[i][1];
      goal_marker.pose.position.z = waypoints_[i][2];
      goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.5;
      goal_marker.color.r = 1.0;
      goal_marker.color.g = 0.0;
      goal_marker.color.b = 0.0;
      goal_marker.color.a = 0.8;
      set_target_point_pub_.publish(goal_marker);
    }
    
    wpt_id_ = 0;
    end_pt_ = wps_[wpt_id_].pos;
    end_yaw_ = wps_[wpt_id_].yaw;
    end_vel_ = wps_[wpt_id_].vel;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(end_pt_);
    planner_manager_->planGlobalTrajWaypoints(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
                                               waypoints, end_vel_, Eigen::Vector3d::Zero());

    have_target_ = true;
    
    if (std::isnan(end_yaw_))
      ROS_INFO("[EGOReplanFSM] Loaded %d waypoints, first: pos=(%.2f, %.2f, %.2f), yaw=auto (follow velocity)",
               waypoint_num_, end_pt_.x(), end_pt_.y(), end_pt_.z());
    else
      ROS_INFO("[EGOReplanFSM] Loaded %d waypoints, first: pos=(%.2f, %.2f, %.2f), yaw=%.2f",
               waypoint_num_, end_pt_.x(), end_pt_.y(), end_pt_.z(), end_yaw_);
  }

  void EGOReplanFSM::planNextWaypoint(const Waypoint& next_wp, const Waypoint& previous_wp)
  {
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(next_wp.pos);
    // Use previous waypoint's velocity as start, next waypoint's velocity as end
    planner_manager_->planGlobalTrajWaypoints(previous_wp.pos, previous_wp.vel, Eigen::Vector3d::Zero(),
                                               waypoints, next_wp.vel, Eigen::Vector3d::Zero());
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("[EGOReplanFSM] Received mandatory stop!");
    callEmergencyStop(odom_pos_);
    changeFSMExecState(EMERGENCY_STOP, "MANDATORY_STOP");
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // Extract yaw from quaternion using stable method (avoids gimbal lock)
    // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
    double qw = odom_orient_.w();
    double qx = odom_orient_.x();
    double qy = odom_orient_.y();
    double qz = odom_orient_.z();
    odom_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    odom_yaw_vel_ = msg->twist.twist.angular.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    ROS_INFO("[EGOReplanFSM] Trigger received!");
  }

  void EGOReplanFSM::polyTraj2ROSMsg(rog_map::PolyTraj &poly_msg)
  {
    poly_traj::Trajectory &traj = planner_manager_->traj_.local_traj.traj;
    
    poly_msg.traj_id = planner_manager_->traj_.local_traj.traj_id;
    poly_msg.start_time = ros::Time(planner_manager_->traj_.local_traj.start_time);
    poly_msg.order = 5;  // MINCO uses order 5
    
    // Yaw planning: 
    // If end_yaw_ is NaN, it means "follow velocity direction" (preset waypoint mode)
    // Otherwise, interpolate from current yaw to target yaw (exploration mode)
    poly_msg.start_yaw = odom_yaw_;
    poly_msg.end_yaw = end_yaw_;  // May be NaN for preset waypoint mode

    int piece_num = traj.getPieceNum();
    for (int i = 0; i < piece_num; i++)
    {
      poly_msg.duration.push_back(traj.getPiece(i).getDuration());
      
      Eigen::MatrixXd coef = traj.getPiece(i).getCoeffMat();
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x.push_back(coef(0, j));
        poly_msg.coef_y.push_back(coef(1, j));
        poly_msg.coef_z.push_back(coef(2, j));
      }
    }
  }

  void EGOReplanFSM::publishTakeoffCmd()
  {
    double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
    
    // Smooth takeoff trajectory
    double t_ratio = std::min(elapsed / takeoff_accel_time_, 1.0);
    double smooth_t = t_ratio * t_ratio * (3.0 - 2.0 * t_ratio);  // Smoothstep
    
    double current_target_z = initial_pos_.z() + smooth_t * takeoff_height_;
    
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.position.x = initial_pos_.x();
    cmd.position.y = initial_pos_.y();
    cmd.position.z = current_target_z;
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = (elapsed < takeoff_accel_time_) ? takeoff_height_ / takeoff_accel_time_ * 
                      (6 * t_ratio * (1 - t_ratio)) : 0.0;
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.yaw = hover_yaw_;
    cmd.yaw_dot = 0.0;
    
    pos_cmd_pub_.publish(cmd);
  }

  void EGOReplanFSM::publishHoverCmd()
  {
    if (!hover_pos_set_)
    {
      hover_pos_ = odom_pos_;
      hover_yaw_ = odom_yaw_;
      hover_pos_set_ = true;
    }
    
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.position.x = hover_pos_.x();
    cmd.position.y = hover_pos_.y();
    cmd.position.z = hover_pos_.z();
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.yaw = hover_yaw_;
    cmd.yaw_dot = 0.0;
    
    pos_cmd_pub_.publish(cmd);
  }

} // namespace ego_planner

