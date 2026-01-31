#include "ego_planner/ego_replan_fsm.h"
#include <cmath>
#include <tf/transform_datatypes.h>

// 性能监控
#include "utils/perf_logger.hpp"

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh, star_mvf::MapManager::Ptr map)
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
    
    // 轨迹执行状态初始化
    has_prev_traj_ = false;
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;
    last_pos_ = Eigen::Vector3d::Zero();
    traj_start_time_ = 0.0;
    traj_duration_ = 0.0;

    map_manager_ = map;

    /* initialize main modules */
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, map_manager_);

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
    
    /* Simulation mode: 0=gazebo, 1=open3d, 2=real */
    nh.param("fsm/sim_mode", sim_mode_, 0);
    
    // FALCON风格：设置重规划时间偏移，用于轨迹连续性
    planner_manager_->setReplanTimeOffset(replan_duration_);

    /* waypoints */
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
      // 读取yaw参数，默认为NaN表示跟随速度方向
      double yaw_default = std::numeric_limits<double>::quiet_NaN();
      nh.param("fsm/waypoint" + std::to_string(i) + "_yaw", waypoints_yaw_[i], yaw_default);
    }

    /* callbacks */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
    execution_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::executionCallback, this);  // 100Hz轨迹执行

    // 从参数读取odom话题名（与MapManager一致）
    std::string odom_topic;
    nh.param<std::string>("rog_map/odom_topic", odom_topic, "/odom");
    odom_sub_ = nh.subscribe(odom_topic, 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);
    state_sub_ = nh.subscribe("/mavros/state", 10, &EGOReplanFSM::stateCallback, this);

    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
    replan_pub_ = nh.advertise<std_msgs::Int8>("/planning/replan", 10);
    set_target_point_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/set_target_point", 10);
    traj_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/planning/trajectory_vis", 10);
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
      // Exploration viewpoint with full state (pos, yaw, vel)
      viewpoint_sub_ = nh.subscribe("/exploration/viewpoint", 1, &EGOReplanFSM::viewpointCallback, this);
      // Exploration viewpoint path with intermediate waypoints (for obstacle avoidance)
      viewpoint_path_sub_ = nh.subscribe("/exploration/viewpoint_path", 1, &EGOReplanFSM::viewpointPathCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
    }

    const char* sim_mode_str[] = {"GAZEBO", "OPEN3D", "REAL"};
    ROS_INFO("[EGOReplanFSM] Initialized with target_type=%d, sim_mode=%s", 
             target_type_, sim_mode_str[sim_mode_]);
    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ROS_INFO("[EGOReplanFSM] Preset waypoints (%d):", waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        if (std::isnan(waypoints_yaw_[i]))
          ROS_INFO("  [%d] (%.2f, %.2f, %.2f), yaw=auto", i, waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
        else
          ROS_INFO("  [%d] (%.2f, %.2f, %.2f), yaw=%.2f", i, waypoints_[i][0], waypoints_[i][1], waypoints_[i][2], waypoints_yaw_[i]);
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
      
      if (sim_mode_ == SIM_OPEN3D)
      {
        // Open3D mode: no FCU, directly go to TAKEOFF
        offboard_ready_ = true;
        takeoff_start_time_ = ros::Time::now();
        // 关键修复：hover_pos_ 应该设置为目标起飞高度，而不是当前位置（z=0）
        // 这样即使起飞过程中出现问题，悬停位置也不会是地面
        double target_height = initial_pos_.z() + takeoff_height_;
        hover_pos_ = Eigen::Vector3d(initial_pos_.x(), initial_pos_.y(), target_height);
        hover_yaw_ = odom_yaw_;
        hover_pos_set_ = true;
        ROS_INFO("[EGOReplanFSM] Open3D mode: skipping PRE_ARM, starting takeoff to z=%.2f", target_height);
        changeFSMExecState(TAKEOFF, "FSM");
      }
      else
      {
        // Gazebo or Real mode: go through PRE_ARM
        changeFSMExecState(PRE_ARM, "FSM");
      }
      break;
    }

    case PRE_ARM:
    {
      if (sim_mode_ == SIM_REAL)
      {
        // Real flight mode: wait for manual arming and OFFBOARD mode switch
        // User arms and switches to OFFBOARD manually, then we auto-takeoff
        static bool waiting_printed = false;
        
        if (!current_state_.connected)
        {
          ROS_INFO_THROTTLE(1.0, "[EGOReplanFSM] REAL mode: Waiting for FCU connection...");
          goto force_return;
        }
        
        if (!waiting_printed)
        {
          ROS_INFO("[EGOReplanFSM] REAL mode: Waiting for manual arming and OFFBOARD mode...");
          ROS_INFO("[EGOReplanFSM] Please arm the vehicle and switch to OFFBOARD mode manually.");
          waiting_printed = true;
        }
        
        // Send hover command to keep setpoint active
        publishHoverCmd();
        
        // Wait for both armed and OFFBOARD mode (set manually by pilot)
        if (current_state_.mode == "OFFBOARD" && current_state_.armed)
        {
          offboard_ready_ = true;
          takeoff_start_time_ = ros::Time::now();
          hover_pos_ = odom_pos_;
          hover_yaw_ = odom_yaw_;
          hover_pos_set_ = true;
          changeFSMExecState(TAKEOFF, "FSM");
          ROS_INFO("[EGOReplanFSM] REAL mode: Manual arming detected, starting takeoff");
          waiting_printed = false;
        }
        else
        {
          ROS_INFO_THROTTLE(2.0, "[EGOReplanFSM] REAL mode: armed=%d, mode=%s (waiting for OFFBOARD+armed)", 
                           current_state_.armed, current_state_.mode.c_str());
        }
      }
      else  // SIM_GAZEBO mode
      {
        // Wait for connection and request OFFBOARD mode automatically
        static ros::Time last_request = ros::Time::now();
        static int arm_attempts = 0;
        
        if (!current_state_.connected)
        {
          ROS_INFO_THROTTLE(1.0, "[EGOReplanFSM] GAZEBO mode: Waiting for FCU connection...");
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
            ROS_INFO("[EGOReplanFSM] GAZEBO mode: OFFBOARD mode requested");
          }
          last_request = ros::Time::now();
        }
        else if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
          mavros_msgs::CommandBool arm_cmd;
          arm_cmd.request.value = true;
          if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("[EGOReplanFSM] GAZEBO mode: Vehicle armed");
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
          ROS_INFO("[EGOReplanFSM] GAZEBO mode: OFFBOARD enabled, starting takeoff");
        }
      }
      break;
    }

    case TAKEOFF:
    {
      double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
      double target_height = initial_pos_.z() + takeoff_height_;
      
      // All modes use the same takeoff logic - send position commands to reach target height
      publishTakeoffCmd();
      
      // Check if takeoff is complete
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
          if (sim_mode_ == SIM_OPEN3D)
            ROS_INFO("[EGOReplanFSM] Open3D mode: Takeoff complete (z=%.2f), starting preset path", odom_pos_.z());
          else
            ROS_INFO("[EGOReplanFSM] Takeoff complete (z=%.2f), starting preset path", odom_pos_.z());
        }
        else
        {
          changeFSMExecState(WAIT_TARGET, "FSM");
          if (sim_mode_ == SIM_OPEN3D)
            ROS_INFO("[EGOReplanFSM] Open3D mode: Takeoff complete (z=%.2f), waiting for target", odom_pos_.z());
          else
            ROS_INFO("[EGOReplanFSM] Takeoff complete (z=%.2f), waiting for target", odom_pos_.z());
        }
      }
      else
      {
        // Print takeoff progress periodically
        static ros::Time last_print = ros::Time(0);
        if ((ros::Time::now() - last_print).toSec() > 1.0)
        {
          ROS_INFO("[EGOReplanFSM] Takeoff in progress: z=%.2f / %.2f", odom_pos_.z(), target_height);
          last_print = ros::Time::now();
        }
      }
      break;
    }

    case WAIT_TARGET:
    {
      publishHoverCmd();
      
      if (!have_target_ || !have_trigger_) {
        // 调试日志：每500次循环打印一次
        static int wait_count = 0;
        if (++wait_count >= 500) {
          ROS_DEBUG("[EGOReplanFSM] WAIT_TARGET: have_target=%d, have_trigger=%d",
                   have_target_, have_trigger_);
          wait_count = 0;
        }
        goto force_return;
      }
      else
      {
        ROS_INFO("[EGOReplanFSM] WAIT_TARGET -> GEN_NEW_TRAJ: target=(%.2f,%.2f,%.2f)",
                 end_pt_.x(), end_pt_.y(), end_pt_.z());
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      // 关键：在规划过程中持续发布悬停指令，避免指令空隙
      publishHoverCmd();
      
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
      // 关键：在重规划过程中持续发布悬停指令，避免指令空隙
      // 注意：这里不发布悬停指令，因为 ego_traj_server 正在执行旧轨迹
      // 如果发布悬停指令会与 ego_traj_server 冲突
      // ego_traj_server 的 FALCON 风格双缓存机制会在重规划期间继续执行旧轨迹
      
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
      // 安全检查：确保悬停高度不低于起飞高度
      double min_safe_height = initial_pos_.z() + takeoff_height_ * 0.5;  // 至少是起飞高度的一半
      if (hover_pos_.z() < min_safe_height)
      {
        ROS_WARN_ONCE("[EGOReplanFSM] Emergency hover height %.2f is too low, correcting to %.2f",
                      hover_pos_.z(), min_safe_height);
        hover_pos_.z() = min_safe_height;
      }
      
      publishHoverCmd();
      
      // 计数并记录日志
      static int estop_count = 0;
      static ros::Time estop_start_time;
      if (estop_count == 0) {
        estop_start_time = ros::Time::now();
      }
      estop_count++;
      
      double estop_duration = (ros::Time::now() - estop_start_time).toSec();
      
      if (PERF_LOG.isInitialized()) {
          PERF_LOG.logf("PLAN", "EMERGENCY_STOP", "count=%d, duration=%.1fs, hover=(%.2f,%.2f,%.2f), have_target=%d",
                        estop_count, estop_duration, hover_pos_.x(), hover_pos_.y(), hover_pos_.z(), have_target_);
      }
      ROS_WARN_THROTTLE(10.0, "[EGOReplanFSM] EMERGENCY STOP! Hovering at (%.2f, %.2f, %.2f) for %.1fs (see log for details)",
                        hover_pos_.x(), hover_pos_.y(), hover_pos_.z(), estop_duration);
      
      // ========== 恢复逻辑 ==========
      // 1. 检查当前位置是否安全
      star_mvf::Position curr_pos_f(static_cast<float>(odom_pos_.x()), 
                                    static_cast<float>(odom_pos_.y()), 
                                    static_cast<float>(odom_pos_.z()));
      bool current_pos_safe = !map_manager_->isOccupiedInflate(curr_pos_f);
      
      // 2. 悬停至少1秒后，如果当前位置安全且有新目标，尝试恢复
      if (estop_duration >= 1.0 && current_pos_safe && have_target_ && have_new_target_)
      {
        PERF_LOG.logf("PLAN", "ESTOP_RECOVERY", "Attempting recovery after %.1fs, pos_safe=%d, target=(%.2f,%.2f,%.2f)",
                      estop_duration, current_pos_safe, end_pt_.x(), end_pt_.y(), end_pt_.z());
        ROS_WARN("[EGOReplanFSM] Recovering from EMERGENCY_STOP after %.1fs - generating new trajectory", estop_duration);
        
        estop_count = 0;  // 重置计数
        have_new_target_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "RECOVERY");
      }
      // 3. 悬停超过5秒，即使没有新目标也检查是否可以切换到 WAIT_TARGET
      else if (estop_duration >= 5.0 && current_pos_safe)
      {
        PERF_LOG.logf("PLAN", "ESTOP_TIMEOUT", "Timeout after %.1fs, switching to WAIT_TARGET", estop_duration);
        ROS_WARN("[EGOReplanFSM] EMERGENCY_STOP timeout (%.1fs) - returning to WAIT_TARGET", estop_duration);
        
        estop_count = 0;  // 重置计数
        have_target_ = false;
        have_trigger_ = false;
        changeFSMExecState(WAIT_TARGET, "TIMEOUT_RECOVERY");
      }
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
    
    int pre_s = static_cast<int>(exec_state_);
    exec_state_ = new_state;
    
    // 详细状态转换日志（到性能日志文件）
    PERF_LOG.logf("PLAN", "STATE_CHANGE", "%s -> %s (by %s), pos=(%.2f,%.2f,%.2f), have_target=%d, have_trigger=%d",
                  state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str(), pos_call.c_str(),
                  odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), have_target_, have_trigger_);
    
    // 终端只输出重要的状态转换
    if (new_state == EMERGENCY_STOP || pre_s == static_cast<int>(EMERGENCY_STOP) || 
        pos_call == "RECOVERY" || pos_call == "TIMEOUT_RECOVERY")
    {
      ROS_WARN("[EGOReplanFSM] State: %s -> %s (by %s)",
               state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str(), pos_call.c_str());
    }
    else
    {
      ROS_INFO("[EGOReplanFSM] State: %s -> %s (by %s)",
               state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str(), pos_call.c_str());
    }
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static std::string state_str[8] = {"INIT", "PRE_ARM", "TAKEOFF", "WAIT_TARGET",
                                       "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    // 减少终端输出频率
    PERF_LOG.logf("PLAN", "STATE_INFO", "current=%s", state_str[static_cast<int>(exec_state_)].c_str());
    ROS_DEBUG("[EGOReplanFSM] Current state: %s", state_str[static_cast<int>(exec_state_)].c_str());
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
        star_mvf::Position pos_f(static_cast<float>(pos.x()), 
                                 static_cast<float>(pos.y()), 
                                 static_cast<float>(pos.z()));
        if (map_manager_->isOccupiedInflate(pos_f))
        {
          if (enable_fail_safe_)
          {
            // 详细碰撞日志
            PERF_LOG.logf("PLAN", "COLLISION_DETECT", 
                          "t=%.2f, traj_pos=(%.2f,%.2f,%.2f), odom_pos=(%.2f,%.2f,%.2f), "
                          "t_cur=%.2f, duration=%.2f, emergency_time=%.2f",
                          t, pos.x(), pos.y(), pos.z(),
                          odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
                          t_cur, info->duration, emergency_time_);
            
            // 检查当前位置是否也碰撞
            star_mvf::Position odom_pos_f(static_cast<float>(odom_pos_.x()), 
                                          static_cast<float>(odom_pos_.y()), 
                                          static_cast<float>(odom_pos_.z()));
            bool odom_collision = map_manager_->isOccupiedInflate(odom_pos_f);
            PERF_LOG.logf("PLAN", "COLLISION_DETAIL", 
                          "current_pos_collision=%d, will_stop_at=(%.2f,%.2f,%.2f)",
                          odom_collision, odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
            
            ROS_ERROR("[EGOReplanFSM] Collision detected at t=%.2f, pos=(%.2f,%.2f,%.2f), emergency stop!", 
                      t, pos.x(), pos.y(), pos.z());
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
    
    // 安全检查：确保紧急停止高度不低于起飞高度的一半
    double min_safe_height = initial_pos_.z() + takeoff_height_ * 0.5;
    if (hover_pos_.z() < min_safe_height)
    {
      ROS_WARN("[EGOReplanFSM] Emergency stop height %.2f is too low, correcting to %.2f",
               hover_pos_.z(), min_safe_height);
      hover_pos_.z() = min_safe_height;
    }
    
    planner_manager_->EmergencyStop(hover_pos_);  // 使用修正后的位置
    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // [优化] 规划前保存旧轨迹（用于衔接）
    savePrevTrajectory();
    
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
                                                   flag_use_poly_init, flag_randomPolyTraj, touch_goal_,
                                                   odom_yaw_, end_yaw_);  // 传递yaw参数以启用yaw优化
    
    if (success)
    {
      // [优化] 更新轨迹执行状态（不再发布poly_traj，改为executionCallback直接发布pos_cmd）
      auto& local_traj = planner_manager_->traj_.local_traj;
      traj_start_time_ = local_traj.start_time;
      traj_duration_ = local_traj.duration;
      
      // 同步 last_yaw_ 到新轨迹起点
      if (local_traj.use_yaw_traj && !local_traj.yaw_ctrl_pts.empty())
      {
        last_yaw_ = YawOptimizer::evaluateYaw(local_traj.yaw_ctrl_pts, 0.0, traj_duration_);
      }
      
      // Notify trajectory server (保留用于调试)
      std_msgs::Int8 replan_msg;
      replan_msg.data = 1;
      replan_pub_.publish(replan_msg);
    }
    
    return success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times)
  {
    // 关键修复：使用全局轨迹的起点作为局部轨迹的起点
    // 这确保了 viewpointCallback 中设置的起点状态被保留
    // 全局轨迹的起点已经在 viewpointCallback/waypointCallback 中正确计算过了
    
    double t_plan_start = ros::Time::now().toSec();
    
    GlobalTrajData *global_info = &planner_manager_->traj_.global_traj;
    if (global_info->traj.getPieceNum() > 0)
    {
      // 使用全局轨迹在 t=0 时刻的状态（与 viewpointCallback 中的 start_pos 一致）
      start_pt_ = global_info->traj.getPos(0);
      start_vel_ = global_info->traj.getVel(0);
      start_acc_ = global_info->traj.getAcc(0);
      
      // 使用全局轨迹的开始时间（已在 viewpointCallback 中设置为 t_plan_start + replan_duration）
      // 不需要在这里再次设置 setReplanStartTime，因为已经在 viewpointCallback 中设置了
    }
    else
    {
      // 后备：如果没有全局轨迹，使用里程计数据
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_ = Eigen::Vector3d::Zero();
      planner_manager_->setReplanStartTime(t_plan_start + replan_duration_);
    }
    
    // 安全检查：如果起点位置在障碍物内，使用当前里程计位置
    star_mvf::Position start_pos_f(static_cast<float>(start_pt_.x()), 
                                    static_cast<float>(start_pt_.y()), 
                                    static_cast<float>(start_pt_.z()));
    if (map_manager_ && map_manager_->isOccupiedInflate(start_pos_f))
    {
      ROS_WARN("[EGOReplanFSM] Global traj start (%.2f,%.2f,%.2f) is in obstacle, using odom position",
               start_pt_.x(), start_pt_.y(), start_pt_.z());
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_ = Eigen::Vector3d::Zero();
      planner_manager_->setReplanStartTime(t_plan_start + replan_duration_);
    }

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
    
    // 记录规划开始时间（关键：用于确保 start_time 与预测状态匹配）
    double t_plan_start = ros::Time::now().toSec();
    double t_cur = t_plan_start - info->start_time;
    
    // 预测 t_plan_start + replan_duration 时刻的状态作为新轨迹起点
    // 新轨迹的 start_time 也应该设为这个时刻
    double t_predict = t_cur + replan_duration_;
    t_predict = std::max(0.0, std::min(t_predict, info->traj.getTotalDuration()));

    start_pt_ = info->traj.getPos(t_predict);
    start_vel_ = info->traj.getVel(t_predict);
    start_acc_ = info->traj.getAcc(t_predict);
    
    // 安全检查：如果预测的起点位置在障碍物内，使用当前里程计位置
    star_mvf::Position start_pos_f(static_cast<float>(start_pt_.x()), 
                                    static_cast<float>(start_pt_.y()), 
                                    static_cast<float>(start_pt_.z()));
    if (map_manager_ && map_manager_->isOccupiedInflate(start_pos_f))
    {
      ROS_WARN("[EGOReplanFSM] Predicted start (%.2f,%.2f,%.2f) is in obstacle, using odom position",
               start_pt_.x(), start_pt_.y(), start_pt_.z());
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_ = Eigen::Vector3d::Zero();
    }
    
    // 设置新轨迹的精确 start_time（与预测状态匹配）
    // start_time = 规划开始时间 + replan_duration（而非规划完成时间 + replan_duration）
    planner_manager_->setReplanStartTime(t_plan_start + replan_duration_);

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

    // 记录规划开始时间（关键：用于确保 start_time 与预测状态匹配）
    double t_plan_start = ros::Time::now().toSec();

    // 确定起点状态：
    // 关键：使用预测位置！因为规划需要时间，在规划期间无人机继续飞行
    Eigen::Vector3d start_pos, start_vel, start_acc;
    if (exec_state_ == EXEC_TRAJ && planner_manager_->traj_.local_traj.traj_id > 0)
    {
      // 正在执行轨迹：预测到 t_plan_start + replan_duration 时的轨迹状态
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = t_plan_start - info->start_time;
      double t_predict = t_cur + replan_duration_;
      t_predict = std::max(0.0, std::min(t_predict, info->traj.getTotalDuration()));
      start_pos = info->traj.getPos(t_predict);
      start_vel = info->traj.getVel(t_predict);
      start_acc = info->traj.getAcc(t_predict);
    }
    else
    {
      // 没有正在执行的轨迹：使用里程计位置 + 速度预测
      start_pos = odom_pos_ + odom_vel_ * replan_duration_;
      start_vel = odom_vel_;
      start_acc = Eigen::Vector3d::Zero();
    }
    
    // 设置精确的新轨迹开始时间（与预测状态匹配）
    planner_manager_->setReplanStartTime(t_plan_start + replan_duration_);

    // Plan global trajectory with smooth start from predicted position
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(end_pt_);
    planner_manager_->planGlobalTrajWaypoints(start_pos, start_vel, start_acc,
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

    // 记录规划开始时间（关键：用于确保 start_time 与预测状态匹配）
    double t_plan_start = ros::Time::now().toSec();
    
    // 确定起点状态：
    // 关键：使用预测位置！因为规划需要时间，在规划期间无人机继续飞行
    Eigen::Vector3d start_pos, start_vel, start_acc;
    if (exec_state_ == EXEC_TRAJ && planner_manager_->traj_.local_traj.traj_id > 0)
    {
      // 正在执行轨迹：预测到 t_plan_start + replan_duration 时的轨迹状态
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = t_plan_start - info->start_time;
      double t_predict = t_cur + replan_duration_;
      t_predict = std::max(0.0, std::min(t_predict, info->traj.getTotalDuration()));
      start_pos = info->traj.getPos(t_predict);
      start_vel = info->traj.getVel(t_predict);
      start_acc = info->traj.getAcc(t_predict);
    }
    else
    {
      // 没有正在执行的轨迹：使用里程计位置 + 速度预测
      start_pos = odom_pos_ + odom_vel_ * replan_duration_;
      start_vel = odom_vel_;
      start_acc = Eigen::Vector3d::Zero();
    }
    
    // 设置精确的新轨迹开始时间（与预测状态匹配）
    planner_manager_->setReplanStartTime(t_plan_start + replan_duration_);

    // Plan global trajectory with smooth start from predicted position
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(end_pt_);
    planner_manager_->planGlobalTrajWaypoints(start_pos, start_vel, start_acc,
                                               waypoints, end_vel_, Eigen::Vector3d::Zero());

    have_target_ = true;
    have_trigger_ = true;
    have_new_target_ = true;

    PERF_LOG.logf("PLAN", "RECV_VP", "pos=(%.2f,%.2f,%.2f), yaw=%.2f, vel=(%.2f,%.2f,%.2f), state=%d", 
                  end_pt_.x(), end_pt_.y(), end_pt_.z(), end_yaw_,
                  end_vel_.x(), end_vel_.y(), end_vel_.z(), static_cast<int>(exec_state_));

    if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ)
    {
      changeFSMExecState(GEN_NEW_TRAJ, "VIEWPOINT");
    }
    else if (exec_state_ == EMERGENCY_STOP)
    {
      // 在 EMERGENCY_STOP 状态下收到新视点，标记为待恢复
      // 恢复逻辑在 EMERGENCY_STOP case 中处理
      PERF_LOG.logf("PLAN", "RECV_VP_ESTOP", "New target received during EMERGENCY_STOP, waiting for recovery");
      ROS_INFO("[EGOReplanFSM] New viewpoint received during EMERGENCY_STOP - will attempt recovery");
    }
  }

  void EGOReplanFSM::viewpointPathCallback(const nav_msgs::PathConstPtr &msg)
  {
    // 接收带中间航点的路径（用于绕过障碍物）
    if (msg->poses.empty()) {
      ROS_WARN("[EGOReplanFSM] Received empty viewpoint path");
      return;
    }
    
    // 解析所有航点
    wps_.clear();
    for (size_t i = 0; i < msg->poses.size(); ++i) {
      const auto& pose = msg->poses[i].pose;
      Eigen::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);
      
      // 从 quaternion 提取 yaw
      double yaw = 0.0;
      tf::Quaternion q(pose.orientation.x, pose.orientation.y, 
                       pose.orientation.z, pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch;
      m.getRPY(roll, pitch, yaw);
      
      // 对于中间航点，不强制 yaw（设为 NaN 表示跟随速度方向）
      // 只有最后一个航点使用精确 yaw
      Eigen::Vector3d vel = Eigen::Vector3d::Zero();
      if (i == msg->poses.size() - 1) {
        // 最后一个航点：使用精确 yaw，终端速度为 0
        wps_.push_back(Waypoint(pos, yaw, vel));
      } else {
        // 中间航点：yaw 跟随速度方向
        wps_.push_back(Waypoint(pos, std::nan(""), vel));
      }
    }
    
    wpt_id_ = 0;
    
    // 获取最终目标
    end_pt_ = wps_.back().pos;
    end_yaw_ = wps_.back().yaw;
    end_vel_ = wps_.back().vel;
    
    // 记录规划开始时间
    double t_plan_start = ros::Time::now().toSec();
    
    // 确定起点状态
    Eigen::Vector3d start_pos, start_vel, start_acc;
    if (exec_state_ == EXEC_TRAJ && planner_manager_->traj_.local_traj.traj_id > 0)
    {
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = t_plan_start - info->start_time;
      double t_predict = t_cur + replan_duration_;
      t_predict = std::max(0.0, std::min(t_predict, info->traj.getTotalDuration()));
      start_pos = info->traj.getPos(t_predict);
      start_vel = info->traj.getVel(t_predict);
      start_acc = info->traj.getAcc(t_predict);
    }
    else
    {
      start_pos = odom_pos_ + odom_vel_ * replan_duration_;
      start_vel = odom_vel_;
      start_acc = Eigen::Vector3d::Zero();
    }
    
    planner_manager_->setReplanStartTime(t_plan_start + replan_duration_);
    
    // 构建所有航点位置列表
    std::vector<Eigen::Vector3d> waypoints;
    for (const auto& wp : wps_) {
      waypoints.push_back(wp.pos);
    }
    
    // 使用多航点规划全局轨迹
    planner_manager_->planGlobalTrajWaypoints(start_pos, start_vel, start_acc,
                                               waypoints, end_vel_, Eigen::Vector3d::Zero());
    
    have_target_ = true;
    have_trigger_ = true;
    have_new_target_ = true;
    
    PERF_LOG.logf("PLAN", "RECV_VP_PATH", "waypoints=%zu, target=(%.2f,%.2f,%.2f), yaw=%.2f, state=%d",
                  wps_.size(), end_pt_.x(), end_pt_.y(), end_pt_.z(), end_yaw_, static_cast<int>(exec_state_));
    
    if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ)
    {
      changeFSMExecState(GEN_NEW_TRAJ, "VIEWPOINT_PATH");
    }
    else if (exec_state_ == EMERGENCY_STOP)
    {
      // 在 EMERGENCY_STOP 状态下收到新路径，标记为待恢复
      PERF_LOG.logf("PLAN", "RECV_VP_PATH_ESTOP", "New path received during EMERGENCY_STOP, waiting for recovery");
      ROS_INFO("[EGOReplanFSM] New viewpoint path received during EMERGENCY_STOP - will attempt recovery");
    }
  }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    wps_.clear();
    for (int i = 0; i < waypoint_num_; i++)
    {
      // 从参数读取的yaw值（NaN表示跟随速度方向）
      Eigen::Vector3d pos(waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
      wps_.push_back(Waypoint(pos, waypoints_yaw_[i], Eigen::Vector3d::Zero()));
      
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
    
    // 计算start_yaw和end_yaw
    double start_yaw = previous_wp.yaw;
    double end_yaw = next_wp.yaw;
    
    // 如果yaw是NaN，使用当前的odom_yaw_作为起点，或使用速度方向（10000.0表示自动）
    if (std::isnan(start_yaw))
      start_yaw = odom_yaw_;
    if (std::isnan(end_yaw))
      end_yaw = 10000.0;  // 特殊值表示跟随速度方向
    
    // Use previous waypoint's velocity as start, next waypoint's velocity as end
    planner_manager_->planGlobalTrajWaypoints(previous_wp.pos, previous_wp.vel, Eigen::Vector3d::Zero(),
                                             waypoints, next_wp.vel, Eigen::Vector3d::Zero(),
                                             start_yaw, end_yaw);
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
    
    // 获取四元数
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // Extract yaw from quaternion using stable method (avoids gimbal lock)
    double qw = odom_orient_.w();
    double qx = odom_orient_.x();
    double qy = odom_orient_.y();
    double qz = odom_orient_.z();
    odom_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    odom_yaw_vel_ = msg->twist.twist.angular.z;

    // 将机体坐标系速度转换到世界坐标系
    // 里程计的twist通常是机体坐标系，需要用四元数旋转到世界坐标系
    Eigen::Vector3d vel_body(msg->twist.twist.linear.x, 
                              msg->twist.twist.linear.y, 
                              msg->twist.twist.linear.z);
    odom_vel_ = odom_orient_ * vel_body;  // 四元数旋转: 机体系 -> 世界系

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    ROS_INFO("[EGOReplanFSM] Trigger received!");
  }

  void EGOReplanFSM::polyTraj2ROSMsg(semantic_exploration::PolyTraj &poly_msg)
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
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = 0;
    
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
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = 0;
    
    pos_cmd_pub_.publish(cmd);
  }

  // ==================== 轨迹执行相关函数 ====================
  
  void EGOReplanFSM::savePrevTrajectory()
  {
    auto& local_traj = planner_manager_->traj_.local_traj;
    if (local_traj.traj.getPieceNum() > 0)
    {
      has_prev_traj_ = true;
      prev_traj_start_time_ = traj_start_time_;
      prev_traj_duration_ = traj_duration_;
      prev_traj_ = local_traj.traj;
      prev_use_yaw_traj_ = local_traj.use_yaw_traj;
      prev_last_yaw_ = last_yaw_;
      if (local_traj.use_yaw_traj && !local_traj.yaw_ctrl_pts.empty())
      {
        prev_yaw_ctrl_pts_ = local_traj.yaw_ctrl_pts;
      }
      else
      {
        prev_yaw_ctrl_pts_.clear();
      }
    }
  }

  void EGOReplanFSM::executionCallback(const ros::TimerEvent &e)
  {
    // 只在有效状态下执行轨迹
    if (!have_odom_ || exec_state_ == INIT || exec_state_ == PRE_ARM || 
        exec_state_ == TAKEOFF || exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP)
    {
      return;
    }
    
    ros::Time now = ros::Time::now();
    auto& local_traj = planner_manager_->traj_.local_traj;
    
    // 无有效轨迹时悬停
    if (local_traj.traj.getPieceNum() == 0)
    {
      return;  // publishHoverCmd()由其他回调处理
    }
    
    double t_cur = now.toSec() - traj_start_time_;
    
    // Case 1: 过渡期（新轨迹还没开始）-> 执行旧轨迹
    if (t_cur < 0 && has_prev_traj_)
    {
      double t_prev = now.toSec() - prev_traj_start_time_;
      if (prev_traj_.getPieceNum() > 0 && t_prev >= 0 && t_prev < prev_traj_duration_)
      {
        double t_eval = std::min(t_prev, prev_traj_.getTotalDuration());
        Eigen::Vector3d pos = prev_traj_.getPos(t_eval);
        Eigen::Vector3d vel = prev_traj_.getVel(t_eval);
        Eigen::Vector3d acc = prev_traj_.getAcc(t_eval);
        
        double yaw = last_yaw_, yaw_dot = 0.0;
        if (prev_use_yaw_traj_ && !prev_yaw_ctrl_pts_.empty())
        {
          yaw = YawOptimizer::evaluateYaw(prev_yaw_ctrl_pts_, t_eval, prev_traj_duration_);
          yaw_dot = YawOptimizer::evaluateYawDot(prev_yaw_ctrl_pts_, t_eval, prev_traj_duration_);
        }
        
        last_pos_ = pos;
        last_yaw_ = yaw;
        last_yaw_dot_ = yaw_dot;
        publishCmd(pos, vel, acc, yaw, yaw_dot);
        return;
      }
    }
    
    // Case 2: 正常执行当前轨迹
    if (t_cur >= 0 && t_cur < traj_duration_)
    {
      double t_eval = std::min(t_cur, local_traj.traj.getTotalDuration());
      Eigen::Vector3d pos = local_traj.traj.getPos(t_eval);
      Eigen::Vector3d vel = local_traj.traj.getVel(t_eval);
      Eigen::Vector3d acc = local_traj.traj.getAcc(t_eval);
      
      double yaw = last_yaw_, yaw_dot = 0.0;
      if (local_traj.use_yaw_traj && !local_traj.yaw_ctrl_pts.empty())
      {
        yaw = YawOptimizer::evaluateYaw(local_traj.yaw_ctrl_pts, t_eval, traj_duration_);
        yaw_dot = YawOptimizer::evaluateYawDot(local_traj.yaw_ctrl_pts, t_eval, traj_duration_);
      }
      
      last_pos_ = pos;
      last_yaw_ = yaw;
      last_yaw_dot_ = yaw_dot;
      publishCmd(pos, vel, acc, yaw, yaw_dot);
      
      // 降频可视化（每10帧一次）
      static int vis_count = 0;
      if (++vis_count >= 10)
      {
        vis_count = 0;
        visualizeTrajectory();
      }
      return;
    }
    
    // Case 3: 轨迹执行完毕 -> 由FSM状态机处理（不在这里悬停，避免冲突）
  }

  void EGOReplanFSM::publishCmd(const Eigen::Vector3d& p, const Eigen::Vector3d& v, 
                                 const Eigen::Vector3d& a, double yaw, double yaw_dot)
  {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    
    cmd.position.x = p.x();
    cmd.position.y = p.y();
    cmd.position.z = p.z();
    cmd.velocity.x = v.x();
    cmd.velocity.y = v.y();
    cmd.velocity.z = v.z();
    cmd.acceleration.x = a.x();
    cmd.acceleration.y = a.y();
    cmd.acceleration.z = a.z();
    cmd.yaw = yaw;
    cmd.yaw_dot = yaw_dot;
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = 0;
    
    pos_cmd_pub_.publish(cmd);
  }

  void EGOReplanFSM::visualizeTrajectory()
  {
    auto& local_traj = planner_manager_->traj_.local_traj;
    if (local_traj.traj.getPieceNum() == 0 || local_traj.duration <= 0)
      return;
    
    visualization_msgs::MarkerArray marker_array;
    
    // ========== 1. 轨迹线 ==========
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.ns = "trajectory";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = 0.08;
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 0.5;
    traj_marker.color.b = 1.0;
    traj_marker.color.a = 1.0;
    
    for (double t = 0; t < local_traj.duration; t += 0.1)
    {
      Eigen::Vector3d pos = local_traj.traj.getPos(t);
      if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z()))
        continue;
      geometry_msgs::Point pt;
      pt.x = pos.x();
      pt.y = pos.y();
      pt.z = pos.z();
      traj_marker.points.push_back(pt);
    }
    if (!traj_marker.points.empty())
      marker_array.markers.push_back(traj_marker);
    
    // ========== 2. 终点Yaw视锥体（箭头表示朝向） ==========
    if (!std::isnan(end_yaw_))
    {
      visualization_msgs::Marker goal_yaw_marker;
      goal_yaw_marker.header.frame_id = "map";
      goal_yaw_marker.header.stamp = ros::Time::now();
      goal_yaw_marker.ns = "goal_yaw";
      goal_yaw_marker.id = 1;
      goal_yaw_marker.type = visualization_msgs::Marker::ARROW;
      goal_yaw_marker.action = visualization_msgs::Marker::ADD;
      
      // 箭头起点和终点
      geometry_msgs::Point start, end;
      start.x = end_pt_.x();
      start.y = end_pt_.y();
      start.z = end_pt_.z();
      double arrow_len = 1.0;
      end.x = end_pt_.x() + arrow_len * cos(end_yaw_);
      end.y = end_pt_.y() + arrow_len * sin(end_yaw_);
      end.z = end_pt_.z();
      goal_yaw_marker.points.push_back(start);
      goal_yaw_marker.points.push_back(end);
      
      goal_yaw_marker.scale.x = 0.15;  // 箭杆直径
      goal_yaw_marker.scale.y = 0.25;  // 箭头直径
      goal_yaw_marker.scale.z = 0.0;
      goal_yaw_marker.color.r = 1.0;
      goal_yaw_marker.color.g = 0.3;
      goal_yaw_marker.color.b = 0.0;
      goal_yaw_marker.color.a = 1.0;
      marker_array.markers.push_back(goal_yaw_marker);
    }
    
    // ========== 3. 无人机实时Yaw视锥体 ==========
    visualization_msgs::Marker fov_marker;
    fov_marker.header.frame_id = "map";
    fov_marker.header.stamp = ros::Time::now();
    fov_marker.ns = "uav_fov";
    fov_marker.id = 2;
    fov_marker.type = visualization_msgs::Marker::LINE_LIST;
    fov_marker.action = visualization_msgs::Marker::ADD;
    fov_marker.pose.orientation.w = 1.0;
    fov_marker.scale.x = 0.05;  // 线宽
    fov_marker.color.r = 0.0;
    fov_marker.color.g = 1.0;
    fov_marker.color.b = 0.0;
    fov_marker.color.a = 0.8;
    
    // 视锥体参数
    double fov_h = 0.8;   // 水平FOV半角 (~70度)
    double fov_v = 0.4;   // 垂直FOV半角 (~35度)
    double fov_len = 1.5; // 视锥体深度
    
    Eigen::Vector3d cam_pos = odom_pos_;
    double yaw = odom_yaw_;
    
    // 计算视锥体4个角点
    Eigen::Vector3d corners[4];
    double dx = fov_len * cos(yaw);
    double dy = fov_len * sin(yaw);
    double half_w = fov_len * tan(fov_h);  // 水平半宽
    double half_h = fov_len * tan(fov_v);  // 垂直半高
    
    // 右上、左上、左下、右下
    corners[0] = cam_pos + Eigen::Vector3d(dx - half_w * sin(yaw), dy + half_w * cos(yaw), half_h);
    corners[1] = cam_pos + Eigen::Vector3d(dx + half_w * sin(yaw), dy - half_w * cos(yaw), half_h);
    corners[2] = cam_pos + Eigen::Vector3d(dx + half_w * sin(yaw), dy - half_w * cos(yaw), -half_h);
    corners[3] = cam_pos + Eigen::Vector3d(dx - half_w * sin(yaw), dy + half_w * cos(yaw), -half_h);
    
    // 从相机位置到4个角点的线
    for (int i = 0; i < 4; ++i) {
      geometry_msgs::Point p1, p2;
      p1.x = cam_pos.x(); p1.y = cam_pos.y(); p1.z = cam_pos.z();
      p2.x = corners[i].x(); p2.y = corners[i].y(); p2.z = corners[i].z();
      fov_marker.points.push_back(p1);
      fov_marker.points.push_back(p2);
    }
    // 4个角点连成的矩形框
    for (int i = 0; i < 4; ++i) {
      geometry_msgs::Point p1, p2;
      p1.x = corners[i].x(); p1.y = corners[i].y(); p1.z = corners[i].z();
      p2.x = corners[(i+1)%4].x(); p2.y = corners[(i+1)%4].y(); p2.z = corners[(i+1)%4].z();
      fov_marker.points.push_back(p1);
      fov_marker.points.push_back(p2);
    }
    marker_array.markers.push_back(fov_marker);
    
    traj_vis_pub_.publish(marker_array);
  }

} // namespace ego_planner

