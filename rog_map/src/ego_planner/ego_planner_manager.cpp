#include "ego_planner/ego_planner_manager.h"
#include <thread>

namespace ego_planner
{

  EGOPlannerManager::EGOPlannerManager() {
    yaw_optimizer_ = std::make_unique<YawOptimizer>();
  }

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "EGOPlannerManager destroyed" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, star_mvf::MapManager::Ptr map)
  {
    /* 从统一的 dynamics 部分读取参数，若无则从 manager 部分读取（向后兼容） */
    nh.param("manager/max_vel", pp_.max_vel_, 2.0);
    nh.param("manager/max_acc", pp_.max_acc_, 2.0);
    // nh.param("manager/max_yaw_rate", pp_.max_yaw_rate_, 1.0);
    // nh.param("manager/max_yaw_acc", pp_.max_yaw_acc_, 1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.05);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, 1.5);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 7.5);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);

    map_manager_ = map;

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(map_manager_);

    ROS_INFO("[EGOPlannerManager] Initialized with:");
    ROS_INFO("  max_vel: %.2f, max_acc: %.2f", pp_.max_vel_, pp_.max_acc_);
    ROS_INFO("  planning_horizon: %.2f, piece_length: %.2f", pp_.planning_horizen_, pp_.polyTraj_piece_length);
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the init of init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initializing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if ( t_to_lc_end < 0 )
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal,
                                               const double time_offset)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret)
    {
      double start_time;
      if (use_precise_start_time_)
      {
        // 使用精确的开始时间（与预测状态完全匹配）
        start_time = replan_start_time_;
        use_precise_start_time_ = false;  // 重置标志
        double offset = start_time - ros::Time::now().toSec();
        ROS_INFO("[PlannerMgr] Using precise start_time (offset=%.3f s)", offset);
      }
      else
      {
        // 后备方案：使用当前时间 + 偏移
        start_time = ros::Time::now().toSec() + time_offset;
        ROS_WARN("[PlannerMgr] Using fallback start_time (offset=%.3f s)", time_offset);
      }
      traj_.setLocalTraj(traj, pts_to_check, start_time);
    }

    return ret;
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal,
      double start_yaw, double end_yaw)
  {
    // 检查优化器是否初始化
    if (!ploy_traj_opt_) {
      ROS_ERROR("[EGO_REPLAN] ploy_traj_opt_ is NULL!");
      return false;
    }

    ploy_traj_opt_->setIfTouchGoal(touch_goal);

    ros::WallTime t_start = ros::WallTime::now();
    ros::WallDuration t_init, t_opt;

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO;
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, initMJO))
    {
      return false;
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    std::vector<std::pair<int, int>> segments;
    auto check_ret = ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true);
    if (check_ret == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = ros::WallTime::now() - t_start;

    t_start = ros::WallTime::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    poly_traj::MinJerkOpt best_MJO;

    poly_traj::Trajectory initTraj = initMJO.getTraj();
    int PN = initTraj.getPieceNum();
    
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
    tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
    double final_cost;
    flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                      innerPts, initTraj.getDurations(),
                                                      cstr_pts, final_cost);
    best_MJO = ploy_traj_opt_->getMinJerkOpt();

    t_opt = ros::WallTime::now() - t_start;

    // ====================================================================================
    // [ADD] RETRY LOGIC for Long Detours / Infeasible Constraints
    // ====================================================================================
    // 定义扩展时间步长，供多次重试复用
    double ts_extended = ts * 1.5;

    if (!flag_success)
    {
        ROS_WARN("[EGOPlanner] Optimization failed! Retrying with extended time step...");
        
        // 策略：将时间分配扩大 1.5 倍，降低动力学要求
        poly_traj::MinJerkOpt initMJO_retry;
        // 使用新的时间片长度重新计算初值
        bool init_success = computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                                            flag_polyInit, flag_randomPolyTraj, ts_extended, initMJO_retry);
        
        if (init_success)
        {
             // 必须重新生成约束点，因为轨迹形状和采样点分布变了
             std::vector<std::pair<int, int>> segments_retry;
             Eigen::MatrixXd cstr_pts_retry = initMJO_retry.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
             
             auto check_ret_retry = ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments_retry, initMJO_retry, true);
             if (check_ret_retry != PolyTrajOptimizer::CHK_RET::ERR)
             {
                 poly_traj::Trajectory initTrajRetry = initMJO_retry.getTraj();
                 int PN_retry = initTrajRetry.getPieceNum();
                 Eigen::MatrixXd all_pos_retry = initTrajRetry.getPositions();
                 Eigen::MatrixXd innerPts_retry = all_pos_retry.block(0, 1, 3, PN_retry - 1);
                 
                 Eigen::Matrix<double, 3, 3> headState_retry, tailState_retry;
                 headState_retry << initTrajRetry.getJuncPos(0), initTrajRetry.getJuncVel(0), initTrajRetry.getJuncAcc(0);
                 tailState_retry << initTrajRetry.getJuncPos(PN_retry), initTrajRetry.getJuncVel(PN_retry), initTrajRetry.getJuncAcc(PN_retry);
                 
                 double final_cost_retry;
                 bool retry_success = ploy_traj_opt_->optimizeTrajectory(headState_retry, tailState_retry, 
                                                                       innerPts_retry, initTrajRetry.getDurations(), 
                                                                       cstr_pts_retry, final_cost_retry);
                                                                       
                 if (retry_success) {
                     ROS_INFO("[EGOPlanner] Retry SUCCESS! Detour planned.");
                     flag_success = true;
                     best_MJO = ploy_traj_opt_->getMinJerkOpt();
                 }
             }
        }
    }

    // [New] 第二次重试：如果单纯延长时间不够，尝试引入随机扰动 (Random Perturbation)
    // 应对直连路径正好穿过障碍物中心导致的梯度为零/局部极小值问题
    if (!flag_success)
    {
         ROS_WARN("[EGOPlanner] Retry 1 Failed. Trying Random Perturbation...");
         poly_traj::MinJerkOpt initMJO_random;
         bool init_random_success = computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                                       flag_polyInit, true, ts_extended, initMJO_random); // flag_random=true

         if (init_random_success)
         {
             std::vector<std::pair<int, int>> segments_random;
             Eigen::MatrixXd cstr_pts_random = initMJO_random.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
             auto check_ret_random = ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments_random, initMJO_random, true);
             
             if (check_ret_random != PolyTrajOptimizer::CHK_RET::ERR)
             {
                 poly_traj::Trajectory initTrajRandom = initMJO_random.getTraj();
                 int PN_rand = initTrajRandom.getPieceNum();
                 Eigen::MatrixXd all_pos_rand = initTrajRandom.getPositions();
                 Eigen::MatrixXd innerPts_rand = all_pos_rand.block(0, 1, 3, PN_rand - 1);
                 
                 Eigen::Matrix<double, 3, 3> headState_rand, tailState_rand;
                 headState_rand << initTrajRandom.getJuncPos(0), initTrajRandom.getJuncVel(0), initTrajRandom.getJuncAcc(0);
                 tailState_rand << initTrajRandom.getJuncPos(PN_rand), initTrajRandom.getJuncVel(PN_rand), initTrajRandom.getJuncAcc(PN_rand);
                 
                 double final_cost_rand;
                 bool rand_success = ploy_traj_opt_->optimizeTrajectory(headState_rand, tailState_rand, 
                                                                       innerPts_rand, initTrajRandom.getDurations(), 
                                                                       cstr_pts_random, final_cost_rand);
                 if (rand_success) {
                     ROS_INFO("[EGOPlanner] Retry 2 (Random) SUCCESS!");
                     flag_success = true;
                     best_MJO = ploy_traj_opt_->getMinJerkOpt();
                 }
             }
         }
    }

    // save and display planned results
    if (!flag_success)
    {
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    // ROS_INFO("total time: %.3f, init: %.3f, optimize: %.3f, avg_time=%.3f",
    //          (t_init + t_opt).toSec(), t_init.toSec(), t_opt.toSec(), sum_time / count_success);

    // FALCON风格：使用 replan_time_offset_ 延迟新轨迹生效
    setLocalTrajFromOpt(best_MJO, touch_goal, replan_time_offset_);

    // [Innovation] Yaw Trajectory Optimization (4D Planning)
    // Only optimize if end_yaw is valid (passed from FSM)
    if (yaw_optimizer_ && end_yaw < 9999.0)
    {
       // Use start_yaw passed from FSM (or default 0 which assumes aligned?)
       // Actually if start_yaw is 0.0 and drone is at PI, this is bad.
       // We trust FSM passes valid start_yaw.
       
       std::vector<double> yaw_cps;
       bool yaw_success = yaw_optimizer_->optimize(traj_.local_traj.traj, 
                                                  start_yaw, end_yaw, yaw_cps);
       
       if (yaw_success) {
           traj_.local_traj.use_yaw_traj = true;
           traj_.local_traj.yaw_ctrl_pts = yaw_cps;
           traj_.local_traj.yaw_start_time = traj_.local_traj.start_time;
           // ROS_INFO("[EGOPlanner] Yaw Optimization SUCCESS. Start=%.2f, End=%.2f", start_yaw, end_yaw);
       } else {
           traj_.local_traj.use_yaw_traj = false;
           ROS_WARN("[EGOPlanner] Yaw Optimization FAILED. Fallback to velocity alignment.");
       }
    } else {
        traj_.local_traj.use_yaw_traj = false;
    }

    // success
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false);

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
      double start_yaw, double end_yaw)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {
      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 1)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());
    
    // 同时设置为local_traj以便执行和yaw优化
    traj_.local_traj.traj = globalMJO.getTraj();
    traj_.local_traj.start_time = time_now.toSec();
    traj_.local_traj.duration = globalMJO.getTraj().getTotalDuration();
    
    // [新增] Yaw轨迹优化 (适用于重建任务)
    if (yaw_optimizer_ && end_yaw < 9999.0)
    {
        std::vector<double> yaw_cps;
        bool yaw_success = yaw_optimizer_->optimize(traj_.local_traj.traj, 
                                                    start_yaw, end_yaw, yaw_cps);
        
        if (yaw_success) {
            traj_.local_traj.use_yaw_traj = true;
            traj_.local_traj.yaw_ctrl_pts = yaw_cps;
            traj_.local_traj.yaw_start_time = traj_.local_traj.start_time;
            ROS_INFO("[EGOPlanner] Global trajectory Yaw optimized. Start=%.2f, End=%.2f", 
                     start_yaw, end_yaw);
        } else {
            traj_.local_traj.use_yaw_traj = false;
            ROS_WARN("[EGOPlanner] Global Yaw Optimization FAILED.");
        }
    } else {
        traj_.local_traj.use_yaw_traj = false;
    }

    return true;
  }


} // namespace ego_planner

