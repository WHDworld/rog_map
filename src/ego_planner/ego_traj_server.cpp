#include <cmath>
#include <nav_msgs/Odometry.h>
#include <semantic_exploration/PolyTraj.h>
#include "ego_planner/optimizer/poly_traj_utils.hpp"
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace Eigen;

ros::Publisher pos_cmd_pub;
ros::Publisher traj_vis_pub;

quadrotor_msgs::PositionCommand cmd;

bool receive_traj_ = false;
bool task_finished_ = false;

// ===== 双轨迹缓存机制 (FALCON风格) =====
// 新轨迹
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_ = -1;
double start_yaw_ = 0.0, end_yaw_ = 0.0;

// 旧轨迹（用于过渡期间）
boost::shared_ptr<poly_traj::Trajectory> prev_traj_;
double prev_traj_duration_;
ros::Time prev_start_time_;
double prev_start_yaw_ = 0.0, prev_end_yaw_ = 0.0;
bool has_prev_traj_ = false;

int last_traj_id_ = -1;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;
Eigen::Vector3d odom_pos_;

// yaw control
double last_yaw_ = 0.0, last_yawdot_ = 0.0;
double time_forward_;
bool yaw_initialized_ = false;

// target yaw (from planner)
bool use_target_yaw_ = false;  // If true, interpolate to target yaw; otherwise, follow velocity direction

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void replanCallback(const std_msgs::Int8ConstPtr &msg)
{
  if (msg->data == 2)
  {
    // Task finished signal
    task_finished_ = true;
    receive_traj_ = false;
    ROS_INFO("[EGOTrajServer] Task finished, stopping trajectory execution");
  }
  else if (msg->data == 1)
  {
    task_finished_ = false;
  }
}

void polyTrajCallback(semantic_exploration::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[EGOTrajServer] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[EGOTrajServer] WRONG trajectory parameters");
    return;
  }

  // ===== 保存旧轨迹用于过渡期间 (FALCON风格) =====
  if (traj_ != nullptr && receive_traj_)
  {
    prev_traj_ = traj_;
    prev_start_time_ = start_time_;
    prev_traj_duration_ = traj_duration_;
    prev_start_yaw_ = start_yaw_;
    prev_end_yaw_ = end_yaw_;
    has_prev_traj_ = true;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  last_traj_id_ = traj_id_;
  traj_id_ = msg->traj_id;
  
  // 调试：检查 start_time 是否在未来
  double time_offset = (start_time_ - ros::Time::now()).toSec();
  if (time_offset > 0.01)
  {
    ROS_INFO("[EGOTrajServer] New traj id=%d starts in %.3f s (FALCON mode)", traj_id_, time_offset);
  }
  else
  {
    ROS_WARN("[EGOTrajServer] New traj id=%d starts immediately (offset=%.3f s) - NOT delayed!", traj_id_, time_offset);
  }

  // Extract yaw info
  // 关键：start_yaw 使用当前实际的 last_yaw_，确保yaw连续性！
  // 这类似于FALCON的边界约束机制
  if (yaw_initialized_ && receive_traj_)
  {
    start_yaw_ = last_yaw_;  // 从当前yaw开始，保证连续
  }
  else
  {
    start_yaw_ = msg->start_yaw;  // 第一次使用消息中的值
  }
  end_yaw_ = msg->end_yaw;
  
  // Check if target yaw is specified:
  // - If end_yaw is NaN, it means "follow velocity direction" (preset waypoint mode)
  // - Otherwise, ALWAYS interpolate to target yaw (exploration mode needs smooth yaw transition)
  if (std::isnan(end_yaw_))
  {
    use_target_yaw_ = false;  // Follow velocity direction
  }
  else
  {
    // Always use target yaw interpolation for exploration mode
    use_target_yaw_ = true;
  }

  receive_traj_ = true;
  task_finished_ = false;
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  
  if (!yaw_initialized_)
  {
    // Extract yaw from quaternion using stable method (avoids gimbal lock)
    double qw = msg->pose.pose.orientation.w;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    last_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw_initialized_ = true;
  }
}

// Normalize angle to [-PI, PI]
double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

// Calculate shortest angle difference
double angleDiff(double target, double current)
{
  double diff = target - current;
  return normalizeAngle(diff);
}

// 可选的轨迹指针，用于过渡期间使用旧轨迹
boost::shared_ptr<poly_traj::Trajectory> active_traj_for_yaw_ = nullptr;

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  double yaw_temp;
  
  // 使用活动轨迹（正常情况用traj_，过渡期间用prev_traj_）
  auto active_traj = active_traj_for_yaw_ ? active_traj_for_yaw_ : traj_;
  
  // 计算速度方向的 yaw（始终计算，用于安全飞行）
  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? active_traj->getPos(t_cur + time_forward_) - pos
                            : active_traj->getPos(traj_duration_) - pos;
  double velocity_yaw = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  
  if (use_target_yaw_)
  {
    // 混合策略：
    // - 轨迹前 70%：跟随速度方向（安全飞行，朝前看）
    // - 轨迹后 30%：从速度方向平滑过渡到目标 yaw（到达目标点前调整朝向）
    double progress = std::min(1.0, t_cur / traj_duration_);
    
    const double transition_start = 0.7;  // 70% 时开始过渡
    
    if (progress < transition_start) {
      // 前 70%：跟随速度方向
      yaw_temp = velocity_yaw;
    } else {
      // 后 30%：从当前速度方向平滑过渡到目标 yaw
      double transition_progress = (progress - transition_start) / (1.0 - transition_start);
      double smooth_progress = transition_progress * transition_progress * (3.0 - 2.0 * transition_progress);
      
      // 从速度方向过渡到目标 yaw
      double yaw_diff = angleDiff(end_yaw_, velocity_yaw);
      yaw_temp = normalizeAngle(velocity_yaw + smooth_progress * yaw_diff);
    }
  }
  else
  {
    // Follow velocity direction (original behavior for preset waypoints)
    yaw_temp = velocity_yaw;
  }

  double d_yaw = angleDiff(yaw_temp, last_yaw_);

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  double yawdot = d_yaw / dt;

  double yaw = normalizeAngle(last_yaw_ + d_yaw);
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, double y, double yd)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void visualizeTrajectory()
{
  if (!receive_traj_)
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "ego_trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  double step = 0.05;
  for (double t = 0; t < traj_duration_; t += step)
  {
    Eigen::Vector3d pos = traj_->getPos(t);
    geometry_msgs::Point pt;
    pt.x = pos.x();
    pt.y = pos.y();
    pt.z = pos.z();
    marker.points.push_back(pt);
  }

  traj_vis_pub.publish(marker);
}

void cmdCallback(const ros::TimerEvent &e)
{
  if (task_finished_)
    return;
    
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();

  if (t_cur < 0.0)
  {
    // ===== FALCON风格：新轨迹尚未开始，继续执行旧轨迹 =====
    if (has_prev_traj_ && prev_traj_ != nullptr)
    {
      double t_prev = (time_now - prev_start_time_).toSec();
      if (t_prev >= 0 && t_prev < prev_traj_duration_)
      {
        // 继续执行旧轨迹的位置/速度/加速度
        pos = prev_traj_->getPos(t_prev);
        vel = prev_traj_->getVel(t_prev);
        acc = prev_traj_->getAcc(t_prev);
        
        // 临时切换到旧轨迹的yaw参数，使用统一的calculate_yaw处理
        double saved_end_yaw = end_yaw_;
        bool saved_use_target = use_target_yaw_;
        double saved_duration = traj_duration_;
        
        end_yaw_ = prev_end_yaw_;
        use_target_yaw_ = !std::isnan(prev_end_yaw_);
        traj_duration_ = prev_traj_duration_;
        active_traj_for_yaw_ = prev_traj_;  // 使用旧轨迹计算yaw
        
        // 使用统一的yaw计算逻辑（处理角度环绕和速度限制）
        yaw_yawdot = calculate_yaw(t_prev, pos, (time_now - time_last).toSec());
        
        // 恢复新轨迹的yaw参数
        end_yaw_ = saved_end_yaw;
        use_target_yaw_ = saved_use_target;
        traj_duration_ = saved_duration;
        active_traj_for_yaw_ = nullptr;  // 恢复默认
        
        time_last = time_now;
        last_yaw_ = yaw_yawdot.first;
        last_pos_ = pos;
        
        publish_cmd(pos, vel, acc, yaw_yawdot.first, yaw_yawdot.second);
        return;
      }
      else if (t_prev >= prev_traj_duration_)
      {
        // 旧轨迹已结束，等待新轨迹开始
        // 关键修复：发送新轨迹起点位置，而不是旧轨迹末端位置
        // 这样当 t_cur=0 新轨迹开始时，位置是连续的
        pos = traj_->getPos(0);
        vel = traj_->getVel(0);
        acc = traj_->getAcc(0);
        // yaw保持last_yaw_，由calculate_yaw的速度限制保证平滑
        publish_cmd(pos, vel, acc, last_yaw_, 0);
        return;
      }
    }
    else
    {
      // 没有旧轨迹可执行，发送新轨迹起点位置等待
      pos = traj_->getPos(0);
      vel = traj_->getVel(0);
      acc = traj_->getAcc(0);
      publish_cmd(pos, vel, acc, last_yaw_, 0);
      return;
    }
  }
  else if (t_cur < traj_duration_)
  {
    // ===== 新轨迹已开始，正常执行 =====
    // 清除旧轨迹引用（不再需要）
    if (has_prev_traj_)
    {
      has_prev_traj_ = false;
      prev_traj_.reset();
    }
    
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    publish_cmd(pos, vel, acc, yaw_yawdot.first, yaw_yawdot.second);
  }
  else
  {
    // Trajectory ended, hold final position
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    
    // Use target yaw if specified and valid, otherwise keep last yaw (from velocity direction)
    double final_yaw = (use_target_yaw_ && !std::isnan(end_yaw_)) ? end_yaw_ : last_yaw_;
    publish_cmd(pos, vel, acc, final_yaw, 0);
  }

  // Visualize trajectory periodically
  static int vis_counter = 0;
  if (++vis_counter >= 50)
  {
    vis_counter = 0;
    visualizeTrajectory();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_traj_server");
  ros::NodeHandle node("~");

  node.param("traj_server/time_forward", time_forward_, 0.5);

  ros::Subscriber poly_traj_sub = node.subscribe("/planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = node.subscribe("/planning/heartbeat", 10, heartbeatCallback);
  ros::Subscriber replan_sub = node.subscribe("/planning/replan", 10, replanCallback);
  ros::Subscriber odom_sub = node.subscribe("odom", 10, odomCallback);

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
  traj_vis_pub = node.advertise<visualization_msgs::Marker>("/ego_planner/trajectory_vis", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  ros::Duration(1.0).sleep();

  ROS_INFO("[EGOTrajServer] Trajectory server started");
  ROS_INFO("[EGOTrajServer] time_forward: %.2f", time_forward_);

  ros::spin();

  return 0;
}

