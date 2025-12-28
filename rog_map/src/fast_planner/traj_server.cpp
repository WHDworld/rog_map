#include "fast_planner/traj_server.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fast_planner {

void TrajServer::init(ros::NodeHandle &nh) {
  nh_ = nh;
  
  // Load parameters
  nh.param("traj_server/cmd_rate", cmd_rate_, 100.0);
  
  // Initial position
  nh.param("map_config/init_x", init_pos_[0], 0.0);
  nh.param("map_config/init_y", init_pos_[1], 0.0);
  nh.param("map_config/init_z", init_pos_[2], 0.0);
  nh.param("map_config/init_yaw", init_yaw_, 0.0);
  
  // Controller gains
  nh.param("traj_server/kx_x", cmd_.kx[0], -1.0);
  nh.param("traj_server/kx_y", cmd_.kx[1], -1.0);
  nh.param("traj_server/kx_z", cmd_.kx[2], -1.0);
  nh.param("traj_server/kv_x", cmd_.kv[0], -1.0);
  nh.param("traj_server/kv_y", cmd_.kv[1], -1.0);
  nh.param("traj_server/kv_z", cmd_.kv[2], -1.0);
  
  // Publishers
  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
  cmd_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/position_cmd_vis", 10);
  traj_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/travel_traj", 10);
  traj_vel_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planning/travel_vel_traj", 10);
  
  // Subscribers
  bspline_sub_ = nh.subscribe("/planning/bspline", 1, &TrajServer::bsplineCallback, this);
  replan_sub_ = nh.subscribe("/planning/replan", 1, &TrajServer::replanCallback, this);
  
  // Timers
  cmd_timer_ = nh.createTimer(ros::Duration(1.0 / cmd_rate_), &TrajServer::cmdCallback, this);
  vis_timer_ = nh.createTimer(ros::Duration(0.25), &TrajServer::visCallback, this);
  
  // Initialize command
  cmd_.header.frame_id = "world";
  cmd_.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_.trajectory_id = 0;
  cmd_.position.x = init_pos_[0];
  cmd_.position.y = init_pos_[1];
  cmd_.position.z = init_pos_[2];
  cmd_.velocity.x = 0.0;
  cmd_.velocity.y = 0.0;
  cmd_.velocity.z = 0.0;
  cmd_.acceleration.x = 0.0;
  cmd_.acceleration.y = 0.0;
  cmd_.acceleration.z = 0.0;
  cmd_.yaw = init_yaw_;
  cmd_.yaw_dot = 0.0;
  
  ROS_INFO("[TrajServer] Initialized. Rate: %.1f Hz", cmd_rate_);
}

void TrajServer::setTrajectory(const std::vector<NonUniformBspline> &traj,
                               const ros::Time &start_time,
                               int traj_id) {
  traj_ = traj;
  start_time_ = start_time;
  traj_id_ = traj_id;
  traj_duration_ = traj_[0].getTimeSum();
  receive_traj_ = true;
  
  ROS_INFO("[TrajServer] Set trajectory, id: %d, duration: %.2f s", traj_id_, traj_duration_);
}

double TrajServer::getCurrentTime() const {
  if (!receive_traj_) return 0.0;
  return (ros::Time::now() - start_time_).toSec();
}

void TrajServer::bsplineCallback(const rog_map::BsplineConstPtr &msg) {
  // Check trajectory id
  if (msg->traj_id <= traj_id_) {
    ROS_ERROR("[TrajServer] Trajectory ID misordered, incoming: %ld, current: %d",
              msg->traj_id, traj_id_);
    return;
  }
  
  ROS_INFO("[TrajServer] Received Bspline trajectory, id: %ld", msg->traj_id);
  
  // Save previous trajectory for smooth transition
  // Only save if we have a valid trajectory that hasn't ended
  if (receive_traj_ && !traj_.empty()) {
    prev_traj_ = traj_;
    prev_start_time_ = start_time_;
    prev_traj_duration_ = traj_duration_;
    have_prev_traj_ = true;
  }
  
  // Parse position trajectory
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }
  
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);
  
  // Parse yaw trajectory
  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (size_t i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  
  // Store new trajectories
  traj_.clear();
  traj_.push_back(pos_traj);                    // 0: position
  traj_.push_back(traj_[0].getDerivative());    // 1: velocity
  traj_.push_back(traj_[1].getDerivative());    // 2: acceleration
  traj_.push_back(yaw_traj);                    // 3: yaw
  traj_.push_back(yaw_traj.getDerivative());    // 4: yaw_dot
  traj_.push_back(traj_[2].getDerivative());    // 5: jerk
  
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;
  traj_duration_ = traj_[0].getTimeSum();
  
  // Reset task finished flag when new trajectory is received
  task_finished_ = false;
  receive_traj_ = true;
}

void TrajServer::replanCallback(const std_msgs::Int32ConstPtr &msg) {
  const double time_out = 0.5;
  ros::Time now = ros::Time::now();
  double t_stop;
  
  if (msg->data == 0) {
    // Normal replan
    t_stop = (now - start_time_).toSec() + time_out;
  } else if (msg->data == 1) {
    // Emergency replan
    ROS_WARN("[TrajServer] Emergency replan: stop immediately");
    t_stop = (now - start_time_).toSec();
  } else if (msg->data == 2) {
    // Task finished
    ROS_INFO("[TrajServer] Task finished");
    t_stop = (now - start_time_).toSec();
    task_finished_ = true;
  } else {
    ROS_ERROR("[TrajServer] Invalid replan type: %d", msg->data);
    return;
  }
  
  traj_duration_ = std::min(t_stop, traj_duration_);
}

void TrajServer::cmdCallback(const ros::TimerEvent &e) {
  if (!receive_traj_) return;
  
  // If task is finished, stop publishing commands (let FSM handle hover)
  if (task_finished_) return;
  
  ros::Time now = ros::Time::now();
  double t_cur = (now - start_time_).toSec();
  
  Eigen::Vector3d pos, vel, acc;
  double yaw, yaw_dot;
  bool traj_ended = false;
  
  if (t_cur < 0.0) {
    // New trajectory hasn't started yet
    // Continue using the previous trajectory if available for smooth transition
    if (have_prev_traj_ && !prev_traj_.empty()) {
      double t_prev = (now - prev_start_time_).toSec();
      if (t_prev >= 0.0 && t_prev < prev_traj_duration_) {
        // Previous trajectory still valid - use it
        pos = prev_traj_[0].evaluateDeBoorT(t_prev);
        vel = prev_traj_[1].evaluateDeBoorT(t_prev);
        acc = prev_traj_[2].evaluateDeBoorT(t_prev);
        yaw = prev_traj_[3].evaluateDeBoorT(t_prev)[0];
        yaw_dot = prev_traj_[4].evaluateDeBoorT(t_prev)[0];
      } else {
        // Previous trajectory ended - hold at previous end position with current velocity
        // This bridges the gap between old trajectory end and new trajectory start
        pos = prev_traj_[0].evaluateDeBoorT(prev_traj_duration_);
        vel = prev_traj_[1].evaluateDeBoorT(prev_traj_duration_);
        acc = prev_traj_[2].evaluateDeBoorT(prev_traj_duration_);
        yaw = prev_traj_[3].evaluateDeBoorT(prev_traj_duration_)[0];
        yaw_dot = prev_traj_[4].evaluateDeBoorT(prev_traj_duration_)[0];
      }
    } else {
      // No previous trajectory - use new trajectory start position
      pos = traj_[0].evaluateDeBoorT(0.0);
      vel = traj_[1].evaluateDeBoorT(0.0);  // Use planned start velocity, not zero!
      acc = traj_[2].evaluateDeBoorT(0.0);
      yaw = traj_[3].evaluateDeBoorT(0.0)[0];
      yaw_dot = traj_[4].evaluateDeBoorT(0.0)[0];
    }
  } else if (t_cur < traj_duration_) {
    // Current time within trajectory - new trajectory is active
    // Clear previous trajectory since we're now fully on the new one
    if (have_prev_traj_) {
      prev_traj_.clear();
      have_prev_traj_ = false;
    }
    
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yaw_dot = traj_[4].evaluateDeBoorT(t_cur)[0];
  } else {
    // Trajectory ended - hold final position with ZERO yaw_dot
    traj_ended = true;
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    // Use the trajectory's final yaw value (not rate-limited last_yaw_)
    // This ensures consistency with FSM's hover_yaw_ which uses local_data_.end_yaw_
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    // Update last_yaw_ to this final value for consistency
    last_yaw_ = yaw;
    yaw_dot = 0.0;
  }
  
  // Yaw smoothing and rate limiting
  const double max_yaw_rate = 1.57;  // rad/s (90 deg/s)
  double dt = 1.0 / cmd_rate_;
  double max_change = max_yaw_rate * dt;
  
  if (traj_id_ != last_traj_id_) {
    // New trajectory - initialize yaw tracking
    last_traj_id_ = traj_id_;
    last_yaw_ = yaw;
    yaw_initialized_ = true;
  } else if (yaw_initialized_ && !traj_ended) {
    // Compute yaw change from last command
    double yaw_diff = yaw - last_yaw_;
    
    // Handle wrap-around: find shortest path
    while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
    
    // Rate limiting
    if (std::abs(yaw_diff) > max_change) {
      yaw_diff = (yaw_diff > 0) ? max_change : -max_change;
    }
    
    yaw = last_yaw_ + yaw_diff;
    last_yaw_ = yaw;
  }
  // If traj_ended, use last_yaw_ which was already set above
  
  // Normalize yaw to [-PI, PI] for publishing
  double yaw_normalized = yaw;
  while (yaw_normalized < -M_PI) yaw_normalized += 2 * M_PI;
  while (yaw_normalized > M_PI) yaw_normalized -= 2 * M_PI;
  
  // Limit yaw_dot to prevent excessive angular rates
  const double max_yaw_dot = 1.57;  // rad/s
  yaw_dot = std::max(-max_yaw_dot, std::min(max_yaw_dot, yaw_dot));
  
  // Fill command
  cmd_.header.stamp = now;
  cmd_.trajectory_id = traj_id_;
  cmd_.position.x = pos.x();
  cmd_.position.y = pos.y();
  cmd_.position.z = pos.z();
  cmd_.velocity.x = vel.x();
  cmd_.velocity.y = vel.y();
  cmd_.velocity.z = vel.z();
  cmd_.acceleration.x = acc.x();
  cmd_.acceleration.y = acc.y();
  cmd_.acceleration.z = acc.z();
  cmd_.yaw = yaw_normalized;
  cmd_.yaw_dot = yaw_dot;
  
  pos_cmd_pub_.publish(cmd_);
  
  // Record executed trajectory for visualization
  if (traj_cmd_.empty()) {
    traj_cmd_.push_back(pos);
    vel_cmd_.push_back(vel);
  } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    traj_cmd_.push_back(pos);
    vel_cmd_.push_back(vel);
  }
  
  // Draw current command arrow
  drawFOV(pos, yaw);
}

void TrajServer::visCallback(const ros::TimerEvent &e) {
  displayTraj();
}

void TrajServer::displayTraj() {
  if (traj_cmd_.empty()) return;
  
  // Publish trajectory as sphere list
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  
  for (const auto &pt : traj_cmd_) {
    geometry_msgs::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    marker.points.push_back(p);
  }
  
  traj_pub_.publish(marker);
  
  // Publish velocity-colored trajectory
  if (traj_cmd_.size() != vel_cmd_.size()) return;
  
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.points.resize(traj_cmd_.size());
  
  for (size_t i = 0; i < traj_cmd_.size(); ++i) {
    cloud.points[i].x = traj_cmd_[i].x();
    cloud.points[i].y = traj_cmd_[i].y();
    cloud.points[i].z = traj_cmd_[i].z();
    
    double vel_norm = std::min(std::max(vel_cmd_[i].norm() / 2.0, 0.0), 1.0);
    
    // Color gradient from white to yellow to red
    if (vel_norm < 0.5) {
      double t = vel_norm * 2;
      cloud.points[i].r = 255;
      cloud.points[i].g = 255;
      cloud.points[i].b = (1 - t) * 255;
    } else {
      double t = (vel_norm - 0.5) * 2;
      cloud.points[i].r = 255;
      cloud.points[i].g = (1 - t) * 255;
      cloud.points[i].b = 0;
    }
  }
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = ros::Time::now();
  traj_vel_pub_.publish(cloud_msg);
}

void TrajServer::drawFOV(const Eigen::Vector3d &pos, double yaw) {
  // Draw a simple direction arrow
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.ns = "current_cmd";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.2;
  marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  
  geometry_msgs::Point start_pt, end_pt;
  start_pt.x = pos.x();
  start_pt.y = pos.y();
  start_pt.z = pos.z();
  
  double arrow_len = 1.0;
  end_pt.x = pos.x() + arrow_len * std::cos(yaw);
  end_pt.y = pos.y() + arrow_len * std::sin(yaw);
  end_pt.z = pos.z();
  
  marker.points.push_back(start_pt);
  marker.points.push_back(end_pt);
  
  cmd_vis_pub_.publish(marker);
}

} // namespace fast_planner

