#include <ros/ros.h>
#include <csignal>

#include "fast_planner/planning_fsm.h"
#include "fast_planner/traj_server.h"
#include "rog_map/rog_map.h"

using namespace fast_planner;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
  ROS_WARN("[FastPlannerNode] Interrupt signal received, shutting down...");
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");
  
  // Register signal handler
  std::signal(SIGINT, signalHandler);
  
  ROS_INFO("[FastPlannerNode] Starting Fast Planner Node...");
  
  // ==================== Create ROG-Map ====================
  ROS_INFO("[FastPlannerNode] Initializing ROG-Map...");
  rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(nh);
  
  // Wait a bit for map initialization
  ros::Duration(1.0).sleep();
  
  // ==================== Create Planning FSM ====================
  ROS_INFO("[FastPlannerNode] Initializing Planning FSM...");
  PlanningFSM::Ptr planning_fsm(new PlanningFSM);
  planning_fsm->init(nh, rog_map_ptr);
  
  // ==================== Create Trajectory Server ====================
  ROS_INFO("[FastPlannerNode] Initializing Trajectory Server...");
  TrajServer::Ptr traj_server(new TrajServer);
  traj_server->init(nh);
  
  ROS_INFO("[FastPlannerNode] All modules initialized successfully!");
  ROS_INFO("[FastPlannerNode] ============================================");
  ROS_INFO("[FastPlannerNode] Subscriptions:");
  ROS_INFO("[FastPlannerNode]   - Odometry: ~odom");
  ROS_INFO("[FastPlannerNode]   - Target: /move_base_simple/goal");
  ROS_INFO("[FastPlannerNode]   - Trigger: /traj_start_trigger");
  ROS_INFO("[FastPlannerNode]   - MAVROS State: /mavros/state");
  ROS_INFO("[FastPlannerNode]   - Point Cloud: ~cloud (via ROG-Map)");
  ROS_INFO("[FastPlannerNode] Publications:");
  ROS_INFO("[FastPlannerNode]   - Position Command: /planning/pos_cmd");
  ROS_INFO("[FastPlannerNode]   - Bspline Trajectory: /planning/bspline");
  ROS_INFO("[FastPlannerNode]   - Replan Signal: /planning/replan");
  ROS_INFO("[FastPlannerNode] ============================================");
  
  // Use multi-threaded spinner for better performance
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  ros::waitForShutdown();
  
  ROS_INFO("[FastPlannerNode] Shutdown complete.");
  return 0;
}

