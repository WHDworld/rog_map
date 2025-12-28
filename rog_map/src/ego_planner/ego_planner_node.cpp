#include <ros/ros.h>
#include "ego_planner/ego_replan_fsm.h"
#include <rog_map/rog_map.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  // Initialize ROG-Map (using constructor with NodeHandle)
  rog_map::ROGMap::Ptr rog_map;
  rog_map.reset(new rog_map::ROGMap(nh));

  // Initialize EGO Replan FSM
  ego_planner::EGOReplanFSM ego_fsm;
  ego_fsm.init(nh, rog_map);

  ROS_INFO("[EGOPlannerNode] EGO Planner initialized successfully");

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}

