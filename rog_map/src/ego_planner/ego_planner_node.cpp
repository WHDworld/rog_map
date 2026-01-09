#include <ros/ros.h>
#include "ego_planner/ego_replan_fsm.h"
// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // 初始化 STAR-MVF MapManager（替代旧的 ROGMap）
  ROS_INFO("[EGOPlannerNode] Creating STAR-MVF MapManager...");
  star_mvf::MapManager::Ptr map_manager;
  map_manager.reset(new star_mvf::MapManager(nh, nh_private));
  map_manager->initFromROS("rog_map");
  map_manager->start();

  // 初始化 EGO Replan FSM
  ego_planner::EGOReplanFSM ego_fsm;
  ego_fsm.init(nh_private, map_manager);

  ROS_INFO("[EGOPlannerNode] EGO Planner initialized with STAR-MVF Map System");

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  
  map_manager->stop();

  return 0;
}

