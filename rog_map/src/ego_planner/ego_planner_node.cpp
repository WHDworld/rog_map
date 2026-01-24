/**
 * @file ego_planner_node.cpp
 * @brief 规划器测试节点
 * 
 * 使用方式：
 * - 在 config/planner_test.yaml 中配置目标点（x,y,z,yaw）
 * - 启动 roslaunch rog_map planner_test.launch
 */

#include <ros/ros.h>
#include "star_mvf/star_mvf.h"
#include "ego_planner/ego_replan_fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ego_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO("==========================================");
    ROS_INFO("    EGO Planner Test Node                 ");
    ROS_INFO("==========================================");
    
    // 创建共享的 MapManager
    ROS_INFO("[EgoPlannerNode] Creating MapManager...");
    star_mvf::MapManager::Ptr map_manager;
    map_manager.reset(new star_mvf::MapManager(nh, nh_private));
    map_manager->initFromROS("rog_map");
    map_manager->start();
    
    // 初始化 EGO-Planner FSM
    ROS_INFO("[EgoPlannerNode] Initializing EGOReplanFSM...");
    ego_planner::EGOReplanFSM ego_fsm;
    ego_fsm.init(nh_private, map_manager);
    
    ROS_INFO("[EgoPlannerNode] Ready. Waiting for trigger...");
    ROS_INFO("==========================================");
    
    // 多线程 spinner
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::waitForShutdown();
    
    ROS_INFO("[EgoPlannerNode] Shutting down...");
    map_manager->stop();
    
    return 0;
}
