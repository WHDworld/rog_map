/**
 * @file exploration_planner_node.cpp
 * @brief 统一的探索规划节点
 * 
 * 整合架构：
 * - 唯一的 ROG-Map 实例（地图管理）
 * - EGO-Planner FSM（轨迹规划）
 * - Exploration FSM（探索策略和视点生成）
 * 
 * 所有模块共享同一个地图，保证数据同步，减少内存占用
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "rog_map/rog_map.h"
#include "rog_map/global_map.h"
#include "rog_map/exploration_fsm.h"
#include "ego_planner/ego_replan_fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_planner_node");
    ros::NodeHandle nh("~");
    
    ROS_INFO("===========================================");
    ROS_INFO("    Unified Exploration Planner Node       ");
    ROS_INFO("===========================================");
    ROS_INFO("  Architecture:");
    ROS_INFO("  - Single ROG-Map instance (shared)");
    ROS_INFO("  - EGO-Planner for trajectory planning");
    ROS_INFO("  - Exploration FSM for viewpoint generation");
    ROS_INFO("===========================================");
    
    // ========== Step 1: 创建唯一的 ROG-Map 实例 ==========
    ROS_INFO("[ExplorationPlannerNode] Creating ROG-Map...");
    rog_map::ROGMap::Ptr rog_map;
    rog_map.reset(new rog_map::ROGMap(nh));
    
    // 获取 GlobalMap 指针（用于探索模块）
    rog_map::GlobalMap::Ptr global_map = rog_map->getGlobalMap();
    
    if (!global_map) {
        ROS_ERROR("[ExplorationPlannerNode] Failed to get GlobalMap from ROGMap!");
        return -1;
    }
    
    ROS_INFO("[ExplorationPlannerNode] ROG-Map initialized (shared instance)");
    
    // ========== Step 2: 初始化 EGO-Planner FSM ==========
    ROS_INFO("[ExplorationPlannerNode] Initializing EGO-Planner...");
    ego_planner::EGOReplanFSM ego_fsm;
    ego_fsm.init(nh, rog_map);  // 共享 ROG-Map
    
    ROS_INFO("[ExplorationPlannerNode] EGO-Planner initialized");
    
    // ========== Step 3: 初始化探索状态机 ==========
    ROS_INFO("[ExplorationPlannerNode] Initializing Exploration FSM...");
    rog_map::ExplorationFSM::Ptr exploration_fsm;
    exploration_fsm.reset(new rog_map::ExplorationFSM);
    exploration_fsm->init(nh, rog_map, global_map);  // 共享同一个 ROG-Map
    
    ROS_INFO("[ExplorationPlannerNode] Exploration FSM initialized");
    
    ROS_INFO("===========================================");
    ROS_INFO("  All modules share the same ROG-Map!");
    ROS_INFO("  -----------------------------------");
    ROS_INFO("  Usage:");
    ROS_INFO("  1. UAV will auto-takeoff and follow waypoints");
    ROS_INFO("  2. Exploration visualizes frontiers & viewpoints");
    ROS_INFO("  3. Use '2D Nav Goal' in RViz to trigger manual target");
    ROS_INFO("  -----------------------------------");
    ROS_INFO("  RViz Topics:");
    ROS_INFO("  - Planner: /ego_planner/trajectory_vis");
    ROS_INFO("  - Frontiers: /exploration/frontier_clusters");
    ROS_INFO("  - Viewpoints: /exploration/viewpoints");
    ROS_INFO("===========================================");
    
    // 启动多线程 spinner
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::waitForShutdown();
    
    ROS_INFO("[ExplorationPlannerNode] Shutting down...");
    
    return 0;
}

