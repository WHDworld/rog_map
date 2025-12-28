/**
 * @file exploration_node.cpp
 * @brief 探索可视化节点（被动模式）
 * 
 * 功能：
 * 1. 初始化ROG-Map（接收点云和里程计，构建地图）
 * 2. 运行探索状态机（被动可视化模式）
 * 3. 实时可视化探索视点和视锥体
 * 
 * 使用方法：
 * 1. 与 ego_planner.launch 或其他规划器一起启动
 * 2. 无人机飞行时，自动显示前沿区域和探索视点
 * 3. 使用RViz中的"2D Nav Goal"触发主动探索模式
 * 
 * 注意：
 * 本节点会创建独立的 ROG-Map 实例，与规划器的地图独立。
 * 两者都会接收相同的点云数据，构建各自的地图。
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "rog_map/rog_map.h"
#include "rog_map/global_map.h"
#include "rog_map/exploration_fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_node");
    ros::NodeHandle nh("~");
    
    ROS_INFO("===========================================");
    ROS_INFO("    Exploration Visualization Node         ");
    ROS_INFO("===========================================");
    
    // 创建 ROG-Map（会自动订阅点云和里程计，构建地图）
    ROS_INFO("[ExplorationNode] Creating ROG-Map...");
    rog_map::ROGMap::Ptr rog_map;
    rog_map.reset(new rog_map::ROGMap(nh));
    
    // 从 ROG-Map 获取 GlobalMap 指针
    // GlobalMap 存储全局体素状态和前沿信息
    rog_map::GlobalMap::Ptr global_map = rog_map->getGlobalMap();
    
    if (!global_map) {
        ROS_ERROR("[ExplorationNode] Failed to get GlobalMap from ROGMap!");
        return -1;
    }
    
    ROS_INFO("[ExplorationNode] ROG-Map and GlobalMap initialized");
    
    // 创建探索状态机
    ROS_INFO("[ExplorationNode] Creating ExplorationFSM...");
    rog_map::ExplorationFSM::Ptr exploration_fsm;
    exploration_fsm.reset(new rog_map::ExplorationFSM);
    exploration_fsm->init(nh, rog_map, global_map);
    
    ROS_INFO("[ExplorationNode] ExplorationFSM initialized");
    ROS_INFO("===========================================");
    ROS_INFO("  Passive Visualization Mode Active");
    ROS_INFO("  -----------------------------------");
    ROS_INFO("  The node will:");
    ROS_INFO("  1. Build map from point cloud");
    ROS_INFO("  2. Extract frontiers in real-time");
    ROS_INFO("  3. Generate exploration viewpoints");
    ROS_INFO("  4. Visualize in RViz");
    ROS_INFO("  -----------------------------------");
    ROS_INFO("  RViz Topics:");
    ROS_INFO("  - /exploration/frontier_clusters");
    ROS_INFO("  - /exploration/viewpoints");
    ROS_INFO("  - /exploration/frustums");
    ROS_INFO("  - /exploration/region_route");
    ROS_INFO("===========================================");
    
    // 启动多线程 spinner（处理回调）
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::waitForShutdown();
    
    ROS_INFO("[ExplorationNode] Shutting down...");
    
    return 0;
}
