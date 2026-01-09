/**
 * @file exploration_node.cpp
 * @brief 探索可视化节点（被动模式）
 * 
 * 功能：
 * 1. 初始化 STAR-MVF MapManager（接收点云和里程计，构建地图）
 * 2. 运行探索状态机（被动可视化模式）
 * 3. 实时可视化探索视点和视锥体
 * 
 * 使用方法：
 * 1. 与 ego_planner.launch 或其他规划器一起启动
 * 2. 无人机飞行时，自动显示前沿区域和探索视点
 * 3. 使用RViz中的"2D Nav Goal"触发主动探索模式
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"
#include "rog_map/exploration_fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO("===========================================");
    ROS_INFO("    Exploration Visualization Node         ");
    ROS_INFO("    (STAR-MVF Map System)                  ");
    ROS_INFO("===========================================");
    
    // 创建 STAR-MVF MapManager（会自动订阅点云和里程计，构建地图）
    ROS_INFO("[ExplorationNode] Creating STAR-MVF MapManager...");
    star_mvf::MapManager::Ptr map_manager;
    map_manager.reset(new star_mvf::MapManager(nh, nh_private));
    map_manager->initFromROS("rog_map");
    map_manager->start();
    
    // 获取 GlobalMap 指针（用于探索状态机）
    star_mvf::GlobalMap::Ptr global_map = map_manager->getGlobalMap();
    
    if (!global_map) {
        ROS_ERROR("[ExplorationNode] Failed to get GlobalMap from MapManager!");
        return -1;
    }
    
    ROS_INFO("[ExplorationNode] MapManager initialized");
    
    // 创建探索状态机
    ROS_INFO("[ExplorationNode] Creating ExplorationFSM...");
    rog_map::ExplorationFSM::Ptr exploration_fsm;
    exploration_fsm.reset(new rog_map::ExplorationFSM);
    exploration_fsm->init(nh_private, map_manager);
    
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
    map_manager->stop();
    
    return 0;
}
