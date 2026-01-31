/**
 * @file exploration_planner_node_new.cpp
 * @brief 新架构探索规划节点入口点
 * 
 * 启动 ExplorationFSMNew 状态机，集成：
 * - MapManager (局部/全局地图同步)
 * - ViewpointManager (视点生成与管理)
 * - HistoryGraph (历史节点网络)
 * - EGOPlannerManager (轨迹规划)
 * - TargetReconstructor (目标重建)
 * - SemanticTargetExtractor (语义目标提取)
 */

#include <ros/ros.h>
#include "map/map_manager.h"
#include "exploration/exploration_fsm_new.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO("===========================================");
    ROS_INFO("    STAR-Searcher Exploration Planner");
    ROS_INFO("===========================================");
    ROS_INFO("  New Architecture: 4-State FSM");
    ROS_INFO("  Features: ViewpointManager, HistoryGraph,");
    ROS_INFO("            EGO Planner, TargetReconstructor");
    ROS_INFO("===========================================");
    
    // 1. 创建并初始化地图管理器
    ROS_INFO("[Main] Creating MapManager...");
    star_mvf::MapManager::Ptr map_manager = std::make_shared<star_mvf::MapManager>(nh, nh_private);
    map_manager->initFromROS("rog_map");
    map_manager->start();
    
    // 2. 创建并初始化探索状态机
    ROS_INFO("[Main] Creating ExplorationFSMNew...");
    rog_map::ExplorationFSMNew fsm;
    fsm.init(nh_private, map_manager);
    
    ROS_INFO("[Main] Exploration Planner started, spinning...");
    
    // 3. 使用 AsyncSpinner 支持多线程回调
    ros::AsyncSpinner spinner(4);  // 4 线程
    spinner.start();
    ros::waitForShutdown();
    
    // 4. 清理
    map_manager->stop();
    ROS_INFO("[Main] Shutdown complete");
    
    return 0;
}
