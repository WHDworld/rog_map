/**
 * @file exploration_planner_node_new.cpp
 * @brief 新架构探索规划节点入口
 * 
 * 新架构特点：
 * 1. 地图更新在 lidarCallback 中完成（~10Hz）
 * 2. 视点生成由 ViewpointManager 定时器处理（5Hz）
 * 3. 历史节点由 HistoryGraph 定时器处理（2Hz）
 * 4. 探索状态机为4状态设计（100Hz）
 * 5. 规划器直接在状态机中调用，不再有单独的规划状态机
 */

#include <ros/ros.h>
#include <ros/package.h>

// 地图系统
#include "star_mvf/star_mvf.h"

// 新架构模块
#include "rog_map/exploration_fsm_new.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO("[SYSTEM] New architecture exploration node starting...");
    
    // 创建地图管理器
    auto map_manager = std::make_shared<star_mvf::MapManager>(nh, nh_private);
    map_manager->initFromROS("rog_map");
    map_manager->start();
    
    ROS_INFO("[SYSTEM] MapManager initialized and started");
    
    // 创建探索状态机（新架构，包含HistoryGraph, ViewpointManager, Planner）
    auto exploration_fsm = std::make_shared<rog_map::ExplorationFSMNew>();
    exploration_fsm->init(nh_private, map_manager);
    
    ROS_INFO("[SYSTEM] ExplorationFSMNew initialized");
    
    // 使用异步Spinner处理回调
    ros::AsyncSpinner spinner(4);  // 4个线程
    spinner.start();
    
    ROS_INFO("\n"
             "========================================\n"
             "  STAR-Searcher New Architecture\n"
             "========================================\n"
             "  - MapManager: sync in lidarCallback\n"
             "  - ViewpointManager: 5Hz\n"
             "  - HistoryGraph: 2Hz\n"
             "  - ExplorationFSM: 4-state, 100Hz\n"
             "  - Planner: integrated in FSM\n"
             "========================================\n");
    
    ros::waitForShutdown();
    
    ROS_INFO("[SYSTEM] Node shutting down");
    
    return 0;
}
