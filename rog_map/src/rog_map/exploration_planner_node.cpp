/**
 * @file exploration_planner_node.cpp
 * @brief 统一的探索规划节点
 * 
 * 整合架构：
 * - 唯一的 STAR-MVF MapManager 实例（地图管理）
 * - EGO-Planner FSM（轨迹规划）
 * - Exploration FSM（探索策略和视点生成）
 * 
 * 所有模块共享同一个地图，保证数据同步，减少内存占用
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"
#include "rog_map/exploration_fsm.h"
#include "ego_planner/ego_replan_fsm.h"

// 性能监控日志系统
#include "utils/perf_logger.hpp"
#include "utils/ros_log_redirect.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // ========== 初始化性能日志系统 ==========
    std::string log_path;
    nh_private.param<std::string>("log_path", log_path, 
        "/home/whd/experiment/STAR-Searcher/src/ROG-Map/rog_map/log");
    PERF_LOG.init(log_path, true);
    
    // ========== 启用 ROS 日志重定向（所有 ROS_INFO/WARN/ERROR 输出到日志文件）==========
    perf::initRosLogRedirect();
    
    PERF_INFO("NODE", "===========================================");
    PERF_INFO("NODE", "    Unified Exploration Planner Node       ");
    PERF_INFO("NODE", "    (STAR-MVF Map System)                  ");
    PERF_INFO("NODE", "===========================================");
    
    ROS_INFO("===========================================");
    ROS_INFO("    Unified Exploration Planner Node       ");
    ROS_INFO("    (STAR-MVF Map System)                  ");
    ROS_INFO("===========================================");
    ROS_INFO("  Architecture:");
    ROS_INFO("  - Single STAR-MVF MapManager instance (shared)");
    ROS_INFO("  - EGO-Planner for trajectory planning");
    ROS_INFO("  - Exploration FSM for viewpoint generation");
    ROS_INFO("===========================================");
    
    // ========== Step 1: 创建唯一的 STAR-MVF MapManager 实例 ==========
    ROS_INFO("[ExplorationPlannerNode] Creating STAR-MVF MapManager...");
    star_mvf::MapManager::Ptr map_manager;
    map_manager.reset(new star_mvf::MapManager(nh, nh_private));
    map_manager->initFromROS("rog_map");
    map_manager->start();
    
    // 获取 GlobalMap 指针（用于探索模块）
    star_mvf::GlobalMap::Ptr global_map = map_manager->getGlobalMap();
    
    if (!global_map) {
        ROS_ERROR("[ExplorationPlannerNode] Failed to get GlobalMap from MapManager!");
        return -1;
    }
    
    ROS_INFO("[ExplorationPlannerNode] MapManager initialized (shared instance)");
    
    // ========== Step 2: 初始化 EGO-Planner FSM ==========
    ROS_INFO("[ExplorationPlannerNode] Initializing EGO-Planner...");
    ego_planner::EGOReplanFSM ego_fsm;
    ego_fsm.init(nh_private, map_manager);  // 共享 MapManager
    
    ROS_INFO("[ExplorationPlannerNode] EGO-Planner initialized");
    
    // ========== Step 3: 初始化探索状态机 ==========
    ROS_INFO("[ExplorationPlannerNode] Initializing Exploration FSM...");
    rog_map::ExplorationFSM::Ptr exploration_fsm;
    exploration_fsm.reset(new rog_map::ExplorationFSM);
    exploration_fsm->init(nh_private, map_manager);  // 共享同一个 MapManager
    
    ROS_INFO("[ExplorationPlannerNode] Exploration FSM initialized");
    
    ROS_INFO("===========================================");
    ROS_INFO("  All modules share the same MapManager!");
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
    PERF_INFO("NODE", "Shutting down exploration planner node...");
    
    map_manager->stop();
    
    // 关闭日志系统
    PERF_LOG.close();
    
    return 0;
}

