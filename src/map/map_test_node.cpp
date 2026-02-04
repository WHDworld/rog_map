/**
 * @file map_test_node.cpp
 * @brief 地图系统测试节点 - 单独测试 LocalMap 和 GlobalMap 的同步
 */

#include <ros/ros.h>
#include "map/map_manager.h"

star_mvf::MapManager::Ptr g_map_manager;

void statusTimerCallback(const ros::TimerEvent& event) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO("===========================================");
    ROS_INFO("    STAR-MVF Map Test Node");
    ROS_INFO("===========================================");
    ROS_INFO("  This node tests LocalMap <-> GlobalMap sync");
    ROS_INFO("  Subscribe: odom, lidar point cloud");
    ROS_INFO("  Publish: global_map/occupied, frontier, etc.");
    ROS_INFO("===========================================");
    
    // 创建 MapManager
    ROS_INFO("[MapTestNode] Creating MapManager...");
    g_map_manager = std::make_shared<star_mvf::MapManager>(nh, nh_private);
    
    ROS_INFO("[MapTestNode] Calling initFromROS...");
    g_map_manager->initFromROS("rog_map");
    
    ROS_INFO("[MapTestNode] Calling start...");
    g_map_manager->start();
    
    // 状态监控定时器
    ros::Timer status_timer = nh.createTimer(ros::Duration(1.0), statusTimerCallback);
    
    ROS_INFO("[MapTestNode] MapManager started, spinning...");
    
    // 使用标准 spin
    ros::spin();
    
    g_map_manager->stop();
    ROS_INFO("[MapTestNode] Shutdown");
    
    return 0;
}
