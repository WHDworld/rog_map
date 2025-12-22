/**
 * @file gcopter_node.cpp
 * @brief GCOPTER 规划节点 - 集成 FSM 和 TrajServer
 * 
 * 功能:
 * 1. 初始化 ROGMap
 * 2. 初始化 FSM 和 TrajServer
 * 3. 连接 FSM 和 TrajServer（轨迹传递）
 * 
 * 话题:
 * - 订阅: odom, /move_base_simple/goal, pointcloud
 * - 发布: /planning/pos_cmd, visualization markers
 */

#include <ros/ros.h>
#include "rog_map/rog_map.h"
#include "gcopter/gcopter_fsm.h"
#include "gcopter/gcopter_traj_server.h"

using namespace rog_map;

class GCOPTERNode {
public:
    GCOPTERNode(ros::NodeHandle& nh) : nh_(nh) {
        ROS_INFO("[GCOPTERNode] Initializing...");
        
        // 初始化地图
        rog_map_ = std::make_unique<ROGMap>(nh);
        
        // 初始化轨迹服务器（先于 FSM，因为 FSM 需要引用它）
        traj_server_ = std::make_unique<GCOPTERTrajServer>();
        traj_server_->init(nh);
        
        // 初始化 FSM，传入 TrajServer 指针
        fsm_ = std::make_unique<GCOPTERReplanFSM>();
        fsm_->init(nh, rog_map_.get(), traj_server_.get());
        
        // 创建定时器连接 FSM 和 TrajServer（作为备用同步机制）
        sync_timer_ = nh.createTimer(ros::Duration(0.02), 
                                      &GCOPTERNode::syncCallback, this);
        
        ROS_INFO("[GCOPTERNode] Initialized successfully!");
    }

    ~GCOPTERNode() = default;

private:
    /**
     * @brief 同步回调 - 将 FSM 的轨迹传递给 TrajServer
     */
    void syncCallback(const ros::TimerEvent& e) {
        if (!fsm_ || !traj_server_) return;
        
        // 检查 FSM 是否有新轨迹
        const auto& traj_container = fsm_->getTraj();
        const auto& local_traj = traj_container.local_traj;
        
        static int last_traj_id = -1;
        
        // 新轨迹到达
        if (local_traj.traj_id != last_traj_id && 
            local_traj.traj.getPieceNum() > 0) {
            
            ROS_INFO("[GCOPTERNode] Passing trajectory %d to TrajServer", 
                     local_traj.traj_id);
            
            traj_server_->setTrajectory(local_traj.traj, local_traj.traj_id);
            traj_server_->setGoalPosition(fsm_->getGoalPosition());
            
            last_traj_id = local_traj.traj_id;
        }
        
        // 检查 FSM 状态并同步到 TrajServer
        auto fsm_state = fsm_->getCurrentState();
        if (fsm_state == GCOPTERReplanFSM::EMERGENCY_STOP) {
            traj_server_->emergencyStop();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Timer sync_timer_;
    
    std::unique_ptr<ROGMap> rog_map_;
    std::unique_ptr<GCOPTERReplanFSM> fsm_;
    std::unique_ptr<GCOPTERTrajServer> traj_server_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gcopter_node");
    ros::NodeHandle nh("~");
    
    GCOPTERNode node(nh);
    
    ros::spin();
    
    return 0;
}

