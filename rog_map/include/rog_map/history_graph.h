#ifndef HISTORY_GRAPH_H
#define HISTORY_GRAPH_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>

#include <visualization_msgs/MarkerArray.h>

#include "star_mvf/star_mvf.h"
#include "rog_map/dtg_history_graph.h"

namespace rog_map {

using GlobalMap = star_mvf::GlobalMap;
class ViewpointManager;

// Kept for FSM compatibility
struct ViewpointNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int viewpoint_id = -1;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    float yaw = 0.0f;
    int cluster_id = -1;
    int connected_history_node = -1; // Deprecated, but kept for API stability
    float score = 0.0f;
    float timestamp = 0.0f;
    bool explored = false;
    bool valid = true;
};

struct PathResult {
    bool success = false;
    std::vector<int> history_node_path; // Deprecated, but kept for API stability
    std::vector<Eigen::Vector3f> waypoints;
    float total_distance = 0.0f;
    int target_viewpoint_id = -1;
};

class HistoryGraph {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<HistoryGraph> Ptr;

    HistoryGraph();
    ~HistoryGraph() = default;

    void init(ros::NodeHandle& nh, GlobalMap::Ptr global_map);
    void setViewpointManager(ViewpointManager* vp_manager);
    void start();
    void stop();

    int updateUAVPosition(const Eigen::Vector3f& uav_pos);

    // --- API for FSM (forwarded to DTG) ---
    void markViewpointExplored(int viewpoint_id);
    void markViewpointInvalid(int viewpoint_id);
    std::vector<ViewpointNode> getValidViewpoints() const;
    PathResult findPathToViewpoint(const Eigen::Vector3f& start_pos, int viewpoint_id);
    void publishVisualization();

    // --- Legacy compatibility (used by existing FSM/ViewpointManager code) ---
    int findNearestHistoryNode(const Eigen::Vector3f& pos) const;
    bool hasObstacleBetween(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) const;
    bool getViewpointInfo(int viewpoint_id, Eigen::Vector3f& pos, float& yaw) const;
    const ViewpointNode* getViewpoint(int viewpoint_id) const;
    size_t getHistoryNodeCount() const;

private:
    void timerCallback(const ros::TimerEvent& e);
    void visualizationTimerCallback(const ros::TimerEvent& e);

    // The new DTG implementation
    std::unique_ptr<DtgHistoryGraph> dtg_impl_;

    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Timer vis_timer_;
    ros::Publisher history_graph_pub_;

    GlobalMap::Ptr global_map_;
    ViewpointManager* viewpoint_manager_ = nullptr;

    Eigen::Vector3f current_uav_pos_ = Eigen::Vector3f::Zero();
    bool has_uav_pos_ = false;
    bool running_ = false;
    mutable std::mutex mutex_;
};

} // namespace rog_map

#endif // HISTORY_GRAPH_H
