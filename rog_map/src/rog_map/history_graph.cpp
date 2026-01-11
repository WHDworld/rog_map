#include "rog_map/history_graph.h"
#include "rog_map/viewpoint_manager.h"
#include "rog_map/dtg_history_graph.h"
#include <algorithm>

namespace {
// thread-local scratch to satisfy legacy API returning pointer
thread_local std::vector<rog_map::ViewpointNode> tl_cached_vps;
}


#define HIST_INFO(fmt, ...) ROS_INFO("[HistGraph] " fmt, ##__VA_ARGS__)

namespace rog_map {

HistoryGraph::HistoryGraph() : dtg_impl_(new DtgHistoryGraph()) {}

void HistoryGraph::init(ros::NodeHandle& nh, GlobalMap::Ptr global_map) {
    nh_ = nh;
    global_map_ = global_map;
    history_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/region_route", 1);
    HIST_INFO("Initialized (DTG mode)");
}

void HistoryGraph::setViewpointManager(ViewpointManager* vp_manager) {
    viewpoint_manager_ = vp_manager;
    dtg_impl_->init(nh_, "rog_map", global_map_, viewpoint_manager_);
}

void HistoryGraph::start() {
    if (running_) return;
    running_ = true;
    double update_rate = 2.0;
    nh_.param("rog_map/update_rate", update_rate, update_rate);
    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate), &HistoryGraph::timerCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(0.1), &HistoryGraph::visualizationTimerCallback, this);
    HIST_INFO("Started");
}

void HistoryGraph::stop() {
    if (!running_) return;
    running_ = false;
    timer_.stop();
    vis_timer_.stop();
    HIST_INFO("Stopped");
}

void HistoryGraph::timerCallback(const ros::TimerEvent& e) {
    if (!has_uav_pos_) return;
    ros::Time t_start = ros::Time::now();
    dtg_impl_->updateRobotPos(current_uav_pos_.cast<double>());
    double t_total = (ros::Time::now() - t_start).toSec() * 1000.0;
    ROS_INFO_THROTTLE(1.0, "[HistGraph][DTG] Update: H=%zu, F=%zu | total=%.1fms",
                      dtg_impl_->getHNodeCount(), dtg_impl_->getFNodeCount(), t_total);
}

int HistoryGraph::updateUAVPosition(const Eigen::Vector3f& uav_pos) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_uav_pos_ = uav_pos;
    has_uav_pos_ = true;
    return -1; // Return value is deprecated
}

// --- API forwarded to DTG ---

void HistoryGraph::markViewpointExplored(int viewpoint_id) {
    dtg_impl_->markViewpointExplored(viewpoint_id);
}

void HistoryGraph::markViewpointInvalid(int viewpoint_id) {
    dtg_impl_->markViewpointInvalid(viewpoint_id);
}

std::vector<ViewpointNode> HistoryGraph::getValidViewpoints() const {
    return dtg_impl_->getValidViewpoints();
}

PathResult HistoryGraph::findPathToViewpoint(const Eigen::Vector3f& start_pos, int viewpoint_id) {
    PathResult result;
    dtg_impl_->findPathToViewpoint(start_pos, viewpoint_id, result);
    return result;
}

// --- Visualization ---

void HistoryGraph::visualizationTimerCallback(const ros::TimerEvent& e) {
    publishVisualization();
}

void HistoryGraph::publishVisualization() {
    if (history_graph_pub_.getNumSubscribers() == 0) return;
    visualization_msgs::MarkerArray markers;
    dtg_impl_->visualize(markers);
    history_graph_pub_.publish(markers);
}

void HistoryGraph::pinViewpoint(int viewpoint_id) {
    dtg_impl_->pinViewpoint(viewpoint_id);
}

void HistoryGraph::unpinViewpoint(int viewpoint_id) {
    dtg_impl_->unpinViewpoint(viewpoint_id);
}

bool HistoryGraph::isViewpointPinned(int viewpoint_id) const {
    return dtg_impl_->isViewpointPinned(viewpoint_id);
}

// --- Legacy compatibility implementations ---

int HistoryGraph::findNearestHistoryNode(const Eigen::Vector3f& pos) const {
    // DTG does not have the same concept of nearest history node for frontier grouping.
    // Returning a constant groups all frontiers together, which is a safe default.
    return 0;
}

bool HistoryGraph::hasObstacleBetween(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) const {
    if (!global_map_) return false;
    return !global_map_->isLineFreeInflate(p1, p2, true);
}

bool HistoryGraph::getViewpointInfo(int viewpoint_id, Eigen::Vector3f& pos, float& yaw) const {
    auto vps = dtg_impl_->getValidViewpoints();
    for (const auto& vp : vps) {
        if (vp.viewpoint_id == viewpoint_id) {
            pos = vp.position;
            yaw = vp.yaw;
            return true;
        }
    }
    return false;
}

const ViewpointNode* HistoryGraph::getViewpoint(int viewpoint_id) const {
    tl_cached_vps = dtg_impl_->getValidViewpoints();
    for (const auto& vp : tl_cached_vps) {
        if (vp.viewpoint_id == viewpoint_id) {
            return &vp;
        }
    }
    return nullptr;
}

size_t HistoryGraph::getHistoryNodeCount() const {
    return dtg_impl_->getHNodeCount();
}

} // namespace rog_map
