/**
 * @file history_graph.cpp
 * @brief 历史节点图管理器实现
 */

#include "rog_map/history_graph.h"
#include "rog_map/viewpoint_manager.h"
#include "rog_map/dtg_history_graph.h"
#include <algorithm>

// 简化日志宏
#define HIST_INFO(fmt, ...) ROS_INFO("[HistGraph] " fmt, ##__VA_ARGS__)
#define HIST_WARN(fmt, ...) ROS_WARN("[HistGraph] " fmt, ##__VA_ARGS__)
#define HIST_DEBUG(fmt, ...) ROS_DEBUG("[HistGraph] " fmt, ##__VA_ARGS__)

namespace rog_map {

HistoryGraph::HistoryGraph() {}

void HistoryGraph::init(ros::NodeHandle& nh, GlobalMap::Ptr global_map) {
    nh_ = nh;
    global_map_ = global_map;

    // DTG init (new)
    dtg_.init(nh_, "dtg/history", global_map_, nullptr);
    
    // 初始化可视化发布器（使用与老架构相同的话题名称）
    history_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration/region_route", 1);

    HIST_INFO("Initialized (DTG mode)");
}

void HistoryGraph::start() {
    if (running_) return;
    running_ = true;
    
    // 2Hz定时器用于维护（可选，主要依赖updateUAVPosition）
    timer_ = nh_.createTimer(ros::Duration(0.5), &HistoryGraph::timerCallback, this);
    
    // 10Hz可视化定时器
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

    // DTG update
    dtg_.updateRobotPos(current_uav_pos_.cast<double>());

    double t_total = (ros::Time::now() - t_start).toSec() * 1000.0;

    ROS_INFO_THROTTLE(1.0, "[HistGraph][DTG] Update: H=%zu, F=%zu | total=%.1fms",
                      dtg_.getHNodeCount(), dtg_.getFNodeCount(), t_total);
}

// ==================== 历史节点管理 ====================

int HistoryGraph::updateUAVPosition(const Eigen::Vector3f& uav_pos) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    current_uav_pos_ = uav_pos;
    has_uav_pos_ = true;
    
    // 第一个节点
    if (!first_node_created_) {
        int node_id = createHistoryNode(uav_pos, -1);
        first_node_created_ = true;
        last_node_position_ = uav_pos;
        last_connected_node_idx_ = 0;
        HIST_INFO("First node %d at (%.2f,%.2f,%.2f)",
                  node_id, uav_pos.x(), uav_pos.y(), uav_pos.z());
        return node_id;
    }
    
    // 检查是否需要创建新节点
    float dist_to_last = (uav_pos - last_node_position_).norm();
    if (dist_to_last < config_.history_node_distance) {
        return -1;  // 距离不够，不创建
    }
    
    // 找最近的历史节点作为父节点
    int nearest_idx = -1;
    float min_dist = std::numeric_limits<float>::max();
    
    for (size_t i = 0; i < history_nodes_.size(); ++i) {
        float d = (history_nodes_[i].position - uav_pos).norm();
        if (d < min_dist) {
            min_dist = d;
            nearest_idx = static_cast<int>(i);
        }
    }
    
    // 创建新节点
    int parent_id = (nearest_idx >= 0) ? history_nodes_[nearest_idx].node_id : -1;
    int new_node_id = createHistoryNode(uav_pos, parent_id);
    
    // 增量连通性：只连接到最近的可达历史节点（形成树状结构）
    // 不需要连接到所有可达节点，因为图的联通性通过路径传递
    bool connected = false;
    if (nearest_idx >= 0) {
        if (!hasObstacleBetween(uav_pos, history_nodes_[nearest_idx].position)) {
            // 双向连接
            history_nodes_.back().addNeighbor(history_nodes_[nearest_idx].node_id);
            history_nodes_[nearest_idx].addNeighbor(new_node_id);
            connected = true;
            
            HIST_DEBUG("Connected node %d <-> %d (dist=%.2f)",
                       new_node_id, history_nodes_[nearest_idx].node_id, min_dist);
        }
    }
    
    // 如果最近节点不可达，尝试找其他可达节点（保证连通性）
    if (!connected) {
        for (size_t i = 0; i < history_nodes_.size() - 1; ++i) {
            float d = (history_nodes_[i].position - uav_pos).norm();
            if (d < config_.max_connection_distance) {
                if (!hasObstacleBetween(uav_pos, history_nodes_[i].position)) {
                    history_nodes_.back().addNeighbor(history_nodes_[i].node_id);
                    history_nodes_[i].addNeighbor(new_node_id);
                    connected = true;
                    HIST_DEBUG("Fallback connected node %d <-> %d (dist=%.2f)",
                               new_node_id, history_nodes_[i].node_id, d);
                    break;  // 只连接一个
                }
            }
        }
    }
    
    last_node_position_ = uav_pos;
    last_connected_node_idx_ = static_cast<int>(history_nodes_.size()) - 1;
    
    HIST_INFO("New node %d at (%.2f,%.2f,%.2f), neighbors=%zu",
              new_node_id, uav_pos.x(), uav_pos.y(), uav_pos.z(),
              history_nodes_.back().neighbors.size());
    
    return new_node_id;
}

int HistoryGraph::createHistoryNode(const Eigen::Vector3f& pos, int parent_id) {
    GraphNode node;
    node.node_id = next_node_id_++;
    node.position = pos;
    node.timestamp = ros::Time::now().toSec();
    node.parent_node_id = parent_id;
    
    history_nodes_.push_back(node);
    
    return node.node_id;
}

int HistoryGraph::findNearestHistoryNode(const Eigen::Vector3f& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (history_nodes_.empty()) return -1;
    
    int nearest_id = -1;
    float min_dist = std::numeric_limits<float>::max();
    
    for (const auto& node : history_nodes_) {
        float d = (node.position - pos).norm();
        if (d < min_dist) {
            min_dist = d;
            nearest_id = node.node_id;
        }
    }
    
    return nearest_id;
}

int HistoryGraph::findNearestReachableHistoryNode(const Eigen::Vector3f& pos) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (history_nodes_.empty()) return -1;
    
    // 按距离排序
    std::vector<std::pair<float, int>> dist_nodes;
    for (const auto& node : history_nodes_) {
        float d = (node.position - pos).norm();
        dist_nodes.emplace_back(d, node.node_id);
    }
    std::sort(dist_nodes.begin(), dist_nodes.end());
    
    // 找最近的可达节点
    for (const auto& dn : dist_nodes) {
        int idx = -1;
        for (size_t i = 0; i < history_nodes_.size(); ++i) {
            if (history_nodes_[i].node_id == dn.second) {
                idx = static_cast<int>(i);
                break;
            }
        }
        if (idx < 0) continue;
        
        if (!hasObstacleBetween(pos, history_nodes_[idx].position)) {
            return dn.second;
        }
    }
    
    return -1;
}

// ==================== 探索视点管理 ====================

int HistoryGraph::addViewpoint(const Eigen::Vector3f& position, float yaw,
                                int cluster_id, float score) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    ViewpointNode vp;
    vp.viewpoint_id = next_viewpoint_id_++;
    vp.position = position;
    vp.yaw = yaw;
    vp.cluster_id = cluster_id;
    vp.score = score;
    vp.timestamp = ros::Time::now().toSec();
    vp.explored = false;
    vp.valid = true;
    vp.connected_history_node = -1;
    
    // 连接到最佳历史节点
    connectViewpointToGraph(vp);
    
    viewpoints_[vp.viewpoint_id] = vp;
    
    return vp.viewpoint_id;
}

void HistoryGraph::updateViewpoints(const std::vector<ViewpointNode>& new_viewpoints) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 清除旧的视点
    viewpoints_.clear();
    
    // 添加新视点并连接
    for (auto vp : new_viewpoints) {
        vp.viewpoint_id = next_viewpoint_id_++;
        vp.timestamp = ros::Time::now().toSec();
        vp.explored = false;
        vp.valid = true;
        
        connectViewpointToGraph(vp);
        viewpoints_[vp.viewpoint_id] = vp;
    }
    
    HIST_DEBUG("Updated %zu viewpoints", viewpoints_.size());
}

int HistoryGraph::connectViewpointToGraph(ViewpointNode& vp) {
    if (history_nodes_.empty()) {
        vp.connected_history_node = -1;
        return -1;
    }
    
    // 找最近的可达历史节点
    std::vector<std::pair<float, int>> dist_nodes;
    for (const auto& node : history_nodes_) {
        float d = (node.position - vp.position).norm();
        if (d < config_.viewpoint_connection_radius) {
            dist_nodes.emplace_back(d, node.node_id);
        }
    }
    
    if (dist_nodes.empty()) {
        // 没有在范围内的节点，找最近的
        for (const auto& node : history_nodes_) {
            float d = (node.position - vp.position).norm();
            dist_nodes.emplace_back(d, node.node_id);
        }
    }
    
    std::sort(dist_nodes.begin(), dist_nodes.end());
    
    for (const auto& dn : dist_nodes) {
        // 找到节点索引
        const GraphNode* node_ptr = nullptr;
        for (const auto& node : history_nodes_) {
            if (node.node_id == dn.second) {
                node_ptr = &node;
                break;
            }
        }
        if (!node_ptr) continue;
        
        // 检查可达性
        if (!hasObstacleBetween(vp.position, node_ptr->position)) {
            vp.connected_history_node = dn.second;
            return dn.second;
        }
    }
    
    vp.connected_history_node = -1;
    return -1;
}

void HistoryGraph::markViewpointExplored(int viewpoint_id) {
    dtg_.markViewpointExplored(viewpoint_id);
}

void HistoryGraph::markViewpointInvalid(int viewpoint_id) {
    dtg_.markViewpointInvalid(viewpoint_id);
}

void HistoryGraph::cleanupViewpoints() {
    // 移除已探索和无效的视点
    auto it = viewpoints_.begin();
    while (it != viewpoints_.end()) {
        if (it->second.explored || !it->second.valid) {
            it = viewpoints_.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<ViewpointNode> HistoryGraph::getValidViewpoints() const {
    return dtg_.getValidViewpoints();
}

bool HistoryGraph::getViewpointInfo(int viewpoint_id, Eigen::Vector3f& position, float& yaw) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = viewpoints_.find(viewpoint_id);
    if (it != viewpoints_.end()) {
        position = it->second.position;
        yaw = it->second.yaw;
        return true;
    }
    return false;
}

const ViewpointNode* HistoryGraph::getViewpoint(int viewpoint_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = viewpoints_.find(viewpoint_id);
    if (it != viewpoints_.end()) {
        return &(it->second);
    }
    return nullptr;
}

// ==================== 路径查询 ====================

PathResult HistoryGraph::findPathToViewpoint(const Eigen::Vector3f& start_pos, int viewpoint_id) {
    PathResult result;

    // DTG path query
    if (!dtg_.findPathToViewpoint(start_pos, viewpoint_id, result)) {
        return result;
    }
    return result;
}

PathResult HistoryGraph::findPath(const Eigen::Vector3f& start_pos, 
                                   const Eigen::Vector3f& end_pos) {
    PathResult result;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 直接可达
    if (!hasObstacleBetween(start_pos, end_pos)) {
        result.success = true;
        result.waypoints.push_back(end_pos);
        result.total_distance = (end_pos - start_pos).norm();
        return result;
    }
    
    if (history_nodes_.empty()) {
        return result;
    }
    
    // 找起点和终点最近的可达节点
    int start_node_id = -1, end_node_id = -1;
    float min_start_dist = std::numeric_limits<float>::max();
    float min_end_dist = std::numeric_limits<float>::max();
    
    for (const auto& node : history_nodes_) {
        float ds = (node.position - start_pos).norm();
        float de = (node.position - end_pos).norm();
        
        if (ds < min_start_dist && !hasObstacleBetween(start_pos, node.position)) {
            min_start_dist = ds;
            start_node_id = node.node_id;
        }
        if (de < min_end_dist && !hasObstacleBetween(end_pos, node.position)) {
            min_end_dist = de;
            end_node_id = node.node_id;
        }
    }
    
    if (start_node_id < 0 || end_node_id < 0) {
        return result;
    }
    
    // BFS
    std::vector<int> node_path;
    if (!bfsSearchPath(start_node_id, end_node_id, node_path)) {
        return result;
    }
    
    simplifyPath(node_path);
    
    // 构建结果
    result.success = true;
    result.history_node_path = node_path;
    
    for (int nid : node_path) {
        for (const auto& node : history_nodes_) {
            if (node.node_id == nid) {
                result.waypoints.push_back(node.position);
                break;
            }
        }
    }
    result.waypoints.push_back(end_pos);
    
    // 计算距离
    result.total_distance = min_start_dist;
    Eigen::Vector3f prev = start_pos;
    for (const auto& wp : result.waypoints) {
        result.total_distance += (wp - prev).norm();
        prev = wp;
    }
    
    return result;
}

bool HistoryGraph::hasObstacleBetween(const Eigen::Vector3f& start, const Eigen::Vector3f& end) {
    if (!global_map_) return false;
    
    Eigen::Vector3f dir = end - start;
    float dist = dir.norm();
    if (dist < 1e-6f) return false;
    
    dir.normalize();
    float step = config_.raycast_step;
    int num_steps = static_cast<int>(dist / step);
    
    int consecutive_occupied = 0;
    
    for (int i = 0; i <= num_steps; ++i) {
        Eigen::Vector3f pos = start + dir * (i * step);
        
        if (global_map_->isOccupied(pos)) {
            consecutive_occupied++;
            if (consecutive_occupied >= config_.consecutive_occupied_threshold) {
                return true;
            }
        } else {
            consecutive_occupied = 0;
        }
    }
    
    return false;
}

float HistoryGraph::estimatePathLength(const Eigen::Vector3f& start_pos, int viewpoint_id) {
    PathResult result = findPathToViewpoint(start_pos, viewpoint_id);
    if (!result.success) return -1.0f;
    return result.total_distance;
}

bool HistoryGraph::bfsSearchPath(int start_node_id, int end_node_id, std::vector<int>& path) {
    path.clear();
    
    if (start_node_id == end_node_id) {
        path.push_back(start_node_id);
        return true;
    }
    
    // 建立ID到索引的映射
    std::unordered_map<int, int> id_to_idx;
    for (size_t i = 0; i < history_nodes_.size(); ++i) {
        id_to_idx[history_nodes_[i].node_id] = static_cast<int>(i);
    }
    
    auto it_start = id_to_idx.find(start_node_id);
    auto it_end = id_to_idx.find(end_node_id);
    if (it_start == id_to_idx.end() || it_end == id_to_idx.end()) {
        return false;
    }
    
    int start_idx = it_start->second;
    int end_idx = it_end->second;
    
    // BFS
    std::vector<bool> visited(history_nodes_.size(), false);
    std::vector<int> parent(history_nodes_.size(), -1);
    std::queue<int> q;
    
    q.push(start_idx);
    visited[start_idx] = true;
    
    while (!q.empty()) {
        int curr_idx = q.front();
        q.pop();
        
        if (curr_idx == end_idx) break;
        
        const GraphNode& curr_node = history_nodes_[curr_idx];
        
        for (int neighbor_id : curr_node.neighbors) {
            auto it = id_to_idx.find(neighbor_id);
            if (it == id_to_idx.end()) continue;
            
            int neighbor_idx = it->second;
            if (!visited[neighbor_idx]) {
                visited[neighbor_idx] = true;
                parent[neighbor_idx] = curr_idx;
                q.push(neighbor_idx);
            }
        }
    }
    
    if (!visited[end_idx]) {
        return false;
    }
    
    // 回溯路径
    int curr = end_idx;
    while (curr != -1) {
        path.push_back(history_nodes_[curr].node_id);
        curr = parent[curr];
    }
    std::reverse(path.begin(), path.end());
    
    return true;
}

void HistoryGraph::simplifyPath(std::vector<int>& path) {
    if (path.size() <= 2) return;
    
    // 建立ID到位置的映射
    std::unordered_map<int, Eigen::Vector3f> id_to_pos;
    for (const auto& node : history_nodes_) {
        id_to_pos[node.node_id] = node.position;
    }
    
    std::vector<int> simplified;
    simplified.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        // 找从i开始能直接到达的最远点
        size_t furthest = i + 1;
        for (size_t j = i + 2; j < path.size(); ++j) {
            if (!hasObstacleBetween(id_to_pos[path[i]], id_to_pos[path[j]])) {
                furthest = j;
            }
        }
        
        simplified.push_back(path[furthest]);
        i = furthest;
    }
    
    path = simplified;
}

int HistoryGraph::getEdgeCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    int count = 0;
    for (const auto& node : history_nodes_) {
        count += static_cast<int>(node.neighbors.size());
    }
    return count / 2;  // 每条边被计数两次
}

// ==================== 可视化 ====================

// 颜色表（与老架构一致）
static const std::vector<std::array<float, 4>> kClusterColors = {
    {1.0f, 0.2f, 0.2f, 0.8f},   // 红色
    {0.2f, 1.0f, 0.2f, 0.8f},   // 绿色
    {0.2f, 0.4f, 1.0f, 0.8f},   // 蓝色
    {1.0f, 1.0f, 0.2f, 0.8f},   // 黄色
    {1.0f, 0.2f, 1.0f, 0.8f},   // 品红
    {0.2f, 1.0f, 1.0f, 0.8f},   // 青色
    {1.0f, 0.6f, 0.2f, 0.8f},   // 橙色
    {0.6f, 0.2f, 1.0f, 0.8f},   // 紫色
};

static std::array<float, 4> getNodeColor(int node_id) {
    if (node_id < 0) node_id = 0;
    return kClusterColors[node_id % kClusterColors.size()];
}

void HistoryGraph::visualizationTimerCallback(const ros::TimerEvent& e) {
    publishVisualization();
}

void HistoryGraph::publishVisualization() {
    if (history_graph_pub_.getNumSubscribers() == 0) return;
    
    visualization_msgs::MarkerArray markers;
    visualizeHistoryGraph(markers);
    history_graph_pub_.publish(markers);
}

void HistoryGraph::visualizeHistoryGraph(visualization_msgs::MarkerArray& markers) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    int marker_id = 0;
    
    // 清除旧marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.ns = "history_nodes";
    markers.markers.push_back(delete_marker);
    
    delete_marker.ns = "history_node_ids";
    markers.markers.push_back(delete_marker);
    
    delete_marker.ns = "history_edges";
    markers.markers.push_back(delete_marker);
    
    delete_marker.ns = "viewpoint_connections";
    markers.markers.push_back(delete_marker);
    
    // 1. 绘制历史节点（球体）
    for (const auto& node : history_nodes_) {
        auto color = getNodeColor(node.node_id);
        
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = ros::Time::now();
        sphere.ns = "history_nodes";
        sphere.id = marker_id++;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        
        sphere.pose.position.x = node.position.x();
        sphere.pose.position.y = node.position.y();
        sphere.pose.position.z = node.position.z();
        sphere.pose.orientation.w = 1.0;
        
        sphere.scale.x = 0.4;
        sphere.scale.y = 0.4;
        sphere.scale.z = 0.4;
        
        sphere.color.r = color[0];
        sphere.color.g = color[1];
        sphere.color.b = color[2];
        sphere.color.a = 0.9f;
        
        markers.markers.push_back(sphere);
        
        // 节点ID文本
        visualization_msgs::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = ros::Time::now();
        text.ns = "history_node_ids";
        text.id = marker_id++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        
        text.pose.position.x = node.position.x();
        text.pose.position.y = node.position.y();
        text.pose.position.z = node.position.z() + 0.5;
        
        text.scale.z = 0.3;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        text.text = "N" + std::to_string(node.node_id);
        
        markers.markers.push_back(text);
    }
    
    // 2. 绘制历史节点之间的边（线段）
    visualization_msgs::Marker edges;
    edges.header.frame_id = "map";
    edges.header.stamp = ros::Time::now();
    edges.ns = "history_edges";
    edges.id = marker_id++;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    
    edges.scale.x = 0.05;
    edges.color.r = 0.8f;
    edges.color.g = 0.8f;
    edges.color.b = 0.8f;
    edges.color.a = 0.6f;
    
    // 构建node_id到position的映射
    std::unordered_map<int, Eigen::Vector3f> id_to_pos;
    for (const auto& node : history_nodes_) {
        id_to_pos[node.node_id] = node.position;
    }
    
    // 遍历所有边（避免重复，只绘制 node_id < neighbor_id 的边）
    for (const auto& node : history_nodes_) {
        for (int neighbor_id : node.neighbors) {
            if (node.node_id < neighbor_id) {
                auto it = id_to_pos.find(neighbor_id);
                if (it != id_to_pos.end()) {
                    geometry_msgs::Point p1, p2;
                    p1.x = node.position.x();
                    p1.y = node.position.y();
                    p1.z = node.position.z();
                    p2.x = it->second.x();
                    p2.y = it->second.y();
                    p2.z = it->second.z();
                    edges.points.push_back(p1);
                    edges.points.push_back(p2);
                }
            }
        }
    }
    
    if (!edges.points.empty()) {
        markers.markers.push_back(edges);
    }
    
    // 3. 绘制视点到历史节点的连接线
    visualization_msgs::Marker vp_connections;
    vp_connections.header.frame_id = "map";
    vp_connections.header.stamp = ros::Time::now();
    vp_connections.ns = "viewpoint_connections";
    vp_connections.id = marker_id++;
    vp_connections.type = visualization_msgs::Marker::LINE_LIST;
    vp_connections.action = visualization_msgs::Marker::ADD;
    
    vp_connections.scale.x = 0.03;
    vp_connections.color.r = 0.2f;
    vp_connections.color.g = 0.8f;
    vp_connections.color.b = 0.2f;
    vp_connections.color.a = 0.5f;
    
    for (const auto& vp_pair : viewpoints_) {
        const ViewpointNode& vp = vp_pair.second;
        if (!vp.valid || vp.explored) continue;
        if (vp.connected_history_node < 0) continue;
        
        auto it = id_to_pos.find(vp.connected_history_node);
        if (it != id_to_pos.end()) {
            geometry_msgs::Point p1, p2;
            p1.x = vp.position.x();
            p1.y = vp.position.y();
            p1.z = vp.position.z();
            p2.x = it->second.x();
            p2.y = it->second.y();
            p2.z = it->second.z();
            vp_connections.points.push_back(p1);
            vp_connections.points.push_back(p2);
        }
    }
    
    if (!vp_connections.points.empty()) {
        markers.markers.push_back(vp_connections);
    }
}

} // namespace rog_map
