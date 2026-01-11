#ifndef ROG_MAP_HISTORY_GRAPH_H
#define ROG_MAP_HISTORY_GRAPH_H

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <mutex>
#include <ros/ros.h>
#include "star_mvf/map/global_map.h"

// 前向声明 ViewpointManager
namespace rog_map { class ViewpointManager; }

namespace rog_map {

using GlobalMap = star_mvf::GlobalMap;

// ==================== 数据结构 ====================

struct NavNode {
    int id;
    Eigen::Vector3d position;
    std::vector<int> edge_ids;           // 物理连接的其他拓扑边ID
    std::unordered_set<int> associated_subgrid_ids; // 归属该节点的子网格线性ID
};

struct NavEdge {
    int id;
    int node_a, node_b;
    std::vector<Eigen::Vector3d> waypoints; // 避障精简路径
    double distance;                        // 实际路径代价
};

// Dijkstra 缓存结果 (FSM 中使用)
struct DijkstraResult {
    std::unordered_map<int, double> dists;   // ID -> Distance
    std::unordered_map<int, int> parents;    // ID -> ParentID
    int start_node_id = -1;
};

// A* 辅助节点
struct AStarNode {
    VoxelIndex index;
    double g_score;
    double f_score;
    AStarNode* parent;
    AStarNode(VoxelIndex idx, double g, double f, AStarNode* p)
        : index(idx), g_score(g), f_score(f), parent(p) {}
};

struct AStarCompare {
    bool operator()(AStarNode* a, AStarNode* b) { return a->f_score > b->f_score; }
};

/**
 * @brief 配置参数
 */
struct HistoryGraphConfig {
    float update_rate = 2.0f;
    float node_spacing = 5.0f;
    float max_assoc_dist = 10.0f;
};

// ==================== HistoryGraph 类 ====================
class HistoryGraph {
public:
    typedef std::shared_ptr<HistoryGraph> Ptr;

    Eigen::Vector3d current_position;
    bool has_uav_state_ = false;
    ros::NodeHandle nh_;
    ros::Timer timer_;

    // 兼容你的无参构造需求(如果有的话)，但通常推荐上面的带参构造
    HistoryGraph() {} 

    void init(ros::NodeHandle& nh, GlobalMap::Ptr global_map) {
        nh_ = nh;
        global_map_ = global_map;
        
        nh_.param("history_graph/update_rate", config_.update_rate, 2.0f);
        nh_.param("history_graph/node_spacing", config_.node_spacing, 5.0f);
        nh_.param("history_graph/max_assoc_dist", config_.max_assoc_dist, 10.0f);
        
        resolution_ = global_map_->getResolution();
        next_id_ = 0;
    }

    void start() {
        timer_ = nh_.createTimer(ros::Duration(1.0 / config_.update_rate),
                                 &HistoryGraph::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent& e) {
        update();
    }

    // 设置 ViewpointManager 指针 (如果需要)
    void setViewpointManager(ViewpointManager* vm) { vm_ = vm; }

    void updateUAVState(const Eigen::Vector3f& position) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_position = position.cast<double>(); // 更新成员变量
        has_uav_state_ = true;
    }

    /**
     * @brief 尝试在当前位置更新拓扑图 (无参版本，使用内部状态)
     */
    int update() {
        if (!has_uav_state_) return -1;
        return update(current_position);
    }

    /**
     * @brief 尝试在指定位置更新拓扑图 (有参版本)
     */
    int update(const Eigen::Vector3d& curr_pos) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (nodes_.empty()) {
            int new_id = addNodeInternal(curr_pos, -1);
            refreshSubGridOwnershipInternal(); 
            return new_id;
        }

        int nearest_id = findNearestNode2DInternal(curr_pos);
        if (nearest_id == -1) return -1;

        double dist = (nodes_[nearest_id].position - curr_pos).norm();

        if (dist >= config_.node_spacing) {
            int new_id = addNodeInternal(curr_pos, nearest_id);
            performLoopClosureInternal(new_id);
            refreshSubGridOwnershipInternal(); 
            return new_id;
        }
        return -1;
    }

    // [恢复] 获取所有节点ID (ViewpointManager 需要)
    std::vector<int> getAllNodeIds() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::vector<int> ids;
        ids.reserve(nodes_.size());
        for(const auto& kv : nodes_) {
            ids.push_back(kv.first);
        }
        return ids;
    }

    size_t getHistoryNodeCount() const { 
        std::lock_guard<std::mutex> lock(data_mutex_);
        return nodes_.size(); 
    }

    // ==================== 子网格关联接口 ====================
    std::vector<int> getAssociatedSubGrids(int node_id) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (nodes_.find(node_id) == nodes_.end()) return {};
        const auto& set = nodes_[node_id].associated_subgrid_ids;
        return std::vector<int>(set.begin(), set.end());
    }

    void releaseSubGrid(int subgrid_id) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (subgrid_to_node_owner_.count(subgrid_id)) {
            int owner_id = subgrid_to_node_owner_[subgrid_id];
            nodes_[owner_id].associated_subgrid_ids.erase(subgrid_id);
            subgrid_to_node_owner_.erase(subgrid_id);
        }
    }

    void refreshSubGridOwnership() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        refreshSubGridOwnershipInternal();
    }

    // ==================== 核心计算接口 ====================

    /**
     * @brief 计算单源 Dijkstra 到指定目标集合
     */
    void computeDijkstraToTargets(int start_node_id, 
                                  std::unordered_set<int> targets, 
                                  DijkstraResult& result) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        result.start_node_id = start_node_id;
        result.dists.clear();
        result.parents.clear();
        
        if (nodes_.find(start_node_id) == nodes_.end()) return;

        auto cmp = [](std::pair<int, double> a, std::pair<int, double> b) { return a.second > b.second; };
        std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, decltype(cmp)> pq(cmp);

        result.dists[start_node_id] = 0.0;
        pq.push({start_node_id, 0.0});

        std::unordered_set<int> remaining_targets = targets;
        if (remaining_targets.count(start_node_id)) remaining_targets.erase(start_node_id);

        while (!pq.empty()) {
            if (remaining_targets.empty()) break; 

            int u = pq.top().first;
            double d = pq.top().second;
            pq.pop();

            if (d > result.dists[u]) continue;

            for (int edge_id : nodes_[u].edge_ids) {
                const NavEdge& edge = edges_[edge_id];
                int v = (edge.node_a == u) ? edge.node_b : edge.node_a;
                double new_dist = d + edge.distance;

                if (result.dists.find(v) == result.dists.end() || new_dist < result.dists[v]) {
                    result.dists[v] = new_dist;
                    result.parents[v] = u;
                    pq.push({v, new_dist});

                    if (remaining_targets.count(v)) remaining_targets.erase(v);
                }
            }
        }
    }

    /**
     * @brief 极速评估视点代价
     */
    double evaluateViewpointCost(const Eigen::Vector3d& curr_pos, 
                                 int anchor_node_id,
                                 const Eigen::Vector3d& vp_pos,
                                 const DijkstraResult& cached_dijkstra,
                                 std::vector<Eigen::Vector3d>* out_path) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        double euclidean_dist = (curr_pos - vp_pos).norm();

        // 1. 近场快速通道 (< 5m)
        if (euclidean_dist < 5.0) {
            // A. 射线检测
            if (global_map_->isLineFreeInflate(curr_pos, vp_pos, true)) {
                if (out_path) *out_path = {curr_pos, vp_pos};
                return euclidean_dist;
            }
            // B. 局部 A*
            auto direct_path = runLocalAStarInternal(curr_pos, vp_pos);
            if (!direct_path.empty()) {
                double len = calculatePathLengthInternal(direct_path);
                if (len < 10.0) {
                    if (out_path) *out_path = shortcutPathInternal(curr_pos, direct_path);
                    return len;
                }
            }
        }

        // 2. 远场通道
        if (nodes_.find(anchor_node_id) == nodes_.end()) return std::numeric_limits<double>::max();
        if (cached_dijkstra.dists.find(anchor_node_id) == cached_dijkstra.dists.end()) {
            return std::numeric_limits<double>::max(); 
        }
        
        double cost_graph = cached_dijkstra.dists.at(anchor_node_id);
        
        // Exit Path
        std::vector<Eigen::Vector3d> exit_path;
        const Eigen::Vector3d& anchor_pos = nodes_[anchor_node_id].position;
        
        if (global_map_->isLineFreeInflate(anchor_pos, vp_pos, true)) {
            exit_path = {anchor_pos, vp_pos};
        } else {
            exit_path = runLocalAStarInternal(anchor_pos, vp_pos);
            if (exit_path.empty()) return std::numeric_limits<double>::max();
        }
        double cost_exit = calculatePathLengthInternal(exit_path);

        if (out_path) *out_path = exit_path; 

        return cost_graph + cost_exit; 
    }

    /**
     * @brief 利用 Dijkstra 缓存重建全局路径
     */
    bool reconstructPathFromCache(const DijkstraResult& cache, 
                                  int end_node_id, 
                                  std::vector<Eigen::Vector3d>& out_path) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        out_path.clear();

        if (cache.dists.find(end_node_id) == cache.dists.end()) return false;

        int curr = end_node_id;
        std::vector<Eigen::Vector3d> reverse_path;
        
        while (curr != cache.start_node_id) {
            reverse_path.push_back(nodes_[curr].position);
            if (cache.parents.find(curr) == cache.parents.end()) break; 
            curr = cache.parents.at(curr);
        }
        reverse_path.push_back(nodes_[cache.start_node_id].position);
        
        std::reverse(reverse_path.begin(), reverse_path.end());
        out_path = std::move(reverse_path);
        return true;
    }

    // ==================== 基础工具函数 ====================

    int findNearestNode(const Eigen::Vector3d& pos) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return findNearestNode2DInternal(pos);
    }

    std::vector<Eigen::Vector3d> runLocalAStar(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return runLocalAStarInternal(start, end);
    }

    std::vector<Eigen::Vector3d> shortcutPath(const Eigen::Vector3d& start, const std::vector<Eigen::Vector3d>& path) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return shortcutPathInternal(start, path);
    }

    double calculatePathLength(const std::vector<Eigen::Vector3d>& path) {
        return calculatePathLengthInternal(path);
    }

    Eigen::Vector3d getNodePos(int id) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (nodes_.count(id)) return nodes_[id].position;
        return Eigen::Vector3d::Zero();
    }
    
    void computeDijkstraToAll(int start_node_id, std::unordered_map<int, double>& dist_map) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (nodes_.find(start_node_id) == nodes_.end()) return;
        std::unordered_map<int, int> p_node, p_edge;
        runDijkstraInternal(start_node_id, dist_map, p_node, p_edge);
    }

private:
    GlobalMap::Ptr global_map_;
    ViewpointManager* vm_ = nullptr;
    HistoryGraphConfig config_;

    float resolution_;
    int next_id_;

    std::unordered_map<int, NavNode> nodes_;
    std::unordered_map<int, NavEdge> edges_;
    std::unordered_map<uint64_t, std::vector<int>> node_grid_index_; 
    std::unordered_map<int, int> subgrid_to_node_owner_;            
    
    mutable std::mutex data_mutex_;

    // ==================== 内部无锁实现 ====================
    void refreshSubGridOwnershipInternal() {
        std::vector<int> frontier_sg_ids;
        global_map_->getFrontierSubGrids(frontier_sg_ids);

        for (int sg_id : frontier_sg_ids) {
            const auto& sg = global_map_->getSubGrid(sg_id);
            Eigen::Vector3d sg_center = sg.center.cast<double>();

            int best_node_id = -1;
            double min_dist = config_.max_assoc_dist;
            
            std::vector<int> nearby_nodes = findNodesInRange2DInternal(sg_center, config_.max_assoc_dist);
            
            for (int node_id : nearby_nodes) {
                double d = (nodes_[node_id].position - sg_center).norm();
                if (d < min_dist) {
                    min_dist = d;
                    best_node_id = node_id;
                }
            }

            if (best_node_id != -1) {
                int current_owner = subgrid_to_node_owner_.count(sg_id) ? subgrid_to_node_owner_[sg_id] : -1;
                if (current_owner != best_node_id) {
                    if (current_owner != -1) nodes_[current_owner].associated_subgrid_ids.erase(sg_id);
                    subgrid_to_node_owner_[sg_id] = best_node_id;
                    nodes_[best_node_id].associated_subgrid_ids.insert(sg_id);
                }
            }
        }
    }

    int addNodeInternal(const Eigen::Vector3d& pos, int prev_id) {
        int id = next_id_++;
        NavNode node;
        node.id = id;
        node.position = pos;
        nodes_[id] = node;

        registerToIndexInternal(pos, id);
        if (prev_id != -1) tryAddEdgeInternal(id, prev_id);
        return id;
    }

    void performLoopClosureInternal(int new_node_id) {
        const Eigen::Vector3d& pos = nodes_[new_node_id].position;
        std::vector<int> nearby = findNodesInRange2DInternal(pos, config_.node_spacing * 1.2);
        for (int nid : nearby) {
            if (nid == new_node_id || hasEdgeBetweenInternal(new_node_id, nid)) continue;
            tryAddEdgeInternal(new_node_id, nid);
        }
    }

    bool tryAddEdgeInternal(int id_a, int id_b) {
        auto pts = computeSimplifiedPathInternal(nodes_[id_a].position, nodes_[id_b].position);
        if (pts.empty()) return false;
        NavEdge e; e.id = edges_.size(); e.node_a = id_a; e.node_b = id_b;
        e.waypoints = pts; e.distance = calculatePathLengthInternal(pts);
        edges_[e.id] = e;
        nodes_[id_a].edge_ids.push_back(e.id);
        nodes_[id_b].edge_ids.push_back(e.id);
        return true;
    }

    std::vector<Eigen::Vector3d> computeSimplifiedPathInternal(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
        if (global_map_->isLineFreeInflate(start, end, true)) return {start, end};
        auto raw = runLocalAStarInternal(start, end);
        if (raw.size() < 2) return {};
        return simplifyPathInternal(raw);
    }
    
    std::vector<Eigen::Vector3d> simplifyPathInternal(const std::vector<Eigen::Vector3d>& path) {
        if (path.empty()) return {};
        return shortcutPathInternal(path.front(), path); 
    }

    std::vector<Eigen::Vector3d> shortcutPathInternal(const Eigen::Vector3d& start, const std::vector<Eigen::Vector3d>& path) {
        if (path.empty()) return {};
        std::vector<Eigen::Vector3d> optimized;
        optimized.push_back(start);
        int curr_idx = -1; 
        while (true) {
            Eigen::Vector3d cur_pt = (curr_idx == -1) ? start : path[curr_idx];
            int best_next = -1;
            for (int i = (int)path.size() - 1; i > curr_idx; --i) {
                if (global_map_->isLineFreeInflate(cur_pt, path[i], true)) {
                    best_next = i; break;
                }
            }
            if (best_next != -1) {
                optimized.push_back(path[best_next]);
                curr_idx = best_next;
                if (best_next == (int)path.size() - 1) break;
            } else {
                if (curr_idx + 1 < (int)path.size()) optimized.push_back(path[++curr_idx]); else break;
            }
        }
        return optimized;
    }

    std::vector<Eigen::Vector3d> runLocalAStarInternal(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
        std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarCompare> open_list;
        std::unordered_map<VoxelIndex, double, VoxelIndexHash> g_scores;
        std::vector<AStarNode*> pool;
        VoxelIndex s_idx = global_map_->posToIndex(start), e_idx = global_map_->posToIndex(end);
        AStarNode* s_n = new AStarNode(s_idx, 0, (end-start).norm(), nullptr);
        open_list.push(s_n); pool.push_back(s_n); g_scores[s_idx] = 0;
        
        std::vector<Eigen::Vector3d> path;
        int steps = 0; int max_steps = 3000; 

        while(!open_list.empty()){
            AStarNode* curr = open_list.top(); open_list.pop();
            
            if(curr->index == e_idx){ 
                path = reconstructPathInternal(curr); 
                break; 
            }
            
            if (++steps > max_steps) break;

            for(int dx=-1; dx<=1; ++dx) for(int dy=-1; dy<=1; ++dy){
                if(dx==0 && dy==0) continue;
                VoxelIndex n_idx = curr->index; n_idx.x+=dx; n_idx.y+=dy;
                if(global_map_->isOccupiedInflate(n_idx)) continue;
                
                double step = (dx!=0 && dy!=0) ? 1.414 : 1.0;
                double tg = curr->g_score + step * resolution_;
                
                if(g_scores.find(n_idx) == g_scores.end() || tg < g_scores[n_idx]){
                    g_scores[n_idx] = tg;
                    double h = (global_map_->indexToPos(n_idx) - end).norm();
                    open_list.push(new AStarNode(n_idx, tg, tg+h, curr));
                    pool.push_back(open_list.top());
                }
            }
        }
        for(auto n : pool) delete n;
        return path;
    }

    void runDijkstraInternal(int target_id, std::unordered_map<int, double>& dists, 
                             std::unordered_map<int, int>& parent_node, std::unordered_map<int, int>& parent_edge) {
        auto cmp = [](std::pair<int, double> a, std::pair<int, double> b) { return a.second > b.second; };
        std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, decltype(cmp)> pq(cmp);
        for (auto const& [id, n] : nodes_) dists[id] = std::numeric_limits<double>::max();
        dists[target_id] = 0;
        pq.push({target_id, 0.0});
        while (!pq.empty()) {
            int u = pq.top().first; double d = pq.top().second; pq.pop();
            if (d > dists[u]) continue;
            for (int e_id : nodes_[u].edge_ids) {
                const NavEdge& e = edges_[e_id];
                int v = (e.node_a == u) ? e.node_b : e.node_a;
                if (dists[u] + e.distance < dists[v]) {
                    dists[v] = dists[u] + e.distance;
                    parent_node[v] = u; parent_edge[v] = e_id;
                    pq.push({v, dists[v]});
                }
            }
        }
    }

    int findIdealAnchorNodeInternal(const Eigen::Vector3d& pos, const std::unordered_map<int, double>& dist_map) {
        std::vector<int> candidates = findNodesInRange2DInternal(pos, config_.node_spacing * 1.6);
        int best_id = -1; double min_total_cost = std::numeric_limits<double>::max();
        for (int id : candidates) {
            if (dist_map.find(id) == dist_map.end()) continue;
            auto local_path = runLocalAStarInternal(pos, nodes_[id].position);
            if (local_path.empty()) continue; 
            double total_cost = calculatePathLengthInternal(local_path) + dist_map.at(id);
            if (total_cost < min_total_cost) { min_total_cost = total_cost; best_id = id; }
        }
        return best_id;
    }

    std::vector<int> findNodesInRange2DInternal(const Eigen::Vector3d& pos, double r) {
        int gx = std::floor(pos.x() / config_.node_spacing), gy = std::floor(pos.y() / config_.node_spacing);
        int r_grid = std::ceil(r / config_.node_spacing);
        std::vector<int> res;
        for (int i = -r_grid; i <= r_grid; ++i) for (int j = -r_grid; j <= r_grid; ++j) {
            uint64_t key = packKey(gx + i, gy + j);
            if (node_grid_index_.count(key)) {
                for (int id : node_grid_index_[key]) 
                    if ((nodes_[id].position - pos).norm() <= r) res.push_back(id);
            }
        }
        return res;
    }

    int findNearestNode2DInternal(const Eigen::Vector3d& pos) {
        auto cands = findNodesInRange2DInternal(pos, config_.node_spacing * 2.0);
        int bid = -1; double md = std::numeric_limits<double>::max();
        for (int id : cands) {
            double d = (nodes_[id].position - pos).norm();
            if (d < md) { md = d; bid = id; }
        }
        return bid;
    }

    std::vector<Eigen::Vector3d> reconstructPathInternal(AStarNode* end_node) {
        std::vector<Eigen::Vector3d> path;
        AStarNode* curr = end_node;
        while (curr) { path.push_back(global_map_->indexToPos(curr->index)); curr = curr->parent; }
        std::reverse(path.begin(), path.end()); return path;
    }

    double calculatePathLengthInternal(const std::vector<Eigen::Vector3d>& path) {
        double len = 0;
        for (size_t i = 1; i < path.size(); ++i) len += (path[i] - path[i-1]).norm();
        return len;
    }

    inline uint64_t packKey(int x, int y) {
        return ((uint64_t)static_cast<int32_t>(x) << 32) | static_cast<uint32_t>(static_cast<int32_t>(y));
    }

    void registerToIndexInternal(const Eigen::Vector3d& pos, int id) {
        int gx = std::floor(pos.x() / config_.node_spacing), gy = std::floor(pos.y() / config_.node_spacing);
        node_grid_index_[packKey(gx, gy)].push_back(id);
    }

    bool hasEdgeBetweenInternal(int id_a, int id_b) {
        for (int eid : nodes_[id_a].edge_ids)
            if (edges_[eid].node_a == id_b || edges_[eid].node_b == id_b) return true;
        return false;
    }
};

} // namespace rog_map

#endif