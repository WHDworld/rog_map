#include "rog_map/history_graph.h"

#include <algorithm>

#include "rog_map/viewpoint_manager.h"

namespace rog_map {

HistoryGraph::HistoryGraph() = default;

void HistoryGraph::init(ros::NodeHandle& nh, GlobalMap::Ptr global_map) {
  nh_ = nh;
  global_map_ = global_map;

  nh_.param("history_graph/update_rate", config_.update_rate, 2.0f);
  nh_.param("history_graph/node_spacing", config_.node_spacing, 5.0f);
  nh_.param("history_graph/max_assoc_dist", config_.max_assoc_dist, 10.0f);
  nh_.param("viewpoint_manager/uav_radius", config_.uav_radius, 0.3f); // Read from shared param
  resolution_ = global_map_ ? global_map_->getResolution() : 0.0f;
  next_id_ = 0;
}

void HistoryGraph::start() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / config_.update_rate), &HistoryGraph::timerCallback, this);
}

void HistoryGraph::timerCallback(const ros::TimerEvent& /*e*/) { 
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_uav_state_) return;
    update(current_position);
}

void HistoryGraph::updateUAVState(const Eigen::Vector3d& position) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_position = position.cast<double>();
  has_uav_state_ = true;
}

int HistoryGraph::update(const Eigen::Vector3d& curr_pos) {
  double t_start = ros::Time::now().toSec();
  if (nodes_.empty()) {
    int new_id = addNodeInternal(curr_pos, -1);
    // [Added] Check if initial node failed (unlikely but possible if taking off near wall)
    if (new_id != -1) {
        refreshSubGridOwnershipInternal();
    }
    return new_id;
  }

  int nearest_id = findNearestNode2DInternal(curr_pos);
  if (nearest_id == -1) return -1;

  double dist = (nodes_[nearest_id].position - curr_pos).norm();

  if (dist >= config_.node_spacing) {
    int new_id = addNodeInternal(curr_pos, nearest_id);
    
    // [Added] Check if node added successfully
    if (new_id != -1) {
        performLoopClosureInternal(new_id);
        // ▼▼▼▼▼▼ [新增] 孤立检查与强制重连 ▼▼▼▼▼▼
        if (nodes_[new_id].edge_ids.empty()) {
            ROS_WARN("[HistoryGraph] Node %d is ORPHANED (No edges)! Trying aggressive reconnect...", new_id);
            // 尝试连接最近的 3 个节点，哪怕稍微远一点
            std::vector<int> nearby = findNodesInRange2DInternal(curr_pos, config_.node_spacing * 3.0);
            for (int nid : nearby) {
                if (nid == new_id) continue;
                // 强制尝试连边，这里我们甚至可以考虑再次放宽条件
                tryAddEdgeInternal(new_id, nid); 
            }
        }
        refreshSubGridOwnershipInternal();
        ROS_INFO("HistoryGraph add node time: %f ms", (ros::Time::now().toSec() - t_start) * 1000.0);
        return new_id;
    } else {
        // Failed to add (e.g. unsafe), keep using nearest
        return nearest_id;
    }
  }
  // ROS_INFO("HistoryGraph update time: %f ms", (ros::Time::now().toSec() - t_start) * 1000.0);

  return -1;
}

std::vector<int> HistoryGraph::getAllNodeIds() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::vector<int> ids;
  ids.reserve(nodes_.size());
  for (const auto& kv : nodes_) {
    ids.push_back(kv.first);
  }
  return ids;
}

Eigen::Vector3d HistoryGraph::getHistoryNodePos(int id) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (nodes_.find(id) == nodes_.end()) return Eigen::Vector3d::Zero();
  return nodes_[id].position;
}

size_t HistoryGraph::getHistoryNodeCount() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return nodes_.size();
}

std::vector<int> HistoryGraph::getAssociatedSubGrids(int node_id) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (nodes_.find(node_id) == nodes_.end()) return {};
  const auto& set = nodes_[node_id].associated_subgrid_ids;
  return std::vector<int>(set.begin(), set.end());
}

void HistoryGraph::releaseSubGrid(int subgrid_id) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (subgrid_to_node_owner_.count(subgrid_id)) {
    int owner_id = subgrid_to_node_owner_[subgrid_id];
    nodes_[owner_id].associated_subgrid_ids.erase(subgrid_id);
    subgrid_to_node_owner_.erase(subgrid_id);
  }
}

void HistoryGraph::refreshSubGridOwnership() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  refreshSubGridOwnershipInternal();
}

void HistoryGraph::computeDijkstraToTargets(int start_node_id,
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

  std::unordered_set<int> remaining_targets = std::move(targets);
  remaining_targets.erase(start_node_id);

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

        remaining_targets.erase(v);
      }
    }
  }
}

double HistoryGraph::evaluateViewpointCost(const Eigen::Vector3d& curr_pos,
                                          int anchor_node_id,
                                          const Eigen::Vector3d& vp_pos,
                                          const DijkstraResult& cached_dijkstra,
                                          std::vector<Eigen::Vector3d>* out_path) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  double euclidean_dist = (curr_pos - vp_pos).norm();

  // 1. 近场快速通道 (< 5m)
  if (euclidean_dist < 5.0) {
    // A. 射线检测 [Fix]: strict collision check
    if (global_map_ && global_map_->isPathSafe(curr_pos.cast<float>(), vp_pos.cast<float>(), config_.uav_radius + 0.15)) {
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

  if (global_map_ && global_map_->isPathSafe(anchor_pos.cast<float>(), vp_pos.cast<float>(), config_.uav_radius + 0.15)) {
    exit_path = {anchor_pos, vp_pos};
  } else {
    // [Fixed] Relaxed A* for viewpoint evaluation (User Request)
    // "Just a simple A* path is enough, not necessarily safe obstacle-avoidance path"
    exit_path = runLocalAStarInternal(anchor_pos, vp_pos, true); 
    if (exit_path.empty()) return std::numeric_limits<double>::max();
  }

  double cost_exit = calculatePathLengthInternal(exit_path);
  if (out_path) *out_path = exit_path;

  return cost_graph + cost_exit;
}

bool HistoryGraph::reconstructPathFromCache(const DijkstraResult& cache,
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

int HistoryGraph::findNearestNode(const Eigen::Vector3d& pos) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return findNearestNode2DInternal(pos);
}

std::vector<Eigen::Vector3d> HistoryGraph::runLocalAStar(const Eigen::Vector3d& start,
                                                         const Eigen::Vector3d& end) {
  return runLocalAStarInternal(start, end, false);
}

std::vector<Eigen::Vector3d> HistoryGraph::shortcutPath(const Eigen::Vector3d& start,
                                                        const std::vector<Eigen::Vector3d>& path) {
  return shortcutPathInternal(start, path);
}

double HistoryGraph::calculatePathLength(const std::vector<Eigen::Vector3d>& path) {
  return calculatePathLengthInternal(path);
}

Eigen::Vector3d HistoryGraph::getNodePos(int id) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (nodes_.count(id)) return nodes_[id].position;
  return Eigen::Vector3d::Zero();
}

void HistoryGraph::computeDijkstraToAll(int start_node_id, std::unordered_map<int, double>& dist_map) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (nodes_.find(start_node_id) == nodes_.end()) return;
  std::unordered_map<int, int> p_node, p_edge;
  runDijkstraInternal(start_node_id, dist_map, p_node, p_edge);
}

// ==================== 内部无锁实现 ====================

void HistoryGraph::refreshSubGridOwnershipInternal() {
  if (!global_map_) return;

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

int HistoryGraph::addNodeInternal(const Eigen::Vector3d& pos, int prev_id) {
  // [Added] Safety Check: Strict safety check using UAV radius
  // User Requirement: "Use uav_radius + safety_margin for safety check"
  if (global_map_) {
      star_mvf::Position p(pos.x(), pos.y(), pos.z());
      float safe_dist = config_.uav_radius + config_.safety_margin;
      if (!global_map_->isSphereSafe(p, safe_dist)) {
          ROS_WARN_THROTTLE(1.0, "[HistGraph] Cannot add node at (%.2f, %.2f): Wall collision.", pos.x(), pos.y());
          return -1;
      }
  }

  int id = next_id_++;
  NavNode node;
  node.id = id;
  node.position = pos;
  nodes_[id] = node;

  registerToIndexInternal(pos, id);
  if (prev_id != -1) tryAddEdgeInternal(id, prev_id);
  return id;
}

void HistoryGraph::performLoopClosureInternal(int new_node_id) {
  const Eigen::Vector3d& pos = nodes_[new_node_id].position;
  std::vector<int> nearby = findNodesInRange2DInternal(pos, config_.node_spacing * 2);
  for (int nid : nearby) {
    if (nid == new_node_id || hasEdgeBetweenInternal(new_node_id, nid)) continue;
    tryAddEdgeInternal(new_node_id, nid);
  }
}

bool HistoryGraph::tryAddEdgeInternal(int id_a, int id_b) {
  auto pts = computeSimplifiedPathInternal(nodes_[id_a].position, nodes_[id_b].position);
  if (pts.empty()) return false;
  NavEdge e;
  e.id = static_cast<int>(edges_.size());
  e.node_a = id_a;
  e.node_b = id_b;
  e.waypoints = pts;
  e.distance = calculatePathLengthInternal(pts);
  edges_[e.id] = e;
  nodes_[id_a].edge_ids.push_back(e.id);
  nodes_[id_b].edge_ids.push_back(e.id);
  return true;
}

std::vector<Eigen::Vector3d> HistoryGraph::computeSimplifiedPathInternal(const Eigen::Vector3d& start,
                                                                        const Eigen::Vector3d& end) {
  // [Fix] Use isPathSafe instead of isLineFreeInflate for robust collision check
  // isLineFreeInflate relies on pre-computed inflation map which might be broken or desync.
  // isPathSafe checks raw voxels with dynamic margin.
  if (global_map_ && global_map_->isPathSafe(start.cast<float>(), end.cast<float>(), config_.uav_radius + 0.15f)) {
      return {start, end};
  }
  auto raw = runLocalAStarInternal(start, end, true);
  if (raw.size() < 2) return {};
  return simplifyPathInternal(raw);
}

std::vector<Eigen::Vector3d> HistoryGraph::simplifyPathInternal(const std::vector<Eigen::Vector3d>& path) {
  if (path.empty()) return {};
  return shortcutPathInternal(path.front(), path);
}

std::vector<Eigen::Vector3d> HistoryGraph::shortcutPathInternal(const Eigen::Vector3d& start,
                                                                const std::vector<Eigen::Vector3d>& path) {
  if (path.empty()) return {};
  std::vector<Eigen::Vector3d> optimized;
  optimized.push_back(start);
  int curr_idx = -1;
  while (true) {
    Eigen::Vector3d cur_pt = (curr_idx == -1) ? start : path[curr_idx];
    int best_next = -1;
    for (int i = static_cast<int>(path.size()) - 1; i > curr_idx; --i) {
      // [Fix] Use isPathSafe here too
      if (global_map_ && global_map_->isPathSafe(cur_pt.cast<float>(), path[i].cast<float>(), config_.uav_radius + 0.1f)) {
        best_next = i;
        break;
      }
    }
    if (best_next != -1) {
      optimized.push_back(path[best_next]);
      curr_idx = best_next;
      if (best_next == static_cast<int>(path.size()) - 1) break;
    } else {
      if (curr_idx + 1 < static_cast<int>(path.size())) {
        optimized.push_back(path[++curr_idx]);
      } else {
        break;
      }
    }
  }
  return optimized;
}

std::vector<Eigen::Vector3d> HistoryGraph::runLocalAStarInternal(const Eigen::Vector3d& start,
                                                                 const Eigen::Vector3d& end,
                                                                 bool relaxed) {
  if (!global_map_) return {};

  // [Fix] Use isPathSafe with radius check instead of point-check isLineFree
  // If relaxed, skip strict SAFE check (direct line check still good optimization though)
  if (!relaxed && global_map_->isPathSafe(start.cast<float>(), end.cast<float>(), config_.uav_radius + 0.05f)) {
    return {start, end};
  }
  // If relaxed, use smaller margin for direct check
  if (relaxed && global_map_->isPathSafe(start.cast<float>(), end.cast<float>(), 0.0f)) {
      return {start, end};
  }
  std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarCompare> open_list;
  std::unordered_map<VoxelIndex, double, VoxelIndexHash> g_scores;
  std::vector<AStarNode*> pool;

  VoxelIndex s_idx = global_map_->posToIndex(start.cast<float>()), e_idx = global_map_->posToIndex(end.cast<float>());
  double h_start = (end - start).norm();
  AStarNode* s_n = new AStarNode(s_idx, 0, h_start, nullptr);
  open_list.push(s_n);
  pool.push_back(s_n);
  g_scores[s_idx] = 0;

  std::vector<Eigen::Vector3d> path;
  int steps = 0;
  int max_steps = 3000;

  while (!open_list.empty()) {
    AStarNode* curr = open_list.top();
    open_list.pop();

    if (curr->index == e_idx) {
      path = reconstructPathInternal(curr);
      break;
    }

    if (++steps > max_steps) break;

    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        VoxelIndex n_idx = curr->index;
        n_idx.x += dx;
        n_idx.y += dy;
        // if (global_map_->isOccupiedInflate(n_idx)) continue;
        bool is_target_voxel = (n_idx == e_idx);
        
        // [Optimized] Relaxed check: use isOccupied (raw) vs isOccupiedInflate (safe)
        bool collision = false;
        if (relaxed) {
            collision = global_map_->isOccupied(n_idx); // Allow close to walls
        } else {
            collision = global_map_->isOccupiedInflate(n_idx); // Strict safety
        }
        
        if (!is_target_voxel && collision) 
        {
          continue;
        }

        // [Fix] Diagonal Safety Check: prevent cutting corners
        if (dx != 0 && dy != 0) {
            VoxelIndex idx1 = curr->index; idx1.x += dx;
            VoxelIndex idx2 = curr->index; idx2.y += dy;
            if (global_map_->isOccupiedInflate(idx1) || global_map_->isOccupiedInflate(idx2)) {
                continue;
            }
        }
        double step = (dx != 0 && dy != 0) ? 1.414 : 1.0;
        double tg = curr->g_score + step * resolution_;

        if (g_scores.find(n_idx) == g_scores.end() || tg < g_scores[n_idx]) {
          g_scores[n_idx] = tg;
          double h = (global_map_->indexToPos(n_idx).cast<double>() - end).norm();
          auto* new_node = new AStarNode(n_idx, tg, tg + h, curr);
          open_list.push(new_node);
          pool.push_back(new_node);
        }
      }
  }

  for (auto n : pool) delete n;
  return path;
}

void HistoryGraph::runDijkstraInternal(int target_id,
                                      std::unordered_map<int, double>& dists,
                                      std::unordered_map<int, int>& parent_node,
                                      std::unordered_map<int, int>& parent_edge) {
  auto cmp = [](std::pair<int, double> a, std::pair<int, double> b) { return a.second > b.second; };
  std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, decltype(cmp)> pq(cmp);

  for (auto const& kv : nodes_) dists[kv.first] = std::numeric_limits<double>::max();
  dists[target_id] = 0;
  pq.push({target_id, 0.0});

  while (!pq.empty()) {
    int u = pq.top().first;
    double d = pq.top().second;
    pq.pop();

    if (d > dists[u]) continue;

    for (int e_id : nodes_[u].edge_ids) {
      const NavEdge& e = edges_[e_id];
      int v = (e.node_a == u) ? e.node_b : e.node_a;
      if (dists[u] + e.distance < dists[v]) {
        dists[v] = dists[u] + e.distance;
        parent_node[v] = u;
        parent_edge[v] = e_id;
        pq.push({v, dists[v]});
      }
    }
  }
}

int HistoryGraph::findIdealAnchorNodeInternal(const Eigen::Vector3d& pos,
                                             const std::unordered_map<int, double>& dist_map) {
  std::vector<int> candidates = findNodesInRange2DInternal(pos, config_.node_spacing * 1.6);
  int best_id = -1;
  double min_total_cost = std::numeric_limits<double>::max();

  for (int id : candidates) {
    if (dist_map.find(id) == dist_map.end()) continue;
    auto local_path = runLocalAStarInternal(pos, nodes_[id].position);
    if (local_path.empty()) continue;
    double total_cost = calculatePathLengthInternal(local_path) + dist_map.at(id);
    if (total_cost < min_total_cost) {
      min_total_cost = total_cost;
      best_id = id;
    }
  }

  return best_id;
}

std::vector<int> HistoryGraph::findNodesInRange2DInternal(const Eigen::Vector3d& pos, double r) {
  int gx = static_cast<int>(std::floor(pos.x() / config_.node_spacing));
  int gy = static_cast<int>(std::floor(pos.y() / config_.node_spacing));
  int r_grid = static_cast<int>(std::ceil(r / config_.node_spacing));

  std::vector<int> res;
  for (int i = -r_grid; i <= r_grid; ++i)
    for (int j = -r_grid; j <= r_grid; ++j) {
      uint64_t key = packKey(gx + i, gy + j);
      if (node_grid_index_.count(key)) {
        for (int id : node_grid_index_[key]) {
          if ((nodes_[id].position - pos).norm() <= r) res.push_back(id);
        }
      }
    }

  return res;
}

int HistoryGraph::findNearestNode2DInternal(const Eigen::Vector3d& pos) {
  auto cands = findNodesInRange2DInternal(pos, config_.node_spacing * 2.0);
  int bid = -1;
  double md = std::numeric_limits<double>::max();

  for (int id : cands) {
    double d = (nodes_[id].position - pos).norm();
    if (d < md) {
      md = d;
      bid = id;
    }
  }

  return bid;
}

std::vector<int> HistoryGraph::findNodesInRange(const Eigen::Vector3d& pos, double range) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return findNodesInRange2DInternal(pos, range);
}

int HistoryGraph::findNearestSafeNode(const Eigen::Vector3d& pos, double range_limit) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto cands = findNodesInRange2DInternal(pos, range_limit);
    if (cands.empty()) return -1;

    // Sort by distance
    std::sort(cands.begin(), cands.end(), [&](int a, int b) {
        return (nodes_[a].position - pos).squaredNorm() < (nodes_[b].position - pos).squaredNorm();
    });

    for (int id : cands) {
        // Check safety (Strict margin to ensure Planner success)
        if (global_map_ && global_map_->isPathSafe(pos.cast<float>(), nodes_[id].position.cast<float>(), config_.uav_radius + 0.3f)) {
            return id;
        }
    }
    return -1;
}

std::vector<Eigen::Vector3d> HistoryGraph::reconstructPathInternal(AStarNode* end_node) {
  std::vector<Eigen::Vector3d> path;
  AStarNode* curr = end_node;
  while (curr) {
    path.push_back(global_map_->indexToPos(curr->index).cast<double>());
    curr = curr->parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

double HistoryGraph::calculatePathLengthInternal(const std::vector<Eigen::Vector3d>& path) {
  double len = 0;
  for (size_t i = 1; i < path.size(); ++i) len += (path[i] - path[i - 1]).norm();
  return len;
}

uint64_t HistoryGraph::packKey(int x, int y) {
  return (static_cast<uint64_t>(static_cast<int32_t>(x)) << 32) |
         static_cast<uint32_t>(static_cast<int32_t>(y));
}

void HistoryGraph::registerToIndexInternal(const Eigen::Vector3d& pos, int id) {
  int gx = static_cast<int>(std::floor(pos.x() / config_.node_spacing));
  int gy = static_cast<int>(std::floor(pos.y() / config_.node_spacing));
  node_grid_index_[packKey(gx, gy)].push_back(id);
}

bool HistoryGraph::hasEdgeBetweenInternal(int id_a, int id_b) {
  for (int eid : nodes_[id_a].edge_ids) {
    if (edges_[eid].node_a == id_b || edges_[eid].node_b == id_b) return true;
  }
  return false;
}

// ==================== [Visual] Visualization Helpers ====================
std::vector<NavEdge> HistoryGraph::getAllEdges() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::vector<NavEdge> res;
  res.reserve(edges_.size());
  for (const auto& kv : edges_) {
    res.push_back(kv.second);
  }
  return res;
}

std::vector<NavNode> HistoryGraph::getAllNodes() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::vector<NavNode> res;
  res.reserve(nodes_.size());
  for (const auto& kv : nodes_) {
    res.push_back(kv.second);
  }
  return res;
}

Eigen::Vector3d HistoryGraph::getSubGridCenter(int subgrid_id) {
  if (!global_map_) return Eigen::Vector3d::Zero();
  const auto& sg = global_map_->getSubGrid(subgrid_id);
  return sg.center.cast<double>();
}

}  // namespace rog_map
