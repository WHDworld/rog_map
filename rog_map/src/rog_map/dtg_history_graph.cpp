#include "rog_map/dtg_history_graph.h"

#include <ros/ros.h>

#include "rog_map/history_graph.h"
#include "rog_map/viewpoint_manager.h"

namespace rog_map {

void DtgHistoryGraph::pinViewpoint(int viewpoint_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  pinned_vps_.insert(viewpoint_id);
}

void DtgHistoryGraph::unpinViewpoint(int viewpoint_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  pinned_vps_.erase(viewpoint_id);
}

bool DtgHistoryGraph::isViewpointPinned(int viewpoint_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pinned_vps_.count(viewpoint_id) > 0;
}


void DtgHistoryGraph::init(const ros::NodeHandle& nh,
                          const std::string& ns,
                          star_mvf::GlobalMap::Ptr global_map,
                          ViewpointManager* vp_mgr) {
  std::lock_guard<std::mutex> lock(mutex_);
  global_map_ = global_map;
  vp_mgr_ = vp_mgr;

  nh.param(ns + "/h_thresh", cfg_.h_thresh, cfg_.h_thresh);
  nh.param(ns + "/sensor_range", cfg_.sensor_range, cfg_.sensor_range);
  nh.param(ns + "/lambda_e", cfg_.lambda_e, cfg_.lambda_e);
  nh.param(ns + "/lambda_a", cfg_.lambda_a, cfg_.lambda_a);
  nh.param(ns + "/v_max", cfg_.v_max, cfg_.v_max);
  nh.param(ns + "/a_max", cfg_.a_max, cfg_.a_max);
  nh.param(ns + "/yv_max", cfg_.yv_max, cfg_.yv_max);
  nh.param(ns + "/allow_unknown", cfg_.allow_unknown, cfg_.allow_unknown);

  lrm_.init(nh, "rog_map/lowres", global_map_);

  ROS_INFO("[DTG] init: h_thresh=%.2f, sensor_range=%.2f", cfg_.h_thresh, cfg_.sensor_range);
}

void DtgHistoryGraph::updateRobotPos(const Eigen::Vector3d& robot_pos) {
  std::lock_guard<std::mutex> lock(mutex_);
  bfUpdate(robot_pos);
}

void DtgHistoryGraph::syncViewpointsFromManager() {
  // Caller must already hold mutex_ (updateRobotPos/bfUpdate).
  if (!vp_mgr_) return;

  auto clusters = vp_mgr_->getClusters();
  std::unordered_set<int> current_vp_ids;

  for (const auto& c : clusters) {
    if (!c.has_viewpoint) continue;

    int viewpoint_id = c.cluster_id;
    current_vp_ids.insert(viewpoint_id);

    if (explored_vps_.count(viewpoint_id) || invalid_vps_.count(viewpoint_id)) continue;

    auto it = f_nodes_by_vpid_.find(viewpoint_id);
    if (it == f_nodes_by_vpid_.end()) {
      auto fn = std::make_shared<FNode>();
      fn->pos = c.viewpoint_position;
      fn->yaw = c.viewpoint_yaw;
      fn->score = c.viewpoint_score;

      if (global_map_) {
        star_mvf::SubGridIndex sg;
        global_map_->posToSubGrid(c.viewpoint_position, sg);
        fn->fid = static_cast<uint32_t>(global_map_->subGridToLinearIndex(sg));
      } else {
        fn->fid = 0;
      }
      fn->vid = static_cast<uint8_t>(c.cluster_id & 0xFF);

      f_nodes_by_vpid_[viewpoint_id] = fn;
    } else {
      it->second->pos = c.viewpoint_position;
      it->second->yaw = c.viewpoint_yaw;
      it->second->score = c.viewpoint_score;
    }
  }

  // Clean up stale F-nodes (scheme 2): keep only (pinned + current)
  for (auto it = f_nodes_by_vpid_.begin(); it != f_nodes_by_vpid_.end(); ) {
    const int vpid = it->first;

    const bool keep = (pinned_vps_.count(vpid) > 0) || (current_vp_ids.count(vpid) > 0);
    if (!keep) {
      // Remove HF edge from its head H-node to avoid stale edges
      if (it->second && it->second->hf_edge && it->second->hf_edge->head) {
        it->second->hf_edge->head->hf_edges.remove(it->second->hf_edge);
        it->second->hf_edge.reset();
      }
      it = f_nodes_by_vpid_.erase(it);
    } else {
      ++it;
    }
  }
}

std::shared_ptr<HNode> DtgHistoryGraph::createHnode(const Eigen::Vector3d& pos) {
  auto h = std::make_shared<HNode>();
  h->id = next_hid_++;
  h->pos = lrm_.getStdPos(pos);
  h_nodes_[h->id] = h;
  h_list_.push_back(h);
  return h;
}

void DtgHistoryGraph::connectHF(const std::shared_ptr<HNode>& h,
                               const std::shared_ptr<FNode>& f,
                               const std::list<Eigen::Vector3d>& path,
                               double length) {
  if (f->hf_edge) {
      if (f->hf_edge->head == h) { // Same edge, update it
          f->hf_edge->path = path;
          f->hf_edge->length = length;
          return;
      }
      // Belongs to another H-node, remove it from old H-node
      auto& old_h_edges = f->hf_edge->head->hf_edges;
      old_h_edges.remove(f->hf_edge);
  }

  auto e = std::make_shared<DtgEdge<HNode, FNode>>();
  e->head = h;
  e->tail = f;
  e->path = path;
  e->length = length;

  f->hf_edge = e;
  h->hf_edges.push_back(e);
}

void DtgHistoryGraph::connectHH(const std::shared_ptr<HNode>& h1,
                               const std::shared_ptr<HNode>& h2,
                               const std::list<Eigen::Vector3d>& path,
                               double length) {
  // Avoid duplicate edges
  for (const auto& edge : h1->hh_edges) {
      if (edge->tail == h2) {
          edge->path = path;
          edge->length = length;
          return;
      }
  }

  auto e = std::make_shared<DtgEdge<HNode, HNode>>();
  e->head = h1;
  e->tail = h2;
  e->path = path;
  e->length = length;

  h1->hh_edges.push_back(e);
  h2->hh_edges.push_back(e);
}

void DtgHistoryGraph::bfUpdate(const Eigen::Vector3d& robot_pos) {
  syncViewpointsFromManager();
  lrm_.reloadNodesBf(robot_pos);

  if (!root_) {
    root_ = createHnode(robot_pos);
    active_h_node_ = root_;
  }

  // Find nearest H-node
  std::shared_ptr<HNode> nearest_h = root_;
  double min_dist_to_h = (root_->pos - robot_pos).norm();
  for (const auto& h : h_list_) {
    double d = (h->pos - robot_pos).norm();
    if (d < min_dist_to_h) {
      min_dist_to_h = d;
      nearest_h = h;
    }
  }

  // Create a new H-node if needed and set it as active
  if (min_dist_to_h > cfg_.h_thresh) {
    active_h_node_ = createHnode(robot_pos);
  } else {
    active_h_node_ = nearest_h;
  }

  std::list<std::pair<uint32_t, Eigen::Vector3d>> search_origins;
  std::unordered_map<uint32_t, uint32_t> h_id_to_idx;
  std::vector<std::shared_ptr<HNode>> h_nodes_vec;
  for(const auto& h : h_list_){
      search_origins.push_back({h->id, h->pos});
      h_id_to_idx[h->id] = h_nodes_vec.size();
      h_nodes_vec.push_back(h);
  }

  std::list<Eigen::Vector3i> margins;
  lrm_.voronoiPartitionSearch(search_origins, margins);

  std::list<std::pair<uint32_t, uint32_t>> hh_indices;
  std::list<std::list<Eigen::Vector3d>> paths;
  std::list<double> lengths;
  lrm_.voronoiPathRetrieve(h_id_to_idx, margins, hh_indices, paths, lengths);

  auto path_it = paths.begin();
  auto len_it = lengths.begin();
  for(const auto& hh_idx : hh_indices){
      connectHH(h_nodes_vec[hh_idx.first], h_nodes_vec[hh_idx.second], *path_it, *len_it);
      path_it++;
      len_it++;
  }

  std::vector<Eigen::Vector3d> f_positions;
  std::vector<std::shared_ptr<FNode>> f_nodes_to_connect;
  for (const auto& pair : f_nodes_by_vpid_) {
      f_positions.push_back(pair.second->pos.cast<double>());
      f_nodes_to_connect.push_back(pair.second);
  }

  if (!f_positions.empty()) {
      std::vector<double> dists;
      std::vector<std::vector<Eigen::Vector3d>> hf_paths;
      const auto& h_start = active_h_node_ ? active_h_node_ : root_;
      lrm_.getDists(h_start->pos, f_positions, dists, hf_paths);

      for (size_t i = 0; i < f_nodes_to_connect.size(); ++i) {
          if (dists[i] < 1e9) {
              connectHF(h_start, f_nodes_to_connect[i], std::list<Eigen::Vector3d>(hf_paths[i].begin(), hf_paths[i].end()), dists[i]);
          }
      }
  }
}

std::vector<ViewpointNode> DtgHistoryGraph::getValidViewpoints() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<ViewpointNode> out;
  out.reserve(f_nodes_by_vpid_.size());

  for (const auto& kv : f_nodes_by_vpid_) {
    int vpid = kv.first;
    const auto& f = kv.second;
    if (!f) continue;
    if (explored_vps_.count(vpid) || invalid_vps_.count(vpid)) continue;

    ViewpointNode vp;
    vp.viewpoint_id = vpid;
    vp.position = f->pos;
    vp.yaw = f->yaw;
    vp.cluster_id = static_cast<int>(f->fid);
    vp.score = f->score;
    vp.explored = false;
    vp.valid = true;
    vp.connected_history_node = (f->hf_edge && f->hf_edge->head) ? static_cast<int>(f->hf_edge->head->id) : -1;
    out.push_back(vp);
  }

  return out;
}

bool DtgHistoryGraph::findPathToViewpoint(const Eigen::Vector3f& start_pos, int viewpoint_id, PathResult& out) {
  out = PathResult();
  out.target_viewpoint_id = viewpoint_id;

  Eigen::Vector3d target;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = f_nodes_by_vpid_.find(viewpoint_id);
    if (it == f_nodes_by_vpid_.end() || !it->second) {
      return false;
    }
    target = it->second->pos.cast<double>();
  }

  std::vector<Eigen::Vector3d> targets = {target};
  std::vector<double> dists;
  std::vector<std::vector<Eigen::Vector3d>> paths;

  lrm_.getDists(start_pos.cast<double>(), targets, dists, paths);

  if (dists.empty() || paths.empty() || dists[0] > 9e5 || paths[0].empty()) {
    return false;
  }

  out.success = true;
  out.total_distance = static_cast<float>(dists[0]);
  out.waypoints.clear();
  out.waypoints.reserve(paths[0].size());
  for (const auto& p : paths[0]) {
    out.waypoints.push_back(p.cast<float>());
  }

  return true;
}

void DtgHistoryGraph::markViewpointExplored(int viewpoint_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  explored_vps_.insert(viewpoint_id);
  pinned_vps_.erase(viewpoint_id);

  auto it = f_nodes_by_vpid_.find(viewpoint_id);
  if (it != f_nodes_by_vpid_.end()) {
    if (it->second && it->second->hf_edge && it->second->hf_edge->head) {
      it->second->hf_edge->head->hf_edges.remove(it->second->hf_edge);
      it->second->hf_edge.reset();
    }
    f_nodes_by_vpid_.erase(it);
  }
}

void DtgHistoryGraph::markViewpointInvalid(int viewpoint_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  invalid_vps_.insert(viewpoint_id);
  pinned_vps_.erase(viewpoint_id);

  auto it = f_nodes_by_vpid_.find(viewpoint_id);
  if (it != f_nodes_by_vpid_.end()) {
    if (it->second && it->second->hf_edge && it->second->hf_edge->head) {
      it->second->hf_edge->head->hf_edges.remove(it->second->hf_edge);
      it->second->hf_edge.reset();
    }
    f_nodes_by_vpid_.erase(it);
  }
}

size_t DtgHistoryGraph::getHNodeCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return h_list_.size();
}

size_t DtgHistoryGraph::getFNodeCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return f_nodes_by_vpid_.size();
}

void DtgHistoryGraph::visualize(visualization_msgs::MarkerArray& markers) const {
    std::lock_guard<std::mutex> lock(mutex_);

    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);

    int id = 0;

    for (const auto& h_pair : h_nodes_) {
        const auto& h = h_pair.second;
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "h_nodes";
        m.id = id++;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = h->pos.x();
        m.pose.position.y = h->pos.y();
        m.pose.position.z = h->pos.z();
        m.scale.x = 0.4; m.scale.y = 0.4; m.scale.z = 0.4;
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
        markers.markers.push_back(m);
    }

    for (const auto& f_pair : f_nodes_by_vpid_) {
        const auto& f = f_pair.second;
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "f_nodes";
        m.id = id++;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = f->pos.x();
        m.pose.position.y = f->pos.y();
        m.pose.position.z = f->pos.z();
        m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2;
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;
        markers.markers.push_back(m);
    }

    visualization_msgs::Marker hf_edges;
    hf_edges.header.frame_id = "map";
    hf_edges.header.stamp = ros::Time::now();
    hf_edges.ns = "hf_edges";
    hf_edges.id = id++;
    hf_edges.type = visualization_msgs::Marker::LINE_LIST;
    hf_edges.action = visualization_msgs::Marker::ADD;
    hf_edges.scale.x = 0.05;
    hf_edges.color.r = 0.0; hf_edges.color.g = 1.0; hf_edges.color.b = 1.0; hf_edges.color.a = 0.6;

    for (const auto& h_pair : h_nodes_) {
        for (const auto& edge : h_pair.second->hf_edges) {
            geometry_msgs::Point p1, p2;
            p1.x = edge->head->pos.x(); p1.y = edge->head->pos.y(); p1.z = edge->head->pos.z();
            p2.x = edge->tail->pos.x(); p2.y = edge->tail->pos.y(); p2.z = edge->tail->pos.z();
            hf_edges.points.push_back(p1);
            hf_edges.points.push_back(p2);
        }
    }
    if (!hf_edges.points.empty()) markers.markers.push_back(hf_edges);

    visualization_msgs::Marker hh_edges;
    hh_edges.header.frame_id = "map";
    hh_edges.header.stamp = ros::Time::now();
    hh_edges.ns = "hh_edges";
    hh_edges.id = id++;
    hh_edges.type = visualization_msgs::Marker::LINE_LIST;
    hh_edges.action = visualization_msgs::Marker::ADD;
    hh_edges.scale.x = 0.1;
    hh_edges.color.r = 1.0; hh_edges.color.g = 1.0; hh_edges.color.b = 0.0; hh_edges.color.a = 0.6;

    for (const auto& h_pair : h_nodes_) {
        for (const auto& edge : h_pair.second->hh_edges) {
            if (edge->head->id < edge->tail->id) { // Avoid duplicates
                geometry_msgs::Point p1, p2;
                p1.x = edge->head->pos.x(); p1.y = edge->head->pos.y(); p1.z = edge->head->pos.z();
                p2.x = edge->tail->pos.x(); p2.y = edge->tail->pos.y(); p2.z = edge->tail->pos.z();
                hh_edges.points.push_back(p1);
                hh_edges.points.push_back(p2);
            }
        }
    }
    if (!hh_edges.points.empty()) markers.markers.push_back(hh_edges);
}

}  // namespace rog_map
