#include "rog_map/dtg_history_graph.h"

#include <ros/ros.h>

#include "rog_map/viewpoint_manager.h"

namespace rog_map {
namespace dtg {

void DtgHistoryGraph::init(const ros::NodeHandle& nh,
                          const std::string& ns,
                          star_mvf::GlobalMap::Ptr global_map,
                          ViewpointManager* vp_mgr) {
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

  lrm_.init(nh, ns + "/lowres", global_map_);

  ROS_INFO("[DTG] init: h_thresh=%.2f, sensor_range=%.2f", cfg_.h_thresh, cfg_.sensor_range);
}

void DtgHistoryGraph::updateRobotPos(const Eigen::Vector3d& robot_pos) {
  bfUpdate(robot_pos);
}

void DtgHistoryGraph::syncViewpointsFromManager() {
  if (!vp_mgr_) return;

  // 当前设计：ViewpointManager 每次更新生成的一组 clusters 视为“本地有效视点”
  // 其中 has_viewpoint==true 的 cluster 会进入 DTG
  auto clusters = vp_mgr_->getClusters();

  // 这里先只维护一个 vpid->FNode 的字典，后续再做“按subgrid(f_id)分组”
  // 目前我们用 cluster.cluster_id 直接当 viewpoint_id，便于和现有 HistoryGraph 对齐。
  for (const auto& c : clusters) {
    if (!c.has_viewpoint) continue;

    int viewpoint_id = c.cluster_id;
    if (explored_vps_.count(viewpoint_id) || invalid_vps_.count(viewpoint_id)) continue;

    auto it = f_nodes_by_vpid_.find(viewpoint_id);
    if (it == f_nodes_by_vpid_.end()) {
      auto fn = std::make_shared<FNode>();
      fn->pos = c.viewpoint_position;
      fn->yaw = c.viewpoint_yaw;
      fn->score = c.viewpoint_score;

      // fid: SubGrid linear index (SubGrid对标EROI)
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

      if (global_map_) {
        star_mvf::SubGridIndex sg;
        global_map_->posToSubGrid(c.viewpoint_position, sg);
        it->second->fid = static_cast<uint32_t>(global_map_->subGridToLinearIndex(sg));
      } else {
        it->second->fid = 0;
      }
      it->second->vid = static_cast<uint8_t>(c.cluster_id & 0xFF);
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
  auto e = std::make_shared<DtgEdge<HNode, HNode>>();
  e->head = h1;
  e->tail = h2;
  e->path = path;
  e->length = length;

  h1->hh_edges.push_back(e);
  h2->hh_edges.push_back(e);
}

void DtgHistoryGraph::bfUpdate(const Eigen::Vector3d& robot_pos) {
  // 1) 同步本地视点
  syncViewpointsFromManager();

  // 2) 刷新低分辨率局部地图节点
  lrm_.reloadNodesBf(robot_pos);

  // 3) 创建/更新 root HNode
  if (!root_) {
    root_ = createHnode(robot_pos);
  }

  // TODO: 按EDEN实现：
  // - GetHnodesBBX
  // - CheckHHEdges
  // - VoronoiPartitionSearch
  // - if(dist_to_nearest_h > H_thresh) CreateHnode
  // - VoronoiPathRetrieve + ConnectHH
  // - CreateConnectFnode: RetrieveHPath(vp_pos) + ConnectHF
}

std::vector<ViewpointNode> DtgHistoryGraph::getValidViewpoints() const {
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
    vp.connected_history_node = -1;
    out.push_back(vp);
  }

  return out;
}

bool DtgHistoryGraph::findPathToViewpoint(const Eigen::Vector3f& start_pos, int viewpoint_id, PathResult& out) {
  out = PathResult();
  out.target_viewpoint_id = viewpoint_id;

  auto it = f_nodes_by_vpid_.find(viewpoint_id);
  if (it == f_nodes_by_vpid_.end() || !it->second) {
    return false;
  }

  const auto& f = it->second;

  // 使用低分辨率Dijkstra路径（允许unknown由dtg/lowres/allow_unknown控制）
  std::vector<Eigen::Vector3d> targets;
  targets.emplace_back(f->pos.cast<double>());
  std::vector<double> dists;
  std::vector<std::vector<Eigen::Vector3d>> paths;

  Eigen::Vector3d origin = start_pos.cast<double>();
  lrm_.getDists(origin, targets, dists, paths);

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
  explored_vps_.insert(viewpoint_id);
}

void DtgHistoryGraph::markViewpointInvalid(int viewpoint_id) {
  invalid_vps_.insert(viewpoint_id);
}

size_t DtgHistoryGraph::getHNodeCount() const {
  return h_list_.size();
}

size_t DtgHistoryGraph::getFNodeCount() const {
  return f_nodes_by_vpid_.size();
}

double DtgHistoryGraph::gainExp1(const Eigen::Vector4d& ps,
                                const Eigen::Vector4d& vp,
                                const Eigen::Vector4d& vs,
                                double dist) const {
  (void)vs;
  double t = dist / std::max(1e-3, cfg_.v_max);
  return std::exp(-cfg_.lambda_e * t);
}

}  // namespace dtg
}  // namespace rog_map
