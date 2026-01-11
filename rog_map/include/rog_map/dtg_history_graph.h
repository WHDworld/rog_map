#ifndef ROG_MAP_DTG_HISTORY_GRAPH_H
#define ROG_MAP_DTG_HISTORY_GRAPH_H

#include <Eigen/Dense>
#include <cstdint>
#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "rog_map/dtg_lowres_map.h"
// NOTE: do NOT include history_graph.h here (circular dependency).
// Forward declarations for lightweight compatibility structs used by FSM.
namespace rog_map {
struct ViewpointNode;
struct PathResult;
}

namespace rog_map {

class ViewpointManager;



struct DtgSchNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  float g = 0.0f;
  float f = 0.0f;
  uint32_t id = 0;
  uint8_t vid = 0;
  uint8_t flag = 0;
  std::shared_ptr<DtgSchNode> parent;
};

struct HNode;
struct FNode;

template <typename HeadNode, typename TailNode>
struct DtgEdge {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double length = 2e5;
  double length_s = 1e5;
  uint8_t e_flag = 0;
  std::list<Eigen::Vector3d> path;
  std::shared_ptr<HeadNode> head;
  std::shared_ptr<TailNode> tail;
};

struct FNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint32_t fid = 0;      // SubGrid linear index
  uint8_t vid = 0;       // viewpoint id in this fid (we map from cluster_id)
  uint8_t f_flag = 0;
  std::shared_ptr<DtgSchNode> sch;
  std::shared_ptr<DtgEdge<HNode, FNode>> hf_edge;

  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  float yaw = 0.0f;
  float score = 0.0f;
};

struct HNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint32_t id = 0;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  uint8_t h_flag = 0;
  std::shared_ptr<DtgSchNode> sch;
  double last_maintain_t = 0.0;

  std::list<std::shared_ptr<DtgEdge<HNode, FNode>>> hf_edges;
  std::list<std::shared_ptr<DtgEdge<HNode, HNode>>> hh_edges;
};

struct DtgHistoryConfig {
  double h_thresh = 6.0;
  double sensor_range = 8.0;
  double lambda_e = 0.2;
  double lambda_a = 1.0;
  double v_max = 1.0;
  double a_max = 1.0;
  double yv_max = 1.0;
  bool allow_unknown = true;
};

class DtgHistoryGraph {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void init(const ros::NodeHandle& nh,
            const std::string& ns,
            star_mvf::GlobalMap::Ptr global_map,
            ViewpointManager* vp_mgr);

  void updateRobotPos(const Eigen::Vector3d& robot_pos);

  void syncViewpointsFromManager();

  void bfUpdate(const Eigen::Vector3d& robot_pos);

  bool findPathToViewpoint(const Eigen::Vector3f& start_pos, int viewpoint_id, PathResult& out);

  std::vector<ViewpointNode> getValidViewpoints() const;

  void markViewpointExplored(int viewpoint_id);
  void markViewpointInvalid(int viewpoint_id);

  void pinViewpoint(int viewpoint_id);
  void unpinViewpoint(int viewpoint_id);
  bool isViewpointPinned(int viewpoint_id) const;

  size_t getHNodeCount() const;
  size_t getFNodeCount() const;

  void visualize(visualization_msgs::MarkerArray& markers) const;

private:
  mutable std::mutex mutex_;

  // lowres + dtg state
  star_mvf::GlobalMap::Ptr global_map_;
  ViewpointManager* vp_mgr_ = nullptr;
  DtgLowresMap lrm_;
  DtgHistoryConfig cfg_;

  // H storage: id->node
  std::unordered_map<uint32_t, std::shared_ptr<HNode>> h_nodes_;
  std::vector<std::shared_ptr<HNode>> h_list_;  // stable iteration list
  uint32_t next_hid_ = 1;
  std::shared_ptr<HNode> root_;
  std::shared_ptr<HNode> active_h_node_;

  // F storage: viewpoint_id -> FNode
  std::unordered_map<int, std::shared_ptr<FNode>> f_nodes_by_vpid_;

  // viewpoint lifecycle (mirrors HistoryGraph public API)
  std::unordered_set<int> explored_vps_;
  std::unordered_set<int> invalid_vps_;
  std::unordered_set<int> pinned_vps_;

  // Thread safety and performance cache
  size_t last_h_count_ = 0;
  size_t last_f_count_ = 0;

  // helpers
  std::shared_ptr<HNode> createHnode(const Eigen::Vector3d& pos);
  void connectHF(const std::shared_ptr<HNode>& h, const std::shared_ptr<FNode>& f,
                 const std::list<Eigen::Vector3d>& path, double length);
  void connectHH(const std::shared_ptr<HNode>& h1, const std::shared_ptr<HNode>& h2,
                 const std::list<Eigen::Vector3d>& path, double length);

  double gainExp1(const Eigen::Vector4d& ps, const Eigen::Vector4d& vp, const Eigen::Vector4d& vs, double dist) const;
};

}  // namespace rog_map

#endif
