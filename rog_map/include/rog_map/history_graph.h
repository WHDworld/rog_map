#ifndef ROG_MAP_HISTORY_GRAPH_H
#define ROG_MAP_HISTORY_GRAPH_H

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <ros/ros.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "star_mvf/map/global_map.h"

namespace rog_map {
using GlobalMap = star_mvf::GlobalMap;
using VoxelIndex = star_mvf::VoxelIndex;
// ==================== 数据结构 ====================

struct NavNode {
  int id;
  Eigen::Vector3d position;
  std::vector<int> edge_ids;
  std::unordered_set<int> associated_subgrid_ids;
};

struct NavEdge {
  int id;
  int node_a, node_b;
  std::vector<Eigen::Vector3d> waypoints;
  double distance;
};

struct DijkstraResult {
  std::unordered_map<int, double> dists;
  std::unordered_map<int, int> parents;
  int start_node_id = -1;
};

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

struct HistoryGraphConfig {
  float update_rate = 2.0f;
  float node_spacing = 5.0f;
  float max_assoc_dist = 10.0f;
  float takeoff_height = 1.0;
  float uav_radius = 0.3f; // [New] for collision check
  float safety_margin = 0.25f; // [New] for strict safety
};

// ==================== HistoryGraph 类 ====================
class HistoryGraph {
 public:
  using Ptr = std::shared_ptr<HistoryGraph>;

  Eigen::Vector3d current_position;
  bool has_uav_state_ = false;
  ros::NodeHandle nh_;
  ros::Timer timer_;

  HistoryGraph();

  void init(ros::NodeHandle& nh, GlobalMap::Ptr global_map);
  void start();
  void timerCallback(const ros::TimerEvent& e);

  void updateUAVState(const Eigen::Vector3d& position);

  int update(const Eigen::Vector3d& curr_pos);

  std::vector<int> getAllNodeIds() const;
  size_t getHistoryNodeCount() const;
  Eigen::Vector3d getHistoryNodePos(int id);
  
  std::vector<int> getAssociatedSubGrids(int node_id);
  void releaseSubGrid(int subgrid_id);
  void refreshSubGridOwnership();
  
  // [Visual] Visualization Helpers
  std::vector<NavEdge> getAllEdges() const;
  std::vector<NavNode> getAllNodes() const; // Return copy for safety
  Eigen::Vector3d getSubGridCenter(int subgrid_id);

  void computeDijkstraToTargets(int start_node_id,
                               std::unordered_set<int> targets,
                               DijkstraResult& result);
                               
  int findNearestSafeNode(const Eigen::Vector3d& pos, double range_limit);
  int findNearestNode(const Eigen::Vector3d& pos);
  
  std::vector<int> findNodesInRange(const Eigen::Vector3d& pos, double range);
  std::vector<Eigen::Vector3d> runLocalAStar(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

  double evaluateViewpointCost(const Eigen::Vector3d& curr_pos,
                               int anchor_node_id,
                               const Eigen::Vector3d& vp_pos,
                               const DijkstraResult& cached_dijkstra,
                               std::vector<Eigen::Vector3d>* out_path);

  bool reconstructPathFromCache(const DijkstraResult& cache,
                                int end_node_id,
                                std::vector<Eigen::Vector3d>& out_path);

  std::vector<Eigen::Vector3d> shortcutPath(const Eigen::Vector3d& start, const std::vector<Eigen::Vector3d>& path);
  double calculatePathLength(const std::vector<Eigen::Vector3d>& path);
  Eigen::Vector3d getNodePos(int id);

  void computeDijkstraToAll(int start_node_id, std::unordered_map<int, double>& dist_map);

 private:
  GlobalMap::Ptr global_map_;
  HistoryGraphConfig config_;

  float resolution_ = 0.0f;
  int next_id_ = 0;

  std::unordered_map<int, NavNode> nodes_;
  std::unordered_map<int, NavEdge> edges_;
  std::unordered_map<uint64_t, std::vector<int>> node_grid_index_;
  std::unordered_map<int, int> subgrid_to_node_owner_;

  mutable std::mutex data_mutex_;

  void refreshSubGridOwnershipInternal();
  int addNodeInternal(const Eigen::Vector3d& pos, int prev_id);
  void performLoopClosureInternal(int new_node_id);
  bool tryAddEdgeInternal(int id_a, int id_b);

  std::vector<Eigen::Vector3d> computeSimplifiedPathInternal(const Eigen::Vector3d& start,
                                                             const Eigen::Vector3d& end);
  std::vector<Eigen::Vector3d> simplifyPathInternal(const std::vector<Eigen::Vector3d>& path);
  std::vector<Eigen::Vector3d> shortcutPathInternal(const Eigen::Vector3d& start,
                                                    const std::vector<Eigen::Vector3d>& path);
  std::vector<Eigen::Vector3d> runLocalAStarInternal(const Eigen::Vector3d& start, const Eigen::Vector3d& end, bool relaxed = false);

  void runDijkstraInternal(int target_id,
                           std::unordered_map<int, double>& dists,
                           std::unordered_map<int, int>& parent_node,
                           std::unordered_map<int, int>& parent_edge);

  int findIdealAnchorNodeInternal(const Eigen::Vector3d& pos,
                                  const std::unordered_map<int, double>& dist_map);

  std::vector<int> findNodesInRange2DInternal(const Eigen::Vector3d& pos, double r);
  int findNearestNode2DInternal(const Eigen::Vector3d& pos);

  std::vector<Eigen::Vector3d> reconstructPathInternal(AStarNode* end_node);
  double calculatePathLengthInternal(const std::vector<Eigen::Vector3d>& path);

  uint64_t packKey(int x, int y);
  void registerToIndexInternal(const Eigen::Vector3d& pos, int id);
  bool hasEdgeBetweenInternal(int id_a, int id_b);
};

}  // namespace rog_map

#endif
