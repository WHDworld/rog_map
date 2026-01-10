#ifndef ROG_MAP_DTG_LOWRES_MAP_H
#define ROG_MAP_DTG_LOWRES_MAP_H

#include <Eigen/Dense>
#include <cstdint>

#include <ros/ros.h>
#include <list>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "star_mvf/map/global_map.h"

namespace rog_map {
// 说明：这里不直接复用 star_mvf::VoxelState（位掩码设计），因为 LowResMap/DTG 只关心“几何三态”
// (unknown/free/occupied) + 越界(out)。为避免把 VISITED/TARGET/FRONTIER 等高层位混入几何判断，
// 低分辨率层使用独立的几何状态枚举。
enum class VoxelStateLite : uint8_t {
  Unknown = 0,
  Free = 1,
  Occupied = 2,
  Out = 3,
};

struct LRNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float path_g = 999999.0f;
  uint32_t root_id = 0;
  uint8_t parent_dir = 13;  // 0~26, 13 means self
  uint32_t children_dir = 0;
  uint16_t free_num = 0;
  uint16_t unknown_num = 0;
};

struct LRBlock {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<std::shared_ptr<LRNode>> local_grid;
  Eigen::Vector3i origin = Eigen::Vector3i::Zero();
  Eigen::Vector3i size = Eigen::Vector3i::Zero();
};

struct DtgLowresConfig {
  double resolution = 0.5;

  Eigen::Vector3d map_origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_min = Eigen::Vector3d(-50, -50, -2);
  Eigen::Vector3d map_max = Eigen::Vector3d(50, 50, 5);

  double max_search_time = 0.05;
  bool allow_unknown = true;

  Eigen::Vector3d local_half_size = Eigen::Vector3d(12.0, 12.0, 3.0);
  double occ_sample_resolution = 0.2;
};

class DtgLowresMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void init(const ros::NodeHandle& nh, const std::string& ns, star_mvf::GlobalMap::Ptr global_map);

  void reloadNodesBf(const Eigen::Vector3d& robot_pos);

  void getDists(const Eigen::Vector3d& origin,
                std::vector<Eigen::Vector3d>& targets,
                std::vector<double>& dist,
                std::vector<std::vector<Eigen::Vector3d>>& paths);

  // multi-source dijkstra for voronoi, EDEN-style
  void voronoiPartitionSearch(std::list<std::pair<uint32_t, Eigen::Vector3d>>& search_origins,
                              std::list<Eigen::Vector3i>& margins);

  void voronoiPathRetrieve(std::unordered_map<uint32_t, uint32_t>& idx_dict,
                           std::list<Eigen::Vector3i>& margins,
                           std::list<std::pair<uint32_t, uint32_t>>& hh_idx,
                           std::list<std::list<Eigen::Vector3d>>& paths,
                           std::list<double>& lengths);

  bool retrieveHPath(const Eigen::Vector3d& start,
                     std::list<Eigen::Vector3d>& path,
                     double& length,
                     bool reverse_order);

bool retrieveHPath(const Eigen::Vector3i& start_id3,
                   std::list<Eigen::Vector3d>& path,
                   double& length,
                   bool reverse_order);

  Eigen::Vector3d getStdPos(const Eigen::Vector3d& pos) const;

  const LRBlock* localBlock() const { return blocks_.empty() ? nullptr : blocks_[0].get(); }

private:
  star_mvf::GlobalMap::Ptr global_map_;
  DtgLowresConfig cfg_;

  std::vector<std::shared_ptr<LRBlock>> blocks_;
  Eigen::Vector3d node_scale_ = Eigen::Vector3d::Ones();
  Eigen::Vector3i voxel_num_ = Eigen::Vector3i::Zero();

  std::vector<Eigen::Vector3i> connect_diff_;

  inline bool insideMap(const Eigen::Vector3d& pos) const;
  inline bool insideMap(const Eigen::Vector3i& idx3) const;

  inline void posToId3(const Eigen::Vector3d& pos, Eigen::Vector3i& id3) const;
  inline Eigen::Vector3d id3ToPos(const Eigen::Vector3i& id3) const;

  inline bool id3ToIndex(const LRBlock& blk, const Eigen::Vector3i& id3, int& out_idx) const;

  VoxelStateLite getVoxelStateLite(const Eigen::Vector3d& pos) const;
  void computeNodeOccupancy(const Eigen::Vector3i& node_id3, LRNode& node) const;
};

}  // namespace rog_map

#endif
