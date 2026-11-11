#ifndef SEMANTIC_EXPLORATION_GRID_PATH_FINDER_H
#define SEMANTIC_EXPLORATION_GRID_PATH_FINDER_H

#include <Eigen/Dense>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "map/global_map.h"

namespace rog_map {

enum class UnknownSpacePolicy { ALLOW, BLOCK };

struct GridPathResult {
  bool success = false;
  double length = 0.0;
  std::vector<Eigen::Vector3d> points;
};

class GridPathFinder {
 public:
  void init(star_mvf::GlobalMap::Ptr global_map, double safety_margin,
            double min_flight_height = 0.0);
  GridPathResult search(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                        UnknownSpacePolicy unknown_policy, int max_expansions);
  bool validatePath(const std::vector<Eigen::Vector3d>& path,
                    UnknownSpacePolicy unknown_policy) const;
  std::vector<Eigen::Vector3d> shortcut(const std::vector<Eigen::Vector3d>& path,
                                        UnknownSpacePolicy unknown_policy) const;

 private:
  struct OpenNode {
    size_t address;
    double f;
    bool operator<(const OpenNode& other) const { return f > other.f; }
  };

  bool traversable(const star_mvf::VoxelIndex& index, UnknownSpacePolicy policy) const;
  bool segmentSafeWithMargin(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                             UnknownSpacePolicy policy, double margin) const;
  bool segmentSafe(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                   UnknownSpacePolicy policy) const;
  bool segmentStrict(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                     UnknownSpacePolicy policy) const;
  double pathLength(const std::vector<Eigen::Vector3d>& path) const;

  star_mvf::GlobalMap::Ptr global_map_;
  double safety_margin_ = 0.5;
  // Voxel z-index below which A* must not expand (cruise-altitude floor).
  // The start and goal neighborhoods are exempt so the vehicle can climb out
  // of a low state and low goals remain reachable.
  int64_t min_z_index_ = std::numeric_limits<int64_t>::min();
  uint32_t generation_ = 0;
  std::vector<uint32_t> visit_stamp_;
  std::vector<uint32_t> close_stamp_;
  std::vector<double> g_score_;
  std::vector<size_t> parent_;
};

}  // namespace rog_map

#endif
