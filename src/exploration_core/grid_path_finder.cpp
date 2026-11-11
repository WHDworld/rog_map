#include "exploration/grid_path_finder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include <ros/ros.h>

namespace rog_map {

namespace {
// Low-space expansion is allowed only this close (3-D) to the start/goal so
// the vehicle can climb out of a low state and low goals stay reachable.
constexpr double kAltitudeExceptionRadius = 1.5;
}  // namespace

void GridPathFinder::init(star_mvf::GlobalMap::Ptr global_map, double safety_margin,
                          double min_flight_height) {
  global_map_ = std::move(global_map);
  safety_margin_ = safety_margin;
  min_z_index_ = std::numeric_limits<int64_t>::min();
  if (global_map_ && min_flight_height > 0.0) {
    min_z_index_ = global_map_
                       ->posToIndex(star_mvf::Position(
                           0.0f, 0.0f, static_cast<float>(min_flight_height)))
                       .z;
  }
  const size_t count = global_map_ ? global_map_->getTotalVoxelCount() : 0;
  visit_stamp_.assign(count, 0);
  close_stamp_.assign(count, 0);
  g_score_.assign(count, std::numeric_limits<double>::infinity());
  parent_.assign(count, std::numeric_limits<size_t>::max());
}

bool GridPathFinder::traversable(const star_mvf::VoxelIndex& index,
                                 UnknownSpacePolicy policy) const {
  if (!global_map_ || !global_map_->isInGlobalMapRange(index)) return false;
  if (global_map_->isOccupiedInflate(index)) return false;
  if (policy == UnknownSpacePolicy::BLOCK && global_map_->isUnknown(index)) return false;
  return true;
}

double GridPathFinder::pathLength(const std::vector<Eigen::Vector3d>& path) const {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) length += (path[i] - path[i - 1]).norm();
  return length;
}

bool GridPathFinder::segmentSafeWithMargin(const Eigen::Vector3d& from,
                                           const Eigen::Vector3d& to,
                                           UnknownSpacePolicy policy,
                                           double margin) const {
  if (!global_map_) return false;
  // The clearance margin applies to occupied voxels only. Unknown space is
  // checked strictly on the traversed line: frontier goals are inherently
  // adjacent to unknown space, and the LiDAR cannot observe directly below or
  // above the hovering robot, so a 3-D unknown margin would reject every path
  // in a partially explored map.
  if (!global_map_->isPathSafe(from.cast<float>(), to.cast<float>(),
                               static_cast<float>(margin), false)) {
    return false;
  }
  if (policy != UnknownSpacePolicy::BLOCK) return true;

  const double length = (to - from).norm();
  if (length < 1e-6) return !global_map_->isUnknown(from.cast<float>());
  const double step = 0.5 * static_cast<double>(global_map_->getResolution());
  const int samples = static_cast<int>(std::ceil(length / step));
  for (int i = 0; i <= samples; ++i) {
    const Eigen::Vector3d pos =
        from + (to - from) * (static_cast<double>(i) / static_cast<double>(samples));
    if (global_map_->isUnknown(pos.cast<float>())) return false;
  }
  return true;
}

bool GridPathFinder::segmentSafe(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                                 UnknownSpacePolicy policy) const {
  return segmentSafeWithMargin(from, to, policy, safety_margin_);
}

bool GridPathFinder::segmentStrict(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                                   UnknownSpacePolicy policy) const {
  // Final acceptance criterion: the traversed line must never touch an occupied
  // or unknown voxel. The configured margin is only a preference used by the
  // direct check and shortcut(); the A* path already keeps inflated-free nodes,
  // so requiring the full margin here would reject every forest path.
  // Check the precomputed inflated occupancy along the segment as well. This
  // catches diagonal corner cutting between individually traversable A* nodes,
  // without applying a 3-D inflation kernel to UNKNOWN voxels.
  if (!global_map_ || !global_map_->isLineFreeInflate(from.cast<float>(), to.cast<float>(), false)) {
    return false;
  }
  return segmentSafeWithMargin(from, to, policy, 0.0);
}

bool GridPathFinder::validatePath(const std::vector<Eigen::Vector3d>& path,
                                  UnknownSpacePolicy policy) const {
  if (!global_map_ || path.empty()) return false;
  for (size_t i = 1; i < path.size(); ++i) {
    if (segmentStrict(path[i - 1], path[i], policy)) continue;
    const bool occupied_safe =
        global_map_->isPathSafe(path[i - 1].cast<float>(), path[i].cast<float>(), 0.0f, false);
    ROS_WARN_THROTTLE(1.0,
                      "[SCHED][PATH] unsafe segment %zu/%zu a=(%.2f,%.2f,%.2f) "
                      "b=(%.2f,%.2f,%.2f) occupied_margin_safe=%d policy=%d",
                      i, path.size() - 1, path[i - 1].x(), path[i - 1].y(),
                      path[i - 1].z(), path[i].x(), path[i].y(), path[i].z(),
                      static_cast<int>(occupied_safe), static_cast<int>(policy));
    return false;
  }
  return true;
}

std::vector<Eigen::Vector3d> GridPathFinder::shortcut(
    const std::vector<Eigen::Vector3d>& path, UnknownSpacePolicy policy) const {
  if (path.size() < 3 || !global_map_) return path;
  std::vector<Eigen::Vector3d> result;
  result.reserve(path.size());
  result.push_back(path.front());
  size_t current = 0;
  while (current + 1 < path.size()) {
    size_t next = current + 1;
    for (size_t candidate = path.size() - 1; candidate > current; --candidate) {
      if (segmentSafe(path[current], path[candidate], policy)) {
        next = candidate;
        break;
      }
    }
    result.push_back(path[next]);
    current = next;
  }
  return result;
}

GridPathResult GridPathFinder::search(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                                      UnknownSpacePolicy policy, int max_expansions) {
  GridPathResult result;
  if (!global_map_ || max_expansions <= 0) return result;
  if (!global_map_->isInGlobalMapRange(start.cast<float>()) ||
      !global_map_->isInGlobalMapRange(goal.cast<float>())) {
    ROS_WARN_THROTTLE(1.0,
                      "[SCHED][PATH] out of map range start=(%.2f,%.2f,%.2f) "
                      "goal=(%.2f,%.2f,%.2f)",
                      start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());
    return result;
  }

  star_mvf::VoxelIndex start_idx = global_map_->posToIndex(start.cast<float>());
  star_mvf::VoxelIndex goal_idx = global_map_->posToIndex(goal.cast<float>());
  if (!traversable(start_idx, policy) || !traversable(goal_idx, policy)) {
    ROS_WARN_THROTTLE(1.0,
                      "[SCHED][PATH] endpoint blocked start=(%.2f,%.2f,%.2f) "
                      "start_unknown=%d start_inflated=%d goal=(%.2f,%.2f,%.2f) "
                      "goal_unknown=%d goal_inflated=%d policy=%d",
                      start.x(), start.y(), start.z(),
                      static_cast<int>(global_map_->isUnknown(start_idx)),
                      static_cast<int>(global_map_->isOccupiedInflate(start_idx)),
                      goal.x(), goal.y(), goal.z(),
                      static_cast<int>(global_map_->isUnknown(goal_idx)),
                      static_cast<int>(global_map_->isOccupiedInflate(goal_idx)),
                      static_cast<int>(policy));
    return result;
  }

  if (segmentSafe(start, goal, policy)) {
    result.success = true;
    result.points = {start, goal};
    result.length = (goal - start).norm();
    return result;
  }

  if (++generation_ == 0) {
    std::fill(visit_stamp_.begin(), visit_stamp_.end(), 0);
    std::fill(close_stamp_.begin(), close_stamp_.end(), 0);
    generation_ = 1;
  }

  size_t start_address = 0, goal_address = 0;
  if (!global_map_->voxelToLinearAddress(start_idx, start_address) ||
      !global_map_->voxelToLinearAddress(goal_idx, goal_address)) {
    ROS_WARN_THROTTLE(1.0, "[SCHED][PATH] linear address conversion failed");
    return result;
  }

  std::priority_queue<OpenNode> open;
  visit_stamp_[start_address] = generation_;
  g_score_[start_address] = 0.0;
  parent_[start_address] = std::numeric_limits<size_t>::max();
  open.push({start_address, (goal - start).norm()});

  int expansions = 0;
  bool reached = false;
  const double resolution = global_map_->getResolution();
  while (!open.empty() && expansions++ < max_expansions) {
    const size_t current_address = open.top().address;
    open.pop();
    if (close_stamp_[current_address] == generation_) continue;
    close_stamp_[current_address] = generation_;
    if (current_address == goal_address) {
      reached = true;
      break;
    }

    star_mvf::VoxelIndex current_idx;
    if (!global_map_->linearAddressToVoxel(current_address, current_idx)) continue;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          star_mvf::VoxelIndex next(current_idx.x + dx, current_idx.y + dy, current_idx.z + dz);
          if (next.z < min_z_index_) {
            const Eigen::Vector3d next_pos =
                global_map_->indexToPos(next).cast<double>();
            if ((next_pos - start).norm() > kAltitudeExceptionRadius &&
                (next_pos - goal).norm() > kAltitudeExceptionRadius) {
              continue;
            }
          }
          if (!traversable(next, policy)) continue;
          size_t next_address = 0;
          if (!global_map_->voxelToLinearAddress(next, next_address) ||
              close_stamp_[next_address] == generation_) continue;
          const double step = resolution * std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
          const double tentative = g_score_[current_address] + step;
          if (visit_stamp_[next_address] != generation_ || tentative < g_score_[next_address]) {
            visit_stamp_[next_address] = generation_;
            g_score_[next_address] = tentative;
            parent_[next_address] = current_address;
            const double heuristic =
                (global_map_->indexToPos(next).cast<double>() - goal).norm();
            open.push({next_address, tentative + heuristic});
          }
        }
  }
  if (!reached) {
    ROS_WARN_THROTTLE(1.0, "[SCHED][PATH] A* not reached expansions=%d open=%zu",
                      expansions, open.size());
    return result;
  }

  std::vector<Eigen::Vector3d> reverse;
  size_t address = goal_address;
  while (address != std::numeric_limits<size_t>::max()) {
    star_mvf::VoxelIndex index;
    if (!global_map_->linearAddressToVoxel(address, index)) return GridPathResult();
    reverse.push_back(global_map_->indexToPos(index).cast<double>());
    if (address == start_address) break;
    address = parent_[address];
  }
  if (reverse.empty() || address == std::numeric_limits<size_t>::max()) return result;
  std::reverse(reverse.begin(), reverse.end());
  reverse.front() = start;
  reverse.back() = goal;
  result.points = shortcut(reverse, policy);
  result.length = pathLength(result.points);
  result.success = validatePath(result.points, policy);
  if (!result.success) {
    ROS_WARN_THROTTLE(1.0, "[SCHED][PATH] validate failed points=%zu len=%.2f",
                      result.points.size(), result.length);
    result.points.clear();
  }
  return result;
}

}  // namespace rog_map
