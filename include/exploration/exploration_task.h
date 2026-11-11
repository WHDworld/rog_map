#ifndef SEMANTIC_EXPLORATION_EXPLORATION_TASK_H
#define SEMANTIC_EXPLORATION_EXPLORATION_TASK_H

#include <Eigen/Dense>
#include <cstdint>
#include <functional>
#include <vector>

namespace rog_map {

enum class ExplorationTaskType : uint8_t { FRONTIER = 0, SEMANTIC = 1 };

struct ExplorationTaskKey {
  ExplorationTaskType type = ExplorationTaskType::FRONTIER;
  int id = -1;

  bool operator==(const ExplorationTaskKey& other) const {
    return type == other.type && id == other.id;
  }
  bool operator!=(const ExplorationTaskKey& other) const { return !(*this == other); }
};

struct ExplorationTaskKeyHash {
  size_t operator()(const ExplorationTaskKey& key) const {
    return (std::hash<int>{}(key.id) << 1) ^ static_cast<size_t>(key.type);
  }
};

struct ExplorationViewpoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double view_score = 0.0;
};

struct ExplorationTask {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ExplorationTaskKey key;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double raw_gain = 0.0;
  double normalized_gain = 0.0;
  double semantic_value = 0.0;
  double utility = 0.0;
  std::vector<ExplorationViewpoint> viewpoints;
};

inline const char* taskTypeName(ExplorationTaskType type) {
  return type == ExplorationTaskType::SEMANTIC ? "semantic" : "frontier";
}

}  // namespace rog_map

#endif
