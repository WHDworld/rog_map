/**
 * @file test_global_scheduler.cpp
 * @brief FUEL 全局调度迁移的单元测试
 *
 * 覆盖：
 * 1. 地图索引/线性地址一致性
 * 2. GridPathFinder 的未知空间策略、障碍绕行与路径校验
 * 3. FUEL 运动时间/效用归一化/首跳代价矩阵
 * 4. LKH ATSP 求解与回退（最近邻 + 有向 2-opt）
 * 5. 局部视点精化（分层 DP、半径截断、首层不可用）
 * 6. 端到端 GlobalTourPlanner::plan
 * 7. 20k 点前沿聚类 P95 延迟
 *
 * 注意：需要运行中的 ROS master（roscore），否则所有用例会被 GTEST_SKIP。
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "exploration/exploration_task.h"
#include "exploration/global_tour_planner.h"
#include "exploration/grid_path_finder.h"
#include "exploration/viewpoint_manager.h"
#include "ego_planner/yaw_optimizer.h"
#include "map/global_map.h"

namespace rog_map {
namespace {

using star_mvf::Position;
using star_mvf::VoxelIndex;

constexpr double kPi = 3.14159265358979323846;

GlobalMap::Ptr makeTestMap(const std::string& ns) {
  ros::NodeHandle nh("~");
  nh.setParam(ns + "/resolution", 0.2);
  nh.setParam(ns + "/global_map/map_size",
              std::vector<float>{20.0f, 20.0f, 3.0f});
  nh.setParam(ns + "/global_map/map_model", 0);
  nh.setParam(ns + "/global_map/ground_height", -0.1f);
  nh.setParam(ns + "/global_map/sub_grid_xy", 0.25f);
  nh.setParam(ns + "/inflation/enable", false);
  nh.setParam(ns + "/sensor/lidar_downsample_filter", 1);
  nh.setParam(ns + "/local_map/raycast_step", 0.1);
  return std::make_shared<GlobalMap>(nh, ns);
}

sensor_msgs::PointCloud2 makePointCloud(const std::vector<Position>& points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const auto& point : points) {
    cloud.emplace_back(point.x(), point.y(), point.z());
  }
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  return msg;
}

// x = 0 处的实体墙，y in [-2, 2], z in [0, 2]。
// 单帧命中不足以超过占据阈值，因此重复插入若干帧。
void buildWall(GlobalMap& map, const Position& sensor) {
  std::vector<Position> points;
  for (float y = -2.0f; y <= 2.0f + 1e-3f; y += 0.2f) {
    for (float z = 0.0f; z <= 2.0f + 1e-3f; z += 0.2f) {
      points.emplace_back(0.0f, y, z);
    }
  }
  const sensor_msgs::PointCloud2 cloud = makePointCloud(points);
  for (int i = 0; i < 6; ++i) {
    map.insertPointCloud(cloud, sensor);
  }
}

ExplorationTask makeTask(ExplorationTaskType type, int id,
                         const Eigen::Vector3d& centroid, double raw_gain,
                         std::vector<ExplorationViewpoint> viewpoints) {
  ExplorationTask task;
  task.key = ExplorationTaskKey{type, id};
  task.centroid = centroid;
  task.raw_gain = raw_gain;
  task.viewpoints = std::move(viewpoints);
  return task;
}

}  // namespace

// 所有用例都依赖 ROS 参数服务器（GlobalMap / GlobalTourPlanner 从 ~ 读取参数）
class RosMasterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!ros::master::check()) {
      GTEST_SKIP() << "ROS master (roscore) is required for parameter loading";
    }
  }
};

TEST(YawOptimizerTest, PreservesCompleteStartBoundaryState) {
  Eigen::Matrix<double, 3, 3> head;
  Eigen::Matrix<double, 3, 3> tail;
  head << Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  tail << Eigen::Vector3d(4.0, 1.0, 0.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

  poly_traj::MinJerkOpt min_jerk;
  min_jerk.reset(head, tail, 1);
  min_jerk.generate(Eigen::MatrixXd(), Eigen::VectorXd::Constant(1, 4.0));

  constexpr double yaw = 0.4;
  constexpr double yaw_rate = 0.25;
  constexpr double yaw_acc = -0.12;
  std::vector<double> control_points;
  ego_planner::YawOptimizer optimizer;
  ASSERT_TRUE(optimizer.optimize(min_jerk.getTraj(), yaw, 1.0, control_points,
                                 0.05, yaw_rate, yaw_acc));
  ASSERT_GE(control_points.size(), 4u);
  EXPECT_NEAR(ego_planner::YawOptimizer::evaluateYaw(control_points, 0.0, 4.0),
              yaw, 2e-3);
  EXPECT_NEAR(ego_planner::YawOptimizer::evaluateYawDot(control_points, 0.0, 4.0),
              yaw_rate, 2e-3);
  EXPECT_NEAR(ego_planner::YawOptimizer::evaluateYawDDot(control_points, 0.0, 4.0),
              yaw_acc, 2e-2);
}

// ==================== 地图一致性 ====================

class MapConsistencyTest : public RosMasterTest {};

TEST_F(MapConsistencyTest, LinearAddressRoundTrip) {
  const GlobalMap::Ptr map = makeTestMap("map_consistency_linear");
  const size_t total = map->getTotalVoxelCount();
  ASSERT_GT(total, 0u);

  // 地图 z 下界由 ground_height 决定（-0.1），不能硬编码为对称范围。
  const Position inset(0.05f, 0.05f, 0.05f);
  const VoxelIndex min_idx =
      map->posToIndex(map->getGlobalMin() + inset);
  const VoxelIndex max_idx =
      map->posToIndex(map->getGlobalMax() - inset);
  size_t checked = 0;
  for (int64_t x = min_idx.x; x <= max_idx.x; x += 7) {
    for (int64_t y = min_idx.y; y <= max_idx.y; y += 11) {
      for (int64_t z = min_idx.z; z <= max_idx.z; z += 3) {
        const VoxelIndex index(x, y, z);
        size_t address = 0;
        ASSERT_TRUE(map->voxelToLinearAddress(index, address));
        ASSERT_LT(address, total);
        VoxelIndex decoded;
        ASSERT_TRUE(map->linearAddressToVoxel(address, decoded));
        EXPECT_EQ(decoded, index);
        ++checked;
      }
    }
  }
  EXPECT_GT(checked, 100u);
}

TEST_F(MapConsistencyTest, PositionIndexRoundTrip) {
  const GlobalMap::Ptr map = makeTestMap("map_consistency_pos");
  const std::vector<Position> samples = {
      Position(-8.0f, -6.0f, 1.0f),
      Position(0.0f, 0.0f, 0.5f),
      Position(3.37f, -1.23f, 2.1f),
      Position(9.5f, 9.5f, 2.8f),
  };
  for (const auto& sample : samples) {
    const VoxelIndex index = map->posToIndex(sample);
    EXPECT_TRUE(map->isInGlobalMapRange(index));
    const Position decoded = map->indexToPos(index);
    EXPECT_LE((decoded - sample).cwiseAbs().maxCoeff(),
              static_cast<float>(map->getResolution()) + 1e-3f);
  }
}

// ==================== GridPathFinder ====================

class GridPathFinderTest : public RosMasterTest {};

TEST_F(GridPathFinderTest, UnknownPolicyIsRespected) {
  const GlobalMap::Ptr map = makeTestMap("grid_map_unknown");
  GridPathFinder finder;
  finder.init(map, 0.0);

  const Eigen::Vector3d start(-8.0, 0.0, 1.0);
  const Eigen::Vector3d goal(8.0, 0.0, 1.0);

  // 全未知地图：BLOCK 必须在起点即失败。
  EXPECT_FALSE(
      finder.search(start, goal, UnknownSpacePolicy::BLOCK, 200000).success);

  const GridPathResult allowed =
      finder.search(start, goal, UnknownSpacePolicy::ALLOW, 200000);
  ASSERT_TRUE(allowed.success);
  EXPECT_GE(allowed.points.size(), 2u);
  EXPECT_TRUE(finder.validatePath(allowed.points, UnknownSpacePolicy::ALLOW));
  EXPECT_FALSE(finder.validatePath(allowed.points, UnknownSpacePolicy::BLOCK));
}

TEST_F(GridPathFinderTest, RoutesAroundOccupiedWall) {
  const GlobalMap::Ptr map = makeTestMap("grid_map_wall");
  buildWall(*map, Position(-8.0f, 0.0f, 1.0f));
  ASSERT_TRUE(map->isOccupied(Position(0.0f, 0.0f, 1.0f)));

  GridPathFinder finder;
  finder.init(map, 0.0);

  const Eigen::Vector3d start(-2.0, 0.0, 1.0);
  const Eigen::Vector3d goal(2.0, 0.0, 1.0);

  // 墙后为未知空间：BLOCK 不可达。
  EXPECT_FALSE(
      finder.search(start, goal, UnknownSpacePolicy::BLOCK, 200000).success);

  // ALLOW 可绕墙到达未知空间。
  const GridPathResult result =
      finder.search(start, goal, UnknownSpacePolicy::ALLOW, 200000);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(finder.validatePath(result.points, UnknownSpacePolicy::ALLOW));

  // 直接穿墙的路径必须被拒绝（即使 ALLOW 也不允许占据体素）。
  const std::vector<Eigen::Vector3d> crossing = {
      Eigen::Vector3d(-2.0, 0.0, 1.0), Eigen::Vector3d(2.0, 0.0, 1.0)};
  EXPECT_FALSE(finder.validatePath(crossing, UnknownSpacePolicy::ALLOW));
}

// ==================== 代价模型 / LKH / 精化 / 端到端 ====================

class GlobalTourPlannerTest : public RosMasterTest {
 protected:
  void SetUp() override {
    RosMasterTest::SetUp();
    if (testing::Test::IsSkipped()) return;
    map_ = makeTestMap("planner_map");
    nh_ = std::make_shared<ros::NodeHandle>("~");
    planner_.init(*nh_, map_);
    planner_.config_.unknown_as_occupied = false;
    planner_.config_.refine_local = true;
  }

  auto& config() { return planner_.config_; }

  double motionTime(const Eigen::Vector3d& from, double from_yaw,
                    const Eigen::Vector3d& to, double to_yaw) {
    return planner_.motionTime(from, from_yaw, to, to_yaw);
  }

  double wrapDiff(double a, double b) {
    return GlobalTourPlanner::wrapDiff(a, b);
  }

  void normalizeUtilities(std::vector<ExplorationTask>& tasks) {
    planner_.normalizeUtilities(tasks);
  }

  void buildCostMatrix(const RobotPlanningState& robot,
                       const std::vector<ExplorationTask>& tasks,
                       const std::optional<ExplorationTaskKey>& previous_goal,
                       std::vector<std::vector<double>>& matrix) {
    planner_.buildCostMatrix(robot, tasks, previous_goal, matrix);
  }

  bool runLkh(const std::vector<std::vector<double>>& matrix,
              std::vector<int>& order, std::string& reason) {
    return planner_.runLkh(matrix, order, reason);
  }

  std::vector<int> runFallback(
      const std::vector<std::vector<double>>& matrix) const {
    return planner_.runFallback(matrix);
  }

  bool refineLocal(const RobotPlanningState& robot,
                   const std::vector<ExplorationTask>& tasks,
                   const std::vector<int>& order,
                   const std::optional<ExplorationTaskKey>& previous_goal,
                   std::vector<int>& refined_task_indices,
                   std::vector<int>& refined_viewpoint_indices) {
    return planner_.refineLocal(robot, tasks, order, previous_goal,
                                refined_task_indices, refined_viewpoint_indices);
  }

  GlobalTourPlan plan(const RobotPlanningState& robot,
                      const std::vector<ExplorationTask>& tasks,
                      const std::optional<ExplorationTaskKey>& previous_goal) {
    return planner_.plan(robot, tasks, previous_goal);
  }

  const GridPathFinder& pathFinder() const { return planner_.pathFinder(); }

  size_t taskEdgeCacheSize() const { return planner_.task_edge_cache_.size(); }

  GlobalMap::Ptr map_;
  std::shared_ptr<ros::NodeHandle> nh_;
  GlobalTourPlanner planner_;
};

TEST_F(GlobalTourPlannerTest, MotionTimeMatchesFuelModel) {
  EXPECT_NEAR(motionTime(Eigen::Vector3d(0, 0, 0), 0.0,
                         Eigen::Vector3d(10, 0, 0), 0.0),
              5.0, 1e-9);
  // 平移与 yaw 可并行，取二者最大值：max(10/2, (pi/2)/0.25)。
  // 方向惩罚由带当前速度的 pathAwareMotionTime 计算，而不是把目标 yaw
  // 重复计入位置时间。
  EXPECT_NEAR(motionTime(Eigen::Vector3d(0, 0, 0), 0.0,
                         Eigen::Vector3d(10, 0, 0), kPi / 2.0),
              2.0 * kPi, 1e-9);
  // 原地转向时由 yaw 项主导：1.0 / 0.25 = 4.0
  EXPECT_NEAR(motionTime(Eigen::Vector3d(0, 0, 0), 0.0,
                         Eigen::Vector3d(0, 0, 0), 1.0),
              4.0, 1e-9);
  EXPECT_NEAR(wrapDiff(3.0 * kPi / 2.0, 0.0), -kPi / 2.0, 1e-9);
}

TEST_F(GlobalTourPlannerTest, NormalizeUtilities) {
  std::vector<ExplorationTask> tasks;
  tasks.push_back(makeTask(ExplorationTaskType::FRONTIER, 1,
                           Eigen::Vector3d::Zero(), 0.0, {}));
  tasks.push_back(makeTask(
      ExplorationTaskType::SEMANTIC, 2, Eigen::Vector3d::Zero(), 3.0,
      {ExplorationViewpoint{Eigen::Vector3d::Zero(), 0.0, 1.0}}));
  tasks[1].semantic_value = 1.0;
  tasks.push_back(makeTask(
      ExplorationTaskType::FRONTIER, 3, Eigen::Vector3d::Zero(), 0.5,
      {ExplorationViewpoint{Eigen::Vector3d::Zero(), 0.0, 0.5}}));
  tasks[2].semantic_value = 1.0;  // FRONTIER 应忽略语义价值

  normalizeUtilities(tasks);

  EXPECT_NEAR(tasks[1].normalized_gain, 1.0, 1e-9);
  EXPECT_NEAR(tasks[0].normalized_gain, 0.0, 1e-9);

  const double expected_gain = std::log1p(0.5) / std::log1p(3.0);
  EXPECT_NEAR(tasks[2].normalized_gain, expected_gain, 1e-9);
  EXPECT_NEAR(tasks[2].semantic_value, 0.0, 1e-9);

  // U = 0.70*g + 0.25*s + 0.05*q，clamp 到 [0,1]
  EXPECT_NEAR(tasks[0].utility, 0.0, 1e-9);
  EXPECT_NEAR(tasks[1].utility, 1.0, 1e-9);
  EXPECT_NEAR(tasks[2].utility, 0.70 * expected_gain + 0.05 * 0.5, 1e-9);
}

TEST_F(GlobalTourPlannerTest, CostMatrixFirstHop) {
  RobotPlanningState robot;
  robot.position = Eigen::Vector3d(0, 0, 0);
  robot.yaw = 0.0;
  robot.valid = true;

  std::vector<ExplorationTask> tasks;
  tasks.push_back(makeTask(
      ExplorationTaskType::FRONTIER, 1, Eigen::Vector3d(8, 0, 0), 1.0,
      {ExplorationViewpoint{Eigen::Vector3d(8, 0, 0), 0.0, 1.0}}));
  tasks.push_back(makeTask(
      ExplorationTaskType::SEMANTIC, 2, Eigen::Vector3d(0, 8, 0), 1.0,
      {ExplorationViewpoint{Eigen::Vector3d(0, 8, 0), 0.0, 1.0}}));
  tasks[0].utility = 1.0;
  tasks[1].utility = 0.0;

  std::vector<std::vector<double>> matrix;
  buildCostMatrix(robot, tasks, tasks[1].key, matrix);
  ASSERT_EQ(matrix.size(), 3u);

  // C_0j = T_0j + 5.0 * (1 - U_j) - 1.0 * [j == previous]
  EXPECT_NEAR(matrix[0][1], 4.0, 1e-9);
  EXPECT_NEAR(matrix[0][2], 4.0 + 5.0 - 1.0, 1e-9);
  // 任务返回机器人无代价
  EXPECT_DOUBLE_EQ(matrix[1][0], 0.0);
  EXPECT_DOUBLE_EQ(matrix[2][0], 0.0);

  const double inter_task = std::sqrt(128.0) / 2.0;
  EXPECT_NEAR(matrix[1][2], inter_task, 1e-9);
  EXPECT_NEAR(matrix[2][1], inter_task, 1e-9);
}

TEST_F(GlobalTourPlannerTest, TaskEdgeMatrixIsIncrementallyCached) {
  RobotPlanningState robot;
  robot.position = Eigen::Vector3d::Zero();
  robot.yaw = 0.0;
  robot.valid = true;

  std::vector<ExplorationTask> tasks;
  tasks.push_back(makeTask(
      ExplorationTaskType::FRONTIER, 101, Eigen::Vector3d(4, 0, 1), 1.0,
      {ExplorationViewpoint{Eigen::Vector3d(4, 0, 1), 0.0, 1.0}}));
  tasks.push_back(makeTask(
      ExplorationTaskType::FRONTIER, 102, Eigen::Vector3d(0, 4, 1), 1.0,
      {ExplorationViewpoint{Eigen::Vector3d(0, 4, 1), 0.0, 1.0}}));
  tasks.push_back(makeTask(
      ExplorationTaskType::FRONTIER, 103, Eigen::Vector3d(-4, 0, 1), 1.0,
      {ExplorationViewpoint{Eigen::Vector3d(-4, 0, 1), 0.0, 1.0}}));
  for (auto& task : tasks) task.utility = 1.0;

  std::vector<std::vector<double>> first_matrix;
  buildCostMatrix(robot, tasks, std::nullopt, first_matrix);
  EXPECT_EQ(taskEdgeCacheSize(), 6u);  // Three undirected pairs, stored both ways.

  std::vector<std::vector<double>> second_matrix;
  buildCostMatrix(robot, tasks, std::nullopt, second_matrix);
  EXPECT_EQ(taskEdgeCacheSize(), 6u);
  EXPECT_EQ(first_matrix, second_matrix);

  // Removing a task prunes all incident cached rows/columns and retains the
  // remaining old↔old edge, matching FUEL's incremental matrix maintenance.
  tasks.pop_back();
  buildCostMatrix(robot, tasks, std::nullopt, second_matrix);
  EXPECT_EQ(taskEdgeCacheSize(), 2u);
}

TEST_F(GlobalTourPlannerTest, LkhSolvesSmallAtsp) {
  // 节点 0 = 机器人；节点 1/2 = 任务 0/1
  std::vector<std::vector<double>> matrix(3, std::vector<double>(3, 0.0));
  matrix[0][1] = 3.0;  // robot -> task0
  matrix[0][2] = 4.0;  // robot -> task1
  matrix[1][2] = 1.0;  // task0 -> task1
  matrix[2][1] = 2.0;  // task1 -> task0

  std::vector<int> order;
  std::string reason;
  ASSERT_TRUE(runLkh(matrix, order, reason)) << reason;
  // 最优巡游：robot -> task0 -> task1（代价 4 < 6）
  EXPECT_EQ(order, (std::vector<int>{0, 1}));
}

TEST_F(GlobalTourPlannerTest, FallbackImprovesWithDirectedTwoOpt) {
  // 节点 0 = 机器人，节点 1..3 = 任务 0..2
  // 最近邻首解为 [0,1,2]（代价 201），有向 2-opt 应改进到 [2,1,0]（代价 52）。
  std::vector<std::vector<double>> matrix(4, std::vector<double>(4, 0.0));
  matrix[0][1] = 1.0;
  matrix[0][2] = 50.0;
  matrix[0][3] = 50.0;

  matrix[1][2] = 100.0;
  matrix[1][3] = 1000.0;
  matrix[2][1] = 1.0;
  matrix[2][3] = 100.0;
  matrix[3][1] = 1.0;
  matrix[3][2] = 1.0;

  const std::vector<int> order = runFallback(matrix);
  ASSERT_EQ(order.size(), 3u);

  std::vector<int> sorted = order;
  std::sort(sorted.begin(), sorted.end());
  EXPECT_EQ(sorted, (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(order, (std::vector<int>{2, 1, 0}));

  auto tour_cost = [&matrix](const std::vector<int>& tour) {
    if (tour.empty()) return 0.0;
    double cost = matrix[0][tour.front() + 1];
    for (size_t i = 1; i < tour.size(); ++i) {
      cost += matrix[tour[i - 1] + 1][tour[i] + 1];
    }
    return cost;
  };
  EXPECT_NEAR(tour_cost(order), 52.0, 1e-9);
}

TEST_F(GlobalTourPlannerTest, RefineSelectsLowestMotionCost) {
  RobotPlanningState robot;
  robot.position = Eigen::Vector3d(0, 0, 0);
  robot.yaw = 0.0;
  robot.valid = true;

  std::vector<ExplorationTask> tasks;
  tasks.push_back(makeTask(
      ExplorationTaskType::FRONTIER, 1, Eigen::Vector3d(3, 0, 0), 1.0,
      {ExplorationViewpoint{Eigen::Vector3d(1, 0, 0), 0.0, 0.9},
       ExplorationViewpoint{Eigen::Vector3d(5, 0, 0), 0.0, 0.5}}));
  tasks.push_back(makeTask(
      ExplorationTaskType::FRONTIER, 2, Eigen::Vector3d(6, 0, 0), 1.0,
      {ExplorationViewpoint{Eigen::Vector3d(2, 0, 0), 0.0, 0.9},
       ExplorationViewpoint{Eigen::Vector3d(10, 0, 0), 0.0, 0.5}}));
  tasks[0].utility = 1.0;
  tasks[1].utility = 1.0;

  std::vector<int> task_indices;
  std::vector<int> viewpoint_indices;
  ASSERT_TRUE(refineLocal(robot, tasks, {0, 1}, std::nullopt, task_indices,
                          viewpoint_indices));
  ASSERT_EQ(task_indices.size(), 2u);
  ASSERT_EQ(viewpoint_indices.size(), 2u);
  EXPECT_EQ(task_indices[0], 0);
  EXPECT_EQ(task_indices[1], 1);
  // 最优组合为 (1,0,0) -> (2,0,0)，总运动时间 0.5 + 0.5。
  EXPECT_EQ(viewpoint_indices[0], 0);
  EXPECT_EQ(viewpoint_indices[1], 0);
}

TEST_F(GlobalTourPlannerTest, RefineWindowRespectsRadius) {
  config().refined_num = 5;
  config().refined_radius = 0.1;

  RobotPlanningState robot;
  robot.position = Eigen::Vector3d(0, 0, 0);
  robot.yaw = 0.0;
  robot.valid = true;

  std::vector<ExplorationTask> tasks;
  for (int i = 0; i < 3; ++i) {
    const double x = 3.0 * static_cast<double>(i + 1);
    tasks.push_back(makeTask(
        ExplorationTaskType::FRONTIER, i + 1, Eigen::Vector3d(x, 0, 0), 1.0,
        {ExplorationViewpoint{Eigen::Vector3d(x, 0, 0), 0.0, 1.0}}));
    tasks.back().utility = 1.0;
  }

  std::vector<int> task_indices;
  std::vector<int> viewpoint_indices;
  ASSERT_TRUE(refineLocal(robot, tasks, {0, 1, 2}, std::nullopt, task_indices,
                          viewpoint_indices));
  // 累计距离超过 refined_radius 后截断，但至少保留 2 个任务。
  EXPECT_EQ(task_indices.size(), 2u);
}

TEST_F(GlobalTourPlannerTest, RefineFailsWithoutFirstLayer) {
  RobotPlanningState robot;
  robot.position = Eigen::Vector3d(0, 0, 0);
  robot.yaw = 0.0;
  robot.valid = true;

  std::vector<ExplorationTask> tasks;
  tasks.push_back(makeTask(ExplorationTaskType::FRONTIER, 1,
                           Eigen::Vector3d(3, 0, 0), 1.0, {}));
  tasks.back().utility = 1.0;

  std::vector<int> task_indices;
  std::vector<int> viewpoint_indices;
  EXPECT_FALSE(refineLocal(robot, tasks, {0}, std::nullopt, task_indices,
                           viewpoint_indices));
}

TEST_F(GlobalTourPlannerTest, PlanEndToEnd) {
  RobotPlanningState robot;
  robot.position = Eigen::Vector3d(0, 0, 1);
  robot.yaw = 0.0;
  robot.valid = true;

  std::vector<ExplorationTask> tasks;
  tasks.push_back(makeTask(
      ExplorationTaskType::FRONTIER, 7, Eigen::Vector3d(5, 0, 1), 2.0,
      {ExplorationViewpoint{Eigen::Vector3d(5, 0, 1), 0.0, 0.9}}));
  tasks.push_back(makeTask(
      ExplorationTaskType::SEMANTIC, 3, Eigen::Vector3d(0, 5, 1), 5.0,
      {ExplorationViewpoint{Eigen::Vector3d(0, 5, 1), 1.57, 0.8}}));

  const GlobalTourPlan result = plan(robot, tasks, std::nullopt);
  ASSERT_TRUE(result.success) << result.failure_reason;
  ASSERT_EQ(result.task_order.size(), 2u);
  EXPECT_EQ(result.next_goal.task_key, result.task_order.front());
  EXPECT_GE(result.next_goal.path.size(), 2u);
  EXPECT_TRUE(pathFinder().validatePath(result.next_goal.path,
                                        UnknownSpacePolicy::ALLOW));
  EXPECT_FALSE(result.refined_viewpoints.empty());
}

// ==================== 聚类性能 ====================

class ViewpointManagerClusterPerf : public RosMasterTest {
 protected:
  void SetUp() override {
    RosMasterTest::SetUp();
    if (testing::Test::IsSkipped()) return;
    map_ = makeTestMap("cluster_map");
    manager_.global_map_ = map_;
    manager_.config_.min_cluster_size = 10;
    manager_.config_.cluster_split_threshold = 1e6f;
  }

  void cluster(const std::vector<Eigen::Vector3f>& points,
               std::vector<FrontierCluster>& clusters) {
    manager_.clusterFrontiers(points, clusters);
  }

  GlobalMap::Ptr map_;
  ViewpointManager manager_;
};

TEST_F(ViewpointManagerClusterPerf, P95Under20msFor20kPoints) {
  std::vector<Eigen::Vector3f> points;
  points.reserve(20000);
  // 20 个互相分离的 10x10x10 体素块，每块 1000 点。
  for (int blob = 0; blob < 20; ++blob) {
    const float cx = -7.0f + 3.5f * static_cast<float>(blob % 5);
    const float cy = -7.0f + 3.5f * static_cast<float>(blob / 5);
    for (int i = 0; i < 1000; ++i) {
      const float dx = 0.2f * static_cast<float>(i % 10);
      const float dy = 0.2f * static_cast<float>((i / 10) % 10);
      const float dz = 0.2f * static_cast<float>(i / 100);
      points.emplace_back(cx + dx + 0.01f, cy + dy + 0.01f, -0.5f + dz + 0.01f);
    }
  }
  ASSERT_EQ(points.size(), 20000u);

  std::vector<double> samples;
  samples.reserve(30);
  std::vector<FrontierCluster> clusters;
  for (int run = 0; run < 30; ++run) {
    const auto start = std::chrono::steady_clock::now();
    cluster(points, clusters);
    const auto end = std::chrono::steady_clock::now();
    samples.push_back(
        std::chrono::duration<double, std::milli>(end - start).count());
    ASSERT_EQ(clusters.size(), 20u);
  }

  std::sort(samples.begin(), samples.end());
  const size_t p95_index =
      static_cast<size_t>(std::ceil(0.95 * samples.size())) - 1;
  EXPECT_LE(samples[p95_index], 20.0);
}

}  // namespace rog_map

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_global_scheduler");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
