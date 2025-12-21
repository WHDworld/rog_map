// frontier_viewpoint_generator.h
// 前端视点生成器 - 基于EDEN的EROI视点生成算法
// 功能：从输入的前沿子区域生成探索视点，并规划区域探索顺序

#ifndef FRONTIER_VIEWPOINT_GENERATOR_H
#define FRONTIER_VIEWPOINT_GENERATOR_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <list>

#include "rog_map/global_map.h"

namespace rog_map {

// ==================== 常量定义 ====================
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;

// ==================== 核心数据结构 ====================

/**
 * @brief 前视簇结构
 * 
 * 前视簇是对前视体素的聚类结果，可能需要分裂
 */
struct FrontierCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    std::vector<GlobalVoxelIndex> voxel_indices;  // 簇内体素索引
    Eigen::Vector3f centroid;            // 簇中心位置
    Eigen::Vector3f normal;              // 簇法向量（指向已知区域）
    Eigen::Vector3f principal_axis;      // 主轴方向（PCA最大特征值方向）
    float extent;                        // 簇的范围（最大特征值的平方根）
    int cluster_id;                      // 簇ID
    int parent_region_id;                // 所属探索区域ID
    bool needs_split;                    // 是否需要分裂
    
    FrontierCluster()
        : centroid(Eigen::Vector3f::Zero())
        , normal(Eigen::Vector3f::UnitZ())
        , principal_axis(Eigen::Vector3f::UnitX())
        , extent(0.0f)
        , cluster_id(-1)
        , parent_region_id(-1)
        , needs_split(false)
    {}
    
    int size() const { return static_cast<int>(voxel_indices.size()); }
};

/**
 * @brief 5-DOF探索视点结构
 */
struct ExplorationViewpoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3f position;           // 3D位置
    float yaw;                          // 偏航角 [-π, π)
    float pitch;                        // 俯仰角 [-π/2, π/2]
    
    int region_id;                      // 所属探索区域ID
    int cluster_id;                     // 目标前视簇ID
    
    // 评分
    float total_score;                  // 综合评分
    int visible_frontier_count;         // 可见前视体素数
    
    // 覆盖信息（用于集合覆盖）
    std::vector<int> covered_cluster_ids;  // 可覆盖的簇ID列表
    
    bool is_valid;
    
    ExplorationViewpoint() 
        : position(Eigen::Vector3f::Zero())
        , yaw(0.0f), pitch(0.0f)
        , region_id(-1), cluster_id(-1)
        , total_score(0.0f)
        , visible_frontier_count(0)
        , is_valid(false)
    {}
};

/**
 * @brief 探索区域块（由一个或多个子网格合并而成）
 */
struct ExplorationRegion {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int region_id;                                    // 区域ID
    std::vector<int> subgrid_ids;                     // 包含的子网格ID
    Eigen::Vector3f centroid;                         // 区域中心
    
    std::vector<FrontierCluster> clusters;            // 区域内的前视簇
    std::vector<ExplorationViewpoint> viewpoints;     // 区域内的探索视点
    
    float total_frontier_area;                        // 总前视面积
    float exploration_priority;                       // 探索优先级
    
    ExplorationRegion()
        : region_id(-1)
        , centroid(Eigen::Vector3f::Zero())
        , total_frontier_area(0.0f)
        , exploration_priority(0.0f)
    {}
};

/**
 * @brief 区域路由节点
 */
struct RegionRouteNode {
    int region_id;                  // 区域ID
    float distance_from_uav;        // 距离UAV当前位置的路径长度
    float exploration_gain;         // 探索收益
    float route_score;              // 路由评分
    int visit_order;                // 访问顺序（-1表示未分配）
    
    RegionRouteNode()
        : region_id(-1)
        , distance_from_uav(std::numeric_limits<float>::max())
        , exploration_gain(0.0f)
        , route_score(0.0f)
        , visit_order(-1)
    {}
};

/**
 * @brief UAV状态
 */
struct UAVState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    float yaw;
    
    UAVState()
        : position(Eigen::Vector3f::Zero())
        , velocity(Eigen::Vector3f::Zero())
        , yaw(0.0f)
    {}
};

/**
 * @brief 视点生成器配置参数
 */
struct ViewpointGeneratorConfig {
    // 传感器参数
    float sensor_range_min = 0.3f;
    float sensor_range_max = 5.0f;
    float sensor_fov_h = 1.57f;           // 90°
    float sensor_fov_v = 1.22f;           // 70°
    float optimal_viewing_distance = 3.0f;
    
    // 无人机参数
    float uav_radius = 0.35f;
    float safety_margin = 0.2f;
    float max_velocity = 2.0f;
    
    // 采样参数
    int sampling_step = 4;
    int max_viewpoints_per_region = 20;
    float min_viewpoint_distance = 0.5f;
    
    // 前视簇参数
    int min_cluster_size = 10;
    int max_clusters_per_region = 15;
    float cluster_split_threshold = 3.0f;  // 超过此距离需要分裂(m)
    int absolute_noise_threshold = 5;
    float obstacle_ratio_threshold = 0.7f;
    
    // 覆盖评分参数
    float min_coverage_score = 0.1f;
    float max_viewing_angle = 0.785f;     // 45°
    
    // 路由参数
    float exploration_weight = 0.6f;      // 探索收益权重
    float distance_weight = 0.4f;         // 距离权重
};

// ==================== 主类定义 ====================

/**
 * @brief 前端视点生成器
 * 
 * 主要功能：
 * 1. 从输入的前沿子区域ID生成探索区域块
 * 2. 对每个区域进行前视簇聚类和分裂
 * 3. 生成覆盖所有前视簇的最小视点集
 * 4. 规划区域探索顺序
 */
class FrontierViewpointGenerator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<FrontierViewpointGenerator> Ptr;
    
    FrontierViewpointGenerator(GlobalMap::Ptr global_map, 
                               const ViewpointGeneratorConfig& config);
    ~FrontierViewpointGenerator() = default;
    
    // ==================== 核心接口 ====================
    
    /**
     * @brief 更新UAV状态
     */
    void updateUAVState(const UAVState& state);
    
    /**
     * @brief 主入口：从子区域ID生成探索视点和区域路由
     * 
     * @param subgrid_ids 前沿子区域ID列表
     * @param regions 输出的探索区域（含视点）
     * @param route 输出的区域访问顺序
     * @return 是否成功
     */
    bool generateExplorationPlan(
        const std::vector<int>& subgrid_ids,
        std::vector<ExplorationRegion>& regions,
        std::vector<int>& route);
    
    /**
     * @brief 获取首个区域的探索视点（用于立即执行）
     */
    bool getFirstRegionViewpoints(
        std::vector<ExplorationViewpoint>& viewpoints);
    
    // ==================== 查询接口 ====================
    
    const std::vector<ExplorationRegion>& getRegions() const { return regions_; }
    const std::vector<int>& getRoute() const { return route_; }
    const ViewpointGeneratorConfig& getConfig() const { return config_; }
    void setConfig(const ViewpointGeneratorConfig& config) { config_ = config; }

private:
    // ==================== 核心算法 ====================
    
    /**
     * @brief Step 1: 将子区域划分为探索区域块
     */
    void partitionIntoRegions(
        const std::vector<int>& subgrid_ids,
        std::vector<ExplorationRegion>& regions);
    
    /**
     * @brief Step 2: 对区域内的前视体素进行聚类
     */
    void clusterFrontiersInRegion(ExplorationRegion& region);
    
    /**
     * @brief Step 3: 对大簇进行PCA分裂
     * @return 是否进行了分裂
     */
    bool splitLargeClusters(std::vector<FrontierCluster>& clusters);
    
    /**
     * @brief 递归分裂单个簇（水平方向）
     */
    bool splitClusterHorizontally(
        const FrontierCluster& cluster,
        std::list<FrontierCluster>& splits);
    
    /**
     * @brief Step 4: 为区域生成覆盖所有簇的视点
     */
    void generateViewpointsForRegion(ExplorationRegion& region);
    
    /**
     * @brief Step 5: 贪心选择最小视点集
     */
    void greedySelectViewpoints(
        const std::vector<Eigen::Vector3f>& candidates,
        const std::vector<FrontierCluster>& clusters,
        std::vector<ExplorationViewpoint>& selected);
    
    /**
     * @brief Step 6: 规划区域访问顺序
     */
    void planRegionRoute(std::vector<ExplorationRegion>& regions,
                         std::vector<int>& route);
    
    // ==================== 辅助函数 ====================
    
    // 聚类相关
    void computeClusterPCA(FrontierCluster& cluster);
    bool checkClusterNeedsSplit(const FrontierCluster& cluster);
    
    // 采样相关
    void sampleFreeSpaceInRegion(
        const ExplorationRegion& region,
        std::vector<Eigen::Vector3f>& samples);
    
    // 评分相关
    float computeCoverageScore(
        const Eigen::Vector3f& position,
        const FrontierCluster& cluster,
        float& best_yaw, float& best_pitch);
    
    int countVisibleFrontierVoxels(
        const Eigen::Vector3f& pos, float yaw, float pitch,
        const FrontierCluster& cluster);
    
    // 安全检查
    bool isPositionSafe(const Eigen::Vector3f& pos);
    bool isLineVisible(const Eigen::Vector3f& from, const Eigen::Vector3f& to);
    
    // 工具函数
    float normalizeAngle(float angle);
    float distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b);

private:
    GlobalMap::Ptr global_map_;
    ViewpointGeneratorConfig config_;
    
    UAVState uav_state_;
    std::mutex state_mutex_;
    
    // 缓存
    std::vector<ExplorationRegion> regions_;
    std::vector<int> route_;
};

} // namespace rog_map

#endif // FRONTIER_VIEWPOINT_GENERATOR_H
