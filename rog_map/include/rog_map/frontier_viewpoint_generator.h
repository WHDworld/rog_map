/**
 * @file frontier_viewpoint_generator.h
 * @brief 前沿视点生成器 - 基于子网格的前沿簇聚类与视点采样
 * 
 * 核心流程：
 * 1. 基于子网格分区获取前沿点
 * 2. 在每个子网格内聚类生成前沿簇
 * 3. 对大簇进行分裂
 * 4. 圆周采样生成探索视点
 * 5. 去重滤波得到最终视点集
 */

#ifndef FRONTIER_VIEWPOINT_GENERATOR_H
#define FRONTIER_VIEWPOINT_GENERATOR_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <mutex>
#include <algorithm>

#include "rog_map/global_map.h"

namespace rog_map {

constexpr float PI = 3.14159265358979323846f;

// ==================== 数据结构 ====================

/**
 * @brief 前沿簇
 */
struct FrontierCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int cluster_id;                      // 簇ID
    int subgrid_id;                      // 所属子网格ID
    Eigen::Vector3f centroid;            // 簇中心
    Eigen::Vector3f normal;              // 法向量
    int point_count;                     // 点数
    float extent;                        // 范围（用于判断是否需要分裂）
    
    // 前沿点位置列表（用于可视化）
    std::vector<Eigen::Vector3f> points;
    
    FrontierCluster()
        : cluster_id(-1), subgrid_id(-1)
        , centroid(Eigen::Vector3f::Zero())
        , normal(Eigen::Vector3f::UnitZ())
        , point_count(0), extent(0.0f) {}
    
    int size() const { return point_count; }
};

/**
 * @brief 探索视点（5-DOF: x, y, z, yaw, pitch）
 */
struct ExplorationViewpoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3f position;            // 位置
    float yaw;                           // 偏航角
    float pitch;                         // 俯仰角（通常为0）
    int cluster_id;                      // 目标簇ID
    float score;                         // 评分
    bool is_valid;
    
    ExplorationViewpoint()
        : position(Eigen::Vector3f::Zero())
        , yaw(0.0f), pitch(0.0f)
        , cluster_id(-1), score(0.0f), is_valid(false) {}
};

/**
 * @brief UAV状态
 */
struct UAVState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f position;
    float yaw;
    UAVState() : position(Eigen::Vector3f::Zero()), yaw(0.0f) {}
};

/**
 * @brief 配置参数
 */
struct ViewpointGeneratorConfig {
    // ========== 传感器参数 ==========
    float sensor_range_min = 0.5f;       // 最小感知距离 [m]
    float sensor_range_max = 5.0f;       // 最大感知距离 [m]
    float sensor_fov_h = 1.57f;          // 水平FOV [rad] (~90°)
    float sensor_fov_v = 1.22f;          // 垂直FOV [rad] (~70°)
    
    // ========== 视点采样参数 ==========
    float viewpoint_height = 1.0f;       // 视点高度（起飞高度）
    float candidate_rmin = 1.5f;         // 最小采样半径
    float candidate_rmax = 4.5f;         // 最大采样半径
    int candidate_rnum = 3;              // 半径采样数量
    float candidate_dphi = 0.5f;         // 角度采样间隔 [rad]
    float optimal_distance = 3.0f;       // 最优观测距离
    
    // ========== 前沿簇参数 ==========
    float cluster_eps = 1.0f;            // 聚类半径
    int min_cluster_size = 10;           // 最小簇大小
    float cluster_split_threshold = 3.0f;// 簇分裂阈值
    int max_clusters = 20;               // 最大簇数
    
    // ========== 安全参数 ==========
    float uav_radius = 0.35f;
    float safety_margin = 0.2f;
    
    // ========== 滤波参数 ==========
    float min_viewpoint_distance = 0.8f; // 视点最小间距
    float min_coverage_score = 0.3f;     // 最小覆盖评分（低于此值的视点被过滤）
};

// ==================== 主类 ====================

class FrontierViewpointGenerator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<FrontierViewpointGenerator> Ptr;
    
    FrontierViewpointGenerator(GlobalMap::Ptr global_map, 
                               const ViewpointGeneratorConfig& config);
    ~FrontierViewpointGenerator() = default;
    
    // ==================== 核心接口 ====================
    
    void updateUAVState(const UAVState& state);
    
    /**
     * @brief 主接口：生成探索视点
     * 
     * 流程：子网格分区 → 聚类 → 分裂 → 采样 → 滤波
     * 
     * @param clusters 输出的前沿簇
     * @param viewpoints 输出的探索视点
     * @return 是否成功
     */
    bool generate(std::vector<FrontierCluster>& clusters,
                  std::vector<ExplorationViewpoint>& viewpoints);
    
    const ViewpointGeneratorConfig& getConfig() const { return config_; }
    void setConfig(const ViewpointGeneratorConfig& config) { config_ = config; }

private:
    // ==================== 核心算法 ====================
    
    /**
     * @brief Step 1: 在子网格内聚类前沿点
     */
    void clusterInSubgrid(const std::vector<Eigen::Vector3f>& points,
                          int subgrid_id,
                          std::vector<FrontierCluster>& clusters);
    
    /**
     * @brief Step 2: 分裂大簇
     */
    void splitLargeClusters(std::vector<FrontierCluster>& clusters);
    
    /**
     * @brief Step 3: 为簇采样视点（圆周采样）
     */
    void sampleViewpoints(const FrontierCluster& cluster,
                          std::vector<ExplorationViewpoint>& viewpoints);
    
    /**
     * @brief Step 4: 去重滤波
     */
    void filterViewpoints(std::vector<ExplorationViewpoint>& viewpoints);
    
    // ==================== 辅助函数 ====================
    
    bool isPositionSafe(const Eigen::Vector3f& pos);
    bool isPositionFree(const Eigen::Vector3f& pos);
    
    /**
     * @brief 计算视点的覆盖评分
     * 
     * 评估视点能否有效观测到指定簇：
     * 1. 距离评分 - 是否在最优观测距离
     * 2. 可见性评分 - 射线是否被障碍物遮挡
     * 3. 水平角度评分 - 簇中心是否在水平FOV内
     * 4. 垂直角度评分 - 簇中心是否在垂直FOV内
     * 5. 覆盖评分 - 簇范围是否能被视锥体覆盖
     */
    float computeCoverageScore(const Eigen::Vector3f& viewpoint,
                               float yaw,
                               const FrontierCluster& cluster);
    
    /**
     * @brief 计算可见性评分
     * 
     * 从视点向簇中心发射射线，检查是否有障碍物遮挡
     * @return 可见性评分 [0, 1]，0表示完全遮挡，1表示完全可见
     */
    float computeVisibilityScore(const Eigen::Vector3f& viewpoint,
                                 const FrontierCluster& cluster);
    
    /**
     * @brief 检查射线可见性
     * 
     * 从起点到终点发射射线，检查是否有障碍物
     * @return true 如果可见（无遮挡），false 如果被遮挡
     */
    bool checkRayVisibility(const Eigen::Vector3f& start,
                            const Eigen::Vector3f& end);
    
    /**
     * @brief 检查沿指定方向的射线是否畅通
     * 
     * @param start 起点
     * @param direction 方向（单位向量）
     * @param max_dist 最大检测距离
     * @return true 如果畅通，false 如果被障碍物阻挡
     */
    bool checkRayClear(const Eigen::Vector3f& start,
                       const Eigen::Vector3f& direction,
                       float max_dist);

private:
    GlobalMap::Ptr global_map_;
    ViewpointGeneratorConfig config_;
    UAVState uav_state_;
    std::mutex state_mutex_;
};

} // namespace rog_map

#endif // FRONTIER_VIEWPOINT_GENERATOR_H
