/**
 * @file frontier_viewpoint_generator.h
 * @brief 前沿视点生成器 - 基于历史节点的前沿簇聚类与视点采样
 * 
 * 核心流程（借鉴EDEN算法）：
 * 1. 沿无人机轨迹采集历史节点
 * 2. 将前沿区域（子网格）关联到最近的历史节点
 * 3. 同一历史节点关联的前沿区域合并聚类
 * 4. 对大簇进行分裂
 * 5. 圆周采样生成探索视点
 * 6. 去重滤波得到最终视点集
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
 * @brief 关联的子网格区域信息（用于可视化）
 */
struct AssociatedSubgrid {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int subgrid_id;                   // 子网格ID
    int history_node_id;              // 关联的历史节点ID
    Eigen::Vector3f center;           // 子网格中心（前沿点的质心）
    Eigen::Vector3f bbox_min;         // 边界框最小点
    Eigen::Vector3f bbox_max;         // 边界框最大点
    int point_count;                  // 前沿点数量
    
    AssociatedSubgrid()
        : subgrid_id(-1), history_node_id(-1)
        , center(Eigen::Vector3f::Zero())
        , bbox_min(Eigen::Vector3f::Zero())
        , bbox_max(Eigen::Vector3f::Zero())
        , point_count(0) {}
};

/**
 * @brief 历史节点（借鉴EDEN算法）
 * 
 * 沿无人机轨迹采集的节点，用于：
 * 1. 将前沿区域关联到最近的历史节点
 * 2. 合并同一节点关联的相邻前沿区域
 */
struct HistoryNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int node_id;                                  // 节点ID
    Eigen::Vector3f position;                     // 节点位置
    float timestamp;                              // 创建时间戳
    std::vector<int> associated_subgrid_ids;      // 关联的子网格ID列表
    
    HistoryNode()
        : node_id(-1)
        , position(Eigen::Vector3f::Zero())
        , timestamp(0.0f) {}
    
    HistoryNode(int id, const Eigen::Vector3f& pos, float ts)
        : node_id(id), position(pos), timestamp(ts) {}
};

/**
 * @brief 前沿簇
 */
struct FrontierCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int cluster_id;                      // 簇ID
    int subgrid_id;                      // 所属子网格ID
    int history_node_id;                 // 关联的历史节点ID
    Eigen::Vector3f centroid;            // 簇中心
    Eigen::Vector3f normal;              // 法向量
    int point_count;                     // 点数
    float extent;                        // 范围（用于判断是否需要分裂）
    
    // 前沿点位置列表（用于可视化）
    std::vector<Eigen::Vector3f> points;
    
    FrontierCluster()
        : cluster_id(-1), subgrid_id(-1), history_node_id(-1)
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
    
    Eigen::Vector3f position;            // 视点位置（观察位置）
    Eigen::Vector3f target_center;       // 目标簇中心（被观察的前沿区域）
    float yaw;                           // 偏航角
    float pitch;                         // 俯仰角（通常为0）
    int cluster_id;                      // 目标簇ID
    float score;                         // 评分
    bool is_valid;
    
    ExplorationViewpoint() 
        : position(Eigen::Vector3f::Zero())
        , target_center(Eigen::Vector3f::Zero())
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
    float viewpoint_height = 1.0f;       // 视点基准高度（用于起飞检测）
    float viewpoint_height_min = 0.8f;   // 视点最小采样高度 [m]
    float viewpoint_height_max = 1.2f;   // 视点最大采样高度 [m]
    int viewpoint_height_num = 3;        // 高度采样数量（1=仅基准高度）
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
    float min_coverage_score = 0.3f;     // 最小覆盖评分（L1快速过滤阈值）
    float min_visibility_ratio = 0.3f;   // 最小可见性比例（L2可见性过滤阈值）
    
    // ========== 历史节点参数（借鉴EDEN） ==========
    float history_node_distance = 3.0f;  // 历史节点间隔距离 [m]
    float node_frontier_radius = 5.0f;   // 节点关联前沿区域的半径 [m]
    
    // ========== 前沿点过滤参数 ==========
    float ground_height = 0.0f;          // 地面高度 [m]
    float ground_offset = 0.1f;          // 地面偏移（过滤地面噪声）[m]
    
    // ========== 头顶区域过滤参数 ==========
    // 策略：在簇级别过滤，如果簇中心比无人机高且水平距离近，则认为是头顶区域
    float overhead_height_threshold = 0.5f;   // 头顶高度阈值：簇中心比无人机高多少算头顶 [m]
    float overhead_distance_threshold = 2.0f; // 头顶距离阈值：水平距离多近时过滤 [m]
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
     * 流程（借鉴EDEN算法）：
     * 1. 更新历史节点
     * 2. 获取前沿点并过滤（地面噪声+无人机周围）
     * 3. 将前沿区域关联到历史节点
     * 4. 同一节点的前沿区域合并聚类
     * 5. 分裂大簇
     * 6. 圆周采样生成视点
     * 7. 去重滤波
     * 
     * @param clusters 输出的前沿簇
     * @param viewpoints 输出的探索视点
     * @return 是否成功
     */
    bool generate(std::vector<FrontierCluster>& clusters,
        std::vector<ExplorationViewpoint>& viewpoints);
    
    const ViewpointGeneratorConfig& getConfig() const { return config_; }
    void setConfig(const ViewpointGeneratorConfig& config) { config_ = config; }
    
    // 获取历史节点（用于可视化）
    const std::vector<HistoryNode>& getHistoryNodes() const { return history_nodes_; }
    
    // 获取关联的子网格信息（用于可视化）
    const std::vector<AssociatedSubgrid>& getAssociatedSubgrids() const { return associated_subgrids_; }
    
    // ==================== 路径规划接口 ====================
    
    /**
     * @brief 检查两点之间是否有障碍物（用于判断是否需要绕行）
     * 
     * @param start 起点
     * @param end 终点
     * @return true 如果有障碍物遮挡，false 如果直线畅通
     */
    bool hasObstacleBetween(const Eigen::Vector3f& start, const Eigen::Vector3f& end);
    
    /**
     * @brief 通过历史节点查找安全路径
     * 
     * 算法：
     * 1. 找到距离起点最近的历史节点作为起始节点
     * 2. 找到距离终点最近的历史节点作为目标节点
     * 3. 使用 BFS 在历史节点之间搜索路径
     * 4. 简化路径（移除不必要的中间点）
     * 
     * @param start 起点（当前无人机位置）
     * @param end 终点（目标视点）
     * @param waypoints 输出的中间航点（不包含起点和终点）
     * @return true 如果找到路径，false 如果无法找到
     */
    bool findPathViaHistoryNodes(const Eigen::Vector3f& start,
                                  const Eigen::Vector3f& end,
                                  std::vector<Eigen::Vector3f>& waypoints);
    
    /**
     * @brief 估算从起点到终点的实际路径长度
     * 
     * 用于视点选择优先级，考虑是否需要历史节点回溯
     * 
     * @param start 起点位置
     * @param end 终点位置
     * @return 估算的路径长度（如果需要回溯会比直线距离大）
     *         返回 -1 表示无法到达
     */
    float estimatePathLength(const Eigen::Vector3f& start,
                             const Eigen::Vector3f& end);

private:
    // ==================== 历史节点管理 ====================
    
    /**
     * @brief 更新历史节点
     * 
     * 当无人机移动超过history_node_distance时，添加新的历史节点
     */
    void updateHistoryNodes();
    
    /**
     * @brief 将子网格关联到最近的历史节点
     * 
     * @param subgrid_center 子网格中心位置
     * @return 最近历史节点的ID，如果没有则返回-1
     */
    int findNearestHistoryNode(const Eigen::Vector3f& subgrid_center);
    
    // ==================== 前沿点过滤 ====================
    
    /**
     * @brief 过滤前沿点
     * 
     * 过滤条件：
     * 1. 地面噪声：高度 < ground_height + ground_offset
     * 
     * 注意：无人机头顶区域的过滤已移至簇级别（overhead_height_threshold）
     */
    void filterFrontierPoints(std::vector<Eigen::Vector3f>& points);
    
    /**
     * @brief 检查簇是否已被覆盖（即附近未知体素太少）
     * @param cluster 要检查的簇
     * @return true 如果簇已被覆盖，false 如果仍有足够未知体素
     */
    bool isClusterCovered(const FrontierCluster& cluster);
    
    // ==================== 核心算法 ====================
    
    /**
     * @brief 在历史节点关联的前沿区域内聚类
     * 
     * @param points 历史节点关联的所有前沿点
     * @param node_id 历史节点ID
     * @param clusters 输出的前沿簇
     */
    void clusterByHistoryNode(const std::vector<Eigen::Vector3f>& points,
                              int node_id,
                              std::vector<FrontierCluster>& clusters);
    
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
    
    /**
     * @brief Step 5: 视点Yaw优化
     * 
     * 对相邻视点进行聚类，优化每个簇内视点的yaw角
     * 避免相邻视点出现对射情况，确保规划平滑
     */
    void optimizeViewpointYaw(std::vector<ExplorationViewpoint>& viewpoints);
    
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
    
    // 历史节点系统
    std::vector<HistoryNode> history_nodes_;
    int next_node_id_ = 0;
    Eigen::Vector3f last_node_position_ = Eigen::Vector3f::Zero();
    bool first_node_created_ = false;
    
    // 子网格关联信息（用于可视化）
    std::vector<AssociatedSubgrid> associated_subgrids_;
};

} // namespace rog_map

#endif // FRONTIER_VIEWPOINT_GENERATOR_H
