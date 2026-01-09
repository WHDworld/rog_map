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
#include <Eigen/Eigenvalues>
#include <vector>
#include <mutex>
#include <algorithm>
#include <random>
#include <cmath>

// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"

namespace rog_map {

// GlobalMap 和 SubGridFrontierInfo 已通过 star_mvf.h 提供

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
    std::vector<int> neighbors;                   // 邻居节点ID列表（双向连接）
    int parent_node_id;                           // 创建时的父节点（从哪个节点飞来的）
    
    HistoryNode()
        : node_id(-1)
        , position(Eigen::Vector3f::Zero())
        , timestamp(0.0f)
        , parent_node_id(-1) {}
    
    HistoryNode(int id, const Eigen::Vector3f& pos, float ts)
        : node_id(id), position(pos), timestamp(ts), parent_node_id(-1) {}
    
    // 添加邻居（双向连接）
    void addNeighbor(int neighbor_id) {
        if (std::find(neighbors.begin(), neighbors.end(), neighbor_id) == neighbors.end()) {
            neighbors.push_back(neighbor_id);
        }
    }
    
    bool hasNeighbor(int neighbor_id) const {
        return std::find(neighbors.begin(), neighbors.end(), neighbor_id) != neighbors.end();
    }
};

/**
 * @brief 带法向量的表面前沿点（SFExplorer策略）
 */
struct SurfaceFrontierPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3f position;            // 位置
    Eigen::Vector3f normal;              // 表面法向量（指向未知区域）
    bool has_normal;                     // 是否成功计算法向量
    
    SurfaceFrontierPoint()
        : position(Eigen::Vector3f::Zero())
        , normal(Eigen::Vector3f::UnitZ())
        , has_normal(false) {}
        
    SurfaceFrontierPoint(const Eigen::Vector3f& pos)
        : position(pos)
        , normal(Eigen::Vector3f::UnitZ())
        , has_normal(false) {}
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
    Eigen::Vector3f normal;              // 法向量（主法线方向）
    Eigen::Vector3f principal_direction; // 主成分方向（用于锚点生成）
    int point_count;                     // 点数
    float extent;                        // 范围（用于判断是否需要分裂）
    
    // 前沿点位置列表（用于可视化）
    std::vector<Eigen::Vector3f> points;
    
    // 带法向量的前沿点列表（SFExplorer策略）
    std::vector<SurfaceFrontierPoint> surface_points;
    
    FrontierCluster()
        : cluster_id(-1), subgrid_id(-1), history_node_id(-1)
        , centroid(Eigen::Vector3f::Zero())
        , normal(Eigen::Vector3f::UnitZ())
        , principal_direction(Eigen::Vector3f::UnitX())
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
 * @brief 视点聚类（用于增量聚类和锚点生成，SFExplorer策略）
 */
struct ViewpointCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int cluster_id;                               // 聚类ID
    std::vector<ExplorationViewpoint> viewpoints; // 聚类内的视点
    Eigen::Vector3f centroid;                     // 聚类中心
    Eigen::Vector3f principal_direction;          // 主成分方向
    float radius;                                 // 外接圆半径
    
    // 锚点（入口和出口）
    Eigen::Vector3f anchor_entry;                 // 入口锚点
    Eigen::Vector3f anchor_exit;                  // 出口锚点
    
    ViewpointCluster()
        : cluster_id(-1)
        , centroid(Eigen::Vector3f::Zero())
        , principal_direction(Eigen::Vector3f::UnitX())
        , radius(0.0f)
        , anchor_entry(Eigen::Vector3f::Zero())
        , anchor_exit(Eigen::Vector3f::Zero()) {}
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
    // ========== 策略选择开关 ==========
    bool use_surface_frontier = false;    // 是否使用表面前沿（SFExplorer策略）
    bool use_spherical_sampling = false;  // 是否使用球扇形采样（SFExplorer策略）
    bool use_viewpoint_clustering = false;// 是否使用增量视点聚类（SFExplorer策略）
    
    // ========== 传感器参数 ==========
    float sensor_range_min = 0.5f;       // 最小感知距离 [m]
    float sensor_range_max = 5.0f;       // 最大感知距离 [m]
    float sensor_fov_h = 1.57f;          // 水平FOV [rad] (~90°)
    float sensor_fov_v = 1.22f;          // 垂直FOV [rad] (~70°)
    
    // ========== 视点采样参数（圆周采样 - 原有策略） ==========
    float viewpoint_height = 1.0f;       // 视点基准高度（用于起飞检测）
    float viewpoint_height_min = 0.8f;   // 视点最小采样高度 [m]
    float viewpoint_height_max = 1.2f;   // 视点最大采样高度 [m]
    int viewpoint_height_num = 3;        // 高度采样数量（1=仅基准高度）
    float candidate_rmin = 1.5f;         // 最小采样半径
    float candidate_rmax = 4.5f;         // 最大采样半径
    int candidate_rnum = 3;              // 半径采样数量
    float candidate_dphi = 0.5f;         // 角度采样间隔 [rad]
    float optimal_distance = 3.0f;       // 最优观测距离
    
    // ========== 球扇形采样参数（SFExplorer策略） ==========
    float sphere_sample_radius = 2.5f;    // 球扇形采样半径（传感器量程的一半）
    float sphere_theta_max = 0.785f;      // 最大极角 [rad] (~45°)
    int sphere_sample_num = 20;           // 球扇形内采样点数
    
    // ========== 前沿簇参数 ==========
    float cluster_eps = 1.0f;            // 聚类半径
    int min_cluster_size = 10;           // 最小簇大小
    float cluster_split_threshold = 3.0f;// 簇分裂阈值
    int max_clusters = 20;               // 最大簇数
    
    // ========== 表面前沿聚类参数（SFExplorer策略） ==========
    float normal_angle_threshold = 0.785f;// 法向量聚类角度阈值 [rad] (~45°)
    int normal_pca_k = 10;                // PCA计算法向量时的邻域点数
    
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
    float overhead_height_threshold = 0.5f;   // 头顶高度阈值 [m]
    float overhead_distance_threshold = 2.0f; // 头顶距离阈值 [m]
    
    // ========== 增量视点聚类参数（SFExplorer策略） ==========
    float vp_cluster_distance = 3.0f;     // 视点聚类距离阈值 [m]
    float vp_cluster_radius = 5.0f;       // 视点聚类外接圆半径阈值 [m]
    float vp_cluster_min_dist = 4.0f;     // 视点需要聚类的最小距离（近处不聚类）[m]
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
    
    // ==================== SFExplorer 策略函数 ====================
    
    /**
     * @brief 获取表面前沿点（SFExplorer策略）
     * 
     * 表面前沿定义：同时存在"占用态邻居"和"自由态邻居"的未知态体素
     * 与传统前沿（仅需自由邻居）不同，聚焦于环境表面信息
     * 
     * @param surface_frontiers 输出的带法向量的表面前沿点
     */
    void extractSurfaceFrontiers(std::vector<SurfaceFrontierPoint>& surface_frontiers);
    
    /**
     * @brief 计算表面前沿点的法向量（SFExplorer策略）
     * 
     * 使用 PCA 分析邻域点云，提取表面法向量（指向未知区域）
     * 
     * @param point 前沿点
     * @param normal 输出的法向量
     * @return 是否成功计算法向量
     */
    bool computeSurfaceNormal(const Eigen::Vector3f& point, Eigen::Vector3f& normal);
    
    /**
     * @brief 基于法向量相似性的区域生长聚类（SFExplorer策略）
     * 
     * 聚类条件：法向量夹角 < normal_angle_threshold
     * 
     * @param surface_points 带法向量的前沿点
     * @param clusters 输出的前沿簇
     */
    void clusterByNormalSimilarity(const std::vector<SurfaceFrontierPoint>& surface_points,
                                   std::vector<FrontierCluster>& clusters);
    
    /**
     * @brief 计算簇的主法线（SFExplorer策略）
     * 
     * 对簇内所有法向量做 PCA，提取主法线方向
     */
    void computeClusterPrincipalNormal(FrontierCluster& cluster);
    
    /**
     * @brief 球扇形区域均匀采样视点（SFExplorer策略）
     * 
     * 在"以簇质心为原点、主法线为z轴"的球坐标系中采样
     * 公式：r = r_s * u1^(1/3), θ = arccos(1 - u2*(1 - cos(θ_max))), φ = 2π*u3
     * 
     * @param cluster 前沿簇
     * @param viewpoints 输出的视点
     */
    void sampleViewpointsSpherical(const FrontierCluster& cluster,
                                   std::vector<ExplorationViewpoint>& viewpoints);
    
    /**
     * @brief SFExplorer 效用函数评估视点（SFExplorer策略）
     * 
     * U(vp) = Σ (n_k · (vp - f_k)) / (||n_k|| * ||vp - f_k||)
     * 综合考虑覆盖度（可见前沿数）和观测视角质量（法向量与视线夹角）
     * 
     * @param viewpoint 视点位置
     * @param cluster 目标簇
     * @return 效用值
     */
    float computeUtilityScore(const Eigen::Vector3f& viewpoint,
                              const FrontierCluster& cluster);
    
    /**
     * @brief 增量视点聚类（SFExplorer策略）
     * 
     * 对远距离视点聚类，用锚点替代聚类内视点，降低路径规划复杂度
     * 聚类条件：距离 < d_c，外接圆半径 < r_c，视点间无障碍物
     * 
     * @param viewpoints 输入视点
     * @param unclustered 输出未聚类的视点（近处视点）
     * @param clusters 输出视点聚类
     */
    void clusterViewpointsIncremental(const std::vector<ExplorationViewpoint>& viewpoints,
                                      std::vector<ExplorationViewpoint>& unclustered,
                                      std::vector<ViewpointCluster>& clusters);
    
    /**
     * @brief 生成视点聚类的锚点（SFExplorer策略）
     * 
     * 使用 PCA 提取聚类主方向，在质心两侧生成入口/出口锚点
     * ap_a = p_c - l_1*v_1, ap_b = p_c + l_1*v_1
     * 
     * @param cluster 视点聚类
     */
    void generateAnchorPoints(ViewpointCluster& cluster);
    
    // ==================== 辅助函数 ====================
    
    bool isPositionSafe(const Eigen::Vector3f& pos);
    bool isPositionFree(const Eigen::Vector3f& pos);
    bool isPositionSafeForExploration(const Eigen::Vector3f& pos);  // 用于球扇形采样的宽松检查
    
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
    int last_connected_node_idx_ = -1;  // 追踪 UAV 最后经过的节点索引
    
    // 子网格关联信息（用于可视化）
    std::vector<AssociatedSubgrid> associated_subgrids_;
    
    // SFExplorer: 增量视点聚类（跨帧持久化）
    std::vector<ViewpointCluster> viewpoint_clusters_;
    int next_vp_cluster_id_ = 0;
};

} // namespace rog_map

#endif // FRONTIER_VIEWPOINT_GENERATOR_H
