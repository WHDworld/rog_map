/**
 * @file viewpoint_manager.h
 * @brief 视点生成管理器 - 5Hz定时器负责视点生成和维护
 * 
 * 功能：
 * 1. 从全局地图获取前沿点
 * 2. 基于历史节点关联进行前沿聚类
 * 3. 为每个簇生成最佳视点（只保留一个）
 * 4. 将视点同步到 HistoryGraph
 * 
 * 支持的策略（来自frontier_viewpoint_generator.cpp）：
 * - use_surface_frontier: 表面前沿聚类（基于法向量相似性）
 * - use_spherical_sampling: 球扇形均匀采样
 * - use_viewpoint_clustering: 增量视点聚类（生成锚点）
 * 
 * 设计原则：
 * - 与探索状态机解耦，定时独立运行
 * - 每个簇只保留一个最佳视点，避免冗余
 * - 与 HistoryGraph 协同维护视点状态
 */

#ifndef VIEWPOINT_MANAGER_H
#define VIEWPOINT_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <mutex>
#include <memory>
#include <random>

// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"
#include "rog_map/history_graph.h"

namespace rog_map {

using GlobalMap = star_mvf::GlobalMap;

// ==================== 数据结构 ====================

constexpr float PI = 3.14159265358979323846f;

/**
 * @brief 表面前沿点（带法向量）
 */
struct SurfaceFrontierPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    bool has_normal;
    
    SurfaceFrontierPoint() 
        : position(Eigen::Vector3f::Zero())
        , normal(Eigen::Vector3f::UnitX())
        , has_normal(false) {}
    
    explicit SurfaceFrontierPoint(const Eigen::Vector3f& pos)
        : position(pos)
        , normal(Eigen::Vector3f::UnitX())
        , has_normal(false) {}
};

/**
 * @brief 视点聚类（用于增量视点聚类策略）
 */
struct ViewpointClusterInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int cluster_id;
    Eigen::Vector3f centroid;
    float radius;
    Eigen::Vector3f anchor_entry;          // 入口锚点
    Eigen::Vector3f anchor_exit;           // 出口锚点
    Eigen::Vector3f principal_direction;   // 主方向
    std::vector<Eigen::Vector3f> viewpoint_positions;
    
    ViewpointClusterInfo()
        : cluster_id(-1)
        , centroid(Eigen::Vector3f::Zero())
        , radius(0.0f)
        , anchor_entry(Eigen::Vector3f::Zero())
        , anchor_exit(Eigen::Vector3f::Zero())
        , principal_direction(Eigen::Vector3f::UnitX()) {}
};

/**
 * @brief 前沿簇
 */
struct FrontierCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int cluster_id;                       // 簇ID
    int history_node_id;                  // 关联的历史节点ID
    int subgrid_id;                       // 子网格ID（兼容性）
    Eigen::Vector3f centroid;             // 质心
    Eigen::Vector3f normal;               // 主法向量（用于表面前沿）
    Eigen::Vector3f principal_direction;  // 主成分方向
    int point_count;                      // 点数
    float extent;                         // 范围大小
    std::vector<Eigen::Vector3f> points;  // 前沿点列表
    std::vector<SurfaceFrontierPoint> surface_points;  // 表面前沿点（带法向量）
    
    // 最佳视点（每个簇只保留一个）
    bool has_viewpoint;                   // 是否有有效视点
    Eigen::Vector3f viewpoint_position;   // 视点位置
    float viewpoint_yaw;                  // 视点朝向
    float viewpoint_score;                // 视点评分

    // 备选视点列表 (New feature for robustness)
    struct CandidateViewpoint {
        Eigen::Vector3f position;
        float yaw;
        float score;
    };
    std::vector<CandidateViewpoint> candidates;
    
    FrontierCluster()
        : cluster_id(-1)
        , history_node_id(-1)
        , subgrid_id(-1)
        , centroid(Eigen::Vector3f::Zero())
        , normal(Eigen::Vector3f::UnitX())
        , principal_direction(Eigen::Vector3f::UnitX())
        , point_count(0)
        , extent(0.0f)
        , has_viewpoint(false)
        , viewpoint_position(Eigen::Vector3f::Zero())
        , viewpoint_yaw(0.0f)
        , viewpoint_score(0.0f) {}
};

/**
 * @brief 配置参数
 */
struct ViewpointManagerConfig {
    // 更新频率
    float update_rate = 5.0f;             // Hz
    
    // 传感器参数
    float sensor_range_min = 0.5f;        // 最小感知距离 [m]
    float sensor_range_max = 5.0f;        // 最大感知距离 [m]
    float sensor_fov_h = 1.57f;           // 水平FOV [rad]
    float sensor_fov_v = 1.22f;           // 垂直FOV [rad]
    
    // 视点采样参数
    float viewpoint_height = 1.0f;        // 视点高度 [m]
    float viewpoint_height_min = 0.5f;    // 最小采样高度 [m]
    float viewpoint_height_max = 2.0f;    // 最大采样高度 [m]
    int viewpoint_height_num = 1;         // 高度采样数量
    float candidate_rmin = 2.0f;          // 最小采样半径 [m]
    float candidate_rmax = 5.0f;          // 最大采样半径 [m]
    int candidate_rnum = 3;               // 半径采样数量
    float candidate_dphi = 0.5f;          // 角度采样间隔 [rad]
    float optimal_distance = 3.0f;        // 最优观测距离 [m]
    
    // 前沿聚类参数
    float cluster_eps = 1.0f;             // 聚类半径 [m]
    int min_cluster_size = 10;            // 最小簇大小
    float cluster_split_threshold = 3.0f; // 簇分裂阈值 [m]
    int max_clusters = 30;                // 最大簇数
    
    // 安全参数
    float uav_radius = 0.35f;             // UAV半径 [m]
    float safety_margin = 0.2f;           // 安全边距 [m]
    
    // 滤波参数
    float min_viewpoint_distance = 0.8f; // 视点最小间距 [m]
    float min_coverage_score = 0.3f;     // 最小覆盖评分 [0,1]
    float min_visibility_ratio = 0.3f;   // 最小可见性比例 [0,1]
    
    // 前沿点过滤参数
    float frontier_height_min = 0.5f;     // 前沿最低高度 [m]
    float frontier_height_max = 2.5f;     // 前沿最高高度 [m]
    float ground_height = 0.0f;           // 地面高度 [m]
    float ground_offset = 0.1f;           // 地面偏移 [m]
    
    // 头顶区域过滤参数
    float overhead_height_threshold = 0.3f;    // 头顶高度阈值 [m]
    float overhead_distance_threshold = 3.0f;  // 头顶水平距离阈值 [m]

    // 策略开关
    bool use_surface_frontier = false;    // 是否使用表面前沿聚类（基于法向量相似性）
    bool use_spherical_sampling = false;  // 是否使用球扇形均匀采样（false则使用圆周采样）
    bool use_viewpoint_clustering = false;// 是否使用增量视点聚类（生成锚点）

    // 表面前沿聚类参数
    float normal_angle_threshold = 0.785f;// 法向量聚类角度阈值 [rad] (~45°)
    int normal_pca_k = 10;                // PCA计算法向量时的邻域点数

    // 球扇形采样参数
    float sphere_sample_radius = 2.5f;    // 球扇形采样半径 [m]
    float sphere_theta_max = 0.785f;      // 最大极角 [rad] (~45°)
    int sphere_sample_num = 20;           // 球扇形内采样点数

    // 增量视点聚类参数
    float vp_cluster_distance = 3.0f;     // 视点聚类距离阈值 [m]
    float vp_cluster_radius = 5.0f;       // 视点聚类外接圆半径阈值 [m]
    float vp_cluster_min_dist = 6.0f;     // 视点需要聚类的最小距离 [m]
    
    // 历史节点关联参数
    float node_frontier_radius = 20.0f;   // 节点关联前沿区域的最大半径 [m]
};

// ==================== 主类 ====================

class ViewpointManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ViewpointManager> Ptr;
    
    ViewpointManager();
    ~ViewpointManager() = default;
    
    /**
     * @brief 初始化
     * @param nh ROS节点句柄
     * @param global_map 全局地图指针
     * @param history_graph 历史节点图指针
     */
    void init(ros::NodeHandle& nh, GlobalMap::Ptr global_map, HistoryGraph::Ptr history_graph);
    
    /**
     * @brief 启动定时器
     */
    void start();
    
    /**
     * @brief 停止定时器
     */
    void stop();
    
    /**
     * @brief 更新UAV状态
     * @param position UAV位置
     * @param yaw UAV朝向
     */
    void updateUAVState(const Eigen::Vector3f& position, float yaw);
    
    // ==================== 数据访问 ====================
    
    /**
     * @brief 获取当前前沿簇列表
     */
    std::vector<FrontierCluster> getClusters() const;

    /**
     * @brief 获取当前有效视点数量
     */
    int getViewpointCount() const;
    
    /**
     * @brief 检查是否有有效视点
     */
    bool hasValidViewpoints() const;
    
    /**
     * @brief 获取配置
     */
    const ViewpointManagerConfig& getConfig() const { return config_; }
    
    /**
     * @brief 手动触发一次视点更新（用于调试）
     */
    void triggerUpdate();
    
    /**
     * @brief 检查是否有视点更新（供HistoryGraph调用）
     * @return 是否有更新
     */
    bool hasViewpointUpdates() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return viewpoints_updated_;
    }
    
    /**
     * @brief 清除视点更新标志（供HistoryGraph调用）
     */
    void clearViewpointUpdates() {
        std::lock_guard<std::mutex> lock(mutex_);
        viewpoints_updated_ = false;
    }
    
private:
    // ==================== 定时器回调 ====================
    
    void timerCallback(const ros::TimerEvent& e);
    
    // ==================== 核心算法 ====================
    
    /**
     * @brief 主更新函数
     */
    void update();
    
    /**
     * @brief 过滤前沿点
     * @param points 输入输出前沿点
     */
    void filterFrontierPoints(std::vector<Eigen::Vector3f>& points);
    
    /**
     * @brief 在历史节点关联的区域内聚类
     * @param points 前沿点
     * @param node_id 历史节点ID
     * @param clusters 输出簇
     */
    void clusterByHistoryNode(const std::vector<Eigen::Vector3f>& points,
                              int node_id,
                              std::vector<FrontierCluster>& clusters);
    
    /**
     * @brief 分裂大簇
     * @param clusters 输入输出簇列表
     */
    void splitLargeClusters(std::vector<FrontierCluster>& clusters);
    
    /**
     * @brief 为簇生成最佳视点
     * @param cluster 输入输出簇（会设置视点信息）
     * @return 是否成功生成视点
     */
    bool generateBestViewpoint(FrontierCluster& cluster);
    
    /**
     * @brief 圆周采样视点（原有策略，分层过滤）
     * @param cluster 目标簇
     * @param viewpoints 输出视点
     */
    void sampleViewpointsCircular(const FrontierCluster& cluster,
                                  std::vector<Eigen::Vector3f>& positions,
                                  std::vector<float>& yaws,
                                  std::vector<float>& scores);
    
    /**
     * @brief 球扇形采样视点（SFExplorer策略）
     * @param cluster 目标簇
     * @param viewpoints 输出视点
     */
    void sampleViewpointsSpherical(const FrontierCluster& cluster,
                                   std::vector<Eigen::Vector3f>& positions,
                                   std::vector<float>& yaws,
                                   std::vector<float>& scores);
    
    /**
     * @brief 评估视点
     * @param position 视点位置
     * @param cluster 目标簇
     * @return 评分，<0表示无效
     */
    float evaluateViewpoint(const Eigen::Vector3f& position, 
                            const FrontierCluster& cluster);
    
    /**
     * @brief 检查位置是否安全（严格）
     * @param pos 位置
     * @return 是否安全
     */
    bool isPositionSafe(const Eigen::Vector3f& pos);
    
    /**
     * @brief 检查位置是否安全（宽松，用于球扇形采样）
     * @param pos 位置
     * @return 是否安全
     */
    bool isPositionSafeForExploration(const Eigen::Vector3f& pos);
    
    /**
     * @brief 计算覆盖评分（考虑FOV边界）
     * @param viewpoint 视点位置
     * @param yaw 视点朝向
     * @param cluster 目标簇
     * @return 覆盖评分 [0,1]
     */
    float computeCoverageScore(const Eigen::Vector3f& viewpoint, float yaw, 
                               const FrontierCluster& cluster);
    
    // ==================== 表面前沿策略 ====================
    
    /**
     * @brief 基于法向量相似性聚类
     * @param surface_points 表面前沿点
     * @param clusters 输出簇
     */
    void clusterByNormalSimilarity(const std::vector<SurfaceFrontierPoint>& surface_points,
                                   std::vector<FrontierCluster>& clusters);
    
    /**
     * @brief 计算表面法向量（PCA）
     * @param point 前沿点位置
     * @param normal 输出法向量
     * @return 是否成功计算
     */
    bool computeSurfaceNormal(const Eigen::Vector3f& point, Eigen::Vector3f& normal);
    
    /**
     * @brief 计算簇的主成分法向量
     * @param cluster 输入输出簇
     */
    void computeClusterPrincipalNormal(FrontierCluster& cluster);
    
    // ==================== 效用和可见性计算 ====================
    
    /**
     * @brief 计算效用评分（SFExplorer公式）
     * @param viewpoint 视点位置
     * @param cluster 目标簇
     * @return 效用评分
     */
    float computeUtilityScore(const Eigen::Vector3f& viewpoint, const FrontierCluster& cluster);
    
    /**
     * @brief 计算可见性评分
     * @param viewpoint 视点位置
     * @param cluster 目标簇
     * @return 可见性比例 [0,1]
     */
    float computeVisibilityScore(const Eigen::Vector3f& viewpoint, const FrontierCluster& cluster);
    
    /**
     * @brief 检查射线可见性（起点到终点）
     * @param start 起点
     * @param end 终点
     * @return 是否可见（无障碍物）
     */
    bool checkRayVisibility(const Eigen::Vector3f& start, const Eigen::Vector3f& end);
    
    // ==================== 增量视点聚类 ====================
    
    /**
     * @brief 增量视点聚类
     * @param viewpoints 输入视点
     * @param unclustered 输出未聚类视点
     * @param clusters 输出视点聚类
     */
    void clusterViewpointsIncremental(const std::vector<FrontierCluster>& input_clusters,
                                      std::vector<FrontierCluster>& unclustered,
                                      std::vector<ViewpointClusterInfo>& vp_clusters);
    
    /**
     * @brief 生成锚点
     * @param cluster 视点聚类
     */
    void generateAnchorPoints(ViewpointClusterInfo& cluster);
    
    // ==================== 辅助函数 ====================
    
    /**
     * @brief 检查簇是否已被覆盖（未知体素太少）
     * @param cluster 簇
     * @return 是否已覆盖
     */
    bool isClusterCovered(const FrontierCluster& cluster);
    
    /**
     * @brief 优化视点Yaw角
     * @param clusters 簇列表
     */
    void optimizeViewpointYaw(std::vector<FrontierCluster>& clusters);
    
    /**
     * @brief 过滤头顶区域簇
     * @param clusters 簇列表
     */
    void filterOverheadClusters(std::vector<FrontierCluster>& clusters);

    FrontierCluster* getClusterById(int id);

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    GlobalMap::Ptr global_map_;
    HistoryGraph::Ptr history_graph_;
    ViewpointManagerConfig config_;
    
    // UAV状态
    Eigen::Vector3f uav_position_ = Eigen::Vector3f::Zero();
    float uav_yaw_ = 0.0f;
    bool has_uav_state_ = false;
    
    // 当前数据
    std::vector<FrontierCluster> current_clusters_;
    
    // 统计
    int total_clusters_ = 0;
    int total_viewpoints_ = 0;
    
    // 视点聚类结果（可选，用于增量聚类策略）
    std::vector<ViewpointClusterInfo> viewpoint_clusters_;
    int next_vp_cluster_id_ = 0;
    
    // 线程安全
    mutable std::mutex mutex_;
    mutable std::mutex uav_mutex_;
        
    // 簇ID计数器
    int next_cluster_id_ = 0;
    
    // 随机数生成器（用于球扇形采样）
    std::mt19937 rng_{std::random_device{}()};
    
    // 视点更新标志（供HistoryGraph轮询）
    bool viewpoints_updated_ = false;
};

} // namespace rog_map

#endif // VIEWPOINT_MANAGER_H
