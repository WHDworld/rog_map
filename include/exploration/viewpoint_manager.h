/**
 * @file viewpoint_manager.h
 * @brief 全局前沿聚类与视点生成（FUEL 风格，无历史节点依赖）
 *
 * 更新流程（固定频率定时器）：
 * 1. 从 GlobalMap 获取全部前沿子网格；未变化的子网格复用点云缓存
 * 2. 合并所有前沿点，建立 VoxelIndex -> point_index 映射
 * 3. 26 邻域体素区域生长，每个体素只处理一次
 * 4. splitLargeClusters() 分裂过大的簇
 * 5. 探索增益 + NMS + top-K 过滤
 * 6. 圆周/多高度采样 + 安全 + 可见性 + 覆盖率 + 评分，输出 top-K 候选视点
 * 7. matchClustersGlobal() 以质心距离/包围盒重叠/点数比例继承全局簇 ID
 */

#ifndef VIEWPOINT_MANAGER_H
#define VIEWPOINT_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <atomic>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <unordered_map>
#include <vector>

#include "map/star_mvf.h"
#include "exploration/exploration_task.h"

namespace rog_map {

using GlobalMap = star_mvf::GlobalMap;

constexpr float PI = 3.14159265358979323846f;

/**
 * @brief 全局前沿簇
 *
 * 只保留调度与可视化需要的数据；簇 ID 由 matchClustersGlobal() 统一分配。
 */
struct FrontierCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int cluster_id = -1;                  // 全局稳定簇 ID
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();
    int point_count = 0;
    float extent = 0.0f;
    std::vector<Eigen::Vector3f> points;

    // 最佳视点（候选列表首项，保持可视化兼容）
    bool has_viewpoint = false;
    Eigen::Vector3f viewpoint_position = Eigen::Vector3f::Zero();
    float viewpoint_yaw = 0.0f;
    float viewpoint_score = 0.0f;

    // 探索增益（外扩未知体素数量）
    float exploration_gain = 0.0f;

    // 按评分降序的 top-K 候选视点
    std::vector<ExplorationViewpoint> candidates;

    FrontierCluster() {
        bbox_min.setConstant(std::numeric_limits<float>::max());
        bbox_max.setConstant(std::numeric_limits<float>::lowest());
    }
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
    int min_cluster_size = 10;            // 最小簇大小
    float cluster_split_threshold = 3.0f; // 簇分裂阈值 [m]
    float cluster_match_dist = 1.0f;      // 全局 ID 继承距离阈值 [m]
    int max_clusters = 30;                // 最大输出簇数
    int gain_max_expand_step = 6;         // 增益计算的未知空间 BFS 深度

    // 安全参数
    float uav_radius = 0.35f;             // UAV半径 [m]
    float safety_margin = 0.2f;           // 安全边距 [m]

    // 滤波参数
    float min_viewpoint_distance = 0.8f; // 候选视点最小间距 [m]
    float min_coverage_score = 0.3f;     // 最小覆盖评分 [0,1]
    float min_visibility_ratio = 0.3f;   // 最小可见性比例 [0,1]

    // 前沿点过滤参数
    float frontier_height_min = 0.5f;     // 前沿最低高度 [m]
    float frontier_height_max = 2.5f;     // 前沿最高高度 [m]

    // 头顶区域过滤参数
    float overhead_height_threshold = 0.3f;    // 头顶高度阈值 [m]
    float overhead_distance_threshold = 3.0f;  // 头顶水平距离阈值 [m]

    // 候选视点输出参数（来自 global_scheduler）
    int top_view_num = 5;                 // 每个任务保留的候选视点数
    float max_decay = 0.8f;               // 候选保留阈值 score >= max_decay * best
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
     */
    void init(ros::NodeHandle& nh, GlobalMap::Ptr global_map);

    /**
     * @brief 启动定时器
     */
    void start();

    /**
     * @brief 更新UAV状态
     */
    void updateUAVState(const Eigen::Vector3f& position, float yaw);

    // ==================== 数据访问 ====================

    std::vector<FrontierCluster> getClusters() const;
    int getViewpointCount() const;
    bool hasValidViewpoints() const;
    void triggerUpdate();
    const ViewpointManagerConfig& getConfig() const { return config_; }

private:
    // 单元测试可访问聚类实现与配置
    friend class ViewpointManagerClusterPerf;

    void timerCallback(const ros::TimerEvent& e);

    /**
     * @brief 主更新函数
     */
    void update();

    /**
     * @brief 全局 26 邻域体素区域生长聚类
     * @param points 全部前沿点
     * @param clusters 输出簇
     */
    void clusterFrontiers(const std::vector<Eigen::Vector3f>& points,
                          std::vector<FrontierCluster>& clusters);

    /**
     * @brief 分裂过大的簇（按质心交替沿 X/Y 切分）
     */
    void splitLargeClusters(std::vector<FrontierCluster>& clusters);

    /**
     * @brief 全局 ID 继承匹配
     * @param new_clusters 新簇（会写入 cluster_id）
     * @param old_clusters 上一帧输出的簇
     */
    void matchClustersGlobal(std::vector<FrontierCluster>& new_clusters,
                             const std::vector<FrontierCluster>& old_clusters);

    /**
     * @brief 为簇生成 top-K 候选视点
     */
    bool generateBestViewpoint(FrontierCluster& cluster);

    /**
     * @brief 圆周/多高度采样，返回全部通过过滤的候选视点（按评分降序）
     */
    void sampleViewpointsCircular(const FrontierCluster& cluster,
                                  std::vector<Eigen::Vector3f>& positions,
                                  std::vector<float>& yaws,
                                  std::vector<float>& scores);

    bool isPositionSafe(const Eigen::Vector3f& pos);
    float computeCoverageScore(const Eigen::Vector3f& viewpoint, float yaw,
                               const FrontierCluster& cluster);
    float computeVisibilityScore(const Eigen::Vector3f& viewpoint,
                                 const FrontierCluster& cluster);
    bool checkRayVisibility(const Eigen::Vector3f& start, const Eigen::Vector3f& end);
    void filterOverheadClusters(std::vector<FrontierCluster>& clusters);
    float calculateClusterGain(const FrontierCluster& cluster);

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    GlobalMap::Ptr global_map_;
    ViewpointManagerConfig config_;

    // UAV状态
    Eigen::Vector3f uav_position_ = Eigen::Vector3f::Zero();
    float uav_yaw_ = 0.0f;
    bool has_uav_state_ = false;

    // 当前输出
    std::vector<FrontierCluster> current_clusters_;

    // 统计
    int total_clusters_ = 0;
    int total_viewpoints_ = 0;

    // 线程安全
    mutable std::mutex mutex_;
    mutable std::mutex uav_mutex_;
    std::mutex update_mutex_;   // 串行化定时器与显式触发

    // 全局簇 ID 计数器（仅新簇使用）
    std::atomic<int> global_cluster_id_{0};

    // 子网格前沿点缓存：未变化的子网格直接复用点云，避免重复遍历体素
    std::map<int, std::vector<Eigen::Vector3f>> subgrid_points_cache_;

    // 增益 BFS 的稠密 generation stamp（避免每簇分配 unordered_set）
    std::vector<uint32_t> gain_visit_stamp_;
    uint32_t gain_generation_ = 0;
};

} // namespace rog_map

#endif // VIEWPOINT_MANAGER_H
