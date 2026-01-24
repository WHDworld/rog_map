/**
 * @file semantic_target_extractor.h
 * @brief 语义目标提取器 - 基于探索率触发的增量式几何显著性检测
 * 
 * 核心功能：
 * 1. 监控子网格探索率，当探索完成后触发一次性几何分析
 * 2. 利用 PCA 分析区分墙面/地面（平面）和不规则物体（目标）
 * 3. 分析完成后永不再处理，极致效率
 * 4. 可视化已提取的潜在目标
 */

#ifndef ROG_MAP_SEMANTIC_TARGET_EXTRACTOR_H
#define ROG_MAP_SEMANTIC_TARGET_EXTRACTOR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include "star_mvf/map/global_map.h"
#include "star_mvf/map/map_types.h"

namespace rog_map {

// 使用 map_types.h 中定义的 SemanticTargetMetadata
using TargetMetadata = star_mvf::SemanticTargetMetadata;

// ========== 配置参数 ==========
struct SemanticExtractorConfig {
    // 探索率触发阈值
    float exploration_threshold = 0.85f;  // 探索率超过此值触发分析
    
    // PCA 分析参数
    int pca_check_interval = 20;        // 每生长多少个点检测一次
    float planarity_threshold = 0.85f;  // 平面度阈值 (>此值判为墙壁)
    float wall_size_threshold = 1.0f;   // 大于此尺寸的平面才判为墙壁 (m)
    
    // 混合 PCA 参数
    float local_pca_radius = 0.3f;      // 局部 PCA 邻域半径 (m)
    float local_planarity_threshold = 0.7f;  // 局部平面度阈值（低于此值为异常点）
    float hybrid_switch_threshold = 0.6f;    // 全局平面度低于此值时使用混合分析
    
    // 目标筛选参数
    int min_voxel_count = 20;           // 最小体素数
    float max_target_height = 2.5f;     // 目标最大高度 (m)
    float min_target_height = 0.3f;     // 目标最小高度 (m)
    float max_target_width = 1.5f;      // 目标最大宽度 (m)
    
    // 自清洁开关
    bool enable_self_cleaning = true;   // 是否将识别的墙面设为VISITED
    
    // 定时器频率
    float process_rate = 2.0f;          // 处理定时器频率 (Hz)
    float vis_rate = 1.0f;              // 可视化定时器频率 (Hz)
};

// ========== PCA 几何特征 ==========
struct ClusterPCAFeatures {
    Eigen::Vector3f eigenvalues;        // 特征值 (降序 λ1 >= λ2 >= λ3)
    Eigen::Matrix3f eigenvectors;       // 特征向量矩阵
    float planarity;                    // 平面度 2*(λ2-λ3)/(λ1+λ2+λ3)
    float sphericity;                   // 散乱度 λ3/λ1
    float linearity;                    // 线性度 (λ1-λ2)/λ1
    Eigen::Vector3f normal;             // 主法线 (对应 λ3 的特征向量)
    
    ClusterPCAFeatures() {
        eigenvalues.setZero();
        eigenvectors.setIdentity();
        planarity = 0.0f;
        sphericity = 0.0f;
        linearity = 0.0f;
        normal.setZero();
    }
};

// ========== 语义聚类 ==========
struct SemanticCluster {
    std::vector<star_mvf::VoxelIndex> voxels;
    
    Eigen::Vector3f centroid;
    Eigen::Vector3f bbox_min;
    Eigen::Vector3f bbox_max;
    
    ClusterPCAFeatures pca_features;
    
    bool is_valid = false;
    bool is_wall = false;
    
    float getHeight() const { return bbox_max.z() - bbox_min.z(); }
    float getWidth() const { 
        return std::max(bbox_max.x() - bbox_min.x(), 
                        bbox_max.y() - bbox_min.y()); 
    }
    float getMaxExtent() const {
        Eigen::Vector3f extent = bbox_max - bbox_min;
        return extent.maxCoeff();
    }
    
    SemanticCluster() {
        centroid.setZero();
        bbox_min.setConstant(std::numeric_limits<float>::max());
        bbox_max.setConstant(std::numeric_limits<float>::lowest());
    }
};

// ========== 核心类：语义目标提取器 ==========
class SemanticTargetExtractor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    SemanticTargetExtractor() = default;
    ~SemanticTargetExtractor() = default;
    
    /**
     * @brief 初始化提取器（带 NodeHandle 用于定时器和可视化）
     */
    void init(ros::NodeHandle& nh, star_mvf::GlobalMap* global_map);
    
    /**
     * @brief 从 ROS 参数服务器加载配置
     */
    void loadConfig(const ros::NodeHandle& nh, const std::string& ns = "semantic_extractor");
    
    /**
     * @brief 手动触发增量更新（也可由定时器自动调用）
     */
    void update();
    
    /**
     * @brief 获取全图所有已提取目标的元数据
     */
    void getAllTargets(std::vector<TargetMetadata>& out_targets) const;
    
    /**
     * @brief 获取指定范围内的目标
     */
    void getTargetsInRange(const Eigen::Vector3f& center, float radius,
                           std::vector<TargetMetadata>& out_targets) const;
    
    /**
     * @brief 根据种子点重新生成完整的 Cluster 几何
     */
    bool regenerateCluster(const TargetMetadata& meta, SemanticCluster& out_cluster);
    
    const SemanticExtractorConfig& getConfig() const { return config_; }
    
    int getTotalTargetCount() const;
    int getAnalyzedSubGridCount() const { return analyzed_count_; }
    int getPrunedWallCount() const { return pruned_wall_count_; }
    
private:
    // ========== 定时器回调 ==========
    void processTimerCallback(const ros::TimerEvent& event);
    void visualizationTimerCallback(const ros::TimerEvent& event);
    
    // ========== 可视化 ==========
    void publishTargetMarkers();
    std_msgs::ColorRGBA getColorForTarget(int target_id) const;
    
    // ========== 核心算法 ==========
    void analyzeSubGrid(star_mvf::SubGrid& sg);
    bool growRegion(const star_mvf::VoxelIndex& seed, SemanticCluster& cluster);
    
    void updatePCAStats(const Eigen::Vector3f& pos,
                        Eigen::Vector3f& sum_pos,
                        Eigen::Matrix3f& sum_sq,
                        int& count);
    
    void computePCAFeatures(const Eigen::Vector3f& sum_pos,
                            const Eigen::Matrix3f& sum_sq,
                            int count,
                            ClusterPCAFeatures& features);
    
    bool isWallOrFloor(const ClusterPCAFeatures& features, 
                       const SemanticCluster& cluster);
    
    bool isValidTarget(const SemanticCluster& cluster);
    
    void pruneWallVoxels(const SemanticCluster& cluster);
    
    // ========== 混合 PCA (贴墙目标检测) ==========
    /**
     * @brief 使用混合 PCA 方案分析簇
     * 全局平面度中等时，使用局部 PCA 筛选异常点
     * @param cluster 输入簇
     * @param anomaly_voxels 输出异常点（非平面点）
     * @return 是否找到足够的异常点可以形成目标
     */
    bool hybridPCAAnalysis(const SemanticCluster& cluster, 
                           std::vector<star_mvf::VoxelIndex>& anomaly_voxels);
    
    /**
     * @brief 计算单个体素的局部 PCA 特征
     */
    bool computeLocalPCA(const star_mvf::VoxelIndex& center, 
                         float radius, 
                         ClusterPCAFeatures& out_features);
    
    // ========== 防重机制 ==========
    bool isVisited(const star_mvf::VoxelIndex& idx) const;
    void markVisited(const star_mvf::VoxelIndex& idx);
    void resetVisitTags();
    
    // ========== 辅助函数 ==========
    void getNeighbors6(const star_mvf::VoxelIndex& idx, 
                       std::vector<star_mvf::VoxelIndex>& neighbors) const;
    
    bool isOccupiedUnobserved(const star_mvf::VoxelIndex& idx) const;
    
private:
    star_mvf::GlobalMap* global_map_ = nullptr;
    SemanticExtractorConfig config_;
    
    // ROS 接口
    ros::NodeHandle nh_;
    ros::Timer process_timer_;
    ros::Timer vis_timer_;
    ros::Publisher target_marker_pub_;
    std::string frame_id_ = "world";
    
    // 防重机制 (时间戳标记法)
    uint32_t current_visit_tag_ = 0;
    std::unordered_map<star_mvf::VoxelIndex, uint32_t, star_mvf::VoxelIndexHash> voxel_visit_tags_;
    
    // 统计信息
    int analyzed_count_ = 0;
    int pruned_wall_count_ = 0;
};

}  // namespace rog_map

#endif  // ROG_MAP_SEMANTIC_TARGET_EXTRACTOR_H
