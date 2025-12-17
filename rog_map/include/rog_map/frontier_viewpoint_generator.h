// frontier_viewpoint_generator.h
#ifndef FRONTIER_VIEWPOINT_GENERATOR_H
#define FRONTIER_VIEWPOINT_GENERATOR_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <algorithm>
#include <mutex>

#include "rog_map/global_map.h"

namespace rog_map {

// ==================== 常量定义 ====================
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;

// ==================== 数据结构定义 ====================

/**
 * @brief 5-DOF探索视点结构
 * @note EDEN策略：视点用于观测EROI内的未知区域，不依赖frontier点
 */
struct ExplorationViewpoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // 基本属性
    Eigen::Vector3f position;           // 3D位置
    float yaw;                          // 偏航角 [-π, π)
    float pitch;                        // 俯仰角 [-π/2, π/2]
    int subgrid_id;                     // 所属子网格(EROI)线性索引
    
    // 评分指标
    float exploration_gain;             // 探索信息增益
    float motion_cost;                  // 运动成本（时间）
    float curvature;                    // 到达该点的轨迹曲率
    float virtual_velocity;             // 曲率惩罚后的虚拟速度
    float total_score;                  // 综合评分
    
    // 可见性信息（EDEN：只关注未知体素，不依赖frontier）
    int visible_unknown_count;          // 可观测的未知体素数
    int covered_voxels_count;           // 总覆盖体素数（未知区域）
    
    // 状态标记
    bool is_valid;                      // 是否为有效视点
    bool is_active;                     // 引力模型中是否活跃
    
    // 连接信息
    std::vector<int> visible_viewpoint_indices;  // 可见的其他视点索引
    
    ExplorationViewpoint() 
        : position(Eigen::Vector3f::Zero())
        , yaw(0.0f)
        , pitch(0.0f)
        , subgrid_id(-1)
        , exploration_gain(0.0f)
        , motion_cost(0.0f)
        , curvature(0.0f)
        , virtual_velocity(0.0f)
        , total_score(0.0f)
        , visible_unknown_count(0)
        , covered_voxels_count(0)
        , is_valid(false)
        , is_active(true)
    {}
};

/**
 * @brief 视点对结构（探索视点 + 连续视点）
 */
struct ViewpointPair {
    ExplorationViewpoint vp_explore;    // 探索视点
    ExplorationViewpoint vp_continuous; // 连续视点
    float explore_cost;                 // 探索成本
    float continuous_cost;              // 连续成本
    float pair_score;                   // 视点对评分
    bool is_feasible;                   // 连线是否可行（无障碍）
    
    ViewpointPair()
        : explore_cost(0.0f)
        , continuous_cost(0.0f)
        , pair_score(0.0f)
        , is_feasible(false)
    {}
};

/**
 * @brief 三视点组合结构（探索 + 连续 + 安全）
 */
struct ViewpointTriplet {
    ExplorationViewpoint vp_e;          // 探索视点
    ExplorationViewpoint vp_c;          // 连续视点
    ExplorationViewpoint vp_s;          // 安全视点
    float triplet_score;                // 组合评分
    
    ViewpointTriplet()
        : triplet_score(0.0f)
    {}
};

/**
 * @brief 骨架点结构（基于FC-Planner思想）
 */
struct SkeletonPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3f position;           // 骨架点位置
    float distance_to_boundary;         // 到边界的距离
    std::vector<int> neighbor_indices;  // 相邻骨架点索引
    bool is_junction;                   // 是否为分支节点
    
    SkeletonPoint()
        : position(Eigen::Vector3f::Zero())
        , distance_to_boundary(0.0f)
        , is_junction(false)
    {}
};

/**
 * @brief 自由空间骨架结构
 */
struct FreeSpaceSkeleton {
    std::vector<SkeletonPoint> points;  // 骨架点集合
    std::vector<std::pair<int, int>> edges;  // 骨架边（连接关系）
    bool is_valid;                      // 骨架是否有效
    
    FreeSpaceSkeleton() : is_valid(false) {}
    
    void clear() {
        points.clear();
        edges.clear();
        is_valid = false;
    }
};

/**
 * @brief 视点生成器参数配置
 */
struct ViewpointGeneratorConfig {
    // 传感器参数
    float sensor_range_min;             // 传感器最小有效距离 (m)
    float sensor_range_max;             // 传感器最大有效距离 (m)
    float sensor_fov_h;                 // 水平FOV (rad)
    float sensor_fov_v;                 // 垂直FOV (rad)
    float optimal_viewing_distance;     // 最佳观测距离 (m)
    
    // 运动约束
    float max_velocity;                 // 最大线速度 (m/s)
    float max_acceleration;             // 最大加速度 (m/s²)
    float max_yaw_rate;                 // 最大偏航角速度 (rad/s)
    float uav_radius;                   // 无人机碰撞半径 (m)
    
    // 视点采样参数
    float sampling_resolution;          // 采样分辨率 (m)
    int max_viewpoints_per_subgrid;     // 每个子网格最大视点数
    float min_viewpoint_distance;       // 视点最小间距 (m)
    
    // 曲率惩罚参数
    float curvature_penalty_lambda;     // 曲率惩罚系数 λ_a
    float anchor_time;                  // 锚点时间参数 (s)
    
    // 评分参数
    float exploration_weight_lambda;    // 探索-连续权重 λ_e
    float gain_weight;                  // 信息增益权重
    float cost_weight;                  // 成本权重
    
    // 引力模型参数
    float gravitation_radius_factor;    // 引力半径因子
    int max_gravitation_iterations;     // 最大引力迭代次数
    float merge_distance_threshold;     // 合并距离阈值 (m)
    
    // 安全参数
    float safety_margin;                // 安全边距 (m)
    float min_esdf_distance;            // 最小ESDF距离 (m)
    
    // 骨架引导采样参数（基于FC-Planner）
    bool enable_skeleton_sampling;      // 是否启用骨架引导采样
    float skeleton_distance_threshold;  // 骨架点最小边界距离阈值 (m)
    int min_skeleton_points;            // 最少骨架点数
    float skeleton_sampling_offset;     // 骨架采样偏移距离 (m)
    
    // 默认构造函数
    ViewpointGeneratorConfig()
        : sensor_range_min(0.3f)
        , sensor_range_max(5.0f)
        , sensor_fov_h(1.57f)           // 90°
        , sensor_fov_v(1.22f)           // 70°
        , optimal_viewing_distance(3.0f)
        , max_velocity(2.0f)
        , max_acceleration(3.0f)
        , max_yaw_rate(1.57f)           // 90°/s
        , uav_radius(0.35f)
        , sampling_resolution(0.5f)
        , max_viewpoints_per_subgrid(30)
        , min_viewpoint_distance(0.5f)
        , curvature_penalty_lambda(0.6f)
        , anchor_time(1.0f)
        , exploration_weight_lambda(0.3f)
        , gain_weight(1.0f)
        , cost_weight(1.0f)
        , gravitation_radius_factor(2.0f)
        , max_gravitation_iterations(50)
        , merge_distance_threshold(0.3f)
        , safety_margin(0.2f)
        , min_esdf_distance(0.3f)
        , enable_skeleton_sampling(true)
        , skeleton_distance_threshold(0.5f)
        , min_skeleton_points(3)
        , skeleton_sampling_offset(1.0f)
    {}
};

/**
 * @brief UAV状态结构
 */
struct UAVState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3f position;           // 当前位置
    Eigen::Vector3f velocity;           // 当前速度
    Eigen::Vector3f acceleration;       // 当前加速度
    float yaw;                          // 当前偏航角
    float yaw_rate;                     // 当前偏航角速度
    
    UAVState()
        : position(Eigen::Vector3f::Zero())
        , velocity(Eigen::Vector3f::Zero())
        , acceleration(Eigen::Vector3f::Zero())
        , yaw(0.0f)
        , yaw_rate(0.0f)
    {}
};

// ==================== 主类定义 ====================

/**
 * @brief 基于EDEN的EROI视点生成器
 * 
 * 重要说明：
 * - 类名保留"Frontier"是历史原因，实际实现采用EDEN策略
 * - EDEN策略：不依赖传统Frontier点，在EROI自由空间内直接采样
 * 
 * 主要功能：
 * 1. 在EROI(SubGrid)的自由空间内采样候选视点（不依赖frontier！）
 * 2. 使用Dijkstra验证视点可达性
 * 3. 使用曲率惩罚评估运动成本
 * 4. 选择探索-连续-安全视点组合
 */
class FrontierViewpointGenerator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<FrontierViewpointGenerator> Ptr;
    
    /**
     * @brief 构造函数
     * @param global_map 全局地图指针
     * @param config 配置参数
     */
    FrontierViewpointGenerator(rog_map::GlobalMap::Ptr global_map, 
                               const ViewpointGeneratorConfig& config);
    
    /**
     * @brief 默认析构函数
     */
    ~FrontierViewpointGenerator() = default;
    
    // ==================== 核心接口 ====================
    
    /**
     * @brief 更新UAV状态
     * @param state 新的UAV状态
     */
    void updateUAVState(const UAVState& state);
    
    /**
     * @brief 从指定子网格集合生成候选视点
     * @param subgrid_indices 子网格索引集合
     * @param candidate_viewpoints 输出的候选视点
     */
    void generateCandidateViewpoints(
        const std::unordered_set<int>& subgrid_indices,
        std::vector<ExplorationViewpoint>& candidate_viewpoints);
    
    /**
     * @brief 从首个路由区域生成候选视点（EDEN风格）
     * @param first_region_subgrid_id 首个路由区域的子网格ID
     * @param candidate_viewpoints 输出的候选视点
     */
    void generateCandidateViewpointsForRegion(
        int first_region_subgrid_id,
        std::vector<ExplorationViewpoint>& candidate_viewpoints);
    
    /**
     * @brief 评估所有候选视点并计算评分
     * @param candidate_viewpoints 候选视点（会被修改以填充评分）
     */
    void evaluateViewpoints(std::vector<ExplorationViewpoint>& candidate_viewpoints);
    
    /**
     * @brief 使用引力模型优化视点分布
     * @param viewpoints 视点集合（会被修改）
     */
    void applyGravitationModel(std::vector<ExplorationViewpoint>& viewpoints);
    
    /**
     * @brief 选择最优视点对（探索视点 + 连续视点）
     * @param candidate_viewpoints 候选视点集合
     * @param best_pair 输出的最优视点对
     * @return 是否找到有效的视点对
     */
    bool selectBestViewpointPair(
        const std::vector<ExplorationViewpoint>& candidate_viewpoints,
        ViewpointPair& best_pair);
    
    /**
     * @brief 选择最优三视点组合（探索 + 连续 + 安全）
     * @param candidate_viewpoints 候选视点集合
     * @param best_triplet 输出的最优三视点组合
     * @return 是否找到有效的三视点组合
     */
    bool selectBestViewpointTriplet(
        const std::vector<ExplorationViewpoint>& candidate_viewpoints,
        ViewpointTriplet& best_triplet);
    
    /**
     * @brief 一站式接口：生成并选择最优视点组合
     * @param frontier_subgrids 包含frontier的子网格索引
     * @param best_triplet 输出的最优三视点组合
     * @return 是否成功
     */
    bool generateAndSelectViewpoints(
        const std::unordered_set<int>& frontier_subgrids,
        ViewpointTriplet& best_triplet);
    
    // ==================== 查询接口 ====================
    
    /**
     * @brief 获取当前所有候选视点
     */
    const std::vector<ExplorationViewpoint>& getCandidateViewpoints() const {
        return candidate_viewpoints_;
    }
    
    /**
     * @brief 获取优化后的视点
     */
    const std::vector<ExplorationViewpoint>& getOptimizedViewpoints() const {
        return optimized_viewpoints_;
    }
    
    /**
     * @brief 获取配置参数
     */
    const ViewpointGeneratorConfig& getConfig() const {
        return config_;
    }
    
    /**
     * @brief 设置配置参数
     */
    void setConfig(const ViewpointGeneratorConfig& config) {
        config_ = config;
    }

private:
    // ==================== 私有成员函数 ====================
    
    // --- 视点采样相关（基于EDEN的EROI自由空间采样） ---
    
    /**
     * @brief 从单个子网格(EROI)的自由空间生成候选视点
     * @param subgrid_id 子网格线性索引
     * @param viewpoints 输出视点
     * @note EDEN策略：在EROI的自由空间内直接采样，不依赖frontier
     */
    void generateViewpointsFromSubGrid(
        int subgrid_id, 
        std::vector<ExplorationViewpoint>& viewpoints);
    
    /**
     * @brief 在EROI的自由空间内均匀采样候选位置
     * @param subgrid 子网格引用
     * @param sample_positions 输出的采样位置
     * @note EDEN核心策略：不从frontier采样，而是在自由空间内直接采样
     */
    void sampleFreeSpaceInEROI(
        const rog_map::SubGrid& subgrid,
        std::vector<Eigen::Vector3f>& sample_positions);
    
    // --- 骨架引导采样相关（基于FC-Planner思想） ---
    
    /**
     * @brief 从EROI的自由空间提取拓扑骨架
     * @param subgrid 子网格引用
     * @param skeleton 输出的骨架结构
     * @note 基于距离场的中轴提取方法
     */
    void extractFreeSpaceSkeleton(
        const rog_map::SubGrid& subgrid,
        FreeSpaceSkeleton& skeleton);
    
    /**
     * @brief 计算自由体素到边界的距离场
     * @param subgrid 子网格引用
     * @param distance_field 输出的距离场（体素索引->距离）
     */
    void computeDistanceField(
        const rog_map::SubGrid& subgrid,
        std::unordered_map<rog_map::GlobalVoxelIndex, float, 
                          rog_map::GlobalVoxelIndexHash>& distance_field);
    
    /**
     * @brief 从骨架点生成视点采样射线
     * @param skeleton_point 骨架点
     * @param subgrid 子网格引用
     * @param sample_positions 输出的采样位置
     * @note 类似FC-Planner的骨架射线采样
     */
    void generateSkeletonSamplingRays(
        const SkeletonPoint& skeleton_point,
        const rog_map::SubGrid& subgrid,
        std::vector<Eigen::Vector3f>& sample_positions);
    
    /**
     * @brief 使用骨架引导采样生成候选位置
     * @param subgrid 子网格引用
     * @param sample_positions 输出的采样位置
     */
    void sampleFromSkeleton(
        const rog_map::SubGrid& subgrid,
        std::vector<Eigen::Vector3f>& sample_positions);
    
    /**
     * @brief 融合均匀采样和骨架采样的结果
     * @param uniform_samples 均匀采样结果
     * @param skeleton_samples 骨架采样结果
     * @param merged_samples 融合后的采样结果
     */
    void mergeSamplingResults(
        const std::vector<Eigen::Vector3f>& uniform_samples,
        const std::vector<Eigen::Vector3f>& skeleton_samples,
        std::vector<Eigen::Vector3f>& merged_samples);
    
    /**
     * @brief 使用Dijkstra搜索验证视点的可达性
     * @param viewpoint_pos 候选视点位置
     * @param path_length 输出的路径长度
     * @return 是否可达
     * @note EDEN要求：视点需通过Dijkstra路径从UAV当前位置可达
     */
    bool verifyReachabilityDijkstra(
        const Eigen::Vector3f& viewpoint_pos,
        float& path_length);
    
    /**
     * @brief 计算视点在给定方向的可见未知体素数量
     * @param viewpoint_pos 视点位置
     * @param yaw 偏航角
     * @param pitch 俯仰角
     * @return 可见未知体素数量
     */
    int countVisibleUnknownVoxels(
        const Eigen::Vector3f& viewpoint_pos,
        float yaw,
        float pitch);
    
    /**
     * @brief 计算视点的最优朝向（指向未知区域重心）
     * @param viewpoint_pos 视点位置
     * @param subgrid 所属子网格
     * @param yaw 输出的偏航角
     * @param pitch 输出的俯仰角
     * @note 朝向计算：指向FOV内未知体素的重心方向
     */
    void computeOptimalOrientation(
        const Eigen::Vector3f& viewpoint_pos,
        const rog_map::SubGrid& subgrid,
        float& yaw,
        float& pitch);
    
    // --- 曲率与成本计算 ---
    
    /**
     * @brief 计算到达视点的虚拟圆弧轨迹曲率
     * @param viewpoint 目标视点
     * @return 曲率值
     */
    float computeCurvatureToViewpoint(const ExplorationViewpoint& viewpoint);
    
    /**
     * @brief 计算曲率惩罚后的虚拟速度
     * @param curvature 轨迹曲率
     * @return 虚拟速度
     */
    float computeVirtualVelocity(float curvature);
    
    /**
     * @brief 计算探索成本（从当前位置到视点）
     * @param viewpoint 目标视点
     * @return 探索成本（时间）
     */
    float computeExplorationCost(const ExplorationViewpoint& viewpoint);
    
    /**
     * @brief 计算连续成本（从探索视点到连续视点）
     * @param vp_explore 探索视点
     * @param vp_continuous 连续视点
     * @return 连续成本（时间）
     */
    float computeContinuousCost(
        const ExplorationViewpoint& vp_explore,
        const ExplorationViewpoint& vp_continuous);
    
    /**
     * @brief 计算视点对评分
     * @param vp_explore 探索视点
     * @param vp_continuous 连续视点
     * @return 视点对评分
     */
    float computeViewpointPairScore(
        const ExplorationViewpoint& vp_explore,
        const ExplorationViewpoint& vp_continuous);
    
    // --- 信息增益计算 ---
    
    /**
     * @brief 计算视点的探索信息增益
     * @param viewpoint 视点（会被修改以填充可见性信息）
     */
    void computeExplorationGain(ExplorationViewpoint& viewpoint);
    
    /**
     * @brief 执行视点可见性射线投射
     * @param viewpoint_pos 视点位置
     * @param viewpoint_yaw 视点偏航角
     * @param viewpoint_pitch 视点俯仰角
     * @param visible_unknown_count 输出的可见未知体素数
     * @note EDEN策略：只统计未知体素，不依赖frontier
     */
    void performVisibilityRaycasting(
        const Eigen::Vector3f& viewpoint_pos,
        float viewpoint_yaw,
        float viewpoint_pitch,
        int& visible_unknown_count);
    
    // --- 引力模型相关 ---
    
    /**
     * @brief 计算引力模型的查询半径
     * @return 查询半径
     */
    float computeGravitationRadius();
    
    /**
     * @brief 对单个视点应用引力更新
     * @param query_vp 查询视点（会被修改）
     * @param neighbor_vps 邻域视点
     */
    void applyGravitationUpdate(
        ExplorationViewpoint& query_vp,
        const std::vector<ExplorationViewpoint>& neighbor_vps);
    
    /**
     * @brief 合并距离过近的视点
     * @param viewpoints 视点集合（会被修改）
     */
    void mergeCloseViewpoints(std::vector<ExplorationViewpoint>& viewpoints);
    
    // --- 安全视点选择 ---
    
    /**
     * @brief 寻找安全视点
     * @param vp_explore 探索视点
     * @param candidate_viewpoints 候选视点集合
     * @param vp_safety 输出的安全视点
     * @return 是否找到有效的安全视点
     */
    bool findSafetyViewpoint(
        const ExplorationViewpoint& vp_explore,
        const std::vector<ExplorationViewpoint>& candidate_viewpoints,
        ExplorationViewpoint& vp_safety);
    
    // --- 碰撞检测与可行性 ---
    
    /**
     * @brief 检查位置是否安全（无碰撞）
     * @param pos 位置
     * @return 是否安全
     */
    bool isPositionSafe(const Eigen::Vector3f& pos);
    
    /**
     * @brief 检查两点之间的连线是否可行（无障碍）
     * @param start 起点
     * @param end 终点
     * @return 是否可行
     */
    bool isLineFeasible(const Eigen::Vector3f& start, const Eigen::Vector3f& end);
    
    /**
     * @brief 检查视点对连线的可行性（EDEN约束：连线可穿过未知但不能穿过占用）
     * @param vp_explore 探索视点
     * @param vp_continuous 连续视点
     * @return 是否可行
     */
    bool isViewpointPairFeasible(
        const ExplorationViewpoint& vp_explore,
        const ExplorationViewpoint& vp_continuous);
    
    // --- 工具函数 ---
    
    /**
     * @brief 角度解缠（计算偏航角差）
     * @param yaw_target 目标偏航角
     * @param yaw_current 当前偏航角
     * @return 偏航角差 [-π, π)
     */
    float yawDifference(float yaw_target, float yaw_current);
    
    /**
     * @brief 归一化角度到 [-π, π)
     * @param angle 输入角度
     * @return 归一化后的角度
     */
    float normalizeAngle(float angle);
    
    /**
     * @brief 计算两点间的欧氏距离
     */
    float distance(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2);
    
    /**
     * @brief 计算路径长度（考虑障碍物绕行，简化为直线距离）
     */
    float pathLength(const Eigen::Vector3f& start, const Eigen::Vector3f& end);

private:
    // ==================== 成员变量 ====================
    
    // 外部依赖
    rog_map::GlobalMap::Ptr global_map_;
    
    // 配置
    ViewpointGeneratorConfig config_;
    
    // UAV状态
    UAVState uav_state_;
    std::mutex state_mutex_;
    
    // 视点缓存
    std::vector<ExplorationViewpoint> candidate_viewpoints_;
    std::vector<ExplorationViewpoint> optimized_viewpoints_;
    
    // 骨架缓存（基于FC-Planner思想）
    std::unordered_map<int, FreeSpaceSkeleton> skeleton_cache_;  // subgrid_id -> skeleton
    
    // 用于加速的空间索引（可选）
    // TODO: 可添加KD-Tree或八叉树索引
};

} // namespace rog_map

#endif // FRONTIER_VIEWPOINT_GENERATOR_H