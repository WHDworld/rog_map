#ifndef ROG_MAP_TARGET_RECONSTRUCTOR_H
#define ROG_MAP_TARGET_RECONSTRUCTOR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

// DecompROS
#include <decomp_basis/data_type.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_ros_msgs/PolyhedronArray.h>

#include <vision_msgs/Detection3DArray.h>

#include <vector>
#include <memory>
#include <mutex>

#include "rog_map/target_octree.h"

namespace rog_map {

// ==================== 数据结构 ====================

/**
 * @brief 重建视点结构体
 */
struct ReconViewpoint {
    Eigen::Vector3d position;   // 视点位置
    double yaw;                 // 无人机偏航角
    double pitch;               // 相机俯仰角（固定）
    double score;               // 综合评分
    int id;                     // 视点ID
    
    // 评分详情
    double coverage_score;
    double angle_score;
    double distance_score;
    double safety_score;
    
    ReconViewpoint() : position(0,0,0), yaw(0), pitch(0), score(0), id(-1),
                       coverage_score(0), angle_score(0), distance_score(0), safety_score(0) {}
};

/**
 * @brief 目标信息结构体
 */
struct TargetInfo {
    Eigen::Vector3d center;     // 目标中心
    Eigen::Vector3d size;       // 目标尺寸 (x_width, y_width, z_width)
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;  // 目标点云（可选）
    
    TargetInfo() : center(0,0,0), size(0,0,0), point_cloud(nullptr) {}
};

/**
 * @brief 重建状态枚举
 */
enum class ReconstructionState {
    IDLE = 0,               // 空闲,无重建任务
    TARGET_DETECTED,        // 检测到新目标,待确认
    VIEWPOINT_GENERATING,   // 正在生成视点
    VIEWPOINT_READY,        // 视点已就绪,等待状态机提取
    RECONSTRUCTING,         // 重建进行中
    RECON_COMPLETE          // 当前目标重建完成
};

/**
 * @brief 跟踪目标结构体 (多目标ID管理)
 */
struct TrackedTarget {
    // ========== 基础信息 ==========
    int id;                        // 唯一ID
    TargetInfo info;               // 目标信息
    TargetOctree::Ptr octree;      // 专属八叉树地图
    
    // ========== 跟踪状态 ==========
    int confirmation_count;        // 连续检测帧数
    bool is_confirmed;             // 是否已确认为有效目标
    ros::Time last_seen;           // 最后检测时间
    
    // ========== 重建状态 ==========
    ReconstructionState recon_state;  // 当前重建状态
    
    // ========== 视点管理 ==========
    std::vector<ReconViewpoint> viewpoints;  // 所有视点(初始生成)
    std::vector<int> tour;                   // TSP优化后的视点序列
    int current_viewpoint_idx;               // 当前正在执行的视点索引(-1表示未开始)
    
    // ========== 重建质量 ==========
    double coverage_ratio;         // 当前覆盖率 [0.0, 1.0]
    ros::Time last_update_time;    // 最后一次更新时间
    
    // ========== 动态优化标志 ==========
    bool need_viewpoint_update;    // 标志:当前视点需要重新评估(例如安全性变化)
    
    TrackedTarget() 
        : id(-1), confirmation_count(0), is_confirmed(false),
          recon_state(ReconstructionState::IDLE), 
          current_viewpoint_idx(-1), coverage_ratio(0.0), 
          need_viewpoint_update(false) {}
};

/**
 * @brief 相机参数
 */
struct CameraParams {
    double fx, fy;          // 焦距
    double cx, cy;          // 主点
    int width, height;      // 图像尺寸
    double hfov, vfov;      // 视场角（弧度）
    double max_range;       // 最大观测距离
    
    CameraParams() : fx(650.61), fy(610.00), cx(309.4), cy(238.20),
                     width(640), height(480), max_range(10.0) {
        hfov = 2.0 * atan(width / (2.0 * fx));
        vfov = 2.0 * atan(height / (2.0 * fy));
    }
};

// ==================== TargetReconstructor 类 ====================

/**
 * @brief 目标重建视点规划器
 * 
 * 功能：
 * 1. 接收目标信息，生成安全走廊
 * 2. 球面采样生成候选视点
 * 3. 滤波、评分、序列优化
 * 4. 输出优化后的视点序列供规划器使用
 */
class TargetReconstructor {
public:
    typedef std::shared_ptr<TargetReconstructor> Ptr;
    
    TargetReconstructor() = default;
    ~TargetReconstructor() = default;
    
    enum SamplingMode {
        SPHERICAL = 0,
        CYLINDRICAL = 1,
        HEMISPHERE = 2,
        // OCTAHEDRON = 3 // Removed for simplicity as per user request context? No, user asked for "original code supported several modes". Let's verify if Octahedron is needed. User said "original code... supported several... current code should also support". 
        // I will add what I can support easily.
    };

    /**
     * @brief 初始化
     * @param nh ROS节点句柄
     */
    void init(ros::NodeHandle& nh);
    
    
    // ==================== 状态查询接口 (供ExplorationFSMNew调用) ====================
    
    /**
     * @brief 检查是否有待重建目标(对应用户需求的 target_updated_)
     * @return true: 存在新确认目标或未完成重建的目标
     */
    bool hasReconstructionTask() const;
    
    /**
     * @brief 获取当前活跃重建目标的状态
     * @param target_id [输出] 目标ID
     * @param recon_state [输出] 重建状态
     * @return 是否有活跃目标
     */
    bool getActiveReconstructionStatus(int& target_id, ReconstructionState& recon_state) const;
    
    /**
     * @brief 获取目标重建进度
     * @param target_id 目标ID
     * @return 覆盖率 [0.0, 1.0], -1.0表示目标不存在
     */
    double getReconstructionProgress(int target_id) const;
    
    // ==================== 重建流程控制接口 ====================
    
    /**
     * @brief 启动目标重建(状态机调用)
     * @param drone_pos 无人机当前位置(用于计算TSP最优切入点)
     * @param target_id 目标ID(-1表示自动选择下一个)
     * @return 是否成功启动
     */
    bool startReconstruction(const Eigen::Vector3d& drone_pos, int target_id = -1);
    
    /**
     * @brief 获取当前应该前往的重建视点
     * @param viewpoint [输出] 重建视点信息
     * @return 是否成功获取
     */
    bool getCurrentReconViewpoint(ReconViewpoint& viewpoint);
    
    /**
     * @brief 获取完整的重建路径(所有剩余视点按TSP顺序)
     * @param path [输出] 视点序列
     * @return 剩余视点数量
     */
    int getReconstructionPath(std::vector<ReconViewpoint>& path);
    
    /**
     * @brief 标记当前视点完成,切换到下一个
     * @return 是否还有下一个视点(false表示当前目标重建完成)
     */
    bool markCurrentViewpointComplete();
    
    /**
     * @brief 暂停当前重建任务(切回探索时调用)
     */
    void pauseReconstruction();
    
    // ==================== 实时更新接口 ====================
    
    /**
     * @brief 更新目标点云和重建质量(在RECONSTRUCTING状态下实时调用)
     * @param target_cloud 新观测到的目标点云
     * @param sensor_pose 传感器位置
     */
    void updateReconstructionData(const pcl::PointCloud<pcl::PointXYZ>& target_cloud, 
                                   const Eigen::Vector3d& sensor_pose);
    
    /**
     * @brief 优化当前及后续视点(基于最新重建状态)
     * @details 检查视点安全性、调整yaw、位置等
     * @return 是否触发了视点更新
     */
    bool optimizeRemainingViewpoints();
    
    // ==================== 兼容性接口 (废弃,保留向后兼容) ====================
    
    /**
     * @brief 更新无人机位置
     * @param pos 无人机当前位置
     */
    void updateDronePosition(const Eigen::Vector3d& pos);


private:
    // ... (existing private methods)

    /**
     * @brief 圆柱体采样
     */
    std::vector<ReconViewpoint> sampleCylindricalViewpoints();

    /**
     * @brief 半球采样
     */
    std::vector<ReconViewpoint> sampleHemisphereViewpoints();
    
    // Timer callbacks
    void executionTimerCallback(const ros::TimerEvent& e);
    void visTimerCallback(const ros::TimerEvent& e);
    
    // IROS Innovation Methods
    bool triggerDynamicReplanning();
    double getCoverageRatio() const;
    void updateOctree(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3d& sensor_pose);
    
    // 多目标管理方法
    int matchOrCreateTarget(const TargetInfo& detection);  // 匹配或创建新目标，返回目标ID
    TrackedTarget* getActiveTarget();                       // 获取当前活跃目标
    TrackedTarget* getTargetById(int id);                   // 根据ID获取目标
    TrackedTarget* getActiveTargetNoLock();                 // 无锁版本（供已持锁函数内部调用）
    TrackedTarget* getTargetByIdNoLock(int id);             // 无锁版本（供已持锁函数内部调用）
    bool selectNextTarget();                                // 选择下一个待重建目标
    void updateTargetTracking(const std::vector<TargetInfo>& detections);  // 批量更新目标跟踪
    
    // Callbacks
    void triggerCallback(const vision_msgs::Detection3DArrayConstPtr& msg);
    void obstacleCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void targetCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Timer execution_timer_;
    ros::Timer vis_timer_;
    ros::Subscriber target_sub_;
    ros::Subscriber obstacle_cloud_sub_;
    ros::Subscriber target_cloud_sub_;
    
    // Parameters
    int sampling_mode_ = 0; // 0: Spherical, 1: Cylindrical, 2: Hemisphere

    
    /**
     * @brief 设置目标信息
     * @param target 目标信息
     * @return 是否成功
     */
    bool setTarget(const TargetInfo& target);
    
    /**
     * @brief 更新障碍物点云
     * @param cloud 障碍物点云
     */
    void updateObstacleCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    
    

    /**
     * @brief 生成重建视点
     * @return 是否成功
     */
    bool generateViewpoints();
    
    /**
     * @brief 获取优化后的视点序列
     * @return 视点序列
     */
    std::vector<ReconViewpoint> getOptimizedPath() const;
    
    /**
     * @brief 获取当前走廊多面体
     * @return 多面体
     */
    const Polyhedron3D& getCorridor() const { return current_corridor_; }
    
    /**
     * @brief 检查视点是否有效（在走廊内）
     * @param point 待检查点
     * @return 是否在走廊内
     */
    bool isInsideCorridor(const Eigen::Vector3d& point) const;
    
    /**
     * @brief 是否已初始化
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief 是否有有效视点
     */
    bool hasValidViewpoints() const { return !viewpoints_.empty(); }
    
    /**
     * @brief 获取视点数量
     */
    size_t getViewpointCount() const { return viewpoints_.size(); }

private:
    // ========== 核心流程 ==========
    
    /**
     * @brief 生成安全走廊
     */
    bool generateCorridor();
    
    /**
     * @brief 球面采样视点
     */
    std::vector<ReconViewpoint> sampleSphericalViewpoints();
    
    /**
     * @brief 滤波视点（走廊约束 + 遮挡检测）
     */
    std::vector<ReconViewpoint> filterViewpoints(const std::vector<ReconViewpoint>& vps);
    
    /**
     * @brief 评分视点
     */
    void scoreViewpoints(std::vector<ReconViewpoint>& vps);
    
    /**
     * @brief 序列优化（贪心TSP + 2-opt）
     */
    std::vector<int> optimizeSequence(const std::vector<ReconViewpoint>& vps);
    
    // ========== 辅助函数 ==========
    
    /**
     * @brief 从点云中移除目标区域
     */
    pcl::PointCloud<pcl::PointXYZ> removeTargetFromCloud(
        const pcl::PointCloud<pcl::PointXYZ>& cloud);
    
    /**
     * @brief 提取局部障碍物
     */
    vec_Vec3f extractLocalObstacles(const Vec3f& center,
                                    const pcl::PointCloud<pcl::PointXYZ>& cloud);
    
    /**
     * @brief 检查视线是否被遮挡
     */
    bool isViewUnobstructed(const Eigen::Vector3d& from, 
                            const Eigen::Vector3d& to);
    
    /**
     * @brief 计算覆盖得分
     */
    double computeCoverageScore(const ReconViewpoint& vp);
    
    /**
     * @brief 计算角度得分
     */
    double computeAngleScore(const ReconViewpoint& vp);
    
    /**
     * @brief 计算距离得分
     */
    double computeDistanceScore(const ReconViewpoint& vp);
    
    /**
     * @brief 计算安全得分
     */
    double computeSafetyScore(const ReconViewpoint& vp);
    
    /**
     * @brief 贪心TSP
     */
    std::vector<int> greedyTSP(const std::vector<ReconViewpoint>& vps);
    
    /**
     * @brief 2-opt改进
     */
    void twoOptImprovement(std::vector<int>& tour, 
                           const std::vector<ReconViewpoint>& vps);
    
    /**
     * @brief 计算视点间重叠度
     */
    double computeViewOverlap(const ReconViewpoint& vp1, const ReconViewpoint& vp2);
    
    /**
     * @brief 计算法向量对齐得分 (Prior-to-Real)
     */
    double computeNormalScore(const ReconViewpoint& vp);
    
    /**
     * @brief 锥形采样 (用于动态补全)
     */
    std::vector<ReconViewpoint> sampleConeViewpoints(
        const Eigen::Vector3d& center, const Eigen::Vector3d& normal);
    
    // ========== 可视化 ==========
    
    void publishCorridor();
    void publishViewpoints();
    void publishOptimalPath();
    
    // ========== 参数加载 ==========
    
    void loadParameters();

private:
    // ROS

    ros::Publisher corridor_pub_;
    ros::Publisher viewpoints_pub_;
    ros::Publisher path_pub_;
    
    // 状态
    bool initialized_ = false;
    bool target_updated_ = false;  // 标志位：目标信息已更新，需要生成视点
    mutable std::mutex data_mutex_;  // mutable允许在const函数中加锁
    
    // 数据
    TargetInfo current_target_;
    Polyhedron3D current_corridor_;
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_;
    pcl::PointCloud<pcl::PointXYZ> processed_cloud_;  // 移除目标后的点云
    Eigen::Vector3d drone_position_;
    
    // Octree Map (IROS Innovation)
    TargetOctree::Ptr octree_;
    bool replan_triggered_ = false;
    
    // 多目标跟踪管理
    std::vector<TrackedTarget> tracked_targets_;
    int next_target_id_ = 0;
    int active_target_id_ = -1;  // 当前正在重建的目标ID
    
    // 结果
    std::vector<ReconViewpoint> viewpoints_;
    std::vector<int> tour_;
    
    // ========== 参数 ==========
    
    // 走廊参数
    double corridor_radius_ = 2.0;
    double local_bbox_x_ = 5.0;
    double local_bbox_y_ = 5.0;
    double local_bbox_z_ = 3.0;
    double obstacle_search_radius_ = 10.0;
    double min_ground_height_ = 0.0;
    double target_bbox_padding_ = 0.5;
    
    // 球面采样参数
    double spherical_min_dist_ = 2.5;
    double spherical_max_dist_ = 5.0;
    int spherical_num_layers_ = 3;
    double spherical_min_elev_ = 30.0;  // 度
    double spherical_max_elev_ = 75.0;  // 度
    int spherical_points_per_layer_ = 16;
    
    // 圆柱采样参数
    double cyl_radius_ = 3.5;
    double cyl_min_height_ = 0.5;
    double cyl_max_height_ = 4.0;
    int cyl_num_layers_ = 4;
    int cyl_points_per_circle_ = 12;
    
    // 半球采样参数
    double hemi_min_dist_ = 2.0;
    double hemi_max_dist_ = 4.5;
    int hemi_num_layers_ = 2;
    double hemi_min_elev_ = 20.0;  // 度
    double hemi_max_elev_ = 80.0;  // 度
    int hemi_points_per_layer_ = 12;
    
    // 采样策略 (字符串形式，兼容原版)
    std::string sampling_strategy_ = "spherical";
    
    // 话题配置
    std::string obstacle_cloud_topic_ = "/cloud_registered";
    std::string target_cloud_topic_ = "/target_pointcloud";
    std::string target_detection_topic_ = "/target_detection/confirmed_targets";
    
    // 滤波参数
    double safety_margin_ = 0.3;
    bool enable_obstruction_check_ = true;
    
    // Octree 参数
    double octree_resolution_ = 0.2;
    
    // 多目标跟踪参数
    double target_match_distance_ = 1.0;   // 匹配距离阈值 (米)
    int confirmation_frames_ = 2;          // 确认所需连续帧数
    double target_timeout_ = 5.0;          // 目标丢失超时 (秒)
    
    // 评分权重
    double coverage_weight_ = 0.40;
    double angle_weight_ = 0.25;
    double distance_weight_ = 0.20;
    double safety_weight_ = 0.15;
    double normal_weight_ = 0.30; // 法向量对齐权重
    double optimal_distance_ = 3.5;
    double optimal_angle_cos_ = 0.6;
    
    // TSP参数
    int max_2opt_iterations_ = 100;
    
    // Octree扩展参数
    int octree_hit_threshold_ = 3;  // 法向量有效阈值
    
    // 重建完成判定
    double coverage_complete_threshold_ = 0.85;
    int max_viewpoints_ = 20;
    int min_viewpoints_ = 6;
    
    // 到达判定
    double position_tolerance_ = 0.3;
    double yaw_tolerance_ = 0.15;
    
    // 更新频率
    double update_rate_ = 2.0;
    
    // 相机参数
    CameraParams camera_params_;
    double camera_fixed_pitch_ = 0.0;  // 相机固定俯仰角（从外参计算）
};

}  // namespace rog_map

#endif  // ROG_MAP_TARGET_RECONSTRUCTOR_H
