#ifndef ROG_MAP_TARGET_VOXEL_GRID_H
#define ROG_MAP_TARGET_VOXEL_GRID_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rog_map {

/**
 * @brief 目标体素状态枚举（三态）
 * @note 命名为 TargetVoxelType 避免与 star_mvf::VoxelState 冲突
 */
enum class TargetVoxelType {
    UNKNOWN = 0,   // 未知（初始状态或目标内部）
    FREE = 1,      // 自由空间（射线穿过）
    OCCUPIED = 2   // 占据/表面（点云击中）
};

/**
 * @brief 单个体素的状态
 */
struct TargetVoxelState {
    TargetVoxelType state = TargetVoxelType::UNKNOWN;  // 体素状态
    int hit_count = 0;                                  // 击中次数
    
    TargetVoxelState() = default;
    
    // 便捷方法
    bool isSurface() const { return state == TargetVoxelType::OCCUPIED; }
    bool isFree() const { return state == TargetVoxelType::FREE; }
    bool isUnknown() const { return state == TargetVoxelType::UNKNOWN; }
    bool isWellScanned() const { return hit_count >= 3; }  // [新增] 高质量扫描判定
};

/**
 * @brief 目标体素网格
 * 
 * 替代八叉树的高效数据结构，使用3D数组直接索引，O(1)时间复杂度。
 * 专为小目标重建设计，重点关注：
 * - 表面体素检测
 * - 覆盖率统计
 * - 视线遮挡检测
 */
class TargetVoxelGrid {
public:
    typedef std::shared_ptr<TargetVoxelGrid> Ptr;
    
    /**
     * @brief 构造函数
     * @param center 目标中心
     * @param size 目标包围盒尺寸
     * @param resolution 体素分辨率（默认0.1m）
     */
    TargetVoxelGrid(const Eigen::Vector3d& center, 
                    const Eigen::Vector3d& size, 
                    double resolution = 0.1);
    
    ~TargetVoxelGrid() = default;
    
    // ==================== 增量更新接口 ====================
    
    /**
     * @brief 根据新点云更新体素网格
     * @param cloud 当前帧目标点云（世界坐标系）
     * @param sensor_pose 传感器位置（用于判断观测有效性）
     */
    void update(const pcl::PointCloud<pcl::PointXYZ>& cloud, 
                const Eigen::Vector3d& sensor_pose);
    
    // ==================== 覆盖率查询 ====================
    
    /**
     * @brief 获取当前覆盖率
     * @return 已观测表面体素数 / 总表面体素数
     */
    double getCoverageRatio() const;
    
    /**
     * @brief 获取表面体素总数（OCCUPIED 状态）
     */
    int getSurfaceVoxelCount() const { return occupied_count_; }
    
    /**
     * @brief [改进] 获取高质量扫描体素数 (hit_count >= 3)
     */
    int getWellScannedVoxelCount() const { return well_scanned_count_; }
    
    /**
     * @brief [新增] 获取理论表面体素数（基于 BBox 估算）
     */
    int getEstimatedSurfaceCount() const { return estimated_surface_count_; }
    
    // ==================== 覆盖计算（方案B核心） ====================
    
    /**
      * @brief 计算从给定视点能看到的未覆盖体素
     * @param vp_pos 视点位置
     * @param vp_yaw 视点Yaw角（无人机朝向）
     * @param camera_pitch 相机固定俯仰角
     * @param hfov 水平视场角（弧度）
     * @param vfov 垂直视场角（弧度）
     * @param max_range 最大观测距离
     * @param already_covered 已被其他视点覆盖的体素索引集合
     * @return 该视点能新增覆盖的体素索引集合
     */
    std::unordered_set<int> computeNewCoverage(
        const Eigen::Vector3d& vp_pos,
        double vp_yaw,
        double camera_pitch,
        double hfov,
        double vfov,
        double max_range,
        const std::unordered_set<int>& already_covered) const;
    
    /**
     * @brief [新增] 计算加权覆盖分数
     * @param vp_pos 视点位置
     * @param vp_yaw 视点Yaw角
     * @param camera_pitch 相机俯仰角
     * @return 加权分数（UNKNOWN=1, OCCUPIED未充分扫描=0.5, 已充分扫描=0.25）
     */
    double computeWeightedCoverageScore(
        const Eigen::Vector3d& vp_pos,
        double vp_yaw,
        double camera_pitch,
        double hfov,
        double vfov,
        double max_range) const;
    
    /**
     * @brief 获取所有表面体素的索引
     */
    std::vector<int> getSurfaceVoxelIndices() const;
    
    /**
     * @brief 获取所有未观测的表面体素索引
     */
    std::vector<int> getUnobservedVoxelIndices() const;
    
    /**
     * @brief [新增] 获取所有占据态体素的索引
     */
    std::vector<int> getOccupiedVoxelIndices() const;
    
    // ==================== 遮挡检测 ====================
    
    /**
     * @brief 检测从视点到目标点是否被遮挡（自遮挡）
     * @param from 视点位置
     * @param to 目标点位置
     * @return true=被遮挡，false=可见
     */
    bool isOccluded(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
    
    /**
     * @brief 检查某点是否在表面体素内
     */
    bool isSurface(const Eigen::Vector3d& point) const;
    
    // ==================== 几何查询 ====================
    
    /**
     * @brief 获取体素中心坐标
     */
    Eigen::Vector3d getVoxelCenter(int idx) const;
    
    /**
     * @brief 目标中心
     */
    Eigen::Vector3d getCenter() const { return center_; }
    
    /**
     * @brief 目标尺寸
     */
    Eigen::Vector3d getSize() const { return size_; }
    
private:
    // ==================== 坐标转换 ====================
    
    /**
     * @brief 世界坐标转体素索引
     * @return -1表示超出范围
     */
    int posToIndex(const Eigen::Vector3d& pos) const;
    
    /**
     * @brief 体素索引转世界坐标（体素中心）
     */
    Eigen::Vector3d indexToPos(int idx) const;
    
    /**
     * @brief 检查索引是否有效
     */
    bool isValidIndex(int idx) const { return idx >= 0 && idx < total_voxels_; }
    
    /**
     * @brief 检查点是否在网格范围内
     */
    bool isInBounds(const Eigen::Vector3d& pos) const;
    
    // ==================== FOV检查 ====================
    
    /**
     * @brief 检查点是否在视点的FOV内
     */
    bool isInFOV(const Eigen::Vector3d& vp_pos, double vp_yaw, double camera_pitch,
                 double hfov, double vfov, const Eigen::Vector3d& target_point) const;
    
private:
    // 网格数据
    std::vector<TargetVoxelState> voxels_;
    
    // 几何参数
    Eigen::Vector3d center_;       // 目标中心（世界坐标）
    Eigen::Vector3d size_;         // 目标尺寸
    Eigen::Vector3d origin_;       // 网格原点（左下角）
    double resolution_;            // 体素分辨率
    int nx_, ny_, nz_;             // 各维度体素数量
    int total_voxels_;             // 总体素数
    
    // 统计变量（O(1) 查询）
    int estimated_surface_count_ = 0; // 理论表面体素数（基于 BBox 估算）
    int free_count_ = 0;              // FREE 体素数
    int occupied_count_ = 0;          // OCCUPIED 体素数（实际发现的表面）
    int well_scanned_count_ = 0;      // 高质量扫描体素数 (hit_count >= 3)
    
    // 线程安全
    mutable std::mutex mutex_;
    
    // ==================== 私有辅助函数 ====================
    
    /**
     * @brief 射线投射：标记从 sensor 到 target 路径上的体素为 FREE
     */
    void raycastAndMarkFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    
    /**
     * @brief 判断 UNKNOWN 体素是否为内部空心（被 OCCUPIED 包围）
     */
    bool isInteriorUnknown(int idx) const;
    
    /**
     * @brief [新增] 在 FOV 内均匀采样射线方向
     */
    std::vector<Eigen::Vector3d> sampleRayDirections(
        double yaw, double pitch, double hfov, double vfov,
        int num_h, int num_v) const;
    
    /**
     * @brief [新增] 计算射线与目标 BBox 的交点
     * @return true=有交点，false=无交点
     */
    bool rayBBoxIntersection(
        const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
        Eigen::Vector3d& entry_point, Eigen::Vector3d& exit_point) const;
};

} // namespace rog_map

#endif // ROG_MAP_TARGET_VOXEL_GRID_H
