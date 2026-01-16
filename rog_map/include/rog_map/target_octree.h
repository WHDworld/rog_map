#ifndef ROG_MAP_TARGET_OCTREE_H
#define ROG_MAP_TARGET_OCTREE_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rog_map {

// 八叉树节点结构
struct OctreeNode {
    Eigen::Vector3d center;
    double size;
    
    // 状态
    bool is_occupied = false;      // 是否为目标表面
    bool is_observed = false;      // 是否已被有效观测
    
    // 概率与质量
    double occupancy_prob = 0.5;   // 占据概率
    double observation_quality = 0.0; // 观测质量 (0~1)
    
    // 几何信息
    Eigen::Vector3d normal;        // 表面法向量 (归一化)
    int hit_count = 0;             // 击中次数 (用于计算平均法向量)
    Eigen::Vector3d normal_accum;  // 法向量累加 (用于平均)
    
    // 子节点 (8个)
    std::vector<OctreeNode*> children;
    OctreeNode* parent = nullptr;
    
    OctreeNode(const Eigen::Vector3d& c, double s) 
        : center(c), size(s), normal(0,0,0), normal_accum(0,0,0) {
        children.resize(8, nullptr);
    }
    
    ~OctreeNode() {
        for (auto child : children) {
            if (child) delete child;
        }
    }
    
    bool isLeaf() const {
        for (auto child : children) {
            if (child) return false;
        }
        return true;
    }
};

/**
 * @brief 目标八叉树概率地图
 * 支持增量更新、法向量估算、前沿提取
 */
class TargetOctree {
public:
    typedef std::shared_ptr<TargetOctree> Ptr;
    
    TargetOctree(const Eigen::Vector3d& center, const Eigen::Vector3d& size, double resolution);
    ~TargetOctree();
    
    /**
     * @brief 增量更新地图
     * @param cloud 当前帧点云 (世界系)
     * @param sensor_pose 传感器位姿 (世界系)
     */
    void update(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3d& sensor_pose);
    
    /**
     * @brief 获取所有表面节点 (用于评分)
     */
    std::vector<OctreeNode*> getSurfaceNodes();
    
    /**
     * @brief 获取未观测的前沿节点 (用于补全)
     */
    std::vector<OctreeNode*> getUnobservedFrontiers();
    
    /**
     * @brief 获取节点的引导法向量 (Prior-to-Real)
     * @param node 节点
     * @return 优先返回真实法向量，若不可信则返回 BBox 虚拟法向量
     */
    Eigen::Vector3d getGuideNormal(const OctreeNode* node) const;
    
    /**
     * @brief 获取覆盖率
     */
    double getCoverageRatio() const;
    
    /**
     * @brief 可视化辅助 (获取中心点和状态)
     */
    void getVisualizationData(std::vector<Eigen::Vector3d>& occupied_centers,
                            std::vector<Eigen::Vector3d>& observed_centers,
                            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& normals);

    // Bounding Box 信息
    Eigen::Vector3d bbox_min_;
    Eigen::Vector3d bbox_max_;
    Eigen::Vector3d bbox_center_;

private:
    void insertPoint(OctreeNode* node, const Eigen::Vector3d& point, const Eigen::Vector3d& normal);
    void raycastAndUpdate(OctreeNode* node, const Eigen::Vector3d& origin, const Eigen::Vector3d& end);
    void updateNodeNormal(OctreeNode* node, const Eigen::Vector3d& new_normal);
    
    // 递归辅助
    void getSurfaceNodesRecursive(OctreeNode* node, std::vector<OctreeNode*>& nodes);
    void getUnobservedFrontiersRecursive(OctreeNode* node, std::vector<OctreeNode*>& nodes);
    
    // BBox 法向量辅助
    Eigen::Vector3d getBBoxNormal(const Eigen::Vector3d& point) const;
    
    // 辅助计算
    int getOctant(const Eigen::Vector3d& point, const Eigen::Vector3d& center);
    Eigen::Vector3d getChildCenter(const Eigen::Vector3d& parent_center, double child_size, int octant);

private:
    OctreeNode* root_;
    double resolution_;
    double min_size_; // 最小叶子节点尺寸
    
    std::mutex mutex_;
    
    // 统计
    int total_surface_nodes_ = 0;
    int observed_surface_nodes_ = 0;
};

} // namespace rog_map

#endif // ROG_MAP_TARGET_OCTREE_H
