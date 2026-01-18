#include "rog_map/target_octree.h"
#include <pcl/features/normal_3d.h>
#include <cmath>

namespace rog_map {

TargetOctree::TargetOctree(const Eigen::Vector3d& center, const Eigen::Vector3d& size, double resolution) 
    : resolution_(resolution), min_size_(resolution)
{
    // BBox margin
    double margin = 0.5;
    bbox_center_ = center;
    Eigen::Vector3d half_size = size / 2.0;
    Eigen::Vector3d margin_vec(margin, margin, margin);
    
    // 确保八叉树能够覆盖包括margin在内的区域
    // 根节点尺寸需要是 2^k * resolution
    double max_dim = (size + margin_vec * 2.0).maxCoeff();
    double root_size = resolution;
    while (root_size < max_dim) root_size *= 2.0;
    
    root_ = new OctreeNode(center, root_size);
    
    bbox_min_ = center - half_size;
    bbox_max_ = center + half_size;
}

TargetOctree::~TargetOctree() {
    if (root_) delete root_;
}

void TargetOctree::update(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3d& sensor_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (cloud.empty()) return;

    // 1. 计算点云法向量 (简单估计，指向传感器)
    // 实际项目中应使用 PCA 或 MLS，这里简化为了效率
    for (const auto& pt : cloud.points) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d n = (sensor_pose - p).normalized(); // 简化的法向估计：指向观测者
        insertPoint(root_, p, n);
    }
    
    // 2. 光线投射更新观测状态 (Raycasting)
    // 遍历所有 Occupied 节点，检查是否在当前视角下可见
    // 这里简化处理：如果在 FOV 内且距离合适，认为被观测
    // TODO: 真正的 Raycasting 需要遍历体素路径设置 Empty
    
    std::vector<OctreeNode*> surface_nodes = getSurfaceNodes();
    for (auto node : surface_nodes) {
        if (node->is_observed) continue; // 已观测则跳过
        
        Eigen::Vector3d dir = node->center - sensor_pose;
        double dist = dir.norm();
        
        // 简单可见性检查
        // 1. 距离检查
        if (dist > 10.0) continue; 
        
        // 2. 视锥检查 (假设相机)
        // 需传入相机的朝向，这里简略，假设只要距离够近且无遮挡
        
        // 3. 法向量检查
        // 观测角度应与法向量夹角 < 60度
        Eigen::Vector3d view_dir = -dir.normalized();
        if (view_dir.dot(node->normal) > 0.5) { // cos(60) = 0.5
            node->is_observed = true;
            node->observation_quality = std::min(1.0, node->observation_quality + 0.2); // 累积观测质量
        }
    }
}

void TargetOctree::insertPoint(OctreeNode* node, const Eigen::Vector3d& point, const Eigen::Vector3d& normal) {
    // 如果不在节点范围内（理论上根节点够大不会发生）
     if (std::abs(point.x() - node->center.x()) > node->size/2.0 ||
         std::abs(point.y() - node->center.y()) > node->size/2.0 ||
         std::abs(point.z() - node->center.z()) > node->size/2.0) {
         return;
     }

    // 递归终止条件：达到最小分辨率
    if (node->size <= min_size_) {
        node->is_occupied = true;
        node->occupancy_prob = std::min(1.0, node->occupancy_prob + 0.1); // 增加概率
        
        updateNodeNormal(node, normal);
        return;
    }
    
    // 寻找子节点
    int octant = getOctant(point, node->center);
    if (!node->children[octant]) {
        double child_size = node->size / 2.0;
        Eigen::Vector3d child_center = getChildCenter(node->center, child_size, octant);
        node->children[octant] = new OctreeNode(child_center, child_size);
        node->children[octant]->parent = node;
    }
    
    insertPoint(node->children[octant], point, normal);
    
    // 更新父节点状态 (只要有子节点占据，父节点也标记为占据)
    node->is_occupied = true;
}

void TargetOctree::updateNodeNormal(OctreeNode* node, const Eigen::Vector3d& new_normal) {
    node->normal_accum += new_normal;
    node->hit_count++;
    node->normal = node->normal_accum.normalized();
}

std::vector<OctreeNode*> TargetOctree::getSurfaceNodes() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<OctreeNode*> nodes;
    getSurfaceNodesRecursive(root_, nodes);
    return nodes;
}

void TargetOctree::getSurfaceNodesRecursive(OctreeNode* node, std::vector<OctreeNode*>& nodes) {
    if (!node) return;
    
    if (node->isLeaf()) {
        if (node->is_occupied) {
            nodes.push_back(node);
        }
        return;
    }
    
    for (auto child : node->children) {
        if (child) getSurfaceNodesRecursive(child, nodes);
    }
}

std::vector<OctreeNode*> TargetOctree::getUnobservedFrontiers() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<OctreeNode*> nodes;
    getUnobservedFrontiersRecursive(root_, nodes);
    return nodes;
}

void TargetOctree::getUnobservedFrontiersRecursive(OctreeNode* node, std::vector<OctreeNode*>& nodes) {
    if (!node) return;
    
    if (node->isLeaf()) {
        // 条件：是表面，但未观测，或质量低
        // Prior-to-Real: 如果是未知区域但被认为可能是表面（BBox内），也算
        // 这里简化：只处理已探测到的表面
        if (node->is_occupied && !node->is_observed) {
            nodes.push_back(node);
        }
        return;
    }
    
    for (auto child : node->children) {
        if (child) getUnobservedFrontiersRecursive(child, nodes);
    }
}

Eigen::Vector3d TargetOctree::getGuideNormal(const OctreeNode* node) const {
    // Prior-to-Real 策略核心
    if (node->hit_count > 3) { // 有一定观测信度
        return node->normal;
    } else {
        return getBBoxNormal(node->center); // 代理法向量
    }
}

Eigen::Vector3d TargetOctree::getBBoxNormal(const Eigen::Vector3d& point) const {
    // 计算点到BBox六个面的距离，返回最近面的法向量
    double dist_min_x = std::abs(point.x() - bbox_min_.x());
    double dist_max_x = std::abs(point.x() - bbox_max_.x());
    double dist_min_y = std::abs(point.y() - bbox_min_.y());
    double dist_max_y = std::abs(point.y() - bbox_max_.y());
    double dist_min_z = std::abs(point.z() - bbox_min_.z());
    double dist_max_z = std::abs(point.z() - bbox_max_.z());
    
    double min_dist = dist_min_x;
    Eigen::Vector3d normal(-1, 0, 0); // min x
    
    if (dist_max_x < min_dist) { min_dist = dist_max_x; normal = Eigen::Vector3d(1, 0, 0); }
    if (dist_min_y < min_dist) { min_dist = dist_min_y; normal = Eigen::Vector3d(0, -1, 0); }
    if (dist_max_y < min_dist) { min_dist = dist_max_y; normal = Eigen::Vector3d(0, 1, 0); }
    if (dist_min_z < min_dist) { min_dist = dist_min_z; normal = Eigen::Vector3d(0, 0, -1); }
    if (dist_max_z < min_dist) { min_dist = dist_max_z; normal = Eigen::Vector3d(0, 0, 1); }
    
    return normal;
}

double TargetOctree::getCoverageRatio() const {
    // 简单计算：已观测叶子节点 / 总占据叶子节点
    // 注意：这里的 total 需要真正遍历统计，而不是累加成员变量，因为动态变化
    // 为了效率，我们可以维护成员变量，但在 update 中小心维护
    // 这里采用重新遍历统计
    
    // int total = 0; // Unused
    int observed = 0;
    
    // 我们需要一个 const 版本的遍历
    // 暂时用成员变量近似，但在 update 中并没有完全准确维护 total_surface_nodes_
    // 重新写一个简单的统计遍历
    std::vector<OctreeNode*> nodes;
    TargetOctree* nc_this = const_cast<TargetOctree*>(this); // Hack for reusing non-const helper
    nc_this->getSurfaceNodesRecursive(root_, nodes);
    
    if (nodes.empty()) return 0.0;
    
    for (auto n : nodes) {
        if (n->is_observed) observed++;
    }
    return (double)observed / nodes.size();
}

void TargetOctree::getVisualizationData(std::vector<Eigen::Vector3d>& occupied_centers,
                        std::vector<Eigen::Vector3d>& observed_centers,
                        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& normals) 
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<OctreeNode*> nodes;
    getSurfaceNodesRecursive(root_, nodes);
    
    for (auto n : nodes) {
        if (n->is_observed) {
            observed_centers.push_back(n->center);
        } else {
            occupied_centers.push_back(n->center);
        }
        
        // 可视化法向量 (只显示有一定置信度的)
        if (n->hit_count > 0) {
            normals.push_back({n->center, n->center + n->normal * 0.2});
        }
    }
}

bool TargetOctree::isOccluded(const Eigen::Vector3d& viewpoint, const Eigen::Vector3d& target) const {
    // 射线遮挡检测：从viewpoint到target，检查中途是否有其他占据节点
    // 使用简化的DDA (Digital Differential Analyzer) 算法
    
    Eigen::Vector3d dir = target - viewpoint;
    double total_dist = dir.norm();
    if (total_dist < 1e-6) return false;
    
    Eigen::Vector3d step = dir.normalized() * min_size_;  // 步长为最小分辨率
    int num_steps = static_cast<int>(total_dist / min_size_);
    
    // 沿射线逐步检测
    for (int i = 1; i < num_steps; ++i) {  // 从1开始，跳过起点
        Eigen::Vector3d sample_pt = viewpoint + step * i;
        
        // 检查该点是否接近终点（终点本身不算遮挡）
        if ((sample_pt - target).norm() < min_size_ * 1.5) break;
        
        // 查找该点所在的八叉树节点
        OctreeNode* node = root_;
        while (node && !node->isLeaf()) {
            // 找到包含sample_pt的子节点
            bool found = false;
            for (auto child : node->children) {
                if (!child) continue;
                double half = child->size / 2.0;
                if (std::abs(sample_pt.x() - child->center.x()) <= half &&
                    std::abs(sample_pt.y() - child->center.y()) <= half &&
                    std::abs(sample_pt.z() - child->center.z()) <= half) {
                    node = child;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        }
        
        // 如果找到占据节点，且该节点不是目标点附近，则被遮挡
        if (node && node->is_occupied && node->isLeaf()) {
            // 确保不是目标点本身附近的节点
            if ((node->center - target).norm() > min_size_ * 2.0) {
                return true;  // 被遮挡
            }
        }
    }
    
    return false;  // 可见
}

// 辅助函数
int TargetOctree::getOctant(const Eigen::Vector3d& point, const Eigen::Vector3d& center) {
    int octant = 0;
    if (point.x() >= center.x()) octant |= 1;
    if (point.y() >= center.y()) octant |= 2;
    if (point.z() >= center.z()) octant |= 4;
    return octant;
}

Eigen::Vector3d TargetOctree::getChildCenter(const Eigen::Vector3d& parent_center, double child_size, int octant) {
    Eigen::Vector3d center = parent_center;
    double offset = child_size / 2.0;
    
    center.x() += (octant & 1) ? offset : -offset;
    center.y() += (octant & 2) ? offset : -offset;
    center.z() += (octant & 4) ? offset : -offset;
    return center;
}

} // namespace rog_map
