/**
 * @file history_graph.h
 * @brief 历史节点图管理器 - 维护历史节点和探索视点的连通性
 * 
 * 功能：
 * 1. 维护沿UAV轨迹采集的历史节点
 * 2. 构建历史节点之间的边（增量连通性）
 * 3. 构建历史节点与探索视点的边
 * 4. 提供路径查询接口（BFS搜索）
 * 
 * 设计原则（参考EDEN算法）：
 * - 历史节点连通性在创建时确定，不重新检测
 * - 探索视点只连接到一个最佳历史节点
 * - 已探索的视点会被标记删除
 */

#ifndef HISTORY_GRAPH_H
#define HISTORY_GRAPH_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <mutex>
#include <memory>

// 可视化
#include <visualization_msgs/MarkerArray.h>

// 可视化
#include <visualization_msgs/MarkerArray.h>

// 使用新的 STAR-MVF 地图系统
#include "star_mvf/star_mvf.h"

namespace rog_map {

using GlobalMap = star_mvf::GlobalMap;

// 前向声明
class ViewpointManager;

// ==================== 数据结构 ====================

/**
 * @brief 历史节点
 */
struct GraphNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int node_id;                          // 节点ID（唯一）
    Eigen::Vector3f position;             // 位置
    float timestamp;                      // 创建时间
    std::vector<int> neighbors;           // 邻居节点ID列表（双向）
    int parent_node_id;                   // 父节点ID（创建时从哪个节点飞来的）
    
    GraphNode()
        : node_id(-1)
        , position(Eigen::Vector3f::Zero())
        , timestamp(0.0f)
        , parent_node_id(-1) {}
    
    GraphNode(int id, const Eigen::Vector3f& pos, float ts, int parent = -1)
        : node_id(id), position(pos), timestamp(ts), parent_node_id(parent) {}
    
    void addNeighbor(int id) {
        if (std::find(neighbors.begin(), neighbors.end(), id) == neighbors.end()) {
            neighbors.push_back(id);
        }
    }
    
    bool hasNeighbor(int id) const {
        return std::find(neighbors.begin(), neighbors.end(), id) != neighbors.end();
    }
    
    void removeNeighbor(int id) {
        neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), id), neighbors.end());
    }
};

/**
 * @brief 探索视点节点（与历史节点图连接）
 */
struct ViewpointNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int viewpoint_id;                     // 视点ID（唯一）
    Eigen::Vector3f position;             // 位置
    float yaw;                            // 朝向
    int cluster_id;                       // 所属前沿簇ID
    int connected_history_node;           // 连接的历史节点ID（只连一个）
    float score;                          // 评分
    float timestamp;                      // 创建时间
    bool explored;                        // 是否已探索
    bool valid;                           // 是否有效（可能因前沿消失而失效）
    
    ViewpointNode()
        : viewpoint_id(-1)
        , position(Eigen::Vector3f::Zero())
        , yaw(0.0f)
        , cluster_id(-1)
        , connected_history_node(-1)
        , score(0.0f)
        , timestamp(0.0f)
        , explored(false)
        , valid(true) {}
};

/**
 * @brief 路径结果
 */
struct PathResult {
    bool success;                         // 是否找到路径
    std::vector<int> history_node_path;   // 历史节点路径（节点ID序列）
    std::vector<Eigen::Vector3f> waypoints; // 航点坐标序列
    float total_distance;                 // 总距离
    int target_viewpoint_id;              // 目标视点ID（如果目标是视点）
    
    PathResult() : success(false), total_distance(0.0f), target_viewpoint_id(-1) {}
};

/**
 * @brief 配置参数
 */
struct HistoryGraphConfig {
    float history_node_distance = 3.0f;   // 历史节点间隔距离 [m]
    float max_connection_distance = 8.0f; // 最大连接距离 [m]
    float raycast_step = 0.1f;            // 障碍物检测步长 [m]
    int consecutive_occupied_threshold = 2; // 连续占据体素数阈值（判断障碍物）
    float viewpoint_connection_radius = 10.0f; // 视点连接历史节点的搜索半径 [m]
};

// ==================== 主类 ====================

class HistoryGraph {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<HistoryGraph> Ptr;
    
    HistoryGraph();
    ~HistoryGraph() = default;
    
    /**
     * @brief 初始化
     * @param nh ROS节点句柄
     * @param global_map 全局地图指针
     */
    void init(ros::NodeHandle& nh, GlobalMap::Ptr global_map);
    
    /**
     * @brief 设置ViewpointManager指针（用于轮询视点更新）
     * @param vp_manager ViewpointManager指针
     */
    void setViewpointManager(ViewpointManager* vp_manager) {
        viewpoint_manager_ = vp_manager;
    }
    
    /**
     * @brief 启动定时器
     */
    void start();
    
    /**
     * @brief 停止定时器
     */
    void stop();
    
    // ==================== 历史节点管理 ====================
    
    /**
     * @brief 更新UAV位置，检查是否需要创建新历史节点
     * @param uav_pos 当前UAV位置
     * @return 如果创建了新节点，返回节点ID；否则返回-1
     */
    int updateUAVPosition(const Eigen::Vector3f& uav_pos);
    
    /**
     * @brief 获取所有历史节点
     */
    const std::vector<GraphNode>& getHistoryNodes() const { return history_nodes_; }
    
    /**
     * @brief 获取最近的历史节点
     * @param pos 查询位置
     * @return 最近历史节点的ID，如果没有则返回-1
     */
    int findNearestHistoryNode(const Eigen::Vector3f& pos) const;
    
    /**
     * @brief 获取最近的可达历史节点（无障碍物）
     * @param pos 查询位置
     * @return 最近可达历史节点的ID，如果没有则返回-1
     */
    int findNearestReachableHistoryNode(const Eigen::Vector3f& pos);
    
    // ==================== 探索视点管理 ====================
    
    /**
     * @brief 添加探索视点并连接到历史节点图
     * @param position 视点位置
     * @param yaw 朝向
     * @param cluster_id 所属簇ID
     * @param score 评分
     * @return 视点ID
     */
    int addViewpoint(const Eigen::Vector3f& position, float yaw, 
                     int cluster_id, float score);
    
    /**
     * @brief 更新视点列表（替换所有视点）
     * @param viewpoints 新的视点列表
     */
    void updateViewpoints(const std::vector<ViewpointNode>& viewpoints);
    
    /**
     * @brief 标记视点已探索
     * @param viewpoint_id 视点ID
     */
    void markViewpointExplored(int viewpoint_id);
    
    /**
     * @brief 标记视点失效（前沿已消失）
     * @param viewpoint_id 视点ID
     */
    void markViewpointInvalid(int viewpoint_id);
    
    /**
     * @brief 清除已探索和失效的视点
     */
    void cleanupViewpoints();
    
    /**
     * @brief 获取所有有效视点
     */
    std::vector<ViewpointNode> getValidViewpoints() const;
    
    /**
     * @brief 获取视点信息
     * @param viewpoint_id 视点ID
     * @param position 输出：视点位置
     * @param yaw 输出：视点朝向
     * @return 是否找到该视点
     */
    bool getViewpointInfo(int viewpoint_id, Eigen::Vector3f& position, float& yaw) const;
    
    /**
     * @brief 获取视点
     * @param viewpoint_id 视点ID
     * @return 视点指针，如果不存在返回nullptr
     */
    const ViewpointNode* getViewpoint(int viewpoint_id) const;
    
    // ==================== 路径查询 ====================
    
    /**
     * @brief 查找从当前位置到视点的路径
     * @param start_pos 起点位置（当前UAV位置）
     * @param viewpoint_id 目标视点ID
     * @return 路径结果
     */
    PathResult findPathToViewpoint(const Eigen::Vector3f& start_pos, int viewpoint_id);
    
    /**
     * @brief 查找从当前位置到目标位置的路径（通过历史节点）
     * @param start_pos 起点位置
     * @param end_pos 终点位置
     * @return 路径结果
     */
    PathResult findPath(const Eigen::Vector3f& start_pos, const Eigen::Vector3f& end_pos);
    
    /**
     * @brief 检查两点之间是否有障碍物
     * @param start 起点
     * @param end 终点
     * @return true 如果有障碍物
     */
    bool hasObstacleBetween(const Eigen::Vector3f& start, const Eigen::Vector3f& end);
    
    /**
     * @brief 估算路径长度（用于选择最佳视点）
     * @param start_pos 起点
     * @param viewpoint_id 视点ID
     * @return 估算距离，-1表示不可达
     */
    float estimatePathLength(const Eigen::Vector3f& start_pos, int viewpoint_id);
    
    // ==================== 可视化 ====================
    
    /**
     * @brief 获取历史节点数量
     */
    size_t getHistoryNodeCount() const { return history_nodes_.size(); }
    
    /**
     * @brief 获取视点数量
     */
    size_t getViewpointCount() const { return viewpoints_.size(); }
    
    /**
     * @brief 获取边数量
     */
    int getEdgeCount() const;
    
    /**
     * @brief 获取配置
     */
    const HistoryGraphConfig& getConfig() const { return config_; }
    
    /**
     * @brief 手动触发可视化发布
     */
    void publishVisualization();
    
private:
    // ==================== 定时器回调 ====================
    
    void timerCallback(const ros::TimerEvent& e);
    
    /**
     * @brief 可视化定时器回调
     */
    void visualizationTimerCallback(const ros::TimerEvent& e);
    
    /**
     * @brief 创建历史节点和边的可视化
     */
    void visualizeHistoryGraph(visualization_msgs::MarkerArray& markers);
    
    // ==================== 内部方法 ====================
    
    /**
     * @brief 创建新的历史节点
     * @param pos 位置
     * @param parent_id 父节点ID
     * @return 新节点ID
     */
    int createHistoryNode(const Eigen::Vector3f& pos, int parent_id);
    
    /**
     * @brief 尝试连接两个历史节点
     * @param node_a 节点A的ID
     * @param node_b 节点B的ID
     * @return 是否成功连接
     */
    bool tryConnectNodes(int node_a, int node_b);
    
    /**
     * @brief 连接视点到最佳历史节点
     * @param vp 视点
     * @return 连接的历史节点ID，-1表示失败
     */
    int connectViewpointToGraph(ViewpointNode& vp);
    
    /**
     * @brief BFS搜索路径
     * @param start_node_id 起始历史节点ID
     * @param end_node_id 目标历史节点ID
     * @param path 输出路径（节点ID序列）
     * @return 是否找到路径
     */
    bool bfsSearchPath(int start_node_id, int end_node_id, std::vector<int>& path);
    
    /**
     * @brief 简化路径（移除不必要的中间节点）
     * @param path 输入输出路径
     */
    void simplifyPath(std::vector<int>& path);

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Timer vis_timer_;                          // 可视化定时器
    GlobalMap::Ptr global_map_;
    HistoryGraphConfig config_;
    
    // 可视化发布器
    ros::Publisher history_graph_pub_;              // 历史节点图可视化
    
    // ViewpointManager指针（用于轮询视点更新）
    ViewpointManager* viewpoint_manager_ = nullptr;
    
    // 历史节点
    std::vector<GraphNode> history_nodes_;
    int next_node_id_ = 0;
    Eigen::Vector3f last_node_position_ = Eigen::Vector3f::Zero();
    bool first_node_created_ = false;
    int last_connected_node_idx_ = -1;  // UAV最后经过的历史节点索引
    
    // 探索视点
    std::unordered_map<int, ViewpointNode> viewpoints_;
    int next_viewpoint_id_ = 0;
    
    // 当前UAV位置
    Eigen::Vector3f current_uav_pos_ = Eigen::Vector3f::Zero();
    bool has_uav_pos_ = false;
    
    // 线程安全
    mutable std::mutex mutex_;
    
    // 运行状态
    bool running_ = false;
};

} // namespace rog_map

#endif // HISTORY_GRAPH_H
