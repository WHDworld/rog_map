/**
 * @file gcopter_planner.h
 * @brief GCOPTER 轨迹规划器封装，集成到 ROG-Map
 */
#pragma once

// 重要：先包含 GCOPTER 头文件，避免与 rog_map 命名空间冲突
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "rog_map/rog_map.h"

#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <functional>
#include <ros/ros.h>

// 前向声明
namespace rog_map 
{
class ROGMap;
// Vec3f 已在 common_lib.hpp 中定义
}

namespace rog_map {

/**
 * @brief GCOPTER 轨迹规划器
 * 
 * 封装 GCOPTER 库，提供简洁的接口用于探索任务中的轨迹规划
 */
class GCOPTERPlanner 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // ==================== 配置结构体 ====================
    struct Config {
        // 动力学约束
        double max_vel = 2.0;            // 最大速度 [m/s]
        double max_acc = 2.0;            // 最大加速度 [m/s²]
        double max_body_rate = 2.1;      // 最大角速度 [rad/s]
        double max_tilt_angle = 1.05;    // 最大倾斜角 [rad]
        double min_thrust = 2.0;         // 最小推力 [N]
        double max_thrust = 12.0;        // 最大推力 [N]
        
        // 物理参数
        double vehicle_mass = 0.61;      // 无人机质量 [kg]
        double grav_acc = 9.8;           // 重力加速度 [m/s²]
        double horiz_drag = 0.7;         // 水平阻力系数
        double vert_drag = 0.8;          // 垂直阻力系数
        double paras_drag = 0.01;        // 寄生阻力
        double speed_eps = 0.0001;       // 速度平滑因子
        
        // 安全参数
        double safety_margin = 0.3;      // 安全裕度 [m]
        double dilate_radius = 0.5;      // 膨胀半径 [m]
        
        // 碰撞检查模式
        bool use_esdf = false;            // 是否使用 ESDF 进行碰撞检查（更快更准）
        bool treat_unknown_as_free = true; // 是否将未知区域视为自由（探索场景可设为 true）
        
        // 优化参数
        double weight_time = 20.0;       // 时间权重
        double smoothing_eps = 1.0e-2;   // 平滑因子
        int integral_intervs = 16;       // 积分分辨率
        double rel_cost_tol = 1.0e-5;    // 相对成本容差
        double timeout_rrt = 0.05;       // RRT* 超时时间 [s]
        
        // SFC 参数
        double sfc_progress = 7.0;       // 沿路径前进步长
        double sfc_range = 3.0;          // 搜索范围
    };
    
    // ==================== 规划结果 ====================
    struct PlanResult {
        bool success = false;
        Trajectory<5> trajectory;
        std::vector<Eigen::Vector3d> rrt_path;
        std::vector<Eigen::MatrixX4d> corridors;
        double total_time = 0.0;
        double cost = std::numeric_limits<double>::infinity();
        std::string error_msg;
    };

public:
    GCOPTERPlanner(const ros::NodeHandle& nh, ROGMap* rog_map, const std::string& name_space = "rog_map") : nh_(nh)
    {
        rog_map_ = rog_map;
    
        // 从 ROG-Map 获取地图边界
        if (rog_map_) {
            Vec3f local_origin = rog_map_->getLocalMapOrigin();
            Vec3f local_size = rog_map_->getLocalMapSize();
            map_lower_ = local_origin.cast<double>();
            map_upper_ = (local_origin + local_size).cast<double>();
        }
        LoadParam(name_space + "/planner/max_vel", config_.max_vel, 2.0);
        LoadParam(name_space + "/planner/max_acc", config_.max_acc, 2.0);
        LoadParam(name_space + "/planner/max_body_rate", config_.max_body_rate, 2.1);
        LoadParam(name_space + "/planner/max_tilt_angle", config_.max_tilt_angle, 1.05);
        LoadParam(name_space + "/planner/min_thrust", config_.min_thrust, 2.0);
        LoadParam(name_space + "/planner/max_thrust", config_.max_thrust, 12.0);
        LoadParam(name_space + "/planner/vehicle_mass", config_.vehicle_mass, 0.61);
        LoadParam(name_space + "/planner/safety_margin", config_.safety_margin, 0.3);
        LoadParam(name_space + "/planner/dilate_radius", config_.dilate_radius, 0.5);
        LoadParam(name_space + "/planner/weight_time", config_.weight_time, 20.0);
        LoadParam(name_space + "/planner/smoothing_eps", config_.smoothing_eps, 1.0e-2);
        LoadParam(name_space + "/planner/integral_intervs", config_.integral_intervs, 16);
        LoadParam(name_space + "/planner/rel_cost_tol", config_.rel_cost_tol, 1.0e-5);
        LoadParam(name_space + "/planner/timeout_rrt", config_.timeout_rrt, 0.05);
        LoadParam(name_space + "/planner/sfc_progress", config_.sfc_progress, 7.0);
        LoadParam(name_space + "/planner/sfc_range", config_.sfc_range, 3.0);
        LoadParam(name_space + "/planner/use_esdf", config_.use_esdf, false);
        LoadParam(name_space + "/planner/treat_unknown_as_free", config_.treat_unknown_as_free, true);
        // 可视化发布器
        traj_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("gcopter/trajectory", 1);
        corridor_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("gcopter/corridors", 1);
        
        initialized_ = true;
        ROS_INFO("[GCOPTERPlanner] Initialized with bounds: [%.1f,%.1f,%.1f] to [%.1f,%.1f,%.1f]",
                 map_lower_.x(), map_lower_.y(), map_lower_.z(),
                 map_upper_.x(), map_upper_.y(), map_upper_.z());
    }
    ~GCOPTERPlanner() = default;
                
    /**
     * @brief 设置规划边界
     */
    void setMapBounds(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper);
    
    /**
     * @brief 规划从当前位置到目标位置的轨迹
     * @param start_pos 起始位置
     * @param start_vel 起始速度
     * @param goal_pos 目标位置
     * @param goal_vel 目标速度（默认零，悬停）
     * @return 规划结果
     */
    PlanResult plan(const Eigen::Vector3d& start_pos,
                    const Eigen::Vector3d& start_vel,
                    const Eigen::Vector3d& goal_pos,
                    const Eigen::Vector3d& goal_vel = Eigen::Vector3d::Zero());
    
    /**
     * @brief 规划经过多个路径点的轨迹
     * @param start_pos 起始位置
     * @param start_vel 起始速度
     * @param waypoints 路径点序列（不包含起点）
     * @return 规划结果
     */
    PlanResult planThroughWaypoints(const Eigen::Vector3d& start_pos,
                                    const Eigen::Vector3d& start_vel,
                                    const std::vector<Eigen::Vector3d>& waypoints);
    
    /**
     * @brief 检查位置是否可通行（基于 ESDF）
     */
    bool isPositionFree(const Eigen::Vector3d& pos) const;
    
    /**
     * @brief 获取位置的 ESDF 距离
     */
    double getESDFDistance(const Eigen::Vector3d& pos) const;
    
    /**
     * @brief 碰撞查询（供 RRT* 使用）
     * @return 0 = free, 非0 = occupied
     */
    int queryCollision(const Eigen::Vector3d& pos) const;

private:
    // ==================== 内部方法 ====================
    
    /**
     * @brief RRT* 路径搜索
     */
    bool searchPath(const Eigen::Vector3d& start,
                    const Eigen::Vector3d& goal,
                    std::vector<Eigen::Vector3d>& path);
    
    /**
     * @brief 生成安全飞行走廊
     */
    bool generateCorridor(const std::vector<Eigen::Vector3d>& path,
                          std::vector<Eigen::MatrixX4d>& hPolys);
    
    /**
     * @brief 轨迹优化
     */
    bool optimizeTrajectory(const Eigen::Matrix3d& iniState,
                            const Eigen::Matrix3d& finState,
                            const std::vector<Eigen::MatrixX4d>& hPolys,
                            Trajectory<5>& traj);
    
private:
    ros::NodeHandle nh_;
    Config config_;
    ROGMap* rog_map_ = nullptr;
    
    // 地图边界
    Eigen::Vector3d map_lower_{-25, -25, 0};
    Eigen::Vector3d map_upper_{25, 25, 5};
    
    // 障碍物表面点缓存
    std::vector<Eigen::Vector3d> obstacle_points_;
    
    // ROS
    ros::Publisher traj_vis_pub_;
    ros::Publisher corridor_vis_pub_;
    
    bool initialized_ = false;
    template<class T>
    bool LoadParam(std::string param_name, T &param_value, T default_value = T{}, bool required = false) {
        if (nh_.getParam(param_name, param_value)) {
            printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
            std::cout << param_value << std::endl;
            return true;
        } else {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                   (nh_.getNamespace() + "/" + param_name).c_str());
            param_value = default_value;
            std::cout << param_value << std::endl;
            if (required) {
                throw std::invalid_argument(
                        std::string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
            }
            return false;
        }
    }

    template<class T>
    bool LoadParam(std::string param_name, std::vector<T> &param_value, std::vector<T> default_value = std::vector<T>{},
                   bool required = false) {
        if (nh_.getParam(param_name, param_value)) {
            printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
            for (size_t i = 0; i < param_value.size(); i++) {
                std::cout << param_value[i] << " ";
            }
            std::cout << std::endl;
            return true;
        } else {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                   (nh_.getNamespace() + "/" + param_name).c_str());
            param_value = default_value;
            for (size_t i = 0; i < param_value.size(); i++) {
                std::cout << param_value[i] << " ";
            }
            std::cout << std::endl;
            if (required) {
                throw std::invalid_argument(
                        std::string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
            }
            return false;
        }
    }
};

} // namespace rog_map

