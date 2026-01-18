#include "rog_map/target_reconstructor.h"
#include <random>

namespace rog_map {

void TargetReconstructor::init(ros::NodeHandle& nh) {
    nh_ = nh;
    
    // 先加载参数（包括话题名）
    loadParameters();
    
    // 初始化发布者
    corridor_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>(
        "target_recon/corridor", 1, true);
    viewpoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "target_recon/viewpoints", 1, true);
    path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "target_recon/optimized_path", 1, true);
        
    // 订阅话题（使用从参数加载的话题名）
    target_sub_ = nh_.subscribe(target_detection_topic_, 1, &TargetReconstructor::triggerCallback, this);
    obstacle_cloud_sub_ = nh_.subscribe(obstacle_cloud_topic_, 1, &TargetReconstructor::obstacleCloudCallback, this);
    target_cloud_sub_ = nh_.subscribe(target_cloud_topic_, 1, &TargetReconstructor::targetCloudCallback, this);
    
    ROS_INFO("[TargetReconstructor] Subscribed topics:");
    ROS_INFO("  obstacle_cloud: %s", obstacle_cloud_topic_.c_str());
    ROS_INFO("  target_cloud: %s", target_cloud_topic_.c_str());
    ROS_INFO("  target_detection: %s", target_detection_topic_.c_str());
    
    // 初始化定时器
    execution_timer_ = nh_.createTimer(ros::Duration(0.1), &TargetReconstructor::executionTimerCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(0.2), &TargetReconstructor::visTimerCallback, this);
    
    initialized_ = true;
    ROS_INFO("[TargetReconstructor] Initialized.");
}


void TargetReconstructor::loadParameters() {
    ros::NodeHandle nh_param("~");
    
    // ========== 话题配置 ==========
    // 共用话题：从rog_map读取
    nh_param.param<std::string>("rog_map/lidar_topic", obstacle_cloud_topic_, "/cloud_registered");
    // 专用话题：从target_recon读取
    nh_param.param<std::string>("target_recon/target_cloud_topic", target_cloud_topic_, "/target_pointcloud");
    nh_param.param<std::string>("target_recon/target_detection_topic", target_detection_topic_, "/target_detection/confirmed_targets");
    
    // ========== 采样模式 ==========
    nh_param.param("target_recon/sampling_mode", sampling_mode_, 0);
    nh_param.param<std::string>("target_recon/sampling_strategy", sampling_strategy_, "spherical");
    
    // ========== 走廊参数 ==========
    nh_param.param("target_recon/corridor_radius", corridor_radius_, 2.0);
    nh_param.param("target_recon/local_bbox_x", local_bbox_x_, 5.0);
    nh_param.param("target_recon/local_bbox_y", local_bbox_y_, 5.0);
    nh_param.param("target_recon/local_bbox_z", local_bbox_z_, 3.0);
    nh_param.param("target_recon/obstacle_search_radius", obstacle_search_radius_, 10.0);
    nh_param.param("target_recon/min_ground_height", min_ground_height_, 0.3);
    
    // 球面采样参数
    nh_param.param("target_recon/spherical/min_distance", spherical_min_dist_, 1.5);
    nh_param.param("target_recon/spherical/max_distance", spherical_max_dist_, 3.0);
    nh_param.param("target_recon/spherical/num_layers", spherical_num_layers_, 3);
    nh_param.param("target_recon/spherical/min_elevation", spherical_min_elev_, 15.0);
    nh_param.param("target_recon/spherical/max_elevation", spherical_max_elev_, 60.0);
    nh_param.param("target_recon/spherical/points_per_layer", spherical_points_per_layer_, 12);
    
    // 圆柱采样参数
    nh_param.param("target_recon/cylindrical/radius", cyl_radius_, 2.5);
    nh_param.param("target_recon/cylindrical/min_height", cyl_min_height_, 0.5);
    nh_param.param("target_recon/cylindrical/max_height", cyl_max_height_, 3.0);
    nh_param.param("target_recon/cylindrical/num_height_layers", cyl_num_layers_, 4);
    nh_param.param("target_recon/cylindrical/points_per_circle", cyl_points_per_circle_, 12);
    
    // 半球采样参数
    nh_param.param("target_recon/hemisphere/min_distance", hemi_min_dist_, 1.5);
    nh_param.param("target_recon/hemisphere/max_distance", hemi_max_dist_, 3.0);
    nh_param.param("target_recon/hemisphere/num_layers", hemi_num_layers_, 2);
    nh_param.param("target_recon/hemisphere/min_elevation", hemi_min_elev_, 20.0);
    nh_param.param("target_recon/hemisphere/max_elevation", hemi_max_elev_, 75.0);
    nh_param.param("target_recon/hemisphere/points_per_layer", hemi_points_per_layer_, 12);
    
    // 滤波与评分
    nh_param.param("target_recon/safety_margin", safety_margin_, 0.3);
    nh_param.param("target_recon/coverage_weight", coverage_weight_, 0.40);
    nh_param.param("target_recon/angle_weight", angle_weight_, 0.25);
    nh_param.param("target_recon/distance_weight", distance_weight_, 0.20);
    nh_param.param("target_recon/safety_weight", safety_weight_, 0.15);
    nh_param.param("target_recon/normal_weight", normal_weight_, 0.30);
    nh_param.param("target_recon/optimal_distance", optimal_distance_, 2.0);
    
    // Octree参数
    nh_param.param("target_recon/octree_resolution", octree_resolution_, 0.1);
    nh_param.param("target_recon/octree_hit_threshold", octree_hit_threshold_, 3);
    
    // TSP
    nh_param.param("target_recon/max_2opt_iterations", max_2opt_iterations_, 100);
    
    // 重建完成判定
    nh_param.param("target_recon/coverage_complete_threshold", coverage_complete_threshold_, 0.85);
    nh_param.param("target_recon/max_viewpoints", max_viewpoints_, 20);
    nh_param.param("target_recon/min_viewpoints", min_viewpoints_, 6);
    
    // 到达判定
    nh_param.param("target_recon/position_tolerance", position_tolerance_, 0.3);
    nh_param.param("target_recon/yaw_tolerance", yaw_tolerance_, 0.15);
    
    // 更新频率
    nh_param.param("target_recon/update_rate", update_rate_, 2.0);
    
    // 相机外参 (LiDAR -> Camera)
    std::vector<double> Rcl_vec;
    if (nh_param.getParam("extrin_calib/Rcl", Rcl_vec) && Rcl_vec.size() == 9) {
        Eigen::Matrix3d R_lidar_to_camera;
        R_lidar_to_camera << Rcl_vec[0], Rcl_vec[1], Rcl_vec[2],
                             Rcl_vec[3], Rcl_vec[4], Rcl_vec[5],
                             Rcl_vec[6], Rcl_vec[7], Rcl_vec[8];
                             
        // 提取相机俯仰角 (假设相机安装只有俯仰角变化)
        Eigen::Vector3d cam_optical_axis = R_lidar_to_camera.col(2);
        double z_comp = cam_optical_axis(2);
        double xy_length = sqrt(cam_optical_axis(0)*cam_optical_axis(0) + 
                               cam_optical_axis(1)*cam_optical_axis(1));
        camera_fixed_pitch_ = atan2(-z_comp, xy_length);
        ROS_INFO("[TargetReconstructor] Camera pitch: %.2f deg", camera_fixed_pitch_ * 180.0 / M_PI);
    }
    
    // 参数加载日志
    ROS_INFO("[TargetReconstructor] Parameters loaded:");
    ROS_INFO("  Sampling: strategy=%s, mode=%d", sampling_strategy_.c_str(), sampling_mode_);
    ROS_INFO("  Spherical: dist=[%.1f,%.1f], layers=%d, elev=[%.0f,%.0f]deg, pts=%d",
             spherical_min_dist_, spherical_max_dist_, spherical_num_layers_,
             spherical_min_elev_, spherical_max_elev_, spherical_points_per_layer_);
    ROS_INFO("  Scoring: cov=%.2f, ang=%.2f, dist=%.2f, safe=%.2f, norm=%.2f",
             coverage_weight_, angle_weight_, distance_weight_, safety_weight_, normal_weight_);
    ROS_INFO("  Octree: res=%.2f, hit_thresh=%d", octree_resolution_, octree_hit_threshold_);
    ROS_INFO("  Completion: coverage_thresh=%.2f, viewpoints=[%d,%d]",
             coverage_complete_threshold_, min_viewpoints_, max_viewpoints_);
}

bool TargetReconstructor::setTarget(const TargetInfo& target) {
    if (!initialized_) return false;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_target_ = target;
    
    if (obstacle_cloud_.size() > 0) {
        processed_cloud_ = removeTargetFromCloud(obstacle_cloud_);
    }
    
    // 初始化 Octree
    octree_.reset(new TargetOctree(current_target_.center, current_target_.size, octree_resolution_));
    replan_triggered_ = false;
    
    return true;
}

// ========== 多目标跟踪管理 ==========

int TargetReconstructor::matchOrCreateTarget(const TargetInfo& detection) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 1. 尝试匹配现有目标
    int matched_id = -1;
    double min_dist = target_match_distance_;
    
    for (auto& target : tracked_targets_) {
        double dist = (target.info.center - detection.center).norm();
        if (dist < min_dist) {
            min_dist = dist;
            matched_id = target.id;
        }
    }
    
    // 2. 找到匹配目标
    if (matched_id >= 0) {
        for (auto& target : tracked_targets_) {
            if (target.id == matched_id) {
                // 更新目标信息 (加权平均位置以平滑抖动)
                target.info.center = 0.7 * target.info.center + 0.3 * detection.center;
                target.info.size = detection.size;
                target.confirmation_count++;
                target.last_seen = ros::Time::now();
                
                // 检查是否达到确认阈值
                if (!target.is_confirmed && target.confirmation_count >= confirmation_frames_) {
                    target.is_confirmed = true;
                    // 初始化该目标的 Octree
                    target.octree.reset(new TargetOctree(target.info.center, target.info.size, octree_resolution_));
                    ROS_INFO("[TargetReconstructor] Target %d confirmed at (%.2f, %.2f, %.2f)", 
                             target.id, target.info.center.x(), target.info.center.y(), target.info.center.z());
                }
                
                return matched_id;
            }
        }
    }
    
    // 3. 创建新目标
    TrackedTarget new_target;
    new_target.id = next_target_id_++;
    new_target.info = detection;
    new_target.confirmation_count = 1;
    new_target.is_confirmed = false;
    new_target.last_seen = ros::Time::now();
    new_target.last_seen = ros::Time::now();
    new_target.recon_state = ReconstructionState::IDLE;
    new_target.coverage_ratio = 0.0;
    
    tracked_targets_.push_back(new_target);
    ROS_INFO("[TargetReconstructor] New target %d created at (%.2f, %.2f, %.2f)", 
             new_target.id, detection.center.x(), detection.center.y(), detection.center.z());
    
    return new_target.id;
}

TrackedTarget* TargetReconstructor::getActiveTarget() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto& target : tracked_targets_) {
        if (target.id == active_target_id_) {
            return &target;
        }
    }
    return nullptr;
}

TrackedTarget* TargetReconstructor::getTargetById(int id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto& target : tracked_targets_) {
        if (target.id == id) {
            return &target;
        }
    }
    return nullptr;
}

bool TargetReconstructor::selectNextTarget() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 优先选择已确认但尚未开始重建的目标
    for (auto& target : tracked_targets_) {
        if (target.is_confirmed && target.recon_state == ReconstructionState::IDLE) {
            active_target_id_ = target.id;
            
            // 设置为当前目标
            current_target_ = target.info;
            octree_ = target.octree;
            
            ROS_INFO("[TargetReconstructor] Selected target %d for reconstruction", target.id);
            return true;
        }
    }
    
    // 如果没有待重建目标，选择覆盖率最低的已开始重建目标
    double min_coverage = 1.0;
    int best_id = -1;
    for (auto& target : tracked_targets_) {
        // 选择正在重建或暂停但未完成的目标
        bool is_started = (target.recon_state != ReconstructionState::IDLE) && 
                          (target.recon_state != ReconstructionState::RECON_COMPLETE) &&
                          (target.recon_state != ReconstructionState::TARGET_DETECTED);
                          
        if (target.is_confirmed && is_started && target.coverage_ratio < 0.9) {
            if (target.coverage_ratio < min_coverage) {
                min_coverage = target.coverage_ratio;
                best_id = target.id;
            }
        }
    }
    
    if (best_id >= 0) {
        active_target_id_ = best_id;
        for (auto& target : tracked_targets_) {
            if (target.id == best_id) {
                current_target_ = target.info;
                octree_ = target.octree;
                break;
            }
        }
        ROS_INFO("[TargetReconstructor] Resuming target %d (coverage: %.1f%%)", best_id, min_coverage * 100);
        return true;
    }
    
    active_target_id_ = -1;
    return false;
}

void TargetReconstructor::updateTargetTracking(const std::vector<TargetInfo>& detections) {
    // 1. 清理过期目标
    ros::Time now = ros::Time::now();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        tracked_targets_.erase(
            std::remove_if(tracked_targets_.begin(), tracked_targets_.end(),
                [&](const TrackedTarget& t) {
                    bool timeout = (now - t.last_seen).toSec() > target_timeout_;
                    if (timeout && !t.is_confirmed) {
                        ROS_INFO("[TargetReconstructor] Removing unconfirmed target %d (timeout)", t.id);
                    }
                    return timeout && !t.is_confirmed; // 只移除未确认的超时目标
                }),
            tracked_targets_.end());
    }
    
    // 2. 处理新检测
    for (const auto& det : detections) {
        matchOrCreateTarget(det);
    }
}

// ==================== 新增状态查询接口实现 ====================
bool TargetReconstructor::hasReconstructionTask() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (const auto& target : tracked_targets_) {
        // 条件1: 已确认目标
        if (!target.is_confirmed) continue;
        
        // 条件2: 处于待重建或重建中状态
        if (target.recon_state == ReconstructionState::VIEWPOINT_READY ||
            target.recon_state == ReconstructionState::RECONSTRUCTING) {
            return true;
        }
        
        // 条件3: 新确认但尚未开始的目标
        if (target.recon_state == ReconstructionState::IDLE) {
            return true;
        }
    }
    
    return false;
}

bool TargetReconstructor::getActiveReconstructionStatus(int& target_id, ReconstructionState& recon_state) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (const auto& target : tracked_targets_) {
        if (target.id == active_target_id_) {
            target_id = target.id;
            recon_state = target.recon_state;
            return true;
        }
    }
    
    return false;
}

double TargetReconstructor::getReconstructionProgress(int target_id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (const auto& target : tracked_targets_) {
        if (target.id == target_id) {
            return target.coverage_ratio;
        }
    }
    
    return -1.0;
}

// ==================== 新增重建流程控制接口实现 ====================
bool TargetReconstructor::startReconstruction(const Eigen::Vector3d& drone_pos, int target_id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 更新无人机位置(用于后续TSP计算)
    drone_position_ = drone_pos;
    
    // Step 1: 选择目标
    TrackedTarget* target = nullptr;
    if (target_id == -1) {
        // 自动选择:优先选择IDLE且已确认的目标
        for (auto& t : tracked_targets_) {
            if (t.is_confirmed && t.recon_state == ReconstructionState::IDLE) {
                target = &t;
                break;
            }
        }
    } else {
        target = getTargetById(target_id);
    }
    
    if (!target || !target->is_confirmed) {
        ROS_WARN("[TargetReconstructor] No valid target to start reconstruction");
        return false;
    }
    
    // Step 2: 检查是否需要生成视点
    if (target->viewpoints.empty()) {
        // 需要生成视点
        target->recon_state = ReconstructionState::VIEWPOINT_GENERATING;
        
        // 同步设置current_target_(用于generateViewpoints)
        current_target_ = target->info;
        octree_ = target->octree;
        
        // 暂时解锁以调用generateViewpoints (避免死锁)
        data_mutex_.unlock();
        bool success = generateViewpoints();
        data_mutex_.lock();
        
        if (!success) {
            ROS_ERROR("[TargetReconstructor] Failed to generate viewpoints for target %d", target->id);
            target->recon_state = ReconstructionState::IDLE;
            return false;
        }
        
        // 将生成的视点保存到target
        target->viewpoints = viewpoints_;
    }
    
    // Step 3: 基于当前无人机位置重新计算TSP最优路径
    // 这确保了切入点是距离无人机最近的视点
    target->tour = greedyTSP(target->viewpoints);
    
    // 使用2-opt改进tour
    if (target->tour.size() > 3) {
        twoOptImprovement(target->tour, target->viewpoints);
    }
    
    target->current_viewpoint_idx = 0;
    target->recon_state = ReconstructionState::VIEWPOINT_READY;
    
    // Step 4: 激活目标
    active_target_id_ = target->id;
    target->recon_state = ReconstructionState::RECONSTRUCTING;
    
    ROS_INFO("[TargetReconstructor] Started reconstruction for target %d with %lu viewpoints, entry point at (%.2f, %.2f, %.2f)", 
             target->id, target->viewpoints.size(),
             target->viewpoints[target->tour[0]].position.x(),
             target->viewpoints[target->tour[0]].position.y(),
             target->viewpoints[target->tour[0]].position.z());
    return true;
}

bool TargetReconstructor::getCurrentReconViewpoint(ReconViewpoint& viewpoint) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    TrackedTarget* target = getActiveTarget();
    if (!target) {
        ROS_WARN_THROTTLE(1.0, "[TargetReconstructor] No active target for reconstruction");
        return false;
    }
    
    if (target->recon_state != ReconstructionState::RECONSTRUCTING) {
        ROS_WARN("[TargetReconstructor] Target %d is not in RECONSTRUCTING state", target->id);
        return false;
    }
    
    int idx = target->current_viewpoint_idx;
    if (idx < 0 || idx >= (int)target->tour.size()) {
        ROS_ERROR("[TargetReconstructor] Invalid viewpoint index %d (tour size: %lu)", idx, target->tour.size());
        return false;
    }
    
    int vp_id = target->tour[idx];
    if (vp_id < 0 || vp_id >= (int)target->viewpoints.size()) {
        ROS_ERROR("[TargetReconstructor] Invalid viewpoint ID %d (viewpoints size: %lu)", vp_id, target->viewpoints.size());
        return false;
    }
    
    viewpoint = target->viewpoints[vp_id];
    return true;
}

int TargetReconstructor::getReconstructionPath(std::vector<ReconViewpoint>& path) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    path.clear();
    TrackedTarget* target = getActiveTarget();
    if (!target) return 0;
    
    // 返回从当前视点开始的所有剩余视点
    int start_idx = target->current_viewpoint_idx;
    for (int i = start_idx; i < (int)target->tour.size(); ++i) {
        int vp_id = target->tour[i];
        if (vp_id >= 0 && vp_id < (int)target->viewpoints.size()) {
            path.push_back(target->viewpoints[vp_id]);
        }
    }
    
    ROS_DEBUG("[TargetReconstructor] Returned reconstruction path with %lu remaining viewpoints", path.size());
    return path.size();
}

bool TargetReconstructor::markCurrentViewpointComplete() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    TrackedTarget* target = getActiveTarget();
    if (!target) return false;
    
    // 移动到下一个视点
    target->current_viewpoint_idx++;
    
    // 检查是否完成所有视点
    if (target->current_viewpoint_idx >= (int)target->tour.size()) {
        target->recon_state = ReconstructionState::RECON_COMPLETE;
        active_target_id_ = -1;
        ROS_INFO("[TargetReconstructor] Target %d reconstruction complete! Final coverage: %.1f%%", 
                 target->id, target->coverage_ratio * 100.0);
        return false;  // 没有下一个视点
    }
    
    ROS_INFO("[TargetReconstructor] Marked viewpoint %d complete, moving to %d/%lu", 
             target->current_viewpoint_idx - 1, target->current_viewpoint_idx, target->tour.size());
    return true;  // 还有下一个视点
}

void TargetReconstructor::pauseReconstruction() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    TrackedTarget* target = getActiveTarget();
    if (target && target->recon_state == ReconstructionState::RECONSTRUCTING) {
        target->recon_state = ReconstructionState::VIEWPOINT_READY;
        ROS_INFO("[TargetReconstructor] Paused reconstruction of target %d at viewpoint %d/%lu", 
                 target->id, target->current_viewpoint_idx, target->tour.size());
    }
    
    active_target_id_ = -1;
}

// ==================== 新增实时更新接口实现 ====================
void TargetReconstructor::updateReconstructionData(const pcl::PointCloud<pcl::PointXYZ>& target_cloud, 
                                                     const Eigen::Vector3d& sensor_pose) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    TrackedTarget* target = getActiveTarget();
    if (!target || !target->octree) return;
    
    // 更新octree
    target->octree->update(target_cloud, sensor_pose);
    
    // 更新覆盖率
    target->coverage_ratio = target->octree->getCoverageRatio();
    target->last_update_time = ros::Time::now();
    
    ROS_DEBUG("[TargetReconstructor] Updated target %d: coverage %.1f%%", 
              target->id, target->coverage_ratio * 100.0);
}

bool TargetReconstructor::optimizeRemainingViewpoints() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    TrackedTarget* target = getActiveTarget();
    if (!target || target->recon_state != ReconstructionState::RECONSTRUCTING) {
        return false;
    }
    
    bool updated = false;
    int current_idx = target->current_viewpoint_idx;
    
    // 遍历剩余视点(包括当前未完成的)
    for (int i = current_idx; i < (int)target->tour.size(); ++i) {
        int vp_id = target->tour[i];
        if (vp_id < 0 || vp_id >= (int)target->viewpoints.size()) continue;
        
        ReconViewpoint& vp = target->viewpoints[vp_id];
        
        // ========== 优化1: 安全性检查 ==========
        if (!isInsideCorridor(vp.position)) {
            ROS_WARN("[TargetReconstructor] Viewpoint %d unsafe, removing from tour", vp_id);
            target->tour.erase(target->tour.begin() + i);
            --i;
            updated = true;
            continue;
        }
        
        // ========== 优化2: Yaw调整(基于最新octree) ==========
        if (target->octree) {
            // 获取当前视点视野内的未观测法向量质心
            auto frontiers = target->octree->getUnobservedFrontiers();
            if (!frontiers.empty()) {
                // 简化版:计算前沿点的质心方向
                Eigen::Vector3d centroid(0, 0, 0);
                for (const auto& f : frontiers) {
                    centroid += f->center;
                }
                centroid /= frontiers.size();
                
                Eigen::Vector3d guide_direction = centroid - vp.position;
                if (guide_direction.norm() > 0.1) {
                    double new_yaw = atan2(guide_direction.y(), guide_direction.x());
                    if (fabs(new_yaw - vp.yaw) > 0.2) {  // 差异>11度才更新
                        vp.yaw = new_yaw;
                        updated = true;
                        ROS_INFO("[TargetReconstructor] Updated viewpoint %d yaw to %.2f deg", 
                                 vp_id, new_yaw * 180.0 / M_PI);
                    }
                }
            }
        }
    }
    
    // 如果有视点被移除,检查是否需要动态补充
    if (updated && target->tour.size() < 3) {
        ROS_WARN("[TargetReconstructor] Too few remaining viewpoints (%lu), triggering dynamic replanning", 
                 target->tour.size());
        // 触发动态重规划 (需要解锁以避免死锁)
        data_mutex_.unlock();
        bool replan_success = triggerDynamicReplanning();
        data_mutex_.lock();
        
        if (replan_success) {
            // 将新生成的视点追加到target
            for (const auto& new_vp : viewpoints_) {
                bool duplicate = false;
                for (const auto& existing : target->viewpoints) {
                    if ((existing.position - new_vp.position).norm() < 0.5) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    ReconViewpoint vp_copy = new_vp;
                    vp_copy.id = target->viewpoints.size();
                    target->viewpoints.push_back(vp_copy);
                    target->tour.push_back(vp_copy.id);
                }
            }
        }
    }
    
    return updated;
}



void TargetReconstructor::updateObstacleCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacle_cloud_ = cloud;
}

void TargetReconstructor::updateDronePosition(const Eigen::Vector3d& pos) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    drone_position_ = pos;
}

void TargetReconstructor::obstacleCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacle_cloud_ = cloud;
    
    // 剔除目标区域的点云后用于安全走廊生成
    if (current_target_.center.norm() > 0.1 && current_target_.size.norm() > 0.1) {
        // 有有效目标时，剔除目标区域
        processed_cloud_ = removeTargetFromCloud(cloud);
        ROS_DEBUG("[TargetReconstructor] Processed cloud: %lu -> %lu points (removed target)",
                  cloud.size(), processed_cloud_.size());
    } else {
        // 无目标时直接使用原始点云
        processed_cloud_ = cloud;
    }
}

void TargetReconstructor::targetCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> target_cloud;
    pcl::fromROSMsg(*msg, target_cloud);
    
    if (target_cloud.empty()) return;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 更新 Octree
    if (octree_) {
        // 计算传感器位置 (从消息头或使用当前无人机位置)
        Eigen::Vector3d sensor_pose = drone_position_;
        octree_->update(target_cloud, sensor_pose);
        
        ROS_DEBUG("[TargetReconstructor] Updated Octree with %lu target points", target_cloud.size());
    }
}

bool TargetReconstructor::generateViewpoints() {
    if (!initialized_) return false;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 1. 生成安全走廊
    if (!generateCorridor()) {
        ROS_WARN("[TargetReconstructor] Failed to generate corridor");
        return false;
    }
    
    // 2. 采样视点
    std::vector<ReconViewpoint> raw_vps;
    switch (sampling_mode_) {
        case SPHERICAL:
            raw_vps = sampleSphericalViewpoints();
            break;
        case CYLINDRICAL:
            raw_vps = sampleCylindricalViewpoints();
            break;
        case HEMISPHERE:
            raw_vps = sampleHemisphereViewpoints();
            break;
        default:
            ROS_WARN("[TargetReconstructor] Unknown sampling mode %d, using Spherical", sampling_mode_);
            raw_vps = sampleSphericalViewpoints();
            break;
    }
    
    if (raw_vps.empty()) {
        ROS_WARN("[TargetReconstructor] No viewpoints sampled");
        return false;
    }
    
    // 3. 滤波
    std::vector<ReconViewpoint> filtered_vps = filterViewpoints(raw_vps);
    if (filtered_vps.empty()) {
        ROS_WARN("[TargetReconstructor] No viewpoints passed filtering");
        return false;
    }
    
    // 4. 评分
    scoreViewpoints(filtered_vps);
    
    // 5. 序列优化
    viewpoints_ = filtered_vps; // 保存有效视点
    tour_ = optimizeSequence(viewpoints_);
    
    ROS_INFO("[TargetReconstructor] Generated %lu viewpoints, tour length %lu", 
             viewpoints_.size(), tour_.size());
             
    return !tour_.empty();
}

void TargetReconstructor::triggerCallback(const vision_msgs::Detection3DArrayConstPtr& msg) {
    if (msg->detections.empty()) return;
    
    // 取第一个检测到的目标
    const auto& detection = msg->detections[0];
    
    TargetInfo target;
    target.center = Eigen::Vector3d(
        detection.bbox.center.position.x, 
        detection.bbox.center.position.y, 
        detection.bbox.center.position.z
    );
    target.size = Eigen::Vector3d(
        detection.bbox.size.x, 
        detection.bbox.size.y, 
        detection.bbox.size.z
    );
    
    setTarget(target);
    
    // 设置标志位，触发视点生成
    target_updated_ = true;
    
    ROS_INFO_THROTTLE(5.0, "[TargetReconstructor] Target updated from Detection3DArray.");
}

void TargetReconstructor::executionTimerCallback(const ros::TimerEvent& e) {
    if (!initialized_) return;
    
    // [重构说明] 该回调已改为被动响应模式
    // 旧版本: 自动检测target_updated_并生成视点
    // 新版本: 由ExplorationFSMNew主动调用startReconstruction()触发视点生成
    
    // 保留向后兼容警告
    bool has_old_style_update = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        has_old_style_update = target_updated_;
    }
    
    if (has_old_style_update) {
        ROS_WARN_THROTTLE(5.0, "[TargetReconstructor] Detected old-style target_updated_ flag. "
                          "Please use startReconstruction() API instead.");
    }
}


void TargetReconstructor::visTimerCallback(const ros::TimerEvent& e) {
    if (!initialized_) return;
    
    // 可视化
    publishCorridor();
    publishViewpoints();
    publishOptimalPath();
}

std::vector<ReconViewpoint> TargetReconstructor::getOptimizedPath() const {
    std::vector<ReconViewpoint> path;
    if (tour_.empty()) return path;
    
    for (int idx : tour_) {
        if (idx >= 0 && idx < (int)viewpoints_.size()) {
            path.push_back(viewpoints_[idx]);
        }
    }
    return path;
}

// ========== 内部实现 ==========
bool TargetReconstructor::generateCorridor() {
    if (processed_cloud_.empty() && obstacle_cloud_.empty()) {
        ROS_WARN("[TargetReconstructor] No obstacle cloud for corridor");
        return false;
    }
    
    const auto& cloud = processed_cloud_.empty() ? obstacle_cloud_ : processed_cloud_;
    
    // 提取局部障碍物
    Vec3f seed(current_target_.center.x(), current_target_.center.y(), current_target_.center.z());
    vec_Vec3f obs = extractLocalObstacles(seed, cloud);
    
    // SeedDecomp3D
    SeedDecomp3D decomp(seed);
    decomp.set_obs(obs);
    decomp.set_local_bbox(Vec3f(local_bbox_x_, local_bbox_y_, local_bbox_z_));
    decomp.dilate(corridor_radius_);
    
    current_corridor_ = decomp.get_polyhedron();
    
    // 添加地面约束
    Vec3f ground_p(0, 0, min_ground_height_);
    Vec3f ground_n(0, 0, -1);
    current_corridor_.add(Hyperplane3D(ground_p, ground_n));
    
    return !current_corridor_.hyperplanes().empty();
}

std::vector<ReconViewpoint> TargetReconstructor::sampleSphericalViewpoints() {
    std::vector<ReconViewpoint> vps;
    int id = 0;
    
    // 参数
    double min_elev = spherical_min_elev_ * M_PI / 180.0;
    double max_elev = spherical_max_elev_ * M_PI / 180.0;
    
    for (int layer = 0; layer < spherical_num_layers_; ++layer) {
        // 半径插值
        double r = spherical_min_dist_ + 
                  (spherical_max_dist_ - spherical_min_dist_) * 
                  layer / std::max(1.0, double(spherical_num_layers_ - 1));
                  
        // 仰角插值
        double elev = min_elev + (max_elev - min_elev) * 
                     layer / std::max(1.0, double(spherical_num_layers_ - 1));
                     
        for (int i = 0; i < spherical_points_per_layer_; ++i) {
            double azimuth = 2.0 * M_PI * i / spherical_points_per_layer_;
            
            ReconViewpoint vp;
            // 球坐标 -> 笛卡尔坐标
            vp.position.x() = current_target_.center.x() + r * cos(elev) * cos(azimuth);
            vp.position.y() = current_target_.center.y() + r * cos(elev) * sin(azimuth);
            vp.position.z() = current_target_.center.z() + r * sin(elev);
            
            // 计算yaw: 指向目标
            double dx = current_target_.center.x() - vp.position.x();
            double dy = current_target_.center.y() - vp.position.y();
            vp.yaw = atan2(dy, dx);
            
            // 固定pitch
            vp.pitch = camera_fixed_pitch_;
            vp.id = id++;
            
            vps.push_back(vp);
        }
    }
    return vps;
}

std::vector<ReconViewpoint> TargetReconstructor::sampleCylindricalViewpoints() {
    std::vector<ReconViewpoint> vps;
    int id = 0;
    
    // 使用圆柱采样参数
    double min_z = current_target_.center.z() + cyl_min_height_;
    double max_z = current_target_.center.z() + cyl_max_height_;
    
    for (int layer = 0; layer < cyl_num_layers_; ++layer) {
        double z = min_z + (max_z - min_z) * layer / std::max(1.0, double(cyl_num_layers_ - 1));
        
        for (int i = 0; i < cyl_points_per_circle_; ++i) {
            double azimuth = 2.0 * M_PI * i / cyl_points_per_circle_;
            
            ReconViewpoint vp;
            vp.position.x() = current_target_.center.x() + cyl_radius_ * cos(azimuth);
            vp.position.y() = current_target_.center.y() + cyl_radius_ * sin(azimuth);
            vp.position.z() = z;
            
            double dx = current_target_.center.x() - vp.position.x();
            double dy = current_target_.center.y() - vp.position.y();
            vp.yaw = atan2(dy, dx);
            vp.pitch = camera_fixed_pitch_;
            vp.id = id++;
            vps.push_back(vp);
        }
    }
    return vps;
}

// ========== IROS Innovation Methods ==========

void TargetReconstructor::updateOctree(const pcl::PointCloud<pcl::PointXYZ>& cloud, 
                                     const Eigen::Vector3d& sensor_pose) 
{
    if (!octree_) return;
    octree_->update(cloud, sensor_pose);
}

double TargetReconstructor::getCoverageRatio() const {
    if (!octree_) return 0.0;
    return octree_->getCoverageRatio();
}

bool TargetReconstructor::triggerDynamicReplanning() {
    if (!octree_) return false;
    if (replan_triggered_) return false; // 避免重复触发
    
    // 1. 获取未观测法向量前沿
    auto frontiers = octree_->getUnobservedFrontiers();
    if (frontiers.empty()) return false;
    
    ROS_INFO("[TargetReconstructor] Triggering dynamic replanning with %lu frontiers", frontiers.size());
    
    // 2. 针对主要簇进行锥形采样
    std::vector<ReconViewpoint> candidates;
    // 简单的聚类或直接采样，这里直接对所有前沿采样
    // 为了效率，最多取 5 个最大的前沿
    int count = 0;
    for (auto& f : frontiers) {
        if (count++ > 5) break; 
        
        Eigen::Vector3d guide_normal = octree_->getGuideNormal(f);
        auto cone_samples = sampleConeViewpoints(f->center, guide_normal);
        candidates.insert(candidates.end(), cone_samples.begin(), cone_samples.end());
    }
    
    if (candidates.empty()) return false;
    
    // 3. 评分筛选
    std::vector<ReconViewpoint> filtered = filterViewpoints(candidates);
    scoreViewpoints(filtered);
    
    if (filtered.empty()) return false;
    
    // 4. 添加到路径
    // 取前几个追加到现有视点列表
    // TODO: 应该只追加真正有效的，并重新优化。这里简化为追加
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    int added = 0;
    for (const auto& vp : filtered) {
        if (added >= 3) break; // 最多追加3个
        
        // 简单去重：如果和已有视点太近则不加
        bool duplicate = false;
        for (const auto& existing : viewpoints_) {
            if ((existing.position - vp.position).norm() < 0.5) {
                duplicate = true; 
                break;
            }
        }
        
        if (!duplicate) {
            ReconViewpoint new_vp = vp;
            new_vp.id = viewpoints_.size(); // 更新ID
            viewpoints_.push_back(new_vp);
            added++;
        }
    }
    
    if (added > 0) {
        replan_triggered_ = true;
        // 重新优化序列
        tour_ = optimizeSequence(viewpoints_);
        ROS_INFO("[TargetReconstructor] Dynamic replanning added %d viewpoints", added);
        return true;
    }
    
    return false;
}

std::vector<ReconViewpoint> TargetReconstructor::sampleHemisphereViewpoints() {
    std::vector<ReconViewpoint> vps;
    int id = 0;
    
    // 使用半球采样参数
    double min_elev = std::max(0.0, hemi_min_elev_ * M_PI / 180.0);
    double max_elev = std::min(M_PI/2.0, hemi_max_elev_ * M_PI / 180.0);
    
    for (int layer = 0; layer < hemi_num_layers_; ++layer) {
        // 半径插值（半球模式支持距离变化）
        double r = hemi_min_dist_ + (hemi_max_dist_ - hemi_min_dist_) * 
                  layer / std::max(1.0, double(hemi_num_layers_ - 1));
        double elev = min_elev + (max_elev - min_elev) * layer / std::max(1.0, double(hemi_num_layers_ - 1));
        
        for (int i = 0; i < hemi_points_per_layer_; ++i) {
            double azimuth = 2.0 * M_PI * i / hemi_points_per_layer_;
            
            ReconViewpoint vp;
            vp.position.x() = current_target_.center.x() + r * cos(elev) * cos(azimuth);
            vp.position.y() = current_target_.center.y() + r * cos(elev) * sin(azimuth);
            vp.position.z() = current_target_.center.z() + r * sin(elev);
            
            double dx = current_target_.center.x() - vp.position.x();
            double dy = current_target_.center.y() - vp.position.y();
            vp.yaw = atan2(dy, dx);
            vp.pitch = camera_fixed_pitch_;
            vp.id = id++;
            vps.push_back(vp);
        }
    }
    return vps;
}

std::vector<ReconViewpoint> TargetReconstructor::filterViewpoints(
    const std::vector<ReconViewpoint>& vps) 
{
    std::vector<ReconViewpoint> filtered;
    for (const auto& vp : vps) {
        // 1. 走廊检查
        if (!isInsideCorridor(vp.position)) continue;
        
        // 2. 遮挡检查
        if (enable_obstruction_check_ && 
            !isViewUnobstructed(vp.position, current_target_.center)) continue;
            
        filtered.push_back(vp);
    }
    return filtered;
}

bool TargetReconstructor::isInsideCorridor(const Eigen::Vector3d& p) const {
    Vec3f point(p.x(), p.y(), p.z());
    const auto& hyperplanes = current_corridor_.hyperplanes();
    
    for (const auto& hp : hyperplanes) {
        // dist < 0 为内部，需要大于 -margin 视为太靠近边界
        if ((point - hp.p_).dot(hp.n_) > -safety_margin_) return false;
    }
    return true;
}

bool TargetReconstructor::isViewUnobstructed(const Eigen::Vector3d& from, 
                                            const Eigen::Vector3d& to) 
{
    if (obstacle_cloud_.empty()) return true;
    
    Eigen::Vector3d dir = to - from;
    double dist = dir.norm();
    dir.normalize();
    
    // 简单射线检测: 在视线上采样几点
    std::vector<double> ratios = {0.2, 0.4, 0.6, 0.8}; // 避开端点
    for (double r : ratios) {
        Eigen::Vector3d pt = from + dir * dist * r;
        
        // 检查附近是否有障碍点
        for (const auto& obs_pt : obstacle_cloud_.points) {
            Eigen::Vector3d p(obs_pt.x, obs_pt.y, obs_pt.z);
            if ((p - pt).squaredNorm() < 0.2 * 0.2) { // 20cm阈值
                return false;
            }
        }
    }
    return true;
}

void TargetReconstructor::scoreViewpoints(std::vector<ReconViewpoint>& vps) {
    for (auto& vp : vps) {
        vp.coverage_score = computeCoverageScore(vp);
        vp.angle_score = computeAngleScore(vp);
        vp.distance_score = computeDistanceScore(vp);
        vp.safety_score = computeSafetyScore(vp);
        
        // IROS Innovation: Normal Score included
        double normal_score = computeNormalScore(vp);
        
        vp.score = coverage_weight_ * vp.coverage_score +
                   angle_weight_ * vp.angle_score +
                   distance_weight_ * vp.distance_score +
                   safety_weight_ * vp.safety_score +
                   normal_weight_ * normal_score;
    }
    
    // 按分数排序 (高分在前)
    std::sort(vps.begin(), vps.end(), 
        [](const ReconViewpoint& a, const ReconViewpoint& b) {
            return a.score > b.score;
        });
}

// 评分实现细节 (Raycasting版)
double TargetReconstructor::computeCoverageScore(const ReconViewpoint& vp) {
    if (!octree_) return 0.0;
    
    // 获取目标表面节点
    std::vector<OctreeNode*> surface_nodes = octree_->getSurfaceNodes();
    if (surface_nodes.empty()) return 0.5;
    
    // 视线方向
    Eigen::Vector3d view_dir(cos(vp.yaw) * cos(vp.pitch), 
                              sin(vp.yaw) * cos(vp.pitch), 
                              sin(vp.pitch));
    
    int visible_count = 0;
    int total_in_fov = 0;
    
    for (auto node : surface_nodes) {
        Eigen::Vector3d dir_to_node = node->center - vp.position;
        double dist = dir_to_node.norm();
        
        // 距离筛选
        if (dist > camera_params_.max_range || dist < 0.5) continue;
        
        // 视场筛选
        Eigen::Vector3d dir_norm = dir_to_node.normalized();
        if (view_dir.dot(dir_norm) < cos(camera_params_.hfov / 2.0)) continue;
        
        total_in_fov++;
        
        // 遮挡检测：使用Octree的isOccluded方法
        if (!octree_->isOccluded(vp.position, node->center)) {
            visible_count++;
        }
    }
    
    if (total_in_fov == 0) return 0.0;
    return static_cast<double>(visible_count) / total_in_fov;
}

double TargetReconstructor::computeAngleScore(const ReconViewpoint& vp) {
    Eigen::Vector3d dir = (current_target_.center - vp.position).normalized();
    Eigen::Vector3d radial = (current_target_.center - vp.position);
    radial.z() = 0; 
    radial.normalize();
    
    // 期望有一定的俯视角度
    double cos_angle = dir.dot(radial); // 实际上是水平夹角的余弦
    return std::max(0.0, 1.0 - fabs(cos_angle - optimal_angle_cos_));
}

double TargetReconstructor::computeDistanceScore(const ReconViewpoint& vp) {
    double d = (vp.position - current_target_.center).norm();
    return std::max(0.0, 1.0 - fabs(d - optimal_distance_) / optimal_distance_);
}

double TargetReconstructor::computeSafetyScore(const ReconViewpoint& vp) {
    Vec3f p(vp.position.x(), vp.position.y(), vp.position.z());
    double min_dist = 100.0;
    
    for (const auto& hp : current_corridor_.hyperplanes()) {
        double d = -(p - hp.p_).dot(hp.n_); // 内部为正距离
        if (d < min_dist) min_dist = d;
    }
    return std::min(1.0, min_dist); // 距离边界越远越安全(>1m满分)
}

double TargetReconstructor::computeNormalScore(const ReconViewpoint& vp) {
    if (!octree_) return 0.5; // 无 octree 时返回中庸分，或基于 BBox 计算
    
    // IROS Innovation: Normal-Guided Scoring with Occlusion Culling
    // 遍历视锥内的 Octree 表面节点，计算视线与法向量的对齐度
    
    std::vector<OctreeNode*> surface_nodes = octree_->getSurfaceNodes();
    if (surface_nodes.empty()) return 0.5;
    
    double total_score = 0;
    int visible_count = 0;
    
    // 视锥方向
    Eigen::Vector3d view_dir(cos(vp.yaw)*cos(vp.pitch), sin(vp.yaw)*cos(vp.pitch), sin(vp.pitch));
    
    for (auto node : surface_nodes) {
        Eigen::Vector3d dir_to_node = node->center - vp.position;
        double dist = dir_to_node.norm();
        
        // 距离筛选
        if (dist > camera_params_.max_range || dist < 0.5) continue;
        
        // 视场筛选 (点乘 > cos(hfov/2))
        Eigen::Vector3d dir_norm = dir_to_node.normalized();
        if (view_dir.dot(dir_norm) < cos(camera_params_.hfov / 2.0)) continue; 
        
        // [关键修复] 遮挡检查
        if (octree_->isOccluded(vp.position, node->center)) {
            continue;  // 被遮挡的节点不参与评分
        }
        
        // 法向量对齐度计算
        // 理想情况：视线方向 (dir_norm) 应该与表面法向量 (node->normal) 相反
        // alignment = -dir_norm . normal
        Eigen::Vector3d guide_normal = octree_->getGuideNormal(node);
        double alignment = -dir_norm.dot(guide_normal);
        
        if (alignment > 0) {
            total_score += alignment;
            visible_count++;
        }
    }
    
    return visible_count > 0 ? (total_score / visible_count) : 0.0;
}


std::vector<ReconViewpoint> TargetReconstructor::sampleConeViewpoints(
    const Eigen::Vector3d& center, const Eigen::Vector3d& normal)
{
    std::vector<ReconViewpoint> vps;
    
    // 沿法向量方向，在一定距离和角度范围内采样
    double dist = optimal_distance_;
    double cone_angle = 30.0 * M_PI / 180.0; 
    int num_samples = 5;
    
    // 基础视点：正对法向量
    // Eigen::Vector3d base_pos = center + normal * dist; // Unused
    
    // 创建局部坐标系
    Eigen::Vector3d z_axis = normal;
    Eigen::Vector3d x_axis = z_axis.cross(Eigen::Vector3d::UnitZ());
    if (x_axis.norm() < 1e-3) x_axis = z_axis.cross(Eigen::Vector3d::UnitX());
    x_axis.normalize();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    
    // 生成采样
    for (int i = 0; i < num_samples; ++i) {
        // 在圆锥内随机扰动
        double theta = (double)rand() / RAND_MAX * cone_angle;
        double phi = (double)rand() / RAND_MAX * 2.0 * M_PI;
        
        // 扰动方向 (相对于 z_axis)
        double r_sin = sin(theta);
        Eigen::Vector3d local_dir = x_axis * (r_sin * cos(phi)) + 
                                  y_axis * (r_sin * sin(phi)) + 
                                  z_axis * cos(theta);
        local_dir.normalize();
        
        ReconViewpoint vp;
        vp.position = center + local_dir * dist;
        
        // 计算 LookAt
        Eigen::Vector3d look_dir = center - vp.position;
        vp.yaw = atan2(look_dir.y(), look_dir.x());
        vp.pitch = camera_fixed_pitch_; // 或 atan2(look_dir.z(), ...)
        
        vps.push_back(vp);
    }
    
    return vps;
}

// ========== TSP 优化 ==========

std::vector<int> TargetReconstructor::optimizeSequence(
    const std::vector<ReconViewpoint>& vps) 
{
    if (vps.empty()) return {};
    
    // 1. 贪心构造
    std::vector<int> current_tour = greedyTSP(vps);
    
    // 2. 2-opt 优化
    twoOptImprovement(current_tour, vps);
    
    return current_tour;
}

std::vector<int> TargetReconstructor::greedyTSP(
    const std::vector<ReconViewpoint>& vps) 
{
    std::vector<int> tour;
    std::vector<bool> visited(vps.size(), false);
    
    // 找最近的起点
    int current = -1;
    double min_dist = 1e9;
    for (size_t i = 0; i < vps.size(); ++i) {
        double d = (vps[i].position - drone_position_).norm();
        if (d < min_dist) {
            min_dist = d;
            current = i;
        }
    }
    
    if (current == -1) return tour;
    
    tour.push_back(current);
    visited[current] = true;
    
    // 贪心选择下一个
    for (size_t i = 1; i < vps.size(); ++i) {
        int next = -1;
        double best_cost = 1e9;
        
        for (size_t j = 0; j < vps.size(); ++j) {
            if (visited[j]) continue;
            
            // 代价 = 距离 + 角度惩罚
            double dist = (vps[current].position - vps[j].position).norm();
            double yaw_diff = fabs(vps[current].yaw - vps[j].yaw);
            if (yaw_diff > M_PI) yaw_diff = 2*M_PI - yaw_diff;
            
            double cost = dist + yaw_diff * 0.5;
            if (cost < best_cost) {
                best_cost = cost;
                next = j;
            }
        }
        
        if (next != -1) {
            tour.push_back(next);
            visited[next] = true;
            current = next;
        }
    }
    return tour;
}

void TargetReconstructor::twoOptImprovement(
    std::vector<int>& tour, const std::vector<ReconViewpoint>& vps) 
{
    if (tour.size() < 4) return;
    
    bool improved = true;
    int iterations = 0;
    
    while (improved && iterations < max_2opt_iterations_) {
        improved = false;
        
        for (size_t i = 0; i < tour.size() - 2; ++i) {
            for (size_t j = i + 2; j < tour.size() - 1; ++j) {
                // 原路径：i->i+1, j->j+1
                // 交换后：i->j, i+1->j+1 (中间翻转)
                
                auto dist = [&](int a, int b) {
                    return (vps[tour[a]].position - vps[tour[b]].position).norm();
                };
                
                double d1 = dist(i, i+1) + dist(j, j+1);
                double d2 = dist(i, j) + dist(i+1, j+1);
                
                if (d2 < d1 - 1e-3) { // 有显著改进
                    std::reverse(tour.begin() + i + 1, tour.begin() + j + 1);
                    improved = true;
                }
            }
        }
        iterations++;
    }
}

// ========== 辅助功能 ==========

Vec3f toVec3f(const Eigen::Vector3d& v) { return Vec3f(v.x(), v.y(), v.z()); }

vec_Vec3f TargetReconstructor::extractLocalObstacles(
    const Vec3f& center, const pcl::PointCloud<pcl::PointXYZ>& cloud) 
{
    vec_Vec3f obs;
    double r2 = obstacle_search_radius_ * obstacle_search_radius_;
    
    for (const auto& p : cloud.points) {
        Vec3f pt(p.x, p.y, p.z);
        if ((pt - center).squaredNorm() < r2) {
            obs.push_back(pt);
        }
    }
    return obs;
}

pcl::PointCloud<pcl::PointXYZ> TargetReconstructor::removeTargetFromCloud(
    const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::PointCloud<pcl::PointXYZ> out;
    Eigen::Vector3d min_p = current_target_.center - current_target_.size / 2.0;
    Eigen::Vector3d max_p = current_target_.center + current_target_.size / 2.0;
    double pad = target_bbox_padding_;
    
    for (const auto& p : cloud.points) {
        if (p.x > min_p.x() - pad && p.x < max_p.x() + pad &&
            p.y > min_p.y() - pad && p.y < max_p.y() + pad &&
            p.z > min_p.z() - pad && p.z < max_p.z() + pad) 
        {
            continue; // Inside target bbox
        }
        out.points.push_back(p);
    }
    return out;
}

// ========== 可视化发布 ==========
// (简化实现)

void TargetReconstructor::publishCorridor() {
    decomp_ros_msgs::PolyhedronArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    
    decomp_ros_msgs::Polyhedron poly_msg;
    for (const auto& hp : current_corridor_.hyperplanes()) {
        geometry_msgs::Point p, n;
        p.x = hp.p_.x(); p.y = hp.p_.y(); p.z = hp.p_.z();
        n.x = hp.n_.x(); n.y = hp.n_.y(); n.z = hp.n_.z();
        poly_msg.points.push_back(p);
        poly_msg.normals.push_back(n);
    }
    msg.polyhedrons.push_back(poly_msg);
    corridor_pub_.publish(msg);
}

void TargetReconstructor::publishViewpoints() {
    visualization_msgs::MarkerArray msg;
    int id = 0;
    
    // 显示所有采样点（半透明）
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE;
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.color.a = 0.5;
    m.color.b = 1.0; 
    
    for (const auto& vp : viewpoints_) {
        m.id = id++;
        m.pose.position.x = vp.position.x();
        m.pose.position.y = vp.position.y();
        m.pose.position.z = vp.position.z();
        msg.markers.push_back(m);
    }
    viewpoints_pub_.publish(msg);
}

void TargetReconstructor::publishOptimalPath() {
    if (tour_.empty()) return;
    
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "path";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.05;
    line.color.r = 1.0; line.color.a = 1.0;
    
    for (int idx : tour_) {
        if (idx < (int)viewpoints_.size()) {
            geometry_msgs::Point p;
            p.x = viewpoints_[idx].position.x();
            p.y = viewpoints_[idx].position.y();
            p.z = viewpoints_[idx].position.z();
            line.points.push_back(p);
        }
    }
    msg.markers.push_back(line);
    path_pub_.publish(msg);
}

} // namespace rog_map
