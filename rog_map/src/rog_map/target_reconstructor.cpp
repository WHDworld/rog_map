#include "rog_map/target_reconstructor.h"
#include <random>

namespace rog_map {

void TargetReconstructor::init(ros::NodeHandle& nh) {
    nh_ = nh;
    
    // 初始化发布者
    corridor_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>(
        "target_recon/corridor", 1, true);
    viewpoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "target_recon/viewpoints", 1, true);
    path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "target_recon/optimized_path", 1, true);
        
    // 订阅目标话题 (使用 vision_msgs)
    target_sub_ = nh_.subscribe("/target_detection/confirmed_targets", 1, &TargetReconstructor::triggerCallback, this);
    
    // 订阅点云话题
    obstacle_cloud_sub_ = nh_.subscribe("/rog_map/local_cloud", 1, &TargetReconstructor::obstacleCloudCallback, this);
    target_cloud_sub_ = nh_.subscribe("/target_detection/target_cloud", 1, &TargetReconstructor::targetCloudCallback, this);
    
    // 初始化定时器
    execution_timer_ = nh_.createTimer(ros::Duration(0.1), &TargetReconstructor::executionTimerCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(0.2), &TargetReconstructor::visTimerCallback, this);
        
    // 加载参数
    loadParameters();
    
    initialized_ = true;
    ROS_INFO("[TargetReconstructor] Initialized.");
}

void TargetReconstructor::loadParameters() {
    ros::NodeHandle nh_param("~");
    
    // 采样模式
    nh_param.param("target_recon/sampling_mode", sampling_mode_, 0);
    nh_param.param<std::string>("target_recon/sampling_strategy", sampling_strategy_, "spherical");
    
    // 走廊参数
    nh_param.param("target_recon/corridor_radius", corridor_radius_, 2.0);
    nh_param.param("target_recon/local_bbox_x", local_bbox_x_, 5.0);
    nh_param.param("target_recon/local_bbox_y", local_bbox_y_, 5.0);
    nh_param.param("target_recon/local_bbox_z", local_bbox_z_, 3.0);
    nh_param.param("target_recon/obstacle_search_radius", obstacle_search_radius_, 10.0);
    nh_param.param("target_recon/min_ground_height", min_ground_height_, 0.0);
    
    // 球面采样参数
    nh_param.param("target_recon/spherical/min_distance", spherical_min_dist_, 2.5);
    nh_param.param("target_recon/spherical/max_distance", spherical_max_dist_, 5.0);
    nh_param.param("target_recon/spherical/num_layers", spherical_num_layers_, 3);
    nh_param.param("target_recon/spherical/min_elevation", spherical_min_elev_, 30.0);
    nh_param.param("target_recon/spherical/max_elevation", spherical_max_elev_, 75.0);
    nh_param.param("target_recon/spherical/points_per_layer", spherical_points_per_layer_, 16);
    
    // 圆柱采样参数
    nh_param.param("target_recon/cylindrical/radius", cyl_radius_, 3.5);
    nh_param.param("target_recon/cylindrical/min_height", cyl_min_height_, 0.5);
    nh_param.param("target_recon/cylindrical/max_height", cyl_max_height_, 4.0);
    nh_param.param("target_recon/cylindrical/num_height_layers", cyl_num_layers_, 4);
    nh_param.param("target_recon/cylindrical/points_per_circle", cyl_points_per_circle_, 12);
    
    // 半球采样参数
    nh_param.param("target_recon/hemisphere/min_distance", hemi_min_dist_, 2.0);
    nh_param.param("target_recon/hemisphere/max_distance", hemi_max_dist_, 4.5);
    nh_param.param("target_recon/hemisphere/num_layers", hemi_num_layers_, 2);
    nh_param.param("target_recon/hemisphere/min_elevation", hemi_min_elev_, 20.0);
    nh_param.param("target_recon/hemisphere/max_elevation", hemi_max_elev_, 80.0);
    nh_param.param("target_recon/hemisphere/points_per_layer", hemi_points_per_layer_, 12);
    
    // 滤波与评分
    nh_param.param("target_recon/safety_margin", safety_margin_, 0.3);
    nh_param.param("target_recon/coverage_weight", coverage_weight_, 0.40);
    nh_param.param("target_recon/angle_weight", angle_weight_, 0.25);
    nh_param.param("target_recon/distance_weight", distance_weight_, 0.20);
    nh_param.param("target_recon/safety_weight", safety_weight_, 0.15);
    
    // TSP
    nh_param.param("target_recon/max_2opt_iterations", max_2opt_iterations_, 100);
    
    // IROS Innovation Params
    nh_param.param("target_recon/octree_resolution", octree_resolution_, 0.2);
    nh_param.param("target_recon/normal_weight", normal_weight_, 0.30);
    
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
    new_target.recon_started = false;
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
        if (target.is_confirmed && !target.recon_started) {
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
        if (target.is_confirmed && target.recon_started && target.coverage_ratio < 0.9) {
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
    
    // 前端已剔除目标，直接用于安全走廊生成
    processed_cloud_ = cloud;
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
    
    // 如果目标已更新且有障碍物点云数据，则生成重建视点
    bool should_generate = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        should_generate = target_updated_ && 
                         current_target_.size.norm() > 0.1 && 
                         !obstacle_cloud_.empty();
    }
    
    if (should_generate) {
        ROS_INFO("[TargetReconstructor] Generating viewpoints for new target...");
        bool success = generateViewpoints();
        
        if (success) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            target_updated_ = false;  // 清除标志位，避免重复生成
            ROS_INFO("[TargetReconstructor] Viewpoint generation completed successfully.");
        } else {
            ROS_WARN("[TargetReconstructor] Failed to generate viewpoints, will retry.");
        }
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

// 评分实现细节 (简化版)
double TargetReconstructor::computeCoverageScore(const ReconViewpoint& vp) {
    // 理想情况应使用目标点云投影计算，这里简化为常数或简单几何
    if (current_target_.point_cloud && !current_target_.point_cloud->empty()) {
        // TODO: 真正的视锥体裁剪计算
        return 0.8; // 假设大部分可见
    }
    return 0.5;
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
    
    // IROS Innovation: Normal-Guided Scoring
    // 遍历视锥内的 Octree 表面节点，计算视线与法向量的对齐度
    
    std::vector<OctreeNode*> surface_nodes = octree_->getSurfaceNodes();
    if (surface_nodes.empty()) return 0.5;
    
    double total_score = 0;
    int visible_count = 0;
    
    // 简化视锥检查：距离 + 方向
    Eigen::Vector3d view_dir(cos(vp.yaw)*cos(vp.pitch), sin(vp.yaw)*cos(vp.pitch), sin(vp.pitch));
    
    for (auto node : surface_nodes) {
        Eigen::Vector3d dir_to_node = node->center - vp.position;
        double dist = dir_to_node.norm();
        
        // 距离筛选
        if (dist > camera_params_.max_range) continue;
        
        // 视场筛选 (点乘 > cos(hfov/2))
        Eigen::Vector3d dir_norm = dir_to_node.normalized();
        if (view_dir.dot(dir_norm) < cos(camera_params_.hfov / 1.5)) continue; 
        
        // 遮挡检查 (暂略，假设滤波环节已大致保证)
        
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
