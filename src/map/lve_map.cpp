#include "map/lve_map.h"

namespace lve {

int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    s = mod(s, 1);
    return (1 - s) / ds;
  }
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output) {
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output[output_points_cnt](0) = x;
      output[output_points_cnt](1) = y;
      output[output_points_cnt](2) = z;

      output_points_cnt++;
      dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
                  (z - start(2)) * (z - start(2)));

      if (dist > maxDist) return;
    }

    if (x == endX && y == endY && z == endZ) break;

    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        x += stepX;
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output) {
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  output->clear();

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output->push_back(Eigen::Vector3d(x, y, z));

      dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

      if (dist > maxDist) return;

      if (output->size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    if (x == endX && y == endY && z == endZ) break;

    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        x += stepX;
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

bool RayCaster::setInput(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end) {
  start_ = start;
  end_ = end;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  direction_ = (end_ - start_);
  maxDist_ = std::max(direction_.squaredNorm(), 3.0);

  // Break out direction vector.
  dx_ = endX_ - x_;
  dy_ = endY_ - y_;
  dz_ = endZ_ - z_;

  // Direction to increment x,y,z when stepping.
  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);
  stepZ_ = (int)signum((int)dz_);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  // The change in t when taking a step (always positive).
  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;
  tDeltaZ_ = ((double)stepZ_) / dz_;

  dist_ = 0;

  step_num_ = 0;

  if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
    return false;
  else
    return true;
}

bool RayCaster::step(Eigen::Vector3d& ray_pt) {
  ray_pt = Eigen::Vector3d(x_, y_, z_);

  dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  if (dist_ > maxDist_ * 2.0) // bad thing happens
  {
    return false;
  }

  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {
      x_ += stepX_;
      tMaxX_ += tDeltaX_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  return true;
}

void LVEMap::initMap(const ros::NodeHandle& nh)
{
  map_param_ptr_.reset(new MapParam);
  map_data_ptr_.reset(new MapData);
  // mr_.reset(new MapROS);

  // Params of map properties
  double x_size, y_size, z_size;
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  map_param_ptr_->map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, map_param_ptr_->ground_height_);
  map_param_ptr_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  map_param_ptr_->map_min_ = map_param_ptr_->map_origin_;
  map_param_ptr_->map_max_ = map_param_ptr_->map_origin_ + map_param_ptr_->map_size_;
  for (int i = 0; i < 3; ++i)
    map_param_ptr_->map_voxel_num_(i) = std::ceil(map_param_ptr_->map_size_(i) / map_param_ptr_->resolution_);

  nh.param("sdf_map/resolution", map_param_ptr_->resolution_, -1.0);
  map_param_ptr_->resolution_inv_ = 1.0 / map_param_ptr_->resolution_;

  nh.param("sdf_map/virtual_ceil_height", map_param_ptr_->virtual_ceil_height_, -0.1);
  nh.param("sdf_map/ground_height", map_param_ptr_->ground_height_, 1.0);
  nh.param("sdf_map/obstacles_inflation", map_param_ptr_->obstacles_inflation_, -1.0);

  // Try retriving bounding box of map, set box to map size if not specified
  std::vector<std::string> axis = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i) {
    nh.param("sdf_map/box_min_" + axis[i], map_param_ptr_->box_mind_[i], map_param_ptr_->map_min_[i]);
    nh.param("sdf_map/box_max_" + axis[i], map_param_ptr_->box_maxd_[i], map_param_ptr_->map_max_[i]);
  }
  posToIndex(map_param_ptr_->box_mind_, map_param_ptr_->box_min_);
  posToIndex(map_param_ptr_->box_maxd_, map_param_ptr_->box_max_);

  nh.param("sdf_map/default_dist", map_param_ptr_->default_dist_, 5.0);
  nh.param("sdf_map/local_bound_inflate", map_param_ptr_->local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", map_param_ptr_->local_map_margin_, 1);

  map_param_ptr_->local_bound_inflate_ = std::max(map_param_ptr_->resolution_, map_param_ptr_->local_bound_inflate_);

  // Params of raycasting-based fusion
  nh.param("sdf_map/p_hit", map_param_ptr_->p_hit_, 0.70);
  nh.param("sdf_map/p_miss", map_param_ptr_->p_miss_, 0.35);
  nh.param("sdf_map/p_min", map_param_ptr_->p_min_, 0.12);
  nh.param("sdf_map/p_max", map_param_ptr_->p_max_, 0.97);
  nh.param("sdf_map/p_occ", map_param_ptr_->p_occ_, 0.80);
  nh.param("sdf_map/max_ray_length", map_param_ptr_->max_ray_length_, -0.1);

  auto logit = [](const double& x) { return log(x / (1 - x)); };
  map_param_ptr_->prob_hit_log_ = logit(map_param_ptr_->p_hit_);
  map_param_ptr_->prob_miss_log_ = logit(map_param_ptr_->p_miss_);
  map_param_ptr_->clamp_min_log_ = logit(map_param_ptr_->p_min_);
  map_param_ptr_->clamp_max_log_ = logit(map_param_ptr_->p_max_);
  map_param_ptr_->min_occupancy_log_ = logit(map_param_ptr_->p_occ_);
  map_param_ptr_->unknown_flag_ = 0.01;
  std::cout << "hit: " << map_param_ptr_->prob_hit_log_ << ", miss: " << map_param_ptr_->prob_miss_log_
       << ", min: " << map_param_ptr_->clamp_min_log_ << ", max: " << map_param_ptr_->clamp_max_log_
       << ", thresh: " << map_param_ptr_->min_occupancy_log_ << std::endl;

  // Initialize data buffer of map
  int buffer_size = map_param_ptr_->map_voxel_num_(0) * map_param_ptr_->map_voxel_num_(1) * map_param_ptr_->map_voxel_num_(2);
  map_data_ptr_->occupancy_buffer_ = std::vector<double>(buffer_size, map_param_ptr_->clamp_min_log_ - map_param_ptr_->unknown_flag_);
  map_data_ptr->state_buffer_ = std::vector<VoxelState>(buffer_size, VoxelState::UNKNOWN);
  map_data_ptr_->occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);
  map_data_ptr_->count_hit_ = std::vector<short>(buffer_size, 0);
  map_data_ptr_->count_miss_ = std::vector<short>(buffer_size, 0);
  map_data_ptr_->flag_rayend_ = std::vector<char>(buffer_size, -1);
  map_data_ptr_->raycast_num_ = 0;

  // ==================== 初始化子网格 ====================
  // 读取各轴子网格物理边长（米）
  double sg_x, sg_y, sg_z;
  nh.param("sdf_map/subgrid_size_x", sg_x, 2.0);
  nh.param("sdf_map/subgrid_size_y", sg_y, 2.0);
  nh.param("sdf_map/subgrid_size_z", sg_z, 2.0);
  map_param_ptr_->subgrid_size_d_ = Eigen::Vector3d(sg_x, sg_y, sg_z);
  nh.param("sdf_map/exploration_rate_threshold", map_param_ptr_->exploration_rate_threshold_, 0.95);

  // 直接在物理空间计算各轴子网格数量（向上取整保证覆盖全图）
  for (int i = 0; i < 3; ++i) {
    map_param_ptr_->subgrid_num_(i) =
        static_cast<int>(std::ceil(map_param_ptr_->map_size_(i) / map_param_ptr_->subgrid_size_d_(i)));
    // 每轴每个子网格含多少体素（用于 indexToSubGrid 的整数除法）
    map_param_ptr_->subgrid_voxel_num_(i) =
        static_cast<int>(std::round(map_param_ptr_->subgrid_size_d_(i) * map_param_ptr_->resolution_inv_));
  }
  map_param_ptr_->total_subgrids_ = map_param_ptr_->subgrid_num_(0)
                                  * map_param_ptr_->subgrid_num_(1)
                                  * map_param_ptr_->subgrid_num_(2);

  // 分配子网格数组和体素映射表
  map_data_ptr_->subgrids_.resize(map_param_ptr_->total_subgrids_);
  map_data_ptr_->voxel_to_subgrid_.resize(buffer_size, -1);

  for (int sx = 0; sx < map_param_ptr_->subgrid_num_(0); ++sx)
    for (int sy = 0; sy < map_param_ptr_->subgrid_num_(1); ++sy)
      for (int sz = 0; sz < map_param_ptr_->subgrid_num_(2); ++sz) {
        // X-Y-Z 行优先线性化（与 toAddress / indexToSubGrid 一致）
        int sg_id = sx * map_param_ptr_->subgrid_num_(1) * map_param_ptr_->subgrid_num_(2)
                  + sy * map_param_ptr_->subgrid_num_(2) + sz;
        auto& sg = map_data_ptr_->subgrids_[sg_id];
        sg.grid_idx = sg_id;

        // 子网格体素索引范围（左闭右开：[min_idx, max_idx]，max_idx 为最后一个合法体素）
        // 起始索引：当前子网格编号 * 每轴体素数
        sg.min_idx = Eigen::Vector3i(
            sx * map_param_ptr_->subgrid_voxel_num_(0),
            sy * map_param_ptr_->subgrid_voxel_num_(1),
            sz * map_param_ptr_->subgrid_voxel_num_(2));
        // 终止索引：下一个子网格起始 - 1，但不能超过全局地图边界
        sg.max_idx = Eigen::Vector3i(
            std::min((sx + 1) * map_param_ptr_->subgrid_voxel_num_(0), map_param_ptr_->map_voxel_num_(0)) - 1,
            std::min((sy + 1) * map_param_ptr_->subgrid_voxel_num_(1), map_param_ptr_->map_voxel_num_(1)) - 1,
            std::min((sz + 1) * map_param_ptr_->subgrid_voxel_num_(2), map_param_ptr_->map_voxel_num_(2)) - 1);

        // 物理坐标范围（中心点采样，遵循体素中心对齐约定）
        indexToPos(sg.min_idx, sg.min_pos);
        indexToPos(sg.max_idx, sg.max_pos);
        sg.center_pos = (sg.min_pos + sg.max_pos) * 0.5;

        // 体素统计（边界子网格可能不满编）
        Eigen::Vector3i dims = sg.max_idx - sg.min_idx + Eigen::Vector3i::Ones();
        sg.total_voxels = dims(0) * dims(1) * dims(2);
        sg.unknown_voxels = sg.total_voxels;  // 初始全部未知
        sg.status = SubGrid::Status::UNEXPLORED;

        // 建立每个体素 → 子网格 ID 的映射表
        for (int vx = sg.min_idx(0); vx <= sg.max_idx(0); ++vx)
          for (int vy = sg.min_idx(1); vy <= sg.max_idx(1); ++vy)
            for (int vz = sg.min_idx(2); vz <= sg.max_idx(2); ++vz)
              map_data_ptr_->voxel_to_subgrid_[toAddress(vx, vy, vz)] = sg_id;
      }

  std::cout << "[LVEMap] SubGrid initialized: size=("
            << map_param_ptr_->subgrid_size_d_.transpose() << ")m, voxels_per_sg=("
            << map_param_ptr_->subgrid_voxel_num_.transpose() << "), grid_num=("
            << map_param_ptr_->subgrid_num_.transpose() << "), total="
            << map_param_ptr_->total_subgrids_ << std::endl;

  // mr_->setMap(this);
  // mr_->node_ = nh;
  // mr_->init();

  caster_ptr_.reset(new RayCaster);
  caster_ptr_->setParams(map_param_ptr_->resolution_, map_param_ptr_->map_origin_);
}
void LVEMap::setCacheOccupancy(const int& adr, const int& occ) {
  // Add to update list if first visited
  if (map_data_ptr_->count_hit_[adr] == 0 && map_data_ptr_->count_miss_[adr] == 0) map_data_ptr_->cache_voxel_.push(adr);

  if (occ == 0)
    map_data_ptr_->count_miss_[adr] = 1;
  else if (occ == 1)
    map_data_ptr_->count_hit_[adr] += 1;
}

Eigen::Vector3d LVEMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = map_param_ptr_->map_max_ - camera_pt;
  Eigen::Vector3d min_tc = map_param_ptr_->map_min_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}

void LVEMap::inputPointCloud(
    const sensor_msgs::PointCloud2& cloud,
    const Eigen::Vector3d& sensor_pos) {
    pcl::PointCloud<pcl::PointXYZ> points;
    pcl::fromROSMsg(cloud, points);
    if(points.empty()) return;
    map_data_ptr_->raycast_num_ += 1;

    Eigen::Vector3d pt_w, tmp;
    Eigen::Vector3i idx;
    int vox_adr;
    double length;
    for (int i = 0; i < points.size(); ++i) {
      auto& pt = points.points[i];
      pt_w << pt.x, pt.y, pt.z;
      int tmp_flag;
      // Set flag for projected point
      if (!isInMap(pt_w)) {
        // Find closest point in map and set free
        pt_w = closetPointInMap(pt_w, sensor_pos);
        length = (pt_w - sensor_pos).norm();
        if (length > map_param_ptr_->max_ray_length_)
          pt_w = (pt_w - sensor_pos) / length * map_param_ptr_->max_ray_length_ + sensor_pos;
        if (pt_w[2] < 0.2) continue;
        tmp_flag = 0;// 超出地图 → 标记为 miss (自由)
      } 
      else {
        length = (pt_w - sensor_pos).norm();
        if (length > map_param_ptr_->max_ray_length_) {
          pt_w = (pt_w - sensor_pos) / length * map_param_ptr_->max_ray_length_ + sensor_pos;
          if (pt_w[2] < 0.2) continue;
          tmp_flag = 0;// 超过最大射线长度 → 标记为 miss
        } else
          tmp_flag = 1;// 正常命中 → 标记为 hit (占据)
      }
      posToIndex(pt_w, idx);
      vox_adr = toAddress(idx);
      setCacheOccupancy(vox_adr, tmp_flag);// 处理终点状态 hit=1 或 miss=0

      for (int k = 0; k < 3; ++k) {
        update_min[k] = min(update_min[k], pt_w[k]);
        update_max[k] = max(update_max[k], pt_w[k]);
      }
      // Raycasting between camera center and point
      if (map_data_ptr_->flag_rayend_[vox_adr] == map_data_ptr_->raycast_num_)
        continue;
      else
        map_data_ptr_->flag_rayend_[vox_adr] = map_data_ptr_->raycast_num_;

      caster_ptr_->input(pt_w, camera_pos);
      caster_ptr_->nextId(idx); // 跳过起点(终点体素已处理)
      while (caster_ptr_->nextId(idx))
        setCacheOccupancy(toAddress(idx), 0);
    }

    while (!map_data_ptr_->cache_voxel_.empty()) {
      int adr = map_data_ptr_->cache_voxel_.front();
      map_data_ptr_->cache_voxel_.pop();
      double log_odds_update =
          map_data_ptr_->count_hit_[adr] >= map_data_ptr_->count_miss_[adr] ? map_param_ptr_->prob_hit_log_ : map_param_ptr_->prob_miss_log_;
      map_data_ptr_->count_hit_[adr] = map_data_ptr_->count_miss_[adr] = 0;

      // 2. 在修改任何值之前，从 state_buffer_ 读取旧状态
      uint8_t old_geo = map_data_ptr_->state_buffer_[adr].getGeoState();
      if (map_data_ptr_->occupancy_buffer_[adr] < map_param_ptr_->clamp_min_log_ - 1e-3)
        map_data_ptr_->occupancy_buffer_[adr] = map_param_ptr_->min_occupancy_log_;

      map_data_ptr_->occupancy_buffer_[adr] = std::min(
          std::max(map_data_ptr_->occupancy_buffer_[adr] + log_odds_update, map_param_ptr_->clamp_min_log_),
          map_param_ptr_->clamp_max_log_);
      // 5. 根据更新后的概率判定新状态
      uint8_t new_geo;
      if (map_data_ptr_->occupancy_buffer_[adr] > map_param_ptr_->min_occupancy_log_) {
        new_geo = VoxelState::GEO_OCCUPIED;
      } else {
        new_geo = VoxelState::GEO_FREE;
      }
      // ==========================================================
      // 将 状态同步 与 增量膨胀 完美融合
      // ==========================================================
      // 6. 仅在几何状态真正发生跃变时，才触发后续操作
      if (new_geo != old_geo) 
      {
        map_data_ptr_->state_buffer_[adr].setGeoState(new_geo);
        // 6a. 增量膨胀更新（引用计数）
        if (new_geo == VoxelState::GEO_OCCUPIED) 
        {
          // 跃变：空气/未知 → 墙，发散膨胀 (+1)
          Eigen::Vector3i center_id = addresstoIndex(adr);
          inflatePoint(center_id, inf_step, inf_pts);
          for (auto& inf_pt : inf_pts) {
            if (isInMap(inf_pt))
              map_data_ptr_->occupancy_buffer_inflate_[toAddress(inf_pt)] = 1;
          }
        } 
        else if (old_geo == VoxelState::GEO_OCCUPIED) 
        {
          // 跃变：墙 → 空气，撤销膨胀 (-1)
          Eigen::Vector3i center_id = addresstoIndex(adr);
          inflatePoint(center_id, inf_step, inf_pts);
          for (auto& inf_pt : inf_pts) {
            if (isInMap(inf_pt) &&
                map_data_ptr_->occupancy_buffer_inflate_[toAddress(inf_pt)] > 0)
              map_data_ptr_->occupancy_buffer_inflate_[toAddress(inf_pt)] = 0;
          }
        }
        // 6b. 子网格同步（探索率 + 占据体素列表）
        updateSubGridOnStateChange(adr, old_geo, new_geo);
      }
    }
  }

void LVEMap::updateSubGridOnStateChange(const int& adr, const uint8_t& old_geo, const uint8_t& new_geo)
{
    int sg_id = map_data_ptr_->voxel_to_subgrid_[adr];
    if (sg_id >= 0) 
    {
        map_data_ptr_->dirty_subgrids_.insert(sg_id);
        auto& sg = map_data_ptr_->subgrids_[sg_id];
        // 探索率增量：UNKNOWN → 非UNKNOWN 的首次跃变
        if (old_geo == VoxelState::GEO_UNKNOWN) 
        {
            sg.unknown_voxels--;
            if (sg.status == SubGrid::Status::UNEXPLORED)
              sg.status = SubGrid::Status::EXPLORING;
            if (sg.status == SubGrid::Status::EXPLORING &&
                sg.getExplorationRate() >= map_param_ptr_->exploration_rate_threshold_) {
              sg.status = SubGrid::Status::READY_FOR_ANALYSIS;
              map_data_ptr_->ready_subgrid_queue_.push(sg_id);
            }
        }
        // 占据体素列表维护
        if (new_geo == VoxelState::GEO_OCCUPIED) {
            sg.addOccUnobserved(adr);      // 新墙诞生
        } else if (old_geo == VoxelState::GEO_OCCUPIED) {
            sg.removeOccUnobserved(adr);   // 旧墙消失
        }
    }
}

void LVEMap::extractGlobalFrontierIncremental() {   
    // 只处理脏子网格
    for (int sg_id : map_data_ptr_->dirty_subgrids_) 
    {
        if (sg_id < 0 || sg_id >= static_cast<int>(map_data_ptr_->subgrids_.size())) continue;
        
        SubGrid& sg = map_data_ptr_->subgrids_[sg_id];

        sg.sub_grid_frontier_voxels_.clear();

        // 获取当前探索率
        float exploration_rate = sg.getExplorationRate();
        
        // ==================== 策略分支 ====================
        
        if (exploration_rate < 0.5f) {
            // ==================== 策略 A: 正向查找 (遍历 Free -> 找 Unknown) ====================
            // 适用场景：已探索区域少，列表短，遍历 Free 更快
            
            for (const auto& free_idx : sg.free_voxels_list_) {
                // 安全检查
                auto it = voxels_.find(free_idx);
                if (it == voxels_.end() || !VoxelStateBits::isFree(it->second)) continue;

                // 检查 6 邻域
                for (int d = 0; d < 6; d++) {
                    VoxelIndex nei_idx = free_idx + neighbor_offsets_6[d];
                    
                    if (!isInGlobalMapRange(nei_idx)) continue;
                    
                    // 检查邻居是否为 Unknown
                    auto nei_it = voxels_.find(nei_idx);
                    bool is_neighbor_unknown = (nei_it == voxels_.end()) || 
                                               VoxelStateBits::isUnknown(nei_it->second);
                    
                    if (is_neighbor_unknown) {
                        // [Target = Free Voxel] 
                        // neighbor is Unknown, so 'free_idx' is the boundary free voxel.
                        
                        // Height check on self (Free Voxel)
                        Position free_pos = indexToPos(free_idx);
                        if (free_pos.z() < config_.frontier_height_min || 
                            free_pos.z() > config_.frontier_height_max) break; // Self is invalid, next free
                        
                        sg.sub_grid_frontier_voxels_.insert(free_idx);
                        break; // Found one unknown neighbor, so this free voxel is a frontier. Done.
                    }
                }
            }
        } 
        else {
            // ==================== 策略 B: 反向查找 (遍历 Unknown -> 找 Free) ====================
            // 适用场景：未探索区域少 (如残留的死角)，列表短，遍历 Unknown 更快
            
            for (const auto& unk_idx : sg.unknown_voxels_list_) {
                // Safety check
                auto it = voxels_.find(unk_idx);
                if (it != voxels_.end() && !VoxelStateBits::isUnknown(it->second)) continue;

                // Check 6 neighbors for Free
                VoxelIndex best_free_idx;
                bool has_free_neighbor = false;
                
                for (int d = 0; d < 6; d++) {
                    VoxelIndex nei_idx = unk_idx + neighbor_offsets_6[d];
                    
                    if (!isInGlobalMapRange(nei_idx)) continue;
                    
                    auto nei_it = voxels_.find(nei_idx);
                    if (nei_it != voxels_.end() && VoxelStateBits::isFree(nei_it->second)) {
                        // Found a Free neighbor!
                        // Check Height of THIS Free neighbor
                        Position nei_pos = indexToPos(nei_idx);
                        if (nei_pos.z() >= config_.frontier_height_min && 
                            nei_pos.z() <= config_.frontier_height_max) {
                            
                            best_free_idx = nei_idx;
                            has_free_neighbor = true;
                            break; // Found valid free neighbor
                        }
                    }
                }

                if (has_free_neighbor) {
                    // [Target = Free Voxel]
                    sg.sub_grid_frontier_voxels_.insert(best_free_idx);
                }
            }
        }
        
        // 更新统计信息
        sg.has_frontier = !sg.sub_grid_frontier_voxels_.empty();
        sg.frontier_count = static_cast<int>(sg.sub_grid_frontier_voxels_.size());
        sg.frontier_dirty = false;
        
        // [新增] 记录此子网格的前沿发生了变化（供 ViewpointManager 使用）
        frontier_changed_subgrids_.insert(sg_id);
        
        // [新增] 语义分析触发：探索率达到阈值且未分析过
        if (exploration_rate >= semantic_exploration_threshold_ && 
            (sg.semantic_status_ == SubGrid::SemanticStatus::UNKNOWN ||
             sg.semantic_status_ == SubGrid::SemanticStatus::ACCUMULATING)) {
            ready_for_analysis_subgrids_.insert(sg_id);
            sg.semantic_status_ = SubGrid::SemanticStatus::READY;
        }
    }
    
    dirty_subgrids_.clear();
    updateFrontierSubGridCache();
}

};