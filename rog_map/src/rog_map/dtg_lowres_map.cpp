#include "rog_map/dtg_lowres_map.h"

#include <ros/ros.h>
#include <algorithm>

namespace rog_map {
namespace dtg {

void DtgLowresMap::init(const ros::NodeHandle& nh, const std::string& ns, star_mvf::GlobalMap::Ptr global_map) {
  global_map_ = global_map;

  nh.param(ns + "/resolution", cfg_.resolution, cfg_.resolution);
  nh.param(ns + "/max_search_time", cfg_.max_search_time, cfg_.max_search_time);
  nh.param(ns + "/allow_unknown", cfg_.allow_unknown, cfg_.allow_unknown);
  nh.param(ns + "/local_half_size_x", cfg_.local_half_size.x(), cfg_.local_half_size.x());
  nh.param(ns + "/local_half_size_y", cfg_.local_half_size.y(), cfg_.local_half_size.y());
  nh.param(ns + "/local_half_size_z", cfg_.local_half_size.z(), cfg_.local_half_size.z());
  nh.param(ns + "/occ_sample_resolution", cfg_.occ_sample_resolution, cfg_.occ_sample_resolution);

  if (global_map_) {
    auto minp = global_map_->getGlobalMapMin();
    auto maxp = global_map_->getGlobalMapMax();
    cfg_.map_min = minp.cast<double>();
    cfg_.map_max = maxp.cast<double>();
    cfg_.map_origin = cfg_.map_min;
  }

  node_scale_ = Eigen::Vector3d::Ones() * cfg_.resolution;

  Eigen::Vector3d extent = cfg_.map_max - cfg_.map_min;
  voxel_num_ = (extent.cwiseQuotient(node_scale_)).array().floor().cast<int>();
  voxel_num_ = voxel_num_.cwiseMax(Eigen::Vector3i(1, 1, 1));

  // precompute 26-neighbour diffs (3x3x3)
  connect_diff_.clear();
  connect_diff_.reserve(27);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        connect_diff_.push_back(Eigen::Vector3i(dx, dy, dz));
      }
    }
  }

  if (blocks_.empty()) blocks_.resize(1);

  ROS_INFO("[DTG_LRM] init: res=%.2f, voxel_num=(%d,%d,%d), local_half=(%.1f,%.1f,%.1f)",
           cfg_.resolution,
           voxel_num_.x(), voxel_num_.y(), voxel_num_.z(),
           cfg_.local_half_size.x(), cfg_.local_half_size.y(), cfg_.local_half_size.z());
}

bool DtgLowresMap::insideMap(const Eigen::Vector3d& pos) const {
  return !(pos.x() < cfg_.map_min.x() || pos.y() < cfg_.map_min.y() || pos.z() < cfg_.map_min.z() ||
           pos.x() > cfg_.map_max.x() || pos.y() > cfg_.map_max.y() || pos.z() > cfg_.map_max.z());
}

bool DtgLowresMap::insideMap(const Eigen::Vector3i& idx3) const {
  return !(idx3.x() < 0 || idx3.y() < 0 || idx3.z() < 0 ||
           idx3.x() >= voxel_num_.x() || idx3.y() >= voxel_num_.y() || idx3.z() >= voxel_num_.z());
}

void DtgLowresMap::posToId3(const Eigen::Vector3d& pos, Eigen::Vector3i& id3) const {
  Eigen::Vector3d d = pos - cfg_.map_origin;
  id3.x() = static_cast<int>(std::floor(d.x() / node_scale_.x()));
  id3.y() = static_cast<int>(std::floor(d.y() / node_scale_.y()));
  id3.z() = static_cast<int>(std::floor(d.z() / node_scale_.z()));
}

Eigen::Vector3d DtgLowresMap::id3ToPos(const Eigen::Vector3i& id3) const {
  return cfg_.map_origin + (id3.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)).cwiseProduct(node_scale_);
}

bool DtgLowresMap::id3ToIndex(const LRBlock& blk, const Eigen::Vector3i& id3, int& out_idx) const {
  const Eigen::Vector3i d = id3 - blk.origin;
  if (d.x() < 0 || d.y() < 0 || d.z() < 0 || d.x() >= blk.size.x() || d.y() >= blk.size.y() || d.z() >= blk.size.z()) {
    return false;
  }
  out_idx = (d.z() * blk.size.y() + d.y()) * blk.size.x() + d.x();
  return true;
}

Eigen::Vector3d DtgLowresMap::getStdPos(const Eigen::Vector3d& pos) const {
  Eigen::Vector3i id3;
  posToId3(pos, id3);
  return id3ToPos(id3);
}

VoxelStateLite DtgLowresMap::getVoxelStateLite(const Eigen::Vector3d& pos) const {
  if (!global_map_) return VoxelStateLite::Free;
  if (!insideMap(pos)) return VoxelStateLite::Out;

  const auto st = global_map_->getVoxelState(pos.cast<float>());
  if (star_mvf::VoxelStateBits::isOccupied(st)) return VoxelStateLite::Occupied;
  if (star_mvf::VoxelStateBits::isFree(st)) return VoxelStateLite::Free;
  return VoxelStateLite::Unknown;
}

void DtgLowresMap::computeNodeOccupancy(const Eigen::Vector3i& node_id3, LRNode& node) const {
  const Eigen::Vector3d node_center = id3ToPos(node_id3);
  const Eigen::Vector3d half_size = node_scale_ * 0.5;

  const double step = cfg_.occ_sample_resolution;
  const int nx = std::max(1, static_cast<int>(std::ceil(node_scale_.x() / step)));
  const int ny = std::max(1, static_cast<int>(std::ceil(node_scale_.y() / step)));
  const int nz = std::max(1, static_cast<int>(std::ceil(node_scale_.z() / step)));

  node.free_num = 0;
  node.unknown_num = 0;

  for (int ix = 0; ix < nx; ++ix) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int iz = 0; iz < nz; ++iz) {
        Eigen::Vector3d sample_pos = node_center - half_size + Eigen::Vector3d(ix * step, iy * step, iz * step);
        auto st = getVoxelStateLite(sample_pos);
        if (st == VoxelStateLite::Free) node.free_num++;
        else if (st == VoxelStateLite::Unknown) node.unknown_num++;
      }
    }
  }
}

void DtgLowresMap::reloadNodesBf(const Eigen::Vector3d& robot_pos) {
  if (!global_map_) return;
  if (blocks_.empty()) blocks_.resize(1);
  if (!blocks_[0]) blocks_[0] = std::make_shared<LRBlock>();

  Eigen::Vector3d low = robot_pos - cfg_.local_half_size;
  Eigen::Vector3d up = robot_pos + cfg_.local_half_size;

  Eigen::Vector3i low_id3, up_id3;
  posToId3(low, low_id3);
  posToId3(up, up_id3);

  low_id3 = low_id3.cwiseMax(Eigen::Vector3i(0, 0, 0));
  up_id3 = up_id3.cwiseMin(voxel_num_ - Eigen::Vector3i(1, 1, 1));

  const Eigen::Vector3i size_id3 = (up_id3 - low_id3) + Eigen::Vector3i(1, 1, 1);
  if (size_id3.x() <= 0 || size_id3.y() <= 0 || size_id3.z() <= 0) return;

  auto& blk = *blocks_[0];
  blk.origin = low_id3;
  blk.size = size_id3;

  const int n = blk.size.x() * blk.size.y() * blk.size.z();
  blk.local_grid.resize(n);

  for (int ix = 0; ix < blk.size.x(); ++ix) {
    for (int iy = 0; iy < blk.size.y(); ++iy) {
      for (int iz = 0; iz < blk.size.z(); ++iz) {
        const Eigen::Vector3i nid3 = blk.origin + Eigen::Vector3i(ix, iy, iz);
        const int idx = (iz * blk.size.y() + iy) * blk.size.x() + ix;

        if (!blk.local_grid[idx]) blk.local_grid[idx] = std::make_shared<LRNode>();
        auto& node = *blk.local_grid[idx];

        node.path_g = 999999.0f;
        node.root_id = 0;
        node.parent_dir = 13; // self

        computeNodeOccupancy(nid3, node);
      }
    }
  }
}

void DtgLowresMap::getDists(const Eigen::Vector3d& origin,
                            std::vector<Eigen::Vector3d>& targets,
                            std::vector<double>& dist,
                            std::vector<std::vector<Eigen::Vector3d>>& paths) {
  dist.assign(targets.size(), 999999.0);
  paths.assign(targets.size(), {});

  if (!global_map_ || blocks_.empty() || !blocks_[0]) return;
  auto& blk = *blocks_[0];
  if (blk.size.x() <= 0) return;

  Eigen::Vector3i start_id3;
  posToId3(origin, start_id3);
  int start_idx = -1;
  if (!id3ToIndex(blk, start_id3, start_idx)) return;

  auto nodeFeasible = [&](int idx) -> bool {
    if (idx < 0 || idx >= (int)blk.local_grid.size()) return false;
    const auto& n = blk.local_grid[idx];
    if (!n) return false;
    if (n->free_num > 0) return true;
    if (cfg_.allow_unknown && n->unknown_num > 0) return true;
    return false;
  };

  if (!nodeFeasible(start_idx)) return;

  struct QN { int idx; double g; };
  struct Cmp { bool operator()(const QN& a, const QN& b) const { return a.g > b.g; } };
  std::priority_queue<QN, std::vector<QN>, Cmp> pq;

  const int N = blk.size.x() * blk.size.y() * blk.size.z();
  std::vector<double> g_cost(N, 1e18);
  std::vector<int> parent(N, -1);
  std::vector<uint8_t> closed(N, 0);

  g_cost[start_idx] = 0.0;
  pq.push({start_idx, 0.0});

  std::vector<int> tar_idx(targets.size(), -1);
  for (size_t i = 0; i < targets.size(); ++i) {
    Eigen::Vector3i tid3;
    posToId3(targets[i], tid3);
    int ti = -1;
    if (id3ToIndex(blk, tid3, ti)) tar_idx[i] = ti;
  }

  int remaining = 0;
  for (auto ti : tar_idx) if (ti >= 0) remaining++;

  while (!pq.empty() && remaining > 0) {
    auto cur = pq.top();
    pq.pop();

    if (closed[cur.idx]) continue;
    closed[cur.idx] = 1;

    for (size_t k = 0; k < tar_idx.size(); ++k) {
      if (tar_idx[k] == cur.idx && dist[k] > 9e5) {
        dist[k] = cur.g;
        remaining--;
      }
    }

    const int iz = cur.idx / (blk.size.x() * blk.size.y());
    const int rem = cur.idx - iz * (blk.size.x() * blk.size.y());
    const int iy = rem / blk.size.x();
    const int ix = rem - iy * blk.size.x();
    const Eigen::Vector3i cid3 = blk.origin + Eigen::Vector3i(ix, iy, iz);

    for (int d = 0; d < 27; ++d) {
      if (d == 13) continue;
      const Eigen::Vector3i nid3 = cid3 + connect_diff_[d];
      int nidx = -1;
      if (!id3ToIndex(blk, nid3, nidx)) continue;
      if (closed[nidx]) continue;
      if (!nodeFeasible(nidx)) continue;

      const double ng = cur.g + (connect_diff_[d].cast<double>() * cfg_.resolution).norm();
      if (ng + 1e-9 < g_cost[nidx]) {
        g_cost[nidx] = ng;
        parent[nidx] = cur.idx;
        pq.push({nidx, ng});
      }
    }
  }

  for (size_t i = 0; i < targets.size(); ++i) {
    int ti = tar_idx[i];
    if (ti < 0 || dist[i] > 9e5) continue;

    std::vector<Eigen::Vector3d> p;
    int cur = ti;
    while (cur != -1) {
      const int iz = cur / (blk.size.x() * blk.size.y());
      const int rem = cur - iz * (blk.size.x() * blk.size.y());
      const int iy = rem / blk.size.x();
      const int ix = rem - iy * blk.size.x();
      p.emplace_back(id3ToPos(blk.origin + Eigen::Vector3i(ix, iy, iz)));
      if (cur == start_idx) break;
      cur = parent[cur];
    }
    std::reverse(p.begin(), p.end());
    paths[i] = std::move(p);
  }
}

void DtgLowresMap::voronoiPartitionSearch(std::list<std::pair<uint32_t, Eigen::Vector3d>>& search_origins,
                                         std::list<Eigen::Vector3i>& margins) {
  margins.clear();
  if (!global_map_ || blocks_.empty() || !blocks_[0]) return;
  auto& blk = *blocks_[0];
  if (blk.size.x() <= 0) return;

  struct QN { int idx; double g; uint32_t root; };
  struct Cmp { bool operator()(const QN& a, const QN& b) const { return a.g > b.g; } };
  std::priority_queue<QN, std::vector<QN>, Cmp> pq;

  const int N = blk.size.x() * blk.size.y() * blk.size.z();
  for (int i = 0; i < N; ++i) {
    if (blk.local_grid[i]) {
      blk.local_grid[i]->path_g = 1e18;
      blk.local_grid[i]->root_id = 0;
      blk.local_grid[i]->parent_dir = 13;
    }
  }

  for (const auto& origin : search_origins) {
    Eigen::Vector3i id3;
    posToId3(origin.second, id3);
    int idx = -1;
    if (!id3ToIndex(blk, id3, idx)) continue;

    if (blk.local_grid[idx]) {
      blk.local_grid[idx]->path_g = 0.0;
      blk.local_grid[idx]->root_id = origin.first;
      pq.push({idx, 0.0, origin.first});
    }
  }

  while (!pq.empty()) {
    auto cur = pq.top();
    pq.pop();

    if (cur.g > blk.local_grid[cur.idx]->path_g + 1e-9) continue;

    const int iz = cur.idx / (blk.size.x() * blk.size.y());
    const int rem = cur.idx - iz * (blk.size.x() * blk.size.y());
    const int iy = rem / blk.size.x();
    const int ix = rem - iy * blk.size.x();
    const Eigen::Vector3i cid3 = blk.origin + Eigen::Vector3i(ix, iy, iz);

    for (int d = 0; d < 27; ++d) {
      if (d == 13) continue;
      const Eigen::Vector3i nid3 = cid3 + connect_diff_[d];
      int nidx = -1;
      if (!id3ToIndex(blk, nid3, nidx)) continue;
      if (!blk.local_grid[nidx]) continue;

      double edge_cost = (connect_diff_[d].cast<double>() * cfg_.resolution).norm();
      double new_g = cur.g + edge_cost;

      if (new_g < blk.local_grid[nidx]->path_g) {
        blk.local_grid[nidx]->path_g = new_g;
        blk.local_grid[nidx]->root_id = cur.root;
        blk.local_grid[nidx]->parent_dir = d;
        pq.push({nidx, new_g, cur.root});
      } else if (blk.local_grid[nidx]->root_id != 0 && blk.local_grid[nidx]->root_id != cur.root) {
        margins.push_back(nid3);
      }
    }
  }
}

void DtgLowresMap::voronoiPathRetrieve(std::unordered_map<uint32_t, uint32_t>& h_id_to_idx,
                                      std::list<Eigen::Vector3i>& margins,
                                      std::list<std::pair<uint32_t, uint32_t>>& hh_indices,
                                      std::list<std::list<Eigen::Vector3d>>& paths,
                                      std::list<double>& lengths) {
  hh_indices.clear();
  paths.clear();
  lengths.clear();
  if (blocks_.empty() || !blocks_[0]) return;
  auto& blk = *blocks_[0];

  for (const auto& margin_id3 : margins) {
    int margin_idx = -1;
    if (!id3ToIndex(blk, margin_id3, margin_idx)) continue;
    auto& margin_node = blk.local_grid[margin_idx];
    if (!margin_node || margin_node->root_id == 0) continue;

    for (int d = 0; d < 27; ++d) {
      if (d == 13) continue;
      const Eigen::Vector3i nid3 = margin_id3 + connect_diff_[d];
      int nidx = -1;
      if (!id3ToIndex(blk, nid3, nidx)) continue;
      auto& neighbor_node = blk.local_grid[nidx];
      if (!neighbor_node || neighbor_node->root_id == 0) continue;

      if (margin_node->root_id != neighbor_node->root_id) {
        auto it1 = h_id_to_idx.find(margin_node->root_id);
        auto it2 = h_id_to_idx.find(neighbor_node->root_id);
        if (it1 == h_id_to_idx.end() || it2 == h_id_to_idx.end()) continue;

        hh_indices.push_back({it1->second, it2->second});

        std::list<Eigen::Vector3d> path;
        double length = 0.0;

        // from margin to root1
        std::list<Eigen::Vector3d> path1;
        retrieveHPath(margin_id3, path1, length, false);

        // from neighbor to root2
        std::list<Eigen::Vector3d> path2;
        retrieveHPath(nid3, path2, length, true);

        path1.reverse();
        path.splice(path.end(), path1);
        path.splice(path.end(), path2);
        paths.push_back(path);
        lengths.push_back(length);
      }
    }
  }
}

bool DtgLowresMap::retrieveHPath(const Eigen::Vector3d& start, std::list<Eigen::Vector3d>& path, double& length, bool reverse) {
  path.clear();
  length = 0.0;
  if (blocks_.empty() || !blocks_[0]) return false;

  Eigen::Vector3i start_id3;
  posToId3(start, start_id3);
  return retrieveHPath(start_id3, path, length, reverse);
}

bool DtgLowresMap::retrieveHPath(const Eigen::Vector3i& start_id3, std::list<Eigen::Vector3d>& path, double& length, bool reverse) {
  path.clear();
  length = 0.0;
  auto& blk = *blocks_[0];

  int cur_idx = -1;
  if (!id3ToIndex(blk, start_id3, cur_idx)) return false;

  auto& start_node = blk.local_grid[cur_idx];
  if (!start_node || start_node->root_id == 0) return false;

  length = start_node->path_g;
  Eigen::Vector3i cur_id3 = start_id3;

  while (true) {
    auto& node = blk.local_grid[cur_idx];
    path.push_back(id3ToPos(cur_id3));
    if (node->parent_dir == 13) break;

    uint8_t p_dir_inv = 26 - node->parent_dir;
    cur_id3 = cur_id3 - connect_diff_[p_dir_inv];
    if (!id3ToIndex(blk, cur_id3, cur_idx)) break; // Should not happen
  }

  if (reverse) {
    path.reverse();
  }
  return true;
}

}  // namespace dtg
}  // namespace rog_map
