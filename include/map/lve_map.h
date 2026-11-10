#ifndef LVE_MAP_H
#define LVE_MAP_H

#include <map>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <queue>
namespace lve {
    /**
    * @brief 体素状态位定义封装类
    * 
    * 内存大小：严格 1 字节 (uint8_t)
    * 支持多维度状态组合：
    * - 几何层（Bit 0-1）：UNKNOWN=0, FREE=1, OCCUPIED=2
    * - 视觉层（Bit 2）：VISITED=4 （是否被相机观测过）
    * - 语义层（Bit 3）：TARGET=8 （是否为感兴趣目标）
    * - 前沿层（Bit 4）：FRONTIER=16 （是否为前沿体素）
    */
    struct VoxelState {
    public:
        // ==================== 状态常量定义 ====================
        // 几何状态（2位）
        static constexpr uint8_t GEO_UNKNOWN  = 0b00000000;  // 0: 未知
        static constexpr uint8_t GEO_FREE     = 0b00000001;  // 1: 自由
        static constexpr uint8_t GEO_OCCUPIED = 0b00000010;  // 2: 占据
        static constexpr uint8_t MASK_GEO     = 0b00000011;  // 几何层掩码
        
        // 附加属性状态
        static constexpr uint8_t VISITED      = 0b00000100;  // 4: 已访问
        static constexpr uint8_t TARGET       = 0b00001000;  // 8: 目标
        static constexpr uint8_t FRONTIER     = 0b00010000;  // 16: 前沿
        
    private:
        uint8_t data_{GEO_UNKNOWN}; // 底层物理数据，默认初始化为 UNKNOWN
    public:
        // ==================== 构造与赋值 ====================
        VoxelState() = default;
        // 允许通过常量隐式构造，方便如 vec[i] = VoxelState::GEO_FREE; 的直接赋值
        constexpr VoxelState(uint8_t val) : data_(val) {}
        // ==================== 状态查询方法 ====================
        inline uint8_t getGeoState() const { return data_ & MASK_GEO; }
        
        inline bool isUnknown() const  { return getGeoState() == GEO_UNKNOWN; }
        inline bool isFree() const     { return getGeoState() == GEO_FREE; }
        inline bool isOccupied() const { return getGeoState() == GEO_OCCUPIED; }
        
        inline bool isVisited() const  { return (data_ & VISITED) != 0; }
        inline bool isTarget() const   { return (data_ & TARGET) != 0; }
        inline bool isFrontier() const { return (data_ & FRONTIER) != 0; }
        // ==================== 状态修改方法 ====================
        inline void setGeoState(uint8_t geo) {
            data_ = (data_ & ~MASK_GEO) | (geo & MASK_GEO);
        }
        
        inline void setVisited(bool visited) {
            if (visited) data_ |= VISITED;
            else data_ &= ~VISITED;
        }
        
        inline void setTarget(bool target) {
            if (target) data_ |= TARGET;
            else data_ &= ~TARGET;
        }
        
        inline void setFrontier(bool frontier) {
            if (frontier) data_ |= FRONTIER;
            else data_ &= ~FRONTIER;
        }
        // ==================== 方便直接获取底层数据的接口 ====================
        inline uint8_t raw() const { return data_; }
    };
    struct MapParam{
        Eigen::Vector3d map_origin_, map_size_;
        Eigen::Vector3d map_min_, map_max_;
        Eigen::Vector3i map_voxel_num_; // 各轴体素数量
        double resolution_, resolution_inv_;
        double virtual_ceil_height_, ground_height_;
        double obstacles_inflation_;
        Eigen::Vector3i box_min_, box_max_;
        Eigen::Vector3d box_mind_, box_maxd_;
        double default_dist_;
        double local_bound_inflate_;                           // 局部更新边界膨胀值
        int local_map_margin_;                                 // 局部地图余量

        // 概率融合参数 (Log-Odds 贝叶斯更新)
        double p_hit_, p_miss_, p_min_, p_max_, p_occ_;       // 原始概率值
        double prob_hit_log_, prob_miss_log_;                  // 命中/穿透的对数增量
        double clamp_min_log_, clamp_max_log_;                 // 对数概率截断上下限
        double min_occupancy_log_;                             // 判定为"占据"的对数阈值
        double max_ray_length_;                                // 最大射线长度(m)
        double unknown_flag_;                                  // 未知标志偏移量

        // 子网格参数
        Eigen::Vector3d subgrid_size_d_;              // 各轴子网格物理边长（米）
        Eigen::Vector3i subgrid_voxel_num_;           // 各轴每个子网格含多少个体素
        Eigen::Vector3i subgrid_num_;                 // 各轴子网格数量
        int total_subgrids_;                          // 子网格总数
        double exploration_rate_threshold_;            // 触发分析的探索率阈值
    
    };
 
    struct MapData{
        std::vector<double> occupancy_buffer_;
        std::vector<VoxelState> state_buffer_;
        std::vector<char> occupancy_buffer_inflate_;
        std::vector<double> distance_buffer_;
        std::vector<double> distance_buffer_neg_;

        std::vector<short> count_int_, count_miss_;
        std::vector<char> flag_rayend_, flag_visited_;  // 射线终点去重标志、已访问标志
        char raycast_num_;
        std::queue<int> cache_voxel_;
        Eigen::Vector3i local_bound_min_, local_bound_max_;
        Eigen::Vector3d update_min_, update_max_;  // 累积更新范围
        bool reset_updated_box_;


        std::vector<SubGrid> subgrids_;
        std::vector<int> voxel_to_subgrid_;
        std::queue<int> ready_subgrid_queue_;
        std::unordered_set<int> dirty_subgrids_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    double signum(double x);
    double mod(double value, double modulus);
    double intbound(double s, double ds);
    void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
                const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);
    void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
                const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

    class RayCaster {
    public:
        /* data */
        Eigen::Vector3d start_;
        Eigen::Vector3d end_;
        Eigen::Vector3d direction_;
        Eigen::Vector3d min_;
        Eigen::Vector3d max_;
        int x_;
        int y_;
        int z_;
        int endX_;
        int endY_;
        int endZ_;
        double maxDist_;
        double dx_;
        double dy_;
        double dz_;
        int stepX_;
        int stepY_;
        int stepZ_;
        double tMaxX_;
        double tMaxY_;
        double tMaxZ_;
        double tDeltaX_;
        double tDeltaY_;
        double tDeltaZ_;
        double dist_;

        int step_num_;
        RayCaster() {}
        ~RayCaster() {}

        bool setInput(const Eigen::Vector3d& start,
                        const Eigen::Vector3d& end);

        bool step(Eigen::Vector3d& ray_pt);
    };
    
    struct SubGrid {
        Eigen::Vector3i min_idx, max_idx;
        Eigen::Vector3d min_pos, max_pos;
        Eigen::Vector3d center_pos;
        int grid_idx;
        
        double exploration_rate;
        int total_voxels;
        int unknown_voxels;
        
        bool has_frontier = false;

        std::vector<int> free_voxels_list_;
        std::unordered_map<int, int> free_voxels_map_;
        std::vector<int> sub_grid_frontier_voxels_;
        std::vector<int> occ_unobserved_list_;
        std::unordered_map<int, int> occ_unobserved_map_;
        // ==================== 状态机 ====================
        enum class Status : uint8_t {
            UNEXPLORED = 0,         // 初始状态
            EXPLORING,              // 部分体素已被观测
            READY_FOR_ANALYSIS,     // 达到探索率阈值，等待外部 BFS
            ANALYZED                // 外部已完成 BFS 提取
        };
        Status status{Status::UNEXPLORED};
        void addOccUnobserved(const int& idx) {
            if (occ_unobserved_map_.count(idx)) return;
            
            occ_unobserved_list_.push_back(idx);
            occ_unobserved_map_[idx] = static_cast<int>(occ_unobserved_list_.size() - 1);
            
            // 状态转换：首次发现体素时进入 ACCUMULATING
            if (status_ == Status::UNEXPLORED) {
                status_ = Status::EXPLORING;
            }
        }
        void removeOccUnobserved(const int& idx) {
            auto it = occ_unobserved_map_.find(idx);
            if (it == occ_unobserved_map_.end()) return;
            
            int pos = it->second;
            int last_pos = static_cast<int>(occ_unobserved_list_.size() - 1);
            
            if (pos != last_pos) {
                int last_idx = occ_unobserved_list_[last_pos];
                occ_unobserved_list_[pos] = last_idx;
                occ_unobserved_map_[last_idx] = pos;
            }
            
            occ_unobserved_list_.pop_back();
            occ_unobserved_map_.erase(it);
            // 注意：移除体素不影响状态，ANALYZED 后不再变化
        }

        inline double getExplorationRate() const {
            if(total_voxels == 0) return 0.0;
            return 1.0 - static_cast<double>(unknown_voxels) / static_cast<double>(total_voxels);
        }
        inline double getUnkonwnRate() const {
            if(total_voxels == 0) return 0.0;
            return static_cast<double>(unknown_voxels) / static_cast<double>(total_voxels);
        }


    };
    class LVEMap {
    public:
        LVEMap();
        ~LVEMap();

        void initMap(const ros::NodeHandle& nh);
        void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
                            const Eigen::Vector3d& camera_pos);

        void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
        void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
        void indexToSubGrid(const Eigen::Vector3i& id, int& sg_id);
        void posToSubGrid(const Eigen::Vector3d& pos, int& sg_id);

        void boundIndex(Eigen::Vector3i& id);
        int toAddress(const Eigen::Vector3i& id);
        int toAddress(const int& x, const int& y, const int& z);
        Eigen::Vector3i LVEMap::addresstoIndex(const int& adr);
        bool isInMap(const Eigen::Vector3d& pos);
        bool isInMap(const Eigen::Vector3i& idx);
        bool isInBox(const Eigen::Vector3i& id);
        bool isInBox(const Eigen::Vector3d& pos);
        void boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up);
        VoxelState getVoxelState(const Eigen::Vector3d& pos);
        VoxelState getVoxelState(const Eigen::Vector3i& id);
        int getInflateOccupancy(const Eigen::Vector3d& pos);
        int getInflateOccupancy(const Eigen::Vector3i& id);
        double getDistance(const Eigen::Vector3d& pos);
        double getDistance(const Eigen::Vector3i& id);
        double getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
        void updateESDF3d();
        void resetBuffer();
        void resetBuffer(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

        void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
        void getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);
        double getResolution();
        int getVoxelNum();
        
        void updateSubGridOnStateChange(const int& adr, const uint8_t& old_geo, const uint8_t& new_geo);
        void extractGlobalFrontierIncremental();
    private:
        void clearAndInflateLocalMap();
        void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
        void setCacheOccupancy(const int& adr, const int& occ);
        Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);

        std::unique_ptr<MapParam> map_param_ptr_;
        std::unique_ptr<MapData> map_data_ptr_;
        std::unique_ptr<RayCaster> caster_ptr_;        
    };

    inline void LVEMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
        for(int i = 0; i < 3; i++)
            id(i) = floor((pos(i) - map_param_ptr_->map_origin_(i)) * map_param_ptr_->resolution_inv_);
    }
    inline void LVEMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
        for(int i = 0; i < 3; i++)
            pos(i) = (id(i) + 0.5) * map_param_ptr_->resolution_ + map_param_ptr_->map_origin_(i);
    }
    inline void LVEMap::indexToSubGrid(const Eigen::Vector3i& id, int& sg_id) {
        // 用每轴子网格体素数做整数除法，直接定位子网格三维坐标
        int sx = id(0) / map_param_ptr_->subgrid_voxel_num_(0);
        int sy = id(1) / map_param_ptr_->subgrid_voxel_num_(1);
        int sz = id(2) / map_param_ptr_->subgrid_voxel_num_(2);
        // X-Y-Z 行优先线性化，与 toAddress 保持一致
        sg_id = sx * map_param_ptr_->subgrid_num_(1) * map_param_ptr_->subgrid_num_(2)
              + sy * map_param_ptr_->subgrid_num_(2) + sz;
    }
    inline void LVEMap::posToSubGrid(const Eigen::Vector3d& pos, int& sg_id) {
        Eigen::Vector3i id;
        posToIndex(pos, id);
        indexToSubGrid(id, sg_id);
    }
    inline void LVEMap::boundIndex(Eigen::Vector3i& id) {
        Eigen::Vector3i id1;
        id1(0) = std::max(std::min(id(0), map_param_ptr_->map_voxel_num_(0) - 1), 0);
        id1(1) = std::max(std::min(id(1), map_param_ptr_->map_voxel_num_(1) - 1), 0);
        id1(2) = std::max(std::min(id(2), map_param_ptr_->map_voxel_num_(2) - 1), 0);
        id = id1;
    }
    inline int LVEMap::toAddress(const Eigen::Vector3i& id) {
        return toAddress(id(0), id(1), id(2));
    }
    inline int LVEMap::toAddress(const int& x, const int& y, const int& z) {
        return x * map_param_ptr_->map_voxel_num_.y() * map_param_ptr_->map_voxel_num_.z() +
               y * map_param_ptr_->map_voxel_num_.z() + z;
    }
    inline Eigen::Vector3i LVEMap::addresstoIndex(const int& adr)
    {
        Eigen::Vector3i id;
        // 利用取模和整除的降维法，完美还原 x, y, z
        id(2) = adr % map_param_ptr_->map_voxel_num_(2);
        adr /= map_param_ptr_->map_voxel_num_(2);
        id(1) = adr % map_param_ptr_->map_voxel_num_(1);
        adr /= map_param_ptr_->map_voxel_num_(1);
        id(0) = adr;
        return id;
    }
    inline bool LVEMap::isInMap(const Eigen::Vector3d& pos) {
        Eigen::Vector3i id;
        if(pos(0) < map_param_ptr_->map_min_(0) ||
           pos(1) < map_param_ptr_->map_min_(1) ||
           pos(2) < map_param_ptr_->map_min_(2))
            return false;
        if(pos(0) > map_param_ptr_->map_max_(0) ||
           pos(1) > map_param_ptr_->map_max_(1) ||
           pos(2) > map_param_ptr_->map_max_(2))
            return false;
        return true;
    }
    inline bool LVEMap::isInMap(const Eigen::Vector3i& id) {
        if(id(0) < 0 || id(1) < 0 || id(2) < 0) return false;
        if(id(0) > map_param_ptr_->map_voxel_num_(0) - 1 ||
           id(1) > map_param_ptr_->map_voxel_num_(1) - 1 ||
           id(2) > map_param_ptr_->map_voxel_num_(2) - 1)
            return false;
        return true;
    }
    inline VoxelState LVEMap::getVoxelState(const Eigen::Vector3d& pos) {
        Eigen::Vector3i id;
        posToIndex(pos, id);
        return getVoxelState(id);
    }
    inline VoxelState LVEMap::getVoxelState(const Eigen::Vector3i& id) {
        if(!isInMap(id)) return VoxelState(VoxelState::GEO_UNKNOWN); 
        return map_data_ptr_->state_buffer_[toAddress(id)];
    }
    inline int LVEMap::getInflateOccupancy(const Eigen::Vector3d& pos) {
        Eigen::Vector3i id;
        posToIndex(pos, id);
        return getInflateOccupancy(id);
    }
    inline int LVEMap::getInflateOccupancy(const Eigen::Vector3i& id) {
        if (!isInMap(id)) return -1;
        return int(map_data_ptr_->occupancy_buffer_inflate_[toAddress(id)]);
    }
    inline void LVEMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
        int num = 0;
        for (int x = -step; x <= step; ++x)
            for (int y = -step; y <= step; ++y)
                for (int z = -step; z <= step; ++z) {
                    pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
                }
    }
}

#endif // LVE_MAP_H