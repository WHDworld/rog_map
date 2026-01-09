# DTG图算法详解与优化方案

**Distance-to-Go Plus (DTG+) 层次化路径图算法**

本文档深入分析 EDEN 探索系统中 DTG 图算法的工作原理、性能瓶颈及优化方案。

---

## 目录

1. [DTG图概述](#1-dtg图概述)
2. [核心数据结构](#2-核心数据结构)
3. [UpdateDTG算法详解](#3-updatedtg算法详解)
4. [路径查询算法](#4-路径查询算法)
5. [性能分析](#5-性能分析)
6. [优化方案](#6-优化方案)
7. [实现建议](#7-实现建议)

---

## 1. DTG图概述

### 1.1 什么是DTG图？

DTG (Distance-to-Go) 图是一个**层次化路径图**，用于高效的探索规划和路径查询。它由以下组件构成：

```
┌─────────────────────────────────────────────────────┐
│                    DTG 图结构                       │
├─────────────────────────────────────────────────────┤
│                                                     │
│  H-nodes (Hierarchical Nodes)                      │
│  ○───────○───────○                                 │
│  │ \     │     / │   ← 层次节点（骨架路径网络）    │
│  │  \    │    /  │                                 │
│  │   \   │   /   │                                 │
│  ○────○──○──○────○                                 │
│  │    │  │  │    │                                 │
│  │    │  │  │    │  HF-edges（连接）               │
│  │    │  │  │    │                                 │
│  ▼    ▼  ▼  ▼    ▼                                 │
│  ×    ×  ×  ×    ×   ← F-nodes（EROI视点）         │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### 1.2 设计目标

1. **快速路径查询**：从机器人当前位置到任意EROI视点的路径
2. **高效目标选择**：在大量候选视点中快速找到最优目标
3. **动态更新**：随着地图更新，实时维护图结构
4. **TSP路径规划**：支持多目标访问顺序优化

### 1.3 工作流程概览

```
时间轴 ──────────────────────────────────────>

UpdateDTG (0.4s)
    ↓
┌────────────────────────────────┐
│ 1. 获取局部H-nodes             │
│ 2. Voronoi分区搜索             │
│ 3. 创建/更新HH-edges           │
│ 4. 创建/更新HF-edges           │
│ 5. 清理失效边                  │
└────────┬───────────────────────┘
         ↓
    DTG图数据
         ↓
FindFastExpTarget (LOCALPLAN时)
    ↓
┌────────────────────────────────┐
│ 1. 从第一个H-node查询HF-edges  │
│ 2. 计算每个视点的增益          │
│ 3. 选择最优视点                │
│ 4. 返回路径                    │
└────────────────────────────────┘
```

---

## 2. 核心数据结构

### 2.1 H_node（层次节点）

```cpp
struct H_node{
    uint32_t id_;                                   // 节点ID
    Eigen::Vector3d pos_;                           // 3D位置
    list<shared_ptr<DTG_edge<H_node, FC_node>>> hf_edges_;  // HF边列表
    list<shared_ptr<DTG_edge<H_node, H_node>>> hh_edges_;   // HH边列表
    uint8_t h_flags_;                               // 状态标志
    shared_ptr<DTG_sch_node> sch_node_;            // 搜索节点（用于A*）
    double last_maintain_t_;                        // 最后维护时间
};
```
**作用**：
- 低分辨率地图中的关键导航点
- 形成全局骨架路径网络
- 支持快速全局路径规划

**创建条件**：
```cpp
if(nearest_hnode_distance > H_thresh_){  // H_thresh_ 通常为5-10米
    CreateHnode(robot_pos);  // 创建新的H-node
}
```

### 2.2 FC_node（前视节点）

```cpp
struct FC_node{
    uint32_t fid_;                                  // EROI id
    uint8_t vid_;                                   // 视点 id
    shared_ptr<DTG_sch_node> sch_node_;            // 搜索节点
    shared_ptr<DTG_edge<H_node, FC_node>> hf_edge_; // 连接的HF边
    uint8_t f_flag_;                                // 状态标志
};
```

**作用**：
- 表示一个EROI的一个有效视点
- 存储视点到最近H-node的路径信息
- 用于快速查询可达视点

**创建条件**：
```cpp
if(EROI_[f_id].local_vps_[v_id] == 0){  // 视点可行
    CreateConnectFnode(vp_pos, f_id, v_id, distance, h_node);
}
```

### 2.3 DTG_edge（边）

```cpp
template <typename HeadNode, typename TailNode>
struct DTG_edge{
    double length_;                      // 路径长度
    double length_s_;                    // 备用长度（多机器人）
    uint8_t e_flag_;                     // 状态标志
    list<Eigen::Vector3d> path_;         // 路径点列表
    shared_ptr<HeadNode> head_n_;        // 起点
    shared_ptr<TailNode> tail_n_;        // 终点
};
```

**边类型**：
1. **HH-edge**：连接两个H-node
   - 路径长度：Voronoi分区边界上的最短路径
   - 作用：支持H-nodes之间的快速路径查询

2. **HF-edge**：连接H-node和F-node
   - 路径长度：从H-node到EROI视点的最短路径
   - 作用：支持从骨架网络到探索目标的路径查询
### 2.4 数据存储

```cpp
class MultiDtgPlus{
private:
    // H-nodes存储：按体素网格索引
    vector<list<h_ptr>> H_depot_;  // vox_num_(0) × vox_num_(1) × vox_num_(2)
    
    // F-nodes存储：按EROI id索引
    tr1::unordered_map<uint32_t, tr1::unordered_map<uint8_t, f_ptr>> F_depot_;
    
    // 当前机器人附近的H-node
    h_ptr root_;
    
    // 体素参数
    Eigen::Vector3i vox_num_;      // 体素网格尺寸
    Eigen::Vector3d vox_scl_;      // 体素分辨率
    Eigen::Vector3d origin_;       // 地图原点
    
    // 参数
    double H_thresh_;              // H-node创建阈值（距离）
};
```

**空间复杂度**：
- H-nodes：`O(V)`，其中 `V = vox_num_(0) × vox_num_(1) × vox_num_(2)`
- F-nodes：`O(E × P)`，其中 `E` 是EROI数量，`P` 是每个EROI的平均视点数
- HH-edges：`O(H²)`，其中 `H` 是局部H-nodes数量（通常<100）
- HF-edges：`O(H × F)`，其中 `F` 是局部F-nodes数量（通常<1000）

---

## 3. UpdateDTG算法详解

### 3.1 算法概览

`UpdateDTG` (实际函数名为 `BfUpdate`) 是DTG图的核心更新算法，每0.4秒执行一次。

```cpp
void MultiDtgPlus::BfUpdate(const Eigen::Vector3d &robot_pos){
    // 步骤1：获取局部H-nodes
    list<h_ptr> H_list;
    GetHnodesBBX(local_upbd, local_lowbd, H_list);
    
    // 步骤2：检查并更新现有HH-edges的可行性
    CheckHHEdges(H_list);
    
    // 步骤3：Voronoi分区搜索（核心算法）
    VoronoiPartitionSearch(search_origins, margins);
    
    // 步骤4：创建新的H-node（如果需要）
    if(distance_to_nearest_hnode > H_thresh_){
        CreateHnode(robot_pos);
    }
    
    // 步骤5：提取并创建HH-edges
    VoronoiPathRetrieve(h2i, margins, hh_edges, paths, lengths);
    ConnectHH(h1, h2, path, path_old, length);
    
    // 步骤6：清理死亡的HF-edges
    EraseDeadHFEdges();
    
    // 步骤7：获取局部有效视点并创建HF-edges
    EROI_->GetLocalValidVps(erois, vps, root_ids, distances);
    for(auto vp : vps){
        CreateConnectFnode(vp_pos, f_id, v_id, distance, h_node);
    }
}
```

### 3.2 步骤1：获取局部H-nodes

```cpp
void MultiDtgPlus::GetHnodesBBX(const Eigen::Vector3d &upbd, 
                                 const Eigen::Vector3d &lowbd, 
                                 list<h_ptr> &H_list){
    // 计算体素范围
    Eigen::Vector3i low = GetVoxId3(lowbd);
    Eigen::Vector3i up = GetVoxId3(upbd);
    
    // 遍历BBX内的所有体素
    for(int x = low(0); x <= up(0); x++)
        for(int y = low(1); y <= up(1); y++)
            for(int z = low(2); z <= up(2); z++){
                int vox_idx = x + y * vox_num_(0) + z * vox_num_(0) * vox_num_(1);
                // 获取该体素内的所有H-nodes
                H_list.insert(H_list.end(), 
                             H_depot_[vox_idx].begin(), 
                             H_depot_[vox_idx].end());
            }
}
```

**时间复杂度**：`O(V_local)`，其中 `V_local` 是局部BBX内的体素数量

**空间优化思路**：
- 使用八叉树代替体素网格，减少空间开销
- 空体素不分配内存

### 3.3 步骤2：检查HH-edges可行性

```cpp
for(auto &h : H_list){
    for(auto &hhe : h->hh_edges_){
        if(hhe->e_flag_ & 1) continue;  // 已检查
        
        // 路径可行性检查
        int status = LRM_->PathCheck(hhe->path_, allow_unknown);
        hhe->e_flag_ |= 1;  // 标记已检查
        
        if(status == 2){
            hhe->e_flag_ |= 64;  // 标记为死亡
            EraseEdge(hhe);
        } else if(status == 0){
            hhe->e_flag_ |= 4;   // 标记为不可行
        } else {
            hhe->e_flag_ &= 251; // 标记为可行
        }
    }
}
```

**优化思路**：
- 增量检查：只检查受地图更新影响的边
- 空间哈希：快速找到受影响的边

### 3.4 步骤3：Voronoi分区搜索（核心）

这是DTG图最关键的算法，用于找到H-nodes的Voronoi分区边界。

```cpp
void LowResMap::VoronoiPartitionSearch(
    const vector<pair<uint32_t, Eigen::Vector3d>> &search_origins,
    list<Eigen::Vector3i> &margins){
    
    ClearTopo();  // 清除拓扑标记
    
    // 多源Dijkstra搜索
    priority_queue<LR_node_ptr, vector<LR_node_ptr>, LRCompare> open_list;
    
    // 初始化：所有H-nodes作为源点
    for(auto &origin : search_origins){
        auto node = GlobalPos2LocalNode(origin.second);
        node->root_id_ = origin.first;  // 记录属于哪个H-node
        node->path_g_ = 0;
        open_list.push(node);
    }
    
    // Dijkstra扩展
    while(!open_list.empty()){
        auto cur_node = open_list.top();
        open_list.pop();
        
        if(cur_node->flag_ & 2) continue;  // 已访问
        cur_node->flag_ |= 2;
        
        // 扩展邻居
        for(auto &neighbor : cur_node->neighbors_){
            if(neighbor->flag_ & 2) continue;
            
            double new_g = cur_node->path_g_ + EdgeCost(cur_node, neighbor);
            
            if(new_g < neighbor->path_g_){
                neighbor->path_g_ = new_g;
                neighbor->root_id_ = cur_node->root_id_;
                neighbor->parent_ = cur_node;
                open_list.push(neighbor);
            }
            
            // 检测Voronoi边界
            if(neighbor->root_id_ != cur_node->root_id_){
                // 这是一个边界节点
                margins.push_back(neighbor->pos_id3_);
            }
        }
    }
}
```
**算法分析**：

- **时间复杂度**：`O((N + E) log N)`，其中 `N` 是低分辨率地图节点数，`E` 是边数
- **空间复杂度**：`O(N)`
- **优势**：一次搜索获得所有H-nodes的Voronoi分区
- **劣势**：地图越大，计算量越大

**示意图**：
```
Voronoi分区示意图（2D切面）：

H1的区域 │ H2的区域 │ H3的区域
         │          │
    ○────┼────○────┼────○
    │    │    │    │    │
    │ H1 │    │ H2 │ H3 │
    │    │    │    │    │
────┼────×────┼────×────┼────  ← Voronoi边界（×标记）
    │    │    │    │    │
    │    │    │    │    │

说明：
- ○ 表示H-node
- × 表示Voronoi边界点（margins）
- │ 表示分区边界
- 每个区域内的点到该区域H-node的距离最短
```
### 3.5 步骤4：提取Voronoi路径

```cpp
void LowResMap::VoronoiPathRetrieve(
    tr1::unordered_map<uint32_t, uint32_t> &h2i,  // H-node id到索引的映射
    list<Eigen::Vector3i> &margins,                 // Voronoi边界点
    list<pair<uint32_t, uint32_t>> &hh_edges,      // 输出：HH边索引对
    list<list<Eigen::Vector3d>> &paths,             // 输出：路径
    list<double> &lengths){                         // 输出：路径长度
    
    for(auto &margin : margins){
        auto node = GlobalPos2LocalNode(margin);
        
        // 找到该边界点的两个相邻H-node
        uint32_t root_id1 = node->root_id_;
        
        // 检查8邻域
        for(auto &neighbor : node->neighbors_){
            uint32_t root_id2 = neighbor->root_id_;
            
            if(root_id1 != root_id2){
                // 找到一条HH边
                hh_edges.push_back({h2i[root_id1], h2i[root_id2]});
                
                // 提取路径
                list<Eigen::Vector3d> path;
                double length = 0;
                
                // 从node回溯到H1
                auto n = node;
                while(n->root_id_ == root_id1){
                    path.push_front(n->pos_);
                    if(n->parent_ != NULL){
                        length += (n->pos_ - n->parent_->pos_).norm();
                        n = n->parent_;
                    } else break;
                }
                
                // 从neighbor回溯到H2
                n = neighbor;
                while(n->root_id_ == root_id2){
                    path.push_back(n->pos_);
                    if(n->parent_ != NULL){
                        length += (n->pos_ - n->parent_->pos_).norm();
                        n = n->parent_;
                    } else break;
                }
                
                paths.push_back(path);
                lengths.push_back(length);
            }
        }
    }
}
```

**时间复杂度**：`O(M × K)`，其中 `M` 是边界点数量，`K` 是平均回溯深度

### 3.6 步骤5：创建HF-edges

```cpp
// 获取局部有效视点
EROI_->GetLocalValidVps(erois, vps, root_ids, distances);

for(int i = 0; i < vps.size(); i++){
    auto &vp = vps[i];          // (f_id, v_id)
    auto &r_id = root_ids[i];   // 最近H-node的id
    auto &d = distances[i];      // 到最近H-node的距离
    
    // 获取视点位置
    vp_pose = EROI_->vps_[vp.second];
    vp_pose.head(3) += EROI_->EROI_[vp.first].center_;
    vp_pos = vp_pose.head(3);
    
    // 找到对应的H-node
    auto hi = h2i.find(r_id);
    h_node = H_vec[hi->second];
    
    // 检查F-node是否已存在
    if(!FindFnode(vp_pos, f_id, v_id, f_node)){
        // 创建新的F-node并连接
        CreateConnectFnode(vp_pos, f_id, v_id, distance, h_node);
    } else {
        // 更新现有连接（如果发现更短路径）
        if(LRM_->RetrieveHPath(vp_pos, path_f, length_f, true)){
            ConnectHF(h_node, f_node, v_id, path_f, path_old, distance);
        }
    }
}
```

**优化思路**：
- 批量查询：一次获取所有局部视点的最近H-node
- 缓存路径：避免重复计算相同路径

### 3.7 完整流程图

```
UpdateDTG (BfUpdate) - 0.4秒定时器
    ↓
┌────────────────────────────────────────────────────────┐
│ 1. GetHnodesBBX                                        │
│    └─> 获取局部地图范围内的所有H-nodes                │
│        时间复杂度: O(V_local)                          │
└────────────┬───────────────────────────────────────────┘
             ↓
┌────────────────────────────────────────────────────────┐
│ 2. CheckHHEdges                                        │
│    └─> 检查现有HH-edges的路径可行性                   │
│        时间复杂度: O(H × E_h × L)                      │
│        H: H-nodes数量, E_h: 每个H-node的边数          │
│        L: 平均路径长度                                 │
└────────────┬───────────────────────────────────────────┘
             ↓
┌────────────────────────────────────────────────────────┐
│ 3. VoronoiPartitionSearch  【核心瓶颈】                │
│    └─> 多源Dijkstra搜索，找到Voronoi分区边界          │
│        时间复杂度: O((N + E) log N)                    │
│        N: 低分辨率地图节点数 (~10,000-100,000)         │
│        E: 边数 (~6N for 6-connectivity)                │
│        实际耗时: 50-200ms（取决于地图大小）            │
└────────────┬───────────────────────────────────────────┘
             ↓
┌────────────────────────────────────────────────────────┐
│ 4. CreateHnode (条件判断)                              │
│    └─> 如果机器人距离最近H-node > H_thresh_           │
│        创建新的H-node                                  │
│        时间复杂度: O(1)                                │
└────────────┬───────────────────────────────────────────┘
             ↓
┌────────────────────────────────────────────────────────┐
│ 5. VoronoiPathRetrieve                                 │
│    └─> 从Voronoi边界提取HH-edges的路径                │
│        时间复杂度: O(M × K)                            │
│        M: 边界点数量, K: 回溯深度                      │
└────────────┬───────────────────────────────────────────┘
             ↓
┌────────────────────────────────────────────────────────┐
│ 6. ConnectHH                                           │
│    └─> 创建或更新HH-edges                             │
│        时间复杂度: O(H²)                               │
└────────────┬───────────────────────────────────────────┘
             ↓
┌────────────────────────────────────────────────────────┐
│ 7. EraseDeadHFEdges                                    │
│    └─> 清理已探索的F-nodes                            │
│        时间复杂度: O(F)                                │
└────────────┬───────────────────────────────────────────┘
             ↓
┌────────────────────────────────────────────────────────┐
│ 8. CreateConnectFnode  【次要瓶颈】                    │
│    └─> 为所有局部有效视点创建HF-edges                 │
│        时间复杂度: O(F × log N)                        │
│        F: 局部视点数量 (~100-1000)                     │
│        实际耗时: 10-50ms                               │
└────────────────────────────────────────────────────────┘

总耗时：50-250ms（取决于地图大小和视点数量）
```
---

## 4. 路径查询算法

### 4.1 FindFastExpTarget（快速探索目标查找）

这是LOCALPLAN状态下用于选择最优探索目标的核心算法。

```cpp
void MultiDtgPlus::FindFastExpTarget(
    const vector<h_ptr> &route,  // TSP生成的H-nodes路径
    const Eigen::Vector4d &ps,   // 当前位置+yaw
    const Eigen::Vector4d &vs,   // 当前速度
    // ... 大量输出参数
){
    // ========== 阶段1：从第一个H-node查询可达视点 ==========
    auto h = route.front();  // 第一个H-node
    
    list<hfe_ptr> candidate_hfe;  // 候选HF-edges
    for(auto &hfe : h->hf_edges_){
        candidate_hfe.emplace_back(hfe);
    }
    
    // ========== 阶段2：计算每个视点的综合得分 ==========
    double best_gain = 0;
    pair<uint32_t, uint8_t> best_tar1;  // (f_id, v_id)
    
    for(auto &hfe : candidate_hfe){
        uint32_t f_id = hfe->tail_n_->fid_;
        uint8_t vp_id = hfe->tail_n_->vid_;
        
        // 获取视点信息
        Eigen::Vector4d vp = EROI_->vps_[vp_id];
        vp.head(3) += EROI_->EROI_[f_id].center_;
        Eigen::Vector3d vp_pos = vp.head(3);
        
        // 获取路径长度（通过LRM查询）
        auto lrn = LRM_->GlobalPos2LocalNode(vp_pos);
        double path_length = lrn->path_g_;
        
        // 计算增益（考虑距离衰减）
        double gain = exp(-lambda_ * path_length / v_max_);
        
        // 考虑yaw角差异
        double yaw_cost = abs(YawDiff(vp(3), ps(3))) / yaw_v_max_;
        gain *= exp(-lambda_yaw_ * yaw_cost);
        
        if(gain > best_gain){
            best_gain = gain;
            best_tar1 = {f_id, vp_id};
        }
    }
    
    // ========== 阶段3：找到第二个目标（如果需要）==========
    // 类似逻辑，找到次优视点...
    
    // ========== 阶段4：生成路径 ==========
    LRM_->RetrieveHPath(best_tar1_pos, path_stem, length);
    
    tar_stem = best_tar1;
    y_stem = EROI_->vps_[best_tar1.second](3);
}
```

### 4.2 增益函数分析

```cpp
double gain = exp(-lambda_ * path_length / v_max_);
```

**含义**：
- `lambda_`：衰减系数（通常0.1-0.3）
- `path_length`：从当前位置到视点的路径长度
- `v_max_`：最大速度
- **物理意义**：路径长度越长，到达时间越久，增益指数衰减

**优化思路**：
- 考虑信息增益：`gain = info_gain × exp(-λ × path_length / v)`
- 考虑探索历史：已访问区域降低增益
- 考虑多机器人：避免多机器人访问同一目标

### 4.3 TSP路径规划

DTG图支持TSP（旅行商问题）路径规划，用于优化多目标访问顺序。

```cpp
bool MultiDtgPlus::TspApproxiPlan(
    const Eigen::Vector3d &ps,    // 起点
    vector<h_ptr> &route_h,        // 输出：H-nodes路径
    vector<Eigen::Vector3d> &path2fh,  // 到第一个H-node的路径
    double &d1){                   // 到第一个H-node的距离
    
    // 步骤1：获取所有局部H-nodes
    list<h_ptr> H_list;
    GetHnodesBBX(local_upbd, local_lowbd, H_list);
    
    // 步骤2：计算距离矩阵
    int n = H_list.size();
    Eigen::MatrixXd dist_mat(n, n);
    
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++){
            if(i == j) dist_mat(i, j) = 0;
            else {
                list<Eigen::Vector3d> path;
                double length;
                RetrieveHPath(H_vec[i], H_vec[j], path, length);
                dist_mat(i, j) = length;
            }
        }
    }
    
    // 步骤3：运行近似TSP算法（Prim's MST）
    vector<int> parent;
    vector<vector<int>> branches;
    Prim(dist_mat, parent, branches);
    
    // 步骤4：生成访问顺序
    // DFS遍历MST，生成访问顺序
    // ...
}
```

**时间复杂度**：`O(H² log H)`，其中 `H` 是局部H-nodes数量

---


