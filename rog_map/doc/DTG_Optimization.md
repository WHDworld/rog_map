# DTG 图算法优化设计方案

本文档旨在针对 **快速TSP视点查询** 和 **安全最短路径规划** 两个核心需求，对现有的 DTG (Distance-to-Go) 算法提出优化设计方案。

---

## 1. 安全最短路径规划 (Safe Shortest Path)

**目标**：在 DTG 图中快速找到两点（如当前位姿到目标 H-node）之间的最短路径，且路径必须安全（避开障碍物）并尽可能平滑。

### 1.1 现状分析
目前的 DTG 设计使用 Voronoi 分区生成的骨架路径。虽然安全，但往往不是最短路径，且路径可能包含不必要的“绕路”（因为 Voronoi 路径倾向于保持在障碍物中间）。

### 1.2 优化方案：A* 搜索 + 路径截断 (Path Shortening)

我们建议采用两阶段策略：

#### 阶段一：基于 H-Graph 的 A* 搜索
在 H-nodes 构成的稀疏拓扑图上运行 A* 算法。

*   **节点 (Node)**: H-nodes 和 F-nodes。
*   **边 (Edge)**: 现有的 HH-edges 和 HF-edges。
*   **启发式函数 (Heuristic)**: 欧几里得距离 $h(n) = ||pos(n) - pos(target)||$。
*   **Cost**: 边的实际理路径长度 `length_`。

**算法流程**:
1.  找到离起点最近的 H-node ($H_{start}$) 和目标 H-node ($H_{goal}$)。
2.  在图上运行 A*，得到节点序列: $P_{nodes} = \{H_{start}, H_1, H_2, \dots, H_{goal}\}$。
3.  提取原始几何路径: $P_{geom} = \{p_{start}, \dots, p_{goal}\}$ (连接所有边的 `path_` 点)。

#### 阶段二：路径截断 (Punt-and-Shoot / Path Shortening)
原始几何路径通常蜿蜒曲折。我们需要在保证安全的前提下“拉直”路径。

**算法 (Greedy Path Shortening)**:
1.  初始化优化路径 $P_{opt} = \{p_{start}\}$。
2.  设当前锚点 $p_{curr} = p_{start}$。
3.  从路径末端开始，向后遍历原始路径点 $p_i \in P_{geom}$：
    *   检查线段 $(p_{curr}, p_i)$ 是否**无碰撞** (Collision-Free, 使用 `LRM_->PathCheck` 或 Raycast)。
    *   如果无碰撞：
        *   将 $p_i$ 加入 $P_{opt}$。
        *   更新 $p_{curr} = p_i$。
        *   跳过中间所有点，继续下一轮迭代（从末端开始）。
    *   如果由于障碍物受阻，则尝试下一个点 $p_{i-1}$。

**优势**:
*   相比直接使用 Voronoi 路径，能显著减少路径长度。
*   相比基于梯度的轨迹优化（如 GCOPTER），计算量极小，适合前端快速查询。
*   保证路径安全性（基于 LRM 检查）。

---

## 2. 快速 TSP 视点查询 (Fast TSP Viewpoint Query)

**目标**：在探索过程中，从大量候选视点 (Viewpoints) 中筛选出最佳的一组目标，并确定访问顺序，以最大化探索效率。

### 2.1 现状分析
当前方案可能在全量 H-nodes 上进行全局 TSP 或增益计算，随着地图增大，计算距离矩阵 (`GetHnodesBBX` + `RetrieveHPath`) 的开销为 $O(N^2)$，导致性能下降。

### 2.2 优化方案：Top-K 聚类 + 局部 TSP

我们不试图解决全局 TSP 问题，而是转化为 **收益最大化问题 (Orienteering Problem)** 的简化版。

#### 步骤 1：视点聚类与预筛选 (Pre-selection)
不要对所有视点做 TSP。
1.  **聚类**: 将 F-nodes 按空间邻域聚类（例如通过最近的 H-node 归类）。
2.  **Top-K 筛选**: 计算每个簇的粗略收益 (Heuristic Gain)。
    *   $Gain_{cluster} = \frac{\sum InfoGain(v)}{Distance(Robot, ClusterCenter)}$
    *   选取收益最高的 $K$ 个簇（例如 K=5~10）。

#### 步骤 2：构建局部距离矩阵 (Local Distance Matrix)
仅针对筛选出的 $M$ 个候选视点（来自 Top-K 簇）+ 机器人当前位置构建距离矩阵。
*   矩阵大小从 $N \times N$ 降低到 $(M+1) \times (M+1)$，其中 $M \ll N$。
*   **距离计算**: 使用上述优化后的 A* 算法快速计算 H-node 间的图距离。

#### 步骤 3：求解局部 TSP
对于小规模问题 ($M < 15$)，可以使用精确算法或遗传算法；对于实时性要求高的情况，使用 **最近邻 (Nearest Neighbor)** 或 **插入法 (Insertion Heuristic)** 即可。

*   **路径**: $Robot \to VP_1 \to VP_2 \to \dots \to VP_k$
*   **综合得益**: $Score = \sum_{i=1}^k (Gain(VP_i) \times \gamma^{t_i})$，其中 $\gamma$ 是时间衰减因子。

---

## 3. 实现细节建议

### 3.1 增加 A* 接口
在 `MultiDtgPlus` 类中添加 `AStarSearch` 函数：

```cpp
bool MultiDtgPlus::AStarSearch(h_ptr start, h_ptr goal, list<Eigen::Vector3d> &path_out);
```

### 3.2 增加视线检查接口
在 `LowResMap` 中优化 `PathCheck`，支持两点间的快速射线检测：

```cpp
// 检查两点间连线是否安全
bool LowResMap::IsLineSafe(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);
```

### 3.3 缓存距离矩阵
为了避免重复计算，可以在 H-node 中缓存“到其他枢纽节点的距离表”（Distance Table），利用 Floyd-Warshall 或 Landmark Heuristic 在后台线程更新，供前台快速查询。

---

## 4. 总结

| 功能 | 原始设计 | 优化设计 | 预期提升 |
| :--- | :--- | :--- | :--- |
| **路径搜索** | Voronoi 骨架路径提取 (BFS/Dijkstra) | **H-Graph A*** + **路径截断** | 路径更短、更直，计算更快 |
| **TSP 规划** | 全局 H-nodes 距离矩阵计算 ($O(N^2)$) | **Top-K 筛选** + **局部 TSP** | 大幅降低计算量，支持大规模地图 |
| **安全性** | 依赖骨架远离障碍物 | **显式碰撞检测** | 在保证安全的前提下最大化通行效率 |
