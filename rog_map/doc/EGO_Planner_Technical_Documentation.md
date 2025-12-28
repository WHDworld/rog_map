# EGO-Planner 技术文档

## 目录
1. [概述](#1-概述)
2. [MINCO 轨迹优化原理](#2-minco-轨迹优化原理)
3. [状态机设计](#3-状态机设计)
4. [避障实现](#4-避障实现)
5. [路径规划流程](#5-路径规划流程)
6. [期望状态规划](#6-期望状态规划)
7. [参数配置](#7-参数配置)
8. [使用方法](#8-使用方法)
9. [话题接口](#9-话题接口)

---

## 1. 概述

### 1.1 简介
EGO-Planner 是一个基于 **MINCO (Minimum Control Effort)** 轨迹优化的无人机运动规划器，专为探索、导航等场景设计。相比传统的 B-spline 轨迹优化，MINCO 具有以下优势：

- **更优的动力学性能**：最小化控制力矩（jerk），生成更平滑的轨迹
- **端点速度约束**：支持指定起点和终点速度，适合连续探索任务
- **快速优化**：使用 L-BFGS 优化算法，收敛速度快
- **灵活性**：支持位置、速度、yaw 的完整状态规划

### 1.2 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    EGO-Planner 系统                      │
├─────────────────────────────────────────────────────────┤
│  ┌──────────────┐    ┌──────────────┐    ┌──────────┐  │
│  │ FSM 状态机   │───▶│ 规划管理器    │───▶│ 轨迹服务器│  │
│  │ (ego_replan  │    │ (ego_planner │    │ (ego_traj│  │
│  │  _fsm)       │    │  _manager)   │    │  _server)│  │
│  └──────────────┘    └──────────────┘    └──────────┘  │
│         │                     │                  │       │
│         ▼                     ▼                  ▼       │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────┐  │
│  │ 目标点管理    │    │ A* 全局搜索  │    │ 位置命令  │  │
│  │ 视点输入      │    │ MINCO 优化   │    │ 轨迹可视化│  │
│  └──────────────┘    └──────────────┘    └──────────┘  │
└─────────────────────────────────────────────────────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │   ROG-Map       │
                    │ (Inflated ESDF) │
                    └─────────────────┘
```


---

## 2. MINCO 轨迹优化原理

### 2.1 基本概念

MINCO (Minimum Control Effort) 通过最小化 **jerk**（加速度的导数）来生成平滑轨迹。对于无人机而言，最小化 jerk 等价于最小化控制力矩的变化率，从而提高飞行的稳定性和能效。

### 2.2 轨迹表示

轨迹由 M 段多项式组成，每段是一个 5 阶多项式（MINCO 使用 5 阶以满足位置、速度、加速度约束）。

第 i 段轨迹（t ∈ [0, T_i]）的数学表示为 5 阶多项式，包含 x, y, z 三个维度的系数。

### 2.3 优化目标

MINCO 的优化目标是最小化整条轨迹的 **jerk**（加速度的导数）。

优化目标函数：
- J_smooth：平滑度代价（最小化 jerk 的积分）
- J_time：时间代价（鼓励更快的轨迹）
- J_obs：避障代价（保持与障碍物的安全距离）
- J_feas：可行性代价（满足速度和加速度限制）

### 2.4 约束条件

#### 2.4.1 端点约束
起点约束（位置、速度、加速度）和终点约束必须精确满足。

#### 2.4.2 连续性约束
相邻段在连接点处保证位置、速度、加速度连续。

#### 2.4.3 动力学约束
- 速度约束：||v(t)|| ≤ v_max
- 加速度约束：||a(t)|| ≤ a_max

#### 2.4.4 避障约束
对于轨迹上的采样点，距离最近障碍物的距离应满足：d(p) ≥ d_safe

实现中使用软约束和硬约束两种方式：
- **软约束（Soft Constraint）**：当 d < d_soft 时施加二次方惩罚
- **硬约束（Hard Constraint）**：当 d < d_safe 时施加三次方惩罚

### 2.5 完整优化问题

总代价函数为上述各项的加权和，使用 **L-BFGS** 算法求解，关键是计算目标函数关于优化变量（系数和时间）的梯度。


---

## 3. 状态机设计

EGO-Planner 的 FSM（Finite State Machine）负责管理整个规划流程，确保从起飞到执行轨迹的平滑过渡。

### 3.1 状态定义

```cpp
enum FSM_EXEC_STATE
{
  INIT,            // 初始化状态
  PRE_ARM,         // 预解锁（等待 PX4 连接）
  TAKEOFF,         // 起飞状态
  WAIT_TARGET,     // 等待目标点
  GEN_NEW_TRAJ,    // 生成新轨迹
  REPLAN_TRAJ,     // 重规划轨迹
  EXEC_TRAJ,       // 执行轨迹
  EMERGENCY_STOP   // 紧急停止
};
```

### 3.2 状态转换图

```
         ┌─────┐
         │INIT │
         └──┬──┘
            │ have_odom
            ▼
        ┌────────┐
        │PRE_ARM │
        └───┬────┘
            │ PX4 connected + OFFBOARD + armed
            ▼
        ┌────────┐
        │TAKEOFF │
        └───┬────┘
            │ reached height
            ▼
      ┌────────────┐
      │WAIT_TARGET │◄───────┐
      └─────┬──────┘        │
            │ have_target   │
            ▼               │
     ┌──────────────┐       │
     │GEN_NEW_TRAJ  │       │
     └──────┬───────┘       │
            │ success       │
            ▼               │
       ┌─────────┐          │
       │EXEC_TRAJ│          │
       └────┬────┘          │
            ├─ periodic ────┤
            ├─ reached goal ─┘
            ▼
      ┌──────────────┐
      │EMERGENCY_STOP│
      └──────────────┘
```

### 3.3 状态详解

#### 3.3.1 INIT 状态
**功能**：等待里程计数据
**转换条件**：收到第一帧 odometry → 转到 PRE_ARM

```cpp
case INIT:
{
  if (!have_odom_) goto force_return;
  initial_pos_ = odom_pos_;
  changeFSMExecState(PRE_ARM, "FSM");
  break;
}
```

#### 3.3.2 PRE_ARM 状态
**功能**：等待 PX4 连接，切换 OFFBOARD 模式并解锁
**转换条件**：FCU 连接成功 && OFFBOARD 模式 && 已解锁 → 转到 TAKEOFF

关键实现：
- 持续发送悬停命令作为预设点
- 每 2 秒请求一次 OFFBOARD 模式
- 每 2 秒请求一次解锁

#### 3.3.3 TAKEOFF 状态
**功能**：执行平滑起飞，使用 smoothstep 函数生成加速度轨迹
**转换条件**：达到目标高度 → 转到 WAIT_TARGET

起飞轨迹生成（smoothstep）：
```cpp
double t_ratio = std::min(elapsed / takeoff_accel_time_, 1.0);
double smooth_t = t_ratio * t_ratio * (3.0 - 2.0 * t_ratio);  // S曲线
double current_target_z = initial_pos_.z() + smooth_t * takeoff_height_;
```

#### 3.3.4 WAIT_TARGET 状态
**功能**：悬停等待目标点
**转换条件**：收到目标点 (have_target_) && 收到触发信号 (have_trigger_) → 转到 GEN_NEW_TRAJ

#### 3.3.5 GEN_NEW_TRAJ 状态
**功能**：从全局轨迹生成局部优化轨迹
**转换条件**：
- 规划成功 → 转到 EXEC_TRAJ
- 规划失败 → 重试 GEN_NEW_TRAJ

#### 3.3.6 REPLAN_TRAJ 状态
**功能**：从当前执行的轨迹重新规划
**转换条件**：
- 规划成功 → 转到 EXEC_TRAJ
- 规划失败 → 重试 REPLAN_TRAJ

#### 3.3.7 EXEC_TRAJ 状态
**功能**：执行轨迹，监控重规划条件
**转换条件**：
- 到达目标点 → 转到 WAIT_TARGET（或下一个路径点）
- 定期重规划（t_cur > replan_thresh_）→ 转到 REPLAN_TRAJ

关键逻辑：
```cpp
// 检查是否到达目标
if (touch_goal_ && t_cur > info->duration - 1e-2)
{
  if (wpt_id_ < (int)wps_.size() - 1)
  {
    // 有下一个路径点
    wpt_id_++;
    planNextWaypoint(wps_[wpt_id_], wps_[wpt_id_-1]);
    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
  }
  else
  {
    // 完成所有路径点
    changeFSMExecState(WAIT_TARGET, "FSM");
  }
}
```

#### 3.3.8 EMERGENCY_STOP 状态
**功能**：紧急停止，悬停在当前位置
**转换条件**：无自动转换，需人工干预

### 3.4 碰撞安全监控

独立的安全线程每 50ms 检查一次未来轨迹是否会碰撞：

```cpp
void checkCollisionCallback(const ros::TimerEvent &e)
{
  if (exec_state_ == EXEC_TRAJ)
  {
    // 检查未来 emergency_time_ 秒内的轨迹
    for (double t = t_cur; t < std::min(info->duration, t_cur + emergency_time_); t += 0.05)
    {
      Eigen::Vector3d pos = info->traj.getPos(t);
      if (rog_map_->isOccupiedInflate(pos))
      {
        if (enable_fail_safe_)
        {
          ROS_ERROR("Collision detected, emergency stop!");
          callEmergencyStop(odom_pos_);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }
        return;
      }
    }
  }
}
```


---

## 4. 避障实现

### 4.1 多层避障策略

EGO-Planner 采用三层避障策略：

```
┌─────────────────────────────────────────┐
│  Layer 1: A* 全局路径搜索               │
│  - 使用 inflated map                    │
│  - 保证全局路径无碰撞                   │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│  Layer 2: MINCO 优化避障                │
│  - Hard constraint (d < d_safe)         │
│  - Soft constraint (d < d_soft)         │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│  Layer 3: 实时碰撞监控                  │
│  - 检查未来轨迹                         │
│  - 触发紧急停止                         │
└─────────────────────────────────────────┘
```

### 4.2 Inflated Map

**原理**：将障碍物膨胀一个安全距离，使得路径搜索自然远离障碍物

**实现**：ROG-Map 提供 `isOccupiedInflate(pos)` 接口

```cpp
// A* 搜索中的碰撞检查
bool isOccupied(const Eigen::Vector3d &pos) {
  return rog_map_->isOccupiedInflate(pos);
}
```

**膨胀距离配置**：
```yaml
rog_map:
  inflate_distance: 0.3  # 膨胀距离（米）
```

### 4.3 A* 全局路径搜索

**搜索空间**：3D 网格，分辨率可配置（默认 0.2m）

**启发式函数**：欧几里得距离 h(n) = ||p_n - p_goal||

**代价函数**：g(n) = g(parent(n)) + ||p_n - p_parent(n)||

**关键实现**：
```cpp
int ROGAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
  // 初始化起点
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  cur_node->g_score = 0.0;
  cur_node->f_score = getHeu(cur_node->position, end_pt);
  
  open_set_.push(cur_node);
  
  while (!open_set_.empty())
  {
    cur_node = open_set_.top();
    open_set_.pop();
    
    // 到达目标
    if (isGoal(cur_node->position, end_pt))
      return REACH_END;
    
    // 扩展邻居
    for (auto& offset : neighbor_offsets_)
    {
      Eigen::Vector3d neighbor_pos = cur_node->position + offset;
      
      // 碰撞检查（使用 inflated map）
      if (rog_map_->isOccupiedInflate(neighbor_pos))
        continue;
      
      // 更新 g 和 f 值，加入 open set
      // ...
    }
  }
  
  return NO_PATH;
}
```

### 4.4 MINCO 优化避障

#### 4.4.1 距离场计算

使用 ROG-Map 的 ESDF（Euclidean Signed Distance Field）快速计算点到最近障碍物的距离和梯度：

```cpp
double dist = rog_map_->getDistance(pos);
Eigen::Vector3d grad = rog_map_->getGradient(pos);
```

#### 4.4.2 硬约束（Hard Constraint）

当 d < d_safe 时，施加三次方惩罚：

C_obs(p) = w_obs * (d_safe - d(p))^3

梯度：∇C_obs = -3 * w_obs * (d_safe - d)^2 * ∇d

**实现**：
```cpp
if (dist < obs_clearance_)
{
  double diff = obs_clearance_ - dist;
  double cost = wei_obs_ * diff * diff * diff;
  Eigen::Vector3d grad_cost = -3.0 * wei_obs_ * diff * diff * grad;
  
  costs(0) += cost;
  gdT += grad_cost.dot(vel) * piece_dur;  // 对时间的梯度
  // ... 对控制点的梯度
}
```

#### 4.4.3 软约束（Soft Constraint）

当 d_safe < d < d_soft 时，施加二次方惩罚，使轨迹更远离障碍物：

C_obs_soft(p) = w_obs_soft * (d_soft - d(p))^2

#### 4.4.4 控制点采样

为了高效计算避障代价，不是对整条轨迹进行密集采样，而是在 **关键控制点** 附近采样：

```cpp
// 每段轨迹采样 cps_num_prePiece_ 个控制点
for (int i = 0; i < piece_num; i++)
{
  for (int j = 0; j < cps_num_prePiece_; j++)
  {
    double t_sample = j * piece_dur / (cps_num_prePiece_ - 1);
    Eigen::Vector3d pos = traj.getPos(t_start + t_sample);
    
    // 计算避障代价
    double dist = rog_map_->getDistance(pos);
    if (dist < obs_clearance_soft_)
    {
      // ... 添加代价和梯度
    }
  }
}
```

### 4.5 路径缩短（Path Shortening）

A* 生成的路径通常不是最短的（受网格分辨率限制）。我们使用 **视线检查** 缩短路径：

```cpp
void shortenPath(std::vector<Eigen::Vector3d> &path)
{
  int i = 0;
  while (i < (int)path.size() - 1)
  {
    // 尝试跳过中间点
    int j = i + 2;
    bool can_skip = true;
    
    while (j < (int)path.size() && can_skip)
    {
      // 检查从 path[i] 到 path[j] 的直线是否碰撞
      Eigen::Vector3d dir = (path[j] - path[i]).normalized();
      double dist = (path[j] - path[i]).norm();
      
      for (double t = 0; t < dist; t += 0.1)
      {
        Eigen::Vector3d check_pt = path[i] + t * dir;
        if (rog_map_->isOccupiedInflate(check_pt))
        {
          can_skip = false;
          break;
        }
      }
      
      if (can_skip) j++;
    }
    
    // 移除中间点
    path.erase(path.begin() + i + 1, path.begin() + j - 1);
    i++;
  }
}
```


---

## 5. 路径规划流程

### 5.1 完整流程图

```
┌──────────────────┐
│  收到目标点/视点  │
└────────┬─────────┘
         │
         ▼
┌─────────────────────────┐
│ A* 搜索全局路径         │
│ - 使用 inflated map     │
│ - 生成路径点序列        │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 路径缩短                │
│ - 视线检查              │
│ - 移除冗余点            │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 分配时间                │
│ - 根据速度限制          │
│ - 计算每段时间 T_i      │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ MINCO 初始化            │
│ - 最小 snap 初始轨迹    │
│ - 满足端点约束          │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 迭代优化（L-BFGS）      │
│ - 添加避障代价          │
│ - 添加动力学代价        │
│ - 时间调整              │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 动力学检查              │
│ - 速度是否超限          │
│ - 加速度是否超限        │
└────────┬────────────────┘
         │
    ┌────┴────┐
    │ 可行？   │
    └────┬────┘
         │ No: 时间重分配
         │ Yes
         ▼
┌──────────────────┐
│ 发布轨迹         │
│ - PolyTraj msg   │
│ - 可视化         │
└──────────────────┘
```

### 5.2 关键步骤详解

#### 5.2.1 A* 全局搜索

```cpp
bool EGOPlannerManager::planGlobalTrajWaypoints(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, 
    const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints, 
    const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
  // A* 搜索从起点到第一个路径点
  int status = a_star_->search(start_pos, waypoints[0]);
  
  if (status == ROGAstar::NO_PATH)
  {
    ROS_ERROR("[EGOPlannerManager] A* search failed!");
    return false;
  }
  
  // 获取路径并缩短
  std::vector<Eigen::Vector3d> path = a_star_->getPath();
  shortenPath(path);
  
  // 分配时间
  std::vector<double> segment_times = allocateTime(path);
  
  // 初始化 MINCO
  poly_traj::MinJerkOpt initMJO;
  initMJO.reset(start_pos, start_vel, start_acc,
                path.back(), end_vel, end_acc,
                path.size() - 1);
  
  // 设置中间路径点
  for (int i = 1; i < (int)path.size() - 1; i++)
  {
    initMJO.setInnerPoint(i - 1, path[i]);
  }
  
  // 生成初始轨迹
  initMJO.generate(segment_times);
  
  // 保存全局轨迹
  traj_.setGlobalTraj(initMJO.getTraj(), ros::Time::now().toSec());
  
  return true;
}
```

#### 5.2.2 时间分配

根据路径点之间的距离和速度限制分配时间：

```cpp
std::vector<double> allocateTime(const std::vector<Eigen::Vector3d> &path)
{
  std::vector<double> times;
  
  for (int i = 0; i < (int)path.size() - 1; i++)
  {
    double dist = (path[i+1] - path[i]).norm();
    double time = dist / (pp_.max_vel_ * 0.6);  // 使用 60% 最大速度
    time = std::max(time, 0.5);  // 最小时间
    times.push_back(time);
  }
  
  return times;
}
```

#### 5.2.3 局部重规划

从当前执行的轨迹重新规划，保证连续性：

```cpp
bool EGOPlannerManager::reboundReplan(
    const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, 
    const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt, 
    const Eigen::Vector3d &local_target_vel, bool flag_use_poly_init, 
    bool flag_randomPolyTraj, bool touch_goal)
{
  // 从全局轨迹获取控制点
  Eigen::MatrixXd ctrl_pts = traj_.global_traj.traj.getInnerPoints();
  
  // 时间分配
  std::vector<double> segment_times = allocateTime(ctrl_pts);
  
  // 使用多项式轨迹优化
  poly_traj::MinJerkOpt initMJO;
  initMJO.reset(start_pt, start_vel, start_acc,
                local_target_pt, local_target_vel, Eigen::Vector3d::Zero(),
                ctrl_pts.cols());
  
  for (int i = 0; i < ctrl_pts.cols(); i++)
  {
    initMJO.setInnerPoint(i, ctrl_pts.col(i));
  }
  
  initMJO.generate(segment_times);
  
  // MINCO 优化
  bool success = ploy_traj_opt_->OptimizeTrajectory(
      initMJO, start_pt, start_vel, start_acc,
      local_target_pt, local_target_vel, Eigen::Vector3d::Zero(),
      touch_goal);
  
  if (success)
  {
    setLocalTrajFromOpt(ploy_traj_opt_->getMinJerkOpt(), touch_goal);
  }
  
  return success;
}
```

### 5.3 动力学可行性检查

优化后检查轨迹是否满足速度和加速度限制。如果不可行，重新分配时间（增大 1.2 倍）并重新优化。


---

## 6. 期望状态规划

### 6.1 设计动机

在探索任务中，仅仅规划位置是不够的。我们需要：

1. **期望 yaw**：相机朝向特定方向以观察目标
2. **期望速度**：连续探索时保持运动连续性，避免频繁停止
3. **位置-速度-yaw 一致性**：确保无人机在到达视点时以正确的姿态和速度

### 6.2 Waypoint 数据结构

```cpp
struct Waypoint {
  Eigen::Vector3d pos;   // 期望位置
  double yaw;            // 期望 yaw 角（弧度）
  Eigen::Vector3d vel;   // 期望速度（到达该点时的速度）
};
```

### 6.3 输入接口

#### 6.3.1 标准 RViz Goal（geometry_msgs/PoseStamped）

适用于单点导航，从 Pose 中提取 yaw：

```cpp
void waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
{
  end_pt_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  
  // 从四元数提取 yaw
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                       msg->pose.orientation.y, msg->pose.orientation.z);
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  end_yaw_ = euler(2);
  
  // 默认零速度（停止）
  end_vel_ = Eigen::Vector3d::Zero();
  
  wps_.push_back(Waypoint(end_pt_, end_yaw_, end_vel_));
}
```

**话题**：`/goal` 或 `/move_base_simple/goal`

#### 6.3.2 探索视点（quadrotor_msgs/PositionCommand）

适用于探索任务，完整指定位置、yaw、速度：

```cpp
void viewpointCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
{
  end_pt_ << msg->position.x, msg->position.y, msg->position.z;
  end_yaw_ = msg->yaw;
  end_vel_ << msg->velocity.x, msg->velocity.y, msg->velocity.z;
  
  wps_.push_back(Waypoint(end_pt_, end_yaw_, end_vel_));
}
```

**话题**：`/exploration/viewpoint`

**消息格式**：
```
geometry_msgs/Point position
  float64 x, y, z
geometry_msgs/Vector3 velocity
  float64 x, y, z
float64 yaw
```

### 6.4 终点速度约束

在 MINCO 优化中，终点速度作为硬约束。如果用户指定了非零终点速度，优化器将确保轨迹在到达终点时具有该速度。

```cpp
// 在 reboundReplan 中
if (touch_goal_ && end_vel_.norm() > 1e-3)
{
  local_target_vel_ = end_vel_;  // 使用用户指定的终点速度
}
```

### 6.5 Yaw 规划

#### 6.5.1 轨迹消息增强

在 `PolyTraj.msg` 中添加 yaw 信息：

```
int32 traj_id
time start_time
uint8 order
float32[] coef_x
float32[] coef_y
float32[] coef_z
float32[] duration
float32 start_yaw   # 起始 yaw
float32 end_yaw     # 目标 yaw
```

#### 6.5.2 Yaw 插值策略

轨迹服务器根据 yaw 差异选择插值模式：

**模式 1：目标 yaw 插值**（`use_target_yaw_ = true`）

当 `|end_yaw - start_yaw| > 0.01` 时，使用 smoothstep 插值：

```cpp
double progress = std::min(1.0, t_cur / traj_duration_);
double smooth_progress = progress * progress * (3.0 - 2.0 * progress);

// 考虑角度环绕
double yaw_diff = angleDiff(end_yaw_, start_yaw_);
double yaw_target = normalizeAngle(start_yaw_ + smooth_progress * yaw_diff);
```

**模式 2：跟随速度方向**（`use_target_yaw_ = false`）

当起始和终点 yaw 相近时，跟随速度方向：

```cpp
Eigen::Vector3d dir = traj_->getPos(t_cur + time_forward_) - pos;
double yaw_target = atan2(dir(1), dir(0));
```

#### 6.5.3 Yaw 速度限制

施加 yaw 速度和加速度限制，避免剧烈旋转：

```cpp
constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;        // 360 deg/s
constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;    // 900 deg/s²
```

### 6.6 多路径点连续性

当有多个路径点时，保证速度和 yaw 的连续性：

```cpp
void planNextWaypoint(const Waypoint& next_wp, const Waypoint& previous_wp)
{
  // 使用前一个路径点的速度作为起始速度
  planner_manager_->planGlobalTrajWaypoints(
      previous_wp.pos, 
      previous_wp.vel,  // 前一个点的速度
      Eigen::Vector3d::Zero(),
      waypoints, 
      next_wp.vel,      // 下一个点的速度
      Eigen::Vector3d::Zero());
}
```


---

## 7. 参数配置

### 7.1 配置文件结构

**文件位置**：`config/ego_planner_config.yaml`

```yaml
#####################################################
# ROG-Map 配置
#####################################################
rog_map:
  resolution: 0.1            # 地图分辨率（米）
  inflate_distance: 0.3      # 障碍物膨胀距离（米）
  
  ros_callback:
    odom_topic: "/ground_truth/odom"
    cloud_topic: "/cloud_registered"

#####################################################
# FSM 状态机配置
#####################################################
fsm:
  flight_type: 2             # 1=手动(RViz goal), 2=预设路径点
  thresh_replan_time: 1.0    # 定期重规划时间间隔（秒）
  thresh_no_replan_meter: 1.0  # 接近目标时不重规划的距离（米）
  planning_horizon: 7.5      # 规划视野（米）
  emergency_time: 1.0        # 紧急停止检查时间（秒）
  fail_safe: true            # 是否启用故障保护
  
  # PX4 控制参数
  takeoff_height: 1.0          # 起飞高度（米）
  takeoff_accel_time: 2.0      # 起飞加速时间（秒）
  replan_duration: 0.05        # 重规划周期（秒）
  auto_start_preset: true      # 起飞后自动执行预设路径
  
  # 预设路径点（仅当 flight_type=2 时使用）
  waypoint_num: 4
  waypoint0_x: 5.0
  waypoint0_y: 0.0
  waypoint0_z: 1.0
  # ...

#####################################################
# 规划器参数
#####################################################
manager:
  max_vel: 1.5        # 最大速度（米/秒）
  max_acc: 3.0        # 最大加速度（米/秒²）
  planning_horizon: 7.5
  piece_length: 1.5   # 每段轨迹长度（米）

#####################################################
# MINCO 优化参数
#####################################################
optimization:
  cps_num_prePiece: 5         # 每段采样的控制点数
  
  # 权重
  weight_obstacle: 10000.0        # 硬障碍物避障权重
  weight_obstacle_soft: 5000.0    # 软障碍物避障权重
  weight_feasibility: 10000.0     # 动力学可行性权重
  weight_sqrvariance: 10000.0     # 轨迹平滑度权重
  weight_time: 10.0               # 时间最优权重
  
  # 安全距离
  obstacle_clearance: 0.3         # 硬约束安全距离（米）
  obstacle_clearance_soft: 0.5    # 软约束安全距离（米）
  
  max_vel: 1.5
  max_acc: 3.0

#####################################################
# 轨迹服务器参数
#####################################################
traj_server:
  time_forward: 1.0   # Yaw 前瞻时间（秒）
```

### 7.2 关键参数说明

#### 7.2.1 ROG-Map 参数

| 参数 | 说明 | 推荐值 | 影响 |
|------|------|--------|------|
| `resolution` | 地图分辨率 | 0.1m | 越小越精细，但内存消耗大 |
| `inflate_distance` | 膨胀距离 | 0.3m | 应大于等于 `obstacle_clearance` |

#### 7.2.2 FSM 参数

| 参数 | 说明 | 推荐值 | 影响 |
|------|------|--------|------|
| `thresh_replan_time` | 重规划周期 | 1.0s | 越小响应越快，但计算负担大 |
| `planning_horizon` | 规划视野 | 7.5m | 应大于 `max_vel * thresh_replan_time * 3` |
| `emergency_time` | 碰撞检查时间 | 1.0s | 前瞻时间，越大越安全但误报多 |
| `takeoff_height` | 起飞高度 | 1.0m | 根据场景调整 |

#### 7.2.3 优化参数

| 参数 | 说明 | 推荐值 | 影响 |
|------|------|--------|------|
| `weight_obstacle` | 硬避障权重 | 10000.0 | 越大越不会碰撞，但可能无解 |
| `weight_obstacle_soft` | 软避障权重 | 5000.0 | 控制轨迹与障碍物的距离 |
| `obstacle_clearance` | 硬约束距离 | 0.3m | 绝对不能小于此距离 |
| `obstacle_clearance_soft` | 软约束距离 | 0.5m | 尽量保持的距离 |
| `weight_time` | 时间权重 | 10.0 | 越大轨迹越短，但可能急转弯 |
| `cps_num_prePiece` | 控制点数 | 5 | 越多避障越精细，但优化慢 |

#### 7.2.4 动力学参数（1.99kg 无人机）

| 参数 | 说明 | 推荐值 | 调整建议 |
|------|------|--------|----------|
| `max_vel` | 最大速度 | 1.5 m/s | 根据实际测试调整，留 20% 余量 |
| `max_acc` | 最大加速度 | 3.0 m/s² | 推重比约 1.5 时可用 3.0-4.0 |

---

## 8. 使用方法

### 8.1 基本启动

```bash
# 1. 启动仿真环境
roslaunch ...

# 2. 启动 EGO-Planner
roslaunch rog_map ego_planner.launch

# 参数覆盖示例
roslaunch rog_map ego_planner.launch \
  flight_type:=1 \
  odom_topic:=/mavros/local_position/odom \
  cloud_topic:=/camera/depth/points
```

### 8.2 手动模式（flight_type=1）

通过 RViz 发送目标点：

1. 在 RViz 中点击 "2D Nav Goal"
2. 在地图上点击目标位置
3. 拖动箭头设置目标 yaw 方向
4. 无人机自动规划并执行

### 8.3 预设模式（flight_type=2）

在配置文件中预设路径点，起飞后自动执行：

```yaml
fsm:
  flight_type: 2
  auto_start_preset: true
  waypoint_num: 3
  waypoint0_x: 5.0
  waypoint0_y: 0.0
  waypoint0_z: 1.5
  # ...
```

### 8.4 探索模式（编程接口）

发布探索视点：

```python
import rospy
from quadrotor_msgs.msg import PositionCommand

pub = rospy.Publisher('/exploration/viewpoint', PositionCommand, queue_size=10)

viewpoint = PositionCommand()
viewpoint.position.x = 10.0
viewpoint.position.y = 5.0
viewpoint.position.z = 2.0
viewpoint.yaw = 1.57  # 90度
viewpoint.velocity.x = 0.5  # 期望以 0.5 m/s 到达
viewpoint.velocity.y = 0.0
viewpoint.velocity.z = 0.0

pub.publish(viewpoint)
```

### 8.5 紧急停止

```bash
rostopic pub /ego_planner_node/mandatory_stop std_msgs/Empty "{}"
```

---

## 9. 话题接口

### 9.1 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~odom` | `nav_msgs/Odometry` | 无人机里程计（位置、速度、姿态） |
| `/mavros/state` | `mavros_msgs/State` | PX4 状态（连接、模式、解锁） |
| `/goal` | `geometry_msgs/PoseStamped` | RViz 目标点（手动模式） |
| `/exploration/viewpoint` | `quadrotor_msgs/PositionCommand` | 探索视点（位置+yaw+速度） |
| `/traj_start_trigger` | `std_msgs/Empty` | 触发预设路径执行 |
| `~mandatory_stop` | `std_msgs/Empty` | 紧急停止信号 |

### 9.2 发布话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/planning/trajectory` | `rog_map/PolyTraj` | 事件触发 | MINCO 多项式轨迹 |
| `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 100Hz | 位置控制命令 |
| `/planning/replan` | `std_msgs/Int8` | 事件触发 | 重规划信号（1=新轨迹，2=停止） |
| `/ego_planner/trajectory_vis` | `visualization_msgs/Marker` | 2Hz | 轨迹可视化 |

### 9.3 服务调用

| 服务 | 类型 | 说明 |
|------|------|------|
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 解锁/上锁 |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 设置飞行模式 |


---

## 10. 性能分析

### 10.1 计算效率

| 模块 | 平均耗时 | 最大耗时 | 频率 |
|------|----------|----------|------|
| A* 搜索 | 5-20 ms | 50 ms | 事件触发 |
| MINCO 优化 | 10-30 ms | 100 ms | 事件触发 |
| 碰撞检查 | < 1 ms | 2 ms | 20 Hz |
| 轨迹执行 | < 0.5 ms | 1 ms | 100 Hz |

### 10.2 内存占用

- ROG-Map ESDF：约 50-200 MB（取决于地图大小和分辨率）
- A* 节点池：10-20 MB
- 轨迹数据：< 1 MB

### 10.3 优化建议

1. **降低 A* 分辨率**：将 `resolution` 从 0.1 增加到 0.2，可减少 50% 搜索时间
2. **减少控制点数**：将 `cps_num_prePiece` 从 5 降到 3，可加快优化但避障精度降低
3. **增大重规划周期**：将 `thresh_replan_time` 从 1.0 增加到 1.5，减少计算负担

---

## 11. 常见问题

### 11.1 规划失败

**现象**：`[EGOPlannerManager] A* search failed!`

**原因**：
- 目标点在障碍物内或太近
- 起点被障碍物包围
- A* 节点池耗尽

**解决**：
- 检查目标点是否可达
- 增大 `inflate_distance`
- 增加 A* 节点池大小（代码中修改 `allocate_num`）

### 11.2 轨迹不平滑

**现象**：无人机运动抖动

**原因**：
- `weight_sqrvariance` 太小
- `weight_obstacle` 太大导致轨迹扭曲

**解决**：
- 增大 `weight_sqrvariance` 到 20000
- 适当减小 `weight_obstacle` 到 5000

### 11.3 速度超限

**现象**：`Velocity exceeds limit`，然后时间重分配

**原因**：
- 路径点距离太近
- `max_vel` 设置过高

**解决**：
- 增大路径点间距
- 降低 `max_vel`
- 调整 `piece_length`

### 11.4 碰撞风险

**现象**：轨迹飞得太靠近障碍物

**原因**：
- `obstacle_clearance` 太小
- `weight_obstacle_soft` 太小

**解决**：
- 增大 `obstacle_clearance` 到 0.4-0.5
- 增大 `weight_obstacle_soft` 到 10000
- 增大地图 `inflate_distance`

### 11.5 无人机不起飞

**现象**：FSM 卡在 INIT 或 PRE_ARM 状态

**原因**：
- 没有收到 odometry 数据（话题不匹配）
- PX4 未连接或未响应模式切换

**解决**：
- 检查 `odom_topic` 是否正确
- 确保 MAVROS 正常运行
- 检查 PX4 连接状态

---

## 12. 扩展与开发

### 12.1 添加新的约束

在 `poly_traj_optimizer.cpp` 的 `addPVAGradCost2CT` 函数中添加：

```cpp
// 示例：添加高度约束
if (pos(2) < min_height)
{
  double cost = weight_height * (min_height - pos(2)) * (min_height - pos(2));
  Eigen::Vector3d grad(0, 0, -2 * weight_height * (min_height - pos(2)));
  // 添加到总代价和梯度...
}
```

### 12.2 自定义 FSM 状态

在 `ego_replan_fsm.h` 中添加新状态：

```cpp
enum FSM_EXEC_STATE
{
  // ... existing states
  CUSTOM_STATE  // 新状态
};
```

在 `execFSMCallback` 中添加处理逻辑。

### 12.3 多机协同（未来工作）

当前版本已移除多机代码以简化单机应用。如需多机协同，可参考 `formation_ws` 原始实现，添加：
- 机间轨迹广播
- 机间碰撞避免约束
- 编队保持

### 12.4 添加新的轨迹可视化

```cpp
// 在 ego_replan_fsm.cpp 中添加
ros::Publisher custom_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
    "/planning/custom_vis", 10);

// 在合适的地方调用
void publishCustomVisualization()
{
  visualization_msgs::MarkerArray markers;
  // 添加自定义可视化标记
  custom_vis_pub_.publish(markers);
}
```

---

## 13. 与 Fast-Planner 的对比

| 特性 | EGO-Planner (MINCO) | Fast-Planner (B-spline) |
|------|---------------------|------------------------|
| 轨迹表示 | 5阶多项式（按段） | 均匀B样条 |
| 优化目标 | 最小化 jerk | 最小化 jerk |
| 优化方法 | L-BFGS | NLopt |
| 端点速度约束 | ✅ 精确约束 | ⚠️ 软约束 |
| 连续性 | C² 连续（加速度） | C² 连续 |
| 优化速度 | 快（10-30ms） | 中等（20-50ms） |
| 参数数量 | 较少（6系数/段） | 较多（控制点） |
| 适用场景 | 探索、连续运动 | 通用导航 |

**选择建议**：
- 探索任务、需要指定终点速度 → EGO-Planner
- 通用导航、简单路径跟踪 → Fast-Planner

---

## 14. 参考文献

1. **MINCO 原理**：Z. Wang et al., "Geometrically Constrained Trajectory Optimization for Multicopters," IEEE TRO, 2021.
2. **EGO-Planner 原版**：X. Zhou et al., "EGO-Planner: An ESDF-free Gradient-based Local Planner," IEEE RA-L, 2020.
3. **Fast-Planner**：B. Zhou et al., "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight," IEEE RA-L, 2019.
4. **A* 算法**：P. Hart et al., "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," IEEE Transactions on SSC, 1968.
5. **L-BFGS 优化**：D. Liu and J. Nocedal, "On the Limited Memory BFGS Method for Large Scale Optimization," Mathematical Programming, 1989.

---

## 15. 附录

### 15.1 代码结构

```
rog_map/
├── include/ego_planner/
│   ├── ego_replan_fsm.h           # FSM 状态机头文件
│   ├── ego_planner_manager.h      # 规划管理器头文件
│   ├── poly_traj_optimizer.h      # MINCO 优化器头文件
│   ├── rog_astar.h                # A* 搜索头文件
│   ├── plan_container.hpp         # 轨迹数据容器
│   └── optimizer/
│       └── poly_traj_utils.hpp    # MINCO 数学工具
├── src/ego_planner/
│   ├── ego_replan_fsm.cpp         # FSM 实现
│   ├── ego_planner_manager.cpp    # 规划管理器实现
│   ├── poly_traj_optimizer.cpp    # MINCO 优化器实现
│   ├── rog_astar.cpp              # A* 搜索实现
│   ├── ego_planner_node.cpp       # 主节点
│   └── ego_traj_server.cpp        # 轨迹服务器
├── msg/
│   └── PolyTraj.msg               # 轨迹消息定义
├── config/
│   ├── ego_planner_config.yaml    # 配置文件
│   └── ego_planner.rviz           # RViz 配置
├── launch/
│   └── ego_planner.launch         # 启动文件
└── doc/
    └── EGO_Planner_Technical_Documentation.md  # 本文档
```

### 15.2 关键类说明

**EGOReplanFSM**
- 功能：状态机管理
- 主要方法：`execFSMCallback()`, `planFromGlobalTraj()`, `planFromLocalTraj()`

**EGOPlannerManager**
- 功能：规划流程管理
- 主要方法：`planGlobalTrajWaypoints()`, `reboundReplan()`, `getLocalTarget()`

**PolyTrajOptimizer**
- 功能：MINCO 轨迹优化
- 主要方法：`OptimizeTrajectory()`, `addPVAGradCost2CT()`

**ROGAstar**
- 功能：A* 全局路径搜索
- 主要方法：`search()`, `getPath()`

### 15.3 数学符号表

| 符号 | 说明 |
|------|------|
| p(t) | 位置轨迹 |
| v(t) = dp/dt | 速度（位置的一阶导数） |
| a(t) = d²p/dt² | 加速度（位置的二阶导数） |
| j(t) = d³p/dt³ | Jerk（加速度的一阶导数） |
| c_i,k | 第 i 段第 k 阶多项式系数 |
| T_i | 第 i 段轨迹的时间长度 |
| M | 轨迹段数 |
| d(p) | 点 p 到最近障碍物的距离 |
| d_safe | 硬约束安全距离 |
| d_soft | 软约束安全距离 |
| v_max | 最大速度 |
| a_max | 最大加速度 |
| ω | 权重参数 |

---

**文档版本**：v1.0  
**最后更新**：2025-12-26  
**作者**：EGO-Planner Integration Team  
**项目**：STAR-Searcher / ROG-Map

**致谢**：
- 本规划器基于 Fast-Lab 的 EGO-Planner 开源项目
- MINCO 轨迹优化库来自 ZJU-FAST-Lab
- 感谢 formation_ws 项目提供的参考实现
