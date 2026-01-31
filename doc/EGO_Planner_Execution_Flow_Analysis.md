# EGO-Planner 完整运行流程详解

> **重要修订说明（v1.2）**：
> 本版本对"地图查询时机"章节（第 5.4 节）进行了重大修正。经过对源码的深入分析，发现：
> - ❌ **之前误解**：MINCO 优化阶段使用 `evaluateEDTWithGrad` 查询地图约 1500 次
> - ✅ **实际情况**：MINCO 优化阶段**不查询地图**，而是使用预先计算的 `base_point` 和 `direction` 进行点积运算
> - ✅ **关键发现**：真正的地图查询发生在 `finelyCheckAndSetConstraintPoints` 阶段（~1250 次），包括碰撞检测、A* 搜索和射线投射
> - 🎯 **核心机制**：EGO-Planner 采用**"预计算+缓存"**策略，在碰撞检查阶段一次性计算障碍物方向，在优化阶段复用这些信息，大幅提升性能
>
> 感谢用户指出 `evaluateEDTWithGrad` 函数不存在的问题，本次修订基于实际源码进行了完全重写。

## 目录
1. [文档说明与源码勘误](#1-文档说明与源码勘误)
2. [核心概念](#2-核心概念)
3. [接收目标点后的完整流程](#3-接收目标点后的完整流程)
4. [全局轨迹规划详解](#4-全局轨迹规划详解)
5. [局部轨迹优化详解](#5-局部轨迹优化详解)
6. [实时重规划机制](#6-实时重规划机制)
7. [抵达目标点处理](#7-抵达目标点处理)
8. [源码执行流程图](#8-源码执行流程图)

---

## 1. 文档说明与源码勘误

### 1.1 关于 `planGlobalTrajWaypoints` 的澄清

**您的疑问**：
> 文档中描述的 `planGlobalTrajWaypoints` 和实际源码不一致，而且 waypoints.size() == 1 时为什么不打印 ROS_ERROR？

**真相**：

#### 实际源码（ego_planner_manager.cpp:319-381）

```cpp
bool EGOPlannerManager::planGlobalTrajWaypoints(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
    const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
  poly_traj::MinJerkOpt globalMJO;
  Eigen::Matrix<double, 3, 3> headState, tailState;
  headState << start_pos, start_vel, start_acc;
  tailState << waypoints.back(), end_vel, end_acc;  // 最后一个点作为终点
  Eigen::MatrixXd innerPts;

  if (waypoints.size() > 1)  // 多个路径点
  {
    innerPts.resize(3, waypoints.size() - 1);
    for (int i = 0; i < (int)waypoints.size() - 1; ++i)
    {
      innerPts.col(i) = waypoints[i];  // 前 N-1 个点作为中间点
    }
  }
  else  // 单个路径点：waypoints.size() == 1
  {
    // innerPts 保持为空（Eigen::MatrixXd 默认构造就是空的，size() == 0）
    if (innerPts.size() != 0)
    {
      ROS_ERROR("innerPts.size() != 0");  // 防御性检查，正常不会触发
    }
  }

  globalMJO.reset(headState, tailState, waypoints.size());

  // 时间分配
  double des_vel = pp_.max_vel_ / 1.5;
  Eigen::VectorXd time_vec(waypoints.size());

  for (int j = 0; j < 2; ++j)
  {
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                             : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
    }

    globalMJO.generate(innerPts, time_vec);

    if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
        start_vel.norm() > pp_.max_vel_ ||
        end_vel.norm() > pp_.max_vel_)
    {
      break;
    }

    if (j == 1)
    {
      ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
    }

    des_vel /= 1.5;
  }

  auto time_now = ros::Time::now();
  traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

  return true;
}
```

#### 为什么不打印 ROS_ERROR？

**答案**：当 `waypoints.size() == 1` 时：

1. **不进入** `if (waypoints.size() > 1)` 分支
2. **进入** `else` 分支
3. `innerPts` 是新创建的 `Eigen::MatrixXd`，**默认构造就是空的**（`size() == 0`）
4. 检查 `if (innerPts.size() != 0)` **不满足**，所以不会打印错误

这个 `if` 检查是**防御性编程**，用于检测变量是否被意外修改。在正常情况下，新创建的 `Eigen::MatrixXd` 的 `size()` 就是 0，所以不会触发错误。

#### 单路径点的实际处理

当 `waypoints.size() == 1` 时：

```
起点：start_pos (headState)
  │
  │ 直接生成多项式轨迹（没有中间点）
  │
终点：waypoints[0] (tailState)
```

**MINCO 轨迹生成**：
- `globalMJO.reset(headState, tailState, 1)`：创建 **1 段轨迹**
- `globalMJO.generate(innerPts, time_vec)`：`innerPts` 为空，直接从起点到终点生成 5 阶多项式

**时间分配**：
```cpp
time_vec(0) = (waypoints[0] - start_pos).norm() / des_vel;
```
根据起点到终点的距离和期望速度计算飞行时间。

---

## 2. 核心概念

### 2.1 两级轨迹系统

EGO-Planner 使用**两级轨迹系统**：

```
┌─────────────────────────────────────────────────┐
│           全局轨迹 (Global Trajectory)           │
│  ┌────────────────────────────────────────┐    │
│  │ 从当前位置到目标点的完整路径            │    │
│  │ - 生成时机：收到新目标点时              │    │
│  │ - 特点：粗略、不避障、仅用于导航        │    │
│  │ - 作用：提供全局路径方向                │    │
│  └────────────────────────────────────────┘    │
└─────────────────────────────────────────────────┘
                      │
                      │ 提取局部目标
                      ▼
┌─────────────────────────────────────────────────┐
│           局部轨迹 (Local Trajectory)            │
│  ┌────────────────────────────────────────┐    │
│  │ 当前位置到局部目标点的优化轨迹          │    │
│  │ - 生成时机：初次规划 + 定期重规划       │    │
│  │ - 特点：精细、避障、动力学可行          │    │
│  │ - 作用：实际执行的轨迹                  │    │
│  └────────────────────────────────────────┘    │
└─────────────────────────────────────────────────┘
```

### 2.2 规划视野（Planning Horizon）

**概念**：局部轨迹的规划范围，默认 7.5 米。

**原理**：
- 不是一次性规划到目标点，而是规划到 `planning_horizon` 范围内的中间点
- 然后在执行过程中不断重规划，逐步接近目标

**为什么？**
1. **降低计算复杂度**：短轨迹优化更快
2. **适应环境变化**：及时响应新的障碍物信息
3. **提高鲁棒性**：即使远处的路径有问题，也不会影响当前执行

```
当前位置            局部目标           全局目标
    ●─────────────▶○                    ◉
    └─ 7.5m ─────┘
    (规划这一段)
```

### 2.3 关键标志位

| 标志位 | 类型 | 说明 |
|--------|------|------|
| `have_target_` | `bool` | 是否有目标点 |
| `have_trigger_` | `bool` | 是否触发规划 |
| `touch_goal_` | `bool` | 局部目标是否到达全局目标 |
| `flag_use_poly_init` | `bool` | 是否使用多项式初始化（true=从全局轨迹，false=从前一次局部轨迹） |

---

## 3. 接收目标点后的完整流程

### 3.1 流程总览

```
┌─────────────────┐
│ 1. 接收目标点   │ (waypointCallback / viewpointCallback)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 2. 生成全局轨迹 │ (planGlobalTrajWaypoints)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 3. 首次局部规划 │ (GEN_NEW_TRAJ → planFromGlobalTraj)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 4. 执行轨迹     │ (EXEC_TRAJ)
└────────┬────────┘
         │
         ├─────────────────┐
         ▼                 ▼
┌─────────────────┐  ┌─────────────────┐
│ 5a. 定期重规划  │  │ 5b. 到达目标    │
│ (REPLAN_TRAJ)   │  │ (WAIT_TARGET)   │
└─────────────────┘  └─────────────────┘
```

### 3.2 步骤 1：接收目标点

#### 源码：waypointCallback（ego_replan_fsm.cpp:429-463）

```cpp
void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
{
  // 1. 提取目标位置
  end_pt_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  
  // 2. 从四元数提取 yaw
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                       msg->pose.orientation.y, msg->pose.orientation.z);
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  end_yaw_ = euler(2);
  
  // 3. 默认终点速度为零（停止）
  end_vel_ = Eigen::Vector3d::Zero();
  
  // 4. 清空并设置路径点列表
  wps_.clear();
  wps_.push_back(Waypoint(end_pt_, end_yaw_, end_vel_));
  wpt_id_ = 0;

  // 5. 调用全局规划
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.push_back(end_pt_);  // 单个目标点
  planner_manager_->planGlobalTrajWaypoints(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
                                             waypoints, end_vel_, Eigen::Vector3d::Zero());
  // 注意：waypoints.size() == 1

  // 6. 设置标志位
  have_target_ = true;
  have_trigger_ = true;
  have_new_target_ = true;

  ROS_INFO("[EGOReplanFSM] Received waypoint: pos=(%.2f, %.2f, %.2f), yaw=%.2f", 
           end_pt_.x(), end_pt_.y(), end_pt_.z(), end_yaw_);

  // 7. 如果正在执行轨迹，立即切换到生成新轨迹状态
  if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ)
  {
    changeFSMExecState(GEN_NEW_TRAJ, "WAYPOINT");
  }
}
```

**关键点**：
- **单个目标点**：`waypoints` 只包含一个点（`end_pt_`）
- **全局规划立即执行**：在回调中就调用 `planGlobalTrajWaypoints`
- **状态切换**：如果正在执行中，立即切换到 `GEN_NEW_TRAJ` 重新规划

### 3.3 步骤 2：生成全局轨迹

#### 源码：planGlobalTrajWaypoints（ego_planner_manager.cpp:319-381）

**已在第 1 节详细分析，这里总结关键步骤**：

```cpp
// 1. 设置起点和终点状态
headState << start_pos, start_vel, start_acc;
tailState << waypoints.back(), end_vel, end_acc;  // waypoints[0]

// 2. 单路径点：innerPts 为空
Eigen::MatrixXd innerPts;  // 空矩阵

// 3. 创建 MINCO 优化器（1 段轨迹）
globalMJO.reset(headState, tailState, 1);

// 4. 时间分配
double des_vel = pp_.max_vel_ / 1.5;  // 期望速度 = 1.5 / 1.5 = 1.0 m/s
time_vec(0) = (waypoints[0] - start_pos).norm() / des_vel;

// 5. 生成轨迹（从起点直接到终点，没有中间点）
globalMJO.generate(innerPts, time_vec);

// 6. 检查速度是否超限，如果超限则降低期望速度重新生成
if (globalMJO.getTraj().getMaxVelRate() > pp_.max_vel_)
{
  des_vel /= 1.5;  // 降低到 0.67 m/s
  // 重新计算时间并生成...
}

// 7. 保存全局轨迹
traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());
```

**全局轨迹特点**：
- ✅ **简单直接**：从起点直接到终点，不考虑障碍物
- ✅ **快速生成**：无优化过程，仅保证端点约束和动力学可行
- ❌ **不避障**：仅用于导航方向，不用于实际执行

### 3.4 步骤 3：首次局部规划（GEN_NEW_TRAJ）

#### FSM 状态：GEN_NEW_TRAJ（ego_replan_fsm.cpp:209-222）

```cpp
case GEN_NEW_TRAJ:
{
  bool success = planFromGlobalTraj(10);  // 尝试 10 次
  if (success)
  {
    changeFSMExecState(EXEC_TRAJ, "FSM");
  }
  else
  {
    ROS_WARN("[EGOReplanFSM] Failed to generate trajectory, retrying...");
    changeFSMExecState(GEN_NEW_TRAJ, "FSM");  // 失败则重试
  }
  break;
}
```

#### 函数：planFromGlobalTraj（ego_replan_fsm.cpp:398-410）

```cpp
bool EGOReplanFSM::planFromGlobalTraj(const int trial_times)
{
  // 1. 设置起始状态为当前位置
  start_pt_ = odom_pos_;
  start_vel_ = odom_vel_;
  start_acc_ = Eigen::Vector3d::Zero();

  // 2. 尝试多次规划
  for (int i = 0; i < trial_times; i++)
  {
    if (callReboundReplan(true, false))  // flag_use_poly_init=true
      return true;
  }
  return false;
}
```

**关键参数**：
- `flag_use_poly_init = true`：表示从全局轨迹初始化
- `flag_randomPolyTraj = false`：不使用随机扰动

---

## 4. 全局轨迹规划详解

### 4.1 单路径点 vs 多路径点

#### 情况 1：单路径点（waypoints.size() == 1）

```
起点 ────────────▶ 终点
     (1 段轨迹)
```

**MINCO 参数**：
- `piece_num = 1`
- `innerPts` 为空
- 轨迹：5 阶多项式，直接从起点到终点

**时间分配**：
```cpp
time_vec(0) = ||end_pt - start_pos|| / des_vel
```

#### 情况 2：多路径点（waypoints.size() > 1）

```
起点 ───▶ WP1 ───▶ WP2 ───▶ 终点
   段1       段2       段3
```

**MINCO 参数**：
- `piece_num = 3`
- `innerPts = [WP1, WP2]`（前 N-1 个点）
- `tailState = 终点`（最后一个点）

**时间分配**：
```cpp
time_vec(0) = ||WP1 - start_pos|| / des_vel
time_vec(1) = ||WP2 - WP1|| / des_vel
time_vec(2) = ||end_pt - WP2|| / des_vel
```

### 4.2 速度检查与调整

生成轨迹后检查速度是否超限：

```cpp
for (int j = 0; j < 2; ++j)  // 最多尝试 2 次
{
  // 生成轨迹
  globalMJO.generate(innerPts, time_vec);
  
  // 检查速度
  if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_)
  {
    break;  // 速度可行，退出循环
  }
  
  // 第一次尝试失败
  if (j == 1)
  {
    ROS_WARN("Global traj MaxVel = %f > set_max_vel", 
             globalMJO.getTraj().getMaxVelRate());
  }
  
  // 降低期望速度，增加飞行时间
  des_vel /= 1.5;
  // 重新计算 time_vec...
}
```

**调整策略**：
- 第 1 次：`des_vel = max_vel / 1.5 = 1.0 m/s`（假设 max_vel=1.5）
- 第 2 次：`des_vel = 1.0 / 1.5 = 0.67 m/s`

**原理**：减小期望速度 → 增加飞行时间 → 降低轨迹最大速度

**重要澄清**：
- ❌ 全局轨迹生成**不使用 A* 搜索**
- ❌ 全局轨迹生成**不查询障碍物地图**
- ✅ 仅通过 MINCO 直接生成，只保证动力学可行性
- ✅ 避障工作在后续的**局部轨迹优化阶段**完成

### 4.3 全局轨迹的存储

```cpp
struct GlobalTrajData {
  poly_traj::Trajectory traj;              // MINCO 轨迹
  double global_start_time;                // 全局轨迹的起始时间
  double duration;                         // 总持续时间
  double glb_t_of_lc_tgt;                  // 当前局部目标在全局轨迹上的时间
  double last_glb_t_of_lc_tgt;            // 上一次局部目标的时间
};
```

**作用**：
- 后续重规划时从全局轨迹中提取局部目标点
- 跟踪规划进度

---

## 5. 局部轨迹优化详解

### 5.1 callReboundReplan 流程

#### 源码：callReboundReplan（ego_replan_fsm.cpp:364-396）

```cpp
bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
{
  // 1. 从全局轨迹获取局部目标点
  planner_manager_->getLocalTarget(planning_horizen_, start_pt_, end_pt_,
                                   local_target_pt_, local_target_vel_, touch_goal_);

  // 2. 如果到达目标且用户指定了非零终点速度，使用用户指定的速度
  if (touch_goal_ && end_vel_.norm() > 1e-3)
  {
    local_target_vel_ = end_vel_;
    ROS_DEBUG("[EGOReplanFSM] Using specified end velocity: (%.2f, %.2f, %.2f)",
              end_vel_.x(), end_vel_.y(), end_vel_.z());
  }

  // 3. 调用 MINCO 优化
  bool success = planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_,
                                                 local_target_pt_, local_target_vel_,
                                                 flag_use_poly_init, 
                                                 flag_randomPolyTraj, touch_goal_);
  
  // 4. 如果成功，发布轨迹
  if (success)
  {
    // 发布 PolyTraj 消息
    rog_map::PolyTraj poly_msg;
    polyTraj2ROSMsg(poly_msg);
    poly_traj_pub_.publish(poly_msg);
    
    // 通知轨迹服务器
    std_msgs::Int8 replan_msg;
    replan_msg.data = 1;  // 1 = 新轨迹
    replan_pub_.publish(replan_msg);
  }
  
  return success;
}
```

### 5.2 获取局部目标点（getLocalTarget）

#### 源码：getLocalTarget（ego_planner_manager.cpp:166-207）

```cpp
void EGOPlannerManager::getLocalTarget(
    const double planning_horizen, const Eigen::Vector3d &start_pt,
    const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
    Eigen::Vector3d &local_target_vel, bool &touch_goal)
{
  double t;
  touch_goal = false;

  traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

  // 步长：planning_horizen / 20 / max_vel
  double t_step = planning_horizen / 20 / pp_.max_vel_;
  
  // 从当前局部目标时间开始，沿全局轨迹向前搜索
  for (t = traj_.global_traj.glb_t_of_lc_tgt;
       t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
       t += t_step)
  {
    Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
    double dist = (pos_t - start_pt).norm();

    // 找到距离 >= planning_horizen 的点
    if (dist >= planning_horizen)
    {
      local_target_pos = pos_t;
      traj_.global_traj.glb_t_of_lc_tgt = t;
      break;
    }
  }

  // 如果到达全局轨迹末端
  if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5)
  {
    local_target_pos = global_end_pt;
    traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
    touch_goal = true;  // 标记为到达目标
  }

  // 设置局部目标速度
  if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
  {
    // 接近目标，速度设为零（减速停止）
    local_target_vel = Eigen::Vector3d::Zero();
  }
  else
  {
    // 使用全局轨迹上的速度
    local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
  }
}
```

**工作原理**：

```
全局轨迹：
    当前位置                 局部目标               全局目标
        ●───────7.5m──────▶○                        ◉
        │                  │                        │
     start_pt        local_target_pt            end_pt
     
     touch_goal = false      如果 local_target_pt == end_pt
                             则 touch_goal = true
```

**关键逻辑**：
1. 从上次的局部目标时间开始，沿全局轨迹向前搜索
2. 找到距离当前位置 ≥ `planning_horizen` 的点作为新的局部目标
3. 如果全局轨迹剩余距离 < `planning_horizen`，则局部目标 = 全局目标

### 5.3 MINCO 优化（reboundReplan）

#### 源码：reboundReplan（ego_planner_manager.cpp:221-302）

```cpp
bool EGOPlannerManager::reboundReplan(
    const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
    const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
    const bool flag_randomPolyTraj, const bool touch_goal)
{
  printf("\033[47;30m\n[replan %d]==============================================\033[0m\n", count++);

  ploy_traj_opt_->setIfTouchGoal(touch_goal);

  if ((start_pt - local_target_pt).norm() < 0.2)
  {
    ROS_INFO("Close to goal");
  }

  ros::Time t_start = ros::Time::now();
  ros::Duration t_init, t_opt;

  /*** STEP 1: 初始化轨迹 ***/
  double ts = pp_.polyTraj_piece_length / pp_.max_vel_;  // 每段时间

  poly_traj::MinJerkOpt initMJO;
  if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                        flag_polyInit, flag_randomPolyTraj, ts, initMJO))
  {
    return false;
  }

  // 获取初始轨迹的控制点
  Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
  
  // 精细碰撞检查并调整控制点
  std::vector<std::pair<int, int>> segments;
  if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) 
      == PolyTrajOptimizer::CHK_RET::ERR)
  {
    return false;
  }

  t_init = ros::Time::now() - t_start;
  t_start = ros::Time::now();

  /*** STEP 2: 优化轨迹 ***/
  bool flag_success = false;
  poly_traj::MinJerkOpt best_MJO;

  poly_traj::Trajectory initTraj = initMJO.getTraj();
  int PN = initTraj.getPieceNum();
  Eigen::MatrixXd all_pos = initTraj.getPositions();
  Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
  Eigen::Matrix<double, 3, 3> headState, tailState;
  headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
  tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
  
  double final_cost;
  flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                    innerPts, initTraj.getDurations(),
                                                    cstr_pts, final_cost);
  best_MJO = ploy_traj_opt_->getMinJerkOpt();

  t_opt = ros::Time::now() - t_start;

  // 保存并显示结果
  ROS_INFO("plan_success=%d", flag_success);
  if (!flag_success)
  {
    continous_failures_count_++;
    return false;
  }

  ROS_INFO("total time: %.3f, init: %.3f, optimize: %.3f",
           (t_init + t_opt).toSec(), t_init.toSec(), t_opt.toSec());

  setLocalTrajFromOpt(best_MJO, touch_goal);

  continous_failures_count_ = 0;
  return true;
}
```

**两个步骤**：

#### STEP 1：初始化轨迹（computeInitState）

这是**从全局/局部轨迹中提取初始轨迹**的核心逻辑，有两种初始化方式：

---

##### 方式 1：从多项式初始化（`flag_polyInit = true`）

**使用场景**：首次规划（GEN_NEW_TRAJ）

**完整源码**（ego_planner_manager.cpp:41-110）：

```cpp
if (flag_first_call || flag_polyInit)
{
  // 基本参数
  Eigen::Matrix3d headState, tailState;
  Eigen::MatrixXd innerPs;
  Eigen::VectorXd piece_dur_vec;
  int piece_nums;
  constexpr double init_of_init_totaldur = 2.0;  // "预初始化"轨迹的时间
  
  headState << start_pt, start_vel, start_acc;
  tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

  /*** 步骤 1.1：生成 "init of init" 轨迹（预初始化轨迹） ***/
  if (!flag_randomPolyTraj)  // 正常情况
  {
    // innerPs 为空，直接从起点到终点
    piece_nums = 1;
    piece_dur_vec.resize(1);
    piece_dur_vec(0) = init_of_init_totaldur;  // 2.0 秒
  }
  else  // 随机扰动（用于规划失败后重试）
  {
    // 在起点和终点中间生成一个随机点
    Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
    Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
    innerPs.resize(3, 1);
    innerPs = (start_pt + local_target_pt) / 2 + 
              随机偏移（水平和垂直方向）;
    
    piece_nums = 2;
    piece_dur_vec = Eigen::Vector2d(1.0, 1.0);
  }
  
  // 生成 "init of init" 轨迹
  initMJO.reset(headState, tailState, piece_nums);
  initMJO.generate(innerPs, piece_dur_vec);
  poly_traj::Trajectory initTraj = initMJO.getTraj();  // 这是预初始化轨迹

  /*** 步骤 1.2：从 "init of init" 轨迹采样，生成真正的初始轨迹 ***/
  
  // 计算需要的段数
  piece_nums = round((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
  // polyTraj_piece_length = 1.5m，假设距离 7.5m，则 piece_nums = 5
  if (piece_nums < 2)
    piece_nums = 2;
  
  // 时间参数
  double piece_dur = init_of_init_totaldur / (double)piece_nums;  // 每段时间
  piece_dur_vec.resize(piece_nums);
  piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);  // ts = piece_length / max_vel
  
  // 从 initTraj 上采样 piece_nums - 1 个中间点
  innerPs.resize(3, piece_nums - 1);
  int id = 0;
  double t_s = piece_dur;  // 起始采样时间
  double t_e = init_of_init_totaldur - piece_dur / 2;  // 结束采样时间
  
  for (double t = t_s; t < t_e; t += piece_dur)
  {
    innerPs.col(id++) = initTraj.getPos(t);  // 从预初始化轨迹采样
  }
  
  // 用采样的控制点生成真正的初始轨迹
  initMJO.reset(headState, tailState, piece_nums);
  initMJO.generate(innerPs, piece_dur_vec);
}
```

**关键步骤图解**：

```
步骤 1.1：生成 "init of init" 轨迹（2秒，1段或2段）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
起点 ─────────────────────▶ 局部目标
     (2.0秒，简单轨迹)

步骤 1.2：从 "init of init" 轨迹采样中间点
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
假设距离 7.5m，piece_length = 1.5m
→ piece_nums = 5 段
→ 需要采样 4 个中间点

采样时间：
  t = 0.4s, 0.8s, 1.2s, 1.6s
     │      │      │      │
     ▼      ▼      ▼      ▼
起点 ●─────●─────●─────●─────● 局部目标
   段1   段2   段3   段4   段5

步骤 1.3：用采样的 4 个点生成 5 段轨迹
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
起点 ═══════════════════════════▶ 局部目标
     (5段，更精细的轨迹)
```

**为什么要两步？**
1. **第一步**：快速生成粗略轨迹（2秒，简单）
2. **第二步**：根据实际距离调整段数，采样中间点生成精细轨迹
3. **好处**：保证轨迹段数和距离成正比，避免过长或过短

---

##### 方式 2：从前一次轨迹初始化（`flag_polyInit = false`）

**使用场景**：重规划（REPLAN_TRAJ）

**完整源码**（ego_planner_manager.cpp:111-161）：

```cpp
else  // flag_polyInit = false
{
  if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
  {
    ROS_ERROR("No previous trajectories!");
    return false;
  }

  /*** 步骤 2.1：计算时间系统 ***/
  
  // 当前局部轨迹已执行的时间
  double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
  
  // 局部轨迹剩余时间
  double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
  if (t_to_lc_end < 0)
  {
    ROS_INFO("Local traj already ended");
    return false;
  }
  
  // 从当前位置到新的局部目标的时间
  // = 局部轨迹剩余 + 全局轨迹上从旧目标到新目标的时间
  double t_to_lc_tgt = t_to_lc_end + 
                       (traj_.global_traj.glb_t_of_lc_tgt - 
                        traj_.global_traj.last_glb_t_of_lc_tgt);
  
  // 计算段数
  int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
  if (piece_nums < 2)
    piece_nums = 2;
  
  /*** 步骤 2.2：混合采样（局部轨迹 + 全局轨迹） ***/
  
  Eigen::Matrix3d headState, tailState;
  Eigen::MatrixXd innerPs(3, piece_nums - 1);
  Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
  
  headState << start_pt, start_vel, start_acc;
  tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();
  
  double t = piece_dur_vec(0);
  for (int i = 0; i < piece_nums - 1; ++i)
  {
    if (t < t_to_lc_end)
    {
      // 情况 A：从局部轨迹的剩余部分采样
      innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
    }
    else if (t <= t_to_lc_tgt)
    {
      // 情况 B：从全局轨迹采样
      double glb_t = t - t_to_lc_end + 
                     traj_.global_traj.last_glb_t_of_lc_tgt - 
                     traj_.global_traj.global_start_time;
      innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
    }
    else
    {
      ROS_ERROR("Time calculation error!");
    }
    
    t += piece_dur_vec(i + 1);
  }
  
  // 生成初始轨迹
  initMJO.reset(headState, tailState, piece_nums);
  initMJO.generate(innerPs, piece_dur_vec);
}
```

**关键步骤图解**：

```
时间系统计算
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                  当前时刻
                     │
局部轨迹：  ─────────┼────────▶
            已执行    │ 剩余
                     │ (t_to_lc_end)
                     
全局轨迹：  ──────────●════════●════════◉
            已执行    旧目标   新目标   终点
                      │        │
                      └────────┘
                      这段时间需要加到采样时间中

混合采样策略
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
假设：
- 局部轨迹剩余 1.5 秒
- 需要采样 4 个点
- 采样时间：t = 0.5s, 1.0s, 1.5s, 2.0s

       当前位置
          ●
          │ t=0.5s (< t_to_lc_end)
          ●────▶ 从局部轨迹采样
          │ t=1.0s (< t_to_lc_end)
          ●────▶ 从局部轨迹采样
          │ t=1.5s (= t_to_lc_end)
          ●────▶ 从全局轨迹采样
          │ t=2.0s (> t_to_lc_end)
          ●────▶ 从全局轨迹采样
          │
       新局部目标

结果：前 2 个点从局部轨迹，后 2 个点从全局轨迹
```

**为什么混合采样？**
1. **保证连续性**：局部轨迹的剩余部分已经很平滑
2. **利用全局信息**：全局轨迹提供长期导航方向
3. **平滑过渡**：混合采样避免新旧轨迹的突变

---

##### 两种初始化方式对比

| 特性 | 方式 1（多项式初始化） | 方式 2（轨迹初始化） |
|------|----------------------|---------------------|
| **触发场景** | 首次规划（GEN_NEW_TRAJ） | 重规划（REPLAN_TRAJ） |
| **flag_use_poly_init** | `true` | `false` |
| **起始状态来源** | 当前里程计 | 当前轨迹上的点 |
| **中间点来源** | "init of init" 轨迹采样 | 局部轨迹 + 全局轨迹混合采样 |
| **是否需要前一次轨迹** | ❌ 不需要 | ✅ 需要 |
| **连续性** | 仅保证端点连续 | 完全连续（速度、加速度） |
| **计算复杂度** | 较低（两步生成） | 较低（直接采样） |
| **适用距离** | 任意距离 | 通常 < planning_horizon |

**选择逻辑**：

```cpp
// 在 planFromGlobalTraj 中
bool success = callReboundReplan(true, false);   // flag_use_poly_init = true
                                                 // 从多项式初始化

// 在 planFromLocalTraj 中
bool success = callReboundReplan(false, false);  // flag_use_poly_init = false
                                                 // 从轨迹初始化
```

**首次规划 vs 重规划的区别**：

```
首次规划（GEN_NEW_TRAJ）
━━━━━━━━━━━━━━━━━━━━━━━━━━━
起始状态：里程计（可能静止）
   ●  vel = (0, 0, 0)
   │  acc = (0, 0, 0)
   │
   └─▶ 需要从零开始规划
   └─▶ 使用多项式初始化

重规划（REPLAN_TRAJ）
━━━━━━━━━━━━━━━━━━━━━━━━━━━
起始状态：轨迹上的点（运动中）
旧轨迹： ●═══════▶
         │  vel = (1.2, 0.6, 0.2)  从轨迹上获取
         │  acc = (0.3, 0.1, 0.0)
         │
         └─▶ 已经在运动中
         └─▶ 使用轨迹初始化，保证平滑
```

**实际应用示例**：

```
场景：从 (0, 0, 1) 飞到 (10, 5, 2)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

T = 0s：首次规划（方式 1）
┌─────────────────────────────────┐
│ 起点：(0, 0, 1), vel=0          │
│ 局部目标：(7.2, 3.6, 1.8)       │
│                                 │
│ Step 1: 生成 "init of init"    │
│   起点 ────────▶ 局部目标       │
│   (2秒，1段简单轨迹)            │
│                                 │
│ Step 2: 采样 4 个中间点         │
│   t=0.4s, 0.8s, 1.2s, 1.6s     │
│   ●───●───●───●───●             │
│                                 │
│ Step 3: 生成 5 段初始轨迹       │
│   用于后续优化                  │
└─────────────────────────────────┘

T = 1s：重规划（方式 2）
┌─────────────────────────────────┐
│ 当前位置：(1.0, 0.5, 1.1)       │
│ 当前速度：(1.2, 0.6, 0.2)       │
│ 新局部目标：(8.2, 4.1, 1.9)     │
│                                 │
│ 局部轨迹剩余：1.5 秒            │
│ 需要采样：4 个点                │
│                                 │
│ 采样策略：                      │
│   t=0.5s ──▶ 从局部轨迹        │
│   t=1.0s ──▶ 从局部轨迹        │
│   t=1.5s ──▶ 从全局轨迹        │
│   t=2.0s ──▶ 从全局轨迹        │
│                                 │
│ 结果：前半段跟随旧轨迹          │
│       后半段跟随全局方向        │
└─────────────────────────────────┘
```

#### STEP 2：MINCO 优化（optimizeTrajectory）

使用 **L-BFGS 算法** 优化轨迹，最小化代价函数：

**代价函数**：
```
J = w_smooth * J_smooth            // 平滑度（最小化 jerk）
  + w_time * J_time                // 时间最优
  + w_obs * J_obs                  // 硬避障
  + w_obs_soft * J_obs_soft        // 软避障
  + w_feas * J_feas                // 动力学可行性
  + w_sqrvar * J_sqrvar            // 方差（轨迹平滑）
```

**优化变量**：
- 每段轨迹的中间控制点位置
- 每段轨迹的持续时间

**约束**：
- 起点和终点的位置、速度、加速度（硬约束）
- 速度和加速度限制（软约束，通过代价函数实现）

### 5.4 地图查询时机详解

#### 问题：初始轨迹是否从障碍物地图搜索？

**直接回答：没有。**

初始轨迹的生成**不涉及**障碍物地图或膨胀地图搜索，具体：

1. **全局轨迹**（`planGlobalTrajWaypoints`）：
   - ❌ 没有 A* 搜索
   - ❌ 没有使用障碍物地图
   - ✅ 直接用 MINCO 从起点到终点生成

2. **初始轨迹**（`computeInitState`）：
   - ❌ 方式1：从"init of init"采样，不查询地图
   - ❌ 方式2：从局部/全局轨迹采样，不查询地图

3. **避障在哪里？**
   - ✅ 在 **MINCO 优化阶段**（`optimizeTrajectory`）
   - 通过代价函数 `J_obs` 使用地图的 ESDF 距离场
   - L-BFGS 优化时才会查询障碍物距离并避障

**简单总结**：
```
初始轨迹 → 不避障（仅保证端点约束）
  ↓
MINCO优化 → 才开始避障（通过距离场+梯度下降）
```

#### 问题：整个规划器工作时只查询一次地图吗？

**直接回答：不是。** 至少查询**三次/三个阶段**：

##### 阶段 1：初始化后的碰撞检查

**函数**：`finelyCheckAndSetConstraintPoints`（poly_traj_optimizer.cpp）

```cpp
// 检查初始轨迹的控制点是否碰撞
PolyTrajOptimizer::CHK_RET ret = ploy_traj_opt_->finelyCheckAndSetConstraintPoints(
    segments, initMJO, true);
```

**工作原理**（poly_traj_optimizer.cpp:172-500）：

`finelyCheckAndSetConstraintPoints` 函数是**最复杂的地图查询阶段**，包含多个步骤：

**步骤 1：碰撞检测与分段**
```cpp
// 沿轨迹密集采样，检测哪些段与障碍物碰撞
for (int i = 0; i < i_end; ++i)
{
  for (size_t j = 0; j < pts_check[i].size(); ++j)
  {
    occ = getInflateOccupancy(pts_check[i][j].second);  // 查询地图
    
    // 记录进入/离开障碍物的控制点 ID
    if (occ && !last_occ) { in_id = i; }    // 进入障碍物
    if (!occ && last_occ) { out_id = i; }   // 离开障碍物
  }
}
// 得到碰撞段列表：segment_ids = [(in1, out1), (in2, out2), ...]
```

**步骤 2：A* 搜索绕障路径**
```cpp
for (size_t i = 0; i < segment_ids.size(); ++i)
{
  // 对每个碰撞段，用 A* 搜索绕障路径
  Eigen::Vector3d in = init_points.col(segment_ids[i].second);   // 离开点
  Eigen::Vector3d out = init_points.col(segment_ids[i].first);   // 进入点
  
  ASTAR_RET ret = a_star_->AstarSearch(resolution_, in, out);  // A* 搜索
  if (ret == ASTAR_RET::SUCCESS)
  {
    a_star_pathes.push_back(a_star_->getPath());  // 保存绕障路径
  }
}
```

**步骤 3：计算障碍物方向和基准点**（核心！）
```cpp
// 对每个碰撞段的控制点
for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
{
  // 找到控制点 j 向 A* 路径的投影交点
  Eigen::Vector3d ctrl_pts_law = init_points.col(j+1) - init_points.col(j-1);
  // ... 计算 intersection_point ...
  
  // 从控制点沿交点方向发射射线，找到最近的障碍物边界
  Eigen::Vector3d direction = (intersection_point - init_points.col(j)).normalized();
  double length = (intersection_point - init_points.col(j)).norm();
  
  for (double a = length; a >= 0.0; a -= resolution_)
  {
    Eigen::Vector3d test_pt = (a/length) * intersection_point + 
                              (1-a/length) * init_points.col(j);
    bool occ = getInflateOccupancy(test_pt);  // 查询地图
    
    if (occ || a < resolution_)
    {
      if (occ) a += resolution_;  // 退回到障碍物边界外
      
      // 【关键】存储障碍物信息
      cps_.base_point[j].push_back(
        (a/length) * intersection_point + (1-a/length) * init_points.col(j)
      );
      cps_.direction[j].push_back(direction);
      break;
    }
  }
}
```

**步骤 4：调整控制点到 A* 路径上**
```cpp
// 如果控制点被标记为需要调整（flag_temp[j] = true）
for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
{
  if (cps_.flag_temp[j])
  {
    // 将控制点移动到 A* 路径附近
    cps_.points.col(j) = /* A* 路径上的对应点 */;
  }
}
```

**完整流程图**：
```
初始轨迹控制点
    │
    ├─► 步骤1：密集采样检测碰撞段
    │   └─► 查询地图：~200 次（沿轨迹密集采样）
    │
    ├─► 步骤2：A* 搜索绕障路径
    │   └─► 查询地图：~1000 次/段（A* 节点扩展）
    │
    ├─► 步骤3：计算障碍物方向和基点
    │   └─► 查询地图：~50 次（射线投射）
    │
    └─► 步骤4：调整控制点
        └─► 不查地图（直接赋值）

总查询次数：~1250 次（取决于碰撞段数量）
```

**关键发现**：
- ✅ **查询地图**：主要在步骤 1（碰撞检测）和步骤 2（A* 搜索）
- ✅ **计算方向**：步骤 3 通过射线投射找到障碍物边界，存储 `base_point` 和 `direction`
- ✅ **后续优化**：不再查询地图，仅使用预先计算的方向进行点积运算

**查询次数修正**：
- 假设 1 个碰撞段，5 个控制点需要调整
- 碰撞检测：~200 次
- A* 搜索：~1000 次
- 射线投射：~50 次
- **总计**：**约 1250 次查询**（远多于之前估计的 25 次！）

---

##### 阶段 2：MINCO 优化阶段

**函数**：`optimizeTrajectory`（poly_traj_optimizer.cpp）

```cpp
bool flag_success = ploy_traj_opt_->optimizeTrajectory(
    headState, tailState, innerPts, initTraj.getDurations(), cstr_pts, final_cost);
```

**工作原理**：
- 使用 **L-BFGS 迭代优化**
- 每次迭代调用代价函数 `addPVAGradCost2CT`
- 代价函数中对**轨迹上采样的每个点**查询距离场
- 计算障碍物惩罚和梯度

**实际代码片段**（poly_traj_optimizer.cpp:823-873）：
```cpp
// 在 addPVAGradCost2CT 中
template <typename EIGENVEC>
void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
{
  // ... 初始化 ...
  
  for (int i = 0; i < N; ++i)  // N = 段数
  {
    for (int j = 0; j <= K; ++j)  // K = 每段的采样点数
    {
      // 计算当前时刻的位置、速度、加速度
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      
      // 【关键】调用障碍物代价计算函数
      if (obstacleGradCostP(i_dp, pos, gradp, costp))
      {
        // 将梯度传播到控制点和时间变量
        gradViolaPc = beta0 * gradp.transpose();
        gradViolaPt = alpha * gradp.transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
        gdT(i) += omg * (costp / K + step * gradViolaPt);
        costs(0) += omg * step * costp;
      }
      
      // 动力学可行性检查（速度、加速度）
      if (feasibilityGradCostV(vel, gradv, costv)) { /* ... */ }
      if (feasibilityGradCostA(acc, grada, costa)) { /* ... */ }
    }
  }
}
```

**obstacleGradCostP 函数详解**（poly_traj_optimizer.cpp:944-985）：

```cpp
bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                          const Eigen::Vector3d &p,
                                          Eigen::Vector3d &gradp,
                                          double &costp)
{
  // 只检查轨迹的前 2/3 部分（后面接近目标时不强制避障）
  if (i_dp == 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_))
    return false;

  gradp.setZero();
  costp = 0;

  // 遍历该控制点关联的所有障碍物方向
  for (size_t j = 0; j < cps_.direction[i_dp].size(); ++j)
  {
    // 计算点 p 到障碍物基准点的射线
    Eigen::Vector3d ray = (p - cps_.base_point[i_dp][j]);
    
    // 投影到障碍物方向上，得到距离
    double dist = ray.dot(cps_.direction[i_dp][j]);
    
    // 距离误差（负值表示侵入障碍物）
    double dist_err = obs_clearance_ - dist;       // 硬约束
    double dist_err_soft = obs_clearance_soft_ - dist;  // 软约束
    
    // 梯度方向（指向远离障碍物的方向）
    Eigen::Vector3d dist_grad = cps_.direction[i_dp][j];

    // 硬约束：三次惩罚
    if (dist_err > 0)
    {
      costp += wei_obs_ * pow(dist_err, 3);
      gradp += -wei_obs_ * 3.0 * dist_err * dist_err * dist_grad;
    }

    // 软约束：平滑惩罚（Huber 函数）
    if (dist_err_soft > 0)
    {
      double r = 0.05;
      double rsqr = r * r;
      double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
      costp += wei_obs_soft_ * rsqr * (term - 1.0);
      gradp += -wei_obs_soft_ * dist_err_soft / term * dist_grad;
    }
  }

  return gradp.norm() > 0;  // 返回是否有梯度
}
```

**关键机制**：

1. **`cps_.direction[i_dp]` 和 `cps_.base_point[i_dp]` 的来源**：
   - 这些是在 **阶段 1（碰撞检查）** 的 `finelyCheckAndSetConstraintPoints` 函数中预先计算的
   - 对每个控制点，找到其向 A* 路径方向上的最近障碍物边界
   - 存储障碍物的**基准点**（边界位置）和**方向**（从控制点指向障碍物的单位向量）

2. **距离计算方式**：
   - **不是查询 ESDF 距离场**，而是使用预先计算的射线方向
   - `dist = (p - base_point).dot(direction)`
   - 这是一种**线性近似**，比 ESDF 查询更快

3. **为什么不直接查询 ESDF？**
   - **性能优化**：预先计算障碍物方向，优化时只需点积运算
   - **梯度精确**：射线方向明确，梯度计算简单准确
   - **局部性**：只关注控制点附近的障碍物，不需要全局查询

4. **两级惩罚机制**：
   - **硬约束**（`obs_clearance_`）：三次惩罚，强制保持安全距离
   - **软约束**（`obs_clearance_soft_`）：平滑惩罚，提前开始避障

**查询次数修正**：
- 假设轨迹 5 段，每段 K=10 个采样点 → 约 50-60 个点
- 每个点查询 `cps_.direction[i_dp].size()` 次（通常 1-3 次）
- L-BFGS 迭代 20-50 次
- **实际不查询地图**：只进行点积运算（使用预先存储的方向和基点）
- **计算复杂度**：50 点 × 2 方向 × 30 迭代 = **约 3000 次点积运算**（非常快！）

这是**计算最频繁但最高效**的阶段！

---

##### 阶段 3：实时安全检查

**函数**：`checkCollisionCallback`（ego_planner_manager.cpp，独立线程）

```cpp
void EGOPlannerManager::checkCollisionCallback(const ros::TimerEvent &e)
{
  LocalTrajData *info = &traj_.local_traj;
  
  // 从当前时刻往后检查 2 秒的轨迹
  double t_cur = ros::Time::now().toSec() - info->start_time;
  double t_2_3 = info->duration * 2 / 3;
  
  for (double t = t_cur; t < t_2_3; t += 0.01)  // 每 10ms 检查一个点
  {
    Eigen::Vector3d pos = info->traj.getPos(t);
    
    // 查询地图
    if (rog_map_->isOccupiedInflate(pos))  // 查询地图
    {
      ROS_WARN("Collision detected ahead!");
      // 触发紧急重规划
      have_local_traj_ = false;
      break;
    }
  }
}
```

**工作原理**：
- 独立线程，**20 Hz 持续运行**（每 50ms 一次）
- 检查未来 2 秒轨迹的碰撞
- 每次检查约 200 个点（2秒 / 0.01秒 = 200）
- 如果发现碰撞，立即触发重规划

**查询次数**：
- 每 50ms 检查 1 次
- 每次检查约 200 个点
- **查询频率**：200 点 × 20 Hz = **4000 次查询/秒**

这是**最持续**的查询！

---

#### 地图查询总结

```
完整规划流程中的地图查询：
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

步骤 1：生成初始轨迹 (computeInitState)
  ├─ 方式 1：多项式初始化 (从零生成)
  │  └─ ❌ 不查地图
  └─ 方式 2：轨迹初始化 (从前一次采样)
     └─ ❌ 不查地图

     ↓

步骤 2：碰撞检查与障碍物方向计算 (finelyCheckAndSetConstraintPoints)
  ├─ 2.1 碰撞检测：密集采样轨迹
  │   └─ ✅ 查询地图：约 200 次
  ├─ 2.2 A* 搜索绕障路径
  │   └─ ✅ 查询地图：约 1000 次/段（节点扩展）
  ├─ 2.3 射线投射找障碍物边界
  │   └─ ✅ 查询地图：约 50 次
  │   └─ 存储 base_point 和 direction
  └─ 2.4 调整控制点到 A* 路径
      └─ ❌ 不查地图（直接赋值）
  【总计：约 1250 次查询】

     ↓

步骤 3：MINCO 优化 (optimizeTrajectory)
  └─ ❌ 不查询地图！
     └─ 仅使用预先计算的 base_point 和 direction
     └─ 进行点积运算（非常快）
     └─ 约 3000 次点积运算（每次迭代 ~100 次）

     ↓

步骤 4：轨迹执行 (EXEC_TRAJ)
  └─ 轨迹服务器发送位置命令

     ║ (并行运行)
     ↓

步骤 5：实时安全检查 (checkCollisionCallback, 20Hz)
  └─ ✅ 查询地图：4000 次/秒（持续）
     └─ 检查未来轨迹，发现碰撞立即重规划
```

#### 对比表

| 阶段 | 是否查询地图 | 查询次数 | 计算方式 | 频率 | 作用 |
|------|------------|---------|---------|------|------|
| **初始轨迹生成** | ❌ 否 | 0 | 几何插值 | 一次性 | 提供初值，不避障 |
| **碰撞检查+方向计算** | ✅ 是 | ~1250 | 采样+A*+射线 | 一次性 | 检测碰撞，计算障碍物方向 |
| **MINCO 优化** | ❌ 否 | 0（地图）<br>~3000（点积） | 使用预存方向 | 迭代（20-50次） | 生成避障、动力学可行轨迹 |
| **实时安全检查** | ✅ 是 | 4000/秒 | 碰撞查询 | 20 Hz 持续 | 检测碰撞，触发重规划 |

#### 关键结论

1. **初始轨迹不避障**：
   - 无论是全局轨迹还是初始局部轨迹，生成时都不查询地图
   - 仅保证端点约束和动力学可行性

2. **避障分两阶段**：
   - **阶段 1（碰撞检查）**：查询地图约 1250 次
     - 密集采样检测碰撞（~200 次）
     - A* 搜索绕障路径（~1000 次）
     - 射线投射找边界（~50 次）
     - **关键输出**：`base_point` 和 `direction`
   
   - **阶段 2（MINCO 优化）**：**不查地图**
     - 使用预先计算的方向和基点
     - 仅进行点积运算（~3000 次，非常快）
     - L-BFGS 迭代 20-50 次

3. **执行阶段持续监控**：
   - 安全线程 20 Hz 持续查询
   - 查询频率最高（每秒 4000 次）
   - 发现碰撞立即重规划

4. **"两次查地图"模式**：
   - **第 1 次（碰撞检查）**：密集查询约 1250 次，计算障碍物方向
   - **第 2 次（安全检查）**：持续查询 4000 次/秒，监控执行
   - **优化阶段**：❌ 不查地图，使用预先计算的信息

**这种设计的优势**：
- ✅ **初始化快速**：不查地图，仅几何插值（<10ms）
- ✅ **碰撞检查精确**：通过 A* 和射线投射找到准确的障碍物边界（~5ms）
- ✅ **优化高效**：预先计算方向，优化时只需点积，避免重复查询地图（10-30ms）
- ✅ **执行安全**：持续监控，及时响应环境变化

**关键创新**：
> EGO-Planner 的核心优化策略是**"预计算+缓存"**：
> - 在碰撞检查阶段一次性计算障碍物方向
> - 在优化阶段复用这些信息，避免重复查询
> - 这使得 L-BFGS 迭代非常快，每次迭代仅需点积运算

---

## 6. 实时重规划机制

### 6.1 EXEC_TRAJ 状态

#### 源码：EXEC_TRAJ（ego_replan_fsm.cpp:237-289）

```cpp
case EXEC_TRAJ:
{
  LocalTrajData *info = &planner_manager_->traj_.local_traj;
  double t_cur = ros::Time::now().toSec() - info->start_time;

  // 1. 检查是否到达目标
  if (touch_goal_)
  {
    if (t_cur > info->duration - 1e-2)
    {
      // 更新悬停位置和 yaw
      hover_pos_ = end_pt_;
      hover_yaw_ = end_yaw_;
      hover_pos_set_ = true;
      have_target_ = false;
      have_trigger_ = false;
      
      // 检查是否有下一个路径点
      if (wpt_id_ < (int)wps_.size() - 1)
      {
        wpt_id_++;
        const Waypoint& next_wp = wps_[wpt_id_];
        const Waypoint& prev_wp = wps_[wpt_id_-1];
        end_pt_ = next_wp.pos;
        end_yaw_ = next_wp.yaw;
        end_vel_ = next_wp.vel;
        planNextWaypoint(next_wp, prev_wp);
        have_target_ = true;
        have_trigger_ = true;
        ROS_INFO("[EGOReplanFSM] Next waypoint: pos=(%.2f, %.2f, %.2f), yaw=%.2f", 
                 end_pt_.x(), end_pt_.y(), end_pt_.z(), end_yaw_);
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      else
      {
        ROS_INFO("[EGOReplanFSM] All waypoints completed, waiting for new target");
        // 通知轨迹服务器停止
        std_msgs::Int8 stop_msg;
        stop_msg.data = 2;
        replan_pub_.publish(stop_msg);
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      goto force_return;
    }
  }

  // 2. 检查重规划条件
  if (t_cur > replan_thresh_ || (!touch_goal_ && t_cur > info->duration - 1e-2))
  {
    changeFSMExecState(REPLAN_TRAJ, "FSM");
  }
  break;
}
```

### 6.2 重规划条件

**条件 1：定期重规划**
```cpp
if (t_cur > replan_thresh_)  // 默认 1.0 秒
```

**原理**：
- 每执行 1 秒轨迹，就重新规划一次
- 利用最新的地图信息
- 适应环境变化

**条件 2：轨迹即将结束但未到达目标**
```cpp
if (!touch_goal_ && t_cur > info->duration - 1e-2)
```

**原理**：
- 当前局部轨迹快结束了，但还没到达全局目标
- 需要规划下一段局部轨迹
- 防止无人机停止

### 6.3 REPLAN_TRAJ 状态

#### 源码：REPLAN_TRAJ（ego_replan_fsm.cpp:224-235）

```cpp
case REPLAN_TRAJ:
{
  if (planFromLocalTraj(1))  // 从当前执行的轨迹重规划
  {
    changeFSMExecState(EXEC_TRAJ, "FSM");
  }
  else
  {
    changeFSMExecState(REPLAN_TRAJ, "FSM");  // 失败则重试
  }
  break;
}
```

#### 函数：planFromLocalTraj（ego_replan_fsm.cpp:412-427）

```cpp
bool EGOReplanFSM::planFromLocalTraj(const int trial_times)
{
  LocalTrajData *info = &planner_manager_->traj_.local_traj;
  double t_cur = ros::Time::now().toSec() - info->start_time;

  // 起始状态：从当前执行的轨迹上采样
  start_pt_ = info->traj.getPos(t_cur);
  start_vel_ = info->traj.getVel(t_cur);
  start_acc_ = info->traj.getAcc(t_cur);

  for (int i = 0; i < trial_times; i++)
  {
    if (callReboundReplan(false, false))  // flag_use_poly_init=false
      return true;
  }
  return false;
}
```

**关键区别**：
- `planFromGlobalTraj`：起始状态 = 当前里程计，`flag_use_poly_init = true`
- `planFromLocalTraj`：起始状态 = 当前轨迹上的点，`flag_use_poly_init = false`

**为什么从轨迹上采样？**
- 保证连续性：新轨迹的起点在旧轨迹上
- 避免跳变：起点的速度和加速度也从旧轨迹获取
- 平滑过渡：重规划时间点（replan_duration = 0.05s）后开始执行新轨迹

---

## 7. 抵达目标点处理

### 7.1 到达判断

```cpp
// 在 EXEC_TRAJ 状态中
if (touch_goal_)  // 局部目标 == 全局目标
{
  if (t_cur > info->duration - 1e-2)  // 轨迹执行完毕
  {
    // 到达目标点！
  }
}
```

**两个条件**：
1. `touch_goal_ = true`：当前局部轨迹的终点就是全局目标
2. `t_cur > duration`：轨迹执行完毕

### 7.2 单路径点完成

```cpp
if (wpt_id_ >= (int)wps_.size() - 1)  // 没有下一个路径点
{
  ROS_INFO("[EGOReplanFSM] All waypoints completed, waiting for new target");
  
  // 1. 更新悬停位置
  hover_pos_ = end_pt_;
  hover_yaw_ = end_yaw_;
  hover_pos_set_ = true;
  
  // 2. 清除标志位
  have_target_ = false;
  have_trigger_ = false;
  
  // 3. 通知轨迹服务器停止
  std_msgs::Int8 stop_msg;
  stop_msg.data = 2;  // 2 = 停止信号
  replan_pub_.publish(stop_msg);
  
  // 4. 切换到等待状态
  changeFSMExecState(WAIT_TARGET, "FSM");
}
```

**轨迹服务器响应**：
- 收到 `stop_msg.data = 2`
- 停止发送位置命令
- 无人机悬停在目标位置

### 7.3 多路径点切换

```cpp
if (wpt_id_ < (int)wps_.size() - 1)  // 还有下一个路径点
{
  wpt_id_++;
  const Waypoint& next_wp = wps_[wpt_id_];
  const Waypoint& prev_wp = wps_[wpt_id_-1];
  
  // 1. 更新目标信息
  end_pt_ = next_wp.pos;
  end_yaw_ = next_wp.yaw;
  end_vel_ = next_wp.vel;
  
  // 2. 规划到下一个路径点
  planNextWaypoint(next_wp, prev_wp);
  
  // 3. 设置标志位
  have_target_ = true;
  have_trigger_ = true;
  
  // 4. 生成新轨迹
  changeFSMExecState(GEN_NEW_TRAJ, "FSM");
}
```

**连续性保证**：
- 使用前一个路径点的速度作为起始速度
- 平滑过渡到下一个路径点

---

## 8. 源码执行流程图

### 8.1 完整时序图

```
时间轴 ───────────────────────────────────────────────────────────▶

┌─────────────────────────────────────────────────────────────────┐
│ T=0s: 收到目标点 (10.0, 5.0, 2.0)                              │
└─────────────────────────────────────────────────────────────────┘
       │
       │ waypointCallback
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ planGlobalTrajWaypoints                                          │
│ - waypoints = [(10.0, 5.0, 2.0)]                               │
│ - waypoints.size() = 1                                          │
│ - innerPts 为空                                                 │
│ - piece_num = 1                                                 │
│ - 生成从当前位置到目标点的简单轨迹                               │
│ - 时间 = distance / (max_vel / 1.5)                            │
│ - 存储为 global_traj                                            │
└─────────────────────────────────────────────────────────────────┘
       │
       │ FSM: WAIT_TARGET → GEN_NEW_TRAJ
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ T=0.01s: planFromGlobalTraj                                     │
│ - start_pt = 当前位置 (0, 0, 1)                                │
│ - start_vel = 当前速度 (0, 0, 0)                               │
│ - flag_use_poly_init = true                                     │
└─────────────────────────────────────────────────────────────────┘
       │
       │ callReboundReplan
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ getLocalTarget                                                   │
│ - planning_horizon = 7.5m                                       │
│ - 沿 global_traj 向前搜索                                       │
│ - 找到距离 ≥ 7.5m 的点作为 local_target                        │
│ - 假设找到 (7.2, 3.6, 1.8)                                     │
│ - touch_goal = false (还没到全局目标)                           │
└─────────────────────────────────────────────────────────────────┘
       │
       │ reboundReplan
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ STEP 1: 初始化轨迹 (computeInitState)                          │
│ - 从 global_traj 采样控制点                                     │
│ - piece_num = round(7.5 / 1.5) = 5 段                          │
│ - innerPts = 4 个中间点                                         │
│ - 生成初始 MINCO 轨迹                                           │
└─────────────────────────────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ STEP 2: 优化轨迹 (optimizeTrajectory)                          │
│ - L-BFGS 优化，最小化代价函数                                   │
│ - 避障约束 + 动力学约束 + 平滑度                               │
│ - 耗时约 10-30ms                                                │
│ - 优化成功！                                                    │
└─────────────────────────────────────────────────────────────────┘
       │
       │ 发布 PolyTraj 消息
       │ FSM: GEN_NEW_TRAJ → EXEC_TRAJ
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ T=0.05s: 开始执行局部轨迹 #1                                   │
│ - duration = 5.0s                                               │
│ - 轨迹服务器每 10ms 发送一次位置命令                            │
└─────────────────────────────────────────────────────────────────┘
       │
       │ ... 执行中 ...
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ T=1.05s: 定期重规划触发 (t_cur > replan_thresh_)               │
│ - FSM: EXEC_TRAJ → REPLAN_TRAJ                                  │
└─────────────────────────────────────────────────────────────────┘
       │
       │ planFromLocalTraj
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ - start_pt = 当前轨迹上的点 (1.0, 0.5, 1.1)                    │
│ - start_vel = 当前轨迹上的速度 (1.2, 0.6, 0.2)                 │
│ - flag_use_poly_init = false (从前一次局部轨迹初始化)          │
└─────────────────────────────────────────────────────────────────┘
       │
       │ callReboundReplan → getLocalTarget → reboundReplan
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ T=1.08s: 生成新的局部轨迹 #2                                   │
│ - 从 (1.0, 0.5, 1.1) 到新的 local_target                       │
│ - 平滑连接到旧轨迹                                              │
│ - FSM: REPLAN_TRAJ → EXEC_TRAJ                                  │
└─────────────────────────────────────────────────────────────────┘
       │
       │ ... 执行中 ...
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ T=2.08s: 再次定期重规划...                                     │
└─────────────────────────────────────────────────────────────────┘
       │
       │ ... 重复重规划和执行 ...
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ T=8.5s: 最后一次重规划                                         │
│ - getLocalTarget:                                               │
│   · 全局轨迹剩余距离 < planning_horizon                        │
│   · local_target = global_end_pt = (10.0, 5.0, 2.0)           │
│   · touch_goal = true                                           │
└─────────────────────────────────────────────────────────────────┘
       │
       │ reboundReplan with touch_goal = true
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ - local_target_vel = end_vel (用户指定的终点速度，这里为 0)     │
│ - 生成最后一段轨迹到目标点                                      │
└─────────────────────────────────────────────────────────────────┘
       │
       │ 执行最后一段轨迹
       ▼
┌─────────────────────────────────────────────────────────────────┐
│ T=10.5s: 到达目标点                                             │
│ - touch_goal = true && t_cur > duration                         │
│ - 更新 hover_pos = end_pt, hover_yaw = end_yaw                 │
│ - 发布停止信号 (replan_msg.data = 2)                           │
│ - FSM: EXEC_TRAJ → WAIT_TARGET                                  │
└─────────────────────────────────────────────────────────────────┘
       │
       ▼
    悬停在目标点，等待新目标
```

### 8.2 数据流图

```
┌──────────────────┐
│ 用户输入目标点   │
│ (10.0, 5.0, 2.0) │
└────────┬─────────┘
         │
         ▼
┌────────────────────────────────────────────────┐
│         waypointCallback / viewpointCallback   │
│  - 提取 end_pt_, end_yaw_, end_vel_            │
│  - 构造 waypoints = [end_pt_]                  │
│  - 调用 planGlobalTrajWaypoints                │
└────────┬───────────────────────────────────────┘
         │
         ▼
┌────────────────────────────────────────────────┐
│       planGlobalTrajWaypoints                  │
│  输入：                                         │
│    - start_pos: 当前位置                       │
│    - waypoints: [目标点]                       │
│    - end_vel: 终点速度                         │
│  处理：                                         │
│    - waypoints.size() == 1                     │
│    - innerPts 为空                             │
│    - 创建 1 段多项式轨迹                       │
│  输出：                                         │
│    - global_traj: 粗略的全局轨迹               │
└────────┬───────────────────────────────────────┘
         │
         ▼
┌────────────────────────────────────────────────┐
│              FSM: GEN_NEW_TRAJ                 │
│  - 调用 planFromGlobalTraj(10)                 │
└────────┬───────────────────────────────────────┘
         │
         ▼
┌────────────────────────────────────────────────┐
│          callReboundReplan                     │
│  1. getLocalTarget                             │
│     输入: global_traj, planning_horizon        │
│     输出: local_target_pt, local_target_vel,   │
│           touch_goal                           │
│  2. reboundReplan                              │
│     输入: start_pt, start_vel, local_target    │
│     输出: local_traj (优化后的局部轨迹)        │
└────────┬───────────────────────────────────────┘
         │
         ▼
┌────────────────────────────────────────────────┐
│  发布 PolyTraj 消息 → 轨迹服务器               │
│  发布 replan 信号 (data=1)                     │
└────────┬───────────────────────────────────────┘
         │
         ▼
┌────────────────────────────────────────────────┐
│              FSM: EXEC_TRAJ                    │
│  - 执行 local_traj                             │
│  - 定期检查重规划条件                          │
└────────┬───────────────────────────────────────┘
         │
         ├─────────────┐
         ▼             ▼
┌──────────────┐  ┌──────────────┐
│ 1 秒后重规划 │  │ 到达目标点   │
│ (REPLAN_TRAJ)│  │ (WAIT_TARGET)│
└──────────────┘  └──────────────┘
```

---

## 9. 总结

### 9.1 关键发现

1. **单路径点处理**：
   - `waypoints.size() == 1` 时，`innerPts` 为空
   - 生成从起点直接到终点的 1 段轨迹
   - 不会打印 ROS_ERROR（防御性检查）

2. **两级轨迹系统**：
   - **全局轨迹**：粗略，无优化，仅导航
   - **局部轨迹**：精细，避障优化，实际执行

3. **规划视野**：
   - 不是一次性规划到目标，而是规划 7.5m
   - 通过重规划逐步接近目标

4. **初始轨迹提取**（重要！）：
   - **首次规划**：两步法
     1. 生成 "init of init" 简单轨迹（2秒）
     2. 从中采样控制点生成真实初始轨迹
   - **重规划**：混合采样
     1. 从局部轨迹剩余部分采样（保证连续性）
     2. 从全局轨迹采样（提供导航方向）
     3. 混合采样保证平滑过渡

5. **连续性保证**：
   - 重规划时从当前轨迹上采样起始状态（位置+速度+加速度）
   - 使用 `replan_duration` 延迟执行新轨迹
   - 混合采样策略避免新旧轨迹突变

6. **地图查询时机**（重要！）：
   - **初始轨迹生成**：❌ 不查地图（全局和局部初始化都不查）
   - **碰撞检查与方向计算**：✅ 查询约 1250 次
     - 密集采样碰撞检测（~200 次）
     - A* 搜索绕障路径（~1000 次）
     - 射线投射找障碍物边界（~50 次）
     - **关键输出**：预先计算并存储 `base_point` 和 `direction`
   - **MINCO 优化**：❌ **不查地图**！
     - 使用预先存储的障碍物方向信息
     - 仅进行点积运算（~3000 次点积，非常快）
     - L-BFGS 迭代 20-50 次，每次迭代无需查询地图
   - **实时安全检查**：✅ 查询 4000 次/秒（20 Hz 持续监控）
   - **总结**：避障分两步，第一步查地图计算方向，第二步用方向优化轨迹

### 9.2 执行效率

| 阶段 | 耗时 | 频率 | 地图查询次数 | 计算类型 |
|------|------|------|-------------|---------|
| 全局规划 | < 1 ms | 收到新目标时 | 0 | MINCO 直接生成 |
| 局部规划（初始化） | 5-10 ms | 每次重规划 | 0 | 几何采样 |
| 局部规划（碰撞检查） | 3-5 ms | 每次重规划 | ~1250 次 | 采样+A*+射线 |
| 局部规划（优化） | 10-30 ms | 每次重规划 | 0（仅点积） | L-BFGS 迭代 |
| 总重规划时间 | 18-45 ms | 每 1 秒 | ~1250 次 | - |
| 实时碰撞检查 | < 1 ms | 20 Hz | ~200 次/检查 | 膨胀地图查询 |
| 轨迹执行 | < 0.5 ms | 100 Hz | 0 | 轨迹插值 |

**地图查询效率分析**：

1. **碰撞检查阶段**（~1250 次查询，3-5ms）：
   - 密集采样：~200 次（快速碰撞检测）
   - A* 搜索：~1000 次（路径规划，最耗时）
   - 射线投射：~50 次（找边界）
   - **一次性投入**：仅在每次重规划时执行 1 次

2. **优化阶段**（0 次查询，10-30ms）：
   - ❌ **不查询地图**！
   - ✅ 使用预先计算的方向
   - ✅ 仅进行点积运算（~3000 次，非常快）
   - **关键优化**：避免重复查询，L-BFGS 迭代高效

3. **安全检查**（4000 次/秒，持续）：
   - 每 50ms 检查 200 个点
   - 单次查询快速（仅碰撞判断）
   - **持续监控**：保证执行安全

**性能关键点**：
- ✅ **预计算策略**：碰撞检查一次性计算方向，优化阶段复用
- ✅ **查询分离**：规划阶段查 1250 次，优化阶段不查
- ✅ **点积优化**：用点积代替 ESDF 查询，速度提升 10-100 倍
- ✅ **合理性**：总规划时间 18-45ms，远低于 1 秒重规划间隔

### 9.3 与文档的差异

**原文档错误**：
- 描述了 A* 搜索和路径缩短，但实际 `planGlobalTrajWaypoints` **没有使用 A***
- 描述了复杂的路径处理，但实际就是简单的多项式插值

**实际实现**：
- 全局轨迹：直接用 MINCO 从起点到终点生成最小 jerk 轨迹
- 无 A* 搜索，无路径缩短
- 简单高效，但不避障

---

**文档版本**：v1.1  
**创建日期**：2025-12-26  
**作者**：源码分析团队
**相关文档**：[EGO_Planner_Technical_Documentation.md](./EGO_Planner_Technical_Documentation.md)
