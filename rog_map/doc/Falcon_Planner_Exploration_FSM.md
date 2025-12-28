# Falcon-Planner 探索状态机设计文档

## 目录
1. [概述](#1-概述)
2. [状态定义](#2-状态定义)
3. [状态转换图](#3-状态转换图)
4. [状态详解](#4-状态详解)
5. [重规划机制](#5-重规划机制)
6. [回调系统](#6-回调系统)
7. [安全机制](#7-安全机制)
8. [可视化系统](#8-可视化系统)
9. [与 EGO-Planner 的对比](#9-与-ego-planner-的对比)

---

## 1. 概述

### 1.1 简介

Falcon-Planner 的探索状态机是一个专门为**自主探索任务**设计的有限状态机（FSM），负责协调边界检测、路径规划、轨迹执行和重规划等核心功能。

### 1.2 核心特点

✅ **探索导向**：专注于未知环境的自主探索  
✅ **动态重规划**：支持三种触发条件的实时重规划  
✅ **层次化网格**：使用分层网格（Hierarchical Grid）进行空间管理  
✅ **前沿检测**：实时检测和更新探索边界（Frontier）  
✅ **安全保障**：独立的安全检查线程  
✅ **性能优化**：动态调整重规划周期

### 1.3 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                Exploration FSM System                    │
├─────────────────────────────────────────────────────────┤
│  ┌──────────────┐    ┌──────────────┐    ┌──────────┐  │
│  │ FSM 状态机   │───▶│ 探索管理器    │───▶│ 规划器   │  │
│  │ (100Hz)      │    │ Exploration  │    │ Planner  │  │
│  │              │    │ Manager      │    │ Manager  │  │
│  └──────────────┘    └──────────────┘    └──────────┘  │
│         │                     │                  │       │
│         ▼                     ▼                  ▼       │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────┐  │
│  │ 安全检查     │    │ 前沿检测      │    │ 可视化   │  │
│  │ (20Hz)       │    │ (2Hz)        │    │ 系统     │  │
│  └──────────────┘    └──────────────┘    └──────────┘  │
└─────────────────────────────────────────────────────────┘
```

---

## 2. 状态定义

### 2.1 状态枚举

```cpp
enum EXPL_STATE {
  INIT,           // 初始化状态
  WAIT_TRIGGER,   // 等待触发
  PLAN_TRAJ,      // 规划轨迹
  PUB_TRAJ,       // 发布轨迹
  EXEC_TRAJ,      // 执行轨迹
  FINISH,         // 探索完成
  RTB             // 返回基地（Return To Base）
};
```

### 2.2 状态字符串映射

```cpp
fd_->state_str_ = {
  "INIT", 
  "WAIT_TRIGGER", 
  "PLAN_TRAJ", 
  "PUB_TRAJ", 
  "EXEC_TRAJ", 
  "FINISH", 
  "RTB"
};
```

---

## 3. 状态转换图

### 3.1 完整状态转换图

```
         ┌─────┐
         │INIT │
         └──┬──┘
            │ have_odom && wait 2s
            ▼
      ┌────────────┐
      │WAIT_TRIGGER│◄────────────┐
      └─────┬──────┘              │
            │ triggered ||        │
            │ auto_start          │
            ▼                     │
      ┌──────────┐                │
      │PLAN_TRAJ │◄──┐            │
      └────┬─────┘   │            │
           │         │ collision  │
           ├─────────┤ (safety)   │
           │         │            │
           │ SUCCEED │            │
           ▼         │            │
      ┌─────────┐    │            │
      │PUB_TRAJ │    │            │
      └────┬────┘    │            │
           │         │            │
           │ dt > 0  │            │
           ▼         │            │
      ┌─────────┐    │            │
      │EXEC_TRAJ│────┤            │
      └────┬────┘    │            │
           │         │            │
           ├─ replan trigger ─────┤
           │                      │
           ├─ no frontier ────────┤
           │                      │
           ▼                      │
       ┌──────┐                   │
       │FINISH│───────────────────┘
       └───┬──┘
           │ (optional)
           ▼
        ┌────┐
        │RTB │
        └────┘
```

### 3.2 状态转换条件总览

| 从状态 | 到状态 | 触发条件 | 触发源 |
|--------|--------|----------|--------|
| INIT | WAIT_TRIGGER | 收到里程计 && 等待2秒 | FSMCallback |
| WAIT_TRIGGER | PLAN_TRAJ | 手动触发 OR 自动启动 | triggerCallback / frontierCallback |
| PLAN_TRAJ | PUB_TRAJ | 规划成功 (res == SUCCEED) | FSMCallback |
| PLAN_TRAJ | FINISH | 没有网格 (res == NO_GRID) | FSMCallback |
| PLAN_TRAJ | PLAN_TRAJ | 规划失败 (res == FAIL) | FSMCallback |
| PUB_TRAJ | EXEC_TRAJ | 轨迹发布完成 (dt > 0) | FSMCallback |
| PUB_TRAJ | PLAN_TRAJ | 碰撞检测失败 | FSMCallback |
| EXEC_TRAJ | PLAN_TRAJ | 需要重规划（3种条件） | FSMCallback |
| EXEC_TRAJ | FINISH | 没有边界 | FSMCallback |
| EXEC_TRAJ | PLAN_TRAJ | 安全检查失败 | safetyCallback |
| FINISH | RTB | 可选（延迟5秒） | FSMCallback |

---

## 4. 状态详解

### 4.1 INIT 状态

**功能**：等待系统初始化完成

**进入条件**：程序启动

**退出条件**：
- 收到里程计数据 (`have_odom_ == true`)
- 等待至少 2 秒

**实现**：
```cpp
case INIT: {
  // Wait for odometry ready
  if (!fd_->have_odom_) {
    ROS_INFO_THROTTLE(1.0, "[FSM] No odom");
    return;
  }
  if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
    ROS_INFO_THROTTLE(1.0, "[FSM] Initializing");
    return;
  }
  // Go to wait trigger when odom is ok
  ROS_WARN("[FSM] Receive odom from topic %s", odom_sub_.getTopic().c_str());
  transitState(WAIT_TRIGGER, "FSM");
  break;
}
```

**关键逻辑**：
- 等待 2 秒是为了确保所有传感器和模块都已就绪
- 记录里程计话题名称，便于调试

---

### 4.2 WAIT_TRIGGER 状态

**功能**：等待探索任务触发，同时更新边界和层次化网格

**进入条件**：从 INIT 或 FINISH 转换而来

**主要活动**：
1. 更新边界检测（通过 `frontierCallback`，2Hz）
2. 初始化层次化网格
3. 可视化当前边界

**触发探索的两种方式**：

**方式 1：手动触发**
```cpp
void triggerCallback(const geometry_msgs::PoseStampedPtr &msg) {
  if (state_ != WAIT_TRIGGER) return;
  if (!frontier_ready_) return;  // 必须等边界生成
  
  fd_->triggered_ = true;
  transitState(PLAN_TRAJ, "triggerCallback");
  expl_manager_->ed_->start_time_ = ros::Time::now();
}
```
- 话题：`/move_base_simple/goal`
- 在 RViz 中使用 "2D Nav Goal" 工具触发

**方式 2：自动启动**
```cpp
// 在 frontierCallback 中
if (expl_manager_->ep_->auto_start_ && frontier_ready_ && state_ == WAIT_TRIGGER) {
  fd_->triggered_ = true;
  transitState(PLAN_TRAJ, "frontierCallback");
  expl_manager_->ed_->start_time_ = ros::Time::now();
}
```
- 参数：`auto_start = true`
- 检测到边界后自动开始探索

**实现**：
```cpp
case WAIT_TRIGGER: {
  // Update frontiers and hgrid when waiting for trigger
  if (frontier_ready_)
    expl_manager_->initializeHierarchicalGrid(fd_->odom_pos_, fd_->odom_vel_);
  
  visualize();
  break;
}
```

---

### 4.3 PLAN_TRAJ 状态

**功能**：调用探索规划器生成新轨迹

**进入条件**：
- 从 WAIT_TRIGGER（首次触发）
- 从 EXEC_TRAJ（重规划）
- 从 PUB_TRAJ（碰撞后重试）

**核心流程**：

#### 4.3.1 确定起始状态

**静态状态**（悬停）：
```cpp
if (fd_->static_state_) {
  fd_->start_pos_ = fd_->odom_pos_;
  fd_->start_vel_ = fd_->odom_vel_;
  fd_->start_acc_.setZero();
  fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
  trajectory_start_time_ = ros::Time::now() + ros::Duration(fp_->replan_duration_);
}
```

**动态状态**（执行中）：
```cpp
else {
  // Replan from non-static state
  LocalTrajData *info = &planner_manager_->local_data_;
  ros::Time time_now = ros::Time::now();
  double t_r = (time_now - info->start_time_).toSec() + fp_->replan_duration_;
  
  trajectory_start_time_ = time_now + ros::Duration(fp_->replan_duration_);
  fd_->start_pos_ = info->position_traj_.evaluateDeBoorT(t_r);
  fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
  fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
  fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
  fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
}
```

#### 4.3.2 调用探索规划器

```cpp
int res = callExplorationPlanner();

int ExplorationFSM::callExplorationPlanner() {
  ros::Time plan_start_time = ros::Time::now();
  
  // 主规划函数
  int res = expl_manager_->planExploreMotionHGrid(
      fd_->start_pos_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  
  double plan_total_time = (ros::Time::now() - plan_start_time).toSec();
  ROS_BLUE_STREAM("[FSM] Exploration planner total time: " 
                  << plan_total_time * 1000.0 << "ms");
  
  // 动态调整重规划周期
  if (plan_total_time > fp_->replan_duration_slow_) {
    fp_->replan_duration_ = fp_->replan_duration_slow_;
  } else if (plan_total_time < fp_->replan_duration_fast_) {
    fp_->replan_duration_ = fp_->replan_duration_fast_;
  } else {
    fp_->replan_duration_ = fp_->replan_duration_default_;
  }
  
  return res;
}
```

#### 4.3.3 处理规划结果

```cpp
if (res == SUCCEED) {
  transitState(PUB_TRAJ, "FSM");
} 
else if (res == FAIL) {
  // Keep trying to replan
  fd_->static_state_ = true;
  ROS_WARN("[FSM] Plan fail");
} 
else if (res == NO_GRID) {
  // No grid means exploration finished
  fd_->static_state_ = true;
  transitState(FINISH, "FSM");
}

// Visualize in separate thread
thread vis_thread(&ExplorationFSM::visualize, this);
vis_thread.detach();
```

**规划结果代码**：
- `SUCCEED = 1`：规划成功
- `FAIL = 0`：规划失败（重试）
- `NO_GRID = -1`：没有可探索的网格（探索完成）

---

### 4.4 PUB_TRAJ 状态

**功能**：检查轨迹安全性并发布

**进入条件**：从 PLAN_TRAJ 规划成功后

**核心流程**：

#### 4.4.1 碰撞检查

```cpp
bool safe = planner_manager_->checkTrajCollision();

if (!safe) {
  ROS_ERROR("[FSM] Collision detected before publishing");
  
  // 尝试使用初始安全轨迹
  planner_manager_->local_data_.position_traj_ = 
      planner_manager_->local_data_.init_traj_bspline_;
  // ... 更新其他轨迹信息
  
  bool safe_init_path = planner_manager_->checkTrajCollision();
  
  if (!safe_init_path) {
    // 初始轨迹也不安全，返回重新规划
    fd_->static_state_ = true;
    transitState(PLAN_TRAJ, "FSM");
    break;
  }
  
  ROS_WARN("[FSM] Replacing with safe initial trajectory");
}
```

#### 4.4.2 速度检查与轨迹延长

**目的**：防止轨迹过慢导致探索效率低

```cpp
double pos_traj_length = planner_manager_->local_data_.position_traj_.getLength();
double pos_traj_duration = planner_manager_->local_data_.position_traj_.getTimeSum();
double avg_pos_vel = pos_traj_length / pos_traj_duration;

double yaw_traj_length = planner_manager_->local_data_.yaw_traj_.getLength();
double yaw_traj_duration = planner_manager_->local_data_.yaw_traj_.getTimeSum();
double avg_yaw_vel = yaw_traj_length / yaw_traj_duration;

if (avg_pos_vel < 0.5 && avg_yaw_vel < 0.5) {
  ROS_WARN("[FSM] Slow trajectory detected");
  
  double yaw_ratio = 1.57 / avg_yaw_vel;  // 目标 yaw 速度 90°/s
  double pos_ratio = 2.0 / avg_pos_vel;   // 目标位置速度 2.0 m/s
  double ratio = 1.0 / std::min(yaw_ratio, pos_ratio);
  
  planner_manager_->local_data_.position_traj_.lengthenTime(ratio);
  planner_manager_->local_data_.yaw_traj_.lengthenTime(ratio);
  
  ROS_WARN("[FSM] Lengthen ratio: %.2lf", ratio);
}
```

#### 4.4.3 构造并发布 B-spline 消息

```cpp
trajectory::Bspline bspline;
auto info = &planner_manager_->local_data_;
bspline.order = 3;
bspline.start_time = info->start_time_;
bspline.traj_id = info->traj_id_;

// Position control points
Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
for (int i = 0; i < pos_pts.rows(); ++i) {
  geometry_msgs::Point pt;
  pt.x = pos_pts(i, 0);
  pt.y = pos_pts(i, 1);
  pt.z = pos_pts(i, 2);
  bspline.pos_pts.push_back(pt);
}

// Knots
Eigen::VectorXd knots = info->position_traj_.getKnot();
for (int i = 0; i < knots.rows(); ++i) {
  bspline.knots.push_back(knots(i));
}

// Yaw control points
Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
for (int i = 0; i < yaw_pts.rows(); ++i) {
  bspline.yaw_pts.push_back(yaw_pts(i, 0));
}
bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

// Publish
double dt = (ros::Time::now() - bspline.start_time).toSec();
if (dt > 0.0) {
  bspline_pub_.publish(bspline);
  fd_->static_state_ = false;
  transitState(EXEC_TRAJ, "FSM");
}
```

---

### 4.5 EXEC_TRAJ 状态

**功能**：执行轨迹并监控重规划条件

**进入条件**：从 PUB_TRAJ 发布轨迹后

**三种重规划触发条件**：

#### 4.5.1 条件 1：边界簇被覆盖

```cpp
if (t_cur > fp_->replan_thresh2_ && 
    expl_manager_->frontier_finder_->isFrontierCovered()) {
  ROS_WARN("[FSM] Replan: cluster covered");
  need_replan = true;
  replan_type = 1;
}
```

**参数**：
- `replan_thresh2_`：通常 0.5-1.0 秒
- 表示已经执行了一定时间

**原理**：
- 当前目标边界簇已被观测到
- 继续飞向该簇没有意义，需要重新选择目标

#### 4.5.2 条件 2：轨迹即将执行完毕

```cpp
else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
  ROS_WARN("[FSM] Replan: traj fully executed");
  need_replan = true;
  replan_type = 2;
}
```

**参数**：
- `replan_thresh1_`：通常 0.2-0.5 秒
- 表示轨迹剩余时间

**原理**：
- 防止轨迹执行完后无人机悬停
- 提前规划下一段轨迹

#### 4.5.3 条件 3：周期性重规划

```cpp
else if (t_cur > fp_->replan_thresh3_) {
  ROS_WARN("[FSM] Replan: periodic call");
  need_replan = true;
  replan_type = 3;
}
```

**参数**：
- `replan_thresh3_`：通常 1.0-2.0 秒

**原理**：
- 定期重规划以适应环境变化
- 使用最新的地图信息

#### 4.5.4 执行重规划

```cpp
if (need_replan) {
  // 更新边界结构
  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
    // 有边界，继续规划
    thread vis_thread(&ExplorationFSM::visualize, this);
    vis_thread.detach();
    transitState(PLAN_TRAJ, "FSM");
  } else {
    // 没有边界，探索完成
    transitState(FINISH, "FSM");
    ROS_WARN("[FSM] Finish exploration: No frontier detected");
  }
}
```

**重规划参数调优表**：

| 参数 | 推荐值 | 作用 | 影响 |
|------|--------|------|------|
| `replan_thresh1_` | 0.2-0.5s | 轨迹剩余时间阈值 | 越小越早重规划，越平滑 |
| `replan_thresh2_` | 0.5-1.0s | 边界覆盖检查时间 | 越小响应越快 |
| `replan_thresh3_` | 1.0-2.0s | 周期性重规划间隔 | 越小地图更新越及时 |

---

### 4.6 FINISH 状态

**功能**：探索任务完成，清理可视化并记录统计信息

**进入条件**：
- 从 PLAN_TRAJ（没有可探索网格）
- 从 EXEC_TRAJ（没有边界）

**主要任务**：

#### 4.6.1 发布停止信号

```cpp
std_msgs::Int32 replan_msg;
replan_msg.data = 2;  // 停止信号
replan_pub_.publish(replan_msg);
```

#### 4.6.2 清理可视化标记

```cpp
auto ed_ = expl_manager_->ed_;
ed_->n_points_.clear();
ed_->refined_ids_.clear();
ed_->unrefined_points_.clear();
ed_->refined_points_.clear();
ed_->refined_views_.clear();
// ... 清理其他可视化数据
visualize();
```

#### 4.6.3 记录探索统计

```cpp
double map_coverage = planner_manager_->map_server_->getMapCoverage();
expl_manager_->ed_->finish_time_ = ros::Time::now();

ROS_INFO("[FSM] Exploration finished. "
         "Start time: %.2lf, End time: %.2lf, Duration: %.2lf, Coverage: %.2lf",
         expl_manager_->ed_->start_time_.toSec(), 
         expl_manager_->ed_->finish_time_.toSec(),
         (expl_manager_->ed_->finish_time_ - expl_manager_->ed_->start_time_).toSec(),
         map_coverage);
```

**输出信息**：
- 探索开始时间
- 探索结束时间
- 探索总时长
- 地图覆盖率

#### 4.6.4 性能统计（可选）

代码中包含详细的性能统计功能（默认关闭）：

```cpp
if (false) {  // 改为 true 启用
  // 保存到 CSV 文件
  // - frontier_times_: 边界检测时间
  // - space_decomp_times_: 空间分解时间
  // - connectivity_graph_times_: 连通图构建时间
  // - cp_times_: 覆盖路径规划时间
  // - sop_times_: SOP问题求解时间
  
  // 计算均值、标准差、最大值、最小值
  calc_mean_std(expl_manager_->ee_->frontier_times_, mean, std, max, min);
  // ...
}
```

---

### 4.7 RTB 状态（Return To Base）

**功能**：返回基地（未实现）

**进入条件**：从 FINISH 延迟 5 秒后（当前已注释）

**实现**：
```cpp
case RTB: {
  // TODO
  ROS_INFO_ONCE("[FSM] Return to base");
  break;
}
```

**未来扩展建议**：
1. 计算返回路径
2. 执行返回轨迹
3. 在基地位置降落

---

## 5. 重规划机制

### 5.1 动态重规划周期

Falcon-Planner 会根据规划器的实际耗时**动态调整**重规划周期：

```cpp
// 动态调整策略
if (plan_total_time > fp_->replan_duration_slow_) {
  fp_->replan_duration_ = fp_->replan_duration_slow_;  // 规划慢，延长周期
} else if (plan_total_time < fp_->replan_duration_fast_) {
  fp_->replan_duration_ = fp_->replan_duration_fast_;  // 规划快，缩短周期
} else {
  fp_->replan_duration_ = fp_->replan_duration_default_;  // 使用默认值
}
```

**典型参数**：
```yaml
replan_duration_fast: 0.05      # 50ms（规划很快时）
replan_duration_default: 0.1    # 100ms（正常情况）
replan_duration_slow: 0.2       # 200ms（规划慢时）
```

**优势**：
- 规划快时减少延迟，提高响应速度
- 规划慢时增加缓冲，避免超时
- 自适应不同场景的复杂度

### 5.2 重规划流程图

```
EXEC_TRAJ
    │
    ├─ 检查条件 1：边界簇覆盖？
    ├─ 检查条件 2：轨迹即将结束？
    ├─ 检查条件 3：超过周期时间？
    │
    ▼ (任一条件满足)
需要重规划
    │
    ├─ 更新边界结构
    │
    ▼
  有边界？
    │
    ├─ Yes ──▶ PLAN_TRAJ
    │
    └─ No ───▶ FINISH
```

### 5.3 重规划类型可视化

代码中会在 RViz 中显示重规划类型：

```cpp
string replan_type_str;
switch (replan_type) {
case 1: replan_type_str = "cluster covered"; break;
case 2: replan_type_str = "traj fully executed"; break;
case 3: replan_type_str = "periodic call"; break;
}

visualization_->drawText(Eigen::Vector3d(12, 0, 0), replan_type_str, 1,
                         PlanningVisualization::Color::Black(), 
                         "replan_type", 0);
```

---

## 6. 回调系统

### 6.1 回调函数列表

| 回调函数 | 频率 | 功能 | 关键操作 |
|----------|------|------|----------|
| `FSMCallback` | 100 Hz | 主状态机逻辑 | 状态转换、规划调用 |
| `safetyCallback` | 20 Hz | 安全检查 | 碰撞检测、紧急重规划 |
| `frontierCallback` | 2 Hz | 边界更新 | 搜索边界、自动启动 |
| `triggerCallback` | 事件触发 | 手动启动 | 用户触发探索 |
| `odometryCallback` | 传感器频率 | 里程计更新 | 位置、速度、姿态 |

### 6.2 FSMCallback（100Hz）

**主状态机回调**，负责状态转换和逻辑执行。

**性能监控**：
```cpp
static int cnt_callback = 0;
static ros::Time start_time = ros::Time::now();
double dt = (ros::Time::now() - start_time).toSec();
LOG(INFO) << "[FSM] Current state: " << fd_->state_str_[int(state_)]
          << ", callback rate: " << cnt_callback / dt << "Hz";
```

**日志记录**：
- 当前状态
- 回调频率
- 回调耗时

### 6.3 safetyCallback（20Hz）

**独立安全检查线程**，确保轨迹安全。

```cpp
void ExplorationFSM::safetyCallback(const ros::TimerEvent &e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    bool safe = planner_manager_->checkTrajCollision();
    if (!safe) {
      ROS_ERROR("[FSM] Collision detected! Replanning...");
      
      // 发布重规划信号给 traj_server
      std_msgs::Int32 replan_msg;
      replan_msg.data = 1;
      replan_pub_.publish(replan_msg);
      
      // 强制重新规划
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}
```

**关键特点**：
- 仅在 EXEC_TRAJ 状态检查
- 检测到碰撞立即触发重规划
- 发送信号给 traj_server 停止当前轨迹

### 6.4 frontierCallback（2Hz）

**边界更新回调**，在等待状态时持续更新边界。

**执行条件**：
```cpp
if (state_ == WAIT_TRIGGER || state_ == FINISH) {
  // 搜索边界
  ft->searchFrontiers();
  ft->computeFrontiersToVisit();
  
  // 获取边界信息
  ft->getFrontiers(ed->frontiers_);
  ft->getDormantFrontiers(ed->dormant_frontiers_);
  ft->getTinyFrontiers(ed->tiny_frontiers_);
  ft->getTopViewpointsInfo(fd_->odom_pos_, ed->points_, ed->yaws_, ed->averages_);
  
  // 可视化
  visualization_->drawFrontierPointcloud(ed->frontiers_);
  // ...
}
```

**自动启动逻辑**：
```cpp
if (expl_manager_->ep_->auto_start_ && frontier_ready_ && state_ == WAIT_TRIGGER) {
  fd_->triggered_ = true;
  transitState(PLAN_TRAJ, "frontierCallback");
  expl_manager_->ed_->start_time_ = ros::Time::now();
}
```

**延迟机制**：
```cpp
static int delay = 0;
if (++delay < 5) return;  // 延迟 2.5 秒（5 * 0.5s）
```
- 等待系统完全稳定后再开始检测边界

### 6.5 triggerCallback（事件触发）

**手动触发回调**，响应 RViz 的 "2D Nav Goal"。

```cpp
void ExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg) {
  if (state_ != WAIT_TRIGGER) return;
  if (!frontier_ready_) return;  // 必须等边界生成
  
  fd_->triggered_ = true;
  transitState(PLAN_TRAJ, "triggerCallback");
  expl_manager_->ed_->start_time_ = ros::Time::now();
  ROS_INFO("[FSM] Exploration start time: %lf", 
           expl_manager_->ed_->start_time_.toSec());
}
```

**话题**：`/move_base_simple/goal`

### 6.6 odometryCallback（传感器频率）

**里程计回调**，更新无人机状态。

```cpp
void ExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
  // 位置
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;
  
  // 速度
  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;
  
  // 姿态
  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;
  
  // Yaw 角
  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));
  
  // 标记已收到里程计
  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = ros::Time::now();
  }
}
```

**轨迹记录**（可选）：
```cpp
// 每 10 次回调记录一次位置
static int delay = 0;
static double accum_dist = 0;
if (++delay < 10) return;
else {
  delay = 0;
  if (expl_manager_->ed_->trajecotry_.size() > 0)
    accum_dist += (fd_->odom_pos_ - expl_manager_->ed_->trajecotry_.back()).norm();
  expl_manager_->ed_->trajecotry_.push_back(fd_->odom_pos_);
}
```
- 记录累计飞行距离
- 用于后续分析

---

## 7. 安全机制

### 7.1 多层安全检查

Falcon-Planner 实现了**三层安全检查**：

```
┌──────────────────────────────────┐
│ Layer 1: 发布前检查 (PUB_TRAJ)   │
│ - 检查优化后的轨迹               │
│ - 碰撞则使用初始安全轨迹         │
└──────────────┬───────────────────┘
               │
               ▼
┌──────────────────────────────────┐
│ Layer 2: 独立安全线程 (20Hz)     │
│ - 实时检测执行中的轨迹           │
│ - 碰撞则立即触发重规划           │
└──────────────┬───────────────────┘
               │
               ▼
┌──────────────────────────────────┐
│ Layer 3: 占用检查（可选）         │
│ - 检查当前位置是否在障碍物内     │
│ - 失败则终止程序                 │
└──────────────────────────────────┘
```

### 7.2 发布前碰撞检查

在 PUB_TRAJ 状态中：

```cpp
bool safe = planner_manager_->checkTrajCollision();

if (!safe) {
  ROS_ERROR("[FSM] Collision detected before publishing");
  
  // 回退到初始轨迹
  planner_manager_->local_data_.position_traj_ = 
      planner_manager_->local_data_.init_traj_bspline_;
  planner_manager_->local_data_.velocity_traj_ = 
      planner_manager_->local_data_.position_traj_.getDerivative();
  planner_manager_->local_data_.acceleration_traj_ = 
      planner_manager_->local_data_.velocity_traj_.getDerivative();
  
  // 检查初始轨迹
  bool safe_init_path = planner_manager_->checkTrajCollision();
  
  if (!safe_init_path) {
    // 初始轨迹也不安全，重新规划
    fd_->static_state_ = true;
    transitState(PLAN_TRAJ, "FSM");
    return;
  }
  
  ROS_WARN("[FSM] Using safe initial trajectory");
}
```

**安全策略**：
1. 优化后的轨迹不安全 → 使用初始 A* 轨迹
2. 初始轨迹也不安全 → 从静态状态重新规划
3. 两次检查确保发布的轨迹安全

### 7.3 实时安全监控

独立的安全线程（20Hz）：

```cpp
void ExplorationFSM::safetyCallback(const ros::TimerEvent &e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    bool safe = planner_manager_->checkTrajCollision();
    if (!safe) {
      ROS_ERROR("[FSM] Collision detected! Replanning...");
      
      // 通知 traj_server
      std_msgs::Int32 replan_msg;
      replan_msg.data = 1;
      replan_pub_.publish(replan_msg);
      
      // 强制重规划
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}
```

**优势**：
- 独立于主状态机，不受主循环延迟影响
- 20Hz 检查频率足够快，能及时发现碰撞
- 立即触发重规划，避免执行不安全轨迹

### 7.4 占用检查（可选）

```cpp
// 在 safetyCallback 开头（当前已注释）
CHECK(planner_manager_->map_server_->getOccupancy(fd_->odom_pos_) !=
      voxel_mapping::OccupancyType::OCCUPIED)
    << "Current pos is in occupied area. Please restart.";
```

**用途**：
- 检测是否卡在障碍物内
- 通常在仿真中不需要（里程计准确）
- 实机飞行时可能有用（里程计漂移）

---

## 8. 可视化系统

### 8.1 可视化内容

Falcon-Planner 提供丰富的可视化功能：

#### 8.1.1 边界可视化

```cpp
// 根据地图分辨率选择可视化方式
double map_resolution = planner_manager_->map_server_->getResolution();
if (map_resolution > 0.1) {
  // 高分辨率地图：密集点云
  visualization_->drawFrontierPointcloudHighResolution(ed_ptr->frontiers_, map_resolution);
} else {
  // 低分辨率地图：稀疏点云
  visualization_->drawFrontierPointcloud(ed_ptr->frontiers_);
}
```

**边界类型**：
- **活跃边界**（frontiers）：蓝色点云
- **休眠边界**（dormant_frontiers）：灰色点云
- **微小边界**（tiny_frontiers）：淡色点云

**边界ID标注**：
```cpp
for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
  visualization_->drawText(ed_ptr->averages_[i], to_string(i), 0.5,
                           PlanningVisualization::Color::Red(), 
                           "frontier_id", i);
}
```

#### 8.1.2 视点可视化

```cpp
// 全局视点（深绿色球体）
visualization_->drawSpheres(ed_ptr->points_, 0.2, 
                            PlanningVisualization::Color::DeepGreen(),
                            "points", 0);

// 视点朝向（黄色线）
visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05,
                          PlanningVisualization::Color::Yellow(), 
                          "frontier_view", 0);

// 视场角（黑色线）
visualization_->drawLines(ed_ptr->views1_, ed_ptr->views2_, 0.02,
                          PlanningVisualization::Color::Black(), 
                          "view_fov", 0);
```

#### 8.1.3 精细化视点

```cpp
// 精细化后的视点（蓝色球体）
visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, 
                            PlanningVisualization::Color::Blue(),
                            "refined_pts", 0);

// 精细化视点朝向（浅蓝色线）
visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
                          PlanningVisualization::Color::LightBlue(), 
                          "refined_view", 0);

// 精细化与原始视点的对应关系（黄色线）
visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.02,
                          PlanningVisualization::Color::Yellow(), 
                          "refine_pair", 0);
```

#### 8.1.4 路径可视化

```cpp
// 层次化网格路径（蓝色线）
visualization_->drawLines(ed_ptr->grid_tour2_, 0.1, 
                          PlanningVisualization::Color::Blue2(),
                          "hgrid_tour_mc", 0);

// 精细化路径（深蓝色线）
visualization_->drawLines(ed_ptr->refined_tour_, 0.07, 
                          PlanningVisualization::Color::DeepBlue(),
                          "refined_tour", 0);

// 到下一个目标的路径（深蓝色线）
visualization_->drawLines(ed_ptr->path_next_goal_, 0.07, 
                          PlanningVisualization::Color::DeepBlue(),
                          "path_next_goal", 0);
```

#### 8.1.5 轨迹可视化

```cpp
if (info->position_traj_.getKnot().size() > 0) {
  // 初始轨迹（深绿色 B-spline）
  visualization_->drawBspline(planner_manager_->local_data_.init_traj_bspline_, 
                              0.05,
                              PlanningVisualization::Color::DeepGreen(), 
                              true, 0.1,
                              PlanningVisualization::Color::DeepGreen(), 0);
  
  // 优化后的轨迹（深蓝色 B-spline）
  visualization_->drawBspline(planner_manager_->local_data_.position_traj_, 
                              0.05,
                              PlanningVisualization::Color::DeepBlue(), 
                              true, 0.1,
                              PlanningVisualization::Color::DeepBlue(), 1);
}
```

#### 8.1.6 下一个目标

```cpp
// 下一个目标位置和朝向（箭头）
Eigen::Quaterniond next_q;
next_q = Eigen::AngleAxisd(ed_ptr->next_yaw_, Eigen::Vector3d::UnitZ());
visualization_->drawPose(ed_ptr->next_goal_, next_q, "next_goal", 0);
```

### 8.2 可视化线程

为了不阻塞主状态机，可视化在独立线程中执行：

```cpp
thread vis_thread(&ExplorationFSM::visualize, this);
vis_thread.detach();
```

**优势**：
- 不影响规划性能
- 可视化延迟不会导致重规划延迟
- 更好的实时性

### 8.3 可视化颜色方案

| 元素 | 颜色 | 含义 |
|------|------|------|
| 活跃边界 | 蓝色 | 可探索的边界 |
| 休眠边界 | 灰色 | 暂时不可达的边界 |
| 全局视点 | 深绿色 | 边界的观测视点 |
| 精细化视点 | 蓝色 | 优化后的视点 |
| 初始轨迹 | 深绿色 | A* 生成的安全轨迹 |
| 优化轨迹 | 深蓝色 | B-spline 优化后的轨迹 |
| 视点朝向 | 黄色 | 相机朝向 |
| 视场角 | 黑色 | FOV 范围 |

---

## 9. 与 EGO-Planner 的对比

### 9.1 状态机对比

| 特性 | Falcon-Planner | EGO-Planner |
|------|----------------|-------------|
| 状态数量 | 7 个 | 8 个 |
| 目标 | 自主探索 | 导航到目标点 |
| 触发方式 | 自动 + 手动 | 手动 + 预设路径 |
| 重规划条件 | 3 种（边界覆盖、轨迹结束、周期） | 2 种（轨迹结束、周期） |
| 安全检查 | 独立线程 (20Hz) | 独立线程 (20Hz) |
| 完成条件 | 无边界 | 所有路径点完成 |

### 9.2 状态对应关系

| Falcon-Planner | EGO-Planner | 说明 |
|----------------|-------------|------|
| INIT | INIT | 初始化 |
| WAIT_TRIGGER | WAIT_TARGET | 等待目标 |
| - | PRE_ARM | Falcon无需 PX4 控制 |
| - | TAKEOFF | Falcon无需自动起飞 |
| PLAN_TRAJ | GEN_NEW_TRAJ | 生成新轨迹 |
| PUB_TRAJ | - | Falcon 有独立发布状态 |
| EXEC_TRAJ | EXEC_TRAJ | 执行轨迹 |
| - | REPLAN_TRAJ | Falcon 复用 PLAN_TRAJ |
| FINISH | WAIT_TARGET | Falcon 探索完成后停止 |
| RTB | - | 返回基地（未实现） |
| - | EMERGENCY_STOP | Falcon 直接重规划 |

### 9.3 核心差异

#### 9.3.1 探索 vs 导航

**Falcon-Planner（探索）**：
- 目标动态生成（边界）
- 持续探索直到完成
- 重点：覆盖未知区域

**EGO-Planner（导航）**：
- 目标预先指定
- 到达目标点后停止
- 重点：快速到达目标

#### 9.3.2 重规划触发

**Falcon-Planner**：
```cpp
// 3 种条件
1. 边界簇被覆盖
2. 轨迹即将结束
3. 周期性重规划
```

**EGO-Planner**：
```cpp
// 2 种条件
1. 轨迹即将结束
2. 周期性重规划
```

#### 9.3.3 边界管理

**Falcon-Planner**：
- 持续检测边界（2Hz）
- 维护边界数据库
- 根据边界规划探索路径

**EGO-Planner**：
- 无边界概念
- 直接规划到目标点

#### 9.3.4 自主性

**Falcon-Planner**：
- 完全自主决策探索目标
- 自动启动探索
- 自动判断完成条件

**EGO-Planner**：
- 需要外部指定目标
- 支持探索接口但需外部规划器

---

## 10. 参数配置

### 10.1 FSM 参数

```yaml
/exploration_manager/fsm:
  # 重规划阈值
  replan_thresh1: 0.3          # 轨迹剩余时间（秒）
  replan_thresh2: 0.8          # 边界覆盖检查时间（秒）
  replan_thresh3: 1.5          # 周期性重规划间隔（秒）
  
  # 重规划周期
  replan_duration_fast: 0.05    # 快速规划时（秒）
  replan_duration_default: 0.1  # 默认（秒）
  replan_duration_slow: 0.2     # 慢速规划时（秒）
```

### 10.2 探索管理器参数

```yaml
/exploration_manager:
  auto_start: true              # 自动启动探索
  
  # 边界检测
  frontier:
    cluster_min: 50             # 最小边界簇大小
    cluster_size_: 2.0          # 聚类半径（米）
    min_candidate_dist: 1.0     # 最小候选距离（米）
    min_candidate_clearance: 0.5  # 最小安全间隙（米）
  
  # 视点生成
  viewpoint:
    max_decay: 0.9              # 信息增益衰减系数
    w_dir: 1.0                  # 方向权重
```

---

## 11. 常见问题

### 11.1 探索不启动

**现象**：停在 WAIT_TRIGGER 状态

**可能原因**：
1. `auto_start = false` 且未手动触发
2. 未检测到边界
3. 里程计未初始化

**解决**：
```bash
# 检查边界
rostopic echo /exploration_manager/frontiers

# 手动触发（在 RViz 中使用 2D Nav Goal）
# 或设置 auto_start = true
```

### 11.2 频繁重规划

**现象**：每 0.5 秒就重规划一次

**可能原因**：
- `replan_thresh2` 设置过小
- 边界簇很快被覆盖

**解决**：
```yaml
replan_thresh2: 1.0  # 增大到 1.0 秒
```

### 11.3 探索效率低

**现象**：飞行缓慢，重复探索

**可能原因**：
- 轨迹过慢（被延长）
- 边界选择策略不优

**解决**：
```cpp
// 检查 PUB_TRAJ 中的轨迹延长逻辑
if (avg_pos_vel < 0.5 && avg_yaw_vel < 0.5) {
  // 可能过于保守，调整阈值
  if (avg_pos_vel < 0.3 && avg_yaw_vel < 0.3) {  // 改为 0.3
    // ...
  }
}
```

### 11.4 提前结束探索

**现象**：还有未探索区域就 FINISH 了

**可能原因**：
- 边界检测参数过于严格
- 休眠边界未被唤醒

**解决**：
```yaml
frontier:
  cluster_min: 30  # 减小最小簇大小，检测更多边界
```

---

## 12. 最佳实践

### 12.1 参数调优流程

1. **初始设置**（保守）
   ```yaml
   replan_thresh1: 0.5
   replan_thresh2: 1.0
   replan_thresh3: 2.0
   replan_duration_default: 0.15
   ```

2. **观察探索行为**
   - 是否频繁重规划？→ 增大 thresh2/thresh3
   - 是否反应迟钝？→ 减小 thresh2/thresh3
   - 规划是否超时？→ 增大 replan_duration

3. **性能优化**
   - 规划稳定在 50ms 以内 → 使用 replan_duration_fast = 0.05
   - 规划经常超时 → 增大 replan_duration_slow

### 12.2 调试建议

**启用详细日志**：
```cpp
// 在 FSMCallback 中取消注释
if (false) {  // 改为 true
  // Print current states
  ROS_INFO("[FSM] Current pos: %.2lf, %.2lf, %.2lf", ...);
  // ...
}
```

**可视化重规划类型**：
代码已默认启用，在 RViz 中查看 "replan_type" 文本标记。

**记录性能数据**：
```cpp
// 在 FINISH 状态中
if (false) {  // 改为 true 启用
  // Save times to CSV
  // ...
}
```

---

## 13. 总结

### 13.1 核心优势

✅ **完全自主**：无需人工指定探索目标  
✅ **智能重规划**：3 种触发条件，动态调整周期  
✅ **安全可靠**：多层安全检查，独立安全线程  
✅ **高效探索**：层次化网格 + 边界检测  
✅ **可视化丰富**：实时显示探索过程  

### 13.2 关键设计

1. **状态机设计**：7 个状态，清晰的转换逻辑
2. **回调系统**：100Hz 主循环 + 20Hz 安全 + 2Hz 边界
3. **重规划机制**：动态周期 + 多条件触发
4. **安全保障**：发布前检查 + 实时监控

### 13.3 与 EGO-Planner 的互补

| 场景 | 推荐规划器 |
|------|-----------|
| 自主探索未知环境 | Falcon-Planner |
| 导航到指定目标点 | EGO-Planner |
| 视点探索（有外部规划器） | EGO-Planner + Viewpoint Planner |
| 完整探索系统 | Falcon-Planner |

---

**文档版本**：v1.0  
**最后更新**：2025-12-26  
**作者**：Falcon-Planner Analysis Team  
**参考**：HKUST FAST-Lab Falcon-Planner

**相关文档**：
- [EGO_Planner_Technical_Documentation.md](./EGO_Planner_Technical_Documentation.md)
