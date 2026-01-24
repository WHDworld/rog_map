# EGOReplanFSM 轨迹执行优化方案

> ✅ **状态：已完成** (2026-01-23)
> 
> 编译验证通过，所有修改已实施。

## 目标

在 `ego_replan_fsm.cpp` 中：
1. 移除 `poly_traj_pub_` 发布（不再通过 `ego_traj_server` 中转）
2. 添加独立的轨迹执行定时器 `execution_timer_`（100Hz）
3. 添加新旧轨迹衔接机制（参考 `exploration_fsm_new.cpp`）

---

## 修改文件

### [MODIFY] ego_replan_fsm.h

#### 添加成员变量（在 `private:` 部分）

```cpp
    /* 轨迹执行定时器 */
    ros::Timer execution_timer_;
    
    /* 轨迹执行状态 */
    double traj_start_time_, traj_duration_;
    Eigen::Vector3d last_pos_;
    double last_yaw_, last_yaw_dot_;
    
    /* 旧轨迹保存（用于衔接） */
    bool has_prev_traj_;
    double prev_traj_start_time_, prev_traj_duration_;
    poly_traj::Trajectory prev_traj_;
    std::vector<double> prev_yaw_ctrl_pts_;
    bool prev_use_yaw_traj_;
    double prev_last_yaw_;
```

#### 添加成员函数声明

```cpp
    /* 轨迹执行回调 */
    void executionCallback(const ros::TimerEvent &e);
    void publishCmd(const Eigen::Vector3d& p, const Eigen::Vector3d& v, 
                    const Eigen::Vector3d& a, double yaw, double yaw_dot);
    void savePrevTrajectory();
```

---

### [MODIFY] ego_replan_fsm.cpp

#### 1. 初始化（`init()` 函数）

在初始化成员变量处添加：

```cpp
    // 轨迹执行状态初始化
    has_prev_traj_ = false;
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;
    last_pos_ = Eigen::Vector3d::Zero();
```

在创建定时器处添加：

```cpp
    // 轨迹执行定时器（100Hz）
    execution_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::executionCallback, this);
```

#### 2. 新增 `savePrevTrajectory()` 函数

在规划成功后调用，保存旧轨迹：

```cpp
void EGOReplanFSM::savePrevTrajectory()
{
    auto& local_traj = planner_manager_->traj_.local_traj;
    if (local_traj.traj.getPieceNum() > 0)
    {
        has_prev_traj_ = true;
        prev_traj_start_time_ = local_traj.start_time;
        prev_traj_duration_ = local_traj.duration;
        prev_traj_ = local_traj.traj;
        prev_use_yaw_traj_ = local_traj.use_yaw_traj;
        prev_last_yaw_ = last_yaw_;
        if (local_traj.use_yaw_traj && !local_traj.yaw_ctrl_pts.empty())
        {
            prev_yaw_ctrl_pts_ = local_traj.yaw_ctrl_pts;
        }
        else
        {
            prev_yaw_ctrl_pts_.clear();
        }
    }
}
```

#### 3. 修改 `callReboundReplan()` 函数

在调用规划器前保存旧轨迹，规划成功后更新轨迹状态：

```cpp
bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
{
    // === 新增：规划前保存旧轨迹 ===
    savePrevTrajectory();
    
    // ... 原有规划逻辑 ...
    
    if (success)
    {
        // === 修改：移除 poly_traj_pub_ 发布 ===
        // 更新轨迹执行状态
        auto& local_traj = planner_manager_->traj_.local_traj;
        traj_start_time_ = local_traj.start_time;
        traj_duration_ = local_traj.duration;
        
        // 同步 last_yaw_ 到新轨迹起点
        if (local_traj.use_yaw_traj && !local_traj.yaw_ctrl_pts.empty())
        {
            last_yaw_ = YawOptimizer::evaluateYaw(local_traj.yaw_ctrl_pts, 0.0, traj_duration_);
        }
    }
    
    return success;
}
```

#### 4. 新增 `executionCallback()` 函数

```cpp
void EGOReplanFSM::executionCallback(const ros::TimerEvent &e)
{
    if (!have_odom_ || exec_state_ == INIT || exec_state_ == PRE_ARM || 
        exec_state_ == TAKEOFF || exec_state_ == WAIT_TARGET)
    {
        return;
    }
    
    ros::Time now = ros::Time::now();
    auto& local_traj = planner_manager_->traj_.local_traj;
    
    if (local_traj.traj.getPieceNum() == 0)
    {
        publishHoverCmd();
        return;
    }
    
    double t_cur = now.toSec() - traj_start_time_;
    
    // Case 1: 过渡期（新轨迹还没开始）-> 执行旧轨迹
    if (t_cur < 0 && has_prev_traj_)
    {
        double t_prev = now.toSec() - prev_traj_start_time_;
        if (prev_traj_.getPieceNum() > 0 && t_prev < prev_traj_duration_)
        {
            double t_eval = std::min(t_prev, prev_traj_.getTotalDuration());
            Eigen::Vector3d pos = prev_traj_.getPos(t_eval);
            Eigen::Vector3d vel = prev_traj_.getVel(t_eval);
            Eigen::Vector3d acc = prev_traj_.getAcc(t_eval);
            
            double yaw = last_yaw_, yaw_dot = 0.0;
            if (prev_use_yaw_traj_ && !prev_yaw_ctrl_pts_.empty())
            {
                yaw = YawOptimizer::evaluateYaw(prev_yaw_ctrl_pts_, t_eval, prev_traj_duration_);
                yaw_dot = YawOptimizer::evaluateYawDot(prev_yaw_ctrl_pts_, t_eval, prev_traj_duration_);
            }
            
            last_pos_ = pos;
            last_yaw_ = yaw;
            last_yaw_dot_ = yaw_dot;
            publishCmd(pos, vel, acc, yaw, yaw_dot);
            return;
        }
    }
    
    // Case 2: 正常执行当前轨迹
    if (t_cur >= 0 && t_cur < traj_duration_)
    {
        double t_eval = std::clamp(t_cur, 0.0, local_traj.traj.getTotalDuration());
        Eigen::Vector3d pos = local_traj.traj.getPos(t_eval);
        Eigen::Vector3d vel = local_traj.traj.getVel(t_eval);
        Eigen::Vector3d acc = local_traj.traj.getAcc(t_eval);
        
        double yaw = last_yaw_, yaw_dot = 0.0;
        if (local_traj.use_yaw_traj && !local_traj.yaw_ctrl_pts.empty())
        {
            yaw = YawOptimizer::evaluateYaw(local_traj.yaw_ctrl_pts, t_eval, traj_duration_);
            yaw_dot = YawOptimizer::evaluateYawDot(local_traj.yaw_ctrl_pts, t_eval, traj_duration_);
        }
        
        last_pos_ = pos;
        last_yaw_ = yaw;
        last_yaw_dot_ = yaw_dot;
        publishCmd(pos, vel, acc, yaw, yaw_dot);
        return;
    }
    
    // Case 3: 轨迹执行完毕 -> 悬停
    publishHoverCmd();
}

void EGOReplanFSM::publishCmd(const Eigen::Vector3d& p, const Eigen::Vector3d& v, 
                               const Eigen::Vector3d& a, double yaw, double yaw_dot)
{
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    
    cmd.position.x = p.x();
    cmd.position.y = p.y();
    cmd.position.z = p.z();
    cmd.velocity.x = v.x();
    cmd.velocity.y = v.y();
    cmd.velocity.z = v.z();
    cmd.acceleration.x = a.x();
    cmd.acceleration.y = a.y();
    cmd.acceleration.z = a.z();
    cmd.yaw = yaw;
    cmd.yaw_dot = yaw_dot;
    
    pos_cmd_pub_.publish(cmd);
}
```

---

## 删除的代码

- `poly_traj_pub_.publish(poly_msg);` 调用（第615行）

---

## 验证计划

1. 编译验证
2. 运行 `roslaunch rog_map planner_test.launch`
3. 检查 `/planning/pos_cmd` 话题是否正常发布
4. 观察轨迹切换时是否平滑
