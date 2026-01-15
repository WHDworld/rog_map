# 轨迹日志记录与分析系统使用说明

## 概述

轨迹日志系统自动记录无人机飞行过程中的期望指令与实际反馈，用于分析轨迹规划质量、平滑性和跟踪精度。

**路径自动检测**：系统自动使用 ROS 包路径，无需手动配置。

---

## 1. 自动日志记录

### 日志文件位置
```
<rog_map_package>/log/trajectory_log.csv
```
*路径通过 `ros::package::getPath("rog_map")` 自动获取*

### 日志格式 (CSV)

| 列名 | 说明 | 单位 |
|---|---|---|
| `timestamp` | 时间戳 | s |
| `cmd_x`, `cmd_y`, `cmd_z` | 期望位置 | m |
| `cmd_vx`, `cmd_vy`, `cmd_vz` | 期望速度 | m/s |
| `cmd_yaw`, `cmd_yaw_dot` | 期望Yaw角和角速度 | rad, rad/s |
| `odom_x`, `odom_y`, `odom_z` | 里程计实际位置 | m |
| `odom_vx`, `odom_vy`, `odom_vz` | 里程计实际速度 | m/s |
| `odom_yaw` | 里程计实际Yaw | rad |
| `traj_end_x`, `traj_end_y`, `traj_end_z` | 当前轨迹终点 | m |

### 启用/禁用日志

在 `exploration_fsm_new.h` 中修改：
```cpp
bool enable_traj_logging_ = true;  // 设为 false 禁用日志
```

---

## 2. 可视化分析脚本

### 运行方式

```bash
cd ~/experiment/STAR-Searcher/src/ROG-Map/rog_map/log
python3 analyze_trajectory.py
```

### 输出内容

#### 终端统计信息
- 数据记录数量和时间范围
- 位置/速度/Yaw 跟踪误差（平均值、最大值）

#### 可视化图表 (`trajectory_analysis.png`)

**4行 × 3列布局**：

| 行 | 内容 | 说明 |
|---|---|---|
| **第1行** | 位置 X/Y/Z | 期望 vs 实际，显示轨迹终点，标注最大跳变 |
| **第2行** | 速度 VX/VY/VZ | 期望 vs 实际，标注速度平滑性 |
| **第3行** | Yaw (左) + 3D轨迹 (中右) | Yaw对比图 + 三维空间轨迹 |
| **第4行** | 跟踪误差分析 | 位置误差 / 速度误差 / Yaw误差 |

### 分析指标

**平滑性检测**：
- ✅ 最大跳变 < 0.1m (位置) → 平滑
- ⚠️ 最大跳变 > 0.5m → 可能有问题

**跟踪精度**：
- ✅ 位置误差 < 0.1m → 优秀
- ✅ 速度误差 < 0.2m/s → 优秀
- ✅ Yaw误差 < 10° → 合格

---

## 3. 常见问题

### Q: 日志文件太大怎么办？
A: 日志按 `std::ios::trunc` 模式打开，每次启动会清空。如需保留历史数据，运行前备份：
```bash
cp ~/experiment/STAR-Searcher/src/ROG-Map/rog_map/log/trajectory_log.csv trajectory_log_backup_$(date +%Y%m%d_%H%M%S).csv
```

### Q: 如何只分析某段时间？
A: 修改 `analyze_trajectory.py`，在 `load_trajectory_log` 后添加：
```python
df = df[(df['timestamp'] >= start_time) & (df['timestamp'] <= end_time)]
```

### Q: 如何导出数据到 MATLAB/Excel？
A: CSV 文件可直接导入：
- MATLAB: `data = readtable('trajectory_log.csv');`
- Excel: 直接打开 CSV 文件

---

## 4. 代码集成细节

### 在 `init()` 中
```cpp
// 创建日志目录
system(("mkdir -p " + log_dir).c_str());

// 打开 CSV 文件并写入头部
traj_log_file_.open(log_file, std::ios::out | std::ios::trunc);
traj_log_file_ << "timestamp,cmd_x,cmd_y,...\n";
```

### 在 `publishCmd()` 中（每10ms调用）
```cpp
// 获取里程计反馈
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_p = odom_pos_;
    odom_v = odom_vel_;
}

// 写入CSV
traj_log_file_ << timestamp << "," << cmd_x << "," << odom_x << ...;
```

---

## 5. 性能影响

- **文件I/O频率**: 100Hz (与控制频率相同)
- **单条日志大小**: ~200 bytes
- **1小时数据量**: ~70MB
- **CPU开销**: < 0.1% (主要是格式化字符串)

---

## 附录：Python 依赖

```bash
pip3 install numpy pandas matplotlib
```

如果提示缺少依赖，运行：
```bash
sudo apt-get install python3-matplotlib python3-pandas
```
