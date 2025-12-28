# EGO-Planner & Falcon-Planner 文档目录

## 文档列表

### 1. [EGO_Planner_Technical_Documentation.md](./EGO_Planner_Technical_Documentation.md)
**EGO-Planner 完整技术文档**（1295行，39KB）

详细介绍 EGO-Planner 的设计原理、实现细节和使用方法，专注于**单机导航和探索支持**。

⚠️ **注意**：该文档中关于 `planGlobalTrajWaypoints` 的描述有误，实际实现更简单（无 A* 搜索）。请参阅 [执行流程分析文档](./EGO_Planner_Execution_Flow_Analysis.md) 获取准确的源码级分析。

**核心内容**：
- MINCO 轨迹优化原理
- 完整状态机设计（8个状态）
- 三层避障策略
- 位置+速度+yaw 完整规划
- PX4 集成与自动起飞

### 2. [EGO_Planner_Execution_Flow_Analysis.md](./EGO_Planner_Execution_Flow_Analysis.md) ⭐ **推荐**
**EGO-Planner 运行流程详解**（约2000行）

🔍 **源码级完整分析**：从接收目标点到抵达目标的全流程剖析，包含所有源码细节。

**核心内容**：
- **源码勘误**：纠正技术文档中的错误描述
- **两级轨迹系统**：全局轨迹 vs 局部轨迹的详细说明
- **完整执行流程**：每个步骤的源码和时序分析
- **planGlobalTrajWaypoints 真相**：为什么 waypoints.size()==1 不打印错误
- **重规划机制**：首次规划、定期重规划、到达目标的完整流程
- **数据流图和时序图**：可视化整个执行过程

### 3. [Falcon_Planner_Exploration_FSM.md](./Falcon_Planner_Exploration_FSM.md)
**Falcon-Planner 探索状态机文档**（1398行，39KB）

深入分析 Falcon-Planner 的探索状态机设计，专注于**自主探索任务**。

**核心内容**：
- 探索状态机设计（7个状态）
- 边界检测与管理
- 动态重规划机制（3种触发条件）
- 多线程回调系统
- 丰富的可视化系统

---

## 快速对比

| 特性 | EGO-Planner | Falcon-Planner |
|------|-------------|----------------|
| **主要用途** | 目标点导航 | 自主探索 |
| **轨迹优化** | MINCO | B-spline |
| **状态数量** | 8个 | 7个 |
| **目标生成** | 外部指定 | 自动检测边界 |
| **重规划条件** | 2种 | 3种（含边界覆盖） |
| **PX4集成** | ✅ 完整 | ❌ 无需 |
| **探索支持** | 接口支持 | 完整系统 |
| **适用场景** | 已知目标导航、视点探索 | 未知环境自主探索 |

---

## EGO-Planner 快速导航

### 📖 推荐阅读顺序

1. **快速了解** → [技术文档 概述部分](./EGO_Planner_Technical_Documentation.md#1-概述)
2. **深入理解** → [执行流程分析](./EGO_Planner_Execution_Flow_Analysis.md) ⭐
3. **参数调优** → [技术文档 参数配置](./EGO_Planner_Technical_Documentation.md#7-参数配置)
4. **问题排查** → [技术文档 常见问题](./EGO_Planner_Technical_Documentation.md#11-常见问题)

### 源码级分析（执行流程文档）

- [关键发现：planGlobalTrajWaypoints 真相](./EGO_Planner_Execution_Flow_Analysis.md#1-文档说明与源码勘误)
  - 为什么不打印 ROS_ERROR
  - 单路径点处理细节
  - 与文档描述的差异

- [两级轨迹系统](./EGO_Planner_Execution_Flow_Analysis.md#2-核心概念)
  - 全局轨迹 vs 局部轨迹
  - 规划视野原理
  - 关键标志位说明

- [完整执行流程](./EGO_Planner_Execution_Flow_Analysis.md#3-接收目标点后的完整流程)
  - 接收目标点 → 生成全局轨迹 → 局部规划 → 执行 → 重规划
  - 每个步骤的源码分析
  - 完整时序图

- [重规划机制](./EGO_Planner_Execution_Flow_Analysis.md#6-实时重规划机制)
  - planFromGlobalTraj vs planFromLocalTraj
  - 定期重规划触发条件
  - 连续性保证

### 理论基础（技术文档）

- [MINCO 轨迹优化原理](./EGO_Planner_Technical_Documentation.md#2-minco-轨迹优化原理)
  - 数学模型与公式
  - 优化目标与约束
  - L-BFGS 求解

### 系统设计
- [状态机设计](./EGO_Planner_Technical_Documentation.md#3-状态机设计)
  - 8 个状态详解：INIT → PRE_ARM → TAKEOFF → WAIT_TARGET → GEN_NEW_TRAJ → REPLAN_TRAJ → EXEC_TRAJ → EMERGENCY_STOP
  - 状态转换逻辑
  - PX4 集成（解锁、模式切换）

- [避障实现](./EGO_Planner_Technical_Documentation.md#4-避障实现)
  - 三层避障：A* + MINCO优化 + 实时监控
  - Inflated Map 原理
  - 硬约束 vs 软约束

- [期望状态规划](./EGO_Planner_Technical_Documentation.md#6-期望状态规划)
  - **探索支持**：位置 + yaw + 速度完整规划
  - 两种输入接口（RViz Goal / 探索视点）
  - Yaw 插值策略

### 实用指南
- [参数配置](./EGO_Planner_Technical_Documentation.md#7-参数配置)
- [使用方法](./EGO_Planner_Technical_Documentation.md#8-使用方法)
- [常见问题](./EGO_Planner_Technical_Documentation.md#11-常见问题)

---

## Falcon-Planner 快速导航

### 核心设计
- [状态定义](./Falcon_Planner_Exploration_FSM.md#2-状态定义)
  - 7 个状态：INIT → WAIT_TRIGGER → PLAN_TRAJ → PUB_TRAJ → EXEC_TRAJ → FINISH → RTB

- [状态转换图](./Falcon_Planner_Exploration_FSM.md#3-状态转换图)
  - 完整状态转换流程
  - 转换条件总览表

- [状态详解](./Falcon_Planner_Exploration_FSM.md#4-状态详解)
  - WAIT_TRIGGER：边界更新 + 手动/自动触发
  - PLAN_TRAJ：静态/动态起始状态 + 探索规划
  - PUB_TRAJ：碰撞检查 + 轨迹延长
  - EXEC_TRAJ：三种重规划条件

### 关键机制
- [重规划机制](./Falcon_Planner_Exploration_FSM.md#5-重规划机制)
  - **3种触发条件**：边界簇覆盖、轨迹结束、周期性
  - 动态重规划周期（50ms - 200ms）

- [回调系统](./Falcon_Planner_Exploration_FSM.md#6-回调系统)
  - FSMCallback（100Hz）：主状态机
  - safetyCallback（20Hz）：独立安全检查
  - frontierCallback（2Hz）：边界更新

- [安全机制](./Falcon_Planner_Exploration_FSM.md#7-安全机制)
  - 发布前检查
  - 实时安全监控
  - 初始轨迹回退

- [可视化系统](./Falcon_Planner_Exploration_FSM.md#8-可视化系统)
  - 边界可视化（活跃/休眠/微小）
  - 视点可视化（全局/精细化）
  - 轨迹可视化（初始/优化）

### 对比分析
- [与 EGO-Planner 的对比](./Falcon_Planner_Exploration_FSM.md#9-与-ego-planner-的对比)
  - 状态机对比
  - 核心差异
  - 适用场景

---

## 使用建议

### 选择 EGO-Planner 的场景

✅ 导航到**已知目标点**（手动指定或预设路径）  
✅ 需要**精确控制终点速度和yaw**（连续探索）  
✅ 需要**PX4 集成**（自动起飞、模式切换）  
✅ 与**外部探索规划器**配合（接收视点指令）  
✅ 需要**MINCO 轨迹**（更平滑、更优动力学性能）  

**示例**：
```bash
# 手动模式：RViz发送目标
roslaunch rog_map ego_planner.launch flight_type:=1

# 探索模式：接收外部视点
rostopic pub /exploration/viewpoint quadrotor_msgs/PositionCommand ...
```

### 选择 Falcon-Planner 的场景

✅ **完全自主探索**未知环境  
✅ **无需指定目标**（自动检测边界）  
✅ 需要**边界管理**（活跃/休眠边界）  
✅ 需要**层次化网格**空间管理  
✅ 已有成熟的探索框架  

**示例**：
```bash
# 自动启动探索
roslaunch falcon_planner exploration.launch auto_start:=true

# 手动触发探索（RViz 2D Nav Goal）
```

### 混合使用建议

**方案 1：Falcon-Planner 生成视点 + EGO-Planner 执行**
- Falcon 负责边界检测和视点选择
- EGO 负责轨迹生成和执行
- 优势：结合两者优点

**方案 2：根据阶段切换**
- 探索阶段：Falcon-Planner（自主探索）
- 返航阶段：EGO-Planner（导航到基地）

---

## 文档特点

### EGO-Planner 文档
✅ 理论完整（数学公式+算法原理）  
✅ 实现详细（代码片段+实现逻辑）  
✅ 实用性强（配置+使用+调试）  
✅ 探索友好（完整状态规划支持）  

### Falcon-Planner 文档
✅ 状态机详解（7个状态+转换流程）  
✅ 重规划机制（3种条件+动态周期）  
✅ 回调系统（多线程+独立安全）  
✅ 可视化完整（边界+视点+轨迹）  
✅ 对比分析（与EGO对比+场景选择）  

---

## 快速开始

### EGO-Planner
```bash
# 启动规划器（手动模式）
roslaunch rog_map ego_planner.launch flight_type:=1

# 启动规划器（预设路径）
roslaunch rog_map ego_planner.launch flight_type:=2 auto_start:=true

# 发布探索视点
rostopic pub /exploration/viewpoint quadrotor_msgs/PositionCommand "{position: {x: 10.0, y: 5.0, z: 2.0}, yaw: 1.57, velocity: {x: 0.5, y: 0.0, z: 0.0}}"
```

### Falcon-Planner
```bash
# 启动探索（自动）
roslaunch falcon_planner exploration.launch auto_start:=true

# 启动探索（手动触发）
roslaunch falcon_planner exploration.launch auto_start:=false
# 然后在 RViz 中使用 2D Nav Goal 触发
```

---

## 文件结构

```
doc/
├── README.md                                      # 本文件（文档索引）
├── EGO_Planner_Technical_Documentation.md         # EGO-Planner 技术文档（原理+配置+使用）
├── EGO_Planner_Execution_Flow_Analysis.md         # EGO-Planner 执行流程详解（源码级分析）⭐
└── Falcon_Planner_Exploration_FSM.md              # Falcon-Planner 状态机文档（探索FSM）
```

**推荐阅读**：
- 想了解**使用方法和参数配置** → 技术文档
- 想理解**源码执行逻辑** → 执行流程详解 ⭐
- 想学习**探索状态机** → Falcon-Planner 文档

---

**维护者**：STAR-Searcher Team  
**最后更新**：2025-12-26  
**版本**：v1.0

**致谢**：
- EGO-Planner: ZJU FAST-Lab
- Falcon-Planner: HKUST FAST-Lab
