#!/bin/bash

# ==========================================
# Gazebo仿真一键启动脚本 (修复版)
# ==========================================

# 定义颜色
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 1. 显式加载环境变量（直接从您的 .bashrc 复制，确保非交互模式下也能生效）

# ROS 基础环境
source /opt/ros/noetic/setup.bash
source /home/whd/experiment/gazebo_simulation/devel/setup.bash

# CUDA (如果需要)
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# PX4 Firmware 环境配置
source /home/whd/PX4_Firmware/Tools/setup_gazebo.bash /home/whd/PX4_Firmware /home/whd/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/whd/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/whd/PX4_Firmware/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH

# Livox 插件路径
export GAZEBO_PLUGIN_PATH=/home/whd/experiment/gazebo_simulation/devel/lib:$GAZEBO_PLUGIN_PATH

# Anaconda/Python 环境 (以防 python3 需要)
export PATH=/home/whd/anaconda3/bin:$PATH

# 定义清理函数
cleanup() {
    echo -e "\n${RED}[Script] Caught Signal, FORCE CLEANING UP...${NC}"
    
    # 1. 杀死当前脚本启动的所有后台 PID
    # jobs -p 能获取当前 shell 启动的所有后台进程 PID
    JOBS=$(jobs -p)
    if [ -n "$JOBS" ]; then
        echo "Killing background jobs: $JOBS"
        kill -9 $JOBS 2>/dev/null
    fi
    
    # 2. 强杀关键进程（防止子进程残留）
    # 注意：这里移除了 python3 的全杀，防止误伤系统其他进程，只杀特定的 ros 节点
    killall -9 gzserver gzclient px4 roslaunch rosmaster rviz 2>/dev/null
    pkill -9 -f "get_simulation_pose.py"
    
    # 3. 清理 YOLO 推理进程（防止GPU内存泄漏）
    pkill -9 -f "yolo_inference_node"
    
    echo -e "${GREEN}[Script] Cleanup finished.${NC}"
    exit 0
}

# 捕获 Ctrl+C (SIGINT) 和 脚本退出 (EXIT)
trap cleanup SIGINT SIGTERM EXIT

echo -e "${GREEN}[Script] Starting Simulation Workflow...${NC}"

# Headless by default for profiling/long runs. Override with SIM_GUI=true when needed.
SIM_GUI=${SIM_GUI:-false}

# PX4 可执行文件和 Gazebo MAVLink 插件缺失时，PX4 会一直等待 simulator/FCU，
# 但其余 ROS 节点仍会继续启动。启动前显式检查，避免出现难以识别的半启动状态。
PX4_BUILD_DIR=/home/whd/PX4_Firmware/build/px4_sitl_default
PX4_BINARY="$PX4_BUILD_DIR/bin/px4"
MAVLINK_PLUGIN="$PX4_BUILD_DIR/build_gazebo/libgazebo_mavlink_interface.so"

if [ ! -x "$PX4_BINARY" ] || [ ! -f "$MAVLINK_PLUGIN" ]; then
    echo -e "${RED}[Script] PX4 SITL build is incomplete.${NC}"
    echo "Missing PX4 binary or Gazebo MAVLink plugin under: $PX4_BUILD_DIR"
    echo "Build it with: cd /home/whd/PX4_Firmware && make px4_sitl_default"
    exit 1
fi

# 1. 启动 PX4 Gazebo 仿真
echo -e "${BLUE}[1/6] Launching PX4 Gazebo Simulation...${NC}"
# 直接运行，利用之前 source 的环境变量
roslaunch px4 outdoor3.launch gui:="$SIM_GUI" &
sleep 10  # 给 Gazebo 更多启动时间

# 2. 将 iris_0 命名空间下的 Gazebo 传感器话题桥接到感知模块使用的话题名
echo -e "${BLUE}[2/6] Bridging Gazebo sensor topics...${NC}"
rosrun topic_tools relay /iris_0/velodyne_points /velodyne_points __name:=sim_lidar_relay &
rosrun topic_tools relay /iris_0/realsense/depth_camera/color/image_raw /realsense/depth_camera/color/image_raw __name:=sim_camera_relay &
sleep 1

# 3. 启动 Simulation Pose Relay (Python)
echo -e "${BLUE}[3/6] Starting Simulation Pose Relay...${NC}"
# 使用系统自带 python3 以确保能找到 rospkg
(cd ~/experiment/fast-livo2/src/fast-livo2-target/simulation && /usr/bin/python3 get_simulation_pose.py) &
sleep 2

# 4. 启动 Fast-Livo Mapping (感知模块)
echo -e "${BLUE}[4/6] Launching Fast-Livo Mapping...${NC}"
(cd ~/experiment/fast-livo2 && source devel/setup.bash && roslaunch fast_livo mapping_mid360_simulation.launch) &
sleep 5

# 5. 启动 YOLO 目标检测推理
echo -e "${BLUE}[5/6] Launching YOLO Inference...${NC}"
(cd ~/experiment/yolo_inference && source devel/setup.bash && roslaunch yolo_segmentation yolo_inference.launch) &
sleep 3

# 6. 启动 Offboard Controller
echo -e "${BLUE}[6/6] Starting Offboard Control Node...${NC}"
(source ~/experiment/STAR-Searcher/devel/setup.bash && rosrun my_control_package offboard_node) &

echo -e "${GREEN}[Script] All modules started. Press Ctrl+C to stop.${NC}"

# 永久等待，直到收到信号
wait
