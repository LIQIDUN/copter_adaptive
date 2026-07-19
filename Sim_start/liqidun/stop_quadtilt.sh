#!/bin/bash

echo "正在关闭所有 UAV 仿真组件..."

# 1. 杀死 tmux 会话（这会关闭所有在 tmux 窗格中运行的终端任务）
tmux kill-session -t uav_sim 2>/dev/null
echo "已关闭 tmux 会话。"

# 2. 强行杀死 Gazebo 相关进程 (gzserver 和 gzclient)
# 有时候只关窗口 gzserver 还会残留在后台占用 CPU 和端口
pkill -9 gzserver
pkill -9 gzclient
pkill -9 gazebo
echo "已清理 Gazebo 进程。"

# 3. 杀死 ArduPilot SITL 相关进程
pkill -9 -f "sim_vehicle.py"
pkill -9 -f "arducopter"
pkill -9 -f "mavproxy.py"
echo "已清理 ArduPilot SITL 进程。"

# 4. 杀死 ROS 相关进程 (roslaunch, roscore 等)
pkill -9 -f "roslaunch"
pkill -9 -f "rosmaster"
pkill -9 -f "roscore"
pkill -9 -f "rviz"
# 杀死你在 launch 中启动的具体节点关键字（保险起见）
pkill -9 -f "overactuated_driver"
pkill -9 -f "trajectory_planner"
echo "已清理 ROS 进程。"

# 5. 杀死 QGroundControl
pkill -9 -f "QGroundControl.AppImage"
echo "已清理 QGroundControl。"

pkill -f "terminator -u"

echo "======================================"
echo "仿真环境已彻底关闭！"
echo "======================================"