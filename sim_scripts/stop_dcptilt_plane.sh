#!/bin/bash

# ============================================================
# DCPTilt ArduPlane 4.5.7 + Gazebo Classic 关闭脚本
# 参考用户现有 stop_quadtilt.sh，只清理本项目相关进程
# ============================================================

echo "正在关闭 DCPTilt ArduPlane 仿真组件..."

# 1. 清理 Gazebo Classic
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 gazebo 2>/dev/null
echo "已清理 Gazebo 进程。"

# 2. 清理 ArduPilot SITL / MAVProxy
pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "arduplane" 2>/dev/null
pkill -9 -f "mavproxy.py" 2>/dev/null
pkill -9 -f "MAVProxy" 2>/dev/null
echo "已清理 ArduPlane SITL / MAVProxy。"

# 3. 关闭 QGroundControl
pkill -9 -f "QGroundControl.AppImage" 2>/dev/null
echo "已清理 QGroundControl。"

# 4. 关闭由本启动脚本创建的 Terminator 窗口
pkill -f "DCPTilt ArduPlane SITL" 2>/dev/null
pkill -f "DCPTilt Gazebo" 2>/dev/null

echo "======================================"
echo "DCPTilt 仿真环境已关闭。"
echo "======================================"
