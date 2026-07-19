#!/bin/bash

# --- 1. 环境与路径配置 ---
# 自动设置 ROS 环境变量
# SOURCE_ROS="source /opt/ros/noetic/setup.bash && source ~/ros/ros_quadtilt/devel/setup.bash"

# 自动设置 Gazebo 模型路径 (确保 Gazebo 能找到飞机模型和世界文件)
GAZEBO_MODELS="export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/liqidun/github/ardupilot_gazebo/models:/home/liqidun/github/copter_adaptive/Tools/autotest/models"

# 默认世界文件路径 (根据你之前的报错，这里进入包含 quadtilt.world 的目录)
WORLD_DIR="/home/liqidun/github/ardupilot_gazebo/worlds" # 或者你存放 .world 的实际目录

echo "正在清理旧进程 (SITL, Gazebo, ROS)..."

# --- 2. 强力清理旧环境 ---
pkill -9 -f "ros"
pkill -9 -f "gazebo"
pkill -9 -f "arducopter"
pkill -9 -f "mavproxy"
pkill -9 -f "sim_vehicle.py"

# --- 3. 使用 Terminator 分窗口启动 ---

# [窗口 0] ROS Master: 核心节点管理器
# terminator -u --geometry=500x200+0+0 -T "ROS Master" -x bash -c \
# "$SOURCE_ROS && roscore; exec bash" &
# sleep 2

# [窗口 1] ArduPilot SITL: 仿真固件和 MAVProxy
# 这里增加了 --console --map 参数
terminator -u --geometry=800x400+0+250 -T "ArduPilot SITL" -x bash -c \
"cd /home/liqidun/github/copter_adaptive/Tools/autotest/ && python3 sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map; exec bash" &
sleep 3

# [窗口 2] Gazebo: 物理引擎 (解决找不到 world 的报错)
# 先 cd 到对应目录，再 export 路径，最后启动
terminator -u --geometry=600x400+850+0 -T "Gazebo" -x bash -c \
"$GAZEBO_MODELS && cd $WORLD_DIR && gazebo --verbose quadtilt_plane.world; exec bash" &
# sleep 2

# [窗口 3] QGroundControl: 地面站
terminator -u --geometry=500x300+0+700 -T "QGC" -x bash -c \
"cd /home/liqidun/appimage && ./QGroundControl.AppImage; exec bash" &

# --- 4. ROS 业务节点 (如有) ---

# [窗口 4] ROS 控制节点
# terminator -u --geometry=600x250+850+450 -T "ROS CTRL" -x bash -c \
# "$SOURCE_ROS && roslaunch overactuated_driver run_ctrl_nodes.launch --wait; exec bash" &

# [窗口 5] RViz 可视化
# RVIZ_CONFIG="/home/liqidun/ros/ros_quadtilt/src/trajectory_planner/test/cfg.rviz"
# if [ -f "$RVIZ_CONFIG" ]; then
#     terminator -u --geometry=600x250+1460+700 -T "RViz" -x bash -c \
#     "$SOURCE_ROS && rviz -d $RVIZ_CONFIG; exec bash" &
# fi

echo "---------------------------------------"
echo "所有组件已启动。如有 Gazebo 闪退，请检查模型路径。"
echo "---------------------------------------"