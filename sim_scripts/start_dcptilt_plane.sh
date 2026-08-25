#!/bin/bash

# ============================================================
# DCPTilt ArduPlane 4.5.7 + Gazebo Classic 启动脚本
# 参考用户现有 start_quadtilt.sh 的目录结构与 Terminator 启动方式
# ============================================================

# --- 1. 环境与路径配置 ---

ARDUPILOT_DIR="/home/liqidun/github/copter_adaptive"
AUTOTEST_DIR="$ARDUPILOT_DIR/Tools/autotest"

ARDUPILOT_GAZEBO_DIR="/home/liqidun/github/ardupilot_gazebo"
WORLD_DIR="$ARDUPILOT_GAZEBO_DIR/worlds"
WORLD_FILE="quadtilt_plane.world"

QGC_DIR="/home/liqidun/appimage"
QGC_APP="$QGC_DIR/QGroundControl.AppImage"

# Gazebo 模型搜索路径：
# 1) legacy ardupilot_gazebo/models
# 2) 当前 ArduPilot 4.5.7 仓库 Tools/autotest/models
GAZEBO_MODELS="export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_DIR/Tools/autotest/models"

echo "======================================"
echo "DCPTilt ArduPlane 4.5.7 SITL"
echo "======================================"
echo "ArduPilot : $ARDUPILOT_DIR"
echo "World     : $WORLD_DIR/$WORLD_FILE"
echo

# --- 2. 启动前检查 ---

if [ ! -f "$AUTOTEST_DIR/sim_vehicle.py" ]; then
    echo "[ERROR] 找不到 sim_vehicle.py:"
    echo "        $AUTOTEST_DIR/sim_vehicle.py"
    exit 1
fi

if [ ! -f "$WORLD_DIR/$WORLD_FILE" ]; then
    echo "[ERROR] 找不到 Gazebo world:"
    echo "        $WORLD_DIR/$WORLD_FILE"
    exit 1
fi

if ! command -v gazebo >/dev/null 2>&1; then
    echo "[ERROR] 找不到 gazebo，请确认 Gazebo Classic 已安装且在 PATH 中。"
    exit 1
fi

if ! command -v terminator >/dev/null 2>&1; then
    echo "[ERROR] 找不到 terminator。"
    exit 1
fi

# --- 3. 清理旧仿真环境 ---

echo "正在清理旧进程 (ArduPlane SITL / Gazebo / MAVProxy)..."

pkill -9 -f "arduplane" 2>/dev/null
pkill -9 -f "arducopter" 2>/dev/null
pkill -9 -f "mavproxy" 2>/dev/null
pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "gazebo" 2>/dev/null
pkill -9 -f "gzserver" 2>/dev/null
pkill -9 -f "gzclient" 2>/dev/null

sleep 1

# --- 4. 使用 Terminator 分窗口启动 ---

# [窗口 1] ArduPlane SITL + MAVProxy
#
# Plane 使用 legacy ardupilot_gazebo 的 gazebo-zephyr frame。
# 不使用 master 分支脚本写法，直接从当前 Plane-4.5.7 仓库运行。
#
terminator -u --geometry=800x400+0+250 -T "DCPTilt ArduPlane SITL" -x bash -c \
"cd $AUTOTEST_DIR && python3 sim_vehicle.py -v ArduPlane -f gazebo-zephyr --console --map; exec bash" &

sleep 3

# [窗口 2] Gazebo Classic
terminator -u --geometry=700x450+850+0 -T "DCPTilt Gazebo" -x bash -c \
"$GAZEBO_MODELS && cd $WORLD_DIR && gazebo --verbose $WORLD_FILE; exec bash" &

sleep 2

# [窗口 3] QGroundControl
if [ -x "$QGC_APP" ]; then
    terminator -u --geometry=500x300+0+700 -T "QGC" -x bash -c \
    "cd $QGC_DIR && ./QGroundControl.AppImage; exec bash" &
else
    echo "[WARN] 未找到可执行的 QGroundControl.AppImage:"
    echo "       $QGC_APP"
    echo "       SITL 和 Gazebo 已继续启动。"
fi

echo
echo "---------------------------------------"
echo "DCPTilt 仿真组件已启动。"
echo
echo "SITL:"
echo "  ArduPlane / gazebo-zephyr"
echo
echo "Gazebo:"
echo "  $WORLD_DIR/$WORLD_FILE"
echo
echo "如果某个 Terminator 窗口报错，该窗口会保留，"
echo "可直接查看具体错误，不会执行完立即闪退。"
echo "---------------------------------------"
