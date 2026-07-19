gnome-terminal -- bash -c "cd /home/sama/zhiquan/ArduPilot_QuadTilt/Tools/autotest/ && sim_vehicle.py -v ArduCopter -f gazebo-iris   --console  --map"
gnome-terminal -- bash -c "cd /home/sama/appim && ./QGroundControl.AppImage"
gnome-terminal -- bash -c "gazebo --verbose quadtilt.world"