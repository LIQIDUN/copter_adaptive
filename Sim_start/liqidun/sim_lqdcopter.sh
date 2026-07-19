gnome-terminal -- bash -c "cd /home/sama/zhiquan/copter_adaptive/ && sim_vehicle.py -v ArduCopter -f gazebo-iris   --console  --map"
gnome-terminal -- bash -c "cd /home/sama/appim && ./QGroundControl.AppImage"
gnome-terminal -- bash -c "gazebo --verbose lqd.world"
