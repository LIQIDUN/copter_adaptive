gnome-terminal -- bash -c "cd /home/sama/zhiquan/copter_adaptive/ && sim_vehicle.py -v ArduCopter -f gazebo-iris   --console"
gnome-terminal -- bash -c "cd /home/sama/appim && ./QGroundControl.AppImage"
#gnome-terminal -- bash -c "gazebo --verbose iris_ardupilot.world"

#gnome-terminal -- bash -c "cd /home/sama/zhiquan/copter_adaptive/ && sim_vehicle.py -v ArduPlane -f quadplane   --console"


gnome-terminal -- bash -c "gazebo --verbose standard_vtol_ardupilot.world"
