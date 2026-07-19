gnome-terminal -- bash -c "cd /home/liqidun/github/ArduPilot_QuadTilt/Tools/autotest/ && sim_vehicle.py -v ArduCopter -f gazebo-iris   --console  --map"
gnome-terminal -- bash -c "cd /home/liqidun/appimage && ./QGroundControl.AppImage"
gnome-terminal -- bash -c "gazebo --verbose quadtilt.world"