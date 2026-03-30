
{
	gnome-terminal -t "lidar_driver" -x bash -c "source install/setup.bash; ros2 launch rslidar_sdk start.py; exec bash" 
}
sleep 1s
{
	gnome-terminal -t "lidar_obstacle" -x bash -c "source install/setup.bash; ros2 run lidar_obstacle lidar_obstacle;exec bash" 
}

sleep 1s
{
	gnome-terminal -t "left_lidar_obstacle" -x bash -c "source install/setup.bash; ros2 run left_lidar_obstacle left_lidar_obstacle;exec bash" 
}

sleep 1s
{
	gnome-terminal -t "right_lidar_obstacle" -x bash -c "source install/setup.bash; ros2 run right_lidar_obstacle right_lidar_obstacle;exec bash" 
}

sleep 1s
{
	gnome-terminal -t "radar_obstacle" -x bash -c "source install/setup.bash; ros2 run radar_obstacle radar_obstacle;exec bash"
}

