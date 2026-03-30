# pwd
# ros2 run car_decision car_decision
# ros2 run local_path_planning local_path_planning
# ros2 run pid pid
# ros2 run car_control car_control
# ros2 launch launch/launch_tracker.py

# pwd

# ros2 launch launch/launch_rviz.py





{
	gnome-terminal -t "parking_planning" -x bash -c "source install/setup.bash; ros2 run parking_planning parking_planning;exec bash" 
}

sleep 1s
{
	gnome-terminal -t "park_control" -x bash -c "source install/setup.bash; ros2 run park_control park_control;exec bash" 
}

sleep 1s
{
	gnome-terminal -t "car_control" -x bash -c "source install/setup.bash; ros2 run car_control car_control;exec bash" 
}
