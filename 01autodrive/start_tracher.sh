# pwd
# ros2 run car_decision car_decision
# ros2 run local_path_planning local_path_planning
# ros2 run pid pid
# ros2 run car_control car_control
# ros2 launch launch/launch_tracker.py

# pwd

# ros2 launch launch/launch_rviz.py

gnome-terminal -t "car_decision" -x bash -c "ros2 run car_decision car_decision"

sleep 5
gnome-terminal -t "local_path_planning" -x bash -c "ros2 run local_path_planning local_path_planning"

sleep 5

gnome-terminal -t "pid" -x bash -c "ros2 run pid pid"
sleep 5


gnome-terminal -t "car_control" -x bash -c "ros2 run car_control car_control"