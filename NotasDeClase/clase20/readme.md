# Clase 20

TERMINAL
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.21s]                  
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [2.17s]                  

Summary: 2 packages finished [3.26s]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch my_robot_moveit_config demo.launch.py 