# Clase 23 
/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/gripper.xacro


TERMINAL
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch urdf_tutorial display.launch.py model:=/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/gripper.xacro
[INFO] [launch]: All log files can be found below /home/joho/.ros/log/2025-11-01-13-40-03-701565-joho-X550DP-42319
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [42329]
[INFO] [joint_state_publisher_gui-2]: process started with pid [42331]
[INFO] [rviz2-3]: process started with pid [42333]
[robot_state_publisher-1] [INFO] [1762022404.250516834] [robot_state_publisher]: got segment gripper_base_link
[robot_state_publisher-1] [INFO] [1762022404.251795793] [robot_state_publisher]: got segment gripper_left_finger_link
[robot_state_publisher-1] [INFO] [1762022404.251851039] [robot_state_publisher]: got segment gripper_right_finger_link
[rviz2-3] [INFO] [1762022405.101413091] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1762022405.102100482] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-3] [INFO] [1762022405.154723112] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_gui-2] [INFO] [1762022405.431968374] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1762022405.470798678] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1762022405.736231654] [joint_state_publisher]: Centering
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[rviz2-3] [INFO] [1762022436.040084518] [rclcpp]: signal_handler(signum=2)
[robot_state_publisher-1] [INFO] [1762022436.040172519] [rclcpp]: signal_handler(signum=2)
[ERROR] [joint_state_publisher_gui-2]: process has died [pid 42331, exit code -2, cmd '/opt/ros/humble/lib/joint_state_publisher_gui/joint_state_publisher_gui --ros-args'].
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 42329]
[INFO] [rviz2-3]: process has finished cleanly [pid 42333]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 





