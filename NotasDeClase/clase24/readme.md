# Clase 24

/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/gripper.xacro
/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro

TERMINAL
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.22s]                  
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [0.20s]                  

Summary: 2 packages finished [1.28s]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch my_robot_description display.launch.xml 
[INFO] [launch]: All log files can be found below /home/joho/.ros/log/2025-11-01-13-52-24-370415-joho-X550DP-46760
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [46762]
[INFO] [joint_state_publisher_gui-2]: process started with pid [46764]
[INFO] [rviz2-3]: process started with pid [46766]
[robot_state_publisher-1] [INFO] [1762023145.058842861] [robot_state_publisher]: got segment arm_link
[robot_state_publisher-1] [INFO] [1762023145.059066781] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1762023145.059107850] [robot_state_publisher]: got segment elbow_link
[robot_state_publisher-1] [INFO] [1762023145.059141585] [robot_state_publisher]: got segment forearm_link
[robot_state_publisher-1] [INFO] [1762023145.059175319] [robot_state_publisher]: got segment gripper_base_link
[robot_state_publisher-1] [INFO] [1762023145.059207587] [robot_state_publisher]: got segment gripper_left_finger_link
[robot_state_publisher-1] [INFO] [1762023145.059242300] [robot_state_publisher]: got segment gripper_right_finger_link
[robot_state_publisher-1] [INFO] [1762023145.059274568] [robot_state_publisher]: got segment hand_link
[robot_state_publisher-1] [INFO] [1762023145.059304880] [robot_state_publisher]: got segment shoulder_link
[robot_state_publisher-1] [INFO] [1762023145.059336659] [robot_state_publisher]: got segment tool_link
[robot_state_publisher-1] [INFO] [1762023145.059368927] [robot_state_publisher]: got segment wrist_link
[rviz2-3] [INFO] [1762023145.790943001] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1762023145.791112652] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-3] [INFO] [1762023145.841580771] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_gui-2] [INFO] [1762023146.070168697] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1762023146.086945608] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1762023146.330534719] [joint_state_publisher]: Centering


