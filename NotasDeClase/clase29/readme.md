# Clase 29

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro)"

ros2 run controller_manager ros2_control_node --ros-args --params-file /home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_bringup/config/ros2_controllers.yaml

ros2 run controller_manager spawner joint_state_broadcaster

