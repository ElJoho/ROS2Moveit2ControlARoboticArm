# Clase 12

PASOS:
    1. Añadir un link con todos los orignes con 0
    2. Se añade un joint con todos los origenes con 0
    3. Se arregla el joint
    4. Setear el tipo de joint y el eje
    5. Arreglar el visual origin si es necesario

<FILES>
</FILES>
<TERMINALS>
TERMINAL
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch urdf_tutorial display.launch.py model:=/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/arm.urdf
[INFO] [launch]: All log files can be found below /home/joho/.ros/log/2025-10-28-17-52-42-141296-joho-X550DP-42988
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [42990]
[INFO] [joint_state_publisher_gui-2]: process started with pid [42992]
[INFO] [rviz2-3]: process started with pid [42994]
[robot_state_publisher-1] [INFO] [1761691962.719445985] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1761691962.719657683] [robot_state_publisher]: got segment shoulder_link
[rviz2-3] [INFO] [1761691963.633669415] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1761691963.633822932] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-3] [INFO] [1761691963.692153121] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_gui-2] [INFO] [1761691964.004317195] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1761691964.038708963] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1761691964.276711012] [joint_state_publisher]: Centering
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[robot_state_publisher-1] [INFO] [1761691973.155805982] [rclcpp]: signal_handler(signum=2)
[rviz2-3] [INFO] [1761691973.155462769] [rclcpp]: signal_handler(signum=2)
[ERROR] [joint_state_publisher_gui-2]: process has died [pid 42992, exit code -2, cmd '/opt/ros/humble/lib/joint_state_publisher_gui/joint_state_publisher_gui --ros-args'].
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 42990]
[INFO] [rviz2-3]: process has finished cleanly [pid 42994]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 

</TERMINALS>