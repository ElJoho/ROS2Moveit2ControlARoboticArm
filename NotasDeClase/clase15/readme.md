# Clase 15

TERMINAL1
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch urdf_tutorial display.launch.py model:=/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro
[INFO] [launch]: All log files can be found below /home/joho/.ros/log/2025-10-29-12-01-17-022593-joho-X550DP-84034
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [84036]
[INFO] [joint_state_publisher_gui-2]: process started with pid [84038]
[INFO] [rviz2-3]: process started with pid [84040]
[robot_state_publisher-1] [INFO] [1761757277.557589024] [robot_state_publisher]: got segment arm_link
[robot_state_publisher-1] [INFO] [1761757277.557781650] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1761757277.557819296] [robot_state_publisher]: got segment elbow_link
[robot_state_publisher-1] [INFO] [1761757277.557851563] [robot_state_publisher]: got segment forearm_link
[robot_state_publisher-1] [INFO] [1761757277.557884808] [robot_state_publisher]: got segment hand_link
[robot_state_publisher-1] [INFO] [1761757277.557915609] [robot_state_publisher]: got segment hand_tool_link
[robot_state_publisher-1] [INFO] [1761757277.557947876] [robot_state_publisher]: got segment shoulder_link
[robot_state_publisher-1] [INFO] [1761757277.557979166] [robot_state_publisher]: got segment wrist_link
[rviz2-3] [INFO] [1761757278.330051267] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1761757278.330233138] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-3] [INFO] [1761757278.384491766] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_gui-2] [INFO] [1761757278.710208782] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1761757278.735528432] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1761757278.981935087] [joint_state_publisher]: Centering
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[rviz2-3] [INFO] [1761757283.711300406] [rclcpp]: signal_handler(signum=2)
[robot_state_publisher-1] [INFO] [1761757283.711588857] [rclcpp]: signal_handler(signum=2)
[ERROR] [joint_state_publisher_gui-2]: process has died [pid 84038, exit code -2, cmd '/opt/ros/humble/lib/joint_state_publisher_gui/joint_state_publisher_gui --ros-args'].
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 84036]
[INFO] [rviz2-3]: process has finished cleanly [pid 84040]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.21s]                  

Summary: 1 package finished [1.05s]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch my_robot_description display.launch.xml
[INFO] [launch]: All log files can be found below /home/joho/.ros/log/2025-10-29-13-09-33-081890-joho-X550DP-87230
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [87232]
[INFO] [joint_state_publisher_gui-2]: process started with pid [87234]
[INFO] [rviz2-3]: process started with pid [87236]
[robot_state_publisher-1] [INFO] [1761761373.739318170] [robot_state_publisher]: got segment arm_link
[robot_state_publisher-1] [INFO] [1761761373.739539638] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1761761373.739581194] [robot_state_publisher]: got segment elbow_link
[robot_state_publisher-1] [INFO] [1761761373.739618349] [robot_state_publisher]: got segment forearm_link
[robot_state_publisher-1] [INFO] [1761761373.739654527] [robot_state_publisher]: got segment hand_link
[robot_state_publisher-1] [INFO] [1761761373.739692172] [robot_state_publisher]: got segment hand_tool_link
[robot_state_publisher-1] [INFO] [1761761373.739728839] [robot_state_publisher]: got segment shoulder_link
[robot_state_publisher-1] [INFO] [1761761373.739764528] [robot_state_publisher]: got segment wrist_link
[rviz2-3] [INFO] [1761761374.438199170] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1761761374.438362460] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-3] [INFO] [1761761374.483991183] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_gui-2] [INFO] [1761761374.731520806] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1761761374.752391580] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1761761374.982861827] [joint_state_publisher]: Centering


TERMINAL2
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 node list
/joint_state_publisher
/robot_state_publisher
/rviz
/transform_listener_impl_61a08f2d8890
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ rqt_graph
^C^C[INFO] [1761757684.090109386] [rclcpp]: signal_handler(signum=2)
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 

