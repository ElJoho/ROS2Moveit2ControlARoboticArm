# Clase 13

Terminal1
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch urdf_tutorial display.launch.py model:=/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/arm.urdf
[INFO] [launch]: All log files can be found below /home/joho/.ros/log/2025-10-29-11-13-51-695586-joho-X550DP-82383
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [82386]
[INFO] [joint_state_publisher_gui-2]: process started with pid [82388]
[INFO] [rviz2-3]: process started with pid [82390]
[robot_state_publisher-1] [INFO] [1761754432.191252718] [robot_state_publisher]: got segment arm_link
[robot_state_publisher-1] [INFO] [1761754432.191459026] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1761754432.191511825] [robot_state_publisher]: got segment elbow_link
[robot_state_publisher-1] [INFO] [1761754432.191544580] [robot_state_publisher]: got segment forearm_link
[robot_state_publisher-1] [INFO] [1761754432.191576847] [robot_state_publisher]: got segment hand_link
[robot_state_publisher-1] [INFO] [1761754432.191608624] [robot_state_publisher]: got segment hand_tool_link
[robot_state_publisher-1] [INFO] [1761754432.191639912] [robot_state_publisher]: got segment shoulder_link
[robot_state_publisher-1] [INFO] [1761754432.191670223] [robot_state_publisher]: got segment wrist_link
[rviz2-3] [INFO] [1761754432.918773166] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1761754432.918951608] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-3] [INFO] [1761754432.971534253] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_gui-2] [INFO] [1761754433.236389846] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1761754433.263961310] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1761754433.515079332] [joint_state_publisher]: Centering



TERMINAL2
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 run tf2_tools view_frames 
[INFO] [1761754463.465782486] [view_frames]: Listening to tf data for 5.0 seconds...
[INFO] [1761754468.552782451] [view_frames]: Generating graph in frames.pdf file...
[INFO] [1761754468.558541508] [view_frames]: Result:tf2_msgs.srv.FrameGraph_Response(frame_yaml="shoulder_link: \n  parent: 'base_link'\n  broadcaster: 'default_authority'\n  rate: 10.195\n  most_recent_transform: 1761754468.546397\n  oldest_transform: 1761754463.446080\n  buffer_length: 5.100\narm_link: \n  parent: 'shoulder_link'\n  broadcaster: 'default_authority'\n  rate: 10.195\n  most_recent_transform: 1761754468.546397\n  oldest_transform: 1761754463.446080\n  buffer_length: 5.100\nelbow_link: \n  parent: 'arm_link'\n  broadcaster: 'default_authority'\n  rate: 10.195\n  most_recent_transform: 1761754468.546397\n  oldest_transform: 1761754463.446080\n  buffer_length: 5.100\nforearm_link: \n  parent: 'elbow_link'\n  broadcaster: 'default_authority'\n  rate: 10.195\n  most_recent_transform: 1761754468.546397\n  oldest_transform: 1761754463.446080\n  buffer_length: 5.100\nwrist_link: \n  parent: 'forearm_link'\n  broadcaster: 'default_authority'\n  rate: 10.195\n  most_recent_transform: 1761754468.546397\n  oldest_transform: 1761754463.446080\n  buffer_length: 5.100\nhand_link: \n  parent: 'wrist_link'\n  broadcaster: 'default_authority'\n  rate: 10.195\n  most_recent_transform: 1761754468.546397\n  oldest_transform: 1761754463.446080\n  buffer_length: 5.100\nhand_tool_link: \n  parent: 'hand_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\n")
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 
