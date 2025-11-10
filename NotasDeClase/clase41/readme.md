# Clase 41
/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_interfaces/package.xml
/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_interfaces/CMakeLists.txt

/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_interfaces/msg/PoseCommand.msg



TERMINAL1
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch my_robot_bringup my_robot.launch.xml


TERMINAL2
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 run my_robot_commander_cpp commander


TERMINAL3
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic list
/arm_controller/controller_state
/arm_controller/joint_trajectory
/arm_controller/transition_event
/attached_collision_object
/clicked_point
/collision_object
/controller_manager/activity
/controller_manager/introspection_data/full
/controller_manager/introspection_data/names
/controller_manager/introspection_data/values
/controller_manager/statistics/full
/controller_manager/statistics/names
/controller_manager/statistics/values
/diagnostics
/display_contacts
/display_planned_path
/dynamic_joint_states
/goal_pose
/gripper_controller/controller_state
/gripper_controller/joint_trajectory
/gripper_controller/transition_event
/initialpose
/joint_command
/joint_state_broadcaster/transition_event
/joint_states
/monitored_planning_scene
/open_gripper
/parameter_events
/pipeline_state
/planning_scene
/planning_scene_world
/pose_command
/recognized_object_array
/robot_description
/robot_description_semantic
/rosout
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/tf
/tf_static
/trajectory_execution_event
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic info /pose_command 
Type: my_robot_interfaces/msg/PoseCommand
Publisher count: 0
Subscription count: 1
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 
ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{x:0.7,y:0.0,z:0.4,roll:3.1416,pitch:0.0,yaw:0.0,cartesian_path:false}"
Failed to populate field: 'PoseCommand' object has no attribute 'x:0.7'
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 
ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{ x: 0.7, y: 0.0, z: 0.4, roll: 3.1416, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
Waiting for at least 1 matching subscription(s)...
publisher: beginning loop
publishing #1: my_robot_interfaces.msg.PoseCommand(x=0.7, y=0.0, z=0.4, roll=3.1416, pitch=0.0, yaw=0.0, cartesian_path=False)

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 
ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{ x: 0.7, y: 0.0, z: 0.2, roll: 3.1416, pitch: 0.0, yaw: 0.0, cartesian_path: true}"
publisher: beginning loop
publishing #1: my_robot_interfaces.msg.PoseCommand(x=0.7, y=0.0, z=0.2, roll=3.1416, pitch=0.0, yaw=0.0, cartesian_path=True)

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{ x: 0.7, y: 0.3, z: 0.2, roll: 3.1416, pitch: 0.0, yaw: 0.0, cartesian_path: true}"
publisher: beginning loop
publishing #1: my_robot_interfaces.msg.PoseCommand(x=0.7, y=0.3, z=0.2, roll=3.1416, pitch=0.0, yaw=0.0, cartesian_path=True)

