# Clase 39 - 40 

PRIMERA PARTE
---------------------
TERMINAL1
colcon build
. install/setup.bash
ros2 launch my_robot_bringup my_robot.launch.xml 
TERMINAL2
. install/setup.bash
ros2 run my_robot_commander_cpp commander 

TERMINAL3
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
/recognized_object_array
/robot_description
/robot_description_semantic
/rosout
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/tf
/tf_static
/trajectory_execution_event
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic info /joint_command
Type: example_interfaces/msg/Float64MultiArray
Publisher count: 0
Subscription count: 1
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5,0.5]}"
publisher: beginning loop
publishing #1: example_interfaces.msg.Float64MultiArray(layout=example_interfaces.msg.MultiArrayLayout(dim=[], data_offset=0), data=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5,0.5]}"
