# Clase 38


TERMINAL1
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ colcon build
[0.188s] WARNING:colcon.colcon_core.prefix_path.colcon:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install' in the environment variable COLCON_PREFIX_PATH doesn't exist
[0.188s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install/my_robot_bringup' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.188s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install/my_robot_moveit_config' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.188s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install/my_robot_description' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.188s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install/my_robot_commander_cpp' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.188s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install/my_robot_bringup' in the environment variable CMAKE_PREFIX_PATH doesn't exist
[0.188s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install/my_robot_moveit_config' in the environment variable CMAKE_PREFIX_PATH doesn't exist
[0.188s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install/my_robot_description' in the environment variable CMAKE_PREFIX_PATH doesn't exist
[0.188s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/install/my_robot_commander_cpp' in the environment variable CMAKE_PREFIX_PATH doesn't exist
Starting >>> my_robot_description
Starting >>> my_robot_commander_cpp
Finished <<< my_robot_description [0.11s]                                                                    
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [0.08s]
Starting >>> my_robot_bringup
Finished <<< my_robot_bringup [0.07s]                                                                  
Finished <<< my_robot_commander_cpp [14.9s]                       

Summary: 4 packages finished [15.1s]
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch my_robot_bringup my_robot.launch.xml 
[INFO] [launch]: All log files can be found below /home/johotan/.ros/log/2025-11-09-13-31-34-031089-johotan-43018
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [43022]
[INFO] [ros2_control_node-2]: process started with pid [43023]
[INFO] [spawner-3]: process started with pid [43024]
[INFO] [spawner-4]: process started with pid [43025]
[INFO] [spawner-5]: process started with pid [43026]
[INFO] [move_group-6]: process started with pid [43027]
[INFO] [rviz2-7]: process started with pid [43028]
[robot_state_publisher-1] [INFO] [1762713094.579806413] [robot_state_publisher]: Robot initialized
[ros2_control_node-2] [INFO] [1762713094.590693580] [controller_manager]: Using Steady (Monotonic) clock for triggering controller manager cycles.
[ros2_control_node-2] [INFO] [1762713094.596373687] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[ros2_control_node-2] [INFO] [1762713094.609816897] [controller_manager]: update rate is 100 Hz
[ros2_control_node-2] [INFO] [1762713094.610010804] [controller_manager]: Overruns handling is : enabled
[ros2_control_node-2] [INFO] [1762713094.610030882] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [WARN] [1762713094.610318434] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted). See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
[move_group-6] [INFO] [1762713094.657359962] [move_group.moveit.moveit.ros.rdf_loader]: Loaded robot model in 0.00492339 seconds
[move_group-6] [INFO] [1762713094.657514275] [move_group.moveit.moveit.core.robot_model]: Loading robot model 'my_robot'...
[move_group-6] [INFO] [1762713094.662874338] [move_group.moveit.moveit.kinematics.kdl_kinematics_plugin]: Joint weights for group 'arm': 1 1 1 1 1 1
[move_group-6] [INFO] [1762713094.705227244] [move_group.moveit.moveit.ros.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-6] [INFO] [1762713094.705416927] [move_group.moveit.moveit.ros.moveit_cpp]: Listening to 'joint_states' for joint states
[move_group-6] [INFO] [1762713094.706568021] [move_group.moveit.moveit.ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-6] [INFO] [1762713094.707157582] [move_group.moveit.moveit.ros.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[move_group-6] [INFO] [1762713094.707180123] [move_group.moveit.moveit.ros.planning_scene_monitor]: Stopping existing planning scene publisher.
[move_group-6] [INFO] [1762713094.707318485] [move_group.moveit.moveit.ros.planning_scene_monitor]: Stopped publishing maintained planning scene.
[move_group-6] [INFO] [1762713094.707697887] [move_group.moveit.moveit.ros.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-6] [INFO] [1762713094.707779569] [move_group.moveit.moveit.ros.planning_scene_monitor]: Starting planning scene monitor
[move_group-6] [INFO] [1762713094.709366866] [move_group.moveit.moveit.ros.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-6] [INFO] [1762713094.709396609] [move_group.moveit.moveit.ros.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-6] [INFO] [1762713094.710077309] [move_group.moveit.moveit.ros.planning_scene_monitor]: Listening to 'collision_object'
[move_group-6] [INFO] [1762713094.710899868] [move_group.moveit.moveit.ros.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-6] [WARN] [1762713094.711565782] [move_group.moveit.moveit.ros.occupancy_map_monitor]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-6] [ERROR] [1762713094.711606661] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates
[move_group-6] [INFO] [1762713094.722137123] [move_group.moveit.moveit.ros.planning_pipeline]: Successfully loaded planner 'STOMP'
[move_group-6] [INFO] [1762713094.724149474] [move_group]: Try loading adapter 'default_planning_request_adapters/ResolveConstraintFrames'
[move_group-6] [INFO] [1762713094.726387399] [move_group]: Loaded adapter 'default_planning_request_adapters/ResolveConstraintFrames'
[move_group-6] [INFO] [1762713094.726458503] [move_group]: Try loading adapter 'default_planning_request_adapters/ValidateWorkspaceBounds'
[move_group-6] [INFO] [1762713094.726926630] [move_group]: Loaded adapter 'default_planning_request_adapters/ValidateWorkspaceBounds'
[move_group-6] [INFO] [1762713094.726970656] [move_group]: Try loading adapter 'default_planning_request_adapters/CheckStartStateBounds'
[move_group-6] [INFO] [1762713094.727019294] [move_group]: Loaded adapter 'default_planning_request_adapters/CheckStartStateBounds'
[move_group-6] [INFO] [1762713094.727032109] [move_group]: Try loading adapter 'default_planning_request_adapters/CheckStartStateCollision'
[move_group-6] [INFO] [1762713094.727065814] [move_group]: Loaded adapter 'default_planning_request_adapters/CheckStartStateCollision'
[move_group-6] [INFO] [1762713094.729378582] [move_group]: Try loading adapter 'default_planning_response_adapters/AddTimeOptimalParameterization'
[move_group-6] [INFO] [1762713094.732641585] [move_group]: Loaded adapter 'default_planning_response_adapters/AddTimeOptimalParameterization'
[move_group-6] [INFO] [1762713094.732667076] [move_group]: Try loading adapter 'default_planning_response_adapters/ValidateSolution'
[move_group-6] [INFO] [1762713094.735873783] [move_group]: Loaded adapter 'default_planning_response_adapters/ValidateSolution'
[move_group-6] [INFO] [1762713094.735898705] [move_group]: Try loading adapter 'default_planning_response_adapters/DisplayMotionPath'
[move_group-6] [INFO] [1762713094.736506618] [move_group]: Loaded adapter 'default_planning_response_adapters/DisplayMotionPath'
[move_group-6] [INFO] [1762713094.741050105] [move_group.moveit.moveit.planners.pilz.joint_limits_aggregator]: Reading limits from namespace robot_description_planning
[ros2_control_node-2] [INFO] [1762713094.749170360] [controller_manager]: Received robot description from topic.
[move_group-6] [INFO] [1762713094.749366876] [move_group.moveit.moveit.planners.pilz.command_planner]: Available plugins: pilz_industrial_motion_planner/PlanningContextLoaderCIRC pilz_industrial_motion_planner/PlanningContextLoaderLIN pilz_industrial_motion_planner/PlanningContextLoaderPTP 
[move_group-6] [INFO] [1762713094.749431612] [move_group.moveit.moveit.planners.pilz.command_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderCIRC
[move_group-6] [INFO] [1762713094.752060441] [move_group.moveit.moveit.planners.pilz.command_planner]: Registered Algorithm [CIRC]
[move_group-6] [INFO] [1762713094.752105526] [move_group.moveit.moveit.planners.pilz.command_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderLIN
[ros2_control_node-2] [INFO] [1762713094.752585475] [controller_manager]: Loading hardware 'Arm' 
[ros2_control_node-2] [INFO] [1762713094.753151264] [controller_manager]: Loaded hardware 'Arm' from plugin 'mock_components/GenericSystem'
[ros2_control_node-2] [INFO] [1762713094.753212453] [controller_manager]: Initialize hardware 'Arm' 
[move_group-6] [INFO] [1762713094.753651532] [move_group.moveit.moveit.planners.pilz.command_planner]: Registered Algorithm [LIN]
[move_group-6] [INFO] [1762713094.753699899] [move_group.moveit.moveit.planners.pilz.command_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderPTP
[move_group-6] [INFO] [1762713094.755304251] [move_group.moveit.moveit.planners.pilz.command_planner]: Registered Algorithm [PTP]
[move_group-6] [INFO] [1762713094.755344240] [move_group.moveit.moveit.ros.planning_pipeline]: Successfully loaded planner 'Pilz Industrial Motion Planner'
[move_group-6] [INFO] [1762713094.757702403] [move_group]: Try loading adapter 'default_planning_request_adapters/ResolveConstraintFrames'
[move_group-6] [INFO] [1762713094.757866329] [move_group]: Loaded adapter 'default_planning_request_adapters/ResolveConstraintFrames'
[move_group-6] [INFO] [1762713094.757882463] [move_group]: Try loading adapter 'default_planning_request_adapters/ValidateWorkspaceBounds'
[move_group-6] [INFO] [1762713094.758400979] [move_group]: Loaded adapter 'default_planning_request_adapters/ValidateWorkspaceBounds'
[move_group-6] [INFO] [1762713094.758430210] [move_group]: Try loading adapter 'default_planning_request_adapters/CheckStartStateBounds'
[move_group-6] [INFO] [1762713094.758486557] [move_group]: Loaded adapter 'default_planning_request_adapters/CheckStartStateBounds'
[move_group-6] [INFO] [1762713094.758501132] [move_group]: Try loading adapter 'default_planning_request_adapters/CheckStartStateCollision'
[move_group-6] [INFO] [1762713094.758531499] [move_group]: Loaded adapter 'default_planning_request_adapters/CheckStartStateCollision'
[move_group-6] [INFO] [1762713094.761339223] [move_group]: Try loading adapter 'default_planning_response_adapters/ValidateSolution'
[move_group-6] [INFO] [1762713094.763887984] [move_group]: Loaded adapter 'default_planning_response_adapters/ValidateSolution'
[move_group-6] [INFO] [1762713094.763930350] [move_group]: Try loading adapter 'default_planning_response_adapters/DisplayMotionPath'
[move_group-6] [INFO] [1762713094.764556908] [move_group]: Loaded adapter 'default_planning_response_adapters/DisplayMotionPath'
[ros2_control_node-2] [INFO] [1762713094.767148062] [controller_manager]: Successful initialization of hardware 'Arm'
[ros2_control_node-2] [INFO] [1762713094.767629956] [resource_manager]: 'configure' hardware 'Arm' 
[ros2_control_node-2] [INFO] [1762713094.767655112] [resource_manager]: Successful 'configure' of hardware 'Arm'
[ros2_control_node-2] [INFO] [1762713094.767667413] [resource_manager]: 'activate' hardware 'Arm' 
[ros2_control_node-2] [INFO] [1762713094.767679934] [resource_manager]: Successful 'activate' of hardware 'Arm'
[ros2_control_node-2] [INFO] [1762713094.767790651] [controller_manager]: Registering statistics for : Arm
[ros2_control_node-2] [INFO] [1762713094.767979251] [controller_manager]: Resource Manager has been successfully initialized. Starting Controller Manager services...
[move_group-6] [INFO] [1762713094.772825084] [move_group.moveit.moveit.ros.planning_pipeline]: Successfully loaded planner 'CHOMP'
[move_group-6] [INFO] [1762713094.775043858] [move_group]: Try loading adapter 'default_planning_request_adapters/ResolveConstraintFrames'
[move_group-6] [INFO] [1762713094.775281173] [move_group]: Loaded adapter 'default_planning_request_adapters/ResolveConstraintFrames'
[move_group-6] [INFO] [1762713094.775309172] [move_group]: Try loading adapter 'default_planning_request_adapters/ValidateWorkspaceBounds'
[move_group-6] [INFO] [1762713094.775782888] [move_group]: Loaded adapter 'default_planning_request_adapters/ValidateWorkspaceBounds'
[move_group-6] [INFO] [1762713094.775838257] [move_group]: Try loading adapter 'default_planning_request_adapters/CheckStartStateBounds'
[move_group-6] [INFO] [1762713094.775936432] [move_group]: Loaded adapter 'default_planning_request_adapters/CheckStartStateBounds'
[move_group-6] [INFO] [1762713094.775970272] [move_group]: Try loading adapter 'default_planning_request_adapters/CheckStartStateCollision'
[move_group-6] [INFO] [1762713094.776005239] [move_group]: Loaded adapter 'default_planning_request_adapters/CheckStartStateCollision'
[move_group-6] [INFO] [1762713094.777981566] [move_group]: Try loading adapter 'default_planning_response_adapters/AddTimeOptimalParameterization'
[move_group-6] [INFO] [1762713094.779253877] [move_group]: Loaded adapter 'default_planning_response_adapters/AddTimeOptimalParameterization'
[move_group-6] [INFO] [1762713094.779301965] [move_group]: Try loading adapter 'default_planning_response_adapters/ValidateSolution'
[move_group-6] [INFO] [1762713094.780722599] [move_group]: Loaded adapter 'default_planning_response_adapters/ValidateSolution'
[move_group-6] [INFO] [1762713094.780782747] [move_group]: Try loading adapter 'default_planning_response_adapters/DisplayMotionPath'
[move_group-6] [INFO] [1762713094.781589725] [move_group]: Loaded adapter 'default_planning_response_adapters/DisplayMotionPath'
[move_group-6] [INFO] [1762713094.804242401] [move_group.moveit.moveit.ros.planning_pipeline]: Successfully loaded planner 'OMPL'
[move_group-6] [INFO] [1762713094.805825583] [move_group]: Try loading adapter 'default_planning_request_adapters/ResolveConstraintFrames'
[move_group-6] [INFO] [1762713094.805935863] [move_group]: Loaded adapter 'default_planning_request_adapters/ResolveConstraintFrames'
[move_group-6] [INFO] [1762713094.805944221] [move_group]: Try loading adapter 'default_planning_request_adapters/ValidateWorkspaceBounds'
[move_group-6] [INFO] [1762713094.806260140] [move_group]: Loaded adapter 'default_planning_request_adapters/ValidateWorkspaceBounds'
[move_group-6] [INFO] [1762713094.806276713] [move_group]: Try loading adapter 'default_planning_request_adapters/CheckStartStateBounds'
[move_group-6] [INFO] [1762713094.806314871] [move_group]: Loaded adapter 'default_planning_request_adapters/CheckStartStateBounds'
[move_group-6] [INFO] [1762713094.806328220] [move_group]: Try loading adapter 'default_planning_request_adapters/CheckStartStateCollision'
[move_group-6] [INFO] [1762713094.806354184] [move_group]: Loaded adapter 'default_planning_request_adapters/CheckStartStateCollision'
[move_group-6] [INFO] [1762713094.807901500] [move_group]: Try loading adapter 'default_planning_response_adapters/AddTimeOptimalParameterization'
[move_group-6] [INFO] [1762713094.808540006] [move_group]: Loaded adapter 'default_planning_response_adapters/AddTimeOptimalParameterization'
[move_group-6] [INFO] [1762713094.808561799] [move_group]: Try loading adapter 'default_planning_response_adapters/ValidateSolution'
[move_group-6] [INFO] [1762713094.809553666] [move_group]: Loaded adapter 'default_planning_response_adapters/ValidateSolution'
[move_group-6] [INFO] [1762713094.809594132] [move_group]: Try loading adapter 'default_planning_response_adapters/DisplayMotionPath'
[move_group-6] [INFO] [1762713094.810212176] [move_group]: Loaded adapter 'default_planning_response_adapters/DisplayMotionPath'
[move_group-6] [INFO] [1762713094.877986078] [move_group.moveit.moveit.plugins.simple_controller_manager]: Added FollowJointTrajectory controller for arm_controller
[move_group-6] [INFO] [1762713094.881455373] [move_group.moveit.moveit.plugins.simple_controller_manager]: Added FollowJointTrajectory controller for gripper_controller
[move_group-6] [INFO] [1762713094.881740256] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713094.881796479] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713094.882309205] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Trajectory execution is managing controllers
[move_group-6] [INFO] [1762713094.882334699] [move_group]: MoveGroup debug mode is ON
[move_group-6] [INFO] [1762713094.918211669] [move_group.moveit.moveit.planners.pilz.move_group_sequence_action]: initialize move group sequence action
[move_group-6] [INFO] [1762713094.924670128] [move_group.moveit.moveit.planners.pilz.joint_limits_aggregator]: Reading limits from namespace robot_description_planning
[move_group-6] [INFO] [1762713094.925058480] [move_group.moveit.moveit.planners.pilz.joint_limits_aggregator]: Reading limits from namespace robot_description_planning
[move_group-6] [INFO] [1762713094.931087338] [move_group.moveit.moveit.ros.move_group.executable]: 
[move_group-6] 
[move_group-6] ********************************************************
[move_group-6] * MoveGroup using: 
[move_group-6] *     - apply_planning_scene_service
[move_group-6] *     - clear_octomap_service
[move_group-6] *     - get_group_urdf
[move_group-6] *     - load_geometry_from_file
[move_group-6] *     - CartesianPathService
[move_group-6] *     - execute_trajectory_action
[move_group-6] *     - get_planning_scene_service
[move_group-6] *     - kinematics_service
[move_group-6] *     - move_action
[move_group-6] *     - motion_plan_service
[move_group-6] *     - query_planners_service
[move_group-6] *     - state_validation_service
[move_group-6] *     - save_geometry_to_file
[move_group-6] *     - SequenceAction
[move_group-6] *     - SequenceService
[move_group-6] ********************************************************
[move_group-6] 
[move_group-6] [INFO] [1762713094.931181050] [move_group.moveit.moveit.ros.move_group.context]: MoveGroup context using pipeline ompl
[move_group-6] [INFO] [1762713094.931211367] [move_group.moveit.moveit.ros.move_group.context]: MoveGroup context initialization complete
[move_group-6] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-6] Loading 'move_group/ClearOctomapService'...
[move_group-6] Loading 'move_group/GetUrdfService'...
[move_group-6] Loading 'move_group/LoadGeometryFromFileService'...
[move_group-6] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-6] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-6] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-6] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-6] Loading 'move_group/MoveGroupMoveAction'...
[move_group-6] Loading 'move_group/MoveGroupPlanService'...
[move_group-6] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-6] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-6] Loading 'move_group/SaveGeometryToFileService'...
[move_group-6] Loading 'pilz_industrial_motion_planner/MoveGroupSequenceAction'...
[move_group-6] Loading 'pilz_industrial_motion_planner/MoveGroupSequenceService'...
[move_group-6] 
[move_group-6] You can start planning now!
[move_group-6] 
[rviz2-7] [INFO] [1762713095.049536208] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-7] [INFO] [1762713095.049615993] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-7] [INFO] [1762713095.088763883] [rviz2]: Stereo is NOT SUPPORTED
[spawner-3] [INFO] [1762713095.173821850] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[rviz2-7] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-7]          at line 321 in /opt/ros/jazzy/include/class_loader/class_loader/class_loader_core.hpp
[ros2_control_node-2] [INFO] [1762713095.675595873] [controller_manager]: Loading controller : 'joint_state_broadcaster' of type 'joint_state_broadcaster/JointStateBroadcaster'
[ros2_control_node-2] [INFO] [1762713095.675655721] [controller_manager]: Loading controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1762713095.682564803] [controller_manager]: Controller 'joint_state_broadcaster' node arguments: --ros-args --params-file /home/johotan/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws/install/my_robot_bringup/share/my_robot_bringup/config/ros2_controllers.yaml 
[spawner-3] [INFO] [1762713095.711869534] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[ros2_control_node-2] [INFO] [1762713095.712754427] [controller_manager]: Configuring controller: 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1762713095.712862414] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[ros2_control_node-2] [INFO] [1762713095.722708597] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]
[ros2_control_node-2] [INFO] [1762713095.730767388] [controller_manager]: Successfully switched controllers!
[spawner-3] [INFO] [1762713095.742562925] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[spawner-4] [INFO] [1762713095.863220059] [spawner_arm_controller]: waiting for service /controller_manager/list_controllers to become available...
[INFO] [spawner-3]: process has finished cleanly [pid 43024]
[ros2_control_node-2] [INFO] [1762713096.114925346] [controller_manager]: Loading controller : 'arm_controller' of type 'joint_trajectory_controller/JointTrajectoryController'
[ros2_control_node-2] [INFO] [1762713096.114970968] [controller_manager]: Loading controller 'arm_controller'
[ros2_control_node-2] [INFO] [1762713096.117067450] [controller_manager]: Controller 'arm_controller' node arguments: --ros-args --params-file /home/johotan/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws/install/my_robot_bringup/share/my_robot_bringup/config/ros2_controllers.yaml 
[spawner-4] [INFO] [1762713096.142364658] [spawner_arm_controller]: Loaded arm_controller
[ros2_control_node-2] [INFO] [1762713096.143783096] [controller_manager]: Configuring controller: 'arm_controller'
[ros2_control_node-2] [INFO] [1762713096.144093599] [arm_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[ros2_control_node-2] [INFO] [1762713096.144134147] [arm_controller]: Command interfaces are [position] and state interfaces are [position].
[ros2_control_node-2] [INFO] [1762713096.144167234] [arm_controller]: Using 'splines' interpolation method.
[ros2_control_node-2] [INFO] [1762713096.147116599] [arm_controller]: Action status changes will be monitored at 20.00 Hz.
[ros2_control_node-2] [INFO] [1762713096.161957720] [controller_manager]: Activating controllers: [ arm_controller ]
[ros2_control_node-2] [INFO] [1762713096.170824020] [controller_manager]: Successfully switched controllers!
[spawner-4] [INFO] [1762713096.182919799] [spawner_arm_controller]: Configured and activated arm_controller
[spawner-5] [INFO] [1762713096.326804950] [spawner_gripper_controller]: waiting for service /controller_manager/list_controllers to become available...
[INFO] [spawner-4]: process has finished cleanly [pid 43025]
[ros2_control_node-2] [INFO] [1762713096.578474620] [controller_manager]: Loading controller : 'gripper_controller' of type 'joint_trajectory_controller/JointTrajectoryController'
[ros2_control_node-2] [INFO] [1762713096.578545642] [controller_manager]: Loading controller 'gripper_controller'
[ros2_control_node-2] [INFO] [1762713096.578875689] [controller_manager]: Controller 'gripper_controller' node arguments: --ros-args --params-file /home/johotan/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws/install/my_robot_bringup/share/my_robot_bringup/config/ros2_controllers.yaml 
[spawner-5] [INFO] [1762713096.602349676] [spawner_gripper_controller]: Loaded gripper_controller
[ros2_control_node-2] [INFO] [1762713096.603360327] [controller_manager]: Configuring controller: 'gripper_controller'
[ros2_control_node-2] [INFO] [1762713096.603460539] [gripper_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[ros2_control_node-2] [INFO] [1762713096.603482582] [gripper_controller]: Command interfaces are [position] and state interfaces are [position].
[ros2_control_node-2] [INFO] [1762713096.603494696] [gripper_controller]: Using 'splines' interpolation method.
[ros2_control_node-2] [INFO] [1762713096.604569059] [gripper_controller]: Action status changes will be monitored at 20.00 Hz.
[ros2_control_node-2] [INFO] [1762713096.612011177] [controller_manager]: Activating controllers: [ gripper_controller ]
[ros2_control_node-2] [INFO] [1762713096.620806489] [controller_manager]: Successfully switched controllers!
[spawner-5] [INFO] [1762713096.632072579] [spawner_gripper_controller]: Configured and activated gripper_controller
[INFO] [spawner-5]: process has finished cleanly [pid 43026]
[rviz2-7] [ERROR] [1762713098.313805218] [moveit_1411923596.moveit.ros.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-7] [INFO] [1762713098.333466550] [moveit_1411923596.moveit.ros.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-7] [INFO] [1762713098.456712487] [moveit_1411923596.moveit.ros.rdf_loader]: Loaded robot model in 0.0571885 seconds
[rviz2-7] [INFO] [1762713098.456780668] [moveit_1411923596.moveit.core.robot_model]: Loading robot model 'my_robot'...
[rviz2-7] [WARN] [1762713098.462279751] [moveit_1411923596.moveit.ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[rviz2-7] [INFO] [1762713098.478876367] [moveit_1411923596.moveit.ros.planning_scene_monitor]: Starting planning scene monitor
[rviz2-7] [INFO] [1762713098.479762428] [moveit_1411923596.moveit.ros.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-7] [INFO] [1762713098.494134056] [interactive_marker_display_103360285940448]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-7] [INFO] [1762713098.498190609] [moveit_1411923596.moveit.ros.robot_interaction]: No active joints or end effectors found for group 'arm'. Make sure that kinematics.yaml is loaded in this node's namespace.
[rviz2-7] [INFO] [1762713098.503895664] [moveit_1411923596.moveit.ros.motion_planning_frame]: group arm
[rviz2-7] [INFO] [1762713098.503957869] [moveit_1411923596.moveit.ros.motion_planning_frame]: Constructing new MoveGroup connection for group 'arm' in namespace ''
[rviz2-7] [INFO] [1762713098.515782600] [interactive_marker_display_103360285940448]: Sending request for interactive markers
[rviz2-7] [INFO] [1762713098.524457076] [moveit_1411923596.moveit.ros.move_group_interface]: Ready to take commands for planning group arm.
[rviz2-7] [INFO] [1762713098.542771002] [interactive_marker_display_103360285940448]: Service response received for initialization
[move_group-6] [INFO] [1762713269.571101074] [move_group.moveit.moveit.ros.move_group.move_action]: MoveGroupMoveAction: Received request
[move_group-6] [INFO] [1762713269.571733573] [move_group.moveit.moveit.ros.move_group.move_action]: executing..
[move_group-6] [INFO] [1762713269.581194861] [move_group.moveit.moveit.ros.move_group.move_action]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-6] [INFO] [1762713269.581534936] [move_group]: Calling PlanningRequestAdapter 'ResolveConstraintFrames'
[move_group-6] [INFO] [1762713269.581583619] [move_group]: Calling PlanningRequestAdapter 'ValidateWorkspaceBounds'
[move_group-6] [WARN] [1762713269.581591560] [move_group.moveit.moveit.ros.validate_workspace_bounds]: It looks like the planning volume was not specified. Using default values.
[move_group-6] [INFO] [1762713269.581605551] [move_group]: Calling PlanningRequestAdapter 'CheckStartStateBounds'
[move_group-6] [INFO] [1762713269.581637528] [move_group]: Calling PlanningRequestAdapter 'CheckStartStateCollision'
[move_group-6] [INFO] [1762713269.582458080] [move_group.moveit.moveit.planners.ompl.model_based_planning_context]: Planner configuration 'gripper' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-6] [INFO] [1762713269.582605332] [move_group]: Calling Planner 'OMPL'
[move_group-6] [INFO] [1762713269.600724940] [move_group]: Calling PlanningResponseAdapter 'AddTimeOptimalParameterization'
[move_group-6] [INFO] [1762713269.603231477] [move_group]: Calling PlanningResponseAdapter 'ValidateSolution'
[move_group-6] [INFO] [1762713269.603387585] [move_group]: Calling PlanningResponseAdapter 'DisplayMotionPath'
[move_group-6] [WARN] [1762713269.603461681] [move_group.moveit.moveit.ros.planning_pipeline]: The planner plugin did not fill out the 'planner_id' field of the MotionPlanResponse. Setting it to the planner ID name of the MotionPlanRequest assuming that the planner plugin does warn you if it does not use the requested planner.
[move_group-6] [INFO] [1762713269.603510389] [move_group.moveit.moveit.ros.move_group.move_action]: Motion plan was computed successfully.
[move_group-6] [INFO] [1762713269.604515623] [move_group.moveit.moveit.ros.move_group.clear_octomap_service]: Execution request received
[move_group-6] [INFO] [1762713269.604571366] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713269.604631446] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713269.604732541] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.01
[move_group-6] [INFO] [1762713269.611144638] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-6] [INFO] [1762713269.611198620] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713269.611221383] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713269.611353201] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to gripper_controller
[ros2_control_node-2] [INFO] [1762713269.611689014] [gripper_controller]: Received new action goal
[ros2_control_node-2] [INFO] [1762713269.611721714] [gripper_controller]: Accepted new action goal
[move_group-6] [INFO] [1762713269.611849644] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: gripper_controller started execution
[move_group-6] [INFO] [1762713269.611902731] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[ros2_control_node-2] [INFO] [1762713271.810634560] [gripper_controller]: Goal reached, success!
[move_group-6] [INFO] [1762713271.812428032] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'gripper_controller' successfully finished
[move_group-6] [INFO] [1762713271.840913519] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-6] [INFO] [1762713271.841103512] [move_group.moveit.moveit.ros.move_group.clear_octomap_service]: Execution completed: SUCCEEDED
[move_group-6] [INFO] [1762713287.492205024] [move_group.moveit.moveit.ros.move_group.move_action]: MoveGroupMoveAction: Received request
[move_group-6] [INFO] [1762713287.492377368] [move_group.moveit.moveit.ros.move_group.move_action]: executing..
[move_group-6] [INFO] [1762713287.501004577] [move_group.moveit.moveit.ros.move_group.move_action]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-6] [INFO] [1762713287.501286209] [move_group]: Calling PlanningRequestAdapter 'ResolveConstraintFrames'
[move_group-6] [INFO] [1762713287.501337816] [move_group]: Calling PlanningRequestAdapter 'ValidateWorkspaceBounds'
[move_group-6] [WARN] [1762713287.501346516] [move_group.moveit.moveit.ros.validate_workspace_bounds]: It looks like the planning volume was not specified. Using default values.
[move_group-6] [INFO] [1762713287.501362729] [move_group]: Calling PlanningRequestAdapter 'CheckStartStateBounds'
[move_group-6] [INFO] [1762713287.501382061] [move_group]: Calling PlanningRequestAdapter 'CheckStartStateCollision'
[move_group-6] [INFO] [1762713287.501753108] [move_group.moveit.moveit.planners.ompl.model_based_planning_context]: Planner configuration 'gripper' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-6] [INFO] [1762713287.501837772] [move_group]: Calling Planner 'OMPL'
[move_group-6] [INFO] [1762713287.522897804] [move_group]: Calling PlanningResponseAdapter 'AddTimeOptimalParameterization'
[move_group-6] [INFO] [1762713287.525416351] [move_group]: Calling PlanningResponseAdapter 'ValidateSolution'
[move_group-6] [INFO] [1762713287.525569027] [move_group]: Calling PlanningResponseAdapter 'DisplayMotionPath'
[move_group-6] [WARN] [1762713287.525632020] [move_group.moveit.moveit.ros.planning_pipeline]: The planner plugin did not fill out the 'planner_id' field of the MotionPlanResponse. Setting it to the planner ID name of the MotionPlanRequest assuming that the planner plugin does warn you if it does not use the requested planner.
[move_group-6] [INFO] [1762713287.525671272] [move_group.moveit.moveit.ros.move_group.move_action]: Motion plan was computed successfully.
[move_group-6] [INFO] [1762713287.527119823] [move_group.moveit.moveit.ros.move_group.clear_octomap_service]: Execution request received
[move_group-6] [INFO] [1762713287.527171240] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713287.527200409] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713287.527369277] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.01
[move_group-6] [INFO] [1762713287.531222310] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-6] [INFO] [1762713287.531313379] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713287.531341694] [move_group.moveit.moveit.plugins.simple_controller_manager]: Returned 2 controllers in list
[move_group-6] [INFO] [1762713287.531495984] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to gripper_controller
[ros2_control_node-2] [INFO] [1762713287.531840138] [gripper_controller]: Received new action goal
[ros2_control_node-2] [INFO] [1762713287.531878328] [gripper_controller]: Accepted new action goal
[move_group-6] [INFO] [1762713287.532091772] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: gripper_controller started execution
[move_group-6] [INFO] [1762713287.532107330] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[ros2_control_node-2] [INFO] [1762713289.730621677] [gripper_controller]: Goal reached, success!
[move_group-6] [INFO] [1762713289.732428752] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'gripper_controller' successfully finished
[move_group-6] [INFO] [1762713289.760942412] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-6] [INFO] [1762713289.761092108] [move_group.moveit.moveit.ros.move_group.clear_octomap_service]: Execution completed: SUCCEEDED



TERMINAL2
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 run my_robot_commander_cpp commander 
[INFO] [1762713140.965645363] [moveit_4171606523.moveit.ros.rdf_loader]: Loaded robot model in 0.292161 seconds
[INFO] [1762713140.965729450] [moveit_4171606523.moveit.core.robot_model]: Loading robot model 'my_robot'...
[WARN] [1762713140.970004394] [moveit_4171606523.moveit.ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[INFO] [1762713141.005538255] [moveit_4171606523.moveit.ros.move_group_interface]: Ready to take commands for planning group arm.
[INFO] [1762713141.016195486] [moveit_4171606523.moveit.ros.move_group_interface]: Ready to take commands for planning group gripper.
[INFO] [1762713269.570213148] [moveit_4171606523.moveit.ros.move_group_interface]: MoveGroup action client/server ready
[INFO] [1762713269.571429633] [moveit_4171606523.moveit.ros.move_group_interface]: Planning request accepted
[INFO] [1762713269.603862060] [moveit_4171606523.moveit.ros.move_group_interface]: Planning request complete!
[INFO] [1762713269.604045742] [moveit_4171606523.moveit.ros.move_group_interface]: time taken to generate plan: 0.0173875 seconds
[INFO] [1762713269.604552421] [moveit_4171606523.moveit.ros.move_group_interface]: Execute request accepted
[INFO] [1762713271.841532667] [moveit_4171606523.moveit.ros.move_group_interface]: Execute request success!
[INFO] [1762713287.491995013] [moveit_4171606523.moveit.ros.move_group_interface]: MoveGroup action client/server ready
[INFO] [1762713287.492339126] [moveit_4171606523.moveit.ros.move_group_interface]: Planning request accepted
[INFO] [1762713287.525871130] [moveit_4171606523.moveit.ros.move_group_interface]: Planning request complete!
[INFO] [1762713287.526723583] [moveit_4171606523.moveit.ros.move_group_interface]: time taken to generate plan: 0.0208415 seconds
[INFO] [1762713287.527168487] [moveit_4171606523.moveit.ros.move_group_interface]: Execute request accepted
[INFO] [1762713289.761307650] [moveit_4171606523.moveit.ros.move_group_interface]: Execute request success!



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
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic info /open_gripper 
Type: example_interfaces/msg/Bool
Publisher count: 0
Subscription count: 1
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "{data: false}"
publisher: beginning loop
publishing #1: example_interfaces.msg.Bool(data=False)

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "{data: true}"
Waiting for at least 1 matching subscription(s)...
Waiting for at least 1 matching subscription(s)...
publisher: beginning loop
publishing #1: example_interfaces.msg.Bool(data=True)

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 


