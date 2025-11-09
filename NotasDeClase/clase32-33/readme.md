# Clase 32 - 33
TERMINAL1
ros2 pkg create my_robot_commander_cpp --build-type ament_cmake --dependencies rclcpp

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ colcon build

Starting >>> my_robot_description
Starting >>> my_robot_commander_cpp
Finished <<< my_robot_description [0.13s]                                                                    
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [0.11s]                                                                        
Starting >>> my_robot_bringup
Finished <<< my_robot_bringup [0.12s]                                                                      
Finished <<< my_robot_commander_cpp [9.94s]                      

Summary: 4 packages finished [10.1s]
johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch my_robot_bringup my_robot.launch.xml 


terminal2

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash

johotan@johotan:~/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 run my_robot_commander_cpp test_moveit 
