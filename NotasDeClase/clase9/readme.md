FILES: my_robot_description/CMakeLists.txt

joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm$ ls
moveit2_ws  NotasDeClase  README.md  Setup
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm$ cd moveit2_ws/
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ mkdir src
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ls
src
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ colcon build
                     
Summary: 0 packages finished [1.26s]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ . install/setup.bash
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ cd src/
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ ros2 pkg create my_robot_description
going to create a new package
package name: my_robot_description
destination directory: /home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['joho <johalopezari@unal.edu.co>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
creating folder ./my_robot_description
creating ./my_robot_description/package.xml
creating source and include folder
creating folder ./my_robot_description/src
creating folder ./my_robot_description/include/my_robot_description
creating ./my_robot_description/CMakeLists.txt

[WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
It is recommended to use one of the ament license identitifers:
Apache-2.0
BSL-1.0
BSD-2.0
BSD-2-Clause
BSD-3-Clause
GPL-3.0-only
LGPL-3.0-only
MIT
MIT-0
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ ls
my_robot_description
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ cd ..
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ cd src/my_robot_description/
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description$ ls
CMakeLists.txt  include  package.xml  src
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description$ mkdir urdf launch rviz
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description$  ls
CMakeLists.txt  include  launch  package.xml  rviz  src  urdf
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description$ cd ../..
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [2.52s]                  

Summary: 1 package finished [3.37s]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ 

