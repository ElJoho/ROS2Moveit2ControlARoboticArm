 # Clase 10
 ULR A REVISAR: https://wiki.ros.org/urdf/XML/link
  FILES: arm.urdf

  TERMINAL

 joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ cd src/
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ cd ur
bash: cd: ur: No such file or directory
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ cd my_robot_description/urdf/
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf$ touch arm.urdf
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf$ cd ../../..
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ cd src
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ sudo apt install ros-humble-urdf-tutorial
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
ros-humble-urdf-tutorial is already the newest version (1.1.0-1jammy.20251017.050310).
The following packages were automatically installed and are no longer required:
  libjs-jquery-hotkeys libjs-jquery-isonscreen libjs-jquery-metadata libjs-jquery-tablesorter libjs-jquery-throttle-debounce mercurial mercurial-common python3-colcon-argcomplete python3-colcon-bash
  python3-colcon-cd python3-colcon-cmake python3-colcon-common-extensions python3-colcon-core python3-colcon-defaults python3-colcon-devtools python3-colcon-installed-package-information
  python3-colcon-library-path python3-colcon-metadata python3-colcon-mixin python3-colcon-notification python3-colcon-output python3-colcon-override-check python3-colcon-package-information
  python3-colcon-package-selection python3-colcon-parallel-executor python3-colcon-pkg-config python3-colcon-powershell python3-colcon-python-setup-py python3-colcon-recursive-crawl python3-colcon-ros
  python3-colcon-test-result python3-colcon-zsh python3-cov-core python3-coverage python3-distlib python3-nose2 python3-pytest-cov python3-vcstools ros-build-essential usb-modeswitch usb-modeswitch-data
Use 'sudo apt autoremove' to remove them.
0 upgraded, 0 newly installed, 0 to remove and 14 not upgraded.
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ ls
my_robot_description
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ 
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src$ cd ..
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.22s]                  

Summary: 1 package finished [1.06s]
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch urdf tutorial display.launch.py model:=/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/arm.urdf
file 'tutorial' was not found in the share directory of package 'urdf' which is at '/opt/ros/humble/share/urdf'
joho@joho-X550DP:~/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws$ ros2 launch urdf_tutorial display.launch.py model:=/home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_description/urdf/arm.urdf
[INFO] [launch]: All log files can be found below /home/joho/.ros/log/2025-10-28-14-11-37-973594-joho-X550DP-25824
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [25826]
[INFO] [joint_state_publisher_gui-2]: process started with pid [25828]
[INFO] [rviz2-3]: process started with pid [25830]
[robot_state_publisher-1] [INFO] [1761678698.577869452] [robot_state_publisher]: got segment base_link
[rviz2-3] [INFO] [1761678699.659868268] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1761678699.660037425] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-3] [INFO] [1761678699.759438433] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_gui-2] [INFO] [1761678700.462972963] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[joint_state_publisher_gui-2] [INFO] [1761678700.504223419] [joint_state_publisher]: Centering
[joint_state_publisher_gui-2] [INFO] [1761678700.691921673] [joint_state_publisher]: Centering


