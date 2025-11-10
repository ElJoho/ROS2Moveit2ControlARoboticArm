import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/johotan/ros2/ROS2Moveit2ControlARoboticArm/moveit2_ws/install/my_robot_commander_py'
