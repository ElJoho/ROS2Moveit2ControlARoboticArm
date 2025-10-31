# Clase 21

editar archivo ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_moveit_config/config/joint_limits.yaml y poner todos los numeros como float, es decir, añadir el .0

editar archivo /home/joho/ros2_ws/ROS2Moveit2ControlARoboticArm/moveit2_ws/src/my_robot_moveit_config/config/moveit_controllers.yaml y añadir al final 
      - joint6
    action_ns: follow_joint_trajectory
    default: true