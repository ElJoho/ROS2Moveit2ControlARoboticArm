#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char  **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    
    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");
    gripper.setMaxVelocityScalingFactor(1.0);
    gripper.setMaxAccelerationScalingFactor(1.0);

    // Joint Goal
    /*
    std::vector<double> joints = {1.5, 0.5 , 0.0, 1.5 , 0.0 , -0.7};


    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS); //planing
    if (success3){
        arm.execute(plan3);
    }
    */
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id ="base_link";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan4;
    bool success4 = (arm.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS); //planing
    if (success4){
        arm.execute(plan4);
    }
    /** ---------------------------------------------------------- 
    //Named Goal
    arm.setStartStateToCurrentState(); //starting state
    arm.setNamedTarget("pos_1"); //goal

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS); //planing
    if (success){
        arm.execute(plan1);
    }

 arm.setStartStateToCurrentState();
 arm.setNamedTarget("home");

 moveit::planning_interface::MoveGroupInterface::Plan plan2;
 bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

 if (success2) {
     arm.execute(plan2);
 }
*/
    rclcpp::shutdown();
    spinner.join();
    return 0;
}