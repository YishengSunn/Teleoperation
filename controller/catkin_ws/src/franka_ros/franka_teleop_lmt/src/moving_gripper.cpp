#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_grasp_client");

  // Create the action client (true = don't need ros::spin() for callbacks)
  actionlib::SimpleActionClient<franka_gripper::GraspAction> client("franka_gripper/grasp", true);

  ROS_INFO("Waiting for gripper action server...");
  client.waitForServer();
  ROS_INFO("Gripper action server is up!");

  franka_gripper::GraspGoal goal;
  goal.width = 0.04;         // meters
  goal.speed = 0.05;         // meters/second
  goal.force = 20.0;         // Newtons
  goal.epsilon.inner = 0.005;
  goal.epsilon.outer = 0.005;

  ROS_INFO("Sending grasp goal...");
  client.sendGoal(goal);

  return 0;
}
