#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

constexpr char kNodeName[] = "pick_objects";
constexpr char kActionName[] = "move_base";
constexpr char kFrameID[] = "base_footprint";

constexpr double x1 = 1.0;
constexpr double y1 = 2.0;
constexpr double qw1 = 0.0;

constexpr double x2 = 3.0;
constexpr double y2 = 4.0;
constexpr double qw2 = 1.0;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @brief Drive to the specified position.
 *
 * @param action_client The move base action client
 * @param x The x coordinate of the goal
 * @param y The y coordinate of the goal
 * @param qw The `q_w` component of the goal's quaternion orientation
 * @return `true` if successful; `false` otherwise
 */
bool DriveToGoal(MoveBaseClient *action_client, const double &x, const double &y, const double &qw) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = kFrameID;
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = qw;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  action_client->sendGoal(goal);

  // Wait an infinite time for the results
  action_client->waitForResult();

  // Check if the robot reached its goal
  if (action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Successfully drove to (%f, %f, %f)", x, y, qw);
    return true;
  }

  ROS_INFO("Failed to drive to (%f, %f, %f)", x, y, qw);
  return false;
}

int main(int argc, char **argv) {
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, kNodeName);

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient action_client(kActionName, true);

  // Drive to the first goal
  if (!DriveToGoal(&action_client, x1, y1, qw1)) {
    return -1;
  }

  // Wait 5 seconds
  ros::Duration(5).sleep();

  // Drive to the second goal
  if (!DriveToGoal(&action_client, x2, y2, qw2)) {
    return -1;
  }

  return 0;
}