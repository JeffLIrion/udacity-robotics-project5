#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace project5 {
constexpr char kNodeName[] = "pick_objects";
constexpr char kActionName[] = "move_base";
constexpr char kFrameID[] = "map";

// Topics
constexpr char kGoalTopic[] = "/pick_objects_goal";
constexpr char kSuccessTopic[] = "/pick_objects_success";
constexpr int kPublishQueueSize = 10;

constexpr double x1 = 1.0;
constexpr double y1 = 2.0;
constexpr double qw1 = 0.0;

constexpr double x2 = 3.0;
constexpr double y2 = 4.0;
constexpr double qw2 = 1.0;
};  // namespace project5

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickObjects {
 public:
  PickObjects() {
    // Inform ROS master that we will be publishing messages
    goal_pub_ = node_handle_.advertise<geometry_msgs::Pose>(project5::kGoalTopic, project5::kPublishQueueSize);
    success_pub_ = node_handle_.advertise<std_msgs::Bool>(project5::kSuccessTopic, project5::kPublishQueueSize);
  }

  /**
   * @brief Wait for `add_markers.cpp` to subscribe to the "/pick_objects_goal" and "/pick_objects_success" targets.
   */
  void WaitForAddMarkers() {
    while (goal_pub_.getNumSubscribers() < 1 || success_pub_.getNumSubscribers() < 1) {
      ros::Duration(1).sleep();
    }
  }

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
    goal.target_pose.header.frame_id = project5::kFrameID;
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = qw;

    // Publish the goal pose
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = x;
    goal_pose.position.y = y;
    goal_pose.orientation.w = qw;
    goal_pub_.publish(goal_pose);

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    action_client->sendGoal(goal);

    // Wait an infinite time for the results
    action_client->waitForResult();

    // Check if the robot reached its goal
    if (action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Successfully drove to (%f, %f, %f)", x, y, qw);
      std_msgs::Bool bool_msg;
      bool_msg.data = true;
      success_pub_.publish(bool_msg);
      return true;
    }

    ROS_INFO("Failed to drive to (%f, %f, %f)", x, y, qw);
    std_msgs::Bool bool_msg;
    bool_msg.data = false;
    success_pub_.publish(bool_msg);
    return false;
  }

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher goal_pub_;
  ros::Publisher success_pub_;
};

int main(int argc, char **argv) {
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, project5::kNodeName);

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient action_client(project5::kActionName, true);

  PickObjects pick_objects;
  pick_objects.WaitForAddMarkers();

  // Drive to the first goal
  if (!pick_objects.DriveToGoal(&action_client, project5::x1, project5::y1, project5::qw1)) {
    return -1;
  }

  // Wait 5 seconds
  ros::Duration(5).sleep();

  // Drive to the second goal
  if (!pick_objects.DriveToGoal(&action_client, project5::x2, project5::y2, project5::qw2)) {
    return -1;
  }

  return 0;
}
