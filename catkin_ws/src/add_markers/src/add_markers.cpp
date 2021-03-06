#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

constexpr char kNodeName[] = "add_markers";
constexpr char kFrameID[] = "odom";
constexpr char kMarkerNamespace[] = "udacity_project5";
constexpr char kTopic[] = "visualization_marker";
constexpr int kMarkerType = visualization_msgs::Marker::CUBE;

// Topics from `pick_objects.cpp`
constexpr char kGoalTopic[] = "/pick_objects_goal";
constexpr char kSuccessTopic[] = "/pick_objects_success";
constexpr int kSubscribeQueueSize = 10;

class MarkerPublisher {
 public:
  MarkerPublisher() {
    pub_ = node_handle_.advertise<visualization_msgs::Marker>(kTopic, 1);

    // Subscribe to topics published by `pick_objects.cpp`
    goal_sub_ = node_handle_.subscribe(kGoalTopic, kSubscribeQueueSize, &MarkerPublisher::GoalCallback, this);
    success_sub_ = node_handle_.subscribe(kSuccessTopic, kSubscribeQueueSize, &MarkerPublisher::SuccessCallback, this);

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker_.header.frame_id = kFrameID;
    marker_.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_.ns = kMarkerNamespace;
    marker_.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker_.type = kMarkerType;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // marker_.action = add_marker ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_.pose.position.x = 0.;
    marker_.pose.position.y = 0.;
    marker_.pose.position.z = 00;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_.scale.x = 0.5;
    marker_.scale.y = 0.5;
    marker_.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;

    marker_.lifetime = ros::Duration();
  }

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber success_sub_;
  visualization_msgs::Marker marker_;
  bool marker_is_displayed_ = false;
  bool first_goal_ = true;
  /**
   * @brief Callback for when a new goal is provided.
   *
   * @param pose_msg The destination pose
   */
  void GoalCallback(const geometry_msgs::Pose& pose_msg) {
    if (marker_is_displayed_ || first_goal_) {
      ROS_INFO("Pick up from (%f, %f)", pose_msg.position.x, pose_msg.position.y);
    } else {
      ROS_INFO("Drop off at (%f, %f)", pose_msg.position.x, pose_msg.position.y);
    }

    marker_.pose.position.x = pose_msg.position.x;
    marker_.pose.position.y = pose_msg.position.y;

    if (first_goal_) {
      marker_.action = visualization_msgs::Marker::ADD;
      marker_is_displayed_ = true;
      pub_.publish(marker_);
    }

    first_goal_ = false;
  }

  /**
   * @brief Callback when the robot reaches its destination.
   *
   * @param bool_msg Whether or not the robot successfully reached its destination
   */
  void SuccessCallback(const std_msgs::Bool& bool_msg) {
    if (!marker_is_displayed_) {
      ROS_INFO("Drop off at (%f, %f) was %s", marker_.pose.position.x, marker_.pose.position.y,
               bool_msg.data ? "successful" : "not successful");
    } else {
      ROS_INFO("Pick up from (%f, %f) was %s", marker_.pose.position.x, marker_.pose.position.y,
               bool_msg.data ? "successful" : "not successful");
    }

    marker_is_displayed_ = !marker_is_displayed_;
    marker_.action = marker_is_displayed_ ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    pub_.publish(marker_);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, kNodeName);
  MarkerPublisher marker_publisher;

  ros::spin();

  return 0;
}
