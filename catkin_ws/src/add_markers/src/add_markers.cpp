#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

constexpr char kNodeName[] = "add_markers";
constexpr char kFrameID[] = "map";
constexpr char kMarkerNamespace[] = "udacity_project5";
constexpr char kTopic[] = "visualization_marker";
constexpr int kMarkerType = visualization_msgs::Marker::CUBE;

class MarkerPublisher {
 public:
  MarkerPublisher() { pub_ = node_handle_.advertise<visualization_msgs::Marker>(kTopic, 1); }

  /**
   * @brief Add a marker.
   *
   * @param x The x location of the marker
   * @param y the y location of the marker
   * @param marker_id The ID for the marker
   */
  void AddMarker(const double &x, const double &y, const int32_t &marker_id = 0) {
    AddOrDeleteMarker(true, x, y, marker_id);
  }

  /**
   * @brief Delete a marker.
   *
   * @param x The x location of the marker
   * @param y the y location of the marker
   * @param marker_id The ID for the marker
   */
  void DeleteMarker(const double &x, const double &y, const int32_t &marker_id = 0) {
    AddOrDeleteMarker(false, x, y, marker_id);
  }

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher pub_;

  /**
   * @brief Add a marker.
   *
   * @param add_marker If `true`, we will add the marker; otherwise, we will delete it
   * @param x The x location of the marker
   * @param y the y location of the marker
   * @param marker_id The ID for the marker
   */
  void AddOrDeleteMarker(const bool &add_marker, const double &x, const double &y, const int32_t &marker_id) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = kFrameID;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = kMarkerNamespace;
    marker.id = marker_id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = kMarkerType;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = add_marker ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (pub_.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    pub_.publish(marker);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, kNodeName);
  MarkerPublisher marker_publisher;

  marker_publisher.AddMarker(1, 2);
}
