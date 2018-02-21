/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// ROS includes
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

// General messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// C++ includes
#include <vector>
#include <functional>


// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_fusion");
  ros::NodeHandle nh("~");

  // Get some global information
  if (!nh.getParam("serial", serial_))
    ROS_FATAL("Failed to get serial parameter.");
  if (!nh.getParam("frame", frame_))
    ROS_FATAL("Failed to get frame parameter.");
  if (!GetVectorParam(nh, "gravity", gravity_))
    ROS_FATAL("Failed to get gravity parameter.");

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
