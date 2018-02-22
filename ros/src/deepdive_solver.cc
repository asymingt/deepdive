/*
  This ROS node listens to data from all trackers in the system and provides
  a global solution to the tracking problem. That is, it solves for the
  relative location of the lighthouses and trackers as a function of time.
*/

// ROS includes
#include <ros/ros.h>

// For transforms
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// Standard messages
#include <geometry_msgs/TransformStamped.h>

// Non-standard datra messages
#include <deepdive_ros/Light.h>
#include <deepdive_ros/Lighthouses.h>
#include <deepdive_ros/Trackers.h>

// Utility functions
#include "deepdive.hh"

// Ceres and logging
#include <Eigen/Core>
#include <Eigen/Geometry>

// C++ libraries
#include <map>
#include <vector>
#include <string>
#include <thread>

// Global memory
struct Lighthouse {
  std::string name;
  Eigen::Affine3d wTl;
};
std::map<std::string, Lighthouse> lighthouses_;
struct Tracker {
  std::string name;
  Eigen::Affine3d bTh;
};
std::map<std::string, Tracker> trackers_;
std::string frame_parent_ = "world";
std::string frame_child_ = "body";
bool refine_extrinsics_ = false;
bool refine_calibration_ = false;
double param_max_time_ = 1.0;
int param_max_iterations_ = 1000;
int param_threads_ = 4;
double param_gradient_tolerance_ = 1e-6;
bool param_debug_ = false;

// Utility functions

void BroadcastLighthouseTransforms() {
  static tf2_ros::StaticTransformBroadcaster bc;
  static geometry_msgs::TransformStamped tfs;
  std::map<std::string, Lighthouse>::iterator it;
  for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) {
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = frame_parent_;
    tfs.child_frame_id = it->first;
    tfs.transform.translation.x = it->second.wTl.translation()[0];
    tfs.transform.translation.y = it->second.wTl.translation()[1];
    tfs.transform.translation.z = it->second.wTl.translation()[2];
    Eigen::Quaterniond quat(it->second.wTl.linear());
    tfs.transform.rotation.w = quat.w();
    tfs.transform.rotation.x = quat.x();
    tfs.transform.rotation.y = quat.y();
    tfs.transform.rotation.z = quat.z();
    bc.sendTransform(tfs);
  }
}

void BroadcastTrackerTransforms() {
  static tf2_ros::StaticTransformBroadcaster bc;
  static geometry_msgs::TransformStamped tfs;
  std::map<std::string, Tracker>::iterator it;
  for (it = trackers_.begin(); it != trackers_.end(); it++) {
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = it->first;
    tfs.child_frame_id = it->second.name;
    tfs.transform.translation.x = it->second.bTh.translation()[0];
    tfs.transform.translation.y = it->second.bTh.translation()[1];
    tfs.transform.translation.z = it->second.bTh.translation()[2];
    Eigen::Quaterniond quat(it->second.bTh.linear());
    tfs.transform.rotation.w = quat.w();
    tfs.transform.rotation.x = quat.x();
    tfs.transform.rotation.y = quat.y();
    tfs.transform.rotation.z = quat.z();
    bc.sendTransform(tfs);
  }
}

// Data callbacks

void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg) {
  /*
  std::unique_lock<std::mutex> lock(mutex_);
  std::vector<deepdive_ros::Lighthouse>::const_iterator it;
  for (it = msg->lighthouses.begin(); it != msg->lighthouses.end(); it++) {
    if (std::find(permitted_lighthouses_.begin(), permitted_lighthouses_.end(),
      it->serial) == permitted_lighthouses_.end()) return;
    lighthouses_.push(*it);
  }
  */
}

void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg) {
  /*
  std::unique_lock<std::mutex> lock(mutex_);
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    if (std::find(permitted_trackers_.begin(), permitted_trackers_.end(),
      it->serial) == permitted_trackers_.end()) return;
    trackers_.push(*it);
  }
  */
}

void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  /*
  std::unique_lock<std::mutex> lock(mutex_);
  if (std::find(permitted_trackers_.begin(), permitted_trackers_.end(),
    msg->header.frame_id) == permitted_trackers_.end()) return;
  if (std::find(permitted_lighthouses_.begin(), permitted_lighthouses_.end(),
    msg->lighthouse) == permitted_lighthouses_.end()) return;
  light_.push(*msg);
  */
}


// HELPER FUNCTIONS FOR CONFIG

bool GetVectorParam(ros::NodeHandle &nh,
  std::string const& name, Eigen::Vector3d & data) {
  std::vector<double> tmp;
  if (!nh.getParam(name, tmp) || tmp.size() != 3)
    return false;
  data[0] = tmp[0];
  data[1] = tmp[1];
  data[2] = tmp[2];
  return true;
}

bool GetQuaternionParam(ros::NodeHandle &nh,
  std::string const& name, Eigen::Quaterniond & data) {
  std::vector<double> tmp;
  if (!nh.getParam(name, tmp) || tmp.size() != 4)
    return false;
  data.x() = tmp[0];
  data.y() = tmp[1];
  data.z() = tmp[2];
  data.w() = tmp[3];
  return true;
}

// MAIN ENTRY POINT

int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_solver");
  ros::NodeHandle nh("~");

  // Get the parent information
  std::vector<std::string> lighthouses;
  if (!nh.getParam("lighthouses", lighthouses))
    ROS_FATAL("Failed to get the parent list.");
  std::vector<std::string>::iterator it;
  for (it = lighthouses.begin(); it != lighthouses.end(); it++) {
    // Temporary storage
    std::string serial;
    if (!nh.getParam(*it + "/serial", serial))
      ROS_FATAL("Failed to get serial parameter.");
    // Set the tracker to body transformation
    Eigen::Vector3d translation;
    if (!GetVectorParam(nh,  *it + "/translation", translation))
      ROS_FATAL("Failed to get translation parameter.");
    Eigen::Quaterniond rotation;
    if (!GetQuaternionParam(nh, *it + "/rotation", rotation))
      ROS_FATAL("Failed to get rotation parameter.");
    // Add the transform
    lighthouses_[serial].name = *it;
    lighthouses_[serial].wTl.translation() = translation;
    lighthouses_[serial].wTl.linear() = rotation.toRotationMatrix();
  }

  // Get the parent information
  std::vector<std::string> trackers;
  if (!nh.getParam("trackers", trackers))
    ROS_FATAL("Failed to get trackers parameter.");
  std::vector<std::string>::iterator jt;
  for (jt = trackers.begin(); jt != trackers.end(); jt++) {
    // Temporary storage
    std::string serial;
    if (!nh.getParam(*jt + "/serial", serial))
      ROS_FATAL("Failed to get serial parameter.");
    // Set the tracker to body transformation
    Eigen::Vector3d translation;
    if (!GetVectorParam(nh, *jt + "/translation", translation))
      ROS_FATAL("Failed to get translation parameter.");
    Eigen::Quaterniond rotation;
    if (!GetQuaternionParam(nh,*jt + "/rotation", rotation))
      ROS_FATAL("Failed to get rotation parameter.");
    // Add the transform
    trackers_[serial].name = *jt;
    trackers_[serial].bTh.translation() = translation;
    trackers_[serial].bTh.linear() = rotation.toRotationMatrix();
  }

  // Get some global information
  if (!nh.getParam("frames/parent", frame_parent_))
    ROS_FATAL("Failed to get frames/parent parameter.");
  if (!nh.getParam("frames/child", frame_child_))
    ROS_FATAL("Failed to get frames/child parameter.");  

  // Get ceres parameters
  if (!nh.getParam("parameters/max_time", param_max_time_))
    ROS_FATAL("Failed to get the refine parameters/max_time parameter.");
  if (!nh.getParam("parameters/max_iterations", param_max_iterations_))
    ROS_FATAL("Failed to get the refine parameters/max_iterations parameter.");
  if (!nh.getParam("parameters/threads", param_threads_))
    ROS_FATAL("Failed to get the refine parameters/threads parameter.");
  if (!nh.getParam("parameters/gradient_tolerance", param_gradient_tolerance_))
    ROS_FATAL("Failed to get the refine parameters/gradient_tolerance parameter.");
  if (!nh.getParam("parameters/debug", param_debug_))
    ROS_FATAL("Failed to get the refine parameters/debug parameter.");

  // Subscribe to tracker and lighthouse updates
  ros::Subscriber sub_tracker  =
    nh.subscribe("/trackers", 10, TrackerCallback);
  ros::Subscriber sub_lighthouse =
    nh.subscribe("/lighthouses", 10, LighthouseCallback);
  ros::Subscriber sub_light =
    nh.subscribe("/light", 10, LightCallback);

  // These won't change
  BroadcastTrackerTransforms();

  // These will change as the solution is refined
  BroadcastLighthouseTransforms();

  // Start a thread to listen to vive
  //std::thread thread(WorkerThread);

  // Block until safe shutdown
  ros::spin();

  // Join the thread
  //thread.join();

  // Success!
  return 0;
}