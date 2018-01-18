/*
  This ROS node listens to data from all trackers in the system and provides
  a global solution to the tracking problem. That is, it solves for the
  relative location of the lighthouses and trackers as a function of time.
*/

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// ROS includes
#include <ros/ros.h>

// For transforms
#include <tf2_ros/transform_broadcaster.h>

// Standard messages
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>

// Non-standard datra messages
#include <deepdive_ros/Light.h>

// Services to get tracker/lighthouse/system config
#include <deepdive_ros/GetTracker.h>
#include <deepdive_ros/GetLighthouse.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// STL includes
#include <algorithm>
#include <thread>
#include <mutex>
#include <string>
#include <map>
#include <queue>

// DATA TYPES

#define MAX_NUM_SENSORS 32
#define MAX_NUM_MOTORS  2

enum {
  CAL_PHASE,
  CAL_TILT,
  CAL_GIB_PHASE,
  CAL_GIB_MAG,
  CAL_CURVE,
  NUM_CAL
};

// Pose and trajectory
struct Pose {
  double val[6];
};
typedef std::map<ros::Time,Pose> Trajectory;

// Tracker data structure
struct Tracker {
  Trajectory estimate;
  double parameters[MAX_NUM_SENSORS][6];
};
typedef std::map<std::string,Tracker> Trackers;

// Lighthouse data structures
struct Lighthouse {
  Pose estimate;
  double parameters[MAX_NUM_MOTORS][NUM_CAL];
};
typedef std::map<std::string,Lighthouse> Lighthouses;

// SHARED DATA and MUTEX LOCK

std::map<std::string,deepdive_ros::Tracker> trackers_;
std::map<std::string,deepdive_ros::Lighthouse> lighthouses_;
std::queue<deepdive_ros::Light> light_;
std::queue<nav_msgs::Odometry> odometry_;
std::mutex mutex_;

// CERES SOLVER THREAD

// The Vive system produces a bundle of angle estimates to a given lighthouse.
// This function predicts these measurements, given an estimate of the location
// of the lighthouse, the location of the tracker and calibration parameters
struct LightCost {
  // Constructor
  explicit LightCost(deepdive_ros::Light const& measurement) :
    measurement_(measurement) {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const lighthouse,  // Lighthouse pose in world frame
                  const T* const cal,         // Lighthouse calibration
                  const T* const tracker,     // Tracker pose in world frame
                  const T* const extrinsics,  // Sensor extrinsics
                  T* residual) const {
    // We are going to do this in the tracker frame, so that the extrinsics
    // calculations are extremely simple additions :)
    static T delta[3], ax[2];
    ceres::AngleAxisRotatePoint(&tracker[3], lighthouse, delta);
    delta[0] += tracker[0];
    delta[1] += tracker[1];
    delta[2] += tracker[2];
    // Get the axis of this measurement
    size_t a = static_cast<size_t>(measurement_.axis);
    // Iterate over all the pluses that were received
    for (size_t i = 0; i < measurement_.pulses.size(); i++) {
      // Get the ID of the sensor
      size_t s = static_cast<size_t>(measurement_.pulses[i].sensor);
      // Move to the sensor frame
      delta[0] += T(extrinsics[3*s+0]);
      delta[1] += T(extrinsics[3*s+1]);
      delta[2] += T(extrinsics[3*s+2]);
      // Calculate the raw angles
      ax[deepdive_ros::Motor::AXIS_HORIZONTAL] = atan(delta[0]/delta[2]);
      ax[deepdive_ros::Motor::AXIS_VERTICAL]= atan(delta[1]/delta[2]);
      // Apply the error correction as needed. I am going to assume that the
      // engineers kept this equation as simple as possible, and infer the
      // meaning of the calibration parameters based on their name. It might be
      // the case that these value are subtracted, rather than added. 
      ax[a] += T(cal[CAL_PHASE]);
      ax[a] += T(cal[CAL_TILT]) * ax[1-a];
      ax[a] += T(cal[CAL_CURVE]) * ax[1-a] * ax[1-a];
      ax[a] += T(cal[CAL_GIB_MAG]) * cos(ax[1-a] + T(cal[CAL_GIB_PHASE]));
      // The residual is the axis of interest minus the predicted value as
      // given by the light measurements
      residual[i] = ax[a] - T(measurement_.pulses[i].angle);
      // Go back to the tracker frame
      delta[0] -= T(extrinsics[3*s+0]);
      delta[1] -= T(extrinsics[3*s+1]);
      delta[2] -= T(extrinsics[3*s+2]);
    }
    // Everything went well
    return true;
  }
 // Internal variables
 private:
  deepdive_ros::Light measurement_;
};

// Just keep solving in the background
void WorkerThread() {
  // Construct and persist the ceres solver problem
  static ceres::Problem problem;
  static Trackers trackers;
  static Lighthouses lighthouses;
  // We only want to keep going if ROS is alive. When 
  while (ros::ok()) {

    ROS_INFO("SOLVING");

    // Lock to avoid contention with primary thread
    mutex_.lock();

    // Copy the lighthouse calibration
    {
      std::map<std::string,deepdive_ros::Lighthouse>::iterator it;
      for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) {
        for (size_t i = 0; i < 2; i++) {
          lighthouses[it->first].parameters[i][CAL_PHASE]
            = it->second.motors[i].phase;
          lighthouses[it->first].parameters[i][CAL_TILT]
            = it->second.motors[i].tilt;
          lighthouses[it->first].parameters[i][CAL_GIB_PHASE]
            = it->second.motors[i].gibphase;
          lighthouses[it->first].parameters[i][CAL_GIB_MAG]
            = it->second.motors[i].gibmag;
          lighthouses[it->first].parameters[i][CAL_CURVE]
            = it->second.motors[i].curve;
        }
      }
      // Hold calibration constant
      problem.SetParameterBlockConstant(
        &lighthouses[it->first].parameters[0][0]);
    }
    // Copy the tracker extrinsics
    {
      std::map<std::string,deepdive_ros::Tracker>::iterator it;
      for (it = trackers_.begin(); it != trackers_.end(); it++) {
        for (size_t i = 0; i < MAX_NUM_SENSORS; i++) {
          if (i < it->second.sensors.size()) {
            // Copy the position
            trackers[it->first].parameters[i][0]
              = it->second.sensors[i].position.x;
            trackers[it->first].parameters[i][1]
              = it->second.sensors[i].position.y;
            trackers[it->first].parameters[i][2]
              = it->second.sensors[i].position.z;
            // Copy the normal
            trackers[it->first].parameters[i][3]
              = it->second.sensors[i].normal.x;
            trackers[it->first].parameters[i][4]
              = it->second.sensors[i].normal.y;
            trackers[it->first].parameters[i][5]
              = it->second.sensors[i].normal.z;
            // Hold extrinsics constant
            problem.SetParameterBlockConstant(
              &trackers[it->first].parameters[i][0]);
          }
        }
      }
    }

    // Iterate over all light measurements, adding a reisdual block for each
    // bundle, which uses the predicted lighthouse and tracker poses to predict
    // the angles to the lighthouses.
    while (light_.size()) {
      // Get the critical information about this measurement
      Lighthouse & lighthouse = lighthouses[light_.front().lighthouse];
      Tracker & tracker = trackers[light_.front().header.frame_id];
      ros::Time & time = light_.front().header.stamp;
      uint8_t & axis = light_.front().axis;
      // Add a residual block to the problem representing this measurent
      ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<LightCost,
        ceres::DYNAMIC, 3, NUM_CAL, 3, MAX_NUM_SENSORS>(
          new LightCost(light_.front()), light_.front().pulses.size());
      problem.AddResidualBlock(cost,
        new ceres::CauchyLoss(0.5),
        &lighthouse.estimate.val[0],
        &lighthouse.parameters[axis][0],
        &tracker.estimate[time].val[0],
        &tracker.parameters[0][0]);
      // Remove the measurment
      light_.pop();
    }

    // We are now finished copying the data, so we can unlock the mutex
    mutex_.unlock();

    // By default keep the initial pose constant to lock down the trajectory
    Trackers::iterator it;
    for (it = trackers.begin(); it != trackers.end(); it++)
      problem.SetParameterBlockConstant(
        &it->second.estimate.begin()->second.val[0]);

    // In the find-phase we call on ceres to solve the problem
    static ceres::Solver::Options options;
    static ceres::Solver::Summary summary;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solve(options, &problem, &summary);

    ROS_INFO("SOLVED");
  }
}

// MESSAGE CALLBACKS

std::vector<std::string> permitted_lighthouses_;
std::vector<std::string> permitted_trackers_;

void LighthouseCallback(deepdive_ros::Lighthouse::ConstPtr const& msg) {
  if (std::find(permitted_lighthouses_.begin(), permitted_lighthouses_.end(),
    msg->serial) == permitted_lighthouses_.end()) return;
  std::unique_lock<std::mutex> lock(mutex_);
  lighthouses_[msg->serial] = *msg;
}

void TrackerCallback(deepdive_ros::Tracker::ConstPtr const& msg) {
  if (std::find(permitted_trackers_.begin(), permitted_trackers_.end(),
    msg->serial) == permitted_trackers_.end()) return;
  std::unique_lock<std::mutex> lock(mutex_);
  trackers_[msg->serial] = *msg;
}

void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  if (std::find(permitted_trackers_.begin(), permitted_trackers_.end(),
    msg->header.frame_id) == permitted_trackers_.end()) return;
  if (std::find(permitted_lighthouses_.begin(), permitted_lighthouses_.end(),
    msg->lighthouse) == permitted_lighthouses_.end()) return;
  std::unique_lock<std::mutex> lock(mutex_);
  light_.push(*msg);
}

// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_solver");
  ros::NodeHandle nh("~");

  // Get the list of permitted devices in the system
  if (!nh.getParam("lighthouses", permitted_lighthouses_))
    ROS_FATAL("Failed to get device list.");
  if (!nh.getParam("trackers", permitted_trackers_))
    ROS_FATAL("Failed to get device list.");

  // Subscribe to tracker and lighthouse updates
  ros::Subscriber sub_tracker  =
    nh.subscribe("/tracker", 10, TrackerCallback);
  ros::Subscriber sub_lighthouse =
    nh.subscribe("/lighthouse", 10, LighthouseCallback);
  ros::Subscriber sub_light =
    nh.subscribe("/light", 10, LightCallback);

  // Start a thread to listen to vive
  std::thread thread(WorkerThread);

  // Block until safe shutdown
  ros::spin();

  // Join the thread
  thread.join();

  // Success!
  return 0;
}