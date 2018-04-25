/*
  This ROS node listens to data from all trackers in the system and provides
  a global solution to the tracking problem. That is, it solves for the
  relative location of the lighthouses and trackers as a function of time.
*/

// ROS includes
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Third-party includes
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>

// Non-standard datra messages
#include <deepdive_ros/Light.h>
#include <deepdive_ros/Lighthouses.h>
#include <deepdive_ros/Trackers.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Ceres and logging
#include <Eigen/Core>
#include <Eigen/Geometry>

// C++ libraries
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

// Shared local code
#include "deepdive.hh"

// GLOBAL PARAMETERS

// List of lighthouses
TrackerMap trackers_;
LighthouseMap lighthouses_;
MeasurementMap measurements_;

// Global strings
std::string calfile_ = "deepdive.tf2";
std::string frame_world_ = "world";
std::string frame_vive_ = "vive";
std::string frame_body_ = "truth";

// Whether to apply corrections
bool correct_ = false;

// What to solve for
bool refine_sensors_ = false;
bool refine_head_ = false;
bool refine_params_ = false;

// Rejection thresholds
int thresh_count_ = 4;
double thresh_angle_ = 60.0;
double thresh_duration_ = 1.0;

// What to weight motion cost relative to the other errors
double weight_light_ = 1.0;
double weight_motion_ = 100.0;

// Solver parameters
ceres::Solver::Options options_;

// Are we running in "offline" mode
bool offline_ = false;

// Should we publish rviz markers
bool visualize_ = true;

// Are we recording right now?
bool recording_ = false;

// World -> vive registatration
double registration_[6];

// Sensor visualization publisher
std::map<std::string, ros::Publisher> pub_sensors_;
std::map<std::string, ros::Publisher> pub_path_;

// Timer for managing offline
ros::Timer timer_;

// CERES SOLVER

// Helper function to apply a transform b = Ra + t
template <typename T> inline
void TransformInPlace(const T transform[6], T x[3]) {
  T tmp[3];
  ceres::AngleAxisRotatePoint(&transform[3], x, tmp);
  x[0] = tmp[0] + transform[0];
  x[1] = tmp[1] + transform[1];
  x[2] = tmp[2] + transform[2];
}

// Helper function to invert a transform a = R'(b - t)
template <typename T> inline
void InverseTransformInPlace(const T transform[6], T x[3]) {
  T aa[3], tmp[3];
  tmp[0] = x[0] - transform[0];
  tmp[1] = x[1] - transform[1];
  tmp[2] = x[2] - transform[2];
  aa[0] = -transform[3];
  aa[1] = -transform[4];
  aa[2] = -transform[5];
  ceres::AngleAxisRotatePoint(aa, tmp, x);
}

// Residual error between predicted anfgles to a lighthouse
struct LightCost {
  explicit LightCost(deepdive_ros::Light const& light) : light_(light) {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const vTl,         // Lighthouse -> vive
                  const T* const vTt,         // Tracking -> vive
                  const T* const sensors,     // Lighthouse calibration
                  const T* const params,      // Tracker extrinsics
                  T* residual) const {
    // The position of the sensor
    T x[3], angle[2];
    // Get the light axis
    size_t a = light_.axis;
    // Iterate over all measurements
    for (size_t i = 0; i < light_.pulses.size(); i++) {
      // Get the sensor id
      size_t s = light_.pulses[i].sensor;
      // Get the sensor position in the tracking frame
      x[0] = sensors[6*s+0];
      x[1] = sensors[6*s+1];
      x[2] = sensors[6*s+2];
      // Project the sensor position into the lighthouse frame
      TransformInPlace(vTt, x);           // tracking -> vive
      InverseTransformInPlace(vTl, x);    // vive -> lighthouse
      // Predict the angles
      angle[0] = atan2(x[0], x[2]);
      angle[1] = atan2(x[1], x[2]);
      // Apply the error correction as needed. I am going to assume that the
      // engineers kept this equation as simple as possible, and infer the
      // meaning of the calibration parameters based on their name. It might
      // be the case that these value are subtracted, rather than added.
      if (correct_) { 
        angle[a] += T(params[PARAM_PHASE]);
        angle[a] += T(params[PARAM_TILT]) * angle[1-a];
        angle[a] += T(params[PARAM_CURVE]) * angle[1-a] * angle[1-a];
        angle[a] += T(params[PARAM_GIB_MAG])
                     * cos(angle[1-a] + T(params[PARAM_GIB_PHASE]));
      }
      // The residual angle error for the specific axis
      residual[i] = T(weight_light_) * (angle[a] - T(light_.pulses[i].angle));
    }
    // Everything went well
    return true;
  }
 // Internal variables
 private:
  deepdive_ros::Light light_;
};

// Residual error between sequential poses
struct MotionCost {
  explicit MotionCost() {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const prev_vTt,   // PREV Body -> world (pos xy)
                  const T* const next_vTt,   // NEXT Body -> world (rot z)
                  T* residual) const {
    for (size_t i = 0; i < 6; i++)
      residual[i] = T(weight_motion_) * (next_vTt[i] - prev_vTt[i]);
    return true;
  }
};

// Jointly solve
bool Solve() {
  // Create the ceres problem
  ceres::Problem problem;

  // Check that we have enough measurements
  if (measurements_.empty()) {
    ROS_WARN("No measurements received, so cannot solve the problem.");
    return false;
  } else {
    double t = (measurements_.rbegin()->first
      - measurements_.begin()->first).toSec();
    ROS_INFO_STREAM("Processing " << measurements_.size()
      << " measurements running for " << t << " seconds from "
      << measurements_.begin()->first << " to "
      << measurements_.rbegin()->first);
  }

  // Clear all trajectories
  {
    ROS_INFO("Clearing all existing trajectories.");
    TrackerMap::iterator jt;
    for (jt = trackers_.begin(); jt != trackers_.end(); jt++)
      jt->second.vTt.clear();
  }

  // Keep a log of the data used
  std::map<std::string, size_t> l_tally;
  std::map<std::string, size_t> t_tally;

  // Create a correction if required
  {
    ROS_INFO("Adding measurements.");
    MeasurementMap::iterator mt;
    for (mt = measurements_.begin(); mt != measurements_.end(); mt++) {
      // Get references to the lighthouse, tracker and axis
      Tracker & tracker = trackers_[mt->second.light.header.frame_id];
      Lighthouse & lighthouse = lighthouses_[mt->second.light.lighthouse];
      size_t const& axis = mt->second.light.axis;
      // Register that we have received from this lighthouse and tracker
      l_tally[mt->second.light.lighthouse]++;
      t_tally[mt->second.light.header.frame_id]++;
      // This only works because the measurement time is monotonically
      for (size_t i = 0; i < 6; i++)
        tracker.vTt[mt->first][i] = 0.0;
      std::map<ros::Time, double[6]>::iterator vTt_c = std::prev(tracker.vTt.end());
      std::map<ros::Time, double[6]>::iterator vTt_p = std::prev(vTt_c);
      // Create a cost function
      ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<LightCost,
        ceres::DYNAMIC, 6, 6, NUM_SENSORS * 6, NUM_PARAMS>(
          new LightCost(mt->second.light), mt->second.light.pulses.size());
      // Add a residual block
      problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
        reinterpret_cast<double*>(lighthouse.vTl),
        reinterpret_cast<double*>(vTt_c->second),
        reinterpret_cast<double*>(tracker.sensors),
        reinterpret_cast<double*>(lighthouse.params[axis]));
      // If we are dynamic and there
      if (vTt_p != tracker.vTt.end()) {

        // Create a cost function to represent motion
        ceres::CostFunction* cost = new ceres::AutoDiffCostFunction
              <MotionCost, 6, 6, 6>(new MotionCost());
        // Add a residual block for error
        problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
          reinterpret_cast<double*>(vTt_p->second),
          reinterpret_cast<double*>(vTt_c->second));
      }
    }
  }

  // Print out how much data we received
  {
    ROS_INFO_STREAM("We received this much data from each lighthouse:");
    LighthouseMap::iterator it;
    for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) 
      ROS_INFO_STREAM("- ID " << it->first << ": " << l_tally[it->first]);
    ROS_INFO_STREAM("We received this much data from each tracker:");
    TrackerMap::iterator jt;
    for (jt = trackers_.begin(); jt != trackers_.end(); jt++) 
      ROS_INFO_STREAM("- ID " << jt->first << ": " << t_tally[jt->first]);
    if (l_tally.size() != lighthouses_.size()) {
      ROS_WARN("We didn't receive data from all lighthouses. Aborting");
      return false;
    } else if (t_tally.size() != trackers_.size()) {
      ROS_WARN("We didn't receive data from all trackers. Aborting");
      return false;
    }
  }

  // Set the lighthouse -> vive transform for the first lighthouse to be
  // the identity, and optionally allow other things to be refined.
  {
    ROS_INFO_STREAM("Setting blocks constant if necessary");
    LighthouseMap::iterator it;
    for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) {
      if (it == lighthouses_.begin()) {
        for (size_t i = 0; i < 6; i++)
          it->second.vTl[0] = 0;
        problem.SetParameterBlockConstant(it->second.vTl);
      }
      if (!refine_params_) {
        for (size_t i = 0; i < NUM_MOTORS; i++) {
          problem.SetParameterBlockConstant(it->second.params[i]);
        }
      }
    }
    TrackerMap::iterator jt;
    for (jt = trackers_.begin(); jt != trackers_.end(); jt++)  {
      if (!refine_head_) {
        problem.SetParameterBlockConstant(jt->second.tTh);
      }
      if (!refine_sensors_) {
        problem.SetParameterBlockConstant(jt->second.sensors);
      }
    }
  }

  // Attempt to find a solution
  {
    ROS_INFO_STREAM("Attempting to solve problem");
    ceres::Solver::Summary summary;
    ceres::Solve(options_, &problem, &summary);
    if (summary.IsSolutionUsable()) {
      // Print summary of important measurements
      ROS_INFO("Usable solution found.");
      LighthouseMap::iterator it, jt;
      for (it = lighthouses_.begin(); it != lighthouses_.end(); it++)  {
        for (jt = std::next(it); jt != lighthouses_.end(); jt++)  {
          double dl = 0;
          for (size_t i = 0; i < 3; i++) {
            dl += (it->second.vTl[i] - jt->second.vTl[i]) *
                  (it->second.vTl[i] - jt->second.vTl[i]);
          }
          ROS_INFO_STREAM("Distance between lighthouses " << it->first
            << " and " << jt->first << " is " << sqrt(dl) << " meters");
        }
      }
      // Publish the new solution
      SendTransforms(frame_world_, frame_vive_, frame_body_,
        registration_, lighthouses_, trackers_);
      // Write the solution to a config file
      if (WriteConfig(calfile_, frame_world_, frame_vive_, frame_body_,
        registration_, lighthouses_, trackers_))
        ROS_INFO_STREAM("Calibration written to " << calfile_);
      else
        ROS_INFO_STREAM("Could not write calibration to" << calfile_);
      // Print the trajectory of the body-frame in the world-frame
      if (visualize_) {
        TrackerMap::iterator jt;
        for (jt = trackers_.begin(); jt != trackers_.end(); jt++) {
          nav_msgs::Path msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = frame_vive_;
          std::map<ros::Time, double[6]>::iterator it;
          for (it = jt->second.vTt.begin(); it != jt->second.vTt.end(); it++) {
            geometry_msgs::PoseStamped ps;
            Eigen::Vector3d v(it->second[3], it->second[4], it->second[5]);
            Eigen::AngleAxisd aa;
            if (v.norm() > 0) {
              aa.angle() = v.norm();
              aa.axis() = v.normalized();
            }
            Eigen::Quaterniond q(aa);
            ps.header.stamp = it->first;
            ps.header.frame_id = frame_vive_;
            ps.pose.position.x = it->second[0];
            ps.pose.position.y = it->second[1];
            ps.pose.position.z = it->second[2];
            ps.pose.orientation.w = q.w();
            ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z();
            msg.poses.push_back(ps);
          }
          pub_path_[jt->first].publish(msg);
        }
      }
      // Solution is usable
      return true;
    }
  }
  ROS_INFO("Solution is not usable.");
  // Solution is not usable
  return false;
}

// MESSAGE CALLBACKS

void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  // Reset the timer use din offline mode to determine the end of experiment
  timer_.stop();
  timer_.start();
  // Check that we are recording and that the tracker/lighthouse is ready
  if (!recording_ ||
    trackers_.find(msg->header.frame_id) == trackers_.end() ||
    lighthouses_.find(msg->lighthouse) == lighthouses_.end() ||
    !trackers_[msg->header.frame_id].ready ||
    !lighthouses_[msg->lighthouse].ready) return;
  // Copy over the data
  size_t deleted = 0;
  deepdive_ros::Light data = *msg;
  std::vector<deepdive_ros::Pulse>::iterator it = data.pulses.end();
  while (it-- > data.pulses.begin()) {
    if (it->angle > thresh_angle_ / 57.2958 &&    // Check angle
        it->duration < thresh_duration_ / 1e-6) {  // Check duration
      it = data.pulses.erase(it);
      deleted++;
    }
  }
  if (data.pulses.size() < thresh_count_)
    return; 
  // Add the data
  measurements_[ros::Time::now()].light = data;
}

bool TriggerCallback(std_srvs::Trigger::Request  &req,
                     std_srvs::Trigger::Response &res)
{
  if (!recording_) {
    res.success = true;
    res.message = "Recording started.";
  }
  if (recording_) {
    // Solve the problem
    res.success = Solve();
    if (res.success)
      res.message = "Recording stopped. Solution found.";
    else
      res.message = "Recording stopped. Solution not found.";
    // Clear all the data and corrections
    measurements_.clear();
  }
  // Toggle recording state
  recording_ = !recording_;
  // Success
  return true;
}

// Fake a trigger when the timer expires
void TimerCallback(ros::TimerEvent const& event) {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  TriggerCallback(req, res);
}

// Called when a new lighthouse appears
void NewLighthouseCallback(LighthouseMap::iterator lighthouse) {
  ROS_INFO_STREAM("Found lighthouse " << lighthouse->first);
}

// Called when a new tracker appears
void NewTrackerCallback(TrackerMap::iterator tracker) {
  ROS_INFO_STREAM("Found tracker " << tracker->first);
  if (visualize_) {
    visualization_msgs::MarkerArray msg;
    std::map<std::string, Tracker>::iterator jt;
    for (jt = trackers_.begin(); jt != trackers_.end(); jt++)  {
      for (uint16_t i = 0; i < NUM_SENSORS; i++) {
        // All this code just to convert a normal to a quaternion
        Eigen::Vector3d vfwd(jt->second.sensors[6*i+3],
          jt->second.sensors[6*i+4], jt->second.sensors[6*i+5]);
        if (vfwd.norm() > 0) {
          Eigen::Vector3d vdown(0.0, 0.0, 1.0);
          Eigen::Vector3d vright = vdown.cross(vfwd);
          vfwd = vfwd.normalized();
          vright = vright.normalized();
          vdown = vdown.normalized();
          Eigen::Matrix3d dcm;
          dcm << vfwd.x(), vright.x(), vdown.x(),
                 vfwd.y(), vright.y(), vdown.y(),
                 vfwd.z(), vright.z(), vdown.z();
          Eigen::Quaterniond q(dcm);
          // Now plot an arrow representing the normal
          visualization_msgs::Marker marker;
          marker.header.frame_id = jt->first + "/light";
          marker.header.stamp = ros::Time::now();
          marker.ns = jt->first;
          marker.id = i;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = jt->second.sensors[6*i+0];
          marker.pose.position.y = jt->second.sensors[6*i+1];
          marker.pose.position.z = jt->second.sensors[6*i+2];
          marker.pose.orientation.w = q.w();
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
          marker.scale.x = 0.010;
          marker.scale.y = 0.001;
          marker.scale.z = 0.001;
          marker.color.a = 1.0;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          msg.markers.push_back(marker);
        }
      }
      pub_sensors_[tracker->first].publish(msg);
    }
  }
}

// MAIN ENTRY POINT

int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_calibration");
  ros::NodeHandle nh("~");

  // If we are in offline mode when we will replay the data back at 10x the
  // speed, using it all to find a calibration solution for both the body
  // as a function of  time and the lighthouse positions.
  if (!nh.getParam("offline", offline_))
    ROS_FATAL("Failed to get if we are running in offline mode.");
  if (offline_) {
    ROS_INFO("We are in offline mode. Speeding up bag replay by 10x");
    recording_ = true;
  }

  // Reset the registration information
  for (size_t i = 0; i < 6; i++)
    registration_[i] = 0;

  // Get the parent information
  if (!nh.getParam("calfile", calfile_))
    ROS_FATAL("Failed to get the calfile file.");

  // Get some global information
  if (!nh.getParam("frames/world", frame_world_))
    ROS_FATAL("Failed to get frames/world parameter.");  
  if (!nh.getParam("frames/vive", frame_vive_))
    ROS_FATAL("Failed to get frames/vive parameter.");  
  if (!nh.getParam("frames/body", frame_body_))
    ROS_FATAL("Failed to get frames/body parameter.");  

  // Get the thresholds
  if (!nh.getParam("thresholds/count", thresh_count_))
    ROS_FATAL("Failed to get threshods/count parameter.");
  if (!nh.getParam("thresholds/angle", thresh_angle_))
    ROS_FATAL("Failed to get thresholds/angle parameter.");
  if (!nh.getParam("thresholds/duration", thresh_duration_))
    ROS_FATAL("Failed to get thresholds/duration parameter.");

  // What to refine
  if (!nh.getParam("refine/sensors", refine_sensors_))
    ROS_FATAL("Failed to get refine/sensors parameter.");
  if (!nh.getParam("refine/head", refine_head_))
    ROS_FATAL("Failed to get refine/head parameter.");
  if (!nh.getParam("refine/params", refine_params_))
    ROS_FATAL("Failed to get refine/params parameter.");

  // What weights to use
  if (!nh.getParam("weight/light", weight_light_))
    ROS_FATAL("Failed to get weight/light parameter.");
  if (!nh.getParam("weight/motion", weight_motion_))
    ROS_FATAL("Failed to get weight/motion parameter.");

  // Whether to apply light corrections
  if (!nh.getParam("correct", correct_))
    ROS_FATAL("Failed to get correct parameter.");
  if (!correct_)
    refine_params_ = false;

  // Define the ceres problem
  ceres::Solver::Options options;
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  if (!nh.getParam("solver/max_time", options_.max_solver_time_in_seconds))
    ROS_FATAL("Failed to get the solver/max_time parameter.");
  if (!nh.getParam("solver/max_iterations", options_.max_num_iterations))
    ROS_FATAL("Failed to get the solver/max_iterations parameter.");
  if (!nh.getParam("solver/threads", options_.num_threads))
    ROS_FATAL("Failed to get the solver/threads parameter.");
  if (!nh.getParam("solver/debug", options_.minimizer_progress_to_stdout))
    ROS_FATAL("Failed to get the solver/debug parameter.");

  // Visualization option
  if (!nh.getParam("visualize", visualize_))
    ROS_FATAL("Failed to get the visualize parameter.");

  // Get the parent information
  std::vector<std::string> lighthouses;
  if (!nh.getParam("lighthouses", lighthouses))
    ROS_FATAL("Failed to get the lighthouse list.");
  std::vector<std::string>::iterator it;
  for (it = lighthouses.begin(); it != lighthouses.end(); it++) {
    std::string serial;
    if (!nh.getParam(*it + "/serial", serial))
      ROS_FATAL("Failed to get the lighthouse serial.");
    std::vector<double> transform;
    if (!nh.getParam(*it + "/transform", transform))
      ROS_FATAL("Failed to get the lighthouse transform.");
    if (transform.size() != 7) {
      ROS_FATAL("Failed to parse lighthouse transform.");
      continue;
    }
    Eigen::Quaterniond q(transform[6], transform[3], transform[4], transform[5]);
    Eigen::AngleAxisd aa(q);
    lighthouses_[serial].vTl[0] = transform[0];
    lighthouses_[serial].vTl[1] = transform[1];
    lighthouses_[serial].vTl[2] = transform[2];
    lighthouses_[serial].vTl[3] = aa.angle() * aa.axis()[0];
    lighthouses_[serial].vTl[4] = aa.angle() * aa.axis()[1];
    lighthouses_[serial].vTl[5] = aa.angle() * aa.axis()[2];
    lighthouses_[serial].ready = false;
  }

  // Get the parent information
  std::vector<std::string> trackers;
  if (!nh.getParam("trackers", trackers))
    ROS_FATAL("Failed to get the tracker list.");
  std::vector<std::string>::iterator jt;
  for (jt = trackers.begin(); jt != trackers.end(); jt++) {
    std::string serial;
    if (!nh.getParam(*jt + "/serial", serial))
      ROS_FATAL("Failed to get the tracker serial.");
    std::vector<double> transform;
    if (!nh.getParam(*jt + "/transform", transform))
      ROS_FATAL("Failed to get the tracker transform.");
    if (transform.size() != 7) {
      ROS_FATAL("Failed to parse tracker transform.");
      continue;
    }
    Eigen::Quaterniond q(
      transform[6],  // qw
      transform[3],  // qx
      transform[4],  // qy
      transform[5]); // qz
    Eigen::AngleAxisd aa(q);
    trackers_[serial].bTh[0] = transform[0];
    trackers_[serial].bTh[1] = transform[1];
    trackers_[serial].bTh[2] = transform[2];
    trackers_[serial].bTh[3] = aa.angle() * aa.axis()[0];
    trackers_[serial].bTh[4] = aa.angle() * aa.axis()[1];
    trackers_[serial].bTh[5] = aa.angle() * aa.axis()[2];
    trackers_[serial].ready = false;
    // Publish sensor location and body trajectory 
    pub_sensors_[serial] = nh.advertise<visualization_msgs::MarkerArray>(
      "/sensors/" + *jt, 10, true);
    pub_path_[serial] = nh.advertise<nav_msgs::Path>(
      "/path/" + *jt, 10, true);
  }

  // If reading the configuration file results in inserting the correct
  // number of static transforms into the problem, then we can publish
  // the solution for use by other entities in the system.
  if (ReadConfig(calfile_, frame_world_, frame_vive_, frame_body_,
    registration_, lighthouses_, trackers_)) {
    ROS_INFO("Read transforms from calibration");
  } else {
    ROS_INFO("Could not read calibration file");
  }
  SendTransforms(frame_world_, frame_vive_, frame_body_,
    registration_, lighthouses_, trackers_);

  // Subscribe to tracker and lighthouse updates
  ros::Subscriber sub_tracker  = 
    nh.subscribe<deepdive_ros::Trackers>("/trackers", 1000, std::bind(
      TrackerCallback, std::placeholders::_1, std::ref(trackers_),
        NewTrackerCallback));
  ros::Subscriber sub_lighthouse = 
    nh.subscribe<deepdive_ros::Lighthouses>("/lighthouses", 1000, std::bind(
      LighthouseCallback, std::placeholders::_1, std::ref(lighthouses_),
        NewLighthouseCallback));
  ros::Subscriber sub_light =
    nh.subscribe("/light", 1000, LightCallback);
  ros::ServiceServer service =
    nh.advertiseService("/trigger", TriggerCallback);

  // Setup a timer to automatically trigger solution on end of experiment
  timer_ = nh.createTimer(ros::Duration(1.0), TimerCallback, true, false);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}