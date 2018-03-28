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
LighthouseMap lighthouses_;
TrackerMap trackers_;
MeasurementMap measurements_;
CorrectionMap corrections_;

// Global strings
std::string calfile_ = "deepdive.tf2";
std::string frame_parent_ = "world";
std::string frame_child_ = "truth";
std::string frame_estimate_ = "body";

// Whether to apply corrections
bool correct_ = false;

// Reesolution of the pose graph
double resolution_ = 0.1;

// What to solve for
bool refine_lighthouses_ = true;
bool refine_trajectory_ = false;
bool refine_extrinsics_ = false;
bool refine_sensors_ = false;
bool refine_head_ = false;
bool refine_params_ = false;

// Rejection thresholds
int thresh_count_ = 4;
double thresh_angle_ = 60.0;
double thresh_duration_ = 1.0;
double thresh_correction_ = 0.1;

// What to weight motion cost relative to the other errors
double weight_light_ = 1.0;
double weight_correction_ = 1.0;
double weight_motion_ = 100.0;

// Solver parameters
ceres::Solver::Options options_;

// Are we running in "offline" mode
bool offline_ = false;

// Should we publish rviz markers
bool visualize_ = true;

// Are we recording right now?
bool recording_ = false;

// Force the body frame to move on a plane
bool force2d_ = false;

// Sensor visualization publisher
ros::Publisher pub_sensors_;
ros::Publisher pub_path_;
ros::Publisher pub_corr_;

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
  bool operator()(const T* const wTl,         // Lighthouse -> world
                  const T* const wTb_pos_xy,  // Body -> world (pos xy)
                  const T* const wTb_pos_z,   // Body -> world (pos z)
                  const T* const wTb_rot_xy,  // Body -> world (rot xy)
                  const T* const wTb_rot_z,   // Body -> world (rot z)
                  const T* const bTh,         // Head -> body
                  const T* const tTh,         // Head -> tracking (light)
                  const T* const sensors,     // Lighthouse calibration
                  const T* const params,      // Tracker extrinsics
                  T* residual) const {
    // The position of the sensor
    T x[3], y[3], angle[2], wTb[6];
    // Reconstruct a transform from the components
    wTb[0] = wTb_pos_xy[0];
    wTb[1] = wTb_pos_xy[1];
    wTb[2] = wTb_pos_z[0];
    wTb[3] = wTb_rot_xy[0];
    wTb[4] = wTb_rot_xy[1];
    wTb[5] = wTb_rot_z[0];
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
      y[0] = sensors[6*s+3];
      y[1] = sensors[6*s+4];
      y[2] = sensors[6*s+5];
      // Project the sensor position into the lighthouse frame
      InverseTransformInPlace(tTh, x);    // light -> head
      TransformInPlace(bTh, x);           // head -> body
      TransformInPlace(wTb, x);           // body -> world
      InverseTransformInPlace(wTl, x);    // world -> lighthouse
      // Project the sensor normal into the lighthouse frame
      InverseTransformInPlace(tTh, y);    // light -> head
      TransformInPlace(bTh, y);           // head -> body
      TransformInPlace(wTb, y);           // body -> world
      InverseTransformInPlace(wTl, y);    // world -> lighthouse
      // Predict the angles
      angle[0] = atan2(x[0], x[2]);
      angle[1] = atan2(x[1], x[2]);
      // Apply the error correction as needed. I am going to assume that the
      // engineers kept this equation as simple as possible, and infer the
      // meaning of the calibration parameters based on their name. It might
      // be the case that these value are subtracted, rather than added.
      if (correct_) { 
        angle[a] -= T(params[PARAM_PHASE]);
        angle[a] -= T(params[PARAM_TILT]) * angle[1-a];
        angle[a] -= T(params[PARAM_CURVE]) * angle[1-a] * angle[1-a];
        angle[a] -= T(params[PARAM_GIB_MAG])
                     * cos(angle[1-a] + T(params[PARAM_GIB_PHASE]));
      }
      // The residual angle error for the specific axis
      residual[i] = angle[a] - T(light_.pulses[i].angle);
    }
    // Weight
    for (size_t i = 0; i < light_.pulses.size(); i++)
      residual[i] *= T(weight_light_);
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
  bool operator()(const T* const prev_pos_xy,  // PREV Body -> world (pos xy)
                  const T* const prev_pos_z,   // PREV Body -> world (pos z)
                  const T* const prev_rot_xy,  // PREV Body -> world (rot xy)
                  const T* const prev_rot_z,   // PREV Body -> world (rot z)
                  const T* const next_pos_xy,  // NEXT Body -> world (pos xy)
                  const T* const next_pos_z,   // NEXT Body -> world (pos z)
                  const T* const next_rot_xy,  // NEXT Body -> world (rot xy)
                  const T* const next_rot_z,   // NEXT Body -> world (rot z)
                  T* residual) const {
    residual[0] =  prev_pos_xy[0] - next_pos_xy[0];
    residual[1] =  prev_pos_xy[1] - next_pos_xy[1];
    residual[2] =  prev_pos_z[0] - next_pos_z[0];
    residual[3] =  prev_rot_xy[0] - next_rot_xy[0];
    residual[4] =  prev_rot_xy[1] - next_rot_xy[1];
    residual[5] =  prev_rot_z[0] - next_rot_z[0];
    for (size_t i = 0; i < 6; i++)
      residual[i] *= T(weight_motion_);
    return true;
  }
};

// Correct a state estimate with an EKF measurement
struct CorrectionCost {
  explicit CorrectionCost(double wTb[6]) {
    for (size_t i = 0; i < 6; i++)
      wTb_[i] = wTb[i];
  }
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const pos_xy,  // Body -> world (pos xy)
                  const T* const pos_z,   // Body -> world (pos z)
                  const T* const rot_xy,  // Body -> world (rot xy)
                  const T* const rot_z,   // Body -> world (rot z)
                  T* residual) const {
    residual[0] =  pos_xy[0] - wTb_[0];
    residual[1] =  pos_xy[1] - wTb_[1];
    residual[2] =  pos_z[0] - wTb_[2];
    residual[3] =  rot_xy[0] - wTb_[3];
    residual[4] =  rot_xy[1] - wTb_[4];
    residual[5] =  rot_z[0] - wTb_[5];
    // Apply the weightind
    for (size_t i = 0; i < 6; i++)
      residual[i] *= T(weight_correction_);
    return true;
  }
 private:
  double wTb_[6];
};

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

  // Create a correction if required
  if (corrections_.empty()) {
    ROS_INFO("No corrections, so locking initial pose to origin");
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = measurements_.begin()->first;
    tfs.header.frame_id = frame_parent_;
    tfs.child_frame_id = frame_child_;
    tfs.transform.translation.x = 0.0;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.0;
    tfs.transform.rotation.w = 1.0;
    tfs.transform.rotation.x = 0.0;
    tfs.transform.rotation.y = 0.0;
    tfs.transform.rotation.z = 0.0;
    corrections_[measurements_.begin()->first] = tfs;
  } else {
    double t = (corrections_.rbegin()->first
      - corrections_.begin()->first).toSec();
    ROS_INFO_STREAM("Processing " << corrections_.size()
      << " corrections running for " << t << " seconds from "
      << corrections_.begin()->first << " to "
      << corrections_.rbegin()->first);
  }

  // Tally the lighthouse and tracker measurements
  std::map<std::string, size_t> l_tally;
  std::map<std::string, size_t> t_tally;

  // Calculate the average height if needed
  double height = 0.0;
  double hcount = 0.0;

  // Store the transforms
  std::map<ros::Time, double[6]> wTb;

  // Iterate over corrections
  CorrectionMap::iterator kt, kt_p;
  kt_p = corrections_.end();
  for (kt = corrections_.begin(); kt != corrections_.end(); kt++) {
    // Find the closest measurement to this correction and use it
    MeasurementMap::iterator mt, mt_p;
    mt = measurements_.lower_bound(kt->first);
    if (mt == measurements_.end())
      continue;
    if (mt != measurements_.begin()) {
      mt_p = std::prev(mt);
      if (kt->first - mt_p->first < mt->first - kt->first)
        mt = mt_p;
    }
    if (fabs((kt->first - mt->first).toSec()) > thresh_correction_)
      continue;
    // Avoid super high freuqency samples
    if (kt != corrections_.begin())
      if ((kt->first - kt_p->first).toSec() < resolution_)
        continue;
    // Set the default value
    Eigen::Quaterniond q(
      kt->second.transform.rotation.w,
      kt->second.transform.rotation.x,
      kt->second.transform.rotation.y,
      kt->second.transform.rotation.z);
    Eigen::AngleAxisd aa(q);
    wTb[kt->first][0] = kt->second.transform.translation.x;
    wTb[kt->first][1] = kt->second.transform.translation.y;
    wTb[kt->first][2] = kt->second.transform.translation.z;
    wTb[kt->first][3] = aa.angle() * aa.axis()[0];
    wTb[kt->first][4] = aa.angle() * aa.axis()[1];
    wTb[kt->first][5] = aa.angle() * aa.axis()[2];
    // Add the height data from the measurement
    height += kt->second.transform.translation.z;
    hcount += 1.0;
    // Get references to the lighthouse, tracker and axis
    Tracker & tracker = trackers_[mt->second.light.header.frame_id];
    Lighthouse & lighthouse = lighthouses_[mt->second.light.lighthouse];
    size_t axis = mt->second.light.axis;
    // Register that we have received from this lighthouse
    l_tally[mt->second.light.lighthouse]++;
    t_tally[mt->second.light.header.frame_id]++;
    // Add the cost function
    ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<LightCost,
      ceres::DYNAMIC, 6, 2, 1, 2, 1, 6, 6, NUM_SENSORS * 6, NUM_PARAMS>(
        new LightCost(mt->second.light), mt->second.light.pulses.size());
    // Add the residual block
    problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
      reinterpret_cast<double*>(lighthouse.wTl),
      reinterpret_cast<double*>(&wTb[kt->first][0]),  // pos: xy
      reinterpret_cast<double*>(&wTb[kt->first][2]),  // pos: z
      reinterpret_cast<double*>(&wTb[kt->first][3]),  // rot: xy
      reinterpret_cast<double*>(&wTb[kt->first][5]),  // rot: z
      reinterpret_cast<double*>(tracker.bTh),
      reinterpret_cast<double*>(tracker.tTh),
      reinterpret_cast<double*>(tracker.sensors),
      reinterpret_cast<double*>(lighthouse.params[axis]));
    // THIS CODE BELOW IS CHEATING. DON"T ENABLE IT AGAIN.
    /*
    if (refine_trajectory_) {
      // Add the correction
      ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<
        CorrectionCost, 6, 2, 1, 2, 1>(new CorrectionCost(wTb[kt->first]));
      problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
        reinterpret_cast<double*>(&wTb[kt->first][0]),    // pos: xy
        reinterpret_cast<double*>(&wTb[kt->first][2]),    // pos: z
        reinterpret_cast<double*>(&wTb[kt->first][3]),    // rot: xy
        reinterpret_cast<double*>(&wTb[kt->first][5]));   // rot: z
      // Add the motion cost
      if (kt_p != corrections_.end()) {
        ceres::CostFunction* cost = new ceres::AutoDiffCostFunction
          <MotionCost, 6, 2, 1, 2, 1, 2, 1, 2, 1>(new MotionCost());
        problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
        reinterpret_cast<double*>(&wTb[kt_p->first][0]),  // pos: xy
        reinterpret_cast<double*>(&wTb[kt_p->first][2]),  // pos: z
        reinterpret_cast<double*>(&wTb[kt_p->first][3]),  // rot: xy
        reinterpret_cast<double*>(&wTb[kt_p->first][5]),  // rot: z
        reinterpret_cast<double*>(&wTb[kt->first][0]),    // pos: xy
        reinterpret_cast<double*>(&wTb[kt->first][2]),    // pos: z
        reinterpret_cast<double*>(&wTb[kt->first][3]),    // rot: xy
        reinterpret_cast<double*>(&wTb[kt->first][5]));   // rot: z
      }
    }
    */
    // Set the kt_p
    kt_p = kt;
  }

  // Get the mean height
  height = (hcount > 0 ? height / hcount : 0.0);

  // Print out how much data we received
  if (l_tally.size() != lighthouses_.size()) {
    ROS_WARN("We didn't receive data from all lighthouses. Aborting");
    return false;
  } else if (t_tally.size() != trackers_.size()) {
    ROS_WARN("We didn't receive data from all trackers. Aborting");
    return false;
  } else {
    ROS_INFO_STREAM("We received this much data from each lighthouse:");
    LighthouseMap::iterator it;
    for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) 
      ROS_INFO_STREAM("- ID " << it->first << ": " << l_tally[it->first]);
    ROS_INFO_STREAM("We received this much data from each tracker:");
    TrackerMap::iterator jt;
    for (jt = trackers_.begin(); jt != trackers_.end(); jt++) 
      ROS_INFO_STREAM("- ID " << jt->first << ": " << t_tally[jt->first]);
  }

  // If we want the trajectory refined at the same time as the lighthouse
  // positions. This is completely optional
  if (refine_trajectory_) {
    // In 2D mode we lock the roll and pitch to zero and the height to some
    // fixed value. If we had no corrections the height is zero, otherwise
    // we take the  mean height over all correction values 
    if (force2d_) {
      ROS_INFO_STREAM("Trajectory refined in 2D with fixed height " << height);
      std::map<ros::Time, double[6]>::iterator it;
      for (it = wTb.begin(); it != wTb.end(); it++) {
        it->second[2] = height;   // Height
        problem.SetParameterBlockConstant(&it->second[2]);
        it->second[3] = 0.0;      // Roll
        it->second[4] = 0.0;      // Pitch
        problem.SetParameterBlockConstant(&it->second[3]);
      }
    } else {
      ROS_INFO("Trajectory refined in 3D");
    }
  } else {
    ROS_INFO("Trajectory not refined");
    std::map<ros::Time, double[6]>::iterator it;
    for (it = wTb.begin(); it != wTb.end(); it++) {
      problem.SetParameterBlockConstant(&it->second[0]);
      problem.SetParameterBlockConstant(&it->second[2]);
      problem.SetParameterBlockConstant(&it->second[3]);
      problem.SetParameterBlockConstant(&it->second[5]);
    }
  }

  // Make a subset of parameter blocks constant if we don't want to refine
  LighthouseMap::iterator it;
  for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) {
    if (!refine_lighthouses_)
      problem.SetParameterBlockConstant(it->second.wTl);
    if (!refine_params_)
      for (size_t i = 0; i < NUM_MOTORS; i++)
        problem.SetParameterBlockConstant(it->second.params[i]);
  }
  TrackerMap::iterator jt;
  for (jt = trackers_.begin(); jt != trackers_.end(); jt++)  {
    if (!refine_extrinsics_)
      problem.SetParameterBlockConstant(jt->second.bTh);
    if (!refine_head_)
      problem.SetParameterBlockConstant(jt->second.tTh);
    if (!refine_sensors_)
      problem.SetParameterBlockConstant(jt->second.sensors);
  }

  // Get a solution
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
          dl += (it->second.wTl[i] - jt->second.wTl[i]) *
                (it->second.wTl[i] - jt->second.wTl[i]);
        }
        ROS_INFO_STREAM("Distance between lighthouses " << it->first
          << " and " << jt->first << " is " << sqrt(dl) << " meters");
      }
    }
    // Publish the new solution
    SendTransforms(frame_parent_, frame_child_, lighthouses_, trackers_);
    // Write the solution to a config file
    if (WriteConfig(calfile_, frame_parent_, frame_child_,
      lighthouses_, trackers_))
      ROS_INFO_STREAM("Calibration written to " << calfile_);
    else
      ROS_INFO_STREAM("Could not write calibration to" << calfile_);
    //
    if (refine_trajectory_) {
      ROS_INFO_STREAM("You requested tracjectory refining" << calfile_);
      std::ofstream outfile;
      outfile.open ("statistics.csv");
      std::map<ros::Time, double[6]>::iterator it;
      for (it = wTb.begin(); it != wTb.end(); it++) {
        geometry_msgs::TransformStamped & tfs = corrections_[it->first];
        Eigen::Vector3d v(it->second[3], it->second[4], it->second[5]);
        Eigen::AngleAxisd aa;
        if (v.norm() > 0) {
          aa.angle() = v.norm();
          aa.axis() = v.normalized();
        }
        Eigen::Quaterniond q(aa);
        outfile << (it->first - wTb.begin()->first).toSec() << ", "
                << it->second[0] << ", "
                << it->second[1] << ", "
                << it->second[2] << ", "
                << q.w() << ", "
                << q.x() << ", "
                << q.y() << ", "
                << q.z() << ", "
                << tfs.transform.translation.x << ", "
                << tfs.transform.translation.y << ", "
                << tfs.transform.translation.z << ", "
                << tfs.transform.rotation.w << ", "
                << tfs.transform.rotation.x << ", "
                << tfs.transform.rotation.y << ", "
                << tfs.transform.rotation.z << std::endl;
      }
      outfile.close();
    }
    // Print the trajectory of the body-frame in the world-frame
    if (visualize_) {
      // Publish path
      {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        std::map<ros::Time, double[6]>::iterator it;
        for (it = wTb.begin(); it != wTb.end(); it++) {
          geometry_msgs::PoseStamped ps;
          Eigen::Vector3d v(it->second[3], it->second[4], it->second[5]);
          Eigen::AngleAxisd aa;
          if (v.norm() > 0) {
            aa.angle() = v.norm();
            aa.axis() = v.normalized();
          }
          Eigen::Quaterniond q(aa);
          ps.header.stamp = it->first;
          ps.header.frame_id = "world";
          ps.pose.position.x = it->second[0];
          ps.pose.position.y = it->second[1];
          ps.pose.position.z = it->second[2];
          ps.pose.orientation.w = q.w();
          ps.pose.orientation.x = q.x();
          ps.pose.orientation.y = q.y();
          ps.pose.orientation.z = q.z();
          msg.poses.push_back(ps);
        }
        pub_path_.publish(msg);
      }
      // Publish corrections
      {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        std::map<ros::Time, double[6]>::iterator it;
        for (it = wTb.begin(); it != wTb.end(); it++) {
          geometry_msgs::TransformStamped & tfs = corrections_[it->first];
          geometry_msgs::PoseStamped ps;
          ps.header.stamp = it->first;
          ps.header.frame_id = "world";
          ps.pose.position.x = tfs.transform.translation.x;
          ps.pose.position.y = tfs.transform.translation.y;
          ps.pose.position.z = tfs.transform.translation.z;
          ps.pose.orientation = tfs.transform.rotation;
          msg.poses.push_back(ps);
        }
        pub_corr_.publish(msg);
      }
    }
    // Solution is usable
    return true;
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

void CorrectionCallback(tf2_msgs::TFMessage::ConstPtr const& msg) {
  // Check that we are recording and that the tracker/lighthouse is ready
  if (!recording_)
    return;
  std::vector<geometry_msgs::TransformStamped>::const_iterator it;
  for (it = msg->transforms.begin(); it != msg->transforms.end(); it++) {
    if (it->header.frame_id == frame_parent_ &&
        it->child_frame_id == frame_estimate_) {
      corrections_[ros::Time::now()] = *it;
    }
  }
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
    corrections_.clear();
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
    size_t n = 0;
    for (jt = trackers_.begin(); jt != trackers_.end(); jt++, n++)  {
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
          static visualization_msgs::Marker marker;
          marker.header.frame_id = jt->first + "/light";
          marker.header.stamp = ros::Time::now();
          marker.ns = "sensors";
          marker.id = NUM_SENSORS * n + i;
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
    }
    pub_sensors_.publish(msg);
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

  // Get the parent information
  if (!nh.getParam("calfile", calfile_))
    ROS_FATAL("Failed to get the calfile file.");

  // Get some global information
  if (!nh.getParam("frames/parent", frame_parent_))
    ROS_FATAL("Failed to get frames/parent parameter.");
  if (!nh.getParam("frames/child", frame_child_))
    ROS_FATAL("Failed to get frames/child parameter.");  
  if (!nh.getParam("frames/estimate", frame_estimate_))
    ROS_FATAL("Failed to get frames/estimate parameter.");  

  // Get the pose graph resolution
  if (!nh.getParam("resolution", resolution_))
    ROS_FATAL("Failed to get resolution parameter.");

  // Get the thresholds
  if (!nh.getParam("thresholds/count", thresh_count_))
    ROS_FATAL("Failed to get threshods/count parameter.");
  if (!nh.getParam("thresholds/angle", thresh_angle_))
    ROS_FATAL("Failed to get thresholds/angle parameter.");
  if (!nh.getParam("thresholds/duration", thresh_duration_))
    ROS_FATAL("Failed to get thresholds/duration parameter.");
  if (!nh.getParam("thresholds/correction", thresh_correction_))
    ROS_FATAL("Failed to get thresholds/correction parameter.");

  // What to refine
  if (!nh.getParam("refine/lighthouses", refine_lighthouses_))
    ROS_FATAL("Failed to get refine/lighthouses parameter.");
  if (!nh.getParam("refine/trajectory", refine_trajectory_))
    ROS_FATAL("Failed to get refine/trajectory parameter.");
  if (!nh.getParam("refine/extrinsics", refine_extrinsics_))
    ROS_FATAL("Failed to get refine/extrinsics parameter.");
  if (!nh.getParam("refine/sensors", refine_sensors_))
    ROS_FATAL("Failed to get refine/sensors parameter.");
  if (!nh.getParam("refine/head", refine_head_))
    ROS_FATAL("Failed to get refine/head parameter.");
  if (!nh.getParam("refine/params", refine_params_))
    ROS_FATAL("Failed to get refine/params parameter.");

  // What weights to use
  if (!nh.getParam("weight/light", weight_light_))
    ROS_FATAL("Failed to get weight/light parameter.");
  if (!nh.getParam("weight/correction", weight_correction_))
    ROS_FATAL("Failed to get weight/correction parameter.");
  if (!nh.getParam("weight/motion", weight_motion_))
    ROS_FATAL("Failed to get weight/motion parameter.");

  // Whether to apply light corrections
  if (!nh.getParam("correct", correct_))
    ROS_FATAL("Failed to get correct parameter.");
  if (!correct_)
    refine_params_ = false;

  // Whether to apply light corrections
  if (!nh.getParam("force2d", force2d_))
    ROS_FATAL("Failed to get force2d parameter.");

  // Define the ceres problem
  ceres::Solver::Options options;
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options_.function_tolerance = 1e-12;
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
    Eigen::Quaterniond q(
      transform[6],  // qw
      transform[3],  // qx
      transform[4],  // qy
      transform[5]); // qz
    Eigen::AngleAxisd aa(q);
    lighthouses_[serial].wTl[0] = transform[0];
    lighthouses_[serial].wTl[1] = transform[1];
    lighthouses_[serial].wTl[2] = transform[2];
    lighthouses_[serial].wTl[3] = aa.angle() * aa.axis()[0];
    lighthouses_[serial].wTl[4] = aa.angle() * aa.axis()[1];
    lighthouses_[serial].wTl[5] = aa.angle() * aa.axis()[2];
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
  }

  // If reading the configuration file results in inserting the correct
  // number of static transforms into the problem, then we can publish
  // the solution for use by other entities in the system.
  if (ReadConfig(calfile_, lighthouses_, trackers_)) {
    ROS_INFO("Read transforms from calibration");
  } else {
    ROS_INFO("Could not read calibration file");
  }
  SendTransforms(frame_parent_, frame_child_, lighthouses_, trackers_);

  // Assume body and truth are identity so we get nice visuals
  geometry_msgs::TransformStamped tfs;
  tfs.header.stamp = ros::Time::now();
  tfs.header.frame_id = frame_estimate_;
  tfs.child_frame_id = frame_child_;
  tfs.transform.translation.x = 0.0;
  tfs.transform.translation.y = 0.0;
  tfs.transform.translation.z = 0.0;
  tfs.transform.rotation.x = 0.0;
  tfs.transform.rotation.y = 0.0;
  tfs.transform.rotation.z = 0.0;
  tfs.transform.rotation.w = 1.0;
  SendStaticTransform(tfs);

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
  ros::Subscriber sub_corrections =
    nh.subscribe("/tf", 1000, CorrectionCallback);
  ros::ServiceServer service =
    nh.advertiseService("/trigger", TriggerCallback);

  // Publish sensor location and body trajectory 
  pub_sensors_ =
    nh.advertise<visualization_msgs::MarkerArray>("/sensors", 10, true);
  pub_path_ =
    nh.advertise<nav_msgs::Path>("/path", 10, true);
  pub_corr_ =
    nh.advertise<nav_msgs::Path>("/corr", 10, true);

  // Setup a timer to automatically trigger solution on end of experiment
  timer_ = nh.createTimer(ros::Duration(1.0), TimerCallback, true, false);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}