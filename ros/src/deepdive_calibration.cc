/*
  This ROS node listens to data from all trackers in the system and provides
  a global solution to the tracking problem. That is, it solves for the
  relative location of the lighthouses and trackers as a function of time.
*/

// ROS includes
#include <ros/ros.h>

// Third-party includes
#include <tf2_ros/static_transform_broadcaster.h>
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

// GLOBAL PARAMETERS

// Calibration parameters

static constexpr size_t NUM_MOTORS = 2;
static constexpr size_t NUM_SENSORS = 32;
static constexpr size_t NUM_PARAMS = 5;
enum {
  CAL_PHASE = 0,
  CAL_TILT,
  CAL_GIB_PHASE,
  CAL_GIB_MAG,
  CAL_CURVE
};

// Lighthouse data structure
struct Lighthouse {
  double wTl[6];
  double params[NUM_MOTORS][NUM_PARAMS];
  bool ready;
};
std::map<std::string, Lighthouse> lighthouses_;

// Tracker data structure
struct Tracker {
  double bTh[6];
  double tTh[6];
  double sensors[NUM_SENSORS*6];
  bool ready;
};
std::map<std::string, Tracker> trackers_;

// Pulse measurements
struct Measurement {
  double wTb[6];
  deepdive_ros::Light light;
};
std::map<ros::Time, Measurement> measurements_;

// Corrections
std::vector<geometry_msgs::TransformStamped> corrections_;

// Global strings
std::string cfgfile_ = "deepdive.tf2";
std::string frame_parent_ = "world";
std::string frame_child_ = "body";

// Whether to apply corrections
bool correct_ = false;

// What to solve for
bool refine_extrinsics_ = false;
bool refine_sensors_ = false;
bool refine_head_ = false;
bool refine_params_ = false;

// Rejection thresholds
int thresh_count_ = 4;
double thresh_angle_ = 60.0;
double thresh_duration_ = 1.0;
double thresh_correction_ = 0.1;

// Solver parameters
ceres::Solver::Options options_;

// Should we publish rviz markers
bool visualize_ = true;

// Are we recording right now?
bool recording_ = false;

// Sensor visualization publisher
ros::Publisher pub_sensors_;
ros::Publisher pub_path_;

// STATIC TRANSFORM ENGINE

void SendStaticTransform(geometry_msgs::TransformStamped const& tfs) {
  static tf2_ros::StaticTransformBroadcaster bc;
  bc.sendTransform(tfs);
}

// CONFIGURATION PERSISTENCE

int ReadConfig() {
  // Entries take the form x y z qx qy qz qw parent child
  std::ifstream infile(cfgfile_);
  if (!infile.is_open()) {
    ROS_WARN("Could not open config file for reading");
    return 0;
  }
  std::string line;
  int count = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    double x, y, z, qx, qy, qz, qw;
    std::string p, c;
    if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw >> p >> c)) {
      ROS_ERROR("Badly formatted config file");
      return 0;
    }
    Eigen::AngleAxisd aa(Eigen::Quaterniond(qw, qx, qy, qz));
    if (p == frame_parent_ && lighthouses_.find(c) != lighthouses_.end()) {
      lighthouses_[c].wTl[0] = x;
      lighthouses_[c].wTl[1] = y;
      lighthouses_[c].wTl[2] = z;
      lighthouses_[c].wTl[3] = aa.angle() * aa.axis()[0];
      lighthouses_[c].wTl[4] = aa.angle() * aa.axis()[1];
      lighthouses_[c].wTl[5] = aa.angle() * aa.axis()[2];
      count++;
      continue;
    }
    if (p == frame_child_ && trackers_.find(c) != trackers_.end()) {
      trackers_[c].bTh[0] = x;
      trackers_[c].bTh[1] = y;
      trackers_[c].bTh[2] = z;
      trackers_[c].bTh[3] = aa.angle() * aa.axis()[0];
      trackers_[c].bTh[4] = aa.angle() * aa.axis()[1];
      trackers_[c].bTh[5] = aa.angle() * aa.axis()[2];
      count++;
      continue;
    }
    ROS_WARN_STREAM("Transform " << p << " ->" << c << " invalid");
  }
  return count;
}

int WriteConfig() {
  std::ofstream outfile(cfgfile_);
  if (!outfile.is_open()) {
    ROS_WARN("Could not open config file for writing");
    return 0;
  }
  int count = 0;
  std::map<std::string, Lighthouse>::iterator it;
  for (it = lighthouses_.begin(); it != lighthouses_.end(); it++)  {
    Eigen::Vector3d v(it->second.wTl[3], it->second.wTl[4], it->second.wTl[5]);
    Eigen::AngleAxisd aa;
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    outfile << it->second.wTl[0] << " "
            << it->second.wTl[1] << " "
            << it->second.wTl[2] << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << frame_parent_ << " " << it->first
            << std::endl;
    count++;
  }
  std::map<std::string, Tracker>::iterator jt;
  for (jt = trackers_.begin(); jt !=trackers_.end(); jt++)  {
    Eigen::Vector3d v(jt->second.bTh[3], jt->second.bTh[4], jt->second.bTh[5]);
    Eigen::AngleAxisd aa;
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    outfile << jt->second.bTh[0] << " "
            << jt->second.bTh[1] << " "
            << jt->second.bTh[2] << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << frame_child_ << " " << jt->first
            << std::endl;
    count++;
  }
  return count;
}

// SOLUTION PUBLISHING

void Publish() {
  geometry_msgs::TransformStamped tfs;
  tfs.header.stamp = ros::Time::now();
  // Publish lighthouse positions
  std::map<std::string, Lighthouse>::iterator it;
  for (it = lighthouses_.begin(); it != lighthouses_.end(); it++)  {
    Eigen::Vector3d v(it->second.wTl[3], it->second.wTl[4], it->second.wTl[5]);
    Eigen::AngleAxisd aa;
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    tfs.header.frame_id = frame_parent_;
    tfs.child_frame_id = it->first;
    tfs.transform.translation.x = it->second.wTl[0];
    tfs.transform.translation.y = it->second.wTl[1];
    tfs.transform.translation.z = it->second.wTl[2];
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    SendStaticTransform(tfs);
  }
  // Publish tracker positions
  std::map<std::string, Tracker>::iterator jt;
  for (jt = trackers_.begin(); jt != trackers_.end(); jt++)  {
    Eigen::Vector3d v(jt->second.bTh[3], jt->second.bTh[4], jt->second.bTh[5]);
    Eigen::AngleAxisd aa;
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    tfs.header.frame_id = frame_child_;
    tfs.child_frame_id = jt->first;
    tfs.transform.translation.x = jt->second.bTh[0];
    tfs.transform.translation.y = jt->second.bTh[1];
    tfs.transform.translation.z = jt->second.bTh[2];
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    SendStaticTransform(tfs);
  }
}

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
  bool operator()(const T* const wTl,        // Lighthouse -> world
                  const T* const wTb,        // Body -> world
                  const T* const bTh,        // Head -> body
                  const T* const tTh,        // Head -> tracking (light)
                  const T* const sensors,    // Lighthouse calibration
                  const T* const params,     // Tracker extrinsics
                  T* residual) const {
    // The position of the sensor
    T x[3], y[3], angle[2];
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
      angle[deepdive_ros::Motor::AXIS_HORIZONTAL] = atan2(x[0], x[2]);
      angle[deepdive_ros::Motor::AXIS_VERTICAL] = atan2(x[1], x[2]);
      // We want the normal to be negative in the lighthouse frame, otherwise
      // it's not pointing at the lighthouse
      // if (y[2] > T(0.0)) {
      //   angle[deepdive_ros::Motor::AXIS_HORIZONTAL] += T(M_PI/2.0);
      //   angle[deepdive_ros::Motor::AXIS_VERTICAL] += T(M_PI/2.0);
      // }
      // Apply the error correction as needed. I am going to assume that the
      // engineers kept this equation as simple as possible, and infer the
      // meaning of the calibration parameters based on their name. It might
      // be the case that these value are subtracted, rather than added.
      if (correct_) { 
        angle[a] -= T(params[CAL_PHASE]);
        angle[a] -= T(params[CAL_TILT]) * angle[1-a];
        angle[a] -= T(params[CAL_CURVE]) * angle[1-a] * angle[1-a];
        angle[a] -= T(params[CAL_GIB_MAG])
                      * cos(angle[1-a] + T(params[CAL_GIB_PHASE]));
      }
      // The residual angle error for the specific axis
      residual[i] = angle[a] - T(light_.pulses[i].angle);
    }
    // Everything went well
    return true;
  }
 // Internal variables
 private:
  deepdive_ros::Light light_;
};

// Residual error between predicted anfgles to a lighthouse
struct MotionCost {
  explicit MotionCost() {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const wTb_p,        // Body -> world (prev)
                  const T* const wTb_n,        // Body -> world (next)
                  T* residual) const {
    for (size_t i = 0; i < 6; i++)
      residual[i] =  wTb_n[i] - wTb_p[i];
    return true;
  }
};

bool Solve() {
  // Create the ceres problem
  ceres::Problem problem;

  // Add the measurements
  if (measurements_.empty()) {
    ROS_WARN("No measurements received, so cannot solve the problem.");
    return false;
  } else {
    ROS_INFO_STREAM("Processing " << measurements_.size() << " measurements");
  }
  std::map<ros::Time, Measurement>::iterator mt, mt_p;
  std::map<std::string, size_t> tally;
  for (mt = measurements_.begin(); mt != measurements_.end(); mt++) {
    // Get references to the lighthouse, tracker and axis
    Tracker & tracker = trackers_[mt->second.light.header.frame_id];
    Lighthouse & lighthouse = lighthouses_[mt->second.light.lighthouse];
    size_t axis = mt->second.light.axis;

    // Register that we have received from this lighthouse
    tally[mt->second.light.lighthouse]++;

    // Add the cost function
    ceres::CostFunction* cost = new ceres::AutoDiffCostFunction
      <LightCost, ceres::DYNAMIC, 6, 6, 6, 6, NUM_SENSORS * 6, NUM_PARAMS>(
        new LightCost(mt->second.light), mt->second.light.pulses.size());

    // Add the residual block
    problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
      reinterpret_cast<double*>(lighthouse.wTl),
      reinterpret_cast<double*>(mt->second.wTb),
      reinterpret_cast<double*>(tracker.bTh),
      reinterpret_cast<double*>(tracker.tTh),
      reinterpret_cast<double*>(tracker.sensors),
      reinterpret_cast<double*>(lighthouse.params[axis]));

    // If we are not on the first pose, then create a motion cost
    if (mt != measurements_.begin()) {
      ceres::CostFunction* cost = new ceres::AutoDiffCostFunction
        <MotionCost, 6, 6, 6>(new MotionCost());
      problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
        reinterpret_cast<double*>(std::prev(mt)->second.wTb),
        reinterpret_cast<double*>(mt->second.wTb));
    }
  }
  if (tally.size() != lighthouses_.size()) {
    ROS_WARN("We didn't receive data from all lighthouses. Aborting");
    return false;
  } else {
    ROS_INFO_STREAM("Data summary:");
    std::map<std::string, Lighthouse>::iterator it;
    for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) 
      ROS_INFO_STREAM("- LH " << it->first << ": " << tally[it->first]);
  }

  // Add corrections
  if (corrections_.empty()) {
    ROS_WARN("No corrections received, so locking inital pose to origin");
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = measurements_.begin()->first;
    tfs.transform.translation.x = 0.0;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.0;
    tfs.transform.rotation.w = 1.0;
    tfs.transform.rotation.x = 0.0;
    tfs.transform.rotation.y = 0.0;
    tfs.transform.rotation.z = 0.0;
    corrections_.push_back(tfs);
  } else {
    ROS_INFO_STREAM("Applying " << corrections_.size() << " corrections");
  }
  std::vector<geometry_msgs::TransformStamped>::iterator kt;
  for (kt = corrections_.begin(); kt != corrections_.end(); kt++) {
    // Find the closest measurement time to this correction time
    mt = measurements_.lower_bound(kt->header.stamp);
    if (mt == measurements_.end()) {
      continue;
    } else if (mt != measurements_.begin()) {
      mt_p = std::prev(mt);
      if (kt->header.stamp - mt_p->first < mt->first - kt->header.stamp)
        mt = mt_p;
    }
    if ((kt->header.stamp - mt->first).toSec() > thresh_correction_) {
      ROS_WARN("Correction discarded, as no matching timestamp");
      continue;
    }
    // Set the value of the body at the time of the measurement
    Eigen::Quaterniond q(
      kt->transform.rotation.w,
      kt->transform.rotation.x,
      kt->transform.rotation.y,
      kt->transform.rotation.z);
    Eigen::AngleAxisd aa(q);
    mt->second.wTb[0] = kt->transform.translation.x;
    mt->second.wTb[1] = kt->transform.translation.y;
    mt->second.wTb[2] = kt->transform.translation.z;
    mt->second.wTb[3] = aa.angle() * aa.axis()[0];
    mt->second.wTb[4] = aa.angle() * aa.axis()[1];
    mt->second.wTb[5] = aa.angle() * aa.axis()[2];
    // Correction is just a constant parameter block with known value
    problem.SetParameterBlockConstant(measurements_.begin()->second.wTb);
  }

  // Make a subset of parameter blocks constant if we don't want to refine
  std::map<std::string, Lighthouse>::iterator it;
  for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) 
    if (!refine_params_) 
      for (size_t i = 0; i < NUM_MOTORS; i++)
        problem.SetParameterBlockConstant(it->second.params[i]);
  std::map<std::string, Tracker>::iterator jt;
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
    // Publish the new solution
    Publish();
    // Write the solution to a config file
    WriteConfig();
    // Print the trajectory of the body-frame in the world-frame
    if (visualize_) {
      nav_msgs::Path msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "world";
      std::map<ros::Time, Measurement>::iterator m;
      for (m = measurements_.begin(); m != measurements_.end(); m++) {
        geometry_msgs::PoseStamped ps;
        Eigen::Vector3d v(m->second.wTb[3], m->second.wTb[4], m->second.wTb[5]);
        Eigen::AngleAxisd aa;
        if (v.norm() > 0) {
          aa.angle() = v.norm();
          aa.axis() = v.normalized();
        }
        Eigen::Quaterniond q(aa);
        ps.header.stamp = m->first;
        ps.header.frame_id = "world";
        ps.pose.position.x = m->second.wTb[0];
        ps.pose.position.y = m->second.wTb[1];
        ps.pose.position.z = m->second.wTb[2];
        ps.pose.orientation.w = q.w();
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        msg.poses.push_back(ps);    
      }
      pub_path_.publish(msg);
    }
    // Solution is usable
    return true;
  }

  // Solution is not usable
  return false;
}

// MESSAGE CALLBACKS

void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg) {
  std::vector<deepdive_ros::Lighthouse>::const_iterator it;
  for (it = msg->lighthouses.begin(); it != msg->lighthouses.end(); it++) {
    if (lighthouses_.find(it->serial) == lighthouses_.end())
      return;
    Lighthouse & lighthouse = lighthouses_[it->serial];
    for (size_t i = 0; i < it->motors.size() && i < NUM_MOTORS; i++) {
      lighthouse.params[i][CAL_PHASE] = it->motors[i].phase;
      lighthouse.params[i][CAL_TILT] = it->motors[i].tilt;
      lighthouse.params[i][CAL_GIB_PHASE] = it->motors[i].gibphase;
      lighthouse.params[i][CAL_GIB_MAG] = it->motors[i].gibmag;
      lighthouse.params[i][CAL_CURVE] = it->motors[i].curve;
    }
    ROS_INFO_STREAM("Received data from lighthouse " << it->serial);
    lighthouse.ready = true;
  }
}

void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg) {
  // Iterate over the trackers in this message
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    if (trackers_.find(it->serial) == trackers_.end())
      return;
    // COnvert to an angle-axis representation
    Eigen::Quaterniond q(
      it->head_transform.rotation.w,
      it->head_transform.rotation.x,
      it->head_transform.rotation.y,
      it->head_transform.rotation.z);
    Eigen::AngleAxisd aa(q);
    // Modify the tracker
    Tracker & tracker = trackers_[it->serial];
    tracker.tTh[0] = it->head_transform.translation.x;
    tracker.tTh[1] = it->head_transform.translation.y;
    tracker.tTh[2] = it->head_transform.translation.z;
    tracker.tTh[3] = aa.angle() * aa.axis()[0];
    tracker.tTh[4] = aa.angle() * aa.axis()[1];
    tracker.tTh[5] = aa.angle() * aa.axis()[2];
    for (size_t i = 0; i < it->sensors.size() &&  i < NUM_SENSORS; i++) {
      tracker.sensors[6*i+0] = it->sensors[i].position.x;
      tracker.sensors[6*i+1] = it->sensors[i].position.y;
      tracker.sensors[6*i+2] = it->sensors[i].position.z;
      tracker.sensors[6*i+3] = it->sensors[i].normal.x;
      tracker.sensors[6*i+4] = it->sensors[i].normal.y;
      tracker.sensors[6*i+5] = it->sensors[i].normal.z;
    }
    // Add the transforms
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = it->serial + "/light";
    tfs.child_frame_id = it->serial;
    tfs.transform = it->head_transform;
    SendStaticTransform(tfs);
    tfs.header.frame_id = it->serial + "/light";
    tfs.child_frame_id = it->serial + "/imu";
    tfs.transform = it->imu_transform;
    SendStaticTransform(tfs);
    // Print update
    ROS_INFO_STREAM("Received data from tracker " << it->serial);
    tracker.ready = true;
  }
  // Tracker callback triggers an update of all sensor locations and static
  // frame transforms rtelating the head, light and body frames
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

void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  // Check that we are recording and that the tracker/lighthouse is ready
  if (!recording_ ||
    trackers_.find(msg->header.frame_id) == trackers_.end() ||
    lighthouses_.find(msg->lighthouse) == lighthouses_.end() ||
    !trackers_[msg->header.frame_id].ready ||
    !lighthouses_[msg->lighthouse].ready) return;
  // Create a measurement
  measurements_[msg->header.stamp].light = *msg;
}

void CorrectionCallback(geometry_msgs::TransformStamped::ConstPtr const& msg) {
  // Check that we are recording and that the tracker/lighthouse is ready
  if (!recording_) return;
  corrections_.push_back(*msg);
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

// MAIN ENTRY POINT

int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_calibration");
  ros::NodeHandle nh("~");

  // Get the parent information
  if (!nh.getParam("cfgfile", cfgfile_))
    ROS_FATAL("Failed to get the cfgfile file.");

  // Get some global information
  if (!nh.getParam("frames/parent", frame_parent_))
    ROS_FATAL("Failed to get frames/parent parameter.");
  if (!nh.getParam("frames/child", frame_child_))
    ROS_FATAL("Failed to get frames/child parameter.");  

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
  if (!nh.getParam("refine/extrinsics", refine_extrinsics_))
    ROS_FATAL("Failed to get refine/extrinsics parameter.");
  if (!nh.getParam("refine/sensors", refine_sensors_))
    ROS_FATAL("Failed to get refine/sensors parameter.");
  if (!nh.getParam("refine/head", refine_head_))
    ROS_FATAL("Failed to get refine/head parameter.");
  if (!nh.getParam("refine/params", refine_params_))
    ROS_FATAL("Failed to get refine/params parameter.");

  // Whether to apply light corrections
  if (!nh.getParam("correct", correct_))
    ROS_FATAL("Failed to get correct parameter.");

  // If we aren't applying corrections, then we cant refine them.
  if (!correct_)
    refine_params_ = false;

  // Define the ceres problem
  ceres::Solver::Options options;
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  if (!nh.getParam("solver/max_time", options_.max_solver_time_in_seconds))
    ROS_FATAL("Failed to get the solver/max_time parameter.");
  if (!nh.getParam("solver/max_iterations", options_.max_num_iterations))
    ROS_FATAL("Failed to get the solver/max_iterations parameter.");
  if (!nh.getParam("solver/solver_threads", options_.num_linear_solver_threads))
    ROS_FATAL("Failed to get the solver/solver_threads parameter.");
  if (!nh.getParam("solver/jacobian_threads", options_.num_threads))
    ROS_FATAL("Failed to get the solver/jacobian_threads parameter.");
  if (!nh.getParam("solver/gradient_tolerance", options_.gradient_tolerance))
    ROS_FATAL("Failed to get the solver/gradient_tolerance parameter.");
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
  for (it = lighthouses.begin(); it != lighthouses.end(); it++)
    lighthouses_[*it].ready = false;

  // Get the parent information
  std::vector<std::string> trackers;
  if (!nh.getParam("trackers", trackers))
    ROS_FATAL("Failed to get the tracker list.");
  std::vector<std::string>::iterator jt;
  for (jt = trackers.begin(); jt != trackers.end(); jt++)
    trackers_[*jt].ready = false;

  // Subscribe to tracker and lighthouse updates
  ros::Subscriber sub_tracker  =
    nh.subscribe("/trackers", 10, TrackerCallback);
  ros::Subscriber sub_lighthouse =
    nh.subscribe("/lighthouses", 10, LighthouseCallback);
  ros::Subscriber sub_light =
    nh.subscribe("/light", 10, LightCallback);
  ros::Subscriber sub_corrections =
    nh.subscribe("/corrections", 10, CorrectionCallback);
  ros::ServiceServer service =
    nh.advertiseService("/trigger", TriggerCallback);

  // Publish sensor location and body trajectory 
  pub_sensors_ =
    nh.advertise<visualization_msgs::MarkerArray>("/sensors", 10, true);
  pub_path_ =
    nh.advertise<nav_msgs::Path>("/path", 10, true);

  // If reading the configuration file results in inserting the correct
  // number of static transforms into the problem, then we can publish
  // the solution for use by other entities in the system.
  int n = ReadConfig();
  if (n == lighthouses_.size() + trackers_.size()) {
    ROS_INFO_STREAM("Read " << n << " frame transforms from calibration");
    Publish();
  } else {
    ROS_INFO_STREAM("Could not read calibration file");
  }

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}