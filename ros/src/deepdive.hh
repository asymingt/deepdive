#ifndef SRC_DEEPDIVE_HH
#define SRC_DEEPDIVE_HH

// ROS and TF2
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Messages
#include <geometry_msgs/TransformStamped.h>
#include <deepdive_ros/Lighthouses.h>
#include <deepdive_ros/Trackers.h>
#include <deepdive_ros/Light.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <fstream>
#include <sstream>
#include <string>
#include <map>

// Universal constants
static constexpr size_t NUM_SENSORS = 32;

// ESSENTIAL STRUCTURES

// Lighthouse parameters
enum Params {
  PARAM_PHASE,
  PARAM_TILT,
  PARAM_GIB_PHASE,
  PARAM_GIB_MAG,
  PARAM_CURVE,
  NUM_PARAMS
};

// Lighthouse parameters
enum Errors {
  ERROR_GYR_BIAS,
  ERROR_GYR_SCALE,
  ERROR_ACC_BIAS,
  ERROR_ACC_SCALE,
  NUM_ERRORS
};

enum Motors {
  MOTOR_VERTICAL,
  MOTOR_HORIZONTAL,
  NUM_MOTORS
};

// Lighthouse data structure
struct Lighthouse {
  double wTl[6];
  double params[NUM_MOTORS][NUM_PARAMS];
  bool ready;
};
typedef std::map<std::string, Lighthouse> LighthouseMap;

// Tracker data structure
struct Tracker {
  double bTh[6];
  double tTh[6];
  double tTi[6];
  double sensors[NUM_SENSORS*6];
  double errors[NUM_ERRORS*3];
  bool ready;
};
typedef std::map<std::string, Tracker> TrackerMap;

// Pulse measurements
struct Measurement {
  double wTb[6];
  deepdive_ros::Light light;
};
typedef std::map<ros::Time, Measurement> MeasurementMap;

// Correction data structure
typedef std::map<ros::Time, geometry_msgs::TransformStamped> CorrectionMap;

// TRANSFORM ENGINE

void SendStaticTransform(geometry_msgs::TransformStamped const& tfs) {
  static tf2_ros::StaticTransformBroadcaster bc;
  bc.sendTransform(tfs);
}

void SendDynamicTransform(geometry_msgs::TransformStamped const& tfs) {
  static tf2_ros::TransformBroadcaster bc;
  bc.sendTransform(tfs);
}

// CONFIG MANAGEMENT

// Parse a human-readable configuration
bool ReadConfig(std::string const& calfile,
  LighthouseMap & lighthouses, TrackerMap & trackers) {
  // Entries take the form x y z qx qy qz qw parent child
  std::ifstream infile(calfile);
  if (!infile.is_open()) {
    ROS_WARN("Could not open config file for reading");
    return false;
  }
  std::string line;
  int count = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    double x, y, z, qx, qy, qz, qw;
    std::string p, c;
    if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw >> p >> c)) {
      ROS_ERROR("Badly formatted config file");
      return false;
    }
    Eigen::AngleAxisd aa(Eigen::Quaterniond(qw, qx, qy, qz));
    if (lighthouses.find(c) != lighthouses.end()) {
      lighthouses[c].wTl[0] = x;
      lighthouses[c].wTl[1] = y;
      lighthouses[c].wTl[2] = z;
      lighthouses[c].wTl[3] = aa.angle() * aa.axis()[0];
      lighthouses[c].wTl[4] = aa.angle() * aa.axis()[1];
      lighthouses[c].wTl[5] = aa.angle() * aa.axis()[2];
      count++;
      continue;
    }
    if (trackers.find(c) != trackers.end()) {
      trackers[c].bTh[0] = x;
      trackers[c].bTh[1] = y;
      trackers[c].bTh[2] = z;
      trackers[c].bTh[3] = aa.angle() * aa.axis()[0];
      trackers[c].bTh[4] = aa.angle() * aa.axis()[1];
      trackers[c].bTh[5] = aa.angle() * aa.axis()[2];
      count++;
      continue;
    }
    ROS_WARN_STREAM("Transform " << p << " -> " << c << " invalid");
  }
  return (count == lighthouses.size() + trackers.size());
}

// Write a human-readable configuration
bool WriteConfig(std::string const& calfile,
  std::string const& parent_frame, std::string const& child_frame,
    LighthouseMap const& lighthouses, TrackerMap const& trackers) {
  std::ofstream outfile(calfile);
  if (!outfile.is_open()) {
    ROS_WARN("Could not open config file for writing");
    return false;
  }
  LighthouseMap::const_iterator it;
  for (it = lighthouses.begin(); it != lighthouses.end(); it++)  {
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
            << parent_frame << " " << it->first
            << std::endl;
  }
  TrackerMap::const_iterator jt;
  for (jt = trackers.begin(); jt != trackers.end(); jt++)  {
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
            << child_frame << " " << jt->first
            << std::endl;
  }
  return true;
}

// REUSABLE CALLS

void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg,
  LighthouseMap & lighthouses) {
  std::vector<deepdive_ros::Lighthouse>::const_iterator it;
  for (it = msg->lighthouses.begin(); it != msg->lighthouses.end(); it++) {
    LighthouseMap::iterator lighthouse = lighthouses.find(it->serial);
    if (lighthouse == lighthouses.end())
      return;
    for (size_t i = 0; i < it->motors.size() && i < NUM_MOTORS; i++) {
      lighthouse->second.params[i][PARAM_PHASE] = it->motors[i].phase;
      lighthouse->second.params[i][PARAM_TILT] = it->motors[i].tilt;
      lighthouse->second.params[i][PARAM_GIB_PHASE] = it->motors[i].gibphase;
      lighthouse->second.params[i][PARAM_GIB_MAG] = it->motors[i].gibmag;
      lighthouse->second.params[i][PARAM_CURVE] = it->motors[i].curve;
    }
    if (!lighthouse->second.ready)
      ROS_INFO_STREAM("RX data from lighthouse " << it->serial);
    lighthouse->second.ready = true;
  }
}

void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg,
  TrackerMap & trackers) {
  // Iterate over the trackers in this message
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    TrackerMap::iterator tracker = trackers.find(it->serial);
    if (tracker == trackers.end())
      return;
    for (size_t i = 0; i < it->sensors.size() &&  i < NUM_SENSORS; i++) {
      tracker->second.sensors[6*i+0] = it->sensors[i].position.x;
      tracker->second.sensors[6*i+1] = it->sensors[i].position.y;
      tracker->second.sensors[6*i+2] = it->sensors[i].position.z;
      tracker->second.sensors[6*i+3] = it->sensors[i].normal.x;
      tracker->second.sensors[6*i+4] = it->sensors[i].normal.y;
      tracker->second.sensors[6*i+5] = it->sensors[i].normal.z;
    }
    // Add the head -> light transform
    {
      // Write to the global data structure
      Eigen::Quaterniond q(
        it->head_transform.rotation.w,
        it->head_transform.rotation.x,
        it->head_transform.rotation.y,
        it->head_transform.rotation.z);
      Eigen::AngleAxisd aa(q);
      Eigen::Affine3d tTh;
      tTh.linear() = q.toRotationMatrix();
      tTh.translation() = Eigen::Vector3d(
        it->head_transform.translation.x,
        it->head_transform.translation.y,
        it->head_transform.translation.z);
      tracker->second.tTh[0] = tTh.translation()[0];
      tracker->second.tTh[1] = tTh.translation()[1];
      tracker->second.tTh[2] = tTh.translation()[2];
      tracker->second.tTh[3] = aa.angle() * aa.axis()[0];
      tracker->second.tTh[4] = aa.angle() * aa.axis()[1];
      tracker->second.tTh[5] = aa.angle() * aa.axis()[2];
      // Send off the transform
      Eigen::Affine3d hTt = tTh.inverse();
      q = Eigen::Quaterniond(hTt.linear());
      geometry_msgs::TransformStamped tfs;
      tfs.header.frame_id = it->serial;
      tfs.child_frame_id = it->serial + "/light";
      tfs.transform.translation.x = hTt.translation()[0];
      tfs.transform.translation.y = hTt.translation()[1];
      tfs.transform.translation.z = hTt.translation()[2];
      tfs.transform.rotation.w = q.w();
      tfs.transform.rotation.x = q.x();
      tfs.transform.rotation.y = q.y();
      tfs.transform.rotation.z = q.z();
      SendStaticTransform(tfs);
    }
    // Add the imu -> light transform
    {
      // Write to the global data structure
      Eigen::Quaterniond q(
        it->imu_transform.rotation.w,
        it->imu_transform.rotation.x,
        it->imu_transform.rotation.y,
        it->imu_transform.rotation.z);
      Eigen::AngleAxisd aa(q);
      Eigen::Affine3d tTi;
      tTi.linear() = q.toRotationMatrix();
      tTi.translation() = Eigen::Vector3d(
        it->imu_transform.translation.x,
        it->imu_transform.translation.y,
        it->imu_transform.translation.z);
      tracker->second.tTi[0] = tTi.translation()[0];
      tracker->second.tTi[1] = tTi.translation()[1];
      tracker->second.tTi[2] = tTi.translation()[2];
      tracker->second.tTi[3] = aa.angle() * aa.axis()[0];
      tracker->second.tTi[4] = aa.angle() * aa.axis()[1];
      tracker->second.tTi[5] = aa.angle() * aa.axis()[2];
      // Send off the transform
      geometry_msgs::TransformStamped tfs;  
      tfs.header.frame_id = it->serial + "/light";
      tfs.child_frame_id = it->serial + "/imu";
      tfs.transform = it->imu_transform;
      SendStaticTransform(tfs);
    }
    //  We are now ready
    if (!tracker->second.ready)
      ROS_INFO_STREAM("RX data from tracker " << it->serial);
    tracker->second.ready = true;
  }
}

// RUNTIME STATISTICS

class Statistics {
 public:
  // Constructor and initialization
  Statistics() : count_(0.0), mean_(0.0), var_(0.0) {};

  // Feed a value and calculate recursive statistics
  void Feed(double value) {
    count_ += 1.0;
    if (count_ < 2) {
      mean_ = value;
      var_ = 0.0;
      return;
    }
    var_ = (count_ - 2.0) / (count_ - 1.0) * var_
         + (value - mean_) * (value - mean_) / count_;
    mean_ = ((count_ - 1.0) * mean_ + value) / count_;
  }

  // Get statistics
  double Count() { return count_; }
  double Mean() { return mean_; }
  double Variance() { return var_; }
  double Deviation() { return sqrt(var_); }

  // Reset the statistics
  void Reset() {
    count_ = 0.0;
    mean_ = 0.0;
    var_ = 0.0;
  }

 private:
  double count_;
  double var_;
  double mean_;
};

#endif
