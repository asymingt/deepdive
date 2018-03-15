#ifndef SRC_DEEPDIVE_HH
#define SRC_DEEPDIVE_HH

// ROS
#include <ros/ros.h>

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
static constexpr size_t NUM_MOTORS  = 2;
static constexpr size_t NUM_SENSORS = 32;
static constexpr size_t NUM_PARAMS  = 5;

// Lighthouse parameters
enum Parameters {
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
typedef std::map<std::string, Lighthouse> LighthouseMap;

// Tracker data structure
struct Tracker {
  double bTh[6];
  double tTh[6];
  double sensors[NUM_SENSORS*6];
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

// Recursively calculates mean and standard deviation
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
