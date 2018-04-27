#ifndef SRC_DEEPDIVE_HH
#define SRC_DEEPDIVE_HH

// Messages
#include <geometry_msgs/TransformStamped.h>
#include <deepdive_ros/Lighthouses.h>
#include <deepdive_ros/Trackers.h>
#include <deepdive_ros/Light.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <string>
#include <vector>
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
  double vTl[6];
  double params[NUM_MOTORS*NUM_PARAMS];
  bool ready;
};
typedef std::map<std::string, Lighthouse> LighthouseMap;

// Tracker data structure
struct Tracker {
  double bTh[6];
  double tTh[6];
  double tTi[6];
  double sensors[NUM_SENSORS*6];
  double errors[NUM_ERRORS][3];
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

void SendStaticTransform(geometry_msgs::TransformStamped const& tfs);

void SendDynamicTransform(geometry_msgs::TransformStamped const& tfs);

void SendTransforms(
  std::string const& frame_world,   // World name
  std::string const& frame_vive,    // Vive frame name
  std::string const& frame_body,    // Body frame name
  double registration[6],
  LighthouseMap const& lighthouses, TrackerMap const& trackers);


// CONFIG MANAGEMENT

// Parse a human-readable configuration
bool ReadConfig(std::string const& calfile,   // Calibration file
  std::string const& frame_world,             // World name
  std::string const& frame_vive,              // Vive frame name
  std::string const& frame_body,              // Body frame name
  double registration[6],
  LighthouseMap lighthouses, TrackerMap trackers);

// Write a human-readable configuration
bool WriteConfig(std::string const& calfile,    // Calibration file
  std::string const& frame_world,               // World name
  std::string const& frame_vive,                // Vive frame name
  std::string const& frame_body,                // Body frame name
  double registration[6],
  LighthouseMap const& lighthouses, TrackerMap const& trackers);

// REUSABLE CALLS

void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg,
  LighthouseMap & lighthouses, std::function<void(LighthouseMap::iterator)> cb);

void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg,
  TrackerMap & trackers, std::function<void(TrackerMap::iterator)> cb);

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

// Get the average of a vector of doubles
bool Mean(std::vector<double> const& v, double & d);

#endif

