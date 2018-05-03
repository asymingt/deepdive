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

// Convert a ceres to an Eigen transform
Eigen::Affine3d CeresToEigen(double ceres[6], bool invert = false);

// CONFIG MANAGEMENT

// Parse a human-readable configuration
bool ReadConfig(std::string const& calfile,   // Calibration file
  std::string const& frame_world,             // World name
  std::string const& frame_vive,              // Vive frame name
  std::string const& frame_body,              // Body frame name
  double registration[6],
  LighthouseMap & lighthouses, TrackerMap & trackers);

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

class Statistic {
 public:
  // Constructor and initialization
  Statistic() : count_(0.0), mean_(0.0), var_(0.0) {};

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

// TRACKING ROUTINES

// This algorithm solves the Procrustes problem in that it finds an affine transform
// (rotation, translation, scale) that maps the "in" matrix to the "out" matrix
// Code from: https://github.com/oleg-alexandrov/projects/blob/master/eigen/Kabsch.cpp
// License is that this code is release in the public domain... Thanks, Oleg :)
template <typename T>
static bool Kabsch(
  Eigen::Matrix<T, 3, Eigen::Dynamic> in,
  Eigen::Matrix<T, 3, Eigen::Dynamic> out,
  Eigen::Transform<T, 3, Eigen::Affine> &A, bool allowScale) {
  // Default output
  A.linear() = Eigen::Matrix<T, 3, 3>::Identity(3, 3);
  A.translation() = Eigen::Matrix<T, 3, 1>::Zero();
  // A simple check to see that we have a sufficient number of correspondences
  if (in.cols() < 4) {
    // ROS_WARN("Visualeyez needs to see at least four LEDs to track");
    return false;
  }
  // A simple check to see that we have a sufficient number of correspondences
  if (in.cols() != out.cols()) {
    // ROS_ERROR("Same number of points required in input matrices");
    return false;
  }
  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  T dist_in = T(0.0), dist_out = T(0.0);
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= T(0.0) || dist_out <= T(0.0))
    return true;
  T scale = T(1.0);
  if (allowScale) {
    scale = dist_out/dist_in;
    out /= scale;
  }
  // Find the centroids then shift to the origin
  Eigen::Matrix<T, 3, 1> in_ctr = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> out_ctr = Eigen::Matrix<T, 3, 1>::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= T(in.cols());
  out_ctr /= T(out.cols());
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }
  // SVD
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Cov = in * out.transpose();
  Eigen::JacobiSVD < Eigen::Matrix < T, Eigen::Dynamic, Eigen::Dynamic > > svd(Cov,
    Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Find the rotation
  T d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > T(0.0))
    d = T(1.0);
  else
    d = T(-1.0);
  Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix<T, 3, 3> R = svd.matrixV() * I * svd.matrixU().transpose();
  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);
  // Success
  return true;
}

// Lighthouse correction
// see: https://github.com/cnlohr/libsurvive/wiki/BSD-Calibration-Values

// Given a point in space, predict the lighthouse angle
template <typename T>
static void Predict(T const* params, T const* xyz, T* ang, bool correct) {
  if (correct) {
    ang[0] = atan2(xyz[0] - (params[0*NUM_PARAMS + PARAM_TILT] + params[0*NUM_PARAMS + PARAM_CURVE] * xyz[1]) * xyz[1], xyz[2]);
    ang[1] = atan2(xyz[1] - (params[1*NUM_PARAMS + PARAM_TILT] + params[1*NUM_PARAMS + PARAM_CURVE] * xyz[0]) * xyz[0], xyz[2]);
    ang[0] -= params[0*NUM_PARAMS + PARAM_PHASE] + params[0*NUM_PARAMS + PARAM_GIB_MAG] * sin(ang[0] + params[0*NUM_PARAMS + PARAM_GIB_PHASE]);
    ang[1] -= params[1*NUM_PARAMS + PARAM_PHASE] + params[1*NUM_PARAMS + PARAM_GIB_MAG] * sin(ang[1] + params[1*NUM_PARAMS + PARAM_GIB_PHASE]);
  } else {
    ang[0] = atan2(xyz[0], xyz[2]);
    ang[1] = atan2(xyz[1], xyz[2]);
  }
}

// Given the lighthouse angle, predict the point in space
template <typename T>
static void Correct(T const* params, T * angle, bool correct) {
  if (correct) {
    T ideal[2], pred[2], xyz[3];
    ideal[0] = angle[0];
    ideal[1] = angle[1];
    for (size_t i = 0; i < 10; i++) {
      xyz[0] = tan(ideal[0]);
      xyz[1] = tan(ideal[1]);
      xyz[2] = T(1.0);
      Predict(params, xyz, pred, correct);
      ideal[0] += (angle[0] - pred[0]);
      ideal[1] += (angle[1] - pred[1]);
    }
    angle[0] = ideal[0];
    angle[1] = ideal[1];
  }
}


#endif

