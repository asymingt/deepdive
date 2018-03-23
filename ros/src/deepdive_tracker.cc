/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// ROS includes
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// General messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <deepdive_ros/Trackers.h>
#include <deepdive_ros/Light.h>
#include <deepdive_ros/Lighthouses.h>

// Eigen includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// UKF includes
#include <UKF/Types.h>
#include <UKF/Integrator.h>
#include <UKF/StateVector.h>
#include <UKF/MeasurementVector.h>
#include <UKF/Core.h>

// C++ includes
#include <vector>
#include <functional>

// Deepdive internal
#include "deepdive.hh"

// FILTER KEYS

// State indexes
enum Keys : uint8_t {
  // STATE
  Position,             // Position (world frame, m)
  Attitude,             // Attitude as a quaternion (world to body frame)
  Velocity,             // Velocity (body frame, m/s)
  Omega,                // Angular velocity (body frame, rads/s)
  Acceleration,         // Acceleration (body frame, m/s^2)
  Alpha,                // Angular acceleration (body frame, rads/s^2)
  // ERRORS
  GyroscopeBias,        // Gyroscope bias offset (body frame, rad/s)
  GyroscopeScale,       // Gyroscope scale factor (body frame, multiplier)
  AccelerometerBias,    // Accelerometer bias offset (body frame, m/s^2)
  AccelerometerScale,   // Accelerometer scale factor (body frame, mutliplier)
  // MEASUREMENTS
  Accelerometer,        // Acceleration (body frame, m/s^2)
  Gyroscope,            // Gyroscope (body frame, rads/s)
  Angle
};

// OBSERVATION

// Observation vector
using Observation = UKF::DynamicMeasurementVector<
  UKF::Field<Accelerometer, UKF::Vector<3>>,
  UKF::Field<Gyroscope, UKF::Vector<3>>,
  UKF::Field<Angle, real_t>
>;

// TRACKING FILTER

// State vector
using State = UKF::StateVector<
  UKF::Field<Position, UKF::Vector<3>>,
  UKF::Field<Attitude, UKF::Quaternion>,
  UKF::Field<Velocity, UKF::Vector<3>>,
  UKF::Field<Omega, UKF::Vector<3>>,
  UKF::Field<Acceleration, UKF::Vector<3>>,
  UKF::Field<Alpha, UKF::Vector<3>>
>;

// For tracking
using TrackingFilter = UKF::Core<
  State, Observation, UKF::IntegratorRK4
>;

// ERROR FILTER

// Parameters
using Error = UKF::StateVector<
  UKF::Field<AccelerometerBias, UKF::Vector<3>>,
  UKF::Field<AccelerometerScale, UKF::Vector<3>>,
  UKF::Field<GyroscopeBias, UKF::Vector<3>>,
  UKF::Field<GyroscopeScale, UKF::Vector<3>>
>;

// For parameter estimation
using ErrorFilter = UKF::Core<
  Error, Observation, UKF::IntegratorEuler
>;

// For IMU parameter estimation

typedef std::map<std::string, Eigen::Affine3d> TransformMap;
typedef std::map<std::string, ErrorFilter> ErrorMap;

// Default IMU errors
Eigen::Vector3d imu_cov_ab_;         // Initial covariance: Accel bias
Eigen::Vector3d imu_cov_as_;         // Initial covariance: Accel scale
Eigen::Vector3d imu_cov_gb_;         // Initial covariance: Gyro bias
Eigen::Vector3d imu_cov_gs_;         // Initial covariance: Gyro scale
Eigen::Vector3d imu_proc_ab_;        // Process noise: Accel bias
Eigen::Vector3d imu_proc_as_;        // Process noise: Accel scale
Eigen::Vector3d imu_proc_gb_;        // Process noise: Gyro bias
Eigen::Vector3d imu_proc_gs_;        // Process noise: Gyro scale

// GLOBAL DATA STRUCTURES

LighthouseMap lighthouses_;          // List of lighthouses
TrackerMap trackers_;                // List of trackers
ErrorMap errors_;                    // List of error filters
TrackingFilter filter_;              // Tracking filter
std::string frame_parent_;           // Parent frame, eg "world"
std::string frame_child_;            // Child frame, eg "truth"
double rate_ = 10.0;                 // Desired tracking rate in Hz
int thresh_count_ = 4;               // Min num measurements required per bundle
double thresh_angle_ = 60.0;         // Angle threshold in degrees
double thresh_duration_ = 1.0;       // Duration threshold in micorseconds
bool use_gyroscope_ = true;          // Input measurements from gyroscope
bool use_accelerometer_ = true;      // Input measurements from accelerometer
bool use_light_ = true;              // Input measurements from light

// ROS publishers
ros::Publisher pub_pose_;
ros::Publisher pub_twist_;

// Default measurement errors
bool correct_ = false;               // Whether to correct light parameters
Eigen::Vector3d gravity_;            // Gravity
Eigen::Vector3d obs_cov_acc_;        // Measurement covariance: Accelerometer
Eigen::Vector3d obs_cov_gyr_;        // Measurement covariance: Gyroscope
double obs_cov_ang_;                 // Measurement covariance: Angle

// Intermediary data
TransformMap lTw_;                   // ALL: world -> lighthouse
TransformMap iTb_;                   // ALL: body -> imu
TransformMap bTt_;                   // ALL: tracking -> body
std::string lighthouse_;             // Active lighthouse
std::string tracker_;                // Active tracker
Eigen::Vector3d sensor_;             // Active sensor
uint8_t axis_;                       // Active axis

// Are we initialized and ready to track
bool initialized_ = false;

// TRACKING FILTER

namespace UKF {

  // MEASURMENT

  template <>
  Observation::CovarianceVector Observation::measurement_covariance(
    (Observation::CovarianceVector() << 
      1.0e-4, 1.0e-4, 1.0e-4,   // Accel
      1.0e-6, 1.0e-6, 1.0e-6,   // Gyro
      1.0e-8).finished());

  // TRACKING FILTER

  // Standard 6DoF kinematics with constant Omega and Acceleration assumption
  template <> template <> State
  State::derivative<>() const {
    State output;
    output.set_field<Position>(
      get_field<Attitude>().conjugate() * get_field<Velocity>());
    output.set_field<Velocity>(get_field<Acceleration>());
    UKF::Quaternion omega_q;
    omega_q.vec() = get_field<Omega>() * 0.5;
    omega_q.w() = 0;
    output.set_field<Attitude>(omega_q.conjugate() * get_field<Attitude>());
    output.set_field<Omega>(get_field<Alpha>());
    output.set_field<Acceleration>(UKF::Vector<3>(0, 0, 0));
    output.set_field<Alpha>(UKF::Vector<3>(0, 0, 0));
    return output;
  }

  // See http://www.mdpi.com/1424-8220/11/7/6771/htm
  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<State, Accelerometer, Error>(
    State const& state, Error const& error) {
    Eigen::Matrix3d iRb = iTb_[tracker_].linear();
    Eigen::Vector3d r = iTb_[tracker_].inverse().translation();
    Eigen::Vector3d w = state.get_field<Omega>();
    /*
    Eigen::Vector3d specific = iRb * state.get_field<Acceleration>();
    Eigen::Vector3d centripetal = iRb * w.cross(w.cross(r));
    Eigen::Vector3d gravitational = iRb * state.get_field<Attitude>() * gravity_;
    Eigen::Vector3d bias = -error.get_field<AccelerometerBias>();
    Eigen::Vector3d scale = error.get_field<AccelerometerScale>().cwiseInverse();
    Eigen::Vector3d accel = 
    ROS_INFO_STREAM("tracker_: " << tracker_);
    ROS_INFO_STREAM("specific accel: " << specific);
    ROS_INFO_STREAM("centripetal accel: " << centripetal);
    ROS_INFO_STREAM("gravitational accel: " << gravitational);
    ROS_INFO_STREAM("bias accel: " << bias);
    ROS_INFO_STREAM("scale accel: " << scale);
    ROS_INFO_STREAM("total accel: " << accel);
    */
    // Return acceleration
    return error.get_field<AccelerometerScale>().cwiseInverse().cwiseProduct(
       -error.get_field<AccelerometerBias>()              // Bias
      + iRb * state.get_field<Acceleration>()             // Specific
      + iRb * w.cross(w.cross(r))                         // Centripetal
      + iRb * state.get_field<Attitude>() * gravity_);    // Gravity
  }

  // See http://www.mdpi.com/1424-8220/11/7/6771/htm
  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<State, Gyroscope, Error>(
    State const& state, Error const& error) {
    Eigen::Matrix3d iRb = iTb_[tracker_].linear();
    return error.get_field<GyroscopeScale>().cwiseInverse().cwiseProduct(
       -error.get_field<GyroscopeBias>()                  // Bias
      + iRb * state.get_field<Omega>());                  // Specific
  }

  template <> template <> real_t
  Observation::expected_measurement<State, Angle, Error>(
    State const& state, Error const& error) {
    Eigen::Affine3d wTb;
    wTb.translation() = state.get_field<Position>();
    wTb.linear() = state.get_field<Attitude>().toRotationMatrix();
    UKF::Vector<3> x = lTw_[lighthouse_] * wTb * bTt_[tracker_] * sensor_;
    UKF::Vector<2> angles;
    angles[0] = std::atan2(x[0], x[2]);
    angles[1] = std::atan2(x[1], x[2]);
    uint8_t const& a = axis_;
    if (correct_) {
      double const * const params = lighthouses_[lighthouse_].params[a];
      angles[a] -= params[PARAM_PHASE];
      angles[a] -= params[PARAM_TILT] * angles[1-a];
      angles[a] -= params[PARAM_CURVE] * angles[1-a] * angles[1-a];
      angles[a] -= params[PARAM_GIB_MAG] *
        std::cos(angles[1-a] + params[PARAM_GIB_PHASE]);
    }
    return angles[a];
  }

  // ERROR FILTER

  template <> template <> Error
  Error::derivative<>() const {
    return Error::Zero();
  }

  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<Error, Accelerometer, State>(
    Error const& errors, State const& state) {
    return expected_measurement<State, Accelerometer, Error>(state, errors);
  }

  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<Error, Gyroscope, State>(
    Error const& errors, State const& state) {
    return expected_measurement<State, Gyroscope, Error>(state, errors);
  }

  template <> template <> real_t
  Observation::expected_measurement<Error, Angle, State>(
    Error const& errors, State const& state) {
    return expected_measurement<State, Angle, Error>(state, errors);
  }
}

// UTILITY FUNCTIONS

bool Delta(double & dt) {
  static ros::Time last = ros::Time::now(), now;
  now = ros::Time::now();
  dt = (now - last).toSec();
  if (dt > 0)
    last = now;
  return (dt > 0 && dt < 1.0);
}

// CALLBACKS

// This will be called at approximately 120Hz
// - Single lighthouse in 'A' mode : 120Hz (60Hz per axis)
// - Dual lighthouses in b/A or b/c modes : 120Hz (30Hz per axis)
void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  static double dt;
  if (!use_light_ || !initialized_ || !Delta(dt))
    return;

  // Check that we are recording and that the tracker/lighthouse is ready
  TrackerMap::iterator tracker = trackers_.find(msg->header.frame_id);
  if (tracker == trackers_.end() || !tracker->second.ready) {
    ROS_INFO_STREAM_THROTTLE(1, "Tracker not found or ready");
    return;
  }

  // Check that we are recording and that the tracker/lighthouse is ready
  LighthouseMap::iterator lighthouse = lighthouses_.find(msg->lighthouse);
  if (lighthouse == lighthouses_.end() || !lighthouse->second.ready) {
    ROS_INFO_STREAM_THROTTLE(1, "Lighthouse not found or ready");
    return;
  }

  // Make sure we have a filter setup for this
  ErrorMap::iterator error = errors_.find(msg->header.frame_id);
  if (error == errors_.end()) {
    ROS_INFO_STREAM_THROTTLE(1, "Tracker error filter not initialized");
    return;
  }

  // Clean up the measurments
  std::vector<deepdive_ros::Pulse> data;
  for (size_t i = 0; i < msg->pulses.size(); i++) {
    // Basic sanity checks on the data
    if (fabs(msg->pulses[i].angle) > thresh_angle_ / 57.2958) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Rejected based on angle");
      continue;
    }
    if (fabs(msg->pulses[i].duration) < thresh_duration_ / 1e6) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Rejected based on duration");
      continue;
    }
    if (msg->pulses[i].sensor >= NUM_SENSORS) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Rejected based on invalid sensor id");
      continue;
    }
    data.push_back(msg->pulses[i]);
  }
  if (thresh_count_ > 0 && data.size() < thresh_count_) {
    ROS_INFO_STREAM_THROTTLE(1, "Not enough data so skipping bundle.");
    return;
  }

  // Set the context correctly
  tracker_ = msg->header.frame_id;
  lighthouse_ = msg->lighthouse;
  axis_ = msg->axis;

  // Correct the error filter
  error->second.a_priori_step(dt);
  for (uint8_t i = 0; i < data.size(); i++) {
    // Set the context correctly
    sensor_[0] = tracker->second.sensors[6 * data[i].sensor + 0];
    sensor_[1] = tracker->second.sensors[6 * data[i].sensor + 1];
    sensor_[2] = tracker->second.sensors[6 * data[i].sensor + 2];
    // Create the observation
    Observation obs;
    obs.set_field<Angle>(data[i].angle);
    error->second.innovation_step(obs, filter_.state);
  }
  error->second.a_posteriori_step();

  // Correct the tracking filter
  filter_.a_priori_step(dt);
  for (uint8_t i = 0; i < data.size(); i++) {
    // Set the context correctly
    sensor_[0] = tracker->second.sensors[6 * data[i].sensor + 0];
    sensor_[1] = tracker->second.sensors[6 * data[i].sensor + 1];
    sensor_[2] = tracker->second.sensors[6 * data[i].sensor + 2];
    // Create the observation
    Observation obs;
    obs.set_field<Angle>(data[i].angle);
    filter_.innovation_step(obs, error->second.state);
  }
  filter_.a_posteriori_step();
}
// This will be called at approximately 250Hz
void ImuCallback(sensor_msgs::Imu::ConstPtr const& msg) {
  static double dt;
  if ((!use_accelerometer_ && !use_gyroscope_) || !initialized_ || !Delta(dt))
    return;

  // Check that we are recording and that the tracker/lighthouse is ready
  TrackerMap::iterator tracker = trackers_.find(msg->header.frame_id);
  if (tracker == trackers_.end() || !tracker->second.ready) {
    ROS_INFO_STREAM_THROTTLE(1, "Tracker not found or ready");
    return;
  }

  // Make sure we have a filter setup for this
  ErrorMap::iterator error = errors_.find(msg->header.frame_id);
  if (error == errors_.end()) {
    ROS_INFO_STREAM_THROTTLE(1, "Tracker error filter not initialized");
    return;
  }

  // Get the measurements
  Eigen::Vector3d acc(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);
  Eigen::Vector3d gyr(
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z);

  // Set the context correctly
  tracker_ = msg->header.frame_id;

  // Create a measurement
  Observation obs;
  if (use_accelerometer_)
    obs.set_field<Accelerometer>(acc);
  if (use_gyroscope_)
    obs.set_field<Gyroscope>(gyr);

  // Step the parameter filter
  error->second.a_priori_step(dt);
  error->second.innovation_step(obs, filter_.state);
  error->second.a_posteriori_step(); 

  // Propagate the filter
  filter_.a_priori_step(dt);
  filter_.innovation_step(obs, error->second.state);
  filter_.a_posteriori_step();
}

// This will be called back at the desired tracking rate
void TimerCallback(ros::TimerEvent const& info) {
  static double dt;
  if (!initialized_ || !Delta(dt))
    return;

  // Propagate the filter forward
  filter_.a_priori_step(dt);

  // Debug
  ErrorMap::iterator it;
  for (it = errors_.begin(); it != errors_.end(); it++) {
    ROS_INFO_STREAM(it->first << ":");
    ROS_INFO_STREAM(it->second.state);
  }
  ROS_INFO_STREAM(filter_.state);
  ROS_INFO_STREAM("Filter:");
  ROS_INFO_STREAM(filter_.state);

  // The filter relates WORLD and IMU frames
  ros::Time now = ros::Time::now();

  // Broadcast the tracker pose on TF2
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tfs;
  tfs.header.stamp = now;
  tfs.header.frame_id = frame_parent_;
  tfs.child_frame_id = frame_child_;
  tfs.transform.translation.x = filter_.state.get_field<Position>()[0];
  tfs.transform.translation.y = filter_.state.get_field<Position>()[1];
  tfs.transform.translation.z = filter_.state.get_field<Position>()[2];
  tfs.transform.rotation.w = filter_.state.get_field<Attitude>().w();
  tfs.transform.rotation.x = filter_.state.get_field<Attitude>().x();
  tfs.transform.rotation.y = filter_.state.get_field<Attitude>().y();
  tfs.transform.rotation.z = filter_.state.get_field<Attitude>().z();
  SendDynamicTransform(tfs);

  // Broadcast the pose with covariance
  geometry_msgs::PoseWithCovarianceStamped pwcs;
  pwcs.header.stamp = now;
  pwcs.header.frame_id = frame_parent_;
  pwcs.pose.pose.position.x = filter_.state.get_field<Position>()[0];
  pwcs.pose.pose.position.y = filter_.state.get_field<Position>()[1];
  pwcs.pose.pose.position.z = filter_.state.get_field<Position>()[2];
  pwcs.pose.pose.orientation.w = filter_.state.get_field<Attitude>().w();
  pwcs.pose.pose.orientation.x = filter_.state.get_field<Attitude>().x();
  pwcs.pose.pose.orientation.y = filter_.state.get_field<Attitude>().y();
  pwcs.pose.pose.orientation.z = filter_.state.get_field<Attitude>().z();
  for (size_t i = 0; i < 6; i++)
    for (size_t j = 0; j < 6; j++)
      pwcs.pose.covariance[i*6 + j] = filter_.covariance(i, j);
  pub_pose_.publish(pwcs);

  // Broadcast the twist with covariance
  geometry_msgs::TwistWithCovarianceStamped twcs;
  twcs.header.stamp = now;
  twcs.header.frame_id = frame_child_;
  twcs.twist.twist.linear.x = filter_.state.get_field<Velocity>()[0];
  twcs.twist.twist.linear.y = filter_.state.get_field<Velocity>()[1];
  twcs.twist.twist.linear.z = filter_.state.get_field<Velocity>()[2];
  twcs.twist.twist.angular.x = filter_.state.get_field<Omega>()[0];
  twcs.twist.twist.angular.y = filter_.state.get_field<Omega>()[1];
  twcs.twist.twist.angular.z = filter_.state.get_field<Omega>()[2];
  for (size_t i = 0; i < 6; i++)
    for (size_t j = 0; j < 6; j++)
      twcs.twist.covariance[i*6 + j] = filter_.covariance(6+i, 6+j);
  pub_twist_.publish(twcs);
}

void CheckIfReadyToTrack() {
  if (!initialized_) {
    TrackerMap::const_iterator it;
    for (it = trackers_.begin(); it != trackers_.end(); it++)
      if (!it->second.ready) return;
    LighthouseMap::const_iterator jt;
    for (jt = lighthouses_.begin(); jt != lighthouses_.end(); jt++)
      if (!jt->second.ready) return;
    ROS_INFO_STREAM("All trackers and lighthouses found. Tracking started.");
    initialized_ = true;
  }
}

// Called when a new lighthouse appears
void NewLighthouseCallback(LighthouseMap::iterator lighthouse) {
  ROS_INFO_STREAM("Found lighthouse " << lighthouse->first);
  ///////////////////////
  // Cache a transform //
  ///////////////////////
  Eigen::Vector3d v;
  Eigen::Affine3d wTl;
  // head -> body
  v = Eigen::Vector3d(lighthouse->second.wTl[3],
    lighthouse->second.wTl[4], lighthouse->second.wTl[5]);
  wTl.translation() = Eigen::Vector3d(lighthouse->second.wTl[0],
    lighthouse->second.wTl[1], lighthouse->second.wTl[2]);
  wTl.linear() = Eigen::AngleAxisd(v.norm(),
    (v.norm() > 0 ? v.normalized() : Eigen::Vector3d::Zero())).toRotationMatrix();
  // Cache a transform
  lTw_[lighthouse->first] = wTl.inverse();
  // Check if we have got all info from lighthouses and trackers
  CheckIfReadyToTrack();
}

// Called when a new tracker appears
void NewTrackerCallback(TrackerMap::iterator tracker) {
  ROS_INFO_STREAM("Found tracker " << tracker->first);
  // Initialize the error filter
  ErrorFilter & error = errors_[tracker->first];
  error.state.set_field<AccelerometerBias>(UKF::Vector<3>(
    tracker->second.errors[ERROR_ACC_BIAS][0],
    tracker->second.errors[ERROR_ACC_BIAS][1],
    tracker->second.errors[ERROR_ACC_BIAS][2]));
  error.state.set_field<AccelerometerScale>(UKF::Vector<3>(
    tracker->second.errors[ERROR_ACC_SCALE][0],
    tracker->second.errors[ERROR_ACC_SCALE][1],
    tracker->second.errors[ERROR_ACC_SCALE][2]));
  error.state.set_field<GyroscopeBias>(UKF::Vector<3>(
    tracker->second.errors[ERROR_GYR_BIAS][0],
    tracker->second.errors[ERROR_GYR_BIAS][1],
    tracker->second.errors[ERROR_GYR_BIAS][2]));
  error.state.set_field<GyroscopeScale>(UKF::Vector<3>(
    tracker->second.errors[ERROR_GYR_SCALE][0],
    tracker->second.errors[ERROR_GYR_SCALE][1],
    tracker->second.errors[ERROR_GYR_SCALE][2]));
  error.covariance = Error::CovarianceMatrix::Zero();
  error.covariance.diagonal() <<
    imu_cov_ab_, imu_cov_as_, imu_cov_gb_, imu_cov_gs_;
  error.process_noise_covariance = Error::CovarianceMatrix::Zero();
  error.process_noise_covariance.diagonal() <<
    imu_proc_ab_, imu_proc_as_, imu_proc_gb_, imu_proc_gs_;
  // Cache a transform //
  Eigen::Vector3d v;
  Eigen::Affine3d bTh, tTh, tTi;
  // head -> body
  v = Eigen::Vector3d(
    tracker->second.bTh[3], tracker->second.bTh[4], tracker->second.bTh[5]);
  bTh.translation() = Eigen::Vector3d(
    tracker->second.bTh[0], tracker->second.bTh[1], tracker->second.bTh[2]);
  bTh.linear() = Eigen::AngleAxisd(v.norm(),
    (v.norm() > 0 ? v.normalized() : Eigen::Vector3d::Zero())).toRotationMatrix();
  // head -> light
  v = Eigen::Vector3d(
    tracker->second.tTh[3], tracker->second.tTh[4], tracker->second.tTh[5]);
  tTh.translation() = Eigen::Vector3d(
    tracker->second.tTh[0], tracker->second.tTh[1], tracker->second.tTh[2]);
  tTh.linear() = Eigen::AngleAxisd(v.norm(),
    (v.norm() > 0 ? v.normalized() : Eigen::Vector3d::Zero())).toRotationMatrix();
  // imu -> light
  v = Eigen::Vector3d(
    tracker->second.tTi[3], tracker->second.tTi[4], tracker->second.tTi[5]);
  tTi.translation() =Eigen::Vector3d(
    tracker->second.tTi[0], tracker->second.tTi[1], tracker->second.tTi[2]);
  tTi.linear() = Eigen::AngleAxisd(v.norm(),
    (v.norm() > 0 ? v.normalized() : Eigen::Vector3d::Zero())).toRotationMatrix();
  // Global cache
  bTt_[tracker->first] = bTh * tTh.inverse();
  iTb_[tracker->first] = tTi.inverse() * tTh * bTh.inverse();
  // Check if we have got all info from lighthouses and trackers
  CheckIfReadyToTrack();
}

// MAIN ENTRY POINT OF APPLICATION

bool GetPairParam(ros::NodeHandle &nh,
  std::string const& name, UKF::Vector<2> & data) {
  std::vector<double> tmp;
  if (!nh.getParam(name, tmp) || tmp.size() != 2)
    return false;
  data[0] = tmp[0];
  data[1] = tmp[1];
  return true;
}

bool GetVectorParam(ros::NodeHandle &nh,
  std::string const& name, UKF::Vector<3> & data) {
  std::vector<double> tmp;
  if (!nh.getParam(name, tmp) || tmp.size() != 3)
    return false;
  data[0] = tmp[0];
  data[1] = tmp[1];
  data[2] = tmp[2];
  return true;
}

bool GetQuaternionParam(ros::NodeHandle &nh,
  std::string const& name, UKF::Quaternion & data) {
  std::vector<double> tmp;
  if (!nh.getParam(name, tmp) || tmp.size() != 4)
    return false;
  data.x() = tmp[0];
  data.y() = tmp[1];
  data.z() = tmp[2];
  data.w() = tmp[3];
  return true;
}

int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_tracker");
  ros::NodeHandle nh("~");

  // Get the parent information
  std::string calfile;
  if (!nh.getParam("calfile", calfile))
    ROS_FATAL("Failed to get the calfile file.");

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

  // Get the frame names for Tf2
  if (!nh.getParam("frames/parent", frame_parent_))
    ROS_FATAL("Failed to get frames/parent parameter.");
  if (!nh.getParam("frames/child", frame_child_))
    ROS_FATAL("Failed to get frames/child parameter.");

  // Get the topics for data topics
  std::string topic_pose, topic_twist;
  if (!nh.getParam("topics/pose", topic_pose))
    ROS_FATAL("Failed to get topics/pose parameter.");
  if (!nh.getParam("topics/twist", topic_twist))
    ROS_FATAL("Failed to get topics/twist parameter.");

  // Get the thresholds
  if (!nh.getParam("thresholds/angle", thresh_angle_))
    ROS_FATAL("Failed to get thresholds/angle parameter.");
  if (!nh.getParam("thresholds/duration", thresh_duration_))
    ROS_FATAL("Failed to get thresholds/duration parameter.");
  if (!nh.getParam("thresholds/count", thresh_count_))
    ROS_FATAL("Failed to get thresholds/count parameter.");

  // Whether to apply light corrections
  if (!nh.getParam("correct", correct_))
    ROS_FATAL("Failed to get correct parameter.");

  // Get the tracker update rate.
  if (!nh.getParam("rate", rate_))
    ROS_FATAL("Failed to get rate parameter.");

  // Get the tracker update rate.
  if (!nh.getParam("use/gyroscope", use_gyroscope_))
    ROS_FATAL("Failed to get use/gyroscope  parameter.");
  if (!nh.getParam("use/accelerometer", use_accelerometer_))
    ROS_FATAL("Failed to get use/accelerometer  parameter.");
  if (!nh.getParam("use/light", use_light_))
    ROS_FATAL("Failed to get use/light parameter.");

  // Get gravity
  if (!GetVectorParam(nh, "gravity", gravity_))
    ROS_FATAL("Failed to get gravity parameter.");

  // Tracking filter: Initial estimates
  UKF::Vector<3> est_position(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/position", est_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Quaternion est_attitude(1, 0, 0, 0);
  if (!GetQuaternionParam(nh, "initial_estimate/attitude", est_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> est_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/velocity", est_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> est_omega(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/omega", est_omega))
    ROS_FATAL("Failed to get angular_velocity parameter.");
  UKF::Vector<3> est_acceleration(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/acceleration", est_acceleration))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> est_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/alpha", est_alpha))
    ROS_FATAL("Failed to get alpha parameter.");

  // Tracking filter:  Initial covariances
  UKF::Vector<3> cov_position(0, 0, 0);
  if (!GetVectorParam(nh, "initial_cov/position", cov_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Vector<3> cov_attitude(0, 0, 0);
  if (!GetVectorParam(nh, "initial_cov/attitude", cov_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> cov_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "initial_cov/velocity", cov_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> cov_omega(0, 0, 0);
  if (!GetVectorParam(nh, "initial_cov/omega", cov_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> cov_accel(0, 0, 0);
  if (!GetVectorParam(nh, "initial_cov/acceleration", cov_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> cov_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "initial_cov/alpha", cov_alpha))
    ROS_FATAL("Failed to get acceleration parameter.");

  // Tracking filter:  Noise
  UKF::Vector<3> noise_position(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_cov/position", noise_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Vector<3> noise_attitude(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_cov/attitude", noise_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> noise_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_cov/velocity", noise_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> noise_omega(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_cov/omega", noise_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> noise_accel(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_cov/acceleration", noise_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> noise_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_cov/alpha", noise_alpha))
    ROS_FATAL("Failed to get acceleration parameter.");

  // Setup the filter
  filter_.state.set_field<Position>(est_position);
  filter_.state.set_field<Attitude>(est_attitude);
  filter_.state.set_field<Velocity>(est_velocity);
  filter_.state.set_field<Omega>(est_omega);
  filter_.state.set_field<Acceleration>(est_acceleration);
  filter_.state.set_field<Alpha>(est_alpha);
  filter_.covariance = State::CovarianceMatrix::Zero();
  filter_.covariance.diagonal() << 
    cov_position, cov_attitude,
    cov_velocity, cov_omega,
    cov_accel, cov_alpha;
  filter_.process_noise_covariance = State::CovarianceMatrix::Zero();
  filter_.process_noise_covariance.diagonal() <<
    noise_position, noise_attitude,
    noise_velocity, noise_omega,
    noise_accel, noise_alpha;

  // IMU error : initial estimate
  if (!GetVectorParam(nh, "measurement_cov/accelerometer", obs_cov_acc_))
    ROS_FATAL("Failed to get accelerometer parameter.");
  if (!GetVectorParam(nh, "measurement_cov/gyroscope", obs_cov_gyr_))
    ROS_FATAL("Failed to get gyroscope parameter.");
  if (!nh.getParam("measurement_cov/angle", obs_cov_ang_))
    ROS_FATAL("Failed to get angle parameter.");

  // IMU error : initial estimate
  if (!GetVectorParam(nh, "imu_initial_cov/acc_bias", imu_cov_ab_))
    ROS_FATAL("Failed to get acc_bias parameter.");
  if (!GetVectorParam(nh, "imu_initial_cov/acc_scale", imu_cov_as_))
    ROS_FATAL("Failed to get acc_scale parameter.");
  if (!GetVectorParam(nh, "imu_initial_cov/gyr_bias", imu_cov_gb_))
    ROS_FATAL("Failed to get gyr_bias parameter.");
  if (!GetVectorParam(nh, "imu_initial_cov/gyr_scale", imu_cov_gs_))
    ROS_FATAL("Failed to get gyr_scale parameter.");

  // IMU error : initial estimate
  if (!GetVectorParam(nh, "imu_process_noise_cov/acc_bias", imu_proc_ab_))
    ROS_FATAL("Failed to get acc_bias parameter.");
  if (!GetVectorParam(nh, "imu_process_noise_cov/acc_scale", imu_proc_as_))
    ROS_FATAL("Failed to get acc_scale parameter.");
  if (!GetVectorParam(nh, "imu_process_noise_cov/gyr_bias", imu_proc_gb_))
    ROS_FATAL("Failed to get gyr_bias parameter.");
  if (!GetVectorParam(nh, "imu_process_noise_cov/gyr_scale", imu_proc_gs_))
    ROS_FATAL("Failed to get gyr_scale parameter.");

  // If reading the configuration file results in inserting the correct
  // number of static transforms into the problem, then we can publish
  // the solution for use by other entities in the system.
  if (ReadConfig(calfile, lighthouses_, trackers_)) {
    ROS_INFO("Read transforms from calibration");
  } else {
    ROS_INFO("Could not read calibration file");
  }
  SendTransforms(frame_parent_, frame_child_, lighthouses_, trackers_);

  // Markers showing sensor positions
  pub_pose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>
    (topic_pose, 0);
  pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>
    (topic_twist, 0);

  // Subscribe to the motion and light callbacks
  std::vector<ros::Subscriber> subs;
  subs.push_back(nh.subscribe<deepdive_ros::Trackers>("/trackers", 1000,
    std::bind(TrackerCallback, std::placeholders::_1,
      std::ref(trackers_), NewTrackerCallback)));
  subs.push_back(nh.subscribe<deepdive_ros::Lighthouses>("/lighthouses", 1000,
    std::bind(LighthouseCallback, std::placeholders::_1,
      std::ref(lighthouses_), NewLighthouseCallback)));
  subs.push_back(nh.subscribe("/light", 1000, LightCallback));
  subs.push_back(nh.subscribe("/imu", 1000, ImuCallback));

  // Start a timer to callback
  ros::Timer timer = nh.createTimer(
    ros::Duration(ros::Rate(rate_)), TimerCallback, false, true);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
