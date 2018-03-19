/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// ROS includes
#include <ros/ros.h>

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
  Position,                     // Position (world frame, m)
  Attitude,                     // Attitude as a quaternion (world to body frame)
  Velocity,                     // Velocity (world frame, m/s)
  Omega,                        // Angular velocity (body frame, rads/s)
  Acceleration,                 // Acceleration (body frame, m/s^2)
  Alpha,                        // Angular acceleration (body frame, rads/s^2)
  // ERRORS
  GyroscopeBias,                // Gyroscope bias offset (body frame, rad/s)
  GyroscopeScale,               // Gyroscope scale factor (body frame, multiplier)
  AccelerometerBias,            // Accelerometer bias offset (body frame, m/s^2)
  AccelerometerScale,           // Accelerometer scale factor (body frame, mutliplier)
  // MEASUREMENTS
  Accelerometer,                // Acceleration (body frame, m/s^2)
  Gyroscope,                    // Gyroscope (body frame, rads/s)
  Angle
};

// OBSERVATION

// Observation vector
using Observation = UKF::DynamicMeasurementVector<
  UKF::Field<Accelerometer, UKF::Vector<3>>,
  UKF::Field<Gyroscope, UKF::Vector<3>>,
  UKF::Field<Angle, real_t>
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
using ErrorFilter =
  UKF::SquareRootParameterEstimationCore<Error, Observation>;

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
using TrackingFilter =
  UKF::SquareRootCore<State, Observation, UKF::IntegratorRK4>;

// For IMU parameter estimation
typedef std::map<std::string, ErrorFilter> ErrorMap;
typedef std::map<std::string, Eigen::Affine3d> TransformMap;

// GLOBAL DATA STRUCTURES

ErrorMap errors_;                    // Errors for each tracker
LighthouseMap lighthouses_;          // List of lighthouses
TrackerMap trackers_;                // List of trackers
TrackingFilter filter_;              // Tracking filter
std::string frame_parent_;           // Parent frame, eg "world"
std::string frame_child_;            // Child frame, eg "truth"
double rate_;                        // Desired tracking rate
int thresh_count_ = 4;               // Min num measurements required per bundle
double thresh_angle_ = 60.0;         // Angle threshold in degrees
double thresh_duration_ = 1.0;       // Duration threshold in micorseconds

// ROS publishers

ros::Publisher pub_pose_;
ros::Publisher pub_twist_;

// Default IMU errors
Eigen::Vector3d imu_cov_ab_;         // Initial covariance: Accel bias
Eigen::Vector3d imu_cov_as_;         // Initial covariance: Accel scale
Eigen::Vector3d imu_cov_gb_;         // Initial covariance: Gyro bias
Eigen::Vector3d imu_cov_gs_;         // Initial covariance: Gyro scale
Eigen::Vector3d imu_proc_ab_;        // Process noise: Accel bias
Eigen::Vector3d imu_proc_as_;        // Process noise: Accel scale
Eigen::Vector3d imu_proc_gb_;        // Process noise: Gyro bias
Eigen::Vector3d imu_proc_gs_;        // Process noise: Gyro scale

// Default measurement errors
bool correct_ = false;               // Whether to correct light parameters
Eigen::Vector3d gravity_;            // Gravity
Eigen::Vector3d obs_cov_acc_;        // Measurement covariance: Accelerometer
Eigen::Vector3d obs_cov_gyr_;        // Measurement covariance: Gyroscope
double obs_cov_ang_;                 // Measurement covariance: Angle

// Intermediary data
TransformMap lTw_;                      // ALL: world -> lighthouse
TransformMap bTi_;                      // ALL: imu -> body
TransformMap bTt_;                      // ALL: tracking -> body
std::string lighthouse_;                // Active lighthouse
std::string tracker_;                   // Active tracker
Eigen::Vector3d sensor_;                // Active sensor
uint8_t axis_;                          // Active axis

// TRACKING FILTER

namespace UKF {
  namespace Parameters {
    template <> constexpr real_t AlphaSquared<State> = 1e-2;
    template <> constexpr real_t Kappa<State> = 3.0;
    template <> constexpr real_t AlphaSquared<Error> = 1.0;
    template <> constexpr real_t Kappa<Error> = 3.0;
  }

  template <>
  Observation::CovarianceVector Observation::measurement_root_covariance(
    (Observation::CovarianceVector() << 
      obs_cov_acc_, obs_cov_gyr_, obs_cov_ang_).finished());

  template <> template <> State
  State::derivative<>() const {
    State output;
    output.set_field<Position>(get_field<Velocity>());
    output.set_field<Velocity>(
      get_field<Attitude>().conjugate() * get_field<Acceleration>());
    UKF::Quaternion omega_q;
    omega_q.vec() = get_field<Omega>() * 0.5;
    omega_q.w() = 0;
    output.set_field<Attitude>(omega_q.conjugate() * get_field<Attitude>());
    output.set_field<Omega>(get_field<Alpha>());
    output.set_field<Acceleration>(UKF::Vector<3>(0, 0, 0));
    output.set_field<Alpha>(UKF::Vector<3>(0, 0, 0));
    return output;
  }

  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<Error, Accelerometer, State>(
    Error const& errors, State const& state) {
    UKF::Vector<3> const& w = state.get_field<Omega>();
    UKF::Vector<3> const& p = bTi_[tracker_].translation();
    return errors.get_field<AccelerometerScale>().cwiseProduct(
      bTi_[tracker_].linear().inverse() * w.cross(w.cross(p))
        + state.get_field<Acceleration>()
        + errors.get_field<AccelerometerBias>()
        + state.get_field<Attitude>() * gravity_);
  }

  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<Error, Gyroscope, State>(
    Error const& errors, State const& state) {
    return errors.get_field<GyroscopeScale>().cwiseProduct(
      bTi_[tracker_].linear().inverse() * state.get_field<Omega>()
        + errors.get_field<GyroscopeBias>());
  }

  template <> template <> real_t
  Observation::expected_measurement<Error, Angle, State>(
  Error const& errors, State const& state) {
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

  template <> template <> Error
  Error::derivative<>() const {
    return Error::Zero();
  }

  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<State, Accelerometer, Error>(
    State const& state, Error const& errors) {
    UKF::Vector<3> const& w = state.get_field<Omega>();
    UKF::Vector<3> const& p = bTi_[tracker_].translation();
    return errors.get_field<AccelerometerScale>().cwiseProduct(
      bTi_[tracker_].linear().inverse() * w.cross(w.cross(p))
        + state.get_field<Acceleration>()
        + errors.get_field<AccelerometerBias>()
        + state.get_field<Attitude>() * gravity_);
  }

  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<State, Gyroscope, Error>(
    State const& state, Error const& errors) {
    return errors.get_field<GyroscopeScale>().cwiseProduct(
      bTi_[tracker_].linear().inverse() * state.get_field<Omega>()
        + errors.get_field<GyroscopeBias>());
  }

  template <> template <> real_t
  Observation::expected_measurement<State, Angle, Error>(
    State const& state, Error const& errors) {
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

}

// UTILITY FUNCTIONS

double Delta(ros::Time const& now) {
  static ros::Time last = ros::Time::now();
  double dt = (now - last).toSec();
  last = now;
  if (dt > rate_ * 2)
    ROS_WARN_STREAM_THROTTLE(1, "Made dt call at > 2x target period");
  return (dt > 0 ? dt : rate_);
}

// CALLBACKS

// This will be called at approximately 120Hz
// - Single lighthouse in 'A' mode : 120Hz (60Hz per axis)
// - Dual lighthouses in b/A or b/c modes : 120Hz (30Hz per axis)
void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  double dt = Delta(ros::Time::now());

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
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on angle");
      return;
    }
    if (fabs(msg->pulses[i].duration) < thresh_duration_ / 1e-6) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on duration");
      return;
    }
    if (msg->pulses[i].sensor >= NUM_SENSORS) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on invalid sensor id");
      return;
    }
    data.push_back(msg->pulses[i]);
  }
  if (data.size() < thresh_count_)
    ROS_INFO_STREAM_THROTTLE(1, "Not enough data so skipping bundle.");

  // Set the context correctly
  tracker_ = msg->header.frame_id;
  lighthouse_ = msg->lighthouse;
  axis_ = msg->axis;

  // Correct the error filter
  error->second.a_priori_step();
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
  double dt = Delta(ros::Time::now());

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
  obs.set_field<Accelerometer>(acc);
  obs.set_field<Gyroscope>(gyr);

  // Propagate the IMU error filter using the tracking filter state
  error->second.a_priori_step();
  error->second.innovation_step(obs, filter_.state);
  error->second.a_posteriori_step(); 

  // Propagate the filter
  filter_.a_priori_step(dt);
  filter_.innovation_step(obs, error->second.state);
  filter_.a_posteriori_step();
}

// This will be called back at the desired tracking rate
void TimerCallback(ros::TimerEvent const& info) {
  double dt = Delta(ros::Time::now());
  // Propagate the filter forward
  filter_.a_priori_step(dt);

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
      pwcs.pose.covariance[i*6 + j] = filter_.root_covariance(i, j)
                                    * filter_.root_covariance(i, j);
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
      twcs.twist.covariance[i*6 + j] = filter_.root_covariance(6+i, 6+j)
                                     * filter_.root_covariance(6+i, 6+j);
  pub_twist_.publish(twcs);
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
  wTl.linear() = Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix();
  // Cache a transform
  lTw_[lighthouse->first] = wTl.inverse();
}

// Called when a new tracker appears
void NewTrackerCallback(TrackerMap::iterator tracker) {
  ROS_INFO_STREAM("Found tracker " << tracker->first);
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
  error.root_covariance = Error::CovarianceMatrix::Zero();
  error.root_covariance.diagonal() <<
    imu_cov_ab_, imu_cov_as_, imu_cov_gb_, imu_cov_gs_;
  error.process_noise_root_covariance = Error::CovarianceMatrix::Zero();
  error.process_noise_root_covariance.diagonal() <<
    imu_proc_ab_, imu_proc_as_, imu_proc_gb_, imu_proc_gs_;
  ///////////////////////
  // Cache a transform //
  ///////////////////////
  Eigen::Vector3d v;
  Eigen::Affine3d bTh, tTh, tTi;
  // head -> body
  v = Eigen::Vector3d(
    tracker->second.bTh[3], tracker->second.bTh[4], tracker->second.bTh[5]);
  bTh.translation() = Eigen::Vector3d(
    tracker->second.bTh[0], tracker->second.bTh[1], tracker->second.bTh[2]);
  bTh.linear() = Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix();
  // head -> light
  v = Eigen::Vector3d(
    tracker->second.tTh[3], tracker->second.tTh[4], tracker->second.tTh[5]);
  tTh.translation() = Eigen::Vector3d(
    tracker->second.tTh[0], tracker->second.tTh[1], tracker->second.tTh[2]);
  tTh.linear() = Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix();
  // imu -> light
  v = Eigen::Vector3d(
    tracker->second.tTi[3], tracker->second.tTi[4], tracker->second.tTi[5]);
  tTi.translation() =Eigen::Vector3d(
    tracker->second.tTi[0], tracker->second.tTi[1], tracker->second.tTi[2]);
  tTi.linear() = Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix();
  // Global cache
  bTt_[tracker->first] = bTh * tTh.inverse();
  bTi_[tracker->first] = bTh * tTh.inverse() * tTi;
}

// MAIN ENTRY POINT OF APPLICATION

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
  double rate = 100;
  if (!nh.getParam("rate", rate))
    ROS_FATAL("Failed to get rate parameter.");

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
  if (!GetVectorParam(nh, "initial_root_cov/position", cov_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Vector<3> cov_attitude(0, 0, 0);
  if (!GetVectorParam(nh, "initial_root_cov/attitude", cov_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> cov_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "initial_root_cov/velocity", cov_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> cov_omega(0, 0, 0);
  if (!GetVectorParam(nh, "initial_root_cov/omega", cov_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> cov_accel(0, 0, 0);
  if (!GetVectorParam(nh, "initial_root_cov/acceleration", cov_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> cov_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "initial_root_cov/alpha", cov_alpha))
    ROS_FATAL("Failed to get alpha parameter.");

  // Tracking filter:  Noise
  UKF::Vector<3> noise_position(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_root_cov/position", noise_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Vector<3> noise_attitude(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_root_cov/attitude", noise_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> noise_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_root_cov/velocity", noise_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> noise_omega(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_root_cov/omega", noise_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> noise_accel(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_root_cov/acceleration", noise_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> noise_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise_root_cov/alpha", noise_alpha))
    ROS_FATAL("Failed to get alpha parameter.");

  // Setup the filter
  filter_.state.set_field<Position>(est_position);
  filter_.state.set_field<Attitude>(est_attitude);
  filter_.state.set_field<Velocity>(est_velocity);
  filter_.state.set_field<Omega>(est_omega);
  filter_.state.set_field<Acceleration>(est_acceleration);
  filter_.state.set_field<Alpha>(est_alpha);
  filter_.root_covariance = State::CovarianceMatrix::Zero();
  filter_.root_covariance.diagonal() << cov_position, cov_attitude, cov_velocity,
    cov_omega, cov_accel, cov_alpha;
  filter_.process_noise_root_covariance = State::CovarianceMatrix::Zero();
  filter_.process_noise_root_covariance.diagonal() << noise_position, noise_attitude,
    noise_velocity, noise_omega, noise_accel, noise_alpha;

  // IMU error : initial estimate
  if (!GetVectorParam(nh, "imu_initial_root_cov/acc_bias", imu_cov_ab_))
    ROS_FATAL("Failed to get acc_bias parameter.");
  if (!GetVectorParam(nh, "imu_initial_root_cov/acc_scale", imu_cov_as_))
    ROS_FATAL("Failed to get acc_scale parameter.");
  if (!GetVectorParam(nh, "imu_initial_root_cov/gyr_bias", imu_cov_gb_))
    ROS_FATAL("Failed to get gyr_bias parameter.");
  if (!GetVectorParam(nh, "imu_initial_root_cov/gyr_scale", imu_cov_gs_))
    ROS_FATAL("Failed to get gyr_scale parameter.");

  // IMU error : initial estimate
  if (!GetVectorParam(nh, "imu_process_noise_root_cov/acc_bias", imu_proc_ab_))
    ROS_FATAL("Failed to get acc_bias parameter.");
  if (!GetVectorParam(nh, "imu_process_noise_root_cov/acc_scale", imu_proc_as_))
    ROS_FATAL("Failed to get acc_scale parimu_process_noise_root_covameter.");
  if (!GetVectorParam(nh, "imu_process_noise_root_cov/gyr_bias", imu_proc_gb_))
    ROS_FATAL("Failed to get gyr_bias parameter.");
  if (!GetVectorParam(nh, "imu_process_noise_root_cov/gyr_scale", imu_proc_gs_))
    ROS_FATAL("Failed to get gyr_scale parameter.");

  // IMU error : initial estimate
  if (!GetVectorParam(nh, "measurement_root_cov/accelerometer", obs_cov_acc_))
    ROS_FATAL("Failed to get accelerometer parameter.");
  if (!GetVectorParam(nh, "measurement_root_cov/gyroscope", obs_cov_gyr_))
    ROS_FATAL("Failed to get gyroscope parameter.");
  
  if (!nh.getParam("measurement_root_cov/angle", obs_cov_ang_))
    ROS_FATAL("Failed to get angle parameter.");

  // If reading the configuration file results in inserting the correct
  // number of static transforms into the problem, then we can publish
  // the solution for use by other entities in the system.
  if (ReadConfig(calfile, lighthouses_, trackers_)) {
    ROS_INFO("Read transforms from calibration");
  } else {
    ROS_INFO("Could not read calibration file");
  }

  // Subscribe to the motion and light callbacks
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
  ros::Subscriber sub_imu =
    nh.subscribe("/imu", 1000, ImuCallback);
  
  // Markers showing sensor positions
  pub_pose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>
    (topic_pose, 0);
  pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>
    (topic_twist, 0);

  // Start a timer to callback
  ros::Timer timer = nh.createTimer(
    ros::Duration(ros::Rate(rate)), TimerCallback, false, true);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
