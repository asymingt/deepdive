/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// ROS includes
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

// Standard messages
#include <sensor_msgs/Imu.h>

// Tracker information
#include <deepdive_ros/Trackers.h>

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

// STATE AND PROCESS MODEL

// State indexes
enum StateElement {
  Position,               // Position (world frame, m)
  Attitude,               // Attitude as a quaternion (world to body frame)
  Velocity,               // Velocity (world frame, m/s)
  Acceleration,           // Acceleration (body frame, m/s^2)
  Omega,                  // Angular velocity (body frame, rad/s)
  Alpha,                  // Angular acceleration (body frame, rad/s^2)
  GyroBias,               // Gyro bias (body frame, rad/s)
  AccelBias               // Accel bias (body frame, m/s^2)
};

// State vector
using State = UKF::StateVector<
  UKF::Field<Position, UKF::Vector<3>>,
  UKF::Field<Attitude, UKF::Quaternion>,
  UKF::Field<Velocity, UKF::Vector<3>>,
  UKF::Field<Acceleration, UKF::Vector<3>>,
  UKF::Field<Omega, UKF::Vector<3>>,
  UKF::Field<Alpha, UKF::Vector<3>>,
  UKF::Field<GyroBias, UKF::Vector<3>>,
  UKF::Field<AccelBias, UKF::Vector<3>>
>;

// Measurement indices
enum MeasurementElement {
  Accelerometer,          // Acceleration (body frame, m/s^2)
  Gyroscope,              // Gyroscope (body frame, rads/s)
};

// Measurement vector
using Measurement = UKF::DynamicMeasurementVector<
  UKF::Field<Accelerometer, UKF::Vector<3>>,
  UKF::Field<Gyroscope, UKF::Vector<3>>
>;

using Filter = UKF::Core<State, Measurement, UKF::IntegratorRK4>;

// IMU CALIBRATION MODEL

struct Calibration {
  double ab[3] = {0.0, 0.0, 0.0};   // Accelerometter bias
  double as[3] = {1.0, 1.0, 1.0};   // Accelerometer scale
  double gb[3] = {0.0, 0.0, 0.0};   // Gyroscope bias
  double gs[3] = {1.0, 1.0, 1.0};   // Gyroscope scale
};

// MEMORY ALLOCATION

std::string serial_, frame_;

Filter filter_;

Measurement measurement_;

Calibration calibration_;

UKF::Vector<3> gravity_ = UKF::Vector<3>(0, 0, -9.80665);

// Process and measurment models

namespace UKF {

template <> template <>
State State::derivative<>() const {
  State output;
  output.set_field<Position>(get_field<Velocity>());
  output.set_field<Velocity>(
    get_field<Attitude>().conjugate() * get_field<Acceleration>());
  output.set_field<Acceleration>(UKF::Vector<3>(0, 0, 0));
  UKF::Quaternion omega_q;
  omega_q.vec() = get_field<Omega>() * 0.5;
  omega_q.w() = 0;
  output.set_field<Attitude>(omega_q.conjugate() * get_field<Attitude>());
  output.set_field<Omega>(get_field<Alpha>());
  output.set_field<Alpha>(UKF::Vector<3>(0, 0, 0));
  output.set_field<GyroBias>(UKF::Vector<3>(0, 0, 0));
  output.set_field<AccelBias>(UKF::Vector<3>(0, 0, 0));
  return output;
}

template <> template <>
UKF::Vector<3> Measurement::expected_measurement
  <State, Accelerometer>(const State& state) {
    return state.get_field<Acceleration>()
      + state.get_field<Attitude>() * gravity_
      + state.get_field<AccelBias>();
}

template <> template <>
UKF::Vector<3> Measurement::expected_measurement
  <State, Gyroscope>(const State& state) {
    return state.get_field<Omega>() + state.get_field<GyroBias>();
}

template <>
Measurement::CovarianceVector Measurement::measurement_covariance(
  (Measurement::CovarianceVector() << 
    2.3e-5, 2.3e-5, 2.3e-5, 3e-6, 3e-6, 3e-6).finished());
}

// ROS CALLBACKS

void TimerCallback(ros::TimerEvent const& info) {
  // Check that we are being called fast enough
  double dt = (info.current_real - info.last_real).toSec();
  if (dt > 1.0)
    return;
  // Step the filter forward
  filter_.step(dt, measurement_);
  // Clear the measurement
  measurement_ = Measurement();
  // Broadcast the tracker pose
  static tf2_ros::TransformBroadcaster br;
  static geometry_msgs::TransformStamped tf;
  tf.header.stamp = info.current_real;
  tf.header.frame_id = "world";
  tf.child_frame_id = frame_;
  tf.transform.translation.x = filter_.state.get_field<Position>()[0];
  tf.transform.translation.y = filter_.state.get_field<Position>()[1];
  tf.transform.translation.z = filter_.state.get_field<Position>()[2];
  tf.transform.rotation.w = filter_.state.get_field<Attitude>().w();
  tf.transform.rotation.x = filter_.state.get_field<Attitude>().x();
  tf.transform.rotation.y = filter_.state.get_field<Attitude>().y();
  tf.transform.rotation.z = filter_.state.get_field<Attitude>().z();
  br.sendTransform(tf);
}

void ImuCallback(sensor_msgs::Imu::ConstPtr const& msg) {
  // Ignore data from other trackers
  if (msg->header.frame_id != serial_) return;
  /*** CODE BELOW USED TO MEASURE IMU RATE of 250Hz **********************
  static double count = 0.0;
  static double ratev = 0.0;
  static double tdiff = 0.0;
  static ros::Time ltime = msg->header.stamp;
  tdiff = (msg->header.stamp - ltime).toSec();
  ratev = (ratev * count + tdiff) / (count + 1.0);
  count = count + 1.0;
  ltime = msg->header.stamp;
  if (ratev > 0)
    ROS_INFO_STREAM(1.0/ratev);
  ************************************************************************/
  // Add the calibrated
  measurement_.set_field<Accelerometer>(UKF::Vector<3>(
    msg->linear_acceleration.x * calibration_.as[0] + calibration_.ab[0],
    msg->linear_acceleration.y * calibration_.as[1] + calibration_.ab[1],
    msg->linear_acceleration.z * calibration_.as[2] + calibration_.ab[2]));
  measurement_.set_field<Gyroscope>(UKF::Vector<3>(
    msg->angular_velocity.x * calibration_.gs[0] + calibration_.gb[0],
    msg->angular_velocity.y * calibration_.gs[1] + calibration_.gb[1],
    msg->angular_velocity.z * calibration_.gs[2] + calibration_.gb[2]));
}

void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg) {
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    // Ignore data from other trackers
    if (it->serial != serial_)
      continue;
    // See if we have tracker data
    ROS_INFO("Tracker data received");
    // Copy the IMU calibration parameters into the tracking filter
    Convert(it->acc_bias, calibration_.ab);
    Convert(it->acc_scale, calibration_.as);
    Convert(it->gyr_bias, calibration_.gb);
    Convert(it->gyr_scale, calibration_.gs);
  }
}

// HELPER FUNCTIONS FOR CONFIG

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
  if (!nh.getParam(name, tmp) || tmp.size() != 4) {
    ROS_INFO_STREAM(tmp.size());
    return false;
  }
  data.x() = tmp[0];
  data.y() = tmp[1];
  data.z() = tmp[2];
  data.w() = tmp[3];
  return true;
}

// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_filter");
  ros::NodeHandle nh("~");

  // Get some global information
  if (!nh.getParam("serial", serial_))
    ROS_FATAL("Failed to get serial parameter.");
  if (!nh.getParam("frame", frame_))
    ROS_FATAL("Failed to get frame parameter.");
  if (!GetVectorParam(nh, "gravity", gravity_))
    ROS_FATAL("Failed to get gravity parameter.");

  // Get the tracker update rate. Anything over the IMU rate is really not
  // adding much, since we don't have a good dynamics model.
  double rate = 100;
  if (!nh.getParam("rate",rate))
    ROS_FATAL("Failed to get rate parameter.");

  // Initial estimates
  UKF::Vector<3> est_position(0, 0, 0);
  if (!GetVectorParam(nh, "estimate/position", est_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Quaternion est_attitude(1, 0, 0, 0);
  if (!GetQuaternionParam(nh, "estimate/attitude", est_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> est_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "estimate/velocity", est_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> est_acceleration(0, 0, 0);
  if (!GetVectorParam(nh, "estimate/acceleration", est_acceleration))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> est_omega(0, 0, 0);
  if (!GetVectorParam(nh, "estimate/omega", est_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> est_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "estimate/alpha", est_alpha))
    ROS_FATAL("Failed to get alpha parameter.");
  UKF::Vector<3> est_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "estimate/gyro_bias", est_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");
  UKF::Vector<3> est_accel_bias(0, 0, 0);
  if (!GetVectorParam(nh, "estimate/accel_bias", est_accel_bias))
    ROS_FATAL("Failed to get accel_bias parameter.");

  // Initial covariances
  UKF::Vector<3> cov_position(0, 0, 0);
  if (!GetVectorParam(nh, "covariance/position", cov_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Vector<3> cov_attitude(0, 0, 0);
  if (!GetVectorParam(nh, "covariance/attitude", cov_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> cov_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "covariance/velocity", cov_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> cov_accel(0, 0, 0);
  if (!GetVectorParam(nh, "covariance/acceleration", cov_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> cov_omega(0, 0, 0);
  if (!GetVectorParam(nh, "covariance/omega", cov_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> cov_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "covariance/alpha", cov_alpha))
    ROS_FATAL("Failed to get alpha parameter.");
  UKF::Vector<3> cov_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "covariance/gyro_bias", cov_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");
  UKF::Vector<3> cov_accel_bias(0, 0, 0);
  if (!GetVectorParam(nh, "covariance/accel_bias", cov_accel_bias))
    ROS_FATAL("Failed to get accel_bias parameter.");

  // Noise
  UKF::Vector<3> noise_position(0, 0, 0);
  if (!GetVectorParam(nh, "noise/position", noise_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Vector<3> noise_attitude(0, 0, 0);
  if (!GetVectorParam(nh, "noise/attitude", noise_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> noise_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "noise/velocity", noise_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> noise_accel(0, 0, 0);
  if (!GetVectorParam(nh, "noise/acceleration", noise_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> noise_omega(0, 0, 0);
  if (!GetVectorParam(nh, "noise/omega", noise_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> noise_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "noise/alpha", noise_alpha))
    ROS_FATAL("Failed to get alpha parameter.");
  UKF::Vector<3> noise_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "noise/gyro_bias", noise_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");
  UKF::Vector<3> noise_accel_bias(0, 0, 0);
  if (!GetVectorParam(nh, "noise/accel_bias", noise_accel_bias))
    ROS_FATAL("Failed to get accel_bias parameter.");

  // Setup the filter
  filter_.state.set_field<Position>(est_position);
  filter_.state.set_field<Attitude>(est_attitude);
  filter_.state.set_field<Velocity>(est_velocity);
  filter_.state.set_field<Acceleration>(est_acceleration);
  filter_.state.set_field<Omega>(est_omega);
  filter_.state.set_field<Alpha>(est_alpha);
  filter_.state.set_field<GyroBias>(UKF::Vector<3>(0, 0, 0));
  filter_.state.set_field<AccelBias>(UKF::Vector<3>(0, 0, 0));
  filter_.covariance = State::CovarianceMatrix::Zero();
  filter_.covariance.diagonal() <<
    cov_position[0], cov_position[1], cov_position[2],
    cov_attitude[0], cov_attitude[1], cov_attitude[2],
    cov_velocity[0], cov_velocity[1], cov_velocity[2],
    cov_accel[0], cov_accel[1], cov_accel[2],
    cov_omega[0], cov_omega[1], cov_omega[2],
    cov_alpha[0], cov_alpha[1], cov_alpha[2],
    cov_gyro_bias[0], cov_gyro_bias[1], cov_gyro_bias[2],
    cov_accel_bias[0], cov_accel_bias[1], cov_accel_bias[2];
  filter_.process_noise_covariance = State::CovarianceMatrix::Zero();
  filter_.process_noise_covariance.diagonal() <<
    noise_position[0], noise_position[1], noise_position[2],
    noise_attitude[0], noise_attitude[1], noise_attitude[2],
    noise_velocity[0], noise_velocity[1], noise_velocity[2],
    noise_accel[0], noise_accel[1], noise_accel[2],
    noise_omega[0], noise_omega[1], noise_omega[2],
    noise_alpha[0], noise_alpha[1], noise_alpha[2],
    noise_gyro_bias[0], noise_gyro_bias[1], noise_gyro_bias[2],
    noise_accel_bias[0], noise_accel_bias[1], noise_accel_bias[2];

  // Zero measurement
  measurement_.set_field<Gyroscope>(
    UKF::Vector<3>(0, 0, 0));
  measurement_.set_field<Accelerometer>(
    UKF::Vector<3>(0, 0, 0));

  // Start a timer to callback
  ros::Timer timer = nh.createTimer(
    ros::Duration(ros::Rate(rate)), TimerCallback, false, true);

  // Subscribe to the motion and light callbacks
  ros::Subscriber sub_tracker  =
    nh.subscribe("/trackers", 10, TrackerCallback);
  ros::Subscriber sub_imu =
    nh.subscribe("/imu", 10, ImuCallback);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
