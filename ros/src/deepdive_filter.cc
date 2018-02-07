/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// ROS includes
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

// Data messages
#include <sensor_msgs/Imu.h>
#include <deepdive_ros/Trackers.h>
#include <deepdive_ros/Light.h>

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
  GyroBias                // Gyro bias (body frame, rad/s)
};

// State vector
using State = UKF::StateVector<
  UKF::Field<Position, UKF::Vector<3>>,
  UKF::Field<Attitude, UKF::Quaternion>,
  UKF::Field<Velocity, UKF::Vector<3>>,
  UKF::Field<GyroBias, UKF::Vector<3>>
>;

// Measurement indices
enum MeasurementElement {
  Angle                  // Angle between lighthouse and sensor (radians)
};

// Measurement vector
using Measurement = UKF::FixedMeasurementVector<
  UKF::Field<Angle, real_t>
>;

using Filter = UKF::Core<State, Measurement, UKF::IntegratorRK4>;

// IMU CALIBRATION MODEL

static constexpr size_t MAX_NUM_SENSORS = 32;

struct Tracker {
  // Metadata 
  std::string serial;                           // Serial number
  std::string frame;                            // Frame
  // Filter internals
  Filter filter;                                // UKF
  // Cached measurements
  Measurement imu;                              // IMU measurement
  deepdive_ros::Light measurement;              // Light pulses
  // World data
  UKF::Vector<3> gravity;                       // Gravity vector
  // Calibration data (read from the tracker)
  double ab[3] = {0.0, 0.0, 0.0};               // Accelerometter bias
  double as[3] = {1.0, 1.0, 1.0};               // Accelerometer scale
  double gb[3] = {0.0, 0.0, 0.0};               // Gyroscope bias
  double gs[3] = {1.0, 1.0, 1.0};               // Gyroscope scale
  UKF::Vector<3> extrinsics[MAX_NUM_SENSORS];   // Sensor extrinsics
  UKF::Vector<3> normals[MAX_NUM_SENSORS];      // Sensor normals
  UKF::Vector<3> iTt;                           // Translation: Tracker -> IMU
  UKF::Quaternion iRt;                          // Rotation: Tracker -> IMU
  UKF::Vector<3> hTt;                           // Translation: Tracker -> IMU
  UKF::Quaternion hRt;                          // Rotation: Tracker -> HEAD
  // Interation information
  bool initialized = false;                     // Wait for light data
  bool gset = false;                            // Number of bias measurements taken
  size_t sensor = 0;                            // Current axis
};

// MEMORY ALLOCATION

static Tracker tracker_;

// Process and measurment models

namespace UKF {

template <> template <>
State State::derivative<>(const UKF::Vector<3>& acc,
                          const UKF::Vector<3>& gyr) const {
  State output;
  output.set_field<Position>(get_field<Velocity>());
  output.set_field<Velocity>(get_field<Attitude>().conjugate() * acc);
  UKF::Quaternion omega_q;
  omega_q.vec() = gyr * 0.5;
  omega_q.w() = 0;
  output.set_field<Attitude>(omega_q.conjugate() * get_field<Attitude>());
  output.set_field<GyroBias>(UKF::Vector<3>(0, 0, 0));
  return output;
}

template <> template <>
real_t Measurement::expected_measurement
  <State, Angle>(State const& s) {
    UKF::Vector<3> x = tracker_.extrinsics[tracker_.sensor];
    x = s.get_field<Attitude>().conjugate() * x + s.get_field<Position>();
    return (tracker_.measurement.axis ? -atan2(x[1], x[2]) : atan2(x[0], x[2]));
}

// The angle error is about 1mm over 10m, therefore tan(1/100)^2 = 1e-08
template <>
Measurement::CovarianceVector Measurement::measurement_covariance(
  (Measurement::CovarianceVector() << 1.0e-8).finished());      // Range
}

// ROS CALLBACKS

void Convert(geometry_msgs::Quaternion const& from, UKF::Quaternion & to) {
  to.w() = from.w;
  to.x() = from.x;
  to.y() = from.y;
  to.z() = from.z;
}

void Convert(geometry_msgs::Point const& from, UKF::Vector<3> & to) {
  to[0] = from.x;
  to[1] = from.y;
  to[2] = from.z;
}

void Convert(geometry_msgs::Vector3 const& from, UKF::Vector<3> & to) {
  to[0] = from.x;
  to[1] = from.y;
  to[2] = from.z;
}

// This will be called at approximately 120Hz
// - Single lighthouse in 'A' mode : 120Hz (60Hz per axis)
// - Dual lighthouses in b/A or b/c modes : 120Hz (30Hz per axis)
void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  // Ignore data from other trackers
  if (msg->header.frame_id != tracker_.serial)
    return;
  // Add measurements
  static Measurement measurement;
  for (size_t i = 0; i < tracker_.measurement.pulses.size(); i++) {
    tracker_.sensor = tracker_.measurement.pulses[i].sensor;
    measurement.set_field<Angle>(tracker_.measurement.pulses[i].angle);
    tracker_.filter.innovation_step(measurement);
  }
  tracker_.filter.a_posteriori_step();
  // Broadcast the tracker pose
  static tf2_ros::TransformBroadcaster br;
  static geometry_msgs::TransformStamped tf;
  tf.header = msg->header;
  tf.child_frame_id = tracker_.frame;
  tf.transform.translation.x = tracker_.filter.state.get_field<Position>()[0];
  tf.transform.translation.y = tracker_.filter.state.get_field<Position>()[1];
  tf.transform.translation.z = tracker_.filter.state.get_field<Position>()[2];
  tf.transform.rotation.w = tracker_.filter.state.get_field<Attitude>().w();
  tf.transform.rotation.x = tracker_.filter.state.get_field<Attitude>().x();
  tf.transform.rotation.y = tracker_.filter.state.get_field<Attitude>().y();
  tf.transform.rotation.z = tracker_.filter.state.get_field<Attitude>().z();
  br.sendTransform(tf);
}

// This will be called at approximately 250Hz
void ImuCallback(sensor_msgs::Imu::ConstPtr const& msg) {
  // Ignore data from other trackers
  if (msg->header.frame_id != tracker_.serial)
    return;
  // Check that we are being called fast enough
  static ros::Time last(0);
  double dt = (msg->header.stamp - last).toSec();
  last = msg->header.stamp;
  if (dt > 1.0)
    return;
  // Add the accelerometer and gyro data
  UKF::Vector<3> acc(
    tracker_.as[0] * msg->linear_acceleration.x - tracker_.ab[0],
    tracker_.as[1] * msg->linear_acceleration.y - tracker_.ab[1],
    tracker_.as[2] * msg->linear_acceleration.z - tracker_.ab[2]);
  UKF::Vector<3> gyr(
    tracker_.gs[0] * msg->angular_velocity.x - tracker_.gb[0],
    tracker_.gs[1] * msg->angular_velocity.y - tracker_.gb[1],
    tracker_.gs[2] * msg->angular_velocity.z - tracker_.gb[2]);
  // A priori step
  tracker_.filter.a_priori_step(dt, acc, gyr);
}

// This will be called once on startup by a latched topic
void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg) {
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    // Ignore data from other trackers
    if (it->serial != tracker_.serial)
      continue;
    // Copy the IMU calibration parameters
    Convert(it->acc_bias, tracker_.ab);
    Convert(it->acc_scale, tracker_.as);
    Convert(it->gyr_bias, tracker_.gb);
    Convert(it->gyr_scale, tracker_.gs);
    // Copy over the IMU and HEAD transforms
    Convert(it->imu_transform.translation, tracker_.iTt);
    Convert(it->imu_transform.rotation, tracker_.iRt);
    Convert(it->head_transform.translation, tracker_.hTt);
    Convert(it->head_transform.rotation, tracker_.hRt);
    // Copy over the sensor exstrincis and normals
    for (size_t i = 0; i < it->sensors.size(); i++) {
      Convert(it->sensors[i].position, tracker_.extrinsics[i]);
      Convert(it->sensors[i].normal, tracker_.normals[i]);
    }
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
  if (!nh.getParam("serial", tracker_.serial))
    ROS_FATAL("Failed to get serial parameter.");
  if (!nh.getParam("frame", tracker_.frame))
    ROS_FATAL("Failed to get frame parameter.");
  if (!GetVectorParam(nh, "gravity", tracker_.gravity))
    ROS_FATAL("Failed to get gravity parameter.");

  // Get the tracker update rate. Anything over the IMU rate is really not
  // adding much, since we don't have a good dynamics model.
  double rate = 100;
  if (!nh.getParam("rate",rate))
    ROS_FATAL("Failed to get rate parameter.");

  // Initial estimates
  UKF::Vector<3> est_position(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/position", est_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Quaternion est_attitude(1, 0, 0, 0);
  if (!GetQuaternionParam(nh, "initial_estimate/attitude", est_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> est_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/velocity", est_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> est_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/gyro_bias", est_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");

  // Initial covariances
  UKF::Vector<3> cov_position(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/position", cov_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Vector<3> cov_attitude(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/attitude", cov_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> cov_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/velocity", cov_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> cov_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/gyro_bias", cov_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");

  // Noise
  UKF::Vector<3> noise_position(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/position", noise_position))
    ROS_FATAL("Failed to get position parameter.");
  UKF::Vector<3> noise_attitude(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/attitude", noise_attitude))
    ROS_FATAL("Failed to get attitude parameter.");
  UKF::Vector<3> noise_velocity(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/velocity", noise_velocity))
    ROS_FATAL("Failed to get velocity parameter.");
  UKF::Vector<3> noise_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/gyro_bias", noise_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");

  // Setup the filter
  Filter & filter = tracker_.filter;
  filter.state.set_field<Position>(est_position);
  filter.state.set_field<Attitude>(est_attitude);
  filter.state.set_field<Velocity>(est_velocity);
  filter.state.set_field<GyroBias>(est_gyro_bias);
  filter.covariance = State::CovarianceMatrix::Zero();
  filter.covariance.diagonal() <<
    cov_position[0], cov_position[1], cov_position[2],
    cov_attitude[0], cov_attitude[1], cov_attitude[2],
    cov_velocity[0], cov_velocity[1], cov_velocity[2],
    cov_gyro_bias[0], cov_gyro_bias[1], cov_gyro_bias[2];
  filter.process_noise_covariance = State::CovarianceMatrix::Zero();
  filter.process_noise_covariance.diagonal() <<
    noise_position[0], noise_position[1], noise_position[2],
    noise_attitude[0], noise_attitude[1], noise_attitude[2],
    noise_velocity[0], noise_velocity[1], noise_velocity[2],
    noise_gyro_bias[0], noise_gyro_bias[1], noise_gyro_bias[2];

  // Subscribe to the motion and light callbacks
  ros::Subscriber sub_tracker  =
    nh.subscribe("/trackers", 10, TrackerCallback);
  ros::Subscriber sub_imu =
    nh.subscribe("/imu", 10, ImuCallback);
  ros::Subscriber sub_light =
    nh.subscribe("/light", 10, LightCallback);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
