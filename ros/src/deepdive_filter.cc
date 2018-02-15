/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// ROS includes
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

// General messages
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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

// STATE AND PROCESS MODEL

// State indexes
enum StateElement {
  Position,               // Position (world frame, m)
  Attitude,               // Attitude as a quaternion (world to body frame)
  Velocity,               // Velocity (world frame, m/s)
  Acceleration,           // Acceleration (body frame, m/s^2)
  Omega,                  // Angular velocity (body frame, m/s^2)
  GyroBias                // Gyro bias (body frame, rad/s)
};

// State vector
using State = UKF::StateVector<
  UKF::Field<Position, UKF::Vector<3>>,
  UKF::Field<Attitude, UKF::Quaternion>,
  UKF::Field<Velocity, UKF::Vector<3>>,
  UKF::Field<Acceleration, UKF::Vector<3>>,
  UKF::Field<Omega, UKF::Vector<3>>,
  UKF::Field<GyroBias, UKF::Vector<3>>
>;

// Measurement indices
enum MeasurementElement {
  Accelerometer,         // Acceleration (body frame, m/s^2)
  Gyroscope,             // Gyroscope (body frame, rads/s)
  Angle                  // Angle between lighthouse and sensor (radians)
};

// Measurement vector
using Measurement = UKF::DynamicMeasurementVector<
  UKF::Field<Accelerometer, UKF::Vector<3>>,
  UKF::Field<Gyroscope, UKF::Vector<3>>,
  UKF::Field<Angle, real_t>
>;

using Filter = UKF::Core<State, Measurement, UKF::IntegratorRK4>;

// Memory allocation for this tracker

std::map<std::string, deepdive_ros::Motor[2]> cal_; // Motor calibration
UKF::Vector<3> gravity_;                            // Gravity vector
std::string serial_;                                // Tracker serial
std::string frame_;                                 // Frame ID
Filter filter_;                                     // Filter
ros::Time last_(0);                                 // Last update time
std::vector<UKF::Vector<3>> extrinsics_;            // Extrinsics (tracking frame)
std::map<std::string, Eigen::Affine3d> lTw_;        // World -> Lighthouse transforms
Eigen::Affine3d iTt_;                               // Tracking -> IMU transform
Eigen::Affine3d hTt_;                               // Tracking -> body transform
UKF::Vector<3> gyr_s_;                              // Gyro scale / cross-coupline
UKF::Vector<3> gyr_b_;                              // Gyro bias
UKF::Vector<3> acc_s_;                              // Accel scale / cross-coupling
UKF::Vector<3> acc_b_;                              // Accel bias
bool ready_ = false;                                // Parameters received
bool initialized_ = false;                          // One light measurement
ros::Publisher pub_markers_;                        // Visualization publisher
ros::Publisher pub_pose_;                           // Pose publisher
ros::Publisher pub_twist_;                          // Twist publisher

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
  output.set_field<Omega>(UKF::Vector<3>(0, 0, 0));
  output.set_field<GyroBias>(UKF::Vector<3>(0, 0, 0));
  return output;
}

template <> template <>
UKF::Vector<3> Measurement::expected_measurement
  <State, Accelerometer>(const State& state, std::string const& l,
    uint8_t const& a, uint16_t const& s) {
    return state.get_field<Acceleration>()
      + state.get_field<Attitude>() * gravity_;
}

template <> template <>
UKF::Vector<3> Measurement::expected_measurement
  <State, Gyroscope>(const State& state, std::string const& l,
    uint8_t const& a, uint16_t const& s) {
    return state.get_field<Omega>() + state.get_field<GyroBias>();
}

template <> template <>
real_t Measurement::expected_measurement
  <State, Angle>(State const& state, std::string const& l,
    uint8_t const& a, uint16_t const& s) {
    // Position of the sensor in the IMU frame
    UKF::Vector<3> x = iTt_ * extrinsics_[s];
    // Position of the sensor in the world frame
    x = state.get_field<Attitude>().conjugate() * x
      + state.get_field<Position>();
    // Position of the sensor in the lighthouse frame
    x = lTw_[l].inverse() * x;
    // Get the angles in X and Y
    UKF::Vector<2> angles;
    angles[0] =  std::atan2(x[0], x[2]);
    angles[1] = -std::atan2(x[1], x[2]);
    // Apply the corrections
    deepdive_ros::Motor & motor = cal_[l][a];
    angles[a] += motor.phase;
    angles[a] += motor.tilt * angles[1-a];
    angles[a] += motor.curve * angles[1-a] * angles[1-a];
    angles[a] += motor.gibmag * std::cos(angles[1-a] + motor.gibphase);
    // Vertical or horizontal angle
    return angles[a];
}

// The angle error is about 1mm over 10m, therefore tan(1/100)^2 = 1e-08
template <>
Measurement::CovarianceVector Measurement::measurement_covariance(
  (Measurement::CovarianceVector() << 
    1.0e-4, 1.0e-4, 1.0e-4,   // Accel
    3.0e-6, 3.0e-6, 3.0e-6,   // Gyro
    1.0e-8).finished());      // Range
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

// Calculate a delta t and return if it is smaller than 1s
bool Delta(ros::Time const& now, double & dt) {
  dt = (now - last_).toSec();
  last_ = now;
  // ROS_INFO_STREAM(dt);
  return (dt > 0 && dt < 1.0);
}

void TimerCallback(ros::TimerEvent const& info) {
  if (!initialized_) return;

  // Check that we are being called fast enough
  static double dt;
  if (!Delta(ros::Time::now(), dt))
    return;

  // Propagate the filter forward
  filter_.a_priori_step(dt);

  // Get the transformation from the world to the tracking head
  static Eigen::Affine3d tTw, iTw;
  iTw.translation() =
    filter_.state.get_field<Position>();
  iTw.linear() =
    filter_.state.get_field<Attitude>().toRotationMatrix();
  tTw = iTt_.inverse() * iTw;
  Eigen::Quaterniond q(tTw.linear());

  // Calculate omega

  // Broadcast the tracker pose on TF2
  static tf2_ros::TransformBroadcaster br;
  static geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = "world";
  tf.child_frame_id = frame_;
  tf.transform.translation.x = tTw.translation()[0];
  tf.transform.translation.y = tTw.translation()[1];
  tf.transform.translation.z = tTw.translation()[2];
  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  br.sendTransform(tf);

  // Broadcast the pose (world -> body transform)
  static geometry_msgs::PoseStamped pose;
  pose.header = tf.header;
  pose.pose.position.x = tTw.translation()[0];
  pose.pose.position.y = tTw.translation()[1];
  pose.pose.position.z = tTw.translation()[2];
  pose.pose.orientation.w = q.w();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pub_pose_.publish(pose);

  // Broadcast the twist (in the world frame)
  // static geometry_msgs::TwistStamped twist;
  // twist.header = tf.header;
  // twist.twist.linear.x = tTw.translation()[0];
  // twist.twist.linear.y = tTw.translation()[1];
  // twist.twist.linear.z = tTw.translation()[2];
  // twist.twist.angular.x = q.x();
  // twist.twist.angular.y = q.y();
  // twist.twist.angular.z = q.z();
  // pub_pose_.publish(pose);

  // Publish the static markers
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(extrinsics_.size());
  for (uint16_t i = 0; i < extrinsics_.size(); i++) {
    visualization_msgs::Marker & marker = msg.markers[i];
    marker.header.frame_id = frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "deepdive";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = extrinsics_[i](0);
    marker.pose.position.y = extrinsics_[i](1);
    marker.pose.position.z = extrinsics_[i](2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  pub_markers_.publish(msg);
}

// This will be called at approximately 120Hz
// - Single lighthouse in 'A' mode : 120Hz (60Hz per axis)
// - Dual lighthouses in b/A or b/c modes : 120Hz (30Hz per axis)
void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  if (!ready_ || msg->header.frame_id != serial_) return;

  // Check that we are being called fast enough
  static double dt;
  if (!Delta(ros::Time::now(), dt))
    return;

  // Check that we have lighthouse calibration information
  if (cal_.find(msg->lighthouse) == cal_.end())
    return;

  // Check that the solver has produced a lighthouse transform
  static tf2_ros::Buffer buffer;
  static tf2_ros::TransformListener listener(buffer);
  try {
    geometry_msgs::TransformStamped tf;
    tf = buffer.lookupTransform("world", msg->lighthouse, ros::Time(0));
    Eigen::Affine3d & T = lTw_[msg->lighthouse];
    T.translation() = Eigen::Vector3d(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);
    T.linear() = Eigen::Quaterniond(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z).toRotationMatrix();
  }
  catch (tf2::TransformException &ex) {
    return;
  }

  // Process update
  filter_.a_priori_step(dt);

  // Innovation updates - one for each pulse angle
  for (size_t i = 0; i < msg->pulses.size(); i++) {
    Measurement measurement;
    measurement.set_field<Angle>(msg->pulses[i].angle);
    filter_.innovation_step(measurement, msg->lighthouse,
      msg->axis, msg->pulses[i].sensor);
  }
 
  // Correction step 
  filter_.a_posteriori_step();
 
  // We are now initialized
  initialized_ = true;
}

// This will be called at approximately 250Hz
void ImuCallback(sensor_msgs::Imu::ConstPtr const& msg) {
  if (!ready_ || msg->header.frame_id != serial_) return;

  // Check that we are being called fast enough
  static double dt;
  if (!Delta(ros::Time::now(), dt))
    return;

  // Grab the accelerometer and gyro data
  static UKF::Vector<3> acc, gyr;
  Convert(msg->linear_acceleration, acc);
  Convert(msg->angular_velocity, gyr);

  // Scale and convert
  acc = acc_s_.asDiagonal() * acc + acc_b_;
  gyr = gyr_s_.asDiagonal() * gyr + gyr_b_;

  // A priori step
  filter_.a_priori_step(dt);
  
  // Measurement update
  Measurement measurement;
  measurement.set_field<Accelerometer>(acc);
  measurement.set_field<Gyroscope>(gyr);
  filter_.innovation_step(measurement, std::string(""),
    static_cast<uint8_t>(0), static_cast<uint16_t>(0));

  // A posterori step
  filter_.a_posteriori_step();
}

// This will be called once on startup by a latched topic
void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg) {
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    if (it->serial == serial_) {
      // Copy the extrinsics
      extrinsics_.resize(it->sensors.size());
      for (uint16_t s = 0; s < it->sensors.size(); s++)
        Convert(it->sensors[s].position, extrinsics_[s]);
      // Copy the IMU calibration parameters
      Convert(it->gyr_bias, gyr_b_);
      Convert(it->acc_bias, acc_b_);
      Convert(it->gyr_scale, gyr_s_);
      Convert(it->acc_scale, acc_s_);
      // Tracker -> IMU transform
      iTt_.translation() = Eigen::Vector3d(
        it->imu_transform.translation.x,
        it->imu_transform.translation.y,
        it->imu_transform.translation.z);
      iTt_.linear() = Eigen::Quaterniond(
        it->imu_transform.rotation.w,
        it->imu_transform.rotation.x,
        it->imu_transform.rotation.y,
        it->imu_transform.rotation.z).toRotationMatrix();
      // Tracker to head transform
      hTt_.translation() = Eigen::Vector3d(
        it->head_transform.translation.x,
        it->head_transform.translation.y,
        it->head_transform.translation.z);
      hTt_.linear() = Eigen::Quaterniond(
        it->head_transform.rotation.w,
        it->head_transform.rotation.x,
        it->head_transform.rotation.y,
        it->head_transform.rotation.z).toRotationMatrix();
      // Broadcast static transforms for tracker -> head / imu
      static tf2_ros::StaticTransformBroadcaster broadcaster;
      geometry_msgs::TransformStamped tfs;
      tfs.header.stamp = ros::Time::now();
      tfs.header.frame_id = frame_;
      tfs.child_frame_id = frame_ + "/imu";
      tfs.transform = it->imu_transform;
      broadcaster.sendTransform(tfs);
      tfs.child_frame_id = frame_ + "/head";
      tfs.transform = it->head_transform;
      broadcaster.sendTransform(tfs);
      // ROS_INFO_STREAM("Tracker " << serial_ << " initialized");
      ready_ = true;
    }
  }
}

// This will be called once on startup by a latched topic
void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg) {
  std::vector<deepdive_ros::Lighthouse>::const_iterator it;
  for (it = msg->lighthouses.begin(); it != msg->lighthouses.end(); it++) {
    for (uint8_t a = 0; a < it->motors.size(); a++)
      cal_[it->serial][a] = it->motors[a];
    // ROS_INFO_STREAM("Lighthouse " << it->serial << " initialized");
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
  if (!nh.getParam(name, tmp) || tmp.size() != 4)
    return false;
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

  // Get the topics for publishing
  std::string topic_pose, topic_twist;
  if (!nh.getParam("topics/pose", topic_pose))
    ROS_FATAL("Failed to get topics/pose parameter.");
  if (!nh.getParam("topics/twist", topic_twist))
    ROS_FATAL("Failed to get topics/twist parameter.");  

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
  UKF::Vector<3> est_acceleration(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/acceleration", est_acceleration))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> est_omega(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/omega", est_omega))
    ROS_FATAL("Failed to get omega parameter.");
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
  UKF::Vector<3> cov_accel(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/acceleration", cov_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> cov_omega(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/omega", cov_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> cov_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/gyro_bias", cov_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");
  UKF::Vector<3> cov_accel_bias(0, 0, 0);

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
  UKF::Vector<3> noise_accel(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/acceleration", noise_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> noise_omega(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/omega", noise_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> noise_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/gyro_bias", noise_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");

  // Setup the filter
  filter_.state.set_field<Position>(est_position);
  filter_.state.set_field<Attitude>(est_attitude);
  filter_.state.set_field<Velocity>(est_velocity);
  filter_.state.set_field<Acceleration>(est_acceleration);
  filter_.state.set_field<Omega>(est_omega);
  filter_.state.set_field<GyroBias>(est_gyro_bias);
  filter_.covariance = State::CovarianceMatrix::Zero();
  filter_.covariance.diagonal() <<
    cov_position[0], cov_position[1], cov_position[2],
    cov_attitude[0], cov_attitude[1], cov_attitude[2],
    cov_velocity[0], cov_velocity[1], cov_velocity[2],
    cov_accel[0], cov_accel[1], cov_accel[2],
    cov_omega[0], cov_omega[1], cov_omega[2],
    cov_gyro_bias[0], cov_gyro_bias[1], cov_gyro_bias[2];
  filter_.process_noise_covariance = State::CovarianceMatrix::Zero();
  filter_.process_noise_covariance.diagonal() <<
    noise_position[0], noise_position[1], noise_position[2],
    noise_attitude[0], noise_attitude[1], noise_attitude[2],
    noise_velocity[0], noise_velocity[1], noise_velocity[2],
    noise_accel[0], noise_accel[1], noise_accel[2],
    noise_omega[0], noise_omega[1], noise_omega[2],
    noise_gyro_bias[0], noise_gyro_bias[1], noise_gyro_bias[2];

  // Start a timer to callback
  ros::Timer timer = nh.createTimer(
    ros::Duration(ros::Rate(rate)), TimerCallback, false, true);

  // Subscribe to the motion and light callbacks
  ros::Subscriber sub_lighthouse  =
    nh.subscribe("/lighthouses", 10, LighthouseCallback);
  ros::Subscriber sub_tracker  =
    nh.subscribe("/trackers", 10, TrackerCallback);
  ros::Subscriber sub_imu =
    nh.subscribe("/imu", 10, ImuCallback);
  ros::Subscriber sub_light =
    nh.subscribe("/light", 10, LightCallback);

  // Markers showing sensor positions
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/sensors", 0);
  pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>(topic_pose, 0);
  pub_twist_ = nh.advertise<geometry_msgs::TwistStamped>(topic_twist, 0);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
