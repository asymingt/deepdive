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

// STATE AND PROCESS MODEL

// State indexes
enum StateElement {
  Position,               // Position (world frame, m)
  Attitude,               // Attitude as a quaternion (world to body frame)
  Velocity,               // Velocity (world frame, m/s)
  AngularVelocity,        // Angular velocity (world frame, rads/s)
  Acceleration,           // Acceleration (world frame, m/s^2)
  GyroBias                // Gyro bias (body frame, rad/s)
};

// State vector
using State = UKF::StateVector<
  UKF::Field<Position, UKF::Vector<3>>,
  UKF::Field<Attitude, UKF::Quaternion>,
  UKF::Field<Velocity, UKF::Vector<3>>,
  UKF::Field<AngularVelocity, UKF::Vector<3>>,
  UKF::Field<Acceleration, UKF::Vector<3>>,
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
std::string frame_parent_;                          // Fixed frame
std::string frame_child_;                           // Child frame
Filter filter_;                                     // Filter
ros::Time last_(0);                                 // Last update time
std::vector<UKF::Vector<3>> extrinsics_;            // Extrinsics (tracking frame)
std::map<std::string, Eigen::Affine3d> wTl_;        // Lighthouse -> world transforms
Eigen::Affine3d tTh_;                               // Head -> tracking transform
Eigen::Affine3d iTt_;                               // Tracking -> IMU transform
UKF::Vector<3> gyr_s_;                              // Gyro scale / cross-coupline
UKF::Vector<3> gyr_b_;                              // Gyro bias
UKF::Vector<3> acc_s_;                              // Accel scale / cross-coupling
UKF::Vector<3> acc_b_;                              // Accel bias
bool ready_ = false;                                // Parameters received
bool initialized_ = false;                          // One light measurement
bool correct_imu_ = false;                          // Calibrate IMU
bool correct_light_ = false;                        // Calibrate light
ros::Publisher pub_markers_;                        // Visualization publisher
ros::Publisher pub_pose_;                           // Pose publisher
ros::Publisher pub_twist_;                          // Twist publisher
double thresh_angle_;                               // Threshold on angle
double thresh_duration_;                            // Threshold on duration

namespace UKF {

template <> template <>
State State::derivative<>() const {
  State output;
  output.set_field<Position>(get_field<Velocity>());
  output.set_field<Velocity>(get_field<Acceleration>());
  UKF::Quaternion angvel_q;
  angvel_q.vec() = get_field<AngularVelocity>() * 0.5;
  angvel_q.w() = 0;
  output.set_field<Attitude>(angvel_q);
  output.set_field<AngularVelocity>(UKF::Vector<3>(0, 0, 0));
  output.set_field<Acceleration>(UKF::Vector<3>(0, 0, 0));
  output.set_field<GyroBias>(UKF::Vector<3>(0, 0, 0));
  return output;
}

template <> template <>
UKF::Vector<3> Measurement::expected_measurement
  <State, Accelerometer>(const State& state, std::string const& l,
    uint8_t const& a, uint16_t const& s) {
    return state.get_field<Attitude>()
      * (gravity_ + state.get_field<Acceleration>());
}

template <> template <>
UKF::Vector<3> Measurement::expected_measurement
  <State, Gyroscope>(const State& state, std::string const& l,
    uint8_t const& a, uint16_t const& s) {
    return state.get_field<Attitude>() * state.get_field<AngularVelocity>()
      + state.get_field<GyroBias>();
}

template <> template <>
real_t Measurement::expected_measurement
  <State, Angle>(State const& state, std::string const& l,
    uint8_t const& a, uint16_t const& s) {
    // Convert the current state estimate to a transform
    static Eigen::Affine3d wTi;
    wTi.translation() = state.get_field<Position>();
    wTi.linear() = state.get_field<Attitude>().toRotationMatrix();
    // Get the position of the sensor in the lighthouse frame
    static UKF::Vector<3> x;
    x = wTl_[l].inverse() * wTi * iTt_ * extrinsics_[s];
    // Get the angles in X and Y
    UKF::Vector<2> angles;
    angles[0] = std::atan2(x[0], x[2]);
    angles[1] = std::atan2(x[1], x[2]);
    // Apply the corrections
    if (correct_light_) {
      deepdive_ros::Motor & motor = cal_[l][a];
      angles[a] += motor.phase;
      angles[a] += motor.tilt * angles[1-a];
      angles[a] += motor.curve * angles[1-a] * angles[1-a];
      angles[a] += motor.gibmag * std::sin(angles[1-a] + motor.gibphase);
    }
    // Vertical or horizontal angle
    return angles[a];
}

// The aceleration error is (1e-02)^2 meters/sec^2
// The angular velocity error is (1e-03)^2 rads/sec
// The angle error is about 1mm over 10m, therefore atan(1/10000)^2 = 1e-08
template <>
Measurement::CovarianceVector Measurement::measurement_covariance(
  (Measurement::CovarianceVector() << 
    1.0e-4, 1.0e-4, 1.0e-4,   // Accel
    1.0e-6, 1.0e-6, 1.0e-6,   // Gyro
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

  // The filter expresses all quantitued as relationships between IMU and the world
  // frames. We now need to transform these quantities to the mechanical frame,
  // which should match the diagram in the Developer Guidelines 1.3 PDF page 5.
  // https://dl.vive.com/Tracker/Guideline/HTC_Vive_Tracker_Developer_Guidelines_v1.3.pdf
  Eigen::Affine3d wTi = Eigen::Affine3d::Identity();
  wTi.translation() = filter_.state.get_field<Position>();
  wTi.linear() = filter_.state.get_field<Attitude>().toRotationMatrix();
  Eigen::Affine3d iTh = iTt_ * tTh_;
  Eigen::Affine3d wTh = wTi * iTh;
  UKF::Vector<3> linvel =
    iTh.linear().inverse() * filter_.state.get_field<Velocity>();
  UKF::Vector<3> angvel =
    iTh.linear().inverse() * filter_.state.get_field<AngularVelocity>();

  // Transform the covariances to reflect the coordinate frame change. See:
  // https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance
  Eigen::Matrix<double, 12, 12> P = filter_.covariance.block<12, 12>(0, 0);
  Eigen::Matrix<double, 12, 12> R = Eigen::Matrix<double, 12, 12>::Identity();
  R.block<3,3>(0, 0) = iTh.linear().inverse();
  R.block<3,3>(3, 3) = R.block<3,3>(0, 0);
  R.block<3,3>(6, 6) = R.block<3,3>(0, 0);
  R.block<3,3>(9, 9) = R.block<3,3>(0, 0);
  P = R * P * R.transpose();

  // The filter relates WORLD and IMU frames
  static std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_parent_;

  // Broadcast the tracker pose on TF2
  static tf2_ros::TransformBroadcaster br;
  static geometry_msgs::TransformStamped tf;
  tf.header = header;
  tf.child_frame_id = frame_child_;
  tf.transform.translation.x = wTh.translation()[0];
  tf.transform.translation.y = wTh.translation()[1];
  tf.transform.translation.z = wTh.translation()[2];
  Eigen::Quaterniond q(wTh.linear());
  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  br.sendTransform(tf);

  // Broadcast the pose with covariance
  static geometry_msgs::PoseWithCovarianceStamped pwcs;
  pwcs.header = header;
  pwcs.pose.pose.position.x = tf.transform.translation.x;
  pwcs.pose.pose.position.y = tf.transform.translation.y;
  pwcs.pose.pose.position.z = tf.transform.translation.z;
  pwcs.pose.pose.orientation = tf.transform.rotation;
  for (size_t i = 0; i < 6; i++)
    for (size_t j = 0; j < 6; j++)
      pwcs.pose.covariance[i*6 + j] = P(i, j);
  pub_pose_.publish(pwcs);

  // Broadcast the twist with covariance
  static geometry_msgs::TwistWithCovarianceStamped twcs;
  twcs.header = header;
  twcs.twist.twist.linear.x = linvel[0];
  twcs.twist.twist.linear.y = linvel[1];
  twcs.twist.twist.linear.z = linvel[2];
  twcs.twist.twist.angular.x = angvel[0];
  twcs.twist.twist.angular.y = angvel[1];
  twcs.twist.twist.angular.z = angvel[2];
  for (size_t i = 0; i < 6; i++)
    for (size_t j = 0; j < 6; j++)
      twcs.twist.covariance[i*6 + j] = P(6 + i, 6 + j);
  pub_twist_.publish(twcs);

  // Publish the static markers
  static visualization_msgs::MarkerArray msg;
  msg.markers.resize(extrinsics_.size());
  for (uint16_t i = 0; i < extrinsics_.size(); i++) {
    visualization_msgs::Marker & marker = msg.markers[i];
    marker.header.frame_id = frame_child_ + "/light";
    marker.header.stamp = ros::Time::now();
    marker.ns = frame_child_;
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
    tf = buffer.lookupTransform(frame_parent_, msg->lighthouse, ros::Time(0));
    Eigen::Affine3d & T = wTl_[msg->lighthouse];
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
    if (fabs(msg->pulses[i].angle) > thresh_angle_) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on angle");
      return;
    }
    if (fabs(msg->pulses[i].duration) < thresh_duration_) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on duration");
      return;
    }
    // Package up the measurement
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

  // Scale and convert IMU
  if (correct_light_) {
    acc = acc_s_.asDiagonal() * acc + acc_b_;
    gyr = gyr_s_.asDiagonal() * gyr + gyr_b_;
  }

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
      // Tracking to head frame transform
      Eigen::Affine3d hTt = Eigen::Affine3d::Identity();
      hTt.translation() = Eigen::Vector3d(
        it->head_transform.translation.x,
        it->head_transform.translation.y,
        it->head_transform.translation.z);
      hTt.linear() = Eigen::Quaterniond(
        it->head_transform.rotation.w,
        it->head_transform.rotation.x,
        it->head_transform.rotation.y,
        it->head_transform.rotation.z).toRotationMatrix();
      // Tracking to IMU frame transform
      Eigen::Affine3d iTt = Eigen::Affine3d::Identity();
      iTt.translation() = Eigen::Vector3d(
        it->imu_transform.translation.x,
        it->imu_transform.translation.y,
        it->imu_transform.translation.z);
      iTt.linear() = Eigen::Quaterniond(
        it->imu_transform.rotation.w,
        it->imu_transform.rotation.x,
        it->imu_transform.rotation.y,
        it->imu_transform.rotation.z).toRotationMatrix();
      // Get the two derived transforms that we care about
      tTh_ = hTt.inverse();
      iTt_ = iTt;
      // Broadcast static transforms for tracker -> head / imu
      static tf2_ros::StaticTransformBroadcaster broadcaster;
      static Eigen::Quaterniond q;
      static geometry_msgs::TransformStamped tfs;
      tfs.header.stamp = ros::Time::now();
      tfs.header.frame_id = frame_child_;
      tfs.child_frame_id = frame_child_ + "/light";
      tfs.transform.translation.x = tTh_.translation()[0];
      tfs.transform.translation.y = tTh_.translation()[1];
      tfs.transform.translation.z = tTh_.translation()[2];
      q = tTh_.linear();
      tfs.transform.rotation.w = q.w();
      tfs.transform.rotation.x = q.x();
      tfs.transform.rotation.y = q.y();
      tfs.transform.rotation.z = q.z();
      broadcaster.sendTransform(tfs);
      // Send the light -> IMU transform
      tfs.header.stamp = ros::Time::now();
      tfs.header.frame_id = frame_child_ + "/light";
      tfs.child_frame_id = frame_child_ + "/imu";
      tfs.transform.translation.x = iTt_.translation()[0];
      tfs.transform.translation.y = iTt_.translation()[1];
      tfs.transform.translation.z = iTt_.translation()[2];
      q = iTt_.linear();
      tfs.transform.rotation.w = q.w();
      tfs.transform.rotation.x = q.x();
      tfs.transform.rotation.y = q.y();
      tfs.transform.rotation.z = q.z();
      broadcaster.sendTransform(tfs);
      // ROS_INFO_STREAM("Tracker " << serial_ << " initialized");
      ready_ = true;
    }
  }
}

// This will be called once on startup by a latched topic
void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg) {
  std::vector<deepdive_ros::Lighthouse>::const_iterator it;
  for (it = msg->lighthouses.begin(); it != msg->lighthouses.end(); it++)
    for (uint8_t a = 0; a < it->motors.size(); a++)
      cal_[it->serial][a] = it->motors[a];
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
  if (!GetVectorParam(nh, "gravity", gravity_))
    ROS_FATAL("Failed to get gravity parameter.");

  // Get the frame names for Tf2
  if (!nh.getParam("frames/parent", frame_parent_))
    ROS_FATAL("Failed to get frames/parent parameter.");
  if (!nh.getParam("frames/child", frame_child_))
    ROS_FATAL("Failed to get frames/child parameter.");  

  // Get the topics for data topics
  std::string topic_pose, topic_twist, topic_sensors;
  if (!nh.getParam("topics/pose", topic_pose))
    ROS_FATAL("Failed to get topics/pose parameter.");
  if (!nh.getParam("topics/twist", topic_twist))
    ROS_FATAL("Failed to get topics/twist parameter.");  
  if (!nh.getParam("topics/sensors", topic_sensors))
    ROS_FATAL("Failed to get topics/sensora parameter.");  

  // Get the thresholds
  if (!nh.getParam("thresholds/angle", thresh_angle_))
    ROS_FATAL("Failed to get thresholds/angle parameter.");
  if (!nh.getParam("thresholds/duration", thresh_duration_))
    ROS_FATAL("Failed to get thresholds/duration parameter.");

  // Apply corrections?
  if (!nh.getParam("correct/imu", correct_imu_))
    ROS_FATAL("Failed to get correct/imu parameter.");
  if (!nh.getParam("correct/light", correct_light_))
    ROS_FATAL("Failed to get correct/light parameter.");

  // Use data?
  bool use_imu, use_light;
  if (!nh.getParam("use/imu", use_imu))
    ROS_FATAL("Failed to get use/imu parameter.");
  if (!nh.getParam("use/light", use_light))
    ROS_FATAL("Failed to get use/light parameter.");

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
  UKF::Vector<3> est_angvel(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/angvel", est_angvel))
    ROS_FATAL("Failed to get angular_velocity parameter.");
  UKF::Vector<3> est_acceleration(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/acceleration", est_acceleration))
    ROS_FATAL("Failed to get acceleration parameter.");
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
  UKF::Vector<3> cov_angvel(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/angvel", cov_angvel))
    ROS_FATAL("Failed to get angular_velocity parameter.");
  UKF::Vector<3> cov_accel(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/acceleration", cov_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
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
  UKF::Vector<3> noise_angvel(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/angvel", noise_angvel))
    ROS_FATAL("Failed to get angular_velocity parameter.");
  UKF::Vector<3> noise_accel(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/acceleration", noise_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> noise_gyro_bias(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/gyro_bias", noise_gyro_bias))
    ROS_FATAL("Failed to get gyro_bias parameter.");

  // Setup the filter
  filter_.state.set_field<Position>(est_position);
  filter_.state.set_field<Attitude>(est_attitude);
  filter_.state.set_field<Velocity>(est_velocity);
  filter_.state.set_field<AngularVelocity>(est_angvel);
  filter_.state.set_field<Acceleration>(est_acceleration);
  filter_.state.set_field<GyroBias>(est_gyro_bias);
  filter_.covariance = State::CovarianceMatrix::Zero();
  filter_.covariance.diagonal() <<
    cov_position[0], cov_position[1], cov_position[2],
    cov_attitude[0], cov_attitude[1], cov_attitude[2];
    cov_velocity[0], cov_velocity[1], cov_velocity[2],
    cov_angvel[0], cov_angvel[1], cov_angvel[2],
    cov_accel[0], cov_accel[1], cov_accel[2],
    cov_gyro_bias[0], cov_gyro_bias[1], cov_gyro_bias[2];
  filter_.process_noise_covariance = State::CovarianceMatrix::Zero();
  filter_.process_noise_covariance.diagonal() <<
    noise_position[0], noise_position[1], noise_position[2],
    noise_attitude[0], noise_attitude[1], noise_attitude[2];
    noise_velocity[0], noise_velocity[1], noise_velocity[2],
    noise_angvel[0], noise_angvel[1], noise_angvel[2],
    noise_accel[0], noise_accel[1], noise_accel[2],
    noise_gyro_bias[0], noise_gyro_bias[1], noise_gyro_bias[2];

  // Start a timer to callback
  ros::Timer timer = nh.createTimer(
    ros::Duration(ros::Rate(rate)), TimerCallback, false, true);

  // Subscribe to the motion and light callbacks
  ros::Subscriber sub_lighthouse  =
    nh.subscribe("/lighthouses", 10, LighthouseCallback);
  ros::Subscriber sub_tracker  =
    nh.subscribe("/trackers", 10, TrackerCallback);
  ros::Subscriber sub_light;
  ros::Subscriber sub_imu;
  if (use_imu)
    sub_imu = nh.subscribe("/imu", 10, ImuCallback);
  if (use_light)
    sub_light = nh.subscribe("/light", 10, LightCallback);

  // Markers showing sensor positions
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>
    (topic_sensors, 0);
  pub_pose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>
    (topic_pose, 0);
  pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>
    (topic_twist, 0);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
