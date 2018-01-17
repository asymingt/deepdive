/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// ROS includes
#include <ros/ros.h>

// UKF includes
#include <UKF/Types.h>
#include <UKF/Integrator.h>
#include <UKF/StateVector.h>
#include <UKF/MeasurementVector.h>
#include <UKF/Core.h>

// Standard messages
#include <sensor_msgs/Imu.h>

// Non-standard messages
#include <deepdive_ros/Light.h>

// Various gravitational 
static UKF::Vector<3> GRAVITY = UKF::Vector<3>(0, 0, -9.80665);

// STATE AND PROCESS MODEL

// State indexes
enum States {
  Position,               // Position (world frame, m)
  Attitude,               // Attitude as a quaternion (world to body frame)
  Velocity,               // Velocity (world frame, m/s)
  Acceleration,           // Acceleration (body frame, m/s^2)
  AngularVelocity,        // Angular velocity (body frame, rad/s)
  AngularAcceleration,    // Angular acceleration (body frame, rad/s^2)
  GyroBias,               // Gyro bias (body frame, rad/s)
  AccelBias               // Accel bias (body frame, m/s^2)
};

// State vector
using StateVector = UKF::StateVector<
  UKF::Field<Position, UKF::Vector<3>>,
  UKF::Field<Attitude, UKF::Quaternion>,
  UKF::Field<Velocity, UKF::Vector<3>>,
  UKF::Field<Acceleration, UKF::Vector<3>>,
  UKF::Field<AngularVelocity, UKF::Vector<3>>,
  UKF::Field<AngularAcceleration, UKF::Vector<3>>,
  UKF::Field<GyroBias, UKF::Vector<3>>,
  UKF::Field<AccelBias, UKF::Vector<3>>
>;

template <> template <>
StateVector StateVector::derivative<>() const {
  StateVector output;
  output.set_field<Position>(get_field<Velocity>());
  output.set_field<Velocity>(
    get_field<Attitude>().conjugate() * get_field<Acceleration>());
  output.set_field<Acceleration>(UKF::Vector<3>(0, 0, 0));
  UKF::Quaternion omega_q;
  omega_q.vec() = get_field<AngularVelocity>() * 0.5;
  omega_q.w() = 0;
  output.set_field<Attitude>(omega_q.conjugate() * get_field<Attitude>());
  output.set_field<AngularVelocity>(get_field<AngularAcceleration>());
  output.set_field<AngularAcceleration>(UKF::Vector<3>(0, 0, 0));
  output.set_field<GyroBias>(UKF::Vector<3>(0, 0, 0));
  output.set_field<AccelBias>(UKF::Vector<3>(0, 0, 0));
  return output;
}

// OBSERVATIONS AND MEASURMEMENT MODEL

// Measurement indices
enum Measurements {
  Accelerometer,          // Acceleration (body frame, m/s^2)
  Gyroscope,              // Gyroscope (body frame, rads/s)
  Quaternion,             // Current orientation
  Location                // Current location
};

// Measurment vector
using MeasurementVector = UKF::DynamicMeasurementVector<
  UKF::Field<Accelerometer, UKF::Vector<3>>,
  UKF::Field<Gyroscope, UKF::Vector<3>>,
  UKF::Field<Quaternion, UKF::Quaternion>,
  UKF::Field<Location, UKF::Vector<3>>
>;

template <> template <>
UKF::Vector<3> MeasurementVector::expected_measurement
  <StateVector, Accelerometer>(const StateVector& state) {
    return state.get_field<Acceleration>()
      + state.get_field<Attitude>() * GRAVITY
      + state.get_field<AccelBias>();
}

template <> template <>
UKF::Vector<3> MeasurementVector::expected_measurement
  <StateVector, Gyroscope>(const StateVector& state) {
    return state.get_field<AngularVelocity>() + state.get_field<GyroBias>();
}

template <> template <>
UKF::Quaternion MeasurementVector::expected_measurement
  <StateVector, Quaternion>(const StateVector& state) {
    return state.get_field<Attitude>();
}

template <> template <>
UKF::Vector<3> MeasurementVector::expected_measurement
  <StateVector, Location>(const StateVector& state) {
    return state.get_field<Position>();
}

// ALLOCATE THE UKF

using Filter = UKF::Core<
  StateVector,
  MeasurementVector,
  UKF::IntegratorRK4
>;

struct FilterInstance {
  Filter ukf;
  MeasurementVector measurement;
  std::map<ros::Time, geometry_msgs::PoseWithCovariance> registrations;
};

static std::map<std::string, FilterInstance> filters_;

// CALLBACKS

// When a light measurement is received, immediately store the 
void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  // Get a reference to the registration for this tracker/time
  Filter & filter = filters_[mgs->header.frame_id].filter;
  geometry_msgs::Pose & registration =
    filters_[mgs->header.frame_id].registrations[msg->header.stamp];
  // Get the state estimate
  registration.pose.position.x = filter.state.get_field<Position>()[0];
  registration.pose.position.y = filter.state.get_field<Position>()[1];
  registration.pose.position.z = filter.state.get_field<Position>()[2];
  registration.pose.orientation.w = filter.state.get_field<Attitude>().w();
  registration.pose.orientation.x = filter.state.get_field<Attitude>().x();
  registration.pose.orientation.y = filter.state.get_field<Attitude>().y();
  registration.pose.orientation.z = filter.state.get_field<Attitude>().z();
  // Get the covariance in the state estimate
  for (size_t i = 0; i < 36; i++)
    registration.covariance[i] = filter.covariance(i / 6, i % 6);
}

// When an IMU measurement occurs, assume
void ImuCallback(sensor_msgs::Imu::ConstPtr const& msg) {
  measurement_.set_field<Accelerometer>(UKF::Vector<3>(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z));
  measurement_.set_field<Gyroscope>(UKF::Vector<3>(
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z));
}

// When the solver obtains a solution for the latest round of light
// measurements, it sends the solution out for consumption. Critically, both
// the frame_id and the timestamp match the registration pulse.
void PoseStampedCallback(geometry_msgs::PoseStamped::ConstPtr const& msg) {

}

// Called to move the filters forward
void TimerCallback(ros::TimerEvent const& info) {
  static std::map<std::string, FilterInstance>::iterator it;
  double dt = (info.current_real - info.last_real).toSec();
  for (it = filters_.begin(); it != filters_.end(); it++) {
    // Step the filter forward
    it->second.ukf.step(dt, it->second.measurement);
    // 
    // Broadcast the tracker pose
    static tf2_ros::TransformBroadcaster br;  
    static geometry_msgs::TransformStamped tf;
    tf.header.stamp = info.current_real;
    tf.header.frame_id = "world";
    tf.child_frame_id = it->first;
    tf.transform.translation.x = it->second.ukf.state.get_field<Position>()[0];
    tf.transform.translation.y = it->second.ukf.state.get_field<Position>()[1];
    tf.transform.translation.z = it->second.ukf.state.get_field<Position>()[2];
    tf.transform.rotation.w = it->second.ukf.state.get_field<Attitude>().w();
    tf.transform.rotation.x = it->second.ukf.state.get_field<Attitude>().x();
    tf.transform.rotation.y = it->second.ukf.state.get_field<Attitude>().y();
    tf.transform.rotation.z = it->second.ukf.state.get_field<Attitude>().z();
    br.sendTransform(tf);
  }
}

// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_filter");
  ros::NodeHandle nh;

  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun deepdive_ros deepdive_filter <opts>");
  google::SetVersionString("0.0.1");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Subscribe to the motion and light callbacks
  ros::Subscriber sub_imu
    = nh.subscribe("imu", 10, InertialCallback);
  ros::Subscriber sub_light
    = nh.subscribe("light", 10, ImuCallback);

  // Start a thread to listen to vive
  ros::Timer timer = nh.createTimer(ros::Rate(100.0), TimerCallback);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
