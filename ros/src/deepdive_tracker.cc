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

// PASSED INTO TRACKING FILTER

struct Context {
  Eigen::Affine3d lTw;          // Lighthouse to world frame
  Eigen::Affine3d bTi;          // IMU to body frame
  Eigen::Affine3d bTt;          // Tracking to body transform
  Eigen::Vector3d sensor;       // Sensor position in the tracking frame
  Eigen::Vector3d gravity;      // Gravity in the world frame
  deepdive_ros::Motor params;   // Lighthouse parameters
  uint8_t axis;                 // Tracking axis - horizontal (0), vertical (1)
  bool correct;                 // SHould we correct angles using parameters
};

// TRACKING FILTER

// State indexes
enum Keys {
  Position,               // Position (world frame, m)
  Attitude,               // Attitude as a quaternion (world to body frame)
  Velocity,               // Velocity (world frame, m/s)
  AngularVelocity,        // Angular velocity (body frame, rads/s)
  Acceleration,           // Acceleration (body frame, m/s^2)
  AngularAcceleration,    // Angular acceleration (body frame, rads/s^2)
  GyroscopeBias,          // Gyroscope bias offset (body frame, rad/s)
  GyroscopeScale,         // Gyroscope scale factor (body frame, multiplier)
  AccelerometerBias,      // Accelerometer bias offset (body frame, m/s^2)
  AccelerometerScale,     // Accelerometer scale factor (body frame, mutliplier)
  Accelerometer,          // Acceleration (body frame, m/s^2)
  Gyroscope,              // Gyroscope (body frame, rads/s)
  Angle                   // Angle between lighthouse and sensor (radians)
};

// State vector
using State = UKF::StateVector<
  UKF::Field<Position, UKF::Vector<3>>,
  UKF::Field<Attitude, UKF::Quaternion>,
  UKF::Field<Velocity, UKF::Vector<3>>,
  UKF::Field<AngularVelocity, UKF::Vector<3>>,
  UKF::Field<Acceleration, UKF::Vector<3>>,
  UKF::Field<AngularAcceleration, UKF::Vector<3>>
>;

// Measurement vector
using Measurement = UKF::DynamicMeasurementVector<
  UKF::Field<Accelerometer, UKF::Vector<3>>,
  UKF::Field<Gyroscope, UKF::Vector<3>>,
  UKF::Field<Angle, real_t>
>;

// Parameters
using Error = UKF::StateVector<
  UKF::Field<AccelerometerBias, UKF::Vector<3>>,
  UKF::Field<AccelerometerScale, UKF::Vector<3>>,
  UKF::Field<GyroscopeBias, UKF::Vector<3>>,
  UKF::Field<GyroscopeScale, UKF::Vector<3>>
>;

// For tracking
using TrackingFilter =
  UKF::SquareRootCore<State, Measurement, UKF::IntegratorRK4>;

// For parameter estimation
using ParameterFilter =
  UKF::SquareRootParameterEstimationCore<Error, Measurement>;

namespace UKF {
  namespace Parameters {
    template <> constexpr real_t AlphaSquared<State> = 1e-2;
    template <> constexpr real_t Kappa<State> = 3.0;
    template <> constexpr real_t AlphaSquared<Error> = 1.0;
    template <> constexpr real_t Kappa<Error> = 3.0;
  }

  template <>
  Measurement::CovarianceVector Measurement::measurement_covariance(
    (Measurement::CovarianceVector() << 
      1.0e-4, 1.0e-4, 1.0e-4,   // (1e-02)^2 meters/sec^2
      1.0e-6, 1.0e-6, 1.0e-6,   // (1e-03)^2 rads/sec
      1.0e-8).finished());      // atan(1/10000)^2 = 1e-08

  template <> template <> Error
  Error::derivative<>() const {
    return Error::Zero();
  }

  template <> template <> State
  State::derivative<>() const {
    State output;
    output.set_field<Position>(get_field<Velocity>());
    output.set_field<Velocity>(
      get_field<Attitude>().conjugate() * get_field<Acceleration>());
    UKF::Quaternion omega_q;
    omega_q.vec() = get_field<AngularVelocity>() * 0.5;
    omega_q.w() = 0;
    output.set_field<Attitude>(omega_q.conjugate() * get_field<Attitude>());
    output.set_field<AngularVelocity>(get_field<AngularAcceleration>());
    output.set_field<Acceleration>(UKF::Vector<3>(0, 0, 0));
    output.set_field<AngularAcceleration>(UKF::Vector<3>(0, 0, 0));
    return output;
  }

  template <> template <> UKF::Vector<3>
  Measurement::expected_measurement<State, Accelerometer, Error>(
    State const& state, Error const& input, Context const& context) {
    UKF::Vector<3> const& w = state.get_field<AngularVelocity>();
    UKF::Vector<3> const& p = context.bTi.translation();
    return input.get_field<AccelerometerScale>().cwiseProduct(
      context.bTi.linear().inverse() * w.cross(w.cross(p)
        + state.get_field<Acceleration>()
        + state.get_field<AccelerometerBias>()
        + state.get_field<Attitude>() * context.gravity));
  }

  template <> template <> UKF::Vector<3>
  Measurement::expected_measurement<State, Gyroscope, Error>(
    State const& state, Error const& input, Context const& context) {
    return input.get_field<GyroscopeScale>().cwiseProduct(
      context.bTi.linear().inverse() * state.get_field<AngularVelocity>()
        + input.get_field<GyroscopeBias>());
  }

  template <> template <> real_t
  Measurement::expected_measurement<State, Angle, Error>(
    State const& state, Error const& input, Context const& context) {
      // Convert the current state estimate to a transform
      Eigen::Affine3d wTb;
      wTb.translation() = state.get_field<Position>();
      wTb.linear() = state.get_field<Attitude>().toRotationMatrix();
      // Get the position of the sensor in the lighthouse frame
      UKF::Vector<3> x = context.lTw * wTb * context.bTt * context.sensor;
      // Get the angles in X and Y
      UKF::Vector<2> angles;
      angles[0] = std::atan2(x[0], x[2]);
      angles[1] = std::atan2(x[1], x[2]);
      // Apply the corrections
      uint8_t const& a = context.axis;
      if (context.correct) {
        angles[a] -= context.params.phase;
        angles[a] -= context.params.tilt * angles[1-a];
        angles[a] -= context.params.curve * angles[1-a] * angles[1-a];
        angles[a] -= context.params.gibmag *
          std::cos(angles[1-a] + context.params.gibphase);
      }
      // Vertical or horizontal angle
      return angles[a];
  }
}

// GLOBAL DATA STRUCTURES

// Lighthouse data structure
struct Lighthouse {
  deepdive_ros::Motor params[NUM_MOTORS];   // Parameters
  bool ready;                               // Data received
};
typedef std::map<std::string, Lighthouse> LighthouseMap;

// Tracker data structure
struct Tracker {
  ParameterFilter error;                    // IMU error filter
  Eigen::Vector3d sensors[NUM_SENSORS];     // Sensor position frame
  bool ready;                               // Data received
};
typedef std::map<std::string, Tracker> TrackerMap;

static tf2_ros::Buffer buffer_;             // Transform buffer
static LighthouseMap lighthouses_;          // List of lighthouses
static TrackerMap trackers_;                // List of trackers
static Context context_;                    // Measurement context
static TrackingFilter filter_;              // Tracking filter

// UTILITY FUNCTIONS

double Delta(ros::Time const& now) {
  static ros::Time last = ros::Time::now();
  double dt = (now - last).toSec();
  last = now;
  if (dt > rate_ * 2)
    ROS_WARN_STREAM_THROTTLE(1, "Delta call greater than 2x target period");
  return (dt > 0 ? dt : rate_);
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
  return
}

// CALLBACKS

// This will be called at approximately 120Hz
// - Single lighthouse in 'A' mode : 120Hz (60Hz per axis)
// - Dual lighthouses in b/A or b/c modes : 120Hz (30Hz per axis)
void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  // Get a delta t for this measurement
  double dt = Delta(ros::Time::now());
  // Check that we are recording and that the tracker/lighthouse is ready
  if (trackers_.find(msg->header.frame_id) == trackers_.end() ||
    lighthouses_.find(msg->lighthouse) == lighthouses_.end() ||
    !trackers_[msg->header.frame_id].ready ||
    !lighthouses_[msg->lighthouse].ready) return;
  // Get references to the
  Tracker const& tracker = trackers_[msg->header.frame_id];
  Lighthouse const& lighthouse = lighthouses_[msg->lighthouse];
  // Do a simple clean of the incoming data, looking to see if the pulse angle
  // and duration exceed some threshold values, and that the sensor id is valid
  std::vector<deepdive_ros::Pulse> data;
  for (size_t i = 0; i < msg->pulses.size(); i++) {
    // Basic sanity checks on the data
    if (fabs(msg->pulses[i].angle) > thresh_angle_ * M_PI / 180.0) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on angle");
      return;
    }
    if (fabs(msg->pulses[i].duration) < thresh_duration_ * 1e-6) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on duration");
      return;
    }
    if (msg->pulses[i].sensor >= NUM_SENSORS) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on sensor id");
      return;
    }
    data.push_back(msg->pulses[i]);
  }
  if (data.empty())
    ROS_INFO_STREAM_THROTTLE(1, "No data, so skipping correction.");
  // Check that calibration has produced a lighthouse transform, or else we
  // don't have a wortld reference against which to measure the angle.
  context_.axis = it->axis;
  context_.params = lighthouse.params;
  try {
    static geometry_msgs::TransformStamped tf;
    // Get a world <-> lighthouse transform
    tf = buffer_.lookupTransform(
      frame_parent_, msg->lighthouse, ros::Time(0));
    context_.lTw.translation() = Eigen::Vector3d(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);
    context_.lTw.linear() = Eigen::Quaterniond(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z).toRotationMatrix();
    // Get a body <-> tracking transform
    tf = buffer_.lookupTransform(
      frame_child_, msg->header.frame_id + "/light", ros::Time(0));
    context.bTt.translation() = Eigen::Vector3d(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);
    context.bTt.linear() = Eigen::Quaterniond(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z).toRotationMatrix();
  } catch (tf2::TransformException &ex) {
    ROS_INFO_STREAM_THROTTLE(1, "Rejected based on transform unavailability");
    return;
  }
  // Process update
  filter_.a_priori_step(dt);
  for (it = msg->pulses.begin(); it != msg->pulses.end(); i++) {
    context_.sensor = tracker.sensors[it->sensor];
    Measurement measurement;
    measurement.set_field<Angle>(it->angle);
    filter_.innovation_step(measurement, tracker.error, context_);
  }
  filter_.a_posteriori_step();
}

// This will be called at approximately 250Hz
void ImuCallback(sensor_msgs::Imu::ConstPtr const& msg) {
  // Get the delta-t
  double dt= Delta(ros::Time::now());
  // Check that we are recording and that the tracker/lighthouse is ready
  TrackerMap::iterator tracker = trackers_.find(msg->header.frame_id)
  if (tracker == trackers_.end()) {
    ROS_INFO_STREAM_THROTTLE(1, "Could not find tracker");
    return;
  }
  // Get the measurements
  static Eigen::Vector3d acc;
  acc(0) = msg->linear_acceleration.x;
  acc(1) = msg->linear_acceleration.y;
  acc(2) = msg->linear_acceleration.z;
  static Eigen::Vector3d gyr;
  gyr(0) = msg->angular_velocity.x;
  gyr(1) = msg->angular_velocity.y;
  gyr(2) = msg->angular_velocity.z;
  // Update the context with a body -> imu transform for the current tracker.
  // This should return immediately, as the transform is captured by a buffer
  // running in the background.
  try {
    static geometry_msgs::TransformStamped tf;
    tf = buffer_.lookupTransform(
      frame_child_, msg->header.frame_id + "/imu", ros::Time(0));
    context.bTi.translation() = Eigen::Vector3d(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);
    context.bTi.linear() = Eigen::Quaterniond(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z).toRotationMatrix();
  } catch (tf2::TransformException &ex) {
    ROS_INFO_STREAM_THROTTLE(1, "Rejected based on transform unavailability");
    return;
  }
  // Create a measurement
  static Measurement measurement;
  measurement.set_field<Accelerometer>(acc);
  measurement.set_field<Gyroscope>(gyr);
  // Propagate the IMU error filter using the tracking filter state
  tracker->error.a_priori_step();
  tracker->error.innovation_step(measurement, filter_.state, context_);
  tracker->error.a_posteriori_step(); 
  // Propagate the filter
  filter_.a_priori_step(dt);
  filter_.innovation_step(measurement, tracker->error.state, context_);
  filter_.a_posteriori_step();
}

// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_tracker");
  ros::NodeHandle nh("~");

  // Start listening for transforms now that we hav initialized ROS
  tf2_ros::TransformListener listener(buffer_);

  // Get the parent information
  std::vector<std::string> trackers;
  if (!nh.getParam("trackers", trackers))
    ROS_FATAL("Failed to get the tracker list.");
  std::vector<std::string>::iterator jt;
  for (jt = trackers.begin(); jt != trackers.end(); jt++) {
    std::string serial;
    if (!nh.getParam(*jt + "/serial", serial))
      ROS_FATAL("Failed to get the tracker serial.");
    trackers_[serial].ready = false;
  }

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
    cov_attitude[0], cov_attitude[1], cov_attitude[2],
    cov_velocity[0], cov_velocity[1], cov_velocity[2],
    cov_angvel[0], cov_angvel[1], cov_angvel[2],
    cov_accel[0], cov_accel[1], cov_accel[2],
    cov_gyro_bias[0], cov_gyro_bias[1], cov_gyro_bias[2];
  filter_.process_noise_covariance = State::CovarianceMatrix::Zero();
  filter_.process_noise_covariance.diagonal() <<
    noise_position[0], noise_position[1], noise_position[2],
    noise_attitude[0], noise_attitude[1], noise_attitude[2],
    noise_velocity[0], noise_velocity[1], noise_velocity[2],
    noise_angvel[0], noise_angvel[1], noise_angvel[2],
    noise_accel[0], noise_accel[1], noise_accel[2],
    noise_gyro_bias[0], noise_gyro_bias[1], noise_gyro_bias[2];

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
