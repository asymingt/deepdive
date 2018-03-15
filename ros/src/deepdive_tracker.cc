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
  UKF::Vector<3> gravity;       // Gravity in the world frame
  deepdive_ros::Motor params;   // Lighthouse parameters
  uint8_t axis;                 // Tracking axis - horizontal (0), vertical (1)
  bool correct;                 // Should we correct angles using parameters
  double thresh_angle;          // Threshold on angle acceptance (in degrees)
  double thresh_duration;       // Threshold on duration acceptance (in us)
  double thresh_count;          // Threshold on measurement count
  std::string frame_parent;     // Parent frame "world"
  std::string frame_child;      // Child frame "body"
  double rate;                  // Targe tracking rate
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

// Observation vector
using Observation = UKF::DynamicMeasurementVector<
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
  UKF::SquareRootCore<State, Observation, UKF::IntegratorRK4>;

// For parameter estimation
using ParameterFilter =
  UKF::SquareRootParameterEstimationCore<Error, Observation>;

namespace UKF {
  namespace Parameters {
    template <> constexpr real_t AlphaSquared<State> = 1e-2;
    template <> constexpr real_t Kappa<State> = 3.0;
    template <> constexpr real_t AlphaSquared<Error> = 1.0;
    template <> constexpr real_t Kappa<Error> = 3.0;
  }

  template <>
  Observation::CovarianceVector Observation::measurement_covariance(
    (Observation::CovarianceVector() << 
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
  Observation::expected_measurement<State, Accelerometer, Error>(
    State const& state, Error const& input, Context const& context) {
    UKF::Vector<3> const& w = state.get_field<AngularVelocity>();
    UKF::Vector<3> const& p = context.bTi.translation();
    return input.get_field<AccelerometerScale>().cwiseProduct(
      context.bTi.linear().inverse() * w.cross(w.cross(p))
        + state.get_field<Acceleration>()
        + input.get_field<AccelerometerBias>()
        + state.get_field<Attitude>() * context.gravity);
  }

  template <> template <> UKF::Vector<3>
  Observation::expected_measurement<State, Gyroscope, Error>(
    State const& state, Error const& input, Context const& context) {
    return input.get_field<GyroscopeScale>().cwiseProduct(
      context.bTi.linear().inverse() * state.get_field<AngularVelocity>()
        + input.get_field<GyroscopeBias>());
  }

  template <> template <> real_t
  Observation::expected_measurement<State, Angle, Error>(
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

static LighthouseMap lighthouses_;          // List of lighthouses
static TrackerMap trackers_;                // List of trackers
static Context context_;                    // Observation context
static TrackingFilter filter_;              // Tracking filter

// UTILITY FUNCTIONS

double Delta(ros::Time const& now) {
  static ros::Time last = ros::Time::now();
  double dt = (now - last).toSec();
  last = now;
  if (dt > context_.rate * 2)
    ROS_WARN_STREAM_THROTTLE(1, "Made dt call at > 2x target period");
  return (dt > 0 ? dt : context_.rate);
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

// CALLBACKS

// This will be called at approximately 120Hz
// - Single lighthouse in 'A' mode : 120Hz (60Hz per axis)
// - Dual lighthouses in b/A or b/c modes : 120Hz (30Hz per axis)
void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  double dt = Delta(ros::Time::now());
  // Check that we are recording and that the tracker/lighthouse is ready
  context_.tracker = trackers_.find(msg->header.frame_id)
  if (context_.tracker == trackers_.end() || !context_.tracker->ready) {
    ROS_INFO_STREAM_THROTTLE(1, "Tracker not found or ready");
    return;
  }
  // Check that we are recording and that the tracker/lighthouse is ready
  context_.lighthouse = lighthouses_.find(msg->lighthouse)
  if (context_.lighthouse == lighthouse.end() || !context_.lighthouse->ready) {
    ROS_INFO_STREAM_THROTTLE(1, "Lighthouse not found or ready");
    return;
  }
  // Set the measurement axis
  context_.axis = msg->axis;
  // Do a simple clean of the incoming data, looking to see if the pulse angle
  // and duration exceed some threshold values, and that the sensor id is valid
  std::vector<deepdive_ros::Pulse> data;
  for (size_t i = 0; i < msg->pulses.size(); i++) {
    // Basic sanity checks on the data
    if (fabs(msg->pulses[i].angle) > context_.thresh_angle * M_PI / 180.0) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on angle");
      return;
    }
    if (fabs(msg->pulses[i].duration) < context_.thresh_duration * 1e-6) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on duration");
      return;
    }
    if (msg->pulses[i].sensor >= NUM_SENSORS) {
      ROS_INFO_STREAM_THROTTLE(1, "Rejected based on invalid sensor id");
      return;
    }
    data.push_back(msg->pulses[i]);
  }
  if (data.size() < context_.thresh_count)
    ROS_INFO_STREAM_THROTTLE(1, "Not enough data so skipping bundle.");
  // Propagate the state forward
  filter_.a_priori_step(dt);
  std::vector<deepdive_ros::Pulse>::iterator it;
  for (it = msg->pulses.begin(); it != msg->pulses.end(); it++) {
    context_.sensor = it->sensor;
    Observation obs;
    obs.set_field<Angle>(it->angle);
    filter_.innovation_step(obs, tracker.errors, context_);
  }
  filter_.a_posteriori_step();
}

// This will be called at approximately 250Hz
void ImuCallback(sensor_msgs::Imu::ConstPtr const& msg) {
  double dt = Delta(ros::Time::now());
  // Check that we are recording and that the tracker/lighthouse is ready
  TrackerMap::iterator tracker = trackers_.find(msg->header.frame_id)
  if (tracker == trackers_.end() || !tracker->ready) {
    ROS_INFO_STREAM_THROTTLE(1, "Tracker not found or ready");
    return;
  }
  // Make sure we have a filter setup for this
  ErrorMap::iterator error = errors.find(msg->header.frame_id);
  if (error == error.end()) {
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
  // TODO(Andrew) inject the transforms
  // Create a measurement
  Observation obs;
  obs.set_field<Accelerometer>(acc);
  obs.set_field<Gyroscope>(gyr);
  // Propagate the IMU error filter using the tracking filter state
  tracker->error.a_priori_step();
  tracker->error.innovation_step(obs, filter_.state, context_);
  tracker->error.a_posteriori_step(); 
  // Propagate the filter
  filter_.a_priori_step(dt);
  filter_.innovation_step(obs, tracker->error.state, context_);
  filter_.a_posteriori_step();
}

void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg) {
  std::vector<deepdive_ros::Lighthouse>::const_iterator it;
  for (it = msg->lighthouses.begin(); it != msg->lighthouses.end(); it++) {
    if (lighthouses_.find(it->serial) == lighthouses_.end())
      return;
    Lighthouse & lighthouse = lighthouses_[it->serial];
    for (size_t i = 0; i < it->motors.size() && i < NUM_MOTORS; i++) {
      lighthouse.params[i][CAL_PHASE] = it->motors[i].phase;
      lighthouse.params[i][CAL_TILT] = it->motors[i].tilt;
      lighthouse.params[i][CAL_GIB_PHASE] = it->motors[i].gibphase;
      lighthouse.params[i][CAL_GIB_MAG] = it->motors[i].gibmag;
      lighthouse.params[i][CAL_CURVE] = it->motors[i].curve;
    }
    if (!lighthouse.ready)
      ROS_INFO_STREAM("Received data from lighthouse " << it->serial);
    lighthouse.ready = true;
  }
}

void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg) {
  // Iterate over the trackers in this message
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    if (trackers_.find(it->serial) == trackers_.end())
      return;
    // Convert to an angle-axis representation
    Eigen::Quaterniond q(
      it->head_transform.rotation.w,
      it->head_transform.rotation.x,
      it->head_transform.rotation.y,
      it->head_transform.rotation.z);
    Eigen::AngleAxisd aa(q);
    Eigen::Affine3d tTh;
    tTh.linear() = q.toRotationMatrix();
    tTh.translation() = Eigen::Vector3d(
      it->head_transform.translation.x,
      it->head_transform.translation.y,
      it->head_transform.translation.z);
    // Modify the tracker
    Tracker & tracker = trackers_[it->serial];
    tracker.tTh[0] = tTh.translation()[0];
    tracker.tTh[1] = tTh.translation()[1];
    tracker.tTh[2] = tTh.translation()[2];
    tracker.tTh[3] = aa.angle() * aa.axis()[0];
    tracker.tTh[4] = aa.angle() * aa.axis()[1];
    tracker.tTh[5] = aa.angle() * aa.axis()[2];
    for (size_t i = 0; i < it->sensors.size() &&  i < NUM_SENSORS; i++) {
      tracker.sensors[6*i+0] = it->sensors[i].position.x;
      tracker.sensors[6*i+1] = it->sensors[i].position.y;
      tracker.sensors[6*i+2] = it->sensors[i].position.z;
      tracker.sensors[6*i+3] = it->sensors[i].normal.x;
      tracker.sensors[6*i+4] = it->sensors[i].normal.y;
      tracker.sensors[6*i+5] = it->sensors[i].normal.z;
    }
    // Invert the transformation retain frame hierarchy in TF2
    Eigen::Affine3d hTt = tTh.inverse();
    q = Eigen::Quaterniond(hTt.linear());
    // Add the transforms
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = it->serial;
    tfs.child_frame_id = it->serial + "/light";
    tfs.transform.translation.x = hTt.translation()[0];
    tfs.transform.translation.y = hTt.translation()[1];
    tfs.transform.translation.z = hTt.translation()[2];
    tfs.transform.rotation.w = q.w();
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    SendStaticTransform(tfs);
    tfs.header.frame_id = it->serial + "/light";
    tfs.child_frame_id = it->serial + "/imu";
    tfs.transform = it->imu_transform;
    SendStaticTransform(tfs);
    // Print update
    ROS_INFO_STREAM("Received data from tracker " << it->serial);
    tracker.ready = true;
  }
}

// MAIN ENTRY POINT OF APPLICATION

int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_tracker");
  ros::NodeHandle nh("~");

  // Get the lighthouse information
  std::vector<std::string> lighthouses;
  if (!nh.getParam("lighthouses", lighthouses))
    ROS_FATAL("Failed to get the lighthouse list.");
  std::vector<std::string>::iterator it;
  for (it = lighthouses.begin(); it != lighthouses.end(); it++)
    lighthouses_[*it].ready = false;

  // Get the tracker information
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
  if (!nh.getParam("frames/parent", context_.frame_parent))
    ROS_FATAL("Failed to get frames/parent parameter.");
  if (!nh.getParam("frames/child", context_.frame_child))
    ROS_FATAL("Failed to get frames/child parameter.");

  // Get the topics for data topics
  std::string topic_pose, topic_twist;
  if (!nh.getParam("topics/pose", topic_pose))
    ROS_FATAL("Failed to get topics/pose parameter.");
  if (!nh.getParam("topics/twist", topic_twist))
    ROS_FATAL("Failed to get topics/twist parameter.");

  // Get the thresholds
  if (!nh.getParam("thresholds/angle", context_.thresh_angle_))
    ROS_FATAL("Failed to get thresholds/angle parameter.");
  if (!nh.getParam("thresholds/duration", context_.thresh_duration_))
    ROS_FATAL("Failed to get thresholds/duration parameter.");
  if (!nh.getParam("thresholds/count", context_.thresh_count_))
    ROS_FATAL("Failed to get thresholds/count parameter.");

  // Whether to apply light corrections
  if (!nh.getParam("correct", context_.correct))
    ROS_FATAL("Failed to get correct parameter.");

  // Get the tracker update rate.
  double rate = 100;
  if (!nh.getParam("rate", rate))
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
  UKF::Vector<3> est_omega(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/omega", est_omega))
    ROS_FATAL("Failed to get angular_velocity parameter.");
  UKF::Vector<3> est_acceleration(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/acceleration", est_acceleration))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> est_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "initial_estimate/alpha", est_alpha))
    ROS_FATAL("Failed to get alpha parameter.");

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
  UKF::Vector<3> cov_omega(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/omega", cov_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> cov_accel(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/acceleration", cov_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> cov_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "initial_covariance/alpha", cov_alpha))
    ROS_FATAL("Failed to get alpha parameter.");

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
  UKF::Vector<3> noise_omega(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/omega", noise_omega))
    ROS_FATAL("Failed to get omega parameter.");
  UKF::Vector<3> noise_accel(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/acceleration", noise_accel))
    ROS_FATAL("Failed to get acceleration parameter.");
  UKF::Vector<3> noise_alpha(0, 0, 0);
  if (!GetVectorParam(nh, "process_noise/alpha", noise_alpha))
    ROS_FATAL("Failed to get alpha parameter.");

  // Setup the filter
  filter_.state.set_field<Position>(est_position);
  filter_.state.set_field<Attitude>(est_attitude);
  filter_.state.set_field<Velocity>(est_velocity);
  filter_.state.set_field<Omega>(est_omega);
  filter_.state.set_field<Acceleration>(est_acceleration);
  filter_.state.set_field<Alpha>(est_alpha);
  filter_.covariance = State::CovarianceMatrix::Zero();
  filter_.covariance.diagonal() <<
    cov_position[0], cov_position[1], cov_position[2],
    cov_attitude[0], cov_attitude[1], cov_attitude[2],
    cov_velocity[0], cov_velocity[1], cov_velocity[2],
    cov_omega[0], cov_omega[1], cov_omega[2],
    cov_accel[0], cov_accel[1], cov_accel[2],
    cov_alpha[0], cov_alpha[1], cov_alpha[2];
  filter_.process_noise_covariance = State::CovarianceMatrix::Zero();
  filter_.process_noise_covariance.diagonal() <<
    noise_position[0], noise_position[1], noise_position[2],
    noise_attitude[0], noise_attitude[1], noise_attitude[2],
    noise_velocity[0], noise_velocity[1], noise_velocity[2],
    noise_omega[0], noise_omega[1], noise_omega[2],
    noise_accel[0], noise_accel[1], noise_accel[2],
    noise_alpha[0], noise_alpha[1], noise_alpha[2];

  // Subscribe to the motion and light callbacks
  ros::Subscriber sub_lighthouse  =
    nh.subscribe("/lighthouses", 1000, LighthouseCallback);
  ros::Subscriber sub_tracker  =
    nh.subscribe("/trackers", 1000, TrackerCallback);
  ros::Subscriber sub_light =
    nh.subscribe("/light", 1000, LightCallback);
  ros::Subscriber sub_imu =
    nh.subscribe("/imu", 1000, ImuCallback);
  
  // Markers showing sensor positions
  pub_pose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>
    (topic_pose, 0);
  pub_twist_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>
    (topic_twist, 0);

  // If reading the configuration file results in inserting the correct
  // number of static transforms into the problem, then we can publish
  // the solution for use by other entities in the system.
  if (ReadConfig(calfile_, lighthouses_, trackers_)) {
    ROS_INFO("Read transforms from calibration");
  } else {
    ROS_INFO("Could not read calibration file");
  }

  // Start a timer to callback
  ros::Timer timer = nh.createTimer(
    ros::Duration(ros::Rate(rate)), TimerCallback, false, true);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
