/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// Libdeepdive interface
extern "C" {
  #include <deepdive/deepdive.h>
}

// ROS includes
#include <ros/ros.h>

// Standard messages
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

// Non-standard messages
#include <deepdive_ros/Button.h>
#include <deepdive_ros/Light.h>
#include <deepdive_ros/Pulse.h>
#include <deepdive_ros/Motor.h>
#include <deepdive_ros/Sensor.h>
#include <deepdive_ros/Lighthouses.h>
#include <deepdive_ros/Trackers.h>

// C++ includes
#include <cstdint>
#include <cmath>
#include <map>
#include <string>
#include <limits>

// Various constants used by the Vive system
static constexpr double GRAVITY         = 9.80665;
static constexpr double GYRO_SCALE      = 32.768;
static constexpr double ACC_SCALE       = 4096.0;
static constexpr double SWEEP_DURATION  = 400000.0;
static constexpr double SWEEP_CENTER    = 200000.0;
static constexpr double TICKS_PER_SEC   = 48e6;

// DATA STRUCTURES

// Data structures for storing lighthouses and trackers
static std::map<std::string, deepdive_ros::Lighthouse> lighthouses_;
static std::map<std::string, deepdive_ros::Tracker> trackers_;

// Create data publishers
static ros::Publisher pub_lighthouses_;
static ros::Publisher pub_trackers_;
static ros::Publisher pub_button_;
static ros::Publisher pub_light_;
static ros::Publisher pub_imu_;

// Quaternion :: ROS <-> DOUBLE

template <typename T> inline
void Convert(geometry_msgs::Quaternion const& from, T to[4]) {
  to[0] = from.w;
  to[1] = from.x;
  to[2] = from.y;
  to[3] = from.z;
}

template <typename T> inline
void Convert(const T from[4], geometry_msgs::Quaternion & to) {
  to.w = from[0];
  to.x = from[1];
  to.y = from[2];
  to.z = from[3];
}

// Vector :: ROS <-> DOUBLE

template <typename T> inline
void Convert(geometry_msgs::Vector3 const& from, T to[3]) {
  to[0] = from.x;
  to[1] = from.y;
  to[2] = from.z;
}

template <typename T> inline
void Convert(const T from[3], geometry_msgs::Vector3 & to) {
  to.x = from[0];
  to.y = from[1];
  to.z = from[2];
}

// Point :: ROS <-> DOUBLE

template <typename T> inline
void Convert(geometry_msgs::Point const& from, T to[3]) {
  to[0] = from.x;
  to[1] = from.y;
  to[2] = from.z;
}

template <typename T> inline
void Convert(const T from[3], geometry_msgs::Point & to) {
  to.x = from[0];
  to.y = from[1];
  to.z = from[2];
}

// CALLBACKS

// Callback to display light info
void LightCallback(struct Tracker * tracker,
  struct Lighthouse * lighthouse, uint8_t axis, uint32_t synctime,
  uint16_t num_sensors, uint16_t *sensors, uint32_t *sweeptimes,
  uint32_t *angles, uint16_t *lengths) {
  deepdive_ros::Light msg;
  msg.header.frame_id = tracker->serial;
  msg.header.stamp = ros::Time::now();
  msg.lighthouse = lighthouse->serial;
  // Make sure we convert to RHS
  switch (axis) {
  case MOTOR_AXIS0:
    msg.axis = deepdive_ros::Motor::AXIS_0;
    break;
  case MOTOR_AXIS1:
    msg.axis = deepdive_ros::Motor::AXIS_1;
    break;
  default:
    ROS_WARN("Received light with invalid axis");
    return;
  }
  // Add the pulses
  msg.pulses.resize(num_sensors);
  for (uint16_t i = 0; i < num_sensors; i++) {
    msg.pulses[i].sensor = sensors[i];
    msg.pulses[i].angle = (M_PI / SWEEP_DURATION)
      * (static_cast<double>(angles[i]) - SWEEP_CENTER);
    msg.pulses[i].duration =
      static_cast<double>(lengths[i]) / TICKS_PER_SEC;
  }
  // Publish the data
  pub_light_.publish(msg);
}

// Called back when new IMU data is available
void ImuCallback(struct Tracker * tracker, uint32_t timecode,
  int16_t acc[3], int16_t gyr[3], int16_t mag[3]) {
  // Package up the IMU data
  sensor_msgs::Imu msg;
  msg.header.frame_id = tracker->serial;
  msg.header.stamp = ros::Time::now();
  msg.linear_acceleration.x =
    static_cast<double>(acc[0]) * GRAVITY / ACC_SCALE;
  msg.linear_acceleration.y =
    static_cast<double>(acc[1]) * GRAVITY / ACC_SCALE;
  msg.linear_acceleration.z =
    static_cast<double>(acc[2]) * GRAVITY / ACC_SCALE;
  msg.angular_velocity.x =
    static_cast<double>(gyr[0]) * (1./GYRO_SCALE) * (M_PI/180.);
  msg.angular_velocity.y =
    static_cast<double>(gyr[1]) * (1./GYRO_SCALE) * (M_PI/180.);
  msg.angular_velocity.z =
    static_cast<double>(gyr[2]) * (1./GYRO_SCALE) * (M_PI/180.);
  // Publish the data
  pub_imu_.publish(msg);
}

// Called when a button is pressed
void ButtonCallback(struct Tracker * tracker,
  uint32_t mask, uint16_t trigger, int16_t horizontal, int16_t vertical) {
  // Package up the button data
  deepdive_ros::Button msg;
  msg.tracker = tracker->serial;
  msg.mask = mask;
  msg.trigger_val = trigger;
  msg.pad_x = horizontal;
  msg.pad_y = vertical;
  // Publish the data
  pub_button_.publish(msg);
}

// Configuration call from the vive_tool
void TrackerCallback(struct Tracker * t) {
  if (!t) return;
  deepdive_ros::Tracker & tracker = trackers_[t->serial];
  tracker.serial = t->serial;
  tracker.sensors.resize(t->cal.num_channels);
  for (size_t i = 0; i < t->cal.num_channels; i++) {
    Convert(t->cal.positions[i], tracker.sensors[i].position);
    Convert(t->cal.normals[i], tracker.sensors[i].normal);
  }
  // Set the IMU sensor calibration data
  Convert(t->cal.acc_bias, tracker.acc_bias);
  Convert(t->cal.acc_scale, tracker.acc_scale);
  Convert(t->cal.gyr_bias, tracker.gyr_bias);
  Convert(t->cal.gyr_scale, tracker.gyr_scale);
  // Set the default IMU transform
  Convert(&t->cal.imu_transform[0], tracker.imu_transform.rotation);
  Convert(&t->cal.imu_transform[4], tracker.imu_transform.translation);
  // Set the default IMU transform
  Convert(&t->cal.head_transform[0], tracker.head_transform.rotation);
  Convert(&t->cal.head_transform[4], tracker.head_transform.translation);
  // Send all trackers at once
  deepdive_ros::Trackers msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  std::map<std::string, deepdive_ros::Tracker>::iterator it;
  for (it = trackers_.begin(); it != trackers_.end(); it++)
    msg.trackers.push_back(it->second);
  pub_trackers_.publish(msg);
}

// Configuration call from the vive_tool
void LighthouseCallback(struct Lighthouse *l) {
  if (!l) return;
  deepdive_ros::Lighthouse & lighthouse = lighthouses_[l->serial];
  lighthouse.serial = l->serial;
  lighthouse.motors.resize(2);
  for (size_t i = 0; i < MAX_NUM_MOTORS; i++) {
    lighthouse.motors[i].axis = i;
    lighthouse.motors[i].phase = l->motors[i].phase;
    lighthouse.motors[i].tilt = l->motors[i].tilt;
    lighthouse.motors[i].gibphase = l->motors[i].gibphase;
    lighthouse.motors[i].gibmag = l->motors[i].gibmag;
    lighthouse.motors[i].curve = l->motors[i].curve;
  }
  // Get the lighthouse orientation
  Convert(l->accel, lighthouse.acceleration);
  // Send all lighthouses at once
  deepdive_ros::Lighthouses msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  std::map<std::string, deepdive_ros::Lighthouse>::iterator it;
  for (it = lighthouses_.begin(); it != lighthouses_.end(); it++)
    msg.lighthouses.push_back(it->second);
  pub_lighthouses_.publish(msg);
}

// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "deepdive_bridge");
  ros::NodeHandle nh;

  // Latched publishers
  pub_lighthouses_ =
    nh.advertise<deepdive_ros::Lighthouses>("lighthouses", 10, true);
  pub_trackers_ =
    nh.advertise<deepdive_ros::Trackers>("trackers", 10, true);

  // Non-latched publishers
  pub_light_ = nh.advertise<deepdive_ros::Light>("light", 10);
  pub_button_ = nh.advertise<deepdive_ros::Button>("button", 10);
  pub_imu_ = nh.advertise<sensor_msgs::Imu>("imu", 10);

  // Try to initialize vive
  struct Driver *driver = deepdive_init();
  if (!driver) {
    ROS_ERROR("Deepdive initialization failed");
    return 1;
  }

  // Install the callbacks
  deepdive_install_light_fn(driver, LightCallback);
  deepdive_install_imu_fn(driver, ImuCallback);
  deepdive_install_button_fn(driver, ButtonCallback);
  deepdive_install_lighthouse_fn(driver, LighthouseCallback);
  deepdive_install_tracker_fn(driver, TrackerCallback);

  // Set active to true on initialization
  while (ros::ok()) {
    // Poll the ros driver for activity
    deepdive_poll(driver);
    // Flush the ROS messaging queue
    ros::spinOnce();
  }

  // Close the vive context
  deepdive_close(driver);

  // Success!
  return 0;
}