/*
  This ROS node creates an instance of the libdeepdive driver, which it uses
  to pull data from all available trackers, as well as lighthouse/tracker info.
*/

// Libsurvive interface
extern "C" {
  #include <deepdive/deepdive.h>
}

// ROS includes
#include <ros/ros.h>

// Standard messages
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

// Non-standard messages
#include <deepdive_ros/Event.h>
#include <deepdive_ros/Light.h>
#include <deepdive_ros/Pulse.h>
#include <deepdive_ros/Motor.h>
#include <deepdive_ros/Sensor.h>
#include <deepdive_ros/Lighthouse.h>
#include <deepdive_ros/Tracker.h>

// Services to get tracker/lighthouse/system config
#include <deepdive_ros/GetTracker.h>
#include <deepdive_ros/GetLighthouse.h>

// C++ includes
#include <cstdint>
#include <cmath>
#include <map>
#include <string>

// Various constants used by the Vive system
static constexpr double GRAVITY         = 9.80665;
static constexpr double GYRO_SCALE      = 32.768;
static constexpr double ACC_SCALE       = 4096.0;
static constexpr double SWEEP_DURATION  = 400000.0;
static constexpr double SWEEP_CENTER    = 200000.0;
static constexpr double TICKS_PER_SEC   = 48000000.0;

// Main entry point to the Vive low-level driver
struct Driver* driver_ = nullptr;

// Data structures for storing lighthouses and trackers
std::map<std::string, deepdive_ros::Lighthouse> lighthouses_;
std::map<std::string, deepdive_ros::Tracker> trackers_;
 
// Create data publishers
ros::Publisher pub_light_;
ros::Publisher pub_event_;
ros::Publisher pub_imu_;
ros::ServiceServer srv_tracker_;
ros::ServiceServer srv_lighthouse_;

// Convert time an return overflow count
ros::Time TimeConvert(std::string const& serial, uint32_t timecode) {
  // If we can't find the tracker don't return a valid time
  if (trackers_.find(serial) == trackers_.end()) {
    ROS_WARN_STREAM("Cannot covert time for tracker " << serial);
    return ros::Time(0);
  }
  // Count the number of overflows
  if (trackers_[serial].lastcount > timecode)
    trackers_[serial].tickstore += static_cast<double>(UINT32_MAX);
  // Backup the timecode for use in next iteration
  trackers_[serial].lastcount = timecode;
  // Add the tick store to the time code and covert to seconds
  return ros::Time((trackers_[serial].tickstore + 
    static_cast<double>(timecode)) / TICKS_PER_SEC);
}

// Callback to display light info
void LightCallback(struct Tracker * tracker,
  struct Lighthouse * lighthouse, uint8_t axis, uint32_t synctime,
  uint16_t num_sensors, uint16_t *sensors, uint32_t *sweeptimes,
  uint32_t *angles, uint16_t *lengths) {
  static deepdive_ros::Light msg;
  msg.header.frame_id = tracker->serial;
  msg.header.stamp = TimeConvert(tracker->serial, synctime);
  msg.lighthouse = lighthouse->serial;
  msg.axis = axis;
  msg.pulses.resize(num_sensors);
  for (uint16_t i = 0; i < num_sensors; i++) {
    msg.pulses[i].sensor = sensors[i];
    msg.pulses[i].angle = (M_PI / SWEEP_DURATION)
      * (static_cast<double>(angles[i]) - SWEEP_CENTER);
    msg.pulses[i].duration =
      static_cast<double>(lengths[i]) / TICKS_PER_SEC * 1000000.0;
  }
  // Publish the data
  pub_light_.publish(msg);
}

// Called back when new IMU data is available
void ImuCallback(struct Tracker * tracker, uint32_t timecode,
  int16_t acc[3], int16_t gyr[3], int16_t mag[3]) {
  // Package up the IMU data
  static sensor_msgs::Imu msg;
  msg.header.frame_id = tracker->serial;
  msg.header.stamp = TimeConvert(tracker->serial, timecode);
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
void ButtonCallback(struct Tracker * tracker, uint32_t timecode,
  uint8_t mask) {
  // Package up the IMU data
  static deepdive_ros::Event msg;
  msg.header.frame_id = tracker->serial;
  msg.header.stamp = TimeConvert(tracker->serial, timecode);
  msg.mask = mask;
  // Publish the data
  pub_event_.publish(msg);
}

// Configuration call from the vive_tool
void TrackerCallback(struct Tracker * t) {
  if (!driver_ || !t) return;
  deepdive_ros::Tracker & msg = trackers_[t->serial];
  // Set the serial number
  msg.serial = t->serial;
  // Set the extrinsics
  msg.sensors.resize(t->cal.num_channels);
  for (size_t i = 0; i < t->cal.num_channels; i++) {
    msg.sensors[i].position.x = t->cal.positions[i][0];
    msg.sensors[i].position.y = t->cal.positions[i][1];
    msg.sensors[i].position.z = t->cal.positions[i][2];
    msg.sensors[i].normal.x = t->cal.normals[i][0];
    msg.sensors[i].normal.y = t->cal.normals[i][1];
    msg.sensors[i].normal.z = t->cal.normals[i][2];
  }
  // Set the IMU sensor calibration data
  msg.acc_bias.x = t->cal.acc_bias[0];
  msg.acc_bias.y = t->cal.acc_bias[1];
  msg.acc_bias.z = t->cal.acc_bias[2];
  msg.acc_scale.x = t->cal.acc_scale[0];
  msg.acc_scale.y = t->cal.acc_scale[1];
  msg.acc_scale.z = t->cal.acc_scale[2];
  msg.gyr_bias.x = t->cal.gyr_bias[0];
  msg.gyr_bias.y = t->cal.gyr_bias[1];
  msg.gyr_bias.z = t->cal.gyr_bias[2];
  msg.gyr_scale.x = t->cal.gyr_scale[0];
  msg.gyr_scale.y = t->cal.gyr_scale[1];
  msg.gyr_scale.z = t->cal.gyr_scale[2];
}

// Configuration call from the vive_tool
void LighthouseCallback(struct Lighthouse *l) {
  if (!driver_ || !l) return;
  deepdive_ros::Lighthouse & msg = lighthouses_[l->serial];
  msg.serial = l->serial;
  msg.motors.resize(2);
  for (size_t i = 0; i < MAX_NUM_MOTORS; i++) {
    msg.motors[i].axis = i;
    msg.motors[i].phase = l->motors[i].phase;
    msg.motors[i].tilt = l->motors[i].tilt;
    msg.motors[i].gibphase = l->motors[i].gibphase;
    msg.motors[i].gibmag = l->motors[i].gibmag;
    msg.motors[i].curve = l->motors[i].curve;
  }
}

// Get a single or list of lighthouses 
bool GetLighthouse(deepdive_ros::GetLighthouse::Request  &req,
  deepdive_ros::GetLighthouse::Response &res) {
  if (req.serial.empty()) {
    res.lighthouses.reserve(lighthouses_.size());
    std::map<std::string, deepdive_ros::Lighthouse>::const_iterator it;
    for (it = lighthouses_.begin(); it != lighthouses_.end(); it++)
      res.lighthouses.push_back(it->second);
  } else if (lighthouses_.find(req.serial) != lighthouses_.end()) {
    res.lighthouses.push_back(lighthouses_[req.serial]);
  }
  return true;
}

// Get a single or list of trackers 
bool GetTracker(deepdive_ros::GetTracker::Request  &req,
  deepdive_ros::GetTracker::Response &res) {
  if (req.serial.empty()) {
    res.trackers.reserve(trackers_.size());
    std::map<std::string, deepdive_ros::Tracker>::const_iterator it;
    for (it = trackers_.begin(); it != trackers_.end(); it++)
      res.trackers.push_back(it->second);
  } else if (trackers_.find(req.serial) != trackers_.end()) {
    res.trackers.push_back(trackers_[req.serial]);
  }
  return true;
}

// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "deepdive_bridge");
  ros::NodeHandle nh;

  // Create data publishers
  pub_light_ = nh.advertise<deepdive_ros::Light>("light", 10);
  pub_event_ = nh.advertise<deepdive_ros::Event>("event", 10);
  pub_imu_ = nh.advertise<sensor_msgs::Imu>("imu", 10);

  // Service calls to request tracker metadata
  srv_tracker_ =
    nh.advertiseService("get_tracker", GetTracker);
  srv_lighthouse_ =
    nh.advertiseService("get_lighthouse", GetLighthouse);

  // Try to initialize vive
  struct Driver *driver = deepdive_init();
  if (!driver_) {
    ROS_FATAL("Deepdive initialization failed");
    return 1;
  }

  // Install the callbacks
  deepdive_install_lig_fn(driver_, LightCallback);
  deepdive_install_imu_fn(driver_, ImuCallback);
  deepdive_install_but_fn(driver_, ButtonCallback);
  deepdive_install_lighthouse_fn(driver_, LighthouseCallback);
  deepdive_install_tracker_fn(driver_, TrackerCallback);

  // Set active to true on initialization
  while (ros::ok()) {
    // Poll the ros driver for activity
    if (deepdive_poll(driver_)) {
      ROS_FATAL("Deepdive low-level driver encountered a problem");
      return 2;
    }
    // Flush the ROS messaging queue
    ros::spinOnce();
  }

  // Close the vive context
  deepdive_close(driver_);

  // Success!
  return 0;
}