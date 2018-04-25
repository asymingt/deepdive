// ROS and TF2
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// STL
#include <fstream>
#include <sstream>

// This include
#include "deepdive.hh"

// TRANSFORM ENGINE

void SendStaticTransform(geometry_msgs::TransformStamped const& tfs) {
  static tf2_ros::StaticTransformBroadcaster bc;
  bc.sendTransform(tfs);
}

void SendDynamicTransform(geometry_msgs::TransformStamped const& tfs) {
  static tf2_ros::TransformBroadcaster bc;
  bc.sendTransform(tfs);
}

void SendTransforms(
  std::string const& frame_world,   // World name
  std::string const& frame_vive,    // Vive frame name
  std::string const& frame_body,    // Body frame name
  double registration[6],
  LighthouseMap const& lighthouses, TrackerMap const& trackers) {
  // Publish world -> vive transform
  {
    Eigen::Vector3d v(registration[3], registration[4], registration[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = frame_world;
    tfs.child_frame_id = frame_vive;
    tfs.transform.translation.x = registration[0];
    tfs.transform.translation.y = registration[1];
    tfs.transform.translation.z = registration[2];
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    SendStaticTransform(tfs);
  }
  // Publish lighthouse positions
  LighthouseMap::const_iterator it;
  for (it = lighthouses.begin(); it != lighthouses.end(); it++)  {
    Eigen::Vector3d v(it->second.vTl[3], it->second.vTl[4], it->second.vTl[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = frame_vive;
    tfs.child_frame_id = it->first;
    tfs.transform.translation.x = it->second.vTl[0];
    tfs.transform.translation.y = it->second.vTl[1];
    tfs.transform.translation.z = it->second.vTl[2];
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    SendStaticTransform(tfs);
  }
  // Publish tracker extrinsics
  TrackerMap::const_iterator jt;
  for (jt = trackers.begin(); jt != trackers.end(); jt++)  {
    Eigen::Vector3d v(jt->second.bTh[3], jt->second.bTh[4], jt->second.bTh[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = frame_body;
    tfs.child_frame_id = jt->first;
    tfs.transform.translation.x = jt->second.bTh[0];
    tfs.transform.translation.y = jt->second.bTh[1];
    tfs.transform.translation.z = jt->second.bTh[2];
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    SendStaticTransform(tfs);
  }
}

// CONFIG MANAGEMENT

// Parse a human-readable configuration
bool ReadConfig(std::string const& calfile,   // Calibration file
  std::string const& frame_world,             // World name
  std::string const& frame_vive,              // Vive frame name
  std::string const& frame_body,              // Body frame name
  double registration[6],
  LighthouseMap lighthouses, TrackerMap trackers) {
  // Entries take the form x y z qx qy qz qw parent child
  std::ifstream infile(calfile);
  if (!infile.is_open()) {
    ROS_WARN("Could not open config file for reading");
    return false;
  }
  std::string line;
  int count = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    double x, y, z, qx, qy, qz, qw;
    std::string p, c;
    if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw >> p >> c)) {
      ROS_ERROR("Badly formatted config file");
      return false;
    }
    Eigen::AngleAxisd aa(Eigen::Quaterniond(qw, qx, qy, qz));
    if (p == frame_world && c == frame_vive) {
      registration[0] = x;
      registration[1] = y;
      registration[2] = z;
      registration[3] = aa.angle() * aa.axis()[0];
      registration[4] = aa.angle() * aa.axis()[1];
      registration[5] = aa.angle() * aa.axis()[2];
      count++;
      continue;
    }
    if (p == frame_vive && lighthouses.find(c) != lighthouses.end()) {
      lighthouses[c].vTl[0] = x;
      lighthouses[c].vTl[1] = y;
      lighthouses[c].vTl[2] = z;
      lighthouses[c].vTl[3] = aa.angle() * aa.axis()[0];
      lighthouses[c].vTl[4] = aa.angle() * aa.axis()[1];
      lighthouses[c].vTl[5] = aa.angle() * aa.axis()[2];
      count++;
      continue;
    }
    if (p == frame_body && trackers.find(c) != trackers.end()) {
      trackers[c].bTh[0] = x;
      trackers[c].bTh[1] = y;
      trackers[c].bTh[2] = z;
      trackers[c].bTh[3] = aa.angle() * aa.axis()[0];
      trackers[c].bTh[4] = aa.angle() * aa.axis()[1];
      trackers[c].bTh[5] = aa.angle() * aa.axis()[2];
      count++;
      continue;
    }
    ROS_WARN_STREAM("Transform " << p << " -> " << c << " invalid");
  }
  return (count == 1 + lighthouses.size() + trackers.size());
}

// Write a human-readable configuration
bool WriteConfig(std::string const& calfile,    // Calibration file
  std::string const& frame_world,               // World name
  std::string const& frame_vive,                // Vive frame name
  std::string const& frame_body,                // Body frame name
  double registration[6],
  LighthouseMap const& lighthouses, TrackerMap const& trackers) {
  std::ofstream outfile(calfile);
  if (!outfile.is_open()) {
    ROS_WARN("Could not open config file for writing");
    return false;
  }
  // World registration
  {
    Eigen::Vector3d v(registration[3], registration[4], registration[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    outfile << registration[0] << " "
            << registration[1] << " "
            << registration[2] << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << frame_world << " " << frame_vive
            << std::endl;
  }
  LighthouseMap::const_iterator it;
  for (it = lighthouses.begin(); it != lighthouses.end(); it++)  {
    Eigen::Vector3d v(it->second.vTl[3], it->second.vTl[4], it->second.vTl[5]);
    Eigen::AngleAxisd aa;
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    outfile << it->second.vTl[0] << " "
            << it->second.vTl[1] << " "
            << it->second.vTl[2] << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << frame_vive << " " << it->first
            << std::endl;
  }
  TrackerMap::const_iterator jt;
  for (jt = trackers.begin(); jt != trackers.end(); jt++)  {
    Eigen::Vector3d v(jt->second.bTh[3], jt->second.bTh[4], jt->second.bTh[5]);
    Eigen::AngleAxisd aa;
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    outfile << jt->second.bTh[0] << " "
            << jt->second.bTh[1] << " "
            << jt->second.bTh[2] << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << frame_body << " " << jt->first
            << std::endl;
  }
  return true;
}

// REUSABLE CALLS

void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg,
  LighthouseMap & lighthouses, std::function<void(LighthouseMap::iterator)> cb) {
  std::vector<deepdive_ros::Lighthouse>::const_iterator it;
  for (it = msg->lighthouses.begin(); it != msg->lighthouses.end(); it++) {
    LighthouseMap::iterator lighthouse = lighthouses.find(it->serial);
    if (lighthouse == lighthouses.end())
      return;
    for (size_t i = 0; i < it->motors.size() && i < NUM_MOTORS; i++) {
      lighthouse->second.params[i][PARAM_PHASE] = it->motors[i].phase;
      lighthouse->second.params[i][PARAM_TILT] = it->motors[i].tilt;
      lighthouse->second.params[i][PARAM_GIB_PHASE] = it->motors[i].gibphase;
      lighthouse->second.params[i][PARAM_GIB_MAG] = it->motors[i].gibmag;
      lighthouse->second.params[i][PARAM_CURVE] = it->motors[i].curve;
    }
    if (!lighthouse->second.ready) {
      lighthouse->second.ready = true;
      cb(lighthouse);
    }
  }
}

void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg,
  TrackerMap & trackers, std::function<void(TrackerMap::iterator)> cb) {
  // Iterate over the trackers in this message
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    TrackerMap::iterator tracker = trackers.find(it->serial);
    if (tracker == trackers.end())
      return;
    // Copy over the initial IMU errors
    tracker->second.errors[ERROR_GYR_BIAS][0] = it->gyr_bias.x;
    tracker->second.errors[ERROR_GYR_BIAS][1] = it->gyr_bias.y;
    tracker->second.errors[ERROR_GYR_BIAS][2] = it->gyr_bias.z;
    tracker->second.errors[ERROR_GYR_SCALE][0] = it->gyr_scale.x;
    tracker->second.errors[ERROR_GYR_SCALE][1] = it->gyr_scale.y;
    tracker->second.errors[ERROR_GYR_SCALE][2] = it->gyr_scale.z;
    tracker->second.errors[ERROR_ACC_BIAS][0] = it->acc_bias.x;
    tracker->second.errors[ERROR_ACC_BIAS][1] = it->acc_bias.y;
    tracker->second.errors[ERROR_ACC_BIAS][2] = it->acc_bias.z;
    tracker->second.errors[ERROR_ACC_SCALE][0] = it->acc_scale.x;
    tracker->second.errors[ERROR_ACC_SCALE][1] = it->acc_scale.y;
    tracker->second.errors[ERROR_ACC_SCALE][2] = it->acc_scale.z;
    // Add the sensor locations and normals
    for (size_t i = 0; i < it->sensors.size() &&  i < NUM_SENSORS; i++) {
      tracker->second.sensors[6*i+0] = it->sensors[i].position.x;
      tracker->second.sensors[6*i+1] = it->sensors[i].position.y;
      tracker->second.sensors[6*i+2] = it->sensors[i].position.z;
      tracker->second.sensors[6*i+3] = it->sensors[i].normal.x;
      tracker->second.sensors[6*i+4] = it->sensors[i].normal.y;
      tracker->second.sensors[6*i+5] = it->sensors[i].normal.z;
    }
    // Add the head -> light transform
    {
      // Write to the global data structure
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
      tracker->second.tTh[0] = tTh.translation()[0];
      tracker->second.tTh[1] = tTh.translation()[1];
      tracker->second.tTh[2] = tTh.translation()[2];
      tracker->second.tTh[3] = aa.angle() * aa.axis()[0];
      tracker->second.tTh[4] = aa.angle() * aa.axis()[1];
      tracker->second.tTh[5] = aa.angle() * aa.axis()[2];
      // Send off the transform
      Eigen::Affine3d hTt = tTh.inverse();
      q = Eigen::Quaterniond(hTt.linear());
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
    }
    // Add the imu -> light transform
    {
      // Write to the global data structure
      Eigen::Quaterniond q(
        it->imu_transform.rotation.w,
        it->imu_transform.rotation.x,
        it->imu_transform.rotation.y,
        it->imu_transform.rotation.z);
      Eigen::AngleAxisd aa(q);
      Eigen::Affine3d tTi;
      tTi.linear() = q.toRotationMatrix();
      tTi.translation() = Eigen::Vector3d(
        it->imu_transform.translation.x,
        it->imu_transform.translation.y,
        it->imu_transform.translation.z);
      tracker->second.tTi[0] = tTi.translation()[0];
      tracker->second.tTi[1] = tTi.translation()[1];
      tracker->second.tTi[2] = tTi.translation()[2];
      tracker->second.tTi[3] = aa.angle() * aa.axis()[0];
      tracker->second.tTi[4] = aa.angle() * aa.axis()[1];
      tracker->second.tTi[5] = aa.angle() * aa.axis()[2];
      // Send off the transform
      geometry_msgs::TransformStamped tfs;  
      tfs.header.frame_id = it->serial + "/light";
      tfs.child_frame_id = it->serial + "/imu";
      tfs.transform = it->imu_transform;
      SendStaticTransform(tfs);
    }
    if (!tracker->second.ready) {
      tracker->second.ready = true;
      cb(tracker);
    }
  }
}
