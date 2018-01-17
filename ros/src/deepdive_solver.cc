/*
  This ROS node listens to data from all trackers in the system and provides
  a global solution to the tracking problem. That is, it solves for the
  relative location of the lighthouses and trackers as a function of time.
*/

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// ROS includes
#include <ros/ros.h>

// For transforms
#include <tf2_ros/transform_broadcaster.h>

// Standard messages
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>

// Non-standard datra messages
#include <deepdive_ros/Light.h>

// Services to get tracker/lighthouse/system config
#include <deepdive_ros/GetTracker.h>
#include <deepdive_ros/GetLighthouse.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// STL includes
#include <thread>
#include <mutex>
#include <string>
#include <map>
#include <queue>

// GFLAG OPTIONS

DEFINE_bool(ext, false, "Jointly solve for extrinsics");
DEFINE_bool(cal, false, "Jointly solve for ligthouse calibration");
DEFINE_bool(skip, false, "Skip the light error-correction phase");
DEFINE_bool(tf, false, "Send a TF transform on every solution");
DEFINE_string(init, "0 0 0 0 0 1", "Intial pose");

// DATA TYPES

#define MAX_NUM_SENSORS 32
#define MAX_NUM_MOTORS  2

enum {
  CAL_PHASE,
  CAL_TILT,
  CAL_GIB_PHASE,
  CAL_GIB_MAG,
  CAL_CURVE,
  NUM_CAL
};

// Pose and trajectory
struct Pose {
  double val[6];
};
typedef std::map<ros::Time,Pose> Trajectory;

// Tracker data structure
struct Tracker {
  Trajectory estimate;
  double parameters[MAX_NUM_SENSORS][6];
};
typedef std::map<std::string,Tracker> Trackers;

// Lighthouse data structures
struct Lighthouse {
  Pose estimate;
  double parameters[MAX_NUM_MOTORS][NUM_CAL];
};
typedef std::map<std::string,Lighthouse> Lighthouses;

// SHARED DATA and MUTEX LOCK

std::map<std::string,deepdive_ros::Tracker> trackers_;
std::map<std::string,deepdive_ros::Lighthouse> lighthouses_;
std::queue<deepdive_ros::Light> light_;
std::queue<nav_msgs::Odometry> odometry_;
std::mutex mutex_;

// CERES SOLVER THREAD

ros::Publisher pub_tracker_;
ros::Publisher pub_lighthouse_;

// The Vive system produces a bundle of angle estimates to a given lighthouse.
// This function predicts these measurements, given an estimate of the location
// of the lighthouse, the location of the tracker and calibration parameters
struct LightCost {
  // Constructor
  explicit LightCost(deepdive_ros::Light const& measurement) :
    measurement_(measurement) {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const lighthouse,  // Lighthouse pose in world frame
                  const T* const cal,         // Lighthouse calibration
                  const T* const tracker,     // Tracker pose in world frame
                  const T* const extrinsics,  // Sensor extrinsics
                  T* residual) const {
    // We are going to do this in the tracker frame, so that the extrinsics
    // calculations are extremely simple additions :)
    static T delta[3], ax[2];
    ceres::AngleAxisRotatePoint(&tracker[3], lighthouse, delta);
    delta[0] += tracker[0];
    delta[1] += tracker[1];
    delta[2] += tracker[2];
    // Get the axis of this measurement
    size_t a = static_cast<size_t>(measurement_.axis);
    // Iterate over all the pluses that were received
    for (size_t i = 0; i < measurement_.pulses.size(); i++) {
      // Get the ID of the sensor
      size_t s = static_cast<size_t>(measurement_.pulses[i].sensor);
      // Move to the sensor frame
      delta[0] += T(extrinsics[3*s+0]);
      delta[1] += T(extrinsics[3*s+1]);
      delta[2] += T(extrinsics[3*s+2]);
      // Calculate the raw angles
      ax[deepdive_ros::Motor::AXIS_HORIZONTAL] = atan(delta[0]/delta[2]);
      ax[deepdive_ros::Motor::AXIS_VERTICAL]= atan(delta[1]/delta[2]);
      // Apply the error correction as needed. I am going to assume that the
      // engineers kept this equation as simple as possible, and infer the
      // meaning of the calibration parameters based on their name. It might be
      // the case that these value are subtracted, rather than added. 
      if (!FLAGS_skip) {
        ax[a] += T(cal[CAL_PHASE]);
        ax[a] += T(cal[CAL_TILT]) * ax[1-a];
        ax[a] += T(cal[CAL_CURVE]) * ax[1-a] * ax[1-a];
        ax[a] += T(cal[CAL_GIB_MAG]) * cos(ax[1-a] + T(cal[CAL_GIB_PHASE]));
      }
      // The residual is the axis of interest minus the predicted value as
      // given by the light measurements
      residual[i] = ax[a] - T(measurement_.pulses[i].angle);
      // Go back to the tracker frame
      delta[0] -= T(extrinsics[3*s+0]);
      delta[1] -= T(extrinsics[3*s+1]);
      delta[2] -= T(extrinsics[3*s+2]);
    }
    // Everything went well
    return true;
  }
 // Internal variables
 private:
  deepdive_ros::Light measurement_;
};

// The MotionCost is produced by the tracking filter that runs alongside this
// solver. It is essentially a PoseWithCovariance that acts as a relative
// motion measurment between two tracker poses.
struct MotionCost {
  // Constructor
  explicit MotionCost(geometry_msgs::PoseWithCovariance const& measurement) :
    measurement_(measurement) {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const tracker_i,  // Tracker position at i
                  const T* const tracker_j,  // Tracker pose at j
                  T* residual) const {
    // Measure the position of the tracker at time j, which is simply
    // the original position at i offset by the measurement
    static T x_j_meas[3];
    x_j_meas[0] = T(measurement_.pose.position.x) + tracker_i[0];
    x_j_meas[1] = T(measurement_.pose.position.y) + tracker_i[1];
    x_j_meas[2] = T(measurement_.pose.position.z) + tracker_i[2];
    // Measure the orientation of the ttracker at time j, which is
    // slightly harder than the position. We need to convert the orientation
    // at time i to a quaternion, then multiply it by the transform
    // quaternion.
    static T q_i[4], q_ji[4], q_j_meas[4];
    q_ji[0] = T(measurement_.pose.orientation.w);
    q_ji[1] = T(measurement_.pose.orientation.x);
    q_ji[2] = T(measurement_.pose.orientation.y);
    q_ji[3] = T(measurement_.pose.orientation.z);
    ceres::AngleAxisToQuaternion(&tracker_i[3], q_i);
    ceres::QuaternionProduct(q_ji, q_i, q_j_meas);
    // Calculate the residual error, which is just the predicted tracker
    // position at time j, less the measures position. The orientation
    // error is just qpred.inv() * qmeas.inv() to axis-angle representation.
    static T q_j_pred[4], q_err[4];
    residual[0] = tracker_j[0] - x_j_meas[0];
    residual[1] = tracker_j[1] - x_j_meas[1];
    residual[2] = tracker_j[2] - x_j_meas[2];
    q_j_meas[0] =  q_j_meas[0];
    q_j_meas[1] = -q_j_meas[1];
    q_j_meas[2] = -q_j_meas[2];
    q_j_meas[3] = -q_j_meas[3];
    ceres::AngleAxisToQuaternion(&tracker_j[3], q_j_pred);
    ceres::QuaternionProduct(q_j_pred, q_j_meas, q_err);
    ceres::QuaternionToAngleAxis(q_err, &residual[3]);
    // Everything went well
    return true;
  }
 // Internal variables
 private:
  geometry_msgs::PoseWithCovariance measurement_;
};


// Send a corretc pose on the ROS messaging system
void SendPose(ros::Publisher &pub, std::string const& serial,
  Pose pose, ros::Time epoch) {
  // Get a quaternion from the internal angle axis representation
  static double quaternion[4];
  ceres::AngleAxisToQuaternion<double>(&pose.val[3], quaternion);
  // Assemble the pose correction
  static geometry_msgs::PoseStamped msg;
  msg.header.stamp = epoch;
  msg.header.frame_id = serial;
  msg.pose.position.x = pose.val[0];
  msg.pose.position.y = pose.val[1];
  msg.pose.position.z = pose.val[2];
  msg.pose.orientation.w = quaternion[0];
  msg.pose.orientation.x = quaternion[1];
  msg.pose.orientation.y = quaternion[2];
  msg.pose.orientation.z = quaternion[3];
  // Send out the pose correction
  pub.publish(msg);
  // If we want a TF2 transform, then send it now
  if (FLAGS_tf) {
    static tf2_ros::TransformBroadcaster br;  
    static geometry_msgs::TransformStamped tf;
    tf.header.stamp = epoch;
    tf.header.frame_id = "world";
    tf.child_frame_id = serial;
    tf.transform.translation.x = pose.val[0];
    tf.transform.translation.y = pose.val[1];
    tf.transform.translation.z = pose.val[2];
    tf.transform.rotation.w = quaternion[0];
    tf.transform.rotation.x = quaternion[1];
    tf.transform.rotation.y = quaternion[2];
    tf.transform.rotation.z = quaternion[3];
    br.sendTransform(tf);
  }
}

void WorkerThread() {
  // We only want to keep going if ROS is alive. When 
  while (ros::ok()) {
    // Construct and persist the ceres solver problem
    static ceres::Problem problem;
    // In the copy-phase we must prevent the primary thread from
    // injecting any more data into the structures while we read
    static Trackers trackers;
    static Lighthouses lighthouses;
    mutex_.lock();
    // Copy the lighthouse calibration
    {
      std::map<std::string,deepdive_ros::Lighthouse>::iterator it;
      for (it = lighthouses_.begin(); it != lighthouses_.end(); it++) {
        for (size_t i = 0; i < 2; i++) {
          lighthouses[it->first].parameters[i][CAL_PHASE]
            = it->second.motors[i].phase;
          lighthouses[it->first].parameters[i][CAL_TILT]
            = it->second.motors[i].tilt;
          lighthouses[it->first].parameters[i][CAL_GIB_PHASE]
            = it->second.motors[i].gibphase;
          lighthouses[it->first].parameters[i][CAL_GIB_MAG]
            = it->second.motors[i].gibmag;
          lighthouses[it->first].parameters[i][CAL_CURVE]
            = it->second.motors[i].curve;
        }
      }
      // By default keep the calibration constant throughout estimation. The
      // flag (-cal) will prevent this from happening, and jointly solve for
      // the calibration in addition to the sensor and lighthouse poses
      if (!FLAGS_cal)
        problem.SetParameterBlockConstant(
          &lighthouses[it->first].parameters[0][0]);
    }
    // Copy the tracker extrinsics
    {
      std::map<std::string,deepdive_ros::Tracker>::iterator it;
      for (it = trackers_.begin(); it != trackers_.end(); it++) {
        for (size_t i = 0; i < MAX_NUM_SENSORS; i++) {
          if (i < it->second.sensors.size()) {
            // Copy the position
            trackers[it->first].parameters[i][0]
              = it->second.sensors[i].position.x;
            trackers[it->first].parameters[i][1]
              = it->second.sensors[i].position.y;
            trackers[it->first].parameters[i][2]
              = it->second.sensors[i].position.z;
            // Copy the normal
            trackers[it->first].parameters[i][3]
              = it->second.sensors[i].normal.x;
            trackers[it->first].parameters[i][4]
              = it->second.sensors[i].normal.y;
            trackers[it->first].parameters[i][5]
              = it->second.sensors[i].normal.z;
          }
          // By default keep the extrinsics constant throughout estimation. The
          // flag (-ext) will prevent this from happening, and jointly solve for
          // the extrinsics in addition to the sensor and lighthouse poses
          if (!FLAGS_ext)
            problem.SetParameterBlockConstant(
              &trackers[it->first].parameters[i][0]);
        }
      }
    }
    // Iterate over all light measurements, adding a reisdual block for each
    // bundle, which uses the predicted lighthouse and tracker poses to predict
    // the angles to the lighthouses.
    while (light_.size()) {
      // Get the critical information about this measurement
      Lighthouse & lighthouse = lighthouses[light_.front().lighthouse];
      Tracker & tracker = trackers[light_.front().header.frame_id];
      ros::Time & time = light_.front().header.stamp;
      uint8_t & axis = light_.front().axis;
      // Add a residual block to the problem representing this measurent
      ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<LightCost,
        ceres::DYNAMIC, 3, NUM_CAL, 3, MAX_NUM_SENSORS>(
          new LightCost(light_.front()), light_.front().pulses.size());
      problem.AddResidualBlock(cost,
        new ceres::CauchyLoss(0.5),
        &lighthouse.estimate.val[0],
        &lighthouse.parameters[axis][0],
        &tracker.estimate[time].val[0],
        &tracker.parameters[0][0]);
      // Remove the measurment
      light_.pop();
    }
    // We are now finished copying the data, so we can unlock the mutex and
    // allow the primary thread to send us more data for susequent rounds.
    mutex_.unlock();
    // In the find-phase we call on ceres to solve the problem
    static ceres::Solver::Options options;
    static ceres::Solver::Summary summary;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solve(options, &problem, &summary);
    // In the send-phase, we check and send out the solution
    {
      Lighthouses::iterator it;
      for (it = lighthouses.begin(); it != lighthouses.end(); it++)
        SendPose(pub_lighthouse_, it->first, it->second.estimate,
          ros::Time(0));
    }
    {
      Trackers::iterator it;
      for (it = trackers.begin(); it != trackers.end(); it++)
        SendPose(pub_tracker_, it->first, it->second.estimate.rbegin()->second,
          it->second.estimate.rbegin()->first);
    }
    // We are now done.
  }
}

// MESSAGE CALLBACKS

ros::ServiceClient client_lighthouse_;
ros::ServiceClient client_tracker_;

bool GetLighthouseInfo(std::string const& serial) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (lighthouses_.find(serial) != lighthouses_.end())
    return true;
  // If we get here, then we need to extract the lighthouse
  // data from the bridge. This will take a short while.
  deepdive_ros::GetLighthouse srv;
  srv.request.serial = serial;
  if (client_lighthouse_.call(srv) &&
      srv.response.lighthouses.size() == 1) {
    lighthouses_[serial] = srv.response.lighthouses[0];
    return true;
  }
  // The call failed, or the lighthouse was not found, or
  // the server found too many lighthouses. All are problems.
  return false;
}

bool GetTrackerInfo(std::string const& serial) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (trackers_.find(serial) != trackers_.end())
    return true;
  // If we get here, then we need to extract the lighthouse
  // data from the bridge. This will take a short while.
  deepdive_ros::GetTracker srv;
  srv.request.serial = serial;
  if (client_tracker_.call(srv) &&
    srv.response.trackers.size() == 1) {
    trackers_[serial] = srv.response.trackers[0];
    return true;
  }
  // The call failed, or the lighthouse was not found, or
  // the server found too many lighthouses. All are problems.
  return false;
}

void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  if (!GetLighthouseInfo(msg->lighthouse))
    return;
  if (!GetTrackerInfo(msg->header.frame_id))
    return;
  std::unique_lock<std::mutex> lock(mutex_);
  light_.push(*msg);
}

void OdometryCallback(nav_msgs::Odometry::ConstPtr const& msg) {
  if (!GetTrackerInfo(msg->child_frame_id))
    return;
  std::unique_lock<std::mutex> lock(mutex_);
  odometry_.push(*msg);
}

// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_solver");
  ros::NodeHandle nh;

  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun deepdive_ros deepdive_solver <opts>");
  google::SetVersionString("0.0.1");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Subscribe to the motion and light callbacks
  ros::Subscriber sub_motion
    = nh.subscribe("odometry", 10, OdometryCallback);
  ros::Subscriber sub_light
    = nh.subscribe("light", 10, LightCallback);

  // Service callbacks for getting lighthous and tracker calibration
  client_lighthouse_ = 
    nh.serviceClient<deepdive_ros::GetLighthouse>("get_lighthouse");
  client_tracker_ = 
    nh.serviceClient<deepdive_ros::GetLighthouse>("get_tracker");

  // Publisher for tracker and lighthouse positions
  pub_tracker_
    = nh.advertise<geometry_msgs::PoseStamped>("tracker", 10);
  pub_lighthouse_
    = nh.advertise<geometry_msgs::PoseStamped>("lighthouse", 10);

  // Start a thread to listen to vive
  std::thread(WorkerThread);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}