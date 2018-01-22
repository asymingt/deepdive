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
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

// Non-standard datra messages
#include <deepdive_ros/Light.h>
#include <deepdive_ros/Lighthouses.h>
#include <deepdive_ros/Trackers.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// STL includes
#include <algorithm>
#include <thread>
#include <mutex>
#include <string>
#include <map>
#include <queue>

// [PROBLEM SETUP] /////////////////////////////////////////////////////////////
//
// We have the following relationship:
//
//                                            <motion>       [calibration]
//                                               |                 |
//                                               '                 '
//  Sensor 1-N -- (extrincs) -- Tracker 1-M -- (Tt_b) -- Body -- (Tb_w) -- World
//          |                                                              |
//          '-----[angles,duration]------ Lighthouse 1-L ------(Tl_w)------'
//
// Where
//    Ti = CONSTANT tracker -> sensor transforms
//    Tj = CONSTANT body -> tracker transforms
//    Tk = DYNAMIC robot trajectory (what we ultimately want)
//    Tl = CONSTANT world -> lighthouse transforms
//
// The measurements are angles between the lighthouse and a specific sensor.
//
// For the solution to be tractable, we need to supply at least one calibration
// correction to the Tk. More can be supplied if required. Motion measurements
// from a parallel filter can also be included to constrain the motion between
// sequential synchronization rounds. Right now, we try to push the poses as
// close together as possible, with a weightin based on speed contrraint.
//
////////////////////////////////////////////////////////////////////////////////

#define MAX_NUM_SENSORS      32
#define SIGMA_ANGLE          1.0
#define SIGMA_MOTION         1.0

// Calibration parameter
enum {
  CAL_PHASE,
  CAL_TILT,
  CAL_GIB_PHASE,
  CAL_GIB_MAG,
  CAL_CURVE,
  SIZE_CAL
};

// Motor axes
enum {
  AXIS_VERTICAL,
  AXIS_HORIZONAL,
  SIZE_AXIS
};

// Trajectory of body-frame
struct Pose {
  double Tb_w[6];
};
typedef std::map<ros::Time,Pose> Trajectory;

// Tracker data structure
struct Tracker {
  double Tt_b[6];
  double extrinsics[MAX_NUM_SENSORS*6];
};
typedef std::map<std::string,Tracker> Trackers;

// Lighthouse data structures
struct Lighthouse {
  double Tl_w[6];
  double calibration[SIZE_AXIS][SIZE_CAL];
};
typedef std::map<std::string,Lighthouse> Lighthouses;

// SHARED DATA and MUTEX LOCK

std::queue<deepdive_ros::Tracker> trackers_;
std::queue<deepdive_ros::Lighthouse> lighthouses_;
std::queue<deepdive_ros::Light> light_;
std::queue<geometry_msgs::PoseStamped> corrections_;
std::mutex mutex_;

// CERES SOLVER THREAD

// Helper function to combine transforms Tji = Tj * Ti
template <typename T> inline
void CombineTransforms(const T ti[6], const T tj[6], T tji[6]) {
  static T qi[4], qj[4], qji[4];
  ceres::AngleAxisToQuaternion(&ti[3], qi);
  ceres::AngleAxisToQuaternion(&tj[3], qj);
  ceres::QuaternionProduct(qj, qi, qji);
  ceres::QuaternionToAngleAxis(qji, &tji[3]);
  for (size_t i = 0; i < 3; i++)
    tji[i] = ti[i] + tj[i];
}

// Helper function to apply a transform
template <typename T> inline
void ApplyTransform(const T Tj_i[6], const T x_i[3], T x_j[3]) {
  ceres::AngleAxisRotatePoint(&Tj_i[3], x_i, x_j);
  for (size_t i = 0; i < 3; i++)
    x_j[i] += Tj_i[i];
}

// Helper function to invert a transform
template <typename T> inline
void InvertTransform(const T Tj_i[6], T Ti_j[6]) {
  for (size_t i = 0; i < 6; i++)
    Ti_j[i] = -Tj_i[i];
}

// The basic principle of the cost functor is to predict the angles between
// the lighthouse and the sensor frame. To do this, we need to move the 
struct LightCost {
  // Constructor
  explicit LightCost(deepdive_ros::Light const& measurement) :
    measurement_(measurement) {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const Tl_w,          // Lighthouse pose
                  const T* const Tb_w,          // Body pose
                  const T* const Tt_b,          // Tracker pose
                  const T* const calibration,   // Lighthouse calibration
                  const T* const extrinsics,    // Tracker extrinsics
                  T* residual) const {
    // Get the transform that moves the sensor from the tracker to
    static T Tw_t[6], Tw_b[6], Tb_t[6], Tl_t[6], Sx[3], Sn[3], ax[2];
    static uint16_t a, s;
    // Get the axis of this measurement
    a = static_cast<uint16_t>(measurement_.axis);
    // Get the transform that moves the tracker to the world frame
    InvertTransform(Tb_w, Tw_b);
    InvertTransform(Tt_b, Tb_t);
    CombineTransforms(Tw_b, Tb_t, Tw_t);
    // Iterate over all emasurements
    for (size_t i = 0; i < measurement_.pulses.size(); i++) {
      // Get the transform that moves the tracker to the world frame
      CombineTransforms(Tl_w, Tw_t, Tl_t);
      // Get the sensor position in the lighthouse frame
      s = static_cast<uint16_t>(measurement_.pulses[i].sensor);
      if (s >= MAX_NUM_SENSORS) {
        ROS_WARN("Sensor ID error");
        return false;
      }
      ApplyTransform(Tl_t, &extrinsics[6*s+0], Sx);
      ApplyTransform(Tl_t, &extrinsics[6*s+3], Sn);
      // Get the horizontal / vertical angles between the sensor and lighthouse
      // This might cause numerical instability
      switch (a) {
      case deepdive_ros::Motor::AXIS_HORIZONTAL:
        residual[i] = atan2(Sx[1], Sx[2]) + T(measurement_.pulses[i].angle);
        break;
      case deepdive_ros::Motor::AXIS_VERTICAL:
        residual[i] = atan2(Sx[0], Sx[2]) - T(measurement_.pulses[i].angle);
        break;
      default:
        ROS_WARN("Axis ID error");
        return false;
      }
      residual[i] /= T(SIGMA_ANGLE);
      // ax[deepdive_ros::Motor::AXIS_HORIZONTAL] = -atan(Sx[1] / Sx[2]);
      // ax[deepdive_ros::Motor::AXIS_VERTICAL] = atan(Sx[0] / Sx[2]);
      // Apply the error correction as needed. I am going to assume that the
      // engineers kept this equation as simple as possible, and infer the
      // meaning of the calibration parameters based on their name. It might
      // be the case that these value are subtracted, rather than added. 
      // ax[a] += T(calibration[CAL_PHASE]);
      // ax[a] += T(calibration[CAL_TILT]) * ax[1-a];
      // ax[a] += T(calibration[CAL_CURVE]) * ax[1-a] * ax[1-a];
      // ax[a] += T(calibration[CAL_GIB_MAG]) * cos(ax[1-a] + T(calibration[CAL_GIB_PHASE]));
      // The residual is the axis of interest minus the predicted value as
      // given by the light measurements
      // residual[i] = ax[a] - T(measurement_.pulses[i].angle);
    }
    // Everything went well
    return true;
  }
 // Internal variables
 private:
  deepdive_ros::Light measurement_;
};

// Fix the body pose in several places
struct MotionCost {
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const Tb_w_i,
                  const T* const Tb_w_j,
                  T* residual) const {
    // Try and force the two poses close to each other
    for (size_t i = 0; i < 6; i++)
      residual[i] = (Tb_w_i[i] - Tb_w_j[i]) / T(SIGMA_MOTION);
    return true;
  }
};

// Fix the body pose in several places
struct CorrectionCost {
  // Constructor
  explicit CorrectionCost(geometry_msgs::Pose const& measurement) :
    measurement_(measurement) {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const Tb_w, T* residual) const {
    static T pose[6], q[4];
    // Set the position
    pose[0] = T(measurement_.position.x);
    pose[1] = T(measurement_.position.y);
    pose[2] = T(measurement_.position.z);
    // Set the orientation
    q[0] = T(measurement_.orientation.w);
    q[1] = T(measurement_.orientation.x);
    q[2] = T(measurement_.orientation.y);
    q[3] = T(measurement_.orientation.z);
    ceres::QuaternionToAngleAxis(q, &pose[3]);
    // Calculate the residual error between the poses
    for (size_t i = 0; i < 6; i++)
      residual[i] = Tb_w[i] - pose[i];
    // Everything went well
    return true;
  }
 // Internal variables
 private:
  geometry_msgs::Pose measurement_;
};

// Random initialization of pose
void Print(double t[6]) {
  ROS_INFO_STREAM("[POS]" <<
           " x: "  << t[0] <<
           " y: "  << t[1] <<
           " z: "  << t[2]);
}

// Publisher for the nav_msgs::Path
ros::Publisher pub_trajectory_;
ros::Publisher pub_stations_;

// Just keep solving in the background
void WorkerThread() {
  // Allocate datato solve the problem
  static Trajectory trajectory;
  static Trackers trackers;
  static Lighthouses lighthouses;
  static bool unsolvable = true;
  // Construct and persist the ceres solver problem
  static ceres::Problem problem;
  // We only want to keep going if ROS is alive. When 
  ros::Rate rate_loop(1.0);
  while (ros::ok()) {

    // COPY- PHASE

    // Lock to avoid contention with primary thread
    mutex_.lock();

    // Copy the lighthouse calibration

    // Copy the tracker extrinsics
    while (lighthouses_.size()) {
      deepdive_ros::Lighthouse & lighthouse = lighthouses_.front();
      std::string & serial = lighthouse.serial;
      // ROS_INFO_STREAM("Adding lighthouse with serial " << serial);
      for (size_t i = 0; i < 2; i++) {
        lighthouses[serial].calibration[i][CAL_PHASE]
          = lighthouse.motors[i].phase;
        lighthouses[serial].calibration[i][CAL_TILT]
          = lighthouse.motors[i].tilt;
        lighthouses[serial].calibration[i][CAL_GIB_PHASE]
          = lighthouse.motors[i].gibphase;
        lighthouses[serial].calibration[i][CAL_GIB_MAG]
          = lighthouse.motors[i].gibmag;
        lighthouses[serial].calibration[i][CAL_CURVE]
          = lighthouse.motors[i].curve;
      }
      lighthouses_.pop();
    }

    // Copy the tracker extrinsics
    while (trackers_.size()) {
      deepdive_ros::Tracker & tracker = trackers_.front();
      std::string & serial = tracker.serial;
      // ROS_INFO_STREAM("Adding tracker with serial " << serial);
      for (size_t i = 0; i < tracker.sensors.size(); i++) {
        trackers[serial].extrinsics[6*i+0] = tracker.sensors[i].position.x;
        trackers[serial].extrinsics[6*i+1] = tracker.sensors[i].position.y;
        trackers[serial].extrinsics[6*i+2] = tracker.sensors[i].position.z;
        trackers[serial].extrinsics[6*i+3] = tracker.sensors[i].normal.x;
        trackers[serial].extrinsics[6*i+4] = tracker.sensors[i].normal.y;
        trackers[serial].extrinsics[6*i+5] = tracker.sensors[i].normal.z;
      }
      trackers_.pop();
    }

    // Iterate over all light measurements, adding a reisdual block for each
    // bundle, which uses the predicted lighthouse and tracker poses to predict
    // the angles to the lighthouses.
    // ROS_INFO_STREAM("Adding " << light_.size() << " light measurements");
    while (light_.size()) {
      // If we haven't yet received tracker data, then don't add these
      // measurements, as the extrinsics aren't yet initialized.
      Trackers::iterator curr = trackers.find(light_.front().header.frame_id);
      if (curr != trackers.end()) {
        // Find the closest "earliest" pose to the correction
        Trajectory::iterator prev = trajectory.lower_bound(
          corrections_.front().header.stamp);
        // Create a pose for this light measurement, if needed
        Pose & pose = trajectory[light_.front().header.stamp];
        // Get the critical information about this measurement
        Lighthouse & lighthouse = lighthouses[light_.front().lighthouse];
        // Add a cost function
        ceres::CostFunction* cost = new ceres::AutoDiffCostFunction
          <LightCost, ceres::DYNAMIC, 6, 6, 6, SIZE_CAL, MAX_NUM_SENSORS * 6>(
            new LightCost(light_.front()), light_.front().pulses.size());
        // Get the axis of this measurement
        size_t a = static_cast<size_t>(light_.front().axis);
        // Create a residual block that relates the correct parameters
        problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
          reinterpret_cast<double*>(lighthouse.Tl_w),
          reinterpret_cast<double*>(pose.Tb_w),
          reinterpret_cast<double*>(curr->second.Tt_b),
          reinterpret_cast<double*>(lighthouse.calibration[a]),
          reinterpret_cast<double*>(curr->second.extrinsics));
        // Optional: set the extrinsics and calibration to contants
        problem.SetParameterBlockConstant(
          reinterpret_cast<double*>(curr->second.Tt_b));
        problem.SetParameterBlockConstant(
          reinterpret_cast<double*>(lighthouse.calibration[a]));
        problem.SetParameterBlockConstant(
          reinterpret_cast<double*>(curr->second.extrinsics));
        // Link this pose to a previous pose using a motion cost
        if (prev != trajectory.end()) {
          // Add a cost function
          ceres::CostFunction* cost = new ceres::AutoDiffCostFunction
            <MotionCost, 6, 6, 6>(new MotionCost());
          // Create a residual block that relates the correct parameters
          problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
            reinterpret_cast<double*>(prev->second.Tb_w),
            reinterpret_cast<double*>(pose.Tb_w));
        }
      }
      // Remove the measurment
      light_.pop();
    }

    // Iterate over all pose measurements, adding a residual block for each
    // ROS_INFO_STREAM("Adding " << corrections_.size() << " pose corrrections");
    while (corrections_.size()) {
      // Find the closest "earliest" pose to the correction
      Trajectory::iterator prev = trajectory.lower_bound(
        corrections_.front().header.stamp);
      if (prev != trajectory.end()) {
        ceres::CostFunction* cost =
          new ceres::AutoDiffCostFunction<CorrectionCost, 6, 6>(
            new CorrectionCost(corrections_.front().pose));
        // Create a residual block that relates the correct parameters
        problem.AddResidualBlock(cost, new ceres::CauchyLoss(0.5),
          reinterpret_cast<double*>(prev->second.Tb_w));
      }
      // We are no longer unsolvable
      unsolvable = false;
      // Remove the measurment
      corrections_.pop();
    }

    // We are now finished copying the data, so we can unlock the mutex
    mutex_.unlock();

    // SOLUTION PHASE

    // If we actually have some data, then use it
    if (trajectory.size()) {
      // If we haven't pinned down the segment with at least one constraint,
      // then we'll need to set the initial pose to zero.
      if (unsolvable) {
        trajectory.begin()->second.Tb_w[0] = 0;
        trajectory.begin()->second.Tb_w[1] = 0;
        trajectory.begin()->second.Tb_w[2] = 0;
        trajectory.begin()->second.Tb_w[3] = 0;
        trajectory.begin()->second.Tb_w[4] = 0;
        trajectory.begin()->second.Tb_w[5] = 0;
        problem.SetParameterBlockConstant(
          reinterpret_cast<double*>(trajectory.begin()->second.Tb_w));
      } else {
        problem.SetParameterBlockVariable(
          reinterpret_cast<double*>(trajectory.begin()->second.Tb_w));
      }
      // Define the ceres problem
      ceres::Solver::Options options;
      options.num_threads = 4;
      options.num_linear_solver_threads = 4;
      options.minimizer_progress_to_stdout = false;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      ceres::Solver::Summary summary;
      // Solve the problem
      ceres::Solve(options, &problem, &summary);
      // Print the trajectory
      {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        Trajectory::iterator it;
        for (it = trajectory.begin(); it != trajectory.end(); it++) {
          static geometry_msgs::PoseStamped ps;
          static double q[4];
          ceres::AngleAxisToQuaternion(&it->second.Tb_w[3], q);
          ps.header.stamp = it->first;
          ps.header.frame_id = "world";
          ps.pose.position.x = it->second.Tb_w[0];
          ps.pose.position.y = it->second.Tb_w[1];
          ps.pose.position.z = it->second.Tb_w[2];
          ps.pose.orientation.w = q[0];
          ps.pose.orientation.x = q[1];
          ps.pose.orientation.y = q[2];
          ps.pose.orientation.z = q[3];
          msg.poses.push_back(ps);
        }
        pub_trajectory_.publish(msg);
      }
      // Print the base stations
      {
        geometry_msgs::PoseArray msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        Lighthouses::iterator it;
        for (it = lighthouses.begin(); it != lighthouses.end(); it++) {
          static geometry_msgs::Pose pose;
          static double q[4];
          ceres::AngleAxisToQuaternion(&it->second.Tl_w[3], q);
          pose.position.x = it->second.Tl_w[0];
          pose.position.y = it->second.Tl_w[1];
          pose.position.z = it->second.Tl_w[2];
          pose.orientation.w = q[0];
          pose.orientation.x = q[1];
          pose.orientation.y = q[2];
          pose.orientation.z = q[3];
          msg.poses.push_back(pose);
        }
        pub_stations_.publish(msg);
      }
    }

    // Sleep for a little
    rate_loop.sleep();
  }
}

// MESSAGE CALLBACKS

std::vector<std::string> permitted_lighthouses_;
std::vector<std::string> permitted_trackers_;

void LighthouseCallback(deepdive_ros::Lighthouses::ConstPtr const& msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  std::vector<deepdive_ros::Lighthouse>::const_iterator it;
  for (it = msg->lighthouses.begin(); it != msg->lighthouses.end(); it++) {
    if (std::find(permitted_lighthouses_.begin(), permitted_lighthouses_.end(),
      it->serial) == permitted_lighthouses_.end()) return;
    lighthouses_.push(*it);
  }
}

void TrackerCallback(deepdive_ros::Trackers::ConstPtr const& msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  std::vector<deepdive_ros::Tracker>::const_iterator it;
  for (it = msg->trackers.begin(); it != msg->trackers.end(); it++) {
    if (std::find(permitted_trackers_.begin(), permitted_trackers_.end(),
      it->serial) == permitted_trackers_.end()) return;
    trackers_.push(*it);
  }
}

void LightCallback(deepdive_ros::Light::ConstPtr const& msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (std::find(permitted_trackers_.begin(), permitted_trackers_.end(),
    msg->header.frame_id) == permitted_trackers_.end()) return;
  if (std::find(permitted_lighthouses_.begin(), permitted_lighthouses_.end(),
    msg->lighthouse) == permitted_lighthouses_.end()) return;
  light_.push(*msg);
}

void CorrectionCallback(geometry_msgs::PoseStamped::ConstPtr const& msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  corrections_.push(*msg);
}


// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_solver");
  ros::NodeHandle nh("~");

  // Get the list of permitted devices in the system
  if (!nh.getParam("lighthouses", permitted_lighthouses_))
    ROS_FATAL("Failed to get device list.");
  if (!nh.getParam("trackers", permitted_trackers_))
    ROS_FATAL("Failed to get device list.");

  // Publish the trajectory
  pub_trajectory_ = nh.advertise<nav_msgs::Path>("/path", 10);
  pub_stations_ = nh.advertise<geometry_msgs::PoseArray>("/stations", 10);

  // Subscribe to tracker and lighthouse updates
  ros::Subscriber sub_tracker  =
    nh.subscribe("/trackers", 10, TrackerCallback);
  ros::Subscriber sub_lighthouse =
    nh.subscribe("/lighthouses", 10, LighthouseCallback);
  ros::Subscriber sub_light =
    nh.subscribe("/light", 10, LightCallback);
  ros::Subscriber sub_corrrection =
    nh.subscribe("/correction", 10, CorrectionCallback);

  // Start a thread to listen to vive
  std::thread thread(WorkerThread);

  // Block until safe shutdown
  ros::spin();

  // Join the thread
  thread.join();

  // Success!
  return 0;
}