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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// Eigen includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// C++ includes
#include <vector>
#include <map>
#include <string>
#include <thread>

// HELPER FUNCTIONS FOR CONFIG

bool GetVectorParam(ros::NodeHandle &nh,
  std::string const& name, Eigen::Vector3d & data) {
  std::vector<double> tmp;
  if (!nh.getParam(name, tmp) || tmp.size() != 3)
    return false;
  data[0] = tmp[0];
  data[1] = tmp[1];
  data[2] = tmp[2];
  return true;
}

bool GetQuaternionParam(ros::NodeHandle &nh,
  std::string const& name, Eigen::Quaterniond & data) {
  std::vector<double> tmp;
  if (!nh.getParam(name, tmp) || tmp.size() != 4)
    return false;
  data.x() = tmp[0];
  data.y() = tmp[1];
  data.z() = tmp[2];
  data.w() = tmp[3];
  return true;
}

// GLOBAL MEMORY

// Global publishers
ros::Publisher pub_pose_;
ros::Publisher pub_twist_;

// Frame names
std::string frame_parent_;
std::string frame_child_;

// Parent bundle
struct Parent {
  ros::Subscriber sub_pose;
  ros::Subscriber sub_twist;
  Eigen::Affine3d bTh;
};
std::map<std::string, Parent> parents_;

// Body-frame pose estimates
struct PEst {
  Eigen::Affine3d est;
  Eigen::Matrix<double, 6, 6> cov;
};
std::map<ros::Time, PEst> pose_estimates_;

// Body-frame velocity estimates
struct TEst {
  Eigen::Vector3d lvel;
  Eigen::Vector3d avel;
  Eigen::Matrix<double, 6, 6> cov;
};
std::map<ros::Time, TEst> twist_estimates_;

// CALLBACKS

// Called when a tracking result must be calculated
void TimerCallback(ros::TimerEvent const& info) {
  // If we have enough samples to calculate a pose, then we do so by
  // weighting all samples obtained over the last collection period by
  // the inverse of the diagonal of the covariance matrix. The rotation
  // is converted to axis angle, so that the fusion make sense...
  if (!pose_estimates_.empty()) {
    Eigen::Vector3d pval, pcnt, rval, rcnt;
    std::map<ros::Time, PEst>::const_iterator it;
    for (it = pose_estimates_.begin(); it != pose_estimates_.end(); it++) {
      Eigen::AngleAxisd aa(it->second.est.linear());
      for (size_t i = 0; i < 3; i++) {
        pval[i] += it->second.est.translation()[i] / it->second.cov(i, i);
        pcnt[i] += 1.0 / it->second.cov(i, i);
        rval[i] += aa.angle() * aa.axis()[i] / it->second.cov(3 + i, 3 + i);
        rcnt[i] += 1.0 / it->second.cov(3 + i, 3 + i);
      }
    }
    // Normalize the weighted axis angle
    for (size_t i = 0; i < 3; i++)
      rval[i] /= rcnt[i];
    // Convert back to a quaternion
    Eigen::AngleAxisd aa(rval.norm(), rval.normalized());
    Eigen::Quaterniond q(aa.toRotationMatrix());
    // Package and send the message
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_parent_;
    msg.pose.position.x = pval[0] / pcnt[0];
    msg.pose.position.y = pval[1] / pcnt[1];
    msg.pose.position.z = pval[2] / pcnt[2];
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    pub_pose_.publish(msg);
    // Package and send on TF2
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = frame_child_;
    tf.transform.translation.x = msg.pose.position.x;
    tf.transform.translation.y = msg.pose.position.y;
    tf.transform.translation.z = msg.pose.position.z;
    tf.transform.rotation = msg.pose.orientation;
    br.sendTransform(tf);
    // Clear the data
    pose_estimates_.clear();
  }
  // If we have enough samples to calculate a twist, then we do so by
  // weighting all samples obtained over the last collection period by
  // the inverse of the diagonal of the covariance matrix.
  if (!twist_estimates_.empty()) {
    Eigen::Vector3d lval, lcnt, aval, acnt;
    std::map<ros::Time, TEst>::const_iterator it;
    for (it = twist_estimates_.begin(); it != twist_estimates_.end(); it++) {
      for (size_t i = 0; i < 3; i++) {
        lval[i] += it->second.lvel[i] / it->second.cov(i, i);
        lcnt[i] += 1.0 / it->second.cov(i, i);
        aval[i] += it->second.avel[i] / it->second.cov(3 + i, 3 + i);
        acnt[i] += 1.0 / it->second.cov(3 + i, 3 + i);
      }
    }
    // Package and send the message
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_parent_;
    msg.twist.linear.x = lval[0] / lcnt[0];
    msg.twist.linear.y = lval[1] / lcnt[1];
    msg.twist.linear.z = lval[2] / lcnt[2];
    msg.twist.angular.x = aval[0] / acnt[0];
    msg.twist.angular.y = aval[1] / acnt[1];
    msg.twist.angular.z = aval[2] / acnt[2];
    pub_twist_.publish(msg);
    // Clear the data
    twist_estimates_.clear();
  }
}

// Called when a pose estimate becomes available
void PoseCallback(
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr const& msg,
    Parent const& parent) {

  // Get the matrix needed to rotate change the covariance coordinate frame
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
  R.block<3, 3>(0, 0) = parent.bTh.linear();
  R.block<3, 3>(3, 3) = parent.bTh.linear();

  // Get a reference to the pose estimate
  PEst & pest = pose_estimates_[msg->header.stamp];
  pest.est.translation() = Eigen::Vector3d(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z
  );
  pest.est.linear() = Eigen::Quaterniond(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z
  ).toRotationMatrix();

  // Get the covariance
  pest.cov = Eigen::Matrix<double, 6, 6>::Zero();
  for (size_t i = 0; i < 6; i++)
    for (size_t j = 0; j < 6; j++)
      pest.cov(i, j) = msg->pose.covariance[i*6 + j];

  // Transform the estimate and covariance
  pest.cov = R * pest.cov * R.transpose();
  pest.est = parent.bTh * pest.est;
}

// Called when a twist estimate becomes available
void TwistCallback(
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr const& msg,
    Parent const& parent) {

  // Get the matrix needed to rotate change the covariance coordinate frame
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
  R.block<3, 3>(0, 0) = parent.bTh.linear();
  R.block<3, 3>(3, 3) = parent.bTh.linear();

  // Get the estimate
  TEst & test = twist_estimates_[msg->header.stamp];
  test.lvel = Eigen::Vector3d(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z
  );
  test.avel = Eigen::Vector3d(
    msg->twist.twist.angular.x,
    msg->twist.twist.angular.y,
    msg->twist.twist.angular.z
  );

  // Get the covariance
  test.cov = Eigen::Matrix<double, 6, 6>::Zero();
  for (size_t i = 0; i < 6; i++)
    for (size_t j = 0; j < 6; j++)
      test.cov(i, j) = msg->twist.covariance[i*6 + j];

  // Transform the estimate and covariance
  test.cov = R * test.cov * R.transpose();
  test.lvel = parent.bTh.linear() * test.lvel;
  test.avel = parent.bTh.linear() * test.avel;
}

// Main entry point of application
int main(int argc, char **argv) {
  // Initialize ROS and create node handle
  ros::init(argc, argv, "deepdive_fusion");
  ros::NodeHandle nh("~");

  // Eigen uses [w, x, y, z]
  Eigen::Quaterniond q(.7071, 0, 0, .7071);
  Eigen::Matrix3d mat = q.toRotationMatrix().inverse();
  for (size_t r = 0; r < 3; r++)
    for (size_t c = 0; c < 3; c++)
      ROS_INFO_STREAM("(" << r << ", " << c << ") " << mat(r,c));
  Eigen::Vector3d sol = mat * Eigen::Vector3d(1.0, 0.0, 0.0);
  ROS_INFO_STREAM(sol[0]);
  ROS_INFO_STREAM(sol[1]);
  ROS_INFO_STREAM(sol[2]);


  // Get the parent information
  std::vector<std::string> parents;
  if (!nh.getParam("parents", parents))
    ROS_FATAL("Failed to get the parent list.");
  std::vector<std::string>::iterator it;
  for (it = parents.begin(); it != parents.end(); it++) {
    // Get a reference to the stat storage for this tracker
    Parent & parent = parents_[*it];

    // Temporary storage
    std::string topic;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;

    // Setup a pose subscriber
    if (!nh.getParam(*it + "/pose", topic))
      ROS_FATAL("Failed to get pose parameter.");
    parent.sub_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      topic, 10, boost::bind(PoseCallback, _1, parent));

    // Setup a twist subscriber
    if (!nh.getParam(*it + "/twist", topic))
      ROS_FATAL("Failed to get twist parameter.");
    parent.sub_twist = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>(
      topic, 10, boost::bind(TwistCallback, _1, parent));

    // Set the tracker to body transformation
    if (!GetVectorParam(nh, *it + "/translation", translation))
      ROS_FATAL("Failed to get translation parameter.");
    if (!GetQuaternionParam(nh,*it + "/rotation", rotation))
      ROS_FATAL("Failed to get rotation parameter.");
    parent.bTh.translation() = translation;
    parent.bTh.linear() = rotation.toRotationMatrix();
  }

  // Get the topics for data topics
  std::string topic_pose, topic_twist;
  if (!nh.getParam("topics/pose", topic_pose))
    ROS_FATAL("Failed to get topics/pose parameter.");
  if (!nh.getParam("topics/twist", topic_twist))
    ROS_FATAL("Failed to get topics/twist parameter.");  

  // Get some global information
  if (!nh.getParam("frames/parent", frame_parent_))
    ROS_FATAL("Failed to get frames/parent parameter.");
  if (!nh.getParam("frames/child", frame_child_))
    ROS_FATAL("Failed to get frames/child parameter.");  

  // Get the tracker update rate. Anything over the IMU rate is really not
  // adding much, since we don't have a good dynamics model.
  double rate = 100;
  if (!nh.getParam("rate",rate))
    ROS_FATAL("Failed to get rate parameter.");

  /// Setup the publishers to send out pose and twist with covariances
  pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>(topic_pose, 0);
  pub_twist_ = nh.advertise<geometry_msgs::TwistStamped>(topic_twist, 0);

  // Start a timer to callback
  ros::Timer timer = nh.createTimer(
    ros::Duration(ros::Rate(rate)), TimerCallback, false, true);

  // Block until safe shutdown
  ros::spin();

  // Success!
  return 0;
}
