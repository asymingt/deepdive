#ifndef SRC_DEEPDIVE_HH
#define SRC_DEEPDIVE_HH

// ROS
#include <ros/ros.h>

// Types
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

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

// Recursively calculates mean and standar deviation
class Statistics {
 public:
  // Constructor and initialization
  Statistics() : count_(0.0), mean_(0.0), var_(0.0) {};

  // Feed a value and calculate recursive statistics
  void Feed(double value) {
    count_ += 1.0;
    if (count_ < 2) {
      mean_ = value;
      var_ = 0.0;
      return;
    }
    var_ = (count_ - 2.0) / (count_ - 1.0) * var_
         + (value - mean_) * (value - mean_) / count_;
    mean_ = ((count_ - 1.0) * mean_ + value) / count_;
  }

  // Get statistics
  double Count() { return count_; }
  double Mean() { return mean_; }
  double Variance() { return var_; }
  double Deviation() { return sqrt(var_); }

  // Reset the statistics
  void Reset() {
    count_ = 0.0;
    mean_ = 0.0;
    var_ = 0.0;
  }

 private:
  double count_;
  double var_;
  double mean_;
};



#endif
