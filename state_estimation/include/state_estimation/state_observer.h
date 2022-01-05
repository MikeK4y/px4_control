#pragma once

// ROS
#include "ros/ros.h"

// ROS messages
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <state_estimation/DroneState.h>

// Eigen
#include <eigen3/Eigen/Dense>

class state_observer {
 public:
  state_observer(ros::NodeHandle &nh);
  ~state_observer();

 private:
  // ROS Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber ctrl_sub;

  // ROS Publishers
  ros::Publisher state_pub;

  // ROS Services
  ros::ServiceClient reset_filter;

  // Callbacks
  void odomCallback(const nav_msgs::Odometry &msg);
  void ctrlCallback(const mavros_msgs::AttitudeTarget &msg);

  /** @brief Gets the H matrix for the odometry measurements using the current
    *estimated state. The odometry message includes p, qW_B, qB_W * pdot.
   *
   **/
  void getOdomH();

  // Observer data
  static const int state_size = 12;
  static const double gravity = 9.8066;

  bool is_initialized;

  // state = [x, y, z, xdot, ydot, zdot, yaw, pitch, roll, fdx, fdy, fdz]
  Eigen::Matrix<double, state_size, 1> state;
  Eigen::Matrix<double, state_size, 1> state_pred;
  Eigen::Matrix<double, state_size, state_size> F_mat;
  Eigen::Matrix<double, state_size, state_size> Q_mat;
  Eigen::Matrix<double, state_size, state_size> P_mat;
  Eigen::Matrix<double, state_size, state_size> P_pred_mat;


};
