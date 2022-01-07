#pragma once

// ROS
#include "ros/ros.h"

// ROS messages
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <state_estimation/DroneState.h>

// Eigen
#include <eigen3/Eigen/Dense>

// Sensors
#include "state_estimation/sensors/mavros_odometry.h"

/**
 * @brief State Observer Class. Implements an EKF to calculate an estimation of
 * the drone state and external disturbances
 * TODO: Add mutexes to make sure that the inputs/states won't change while they
 * are being used
 */
class StateObserver {
 public:
  StateObserver(ros::NodeHandle &nh);
  ~StateObserver();

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

  /** @brief Loads the model parameters
   */
  void loadParameters();

  /**
   * @brief Runs the observer's filter prediction step. Updates state_pred,
   * F_mat and P_pred_mat
   * @param pred_time Time for prediction
   */
  void predict(ros::Time pred_time);

  /** @brief Updates the P_pred_mat for the prediction step
   * @param dt Time step for prediction
   * @param cmd The input for the prediction step
   */
  void updatePpred(const double &dt, const Eigen::Vector4d &cmd);

  /**
   * @brief Gets the rotation matrix from the Euler angles
   * @param yaw Yaw angle
   * @param pitch Pitch angle
   * @param roll Roll angle
   * @returns Rotation matrix
   */
  Eigen::Matrix3d yprToRotMat(const double &yaw, const double &pitch,
                              const double &roll);

  // Observer data
  ros::Time past_state_time;
  bool is_initialized;
  static const int state_size = 12;

  // state = [x, y, z, xdot, ydot, zdot, yaw, pitch, roll, fdx, fdy, fdz]T
  Eigen::Matrix<double, state_size, 1> state;
  Eigen::Matrix<double, state_size, 1> state_pred;
  Eigen::Matrix<double, state_size, state_size> F_mat;
  Eigen::Matrix<double, state_size, state_size> Q_mat;
  Eigen::Matrix<double, state_size, state_size> P_mat;
  Eigen::Matrix<double, state_size, state_size> P_pred_mat;

  // input = [yaw_rate, pitch, roll, thrust]T
  Eigen::Vector4d past_cmd, latest_cmd;
  ros::Time latest_cmd_time;

  // Model parameters
  Eigen::Vector3d gravity_vector;
  Eigen::Matrix3d damping_matrix;
  double t_pitch, k_pitch, t_roll, k_roll;
  double damp_x, damp_y, damp_z;
  double k_thrust;
  double gravity;

  // Sensors
  MavrosOdometry *odom_sensor;
};