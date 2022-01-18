#pragma once

// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_drone_w_disturbances.h"

// ROS
#include "ros/ros.h"

// ROS messages
#include <geometry_msgs/Pose.h>
#include "state_estimation/DroneState.h"

/**
 * @brief PX4 Nonlinear Model Predictive Controller. Uses c code generated from
 * acados for the nmpc
 * TODO: Add trajectory tracking
 */
namespace px4_ctrl {
class Px4Nmpc {
 public:
  Px4Nmpc(ros::NodeHandle &nh);
  ~Px4Nmpc();

 private:
  // ROS Subscribers
  ros::Subscriber drone_state_sub;
  ros::Subscriber setpoint_sub;

  // ROS Publishers
  ros::Publisher control_pub;

  // Callbacks
  void droneStateCallback(const state_estimation::DroneState &msg);
  void setpointCallback(const geometry_msgs::Pose &msg);
};
}  // namespace px4_ctrl