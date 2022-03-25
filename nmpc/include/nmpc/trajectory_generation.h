#pragma once

// Acados NMPC
#include "nmpc/acados_nmpc.h"

// ROS
#include "ros/ros.h"

namespace px4_ctrl {
class TrajectoryGeneration {
 public:
  TrajectoryGeneration(ros::NodeHandle &nh);
  ~TrajectoryGeneration();

 private:
  /** @brief Loads the node parameters
   */
  void loadParameters();

  void generateTrajectory();

  // Controller
  AcadosNMPC *nmpc_controller;

  model_parameters model_params;
  trajectory_setpoint drone_state;
  std::vector<double> disturbances, weights;
  std::vector<trajectory_setpoint> current_reference_trajectory;
};
}  // namespace px4_ctrl