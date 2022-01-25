#pragma once

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_drone_w_disturbances.h"

// ROS
#include "ros/ros.h"

// ROS messages
#include "px4_control_msgs/DroneState.h"
#include "px4_control_msgs/Setpoint.h"
#include "px4_control_msgs/Trajectory.h"

/**
 * @brief PX4 Nonlinear Model Predictive Controller. Uses c code generated from
 * acados for the nmpc
 */
namespace px4_ctrl {

struct setpoint {
  double pos_x;
  double pos_y;
  double pos_z;
  double vel_x;
  double vel_y;
  double vel_z;
  double q_roll;
  double q_pitch;
  double q_yaw;
};

class Px4Nmpc {
 public:
  Px4Nmpc(ros::NodeHandle &nh);
  ~Px4Nmpc();

 private:
  // ROS Subscribers
  ros::Subscriber drone_state_sub;
  ros::Subscriber setpoint_sub;
  ros::Subscriber trajectory_sub;

  // ROS Publishers
  ros::Publisher control_pub;

  // Callbacks
  void droneStateCallback(const px4_control_msgs::DroneState &msg);
  void setpointCallback(const px4_control_msgs::Setpoint &msg);
  void trajectoryCallback(const px4_control_msgs::Trajectory &msg);

  /** @brief Loads the model parameters
   */
  void loadParameters();

  double t_pitch, k_pitch, t_roll, k_roll;
  double damp_x, damp_y, damp_z;
  double k_thrust;
  double gravity;

  // Acados
  drone_w_disturbances_solver_capsule *acados_ocp_capsule;
  ocp_nlp_config *nlp_config;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_in *nlp_in;
  ocp_nlp_out *nlp_out;
  ocp_nlp_solver *nlp_solver;
  void *nlp_opts;
  double *model_parameters;

  /**
   * @brief Initializes acados solver
   * @returns True if successful
   */
  bool initializeAcadosSolver();

  /**
   * @brief Sets the reference trajectory
   * @param reference Vector that contains the reference trajectory. If the
   * vector's size is smaller than the discretization steps, the last value is
   * repeated until all steps are filled
   */
  void setReference(const std::vector<setpoint> &reference);

  /**
   * @brief Sets the initial conditions for acados
   * @param state_init Initial state of the drone contained in a setpoint struct
   */
  void setInitialConditions(const setpoint &state_init);

  /**
   * @brief Updates the disturbances
   * @param fdis_x Disturbance on x axis
   * @param fdis_y Disturbance on y axis
   * @param fdis_z Disturbance on z axis
   */
  void updateDisturbances(const double &fdis_x, const double &fdis_y,
                          const double &fdis_z);

  /**
   * @brief Just for testing the acados solver
   * TODO: Remove when done testing
   */
  void testAcados();
};
}  // namespace px4_ctrl