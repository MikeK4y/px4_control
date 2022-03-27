#pragma once

// Acados NMPC
#include "nmpc/acados_nmpc.h"

// Eigen
#include <eigen3/Eigen/Dense>

// ROS
#include "ros/ros.h"

// ROS messages
#include <mavros_msgs/State.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "px4_control_msgs/DroneStateMarker.h"
#include "px4_control_msgs/Setpoint.h"
#include "px4_control_msgs/Trajectory.h"

namespace px4_ctrl {
class MasterControl {
 public:
  MasterControl(ros::NodeHandle &nh);
  ~MasterControl();

 private:
  // ROS Subscribers
  ros::Subscriber drone_state_sub;

  // ROS Publishers
  ros::Publisher trajectory_pub;

  // ROS Services
  ros::ServiceClient start_trajectory_serv;
  ros::ServiceClient enable_controller_serv;

  // Callbacks
  void droneStateCallback(const px4_control_msgs::DroneStateMarker &msg);

  bool triggerTrajectoryTracking();
  bool enableController(bool enable);

  /**
   * @brief Loads the node parameters
   */
  void loadParameters();

  /**
   * @brief Takes a px4_ctrl::trajectory_setpoint vector and returns a
   * px4_control_msgs trajectory
   * @param traj_vector The px4_ctrl::trajectory_setpoint vector
   * @returns The px4_control_msgs trajectory
   */
  px4_control_msgs::Trajectory setpointVectorToTrajMsg(
      const std::vector<trajectory_setpoint> trajectory);

  // NMPC for trajectory generation
  AcadosNMPC *traj_generation;

  trajectory_setpoint goal_world, goal_marker;

  bool traj_world, traj_marker;

  Eigen::Vector3d marker_pos_set, setpoint_marker;

  model_parameters model_params;
  std::vector<double> disturbances, weights;
};
}  // namespace px4_ctrl