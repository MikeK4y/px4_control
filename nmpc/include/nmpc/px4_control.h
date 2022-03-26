#pragma once

#include <mutex>
#include <thread>

// Acados NMPC
#include "nmpc/acados_nmpc.h"

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
class PX4Control {
 public:
  PX4Control(ros::NodeHandle &nh, const double &rate);
  ~PX4Control();

  std::thread publisher_worker_t;

 private:
  // ROS Subscribers
  ros::Subscriber mavros_status_sub;
  ros::Subscriber drone_state_sub;
  ros::Subscriber setpoint_sub;
  ros::Subscriber trajectory_sub;

  // ROS Publishers
  ros::Publisher att_control_pub;
  ros::Publisher vel_control_pub;

  // ROS Services
  ros::ServiceServer go_to_start_serv;
  ros::ServiceServer start_trajectory_serv;
  ros::ServiceServer enable_controller_serv;

  // Callbacks
  void mavrosStatusCallback(const mavros_msgs::State::ConstPtr &msg);
  void droneStateCallback(const px4_control_msgs::DroneStateMarker &msg);
  void setpointCallback(const px4_control_msgs::Setpoint &msg);
  void trajectoryCallback(const px4_control_msgs::Trajectory &msg);

  bool goToStartServCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res);
  bool startTrajectoryServCallback(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res);
  bool enableControllerServCallback(std_srvs::SetBool::Request &req,
                                    std_srvs::SetBool::Response &res);

  /**
   * @brief If tracking is enabled, it publishes the UAS commands. Should run as
   * long as the node is alive
   * @param pub_rate Publishing rate
   */
  void commandPublisher(const double &pub_rate);

  /**
   * @brief Loads the node parameters
   */
  void loadParameters();

  // PX4
  mavros_msgs::State current_status;

  // Controller
  AcadosNMPC *nmpc_controller;
  bool enable_controller;
  bool trajectory_loaded;
  bool has_drone_state;

  model_parameters model_params;
  trajectory_setpoint drone_state;
  std::vector<double> disturbances, weights;
  std::vector<trajectory_setpoint> current_reference_trajectory;

  std::unique_ptr<std::mutex> status_mutex;
  std::unique_ptr<std::mutex> drone_state_mutex;
};
}  // namespace px4_ctrl