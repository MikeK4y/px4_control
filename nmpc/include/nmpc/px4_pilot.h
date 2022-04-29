#pragma once

#include <mutex>
#include <thread>

// ROS
#include "ros/ros.h"

// ROS messages
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>

namespace px4_ctrl {

struct rc_switch {
  uint8_t channel;
  uint16_t high;
  uint16_t low;
};

class PX4Pilot {
 public:
  PX4Pilot(ros::NodeHandle &nh, const double &rate);
  ~PX4Pilot(){};

  std::thread cmd_publisher_worker_t;

 private:
  // ROS Subscribers
  ros::Subscriber mavros_status_sub;
  ros::Subscriber mavros_rc_sub;
  ros::Subscriber controller_cmd_sub;

  // ROS Publishers
  ros::Publisher att_control_pub;
  ros::Publisher vel_control_pub;

  // ROS Service Clients
  ros::ServiceClient mavros_mode_client;
  ros::ServiceClient mavros_arm_client;

  // Callbacks
  void mavrosStatusCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavrosRCCallback(const mavros_msgs::RCIn::ConstPtr &msg);
  void controllerCallback();

  /**
   * @brief Loads the node parameters
   */
  void loadParameters();

  /**
   * @brief If tracking is enabled, it publishes the UAS commands. Should run as
   * long as the node is alive
   * @param pub_rate Publishing rate
   */
  void commandPublisher(const double &pub_rate);

  // PX4 Status
  mavros_msgs::State current_status;
  bool drone_connected;
  bool is_offboard;

  // PX4 RC
  bool got_RC;
  ros::Time last_RC_time;
  rc_switch controller_switch;
  rc_switch offboard_switch;

  // PX4 Controller
  bool controller_enabled;
  bool allow_offboard;

  std::unique_ptr<std::mutex> status_mutex;
  std::unique_ptr<std::mutex> rc_mutex;
};
}  // namespace px4_ctrl