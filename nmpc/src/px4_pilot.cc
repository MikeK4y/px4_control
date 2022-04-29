#include "nmpc/px4_pilot.h"

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <tf2/LinearMath/Quaternion.h>

#include "tf/transform_datatypes.h"

namespace px4_ctrl {
PX4Pilot::PX4Pilot(ros::NodeHandle &nh, const double &rate) {
  // Load parameters
  loadParameters();

  // Initialize variables
  got_RC = false;
  controller_enabled = false;
  allow_offboard = false;
  drone_connected = false;
  drone_connected = false;

  // Initialize mutexes
  status_mutex.reset(new std::mutex);

  // Setup Subscribers
  mavros_status_sub =
      nh.subscribe("/mavros/state", 1, &PX4Pilot::mavrosStatusCallback, this);
  mavros_rc_sub =
      nh.subscribe("/mavros/rc/in", 1, &PX4Pilot::mavrosRCCallback, this);
  // controller_cmd_sub =
  //     nh.subscribe("/controller_cmd", 1, &PX4Pilot::controllerCallback,
  //     this);

  // Setup Publishers
  att_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);
  vel_control_pub = nh.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 1);

  // Setup Service Servers

  // Setup Service Clients
  mavros_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_arm_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  // Start command publisher thread
  cmd_publisher_worker_t = std::thread(&PX4Pilot::commandPublisher, this, rate);
}

// Callbacks
void PX4Pilot::mavrosStatusCallback(const mavros_msgs::State::ConstPtr &msg) {
  std::lock_guard<std::mutex> status_guard(*(status_mutex));
  current_status = *msg;
  drone_connected = msg->connected;
  is_offboard = msg->mode == "OFFBOARD";
}

void PX4Pilot::mavrosRCCallback(const mavros_msgs::RCIn::ConstPtr &msg) {
  if (!got_RC) {
    got_RC = true;
  }

  last_RC_time = msg->header.stamp;

  // Enable-Disable Controller
  if (msg->channels[controller_switch.channel] == controller_switch.high &&
      !controller_enabled) {
    // Enable controller
    controller_enabled = true;
  } else if (msg->channels[controller_switch.channel] ==
                 controller_switch.low &&
             controller_enabled) {
    // Disable controller
    controller_enabled = false;
  }

  // Allow switching to Offboard
  if (msg->channels[offboard_switch.channel] == offboard_switch.high &&
      !allow_offboard) {
    // Allow sending cmds
    allow_offboard = true;
  } else if (msg->channels[offboard_switch.channel] == offboard_switch.low &&
             allow_offboard) {
    // Block all offboard cmds
    allow_offboard = false;
  } else if (msg->channels[offboard_switch.channel] == offboard_switch.high &&
             allow_offboard && !is_offboard) {
    // If Offboard is requested but hasn't happened yet switch here???
  }
}

void PX4Pilot::controllerCallback(){

};

void PX4Pilot::loadParameters() {
  // Create private nodehandle to load parameters
  ros::NodeHandle nh_pvt("~");

  int int_parameter;
  if (nh_pvt.getParam("controller_enable_channel", int_parameter)) {
    controller_switch.channel = uint8_t(int_parameter);
  } else {
    ROS_WARN(
        "Could not retrieve controller enable channel. Setting to default");
    controller_switch.channel = uint8_t(7);
  }

  if (nh_pvt.getParam("controller_enable_low", int_parameter)) {
    controller_switch.low = uint8_t(int_parameter);
  } else {
    ROS_WARN(
        "Could not retrieve controller enable low. Setting to default");
    controller_switch.low = uint8_t(7);
  }

  if (nh_pvt.getParam("controller_enable_high", int_parameter)) {
    controller_switch.high = uint8_t(int_parameter);
  } else {
    ROS_WARN(
        "Could not retrieve controller enable high. Setting to default");
    controller_switch.high = uint8_t(7);
  }
}

void PX4Pilot::commandPublisher(const double &pub_rate) {
  ros::Rate rate(pub_rate);

  // Make sure vehicle is connected
  ROS_INFO("Connecting to vehicle");
  while (true) {
    if (drone_connected) break;
  }
  ROS_INFO("Vehicle is connected");

  // Setup command msgs
  // Attitude
  mavros_msgs::AttitudeTarget att_cmd;
  att_cmd.type_mask = att_cmd.IGNORE_ROLL_RATE | att_cmd.IGNORE_PITCH_RATE;

  // Velocity
  mavros_msgs::PositionTarget vel_cmd;
  vel_cmd.coordinate_frame = vel_cmd.FRAME_BODY_NED;
  vel_cmd.type_mask = vel_cmd.IGNORE_PX | vel_cmd.IGNORE_PY |
                      vel_cmd.IGNORE_PZ | vel_cmd.IGNORE_AFX |
                      vel_cmd.IGNORE_AFY | vel_cmd.IGNORE_AFZ |
                      vel_cmd.IGNORE_YAW;
  vel_cmd.velocity.x = 0.0;
  vel_cmd.velocity.y = 0.0;
  vel_cmd.velocity.z = 0.0;
  vel_cmd.yaw_rate = 0.0;

  while (ros::ok()) {
    if (drone_connected) {
      if (allow_offboard) {
        if (controller_enabled & is_offboard) {
          // Send controller commands
        } else {
          // Send zero velocity commands so that it can be switched to Offboard
          vel_cmd.header.stamp = ros::Time::now();
          vel_control_pub.publish(vel_cmd);
        }
      }
    } else {
      ROS_WARN("Lost Connection to Vehicle!!!");
    }
    rate.sleep();
  }
}

}  // namespace px4_ctrl