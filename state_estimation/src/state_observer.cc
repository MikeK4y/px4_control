#include "state_estimation/state_observer.h"

state_observer::state_observer(ros::NodeHandle &nh) {
  // Setup Subscribers
  odom_sub = nh.subscribe("/mavros/local_position/odom", 1,
                          &state_observer::odomCallback, this);
  ctrl_sub = nh.subscribe("/mavros/setpoint_raw/attitude", 1,
                          &state_observer::ctrlCallback, this);

  // Setup Publisher
  state_pub = nh.advertise<state_estimation::DroneState>("/drone_state", 1);

  // Initialize Parameters
  is_initialized = false;
}

state_observer::~state_observer() {}

// Odometry Callback
void state_observer::odomCallback(const nav_msgs::Odometry &msg) {}

// Control Callback
void state_observer::ctrlCallback(const mavros_msgs::AttitudeTarget &msg) {}