#include "nmpc/px4_nmpc.h"

#include <mavros_msgs/AttitudeTarget.h>

namespace px4_ctrl {
Px4Nmpc::Px4Nmpc(ros::NodeHandle &nh) {
  // Setup Subscribers
  drone_state_sub =
      nh.subscribe("/drone_state", 1, &Px4Nmpc::droneStateCallback, this);
  setpoint_sub =
      nh.subscribe("/drone_setpoint", 1, &Px4Nmpc::setpointCallback, this);

  // Setup Publishers
  control_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);
}

Px4Nmpc::~Px4Nmpc() {}

void Px4Nmpc::droneStateCallback(const px4_control_msgs::DroneState &msg) {}
void Px4Nmpc::setpointCallback(const geometry_msgs::Pose &msg) {}
}  // namespace px4_ctrl