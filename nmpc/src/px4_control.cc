#include "nmpc/px4_control.h"

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <tf2/LinearMath/Quaternion.h>

namespace px4_ctrl {
PX4Control::PX4Control(ros::NodeHandle &nh, const double &rate) {
  // Setup Subscribers
  drone_state_sub =
      nh.subscribe("/drone_state", 1, &PX4Control::droneStateCallback, this);
  setpoint_sub =
      nh.subscribe("/drone_setpoint", 1, &PX4Control::setpointCallback, this);
  trajectory_sub = nh.subscribe("/drone_trajectory", 1,
                                &PX4Control::trajectoryCallback, this);
  mavros_status_sub =
      nh.subscribe("/mavros/state", 1, &PX4Control::mavrosStatusCallback, this);

  // Setup Publishers
  att_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);
  vel_control_pub = nh.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 1);

  // Setup Service Servers
  go_to_start_serv = nh.advertiseService(
      "/go_to_start", &PX4Control::goToStartServCallback, this);
  start_trajectory_serv = nh.advertiseService(
      "/start_trajectory", &PX4Control::startTrajectoryServCallback, this);
  enable_controller_serv = nh.advertiseService(
      "/enable_controller", &PX4Control::enableControllerServCallback, this);

  // Clear trajectory and weights
  current_reference_trajectory.clear();
  weights.clear();

  // Load parameters
  loadParameters();

  // Initialize acados NMPC
  nmpc_controller = new AcadosNMPC();
  if (nmpc_controller->initializeController(model_params) &&
      nmpc_controller->setWeighingMatrix(weights)) {
    ROS_INFO("NMPC Initialized\n");
  } else {
    ROS_ERROR("Failed to initialize Acados NMPC. Exiting\n");
    exit(1);
  }
  enable_controller = false;
  trajectory_loaded = false;
  has_drone_state = false;

  // Initialize mutexes
  status_mutex.reset(new std::mutex);
  drone_state_mutex.reset(new std::mutex);

  // Start command publisher
  publisher_worker_t = std::thread(&PX4Control::commandPublisher, this, rate);
}

PX4Control::~PX4Control() { delete nmpc_controller; };

void PX4Control::loadParameters() {
  // Create private nodehandle to load parameters
  ros::NodeHandle nh_pvt("~");

  nh_pvt.param("t_pitch", model_params.t_pitch, 1.0);
  nh_pvt.param("k_pitch", model_params.k_pitch, 1.0);

  nh_pvt.param("t_roll", model_params.t_roll, 1.0);
  nh_pvt.param("k_roll", model_params.k_roll, 1.0);

  std::vector<double> damping_coef;
  nh_pvt.getParam("damping_coef", damping_coef);
  model_params.damp_x = damping_coef[0];
  model_params.damp_y = damping_coef[1];
  model_params.damp_z = damping_coef[2];

  nh_pvt.param("k_thrust", model_params.k_thrust, 1.0);

  nh_pvt.param("gravity", model_params.gravity, -9.8066);

  std::vector<double> pos_w, vel_w, att_w;
  double yaw_rate_cmd_w, pitch_cmd_w, roll_cmd_w, thrust_cmd_w;

  nh_pvt.getParam("pos_w", pos_w);
  nh_pvt.getParam("vel_w", vel_w);
  nh_pvt.getParam("att_w", att_w);
  nh_pvt.getParam("yaw_rate_cmd_w", yaw_rate_cmd_w);
  nh_pvt.getParam("pitch_cmd_w", pitch_cmd_w);
  nh_pvt.getParam("roll_cmd_w", roll_cmd_w);
  nh_pvt.getParam("thrust_cmd_w", thrust_cmd_w);

  // Cost function weights
  weights.push_back(pos_w[0]);
  weights.push_back(pos_w[1]);
  weights.push_back(pos_w[2]);
  weights.push_back(vel_w[0]);
  weights.push_back(vel_w[1]);
  weights.push_back(vel_w[2]);
  weights.push_back(att_w[0]);
  weights.push_back(att_w[1]);
  weights.push_back(att_w[2]);
  weights.push_back(yaw_rate_cmd_w);
  weights.push_back(pitch_cmd_w);
  weights.push_back(roll_cmd_w);
  weights.push_back(thrust_cmd_w);
}

// Callbacks
void PX4Control::mavrosStatusCallback(const mavros_msgs::State::ConstPtr &msg) {
  std::lock_guard<std::mutex> status_guard(*(status_mutex));
  current_status = *msg;
}

void PX4Control::droneStateCallback(const px4_control_msgs::DroneState &msg) {
  std::lock_guard<std::mutex> state_guard(*(drone_state_mutex));
  drone_state.pos_x = msg.pose.position.x;
  drone_state.pos_y = msg.pose.position.y;
  drone_state.pos_z = msg.pose.position.z;
  drone_state.vel_x = msg.velocity.x;
  drone_state.vel_y = msg.velocity.y;
  drone_state.vel_z = msg.velocity.z;
  drone_state.q_roll = msg.orientation_euler.x;
  drone_state.q_pitch = msg.orientation_euler.y;
  drone_state.q_yaw = msg.orientation_euler.z;

  disturbances.clear();
  disturbances.push_back(msg.disturbances.x);
  disturbances.push_back(msg.disturbances.y);
  disturbances.push_back(msg.disturbances.z);

  has_drone_state = !has_drone_state ? true : has_drone_state;
}

void PX4Control::setpointCallback(const px4_control_msgs::Setpoint &msg) {
  enable_controller = false;
  // Clear old trajectory
  current_reference_trajectory.clear();

  trajectory_setpoint setpoint;
  setpoint.pos_x = msg.position.x;
  setpoint.pos_y = msg.position.y;
  setpoint.pos_z = msg.position.z;
  setpoint.vel_x = msg.velocity.x;
  setpoint.vel_y = msg.velocity.y;
  setpoint.vel_z = msg.velocity.z;
  setpoint.q_roll = msg.orientation.x;
  setpoint.q_pitch = msg.orientation.y;
  setpoint.q_yaw = msg.orientation.z;

  current_reference_trajectory.push_back(setpoint);
}

void PX4Control::trajectoryCallback(const px4_control_msgs::Trajectory &msg) {
  enable_controller = false;
  // Clear old trajectory
  current_reference_trajectory.clear();

  trajectory_setpoint setpoint;
  for (size_t i = 0; i < msg.trajectory.size(); i++) {
    setpoint.pos_x = msg.trajectory[i].position.x;
    setpoint.pos_y = msg.trajectory[i].position.y;
    setpoint.pos_z = msg.trajectory[i].position.z;
    setpoint.vel_x = msg.trajectory[i].velocity.x;
    setpoint.vel_y = msg.trajectory[i].velocity.y;
    setpoint.vel_z = msg.trajectory[i].velocity.z;
    setpoint.q_roll = msg.trajectory[i].orientation.x;
    setpoint.q_pitch = msg.trajectory[i].orientation.y;
    setpoint.q_yaw = msg.trajectory[i].orientation.z;

    current_reference_trajectory.push_back(setpoint);
  }
}

bool PX4Control::goToStartServCallback(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res) {
  if (current_reference_trajectory.size() > 0) {
    enable_controller = false;
    std::vector<trajectory_setpoint> reference_trajectory;
    reference_trajectory.push_back(current_reference_trajectory[0]);
    nmpc_controller->setTrajectory(reference_trajectory);
    trajectory_loaded = true;
    res.success = true;
    ROS_INFO(
        "Setpoint was set to the start of the trajectory. Enable controller to "
        "get there");
    return true;
  } else {
    ROS_WARN("No setpoint or trajectory is loaded. Load one and try again");
    res.success = false;
  }
  return false;
}

bool PX4Control::startTrajectoryServCallback(std_srvs::Trigger::Request &req,
                                             std_srvs::Trigger::Response &res) {
  if (current_reference_trajectory.size() > 0) {
    enable_controller = false;
    nmpc_controller->setTrajectory(current_reference_trajectory);
    trajectory_loaded = true;
    res.success = true;
    ROS_INFO("The full trajectory was loaded. Enable controller to get there");
    return true;
  } else {
    ROS_WARN("No setpoint or trajectory is loaded. Load one and try again");
    res.success = false;
  }
  return false;
}

bool PX4Control::enableControllerServCallback(
    std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  if (trajectory_loaded & req.data) {
    ROS_WARN("Controller enabled");
    enable_controller = true;
    res.success = true;
    return true;
  } else if (!trajectory_loaded & req.data) {
    ROS_WARN("No setpoint or trajectory is loaded. Load one and try again");
    res.success = false;
    return false;
  } else if (!req.data) {
    ROS_WARN("Controller disabled");
    enable_controller = false;
    res.success = true;
    return true;
  }
  ROS_ERROR("Unexpected behavior");
  res.success = false;
  return false;
}

void PX4Control::commandPublisher(const double &pub_rate) {
  ros::Rate rate(pub_rate);

  // Make sure vehicle is connected
  ROS_INFO("Connecting to vehicle");
  while (true) {
    bool status_ok = false;
    {  // Lock status mutex
      std::lock_guard<std::mutex> status_guard(*(status_mutex));
      status_ok = current_status.connected;
    }
    if (status_ok) break;
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
    bool status_ok;
    bool is_offboard;
    {  // Lock status mutex
      std::lock_guard<std::mutex> status_guard(*(status_mutex));
      is_offboard = current_status.mode == "OFFBOARD";
      status_ok = current_status.connected;
    }

    if (status_ok) {
      if (enable_controller & has_drone_state & is_offboard) {
        // Update current state
        double current_yaw;
        {  // Lock state mutex
          std::lock_guard<std::mutex> state_guard(*(drone_state_mutex));
          current_yaw = drone_state.q_yaw;
          nmpc_controller->setCurrentState(drone_state, disturbances);
        }

        std::vector<double> ctrl;
        if (nmpc_controller->getCommands(ctrl)) {
          tf2::Quaternion q;
          q.setRPY(ctrl[2], ctrl[1], current_yaw);
          q.normalize();

          att_cmd.header.stamp = ros::Time::now();
          att_cmd.orientation.x = q[0];
          att_cmd.orientation.y = q[1];
          att_cmd.orientation.z = q[2];
          att_cmd.orientation.w = q[3];

          att_cmd.body_rate.z = ctrl[0];

          double thrust = ctrl[3] < 0.1 ? 0.1 : ctrl[3];
          att_cmd.thrust = thrust;
          att_control_pub.publish(att_cmd);
        } else {
          ROS_ERROR("NMPC failed to return command. Hovering");
          enable_controller = false;
          vel_cmd.header.stamp = ros::Time::now();
          vel_control_pub.publish(vel_cmd);
        }

      } else {
        // If tracking is not requested just try to hover in place
        vel_cmd.header.stamp = ros::Time::now();
        vel_control_pub.publish(vel_cmd);
      }
    }
    rate.sleep();
  }
}

}  // namespace px4_ctrl