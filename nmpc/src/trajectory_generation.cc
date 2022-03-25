#include "nmpc/trajectory_generation.h"

namespace px4_ctrl {
TrajectoryGeneration::TrajectoryGeneration(ros::NodeHandle &nh) {
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
  generateTrajectory();
}

TrajectoryGeneration::~TrajectoryGeneration() { delete nmpc_controller; };

void TrajectoryGeneration::loadParameters() {
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

void TrajectoryGeneration::generateTrajectory() {
  trajectory_setpoint start_point, goal_point;
  std::vector<trajectory_setpoint> trajectory;

  start_point.pos_x = 0.0;
  start_point.pos_y = 0.0;
  start_point.pos_z = 0.0;
  start_point.vel_x = 0.0;
  start_point.vel_y = 0.0;
  start_point.vel_z = 0.0;
  start_point.q_roll = 0.0;
  start_point.q_pitch = 0.0;
  start_point.q_yaw = 0.0;

  goal_point.pos_x = 1.0;
  goal_point.pos_y = 1.0;
  goal_point.pos_z = 7.0;
  goal_point.vel_x = 0.0;
  goal_point.vel_y = 0.0;
  goal_point.vel_z = 0.0;
  goal_point.q_roll = 0.0;
  goal_point.q_pitch = 0.0;
  goal_point.q_yaw = 1.0;

  nmpc_controller->getTrajectory(trajectory, start_point, goal_point);

  std::cout << "Trajectory lenght: " << trajectory.size() << "\n\n";

  for (size_t i = 0; i < trajectory.size(); i++) {
    std::cout << "Point: " << i << "\n";
    std::cout << trajectory[i].pos_x << ", " << trajectory[i].pos_y << ", "
              << trajectory[i].pos_z << "\n";
    std::cout << trajectory[i].vel_x << ", " << trajectory[i].vel_y << ", "
              << trajectory[i].vel_z << "\n";
    std::cout << trajectory[i].q_roll << ", " << trajectory[i].q_pitch << ", "
              << trajectory[i].q_yaw << "\n";
  }
}
}  // namespace px4_ctrl

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_gen_node");
  ros::NodeHandle nh;

  px4_ctrl::TrajectoryGeneration tg(nh);

  return 0;
}