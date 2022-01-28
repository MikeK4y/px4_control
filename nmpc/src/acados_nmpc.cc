#include "nmpc/acados_nmpc.h"

#include <iostream>

namespace px4_ctrl {
AcadosNMPC::AcadosNMPC() {
#ifndef TRACK_TIME
  total_ocp_time = 0.0;
  min_ocp_time = 1.0e9;
  max_ocp_time = 0.0;
  total_ocp_calls = 0;
#endif
}

AcadosNMPC::~AcadosNMPC() {
#ifndef TRACK_TIME
  std::cout << "Average ocp solving time: "
            << 1000 * (total_ocp_time / total_ocp_calls) << "ms\n";
  std::cout << "Maximum ocp solving time: " << 1000 * max_ocp_time << "ms\n";
  std::cout << "Minimum ocp solving time: " << 1000 * min_ocp_time << "ms\n";
#endif
  // Free Acados Solver
  int status = drone_w_disturbances_acados_free(acados_ocp_capsule);
  if (status) {
    std::cerr << "drone_w_disturbances_acados_free() returned status: "
              << status << "\n";
  }
  // Free Acados Solver Capsule
  status = drone_w_disturbances_acados_free_capsule(acados_ocp_capsule);
  if (status) {
    std::cerr << "drone_w_disturbances_acados_free_capsule() returned status: "
              << status << "\n";
  }
}

bool AcadosNMPC::initializeController(const model_parameters &model_params) {
  // Initiallize Acados solver with default shooting intervals
  acados_ocp_capsule = drone_w_disturbances_acados_create_capsule();
  int status = drone_w_disturbances_acados_create(acados_ocp_capsule);

  if (status) {
    std::cerr << "drone_w_disturbances_acados_create() returned status:"
              << status << "\n";
    return false;
  }

  // Initialize Acados variables
  nlp_config = drone_w_disturbances_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = drone_w_disturbances_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = drone_w_disturbances_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = drone_w_disturbances_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = drone_w_disturbances_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = drone_w_disturbances_acados_get_nlp_opts(acados_ocp_capsule);

  // Initial conditions
  int idxbx0[DRONE_W_DISTURBANCES_NBX0];
  idxbx0[0] = 0;
  idxbx0[1] = 1;
  idxbx0[2] = 2;
  idxbx0[3] = 3;
  idxbx0[4] = 4;
  idxbx0[5] = 5;
  idxbx0[6] = 6;
  idxbx0[7] = 7;
  idxbx0[8] = 8;
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx",
                                idxbx0);

  // Set parameters
  acados_model_parameters = new double[DRONE_W_DISTURBANCES_NP];
  acados_model_parameters[0] = model_params.t_roll;     // Roll time constant
  acados_model_parameters[1] = model_params.k_roll;     // Roll gain
  acados_model_parameters[2] = model_params.t_pitch;    // Pitch time constant
  acados_model_parameters[3] = model_params.k_pitch;    // Pitch gain
  acados_model_parameters[4] = model_params.damp_x;     // Damping x
  acados_model_parameters[5] = model_params.damp_y;     // Damping y
  acados_model_parameters[6] = model_params.damp_z;     // Damping z
  acados_model_parameters[7] = 0.0;                     // Disturbance force x
  acados_model_parameters[8] = 0.0;                     // Disturbance force y
  acados_model_parameters[9] = 0.0;                     // Disturbance force z
  acados_model_parameters[10] = model_params.k_thrust;  // Thrust coefficients
  acados_model_parameters[11] = model_params.gravity;   // Gravity

  for (int i = 0; i <= DRONE_W_DISTURBANCES_N; i++)
    drone_w_disturbances_acados_update_params(acados_ocp_capsule, i,
                                              acados_model_parameters,
                                              DRONE_W_DISTURBANCES_NP);

  return true;
}

void AcadosNMPC::setTrajectory(
    const std::vector<trajectory_setpoint> &trajectory) {
  current_reference_trajectory.clear();
  current_reference_trajectory = trajectory;
  trajectory_index = 0;
  trajectory_length = current_reference_trajectory.size();
  // std::cout << "A " << trajectory_length << " point trajectory was loaded\n";
}

void AcadosNMPC::setCurrentState(const trajectory_setpoint &state,
                                 const std::vector<double> &disturbances) {
  // Check if drone is touching the ground
  bool on_ground = (abs(state.pos_z) < 0.1) &
                   (abs(disturbances[2] + acados_model_parameters[11]) < 0.1);

  updateInitialConditions(state);
  if (on_ground) {
    updateDisturbances(disturbances[0], disturbances[1], 0.0);
    std::cout << "It looks like the drone is on the ground\n";
  } else
    updateDisturbances(disturbances[0], disturbances[1], disturbances[2]);
}

bool AcadosNMPC::getCommands(std::vector<double> &ctrl) {
  // Update reference and reference index
  updateReference();
  trajectory_index = trajectory_index + 1 < trajectory_length
                         ? trajectory_index + 1
                         : trajectory_index;

  // Solve OCP
  int status = drone_w_disturbances_acados_solve(acados_ocp_capsule);
#ifndef TRACK_TIME
  double elapsed_time;
  ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
  max_ocp_time = max_ocp_time > elapsed_time ? max_ocp_time : elapsed_time;
  min_ocp_time = min_ocp_time < elapsed_time ? min_ocp_time : elapsed_time;
  total_ocp_time += elapsed_time;
  total_ocp_calls++;
#endif

  // Get first control input
  if (status == ACADOS_SUCCESS) {
    double u_0[DRONE_W_DISTURBANCES_NU];
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_0);
    ctrl.clear();
    ctrl.push_back(u_0[0]);
    ctrl.push_back(u_0[1]);
    ctrl.push_back(u_0[2]);
    ctrl.push_back(u_0[3]);
    return true;
  } else {
    std::cerr << "drone_w_disturbances_acados_solve() failed with status: "
              << status << "\n";
  }
  return false;
}

void AcadosNMPC::updateReference() {
  int ref_index = trajectory_index;
  int acados_index = 0;

  // Trajectory
  while (acados_index < DRONE_W_DISTURBANCES_N) {
    double y_ref[DRONE_W_DISTURBANCES_NY];
    y_ref[0] = current_reference_trajectory[ref_index].pos_x;    // Position x
    y_ref[1] = current_reference_trajectory[ref_index].pos_y;    // Position y
    y_ref[2] = current_reference_trajectory[ref_index].pos_z;    // Position z
    y_ref[3] = current_reference_trajectory[ref_index].vel_x;    // Velocity x
    y_ref[4] = current_reference_trajectory[ref_index].vel_y;    // Velocity y
    y_ref[5] = current_reference_trajectory[ref_index].vel_z;    // Velocity z
    y_ref[6] = current_reference_trajectory[ref_index].q_roll;   // Roll
    y_ref[7] = current_reference_trajectory[ref_index].q_pitch;  // Pitch
    y_ref[8] = current_reference_trajectory[ref_index].q_yaw;    // Yaw
    y_ref[9] = 0.0;                                              // Yaw rate
    y_ref[10] = 0.0;                                             // Pitch cmd
    y_ref[11] = 0.0;                                             // Roll cmd
    y_ref[12] = 0.0;                                             // Thrust

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, acados_index, "yref",
                           y_ref);

    acados_index++;
    ref_index = ref_index + 1 < trajectory_length ? ref_index + 1 : ref_index;
  }

  // End point
  double y_ref_e[DRONE_W_DISTURBANCES_NYN];
  y_ref_e[0] = current_reference_trajectory[ref_index].pos_x;    // Position x
  y_ref_e[1] = current_reference_trajectory[ref_index].pos_y;    // Position y
  y_ref_e[2] = current_reference_trajectory[ref_index].pos_z;    // Position z
  y_ref_e[3] = current_reference_trajectory[ref_index].vel_x;    // Velocity x
  y_ref_e[4] = current_reference_trajectory[ref_index].vel_y;    // Velocity y
  y_ref_e[5] = current_reference_trajectory[ref_index].vel_z;    // Velocity z
  y_ref_e[6] = current_reference_trajectory[ref_index].q_roll;   // Roll
  y_ref_e[7] = current_reference_trajectory[ref_index].q_pitch;  // Pitch
  y_ref_e[8] = current_reference_trajectory[ref_index].q_yaw;    // Yaw

  ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, DRONE_W_DISTURBANCES_N,
                         "yref", y_ref_e);

  // std::cout << "Reference set to: " << y_ref_e[0] << ", " << y_ref_e[1] << ", "
  //           << y_ref_e[2] << ", " << y_ref_e[3] << ", " << y_ref_e[4] << ", "
  //           << y_ref_e[5] << ", " << y_ref_e[6] << ", " << y_ref_e[7] << ", "
  //           << y_ref_e[8] << "\n";
}

void AcadosNMPC::updateInitialConditions(
    const trajectory_setpoint &state_init) {
  double x_init[DRONE_W_DISTURBANCES_NBX0];
  x_init[0] = state_init.pos_x;
  x_init[1] = state_init.pos_y;
  x_init[2] = state_init.pos_z;
  x_init[3] = state_init.vel_x;
  x_init[4] = state_init.vel_y;
  x_init[5] = state_init.vel_z;
  x_init[6] = state_init.q_roll;
  x_init[7] = state_init.q_pitch;
  x_init[8] = state_init.q_yaw;

  // Set the initial conditions by setting the lower and upper bounds to the
  // initial state values
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_init);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_init);

  // std::cout << "Initial state set to: " << x_init[0] << ", " << x_init[1]
  //           << ", " << x_init[2] << ", " << x_init[3] << ", " << x_init[4]
  //           << ", " << x_init[5] << ", " << x_init[6] << ", " << x_init[7]
  //           << ", " << x_init[8] << "\n";
}

void AcadosNMPC::updateDisturbances(const double &fdis_x, const double &fdis_y,
                                    const double &fdis_z) {
  acados_model_parameters[7] = fdis_x;
  acados_model_parameters[8] = fdis_y;
  acados_model_parameters[9] = fdis_z;

  for (int i = 0; i <= DRONE_W_DISTURBANCES_N; i++)
    drone_w_disturbances_acados_update_params(acados_ocp_capsule, i,
                                              acados_model_parameters,
                                              DRONE_W_DISTURBANCES_NP);

  // std::cout << "Parameters set to: " << acados_model_parameters[0] << ", "
  //           << acados_model_parameters[1] << ", " << acados_model_parameters[2]
  //           << ", " << acados_model_parameters[3] << ", "
  //           << acados_model_parameters[4] << ", " << acados_model_parameters[5]
  //           << ", " << acados_model_parameters[6] << ", "
  //           << acados_model_parameters[7] << ", " << acados_model_parameters[8]
  //           << ", " << acados_model_parameters[9] << ", "
  //           << acados_model_parameters[10] << ", "
  //           << acados_model_parameters[11] << "\n";
}

}  // namespace px4_ctrl