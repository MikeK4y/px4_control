#include "nmpc/px4_nmpc.h"

#include <mavros_msgs/AttitudeTarget.h>

namespace px4_ctrl {
Px4Nmpc::Px4Nmpc(ros::NodeHandle &nh) {
  // Setup Subscribers
  drone_state_sub =
      nh.subscribe("/drone_state", 1, &Px4Nmpc::droneStateCallback, this);
  setpoint_sub =
      nh.subscribe("/drone_setpoint", 1, &Px4Nmpc::setpointCallback, this);
  trajectory_sub =
      nh.subscribe("/drone_trajectory", 1, &Px4Nmpc::trajectoryCallback, this);

  // Setup Publishers
  control_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);

  // Load parameters
  loadParameters();

  // Initialize acados
  if (!initializeAcadosSolver()) {
    ROS_ERROR("Failed to initialize Acados solver. Exiting\n");
    exit(1);
  }
  testAcados();
}

Px4Nmpc::~Px4Nmpc() {
  // Free Acados Solver
  int status = drone_w_disturbances_acados_free(acados_ocp_capsule);
  if (status) {
    ROS_WARN("drone_w_disturbances_acados_free() returned status %d. \n",
             status);
  }
  // Free Acados Solver Capsule
  status = drone_w_disturbances_acados_free_capsule(acados_ocp_capsule);
  if (status) {
    ROS_WARN(
        "drone_w_disturbances_acados_free_capsule() returned status %d. \n",
        status);
  }
}

void Px4Nmpc::loadParameters() {
  // Create private nodehandle to load parameters
  ros::NodeHandle nh_pvt("~");

  nh_pvt.param("t_pitch", t_pitch, 1.0);
  nh_pvt.param("k_pitch", k_pitch, 1.0);

  nh_pvt.param("t_roll", t_roll, 1.0);
  nh_pvt.param("k_roll", k_roll, 1.0);

  nh_pvt.param("damping_x", damp_x, -1.0);
  nh_pvt.param("damping_y", damp_y, -1.0);
  nh_pvt.param("damping_z", damp_z, -1.0);

  nh_pvt.param("k_thrust", k_thrust, 1.0);

  nh_pvt.param("gravity", gravity, -9.8066);
}

bool Px4Nmpc::initializeAcadosSolver() {
  // Initiallize Acados solver with default shooting intervals
  acados_ocp_capsule = drone_w_disturbances_acados_create_capsule();
  int status = drone_w_disturbances_acados_create(acados_ocp_capsule);

  if (status) {
    ROS_WARN("drone_w_disturbances_acados_create() returned status %d\n",
             status);
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
  model_parameters = new double[DRONE_W_DISTURBANCES_NP];
  model_parameters[0] = t_roll;     // Roll time constant
  model_parameters[1] = k_roll;     // Roll gain
  model_parameters[2] = t_pitch;    // Pitch time constant
  model_parameters[3] = k_pitch;    // Pitch gain
  model_parameters[4] = damp_x;     // Damping x
  model_parameters[5] = damp_y;     // Damping y
  model_parameters[6] = damp_z;     // Damping z
  model_parameters[7] = 0.0;        // Disturbance force x
  model_parameters[8] = 0.0;        // Disturbance force y
  model_parameters[9] = 0.0;        // Disturbance force z
  model_parameters[10] = k_thrust;  // Thrust coefficients
  model_parameters[11] = gravity;   // Gravity

  for (int i = 0; i < DRONE_W_DISTURBANCES_N; i++)
    drone_w_disturbances_acados_update_params(
        acados_ocp_capsule, i, model_parameters, DRONE_W_DISTURBANCES_NP);

  return true;
}

void Px4Nmpc::testAcados() {
  // Initial conditions
  // double x_init[DRONE_W_DISTURBANCES_NX];
  // x_init[0] = 0.0;  // Position x
  // x_init[1] = 0.0;  // Position y
  // x_init[2] = 1.0;  // Position z
  // x_init[3] = 0.0;  // Velocity x
  // x_init[4] = 0.0;  // Velocity y
  // x_init[5] = 0.0;  // Velocity z
  // x_init[6] = 0.0;  // Roll
  // x_init[7] = 0.0;  // Pitch
  // x_init[8] = 0.0;  // Yaw

  // double W[DRONE_W_DISTURBANCES_NY * DRONE_W_DISTURBANCES_NY];
  // W[0 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e2;    // Position x
  // W[1 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e2;    // Position y
  // W[2 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e2;    // Position z
  // W[3 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-3;    // Velocity x
  // W[4 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-3;    // Velocity y
  // W[5 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-3;    // Velocity z
  // W[6 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-6;   // Roll
  // W[7 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-6;   // Pitch
  // W[8 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-6;   // Yaw
  // W[9 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-9;   // Yaw rate
  // W[10 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-9;  // Pitch cmd
  // W[11 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-9;  // Roll cmd
  // W[12 * (DRONE_W_DISTURBANCES_NY + 1)] = 1.0e-9;  // Thrust

  // Setpoint
  double y_ref[DRONE_W_DISTURBANCES_NY];
  y_ref[0] = 0.1;   // Position x
  y_ref[1] = 0.0;   // Position y
  y_ref[2] = 1.0;   // Position z
  y_ref[3] = 0.0;   // Velocity x
  y_ref[4] = 0.0;   // Velocity y
  y_ref[5] = 0.0;   // Velocity z
  y_ref[6] = 0.0;   // Roll
  y_ref[7] = 0.0;   // Pitch
  y_ref[8] = 0.0;   // Yaw
  y_ref[9] = 0.0;   // Yaw rate
  y_ref[10] = 0.0;  // Pitch cmd
  y_ref[11] = 0.0;  // Roll cmd
  y_ref[12] = 0.0;  // Thrust

  double y_ref_e[DRONE_W_DISTURBANCES_NYN];
  y_ref_e[0] = 0.1;  // Position x
  y_ref_e[1] = 0.0;  // Position y
  y_ref_e[2] = 1.0;  // Position z
  y_ref_e[3] = 0.0;  // Velocity x
  y_ref_e[4] = 0.0;  // Velocity y
  y_ref_e[5] = 0.0;  // Velocity z
  y_ref_e[6] = 0.0;  // Roll
  y_ref_e[7] = 0.0;  // Pitch
  y_ref_e[8] = 0.0;  // Yaw

  for (int i = 0; i < DRONE_W_DISTURBANCES_N; i++) {
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", y_ref);
    // ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x_desired);
    // ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", u_init);
  }
  ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, DRONE_W_DISTURBANCES_N,
                         "yref", y_ref_e);

  int status = drone_w_disturbances_acados_solve(acados_ocp_capsule);
  double elapsed_time;
  double min_time = 1.0e12;
  ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
  min_time = MIN(elapsed_time, min_time);

  // Get solution
  double xtraj[DRONE_W_DISTURBANCES_NX * (DRONE_W_DISTURBANCES_N + 1)];
  double utraj[DRONE_W_DISTURBANCES_NU * DRONE_W_DISTURBANCES_N];
  for (int ii = 0; ii <= nlp_dims->N; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x",
                    &xtraj[ii * DRONE_W_DISTURBANCES_NX]);
  for (int ii = 0; ii < nlp_dims->N; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u",
                    &utraj[ii * DRONE_W_DISTURBANCES_NU]);

  if (status == ACADOS_SUCCESS) {
    printf("Time to solve ocp: %f [ms]\n", min_time * 1000);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat(DRONE_W_DISTURBANCES_NX, DRONE_W_DISTURBANCES_N + 1,
                         xtraj, DRONE_W_DISTURBANCES_NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat(DRONE_W_DISTURBANCES_NU, DRONE_W_DISTURBANCES_N, utraj,
                         DRONE_W_DISTURBANCES_NU);
  } else {
    printf("drone_w_disturbances_acados_solve() failed with status %d.\n",
           status);
  }
}

void Px4Nmpc::droneStateCallback(const px4_control_msgs::DroneState &msg) {}
void Px4Nmpc::setpointCallback(const px4_control_msgs::Setpoint &msg) {}
void Px4Nmpc::trajectoryCallback(const px4_control_msgs::Trajectory &msg) {}

void Px4Nmpc::setReference(const std::vector<setpoint> &reference) {

  int ref_size = reference.size();


    // Setpoint
  double y_ref[DRONE_W_DISTURBANCES_NY];
  y_ref[0] = 0.1;   // Position x
  y_ref[1] = 0.0;   // Position y
  y_ref[2] = 1.0;   // Position z
  y_ref[3] = 0.0;   // Velocity x
  y_ref[4] = 0.0;   // Velocity y
  y_ref[5] = 0.0;   // Velocity z
  y_ref[6] = 0.0;   // Roll
  y_ref[7] = 0.0;   // Pitch
  y_ref[8] = 0.0;   // Yaw
  y_ref[9] = 0.0;   // Yaw rate
  y_ref[10] = 0.0;  // Pitch cmd
  y_ref[11] = 0.0;  // Roll cmd
  y_ref[12] = 0.0;  // Thrust

  double y_ref_e[DRONE_W_DISTURBANCES_NYN];
  y_ref_e[0] = 0.1;  // Position x
  y_ref_e[1] = 0.0;  // Position y
  y_ref_e[2] = 1.0;  // Position z
  y_ref_e[3] = 0.0;  // Velocity x
  y_ref_e[4] = 0.0;  // Velocity y
  y_ref_e[5] = 0.0;  // Velocity z
  y_ref_e[6] = 0.0;  // Roll
  y_ref_e[7] = 0.0;  // Pitch
  y_ref_e[8] = 0.0;  // Yaw

    for (int i = 0; i < DRONE_W_DISTURBANCES_N; i++) {
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", y_ref);
  }
  ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, DRONE_W_DISTURBANCES_N,
                         "yref", y_ref_e);
}

void Px4Nmpc::setInitialConditions(const setpoint &state_init) {
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
}

void Px4Nmpc::updateDisturbances(const double &fdis_x, const double &fdis_y,
                                 const double &fdis_z) {
  model_parameters[7] = fdis_x;
  model_parameters[8] = fdis_y;
  model_parameters[9] = fdis_z;

  for (int i = 0; i < DRONE_W_DISTURBANCES_N; i++)
    drone_w_disturbances_acados_update_params(
        acados_ocp_capsule, i, model_parameters, DRONE_W_DISTURBANCES_NP);
}

}  // namespace px4_ctrl