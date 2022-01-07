#include "state_estimation/state_observer.h"

// ROS
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"

StateObserver::StateObserver(ros::NodeHandle &nh) {
  // Setup Subscribers
  odom_sub = nh.subscribe("/mavros/local_position/odom", 1,
                          &StateObserver::odomCallback, this);
  ctrl_sub = nh.subscribe("/mavros/setpoint_raw/attitude", 1,
                          &StateObserver::ctrlCallback, this);

  // Setup Publisher
  state_pub = nh.advertise<state_estimation::DroneState>("/drone_state", 1);

  // Initialize Sensors
  odom_sensor = new MavrosOdometry();

  // Initialize Parameters
  loadParameters();
  is_initialized = false;
  /** TODO: Not sure about the initialization of the time stamp */
  past_cmd = Eigen::Vector4d::Zero();
  latest_cmd = Eigen::Vector4d::Zero();
  latest_cmd_time = ros::Time::now();
  /** TODO: P_mat and Q_mat need to be initialized */
}

StateObserver::~StateObserver() {}

void StateObserver::loadParameters() {
  // Create private nodehandle to load parameters
  ros::NodeHandle nh_pvt("~");

  nh_pvt.param("t_pitch", t_pitch, 1.0);
  nh_pvt.param("k_pitch", k_pitch, 1.0);

  nh_pvt.param("t_roll", t_roll, 1.0);
  nh_pvt.param("k_roll", k_roll, 1.0);

  nh_pvt.param("damping_x", damp_x, -1.0);
  nh_pvt.param("damping_y", damp_y, -1.0);
  nh_pvt.param("damping_z", damp_z, -1.0);
  damping_matrix << damp_x, 0.0, 0.0, 0.0, damp_y, 0.0, 0.0, 0.0, damp_z;

  nh_pvt.param("k_thrust", k_thrust, 1.0);

  nh_pvt.param("gravity", gravity, -9.8066);
  gravity_vector << 0.0, 0.0, gravity;
}

// Odometry Callback
void StateObserver::odomCallback(const nav_msgs::Odometry &msg) {
  // Initialize state with the first odometry message
  if (!is_initialized) {
    // Time
    past_state_time = msg.header.stamp;

    // Position
    state(0, 1) = msg.pose.pose.position.x;
    state(1, 1) = msg.pose.pose.position.y;
    state(2, 1) = msg.pose.pose.position.z;

    // Attitude
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
    double y, p, r;
    tf::Matrix3x3(q).getEulerYPR(y, p, r);

    state(6, 1) = y;
    state(7, 1) = p;
    state(8, 1) = r;

    // Velocity
    Eigen::Vector3d v_body(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                           msg.twist.twist.linear.z);
    Eigen::Matrix3d R_mat = yprToRotMat(y, p, r);
    Eigen::Vector3d v_world = R_mat * v_body;

    state(3, 1) = v_world(0);
    state(4, 1) = v_world(1);
    state(5, 1) = v_world(2);

    // Disturbances
    state(9, 1) = 0.0;
    state(10, 1) = 0.0;
    state(11, 1) = 0.0;
  } else {
  }
}

// Control Callback
void StateObserver::ctrlCallback(const mavros_msgs::AttitudeTarget &msg) {
  // Get Pitch and Roll commands from the quaternion
  tf::Quaternion q_cmd;
  tf::quaternionMsgToTF(msg.orientation, q_cmd);
  double y_cmd, p_cmd, r_cmd;
  tf::Matrix3x3(q_cmd).getEulerYPR(y_cmd, p_cmd, r_cmd);

  // Construct command
  Eigen::Vector4d cmd(msg.body_rate.z, p_cmd, r_cmd, (double)msg.thrust);

  // Update commands
  past_cmd = latest_cmd;
  latest_cmd = cmd;
  latest_cmd_time = msg.header.stamp;
}

void StateObserver::predict(ros::Time pred_time) {
  // Find input for the prediction time step
  double dt = (pred_time - past_state_time).toSec();
  double dt_p = 0.0;
  double dt_l = (pred_time - latest_cmd_time).toSec();

  if (dt_l < dt)
    dt_p = dt - dt_l;
  else if (dt_l > dt)
    dt_l = dt;

  Eigen::Vector4d cmd = (dt_p / dt) * past_cmd + (dt_l / dt) * latest_cmd;

  // Use input to get state prediction
  Eigen::Matrix3d R_mat = yprToRotMat(state(6, 1), state(7, 1), state(8, 1));

  // Thrust
  Eigen::Vector3d T(0.0, 0.0, k_thrust * cmd(4));
  Eigen::Vector3d T_w = R_mat * T;

  // Damping
  Eigen::Vector3d D = damping_matrix * state.segment(3, 3);

  // Disturbances
  Eigen::Vector3d Fd = state.segment(9, 3);

  // Velocity update
  state_pred.segment(3, 3) =
      state.segment(3, 3) + dt * (D + T_w + gravity_vector + Fd);

  // Position update
  state_pred.segment(0, 3) = state.segment(0, 3) + dt * state.segment(3, 3) +
                             0.5 * dt * dt * state_pred.segment(3, 3);

  // Attitude update
  state_pred(6) = state(6) + dt * cmd(0);
  state_pred(7) = state(7) + (dt * (k_pitch * cmd(1) - state(7))) / t_pitch;
  state_pred(8) = state(8) + (dt * (k_roll * cmd(2) - state(8))) / t_roll;

  // Update P_pred
  updatePpred(dt, cmd);
}

void StateObserver::updatePpred(const double &dt, const Eigen::Vector4d &cmd) {
  // Calculate sines/cosines
  double sy = sin(state(6));
  double cy = cos(state(6));
  double sp = sin(state(7));
  double cp = cos(state(7));
  double sr = sin(state(8));
  double cr = cos(state(8));

  // dt * Thrust
  double dtT = dt * cmd(4);

  // Construct F matrix
  Eigen::Matrix<double, state_size, state_size> F_mat =
      Eigen::Matrix<double, state_size, state_size>::Identity();

  // Velocity rows
  F_mat.block(3, 3, 3, 3) = dt * damping_matrix;
  Eigen::Matrix3d dv_dq;
  F_mat(3, 6) = -(sy * sp * cr + cy * sr) * dtT;
  F_mat(3, 7) = cy * cp * cr * dtT;
  F_mat(3, 8) = -(cy * sp * sr + sy * cr) * dtT;
  F_mat(4, 6) = (cy * sp * cr + sy * sr) * dtT;
  F_mat(4, 7) = sy * cp * cr * dtT;
  F_mat(4, 8) = -(sy * cp * sr + cy * cr) * dtT;
  F_mat(5, 6) = 0.0;
  F_mat(5, 7) = sp * cr * dtT;
  F_mat(5, 8) = cp * sr * dtT;
  F_mat.block(3, 9, 3, 3) = dt * Eigen::Matrix3d::Identity();

  // Position rows
  F_mat.block(0, 3, 3, 3) =
      dt * Eigen::Matrix3d::Identity() + 0.5 * dt * F_mat.block(3, 3, 3, 3);
  F_mat.block(0, 6, 3, 3) = 0.5 * dt * F_mat.block(3, 6, 3, 3);
  F_mat.block(0, 9, 3, 3) = 0.5 * dt * F_mat.block(3, 9, 3, 3);

  // Attitude rows
  F_mat(7, 7) = 1 - dt / t_pitch;
  F_mat(8, 8) = 1 - dt / t_roll;

  // Update P_pred
  P_pred_mat = F_mat * P_mat * F_mat.transpose() + Q_mat;
}

Eigen::Matrix3d StateObserver::yprToRotMat(const double &yaw,
                                           const double &pitch,
                                           const double &roll) {
  // Use ROS tf to get the rotation matrix
  tf::Matrix3x3 R_mat_tf;
  R_mat_tf.setEulerYPR(yaw, pitch, roll);

  Eigen::Matrix3d R_mat;
  R_mat << R_mat_tf[0][0], R_mat_tf[0][1], R_mat_tf[0][2], R_mat_tf[1][0],
      R_mat_tf[1][1], R_mat_tf[1][2], R_mat_tf[2][0], R_mat_tf[2][1],
      R_mat_tf[2][2];

  return R_mat;
}