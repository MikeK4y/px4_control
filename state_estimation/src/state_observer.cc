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
  /** TODO: Load R along with the other parameters */
  Eigen::Matrix<double, 9, 9> R_odom;
  R_odom.setZero();
  R_odom.block(0, 0, 3, 3) = 0.05 * Eigen::Matrix3d::Identity();
  R_odom.block(3, 3, 3, 3) = 0.05 * Eigen::Matrix3d::Identity();
  R_odom.block(6, 6, 3, 3) = 0.01 * Eigen::Matrix3d::Identity();
  odom_sensor = new MavrosOdometry(R_odom);

  // Initialize Parameters
  loadParameters();
  is_initialized = false;

  /** TODO: Not sure about the initialization of the time stamp */
  past_cmd = Eigen::Vector4d::Zero();
  latest_cmd = Eigen::Vector4d::Zero();
  latest_cmd_time = ros::Time::now();

  /** TODO: Load P_mat and Q_mat with the other parameters */
  Q_mat.setZero();
  Q_mat.block(0, 0, 3, 3) = 0.1 * Eigen::Matrix3d::Identity();
  Q_mat.block(3, 3, 3, 3) = 0.1 * Eigen::Matrix3d::Identity();
  Q_mat.block(6, 6, 3, 3) = 0.1 * Eigen::Matrix3d::Identity();
  Q_mat.block(9, 9, 3, 3) = 0.1 * Eigen::Matrix3d::Identity();

  P_mat.setZero();
  P_mat.block(0, 0, 3, 3) = 0.01 * Eigen::Matrix3d::Identity();
  P_mat.block(3, 3, 3, 3) = 0.10 * Eigen::Matrix3d::Identity();
  P_mat.block(6, 6, 3, 3) = 0.01 * Eigen::Matrix3d::Identity();
  P_mat.block(9, 9, 3, 3) = 1.00 * Eigen::Matrix3d::Identity();
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

  nh_pvt.param("disturbance_limit", disturbance_limit, 10.0);
}

// Odometry Callback
void StateObserver::odomCallback(const nav_msgs::Odometry &msg) {
  // Initialize state with the first odometry message
  if (!is_initialized) {
    // Time
    past_state_time = msg.header.stamp;

    // Position
    state(0, 0) = msg.pose.pose.position.x;
    state(1, 0) = msg.pose.pose.position.y;
    state(2, 0) = msg.pose.pose.position.z;

    // Attitude
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
    double y, p, r;
    tf::Matrix3x3(q).getEulerYPR(y, p, r);

    state(6, 0) = y;
    state(7, 0) = p;
    state(8, 0) = r;

    // Velocity
    Eigen::Vector3d v_body(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                           msg.twist.twist.linear.z);
    Eigen::Matrix3d R_mat = yprToRotMat(y, p, r);
    Eigen::Vector3d v_world = R_mat * v_body;

    state(3, 0) = v_world(0);
    state(4, 0) = v_world(1);
    state(5, 0) = v_world(2);

    // Disturbances
    state(9, 0) = 0.0;
    state(10, 0) = 0.0;
    state(11, 0) = 0.0;

    is_initialized = true;
  } else {
    // Prepare measurement
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
    double y, p, r;
    tf::Matrix3x3(q).getEulerYPR(y, p, r);
    Eigen::Matrix<double, odom_size, 1> y_odom;
    y_odom << msg.pose.pose.position.x, msg.pose.pose.position.y,
        msg.pose.pose.position.z, msg.twist.twist.linear.x,
        msg.twist.twist.linear.y, msg.twist.twist.linear.z, y, p, r;

    // Run prediction and update time
    predict(msg.header.stamp);
    past_state_time = msg.header.stamp;

    // Run correction
    Eigen::MatrixXd H_mat(9, 12);
    Eigen::MatrixXd y_expected(9, 1);
    odom_sensor->correctionData(state_pred, H_mat, y_expected);

    Eigen::MatrixXd S =
        H_mat * P_pred_mat * H_mat.transpose() + odom_sensor->getCurrentR();
    Eigen::MatrixXd K_mat = P_pred_mat * H_mat.transpose() * S.inverse();

    state_est = state_pred + K_mat * (y_odom - y_expected);

    // Update P marix and make sure it's symmetric
    P_mat =
        (Eigen::MatrixXd::Identity(state_size, state_size) - K_mat * H_mat) *
        P_pred_mat;
    P_mat = 0.5 * (P_mat + P_mat.transpose());

    // Update state
    updateState();

    // Publish state
    publishState(msg.header.stamp);
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
  double dt_l = clipValue((pred_time - latest_cmd_time).toSec(), 0.0, dt);
  double dt_p = dt_l < dt ? dt - dt_l : 0.0;

  Eigen::Vector4d cmd = (dt_p / dt) * past_cmd + (dt_l / dt) * latest_cmd;

  // Use input to get state prediction
  Eigen::Matrix3d R_mat = yprToRotMat(state(6, 0), state(7, 0), state(8, 0));

  // Thrust
  Eigen::Vector3d T(0.0, 0.0, k_thrust * cmd(3));
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

  state_pred(9) = state(9);
  state_pred(10) = state(10);
  state_pred(11) = state(11);

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
  double dtT = dt * cmd(3);

  // Construct F matrix
  Eigen::Matrix<double, state_size, state_size> F_mat =
      Eigen::MatrixXd::Identity(state_size, state_size);

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

void StateObserver::updateState() {
  // Position
  state.block(0, 0, 3, 1) = state_est.block(0, 0, 3, 1);
  // Velocity
  state.block(3, 0, 3, 1) = state_est.block(3, 0, 3, 1);
  // Attitude
  state(6, 0) = checkAngle(state_est(6, 0));
  state(7, 0) = checkAngle(state_est(7, 0));
  state(8, 0) = checkAngle(state_est(8, 0));
  // Disturbances
  state(9, 0) =
      clipValue(state_est(9, 0), -disturbance_limit, disturbance_limit);
  state(10, 0) =
      clipValue(state_est(10, 0), -disturbance_limit, disturbance_limit);
  state(11, 0) =
      clipValue(state_est(11, 0), -disturbance_limit, disturbance_limit);
}

void StateObserver::publishState(ros::Time time) {
  state_estimation::DroneState msg;

  // Header
  msg.header.stamp = time;
  // Position
  msg.pose.position.x = state(0, 0);
  msg.pose.position.y = state(1, 0);
  msg.pose.position.z = state(2, 0);
  // Velocity
  msg.velocity.x = state(3, 0);
  msg.velocity.y = state(4, 0);
  msg.velocity.z = state(5, 0);
  // Attitude
  tf::Quaternion q;
  geometry_msgs::Quaternion q_msg;
  q.setRPY(state(8, 0), state(7, 0), state(6, 0));
  tf::quaternionTFToMsg(q, q_msg);
  msg.pose.orientation = q_msg;
  // Disturbances
  msg.disturbances.x = state(9, 0);
  msg.disturbances.y = state(10, 0);
  msg.disturbances.z = state(11, 0);

  // Publish message
  state_pub.publish(msg);
}

double StateObserver::checkAngle(const double &angle) {
  double angle_corrected = angle < -M_PI ? angle + 2.0 * M_PI : angle;
  angle_corrected =
      angle_corrected >= M_PI ? angle_corrected - 2.0 * M_PI : angle_corrected;

  return angle_corrected;
}

template <class num>
num StateObserver::clipValue(const num &value, const num &l_bound,
                             const num &u_bound) {
  num clipped_value = value < l_bound ? l_bound : value;
  clipped_value = clipped_value > u_bound ? u_bound : clipped_value;

  return clipped_value;
}