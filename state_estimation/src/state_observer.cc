#include "state_estimation/state_observer.h"

#include <future>

// ROS
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"

namespace px4_ctrl {
StateObserver::StateObserver(ros::NodeHandle &nh) {
  // Setup Subscribers
  odom_sub = nh.subscribe("/mavros/local_position/odom", 1,
                          &StateObserver::odomCallback, this);
  ctrl_sub = nh.subscribe("/mavros/setpoint_raw/attitude", 1,
                          &StateObserver::ctrlCallback, this);

  // Setup Publisher
  state_pub = nh.advertise<px4_control_msgs::DroneState>("/drone_state", 1);

  // Initialize Parameters
  loadParameters();
  is_initialized = false;

  // Initialize Sensor
  odom_sensor = new MavrosOdometry(R_odom, 30);

  /** TODO: Not sure about the initialization of the time stamp */
  past_cmd = Eigen::Vector4d::Zero();
  latest_cmd = Eigen::Vector4d::Zero();
  latest_cmd_time = ros::Time::now();
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

  // Observer parameters
  // Error covariance
  std::vector<double> P_p, P_v, P_dq, P_fd;
  nh_pvt.getParam("P_p", P_p);
  nh_pvt.getParam("P_v", P_v);
  nh_pvt.getParam("P_dq", P_dq);
  nh_pvt.getParam("P_fd", P_fd);

  // Process covariance
  double Q_p, Q_v, Q_dq, Q_fd;
  nh_pvt.param("Q_p", Q_p, 0.1);
  nh_pvt.param("Q_v", Q_v, 0.1);
  nh_pvt.param("Q_dq", Q_dq, 0.1);
  nh_pvt.param("Q_fd", Q_fd, 0.1);

  // Odometry covariance
  std::vector<double> R_odom_p, R_odom_v, R_odom_q;
  nh_pvt.getParam("R_odom_p", R_odom_p);
  nh_pvt.getParam("R_odom_v", R_odom_v);
  nh_pvt.getParam("R_odom_q", R_odom_q);

  // Initialize matrices
  P_mat.setZero();
  P_mat.block(0, 0, 3, 3).diagonal() = Eigen::Vector3d(P_p[0], P_p[1], P_p[2]);
  P_mat.block(3, 3, 3, 3).diagonal() = Eigen::Vector3d(P_v[0], P_v[1], P_v[2]);
  P_mat.block(6, 6, 3, 3).diagonal() =
      Eigen::Vector3d(P_dq[0], P_dq[1], P_dq[2]);
  P_mat.block(9, 9, 3, 3).diagonal() =
      Eigen::Vector3d(P_fd[0], P_fd[1], P_fd[2]);

  Q_mat.setZero();
  Q_mat.block(0, 0, 3, 3) = Q_p * Eigen::Matrix3d::Identity();
  Q_mat.block(3, 3, 3, 3) = Q_v * Eigen::Matrix3d::Identity();
  Q_mat.block(6, 6, 3, 3) = Q_dq * Eigen::Matrix3d::Identity();
  Q_mat.block(9, 9, 3, 3) = Q_fd * Eigen::Matrix3d::Identity();

  R_odom.setZero();
  R_odom.block(0, 0, 3, 3).diagonal() =
      Eigen::Vector3d(R_odom_p[0], R_odom_p[1], R_odom_p[2]);
  R_odom.block(3, 3, 3, 3).diagonal() =
      Eigen::Vector3d(R_odom_v[0], R_odom_v[1], R_odom_v[2]);
  R_odom.block(6, 6, 4, 4).diagonal() =
      Eigen::Vector4d(R_odom_q[0], R_odom_q[1], R_odom_q[2], R_odom_q[3]);
}

// Odometry Callback
void StateObserver::odomCallback(const nav_msgs::Odometry &msg) {
  // Initialize state with the first odometry message
  if (!is_initialized) {
    // Time
    past_state_time = msg.header.stamp;

    // Position
    state.position(0) = msg.pose.pose.position.x;
    state.position(1) = msg.pose.pose.position.y;
    state.position(2) = msg.pose.pose.position.z;

    // Attitude
    state.attitude = Eigen::Quaterniond(
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    // Velocity
    Eigen::Vector3d v_body(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                           msg.twist.twist.linear.z);
    Eigen::Vector3d v_world = state.attitude.toRotationMatrix() * v_body;

    state.velocity(0) = v_world(0);
    state.velocity(1) = v_world(1);
    state.velocity(2) = v_world(2);

    // Disturbances
    state.disturbances(0) = 0.0;
    state.disturbances(1) = 0.0;
    state.disturbances(2) = 0.0;

    is_initialized = true;
  } else {
    // Prepare measurement
    Eigen::Matrix<double, odom_size, 1> y_odom;
    y_odom << msg.pose.pose.position.x, msg.pose.pose.position.y,
        msg.pose.pose.position.z, msg.twist.twist.linear.x,
        msg.twist.twist.linear.y, msg.twist.twist.linear.z,
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;

    // Run prediction and update time
    predict(msg.header.stamp);

    // Run correction
    Eigen::MatrixXd H_mat(9, 12);
    Eigen::MatrixXd y_expected(9, 1);
    odom_sensor->correctionData(state_pred, H_mat, y_expected);

    Eigen::MatrixXd S =
        H_mat * P_pred_mat * H_mat.transpose() + odom_sensor->getCurrentR();
    Eigen::MatrixXd K_mat = P_pred_mat * H_mat.transpose() * S.inverse();

    error_state = K_mat * (y_odom - y_expected);

    // Update state
    std::future<void> state_update_f =
        std::async(std::launch::async, &StateObserver::correctState, this);

    // Update P marix and make sure it's symmetric
    Eigen::MatrixXd IKH_mat =
        Eigen::MatrixXd::Identity(error_state_size, error_state_size) -
        K_mat * H_mat;
    P_mat = IKH_mat * P_pred_mat;
    P_mat = 0.5 * (P_mat + P_mat.transpose());

    // Publish state
    state_update_f.wait();
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
  /** TODO: Doesn't work as I was expecting. In the test bag files the cmds
   * appear first but have a timestamp after the odometry's*/
  Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
  double dt = (pred_time - past_state_time).toSec();

  // If there's been a while without a new cmd set them to zero
  if ((pred_time - latest_cmd_time).toSec() < 0.5) {
    double dt_l = clipValue((pred_time - latest_cmd_time).toSec(), 0.0, dt);
    double dt_p = dt_l < dt ? dt - dt_l : 0.0;

    cmd = (dt_p / dt) * past_cmd + (dt_l / dt) * latest_cmd;
  }

  // Run error covariance prediction asynchronously
  std::future<void> p_pred_update_f =
      std::async(std::launch::async, &StateObserver::updatePpred, this,
                 std::cref(dt), std::cref(cmd));

  // Use Runge Kutta 4 to propagate state
  Eigen::VectorXd k1 = getSystemDerivative(state, cmd);
  Eigen::VectorXd k2 =
      getSystemDerivative(addUpdate(state, 0.5 * dt * k1), cmd);
  Eigen::VectorXd k3 =
      getSystemDerivative(addUpdate(state, 0.5 * dt * k2), cmd);
  Eigen::VectorXd k4 = getSystemDerivative(addUpdate(state, dt * k3), cmd);

  state_pred = addUpdate(state, dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6);
  past_state_time = pred_time;
}

Eigen::VectorXd StateObserver::getSystemDerivative(const eskf_state &state,
                                                   const Eigen::Vector4d &cmd) {
  // Get Rotation matrix
  Eigen::Matrix3d R_mat = state.attitude.toRotationMatrix();

  // Thrust
  Eigen::Vector3d T(0.0, 0.0, k_thrust * cmd(3));
  Eigen::Vector3d T_w = R_mat * T;

  // Damping
  Eigen::Vector3d D = damping_matrix * state.velocity;

  // Initialize derivative
  Eigen::VectorXd systemDerivative(10);
  systemDerivative.setZero();

  // Position derivative
  systemDerivative.segment(0, 3) = state.velocity;

  // Velocity derivative
  systemDerivative.segment(3, 3) =
      D + T_w + gravity_vector + state.disturbances;

  // Omegas
  double pitch = asin(R_mat(2, 0));
  double roll = atan2(R_mat(2, 1), R_mat(2, 2));

  double omega_z = cmd(0);
  double omega_y = (k_pitch * cmd(1) - pitch) / t_pitch;
  double omega_x = (k_roll * cmd(2) - roll) / t_roll;

  // Attitude derivative
  systemDerivative(6) =
      0.5 * (-omega_x * state.attitude.x() - omega_y * state.attitude.y() -
             omega_z * state.attitude.z());
  systemDerivative(7) =
      0.5 * (omega_x * state.attitude.w() + omega_y * state.attitude.z() -
             omega_z * state.attitude.y());
  systemDerivative(8) =
      0.5 * (-omega_x * state.attitude.z() + omega_y * state.attitude.w() +
             omega_z * state.attitude.x());
  systemDerivative(9) =
      0.5 * (omega_x * state.attitude.y() - omega_y * state.attitude.w() +
             omega_z * state.attitude.w());

  return systemDerivative;
}

eskf_state StateObserver::addUpdate(const eskf_state &state,
                                    const Eigen::VectorXd &state_update) {
  eskf_state updated_state;
  updated_state.position = state.position + state_update.segment(0, 3);
  updated_state.velocity = state.velocity + state_update.segment(3, 3);
  updated_state.attitude =
      Eigen::Quaterniond(state.attitude.w() + state_update(6),
                         state.attitude.x() + state_update(7),
                         state.attitude.y() + state_update(8),
                         state.attitude.z() + state_update(9));
  updated_state.attitude.normalize();
  updated_state.disturbances = state.disturbances;

  return updated_state;
}

void StateObserver::updatePpred(const double &dt, const Eigen::Vector4d &cmd) {
  // Construct F matrix
  Eigen::Matrix<double, error_state_size, error_state_size> F_mat;
  F_mat.setZero();

  Eigen::Matrix3d R_mat = state.attitude.toRotationMatrix();
  Eigen::Vector3d T(0.0, 0.0, k_thrust * cmd(3));
  Eigen::Matrix3d T_w = R_mat * toSkew(T);

  // Position
  F_mat.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  F_mat.block(0, 3, 3, 3) =
      dt * Eigen::Matrix3d::Identity() + 0.5 * dt * dt * damping_matrix;
  F_mat.block(0, 6, 3, 3) = -0.5 * dt * dt * T_w;
  F_mat.block(0, 9, 3, 3) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();

  // Velocity
  F_mat.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() + dt * damping_matrix;
  F_mat.block(3, 6, 3, 3) = -dt * T_w;
  F_mat.block(3, 9, 3, 3) = dt * Eigen::Matrix3d::Identity();

  // Omegas
  double pitch = asin(R_mat(2, 0));
  double roll = atan2(R_mat(2, 1), R_mat(2, 2));

  double omega_z = cmd(0);
  double omega_y = (k_pitch * cmd(1) - pitch) / t_pitch;
  double omega_x = (k_roll * cmd(2) - roll) / t_roll;

  // Attitude
  F_mat.block(6, 6, 3, 3) =
      eulerToRotMat(dt * omega_z, dt * omega_y, dt * omega_x);

  // Disturbances
  F_mat.block(9, 9, 3, 3) = Eigen::Matrix3d::Identity();

  // Update P_pred
  P_pred_mat = F_mat * P_mat * F_mat.transpose() + Q_mat;
}

void StateObserver::correctState() {
  // Position
  state.position(0) = state_pred.position(0) + error_state(0);
  state.position(1) = state_pred.position(1) + error_state(1);
  state.position(2) = state_pred.position(2) + error_state(2);

  // Velocity
  state.velocity(0) = state_pred.velocity(0) + error_state(3);
  state.velocity(1) = state_pred.velocity(1) + error_state(4);
  state.velocity(2) = state_pred.velocity(2) + error_state(5);

  // Attitude
  Eigen::Vector3d thetas;
  thetas << error_state(6), error_state(7), error_state(8);
  double angle = thetas.norm();
  if (angle > 0) {
    thetas = thetas / angle;
    Eigen::Quaterniond dq =
        Eigen::Quaterniond(Eigen::AngleAxisd(angle, thetas));
    state.attitude = state_pred.attitude * dq;
  } else {
    state.attitude = state_pred.attitude;
  }

  // Disturbances
  state.disturbances(0) = clipValue(state_pred.disturbances(0) + error_state(9),
                                    -disturbance_limit, disturbance_limit);
  state.disturbances(1) =
      clipValue(state_pred.disturbances(1) + error_state(10),
                -disturbance_limit, disturbance_limit);
  state.disturbances(2) =
      clipValue(state_pred.disturbances(2) + error_state(11),
                -disturbance_limit, disturbance_limit);
}

void StateObserver::publishState(ros::Time time) {
  px4_control_msgs::DroneState msg;

  // Header
  msg.header.stamp = time;

  // Position
  msg.pose.position.x = state.position(0);
  msg.pose.position.y = state.position(1);
  msg.pose.position.z = state.position(2);

  // Velocity
  msg.velocity.x = state.velocity(0);
  msg.velocity.y = state.velocity(1);
  msg.velocity.z = state.velocity(2);

  // Attitude
  msg.pose.orientation.w = state.attitude.w();
  msg.pose.orientation.x = state.attitude.x();
  msg.pose.orientation.y = state.attitude.y();
  msg.pose.orientation.z = state.attitude.z();

  // Disturbances
  msg.disturbances.x = state.disturbances(0);
  msg.disturbances.y = state.disturbances(1);
  msg.disturbances.z = state.disturbances(2);

  // // Error covariance
  // for (size_t i = 0; i < P_mat.rows(); i++) {
  //   for (size_t j = 0; j < P_mat.cols(); j++) {
  //     msg.covariance.push_back(P_mat(i, j));
  //   }
  // }

  // Publish message
  state_pub.publish(msg);
}

}  // namespace px4_ctrl
