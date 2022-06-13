#pragma once

#include "state_estimation/sensors/base_sensor.h"

namespace px4_ctrl {
/**
 * @brief Sensor class for Pose sensor. The measurement is the position in
 * the world frame and the attitude
 */
class PoseSensor : public BaseSensor {
 public:
  PoseSensor() {}
  PoseSensor(Eigen::MatrixXd R_mat, const int &N) {
    R_mat_nom = R_mat;
    R_mat_cur = R_mat;
    res_size = N;
    res_full = false;
    cyclic_index = 0;
    res_Mat = Eigen::MatrixXd::Zero(R_mat.rows(), N);
  }
  ~PoseSensor() {}

  /**
   * @brief Returns the H matrix and expected sensor measurement for the
   * predicted state
   * @param state Predicted state
   * @param H_mat Sensor's H matrix
   * @param y_pred Sensor's expected measurement at the predicted state
   */
  virtual void correctionData(const eskf_state &state, Eigen::MatrixXd &H_mat,
                              Eigen::MatrixXd &y_expected) override {
    H_mat = getHmat(state);
    y_expected = getYExpected(state);
  }

  Eigen::MatrixXd getCurrentR() const { return R_mat_cur; }

 private:
  static const int measurement_size = 7;
  static const int state_size = 24;
  static const int error_state_size = 22;

  /**
   * @brief Calculates the H matrix using the predicted state
   * @param state Predicted state
   * @returns The H matrix at the provided state
   */
  Eigen::MatrixXd getHmat(const eskf_state &state) {
    // Construct H matrix
    Eigen::Matrix<double, measurement_size, state_size> H_mat;
    H_mat.setZero();

    // Position
    H_mat.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    // Random walk bias
    H_mat.block(0, 13, 3, 3) = Eigen::Matrix3d::Identity();

    // Attitude
    Eigen::Vector3d axes;
    axes << 0.0, 0.0, 1.0;
    Eigen::Quaterniond q_phi =
        Eigen::Quaterniond(Eigen::AngleAxisd(state.heading_offset, axes));

    H_mat.block(3, 6, 4, 1) = dpqdqw(q_phi);
    H_mat.block(3, 7, 4, 1) = dpqdqx(q_phi);
    H_mat.block(3, 8, 4, 1) = dpqdqy(q_phi);
    H_mat.block(3, 9, 4, 1) = dpqdqz(q_phi);

    // Heading offset
    H_mat(3, 16) = -state.attitude.w() * sin(0.5 * state.heading_offset) -
                   state.attitude.z() * cos(0.5 * state.heading_offset);
    H_mat(4, 16) = -state.attitude.x() * sin(0.5 * state.heading_offset) -
                   state.attitude.y() * cos(0.5 * state.heading_offset);
    H_mat(5, 16) = -state.attitude.y() * sin(0.5 * state.heading_offset) +
                   state.attitude.x() * cos(0.5 * state.heading_offset);
    H_mat(6, 16) = -state.attitude.z() * sin(0.5 * state.heading_offset) +
                   state.attitude.w() * cos(0.5 * state.heading_offset);

    // Error state derivative of the state
    Eigen::MatrixXd Xddx = getXddx(state, state_size, error_state_size);

    return H_mat * Xddx;
  }

  /**
   * @brief Calculates the expected sensor measurement using the predicted state
   * @param state Predicted state
   * @returns The expected measurement at the provided state
   */
  Eigen::MatrixXd getYExpected(const eskf_state &state) {
    Eigen::Matrix<double, measurement_size, 1> y_exp;

    Eigen::Vector3d axes;
    axes << 0.0, 0.0, 1.0;
    Eigen::Quaterniond q_phi =
        Eigen::Quaterniond(Eigen::AngleAxisd(state.heading_offset, axes));

    Eigen::Quaterniond q_meas = q_phi * state.attitude;

    y_exp.segment(0, 3) = state.position + state.random_walk_bias;
    y_exp(3) = q_meas.w();
    y_exp(4) = q_meas.x();
    y_exp(5) = q_meas.y();
    y_exp(6) = q_meas.z();

    return y_exp;
  }
};
}  // namespace px4_ctrl