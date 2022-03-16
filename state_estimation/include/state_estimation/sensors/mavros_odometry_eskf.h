#include "state_estimation/sensors/base_sensor_eskf.h"

namespace px4_ctrl {
class MavrosOdometry : public BaseSensor {
 public:
  MavrosOdometry() {}
  MavrosOdometry(Eigen::MatrixXd R_mat) : R_mat_nom(R_mat), R_mat_cur(R_mat) {}
  ~MavrosOdometry() {}

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
  Eigen::MatrixXd R_mat_nom, R_mat_cur;
  static const int measurement_size = 10;
  static const int state_size = 13;
  static const int error_state_size = 12;

  /**
   * @brief Calculates the H matrix using the predicted state
   * @param state Predicted state
   * @returns The H matrix at the provided state
   */
  Eigen::MatrixXd getHmat(const eskf_state &state) {
    // Quaternion derivatives of R transpose
    Eigen::Matrix3d R_w = dRdqw(state.attitude).transpose();
    Eigen::Matrix3d R_x = dRdqx(state.attitude).transpose();
    Eigen::Matrix3d R_y = dRdqy(state.attitude).transpose();
    Eigen::Matrix3d R_z = dRdqz(state.attitude).transpose();

    // Construct H matrix
    Eigen::Matrix<double, measurement_size, state_size> H_mat;
    H_mat.setZero();

    // Position
    H_mat.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    // Velocity
    H_mat.block(3, 3, 3, 3) = state.attitude.toRotationMatrix().transpose();
    H_mat.block(3, 6, 3, 1) = R_w * state.velocity;
    H_mat.block(3, 7, 3, 1) = R_x * state.velocity;
    H_mat.block(3, 8, 3, 1) = R_y * state.velocity;
    H_mat.block(3, 9, 3, 1) = R_z * state.velocity;

    // Attitude
    H_mat.block(6, 6, 4, 4) = Eigen::Matrix4d::Identity();

    // Error state derivative of the state
    Eigen::Matrix<double, state_size, error_state_size> Xddx;
    Xddx.setZero();

    // Position
    Xddx.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    // Velocity
    Xddx.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

    // Attitude
    Xddx(6, 6) = -0.5 * state.attitude.x();
    Xddx(6, 7) = -0.5 * state.attitude.y();
    Xddx(6, 8) = -0.5 * state.attitude.z();

    Xddx(7, 6) = 0.5 * state.attitude.w();
    Xddx(7, 7) = -0.5 * state.attitude.z();
    Xddx(7, 8) = 0.5 * state.attitude.y();

    Xddx(8, 6) = 0.5 * state.attitude.z();
    Xddx(8, 7) = 0.5 * state.attitude.w();
    Xddx(8, 8) = -0.5 * state.attitude.x();

    Xddx(9, 6) = -0.5 * state.attitude.y();
    Xddx(9, 7) = 0.5 * state.attitude.x();
    Xddx(9, 8) = 0.5 * state.attitude.w();

    // Disturbances
    Xddx.block(10, 9, 3, 3) = Eigen::Matrix3d::Identity();

    return H_mat * Xddx;
  }

  /**
   * @brief Calculates the expected sensor measurement using the predicted state
   * @param state Predicted state
   * @returns The expected measurement at the provided state
   */
  Eigen::MatrixXd getYExpected(const eskf_state &state) {
    Eigen::Matrix3d R_mat = state.attitude.toRotationMatrix();

    Eigen::Vector3d v_w = R_mat.transpose() * state.velocity;

    Eigen::Matrix<double, measurement_size, 1> y_est;

    y_est.segment(0, 3) = state.position;
    y_est.segment(3, 3) = v_w;
    y_est(6) = state.attitude.w();
    y_est(7) = state.attitude.x();
    y_est(8) = state.attitude.y();
    y_est(9) = state.attitude.z();

    return y_est;
  }
};
}  // namespace px4_ctrl