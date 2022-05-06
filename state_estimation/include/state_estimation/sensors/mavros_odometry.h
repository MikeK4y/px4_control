#include "state_estimation/sensors/base_sensor.h"

namespace px4_ctrl {
class MavrosOdometry : public BaseSensor {
 public:
  MavrosOdometry() {}
  MavrosOdometry(Eigen::MatrixXd R_mat, const int &N)
      : R_mat_nom(R_mat), R_mat_cur(R_mat), res_size(N) {
    res_full = false;
    cyclic_index = 0;
    res_Mat = Eigen::MatrixXd::Zero(R_mat.rows(), N);
  }
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

  void updateR(const Eigen::MatrixXd &innov, const Eigen::MatrixXd &Py) {
    res_Mat.block(0, cyclic_index, innov.rows(), innov.cols()) = innov;
    cyclic_index++;

    if (cyclic_index >= res_size) {
      cyclic_index = 0;
      res_full = true;
    }

    if (res_full) {
      Eigen::MatrixXd R_hat = res_Mat * res_Mat.transpose() - Py;

      for (size_t i = 0; i < R_hat.rows(); i++) {
        if (R_hat(i, i) > R_mat_nom(i, i))
          R_mat_cur(i, i) = R_hat(i, i);
        else
          R_mat_cur(i, i) = R_mat_nom(i, i);
      }
    }
  }

 private:
  Eigen::MatrixXd R_mat_nom, R_mat_cur;
  Eigen::MatrixXd res_Mat;
  bool res_full;
  Eigen::Index cyclic_index;
  int res_size;
  static const int measurement_size = 10;
  static const int state_size = 23;
  static const int error_state_size = 21;

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

    // Random walk bias
    H_mat.block(0, 13, 3, 3) = Eigen::Matrix3d::Identity();

    // Velocity
    H_mat.block(3, 3, 3, 3) = state.attitude.toRotationMatrix().transpose();
    H_mat.block(3, 6, 3, 1) = R_w * state.velocity;
    H_mat.block(3, 7, 3, 1) = R_x * state.velocity;
    H_mat.block(3, 8, 3, 1) = R_y * state.velocity;
    H_mat.block(3, 9, 3, 1) = R_z * state.velocity;

    // Attitude
    H_mat.block(6, 6, 4, 4) = Eigen::Matrix4d::Identity();

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
    Eigen::Matrix3d R_mat = state.attitude.toRotationMatrix();

    Eigen::Vector3d v_w = R_mat.transpose() * state.velocity;

    Eigen::Matrix<double, measurement_size, 1> y_exp;

    y_exp.segment(0, 3) = state.position + state.random_walk_bias;
    y_exp.segment(3, 3) = v_w;
    y_exp(6) = state.attitude.w();
    y_exp(7) = state.attitude.x();
    y_exp(8) = state.attitude.y();
    y_exp(9) = state.attitude.z();

    return y_exp;
  }
};
}  // namespace px4_ctrl