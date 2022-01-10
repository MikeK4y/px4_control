#include "state_estimation/sensors/base_sensor.h"

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
  virtual void correctionData(const Eigen::MatrixXd& state,
                              Eigen::MatrixXd& H_mat,
                              Eigen::MatrixXd& y_expected) override {
    H_mat = getHmat(state);
    y_expected = getYExpected(state);
  }

  Eigen::MatrixXd getCurrentR() const { return R_mat_cur; }

 private:
  Eigen::MatrixXd R_mat_nom, R_mat_cur;
  static const int measurement_size = 9;
  static const int state_size = 12;

  /**
   * @brief Calculates the H matrix using the predicted state
   * @param state Predicted state
   * @returns The H matrix at the provided state
   */
  Eigen::MatrixXd getHmat(const Eigen::MatrixXd& state) {
    // Calculate sines/cosines
    double sy = sin(state(6, 0));
    double cy = cos(state(6, 0));
    double sp = sin(state(7, 0));
    double cp = cos(state(7, 0));
    double sr = sin(state(8, 0));
    double cr = cos(state(8, 0));

    // Construct H matrix
    Eigen::Matrix<double, measurement_size, state_size> H_mat;
    H_mat.setZero();

    // Position
    H_mat.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    // Velocity
    // h4 = cycp*xdot + sycp*ydot - sp*zdot
    H_mat(3, 3) = cy * cp;
    H_mat(3, 4) = sy * cp;
    H_mat(3, 5) = -sp;
    H_mat(3, 6) = -sy * cp * state(3, 0) + cy * cp * state(4, 0);
    H_mat(3, 7) =
        -cy * sp * state(3, 0) - sy * sp * state(4, 0) - cp * state(5, 0);

    // h5 = (cyspsr - sycr)*xdot + (syspsr + cycr)*ydot + cpsr*zdot
    H_mat(4, 3) = cy * sp * sr - sy * cr;
    H_mat(4, 4) = sy * sp * sr + cy * cr;
    H_mat(4, 5) = cp * sr;
    H_mat(4, 6) = -(sy * sp * sr + cy * cr) * state(3, 0) +
                  (cy * sp * sr - sy * cr) * state(4, 0);
    H_mat(4, 7) = (cy * cp * sr) * state(3, 0) + (sy * cp * sr) * state(4, 0) -
                  sp * sr * state(5, 0);
    H_mat(4, 8) = (cy * sp * cr + sy * sr) * state(3, 0) +
                  (sy * sp * cr - cy * sr) * state(4, 0) +
                  cp * cr * state(5, 0);

    // h6 = (cyspcr + sysr)*xdot + (syspcr - cycr)*ydot + cpcr*zdot
    H_mat(5, 3) = cy * sp * cr + sy * sr;
    H_mat(5, 4) = sy * sp * cr - cy * cr;
    H_mat(5, 5) = cp * cr;
    H_mat(5, 6) = -(sy * sp * cr - cy * sr) * state(3, 0) +
                  (cy * sp * cr + sy * sr) * state(4, 0);
    H_mat(5, 7) = (cy * cp * cr) * state(3, 0) + (sy * cp * cr) * state(4, 0) -
                  sp * cr * state(5, 0);
    H_mat(5, 8) = -(cy * sp * sr - sy * cr) * state(3, 0) -
                  (sy * sp * sr + cy * cr) * state(4, 0) -
                  cp * sr * state(5, 0);

    // Attitude
    H_mat.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity();

    return H_mat;
  }

  /**
   * @brief Calculates the expected sensor measurement using the predicted state
   * @param state Predicted state
   * @returns The expected measurement at the provided state
   */
  Eigen::MatrixXd getYExpected(const Eigen::MatrixXd& state) {
    Eigen::Matrix3d Rot_mat =
        eulerToRotMat(state(6, 0), state(7, 0), state(8, 0));

    Eigen::Vector3d v_w = Rot_mat.transpose() * state.block(3, 0, 3, 1);

    Eigen::Matrix<double, measurement_size, 1> y_est;

    y_est.segment(0, 3) = state.block(0, 0, 3, 1);
    y_est.segment(3, 3) = v_w;
    y_est.segment(6, 3) = state.block(6, 0, 3, 1);

    return y_est;
  }
};