#pragma once

// Eigen
#include <eigen3/Eigen/Dense>

// ROS
#include "tf/transform_datatypes.h"

#include "state_estimation/common.hpp"

namespace px4_ctrl {
class BaseSensor {
 public:
  BaseSensor() {}
  BaseSensor(Eigen::MatrixXd R_mat) : R_mat_nom(R_mat), R_mat_cur(R_mat) {}
  ~BaseSensor() {}

  /**
   * @brief Returns the H matrix and expected sensor measurement for the
   * predicted state
   * @param state_pred Predicted state
   * @param H_mat Sensor's H matrix
   * @param y_pred Sensor's expected measurement at the predicted state
   */
  virtual void correctionData(const eskf_state &state_pred,
                              Eigen::MatrixXd &H_mat,
                              Eigen::MatrixXd &y_expected) {}

 private:
  Eigen::MatrixXd R_mat_nom, R_mat_cur;
};
}  // namespace px4_ctrl