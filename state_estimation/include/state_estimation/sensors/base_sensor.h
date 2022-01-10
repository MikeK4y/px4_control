#pragma once

// Eigen
#include <eigen3/Eigen/Dense>

// ROS
#include "tf/transform_datatypes.h"

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
  virtual void correctionData(const Eigen::MatrixXd &state_pred,
                              Eigen::MatrixXd &H_mat,
                              Eigen::MatrixXd &y_expected) {}

 protected:
  /**
   * @brief Gets the rotation matrix from the Euler angles
   * @param yaw Yaw angle
   * @param pitch Pitch angle
   * @param roll Roll angle
   * @returns Rotation matrix
   */
  Eigen::Matrix3d eulerToRotMat(const double &yaw, const double &pitch,
                                const double &roll) {
    tf::Matrix3x3 Rot_mat_tf;
    Rot_mat_tf.setEulerYPR(yaw, pitch, roll);

    Eigen::Matrix3d Rot_mat;
    Rot_mat << Rot_mat_tf[0][0], Rot_mat_tf[0][1], Rot_mat_tf[0][2],
        Rot_mat_tf[1][0], Rot_mat_tf[1][1], Rot_mat_tf[1][2], Rot_mat_tf[2][0],
        Rot_mat_tf[2][1], Rot_mat_tf[2][2];

    return Rot_mat;
  }

 private:
  Eigen::MatrixXd R_mat_nom, R_mat_cur;
};
