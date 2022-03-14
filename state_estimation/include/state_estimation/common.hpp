#pragma once

// Eigen
#include <eigen3/Eigen/Dense>

namespace px4_ctrl {

struct eskf_state {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond attitude;
  Eigen::Vector3d disturbances;
};

/**
 * @brief Clips a value between bounds
 * @param value The value to be clipped
 * @param l_bound Lower bound
 * @param u_bound Upper bound
 */
template <class num>
static inline num clipValue(const num &value, const num &l_bound,
                            const num &u_bound) {
  num clipped_value = value < l_bound ? l_bound : value;
  clipped_value = clipped_value > u_bound ? u_bound : clipped_value;

  return clipped_value;
}

/**
 * @brief Returns a skew symmetric matrix from a vector
 * @param v Vector3 to turn to skew symmetric
 * @return Skew symmetric matrix
 */
static inline Eigen::Matrix3d toSkew(const Eigen::Vector3d v) {
  Eigen::Matrix3d s;
  s << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return s;
}

/**
 * @brief Gets the rotation matrix from the Euler angles
 * @param yaw Yaw angle
 * @param pitch Pitch angle
 * @param roll Roll angle
 * @returns Rotation matrix
 */
static inline Eigen::Matrix3d eulerToRotMat(const double &yaw,
                                            const double &pitch,
                                            const double &roll) {
  tf::Matrix3x3 Rot_mat_tf;
  Rot_mat_tf.setEulerYPR(yaw, pitch, roll);

  Eigen::Matrix3d Rot_mat;
  Rot_mat << Rot_mat_tf[0][0], Rot_mat_tf[0][1], Rot_mat_tf[0][2],
      Rot_mat_tf[1][0], Rot_mat_tf[1][1], Rot_mat_tf[1][2], Rot_mat_tf[2][0],
      Rot_mat_tf[2][1], Rot_mat_tf[2][2];

  return Rot_mat;
}

}  // namespace px4_ctrl
