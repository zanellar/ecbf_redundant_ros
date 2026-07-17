// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <ecbf_franka_controller/math_utils.h>

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

namespace ecbf_franka_controller {

/** Clamp one scalar to a closed interval using C++14 facilities. */
double clampScalar(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

/** Evaluate the fifth-order minimum-jerk interpolation polynomial on [0, 1]. */
double minimumJerk(double progress) {
  const double value = clampScalar(progress, 0.0, 1.0);
  return 10.0 * std::pow(value, 3) - 15.0 * std::pow(value, 4) + 6.0 * std::pow(value, 5);
}

/** Compute the signed base-frame SO(3) error used by the impedance law. */
Eigen::Vector3d orientationError(const Eigen::Quaterniond& desired, const Eigen::Quaterniond& current) {
  Eigen::Quaterniond error_quaternion = desired * current.conjugate();
  error_quaternion.normalize();
  if (error_quaternion.w() < 0.0) {
    error_quaternion.coeffs() *= -1.0;
  }

  const Eigen::Vector3d vector_part = error_quaternion.vec();
  const double vector_norm = vector_part.norm();
  if (vector_norm <= 1.0e-12) {
    return Eigen::Vector3d::Zero();
  }

  const double angle = 2.0 * std::atan2(vector_norm, clampScalar(error_quaternion.w(), -1.0, 1.0));
  return -angle * vector_part / vector_norm;
}

/** Compute a symmetric pseudoinverse of a fixed 6-by-6 matrix. */
Matrix6d symmetricPseudoInverse(const Matrix6d& matrix, double tolerance) {
  const Matrix6d symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::JacobiSVD<Matrix6d> decomposition(symmetric, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 6, 1> inverse_singular_values = Eigen::Matrix<double, 6, 1>::Zero();
  const double maximum = decomposition.singularValues().maxCoeff();
  const double threshold = tolerance * std::max(1.0, maximum);
  for (int index = 0; index < 6; ++index) {
    const double singular_value = decomposition.singularValues()(index);
    inverse_singular_values(index) = singular_value > threshold ? 1.0 / singular_value : 0.0;
  }
  const Matrix6d result = decomposition.matrixV() * inverse_singular_values.asDiagonal() * decomposition.matrixU().transpose();
  return 0.5 * (result + result.transpose());
}

/** Clamp every component of a seven-dimensional vector to independent bounds. */
Vector7d clipVector(const Vector7d& value, const Vector7d& lower, const Vector7d& upper) {
  Vector7d result = value;
  for (int index = 0; index < 7; ++index) {
    result(index) = clampScalar(value(index), lower(index), upper(index));
  }
  return result;
}

}  // namespace ecbf_franka_controller
