// Copyright 2026
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <Eigen/Dense>

#include <ecbf_franka_controller/types.h>

namespace ecbf_franka_controller {

/** Clamp one scalar to a closed interval using C++14 facilities. */
double clampScalar(double value, double lower, double upper);

/** Evaluate the fifth-order minimum-jerk interpolation polynomial on [0, 1]. */
double minimumJerk(double progress);

/** Compute the signed base-frame SO(3) error used by the impedance law. */
Eigen::Vector3d orientationError(const Eigen::Quaterniond& desired, const Eigen::Quaterniond& current);

/** Compute a symmetric pseudoinverse of a fixed 6-by-6 matrix. */
Matrix6d symmetricPseudoInverse(const Matrix6d& matrix, double tolerance = 1.0e-9);

/** Clamp every component of a seven-dimensional vector to independent bounds. */
Vector7d clipVector(const Vector7d& value, const Vector7d& lower, const Vector7d& upper);

}  // namespace ecbf_franka_controller
