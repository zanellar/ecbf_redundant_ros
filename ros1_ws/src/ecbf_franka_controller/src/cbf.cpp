// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <ecbf_franka_controller/cbf.h>

#include <cmath>

#include <ecbf_franka_controller/math_utils.h>

namespace ecbf_franka_controller {

/** Filter a residual torque using the selected energy CBF and torque box. */
CbfResult CbfFilter::filter(
    const RobotData& data,
    const Vector7d& nominal,
    CbfType type,
    double alpha,
    double maximum_energy,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& direction_dot,
    const Vector7d& lower,
    const Vector7d& upper) const {
  if (type == CbfType::kTotal) {
    return filterTotal(data, nominal, alpha, maximum_energy, lower, upper);
  }
  if (type == CbfType::kOperational) {
    return filterOperational(data, nominal, alpha, maximum_energy, lower, upper);
  }
  return filterDirectional(data, nominal, alpha, maximum_energy, direction, direction_dot, lower, upper);
}

/** Apply the total joint-space kinetic-energy inequality. */
CbfResult CbfFilter::filterTotal(const RobotData& data, const Vector7d& nominal, double alpha, double maximum_energy, const Vector7d& lower, const Vector7d& upper) const {
  const double energy = 0.5 * data.dq.dot(data.mass * data.dq);
  const double barrier = maximum_energy - energy;
  const Vector7d normal = -data.dq;
  const double drift = -0.5 * data.dq.dot(data.mass_dot * data.dq);
  return applyConstraint(nominal, normal, drift, barrier, alpha, energy, lower, upper);
}

/** Apply the six-dimensional operational kinetic-energy inequality. */
CbfResult CbfFilter::filterOperational(const RobotData& data, const Vector7d& nominal, double alpha, double maximum_energy, const Vector7d& lower, const Vector7d& upper) const {
  const Matrix6d lambda_inverse = data.jacobian * data.mass_inverse * data.jacobian.transpose();
  const Matrix6d lambda = symmetricPseudoInverse(lambda_inverse);
  const Vector6d velocity = data.jacobian * data.dq;
  const double energy = 0.5 * velocity.dot(lambda * velocity);
  const double barrier = maximum_energy - energy;
  const Vector7d normal = -data.mass_inverse * data.jacobian.transpose() * lambda * velocity;
  const Matrix6d lambda_inverse_dot =
      data.jacobian_dot * data.mass_inverse * data.jacobian.transpose()
      + data.jacobian * data.mass_inverse * data.jacobian_dot.transpose()
      - data.jacobian * data.mass_inverse * data.mass_dot * data.mass_inverse * data.jacobian.transpose();
  const Matrix6d lambda_dot_raw = -lambda * lambda_inverse_dot * lambda;
  const Matrix6d lambda_dot = 0.5 * (lambda_dot_raw + lambda_dot_raw.transpose());
  const double drift = -velocity.dot(lambda * data.jacobian_dot * data.dq) - 0.5 * velocity.dot(lambda_dot * velocity);
  return applyConstraint(nominal, normal, drift, barrier, alpha, energy, lower, upper);
}

/** Apply the translational directional kinetic-energy inequality. */
CbfResult CbfFilter::filterDirectional(
    const RobotData& data,
    const Vector7d& nominal,
    double alpha,
    double maximum_energy,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& direction_dot,
    const Vector7d& lower,
    const Vector7d& upper) const {
  CbfResult result;
  const double norm = direction.norm();
  if (norm <= 1.0e-12) {
    result.torque = clipVector(nominal, lower, upper);
    result.feasible = false;
    result.status = CbfStatus::kInfeasibleZeroGradient;
    return result;
  }

  const Eigen::Vector3d unit_direction = direction / norm;
  const Eigen::Matrix3d tangent_projector = Eigen::Matrix3d::Identity() - unit_direction * unit_direction.transpose();
  const Eigen::Vector3d unit_direction_dot = tangent_projector * direction_dot / norm;
  const RowVector7d jacobian = unit_direction.transpose() * data.jacobian.topRows<3>();
  const RowVector7d jacobian_dot = unit_direction.transpose() * data.jacobian_dot.topRows<3>() + unit_direction_dot.transpose() * data.jacobian.topRows<3>();
  const double lambda_inverse = (jacobian * data.mass_inverse * jacobian.transpose())(0, 0);

  if (lambda_inverse <= 1.0e-12) {
    result.torque = clipVector(nominal, lower, upper);
    result.feasible = false;
    result.status = CbfStatus::kInfeasibleZeroGradient;
    return result;
  }

  const double lambda = 1.0 / lambda_inverse;
  const double velocity = (jacobian * data.dq)(0, 0);
  const double energy = 0.5 * lambda * velocity * velocity;
  const double barrier = maximum_energy - energy;
  const double lambda_inverse_dot =
      (jacobian_dot * data.mass_inverse * jacobian.transpose())(0, 0)
      + (jacobian * data.mass_inverse * jacobian_dot.transpose())(0, 0)
      - (jacobian * data.mass_inverse * data.mass_dot * data.mass_inverse * jacobian.transpose())(0, 0);
  const double lambda_dot = -lambda * lambda * lambda_inverse_dot;
  const Vector7d normal = -velocity * lambda * data.mass_inverse * jacobian.transpose();
  const double drift = -velocity * lambda * (jacobian_dot * data.dq)(0, 0) - 0.5 * velocity * velocity * lambda_dot;
  result = applyConstraint(nominal, normal, drift, barrier, alpha, energy, lower, upper);
  result.directional_velocity = velocity;
  result.directional_inertia = lambda;
  return result;
}

/** Project one nominal torque onto an affine half-space and independent bounds. */
CbfResult CbfFilter::applyConstraint(
    const Vector7d& nominal,
    const Vector7d& normal,
    double drift,
    double barrier,
    double alpha,
    double selected_energy,
    const Vector7d& lower,
    const Vector7d& upper) const {
  CbfResult result;
  const Vector7d bounded_nominal = clipVector(nominal, lower, upper);
  result.selected_energy = selected_energy;
  result.barrier = barrier;

  if (alpha < 0.0) {
    result.torque = bounded_nominal;
    result.status = (bounded_nominal - nominal).norm() > 1.0e-12 ? CbfStatus::kBounded : CbfStatus::kDisabled;
    result.correction_norm = (result.torque - nominal).norm();
    return result;
  }

  const double rhs = -drift - alpha * barrier;
  const double bounded_value = normal.dot(bounded_nominal);
  result.nominal_constraint = normal.dot(nominal) + drift + alpha * barrier;
  if (bounded_value >= rhs - 1.0e-10) {
    result.torque = bounded_nominal;
    result.status = (bounded_nominal - nominal).norm() > 1.0e-12 ? CbfStatus::kBounded : CbfStatus::kInactive;
    result.safe_constraint = normal.dot(result.torque) + drift + alpha * barrier;
    result.correction_norm = (result.torque - nominal).norm();
    return result;
  }

  const double normal_squared = normal.squaredNorm();
  if (normal_squared <= 1.0e-14) {
    result.torque = bounded_nominal;
    result.feasible = false;
    result.status = CbfStatus::kInfeasibleZeroGradient;
    result.safe_constraint = normal.dot(result.torque) + drift + alpha * barrier;
    return result;
  }

  Vector7d maximizing = bounded_nominal;
  for (int index = 0; index < 7; ++index) {
    maximizing(index) = normal(index) >= 0.0 ? upper(index) : lower(index);
  }
  if (normal.dot(maximizing) < rhs - 1.0e-10) {
    result.torque = maximizing;
    result.active = true;
    result.feasible = false;
    result.status = CbfStatus::kInfeasibleBounds;
    result.safe_constraint = normal.dot(result.torque) + drift + alpha * barrier;
    result.correction_norm = (result.torque - nominal).norm();
    return result;
  }

  double low = 0.0;
  double high = 1.0;
  // Evaluate the box projection for one nonnegative dual multiplier.
  auto projected = [&](double multiplier) { return clipVector(nominal + multiplier * normal, lower, upper); };
  while (normal.dot(projected(high)) < rhs && high <= 1.0e16) {
    high *= 2.0;
  }
  if (high > 1.0e16) {
    result.torque = maximizing;
    result.active = true;
    result.feasible = false;
    result.status = CbfStatus::kNumericalFailure;
    result.safe_constraint = normal.dot(result.torque) + drift + alpha * barrier;
    result.correction_norm = (result.torque - nominal).norm();
    return result;
  }

  for (int iteration = 0; iteration < 60; ++iteration) {
    const double midpoint = 0.5 * (low + high);
    if (normal.dot(projected(midpoint)) >= rhs) {
      high = midpoint;
    } else {
      low = midpoint;
    }
  }

  result.torque = projected(high);
  result.active = true;
  result.status = CbfStatus::kActive;
  result.safe_constraint = normal.dot(result.torque) + drift + alpha * barrier;
  result.correction_norm = (result.torque - nominal).norm();
  return result;
}

}  // namespace ecbf_franka_controller
