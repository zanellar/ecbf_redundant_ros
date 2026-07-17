// Copyright 2026
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <cstdint>
#include <limits>

#include <Eigen/Dense>

namespace ecbf_franka_controller {

// ---------------------------------------------------------------------------
// Fixed-size linear-algebra aliases
// ---------------------------------------------------------------------------

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using RowVector7d = Eigen::Matrix<double, 1, 7>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;

// ---------------------------------------------------------------------------
// Enumerations shared by controller and diagnostics
// ---------------------------------------------------------------------------

/** Identify the kinetic-energy quantity constrained by the CBF. */
enum class CbfType : int {
  kTotal = 0,
  kOperational = 1,
  kDirectional = 2,
};

/** Encode the CBF projection result without allocating a real-time string. */
enum class CbfStatus : std::uint8_t {
  kDisabled = 0,
  kInactive = 1,
  kBounded = 2,
  kActive = 3,
  kInfeasibleZeroGradient = 4,
  kInfeasibleBounds = 5,
  kNumericalFailure = 6,
};

// ---------------------------------------------------------------------------
// State and result structures
// ---------------------------------------------------------------------------

/** Store one fixed-size robot state used by the nominal controller and CBF. */
struct RobotData {
  double dt{0.001};
  double elapsed{0.0};
  Vector7d q{Vector7d::Zero()};
  Vector7d dq{Vector7d::Zero()};
  Vector7d coriolis{Vector7d::Zero()};
  Vector7d previous_command{Vector7d::Zero()};
  Matrix7d mass{Matrix7d::Identity()};
  Matrix7d mass_inverse{Matrix7d::Identity()};
  Matrix7d mass_dot{Matrix7d::Zero()};
  Matrix67d jacobian{Matrix67d::Zero()};
  Matrix67d jacobian_dot{Matrix67d::Zero()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Vector6d task_velocity{Vector6d::Zero()};
  double total_energy{0.0};
  double operational_energy{0.0};
};

/** Store the output and diagnostics of one CBF projection. */
struct CbfResult {
  Vector7d torque{Vector7d::Zero()};
  bool active{false};
  bool feasible{true};
  CbfStatus status{CbfStatus::kDisabled};
  double barrier{std::numeric_limits<double>::quiet_NaN()};
  double nominal_constraint{std::numeric_limits<double>::quiet_NaN()};
  double safe_constraint{std::numeric_limits<double>::quiet_NaN()};
  double correction_norm{0.0};
  double selected_energy{0.0};
  double directional_velocity{0.0};
  double directional_inertia{0.0};
};

}  // namespace ecbf_franka_controller
