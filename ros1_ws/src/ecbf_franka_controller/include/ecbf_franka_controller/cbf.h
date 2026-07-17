// Copyright 2026
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <Eigen/Dense>

#include <ecbf_franka_controller/types.h>

namespace ecbf_franka_controller {

/** Apply total, operational, or directional kinetic-energy CBF projections. */
class CbfFilter {
 public:
  /** Filter a residual torque using the selected energy CBF and torque box. */
  CbfResult filter(
      const RobotData& data,
      const Vector7d& nominal,
      CbfType type,
      double alpha,
      double maximum_energy,
      const Eigen::Vector3d& direction,
      const Eigen::Vector3d& direction_dot,
      const Vector7d& lower,
      const Vector7d& upper) const;

 private:
  /** Apply the total joint-space kinetic-energy inequality. */
  CbfResult filterTotal(const RobotData& data, const Vector7d& nominal, double alpha, double maximum_energy, const Vector7d& lower, const Vector7d& upper) const;

  /** Apply the six-dimensional operational kinetic-energy inequality. */
  CbfResult filterOperational(const RobotData& data, const Vector7d& nominal, double alpha, double maximum_energy, const Vector7d& lower, const Vector7d& upper) const;

  /** Apply the translational directional kinetic-energy inequality. */
  CbfResult filterDirectional(
      const RobotData& data,
      const Vector7d& nominal,
      double alpha,
      double maximum_energy,
      const Eigen::Vector3d& direction,
      const Eigen::Vector3d& direction_dot,
      const Vector7d& lower,
      const Vector7d& upper) const;

  /** Project one nominal torque onto an affine half-space and independent bounds. */
  CbfResult applyConstraint(
      const Vector7d& nominal,
      const Vector7d& normal,
      double drift,
      double barrier,
      double alpha,
      double selected_energy,
      const Vector7d& lower,
      const Vector7d& upper) const;
};

}  // namespace ecbf_franka_controller
