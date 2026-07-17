// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <ecbf_franka_controller/cbf.h>

namespace ecbf_franka_controller {
namespace {

/** Create a simple two-active-joint state embedded in the seven-axis controller type. */
RobotData makeTestData() {
  RobotData data;
  data.dq.setZero();
  data.dq(0) = 1.0;
  data.mass.setIdentity();
  data.mass_dot.setZero();
  data.mass_inverse.setIdentity();
  data.jacobian.setZero();
  data.jacobian(0, 0) = 1.0;
  data.jacobian_dot.setZero();
  return data;
}

/** Verify that negative alpha disables the CBF while preserving software bounds. */
TEST(CbfFilterTest, NegativeAlphaDisablesCbf) {
  const RobotData data = makeTestData();
  const Vector7d nominal = Vector7d::Constant(2.0);
  const Vector7d lower = Vector7d::Constant(-1.0);
  const Vector7d upper = Vector7d::Constant(1.0);
  const CbfFilter filter;
  const CbfResult result = filter.filter(data, nominal, CbfType::kTotal, -1.0, 0.1, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero(), lower, upper);
  EXPECT_TRUE(result.feasible);
  EXPECT_FALSE(result.active);
  EXPECT_NEAR(result.torque.maxCoeff(), 1.0, 1.0e-12);
}

/** Verify that an active total-energy CBF returns a nonnegative safe inequality. */
TEST(CbfFilterTest, TotalEnergyProjectionSatisfiesConstraint) {
  const RobotData data = makeTestData();
  Vector7d nominal = Vector7d::Zero();
  nominal(0) = 1.0;
  const Vector7d lower = Vector7d::Constant(-10.0);
  const Vector7d upper = Vector7d::Constant(10.0);
  const CbfFilter filter;
  const CbfResult result = filter.filter(data, nominal, CbfType::kTotal, 2.0, 0.1, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero(), lower, upper);
  EXPECT_TRUE(result.feasible);
  EXPECT_TRUE(result.active);
  EXPECT_GE(result.safe_constraint, -1.0e-9);
  EXPECT_LT(result.torque(0), nominal(0));
}

}  // namespace
}  // namespace ecbf_franka_controller
