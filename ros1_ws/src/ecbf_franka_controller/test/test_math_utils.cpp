// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <ecbf_franka_controller/math_utils.h>

namespace ecbf_franka_controller {
namespace {

/** Verify that zero relative orientation produces zero impedance error. */
TEST(MathUtilsTest, EqualOrientationsHaveZeroError) {
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  EXPECT_TRUE(orientationError(orientation, orientation).isZero(1.0e-12));
}

/** Verify that the error has the current-minus-desired sign expected by -K times error. */
TEST(MathUtilsTest, OrientationErrorMatchesControllerSignConvention) {
  const Eigen::Quaterniond current = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond desired(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d error = orientationError(desired, current);
  EXPECT_NEAR(error.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(error.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(error.z(), -0.2, 1.0e-12);
}

}  // namespace
}  // namespace ecbf_franka_controller
