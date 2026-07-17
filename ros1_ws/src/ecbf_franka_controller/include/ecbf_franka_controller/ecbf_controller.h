// Copyright (c) 2023 Franka Robotics GmbH
// Modifications copyright 2026
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/multi_interface_controller.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>

#include <ecbf_franka_controller/EcbfDiagnostics.h>
#include <ecbf_franka_controller/cbf.h>
#include <ecbf_franka_controller/types.h>

namespace ecbf_franka_controller {

/** Run Cartesian impedance and an energy CBF inside the ROS 1 real-time update loop. */
class EcbfController : public controller_interface::MultiInterfaceController<
                           franka_hw::FrankaModelInterface,
                           hardware_interface::EffortJointInterface,
                           franka_hw::FrankaStateInterface> {
 public:
  /** Acquire hardware interfaces, load parameters, and create real-time publishers. */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;

  /** Initialize the desired pose and derivative history at controller activation. */
  void starting(const ros::Time& time) override;

  /** Execute one 1 kHz state, nominal control, CBF, and effort-command cycle. */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /** Command zero residual torque when the controller is stopped. */
  void stopping(const ros::Time& time) override;

 private:
  /** Load every scalar, vector, matrix, and safety parameter from the controller namespace. */
  bool loadParameters(ros::NodeHandle& node_handle);

  /** Read Franka state/model interfaces and update derivatives and energies. */
  void updateRobotData(double dt);

  /** Return the selected flange or configured end-effector transform in the base frame. */
  Eigen::Affine3d controlledTransform() const;

  /** Estimate filtered Jacobian and inertia derivatives without dynamic allocation. */
  void updateDerivatives(const Matrix67d& jacobian, const Matrix7d& mass, double dt);

  /** Interpolate the desired pose from the activation pose to the configured target. */
  void updateDesiredPose(double elapsed);

  /** Compute the residual Cartesian impedance and optional null-space torque. */
  Vector7d computeNominalTorque() const;

  /** Compute final-command limits and convert them to residual-torque bounds. */
  void computeResidualBounds(Vector7d& lower, Vector7d& upper) const;

  /** Compute a bounded damping torque used during shutdown or a CBF failure. */
  Vector7d computeStopCommand(const Vector7d& lower, const Vector7d& upper) const;

  /** Write one seven-dimensional torque vector to the effort joint handles. */
  void writeCommand(const Vector7d& command);

  /** Publish fixed-size diagnostic data at the configured non-real-time rate. */
  void publishDiagnostics(
      const ros::Time& time,
      const Vector7d& nominal,
      const CbfResult& cbf_result,
      const Vector7d& residual_command,
      const Vector7d& final_command,
      double effective_alpha);

  /** Publish the finished flag so the supervisor can stop and unload the controller. */
  void publishFinished();

  // Franka hardware handles acquired once during init().
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Fixed-size controller state and CBF implementation.
  RobotData data_;
  CbfFilter cbf_filter_;
  Matrix67d previous_jacobian_{Matrix67d::Zero()};
  Matrix7d previous_mass_{Matrix7d::Identity()};
  bool derivatives_initialized_{false};
  int derivative_samples_{0};

  // Selected physical control frame.
  franka::Frame frame_{franka::Frame::kFlange};
  std::string frame_name_{"flange"};

  // Trial timing and lifecycle state.
  ros::Time start_time_;
  ros::Time last_diagnostics_time_;
  double runtime_{5.0};
  double cbf_warmup_time_{0.2};
  double target_transition_duration_{5.0};
  double torque_ramp_duration_{1.0};
  double derivative_filter_tau_{0.01};
  double diagnostics_rate_{100.0};
  bool trial_finished_{false};
  bool finished_published_{false};

  // Desired pose and activation pose used by interpolation.
  bool hold_current_pose_{true};
  Eigen::Vector3d position_start_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_start_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_target_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_target_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_desired_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_desired_{Eigen::Quaterniond::Identity()};
  Vector7d q_nullspace_desired_{Vector7d::Zero()};

  // Nominal Cartesian impedance gains.
  Eigen::Matrix3d translational_stiffness_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d rotational_stiffness_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d translational_damping_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d rotational_damping_{Eigen::Matrix3d::Zero()};
  double nullspace_stiffness_{0.0};
  double nullspace_damping_{0.0};

  // CBF and software-safety parameters.
  CbfType cbf_type_{CbfType::kDirectional};
  double alpha_{-1.0};
  double maximum_energy_{0.1};
  Eigen::Vector3d direction_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d direction_dot_{Eigen::Vector3d::Zero()};
  bool apply_cbf_{false};
  bool abort_on_cbf_infeasible_{true};
  Vector7d torque_limits_{Vector7d::Constant(10.0)};
  Vector7d torque_rate_limits_{Vector7d::Constant(500.0)};
  Vector7d stop_joint_damping_{Vector7d::Constant(5.0)};

  // Real-time publishers consumed by non-real-time Python helper nodes.
  std::unique_ptr<realtime_tools::RealtimePublisher<ecbf_franka_controller::EcbfDiagnostics>> diagnostics_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>> finished_publisher_;
};

}  // namespace ecbf_franka_controller
