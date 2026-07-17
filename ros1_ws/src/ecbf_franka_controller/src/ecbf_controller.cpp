// Copyright (c) 2023 Franka Robotics GmbH
// Modifications copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <ecbf_franka_controller/ecbf_controller.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface_exception.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

#include <ecbf_franka_controller/math_utils.h>

namespace ecbf_franka_controller {
namespace {

/** Load one fixed-length vector parameter into an Eigen vector. */
template <int Size>
bool loadVector(ros::NodeHandle& node_handle, const std::string& name, Eigen::Matrix<double, Size, 1>& output) {
  std::vector<double> values;
  if (!node_handle.getParam(name, values) || values.size() != static_cast<std::size_t>(Size)) {
    ROS_ERROR_STREAM("EcbfController: parameter " << name << " must contain " << Size << " values.");
    return false;
  }
  for (int index = 0; index < Size; ++index) {
    output(index) = values[static_cast<std::size_t>(index)];
  }
  return true;
}

/** Load three diagonal gains and write the corresponding 3-by-3 matrix. */
bool loadDiagonalGain(ros::NodeHandle& node_handle, const std::string& name, Eigen::Matrix3d& output) {
  Eigen::Vector3d values;
  if (!loadVector<3>(node_handle, name, values)) {
    return false;
  }
  output = values.asDiagonal();
  return true;
}

/** Load a row-major 3-by-3 rotation matrix from a nine-value ROS parameter. */
bool loadRotation(ros::NodeHandle& node_handle, const std::string& name, Eigen::Matrix3d& output) {
  std::vector<double> values;
  if (!node_handle.getParam(name, values) || values.size() != 9) {
    ROS_ERROR_STREAM("EcbfController: parameter " << name << " must contain nine values.");
    return false;
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      output(row, column) = values[static_cast<std::size_t>(3 * row + column)];
    }
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> decomposition(output, Eigen::ComputeFullU | Eigen::ComputeFullV);
  output = decomposition.matrixU() * decomposition.matrixV().transpose();
  if (output.determinant() < 0.0) {
    Eigen::Matrix3d corrected_u = decomposition.matrixU();
    corrected_u.col(2) *= -1.0;
    output = corrected_u * decomposition.matrixV().transpose();
  }
  return true;
}

/** Return true when every component is finite and strictly positive. */
bool positiveFinite(const Vector7d& value) {
  return value.allFinite() && (value.array() > 0.0).all();
}

}  // namespace

/** Acquire hardware interfaces, load parameters, and create real-time publishers. */
bool EcbfController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  if (!loadParameters(node_handle)) {
    return false;
  }

  std::string arm_id;
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("EcbfController: missing arm_id parameter.");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("EcbfController: joint_names must contain seven joints.");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  auto* effort_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (model_interface == nullptr || state_interface == nullptr || effort_interface == nullptr) {
    ROS_ERROR("EcbfController: required Franka model, state, or effort interface is unavailable.");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    joint_handles_.reserve(7);
    for (const std::string& joint_name : joint_names) {
      joint_handles_.push_back(effort_interface->getHandle(joint_name));
    }
  } catch (const hardware_interface::HardwareInterfaceException& error) {
    ROS_ERROR_STREAM("EcbfController: failed to acquire hardware handles: " << error.what());
    return false;
  }

  diagnostics_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ecbf_franka_controller::EcbfDiagnostics>>(node_handle, "diagnostics", 4);
  finished_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Bool>>(node_handle, "finished", 2);
  return true;
}

/** Initialize the desired pose and derivative history at controller activation. */
void EcbfController::starting(const ros::Time& time) {
  const franka::RobotState robot_state = state_handle_->getRobotState();
  const Eigen::Affine3d transform = controlledTransform();
  Eigen::Map<const Vector7d> q(robot_state.q.data());

  position_start_ = transform.translation();
  orientation_start_ = Eigen::Quaterniond(transform.rotation());
  orientation_start_.normalize();
  if (orientation_start_.coeffs().dot(orientation_target_.coeffs()) < 0.0) {
    orientation_target_.coeffs() *= -1.0;
  }
  if (hold_current_pose_) {
    position_target_ = position_start_;
    orientation_target_ = orientation_start_;
  }
  position_desired_ = position_start_;
  orientation_desired_ = orientation_start_;
  q_nullspace_desired_ = q;

  derivatives_initialized_ = false;
  derivative_samples_ = 0;
  data_ = RobotData{};
  start_time_ = time;
  last_diagnostics_time_ = time;
  trial_finished_ = false;
  finished_published_ = false;
  ROS_INFO_STREAM("EcbfController started in frame " << frame_name_ << " with alpha=" << alpha_ << ".");
}

/** Execute one 1 kHz state, nominal control, CBF, and effort-command cycle. */
void EcbfController::update(const ros::Time& time, const ros::Duration& period) {
  const double dt = period.toSec() > 0.0 ? period.toSec() : 0.001;
  updateRobotData(dt);
  data_.elapsed = (time - start_time_).toSec();

  Vector7d residual_lower;
  Vector7d residual_upper;
  computeResidualBounds(residual_lower, residual_upper);

  if (trial_finished_) {
    const Vector7d residual_stop = computeStopCommand(residual_lower, residual_upper);
    const Vector7d final_stop = residual_stop + data_.coriolis;
    writeCommand(final_stop);
    publishFinished();
    return;
  }

  updateDesiredPose(data_.elapsed);
  const double ramp = minimumJerk(data_.elapsed / std::max(torque_ramp_duration_, 1.0e-6));
  const Vector7d nominal = ramp * computeNominalTorque();
  const bool derivatives_ready = derivative_samples_ >= 2;
  const double effective_alpha = derivatives_ready && data_.elapsed >= cbf_warmup_time_ ? alpha_ : -1.0;
  const CbfResult cbf_result = cbf_filter_.filter(
      data_,
      nominal,
      cbf_type_,
      effective_alpha,
      maximum_energy_,
      direction_,
      direction_dot_,
      residual_lower,
      residual_upper);

  if (!cbf_result.feasible && abort_on_cbf_infeasible_ && apply_cbf_ && effective_alpha >= 0.0) {
    const Vector7d residual_stop = computeStopCommand(residual_lower, residual_upper);
    const Vector7d final_stop = residual_stop + data_.coriolis;
    writeCommand(final_stop);
    publishDiagnostics(time, nominal, cbf_result, residual_stop, final_stop, effective_alpha);
    trial_finished_ = true;
    publishFinished();
    return;
  }

  const Vector7d bounded_nominal = clipVector(nominal, residual_lower, residual_upper);
  const bool command_cbf = apply_cbf_ && effective_alpha >= 0.0;
  const Vector7d residual_command = command_cbf ? cbf_result.torque : bounded_nominal;
  const Vector7d final_command = residual_command + data_.coriolis;
  writeCommand(final_command);
  publishDiagnostics(time, nominal, cbf_result, residual_command, final_command, effective_alpha);

  if (runtime_ > 0.0 && data_.elapsed >= runtime_) {
    trial_finished_ = true;
    publishFinished();
  }
}

/** Command zero residual torque when the controller is stopped. */
void EcbfController::stopping(const ros::Time& time) {
  (void)time;
  for (hardware_interface::JointHandle& handle : joint_handles_) {
    handle.setCommand(0.0);
  }
}

/** Load every scalar, vector, matrix, and safety parameter from the controller namespace. */
bool EcbfController::loadParameters(ros::NodeHandle& node_handle) {
  int cbf_type = static_cast<int>(CbfType::kDirectional);
  node_handle.param("runtime", runtime_, 5.0);
  node_handle.param("alpha", alpha_, -1.0);
  node_handle.param("cbf_type", cbf_type, static_cast<int>(CbfType::kDirectional));
  node_handle.param("maximum_energy", maximum_energy_, 0.1);
  node_handle.param("apply_cbf", apply_cbf_, false);
  node_handle.param("abort_on_cbf_infeasible", abort_on_cbf_infeasible_, true);
  node_handle.param("cbf_warmup_time", cbf_warmup_time_, 0.2);
  node_handle.param("hold_current_pose", hold_current_pose_, true);
  node_handle.param("target_transition_duration", target_transition_duration_, 5.0);
  node_handle.param("torque_ramp_duration", torque_ramp_duration_, 1.0);
  node_handle.param("derivative_filter_tau", derivative_filter_tau_, 0.01);
  node_handle.param("diagnostics_rate", diagnostics_rate_, 100.0);
  node_handle.param("nullspace_stiffness", nullspace_stiffness_, 0.0);
  node_handle.param("nullspace_damping", nullspace_damping_, 0.0);
  node_handle.param<std::string>("frame", frame_name_, std::string("flange"));

  if (cbf_type < static_cast<int>(CbfType::kTotal) || cbf_type > static_cast<int>(CbfType::kDirectional)) {
    ROS_ERROR("EcbfController: cbf_type must be 0, 1, or 2.");
    return false;
  }
  cbf_type_ = static_cast<CbfType>(cbf_type);

  if (frame_name_ == "flange") {
    frame_ = franka::Frame::kFlange;
  } else if (frame_name_ == "end_effector") {
    frame_ = franka::Frame::kEndEffector;
  } else {
    ROS_ERROR("EcbfController: frame must be 'flange' or 'end_effector'.");
    return false;
  }

  Eigen::Matrix3d target_rotation;
  const bool parameters_ok =
      loadVector<3>(node_handle, "desired_position", position_target_)
      && loadRotation(node_handle, "desired_rotation", target_rotation)
      && loadDiagonalGain(node_handle, "translational_stiffness", translational_stiffness_)
      && loadDiagonalGain(node_handle, "rotational_stiffness", rotational_stiffness_)
      && loadDiagonalGain(node_handle, "translational_damping", translational_damping_)
      && loadDiagonalGain(node_handle, "rotational_damping", rotational_damping_)
      && loadVector<3>(node_handle, "direction", direction_)
      && loadVector<3>(node_handle, "direction_dot", direction_dot_)
      && loadVector<7>(node_handle, "tau_abs_limits", torque_limits_)
      && loadVector<7>(node_handle, "max_torque_rate", torque_rate_limits_)
      && loadVector<7>(node_handle, "stop_joint_damping", stop_joint_damping_);
  if (!parameters_ok) {
    return false;
  }

  orientation_target_ = Eigen::Quaterniond(target_rotation);
  orientation_target_.normalize();
  if (!positiveFinite(torque_limits_) || !positiveFinite(torque_rate_limits_) || !positiveFinite(stop_joint_damping_)) {
    ROS_ERROR("EcbfController: torque limits, torque-rate limits, and stop damping must be positive.");
    return false;
  }
  if (maximum_energy_ <= 0.0 || runtime_ <= 0.0 || diagnostics_rate_ <= 0.0) {
    ROS_ERROR("EcbfController: maximum_energy, runtime, and diagnostics_rate must be positive.");
    return false;
  }
  return true;
}

/** Read Franka state/model interfaces and update derivatives and energies. */
void EcbfController::updateRobotData(double dt) {
  const franka::RobotState robot_state = state_handle_->getRobotState();
  const std::array<double, 49> mass_array = model_handle_->getMass();
  const std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  const std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(frame_);
  Eigen::Map<const Matrix7d> mass_map(mass_array.data());
  Eigen::Map<const Vector7d> coriolis_map(coriolis_array.data());
  Eigen::Map<const Matrix67d> jacobian_map(jacobian_array.data());
  Eigen::Map<const Vector7d> q_map(robot_state.q.data());
  Eigen::Map<const Vector7d> dq_map(robot_state.dq.data());
  Eigen::Map<const Vector7d> previous_command_map(robot_state.tau_J_d.data());

  data_.dt = dt;
  data_.q = q_map;
  data_.dq = dq_map;
  data_.coriolis = coriolis_map;
  data_.previous_command = previous_command_map;
  data_.mass = 0.5 * (mass_map + mass_map.transpose());
  data_.mass_inverse = data_.mass.ldlt().solve(Matrix7d::Identity());
  data_.jacobian = jacobian_map;
  const Eigen::Affine3d transform = controlledTransform();
  data_.position = transform.translation();
  data_.rotation = transform.rotation();
  data_.task_velocity = data_.jacobian * data_.dq;
  updateDerivatives(data_.jacobian, data_.mass, dt);

  data_.total_energy = 0.5 * data_.dq.dot(data_.mass * data_.dq);
  const Matrix6d lambda_inverse = data_.jacobian * data_.mass_inverse * data_.jacobian.transpose();
  const Matrix6d lambda = symmetricPseudoInverse(lambda_inverse);
  data_.operational_energy = 0.5 * data_.task_velocity.dot(lambda * data_.task_velocity);
}

/** Return the selected flange or configured end-effector transform in the base frame. */
Eigen::Affine3d EcbfController::controlledTransform() const {
  const std::array<double, 16> pose_array = model_handle_->getPose(frame_);
  return Eigen::Affine3d(Eigen::Matrix4d::Map(pose_array.data()));
}

/** Estimate filtered Jacobian and inertia derivatives without dynamic allocation. */
void EcbfController::updateDerivatives(const Matrix67d& jacobian, const Matrix7d& mass, double dt) {
  if (!derivatives_initialized_ || dt <= 0.0) {
    data_.jacobian_dot.setZero();
    data_.mass_dot.setZero();
    previous_jacobian_ = jacobian;
    previous_mass_ = mass;
    derivatives_initialized_ = true;
    derivative_samples_ = 0;
    return;
  }

  const Matrix67d jacobian_dot_raw = (jacobian - previous_jacobian_) / dt;
  const Matrix7d mass_dot_raw = (mass - previous_mass_) / dt;
  const double gain = derivative_filter_tau_ <= 0.0 ? 1.0 : dt / (derivative_filter_tau_ + dt);
  data_.jacobian_dot += gain * (jacobian_dot_raw - data_.jacobian_dot);
  data_.mass_dot += gain * (mass_dot_raw - data_.mass_dot);
  data_.mass_dot = 0.5 * (data_.mass_dot + data_.mass_dot.transpose());
  previous_jacobian_ = jacobian;
  previous_mass_ = mass;
  ++derivative_samples_;
}

/** Interpolate the desired pose from the activation pose to the configured target. */
void EcbfController::updateDesiredPose(double elapsed) {
  if (hold_current_pose_) {
    position_desired_ = position_start_;
    orientation_desired_ = orientation_start_;
    return;
  }
  const double scale = minimumJerk(elapsed / std::max(target_transition_duration_, 1.0e-6));
  position_desired_ = position_start_ + scale * (position_target_ - position_start_);
  orientation_desired_ = orientation_start_.slerp(scale, orientation_target_);
  orientation_desired_.normalize();
}

/** Compute the residual Cartesian impedance and optional null-space torque. */
Vector7d EcbfController::computeNominalTorque() const {
  const Eigen::Vector3d position_error = data_.position - position_desired_;
  const Eigen::Quaterniond current_orientation(data_.rotation);
  const Eigen::Vector3d rotation_error = orientationError(orientation_desired_, current_orientation);
  Vector6d error;
  error.head<3>() = position_error;
  error.tail<3>() = rotation_error;

  Matrix6d stiffness = Matrix6d::Zero();
  Matrix6d damping = Matrix6d::Zero();
  stiffness.topLeftCorner<3, 3>() = translational_stiffness_;
  stiffness.bottomRightCorner<3, 3>() = rotational_stiffness_;
  damping.topLeftCorner<3, 3>() = translational_damping_;
  damping.bottomRightCorner<3, 3>() = rotational_damping_;
  const Vector7d task_torque = data_.jacobian.transpose() * (-stiffness * error - damping * data_.task_velocity);

  if (nullspace_stiffness_ <= 0.0 && nullspace_damping_ <= 0.0) {
    return task_torque;
  }
  const Matrix6d jacobian_product_inverse = symmetricPseudoInverse(data_.jacobian * data_.jacobian.transpose());
  const Matrix7d nullspace_projector = Matrix7d::Identity() - data_.jacobian.transpose() * jacobian_product_inverse * data_.jacobian;
  const Vector7d posture_torque = nullspace_stiffness_ * (q_nullspace_desired_ - data_.q) - nullspace_damping_ * data_.dq;
  return task_torque + nullspace_projector * posture_torque;
}

/** Compute final-command limits and convert them to residual-torque bounds. */
void EcbfController::computeResidualBounds(Vector7d& lower, Vector7d& upper) const {
  const Vector7d maximum_change = torque_rate_limits_ * std::max(data_.dt, 1.0e-4);
  Vector7d final_lower = (-torque_limits_).cwiseMax(data_.previous_command - maximum_change);
  Vector7d final_upper = torque_limits_.cwiseMin(data_.previous_command + maximum_change);

  // Recover a valid singleton interval if a previous controller command starts
  // outside the configured absolute software limit.
  for (int index = 0; index < 7; ++index) {
    if (final_lower(index) > final_upper(index)) {
      const double bounded_previous = clampScalar(data_.previous_command(index), -torque_limits_(index), torque_limits_(index));
      final_lower(index) = bounded_previous;
      final_upper(index) = bounded_previous;
    }
  }

  lower = final_lower - data_.coriolis;
  upper = final_upper - data_.coriolis;
}

/** Compute a bounded damping torque used during shutdown or a CBF failure. */
Vector7d EcbfController::computeStopCommand(const Vector7d& lower, const Vector7d& upper) const {
  const Vector7d residual_stop = -stop_joint_damping_.cwiseProduct(data_.dq);
  return clipVector(residual_stop, lower, upper);
}

/** Write one seven-dimensional torque vector to the effort joint handles. */
void EcbfController::writeCommand(const Vector7d& command) {
  for (std::size_t index = 0; index < joint_handles_.size(); ++index) {
    joint_handles_[index].setCommand(command(static_cast<int>(index)));
  }
}

/** Publish fixed-size diagnostic data at the configured non-real-time rate. */
void EcbfController::publishDiagnostics(
    const ros::Time& time,
    const Vector7d& nominal,
    const CbfResult& cbf_result,
    const Vector7d& residual_command,
    const Vector7d& final_command,
    double effective_alpha) {
  if ((time - last_diagnostics_time_).toSec() < 1.0 / diagnostics_rate_) {
    return;
  }
  if (!diagnostics_publisher_->trylock()) {
    return;
  }

  auto& message = diagnostics_publisher_->msg_;
  message.header.stamp = time;
  message.alpha = effective_alpha;
  message.cbf_type = static_cast<std::int32_t>(cbf_type_);
  message.cbf_enabled = effective_alpha >= 0.0;
  message.cbf_applied = apply_cbf_ && effective_alpha >= 0.0;
  message.cbf_active = cbf_result.active;
  message.cbf_feasible = cbf_result.feasible;
  message.cbf_status = static_cast<std::uint8_t>(cbf_result.status);
  message.barrier_function_value = cbf_result.barrier;
  message.cbf_constraint_nom_value = cbf_result.nominal_constraint;
  message.cbf_constraint_safe_value = cbf_result.safe_constraint;
  message.cbf_correction_norm = cbf_result.correction_norm;
  message.total_energy = data_.total_energy;
  message.operational_energy = data_.operational_energy;
  message.selected_energy = cbf_result.selected_energy;
  message.directional_velocity = cbf_result.directional_velocity;
  message.directional_inertia = cbf_result.directional_inertia;
  message.elapsed = data_.elapsed;

  for (int index = 0; index < 7; ++index) {
    message.q[static_cast<std::size_t>(index)] = data_.q(index);
    message.dq[static_cast<std::size_t>(index)] = data_.dq(index);
    message.tau_nominal[static_cast<std::size_t>(index)] = nominal(index);
    message.tau_safe[static_cast<std::size_t>(index)] = cbf_result.torque(index);
    message.tau_residual_command[static_cast<std::size_t>(index)] = residual_command(index);
    message.tau_command[static_cast<std::size_t>(index)] = final_command(index);
    message.coriolis[static_cast<std::size_t>(index)] = data_.coriolis(index);
  }
  for (int index = 0; index < 3; ++index) {
    message.position[static_cast<std::size_t>(index)] = data_.position(index);
  }
  for (int index = 0; index < 6; ++index) {
    message.task_velocity[static_cast<std::size_t>(index)] = data_.task_velocity(index);
  }

  diagnostics_publisher_->unlockAndPublish();
  last_diagnostics_time_ = time;
}

/** Publish the finished flag so the supervisor can stop and unload the controller. */
void EcbfController::publishFinished() {
  if (finished_published_ || !finished_publisher_->trylock()) {
    return;
  }
  finished_publisher_->msg_.data = true;
  finished_publisher_->unlockAndPublish();
  finished_published_ = true;
}

}  // namespace ecbf_franka_controller

PLUGINLIB_EXPORT_CLASS(ecbf_franka_controller::EcbfController, controller_interface::ControllerBase)
