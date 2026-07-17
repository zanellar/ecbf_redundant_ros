"""Damped least-squares inverse kinematics for MuJoCo initialization."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from mypkg.control import rotation_error


@dataclass(frozen=True)
class InverseKinematicsResult:
    """Report the final joint vector and convergence diagnostics."""

    q: np.ndarray
    success: bool
    iterations: int
    position_error: float
    orientation_error: float


def inverse_kinematics(
    model: mujoco.MjModel,
    q_seed: np.ndarray,
    position_desired: np.ndarray,
    rotation_desired: np.ndarray,
    site_id: int,
    damping: float = 1.0e-2,
    step_size: float = 0.4,
    nullspace_gain: float = 0.05,
    q_reference: np.ndarray | None = None,
    position_tolerance: float = 1.0e-4,
    orientation_tolerance: float = 1.0e-3,
    max_iterations: int = 500,
) -> InverseKinematicsResult:
    """Solve a six-dimensional site-pose IK problem using a damped Jacobian step."""
    data = mujoco.MjData(model)
    q = np.asarray(q_seed, dtype=float).reshape(model.nq).copy()
    reference = q.copy() if q_reference is None else np.asarray(q_reference, dtype=float).reshape(model.nq)
    desired_position = np.asarray(position_desired, dtype=float).reshape(3)
    desired_rotation = np.asarray(rotation_desired, dtype=float).reshape(3, 3)
    jacobian_translation = np.zeros((3, model.nv))
    jacobian_rotation = np.zeros((3, model.nv))

    for iteration in range(max_iterations):
        data.qpos[:] = q
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        current_position = data.site_xpos[site_id].copy()
        current_rotation = data.site_xmat[site_id].reshape(3, 3).copy()
        position_vector = desired_position - current_position
        orientation_vector = rotation_error(desired_rotation, current_rotation)

        position_norm = float(np.linalg.norm(position_vector))
        orientation_norm = float(np.linalg.norm(orientation_vector))
        if position_norm <= position_tolerance and orientation_norm <= orientation_tolerance:
            return InverseKinematicsResult(q, True, iteration, position_norm, orientation_norm)

        mujoco.mj_jacSite(model, data, jacobian_translation, jacobian_rotation, site_id)
        jacobian = np.vstack((jacobian_translation, jacobian_rotation))
        task_error = np.concatenate((position_vector, orientation_vector))
        regularized = jacobian @ jacobian.T + damping**2 * np.eye(6)
        jacobian_inverse = jacobian.T @ np.linalg.solve(regularized, np.eye(6))
        nullspace = np.eye(model.nv) - jacobian_inverse @ jacobian
        delta = jacobian_inverse @ task_error + nullspace_gain * nullspace @ (reference[: model.nv] - q[: model.nv])
        q[: model.nv] += step_size * delta

        if model.njnt == model.nv:
            for joint_index in range(model.njnt):
                if model.jnt_limited[joint_index]:
                    address = model.jnt_qposadr[joint_index]
                    q[address] = np.clip(q[address], model.jnt_range[joint_index, 0], model.jnt_range[joint_index, 1])

    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    final_position_error = float(np.linalg.norm(desired_position - data.site_xpos[site_id]))
    final_rotation = data.site_xmat[site_id].reshape(3, 3)
    final_orientation_error = float(np.linalg.norm(rotation_error(desired_rotation, final_rotation)))
    return InverseKinematicsResult(q, False, max_iterations, final_position_error, final_orientation_error)
