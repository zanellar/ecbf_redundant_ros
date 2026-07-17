"""Backend-independent Cartesian impedance calculations used by MuJoCo tests."""

from __future__ import annotations

from typing import Any

import numpy as np


# ---------------------------------------------------------------------------
# SO(3) helpers
# ---------------------------------------------------------------------------

def vee(matrix: np.ndarray) -> np.ndarray:
    """Extract the vector associated with a 3-by-3 skew-symmetric matrix."""
    value = np.asarray(matrix, dtype=float).reshape(3, 3)
    return np.array([value[2, 1], value[0, 2], value[1, 0]], dtype=float)


def so3_log(rotation: np.ndarray) -> np.ndarray:
    """Map a rotation matrix to its base-frame rotation vector."""
    value = np.asarray(rotation, dtype=float).reshape(3, 3)
    cosine = float(np.clip((np.trace(value) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1.0e-8:
        return 0.5 * vee(value - value.T)
    if np.pi - angle < 1.0e-5:
        eigenvalues, eigenvectors = np.linalg.eig(value)
        index = int(np.argmin(np.abs(eigenvalues - 1.0)))
        axis = np.real(eigenvectors[:, index])
        norm = float(np.linalg.norm(axis))
        return np.zeros(3) if norm < 1.0e-12 else angle * axis / norm
    return angle * vee(value - value.T) / (2.0 * np.sin(angle))


def rotation_error(desired: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Return a world/base-expressed orientation error for a zero Jacobian."""
    desired_rotation = np.asarray(desired, dtype=float).reshape(3, 3)
    current_rotation = np.asarray(current, dtype=float).reshape(3, 3)
    return so3_log(desired_rotation @ current_rotation.T)


# ---------------------------------------------------------------------------
# Controller helpers
# ---------------------------------------------------------------------------

def gain_matrix(value: Any, size: int, name: str) -> np.ndarray:
    """Normalize a scalar, vector, or matrix gain into a square NumPy matrix."""
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        return float(array) * np.eye(size)
    if array.shape == (size,):
        return np.diag(array)
    if array.shape == (size, size):
        return array
    raise ValueError(f"{name} must be scalar, length {size}, or shape ({size}, {size}).")


def cartesian_impedance(state: Any, parameters: Any) -> np.ndarray:
    """Compute residual joint torque from Cartesian position and orientation errors."""
    desired_position = np.asarray(parameters.x_desired, dtype=float).reshape(3)
    desired_rotation = np.asarray(parameters.xmat_desired, dtype=float).reshape(3, 3)
    translational_stiffness = gain_matrix(parameters.K_cartesian, 3, "K_cartesian")
    rotational_stiffness = gain_matrix(parameters.Kr_cartesian, 3, "Kr_cartesian")
    translational_damping = gain_matrix(parameters.D_cartesian, 3, "D_cartesian")
    rotational_damping = gain_matrix(parameters.Dr_cartesian, 3, "Dr_cartesian")

    position_error = desired_position - state.eef_positions
    orientation_error = rotation_error(desired_rotation, state.eef_rotation_matrix)
    force = translational_stiffness @ position_error - translational_damping @ state.eef_translational_velocities
    moment = rotational_stiffness @ orientation_error - rotational_damping @ state.eef_rotational_velocities
    torque = state.jacobian_translational.T @ force + state.jacobian_rotational.T @ moment

    if not np.all(np.isfinite(torque)):
        raise FloatingPointError("The nominal Cartesian impedance torque is non-finite.")
    return np.asarray(torque, dtype=float).reshape(state.n_joints)
