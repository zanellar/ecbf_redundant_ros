"""Closed-form and scalar-search CBF filters for the Python simulation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ProjectionResult:
    """Describe a projected torque and the status of the affine CBF constraint."""

    torque: np.ndarray
    active: bool
    feasible: bool
    status: str


# ---------------------------------------------------------------------------
# Projection utilities
# ---------------------------------------------------------------------------

def normalize_bounds(size: int, lower: Optional[Any], upper: Optional[Any]) -> tuple[np.ndarray, np.ndarray]:
    """Convert optional scalar or vector torque bounds into two length-n arrays."""
    lower_array = np.full(size, -np.inf) if lower is None else np.broadcast_to(np.asarray(lower, dtype=float), (size,)).copy()
    upper_array = np.full(size, np.inf) if upper is None else np.broadcast_to(np.asarray(upper, dtype=float), (size,)).copy()
    if np.any(lower_array > upper_array):
        raise ValueError("Every lower torque bound must be less than or equal to its upper bound.")
    return lower_array, upper_array


def project_halfspace_box(
    nominal: np.ndarray,
    normal: np.ndarray,
    rhs: float,
    lower: Optional[Any] = None,
    upper: Optional[Any] = None,
) -> ProjectionResult:
    """Project a nominal torque onto one affine half-space and a box."""
    nominal_vector = np.asarray(nominal, dtype=float).reshape(-1)
    normal_vector = np.asarray(normal, dtype=float).reshape(nominal_vector.size)
    lower_vector, upper_vector = normalize_bounds(nominal_vector.size, lower, upper)
    bounded_nominal = np.clip(nominal_vector, lower_vector, upper_vector)

    bounded_value = float(normal_vector @ bounded_nominal)
    if bounded_value >= rhs - 1.0e-10:
        bounded = not np.allclose(bounded_nominal, nominal_vector)
        return ProjectionResult(bounded_nominal, False, True, "bounded" if bounded else "inactive")

    normal_squared = float(normal_vector @ normal_vector)
    if normal_squared <= 1.0e-14:
        return ProjectionResult(bounded_nominal, False, False, "zero_gradient")

    if not np.any(np.isfinite(lower_vector)) and not np.any(np.isfinite(upper_vector)):
        correction = (rhs - bounded_value) / normal_squared
        return ProjectionResult(nominal_vector + correction * normal_vector, True, True, "active")

    maximizing = bounded_nominal.copy()
    maximum_value = 0.0
    for index, coefficient in enumerate(normal_vector):
        if coefficient > 0.0:
            maximizing[index] = upper_vector[index]
            maximum_value += coefficient * upper_vector[index]
        elif coefficient < 0.0:
            maximizing[index] = lower_vector[index]
            maximum_value += coefficient * lower_vector[index]
    if maximum_value < rhs - 1.0e-10:
        return ProjectionResult(maximizing, True, False, "infeasible_bounds")

    def clipped(multiplier: float) -> np.ndarray:
        """Evaluate the box projection for one nonnegative dual multiplier."""
        return np.clip(nominal_vector + multiplier * normal_vector, lower_vector, upper_vector)

    low = 0.0
    high = 1.0
    while float(normal_vector @ clipped(high)) < rhs:
        high *= 2.0
        if high > 1.0e16:
            return ProjectionResult(maximizing, True, False, "numerical_failure")

    for _ in range(60):
        midpoint = 0.5 * (low + high)
        if float(normal_vector @ clipped(midpoint)) >= rhs:
            high = midpoint
        else:
            low = midpoint

    return ProjectionResult(clipped(high), True, True, "active")


def apply_affine_cbf(
    state: Any,
    nominal: np.ndarray,
    alpha: float,
    barrier: float,
    normal: np.ndarray,
    drift: float,
    lower: Optional[Any] = None,
    upper: Optional[Any] = None,
) -> np.ndarray:
    """Apply one relative-degree-one CBF inequality and update logger diagnostics."""
    nominal_vector = np.asarray(nominal, dtype=float).reshape(state.n_joints)
    if alpha < 0.0:
        lower_vector, upper_vector = normalize_bounds(state.n_joints, lower, upper)
        result = ProjectionResult(np.clip(nominal_vector, lower_vector, upper_vector), False, True, "disabled")
    else:
        rhs = float(-drift - alpha * barrier)
        result = project_halfspace_box(nominal_vector, normal, rhs, lower, upper)

    safe_constraint = float(normal @ result.torque + drift + max(alpha, 0.0) * barrier)
    nominal_constraint = float(normal @ nominal_vector + drift + max(alpha, 0.0) * barrier)
    state.barrier_function_value = float(barrier)
    state.cbf_constraint_nom_value = nominal_constraint
    state.cbf_constraint_safe_value = safe_constraint
    state.cbf_constraint_safe_value_a_nom = float(normal @ nominal_vector)
    state.cbf_constraint_safe_value_a_safe = float(normal @ result.torque)
    state.cbf_constraint_safe_value_b = float(drift)
    state.cbf_correction_norm = float(np.linalg.norm(result.torque - nominal_vector))
    state.cbf_active = result.active
    state.cbf_feasible = result.feasible
    state.cbf_status = result.status
    return result.torque


# ---------------------------------------------------------------------------
# Energy CBF formulations
# ---------------------------------------------------------------------------

def total_kinetic_energy_limit(
    state: Any,
    nominal: np.ndarray,
    alpha: float,
    maximum_energy: float,
    lower: Optional[Any] = None,
    upper: Optional[Any] = None,
) -> np.ndarray:
    """Limit total joint-space kinetic energy under Coriolis-compensated dynamics."""
    velocity = state.joint_velocities
    barrier = float(maximum_energy - state.kinetic_energy)
    normal = -velocity
    drift = float(-0.5 * velocity @ state.inertia_matrix_dot @ velocity)
    return apply_affine_cbf(state, nominal, alpha, barrier, normal, drift, lower, upper)


def operational_kinetic_energy_limit(
    state: Any,
    nominal: np.ndarray,
    alpha: float,
    maximum_energy: float,
    jacobian: Optional[np.ndarray] = None,
    jacobian_dot: Optional[np.ndarray] = None,
    lower: Optional[Any] = None,
    upper: Optional[Any] = None,
) -> np.ndarray:
    """Limit kinetic energy associated with a full or user-defined task Jacobian."""
    task_jacobian = state.jacobian if jacobian is None else np.asarray(jacobian, dtype=float)
    task_jacobian_dot = state.jacobian_dot if jacobian_dot is None else np.asarray(jacobian_dot, dtype=float)
    if task_jacobian.ndim == 1:
        task_jacobian = task_jacobian.reshape(1, -1)
    task_jacobian_dot = task_jacobian_dot.reshape(task_jacobian.shape)

    velocity = state.joint_velocities
    mass_inverse = state.inertia_matrix_inv
    task_velocity = task_jacobian @ velocity
    lambda_inverse = task_jacobian @ mass_inverse @ task_jacobian.T
    lambda_matrix = np.linalg.pinv(lambda_inverse, rcond=1.0e-9)
    kinetic_energy = float(0.5 * task_velocity @ lambda_matrix @ task_velocity)
    barrier = float(maximum_energy - kinetic_energy)

    normal = -(task_velocity @ lambda_matrix @ task_jacobian @ mass_inverse).reshape(-1)
    lambda_inverse_dot = (
        task_jacobian_dot @ mass_inverse @ task_jacobian.T
        + task_jacobian @ mass_inverse @ task_jacobian_dot.T
        - task_jacobian @ mass_inverse @ state.inertia_matrix_dot @ mass_inverse @ task_jacobian.T
    )
    lambda_dot = -lambda_matrix @ lambda_inverse_dot @ lambda_matrix
    lambda_dot = 0.5 * (lambda_dot + lambda_dot.T)
    drift = float(-task_velocity @ lambda_matrix @ task_jacobian_dot @ velocity - 0.5 * task_velocity @ lambda_dot @ task_velocity)
    return apply_affine_cbf(state, nominal, alpha, barrier, normal, drift, lower, upper)


def directional_kinetic_energy_limit(
    state: Any,
    direction: np.ndarray,
    nominal: np.ndarray,
    alpha: float,
    maximum_energy: float,
    direction_dot: Optional[np.ndarray] = None,
    lower: Optional[Any] = None,
    upper: Optional[Any] = None,
) -> np.ndarray:
    """Limit translational kinetic energy along a base/world-expressed direction."""
    direction_vector = np.asarray(direction, dtype=float).reshape(3)
    direction_norm = float(np.linalg.norm(direction_vector))
    if direction_norm <= 1.0e-12:
        raise ValueError("The directional CBF requires a nonzero direction.")
    unit_direction = direction_vector / direction_norm

    if direction_dot is None:
        unit_direction_dot = np.zeros(3)
    else:
        raw_direction_dot = np.asarray(direction_dot, dtype=float).reshape(3)
        unit_direction_dot = (np.eye(3) - np.outer(unit_direction, unit_direction)) @ raw_direction_dot / direction_norm

    directional_jacobian = (unit_direction @ state.jacobian[:3, :]).reshape(1, -1)
    directional_jacobian_dot = (unit_direction @ state.jacobian_dot[:3, :] + unit_direction_dot @ state.jacobian[:3, :]).reshape(1, -1)
    safe_torque = operational_kinetic_energy_limit(
        state,
        nominal,
        alpha,
        maximum_energy,
        directional_jacobian,
        directional_jacobian_dot,
        lower,
        upper,
    )

    inverse_inertia = float((directional_jacobian @ state.inertia_matrix_inv @ directional_jacobian.T)[0, 0])
    reflected_inertia = np.inf if inverse_inertia <= 1.0e-12 else 1.0 / inverse_inertia
    directional_velocity = float((directional_jacobian @ state.joint_velocities)[0])
    state.velocity_dir = directional_velocity
    state.tmp_lambda_dir = reflected_inertia
    state.kinetic_energy_dir = float(0.5 * reflected_inertia * directional_velocity**2)
    return safe_torque
