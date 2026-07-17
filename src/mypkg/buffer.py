"""MuJoCo state buffering and backend-independent derived quantities."""

from __future__ import annotations

from typing import Optional

import mujoco
import numpy as np


# ---------------------------------------------------------------------------
# Linear-algebra helpers
# ---------------------------------------------------------------------------

def symmetric_pseudoinverse(matrix: np.ndarray, rcond: float = 1.0e-9) -> np.ndarray:
    """Return a symmetric Moore-Penrose pseudoinverse for a square matrix."""
    symmetric = 0.5 * (matrix + matrix.T)
    result = np.linalg.pinv(symmetric, rcond=rcond)
    return 0.5 * (result + result.T)


# ---------------------------------------------------------------------------
# Simulation state buffer
# ---------------------------------------------------------------------------

class SimulationBuffer:
    """Store MuJoCo state and compute the quantities required by the controller and CBF."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        site_id: int = 0,
        debug: bool = False,
        derivative_filter_tau: float = 0.0,
    ) -> None:
        """Initialize references, dimensions, and all fixed-size NumPy buffers."""
        self.model = model
        self.data = data
        self.site_id = int(site_id)
        self.debug = bool(debug)
        self.derivative_filter_tau = max(float(derivative_filter_tau), 0.0)
        self.n_joints = int(model.nv)
        self.n_controls = int(model.nu)
        self._mass_api_mode: Optional[str] = None
        self._allocate()

    def _allocate(self) -> None:
        """Allocate or reset every state, derivative, energy, and diagnostic variable."""
        n = self.n_joints
        self.time = 0.0
        self.dt = 0.0

        # Joint-space state and input diagnostics.
        self.joint_positions = np.zeros(self.model.nq)
        self.joint_velocities = np.zeros(n)
        self.joint_velocities_task = np.zeros(n)
        self.joint_velocities_null = np.zeros(n)
        self.commanded_torque = np.zeros(n)
        self.bias_torque = np.zeros(n)

        # Task-space pose and twist in the MuJoCo world frame.
        self.eef_positions = np.zeros(3)
        self.eef_rotation_matrix = np.eye(3)
        self.eef_translational_velocities = np.zeros(3)
        self.eef_rotational_velocities = np.zeros(3)
        self.eef_velocities = np.zeros(6)
        self.velocity_dir = 0.0

        # Geometric Jacobian and numerical derivative.
        self.jacobian = np.zeros((6, n))
        self.jacobian_translational = np.zeros((3, n))
        self.jacobian_rotational = np.zeros((3, n))
        self.jacobian_dot = np.zeros((6, n))
        self.jacobian_dot_raw = np.zeros((6, n))
        self.jacobian_pseudo_inv = np.zeros((n, 6))
        self._jacobian_previous: Optional[np.ndarray] = None

        # Joint-space inertia and numerical derivative.
        self.inertia_matrix = np.eye(n)
        self.inertia_matrix_inv = np.eye(n)
        self.inertia_matrix_dot = np.zeros((n, n))
        self.inertia_matrix_dot_raw = np.zeros((n, n))
        self._inertia_previous: Optional[np.ndarray] = None

        # Compatibility aliases retained for older analysis scripts.
        self.interia_matrix = self.inertia_matrix
        self.interia_matrix_dot = self.inertia_matrix_dot

        # Operational-space quantities.
        self.reflected_inertia = np.zeros((6, 6))
        self.reflected_inertia_inv = np.zeros((6, 6))
        self.tmp_lambda_dir = 0.0

        # Energy values.
        self.potential_energy = 0.0
        self.kinetic_energy = 0.0
        self.kinetic_energy_task = 0.0
        self.kinetic_energy_task_trans = 0.0
        self.kinetic_energy_task_rot = 0.0
        self.kinetic_energy_null = 0.0
        self.kinetic_energy_dir = 0.0

        # CBF diagnostics shared with logger.py.
        self.cbf_alpha = -1.0
        self.barrier_function_value = np.nan
        self.cbf_computation_time = 0.0
        self.cbf_constraint_nom_value = np.nan
        self.cbf_constraint_safe_value = np.nan
        self.cbf_constraint_safe_value_a_safe = np.nan
        self.cbf_constraint_safe_value_a_nom = np.nan
        self.cbf_constraint_safe_value_b = np.nan
        self.cbf_correction_norm = 0.0
        self.cbf_active = False
        self.cbf_feasible = True
        self.cbf_status = "not_evaluated"

        # Optional numerical diagnostics.
        self.SVD_jacobian = np.full(min(6, n), np.nan)
        self.SVD_jacobian_dot = np.full(min(6, n), np.nan)
        self.SVD_interia_matrix_dot = np.full(n, np.nan)
        self.SVD_reflected_inertia = np.full(6, np.nan)
        self.frobenius_norm_reflected_inertia = np.nan
        self.derivatives_ready = False
        self._derivative_samples = 0

    def reset(self) -> None:
        """Reset all buffers before starting another alpha trial."""
        self._allocate()

    def _derivative_gain(self) -> float:
        """Return the first-order low-pass gain used for numerical derivatives."""
        if self.derivative_filter_tau <= 0.0:
            return 1.0
        return self.dt / (self.derivative_filter_tau + self.dt)

    def _update_raw_state(self) -> None:
        """Copy the current MuJoCo state, pose, Jacobian, inertia, and bias torque."""
        self.time = float(self.data.time)
        self.dt = float(self.model.opt.timestep)
        self.joint_positions[:] = self.data.qpos
        self.joint_velocities[:] = self.data.qvel
        self.eef_positions[:] = self.data.site_xpos[self.site_id]
        self.eef_rotation_matrix[:] = self.data.site_xmat[self.site_id].reshape(3, 3)
        mujoco.mj_jacSite(self.model, self.data, self.jacobian_translational, self.jacobian_rotational, self.site_id)
        self.jacobian[:3, :] = self.jacobian_translational
        self.jacobian[3:, :] = self.jacobian_rotational
        # MuJoCo 3.4+ accepts (model, data, destination), while older Python
        # bindings accept (model, destination, data.qM). Detect the API once.
        if self._mass_api_mode != "legacy":
            try:
                mujoco.mj_fullM(self.model, self.data, self.inertia_matrix)
                self._mass_api_mode = "data"
            except TypeError:
                self._mass_api_mode = "legacy"
        if self._mass_api_mode == "legacy":
            mujoco.mj_fullM(self.model, self.inertia_matrix, self.data.qM)
        self.inertia_matrix[:] = 0.5 * (self.inertia_matrix + self.inertia_matrix.T)
        self.inertia_matrix_inv[:] = symmetric_pseudoinverse(self.inertia_matrix)
        self.bias_torque[:] = self.data.qfrc_bias

    def _update_derivatives(self) -> None:
        """Estimate and filter J-dot and M-dot using consecutive control samples."""
        if self._jacobian_previous is None or self._inertia_previous is None or self.dt <= 0.0:
            self.jacobian_dot_raw.fill(0.0)
            self.inertia_matrix_dot_raw.fill(0.0)
        else:
            self.jacobian_dot_raw[:] = (self.jacobian - self._jacobian_previous) / self.dt
            self.inertia_matrix_dot_raw[:] = (self.inertia_matrix - self._inertia_previous) / self.dt

        gain = self._derivative_gain()
        self.jacobian_dot[:] += gain * (self.jacobian_dot_raw - self.jacobian_dot)
        self.inertia_matrix_dot[:] += gain * (self.inertia_matrix_dot_raw - self.inertia_matrix_dot)
        self.inertia_matrix_dot[:] = 0.5 * (self.inertia_matrix_dot + self.inertia_matrix_dot.T)
        self._jacobian_previous = self.jacobian.copy()
        self._inertia_previous = self.inertia_matrix.copy()
        self._derivative_samples += 1
        self.derivatives_ready = self._derivative_samples >= 2

    def _update_derived_quantities(self) -> None:
        """Compute twists, projectors, operational inertia, and kinetic energies."""
        jacobian = self.jacobian
        mass = self.inertia_matrix
        mass_inverse = self.inertia_matrix_inv
        joint_velocity = self.joint_velocities

        task_velocity = jacobian @ joint_velocity
        self.eef_velocities[:] = task_velocity
        self.eef_translational_velocities[:] = task_velocity[:3]
        self.eef_rotational_velocities[:] = task_velocity[3:]

        lambda_inverse = jacobian @ mass_inverse @ jacobian.T
        lambda_matrix = symmetric_pseudoinverse(lambda_inverse)
        self.reflected_inertia_inv[:] = lambda_inverse
        self.reflected_inertia[:] = lambda_matrix

        dynamic_inverse = mass_inverse @ jacobian.T @ lambda_matrix
        self.jacobian_pseudo_inv[:] = dynamic_inverse
        task_projector = dynamic_inverse @ jacobian
        self.joint_velocities_task[:] = task_projector @ joint_velocity
        self.joint_velocities_null[:] = (np.eye(self.n_joints) - task_projector) @ joint_velocity

        self.kinetic_energy = float(0.5 * joint_velocity @ mass @ joint_velocity)
        self.kinetic_energy_task = float(0.5 * task_velocity @ lambda_matrix @ task_velocity)
        self.kinetic_energy_task_trans = float(0.5 * task_velocity[:3] @ lambda_matrix[:3, :3] @ task_velocity[:3])
        self.kinetic_energy_task_rot = float(0.5 * task_velocity[3:] @ lambda_matrix[3:, 3:] @ task_velocity[3:])
        null_velocity = self.joint_velocities_null
        self.kinetic_energy_null = float(0.5 * null_velocity @ mass @ null_velocity)

        gravity = np.asarray(self.model.opt.gravity, dtype=float)
        masses = np.asarray(self.model.body_mass, dtype=float)
        self.potential_energy = float(-np.sum(masses * (self.data.xipos @ gravity)))

    def _update_debugging_quantities(self) -> None:
        """Compute singular values only when debug logging is requested."""
        if not self.debug:
            return
        self.SVD_jacobian[:] = np.linalg.svd(self.jacobian, compute_uv=False)[: self.SVD_jacobian.size]
        self.SVD_jacobian_dot[:] = np.linalg.svd(self.jacobian_dot, compute_uv=False)[: self.SVD_jacobian_dot.size]
        self.SVD_interia_matrix_dot[:] = np.linalg.svd(self.inertia_matrix_dot, compute_uv=False)
        self.SVD_reflected_inertia[:] = np.linalg.svd(self.reflected_inertia, compute_uv=False)
        self.frobenius_norm_reflected_inertia = float(np.linalg.norm(self.reflected_inertia))

    def update(self) -> None:
        """Refresh raw and derived state after mj_step1 and before control evaluation."""
        self._update_raw_state()
        self._update_derivatives()
        self._update_derived_quantities()
        self._update_debugging_quantities()
