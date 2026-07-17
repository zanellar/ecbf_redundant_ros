"""Unit tests for the Python Cartesian impedance calculations."""

from types import SimpleNamespace

import numpy as np

from mypkg import control


def test_rotation_error_is_zero_for_identical_rotations() -> None:
    """Verify that the SO(3) error vanishes for identical frames."""
    np.testing.assert_allclose(control.rotation_error(np.eye(3), np.eye(3)), np.zeros(3), atol=1.0e-12)


def test_cartesian_impedance_maps_force_through_jacobian() -> None:
    """Verify the translational controller for a simple identity Jacobian block."""
    state = SimpleNamespace(
        n_joints=7,
        eef_positions=np.zeros(3),
        eef_rotation_matrix=np.eye(3),
        eef_translational_velocities=np.zeros(3),
        eef_rotational_velocities=np.zeros(3),
        jacobian_translational=np.hstack((np.eye(3), np.zeros((3, 4)))),
        jacobian_rotational=np.zeros((3, 7)),
    )
    parameters = SimpleNamespace(
        x_desired=np.array([1.0, 0.0, 0.0]),
        xmat_desired=np.eye(3),
        K_cartesian=np.eye(3),
        Kr_cartesian=np.eye(3),
        D_cartesian=np.eye(3),
        Dr_cartesian=np.eye(3),
    )
    torque = control.cartesian_impedance(state, parameters)
    np.testing.assert_allclose(torque, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
