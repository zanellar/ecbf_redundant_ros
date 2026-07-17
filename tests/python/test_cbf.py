"""Unit tests for the backend-independent Python CBF projection."""

from types import SimpleNamespace

import numpy as np

from mypkg import cbf


def make_total_energy_state() -> SimpleNamespace:
    """Create the smallest state object required by the total-energy CBF."""
    return SimpleNamespace(
        n_joints=2,
        joint_velocities=np.array([1.0, 0.0]),
        inertia_matrix_dot=np.zeros((2, 2)),
        kinetic_energy=1.0,
        barrier_function_value=np.nan,
        cbf_constraint_nom_value=np.nan,
        cbf_constraint_safe_value=np.nan,
        cbf_constraint_safe_value_a_safe=np.nan,
        cbf_constraint_safe_value_a_nom=np.nan,
        cbf_constraint_safe_value_b=np.nan,
        cbf_correction_norm=0.0,
        cbf_active=False,
        cbf_feasible=True,
        cbf_status="",
    )


def test_negative_alpha_disables_cbf_but_keeps_box_bounds() -> None:
    """Verify that alpha below zero returns the bounded nominal torque."""
    state = make_total_energy_state()
    nominal = np.array([2.0, -2.0])
    safe = cbf.total_kinetic_energy_limit(state, nominal, -1.0, 0.5, [-1.0, -1.0], [1.0, 1.0])
    np.testing.assert_allclose(safe, [1.0, -1.0])
    assert state.cbf_status == "disabled"


def test_total_energy_cbf_projects_violating_nominal_torque() -> None:
    """Verify that an active total-energy CBF satisfies its affine inequality."""
    state = make_total_energy_state()
    nominal = np.array([1.0, 0.0])
    safe = cbf.total_kinetic_energy_limit(state, nominal, 2.0, 0.5)
    assert state.cbf_active
    assert state.cbf_feasible
    assert state.cbf_constraint_safe_value >= -1.0e-9
    assert safe[0] < nominal[0]


def test_projection_reports_infeasible_box() -> None:
    """Verify that the projection reports an impossible half-space and box intersection."""
    result = cbf.project_halfspace_box(np.zeros(2), np.array([1.0, 0.0]), 2.0, [-1.0, -1.0], [1.0, 1.0])
    assert not result.feasible
    assert result.status == "infeasible_bounds"
