"""Unit tests for non-real-time ROS 1 parameter generation."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from mypkg.constants import CbfType
from mypkg.ros1_experiment import controller_parameters


def make_experiment_parameters() -> SimpleNamespace:
    """Create a minimal configuration accepted by controller_parameters()."""
    controller = SimpleNamespace(
        x_desired=np.array([0.4, 0.0, 0.5]),
        xmat_desired=np.eye(3),
        K_cartesian=np.diag([100.0, 100.0, 100.0]),
        Kr_cartesian=np.diag([10.0, 10.0, 10.0]),
        D_cartesian=np.diag([50.0, 50.0, 50.0]),
        Dr_cartesian=np.diag([2.0, 2.0, 2.0]),
    )
    return SimpleNamespace(
        controller_name="ecbf_controller",
        arm_id="fr3",
        frame="flange",
        runtime=5.0,
        CBF_type=CbfType.DIRECTIONAL,
        K_max=0.1,
        direction=np.array([1.0, 0.0, 0.0]),
        apply_cbf=False,
        tau_abs_limits=np.array([20.0, 20.0, 20.0, 20.0, 8.0, 8.0, 8.0]),
        max_torque_rate=np.full(7, 500.0),
        stop_joint_damping=np.full(7, 5.0),
        nomctrl_params=controller,
    )


def test_generated_namespace_matches_fr3_joint_names_and_alpha() -> None:
    """Verify that one Python trial becomes a complete controller namespace."""
    parameters = make_experiment_parameters()
    namespace = controller_parameters(parameters, -1.0)["ecbf_controller"]
    assert namespace["alpha"] == -1.0
    assert namespace["arm_id"] == "fr3"
    assert namespace["frame"] == "flange"
    assert namespace["joint_names"] == [f"fr3_joint{index}" for index in range(1, 8)]


def test_generated_gains_are_diagonal_vectors() -> None:
    """Verify that matrix gains are converted into ROS-friendly diagonal lists."""
    parameters = make_experiment_parameters()
    namespace = controller_parameters(parameters, 5.0)["ecbf_controller"]
    assert namespace["translational_stiffness"] == [100.0, 100.0, 100.0]
    assert namespace["rotational_stiffness"] == [10.0, 10.0, 10.0]
    assert namespace["apply_cbf"] is False
