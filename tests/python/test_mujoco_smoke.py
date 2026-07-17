"""Optional MuJoCo smoke tests for the self-contained FR3 model."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest


mujoco = pytest.importorskip("mujoco")

from mypkg.buffer import SimulationBuffer


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = REPOSITORY_ROOT / "envsim/fr3-model/scene_ideal.xml"


def test_model_loads_with_seven_torque_controlled_joints() -> None:
    """Verify that the packaged primitive model exposes seven states and inputs."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    assert (model.nq, model.nv, model.nu) == (7, 7, 7)


def test_simulation_buffer_produces_finite_dynamics() -> None:
    """Verify one state update, Jacobian, and inertia calculation on current MuJoCo."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    state = SimulationBuffer(model, data, site_id, False, 0.0)
    state.update()
    assert np.all(np.isfinite(state.jacobian))
    assert np.all(np.isfinite(state.inertia_matrix))
    assert np.all(np.linalg.eigvalsh(state.inertia_matrix) > 0.0)
