"""MuJoCo experiment runner using the shared Python controller and CBF equations."""

from __future__ import annotations

import copy
import pickle
import time
from contextlib import nullcontext
from pathlib import Path
from typing import Any, Optional

import mujoco
import mujoco.viewer
import numpy as np

from mypkg import buffer, cbf, control, logger
from mypkg.constants import CbfType
from mypkg.kinematics import inverse_kinematics


# ---------------------------------------------------------------------------
# Path and initialization helpers
# ---------------------------------------------------------------------------

def repository_root() -> Path:
    """Return the repository root independently of the current working directory."""
    return Path(__file__).resolve().parents[2]


def resolve_model_path(model_path: str) -> Path:
    """Resolve an absolute model path or a repository-relative model path."""
    candidate = Path(model_path)
    return candidate if candidate.is_absolute() else repository_root() / candidate


def alpha_log_filename(base_filename: str, alpha: float, index: int) -> str:
    """Create one deterministic CSV filename for each simulated alpha trial."""
    path = Path(base_filename)
    magnitude = f"{abs(alpha):g}".replace(".", "p")
    tag = f"m{magnitude}" if alpha < 0.0 else magnitude
    return str(path.with_name(f"{path.stem}_trial_{index + 1:02d}_alpha_{tag}{path.suffix or '.csv'}"))


def compute_initial_configuration(model: mujoco.MjModel, data: mujoco.MjData, params: Any) -> np.ndarray:
    """Choose a joint initial state directly or solve the requested Cartesian pose."""
    if hasattr(params, "joint_initial"):
        return np.asarray(params.joint_initial, dtype=float).reshape(model.nq)
    if hasattr(params, "x_initial") and hasattr(params, "xmat_initial"):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        seed = np.array([0.0, 0.0, 0.0, -np.pi / 2.0, 0.0, np.pi / 2.0, 0.0])
        result = inverse_kinematics(
            model,
            seed,
            np.asarray(params.x_initial),
            np.asarray(params.xmat_initial),
            site_id,
            damping=1.0e-2,
            step_size=0.4,
            nullspace_gain=0.05,
            q_reference=seed,
            position_tolerance=1.0e-4,
            orientation_tolerance=1.0e-3,
            max_iterations=500,
        )
        if not result.success:
            print(f"IK warning: position error={result.position_error:.3e}, orientation error={result.orientation_error:.3e}")
        return result.q
    return data.qpos.copy()


def configure_viewer(viewer: Any, params: Any) -> None:
    """Apply optional camera parameters and add a desired-position marker."""
    with viewer.lock():
        viewer.user_scn.ngeom = 0
        if hasattr(params, "view_params"):
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.distance = params.view_params["distance"]
            viewer.cam.azimuth = params.view_params["azimuth"]
            viewer.cam.elevation = params.view_params["elevation"]
            viewer.cam.lookat[:] = params.view_params["lookat"]
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[0],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.02, 0.02, 0.02],
            pos=params.nomctrl_params.x_desired,
            mat=np.eye(3).reshape(-1),
            rgba=[1.0, 0.0, 0.0, 1.0],
        )
        viewer.user_scn.ngeom = 1
    viewer.sync()


# ---------------------------------------------------------------------------
# Controller selection
# ---------------------------------------------------------------------------

def apply_selected_cbf(state: Any, params: Any, nominal: np.ndarray, alpha: float) -> np.ndarray:
    """Dispatch to the selected Python CBF formulation."""
    cbf_type = CbfType(int(params.CBF_type))
    if cbf_type == CbfType.TOTAL:
        return cbf.total_kinetic_energy_limit(state, nominal, alpha, params.K_max)
    if cbf_type == CbfType.OPERATIONAL:
        return cbf.operational_kinetic_energy_limit(state, nominal, alpha, params.K_max)
    direction_dot = getattr(params, "direction_dot", np.zeros(3))
    return cbf.directional_kinetic_energy_limit(state, params.direction, nominal, alpha, params.K_max, direction_dot)


# ---------------------------------------------------------------------------
# Main simulation entry point
# ---------------------------------------------------------------------------

def run(params: Any, debug: bool = False) -> dict[str, Any]:
    """Run one reset MuJoCo trial for every alpha in params.alphas."""
    print(f"MuJoCo version: {mujoco.mj_versionString()}")
    output_directory = repository_root() / "output"
    output_directory.mkdir(parents=True, exist_ok=True)
    with (output_directory / "simulation_params.pkl").open("wb") as file:
        pickle.dump(params, file)

    model = mujoco.MjModel.from_xml_path(str(resolve_model_path(params.model_path)))
    data = mujoco.MjData(model)
    model.opt.timestep = float(params.timestep)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    initial_configuration = compute_initial_configuration(model, data, params)
    data.qpos[:] = initial_configuration
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    state = buffer.SimulationBuffer(
        model,
        data,
        site_id,
        debug,
        float(getattr(params, "derivative_filter_tau", 0.0)),
    )

    viewer: Optional[Any] = None
    if bool(getattr(params, "runviewer", False)):
        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
        configure_viewer(viewer, params)

    results: list[dict[str, Any]] = []
    try:
        for index, alpha_value in enumerate(params.alphas):
            alpha = float(alpha_value)
            mujoco.mj_resetDataKeyframe(model, data, 0)
            data.qpos[:] = initial_configuration
            data.qvel[:] = 0.0
            data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)
            state.reset()
            state.cbf_alpha = alpha

            base_log = str(getattr(params, "log_filename", "output/simulation.csv"))
            trial_filename = alpha_log_filename(base_log, alpha, index)
            log_enabled = bool(getattr(params, "log", True))
            log_context = logger.SimulationLogger(state, str(repository_root() / trial_filename)) if log_enabled else nullcontext(None)
            maximum_control_time = 0.0
            print(f"Running simulation trial {index + 1}/{len(params.alphas)} with alpha={alpha:g}")

            with log_context as trial_logger:
                while data.time <= float(params.runtime):
                    mujoco.mj_step1(model, data)
                    control_start = time.perf_counter()
                    state.update()
                    nominal = control.cartesian_impedance(state, params.nomctrl_params)
                    safe = apply_selected_cbf(state, params, nominal, alpha)
                    bias = data.qfrc_bias.copy()
                    command = safe + bias
                    state.commanded_torque[:] = command
                    state.cbf_computation_time = time.perf_counter() - control_start
                    maximum_control_time = max(maximum_control_time, state.cbf_computation_time)
                    data.ctrl[:] = command
                    if trial_logger is not None:
                        trial_logger.log(nominal, bias, safe, command)
                    mujoco.mj_step2(model, data)
                    if viewer is not None:
                        viewer.sync()

            results.append({"alpha": alpha, "logfile": trial_filename, "maximum_control_time": maximum_control_time})
    finally:
        if viewer is not None:
            viewer.close()
    return {"trials": results}
