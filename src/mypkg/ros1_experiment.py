"""Non-real-time Python orchestration for the ROS 1 Franka controller."""

from __future__ import annotations

import copy
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from mypkg.constants import CbfType


# ---------------------------------------------------------------------------
# Configuration conversion helpers
# ---------------------------------------------------------------------------

def repository_root() -> Path:
    """Return the repository root independently of the shell working directory."""
    return Path(__file__).resolve().parents[2]


def require_executable(name: str) -> str:
    """Locate one required ROS command and raise a clear setup error when absent."""
    executable = shutil.which(name)
    if executable is None:
        raise RuntimeError(f"Required command {name!r} was not found. Source ROS Noetic and ros1_ws/devel/setup.bash.")
    return executable


def vector_list(value: Any, size: int, name: str) -> list[float]:
    """Convert a scalar or fixed-length array configuration value into a Python list."""
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.full(size, float(array))
    array = array.reshape(-1)
    if array.size != size:
        raise ValueError(f"{name} must contain {size} values.")
    return [float(item) for item in array]


def matrix_diagonal(value: Any, size: int, name: str) -> list[float]:
    """Extract diagonal controller gains from a scalar, vector, or square matrix."""
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        return [float(array)] * size
    if array.shape == (size,):
        return [float(item) for item in array]
    if array.shape == (size, size):
        return [float(item) for item in np.diag(array)]
    raise ValueError(f"{name} must be scalar, length {size}, or shape ({size}, {size}).")


def alpha_log_filename(base_filename: str, alpha: float, index: int) -> Path:
    """Create a unique experiment CSV path for one alpha trial."""
    path = Path(base_filename)
    magnitude = f"{abs(alpha):g}".replace(".", "p")
    tag = f"m{magnitude}" if alpha < 0.0 else magnitude
    return repository_root() / path.with_name(f"{path.stem}_trial_{index + 1:02d}_alpha_{tag}{path.suffix or '.csv'}")


def controller_parameters(params: Any, alpha: float) -> dict[str, Any]:
    """Translate one Python configuration into the ROS controller parameter namespace."""
    controller = params.nomctrl_params
    arm_id = str(params.arm_id)
    return {
        str(params.controller_name): {
            "type": "ecbf_franka_controller/EcbfController",
            "arm_id": arm_id,
            "joint_names": [f"{arm_id}_joint{index}" for index in range(1, 8)],
            "frame": str(params.frame),
            "runtime": float(params.runtime),
            "alpha": float(alpha),
            "cbf_type": int(CbfType(int(params.CBF_type))),
            "maximum_energy": float(params.K_max),
            "direction": vector_list(params.direction, 3, "direction"),
            "direction_dot": vector_list(getattr(params, "direction_dot", np.zeros(3)), 3, "direction_dot"),
            "apply_cbf": bool(params.apply_cbf),
            "cbf_warmup_time": float(getattr(params, "cbf_warmup_time", 0.2)),
            "abort_on_cbf_infeasible": bool(getattr(params, "abort_on_cbf_infeasible", True)),
            "hold_current_pose": bool(getattr(params, "hold_current_pose", True)),
            "target_transition_duration": float(getattr(params, "target_transition_duration", 5.0)),
            "torque_ramp_duration": float(getattr(params, "torque_ramp_duration", 1.0)),
            "desired_position": vector_list(controller.x_desired, 3, "x_desired"),
            "desired_rotation": [float(item) for item in np.asarray(controller.xmat_desired, dtype=float).reshape(9)],
            "translational_stiffness": matrix_diagonal(controller.K_cartesian, 3, "K_cartesian"),
            "rotational_stiffness": matrix_diagonal(controller.Kr_cartesian, 3, "Kr_cartesian"),
            "translational_damping": matrix_diagonal(controller.D_cartesian, 3, "D_cartesian"),
            "rotational_damping": matrix_diagonal(controller.Dr_cartesian, 3, "Dr_cartesian"),
            "nullspace_stiffness": float(getattr(controller, "nullspace_stiffness", 0.0)),
            "nullspace_damping": float(getattr(controller, "nullspace_damping", 0.0)),
            "tau_abs_limits": vector_list(params.tau_abs_limits, 7, "tau_abs_limits"),
            "max_torque_rate": vector_list(params.max_torque_rate, 7, "max_torque_rate"),
            "stop_joint_damping": vector_list(params.stop_joint_damping, 7, "stop_joint_damping"),
            "derivative_filter_tau": float(getattr(params, "derivative_filter_tau", 0.01)),
            "diagnostics_rate": float(getattr(params, "diagnostics_rate", 100.0)),
        }
    }


def write_trial_yaml(params: Any, alpha: float) -> Path:
    """Write one temporary ROS YAML file and return its path."""
    file = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", prefix="ecbf_controller_", delete=False)
    with file:
        yaml.safe_dump(controller_parameters(params, alpha), file, sort_keys=False)
    return Path(file.name)


# ---------------------------------------------------------------------------
# ROS launch execution
# ---------------------------------------------------------------------------

def launch_trial(params: Any, alpha: float, index: int) -> dict[str, Any]:
    """Launch one blocking ROS 1 hardware trial using a generated controller YAML file."""
    roslaunch = require_executable("roslaunch")
    config_file = write_trial_yaml(params, alpha)
    logfile = alpha_log_filename(params.log_filename, alpha, index)
    logfile.parent.mkdir(parents=True, exist_ok=True)
    command = [
        roslaunch,
        "ecbf_franka_controller",
        "experiment.launch",
        f"robot_ip:={params.robot_ip}",
        f"robot:={params.robot}",
        f"arm_id:={params.arm_id}",
        f"load_gripper:={str(bool(params.load_gripper)).lower()}",
        f"controller_name:={params.controller_name}",
        f"config_file:={config_file}",
        f"log:={str(bool(params.log)).lower()}",
        f"log_file:={logfile}",
    ]
    try:
        completed = subprocess.run(command, check=False)
    except OSError as error:
        raise RuntimeError(f"Unable to start roslaunch: {error}") from error
    finally:
        config_file.unlink(missing_ok=True)
    if completed.returncode != 0:
        raise RuntimeError(f"ROS launch failed for alpha={alpha:g} with return code {completed.returncode}.")
    return {"alpha": alpha, "logfile": str(logfile), "returncode": completed.returncode}


def run(params: Any, debug: bool = False) -> dict[str, Any]:
    """Run one physical ROS 1 controller session for every configured alpha value."""
    del debug
    if not bool(getattr(params, "execute", False)):
        raise RuntimeError("Real-robot execution is disabled. Review the config and set params.execute=True.")
    if os.environ.get("ROS_VERSION") not in (None, "1"):
        raise RuntimeError("The active ROS environment is not ROS 1.")

    alphas = np.asarray(getattr(params, "alphas", [-1.0]), dtype=float).reshape(-1)
    if alphas.size == 0 or not np.all(np.isfinite(alphas)):
        raise ValueError("params.alphas must contain at least one finite value.")

    results: list[dict[str, Any]] = []
    print(f"Configured ROS 1 alpha trials: {[float(alpha) for alpha in alphas]}")
    for index, alpha_value in enumerate(alphas):
        alpha = float(alpha_value)
        if index > 0 and bool(getattr(params, "pause_between_alphas", True)):
            input("Reposition the robot if required, then press Enter to start the next alpha trial...")
        state = "disabled" if alpha < 0.0 else ("applied" if params.apply_cbf else "observation")
        print(f"Starting ROS 1 trial {index + 1}/{alphas.size}: alpha={alpha:g}, CBF={state}")
        trial_params = copy.deepcopy(params)
        results.append(launch_trial(trial_params, alpha, index))
    return {"trials": results}
