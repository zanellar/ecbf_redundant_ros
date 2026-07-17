"""Non-real-time orchestration for ROS 1 FR3 Gazebo CBF experiments."""

from __future__ import annotations

import copy
import os
import subprocess
from pathlib import Path
from typing import Any

import numpy as np

from mypkg.ros1_experiment import (
    alpha_log_filename,
    require_executable,
    write_trial_yaml,
)


def bool_arg(value: object) -> str:
    """Convert a Python value to the lowercase boolean syntax expected by roslaunch."""
    return "true" if bool(value) else "false"


def launch_trial(params: Any, alpha: float, index: int, debug: bool = False) -> dict[str, Any]:
    """Launch one blocking Gazebo trial using a generated controller YAML file."""
    roslaunch = require_executable("roslaunch")
    config_file = write_trial_yaml(params, alpha)
    logfile = alpha_log_filename(params.log_filename, alpha, index)
    logfile.parent.mkdir(parents=True, exist_ok=True)

    command = [
        roslaunch,
        "ecbf_franka_controller",
        "gazebo_experiment.launch",
        f"robot:={getattr(params, 'robot', 'fr3')}",
        f"arm_id:={getattr(params, 'arm_id', 'fr3')}",
        f"use_gripper:={bool_arg(getattr(params, 'load_gripper', False))}",
        f"controller_name:={getattr(params, 'controller_name', 'ecbf_controller')}",
        f"config_file:={config_file}",
        f"log:={bool_arg(getattr(params, 'log', True))}",
        f"log_file:={logfile}",
        f"headless:={bool_arg(getattr(params, 'headless', False))}",
        f"paused:={bool_arg(getattr(params, 'paused', False))}",
        f"rviz:={bool_arg(getattr(params, 'rviz', False))}",
    ]

    world = getattr(params, "world", None)
    if world:
        command.append(f"world:={world}")

    if debug:
        print("Gazebo roslaunch command:")
        print(" ".join(str(item) for item in command))
        print(f"Generated controller YAML: {config_file}")
        print(config_file.read_text(encoding="utf-8"))

    try:
        completed = subprocess.run(command, check=False)
    except KeyboardInterrupt:
        raise
    except OSError as error:
        raise RuntimeError(f"Unable to start Gazebo roslaunch: {error}") from error
    finally:
        config_file.unlink(missing_ok=True)

    if completed.returncode != 0:
        raise RuntimeError(
            f"Gazebo ROS launch failed for alpha={alpha:g} "
            f"with return code {completed.returncode}."
        )

    return {
        "alpha": alpha,
        "logfile": str(logfile),
        "returncode": completed.returncode,
    }


def run(params: Any, debug: bool = False) -> dict[str, Any]:
    """Run one fresh FR3 Gazebo session for every configured alpha value."""
    if os.environ.get("ROS_VERSION") not in (None, "1"):
        raise RuntimeError("The active ROS environment is not ROS 1.")

    alphas = np.asarray(getattr(params, "alphas", [-1.0]), dtype=float).reshape(-1)
    if alphas.size == 0 or not np.all(np.isfinite(alphas)):
        raise ValueError("params.alphas must contain at least one finite value.")

    results: list[dict[str, Any]] = []
    print(f"Configured Gazebo alpha trials: {[float(alpha) for alpha in alphas]}")

    for index, alpha_value in enumerate(alphas):
        alpha = float(alpha_value)

        if index > 0 and bool(getattr(params, "pause_between_alphas", False)):
            input("Press Enter to start the next Gazebo alpha trial...")

        apply_cbf = bool(getattr(params, "apply_cbf", False))
        state = "disabled" if alpha < 0.0 else ("applied" if apply_cbf else "observation")
        print(
            f"Starting Gazebo trial {index + 1}/{alphas.size}: "
            f"alpha={alpha:g}, CBF={state}"
        )

        trial_params = copy.deepcopy(params)
        results.append(launch_trial(trial_params, alpha, index, debug=debug))

    return {"trials": results}
