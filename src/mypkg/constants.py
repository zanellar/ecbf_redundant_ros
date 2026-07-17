"""Shared constants used by the Python simulation and ROS 1 launcher."""

from enum import IntEnum


class CbfType(IntEnum):
    """Identify the kinetic-energy quantity constrained by the CBF."""

    TOTAL = 0
    OPERATIONAL = 1
    DIRECTIONAL = 2


def normalize_mode(value: object) -> str:
    """Normalize a configuration mode and reject unsupported backends."""
    mode = str(value).strip().lower()
    aliases = {
        "simulation": "simulation",
        "sim": "simulation",
        "mujoco": "simulation",
        "experiment_ros1": "experiment_ros1",
        "ros1": "experiment_ros1",
        "robot": "experiment_ros1",
    }
    if mode not in aliases:
        raise ValueError("params.mode must be 'simulation' or 'experiment_ros1'.")
    return aliases[mode]
