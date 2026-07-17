#!/usr/bin/env python3
"""Select one Python configuration and dispatch to simulation or ROS 1."""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Sequence

sys.dont_write_bytecode = True

# Make the repository's src/ package importable when this file is run directly.
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRECTORY = REPOSITORY_ROOT / "src"
if str(SOURCE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SOURCE_DIRECTORY))

from mypkg.constants import normalize_mode


# ---------------------------------------------------------------------------
# Command-line parsing
# ---------------------------------------------------------------------------

def parse_arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse only the selected Python configuration module name."""
    parser = argparse.ArgumentParser(description="Run an ECBF simulation or ROS 1 experiment.")
    parser.add_argument("-c", "--config", required=True, help="Configuration module name, for example config_dirfix")
    return parser.parse_args(argv)


def normalize_module_name(config_name: str) -> str:
    """Convert a simple file-like configuration name into an importable module name."""
    module_name = config_name.strip()
    if module_name.endswith(".py"):
        module_name = module_name[:-3]
    if not module_name:
        raise ValueError("The configuration module name cannot be empty.")
    if "/" in module_name or "\\" in module_name:
        raise ValueError("Pass a module name such as config_dirfix, not a filesystem path.")
    return module_name


def load_configuration(config_name: str) -> tuple[ModuleType, Any]:
    """Import a configuration module from run/ and return its params object."""
    module_name = normalize_module_name(config_name)
    try:
        module = importlib.import_module(module_name)
    except ModuleNotFoundError as error:
        if error.name != module_name:
            raise
        raise ModuleNotFoundError(f"Configuration module {module_name!r} was not found in run/.") from error
    if not hasattr(module, "params"):
        raise AttributeError(f"Configuration module {module_name!r} does not define params.")
    return module, module.params


# ---------------------------------------------------------------------------
# Backend dispatch
# ---------------------------------------------------------------------------

def run_selected_backend(params: Any) -> Any:
    """Run the backend selected by params.mode using lazy imports."""
    mode = normalize_mode(params.mode)
    debug = bool(getattr(params, "debug", False))
    
    if mode == "simulation":
        from mypkg import simulation
        return simulation.run(params, debug=debug)

    if mode == "gazebo":
    from mypkg import gazebo_experiment
    return gazebo_experiment.run(params)

    from mypkg import ros1_experiment
    return ros1_experiment.run(params, debug=debug)


def main(argv: Sequence[str] | None = None) -> Any:
    """Load the selected configuration and execute its requested backend."""
    arguments = parse_arguments(argv)
    module, params = load_configuration(arguments.config)
    mode = normalize_mode(params.mode)
    print(f"Configuration: {module.__name__} | mode: {mode} | debug: {bool(getattr(params, 'debug', False))}")
    return run_selected_backend(params)


if __name__ == "__main__":
    main()
