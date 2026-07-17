"""CSV logging utilities for MuJoCo simulations."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np


@dataclass(frozen=True)
class Field:
    """Describe one scalar CSV field and the callback used to obtain its value."""

    name: str
    getter: Optional[Callable[[], Any]]


class SimulationLogger:
    """Write one row per simulation control step without coupling to controller code."""

    def __init__(self, state: Any, filename: str = "output/logfile.csv") -> None:
        """Open the output file and create a stable field list for the supplied state."""
        self.state = state
        self.filename = Path(filename)
        self.filename.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.filename.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.fields = self._build_fields()
        self.writer.writerow([field.name for field in self.fields])

    def __enter__(self) -> "SimulationLogger":
        """Return this logger when used as a context manager."""
        return self

    def __exit__(self, *_: Any) -> None:
        """Close the CSV file when the context manager exits."""
        self.close()

    def close(self) -> None:
        """Flush and close the underlying CSV file."""
        if not self.file.closed:
            self.file.flush()
            self.file.close()

    @staticmethod
    def vector(prefix: str, getter: Callable[[], Any]) -> list[Field]:
        """Expand a one-dimensional vector callback into indexed scalar fields."""
        size = np.asarray(getter()).reshape(-1).size
        return [Field(f"{prefix}_{index}", lambda getter=getter, index=index: np.asarray(getter()).reshape(-1)[index]) for index in range(size)]

    def _build_fields(self) -> list[Field]:
        """Create the complete logging schema once before the simulation starts."""
        state = self.state
        fields = [Field("time", lambda: state.time), Field("alpha", lambda: state.cbf_alpha)]
        fields += self.vector("qpos", lambda: state.joint_positions)
        fields += self.vector("qvel", lambda: state.joint_velocities)
        fields += self.vector("xpos", lambda: state.eef_positions)
        fields += self.vector("xvel", lambda: state.eef_velocities)
        fields += self.vector("SVD_jacobian", lambda: state.SVD_jacobian)

        for control_name in ("unom", "ubias", "usafe", "command"):
            fields += [Field(f"{control_name}_{index}", None) for index in range(state.n_controls)]

        fields += [
            Field("cbf_computation_time", lambda: state.cbf_computation_time),
            Field("barrier_function_value", lambda: state.barrier_function_value),
            Field("cbf_constraint_nom_value", lambda: state.cbf_constraint_nom_value),
            Field("cbf_constraint_safe_value", lambda: state.cbf_constraint_safe_value),
            Field("cbf_correction_norm", lambda: state.cbf_correction_norm),
            Field("cbf_active", lambda: int(state.cbf_active)),
            Field("cbf_feasible", lambda: int(state.cbf_feasible)),
            Field("cbf_status", lambda: state.cbf_status),
            Field("Ktot", lambda: state.kinetic_energy),
            Field("Ktask", lambda: state.kinetic_energy_task),
            Field("Kdir", lambda: state.kinetic_energy_dir),
            Field("Knull", lambda: state.kinetic_energy_null),
            Field("potential_energy", lambda: state.potential_energy),
            Field("velocity_dir", lambda: state.velocity_dir),
        ]

        if state.debug:
            fields += [
                Field("cbf_constraint_safe_value_a_safe", lambda: state.cbf_constraint_safe_value_a_safe),
                Field("cbf_constraint_safe_value_a_nom", lambda: state.cbf_constraint_safe_value_a_nom),
                Field("cbf_constraint_safe_value_b", lambda: state.cbf_constraint_safe_value_b),
                Field("tmp_lambda_dir", lambda: state.tmp_lambda_dir),
                Field("frobenius_norm_reflected_inertia", lambda: state.frobenius_norm_reflected_inertia),
            ]
            fields += self.vector("SVD_jacobian_dot", lambda: state.SVD_jacobian_dot)
            fields += self.vector("SVD_interia_matrix_dot", lambda: state.SVD_interia_matrix_dot)
            fields += self.vector("SVD_reflected_inertia", lambda: state.SVD_reflected_inertia)
        return fields

    def log(self, nominal: np.ndarray, bias: np.ndarray, safe: np.ndarray, command: np.ndarray) -> None:
        """Append one state and control sample to the CSV file."""
        controls = {
            "unom": np.asarray(nominal, dtype=float),
            "ubias": np.asarray(bias, dtype=float),
            "usafe": np.asarray(safe, dtype=float),
            "command": np.asarray(command, dtype=float),
        }
        row: list[Any] = []
        for field in self.fields:
            if field.getter is not None:
                value = field.getter()
                row.append(value.item() if isinstance(value, np.generic) else value)
                continue
            name, index_text = field.name.rsplit("_", 1)
            row.append(float(controls[name][int(index_text)]))
        self.writer.writerow(row)
