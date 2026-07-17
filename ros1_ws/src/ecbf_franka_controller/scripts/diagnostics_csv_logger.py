#!/usr/bin/env python3
"""Write decimated controller diagnostics to CSV outside the real-time loop."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

import rospy
from ecbf_franka_controller.msg import EcbfDiagnostics


class DiagnosticsCsvLogger:
    """Subscribe to fixed-size diagnostics and append one row per message."""

    def __init__(self) -> None:
        """Open the configured file, write the header, and create the subscriber."""
        self.filename = Path(rospy.get_param("~filename", "/tmp/ecbf_diagnostics.csv"))
        self.filename.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.filename.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.header())
        self.subscriber = rospy.Subscriber("diagnostics", EcbfDiagnostics, self.callback, queue_size=200)
        rospy.on_shutdown(self.close)

    @staticmethod
    def indexed(prefix: str, size: int) -> list[str]:
        """Create deterministic indexed column names for one fixed-size vector."""
        return [f"{prefix}_{index}" for index in range(size)]

    @classmethod
    def header(cls) -> list[str]:
        """Return the complete CSV schema matching the callback row order."""
        fields = [
            "time",
            "elapsed",
            "alpha",
            "cbf_type",
            "cbf_enabled",
            "cbf_applied",
            "cbf_active",
            "cbf_feasible",
            "cbf_status",
            "barrier_function_value",
            "cbf_constraint_nom_value",
            "cbf_constraint_safe_value",
            "cbf_correction_norm",
            "Ktot",
            "Ktask",
            "Kselected",
            "velocity_dir",
            "lambda_dir",
        ]
        for prefix, size in (("qpos", 7), ("qvel", 7), ("unom", 7), ("usafe", 7), ("ucommand", 7), ("tau_command", 7), ("coriolis", 7), ("xpos", 3), ("xvel", 6)):
            fields.extend(cls.indexed(prefix, size))
        return fields

    @staticmethod
    def extend(row: list[object], values: Iterable[float]) -> None:
        """Append a numeric iterable to a mutable CSV row."""
        row.extend(float(value) for value in values)

    def callback(self, message: EcbfDiagnostics) -> None:
        """Convert one ROS diagnostic message into the configured CSV row."""
        row: list[object] = [
            message.header.stamp.to_sec(),
            message.elapsed,
            message.alpha,
            message.cbf_type,
            int(message.cbf_enabled),
            int(message.cbf_applied),
            int(message.cbf_active),
            int(message.cbf_feasible),
            message.cbf_status,
            message.barrier_function_value,
            message.cbf_constraint_nom_value,
            message.cbf_constraint_safe_value,
            message.cbf_correction_norm,
            message.total_energy,
            message.operational_energy,
            message.selected_energy,
            message.directional_velocity,
            message.directional_inertia,
        ]
        for values in (message.q, message.dq, message.tau_nominal, message.tau_safe, message.tau_residual_command, message.tau_command, message.coriolis, message.position, message.task_velocity):
            self.extend(row, values)
        self.writer.writerow(row)

    def close(self) -> None:
        """Flush and close the output file once during ROS shutdown."""
        if not self.file.closed:
            self.file.flush()
            self.file.close()


def main() -> None:
    """Initialize the logger node and process diagnostics until shutdown."""
    rospy.init_node("ecbf_diagnostics_csv_logger")
    DiagnosticsCsvLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
