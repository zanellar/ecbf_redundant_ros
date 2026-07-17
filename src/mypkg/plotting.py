"""Small plotting helpers for simulation and ROS diagnostic CSV files."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


class CsvPlotter:
    """Load one numeric CSV file and expose common ECBF diagnostic plots."""

    def __init__(self, filename: str) -> None:
        """Load the CSV header and all rows into a structured NumPy array."""
        self.filename = Path(filename)
        self.data = np.genfromtxt(self.filename, delimiter=",", names=True, dtype=None, encoding="utf-8")

    def plot_energies(self) -> None:
        """Plot all available kinetic-energy signals against time."""
        figure, axis = plt.subplots()
        for field in ("Ktot", "Ktask", "Kdir", "Knull"):
            if field in self.data.dtype.names:
                axis.plot(self.data["time"], self.data[field], label=field)
        axis.set_xlabel("time [s]")
        axis.set_ylabel("kinetic energy [J]")
        axis.grid(True)
        axis.legend()
        figure.tight_layout()

    def plot_cbf_constraint(self) -> None:
        """Plot nominal and safe CBF inequality values when present."""
        figure, axis = plt.subplots()
        for field in ("cbf_constraint_nom_value", "cbf_constraint_safe_value"):
            if field in self.data.dtype.names:
                axis.plot(self.data["time"], self.data[field], label=field)
        axis.axhline(0.0, linewidth=1.0)
        axis.set_xlabel("time [s]")
        axis.set_ylabel("CBF inequality")
        axis.grid(True)
        axis.legend()
        figure.tight_layout()

    def show(self) -> None:
        """Display all figures created by this plotter."""
        plt.show()
