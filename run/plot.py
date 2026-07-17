#!/usr/bin/env python3
"""Plot one simulation or experiment CSV file."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRECTORY = REPOSITORY_ROOT / "src"
if str(SOURCE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SOURCE_DIRECTORY))

from mypkg.plotting import CsvPlotter


def parse_arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the CSV file selected for plotting."""
    parser = argparse.ArgumentParser(description="Plot ECBF CSV diagnostics.")
    parser.add_argument("csv", help="Simulation or ROS experiment CSV file")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    """Create standard energy and CBF plots for the selected CSV file."""
    arguments = parse_arguments(argv)
    plotter = CsvPlotter(arguments.csv)
    plotter.plot_energies()
    plotter.plot_cbf_constraint()
    plotter.show()


if __name__ == "__main__":
    main()
