"""Directional-energy MuJoCo simulation configuration."""

from types import SimpleNamespace

import numpy as np

from mypkg.constants import CbfType


params = SimpleNamespace(
    # Select the Python MuJoCo backend.
    mode="simulation",
    debug=True,

    # Select the model, integration period, and simulation duration.
    model_path="envsim/fr3-model/scene_ideal.xml",
    timestep=0.001,
    runtime=5.0,

    # Configure the optional interactive viewer.
    runviewer=True,
    view_params={"distance": 2.0, "azimuth": 200.0, "elevation": -20.0, "lookat": [0.4, 0.0, 0.5]},

    # Configure logging and repeated alpha trials.
    log=True,
    log_filename="output/simulation_dirfix.csv",
    alphas=[-1.0, 10.0, 5.0],

    # Select and parameterize the directional energy CBF.
    CBF_type=CbfType.DIRECTIONAL,
    direction=np.array([1.0, 0.0, 0.0]),
    direction_dot=np.zeros(3),
    K_max=0.1,

    # Select the initial Cartesian pose used by inverse kinematics.
    x_initial=np.array([0.25, -0.2, 0.46]),
    xmat_initial=np.array([[np.sqrt(2.0) / 2.0, np.sqrt(2.0) / 2.0, 0.0], [np.sqrt(2.0) / 2.0, -np.sqrt(2.0) / 2.0, 0.0], [0.0, 0.0, -1.0]]),

    # Configure the nominal Cartesian impedance controller.
    nomctrl_params=SimpleNamespace(
        x_desired=np.array([0.45, -0.2, 0.46]),
        xmat_desired=np.array([[np.sqrt(2.0) / 2.0, np.sqrt(2.0) / 2.0, 0.0], [np.sqrt(2.0) / 2.0, -np.sqrt(2.0) / 2.0, 0.0], [0.0, 0.0, -1.0]]),
        K_cartesian=np.diag([100.0, 100.0, 100.0]),
        Kr_cartesian=np.diag([10.0, 10.0, 10.0]),
        D_cartesian=np.diag([50.0, 50.0, 50.0]),
        Dr_cartesian=np.diag([2.0, 2.0, 2.0]),
        nullspace_stiffness=0.0,
    ),
)
