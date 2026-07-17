"""Directional-energy ROS 1 experiment configuration for an FR3 without a gripper."""

from types import SimpleNamespace

import numpy as np

from mypkg.constants import CbfType


params = SimpleNamespace(
    # Select the ROS 1 launcher and require an explicit execution acknowledgement.
    mode="experiment_ros1",
    debug=False,
    execute=False,

    # Configure the Franka ROS hardware connection.
    robot_ip="172.16.0.2",
    robot="fr3",
    arm_id="fr3",
    load_gripper=False,
    controller_name="ecbf_controller",

    # Match the MuJoCo attachment_site by controlling the physical flange frame.
    frame="flange",

    # Run one independent physical trial per alpha value.
    # A negative alpha disables the CBF exactly as in simulation.
    alphas=[-1.0],
    pause_between_alphas=True,
    runtime=8.0,

    # Configure the selected CBF and whether its output is actually commanded.
    CBF_type=CbfType.DIRECTIONAL,
    direction=np.array([1.0, 0.0, 0.0]),
    direction_dot=np.zeros(3),
    K_max=0.1,
    apply_cbf=False,
    cbf_warmup_time=0.20,
    abort_on_cbf_infeasible=True,

    # Hold the measured activation pose during the first hardware validation.
    hold_current_pose=True,
    target_transition_duration=5.0,
    torque_ramp_duration=1.0,

    # Configure conservative software limits for initial tests.
    tau_abs_limits=np.array([20.0, 20.0, 20.0, 20.0, 8.0, 8.0, 8.0]),
    max_torque_rate=np.full(7, 500.0),
    stop_joint_damping=np.full(7, 5.0),
    derivative_filter_tau=0.01,

    # Configure non-real-time diagnostics and CSV logging.
    log=True,
    log_filename="output/experiment_dirfix.csv",
    diagnostics_rate=100.0,

    # Configure the nominal Cartesian impedance controller.
    nomctrl_params=SimpleNamespace(
        x_desired=np.array([0.45, -0.2, 0.46]),
        xmat_desired=np.array([[np.sqrt(2.0) / 2.0, np.sqrt(2.0) / 2.0, 0.0], [np.sqrt(2.0) / 2.0, -np.sqrt(2.0) / 2.0, 0.0], [0.0, 0.0, -1.0]]),
        K_cartesian=np.diag([100.0, 100.0, 100.0]),
        Kr_cartesian=np.diag([10.0, 10.0, 10.0]),
        D_cartesian=np.diag([50.0, 50.0, 50.0]),
        Dr_cartesian=np.diag([2.0, 2.0, 2.0]),
        nullspace_stiffness=0.0,
        nullspace_damping=0.0,
    ),
)
