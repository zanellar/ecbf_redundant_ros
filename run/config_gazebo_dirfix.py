"""Gazebo FR3 directional CBF experiment."""

from types import SimpleNamespace
import numpy as np

from mypkg.constants import CbfType


params = SimpleNamespace(

    mode="gazebo",

    robot="fr3",
    arm_id="fr3",
    debug=False,

    load_gripper=False,
    frame="flange",

    pause_between_alphas=False,

    cbf_warmup_time=0.2,
    abort_on_cbf_infeasible=True,
    torque_ramp_duration=1.0,
    derivative_filter_tau=0.01,

    log=True,
    log_filename="output/gazebo_dirfix.csv",

    controller_name="ecbf_controller",

    alphas=[-1.0],

    runtime=5.0,

    CBF_type=CbfType.DIRECTIONAL,

    direction=np.array([1.0,0.0,0.0]),
    direction_dot=np.zeros(3),

    K_max=0.05,

    apply_cbf=False,

    hold_current_pose=True,
    target_transition_duration=5.0,

    tau_abs_limits=np.array(
        [20,20,20,20,8,8,8]
    ),

    max_torque_rate=np.ones(7)*500,

    stop_joint_damping=np.ones(7)*5,

    diagnostics_rate=100.0,


    nomctrl_params=SimpleNamespace(

        x_desired=np.array(
            [0.45,-0.2,0.46]
        ),

        xmat_desired=np.array(
            [
                [np.sqrt(2.0) / 2.0, np.sqrt(2.0) / 2.0, 0.0],
                [np.sqrt(2.0) / 2.0, -np.sqrt(2.0) / 2.0, 0.0],
                [0.0, 0.0, -1.0],
            ]
        ),

        K_cartesian=np.diag(
            [100,100,100]
        ),

        Kr_cartesian=np.diag(
            [10,10,10]
        ),

        D_cartesian=np.diag(
            [50,50,50]
        ),

        Dr_cartesian=np.diag(
            [2,2,2]
        ),

        nullspace_stiffness=0.0,
        nullspace_damping=0.0,
    ),
)