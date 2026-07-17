"""Gazebo FR3 directional CBF experiment."""

from types import SimpleNamespace
import numpy as np

from mypkg.constants import CbfType


params = SimpleNamespace(

    mode="gazebo",

    robot="fr3",
    arm_id="fr3",

    controller_name="ecbf_controller",

    alphas=[-1.0],

    runtime=8.0,

    CBF_type=CbfType.DIRECTIONAL,

    direction=np.array([1.0,0.0,0.0]),
    direction_dot=np.zeros(3),

    K_max=0.05,

    apply_cbf=False,

    hold_current_pose=False,

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

        xmat_desired=np.eye(3),

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