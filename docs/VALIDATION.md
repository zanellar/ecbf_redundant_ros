# Validation and deployment checklist

## Automated checks supplied with the repository

Run the Python checks from the repository root:

```bash
PYTHONPATH=src pytest -q tests/python
```

The suite covers:

- total-energy CBF activation, disable behavior, and infeasible bounds;
- Cartesian impedance force mapping and SO(3) orientation errors;
- Python configuration to ROS controller YAML conversion;
- documentation and small-call layout conventions;
- optional MuJoCo model and dynamics smoke tests when MuJoCo is installed.

Build and run the C++ tests in a sourced ROS 1 workspace:

```bash
source /opt/ros/noetic/setup.bash
cd ros1_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_ecbf_franka_controller
catkin_test_results
```

The C++ tests cover the bounded CBF projection and the SO(3) orientation-error sign convention.

## Hardware validation sequence

1. Verify the installed `franka_ros`, `libfranka`, robot system image, and Ubuntu versions are compatible.
2. Run an official Franka torque-controller example from the same installed release.
3. Confirm the robot has no unmodeled hand or payload, or configure matching load parameters.
4. Keep `execute=False` until the robot IP, frame, gains, limits, and target have been reviewed.
5. Start with `hold_current_pose=True`, `alphas=[-1.0]`, and `apply_cbf=False`.
6. Confirm joint torque signs, Coriolis compensation, flange pose, and Jacobian convention.
7. Measure worst-case controller timing and communication success on the real-time computer.
8. Use a nonnegative alpha with `apply_cbf=False` to inspect the CBF in observation mode.
9. Apply the CBF only after logged nominal and safe torques have been checked offline.

## Numerical parity checks

At identical stationary joint configurations, compare the following between MuJoCo and ROS:

- flange position and orientation;
- six-by-seven zero Jacobian;
- seven-by-seven mass matrix;
- `J * dq` Cartesian twist;
- total, operational, and directional kinetic energy.

Small discrepancies are expected because the packaged MuJoCo model is a self-contained primitive-geometry model and because the physical robot model includes its configured load and calibration.
