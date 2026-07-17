# Energy-Limited Control of a Redundant Franka Manipulator

This repository provides two implementations of the same control experiment:

- a **Python/MuJoCo simulation** used for rapid development, numerical inspection, and plotting;
- a **ROS 1 C++ controller plugin** used for the physical FR3 at the 1 kHz Franka control rate.

The nominal controller is Cartesian impedance control. A Control Barrier Function filters the residual joint torque to limit total, operational, or directional kinetic energy.

## Repository boundary

Python is retained where hard real-time behavior is not required:

- configuration files;
- MuJoCo simulation;
- inverse kinematics;
- offline CSV logging and plotting;
- ROS launch orchestration and repeated alpha trials.

C++ is used only where deterministic 1 kHz execution is required:

- ROS `ros_control` plugin;
- Franka state and model interfaces;
- Cartesian impedance torque;
- numerical `J_dot` and `M_dot`;
- CBF projection;
- torque magnitude and torque-rate limits;
- effort commands and real-time-safe diagnostic publication.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the complete data flow and [docs/VALIDATION.md](docs/VALIDATION.md) for the software and hardware validation sequence.

## Important assumptions

The ROS implementation targets **ROS 1 Noetic** and the ROS 1 `franka_ros` interface. ROS Noetic reached end of life in May 2025, so use it only when required by the laboratory stack. The installed `franka_ros`, `libfranka`, FR3 system image, and Ubuntu version must be mutually compatible.

The supplied MuJoCo model is self-contained and uses primitive visual geometry. It preserves the FR3 kinematic chain, link inertias, seven direct torque actuators, and the `attachment_site` at the flange. It intentionally does not reproduce the original mesh visuals.

## Python installation

Create and activate the Python environment:

```bash
conda env create -f environment.yml
conda activate ecbf
pip install -e .
```

Run the Python unit tests:

```bash
PYTHONPATH=src pytest -q tests/python
```

## Simulation

The only command-line argument selects a Python configuration module:

```bash
python run/main.py --config config_dirfix
```

Alternative examples:

```bash
python run/main.py --config config_tot
python run/main.py --config config_op
```

The configuration contains:

```python
mode="simulation"
alphas=[-1.0, 10.0, 5.0]
```

A negative alpha disables the CBF. Every alpha is run from the same reset MuJoCo state and receives a separate CSV file.

## ROS 1 workspace build

Install a compatible ROS 1 `franka_ros` stack first. Then build the included package:

```bash
source /opt/ros/noetic/setup.bash
cd ros1_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

The helper script performs the same local build:

```bash
./scripts/build_ros1.sh
```

## Physical FR3 experiment

Edit `run/config_exp_dirfix.py` and review at least:

- `robot_ip`;
- `frame`;
- desired pose and gains;
- `tau_abs_limits`;
- `max_torque_rate`;
- `K_max` and `alphas`;
- `apply_cbf`;
- `runtime`;
- `execute`.

The safest initial sequence is:

1. Keep `hold_current_pose=True`.
2. Use `alphas=[-1.0]`.
3. Keep `apply_cbf=False`.
4. Set conservative gains and limits.
5. Set `execute=True` only at the robot after reviewing the setup.

After sourcing both ROS and the catkin workspace, run:

```bash
python run/main.py --config config_exp_dirfix
```

For each alpha, Python generates a temporary ROS YAML file and starts:

```text
franka_control
  -> controller_manager
  -> ecbf_franka_controller/EcbfController
  -> diagnostics CSV logger
  -> trial supervisor
```

A negative alpha disables the CBF. For nonnegative alpha values:

- `apply_cbf=False` computes and logs the CBF correction without commanding it;
- `apply_cbf=True` commands the CBF-filtered residual torque.

Each physical alpha is a separate controller session. The launcher can pause between sessions because the physical robot cannot be reset automatically like MuJoCo.

## Torque convention

The controller separates the residual torque from model compensation:

```text
tau_nominal -> CBF -> tau_residual_command -> add Coriolis -> effort command
```

Gravity is not added. The Franka torque interface performs gravity compensation. Torque magnitude and torque-rate limits are imposed on the final effort command and translated into residual-torque bounds before CBF projection.

## Controlled frame

The default real-robot frame is:

```yaml
frame: flange
```

This matches the MuJoCo model's `attachment_site`, located 0.107 m from the origin of link 7. With no gripper mounted or modeled, this is the recommended comparison point.

`load_gripper=False` disables the ROS gripper description and node; it does not physically remove a mounted hand. If a hand or tool remains attached, configure its load in the robot setup and add the same mass, inertia, and controlled-point transform to the simulation before comparing energies.

## Logging

Simulation logging is performed directly by Python.

On the robot, the 1 kHz C++ controller publishes a fixed-size diagnostic message through `realtime_tools::RealtimePublisher`. A separate Python ROS node writes the decimated messages to CSV, so no file I/O occurs in the real-time callback.

Plot one CSV file with:

```bash
python run/plot.py output/simulation_dirfix_trial_01_alpha_m1.csv
```

## Validation before enabling the CBF

Before commanding filtered torque, verify:

1. the flange pose and Jacobian match MuJoCo at several stationary joint configurations;
2. the mass matrix is positive definite and reasonably close to the simulation model;
3. numerical `J_dot` and `M_dot` remain bounded at the intended speed;
4. the Cartesian impedance controller is stable with `alpha < 0`;
5. final commanded torques and rate limits remain conservative;
6. the CBF is feasible and behaves correctly in observation mode;
7. the control computer meets Franka real-time communication requirements.

## Licensing

The project is licensed under Apache-2.0. The ROS controller structure and parts of the nominal Cartesian impedance implementation are adapted from Franka Robotics GmbH's Apache-2.0 `franka_ros` example controller. See [NOTICE](NOTICE).
