# Architecture

## Repository structure

```text
run/                              Python entry point and configurations
src/mypkg/                        Python simulation and ROS orchestration
envsim/                           Self-contained MuJoCo FR3 torque model
ros1_ws/src/ecbf_franka_controller/
                                  ROS 1 C++ real-time controller package
tests/python/                     Backend-independent Python tests
output/                           Generated CSV data
docs/                             Setup and architecture notes
```

## Simulation path

```text
run/main.py
  -> Python configuration with mode="simulation"
  -> mypkg.simulation
  -> MuJoCo state
  -> SimulationBuffer
  -> Python Cartesian impedance
  -> Python CBF projection
  -> MuJoCo direct torque actuators
  -> Python CSV logger
```

## Physical ROS 1 path

```text
run/main.py
  -> Python configuration with mode="experiment_ros1"
  -> mypkg.ros1_experiment
  -> generated trial YAML
  -> roslaunch
  -> franka_control
  -> EcbfController::update() at 1 kHz
  -> Franka effort command
```

The ROS controller publishes fixed-size diagnostics. File I/O is delegated to `diagnostics_csv_logger.py` outside the real-time callback.

## Shared mathematical convention

The nominal controller produces a residual torque `u`:

```text
u = J_v^T F + J_w^T moment + optional null-space torque
```

The CBF filters `u`. Coriolis compensation is added after the filter:

```text
tau_command = u_safe + coriolis
```

Gravity is not added.

## CBF convention

All energy constraints use:

```text
h = K_max - K
h_dot + alpha h >= 0
```

A negative alpha disables the CBF but does not disable software torque bounds.

The optimization is the Euclidean projection:

```text
minimize 0.5 ||u - u_nominal||^2
subject to a^T u >= rhs
           lower <= u <= upper
```

Because there is one affine CBF constraint, the implementation uses a scalar dual bisection rather than a general QP solver.

## Frame convention

Both implementations order the geometric Jacobian as:

```text
[linear velocity rows]
[angular velocity rows]
```

The vectors are expressed in the MuJoCo world frame or Franka base frame. These frames coincide in the supplied simulation model. The default controlled point is the flange.
