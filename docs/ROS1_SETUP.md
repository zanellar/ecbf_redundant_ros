# ROS 1 setup notes

## Supported deployment assumption

The package is written for a ROS 1 `ros_control` Franka stack with:

- ROS Noetic;
- `franka_ros` exposing `FrankaModelInterface` and `FrankaStateInterface`;
- `hardware_interface::EffortJointInterface`;
- an FR3 selected with `robot:=fr3` and `arm_id:=fr3`.

The exact compatible `libfranka` and robot system versions depend on the laboratory robot. Confirm the compatibility matrix before installing or upgrading any component.

## Build

```bash
source /opt/ros/noetic/setup.bash
cd ros1_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Check that ROS can resolve the package:

```bash
rospack find ecbf_franka_controller
```

## Direct launch for debugging

The Python launcher normally generates the YAML file. To launch the static example manually:

```bash
roslaunch ecbf_franka_controller experiment.launch \
  robot_ip:=ROBOT_IP \
  robot:=fr3 \
  arm_id:=fr3 \
  load_gripper:=false \
  config_file:=$(rospack find ecbf_franka_controller)/config/ecbf_controller.yaml \
  log:=true \
  log_file:=/tmp/ecbf_test.csv
```

The static YAML has `alpha: -1.0`, `apply_cbf: false`, and `hold_current_pose: true`.

## Real-time restrictions

Do not add file I/O, console output, parameter lookup, dynamic memory growth, blocking mutexes, service calls, or normal ROS publishers to `EcbfController::update()`.

The controller uses fixed-size Eigen matrices and `realtime_tools::RealtimePublisher`. Non-real-time nodes perform CSV writing and controller shutdown.

## First tests

1. Run the official Franka joint or Cartesian impedance example from the installed `franka_ros` release.
2. Launch this controller with negative alpha and current-pose hold.
3. Verify torque signs, frame selection, and Coriolis convention.
4. Compare logged Jacobian-derived velocity with an independent state estimate.
5. Enable CBF observation mode with a generous energy limit.
6. Apply the CBF only after offline verification.
