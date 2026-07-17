# Repository tree with implementation language

```text
ecbf_redundant_ros1/
├── run/                                      [PYTHON]
│   ├── main.py                               unified configuration selector
│   ├── config_dirfix.py                      MuJoCo directional CBF
│   ├── config_tot.py                         MuJoCo total-energy CBF
│   ├── config_op.py                          MuJoCo operational-energy CBF
│   ├── config_exp_dirfix.py                  ROS 1 directional experiment
│   ├── config_exp_tot.py                     ROS 1 total-energy experiment
│   ├── config_exp_op.py                      ROS 1 operational experiment
│   └── plot.py                               offline plotting
├── src/mypkg/                                [PYTHON]
│   ├── simulation.py                         MuJoCo control loop
│   ├── buffer.py                             MuJoCo state and derived dynamics
│   ├── control.py                            Python nominal controller
│   ├── cbf.py                                Python CBF implementation
│   ├── kinematics.py                         MuJoCo inverse kinematics
│   ├── logger.py                             simulation CSV writer
│   ├── plotting.py                           offline plots
│   ├── ros1_experiment.py                    ROS launch orchestration only
│   └── constants.py                          shared enum values
├── envsim/                                   [MUJOCO XML]
│   └── fr3-model/
├── tests/python/                             [PYTHON TESTS]
├── ros1_ws/
│   ├── src/CMakeLists.txt                     [CATKIN WORKSPACE ENTRY]
│   └── src/ecbf_franka_controller/           [ROS 1 CATKIN PACKAGE]
│       ├── include/ecbf_franka_controller/   [C++ HEADERS]
│       ├── src/                              [C++ REAL-TIME CODE]
│       ├── msg/                              [ROS MESSAGE]
│       ├── config/                           [ROS YAML]
│       ├── launch/                           [ROS LAUNCH]
│       ├── scripts/                          [PYTHON ROS HELPER NODES]
│       └── test/                             [C++ GTEST]
├── docs/                                     [DOCUMENTATION]
│   ├── ARCHITECTURE.md
│   ├── ROS1_SETUP.md
│   ├── REPOSITORY_TREE.md
│   └── VALIDATION.md
├── scripts/                                  [SHELL HELPERS]
└── output/                                   [GENERATED DATA]
```

The physical torque loop never calls Python. Python starts and supervises the ROS process, while the C++ plugin owns every 1 kHz operation.
