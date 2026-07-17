# Catkin workspace

This directory contains only source-controlled ROS package files. Generated `build/`, `devel/`, and `install/` directories are excluded.

Build with:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
