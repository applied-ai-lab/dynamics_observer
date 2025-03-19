# Dyanmics Observer

Author [Alexander Mitchell](https://github.com/mitch722) (2025)

## 0. Overview

This package estiamtes useful dynamics quantities such as end-effector forces, end-effector velocities.

## 1. Setup / Install

This package depends on:

```bash
eigen
pinocchio
```
as detailed in the package.xml.

### CMake Build

If not using ROS, you can build the core package using CMake3:

```bash
cd dynamics_estimator_core
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

### ROS Build (Catkin)

To build with catkin, drop into your catkin workspace and build as normal.

```bash
catkin build dynamics_estimator_ros
```

This will build the core package first and then the ros package.


## 2. Run Examples

To run an example:

```bash
roslaunch dyanmics_estiamtor_ros observer.launch
```


