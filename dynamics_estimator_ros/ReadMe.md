# Dynamics Estimator Ros

Author [Alexander Mitchell](https://github.com/mitch722) (2024)

## 0. Overview

This package takes the joint state and predicts the frame twist of target frame. This target frame is defined in config/dynamics_estimator.yaml

## 1. Testing

To test this code launch the spawner for ros kortex:

```bash
roslaunch kortex_gazebo spawn_kortex_robot.launch
```

and then launch:

```bash
roslaunch dynamics_estimator_ros twist.launch
```
