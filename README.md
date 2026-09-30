# Duckiebot Wheel Odometry

This workspace is a Duckietown ROS project for learning wheel-encoder odometry:
estimating a Duckiebot's pose (x, y, θ) over time from its left and right wheel
encoder ticks.

## What's here

- [notebooks/odometry_activity.ipynb](notebooks/odometry_activity.ipynb) – the
  activity. Walks through reading wheel encoder messages, converting ticks to
  wheel rotation and distance, and computing the robot's change in pose.
- [packages/odometry/](packages/odometry/) – the ROS package.
  - `src/odom_test_pub.py` – publishes fake wheel ticks on
    `left_wheel_encoder_driver_node/tick` and `right_wheel_encoder_driver_node/tick`
    that drive the robot along a known path, so you can test without a Duckiebot.
  - `src/odom_graph.py` – subscribes to `pose` (`geometry_msgs/Pose2D`) and plots
    the path it receives.
  - `launch/odometry.launch` – starts the nodes above. Add your own odometry node
    here: it should subscribe to the wheel tick topics and publish `pose`.
- [launchers/](launchers/) – entry points for the Docker image:
  - `odom_test.sh` – runs with the fake tick publisher (`test:=true`).
  - `robot_odom.sh` – runs on a real Duckiebot, using its encoders (`test:=false`).

## Running

Build and run with the Duckietown shell:

```bash
# Test locally with fake wheel ticks
dts devel build -f
dts devel run -L odom_test

# Run on a Duckiebot
dts devel build -H DUCKIEBOT_NAME -f
dts devel run -H DUCKIEBOT_NAME -L robot_odom
```
