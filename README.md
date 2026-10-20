# LiDAR Wall-Following Controller (C++, ROS 2)

A real-time C++ ROS 2 node that drives a 1/10-scale autonomous race car along a wall using a 2D LiDAR. Built for my university's F1TENTH (RoboRacer) team.

## What it does

- Subscribes to `/scan` (LiDAR) and publishes steering and speed commands to `/drive` (Ackermann).
- Uses two LiDAR beams to estimate the car's distance and angle to the left wall, then projects the distance 1 m ahead so it corrects steering before error builds up.
- Holds the car 1 m from the wall and adapts speed from 1 to 3 m/s based on how hard it's steering.

## Key details

- **Sensor filtering:** ignores NaN, infinite and out-of-range readings, and maps beam angles to scan indices safely.
- **Doorway detection:** cross-checks a front beam and a 20° lookahead beam so open doorways don't cause the car to swerve.
- **PID control:** uses the time between scans (from message timestamps) for the integral and derivative terms, with anti-windup (deadzone reset and an integral clamp) and steering limits.
- **Debugging:** throttled logs of front range, lookahead range, error, integral and dt for live tuning.

## Run it

```bash
colcon build --packages-select wall_follow
source install/setup.bash
ros2 run wall_follow wall_follow_node
```

Tested in the F1TENTH simulator on the Levine map.

## Tech

C++17 · ROS 2 Humble · rclcpp · sensor_msgs/LaserScan · ackermann_msgs
