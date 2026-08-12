# My Robot Description

A simulated differential-drive mobile robot with a 16-line 3D LiDAR, built in ROS 2 Jazzy and Gazebo Harmonic.

## Overview

This project models a mobile robot from scratch — mechanical structure (URDF),
physics (mass/inertia, collision), differential-drive control, and a
16-beam 3D LiDAR sensor — and simulates it in Gazebo with real-time
point cloud visualization in RViz.

Built as part of self-directed ROS 2 study, guided by Prof. Liu Shuang's
suggestion to start with ROS/Linux and simulate a robot with multi-line
LiDAR and path planning.

## Features

- Custom URDF: chassis, two driven wheels, 16-line LiDAR mount
- Correct mass/inertia (computed from geometry, not guessed)
- Differential drive plugin (responds to `/cmd_vel`, publishes `/odom`)
- 16-line GPU LiDAR (Velodyne VLP-16 spec: ±15°, 360° horizontal, 10Hz)
  - publishes both `sensor_msgs/LaserScan` (`/scan`) and
    `sensor_msgs/PointCloud2` (`/scan/points`)
- Custom Gazebo world with obstacles for LiDAR testing
- Full TF tree, keyboard teleop support

## Tech Stack

ROS 2 Jazzy · Gazebo Harmonic (gz-sim) · URDF · ros_gz_bridge · RViz2

## Demo

![Demo](demo.gif)

Robot navigating in Gazebo with real-time 16-line LiDAR point cloud visualization in RViz2.

## Run it

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
ros2 launch my_robot_description gazebo.launch.py
```

Teleop in another terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

View the point cloud in RViz2:
- Fixed Frame: `base_link`
- Add → PointCloud2 → Topic: `/scan/points`
- Add → RobotModel → Description Topic: `/robot_description`

## What I learned building this

- URDF link/joint structure and how it auto-generates the TF tree
- Why collision geometry needs clearance at joints (rigid constraint
  vs. contact separation force conflict)
- Debugging a full simulation pipeline: sensor plugin → Gazebo topic →
  ros_gz_bridge → ROS topic, isolating exactly where data flow breaks
- Gazebo's GPU LiDAR publishes LaserScan by default; PointCloud2 requires
  the separate `/scan/points` topic
- `gz_frame_id` for aligning Gazebo sensor frames with URDF link names
