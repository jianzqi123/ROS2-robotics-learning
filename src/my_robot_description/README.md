# My Robot Description

A differential-drive mobile robot with a 16-beam 3D LiDAR, built from scratch in
ROS 2 Jazzy and Gazebo Harmonic, running 2D SLAM with `slam_toolbox`.

## Overview

This project models a mobile robot end to end — mechanical structure (URDF),
physics (mass/inertia, collision, ground contact), differential-drive control,
a 16-beam 3D LiDAR — simulates it in a purpose-built indoor world, and closes
the loop by producing an occupancy grid map with `slam_toolbox`.

Built as part of self-directed ROS 2 study, guided by Prof. Liu Shuang's
suggestion to start with ROS/Linux and work up to a multi-beam LiDAR robot
with path planning.

The emphasis throughout has been on **verifying rather than assuming**: every
geometric constant in the URDF and world file is derived, cross-checked, and
documented inline. Several of the bugs listed at the bottom were found by
hand-checking numbers that looked plausible but weren't.

## Features

- **Custom URDF**: chassis, two driven wheels, two passive casters, LiDAR mount
- **Derived mass/inertia** — computed from geometry, not guessed
  (wheel `izz = ½mr²`, `ixx = iyy = (1/12)m(3r² + h²)`)
- **Differential drive** via `gz-sim-diff-drive-system`, consuming `/cmd_vel`,
  publishing `/odom` and the `odom → base_link` TF
- **16-beam GPU LiDAR** (VLP-16-like: ±15° vertical, 360° horizontal, 10 Hz)
  publishing both `sensor_msgs/LaserScan` (`/scan`) and
  `sensor_msgs/PointCloud2` (`/scan/points`)
- **Purpose-built indoor world**: 10 m × 8 m walled room with internal walls,
  plus objects at deliberately chosen heights to exercise the 3D LiDAR
- **2D SLAM** with `slam_toolbox` (async, Ceres solver, loop closure enabled)
- **Saved occupancy grid** ready for Nav2 (`maps/my_map.yaml` + `.pgm`)
- **Automated physics diagnostic** (`scripts/teleport_test.py`) that drives a
  fixed motion sequence and compares wheel odometry against Gazebo ground truth

## Tech Stack

ROS 2 Jazzy · Gazebo Harmonic (gz-sim) · URDF/SDF · `ros_gz_bridge` ·
`slam_toolbox` · `nav2_map_server` · `nav2_lifecycle_manager` · RViz2

## Demo

![SLAM mapping run](demo_map.gif)

A teleoperated mapping run, seen top-down in RViz: 5 min 20 s of simulated time
from an empty grid to the finished occupancy map, with the live `/scan` points
spraying out ahead of the robot as it goes.

The final frame is worth pausing on, because it shows the sensor's limits as
plainly as its coverage. The two 1 m boxes and the shelf fill in solid. The
0.25 m `platform` resolves as a **hollow outline**. `table1` leaves only a
scatter of **leg dots** where its top should be. `overhead_beam` never appears
**at all**. Each of those is a prediction made from the LiDAR's vertical
geometry before the map existed — see [Results](#the-2d-map-is-a-slice-and-it-shows)
for the numbers behind them.

An [earlier build](demo_lidar.gif) — before the room world and SLAM — shows the
same robot in the original obstacle world with the raw 16-beam point cloud in
RViz.

## Repository Layout

```
my_robot_description/
├── urdf/my_robot.urdf          # robot model + DiffDrive plugin + LiDAR sensor
├── worlds/my_world.sdf          # 10×8 m indoor room with 3D structure
├── launch/
│   ├── gazebo.launch.py         # Gazebo + robot spawn + ros_gz_bridge
│   └── slam.launch.py           # slam_toolbox + lifecycle manager + RViz
├── config/slam_params.yaml      # slam_toolbox tuning
├── rviz/slam.rviz               # RViz layout (map / scan / TF / robot)
├── maps/                        # saved occupancy grid
│   ├── my_map.yaml
│   └── my_map.pgm
└── scripts/teleport_test.py     # automated physics-stability diagnostic
```

## Run it

Build:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

Terminal 1 — simulation:

```bash
ros2 launch my_robot_description gazebo.launch.py
```

Terminal 2 — SLAM (also opens RViz):

```bash
ros2 launch my_robot_description slam.launch.py
```

Terminal 3 — teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Press `x` nine times to bring the speed down from 0.50 to ≈0.19 m/s before
driving. Follow the perimeter first, then the interior, then return near the
origin so `slam_toolbox` can close the loop.

Save the map when coverage looks complete:

```bash
# Nav2 occupancy grid — this is what Nav2 consumes later
ros2 run nav2_map_server map_saver_cli -f src/my_robot_description/maps/my_map \
  --ros-args -p save_map_timeout:=10000.0 -p use_sim_time:=true

# slam_toolbox pose graph — for resuming mapping or relocalizing.
# Not committed: ~23 MB binary. Regenerate with this command when needed.
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/$USER/maps/my_map_posegraph'}"
```

## Results

### Odometry accuracy

`scripts/teleport_test.py` drives a fixed 34 s sequence (straight runs, hard
stops, in-place rotation, an arc, a hard acceleration) and compares wheel
odometry against Gazebo's ground-truth pose:

| Metric | Theoretical | Measured |
|---|---|---|
| `/odom` messages (50 Hz × 34 s) | 1700 | 1702 |
| Path length | 3.90 m | **3.90 m** |
| Net displacement error | — | 0.042 m (**1.08 %** of path) |
| Net heading error | — | 0.2° (**0.15 %** of 129.4° net rotation) |
| Teleport events | — | 0 |

Path length matching exactly means the simulation dropped no frames and ran at
full real-time. The displacement check compares **vectors**, not distances — a
reversed drive direction has the same magnitude and would otherwise pass.

### Map quality

The saved grid is 203 × 163 cells at 0.05 m/px:

| Metric | Value | Ground truth |
|---|---|---|
| Map extent | 10.15 m × 8.15 m | 10 m × 8 m interior + 0.15 m wall |
| Aspect ratio | 1.2454 | 1.2500 (0.37 % error) |
| Wall completeness | 100 % on all four walls | — |
| Scale anisotropy | 0.00 % | — |

`origin: [-5.061, -4.066, 0]` puts the grid edges at ±5.089 / ±4.084 — exactly
one wall thickness outside the ±5.0 / ±4.0 interior, which is where the LiDAR
sees the inner wall surface. Zero scale anisotropy indicates loop closure
converged; a map with unclosed drift stretches along the travel direction.

### The 2D map is a slice, and it shows

The objects in the world were placed at heights chosen to test exactly this.
Measured occupancy in each object's footprint:

| Object | Height | Predicted in 2D map | Measured |
|---|---|---|---|
| `box1` (control) | 1.0 m | full | 50.6 % |
| `box2` (control) | 1.0 m | full | 71.1 % |
| `overhead_beam` | 1.1–1.3 m | **absent** | **0.0 %** |
| `table1` | top at 0.73 m | **legs only** | **4.1 %** |
| `platform` | 0.25 m | **outline only** | **11.8 %** |

The beam passes under the overhead beam, under the tabletop but through the
legs, and — because of the 1° tilt described below — over the top of the low
platform beyond 3.72 m, leaving a hollow outline rather than a filled box.
These are not gaps in coverage; they are the predicted consequences of sampling
the world at one height.

## Design Notes

Every number below is derived rather than tuned by trial and error. The inline
comments in `my_robot.urdf` and `my_world.sdf` carry the same reasoning.

### Wheel joint axis

The wheel joints use `rpy="1.5708 0 0"`, which maps the child frame's `z` axis
onto the parent's **−Y**. Differential drive expects a wheel spinning about
**+Y** to move the robot forward (for a wheel of radius `R` spinning at `ω`
about +Y, the contact point constraint gives `v = ωR`). The axis is therefore
declared as:

```xml
<axis xyz="0 0 -1"/>
```

Getting this sign wrong is quietly destructive: the robot drives backwards
*and* the odometry agrees with itself the whole time, so nothing looks broken
until you compare against ground truth.

### Ground contact: three-point, not four

Two driven wheels alone let the chassis pitch about the wheel axle until a
corner of the chassis box scrapes the floor. Two spherical casters fix that,
but their height matters:

| | value |
|---|---|
| Wheel contact plane | `z = −0.100 m` |
| Caster lowest point | `z = −0.095 m` |
| Clearance | **5 mm** |
| Pitch before a caster touches | `atan(0.005 / 0.15) = 1.91°` |
| Pitch before the chassis corner touches | `atan(0.05 / 0.20) = 14.04°` |

The casters sit 5 mm **above** the wheel contact plane on purpose. If they were
coplanar with the wheels, normal-load distribution across four contact points
would be statically indeterminate; the casters (with friction zeroed) would
absorb load that the driven wheels need for traction and lateral constraint,
and the robot would understeer badly. Sitting 5 mm high, they contribute
nothing in normal driving and only catch the chassis at 1.91° of pitch — well
before the 14.04° that would put a chassis corner on the floor.

Caster friction is set to zero (`mu1 = mu2 = 0`) because they are attached with
`fixed` joints and cannot roll; any friction there would fight every turn.

### LiDAR vertical geometry

The LiDAR sits at **z = 0.185 m** above the floor (base_link origin is 0.100 m
up, LiDAR mount adds 0.085 m). With a ±15° vertical spread, the vertical
coverage at range `r` is approximately `0.185 ± 0.268·r` — about `[0, 0.99] m`
at 3 m and `[0, 1.53] m` at 5 m. The world's objects are sized against that
envelope (see the Results table above for measured outcomes).

### `/scan` is not perfectly horizontal — and can't be, with 16 beams

This one is worth stating explicitly because it is invisible until you go
looking for it.

With `samples = 16` over `[−15°, +15°]`, gz-sensors computes the step as
`(vMax − vMin) / (vSamples − 1) = 30° / 15 = 2°`, placing beams at
−15°, −13°, … , −1°, **+1°**, … , +15°. **An even beam count means no beam
lies exactly on the horizon.**

`ros_gz_bridge` flattens the 3D scan into a 2D `LaserScan` by taking the middle
vertical row — `start = (vertical_count / 2) * count`, i.e. integer index 8,
which is the **+1.000°** beam. So `/scan`, the topic `slam_toolbox` actually
consumes, is tilted 1° upward.

Consequences:

- Beam height above the floor is `0.185 + r·tan(1°)` — 0.22 m at 2 m, 0.36 m at
  10 m. Against 1.5 m walls this is harmless.
- The 0.25 m platform is detected only within **3.72 m**
  (`(0.25 − 0.185) / tan(1°)`); past that the beam passes over it. This is the
  measured 11.8 % hollow outline in the Results table.
- If the chassis pitches nose-down by the caster limit of 1.91°, the effective
  elevation becomes −0.91° and the beam reaches the floor at ~11.7 m — within
  the room's ~12.8 m diagonal. Braking hard at long range can inject spurious
  floor returns.

Fixes, if an exactly horizontal `/scan` is wanted: use an odd beam count
(15 beams → step 30°/14, middle index 7 lands on exactly 0°), shift the range
asymmetrically (−16°/+14° keeps 16 beams at 2° spacing with index 8 on 0°), or
add a separate dedicated 2D LiDAR link for SLAM and keep the 16-beam sensor for
`/scan/points`. The last option is what real robots usually do.

### World layout

Interior is 10 m × 8 m (`x ∈ [−5, 5]`, `y ∈ [−4, 4]`), walls 0.15 m thick and
1.5 m tall. Wall thickness is deliberate — thin walls are prone to tunnelling
under default contact solver settings. Two internal walls create corridors and
corners so scan matching has distinguishable features, but neither encloses a
region, so the robot can always drive a full circuit back to its start and give
loop closure something to work with. The 1.5 m radius around the spawn point is
kept clear so `teleport_test.py` can run its whole sequence in place (its
trajectory stays within 1.21 m of the origin, with 0.45 m of clearance to the
nearest wall).

## Debugging Log

The parts of this project that took the longest were not the parts that
produced errors.

**A silent sign error in the wheel axis.** Symptom: the robot moved, odometry
was smooth and self-consistent, nothing logged a warning — but the physical
motion in Gazebo disagreed with `/odom`. A joint's `rpy` rotates its `<axis>`,
so the axis has to be declared in the *child* frame after that rotation. This
led directly to the ground-truth comparison in `teleport_test.py`: odometry
that is internally consistent proves nothing, because DiffDrive is a velocity
controller and the wheels reach commanded speed whether or not they have
traction.

**"The robot teleports when it hits a wall" turned out to be two problems, and
neither was physics.** The investigation first ruled out inertia tensors
(checked against the triangle inequality and recomputed from geometry),
collision-geometry interpenetration, and DiffDrive parameter mismatch — all
fine. The free-space diagnostic then came back clean (0 events, 1.08 % error),
which narrowed it to contact. But watching Gazebo and RViz side by side showed
the physical robot sitting calmly against the wall while only the RViz robot
flew off: what was "teleporting" was the *estimate*, not the robot.

The mechanism is the velocity controller again. Blocked by a wall, the wheels
keep spinning at the commanded rate, so `/odom` integrates forward at
0.19 m/s into the wall indefinitely. Real robots have exactly this failure
mode; it is why odometry alone is never trusted.

The second problem was operator error: `w`/`x` adjust linear speed in
`teleop_twist_keyboard` and I had been pressing `w`, running at 2.53 m/s — 13×
the intended speed. At that rate one second of wheel slip produces 2.5 m of
odometry drift, far outside `slam_toolbox`'s default 0.5 m
`correlation_search_space_dimension`, so scan matching could never recover and
the map was corrupted. At 0.19 m/s, a 2-second stall drifts 0.38 m, inside the
window — and scan matching snaps the pose back as soon as the robot backs off.
The practical rule that falls out of this: **stalled contact must stay under
`0.5 / v` seconds** (2.6 s at 0.19 m/s) for SLAM to self-correct.

**Sensor pipeline debugging.** Gazebo's GPU LiDAR publishes `LaserScan` by
default; `PointCloud2` requires the separate `/scan/points` topic. Frame names
between the Gazebo sensor and the URDF link are reconciled with `gz_frame_id`.
Tracing a break in `sensor plugin → gz topic → ros_gz_bridge → ROS topic`
means checking each hop, not guessing at the ends.

**Reading the bridge's source instead of assuming.** The `/scan` tilt described
above was found by reading `ros_gz_bridge`'s `convert_gz_to_ros` for
`LaserScan` and gz-sensors' `VerticalAngleResolution()`, then recomputing the
beam table by hand. No log message would ever have mentioned it — and the
prediction it produced (a hollow platform outline beyond 3.72 m) was later
confirmed in the finished map.

**Undeclared dependencies bite exactly once.** `map_saver_cli` failed with
`Package 'nav2_map_server' not found` at the moment the map was ready to save.
`package.xml` had declared only `rclpy` — everything else had happened to be
installed already. See Known Issues.

## Known Issues / Next Steps

- [ ] **`/joint_states` is not bridged from Gazebo.** The generic
  `joint_state_publisher` node publishes static zeros, so the wheels never
  rotate in RViz and the wheel TFs are decoupled from the simulation. Fix by
  adding `gz-sim-joint-state-publisher-system` to the URDF, bridging the
  resulting topic, and removing the `joint_state_publisher` node so the two
  don't compete. Does not affect SLAM — scan matching depends on
  `lidar_link` (fixed joint), `odom`, and `/scan`, none of which involve wheel
  joint state.
- [ ] **Bridge directions are all bidirectional (`@`).** `/odom`, `/scan`,
  `/scan/points` and `/tf` should be Gazebo→ROS (`[`) and `/cmd_vel` should be
  ROS→Gazebo (`]`). Bridging `/tf` both ways pushes `slam_toolbox`'s
  `map → odom` transform back into Gazebo, which nothing there needs.
- [ ] **`package.xml` declares only `rclpy`** — `rosdep install` will not
  reproduce the environment. Needs `robot_state_publisher`, `ros_gz_sim`,
  `ros_gz_bridge`, `slam_toolbox`, `nav2_lifecycle_manager`, `nav2_map_server`,
  `rviz2`, `teleop_twist_keyboard`.
- [ ] **Decide how to handle the 1° `/scan` tilt** (options above).
- [ ] `map_saver_cli` logs a default `free_thresh` of 0.25 but writes 0.196 to
  the YAML. The YAML value is what AMCL will read — worth knowing before
  tuning localization.
- [x] ~~Record a new demo GIF (the existing one predates the SLAM work).~~
  Done — `demo_map.gif`, see [Demo](#demo).
- [ ] Nav2 integration: AMCL localization against the saved map, then
  autonomous navigation.
