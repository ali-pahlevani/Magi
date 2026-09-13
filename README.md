# MAGI: Unitree Go2W in the Rubicon world

![MAGI](docs/images/Magi.png)

ROS 2 Humble + Gazebo Sim (Harmonic) simulation of the **Unitree Go2W**, a
wheeled quadruped with 16 actuated joints (12 leg joints + 4 drive wheels),
driving, mapping and navigating the Rubicon terrain. Control is
`ros2_control`-based; state estimation, 3D SLAM and Nav2 are all wired up, and
every claim below is measured against simulator ground truth.

| Rubicon, wide | The Go2W on the terrain |
|---|---|
| ![Rubicon wide view](docs/images/rubicon_world_1.png) | ![Go2W close-up](docs/images/rubicon_world_2.png) |

## Quick start

```bash
ros2 launch magi_launch magi_test.launch.py
```

Starts Gazebo, spawns the robot, activates the controllers, brings up state
estimation and the stance controller, and opens keyboard teleop and RViz. Turn
either window off with `gui:=false` / `rviz:=false`.

To map, then navigate — the second command needs the map the first one saves:

```bash
ros2 launch magi_launch magi_test.launch.py slam:=true   # drive around, Ctrl-C saves
ros2 launch magi_launch magi_nav.launch.py               # then click 2D Goal Pose
```

`magi_bringup` underneath is the simulation stack alone:

```bash
ros2 launch magi_bringup magi_sim.launch.py teleop:=true
ros2 launch magi_control teleop.launch.py          # or teleop in a second terminal
```

Startup is staged (`spawn_delay` / `controllers_delay` / `rviz_delay`), with a
longer stagger when the Gazebo GUI has 342 MB of terrain to build. RViz opens
last on purpose, so `/robot_description` is latched and `odom -> base` is live
by the time it appears.

**The odometry view** (`magi_launch/rviz/magi_odometry.rviz`) is laid out to make
odometry *checkable* rather than pretty. Fixed Frame is `odom` and the camera
deliberately does not follow the robot — tracking `base` would leave the robot
still while the world slid past, hiding exactly what needs watching. Two trails
are drawn: **green** is the EKF's `/odometry/filtered`, **red** is raw
`/wheel_controller/odom`, and the red one swings away the moment you turn:

| over a path with two turns | final displacement | error vs truth |
|---|---|---|
| ground truth | x +3.23, y +0.55 | — |
| EKF (green) | x +2.47, y +2.01 | **1.65 m** |
| wheel raw (red) | x +2.73, y +4.51 | **3.98 m** |

## Packages

| Package | Contents |
|---|---|
| `magi_description` | URDF/xacro, meshes, `ros2_control` interfaces, Gazebo tags, RViz config |
| `magi_gazebo` | Offline Rubicon world + vendored model, heightmap rebuild, flat test world |
| `magi_control` | Controller YAML, spawners, stance/balance/posture nodes, teleop |
| `magi_localization` | EKF + leg odometry + IMU conditioner; owns `odom -> base` |
| `magi_slam` | RTAB-Map 3D lidar SLAM, the map saver, map-quality checks |
| `magi_navigation` | Nav2 on a saved map: costmaps, planner, controller, behaviours |
| `magi_bringup` | Simulation stack: world, robot, controllers, estimation |
| `magi_launch` | Runnable configurations. Start here |
| `third_party/gz_ros2_control` | Built from source — see below |

`src/unitree_go2w_ros2` and `src/Rubicon_World` are the original inputs, kept for
reference. `unitree_go2w_ros2` carries a `COLCON_IGNORE` because its
`go2w_driver` needs `unitree_sdk2`, which is for the physical robot.

## Build

```bash
cd ~/magi_ws
export GZ_VERSION=harmonic          # required, see below
colcon build --symlink-install
source install/setup.bash
```

**After a fresh clone, fetch the world.** It is not in the repo: 342 MB, with
`rubicon.dae` alone well past GitHub's 100 MB per-file limit. The script
downloads and verifies it, re-applies the terrain friction patch and **rebuilds
the heightmap** ([why](#the-heightmap-is-a-staircase-and-it-is-rebuilt)):

```bash
src/magi_gazebo/scripts/fetch_rubicon.sh   # ~187 MB download, once
colcon build --symlink-install
```

**SLAM needs RTAB-Map**, and a newer `diagnostic_updater` than Humble ships —
the installed 4.0.6 lacks `libdiagnostic_updater.so`, which rtabmap 0.23.7 links
against, and the node dies with exit 127 without it:

```bash
sudo apt install ros-humble-rtabmap-ros
sudo apt install --only-upgrade ros-humble-diagnostic-updater
```

**Why `gz_ros2_control` is built from source.** The Humble binary is compiled
against Ignition Fortress and will not load in Gazebo Harmonic (`gz-sim8`). Its
`humble` branch supports Harmonic when `GZ_VERSION=harmonic` is exported at
build time, so it is vendored into `src/third_party/` and built that way. It
installs its own `GZ_SIM_SYSTEM_PLUGIN_PATH` hook; sourcing the workspace is
enough.

---

## Sensors

Modelled on the real Go2-W in
[go2w_sensors.xacro](src/magi_description/urdf/go2w_sensors.xacro), bridged by
[gz_bridge.yaml](src/magi_gazebo/config/gz_bridge.yaml).

| Real hardware | Simulated as | ROS topic | Rate |
|---|---|---|---|
| Livox MID-360 lidar | `gpu_lidar` | `/lidar/points`, `/lidar/scan` | 10 Hz |
| Front HD wide-angle camera | `camera` 1280×720 | `/front_camera/image`, `/front_camera/camera_info` | 30 Hz |
| Body IMU (6-axis + fusion) | `imu` + MEMS noise | `/imu/data_raw` | 200 Hz |
| Foot force sensors ×4 | `force_torque` on each wheel joint | `/foot_force/{FL,FR,RL,RR}` | 200 Hz |
| UWB positioning | not modelled | — | — |
| **no GPS** | **deliberately absent** | — | — |

Checked against the datasheets, not just "it publishes": exactly **23,040
points** per scan (720 × 32 ≈ 200k points/s, the real MID-360 figure) over
360° × −7…+52°, 0.1–40 m, σ = 2 cm; a **120.0°** camera FOV in the REP-103
convention; an IMU reading 9.8203 m/s² level with the noise model visibly active;
and foot forces that independently reproduce the load asymmetry in the calf
torques. Three caveats:

* **The lidar scan pattern is not authentic.** The MID-360 scans a
  non-repetitive rosette; `gpu_lidar` only does a uniform raster. FOV, range,
  rate and point budget are real, the sampling distribution is not — fine for
  generic LIO, not for Livox-native SLAM keyed to scan lines.
* **The camera is a pinhole, not a fisheye.** Unitree publishes no lens model.
* **Foot wrenches are in the calf frame.** The child link is the wheel, spinning
  at ~11.6 rad/s, so a child-frame wrench would rotate with it.

---

## State estimation

A `robot_localization` EKF ([magi_localization](src/magi_localization/)) owns
`odom → base`; `diff_drive_controller` runs with `enable_odom_tf: false` so there
is exactly one publisher.

**The filter exists to fix heading.** The wheels are fixed and non-steerable, so
every turn scrubs them sideways, and wheel odometry — which infers yaw from the
wheel speed difference — cannot see the scrub:

| manoeuvre | ground truth | wheel odom | EKF |
|---|---|---|---|
| spin 0.5 rad/s, 8.8 s | 118.1° | 197.6° (**+67%**) | 124.1° (**+5%**) |
| arc 0.5 m/s + 0.4 rad/s, 7.1 s | 64.2° | 121.9° (**+90%**) | 60.2° (**−6%**) |

Wheel odometry contributes **vx only**. The IMU contributes roll/pitch and all
three rates. Yaw is therefore integrated from the gyro with no absolute
reference — exactly as on the real robot, whose IMU has no magnetometer — and
drifts slowly, which is what `odom` is supposed to do; SLAM corrects it. `vy = 0`
is *not* asserted: this is skid-steer, lateral slip is real, and the constraint
would inject a lie.

### The vertical channel comes from the legs, not the IMU

An accelerometer cannot supply `z`: fused alone, `az` gives the filter something
to integrate and nothing to correct against, so the bias double-integrates.
Measured with the robot **completely stationary**:

| | z after ~40 s |
|---|---|
| IMU only | **−107.96 m** |
| **+ leg kinematics** | **0.010 m** |

The robot never moved; the IMU-only filter "fell" 108 m, its `vz` growing at
0.0979 m/s² against the 0.1 m/s² accelerometer bias configured in the URDF.

`magi_leg_odometry` fixes it the way real quadruped estimators do. Each loaded
wheel is a known point touching the surface, so body height above that surface
is observable as `h = wheel_radius − (R · t_i).z`, with `R` built from roll and
pitch only — the z component is invariant to yaw, which sidesteps the drifting
heading. Measured: **0.3877 m** against a ground-truth 0.3963 m on rough terrain,
and a commanded −0.123 m crouch read back as −0.123 m. It publishes
`/magi/terrain_height`, `/magi/contacts` and `/magi/leg_twist` (fused as
`twist0`).

That `vz` is relative to the *terrain*: climbing a slope at constant ride height
it reads zero while the robot rises, so `odom` cannot track absolute elevation.
What it buys is a measured vertical channel whose error is a slow random walk
rather than an unbounded quadratic divergence.

**Why there is an IMU conditioner.** `gz.msgs.IMU` has no covariance fields, so
the bridged `/imu/data_raw` carries all-zero covariances, which
`robot_localization` reads as infinite confidence — collapsing the filter.
`magi_imu_covariance` republishes it as `/imu/data` with the URDF's actual noise
variances, and gives yaw a variance of 1e6 so nothing mistakes it for a heading.

---

## 3D SLAM

RTAB-Map builds a 3D lidar map and publishes `map -> odom`
([magi_slam](src/magi_slam/)), on top of the EKF's `odom -> base` — one owner per
link.

```bash
ros2 launch magi_launch magi_test.launch.py slam:=true
```

![RTAB-Map mapping](docs/images/rtabMap_mapping.png)

**No `icp_odometry`.** It would publish `odom -> base` and fight the EKF for it.
RTAB-Map consumes `/odometry/filtered` instead and adds only the correction —
full 6-DoF (`Reg/Force3DoF: false`), because EKF yaw drifts and odom z does not
track the terrain's 5 m of relief, and both are what loop closure absorbs.

**Run one `rtabmap` at a time.** Two of them both publish `map -> odom`, and the
symptom is not an error but a z flickering between two values.

### Where z = 0 is, and why the map used to float

RTAB-Map anchors `map` on the pose of its `frame_id` at the first keyframe. Point
it at `base` and z = 0 lands at the body — a whole ride height off the floor.
Invisible in the 3D cloud, glaring in the 2D one: a `nav_msgs/OccupancyGrid`
always has `origin.position.z = 0`, so RViz drew `/map` as a plane 0.35 m in the
air with the wheels hanging below it. The cloud itself was never wrong (ground
returns within **0.03 m** of the true surface). Only the datum was.

Two independent fixes. `magi_leg_odometry` **sets the odom datum once** at
startup, calling the EKF's `/set_pose` to move z = 0 down by the measured body
height (`set_ground_datum:=false` reverts it). And RTAB-Map **anchors on
`base_footprint`**, a rigid link 0.36 m below `base` — rigid rather than tracked,
because a footprint that bobbed with ride height would push that bobbing into
scan registration.

Standing, the 2D grid now sits within **1 mm** of the wheel contacts, with
`map -> odom` z at 0.000. After nine metres of driving it is **38 mm** off:
terrain-relative odom z drifting, for loop closure to absorb.

**Measured** over a 6 s segment across the basin:

| source | distance | error |
|---|---|---|
| ground truth | 1.810 m | — |
| **SLAM (`map -> base`)** | 1.846 m | **+0.036 m** |
| EKF alone (`odom -> base`) | 2.198 m | +0.387 m |

Position error falls about tenfold. Yaw barely improves (−7.3° vs −8.1°), as
expected: a short segment offers no loop closure, which is what corrects heading.

---

## Mapping and navigation

![Autonomous navigation on the saved map](docs/images/auto_nav_1.png)

### Saving the map

RTAB-Map streams its graph into a database as it maps, so quitting **is** the
save. `slam.launch.py` points that database at its final home from the start —
rather than copying it afterwards, which risks capturing a file rtabmap is still
flushing. `magi_map_saver` runs alongside and writes what Nav2 needs. Ctrl-C is
the whole procedure:

```
~/magi_maps/rubicon.db      the RTAB-Map graph      -> relocalisation
~/magi_maps/rubicon.yaml    the 2D grid, + .pgm     -> Nav2's static layer
~/magi_maps/rubicon.ply     the 3D cloud            -> viewing
```

`map_name:=<name>` keeps several maps side by side; to snapshot mid-session,
`ros2 service call /magi_map_saver/save std_srvs/srv/Trigger`.

Shutdown saving broke where the service worked. rclpy's SIGINT handler tears the
context down from inside the handler, and the executor's next wait raises
`RCLError` rather than the documented `ExternalShutdownException` — uncaught, so
the map was silently never written. The node now declines rclpy's signal handling
and treats shutdown as a flag its spin loop reads.

### Navigating

```bash
ros2 launch magi_launch magi_nav.launch.py
```

RTAB-Map in **localization** mode against the saved map, plus Nav2. Click **2D
Goal Pose** in RViz and the robot plans and drives there.

* **No AMCL.** RTAB-Map owns `map -> odom`, localising against the graph that
  built the map; a second publisher is exactly the silent-failure class this
  repo keeps meeting. Respawning at the map origin brings `map -> odom` up at
  4 cm. **2D Pose Estimate** relocalises RTAB-Map if it is ever lost.
* **Regulated Pure Pursuit, not DWB.** DWB scores rollouts assuming the
  commanded twist is executed — but `magi_stabilizer` sits between Nav2 and the
  wheels, scaling commands for roughness and tipover margin. Pure pursuit
  re-reads the pose every cycle, so a governed command just leaves the robot
  further back along the path.

### This lidar cannot see the ground, and it broke two things

The MID-360 sits 0.46 m above the foot plane with its FOV starting at −7°, so it
looks outward and up. Of the 23,040 returns in one scan on Rubicon, **1.6%** land
within 0.3 m of the foot plane, and the nearest is **1.52 m** away.

**It made the occupancy grid unusable.** RTAB-Map marks a cell free only when a
*ground point* lands in it, so almost nothing was free — including where the
robot stood. `Grid/RayTracing: true` infers free space from the beams instead.
Against the 88 poses the robot physically occupied on a 28 m run:

| | ray tracing off | on |
|---|---|---|
| driven cells marked free | 51% | **100%** |
| driven cells marked **occupied** | 34% | **0%** |
| largest drivable region (0.38 m robot) | 1.4 m² | **364 m²** |

The start cell was blocked in its own map, and no costmap tuning could have
helped: the fault was upstream. Checked against the simulator's terrain, the old
grid carried no information at all — "free" and "occupied" cells were the same
ground, 72.8% vs 70.9% genuinely drivable. With ray tracing, occupied cells are
real (90th-percentile true slope 51° against 33°). `Grid/RangeMax` is halved to
10 m alongside, since ray tracing trusts beams that skim a rise, and longer beams
skim more.

**It rules out a live obstacle layer.** A costmap obstacle layer needs ground to
decide what is *not* an obstacle and to raytrace free space open. Through
`rtabmap_util/obstacles_detection`, every setting put 99%+ of the cloud in
"obstacle":

| settings | obstacle pts/scan | ground pts/scan |
|---|---|---|
| noise filter 0.05 m (as first configured) | 5 | 0 |
| no noise filter | 1379 | 0 |
| rtabmap defaults | 2615 | 0 |
| height segmentation, ground below 0.3 m | 3263 | 224 |

Wired in, the robot walled itself in and froze — "collision ahead" on a cell the
saved map called 100% free. Navigation runs on the SLAM map;
`local_obstacles:=true` restores the live layer for anyone who changes the
sensor. (The first row is its own trap: a noise-filter radius below the 0.1 m
voxel size has no neighbours to count, so the node emitted empty clouds —
silently.)

### A 2D map cannot say "too steep", so the saver adds it

With the grid fixed, Nav2 planned a clean path across ground correctly shown as
empty, and the robot rolled over on it. An occupancy grid says *something is
here*; it cannot say *this ground tilts 30°*. The information is in the 3D cloud,
so `magi_map_saver` takes the lowest return per cell as ground, fits the local
gradient over a footprint-sized window, and marks anything too steep as
occupied.

It is a rough estimate — correlation 0.54 with the true heightmap, since the
lidar samples ground sparsely — so the threshold is **calibrated, not derived**:
scored against the five places runs actually rolled the robot, and the poses it
drove without falling.

| limit / window | occupied | driven path drivable | largest region | tip sites blocked |
|---|---|---|---|---|
| none (`slope_layer:=false`) | 2.5% | 94.9% | 313 m² | **0 of 5** |
| 25° / 1.0 m | 29.2% | 67.9% | 66 m² | 5 of 5 |
| **35° / 1.5 m** | 19.3% | 69.2% | **138 m²** | **4 of 5** |
| 40° / 1.5 m | 15.9% | 69.2% | 179 m² | 4 of 5 |

Without the layer, *not one* site that rolled the robot is marked. 35° buys four
of five for twice the area of the strictest setting. (35° is a smoothed estimate,
not the ~25° the robot actually tips at; it is where it is because it scored
best.)

### Does it navigate?

![Navigating a goal across the basin](docs/images/auto_nav2.png)

Fourteen goals sent as `/goal_pose`, the way the RViz button sends them:

| | |
|---|---|
| goals reached | **10 of 14** |
| one uninterrupted sequence | **6 of 6**, robot upright at the end |
| typical goal | 3–7 m, reached in 7–26 s |
| planning / localisation failures | **0 / 0** |

Every failure was the robot tipping or wedging on terrain, never the planner or
the localiser.

---

## Control architecture

`gz_ros2_control` hosts the `controller_manager` inside Gazebo; the launch files
only run spawners.

| Controller | Type | Joints |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | all 16 |
| `imu_sensor_broadcaster` | IMUSensorBroadcaster | body IMU |
| `leg_controller` | JointTrajectoryController | 12 leg joints, **effort** |
| `wheel_controller` | DiffDriveController | 4 wheels, **velocity**, skid-steer |

Each joint declares exactly **one** command interface, because
`GazeboSimSystem::write()` tests `VELOCITY` before `POSITION` before `EFFORT` and
silently ignores all but the first.

**The legs are torque-controlled**, emitting
`tau = p*(q_des − q) + i*∫ + d*(dq_des − dq)` — the law the real Unitree motor
runs from its `MotorCmd`. Gains come from geometry: with the calf's 0.1535 m
lever, `p = 200` gives ~8500 N/m at the wheel, ~5.7 mm of travel under the
per-leg load, matched to the terrain's 6.8 mm mean facet step. Load sharing,
before and after:

| calf effort on Rubicon | rigid position control | torque control |
|---|---|---|
| max/min spread, standing | **201×** (FL effectively airborne) | **10.2×** |
| max/min spread, driving | — | **3.3×** |

### Stance control and the command governor

`magi_stabilizer` decides where the feet go; `magi_balance` (the original
quasi-static controller) and `magi_posture` (one fixed stance) are kept for A/B:

```bash
ros2 launch magi_launch magi_test.launch.py                            # stabilizer
ros2 launch magi_launch magi_test.launch.py stance_controller:=balance # the old one
ros2 launch magi_launch magi_test.launch.py balance:=false             # fixed stance
```

The stabilizer **sits in the command path**. Teleop and Nav2 publish `/cmd_vel`;
it republishes to `/wheel_controller/cmd_vel_unstamped` after projecting the
twist onto the twists the robot can currently survive. A turn at `(v, w)` needs
`v*w` of lateral acceleration, and the available amount is found by bisecting
the measured stability margin, so the envelope shrinks by itself on a side slope,
over a bump, or when a wheel unloads. Both components scale together, preserving
`v/w` — the operator's arc, driven more slowly. Nothing else may publish to the
wheel topic while it runs.

Stability is a **force-angle margin**: how far the net force (effective gravity
from the accelerometer, so centrifugal, braking and terrain terms included) can
rotate before the robot goes over its worst support edge. Unlike "CoP inside the
polygon" it stays defined on two wheels — the moment that matters — and reads in
degrees of remaining tilt. On Rubicon, 8 s per run:

| profile | `magi_balance` | `magi_stabilizer` |
|---|---|---|
| v 0.7 | 2/3 upright, roll 45° | **3/3**, roll 11° |
| v 1.2 | 0/3 upright, roll 137° | **8/8**, roll 9° |
| v 0.9, w 0.5 | 1/3 upright, roll 99° | **3/3**, roll 11° |
| v 1.5, w 1.2 | 1/3 upright, roll 85° | **7/8**, roll 12° |
| **total** | **4/12** | **21/22** |

Not a guarantee: there is no stepping, so a big enough terrain event still wins.

#### Six faults that made it crawl

The design above was right and the implementation was not. As shipped, the robot
covered 1.07 m per 8 s run, splayed at its stance limit and refusing every
command above a crawl. Each fault is documented at its site in
`magi_stabilizer.py` / `.yaml`:

| # | Fault | The measurement that found it |
|---|---|---|
| 1 | Attitude rate term fed to the legs unfiltered | A 19.5 Hz limit cycle standing still; accelerometer at **±200 m/s²** — 20 g on a stationary robot |
| 2 | Effective gravity built as `-down*G`, pointing **up** | Every envelope probe failed; the governor ran on a hard-coded **0.8 m/s²** of the 10.0 available |
| 3 | Terrain preview fit a **plane**, scoring hills as roughness | **57% of the map** read "fully rough", permanently. It fits a quadratic now |
| 4 | Urgency pinned at 1.00 | Permanently splayed and throttled to 0.35 m/s — its top speed everywhere |
| 5 | Widening triggered by ordinary speed | Track above 0.50 m for **98%** of a course; on flat ground, body 7 cm low |
| 6 | The stance was friction-locked | Both front hips **pinned at −23.700 N·m**; the splay never moved at rest |

**(1) was the one everything hung off.** Through the command horizon, the
trajectory controller and the body's ~20 Hz mode on the hip PD there is 180° of
phase well before 20 Hz, so undelayed rate feedback there is gain, not damping.
Every consumer read the accelerometer it was shaking, and the stability margin
reported **−32° on an upright robot**. Filtered, with `kd` at 0.05: **0.007
rad/s rms**.

**(5) was a trade that wasn't one.** Narrowing `widen_max` from 0.110 to 0.070
*raised* the median tip margin from 31.3° to **35.6°**: cambering the wheels 24°
onto their rim edges cost more contact than the geometry bought. On flat ground
at 0.97 m/s the track went from 0.554 m to **0.443 m** and ride height from
0.290 m to **0.346 m**.

**(6) makes the width lever one-way at rest.** Dragging four loaded wheels
sideways costs 17.3 N·m at the hip on top of the ~12 N·m the splay holds, against
a 23.7 N·m limit. Reshaping is now gated on rolling.

And one guard was simply missing: **the pitch axis**. Its tipping angle,
`atan(0.1934/0.311)` = **31.9°** nose-up, has no lever — widening is a roll
remedy — and the robot drove onto steeper faces and went over backwards in three
quarters of a second, with drive torque at 1.1 of 23.7 N·m: climbing something it
should not have, not grinding. `_govern` now guards on pitch and pitch rate.

### Startup and the limp-leg window

Declaring an effort interface makes the legs torque-controlled from the moment
the model appears
([gz_system.cpp:476](src/third_party/gz_ros2_control/src/gz_system.cpp#L476)),
so they are limp until `leg_controller` activates. **They recover:** the robot
dips and the PD stands it back up, settling at the 0.396 m design stance on every
launch measured.

Two wrong turns are worth recording. A constant gravity-hold seed torque
diverges — the stance is an *unstable* equilibrium — and with the signs wrong it
drove every joint into a limit. That was misread as "the robot cannot recover",
which led to a paused start; but activation itself needs physics ticks, so the
unpause raced the controller switch and on a GUI run lost by two seconds.
`paused:=true` remains available; the default is unpaused, simpler, and verified.

---

## Measured behaviour

Ground truth from the Gazebo server. **Always use `--reps` on terrain:** the same
start pose under an identical configuration has produced 2.7 m and 5.4 m, while
flat ground repeats to sd 0.2 — the variance is terrain, not the harness.

`magi_terrain_trial.py` drives a twelve-leg course — nine start poses across
Rubicon with footprint-scale slopes from 3° to 23°, plus arcs and a spin — and
reports **net displacement**, because a robot shaking itself sideways racks up
path without going anywhere. Three passes, 8 s per leg:

| | **net displacement** | path | upright at end |
|---|---|---|---|
| before the fixes above | **1.07 m** | 1.54 m | 28/31 |
| control fixes, 8-bit terrain | **1.52 m** | 1.81 m | 19/21 |
| control fixes + rebuilt terrain | **3.23 m** | 3.68 m | 18/29 |
| …at `ride_height` 0.36 (default) | **2.57 m** | 3.08 m | **9/11** |

Three times the ground covered. The last row is the shipped default, trading
twenty percent of distance for twenty points of upright rate (see
`magi_stabilizer.yaml`). The honest reading of the upright column: the fixed
robot rolls over more *per run* because it now reaches things — most rollovers
were at the two start poses with a boulder within a metre, which the stock robot
was too slow to arrive at. On flat ground it makes **0.98 m/s of a commanded
1.00** with body rates of 0.007 rad/s rms.

```bash
ros2 run magi_control magi_terrain_trial.py --duration 8 --reps 3
ros2 run magi_control magi_drive_benchmark.py 1.0 0.0 3.5 --reps 8 \
    --reset-world rubicon --reset-pose 3.0,-0.5,1.85
```

**Run one stance controller at a time.** A stray `magi_posture` alongside
`magi_stabilizer` produces measurements that look like physics and are not — the
robot apparently moving at 3.5% of command on flat ground, legs limp at exactly
0.00 N·m, until the second publisher turned up.

**The wheel collision shape dominates everything.** Upstream reuses the wheel's
visual mesh as its collision geometry; swapping in the cylinder it describes
(`gen_body.py`) changed behaviour more than any gain or friction value. With the
mesh, a 0.8 rad/s spin splayed the legs and collapsed the body, saturating the
hips at 23.7 N·m; with the cylinder, 1.5 rad/s is clean.

---

## Known limitations

**Navigation ends where the robot falls over.** All four missed goals were the
robot tipping or wedging, and with no self-righting a fall ends the session —
`base_footprint` lands somewhere meaningless and Nav2 reports collisions that are
really a robot on its side. The slope layer blocks four of the five sites that
caused this, but it is an estimate, not a guarantee. **Self-righting is the
obvious next thing to build.**

**Nothing sees an obstacle that is not in the map.** From 0.5 m up with a −7°
lower FOV, the lidar's lowest ray reaches the ground only 2.69 m out, so anything
under ~0.4 m inside that radius is invisible — against ~200 rock and stump
colliders and 34 tree trunks. The world is static and mapped, so it costs nothing
here and would cost everything around anything that moves. The proper fix is a
traversability estimator, not a costmap parameter.

**Turning is scrub-limited.** Yaw authority is ~78% on flat and ~47% on terrain,
and wheel odometry over-reads yaw. Fuse the IMU before trusting heading; raise
`wheel_separation_multiplier` for snappier turns at the cost of odometry.

**Terrain costs about a third of the speed** — 0.6–0.8 m/s over Rubicon against
0.98 on flat, from the governor's roughness ceiling and ~24% wheel slip. That is
a fair price for the ground. Traction was never the binding constraint; the
governor was, for the reasons above.

**Friction is boxed in from both sides.** The no-wheelie condition is
`mu < half_wheelbase / h_com` = **0.62**, while hill climbing needs `mu` = 1.0 —
hence the rear-up guard rather than a speed limit. And `mu < 1.23` keeps a
sideways-scrubbing wheel from pushing the hip past its torque limit.

**Physics-engine caveats.** `dartsim` uses one isotropic friction coefficient, so
`mu2`/`fdir1` do nothing. `bullet-featherstone` (`physics_engine:=`) cannot load
Rubicon — the robot falls through the terrain — so it only works with
`flat.sdf`.

Tried and **did not** help, each re-measured with reps: wheel friction 1.0 vs
1.4; mesh vs cylinder vs sphere wheel collision; position vs effort legs; command
speed 0.2–1.0 m/s; dartsim's `bullet` collision detector (worse);
`open_loop_control` on `leg_controller` (a 50 Hz roll oscillation on terrain).
Tune on `world:=flat.sdf`, then confirm on terrain.

One methodological note. The terrain rebuild below was once dismissed on a
correct measurement — 72.8% vs 68.6%, inseparable from noise at n=8 — and a
wrong conclusion. The stance controller was the binding constraint at the time,
and **a fix to the second-largest problem does not show up while the largest is
still there.** With the control faults repaired, the same rebuild is worth
1.52 m → 3.23 m per run.

---

## The offline world

`magi_gazebo/worlds/rubicon.sdf` references `model://Rubicon`, vendored under
`magi_gazebo/models/Rubicon` and put on `GZ_SIM_RESOURCE_PATH` by a colcon hook,
so nothing touches the network at run time. `fetch_rubicon.sh` handles three
things a plain download does not: the Fuel URL **truncates** (a straight `curl`
stopped at 128 MB of 187), the heightmap collision declares **no friction** at
all (an explicit `mu` of 1.5 took driving from 16% to 28% of command), and the
heightmap itself is rebuilt.

### The heightmap is a staircase, and it is rebuilt

`Heightmap.png` ships as **8-bit** greyscale over 5 m of relief, so one grey
level is 19.6 mm and the terrain is a flight of stairs — ~2 cm risers every
7.3 cm cell. The mean cell-to-cell step is **15.6 mm**: over most of the map, the
relief the author drew is smaller than the format can represent.

For an 86 mm wheel that matters. Mounting a step `h` with a wheel of radius `r`
needs a tractive force of `sqrt(2rh − h²)/(r − h)`, which at 19.6 mm is **0.82**
against a friction coefficient of 1.0. Every cell boundary is a near-stall, and
every one cleared kicks the body — the bogging down and the rolling over, from
one cause.

[`rebuild_heightmap.py`](src/magi_gazebo/scripts/rebuild_heightmap.py) removes the
staircase without changing the terrain. The true surface lies within half a grey
level of each sample, so it smooths under exactly that constraint — Laplacian
relaxation, clamped to `[h ± ½ level]` every pass — then resamples to 1025² and
writes **16-bit**.

| | resolution | mean step | slope p50 |
|---|---|---|---|
| shipped, 8-bit | 513² (7.3 cm cells) | 15.6 mm | 15.0° |
| rebuilt, 16-bit | 1025² (3.7 cm cells) | **6.7 mm** | **11.6°** |

No sample moves more than **9.8 mm**, so every rock and tree stays where it was
placed; the slope falls because most of that 15° was staircase, not hill. The
original PNG is kept and only `model.sdf` is repointed, so reverting is one line.

---

## Description notes

`go2w_body.xacro` is generated from the upstream URDF by
`magi_description/scripts/gen_body.py` (re-run it if upstream changes). It
repoints mesh URIs, collapses upstream's illegal sibling `<material>` tags, and
swaps the wheel **collision** mesh for the cylinder it describes (r 0.086,
w 0.0518). Kinematics, inertials and visual meshes are untouched.

**The Livox mesh crashed the Gazebo GUI.** `mid-360.dae` declares VERTEX and
NORMAL at the same `offset`, which is legal COLLADA, but gz-common5's loader
strides indices by the number of inputs, runs off the end of the array and
segfaults — killing the GUI the moment the robot spawned, and never the headless
server, which does not load visuals. `fix_collada_offsets.py` rewrites such
primitives (already applied to the vendored meshes):

```bash
python3 src/magi_description/scripts/fix_collada_offsets.py src/magi_description/meshes/*.dae
```

**If RViz dies with `undefined symbol: __libc_pthread_init`**, you are launching
from a snap-packaged terminal (VS Code's integrated one is the usual case), which
leaks `GTK_PATH` / `LOCPATH` into the snap runtime. The launch files blank those
variables for RViz when they point into `/snap`; running `rviz2` by hand, prefix
it with `GTK_PATH= LOCPATH= GIO_MODULE_DIR=`.

Useful checks:

```bash
ros2 launch magi_description display.launch.py     # RViz + joint sliders, no Gazebo
ros2 control list_controllers
ros2 topic echo /joint_states
```
