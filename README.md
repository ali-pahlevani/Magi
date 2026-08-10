# MAGI — Unitree Go2W in the Rubicon world

ROS 2 Humble + Gazebo Sim (Harmonic) simulation of the Unitree Go2W, a wheeled
quadruped with 16 actuated joints (12 leg joints + 4 drive wheels), driving
around the Rubicon terrain. Control is `ros2_control`-based and the robot is
teleoperable from `/cmd_vel`.

One command starts Gazebo, spawns the robot, activates the controllers, brings
up state estimation, opens keyboard teleop and an RViz laid out for inspecting
odometry:

```bash
ros2 launch magi_launch magi_test.launch.py
```

That is the one to use. `magi_bringup` underneath it is the simulation stack on
its own, without teleop or the odometry view:

```bash
ros2 launch magi_bringup magi_sim.launch.py
```

Then drive it — either fold teleop into the same command, or run it separately:

```bash
ros2 launch magi_bringup magi_sim.launch.py teleop:=true
ros2 launch magi_control teleop.launch.py          # or in a second terminal
```

### The odometry view

`magi_launch/rviz/magi_odometry.rviz` is laid out to make odometry *checkable*
rather than pretty. Fixed Frame is `odom` and the camera deliberately does **not**
follow the robot — tracking `base` would leave the robot sitting still while the
world slid past, hiding exactly what needs watching.

Two odometry trails are drawn at once: **green** is `/odometry/filtered` from the
EKF, **red** is raw `/wheel_controller/odom`. They start together and the red one
visibly swings away the moment you turn. Over a driven path with two turns:

| | final displacement | error vs truth |
|---|---|---|
| ground truth | x +3.23, y +0.55 | — |
| EKF (green) | x +2.47, y +2.01 | **1.65 m** |
| wheel raw (red) | x +2.73, y +4.51 | **3.98 m** |

Startup is staged (`spawn_delay` / `controllers_delay` / `rviz_delay`), with a
longer stagger when the Gazebo GUI is on because it has 342 MB of terrain to
build first. RViz is opened last on purpose: by then `/robot_description` is
latched and `odom -> base` is publishing, so the model is there immediately
rather than after a spell of TF errors. Turn either window off with
`gui:=false` / `rviz:=false`.

---

## Packages

| Package | Contents |
|---|---|
| `magi_description` | URDF/xacro, meshes, `ros2_control` interfaces, Gazebo tags, RViz config |
| `magi_gazebo` | Offline Rubicon world + vendored model, flat test world, simulator launch |
| `magi_control` | Controller YAML, spawners, balance/posture nodes, keyboard teleop |
| `magi_localization` | EKF + leg odometry + IMU conditioner; owns `odom -> base` |
| `magi_bringup` | Simulation stack: world, robot, controllers, estimation |
| `magi_launch` | Runnable configurations. Start here; SLAM and navigation land here too |
| `third_party/gz_ros2_control` | Built from source — see below |

`src/unitree_go2w_ros2` and `src/Rubicon_World` are the original inputs. They
are kept for reference; `unitree_go2w_ros2` carries a `COLCON_IGNORE` because
its `go2w_driver` needs `unitree_sdk2`, which is for the physical robot.

## Build

```bash
cd ~/magi_ws
export GZ_VERSION=harmonic          # required, see below
colcon build --symlink-install
source install/setup.bash
```

### After a fresh clone

The Rubicon world is **not** in the repo. It is 342 MB and `rubicon.dae` alone
is 188 MB, well past GitHub's 100 MB per-file limit, so it is git-ignored and
rebuilt from Fuel instead:

```bash
src/magi_gazebo/scripts/fetch_rubicon.sh   # ~187 MB download, once
colcon build --symlink-install
```

The script downloads, verifies the archive and re-applies the terrain friction
patch, so the result is byte-identical to what the measurements below were
taken against. Everything else — the robot description, meshes, controllers and
launch files — is tracked normally.

### Why gz_ros2_control is built from source

This machine runs **Gazebo Harmonic** (`gz-sim8`), but the Humble binary
`ros-humble-gz-ros2-control` is compiled against **Ignition Fortress**
(`libignition-gazebo6`) and will not load here. The `humble` branch of
`gz_ros2_control` supports Harmonic when `GZ_VERSION=harmonic` is exported at
build time, so it is vendored into `src/third_party/` and built that way. It
installs its own `GZ_SIM_SYSTEM_PLUGIN_PATH` hook, so sourcing the workspace is
enough for Gazebo to find the plugin.

---

## Sensors

Modelled to match the real Go2-W. Definitions live in
[go2w_sensors.xacro](src/magi_description/urdf/go2w_sensors.xacro); everything
is bridged to ROS by [gz_bridge.yaml](src/magi_gazebo/config/gz_bridge.yaml).

| Real hardware | Simulated as | ROS topic | Rate |
|---|---|---|---|
| Livox MID-360 lidar | `gpu_lidar` | `/lidar/points`, `/lidar/scan` | 10 Hz |
| Front HD wide-angle camera | `camera` 1280×720 | `/front_camera/image`, `/front_camera/camera_info` | 30 Hz |
| Body IMU (6-axis + fusion) | `imu` + MEMS noise | `/imu/data_raw` | 200 Hz |
| Foot force sensors ×4 | `force_torque` on each wheel joint | `/foot_force/{FL,FR,RL,RR}` | 200 Hz |
| UWB positioning | not modelled | — | — |
| **no GPS** | **deliberately absent** | — | — |

Verified against the datasheets rather than just "it publishes":

* lidar returns exactly **23,040 points** per scan (720 × 32 ≈ 200k points/s,
  the real MID-360 figure) over 360° × −7…+52°, 0.1–40 m, σ = 2 cm
* camera intrinsics give a **120.0°** horizontal FOV, frame
  `front_camera_optical` in the REP-103 convention
* IMU reads 9.8203 m/s² on the level with accel σ 0.016 and gyro σ 2.2e-4,
  i.e. the noise model is actually active
* foot forces independently reproduce the load asymmetry seen in the calf
  torques (FL 9.3 N and RR 5.1 N against FR 71.8 N and RL 78.3 N)

Two caveats worth knowing:

**The lidar scan pattern is not authentic.** The MID-360 uses a non-repetitive
rosette; `gpu_lidar` only produces a uniform raster and no Livox pattern plugin
exists for Harmonic. FOV, range, rate and point budget are the real numbers, but
the sampling *distribution* is uniform. Generic LIO is fine with that;
Livox-native SLAM keyed to scan lines and per-point timestamps is not.

**The camera is a pinhole, not a fisheye.** `wide_angle_camera` needs an
explicit lens model Unitree does not publish, so a 120° rectilinear camera is
the stand-in. Expect less edge distortion than the real optics.

Foot wrenches are measured in the **parent (calf) frame**, not the child. The
child is the wheel, which spins at ~11.6 rad/s while driving, so a child-frame
wrench has its components rotating with it.

---

## State estimation

A `robot_localization` EKF ([magi_localization](src/magi_localization/)) owns
the `odom → base` transform. `diff_drive_controller` runs with
`enable_odom_tf: false` so there is exactly one publisher of it.

**The filter exists to fix heading.** All four wheels are fixed and
non-steerable, so every turn scrubs them sideways, and wheel odometry cannot
see that scrub — it infers yaw purely from the wheel speed difference and badly
over-reads. Measured against Gazebo ground truth:

| manoeuvre | ground truth | wheel odom | EKF |
|---|---|---|---|
| spin 0.5 rad/s, 8.8 s | 118.1° | 197.6° (**+67%**) | 124.1° (**+5%**) |
| arc 0.5 m/s + 0.4 rad/s, 7.1 s | 64.2° | 121.9° (**+90%**) | 60.2° (**−6%**) |

Reproduce with:

```bash
ros2 run magi_localization magi_yaw_compare.py --angular 0.5 --duration 8
```

### What is fused, and what deliberately is not

* `odom0` contributes **vx only** — not yaw, not yaw rate, not vy.
* `imu0` contributes roll/pitch (gravity-referenced, genuinely observable) and
  all three angular rates.

Yaw is therefore integrated from the gyro with no absolute reference, exactly
as on the real robot: the Go2's IMU is 6-axis with no magnetometer. It drifts
slowly, which is correct — `odom` is by definition a smooth, locally-accurate,
drifting frame, and SLAM supplies `map → odom` to correct it in Phase 3.

`vy` is not fused. A differential-drive filter often fuses `vy = 0` as a
non-holonomic constraint, but this is skid-steer: lateral slip is real and large
during turns, so asserting `vy = 0` would inject a lie.

`two_d_mode` stays `false` so the roll/pitch the IMU genuinely measures is kept.

### The vertical channel comes from the legs, not the IMU

An accelerometer cannot supply `z`. With no `z` or `vz` measurement, fusing
`az` gives the filter something to *integrate* but nothing to *correct against*,
so the bias double-integrates — this is the classic unstable vertical channel of
an unaided inertial navigator, and it is why every real INS aids altitude with a
barometer or GNSS.

Measured here, `az` fused and the robot **completely stationary**:

| | z after ~40 s | drift |
|---|---|---|
| IMU only | **−107.96 m** | −102.4 m |
| **+ leg kinematics** | **0.010 m** | **+0.006 m** |

The robot never moved; the IMU-only filter "fell" 108 m. Its `vz` grew linearly
at **0.0979 m/s²** against the **0.1 m/s²** accelerometer `bias_mean` configured
in the URDF — the filter faithfully integrating the bias it was given.

`magi_leg_odometry` fixes this the way real quadruped estimators do. Each loaded
wheel is a point of known position relative to the body touching a surface, so
body height above that surface is directly observable:

```
h = wheel_radius - (R · t_i).z
```

where `t_i` is the axle offset from TF and `R` is body orientation. Only roll and
pitch build `R` — the z component of `R·t` is invariant to yaw — which neatly
sidesteps the IMU's drifting, unreferenced heading. Contact comes from the foot
force sensors added in Phase 1.

Verified against known geometry: leg-derived height **0.3877 m** against a
ground-truth **0.3963 m**, an 8.6 mm error on rough terrain, with all four feet
correctly detected in stance. And `z` now tracks real vertical motion — a
commanded crouch moved the body **−0.123 m** and the EKF reported **−0.123 m**.

| topic | meaning |
|---|---|
| `/magi/terrain_height` | body height above the contact plane |
| `/magi/contacts` | which feet are loaded |
| `/magi/leg_twist` | `vz` only, fused by the EKF as `twist0` |

**What this does not do.** `vz` here is relative to the *terrain*. On level
ground that is true vertical velocity, but climbing a slope at constant ride
height it reads zero while the robot genuinely rises, so `odom` still cannot
track absolute elevation — that needs SLAM in Phase 3. What it buys is a
*measured* vertical channel: `z` follows real body motion and its error is a
slow random walk instead of an unbounded quadratic divergence.

### Why there is an IMU conditioner node

`gz.msgs.IMU` has no covariance fields, so everything `ros_gz_bridge` puts on
`/imu/data_raw` has **all three covariance matrices set to zero**.
`robot_localization` reads a zero measurement covariance as infinite confidence,
which collapses the state covariance and makes the filter degenerate.

`magi_imu_covariance` republishes `/imu/data_raw` as `/imu/data` with the
variances of the noise model actually configured in the URDF (gyro 4.0e-8,
accel 2.89e-4). Orientation is anisotropic on purpose: roll/pitch get ~0.5°,
yaw gets 1e6 to stop any consumer treating it as an absolute heading.

---

## 3D SLAM (Phase 3)

RTAB-Map builds a 3D lidar map and publishes `map -> odom`
([magi_slam](src/magi_slam/)). The Phase 2 EKF keeps publishing `odom -> base`,
so the full chain exists with exactly one owner per link.

```bash
ros2 launch magi_launch magi_test.launch.py slam:=true \
    rviz_config:=$(ros2 pkg prefix magi_slam)/share/magi_slam/rviz/magi_slam.rviz
```

**Prerequisite.** `rtabmap` will not start until `diagnostic_updater` is
upgraded. The installed 4.0.6 does not ship `libdiagnostic_updater.so`, which
rtabmap 0.23.7 links against, and the node dies with exit 127:

```bash
sudo apt install --only-upgrade ros-humble-diagnostic-updater
```

### Why no icp_odometry

`rtabmap_odom`'s `icp_odometry` would publish `odom -> base` itself and fight
the EKF for it. Instead rtabmap consumes `/odometry/filtered` as its odometry
source and adds only the `map -> odom` correction. `Reg/Force3DoF` is false
because the correction has to be full 6-DoF: EKF yaw is gyro-integrated and
drifts, and odom z is terrain-relative and does not track the terrain's 5 m of
relief. Both are exactly what loop closure exists to absorb.

### Measured

A 6 s segment at 0.35 m/s across the basin, against Gazebo ground truth:

| source | distance | error |
|---|---|---|
| ground truth | 1.810 m | — |
| **SLAM (`map -> base`)** | 1.846 m | **+0.036 m** |
| EKF alone (`odom -> base`) | 2.198 m | +0.387 m |

SLAM cuts position error about tenfold. Yaw is barely improved (−7.3° against
the EKF's −8.1°), which is expected: a single short segment offers no loop
closure, and heading correction is what loop closure provides.

The map itself builds as intended — 21,297 points in `/cloud_map` and a
245 × 305 cell occupancy grid at 0.1 m on `/map`, the latter being what Nav2
will consume in Phase 5.

---

## Control architecture

`gz_ros2_control` creates the `controller_manager` inside the Gazebo process;
the launch files only run spawners against it.

| Controller | Type | Joints |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | all 16 |
| `imu_sensor_broadcaster` | IMUSensorBroadcaster | body IMU |
| `leg_controller` | JointTrajectoryController | 12 leg joints, **effort** |
| `wheel_controller` | DiffDriveController | 4 wheels, **velocity**, skid-steer |

Each joint declares exactly **one** command interface. This is deliberate:
`GazeboSimSystem::write()` is an if/else chain testing `VELOCITY` before
`POSITION` before `EFFORT`, so a joint exposing several would silently ignore
all but the first.

### The legs are torque-controlled

`joint_trajectory_controller` in effort mode emits

```
tau = p*(q_des - q) + i*integral + d*(dq_des - dq)
```

which is the same law the real Unitree motor runs from its `MotorCmd`
`(q, dq, tau, kp, kd)`. The simulated joint therefore behaves like the real
actuator rather than a kinematic constraint.

Gains are sized from geometry, not guesswork. Joint stiffness maps to contact
stiffness as `k = p / lever²`; the calf works through a 0.1535 m lever, so
`p = 200` gives ~8500 N/m at the wheel, i.e. ~5.7 mm of travel under the 48.5 N
per-leg load — deliberately matched to the terrain's 6.8 mm mean facet step.

### Startup and the limp-leg window

Torque-controlled legs carry no load until `leg_controller` activates, and
declaring an effort command interface sets `joint_control_method |= EFFORT` at
*init* -- unlike position/velocity, which are only set when a controller claims
them ([gz_system.cpp:476](src/third_party/gz_ros2_control/src/gz_system.cpp#L476)).
So the legs are limp from the moment the model appears.

**They recover from it.** The robot dips and the PD stands it straight back up:
three consecutive GUI launches each settled at 0.396 m, the design stance, with
all four controllers active.

Two things were got wrong here and are worth recording. Seeding a constant
gravity-hold torque does not work -- the stance is an *unstable* equilibrium, so
open-loop torque diverges, and with the signs wrong it drove every joint into a
limit within seconds, leaving the legs splayed at hip +1.0472 and calf -0.8378.
That failure was then misread as "the robot cannot recover from limp legs",
which led to holding the simulation paused until the controllers were ready.

That paused start could not be made reliable, because activation itself needs
physics ticks: the switch sits pending, `controller_manager` gives up after
`--switch-timeout`, and whether the unpause lands first depends on how long
Gazebo took to build the world. On a GUI run the race was lost by two seconds
and the robot stayed on its belly with its wheels spinning against the ground.

`paused:=true` and `magi_unpause` remain available for cases where the limp
window genuinely must be eliminated, but the default path is unpaused, simpler,
and the one that is verified. The spawners still run in parallel with a
generous `--switch-timeout`.

---

## Measured behaviour

Ground truth read from the Gazebo server, yaw accumulated unwrapped.

**Always use `--reps` on terrain.** A single run on `rubicon.sdf` is close to
worthless: the robot veers onto different ground each time, and single runs in
this project produced anything from 15% to 79% for *identical* configurations.
Flat ground repeats to sd 0.2, so the variance is terrain, not the harness.

8 reps × 3.5 s at 1.0 m/s from a fixed reset pose:

| World | Mean linear efficiency | sd |
|---|---|---|
| `flat.sdf` | **88.8%** | 0.2 |
| `rubicon.sdf`, basin | **47.4%** | 2.3 |

Spin and arc figures (single runs on `flat.sdf`, where repeatability is high):
0.5 rad/s → 78%, 1.0 rad/s → 78%, 1.5 rad/s → 81%, all with zero stance drop.

After all of the above the robot returns to stance with every leg joint within
0.013 rad of target, and the base at 0.412 m over local terrain against a 0.408 m
design stance.

Reproduce with the benchmark, which reads Gazebo ground truth rather than
odometry (wheel odometry over-reads yaw badly on a skid-steer) and resets the
robot between reps so every sample starts from the same ground:

```bash
ros2 run magi_control magi_drive_benchmark.py 1.0 0.0 3.5 --reps 8 \
    --reset-world rubicon --reset-pose 3.0,-0.5,1.85
ros2 run magi_control magi_drive_benchmark.py 0.6 0.4 3.5 --reps 8 \
    --reset-world rubicon --reset-pose 3.0,-0.5,1.85     # arcing turn
```

### What torque control changed

Load sharing, measured as calf effort across the four legs on Rubicon:

| | rigid position control | compliant torque control |
|---|---|---|
| FL calf | 0.061 N·m (effectively airborne) | 0.978 N·m |
| RR calf | 1.780 N·m | 5.368 N·m |
| max/min spread, standing | **201×** | **10.2×** |
| max/min spread, driving | — | **3.3×** |

The robot used to teeter on one diagonal pair with a wheel barely touching down;
now all four carry load. Posture tracking is within 5 mm of command
(0.28 → 0.276, 0.46 → 0.456, 0.40 → 0.396) and holds steady.

Forward traction on terrain, however, did **not** improve — see limitations.

### The wheel collision shape dominates everything

Worth knowing before tuning anything else. Upstream reuses the wheel *visual*
mesh as the collision geometry. Swapping it for the primitive cylinder the mesh
describes (done in `gen_body.py`) changed the robot's behaviour more than any
gain or friction value:

| | mesh collision | cylinder collision |
|---|---|---|
| flat, 0.5 rad/s spin | 38% | 78% |
| flat, 0.8 rad/s spin | **legs splay, body collapses** | fine, 0.000 m drop |
| flat, 1.5 rad/s spin | not reachable | 81%, 0.000 m drop |

With the mesh, a pure spin scrubs all four wheels at once and the reaction
saturated the hips at their 23.7 N·m URDF effort limit — joints pinned at
exactly 23.700 N·m, legs splayed, and a position-held stance could not recover.
The cylinder removed that failure mode outright, which is why the angular limit
is 1.5 rad/s rather than the 0.6 the mesh forced.

## Known limitations

**Yaw authority is ~78% on flat, ~47% on terrain.** All four wheels are fixed
and non-steerable, so every turn scrubs them sideways. This also means
`wheel_controller`'s odometry **over-reads yaw**: it infers rotation from wheel
differential and cannot see the scrub. Fuse `/imu_sensor_broadcaster/imu` before
trusting heading. For snappier turning at the cost of odometry fidelity, raise
`wheel_separation_multiplier` in `magi_controllers.yaml`.

**Terrain roughly halves speed (88.8% → 47.4%) and the robot veers.** This is a
property of the asset, not a tuning problem.

The Rubicon heightmap is an **8-bit** PNG spanning 5 m of relief, so one grey
level is 5.0/255 = **19.6 mm**. The terrain is therefore a staircase with ~2 cm
risers every 7.3 cm cell, against an 8.6 cm wheel radius. Step-climbing
resistance goes as `sqrt(2h/r)`, which at h = 19.6 mm is ~0.67 of the wheel
load — enough to explain the whole gap. The measured mean step across the map,
18.0 mm, is almost exactly one quantisation level.

Isolating it: a flat platform placed *inside* the Rubicon world — same lighting,
models, friction and robot, 2 m from the terrain — gives 83.6%. The contact
surface is the only variable.

Things that were tried and did **not** help (each re-measured with reps where
the first single-run result looked promising):

| Change | Result |
|---|---|
| wheel friction 1.0 vs 1.4 | no change |
| wheel collision mesh vs cylinder | no change |
| leg compliance (position vs effort) | no change |
| command speed 0.2 / 0.4 / 1.0 m/s | no change |
| dartsim collision detector `bullet` | worse |
| dartsim collision detector `ode` | worse |
| heightmap `<sampling>` 1 / 2 / 4 | 38.9% vs 40.6% vs — , within noise |
| 16-bit de-quantised heightmap | 72.8% vs 68.6%, within noise (n=8, sd ~12) |

The 16-bit rebuild is the physically-correct fix for the quantisation and had
the better mean, but 8 reps could not separate it from noise, so it is not
applied. Use `world:=flat.sdf` when tuning controllers, then confirm on terrain.

**Rollover: largely addressed by `magi_balance`, not eliminated.** The robot
used to flip under genuinely gentle commands — 0.35 m/s with 0.15 rad/s put it
on its side — because `leg_controller` held a *fixed* stance with no balance
feedback. `magi_control/scripts/magi_balance.py` replaces that with a
closed-loop stance controller (see the module docstring for the full design).

Measured A/B on Rubicon, 10 s runs at 0.3 rad/s, 3 reps per speed:

| | fixed stance | `magi_balance` |
|---|---|---|
| rollovers | 1 / 9 (159.5° at 0.35 m/s) | **0 / 9** |
| roll rms, upright runs | 1.7–4.9° (mean ≈3.4) | **0.6–1.8° (mean ≈1.1)** |
| steady tilt, standing | +2.12 / −1.10° | **+0.02 / +0.01°** |
| track | 0.380 m fixed | 0.41–0.51 m, adaptive |
| distance in 10 s @ 0.9 m/s | 2.57–2.97 m | 1.79–2.31 m |

Two honest caveats. One rollover in nine is thin evidence for the headline
claim — the disturbance-rejection numbers (18 runs, consistent) are the solid
part, and the rollover that did occur was at the *lowest* speed, which fits the
finding that these are terrain impulses rather than centrifugal load. And the
balance controller **costs about 25% of forward progress**: the wider stance
scrubs harder in a skid-steer turn. If a run needs distance more than
stability, `balance:=false` restores the old fixed stance.

There is still no self-righting behaviour, so a large enough disturbance leaves
the robot down until the pose is reset.

*Why the turn was never the problem.* At 0.35 m/s and 0.15 rad/s the lateral
acceleration is 0.0525 m/s², which moves the centre of pressure by **1.7 mm of
the 190 mm half-track available**, against a 31.4° static tipping angle. The
lean feed-forward is therefore almost irrelevant at these speeds; what does the
work is the attitude feedback rejecting terrain impulses, and the anticipatory
widening that raises the tipping angle to 39.9°.

**Hips saturate on terrain.** Driving across Rubicon at 1 m/s pins FL and FR
hip at their 23.7 N·m URDF limit (RL 71%, RR 81%) as the wheels get shoved
laterally by facets. That is the real robot's actuator spec, so it is arguably
correct fidelity rather than a bug — but it caps how hard the platform can be
pushed over rough ground.

**Compliance did not fix terrain traction.** Switching the legs from rigid
position control to compliant torque control left forward efficiency on Rubicon
unchanged at ~25%. Combined with the earlier eliminations — friction level,
collision primitive, command speed — the remaining explanation is dartsim's
wheel-on-heightmap contact behaviour, not the robot model. See the benchmark
note below.

**`mu2`/`fdir1` do nothing.** `dartsim`, the default physics engine, uses a
single isotropic friction coefficient per shape. They are set in
`go2w_gazebo.xacro` for completeness only. `bullet-featherstone` is exposed via
`physics_engine:=` but **cannot load the Rubicon world** ("multiple sub-trees /
floating links" — the robot falls through the terrain), so it is only usable
with `flat.sdf`.

**Wheel friction is capped by the hip torque budget.** `mu1` is 1.0, not higher:
a yaw manoeuvre pushes each wheel sideways with up to `mu*N` (N ~ 48 N per
wheel) acting a stance height below the hip axis, so the hip sees `mu*48*0.4`
N·m and must stay under 23.7 N·m, i.e. `mu < 1.23`.

---

## The offline world

`src/Rubicon_World/rubicon.sdf` pulled the terrain from Fuel over the network at
run time. `magi_gazebo/worlds/rubicon.sdf` instead references `model://Rubicon`,
vendored under `magi_gazebo/models/Rubicon` (342 MB extracted) and put on
`GZ_SIM_RESOURCE_PATH` by a colcon environment hook. Nothing touches the network.

To re-create or update that copy:

```bash
src/magi_gazebo/scripts/fetch_rubicon.sh
```

Two things that script handles and a plain download does not:

* The Fuel URL **truncates** — a straight `curl` stopped at 128 MB of the real
  187,430,027 bytes and produced a corrupt zip. It uses `--retry`/`-C -` and
  verifies with `unzip -t`.
* The upstream heightmap collision declares **no friction surface at all**, so
  it falls back to Gazebo's default. Adding an explicit `mu` of 1.5 took driving
  from 16% to 28% of commanded speed. The script re-applies that patch.

## Description notes

`magi_description/urdf/go2w_body.xacro` is generated from the upstream URDF by
`magi_description/scripts/gen_body.py`; re-run it if upstream changes. The
kinematics and inertials are untouched. It rewrites three things:

* mesh URIs now point at `magi_description/meshes`
* upstream emits several sibling `<material>` tags inside one `<visual>`, which
  is not legal URDF — collapsed to one, and the non-ASCII names replaced
* wheel **collision** geometry swapped from the visual mesh to a primitive
  cylinder (r 0.086, w 0.0518, offset 0.0481 — the mesh's own bounding box).
  Visual meshes are untouched.

### The Livox mesh crashed the Gazebo GUI

`mid-360.dae` shipped with every `<polylist>` declaring both VERTEX and NORMAL
at `offset="0"`. That is legal COLLADA — one index feeds both arrays, so `<p>`
holds a single index per vertex — but gz-common5's `ColladaLoader` strides `<p>`
by the *number of inputs*. It read twice as many indices as the file contains,
ran off the end of the array and segfaulted in `parseInt`, killing the entire
Gazebo GUI the moment the robot spawned.

It only ever showed up with the GUI running, because the server never loads
visual meshes — which is why every headless test passed.

`magi_description/scripts/fix_collada_offsets.py` rewrites such primitives into
the form the loader expects (distinct offsets, `<p>` expanded by repeating each
index once per input — numerically identical). It has already been applied to
the vendored meshes; re-run it if you re-import any:

```bash
python3 src/magi_description/scripts/fix_collada_offsets.py src/magi_description/meshes/*.dae
```

### If RViz dies at startup with a symbol lookup error

Launching from a **snap-packaged terminal** — VS Code's integrated terminal is
the common case — leaks `GTK_PATH` / `LOCPATH` and friends into the snap's own
runtime. Qt then loads the snap's GTK module, which drags in the snap's
libpthread, and RViz exits with:

```
symbol lookup error: /snap/core20/.../libpthread.so.0: undefined symbol: __libc_pthread_init
```

`magi_sim.launch.py` blanks those variables for the RViz process, but only the
ones actually pointing into `/snap`, so a normal shell is left untouched. If you
hit it running `rviz2` by hand, prefix the command with
`GTK_PATH= LOCPATH= GIO_MODULE_DIR=`.

Useful checks:

```bash
ros2 launch magi_description display.launch.py     # RViz + joint sliders, no Gazebo
ros2 control list_controllers
ros2 topic echo /joint_states
```
