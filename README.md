# MAGI: Unitree Go2W in the Rubicon world

![Magi Banner](https://github.com/user-attachments/assets/fcbf9410-ab91-45f4-a014-9caf661a094b)

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
| `magi_gazebo` | Offline Rubicon world + vendored model, heightmap rebuild, flat test world, simulator launch |
| `magi_control` | Controller YAML, spawners, stance/balance/posture nodes, keyboard teleop |
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

The script downloads, verifies the archive, re-applies the terrain friction
patch and **rebuilds the heightmap** (see
[the offline world](#the-offline-world)), so the result matches what the
measurements below were taken against. Everything else — the robot description,
meshes, controllers and launch files — is tracked normally.

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

### Where z = 0 is, and why the map used to float

RTAB-Map anchors its `map` frame on the pose of whatever `frame_id` names, at
the first keyframe. Point it at `base` and z = 0 of the map lands wherever the
body happened to be — which on this robot is a whole ride height off the floor.

That is invisible in the 3D cloud and glaring in the 2D one. A
`nav_msgs/OccupancyGrid` always carries `origin.position.z = 0`, so RViz drew
`/map` as a flat plane 0.35 m in the air, cutting through the robot with its
wheels hanging underneath it. The cloud was never wrong: measured against the
simulator's own heightmap, its ground returns land within **0.03 m** of the
true surface. Neither was the robot. Only the datum was.

Two changes fix it, and they are independent:

* **`magi_leg_odometry` sets the odom datum once**, at startup, when the robot
  is standing with a settled height. It already measures body height above the
  contact plane to ~9 mm, so it calls the EKF's `/set_pose` and moves z = 0
  down by exactly that. `/odometry/filtered` now reports the robot's real ride
  height instead of ~0. (`set_ground_datum:=false` restores the old behaviour.)
* **`rtabmap` anchors on `base_footprint`**, a rigid link 0.36 m below `base`
  added in `magi_go2w.urdf.xacro`. Rigid, not a tracked projection: SLAM wants
  a frame bolted to the robot, and a footprint that bobbed with the ride height
  would push that bobbing into scan registration.

Measured afterwards, standing:

| | |
|---|---|
| map z = 0 against the true ground | **2 mm** |
| 2D grid plane against the wheel contacts | **1 mm** |
| `map -> odom` z | 0.000, sd 0.000 |

and after nine metres of driving over the terrain, the grid is **38 mm** off
the wheels. The residue is the known one: odom z is terrain-relative and does
not track absolute elevation, so the cloud's ground drifts to about 0.13 m
below truth over that distance. That is what loop closure exists to absorb, and
it is drift rather than the fixed 0.35 m offset it replaced.

**Run one `rtabmap` at a time.** Two of them both publish `map -> odom`, and
the symptom is not an error but a z that flickers between two values — it cost
a wrong diagnosis here before the second instance turned up.

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

### Stance control and the command governor

Above the controllers sits one node that decides where the feet go.
`magi_stabilizer` is the default; `magi_balance` (the original quasi-static CoP
controller) and `magi_posture` (one fixed stance) are kept for A/B:

```bash
ros2 launch magi_launch magi_test.launch.py                            # stabilizer
ros2 launch magi_launch magi_test.launch.py stance_controller:=balance # the old one
ros2 launch magi_launch magi_test.launch.py balance:=false             # fixed stance
```

The stabilizer differs from `magi_balance` in one structural way: **it sits in
the command path**. Teleop and navigation publish to `/cmd_vel`, and the node
republishes to `/wheel_controller/cmd_vel_unstamped` after projecting the twist
onto the set of twists the robot can currently survive. A steady turn at
`(v, w)` needs `v*w` of lateral acceleration; how much is available is found by
bisecting the measured stability margin, so the envelope shrinks by itself on a
side slope, over a bump, and whenever a wheel unloads. Both components are
scaled by the same factor, which preserves `v/w` — the robot takes the
operator's arc more slowly rather than a different arc.

Nothing else may publish to `/wheel_controller/cmd_vel_unstamped` while it is
running. The governor stays silent until it receives its first `/cmd_vel`, so
the direct-drive path still works for baseline runs.

Stability is measured as a **force-angle margin**: the angle the net force may
still rotate through before the robot goes over its worst support edge, taken
against the effective gravity the accelerometer reports (which already contains
centrifugal, braking and terrain terms). Unlike "is the CoP inside the
polygon", it is defined when only two wheels are loaded — the moment that
actually matters — and its units are degrees of remaining tilt.

Measured on Rubicon, spawned and reset to (4.0, -0.5, 1.80), 8 s per run:

| profile | `magi_balance` | `magi_stabilizer` |
|---|---|---|
| v 0.7 | 2/3 upright, roll 45° | **3/3**, roll 11° |
| v 1.2 | 0/3 upright, roll 137° | **8/8**, roll 9° |
| v 0.9, w 0.5 | 1/3 upright, roll 99° | **3/3**, roll 11° |
| v 1.5, w 1.2 | 1/3 upright, roll 85° | **7/8**, roll 12° |
| **total** | **4/12** | **21/22** |

It is *not* a guarantee. There is no stepping, so a big enough terrain event
still wins. See the module docstring in
`magi_control/scripts/magi_stabilizer.py`, which also records the three
measurements that shaped the design — why the accelerometer cannot be used raw
as an attitude reference, why contacts have to be held briefly after they go
light, and why splaying the stance helps even though it cambers the wheels.

#### Six faults that made it crawl, and how each was found

Everything above was true of the design and false of the implementation. The
robot as shipped crossed the terrain at 1.07 m per 8 s run, splayed to its
stance limit and stuck there, refusing every command above its roughness crawl.
Each fault is documented at its site in `magi_stabilizer.py` and
`magi_stabilizer.yaml`; in the order they had to be peeled apart:

**1. The attitude loop was shaking the robot blind.** The rate term was fed to
the legs unfiltered, on the reasoning that damping delayed is damping wasted.
But the path from "command a body attitude" to "the body rolls" runs through
the command horizon, the trajectory controller and the ~20 Hz mode of the body
on the hip PD — 180° of phase well before 20 Hz — so undelayed rate feedback
there is gain, not damping. Standing still on level ground it sustained a
**19.5 Hz limit cycle at 3.9 rad/s rms**, and the accelerometer read **±200
m/s²**: 20 g on a stationary robot. Every consumer downstream reads that
accelerometer, so the effective-gravity reference became garbage (a_y 130
against a_z 14), the stability margin reported **−32° on an upright robot**, and
everything below followed. With the filter back on the whole command and `kd`
at 0.05: **0.007 rad/s rms**.

**2. The acceleration envelope was never actually computed.** `_accel_envelope`
built effective gravity as `-down*G`, which points *up*. Every bisection probe
failed the "support pattern must be below the CoM" test, both searches returned
`a_lat_floor`, and the governor had been running on a hard-coded **0.8 m/s² of
lateral authority** — against the 10.0 m/s² a splayed stance is actually good
for — since the envelope was written. One sign.

**3. The terrain preview scored hills as roughness.** It fit a *plane* to lidar
returns 2.6–5.4 m ahead and called the residual roughness. Over a 2.8 m window a
hill is not a plane: measured over 600 random placements on Rubicon, the plane
residual is p50 5.3 cm and p90 16.5 cm on ground that is merely *sloped*, against
a 4.5 cm "fully rough" reference. The preview therefore read **fully rough over
57% of the map**, permanently. It fits a quadratic now — p50 3.4 cm, curvature in
the model where it belongs — and both the roughness and slope terms are gated by
time-to-arrival, because the MID-360 cannot see the ground closer than 2.69 m and
at a walking pace that window is eight seconds away.

**4. Urgency was pinned at 1.00, so the stance never came back in.** With (1) and
(3) feeding it and a slope reference hard-coded at 0.35 rad — below this
terrain's 75th-percentile slope — the robot sat permanently splayed to its
0.600 m limit and permanently throttled to `v_rough_min`, which was 0.35 m/s.
That single number was the robot's top speed everywhere on Rubicon.

**5. It splayed and crouched almost all the time, and mostly for no reason.**
Measured over a course, the track sat above 0.50 m (nominal 0.380) for 98% of
the run and the body below 0.32 m for 69% of it. On *flat ground*, with nothing
wrong at all, the robot drove at a 0.554 m track with its body 7 cm below its
ride height — because `u_speed` ramped from a 0.25 m/s "calm" speed, so an
ordinary 1.0 m/s was already scored 0.79 of maximum urgency. Widening now
starts at 0.80 m/s and the crouch is halved, since lowering the ride height to
0.36 already banked most of what the crouch was borrowing:

| on flat, at 0.97 m/s | track | ride height |
|---|---|---|
| before | 0.554 m | 0.290 m |
| after | **0.443 m** | **0.346 m** |

`widen_max` came down from 0.110 to 0.070 in the same pass, and that one is not
a trade at all — over the course, the narrower stance loses a contact 9% of the
time against 16%, and its median tip margin is **35.6° against 31.3°**. The
wider stance was *less* stable: cambering the wheels 24° onto their rim edges
cost more contact than the extra geometry bought. The stability it was chasing
comes from the ride height instead, which reaches the same 45° of tipping angle
at a 0.520 m track rather than a 0.600 m one.

**6. The stance was friction-locked, and nobody had checked.** Dragging four
loaded wheels sideways costs 17.3 N·m at the hip against a 23.7 N·m limit, so at
rest the width lever does not exist. Measured at 0.413 rad of splay with the
target set back to nominal: both front hips **pinned at exactly −23.700 N·m**,
and the splay did not move for as long as the robot stood there — through a
sweep of `widen_max` from 0.110 to 0. Six seconds of rolling brought the same
joint back to 0.038 rad. Reshaping is gated on rolling now.

And one thing that was simply missing: **nothing guarded the pitch axis.** The
whole node is built around the 31.4° roll tipping angle because the stance can
be widened to answer it. Its pitch twin, `atan(0.1934/0.311)` = **31.9°** of
nose-up, has no lever at all — and the robot drove onto faces steeper than that
and went over backwards. There is a guard for it now; see
[Known limitations](#known-limitations) for what it can and cannot do.

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
worthless: the robot veers onto different ground each time, and the *same* start
pose under an *identical* configuration has produced 2.7 m and 5.4 m. Flat
ground repeats to sd 0.2, so the variance is terrain, not the harness.

### Can it get around the world?

The drive benchmark answers "what fraction of the commanded speed does it make,
here". The question that actually matters is whether the robot can cross the
map, and that needs several places on it. `magi_terrain_trial.py` drives a
twelve-leg course — nine start poses spread over Rubicon with footprint-scale
slopes from 3° to 23°, plus two arcs and a spin — and reports **net
displacement**, not path length, because a robot shaking itself sideways racks
up path without going anywhere.

Three passes of the course each, 8 s per leg, 36 legs per configuration:

| | stood up | **net displacement** | path | upright at end |
|---|---|---|---|---|
| before the fixes below | 31/36 | **1.07 m** | 1.54 m | 28/31 |
| control fixes, 8-bit terrain | — | **1.52 m** | 1.81 m | 19/21 |
| control fixes + rebuilt terrain | 29/36 | **3.23 m** | 3.68 m | 18/29 |
| …at `ride_height` 0.36 (default) | 11/12 | **2.57 m** | 3.08 m | **9/11** |

Three times the ground covered. The last row is one pass rather than three, and
is the shipped default: see the `ride_height` note in `magi_stabilizer.yaml` for
why twenty points of upright rate is worth twenty percent of the distance here.

The honest reading of the upright column: the fixed robot rolls over **more per
run** than the stock one did, and that is because it now reaches things. Eight
of the twelve rollovers in the 0.40 row were at the two start poses that have a
boulder within a metre — ground the stock configuration was simply too slow to
arrive at. Nothing in this stack does obstacle avoidance yet; the course drives
blind into a boulder field for eight seconds at a time.

On flat ground, where there is nothing to hit, the same configuration makes
**0.98 m/s of a commanded 1.00** with body rates of 0.007 rad/s rms.

```bash
ros2 run magi_control magi_terrain_trial.py --duration 8 --reps 3
ros2 run magi_control magi_terrain_trial.py --duration 8 \
    --course '[[0,0,0.5,0,1.0,0.0]]' --world flat      # single leg, flat
```

For per-spot efficiency the older benchmark is still the right tool. It reads
Gazebo ground truth rather than odometry (wheel odometry over-reads yaw badly on
a skid-steer) and resets the robot between reps:

```bash
ros2 run magi_control magi_drive_benchmark.py 1.0 0.0 3.5 --reps 8 \
    --reset-world rubicon --reset-pose 3.0,-0.5,1.85
ros2 run magi_control magi_drive_benchmark.py 0.6 0.4 3.5 --reps 8 \
    --reset-world rubicon --reset-pose 3.0,-0.5,1.85     # arcing turn
```

**Run one stance controller at a time.** Both tools drive the robot from a
separate process, and a stray `magi_posture` left running alongside
`magi_stabilizer` produces measurements that look like physics and are not —
it cost a long detour here, with the robot apparently unable to move on flat
ground at 3.5% of commanded speed, legs limp at exactly 0.00 N·m, until the
second publisher on `/leg_controller/joint_trajectory` turned up.

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

**Terrain costs about a third of the speed, and the robot still veers.** On
flat ground the robot makes 0.98 m/s of a commanded 1.00; over Rubicon the
governor's own roughness ceiling and ~24% of wheel slip bring that to
0.6–0.8 m/s. That is a fair price for the ground, not a fault.

The 8-bit heightmap staircase that used to dominate this section is **fixed** —
see [the offline world](#the-offline-world). It is worth restating why the
earlier attempt at the same fix was dismissed: measured with the old
controller, the 16-bit rebuild gave 72.8% against 68.6% and could not be
separated from noise at n=8. That reading was correct and the conclusion drawn
from it was wrong. The terrain was never the binding constraint at the time —
the stance controller was — and a fix to the second-largest problem does not
show up while the largest one is still there. With the control faults below
repaired, the same terrain rebuild is worth **1.52 m → 3.23 m** of ground per
8 s run.

Things that were tried and did **not** help (each re-measured with reps where
the first single-run result looked promising):

| Change | Result |
|---|---|
| wheel friction 1.0 vs 1.4 | no change |
| wheel collision mesh vs cylinder | no change |
| wheel collision **sphere** vs cylinder | 20% vs 24% slip — within noise, and a sphere of the wheel radius is 6 cm wider than the wheel |
| leg compliance (position vs effort) | no change |
| command speed 0.2 / 0.4 / 1.0 m/s | no change |
| dartsim collision detector `bullet` | worse |
| `open_loop_control` on `leg_controller` | clean on flat, a 50 Hz roll oscillation on terrain — see the note in `magi_controllers.yaml` |

Use `world:=flat.sdf` when tuning controllers, then confirm on terrain.

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
the robot down until the pose is reset. On a teleoperated run that is the
failure that costs most, and it is the obvious next thing to build.

**Nothing avoids obstacles.** The world carries ~200 rock, stump and rockpile
colliders plus 34 tree trunks, and most protrude well above the 8.6 cm wheel
radius. The robot cannot see them: the MID-360 looks only 7° below its own
horizon, so from 0.50 m up its lowest ray does not reach the ground until
2.69 m out and anything shorter than about 0.4 m inside that radius is
invisible. Driven blind, the robot hits them — which is most of what the
rollover column in [Measured behaviour](#measured-behaviour) is counting. A
human driver steers around them; Nav2 in Phase 5 is what closes this properly.

*Why the turn was never the problem.* At 0.35 m/s and 0.15 rad/s the lateral
acceleration is 0.0525 m/s², which moves the centre of pressure by **1.7 mm of
the 190 mm half-track available**, against a 31.4° static tipping angle. The
lean feed-forward is therefore almost irrelevant at these speeds; what does the
work is the attitude feedback rejecting terrain impulses, and the anticipatory
widening that raises the tipping angle to 39.9°.

**The stance width lever only exists while the wheels are turning.** Reshaping
the stance means dragging four loaded wheels sideways, and that costs `mu*N`
through the stance height — 1.0 × 48 N × 0.36 m = **17.3 N·m** at the hip — on
top of the ~12 N·m the splay already holds statically, against a 23.7 N·m hip
limit.

Measured standing on Rubicon at 0.413 rad of splay, with the target set back to
the nominal stance: both front hips sat at exactly **−23.700 N·m**, saturated,
and the splay did not move by one milliradian for as long as the robot stood
there — through a sweep of `widen_max` from 0.110 all the way to 0. Six seconds
of rolling at 0.6 m/s brought the same joint back to 0.038 rad with 10.6 N·m of
effort.

So the lever is one-way at rest, and asking for it anyway does not widen the
stance — it just holds both hip motors against their stops, which on the real
machine is a thermal fault rather than a stance. `reshape_roll_speed` fades the
rate out below a walking pace and `widen_band` keeps the commanded width within
reach of the achieved one, so the hip PD never saturates on a difference it
cannot close.

**Terrain traction was never the binding constraint.** Compliance, friction
level, collision primitive and command speed were each eliminated in turn, and
the conclusion drawn at the time was that dartsim's wheel-on-heightmap contact
was to blame. It is not. Measured directly, wheel surface speed against ground
speed over Rubicon is **24% slip** — real, but nothing like enough to explain
driving at a sixth of the command. Ground speed now tracks the *governed*
command essentially exactly; what had been limiting the robot was the governor,
for the reasons in the section below.

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

**The pitch axis has a tipping angle too, and it is the tighter one to think
about.** Everything in `magi_stabilizer` is built around the 31.4° roll figure,
because the stance can be widened to answer it. The pitch twin

```
atan(half_wheelbase / h_com) = atan(0.1934 / 0.311) = 31.9°
```

has **no lever at all**: widening is a roll remedy and the wheelbase is fixed.
Past 31.9° of nose-up the centre of mass is behind the rear contacts and the
robot goes over its own tail, and on Rubicon it did exactly that — driving onto
a face steeper than that, pitching −7° → −16° → −22° → −34° over three quarters
of a second and landing on its back.

Worth knowing what it is *not*: wheel drive torque stayed at **1.1 N·m of the
available 23.7** throughout, so the robot was not grinding against anything, it
was climbing something it should not have climbed. And the foot-force contact
test still reported all four wheels loaded at 73° of pitch, which is why the
guard in `_govern` keys off pitch and pitch rate instead.

The same geometry says the machine is inherently wheelie-prone under full
tractive effort — the no-wheelie condition is `mu < half_wheelbase / h_com` =
**0.62**, and `mu` is 1.0 because hill climbing needs it. Ordinary driving never
goes near it (accelerating at the governed 1.5 m/s² needs 29 N of the 191 N the
ground could deliver), but it is the reason the rear-up guard exists rather than
a speed limit.

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

Three things that script handles and a plain download does not:

* The Fuel URL **truncates** — a straight `curl` stopped at 128 MB of the real
  187,430,027 bytes and produced a corrupt zip. It uses `--retry`/`-C -` and
  verifies with `unzip -t`.
* The upstream heightmap collision declares **no friction surface at all**, so
  it falls back to Gazebo's default. Adding an explicit `mu` of 1.5 took driving
  from 16% to 28% of commanded speed. The script re-applies that patch.
* The heightmap itself is rebuilt — see below.

### The heightmap is a staircase, and it is rebuilt

`Heightmap.png` ships as an **8-bit** greyscale image stretched over 5 m of
relief. One grey level is therefore

```
5.0 / 255 = 19.6 mm
```

and the terrain is not a surface but a flight of stairs: flat plateaus with
~2 cm risers, one every 7.3 cm cell. Measured on the shipped asset, the mean
cell-to-cell step is **15.6 mm** — essentially one quantisation level
everywhere, i.e. over most of the map the relief the author drew is smaller
than the format can represent.

For an 86 mm wheel that is not cosmetic. Mounting a step of height `h` with a
wheel of radius `r` needs a tractive force of `sqrt(2rh - h²)/(r - h)`, which
at `h` = 19.6 mm is **0.82** against a friction coefficient of 1.0. Every cell
boundary is a near-stall obstacle, and every one that is cleared delivers an
impulse into the body — the bogging down and the rolling over, from one cause.

[`rebuild_heightmap.py`](src/magi_gazebo/scripts/rebuild_heightmap.py) fixes it
without changing the terrain the author drew. The true surface is known to lie
within half a grey level of each sample, so the staircase comes out by smoothing
under exactly that constraint — Laplacian relaxation, clamped back into
`[h ± ½ level]` every pass — and the result is then resampled to 1025×1025 and
written as **16 bit**, where one level is 0.076 mm instead of 19.6.

| | resolution | mean step | slope p50 |
|---|---|---|---|
| shipped, 8-bit | 513² (7.3 cm cells) | 15.6 mm | 15.0° |
| rebuilt, 16-bit | 1025² (3.7 cm cells) | **6.7 mm** | **11.6°** |

No sample moves more than **9.8 mm**, so every rock, tree and structure placed
against the original terrain stays where it was put. The slope figure falls
without the terrain changing shape, because most of that 15° was the staircase
rather than the hill.

Worth **1.52 m → 2.18 m** of ground covered per 8 s run over the standard
course (see [Measured behaviour](#measured-behaviour)). The original
`Heightmap.png` is left in place; only `model.sdf` is repointed, so reverting is
a one-line change.

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
