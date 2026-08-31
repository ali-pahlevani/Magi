#!/usr/bin/env python3
"""Dynamic tipover prevention for the MAGI Go2W.

WHAT THIS REPLACES, AND WHY
---------------------------
magi_balance.py keeps the centre of pressure inside the support polygon using
three levers -- tilt the body, splay the hips, crouch. That is the right idea
and it is kept here. What it could not do is stop the robot rolling over on
Rubicon at speed, for four reasons that this node addresses one by one.

  1. IT NEVER TOUCHED THE COMMAND.
     The old node subscribed to the commanded twist and used it only to guess a
     lean angle. Nothing anywhere in the stack refused a command the robot
     could not survive. A quadruped that may not step has a hard, computable
     envelope of admissible (v, w) -- outside it no amount of leg motion helps,
     and inside it a rollover from commanded motion is impossible. This node
     sits IN the command path and projects the operator's twist onto that
     envelope. See THE GOVERNOR below. This is the single biggest change.

  2. IT MEASURED THE WRONG THING.
     "CoP inside the polygon" is a quasi-static yes/no. It needs three loaded
     feet to mean anything -- so it goes blind exactly when a wheel lifts, the
     one moment that matters -- and its units are metres of ground, which say
     nothing about how much acceleration is left. This node uses the
     force-angle stability measure instead (magi_leg_model.force_angle_margin):
     the angle the net force may still rotate through before the robot goes
     over its worst support edge. It is defined for two contacts, it is signed
     through the tipover, and its units are radians of remaining tilt.

  3. IT ASKED THE WRONG SENSOR.
     The old lean feed-forward computed v*w from the COMMAND and assumed the
     robot obeyed. The accelerometer already reports the answer: proper
     acceleration IS gravity plus every inertial term -- centrifugal, braking,
     terrain impulse, slope -- in one vector, measured rather than modelled.

     It cannot be used raw, and finding out why cost a day: an accelerometer
     reports the legs' OWN push on the body as tilt, so a loop that chases it
     tilts further, which pushes harder. On flat ground, standing still, that
     had two wheels unloaded within a second and the robot inverted a few
     seconds later. So the signal is split by frequency -- attitude from the
     quaternion carries the fast path, and the DIFFERENCE between the two,
     which is purely the inertial part, is low-passed hard and used as the lean
     reference. Same reference at DC, no feedback path. Its bias is estimated
     while the robot is at rest, where the inertial part is zero by
     construction and the bias is therefore exactly what is left over.

  4. ITS DAMPING TERM WAS DELAYED -- AND UNDELAYING IT WAS A MISTAKE.
     The old loop summed P, I and D and put the SUM through a 0.1 s low-pass to
     kill a 10 Hz limit cycle. This node was first written to filter the P+I
     path only, on the argument that delaying D delays the one term that
     arrests a divergent roll.

     That reasoning is right about what D is for and wrong about what this
     plant can carry, and it cost more than everything else here gained. The
     command is a body ATTITUDE realised through differential leg length; the
     path from asking for it to the body rolling runs through the command
     horizon, the trajectory controller, and the ~20 Hz mode of the body
     bouncing in roll against the hip PD, which is 180 deg of phase well before
     20 Hz. Undelayed rate feedback there is not damping, it is gain.

     Measured, standing still on level ground:

         kd 0.09/0.10, D unfiltered    roll rate 3.9 rad/s rms, 19.5 Hz
         kd 0                          roll rate 0.10 rad/s rms

     The accelerometer saw +/-200 m/s^2 of lateral acceleration during that
     limit cycle -- 20 g, on a stationary robot -- and everything downstream
     reads the accelerometer. The effective-gravity reference became garbage
     (a_y 130 m/s^2 against a_z 14), so the margin reported -32 deg on an
     upright robot, the acceleration envelope collapsed, urgency pinned at
     1.00, the stance splayed to its limit and stayed there, and the governor
     held the robot at its roughness crawl. It was not being conservative; it
     was shaking itself blind.

     So the low-pass is back on the whole command, at command_tau, and kd comes
     down to 0.05. Terrain disturbance is a few Hz at most, which is inside
     what the loop can still carry.

Also, the old configuration left most of the robot's stability on the table.
Measured from the kinematics (see the table in magi_stabilizer.yaml): the
nominal stance tips at 31.4 deg, the old widen_max 0.070 took that to 41.1 deg,
and the leg IK comfortably reaches a stance that tips at 45.6 deg without
losing any ground clearance at all. Splaying was worth measuring rather than
assuming -- a wide stance cambers the wheels 18 deg and rolls them onto their
rim edges, which sounded like it should cost more traction than it bought. It
does not: driven at 0.7 m/s over Rubicon from a fixed pose, three runs each,
widen_max 0.110 survived 2/3 and covered 2.79 m, 0.070 survived 2/3 and
covered 2.07 m, and no splay at all survived 0/3 and covered 1.49 m.

THE STABILITY MEASURE
---------------------
Effective gravity  g_eff = -a_imu, unit vector, straight off the accelerometer.
Support pattern    the wheel contact points of the LOADED wheels, computed as
                   the true contact of a cambered cylinder rather than "wheel
                   centre minus a radius", which is wrong by up to 3 cm once
                   the hips splay.
Margin             force_angle_margin(pattern, com, g_eff), radians.

and the same evaluated PREDICTIVELY, by rotating g_eff forward through the
measured body rates by predict_time. A rollover is divergent: by the time the
static margin is small the outcome is already decided, so every consumer here
uses min(now, predicted).

THE GOVERNOR
------------
Given the measured pattern and CoM, the margin is a function of any hypothetical
extra horizontal acceleration. Bisecting on that function gives four numbers:
the lateral acceleration available to the left and to the right, and the
longitudinal acceleration available forward and back, each defined as the
acceleration at which the margin would fall to `tip_margin_safe`.

A steady turn at (v, w) needs lateral acceleration v*w. So the admissible set is

    a_right <= v * w <= a_left

and the commanded twist is scaled uniformly, both components by the same
factor, until it is inside. Uniform scaling is deliberate: it preserves v/w,
the path curvature, so the robot follows the operator's intended arc more
slowly rather than driving off it. Longitudinal limits go to the acceleration
and braking rate the same way.

Because the bisection uses the MEASURED pattern and the MEASURED effective
gravity, the envelope automatically shrinks on a side slope, on a bump, when a
wheel unloads, and when the stance has not finished widening -- none of which
needs a model.

INTERACTION WITH THE REST OF THE STACK
--------------------------------------
The governor publishes the twist that the wheel controller consumes, so it must
sit in the command path: teleop and navigation publish to `cmd_in` (default
/cmd_vel) and this node republishes to `cmd_out` (default
/wheel_controller/cmd_vel_unstamped). To keep the old direct-drive path usable
for A/B runs, the governor STAYS SILENT until it receives its first cmd_in
message -- until then something else may own the output topic.

Splaying changes the effective track, which diff_drive_controller does not know
about; it converts wheel speeds to a twist using a fixed 0.380 m separation. So
while splayed, commanded yaw rate under-delivers and the odometry message
over-reads yaw. The EKF is unaffected -- ekf.yaml takes vx only from odom0 and
yaw rate from the IMU -- and the current effective track is published on
/magi/stance_width so the discrepancy stays visible.

WHAT WAS MEASURABLY BROKEN, BESIDES THAT
----------------------------------------
Four more faults, each found by measurement and each worth its own note in
magi_stabilizer.yaml:

  * THE ACCELERATION ENVELOPE WAS NEVER COMPUTED. `_accel_envelope` built its
    effective-gravity vector as -down*G, which points UP, so every bisection
    probe failed the "pattern must be below the CoM" test and both searches
    returned a_lat_floor unconditionally. The governor had been running on a
    hard-coded 0.8 m/s^2 of lateral authority whatever the stance was good for
    -- 10.0 m/s^2 at full splay.

  * THE TERRAIN PREVIEW SCORED HILLS AS ROUGHNESS. A plane fit over a 2.8 m
    window leaves a hill's curvature in the residual. Measured over 600 random
    placements on Rubicon the plane residual is p50 5.3 cm and p90 16.5 cm on
    ground that is merely sloped, against a 4.5 cm "fully rough" reference --
    so the preview read FULLY ROUGH over 57% of the map. It fits a quadratic
    now (p50 3.4 cm), and both it and the slope term are gated by time to
    arrival, because the lidar cannot see the ground closer than 2.69 m and at
    a walking pace that window is eight seconds away.

  * THE STANCE WAS FRICTION-LOCKED. Dragging four loaded wheels sideways costs
    mu*N through the stance height, 17.3 N.m at the hip, on top of the splay
    moment, against a 23.7 N.m limit. Measured standing on Rubicon at 0.413 rad
    of splay with the target set back to nominal: both front hips pinned at
    exactly -23.700 N.m and the splay did not move for as long as the robot
    stood there, through a sweep of widen_max from 0.110 to 0. Six seconds of
    rolling brought the same joint back to 0.038 rad. The lever is gated on
    rolling now, and the command is kept within reach of the achieved stance.

  * NOTHING GUARDED THE PITCH AXIS. The whole design is built around the
    31.4 deg roll tipping angle; the pitch twin, atan(0.1934/0.311) = 31.9 deg
    of nose-up, had no lever and no guard. The robot drove onto faces steeper
    than that and went over backwards. See the rear-up guard in _govern.

WHAT IT ACTUALLY DOES, MEASURED
------------------------------
Rubicon, spawned at (4.0, -0.5, 1.80), reset to that pose before every run,
8 s per run, three runs per profile. `roll` is the worst absolute roll seen and
`dist` the ground covered:

    profile              magi_balance            magi_stabilizer
    v 0.7                2/3   roll 45  3.66 m   3/3   roll 11  3.09 m
    v 1.2                0/3   roll 137 3.89 m   8/8   roll  9  2.90 m
    v 0.9  w 0.5         1/3   roll 99  3.34 m   3/3   roll 11  3.71 m
    v 1.5  w 1.2         1/3   roll 85  2.48 m   7/8   roll 12  3.19 m
    ----------------------------------------------------------------
    total                4/12                    21/22

The two hardest profiles were repeated five more times each after the first
three, which is where the 8s and the difference between 2/3 and 7/8 come from;
a single run on this terrain is noise and the old node's own test tool says so.

Standing still on Rubicon it is also quieter than the old node -- body rate
0.005 rad/s rms against 0.017, with all four wheels loaded against 3.3.

WHAT THIS IS NOT
----------------
Still quasi-static, still no stepping, and 11/12 is not 12/12. The guarantee
has a shape worth stating plainly: the governor makes a rollover caused by the
COMMANDED motion much harder, because it refuses commands outside the measured
envelope, and the table above is what that is worth. It cannot promise the same
against the terrain -- a drop-off, a rock that lifts one side faster than the
legs can answer, or a slope steeper than the stance can level. Against those it
widens, crouches, leans and brakes sooner and harder than the old node, and the
predictive margin gives it more warning, but "never rolls over" is not a claim
any wheeled machine without a recovery step can honestly make, and this one
does not make it.

The governing costs ground speed: roughly 15-25% of the distance covered in the
runs above, which is the price of the runs finishing upright.

TOPICS
------
  subscribes
    /joint_states                        leg configuration
    /imu/data                            attitude, body rates, proper acceleration
    /foot_force/{FL,FR,RL,RR}            ground reaction, calf frame
    cmd_in  (default /cmd_vel)           operator/navigation twist
    /lidar/points                        terrain preview (optional)
    /magi/body_height  (Float64)         ride-height target
    /magi/posture      (String)          named ride heights
    /magi/balance_enable (Bool)          false freezes stance AND governor

  publishes
    /leg_controller/joint_trajectory     the 12 joint targets, streamed
    cmd_out (default                     governed twist
        /wheel_controller/cmd_vel_unstamped)
    /magi/tip_margin       (Float64)     force-angle margin now, radians
    /magi/tip_margin_pred  (Float64)     the same, predicted forward
    /magi/stability_margin (Float64)     CoP distance to the edge, metres
                                         (kept so the old test tool still reads)
    /magi/speed_limit      (Float64)     governor's current speed ceiling
    /magi/support_polygon  (PolygonStamped)
    /magi/cop              (PointStamped)
    /magi/com              (PointStamped)
    /magi/stance_width     (Float64)     current effective track
    /magi/balance_state    (String)      one-line status
"""

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import (Point32, PointStamped, PolygonStamped, Twist,
                               Wrench)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, PointCloud2
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from magi_leg_model import (G, HALF_TRACK_NOM, HIP_X, JOINT_NAMES, LEGS, SIGN,
                            TOTAL_MASS, WHEEL_R, body_com, clamp,
                            force_angle_margin, leg_fk, leg_ik, link_rotation,
                            order_ccw, order_contacts_ccw, plane_basis,
                            polygon_margin, rx, ry)

# Wheel collision cylinder, from gen_body.py: radius 0.086, width 0.0518.
WHEEL_HALF_W = 0.0259

NAMED_POSTURES = {"tall": 0.46, "stand": 0.40, "crouch": 0.30, "sit": 0.22}


def wheel_contact(centre, axis, down, radius=WHEEL_R, half_width=WHEEL_HALF_W):
    """Lowest point of a cambered wheel.

    The wheel is a cylinder, not a sphere, so once the hip splays the contact
    is not `centre - radius * down`: it migrates to the rim. At the 26 deg of
    camber a full splay produces, the difference is 27 mm laterally -- which is
    14% of the half-track, applied to every stability number this node
    computes, so it is worth getting right.

    centre  wheel centre, base frame
    axis    wheel spin axis, unit, base frame
    down    direction of effective gravity, unit, base frame
    """
    along = float(np.dot(down, axis))          # = sin(camber)
    perp = down - along * axis
    norm = np.linalg.norm(perp)
    if norm < 1e-6:
        # Axis vertical: the wheel is lying on its rim face.
        return centre + math.copysign(half_width, along) * axis
    # The rim term. Strictly, a rigid cylinder on a rigid plane touches at the
    # rim edge for ANY non-zero camber and along a whole line at exactly zero,
    # so the exact extreme point jumps by the full half width as the camber
    # crosses zero. That discontinuity is real geometry but a terrible control
    # input, and it is not what a compliant wheel on compliant ground does
    # either -- the patch migrates. Scaling the offset by sin(camber) gives the
    # same answer at both ends and migrates smoothly between them.
    return centre + radius * (perp / norm) + half_width * along * axis


class MagiStabilizer(Node):
    def __init__(self):
        super().__init__("magi_stabilizer")

        p = self.declare_parameter
        p("rate", 100.0)
        # How far ahead each streamed trajectory point is placed. Each message
        # supersedes the last, so this behaves as a first-order lag with about
        # this time constant -- it is the dominant delay in the fast path, and
        # 0.03 rather than the old 0.05 is most of the bandwidth this node
        # gains back.
        p("command_horizon", 0.03)
        p("controller", "leg_controller")
        p("cmd_in", "/cmd_vel")
        p("cmd_out", "/wheel_controller/cmd_vel_unstamped")

        p("ride_height", 0.36)        # see the note in magi_stabilizer.yaml
        p("stand_time", 2.5)
        p("startup_delay", 2.0)
        p("authority_ramp", 0.75)      # s to fade the attitude loop in

        # ---- attitude loop -------------------------------------------------
        # The reference is "body z along effective gravity", read off the
        # accelerometer, so these gains close a loop on a measured error and
        # need no separate feed-forward.
        p("roll_kp", 0.75)
        p("roll_ki", 0.60)
        p("roll_kd", 0.05)
        p("pitch_kp", 0.55)
        p("pitch_ki", 0.50)
        p("pitch_kd", 0.05)
        p("tilt_i_max", 0.15)
        p("tilt_max", 0.35)
        # Low-pass on the P+I path ONLY. The D term bypasses it.
        p("command_tau", 0.05)
        p("gyro_tau", 0.05)
        p("accel_tau", 0.20)          # s, low-pass on the accelerometer
        # The lean reference is the inertial part of the measured gravity
        # direction. It must be SLOW: fast content in it is the leg-push
        # feedback path, not a real demand. See the note in _update.
        p("lean_tau", 0.40)
        p("lean_max", 0.25)           # rad, cap on the commanded lean
        # Accelerometer bias, learned only while genuinely at rest -- see the
        # note in _update for why rest is exactly when it is observable.
        p("accel_bias_tau", 8.0)
        p("accel_bias_max", 0.05)

        # ---- margin-driven CoM bias ----------------------------------------
        # Below bias_margin the body is actively leaned AWAY from the critical
        # support edge, on top of the levelling loop. This is the "keep the
        # centre of gravity inside the triangle" action in its most direct
        # form: it moves the CoM rather than waiting for the attitude loop to.
        p("bias_margin", 0.30)        # rad (17 deg) of margin below which it acts
        p("bias_gain", 0.6)
        p("bias_max", 0.10)           # rad of extra body tilt

        # ---- prediction ----------------------------------------------------
        p("predict_time", 0.12)       # s of body-rate lookahead
        p("predict_max", 0.12)        # rad, cap on the predicted rotation

        # ---- reshaping -----------------------------------------------------
        # Rate limits are ASYMMETRIC and that is the point. Widening scrubs the
        # wheels sideways, which is why the old node moved at 0.12 m/s in both
        # directions and could not answer a terrain impulse -- a full widen took
        # 0.6 s. But scrub is a cost worth paying instantly to avoid a rollover
        # and not worth paying at all to get back to nominal, so going wide is
        # fast and coming back is slow.
        p("reshape_roll_speed", 0.30)   # m/s for the full reshape rate
        p("reshape_rest_gain", 0.35)    # fraction of it available at a standstill
        p("widen_band", 0.015)          # m the command may lead the real stance
        p("reshape_rate_up", 0.30)    # m/s toward safety
        p("reshape_rate_down", 0.10)  # m/s back to nominal
        p("margin_target", 0.25)      # rad (14 deg); below this, reshape
        p("margin_tau", 0.20)
        p("speed_calm", 0.25)         # m/s below which no anticipation
        p("speed_wide", 1.20)         # m/s at which the stance is fully wide
        p("rough_ref", 0.90)          # rad/s RMS body rate = fully wide
        p("rough_tau", 1.0)
        p("rough_floor", 0.080)       # rad/s of residual gyro noise
        # 0.110 m of extra half-track takes the track from 0.380 to 0.600 m and
        # the tipping angle from 31.4 to 43.7 deg, for 26 deg of hip against a
        # 60 deg limit. Widening costs no ground clearance, so it is the lever
        # to lean on.
        p("widen_max", 0.110)
        # Crouching does cost clearance, and on Rubicon that has already been
        # measured to be dangerous: at 0.10 the old node reached 0.30 m ride
        # height, grounded out, and three feet fell to ~5 N against 191 N of
        # weight. So crouch is GATED ON SMOOTHNESS -- available in full on flat
        # ground where it buys cornering stability, withdrawn over rough ground
        # where clearance matters more.
        p("crouch_max", 0.090)
        p("crouch_rough_gate", 0.60)  # fraction of crouch withdrawn when rough
        p("reach_max", 0.050)         # m of extra extension to chase lost contact
        p("reach_rate", 0.06)         # m/s -- slower than the width levers
        p("contact_debounce", 0.35)   # s before a contact change is believed
        p("urgency_rise_tau", 0.15)
        p("urgency_fall_tau", 1.20)

        p("dz_max", 0.110)
        p("contact_force", 5.0)
        p("contact_hold", 0.25)       # s a contact counts after going light
        p("force_sign", 0.0)

        # ---- governor ------------------------------------------------------
        p("governor_enable", True)
        p("governor_rate", 50.0)
        # Margin the envelope is defined at. The governor allows exactly the
        # acceleration that would bring the margin down to this, so it is the
        # reserve held back for the terrain.
        p("tip_margin_safe", 0.20)    # rad, 11.5 deg
        p("tip_margin_crit", 0.12)    # rad, 7 deg -- below this, intervene hard
        p("v_max", 1.5)
        p("w_max", 1.5)
        p("accel_max", 1.5)           # m/s^2 on the governed command
        p("yaw_accel_max", 2.0)       # rad/s^2 on the governed command
        p("cmd_timeout", 0.5)         # s without input -> stop
        p("a_lat_floor", 0.8)         # m/s^2 -- never govern below this
        p("a_lat_ceiling", 12.0)      # m/s^2 -- bisection upper bracket
        p("v_rough_min", 0.60)        # m/s ceiling over fully rough ground
        p("v_crit_min", 0.25)         # m/s still allowed with no margin left
        p("tip_stop_rate", 1.5)       # rad/s: critical margin + this = going over
        p("tip_stop_hold", 0.6)       # s the drive stays cut after it triggers
        p("drive_tau", 0.20)          # s, low-pass on the wheel-torque readout
        p("rear_pitch", 0.35)         # rad of nose-up before the rear-up guard arms
        p("rear_rate", 0.50)          # rad/s of further nose-up that triggers it
        p("pitch_rate_tau", 0.10)     # s, low-pass on the measured pitch rate
        p("rear_stop_hold", 0.8)      # s the drive stays cut after the guard fires

        # ---- terrain preview -----------------------------------------------
        p("preview_enable", True)
        # The lowest ray reaches the ground 2.69 m out, so the window sits
        # there rather than just ahead of the feet. See the terrain-preview
        # notes in magi_stabilizer.yaml.
        p("preview_near", 2.60)       # m ahead, start of the window
        p("preview_far", 5.40)        # m ahead, end of the window
        p("preview_half_width", 0.80)
        p("preview_tau", 0.30)
        p("preview_rough_ref", 0.10)  # m RMS of surface residual = fully rough
        p("preview_slope_ref", 0.60)  # rad of upcoming slope = fully urgent
        p("preview_lead", 3.0)        # s; ground further off than this is ignored
        p("preview_rough_floor", 0.020)  # m, the lidar's own range noise

        self.rate = float(self.get_parameter("rate").value)
        self.dt = 1.0 / self.rate
        self.horizon = float(self.get_parameter("command_horizon").value)
        self.stand_time = float(self.get_parameter("stand_time").value)

        # ---- state ---------------------------------------------------------
        self.q = {}
        self.roll = self.pitch = 0.0
        self.wx = self.wy = self.wz = 0.0
        self.accel = np.array([0.0, 0.0, G])   # filtered proper acceleration
        self.have_imu = False
        self.force = {}
        self.last_loaded = {}
        self.force_sign = float(self.get_parameter("force_sign").value)

        self.height_target = float(self.get_parameter("ride_height").value)
        self.height = None
        self.widen = 0.0
        self.crouch = 0.0
        self.reach = 0.0
        self.n_contacts = 0
        self.contact_ok = True
        self.lost_t = self.held_t = 0.0
        self.margin = 0.0             # force-angle margin, rad, filtered
        self.margin_pred = 0.0
        self.cop_margin = 0.0         # metres, published for continuity
        self.rough_sq = 0.0
        self.tip_stop_until = 0.0     # see the tip-stop latch in _govern
        self.rear_stop_until = 0.0    # see the wheelie latch in _govern
        self.drive_effort = 0.0       # mean |wheel torque|, low-passed
        self.pitch_rate = 0.0         # rad/s, nose-down positive
        # margin, speed, body roughness, commanded lateral, preview, slope
        self.u_terms = (0.0,) * 6
        self.urgency = 0.0
        self.i_roll = self.i_pitch = 0.0
        self.pi_roll_f = self.pi_pitch_f = 0.0
        self.lean_roll = self.lean_pitch = 0.0
        self.bias_roll = self.bias_pitch = 0.0
        self.enabled = True
        self.phase = "wait"
        self.stand_t = 0.0
        self.authority = 0.0
        self.last_cmd = None
        self.critical_dir = None      # body-frame unit vector toward the edge

        # Governor state.
        self.cmd_in = Twist()
        self.cmd_in_time = None
        self.have_cmd = False
        self.v_out = self.w_out = 0.0
        self.a_lat = [4.0, 4.0]       # available lateral accel, [+y, -y]
        self.a_lon = [4.0, 4.0]       # available longitudinal accel, [+x, -x]
        self.v_limit = float(self.get_parameter("v_max").value)
        self._pattern = None          # (ordered contacts, com, down) for the governor

        # Preview state.
        self.preview_slope = 0.0
        self.preview_rough = 0.0

        # ---- interfaces ------------------------------------------------------
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, sensor_qos)
        for leg in LEGS:
            self.create_subscription(
                Wrench, f"/foot_force/{leg}",
                lambda m, k=leg: self.force.__setitem__(k, m), sensor_qos)
        self.create_subscription(
            Twist, self.get_parameter("cmd_in").value, self._on_cmd, 10)
        self.create_subscription(Float64, "/magi/body_height", self._on_height, 10)
        self.create_subscription(String, "/magi/posture", self._on_posture, 10)
        self.create_subscription(Bool, "/magi/balance_enable", self._on_enable, 10)
        if bool(self.get_parameter("preview_enable").value):
            self.create_subscription(
                PointCloud2, "/lidar/points", self._on_cloud, sensor_qos)

        controller = self.get_parameter("controller").value
        self.pub_traj = self.create_publisher(
            JointTrajectory, f"/{controller}/joint_trajectory", 10)
        self.pub_cmd = self.create_publisher(
            Twist, self.get_parameter("cmd_out").value, 10)
        self.pub_tip = self.create_publisher(Float64, "/magi/tip_margin", 10)
        self.pub_tip_pred = self.create_publisher(
            Float64, "/magi/tip_margin_pred", 10)
        self.pub_margin = self.create_publisher(
            Float64, "/magi/stability_margin", 10)
        self.pub_speed = self.create_publisher(Float64, "/magi/speed_limit", 10)
        self.pub_poly = self.create_publisher(
            PolygonStamped, "/magi/support_polygon", 10)
        self.pub_cop = self.create_publisher(PointStamped, "/magi/cop", 10)
        self.pub_com = self.create_publisher(PointStamped, "/magi/com", 10)
        self.pub_width = self.create_publisher(Float64, "/magi/stance_width", 10)
        self.pub_state = self.create_publisher(String, "/magi/balance_state", 10)

        self.start_time = None
        self.create_timer(self.dt, self._update)
        self.create_timer(1.0 / float(self.get_parameter("governor_rate").value),
                          self._govern)
        self.create_timer(1.0, self._report)

        self.get_logger().info(
            f"stabilizer up: {self.rate:.0f} Hz, ride height "
            f"{self.height_target:.2f} m, safe margin "
            f"{math.degrees(float(self.get_parameter('tip_margin_safe').value)):.0f} deg, "
            f"widen_max {float(self.get_parameter('widen_max').value):.3f} m, "
            f"mass {TOTAL_MASS:.2f} kg; governor "
            f"{'ON' if self.get_parameter('governor_enable').value else 'OFF'} "
            f"{self.get_parameter('cmd_in').value} -> "
            f"{self.get_parameter('cmd_out').value}")

    # ---- callbacks ---------------------------------------------------------
    def _on_joints(self, msg):
        index = {n: i for i, n in enumerate(msg.name)}
        for leg in LEGS:
            try:
                self.q[leg] = tuple(
                    msg.position[index[f"{leg}_{part}_joint"]]
                    for part in ("hip", "thigh", "calf"))
            except (KeyError, IndexError):
                return
        # Wheel drive torque, low-passed. This is the robot's own ammeter and
        # it says something nothing else in the stack can: whether the wheels
        # are DRIVING or PUSHING. See the stall latch in _govern.
        try:
            drive = sum(abs(msg.effort[index[f"{leg}_foot_joint"]])
                        for leg in LEGS) / 4.0
        except (KeyError, IndexError):
            return
        tau = float(self.get_parameter("drive_tau").value)
        a = 0.005 / (tau + 0.005)              # joint_states arrives at 200 Hz
        self.drive_effort += a * (drive - self.drive_effort)

    def _on_imu(self, msg):
        q = msg.orientation
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))
        # Rate of the ATTITUDE estimate rather than the raw gyro, so the sign
        # convention is the same one the rear-up guard's threshold is written
        # in and cannot be got backwards. Filtered at pitch_rate_tau, long
        # enough that a wheel dropping off a facet does not register.
        prate = (pitch - self.pitch) * float(self.get_parameter("rate").value)
        a = 0.005 / (float(self.get_parameter("pitch_rate_tau").value) + 0.005)
        self.pitch_rate += a * (prate - self.pitch_rate)
        self.pitch = pitch

        dt = 1.0 / 200.0                       # IMU rate, from the URDF sensor
        a = dt / (float(self.get_parameter("gyro_tau").value) + dt)
        self.wx += a * (msg.angular_velocity.x - self.wx)
        self.wy += a * (msg.angular_velocity.y - self.wy)
        self.wz += a * (msg.angular_velocity.z - self.wz)

        # Proper acceleration. The IMU is mounted with identity rotation to
        # base (checked against TF: base->imu is a pure 0.026/0.042 offset), so
        # these components are already body axes and need no rotation. The
        # lever arm adds w_dot x r with |r| = 0.05 m, which the low-pass below
        # takes care of.
        b = dt / (float(self.get_parameter("accel_tau").value) + dt)
        raw = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        self.accel = self.accel + b * (raw - self.accel)
        self.have_imu = True

    def _on_cmd(self, msg):
        self.cmd_in = msg
        self.cmd_in_time = self.get_clock().now()
        self.have_cmd = True

    def _on_height(self, msg):
        self.height_target = clamp(float(msg.data), 0.20, 0.48)

    def _on_posture(self, msg):
        name = msg.data.strip().lower()
        if name in NAMED_POSTURES:
            self.height_target = NAMED_POSTURES[name]
        else:
            self.get_logger().warn(
                f"unknown posture '{msg.data}'; known: {', '.join(NAMED_POSTURES)}")

    def _on_enable(self, msg):
        self.enabled = bool(msg.data)
        self.get_logger().info(
            f"stabilizer {'enabled' if self.enabled else 'FROZEN (fixed stance, governor bypassed)'}")

    def _on_cloud(self, msg):
        """Upcoming slope and roughness, from a plane fit ahead of the robot.

        The point of this is anticipation. Widening takes ~0.2 s at the rate
        limit above and crouching longer; a bump met at 1 m/s arrives with no
        warning at all from the IMU, which only reports terrain the robot has
        already hit. The lidar sees it a second and a half out.
        """
        if msg.height * msg.width == 0 or msg.point_step < 12:
            return
        try:
            names = {f.name: f for f in msg.fields}
            if not {"x", "y", "z"} <= names.keys():
                return
            # FLOAT32 == 7. Anything else and the offsets below would be
            # reinterpreting the wrong bytes rather than failing loudly.
            if any(names[c].datatype != 7 for c in ("x", "y", "z")):
                return
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            raw = raw[: (len(raw) // msg.point_step) * msg.point_step]
            pts = raw.reshape(-1, msg.point_step)
            xyz = np.column_stack([
                pts[:, names[c].offset:names[c].offset + 4].copy().view(np.float32).ravel()
                for c in ("x", "y", "z")]).astype(np.float64)
        except (ValueError, KeyError):
            return

        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.shape[0] < 50:
            return

        # lidar -> base: fixed mount, (0.232, 0, 0.104) with 0.08 rad (4.58
        # deg) of downward pitch (checked against TF).
        xyz = xyz @ ry(0.08).T + np.array([0.232, 0.0, 0.104])

        near = float(self.get_parameter("preview_near").value)
        far = float(self.get_parameter("preview_far").value)
        half = float(self.get_parameter("preview_half_width").value)
        # Look where the robot is going; reversing needs the window behind it.
        ahead = -1.0 if self.v_out < -0.05 else 1.0
        x = xyz[:, 0] * ahead
        # z band, in base frame. Sized for the window's reach: at 5.4 m out
        # even a 10 deg grade is 0.95 m of rise or fall, and clipping that
        # would flatten exactly the slope worth previewing.
        sel = ((x > near) & (x < far) & (np.abs(xyz[:, 1]) < half)
               & (xyz[:, 2] > -1.50) & (xyz[:, 2] < 1.10))
        window = xyz[sel]
        if window.shape[0] < 40:
            return

        # The MID-360 only looks 7 deg below its own horizon and sits ~0.50 m
        # up; the 4.58 deg downward mount puts the lowest ray 11.58 deg down,
        # so the ground first appears 2.69 m out and four rings (2.69, 3.18,
        # 3.92, 5.13 m) fall inside the window, ~200 points. A window that
        # catches only a thin band of them makes an ill-conditioned plane fit,
        # which comes back as tens of degrees of imaginary slope. Require real
        # extent in both axes.
        if np.ptp(window[:, 0]) < 0.80 or np.ptp(window[:, 1]) < 0.50:
            return

        # Least-squares QUADRATIC z = c0 + c1x + c2y + c3x^2 + c4xy + c5y^2.
        #
        # The window is 2.8 m deep. Over that distance a HILL is not a plane,
        # and a plane fit to one leaves its curvature in the residual -- which
        # this node would then read as roughness. Measured over 600 random
        # placements on Rubicon, the plane residual has a median of 5.3 cm and
        # a p90 of 16.5 cm on ground that is merely sloped, so against the
        # 4.5 cm reference the preview reported FULLY ROUGH on 57% of the map.
        # The consequences were not subtle: urgency pinned at 1.00, the stance
        # permanently splayed to its 0.600 m limit, and the governor holding
        # the robot at its v_rough_min crawl of 0.35 m/s no matter what was
        # commanded. That is the whole "it hardly moves" complaint.
        #
        # Adding the six quadratic terms puts curvature in the MODEL instead of
        # the residual: median 3.4 cm, p90 8.6 cm. What is left is texture --
        # rocks, ruts, steps -- which is what the term is supposed to measure.
        # The linear coefficients still give the slope, unchanged.
        x_w, y_w, z_w = window[:, 0], window[:, 1], window[:, 2]
        a = np.column_stack([np.ones(window.shape[0]), x_w, y_w,
                             x_w * x_w, x_w * y_w, y_w * y_w])
        try:
            # lstsq is minimum-norm under rank deficiency, so a window that
            # happens to catch only two lidar rings degrades to the plane fit
            # rather than blowing up.
            coef, *_ = np.linalg.lstsq(a, z_w, rcond=None)
        except np.linalg.LinAlgError:
            return
        residual = z_w - a @ coef
        slope = math.atan(math.hypot(coef[1], coef[2]))
        # Range noise is 2 cm stddev by the datasheet and the sensor model, so a
        # fit to genuinely flat ground still shows that much scatter. Removed in
        # quadrature, exactly as the gyro floor is, so smooth ground reports
        # zero roughness instead of the sensor's own noise -- otherwise the
        # stance sits permanently part-widened with nothing happening.
        floor = float(self.get_parameter("preview_rough_floor").value)
        raw_rough = float(np.sqrt(np.mean(residual * residual)))
        rough = math.sqrt(max(raw_rough * raw_rough - floor * floor, 0.0))

        # GATED BY TIME TO ARRIVAL. The MID-360 looks only 7 deg below its own
        # horizon, so from 0.50 m up its lowest ray does not reach the ground
        # until 2.69 m out -- the preview window cannot be moved closer. At
        # 0.5 m/s the middle of that window is eight seconds away, and levers
        # that take 0.4 s to move have no business reacting to it: on a basin
        # world like Rubicon there is a steep wall within 5.4 m of almost
        # anywhere, so an ungated preview held the stance splayed and the
        # governor at its crawl floor permanently, including while the robot
        # was standing still with the window pointing at a hillside it was
        # never going to drive into.
        #
        # The gate is the fraction of preview_lead that the window actually
        # falls inside at the current speed: full weight when the robot will be
        # there within preview_lead seconds, fading to nothing when it is not
        # going anywhere near it.
        lead = float(self.get_parameter("preview_lead").value)
        centre = 0.5 * (near + far)
        eta = centre / max(abs(self.v_out), 1e-3)
        gate = clamp(lead / max(eta, 1e-3), 0.0, 1.0)

        tau = float(self.get_parameter("preview_tau").value)
        alpha = 0.1 / (tau + 0.1)              # cloud arrives at 10 Hz
        self.preview_slope += alpha * (gate * slope - self.preview_slope)
        self.preview_rough += alpha * (gate * rough - self.preview_rough)

    # ---- stability measurement --------------------------------------------
    def _geometry(self):
        """Contact points, loads, CoM and effective gravity, all in base frame.

        Returns (contacts, loaded, com, down, loads). `contacts` has every
        wheel, `loaded` only those carrying weight -- the caller wants both,
        because the support pattern is the loaded set but the polygon drawn for
        the operator is all four.

        This NEVER fails. An earlier version bailed out when the foot sensors
        had nothing to say, which deadlocked the robot: land somewhere the
        wheels are momentarily unloaded, the node declines to command the legs,
        the legs stay where they are, the robot settles onto its belly, and the
        sensors it was waiting for stay at zero forever. Measured on Rubicon,
        the robot sat 0.29 m below its stance height for the whole run without
        the controller ever issuing a single command. Degrading to a sensible
        assumption always beats declining to act.
        """
        norm = float(np.linalg.norm(self.accel))
        if norm >= 3.0:
            down = -self.accel / norm
        else:
            # Near free fall, so the accelerometer cannot say which way is
            # down. The attitude estimate still can.
            down = (ry(self.pitch) @ rx(self.roll)).T @ np.array([0.0, 0.0, -1.0])

        com = body_com(self.q)
        contacts = {}
        loads = {}
        threshold = float(self.get_parameter("contact_force").value)

        for leg in LEGS:
            qh, qt, qc = self.q[leg]
            centre = leg_fk(leg, qh, qt, qc)
            # Wheel spin axis: the calf frame's y, which the thigh/calf pitch
            # rotations leave alone, so only the hip abduction tilts it.
            axis = rx(qh) @ np.array([0.0, 1.0, 0.0])
            contacts[leg] = wheel_contact(centre, axis, down)

            w = self.force.get(leg)
            if w is None:
                continue
            f_calf = np.array([w.force.x, w.force.y, w.force.z])
            if np.linalg.norm(f_calf) < 1e-9:
                continue
            # The sensor reports in the calf frame; rotate to base and take the
            # component along effective up.
            f_b = link_rotation(leg, qh, qt, qc) @ f_calf
            loads[leg] = -float(np.dot(f_b, down))

        if not loads:
            # No usable force reading. Assume every wheel is a contact, which
            # is what the legs are being commanded to make true anyway.
            return contacts, list(LEGS), com, down, {leg: 0.0 for leg in LEGS}

        # measure_direction is parent_to_child, so a loaded wheel reads negative
        # along world-up. Rather than trust that, settle the sign once from the
        # data: whichever choice makes the robot's own weight come out positive.
        #
        # The threshold is a QUARTER OF THE ROBOT'S WEIGHT, not the 1 N the old
        # node used. 1 N is satisfied by sensor noise during the drop onto the
        # ground, and the sign is latched forever: resolving it from an 8.5 N
        # reading against 191 N of weight picked the wrong sign, which inverted
        # the loaded set, which pegged the urgency, which threw the robot into
        # a full splay it did not need and could not undo.
        if self.force_sign == 0.0:
            total = sum(loads.values())
            if abs(total) < 0.25 * TOTAL_MASS * G:
                # Not standing on it yet. Fall back to every wheel being a
                # potential contact, which is right while the legs are still
                # coming down and keeps the levelling loop fed.
                return contacts, list(LEGS), com, down, {leg: 0.0 for leg in LEGS}
            self.force_sign = 1.0 if total > 0 else -1.0
            self.get_logger().info(
                f"foot force sign resolved to {self.force_sign:+.0f} "
                f"(total {self.force_sign * total:.1f} N against "
                f"{TOTAL_MASS * G:.1f} N of weight)")

        signed = {leg: self.force_sign * fz for leg, fz in loads.items()}

        # CONTACTS ARE HELD BRIEFLY after they go light. Driving over Rubicon
        # unloads a wheel below 5 N constantly -- a facet edge does it, and the
        # wheel is back down a few hundredths of a second later. Treating each
        # of those as a lost contact drops the pattern under three, at which
        # point the margin is not "small" but hugely negative, because a
        # two-point support IS geometrically a tipover. So the measure spent
        # much of every drive reading -35 deg on a robot that was upright at
        # 10 deg of roll, which pinned the governor at its crawl floor: the
        # robot survived by never going anywhere, covering 0.24 m in 8 s.
        #
        # A wheel that carried load 0.1 s ago and is 2 cm off the ground is
        # still part of the support the next decision will land on. The hold is
        # shorter than the time any lever takes to act, so nothing is decided
        # on a contact that has genuinely gone.
        now = self.get_clock().now().nanoseconds * 1e-9
        hold = float(self.get_parameter("contact_hold").value)
        loaded = []
        for leg in LEGS:
            if signed.get(leg, 0.0) >= threshold:
                self.last_loaded[leg] = now
                loaded.append(leg)
            elif now - self.last_loaded.get(leg, -1e9) < hold:
                loaded.append(leg)
        return contacts, loaded, com, down, signed

    def _margin_for(self, ordered, com, down):
        """Force-angle margin for an already boundary-ordered support pattern.

        The ordering is deliberately NOT redone here. The envelope search below
        evaluates this a few hundred times a second with only the down vector
        changing, and re-sorting the contacts each time is both the dominant
        cost and pointless: the cyclic order of a convex pattern does not
        change under the tilts being explored.
        """
        if len(ordered) < 2:
            return -0.5, None
        margin, edge = force_angle_margin(ordered, com, down)
        if edge < 0:
            return margin, None
        return margin, (ordered[edge], ordered[(edge + 1) % len(ordered)])

    def _accel_envelope(self, ordered, com, down):
        """How much extra horizontal acceleration the current pattern allows.

        Returns ([+y, -y], [+x, -x]) in m/s^2: the acceleration at which the
        margin would fall to tip_margin_safe. Bisection rather than a closed
        form because the margin is a min over edges and which edge is critical
        changes partway through -- the bisection does not care. Ten steps over
        a 12 m/s^2 bracket resolves to 12 mm/s^2, far finer than anything
        downstream needs.
        """
        safe = float(self.get_parameter("tip_margin_safe").value)
        floor = float(self.get_parameter("a_lat_floor").value)
        ceiling = float(self.get_parameter("a_lat_ceiling").value)
        # `down` is the unit vector along effective gravity, so the vector form
        # is +down * G. It was -down * G, which points UP: force_line_inside
        # then rejected every candidate ("the pattern has to be BELOW the CoM
        # along the force direction"), every bisection probe came back
        # negative, and both searches returned a_lat_floor unconditionally. The
        # governor has therefore been running on a hard-coded 0.8 m/s^2 of
        # lateral authority since the envelope was written, whatever the stance
        # was actually good for -- 10.0 m/s^2 at full splay.
        g_vec = down * G       # effective gravity as a vector, m/s^2

        def margin_at(extra):
            # A commanded acceleration `extra` is felt as a pseudo-force in the
            # opposite direction, so it subtracts from effective gravity.
            v = g_vec - extra
            n = float(np.linalg.norm(v))
            if n < 1e-6:
                return -1.0
            m, _ = self._margin_for(ordered, com, v / n)
            return m

        def search(direction):
            if margin_at(ceiling * direction) >= safe:
                return ceiling
            if margin_at(floor * direction) < safe:
                return floor
            lo, hi = floor, ceiling
            for _ in range(10):
                mid = 0.5 * (lo + hi)
                if margin_at(mid * direction) >= safe:
                    lo = mid
                else:
                    hi = mid
            return lo

        ey = np.array([0.0, 1.0, 0.0])
        ex = np.array([1.0, 0.0, 0.0])
        return ([search(ey), search(-ey)], [search(ex), search(-ex)])

    # ---- control -----------------------------------------------------------
    def _update(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            return
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        if elapsed < float(self.get_parameter("startup_delay").value):
            return
        if len(self.q) < 4 or not self.have_imu:
            return

        geom = self._geometry()
        if geom is None:
            return
        contacts, loaded, com, down, loads = geom
        self.n_contacts = len(loaded)

        # ---- margin, now and predicted -----------------------------------
        pattern = order_contacts_ccw([contacts[leg] for leg in loaded], com, down)
        margin, edge = self._margin_for(pattern, com, down)

        # Predicted: rotate effective gravity by where the body rates are
        # taking the body. A rollover is divergent, so the rate term is the
        # early warning the static margin cannot give.
        # The lookahead is CLAMPED. Body rates on Rubicon spike to 1-2 rad/s
        # every time a wheel crosses a facet, and an unclamped 0.2 s lookahead
        # turns each of those into 17 deg of "predicted" rotation -- so the
        # margin reads as collapsing several times a second on terrain the
        # robot is handling perfectly well. Throttled to a crawl by its own
        # jitter, the robot then rolled at 0.5 m/s where the old node drove
        # 2.7 m without noticing. Predict, but do not panic.
        t_pred = float(self.get_parameter("predict_time").value)
        omega = np.array([self.wx, self.wy, 0.0]) * t_pred
        angle = float(np.linalg.norm(omega))
        cap = float(self.get_parameter("predict_max").value)
        if angle > cap:
            omega *= cap / angle
            angle = cap
        if angle > 1e-6:
            k = omega / angle
            # Rodrigues, applied in reverse: the body turning one way carries
            # gravity the other way in body coordinates.
            c, s = math.cos(angle), math.sin(angle)
            down_p = (down * c + np.cross(-k, down) * s
                      + (-k) * float(np.dot(-k, down)) * (1.0 - c))
            margin_p, _ = self._margin_for(pattern, com, down_p)
        else:
            margin_p = margin

        alpha = self.dt / (float(self.get_parameter("margin_tau").value) + self.dt)
        self.margin += alpha * (margin - self.margin)
        self.margin_pred += alpha * (min(margin, margin_p) - self.margin_pred)
        self.critical_dir = self._edge_direction(edge, com, down)
        self.cop_margin = self._cop_margin(contacts, loaded, loads, down)

        self._publish_stability(contacts, loaded, com, down, loads)

        # ---- ride height: ramp up from wherever the legs actually are ------
        if self.height is None:
            self.stand_from = self.height = self._measured_height(contacts, down)
            self.phase = "stand"
            self.stand_t = 0.0
            self.get_logger().info(
                f"standing up from {self.height:.3f} m to {self.height_target:.3f} m")

        if self.phase == "stand":
            self.stand_t += self.dt
            s = clamp(self.stand_t / self.stand_time, 0.0, 1.0)
            blend = s * s * (3.0 - 2.0 * s)     # smoothstep, no jerk at the ends
            self.height = self.stand_from + blend * (self.height_target - self.stand_from)
            if s >= 1.0:
                self.phase = "active"
                self.get_logger().info("stance reached; stabilizer live")
        else:
            step = float(self.get_parameter("reshape_rate_down").value) * self.dt
            self.height += clamp(self.height_target - self.height, -step, step)

        # Authority is RAMPED, not switched. Stepping the whole attitude loop
        # from zero to full in one cycle applies whatever error has accumulated
        # during the stand-up as a single impulse, which on rough ground is
        # enough to unload two wheels by itself.
        if self.phase == "stand" or not self.enabled:
            self.authority = 0.0
        else:
            self.authority = min(
                1.0, self.authority
                + self.dt / max(float(self.get_parameter("authority_ramp").value),
                                1e-3))
        authority = self.authority

        # ---- attitude error: complementary, and it has to be --------------
        # The target is "body z along EFFECTIVE gravity", and the accelerometer
        # measures exactly that. Feeding it straight to the loop, however, is
        # unstable, and not subtly: an accelerometer measures proper
        # acceleration, so when the legs push the body sideways it reports the
        # push as tilt, the loop tilts further to "correct" it, and that pushes
        # harder. Measured on flat ground, standing still: two wheels unloaded
        # within one second of the loop going live and the robot was upside
        # down a few seconds later.
        #
        # So the signal is split by frequency, which is what it always wanted:
        #
        #   attitude   from the IMU quaternion. Clean, no acceleration in it,
        #              and it carries the whole fast path.
        #   lean       the DIFFERENCE between the two, which is purely the
        #              inertial part -- centrifugal, braking, terrain reaction --
        #              low-passed hard, because a lean reference is a sustained
        #              quantity and anything fast in it is the feedback path
        #              above rather than a real demand.
        #
        # The sum is the same reference as before at DC, so the slope levelling
        # and the cornering lean both survive intact.
        up = -down
        total_roll = -math.atan2(up[1], up[2])
        total_pitch = math.atan2(up[0], up[2])
        att_roll, att_pitch = -self.roll, -self.pitch

        raw_lean_roll = total_roll - att_roll
        raw_lean_pitch = total_pitch - att_pitch

        # ACCELEROMETER BIAS. Standing still, the difference above is zero by
        # construction -- whatever the slope, a body at rest reads gravity and
        # only gravity, and the attitude term cancels it exactly. So anything
        # left at rest IS the bias, which makes it observable, and it has to be
        # taken out: the URDF's 0.1 m/s^2 bias_mean is 0.58 deg of permanent
        # phantom lean, and the loop faithfully holds the body at that angle
        # forever. Measured before this existed, standing on flat ground: roll
        # 0.77 deg rms and pitch 0.98 deg rms, against 0.04 / 0.06 with the lean
        # reference disabled entirely.
        at_rest = (abs(self.v_out) < 0.05 and abs(self.w_out) < 0.05
                   and math.hypot(self.wx, self.wy) < 0.10
                   and self.n_contacts >= 3)
        if at_rest:
            b_tau = float(self.get_parameter("accel_bias_tau").value)
            b = self.dt / (b_tau + self.dt)
            b_max = float(self.get_parameter("accel_bias_max").value)
            self.bias_roll = clamp(
                self.bias_roll + b * (raw_lean_roll - self.bias_roll), -b_max, b_max)
            self.bias_pitch = clamp(
                self.bias_pitch + b * (raw_lean_pitch - self.bias_pitch), -b_max, b_max)

        a = self.dt / (float(self.get_parameter("lean_tau").value) + self.dt)
        self.lean_roll += a * ((raw_lean_roll - self.bias_roll) - self.lean_roll)
        self.lean_pitch += a * ((raw_lean_pitch - self.bias_pitch) - self.lean_pitch)
        lean_max = float(self.get_parameter("lean_max").value)
        err_roll = att_roll + clamp(self.lean_roll, -lean_max, lean_max)
        err_pitch = att_pitch + clamp(self.lean_pitch, -lean_max, lean_max)

        # ---- margin-driven CoM bias ---------------------------------------
        # Lean away from the edge the robot is closest to going over. Tilting
        # by phi moves a CoM at height h by -h*sin(phi) in y and +h*sin(theta)
        # in x, so the tilt that produces a wanted CoM shift is the shift over
        # the height, with that sign pattern.
        bias_roll = bias_pitch = 0.0
        bias_margin = float(self.get_parameter("bias_margin").value)
        bias_max = float(self.get_parameter("bias_max").value)
        if (authority > 0.0 and self.critical_dir is not None
                and self.margin_pred < bias_margin):
            deficit = clamp((bias_margin - self.margin_pred) / bias_margin, 0.0, 1.0)
            lean = float(self.get_parameter("bias_gain").value) * deficit * bias_max
            # critical_dir points from the CoM towards the edge, so leaning by
            # +critical_dir_y in roll and -critical_dir_x in pitch moves the CoM
            # directly away from it. The CoM height cancels: the shift wanted is
            # proportional to it and the tilt that produces a shift divides by
            # it, which is why this is a pure angle.
            bias_roll = clamp(self.critical_dir[1] * lean, -bias_max, bias_max)
            bias_pitch = clamp(-self.critical_dir[0] * lean, -bias_max, bias_max)

        # ---- attitude PID, split fast/slow --------------------------------
        tilt_max = float(self.get_parameter("tilt_max").value)
        i_max = float(self.get_parameter("tilt_i_max").value)

        # Integrate only while genuinely standing on a pattern and in charge,
        # so the term cannot wind up during the stand-up ramp or while a wheel
        # is off the ground and the error is not something the legs can fix.
        if authority > 0.0 and self.n_contacts >= 3 and self.margin > 0.0:
            self.i_roll = clamp(
                self.i_roll + float(self.get_parameter("roll_ki").value) * err_roll * self.dt,
                -i_max, i_max)
            self.i_pitch = clamp(
                self.i_pitch + float(self.get_parameter("pitch_ki").value) * err_pitch * self.dt,
                -i_max, i_max)
        else:
            self.i_roll *= 0.995
            self.i_pitch *= 0.995

        pi_roll = float(self.get_parameter("roll_kp").value) * err_roll + self.i_roll
        pi_pitch = float(self.get_parameter("pitch_kp").value) * err_pitch + self.i_pitch

        # The low-pass belongs HERE and nowhere else. The old node filtered the
        # whole PID output, which delayed the rate term by 0.1 s -- the term
        # that stops a divergent roll was the most delayed signal in the loop.
        # THE LOW-PASS GOES ON THE WHOLE COMMAND, INCLUDING THE RATE TERM.
        #
        # An earlier version filtered only the P+I path, on the argument that
        # delaying the rate term delays the one signal that arrests a divergent
        # roll. That argument is right about what the rate term is for and
        # wrong about what the loop can carry, and the cost was the single
        # worst behaviour in the stack.
        #
        # This command is a body ATTITUDE, realised through differential leg
        # length and tracked by a stiff joint PD. Between asking for it and the
        # body actually rolling there is the 0.03 s command horizon, the leg
        # controller, and the ~20 Hz mode of the body bouncing in roll against
        # the hip PD. That chain passes 180 deg of lag well before 20 Hz, so an
        # UNFILTERED rate term is not damping there -- it is positive feedback.
        #
        # Measured, standing still on level ground:
        #
        #   kd 0.09 / 0.10, D unfiltered   roll rate 3.9 rad/s rms, 19.5 Hz
        #   kd 0.0                         roll rate 0.10 rad/s rms
        #
        # The accelerometer saw +/-200 m/s^2 of lateral acceleration during
        # that limit cycle, which is 20 g on a stationary robot. Everything
        # downstream reads the IMU: the effective-gravity reference became
        # garbage (a_y 130 m/s^2 against a_z 14), so the stability margin
        # reported -32 deg on an upright robot, the acceleration envelope
        # collapsed to its floor, urgency pinned at 1.00, the stance splayed to
        # its limit and stayed there, and the governor held the robot at its
        # roughness crawl. The robot was not being conservative -- it was
        # shaking itself blind.
        #
        # One pole at command_tau on the sum puts the loop gain 6x down by
        # 20 Hz, which is where the mode is, while leaving the rate term intact
        # across the few Hz that terrain disturbances actually occupy.
        c_tau = float(self.get_parameter("command_tau").value)
        a = self.dt / (c_tau + self.dt)
        raw_roll = (pi_roll + bias_roll
                    - float(self.get_parameter("roll_kd").value) * self.wx)
        raw_pitch = (pi_pitch + bias_pitch
                     - float(self.get_parameter("pitch_kd").value) * self.wy)
        self.pi_roll_f += a * (raw_roll - self.pi_roll_f)
        self.pi_pitch_f += a * (raw_pitch - self.pi_pitch_f)

        d_roll = authority * clamp(self.pi_roll_f, -tilt_max, tilt_max)
        d_pitch = authority * clamp(self.pi_pitch_f, -tilt_max, tilt_max)

        # ---- reshaping -----------------------------------------------------
        r_tau = float(self.get_parameter("rough_tau").value)
        floor = float(self.get_parameter("rough_floor").value)
        if self.phase == "active":
            rate_sq = max(self.wx * self.wx + self.wy * self.wy - floor * floor, 0.0)
            self.rough_sq += (self.dt / (r_tau + self.dt)) * (rate_sq - self.rough_sq)
        else:
            # Landing on the terrain and standing up both shake the body hard,
            # and with a 1 s window that transient is still in the estimate when
            # the loop goes live -- which pegged the urgency and slammed the
            # stance to full width before the robot had done anything. Bleed it
            # off instead of carrying it across.
            self.rough_sq *= 0.97
        rough = math.sqrt(self.rough_sq)

        target = float(self.get_parameter("margin_target").value)
        calm = float(self.get_parameter("speed_calm").value)
        wide = float(self.get_parameter("speed_wide").value)
        rough_ref = float(self.get_parameter("rough_ref").value)
        p_rough_ref = float(self.get_parameter("preview_rough_ref").value)

        # Contact loss is DEBOUNCED. On Rubicon the robot genuinely stands on
        # three wheels much of the time and the fourth flickers across the 5 N
        # threshold, so an instantaneous test makes the reach lever chase it:
        # +/-60 mm of ride height at the reshape rate, which is the body being
        # pumped up and down. Measured standing still, that alone took the body
        # rate from 0.006 to 0.13 rad/s rms.
        debounce = float(self.get_parameter("contact_debounce").value)
        if self.n_contacts >= 3:
            self.lost_t = 0.0
            self.held_t = min(self.held_t + self.dt, 10.0)
        else:
            self.held_t = 0.0
            self.lost_t = min(self.lost_t + self.dt, 10.0)
        if self.lost_t > debounce:
            self.contact_ok = False
        elif self.held_t > debounce:
            self.contact_ok = True
        contact_ok = self.contact_ok
        u_rough = clamp(rough / max(rough_ref, 1e-3), 0.0, 1.0)
        u_preview = clamp(self.preview_rough / max(p_rough_ref, 1e-3), 0.0, 1.0)
        if authority > 0.0:
            u_margin = clamp((target - self.margin_pred) / max(target, 1e-3), 0.0, 1.0)
            u_speed = clamp((abs(self.v_out) - calm) / max(wide - calm, 1e-3), 0.0, 1.0)
            # What the CURRENT command is about to demand laterally, as a
            # fraction of what is available. This is the anticipation the old
            # node's speed term only approximated.
            demand = abs(self.v_out * self.w_out)
            avail = max(min(self.a_lat), 1e-3)
            u_cmd = clamp(demand / avail, 0.0, 1.0)
            # Upcoming slope, from the lidar. A side slope eats the margin
            # before the robot is on it, and widening takes time.
            #
            # Referenced to the TIPPING ANGLE, not to a fixed 0.35 rad. The
            # hard-coded 20 deg was below this terrain's 75th percentile slope,
            # so merely being on a hill was scored as maximum urgency and the
            # stance never came back in. Full urgency belongs where the ground
            # ahead approaches the angle the robot actually tips at.
            u_slope = clamp(self.preview_slope
                            / max(float(self.get_parameter(
                                "preview_slope_ref").value), 1e-3), 0.0, 1.0)
            urgency = max(u_margin, u_speed, u_rough, u_cmd, u_preview, u_slope)
        else:
            u_margin = u_speed = u_cmd = u_slope = 0.0
            urgency = 0.0
        # Kept for the state string. Which of the six terms is driving the
        # stance is the first thing anyone asks when the robot is splayed and
        # crawling, and it used to take a code change to find out.
        self.u_terms = (u_margin, u_speed, u_rough, u_cmd, u_preview, u_slope)
        # Smoothed, because every input to it is noisy and the levers it drives
        # move real mass. Rising is quick, falling is slow -- the same asymmetry
        # the rate limits use, and for the same reason.
        u_tau = float(self.get_parameter(
            "urgency_rise_tau" if urgency > self.urgency
            else "urgency_fall_tau").value)
        self.urgency += (self.dt / (u_tau + self.dt)) * (urgency - self.urgency)
        urgency = self.urgency

        up_step = float(self.get_parameter("reshape_rate_up").value) * self.dt
        down_step = float(self.get_parameter("reshape_rate_down").value) * self.dt

        # Losing contact wants the opposite of crouching: the ground has
        # dropped away or the body has grounded out, and either way the legs
        # need to reach DOWN to find the surface.
        # Reach gets its own, slower rate. It is the only lever that moves the
        # ride height directly, so it is the one that shows up as the body
        # bouncing; the width levers move sideways where nothing is stacked on
        # top of them.
        reach_max = float(self.get_parameter("reach_max").value)
        reach_step = float(self.get_parameter("reach_rate").value) * self.dt
        goal = reach_max if (authority > 0.0 and not contact_ok) else 0.0
        self.reach += clamp(goal - self.reach, -reach_step, reach_step)

        # RESHAPING THE STANCE MEANS DRAGGING LOADED WHEELS SIDEWAYS, and the
        # hip cannot do that standing still. The lateral resistance is mu*N
        # through the stance height -- 1.0 * 48 N * 0.36 m = 17.3 N.m -- on top
        # of the 12.2 N.m of static splay moment at full width, against a hip
        # limit of 23.7 N.m.
        #
        # Measured, standing on Rubicon at a splay of 0.413 rad with the target
        # set back to the nominal stance: both front hips sat at exactly
        # -23.700 N.m, saturated, and the splay did not move by one milliradian
        # for as long as the robot stood there -- through a sweep of widen_max
        # from 0.110 all the way to 0. Six seconds of rolling at 0.6 m/s
        # brought the same joint back to 0.038 rad with 10.6 N.m of effort.
        #
        # So the width lever is available only while the wheels are turning,
        # where the drive force shares the friction cone and lateral resistance
        # collapses. Asking for it at rest does not widen the stance; it just
        # holds both hip motors against their stops, which on the real machine
        # is a thermal fault rather than a stance.
        roll_speed = float(self.get_parameter("reshape_roll_speed").value)
        rest_gain = float(self.get_parameter("reshape_rest_gain").value)
        roll_gate = clamp(abs(self.v_out) / max(roll_speed, 1e-3),
                          rest_gain, 1.0)
        up_step *= roll_gate
        down_step *= roll_gate

        widen_goal = urgency * float(self.get_parameter("widen_max").value)
        # Crouch is withdrawn as the ground gets rough -- clearance beats a
        # lower CoM once the belly is at risk, and grounding out on Rubicon has
        # already been measured to cost more than it buys.
        gate = 1.0 - (float(self.get_parameter("crouch_rough_gate").value)
                      * max(u_rough, u_preview))
        crouch_goal = urgency * float(self.get_parameter("crouch_max").value) * gate
        if not contact_ok:
            crouch_goal = 0.0
        self.widen += clamp(widen_goal - self.widen, -down_step, up_step)
        self.crouch += clamp(crouch_goal - self.crouch, -down_step, up_step)

        # And the command is kept within reach of what the hips have actually
        # achieved. The rate gate above stops the target running away quickly;
        # this stops it running away at all. Without it the commanded width and
        # the real one drift apart until the hip PD saturates on the difference
        # and stays there -- the state the robot was measured in above.
        band = float(self.get_parameter("widen_band").value)
        achieved = (sum(abs(leg_fk(leg, *self.q[leg])[1]) for leg in LEGS) / 4.0
                    - HALF_TRACK_NOM)
        self.widen = clamp(self.widen, achieved - band, achieved + band)
        self.widen = clamp(self.widen, 0.0,
                           float(self.get_parameter("widen_max").value))

        # ---- foot targets and IK -------------------------------------------
        half_track = HALF_TRACK_NOM + self.widen
        ride = clamp(self.height - self.crouch + self.reach, 0.24, 0.47)
        z_nom = -(ride - WHEEL_R)
        dz_max = float(self.get_parameter("dz_max").value)

        command = {}
        for leg in LEGS:
            sx, sy = SIGN[leg]
            x = sx * HIP_X
            y = sy * half_track
            dz = clamp(-y * math.sin(d_roll) + x * math.sin(d_pitch),
                       -dz_max, dz_max)
            angles = leg_ik(leg, np.array([x, y, z_nom + dz]))
            if angles is None:
                command = None
                break
            command[leg] = angles

        if command is None:
            if self.last_cmd is None:
                return
            command = self.last_cmd     # unreachable target: hold the last good one
        self.last_cmd = command

        self._send(command)
        self.pub_width.publish(Float64(data=2.0 * half_track))

        # Hand the support pattern to the governor, which runs the envelope
        # bisection on its own slower tick. Doing it here as well would repeat
        # forty margin evaluations at the full control rate for an answer the
        # command path only reads at 50 Hz.
        self._pattern = (pattern, com, down)

    def _edge_direction(self, edge, com, down):
        """Horizontal unit vector from the CoM toward the critical edge."""
        if edge is None:
            return None
        p1, p2 = edge
        axis = p2 - p1
        norm = np.linalg.norm(axis)
        if norm < 1e-6:
            return None
        axis = axis / norm
        l_vec = (np.eye(3) - np.outer(axis, axis)) @ (p1 - com)
        # Drop the component along gravity: only the horizontal part is a
        # direction the CoM can usefully be moved in.
        l_vec = l_vec - float(np.dot(l_vec, down)) * down
        norm = np.linalg.norm(l_vec)
        if norm < 1e-6:
            return None
        return l_vec / norm

    def _cop_margin(self, contacts, loaded, loads, down):
        """Old-style CoP distance to the polygon edge, metres.

        Kept because magi_stability_test.py and the RViz layouts read it, and
        because a distance in metres is easier to eyeball than an angle. It is
        no longer what any control decision is made on.
        """
        if len(loaded) < 3:
            return -1.0
        ex, ey = plane_basis(down)
        total = sum(loads[leg] for leg in loaded)
        if total < 1e-6:
            return -1.0
        pts = []
        cop = np.zeros(2)
        for leg in loaded:
            p = contacts[leg]
            q = (float(np.dot(p, ex)), float(np.dot(p, ey)))
            pts.append(q)
            cop += loads[leg] * np.array(q)
        cop /= total
        return polygon_margin(order_ccw(pts), (cop[0], cop[1]))

    # ---- the governor ------------------------------------------------------
    def _govern(self):
        """Project the operator's twist onto the admissible set."""
        if not self.have_cmd:
            # Something else may still own the output topic; stay off it until
            # the operator has actually addressed us.
            return

        v_max = float(self.get_parameter("v_max").value)
        w_max = float(self.get_parameter("w_max").value)
        gdt = 1.0 / float(self.get_parameter("governor_rate").value)

        v_req = clamp(self.cmd_in.linear.x, -v_max, v_max)
        w_req = clamp(self.cmd_in.angular.z, -w_max, w_max)

        # Input timeout: an operator who has stopped talking wants a stop.
        if self.cmd_in_time is not None:
            age = (self.get_clock().now() - self.cmd_in_time).nanoseconds * 1e-9
            if age > float(self.get_parameter("cmd_timeout").value):
                v_req = w_req = 0.0

        governing = (bool(self.get_parameter("governor_enable").value)
                     and self.enabled and self.phase == "active"
                     and self._pattern is not None)

        if governing:
            pattern, com, down = self._pattern
            self.a_lat, self.a_lon = self._accel_envelope(pattern, com, down)
            crit = float(self.get_parameter("tip_margin_crit").value)

            # 0. THE WHEELIE. This robot rears up under hard tractive effort,
            #    and that is geometry, not tuning: the CoM sits 0.311 m above
            #    the contacts with only 0.1934 m of half-wheelbase behind it,
            #    so a forward ground force pitches the body back harder than
            #    the wheelbase can restore once
            #
            #        mu > half_wheelbase / h_com = 0.1934 / 0.311 = 0.62
            #
            #    and mu is 1.0, chosen for hill climbing. Driving normally
            #    never reaches that -- accelerating at the governed 1.5 m/s^2
            #    needs 29 N against the 191 N the ground could deliver -- but
            #    a BLOCKED FRONT WHEEL does. The rear wheels keep pushing into
            #    a wheel that cannot roll and the couple stands the robot on
            #    its tail.
            #
            #    Measured on the standard course: the robot reared from -7 deg
            #    to -84 deg of pitch in one second, travelling 0.1 m, and went
            #    over backwards -- with the governor still commanding 1.00 m/s
            #    the whole way, because the tipping envelope is a steady-state
            #    measure and this is not steady state.
            #
            #    The limit is geometric and it is the pitch twin of the
            #    31.4 deg roll figure this node is built around:
            #
            #        atan(half_wheelbase / h_com) = atan(0.1934 / 0.311)
            #                                     = 31.9 deg
            #
            #    Past that, nose up, the CoM is behind the rear contacts and
            #    the robot goes over its own tail. There is no lever against
            #    it: widening the stance is a ROLL remedy, and the wheelbase
            #    is fixed. Refusing to keep climbing is the whole answer.
            #
            #    Measured on the standard course, the leg that failed 3/3:
            #    the robot drove onto a face steeper than that, pitched
            #    -7 -> -16 -> -22 -> -34 deg over 0.75 s and went over
            #    backwards. Wheel torque stayed at 1.1 N.m throughout -- it
            #    was not pushing against anything, it was climbing something
            #    it should not have climbed -- and foot contact still reported
            #    all four wheels loaded at 73 deg of pitch, so neither is any
            #    use as the trigger.
            #
            #    ALREADY STEEP **AND** STILL GETTING STEEPER is what separates
            #    this from a legitimate climb. Entering a 25 deg slope pitches
            #    the body fast but only to 25 deg; sitting on one is steep but
            #    no longer rotating. Only a rear-up is both at once.
            #
            #    Cutting for rear_stop_hold and then letting the command back
            #    gives a push-pause-push stutter rather than a refusal, so an
            #    operator can still work at an obstacle. Reverse is never cut:
            #    backing off is the way out.
            now_s = self.get_clock().now().nanoseconds * 1e-9
            nose_up = -self.pitch
            if (self.v_out > 0.2
                    and nose_up > float(self.get_parameter("rear_pitch").value)
                    and -self.pitch_rate
                    > float(self.get_parameter("rear_rate").value)):
                self.rear_stop_until = now_s + float(
                    self.get_parameter("rear_stop_hold").value)
            if now_s < self.rear_stop_until:
                v_req = min(v_req, 0.0)

            # 1. Speed ceiling from the terrain. This is the one limit that
            #    does not come from the tipping envelope: rough ground rolls
            #    the robot through impulses the envelope cannot see, because
            #    the envelope only knows the ground the wheels are on now. Both
            #    the measured body-rate roughness and the lidar preview feed
            #    it, so it drops before the rough patch as well as during it.
            rough_n = clamp(math.sqrt(self.rough_sq)
                            / max(float(self.get_parameter("rough_ref").value), 1e-3),
                            0.0, 1.0)
            prev_n = clamp(self.preview_rough
                           / max(float(self.get_parameter("preview_rough_ref").value),
                                 1e-3), 0.0, 1.0)
            terrain = max(rough_n, prev_n)
            v_rough = float(self.get_parameter("v_rough_min").value)
            self.v_limit = v_max + terrain * (v_rough - v_max)
            v_req = clamp(v_req, -self.v_limit, self.v_limit)

            # 2. Lateral: a steady turn at (v, w) needs v*w of lateral
            #    acceleration, and the sign picks which side of the envelope
            #    applies. Scaling BOTH components by the same factor keeps v/w
            #    -- the path curvature -- so the robot takes the operator's arc
            #    more slowly instead of a different arc.
            demand = v_req * w_req
            avail = self.a_lat[0] if demand > 0 else self.a_lat[1]
            if abs(demand) > avail > 1e-6:
                scale = math.sqrt(avail / abs(demand))
                v_req *= scale
                w_req *= scale

            # 3. Longitudinal: the same envelope, applied to how hard the
            #    command may change speed. Braking pitches the robot forward
            #    exactly as accelerating pitches it back, and on a slope those
            #    are very different numbers -- which is why they are two
            #    separate searches rather than one symmetric limit.
            dv = v_req - self.v_out
            a_fwd, a_back = self.a_lon
            # Accelerating forward throws the CoM back, so the limit that binds
            # is the one for the rearward direction, and vice versa.
            lim = min(a_back if dv > 0 else a_fwd,
                      float(self.get_parameter("accel_max").value))
            v_req = self.v_out + clamp(dv, -lim * gdt, lim * gdt)

            # 4. Under the critical margin, back off -- but NOT to a standstill.
            #    Turning is what tips a robot over, so the yaw command is what
            #    gets taken away; forward speed is capped to a crawl instead of
            #    zeroed, because a robot with no margin is usually a robot on a
            #    slope, and driving off the slope is the way out of that. An
            #    earlier version faded both to zero and simply pinned the robot
            #    wherever it had got into trouble, still sliding, unable to
            #    accept any command that would have taken it somewhere flatter.
            if self.margin_pred < crit:
                # A critical margin has two completely different causes and
                # they need opposite responses.
                #
                #   on a slope    the margin is small because the ground is
                #                 tilted. Driving off it is the way out, which
                #                 is why the crawl floor below is not zero: an
                #                 earlier version faded to a standstill and
                #                 simply pinned the robot where it got into
                #                 trouble, still sliding.
                #   going over    the margin is small because the robot just
                #                 hit something and is already rotating. Here
                #                 the drive torque is what finishes the
                #                 rollover, and a crawl floor keeps applying it.
                #
                # The discriminator is body rate: a slope is static, a rollover
                # is not. Measured on the standard course, the rollovers left
                # after the rest of these fixes were overwhelmingly of the
                # second kind -- 8 of 12 at the two start poses that have a
                # boulder within a metre, reached at 0.7 m/s where the stock
                # configuration had been too slow to get there at all.
                #
                # Reverse is deliberately still allowed while latched: backing
                # off whatever was hit is exactly the command that helps.
                rate = math.hypot(self.wx, self.wy)
                now_s = self.get_clock().now().nanoseconds * 1e-9
                if rate > float(self.get_parameter("tip_stop_rate").value):
                    self.tip_stop_until = now_s + float(
                        self.get_parameter("tip_stop_hold").value)
                if now_s < self.tip_stop_until:
                    v_req = min(v_req, 0.0)
                    w_req = 0.0
                else:
                    fade = clamp(self.margin_pred / max(crit, 1e-3), 0.0, 1.0)
                    w_req *= fade
                    crawl = float(self.get_parameter("v_crit_min").value)
                    ceiling = crawl + fade * (self.v_limit - crawl)
                    v_req = clamp(v_req, -ceiling, ceiling)
        else:
            self.v_limit = v_max

        # Slew limits on the published command in every mode, so the governor
        # is never itself the source of an impulse.
        a_max = float(self.get_parameter("accel_max").value) * gdt
        y_max = float(self.get_parameter("yaw_accel_max").value) * gdt
        self.v_out += clamp(v_req - self.v_out, -a_max, a_max)
        self.w_out += clamp(w_req - self.w_out, -y_max, y_max)

        out = Twist()
        out.linear.x = self.v_out
        out.angular.z = self.w_out
        self.pub_cmd.publish(out)
        self.pub_speed.publish(Float64(data=float(self.v_limit)))

    # ---- output ------------------------------------------------------------
    def _measured_height(self, contacts, down):
        """Body height above the contact plane, from the current joint angles.

        The contacts sit below the base, so their projection ALONG the down
        vector is the positive drop -- getting this sign wrong starts the
        stand-up ramp from the 0.10 m clamp and slams the legs into a full
        crouch before extending.
        """
        drops = [float(np.dot(contacts[leg], down)) for leg in LEGS]
        return clamp(sum(drops) / len(drops), 0.10, 0.48)

    def _send(self, command):
        point = JointTrajectoryPoint()
        point.positions = [command[leg][i] for leg in LEGS for i in range(3)]
        point.velocities = [0.0] * len(JOINT_NAMES)
        # Read live rather than cached, so the horizon can be swept against a
        # running robot -- it is the dominant term in the phantom velocity
        # described below and is not something to get right by guessing.
        horizon = float(self.get_parameter("command_horizon").value)
        point.time_from_start = Duration(
            sec=int(horizon),
            nanosec=int((horizon % 1.0) * 1e9))

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points = [point]
        self.pub_traj.publish(traj)

    def _publish_stability(self, contacts, loaded, com, down, loads):
        stamp = self.get_clock().now().to_msg()

        self.pub_tip.publish(Float64(data=float(self.margin)))
        self.pub_tip_pred.publish(Float64(data=float(self.margin_pred)))
        self.pub_margin.publish(Float64(data=float(self.cop_margin)))

        poly = PolygonStamped()
        poly.header.stamp = stamp
        poly.header.frame_id = "base"
        for leg in order_ccw_legs(contacts):
            p = contacts[leg]
            poly.polygon.points.append(
                Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        self.pub_poly.publish(poly)

        com_msg = PointStamped()
        com_msg.header.stamp = stamp
        com_msg.header.frame_id = "base"
        com_msg.point.x, com_msg.point.y, com_msg.point.z = (float(v) for v in com)
        self.pub_com.publish(com_msg)

        if len(loaded) >= 3:
            total = sum(loads[leg] for leg in loaded)
            if total > 1e-6:
                cop = sum(loads[leg] * contacts[leg] for leg in loaded) / total
                msg = PointStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = "base"
                msg.point.x, msg.point.y, msg.point.z = (float(v) for v in cop)
                self.pub_cop.publish(msg)

    def _report(self):
        if self.height is None:
            return
        state = (f"{self.phase} h={self.height - self.crouch + self.reach:.3f} "
                 f"feet={self.n_contacts} "
                 f"tip={math.degrees(self.margin):+.1f}/"
                 f"{math.degrees(self.margin_pred):+.1f}deg "
                 f"track={2 * (HALF_TRACK_NOM + self.widen):.3f} "
                 f"urg={self.urgency:.2f}"
                 f"[{'/'.join('%.2f' % u for u in self.u_terms)}] "
                 f"alat={self.a_lat[0]:.1f}/{self.a_lat[1]:.1f} "
                 f"cmd={self.v_out:+.2f}/{self.w_out:+.2f} "
                 f"drv={self.drive_effort:.1f}Nm "
                 f"prev={math.degrees(self.preview_slope):.0f}deg/"
                 f"{self.preview_rough * 100:.1f}cm "
                 f"rp={math.degrees(self.roll):+.1f}/{math.degrees(self.pitch):+.1f}deg")
        self.pub_state.publish(String(data=state))


def order_ccw_legs(contacts):
    """All four leg names, ordered counter-clockwise about the stance centroid."""
    legs = list(LEGS)
    cx = sum(contacts[l][0] for l in legs) / len(legs)
    cy = sum(contacts[l][1] for l in legs) / len(legs)
    return sorted(legs, key=lambda l: math.atan2(contacts[l][1] - cy,
                                                 contacts[l][0] - cx))


def main():
    rclpy.init()
    node = MagiStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
