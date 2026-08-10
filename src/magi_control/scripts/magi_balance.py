#!/usr/bin/env python3
"""Whole-body balance controller for the MAGI Go2W.

WHAT THIS REPLACES
------------------
magi_posture.py computes one (thigh, calf) pair from a desired body height and
sends it to all four legs, once. That is a fixed stance: the legs are a rigid
tripod-and-a-half with joint-level PD and nothing above it. It has no idea
which way is up, so on a slope the body tilts with the terrain, and in a turn
the centrifugal load transfers onto the outer wheels with nothing pushing back.
Measured on Rubicon, 0.35 m/s with 0.15 rad/s was enough to roll the robot onto
its side.

A real quadruped does not hold a stance. It continuously repositions its feet
and body so that the ground-reaction resultant stays inside the support
polygon. This node does that.

THE STABILITY CRITERION
-----------------------
A legged robot tips when the centre of pressure -- the point where the resultant
of all ground reaction forces acts -- reaches the boundary of the support
polygon. Not when the centre of mass leaves it: under acceleration the two
differ, and the CoP is the one that actually decides.

We have the CoP directly. Each wheel carries a force_torque sensor, so

    CoP = sum(fz_i * p_i) / sum(fz_i)

over the loaded feet, where p_i are the contact points. No mass model, no
double integration, no assumption that the robot is a point mass on a stick.
When a wheel unloads, its fz goes to zero and it drops out of both the sum and
the polygon on its own. When the CoP touches an edge, that is the tipping
instant, exactly.

The margin -- the shortest distance from the CoP to any polygon edge -- is the
single number this controller is built around. It is published so it can be
watched, plotted, and used as a trigger.

THE THREE LEVERS
----------------
Given a shrinking margin there are three things legs can do about it, and this
node uses all three:

  1. MOVE THE CoP BACK INSIDE, by tilting the body.
     Rolling the body by phi moves the centre of mass laterally by -h*sin(phi)
     without moving a single contact point. This is what a motorcycle does in a
     turn and what the real Go2-W visibly does. It is the fast lever: the legs
     only change length differentially, nothing has to slide on the ground.

     Feed-forward: a turn at (v, w) needs lateral acceleration v*w, so the
     resultant leans by atan2(v*w, g) and the body should lean to match. That
     term is computed from the *command*, so the lean leads the turn instead of
     chasing it.

     Feedback: an attitude PD then drives measured roll/pitch onto that
     reference. On a static slope the reference is zero, so the PD levels the
     body against the terrain -- which is the same thing, because a level body
     on a slope is exactly the one whose CoM hangs over the polygon centre.

  2. GROW THE POLYGON, by abducting the hips.
     Splaying the legs widens the track. Nominal is 0.380 m; the hips reach
     +/-60 deg and can take it past 0.50 m. A wider polygon is unconditionally
     more stable, but it scrubs the wheels sideways to get there, so it is
     applied in proportion to how little margin is left rather than always.

  3. LOWER THE CoM, by shortening all four legs together.
     The tipping moment is (lateral force) x (CoM height). Crouching from
     0.40 m to 0.30 m cuts it by a quarter. Slower than tilting, and it costs
     ground clearance, so like the splay it is margin-driven.

Levers 2 and 3 are what make this "repositioning the legs" rather than just
"tilting the body": the support polygon itself changes shape in response to
where the CoP is inside it.

HOW THE TILT REACHES THE JOINTS
-------------------------------
Rotating the body by (dphi, dtheta) while the feet stay put means each foot
moves, in body coordinates, by

    dz_i = -y_i*sin(dphi) + x_i*sin(dtheta)

which is the third row of R(dphi, dtheta)^T applied to the foot's nominal
position. Add that to the nominal stance, and solve the 3-DoF leg inverse
kinematics for each foot independently. Nothing here assumes the four legs do
the same thing.

WHAT THIS IS NOT
----------------
This is a quasi-static balance controller. It has no notion of stepping, so it
cannot recover by taking a step the way a real quadruped can -- with wheels on
the ground that is the right trade, but it does mean a large enough disturbance
still tips the robot. It also does not do force distribution: the joint-level
PD in leg_controller decides how load is shared, and this node only sets where
the feet go.

INTERACTION WITH THE REST OF THE STACK
--------------------------------------
Splaying changes the effective track, which diff_drive_controller does not know
about -- it converts wheel speeds to a twist using a fixed wheel_separation of
0.380 m. So while splayed, commanded yaw rate under-delivers and the odometry
message over-reads yaw. The EKF is unaffected: ekf.yaml takes vx only from
odom0 and gets yaw rate from the IMU. The current effective track is published
on /magi/stance_width so the discrepancy is at least visible.

TOPICS
------
  subscribes
    /joint_states                        leg configuration
    /imu/data                            attitude and body rates
    /foot_force/{FL,FR,RL,RR}            ground reaction, calf frame
    /wheel_controller/cmd_vel_unstamped  commanded twist, for the lean feed-forward
    /magi/body_height  (Float64)         ride-height target
    /magi/posture      (String)          named ride heights
    /magi/balance_enable (Bool)          false freezes the stance, for A/B tests

  publishes
    /leg_controller/joint_trajectory     the 12 joint targets, streamed
    /magi/stability_margin (Float64)     CoP distance to the polygon edge, metres
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
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------------------------------------------------------------------
# Geometry, read off go2w_body.xacro.
#
#   base -> hip     (+/-0.1934, +/-0.0465, 0)   revolute about x
#   hip  -> thigh   (0, +/-0.0955, 0)           revolute about y
#   thigh-> calf    (0, 0, -0.213)              revolute about y
#   calf -> wheel   (0, 0, -0.2264)             continuous about y
#
# The wheel's rolling plane sits a further 0.0481 m outboard of the foot link
# origin (the collision cylinder offset in gen_body.py). That offset lies along
# the thigh/calf rotation axis and along the wheel's own spin axis, so it is
# invariant to all three of those joints and simply adds to the abduction
# offset. Together: 0.0465 + 0.0955 + 0.0481 = 0.1901 m half-track, i.e. the
# 0.380 m wheel_separation diff_drive is configured with.
# ---------------------------------------------------------------------------
HIP_X, HIP_Y = 0.1934, 0.0465
ABAD_Y = 0.0955
WHEEL_PLANE_Y = 0.0481
D_Y = ABAD_Y + WHEEL_PLANE_Y          # 0.1436, lateral offset below the hip axis
L_THIGH = 0.213
L_CALF = 0.2264                       # calf joint -> wheel centre
WHEEL_R = 0.086
HALF_TRACK_NOM = HIP_Y + D_Y          # 0.1901

LEGS = ("FL", "FR", "RL", "RR")
# (fore/aft, left/right) sign per leg.
SIGN = {"FL": (+1.0, +1.0), "FR": (+1.0, -1.0),
        "RL": (-1.0, +1.0), "RR": (-1.0, -1.0)}

HIP_LIMIT = (-1.0472, 1.0472)
CALF_LIMIT = (-2.7227, -0.83776)
THIGH_LIMIT = {"FL": (-1.5707963267948966, 3.4907),
               "FR": (-1.5707963267948966, 3.4907),
               "RL": (-0.5236, 4.5379),
               "RR": (-0.5236, 4.5379)}

JOINT_NAMES = [f"{leg}_{part}_joint" for leg in LEGS
               for part in ("hip", "thigh", "calf")]

NAMED_POSTURES = {"tall": 0.46, "stand": 0.40, "crouch": 0.30, "sit": 0.22}

G = 9.80665


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quat_to_rpy(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Leg kinematics. Both directions, to the WHEEL CENTRE (not the contact patch:
# the contact is one radius below the centre along world-down, which depends on
# body attitude and is applied separately where it matters).
# ---------------------------------------------------------------------------
def leg_fk(leg, q_hip, q_thigh, q_calf):
    """Wheel centre in the base frame."""
    sx, sy = SIGN[leg]
    # Planar two-link in the hip-rotated plane.
    xl = -L_THIGH * math.sin(q_thigh) - L_CALF * math.sin(q_thigh + q_calf)
    zl = -L_THIGH * math.cos(q_thigh) - L_CALF * math.cos(q_thigh + q_calf)
    ch, sh = math.cos(q_hip), math.sin(q_hip)
    return np.array([
        sx * HIP_X + xl,
        sy * HIP_Y + sy * D_Y * ch - zl * sh,
        sy * D_Y * sh + zl * ch,
    ])


def leg_ik(leg, target):
    """Joint angles putting `leg`'s wheel centre at `target` in the base frame.

    Returns None if the target is out of reach; the caller keeps the previous
    command rather than clamping to something arbitrary.
    """
    sx, sy = SIGN[leg]
    vx = target[0] - sx * HIP_X
    vy = target[1] - sy * HIP_Y
    vz = target[2]

    # Abduction. Rotating about x carries (sy*D_Y, zl) onto (vy, vz), and that
    # rotation preserves the norm, which pins zl before the angle is known.
    zl_sq = vy * vy + vz * vz - D_Y * D_Y
    if zl_sq <= 1e-9:
        return None
    zl = -math.sqrt(zl_sq)
    q_hip = math.atan2(vz, vy) - math.atan2(zl, sy * D_Y)
    q_hip = math.atan2(math.sin(q_hip), math.cos(q_hip))

    # Two-link, knee-back branch -- the one the calf limits already enforce.
    reach_sq = vx * vx + zl * zl
    cos_c = (reach_sq - L_THIGH * L_THIGH - L_CALF * L_CALF) / (2 * L_THIGH * L_CALF)
    if not -1.0 <= cos_c <= 1.0:
        return None
    q_calf = -math.acos(cos_c)

    a = L_THIGH + L_CALF * math.cos(q_calf)
    b = L_CALF * math.sin(q_calf)
    q_thigh = math.atan2(-vx, -zl) - math.atan2(b, a)

    lo, hi = THIGH_LIMIT[leg]
    return (clamp(q_hip, *HIP_LIMIT),
            clamp(q_thigh, lo, hi),
            clamp(q_calf, *CALF_LIMIT))


# Link masses and CoM offsets, straight out of the URDF inertials. Used only to
# draw the CoM and to size the crouch; the balance criterion itself is measured
# from the force sensors and does not depend on any of this.
BODY_MASS = 6.9210
BODY_COM = np.array([0.021112, 0.0, -0.005366])
LEG_LINKS = (  # (frame, mass, com offset; y is mirrored for the right-hand legs)
    ("hip", 0.6780, np.array([-0.0054, 0.00194, -0.000105])),
    ("thigh", 1.1520, np.array([-0.00374, -0.0223, -0.0327])),
    ("calf", 0.1540, np.array([0.00548, -0.000975, -0.115])),
    # foot_motor hangs off the calf at (0, +/-0.01, -0.2264), i.e. beside the
    # wheel centre; its own inertial origin is zero.
    ("motor", 0.6460, np.array([0.0, 0.01, 0.0])),
    ("wheel", 0.5200, np.array([0.0, 0.06, 0.0])),
)
TOTAL_MASS = BODY_MASS + 4.0 * sum(m for _, m, _ in LEG_LINKS)


def body_com(q):
    """Whole-robot CoM in the base frame, for the current leg configuration."""
    acc = BODY_MASS * BODY_COM
    for leg in LEGS:
        sx, sy = SIGN[leg]
        qh, qt, qc = q[leg]
        r_hip = rx(qh)
        p_hip = np.array([sx * HIP_X, sy * HIP_Y, 0.0])
        p_thigh = p_hip + r_hip @ np.array([0.0, sy * ABAD_Y, 0.0])
        r_thigh = r_hip @ ry(qt)
        p_calf = p_thigh + r_thigh @ np.array([0.0, 0.0, -L_THIGH])
        r_calf = r_thigh @ ry(qc)
        p_wheel = p_calf + r_calf @ np.array([0.0, 0.0, -L_CALF])
        frames = {"hip": (p_hip, r_hip), "thigh": (p_thigh, r_thigh),
                  "calf": (p_calf, r_calf), "motor": (p_wheel, r_calf),
                  "wheel": (p_wheel, r_calf)}
        for name, mass, com in LEG_LINKS:
            origin, rot = frames[name]
            # The y-offsets in the URDF are mirrored for the right-hand legs.
            local = np.array([com[0], sy * com[1], com[2]])
            acc = acc + mass * (origin + rot @ local)
    return acc / TOTAL_MASS


def link_rotation(leg, q_hip, q_thigh, q_calf):
    """Rotation of the calf frame in the base frame (foot sensors report here)."""
    return rx(q_hip) @ ry(q_thigh + q_calf)


# ---------------------------------------------------------------------------
# Support polygon
# ---------------------------------------------------------------------------
def polygon_margin(polygon, point):
    """Signed distance from `point` to the boundary of a convex polygon.

    Positive inside. `polygon` must be a list of (x, y) already ordered
    counter-clockwise. Degenerate cases (fewer than three contacts) return a
    negative number, because a robot on two wheels has no polygon left.
    """
    n = len(polygon)
    if n < 3:
        return -1.0
    best = float("inf")
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        length = math.hypot(ex, ey)
        if length < 1e-6:
            continue
        # Left of a CCW edge is inside.
        cross = ex * (point[1] - y1) - ey * (point[0] - x1)
        best = min(best, cross / length)
    return best if best < float("inf") else -1.0


def order_ccw(points):
    """Sort (x, y) points counter-clockwise about their centroid."""
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    return sorted(points, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


class MagiBalance(Node):
    def __init__(self):
        super().__init__("magi_balance")

        p = self.declare_parameter
        p("rate", 100.0)
        # How far ahead each streamed trajectory point is placed. Each new
        # message supersedes the last, so this behaves as a first-order filter
        # with roughly this time constant rather than as a real deadline.
        p("command_horizon", 0.05)
        p("controller", "leg_controller")

        p("ride_height", NAMED_POSTURES["stand"])
        p("stand_time", 2.5)           # limp -> stance ramp
        p("startup_delay", 2.0)        # let the controllers activate first

        # Attitude loop. Gains are dimensionless: 1.0 would command the full
        # measured error as a body rotation in a single cycle, which overshoots
        # given the trajectory horizon above, so they sit below that.
        #
        # The integral is not optional. Proportional-only, this rejected the
        # varying part of the tilt well (roll sd 1.04 -> 0.31 deg standing on
        # Rubicon) but left the STEADY part almost untouched (mean 1.26 ->
        # 1.14 deg), because the legs droop under load and a P term cannot
        # cancel a constant. Levelling the body on a slope is exactly a
        # constant, so it needs the integrator.
        p("roll_kp", 0.55)
        p("roll_ki", 0.60)
        p("roll_kd", 0.06)
        p("pitch_kp", 0.45)
        p("pitch_ki", 0.50)
        p("pitch_kd", 0.10)
        p("tilt_i_max", 0.15)          # rad, integral authority (8.6 deg)
        p("tilt_max", 0.30)            # rad, cap on the commanded body rotation
        p("command_tau", 0.10)         # s, low-pass on the commanded tilt

        # Lean feed-forward.
        p("lean_gain", 1.0)            # 1.0 = geometrically exact atan(v*w/g)
        p("lean_max", 0.25)            # rad
        p("accel_tau", 0.25)           # s, low-pass on d(v_cmd)/dt

        # Reshaping. Widening and crouching are SLOW levers -- the wheels have
        # to scrub sideways to widen, so the rate limit below means a full
        # widen takes about 0.6 s. That is far too slow to answer a terrain
        # impulse, which is what actually rolls this robot: at 0.35 m/s and
        # 0.15 rad/s the steady turn load moves the CoP by under 2 mm out of
        # 190 mm, so a purely margin-reactive trigger would fire only once the
        # robot is already going over. They are therefore driven by the
        # anticipatory terms as well -- speed and recent terrain roughness --
        # so the stance is already wide by the time a bump arrives.
        p("margin_target", 0.070)      # m; below this the polygon starts growing
        p("margin_tau", 0.30)          # s, low-pass on the margin
        p("speed_calm", 0.20)          # m/s below which no anticipation
        p("speed_wide", 0.80)          # m/s at which the stance is fully wide
        p("rough_ref", 0.35)           # rad/s RMS body rate = fully wide
        p("rough_tau", 1.0)            # s, window on the roughness estimate
        # Residual gyro noise left after gyro_tau, removed in quadrature so a
        # motionless robot reports zero roughness rather than its noise floor.
        p("rough_floor", 0.080)        # rad/s
        p("gyro_tau", 0.05)            # s, low-pass on the body rates
        p("widen_max", 0.070)          # m of extra half-track
        p("crouch_max", 0.060)         # m the ride height may drop by
        p("reach_max", 0.060)          # m of extra extension to chase lost contact
        p("reshape_rate", 0.12)        # m/s, how fast width and height may move

        p("dz_max", 0.090)             # m, cap on per-foot deviation from nominal
        p("contact_force", 5.0)        # N
        p("force_sign", 0.0)           # 0 = detect at runtime

        self.rate = float(self.get_parameter("rate").value)
        self.dt = 1.0 / self.rate
        self.horizon = float(self.get_parameter("command_horizon").value)
        self.stand_time = float(self.get_parameter("stand_time").value)

        # ---- state -----------------------------------------------------
        self.q = {}                    # leg -> (hip, thigh, calf) measured
        self.roll = self.pitch = 0.0
        self.wx = self.wy = 0.0
        self.have_imu = False
        self.force = {}
        self.force_sign = float(self.get_parameter("force_sign").value)

        self.v_cmd = self.w_cmd = 0.0
        self.v_prev = 0.0
        self.accel = 0.0

        self.height_target = float(self.get_parameter("ride_height").value)
        self.height = None             # current ramped ride height
        self.widen = 0.0
        self.crouch = 0.0
        self.reach = 0.0
        self.n_contacts = 0
        self.margin = 0.0
        self.margin_valid = False
        self.rough_sq = 0.0
        self.urgency = 0.0
        self.i_roll = self.i_pitch = 0.0
        self.d_roll_f = self.d_pitch_f = 0.0
        self.enabled = True
        self.phase = "wait"
        self.stand_t = 0.0
        self.last_cmd = None

        # ---- interfaces -------------------------------------------------
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, sensor_qos)
        for leg in LEGS:
            self.create_subscription(
                Wrench, f"/foot_force/{leg}",
                lambda m, k=leg: self.force.__setitem__(k, m), sensor_qos)
        self.create_subscription(
            Twist, "/wheel_controller/cmd_vel_unstamped", self._on_cmd, 10)
        self.create_subscription(Float64, "/magi/body_height", self._on_height, 10)
        self.create_subscription(String, "/magi/posture", self._on_posture, 10)
        self.create_subscription(Bool, "/magi/balance_enable", self._on_enable, 10)

        controller = self.get_parameter("controller").value
        self.pub_traj = self.create_publisher(
            JointTrajectory, f"/{controller}/joint_trajectory", 10)
        self.pub_margin = self.create_publisher(Float64, "/magi/stability_margin", 10)
        self.pub_poly = self.create_publisher(PolygonStamped, "/magi/support_polygon", 10)
        self.pub_cop = self.create_publisher(PointStamped, "/magi/cop", 10)
        self.pub_com = self.create_publisher(PointStamped, "/magi/com", 10)
        self.pub_width = self.create_publisher(Float64, "/magi/stance_width", 10)
        self.pub_state = self.create_publisher(String, "/magi/balance_state", 10)

        self.start_time = None
        self.create_timer(self.dt, self._update)
        self.create_timer(1.0, self._report)

        self.get_logger().info(
            f"balance controller up: {self.rate:.0f} Hz, ride height "
            f"{self.height_target:.2f} m, margin target "
            f"{self.get_parameter('margin_target').value:.3f} m, "
            f"nominal half-track {HALF_TRACK_NOM:.4f} m, mass {TOTAL_MASS:.2f} kg")

    # ---- callbacks -------------------------------------------------------
    def _on_joints(self, msg):
        index = {n: i for i, n in enumerate(msg.name)}
        for leg in LEGS:
            try:
                self.q[leg] = tuple(
                    msg.position[index[f"{leg}_{part}_joint"]]
                    for part in ("hip", "thigh", "calf"))
            except (KeyError, IndexError):
                return

    def _on_imu(self, msg):
        self.roll, self.pitch, _ = quat_to_rpy(msg.orientation)
        # The IMU carries a MEMS noise model, and raw gyro is dominated by it:
        # measured standing perfectly still, the rate reads 0.12 rad/s RMS
        # while the attitude itself moves by 0.05 deg. Unfiltered that noise
        # went two places it did not belong -- into the D term, where it became
        # commanded tilt jitter, and into the roughness estimate, where it held
        # the stance permanently widened with nothing happening. Terrain
        # disturbance lives well below a few Hz, so a short low-pass keeps the
        # signal and drops most of the noise power.
        tau = float(self.get_parameter("gyro_tau").value)
        dt = 1.0 / 200.0                       # IMU rate, from the URDF sensor
        a = dt / (tau + dt)
        self.wx += a * (msg.angular_velocity.x - self.wx)
        self.wy += a * (msg.angular_velocity.y - self.wy)
        self.have_imu = True

    def _on_cmd(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z

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
            f"balance feedback {'enabled' if self.enabled else 'FROZEN (fixed stance)'}")

    # ---- stability measurement -------------------------------------------
    def _stability(self, r_grav):
        """Support polygon, CoP and margin, in the yaw-free gravity frame.

        Returns (cop_xy, margin, contacts_base, com_base) or None.
        """
        com_b = body_com(self.q)

        contacts_b = {}
        loads = {}
        # World-up expressed in the base frame: the third row of R(roll, pitch).
        up_b = r_grav.T @ np.array([0.0, 0.0, 1.0])
        threshold = float(self.get_parameter("contact_force").value)

        for leg in LEGS:
            qh, qt, qc = self.q[leg]
            centre = leg_fk(leg, qh, qt, qc)
            contacts_b[leg] = centre - WHEEL_R * up_b

            w = self.force.get(leg)
            if w is None:
                continue
            f_calf = np.array([w.force.x, w.force.y, w.force.z])
            if np.linalg.norm(f_calf) < 1e-9:
                continue
            # The sensor reports in the calf frame; rotate to base, then to the
            # gravity frame, and take the component along world-up.
            f_g = r_grav @ (link_rotation(leg, qh, qt, qc) @ f_calf)
            loads[leg] = f_g[2]

        if not loads:
            return None

        # measure_direction is parent_to_child, so a loaded wheel reads negative
        # along world-up. Rather than trust that, settle the sign once from the
        # data: whichever choice makes the robot's own weight come out positive.
        if self.force_sign == 0.0:
            total = sum(loads.values())
            if abs(total) < 1.0:
                return None
            self.force_sign = 1.0 if total > 0 else -1.0
            self.get_logger().info(
                f"foot force sign resolved to {self.force_sign:+.0f} "
                f"(total {self.force_sign * total:.1f} N against "
                f"{TOTAL_MASS * G:.1f} N of weight)")

        stance = []
        weights = []
        for leg, fz in loads.items():
            load = self.force_sign * fz
            if load >= threshold:
                stance.append(leg)
                weights.append(load)

        if len(stance) < 3 or sum(weights) < 1e-6:
            # Three contacts is the minimum for a polygon at all. Below that the
            # margin is UNKNOWN, not catastrophic -- returning a large negative
            # number here fed straight into the urgency term and pegged it,
            # which crouched the robot onto its belly, which unloaded the feet,
            # which kept the count below three. None keeps that loop open.
            return (None, None, len(stance), contacts_b, com_b)

        pts_g = {leg: r_grav @ contacts_b[leg] for leg in stance}
        total = sum(weights)
        cop = np.zeros(2)
        for leg, wgt in zip(stance, weights):
            cop += wgt * pts_g[leg][:2]
        cop /= total

        polygon = order_ccw([(pts_g[leg][0], pts_g[leg][1]) for leg in stance])
        return (cop, polygon_margin(polygon, cop), len(stance), contacts_b, com_b)

    # ---- control ---------------------------------------------------------
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

        r_grav = ry(self.pitch) @ rx(self.roll)

        # ---- where the robot stands right now ---------------------------
        stab = self._stability(r_grav)
        contacts_b = com_b = None
        if stab is not None:
            cop, margin, self.n_contacts, contacts_b, com_b = stab
            if margin is None:
                self.margin_valid = False
            else:
                alpha = self.dt / (
                    float(self.get_parameter("margin_tau").value) + self.dt)
                self.margin += alpha * (margin - self.margin)
                self.margin_valid = True
            self._publish_stability(cop, contacts_b, com_b)

        # ---- ride height: ramp up from wherever the legs actually are ----
        if self.height is None:
            self.stand_from = self.height = self._measured_height(r_grav)
            self.phase = "stand"
            self.stand_t = 0.0
            self.get_logger().info(
                f"standing up from {self.height:.3f} m to {self.height_target:.3f} m")

        if self.phase == "stand":
            self.stand_t += self.dt
            s = clamp(self.stand_t / self.stand_time, 0.0, 1.0)
            # Smoothstep, so the legs do not jerk at either end of the ramp.
            blend = s * s * (3.0 - 2.0 * s)
            self.height = self.stand_from + blend * (self.height_target - self.stand_from)
            if s >= 1.0:
                self.phase = "active"
                self.get_logger().info("stance reached; balance feedback live")
        else:
            # Track the commanded height with the same rate limit the reshaping
            # uses, so a posture change is a move rather than a step.
            step = float(self.get_parameter("reshape_rate").value) * self.dt
            self.height += clamp(self.height_target - self.height, -step, step)

        authority = 0.0 if self.phase == "stand" else 1.0
        if not self.enabled:
            authority = 0.0

        # ---- lean reference ---------------------------------------------
        a_tau = float(self.get_parameter("accel_tau").value)
        raw_accel = (self.v_cmd - self.v_prev) / self.dt
        self.v_prev = self.v_cmd
        self.accel += (self.dt / (a_tau + self.dt)) * (raw_accel - self.accel)

        lean_gain = float(self.get_parameter("lean_gain").value)
        lean_max = float(self.get_parameter("lean_max").value)
        # A left turn (w > 0) throws the resultant to the right, so the body
        # rolls left -- negative roll -- to put the CoM back over the polygon.
        roll_ref = clamp(-lean_gain * math.atan2(self.v_cmd * self.w_cmd, G),
                         -lean_max, lean_max)
        # Positive pitch is nose-down, which carries the CoM forward; that is
        # what resists the backward pseudo-force under forward acceleration.
        pitch_ref = clamp(lean_gain * math.atan2(self.accel, G), -lean_max, lean_max)

        # ---- attitude PID --------------------------------------------------
        tilt_max = float(self.get_parameter("tilt_max").value)
        i_max = float(self.get_parameter("tilt_i_max").value)
        err_roll = roll_ref - self.roll
        err_pitch = pitch_ref - self.pitch

        # Integrate only while genuinely standing on a polygon and with the
        # loop in charge; otherwise the term would wind up during the stand-up
        # ramp, or while a wheel is off the ground and the error is not
        # something the legs can fix.
        if authority > 0.0 and self.margin_valid and self.margin > 0.0:
            self.i_roll = clamp(
                self.i_roll + float(self.get_parameter("roll_ki").value) * err_roll * self.dt,
                -i_max, i_max)
            self.i_pitch = clamp(
                self.i_pitch + float(self.get_parameter("pitch_ki").value) * err_pitch * self.dt,
                -i_max, i_max)
        else:
            self.i_roll *= 0.995
            self.i_pitch *= 0.995

        d_roll = (float(self.get_parameter("roll_kp").value) * err_roll + self.i_roll
                  - float(self.get_parameter("roll_kd").value) * self.wx)
        d_pitch = (float(self.get_parameter("pitch_kp").value) * err_pitch + self.i_pitch
                   - float(self.get_parameter("pitch_kd").value) * self.wy)
        d_roll = authority * clamp(d_roll, -tilt_max, tilt_max)
        d_pitch = authority * clamp(d_pitch, -tilt_max, tilt_max)

        # Roll the loop gain off above a couple of Hz. Without this the pitch
        # axis limit-cycled at ~10 Hz: body rate rms went 0.037 -> 0.183 rad/s
        # the moment the loop was enabled. The 0.05 s trajectory horizon alone
        # is ~160 deg of phase at 10 Hz, so anything with gain left up there
        # oscillates. Pitch goes first because the wheels are velocity-servoed
        # to zero, so fore-aft contact motion fights a stiff servo, where the
        # lateral motion a roll correction needs is absorbed by tyre slip.
        # Terrain disturbance the legs can actually do something about is below
        # 2 Hz, so nothing useful is lost.
        c_tau = float(self.get_parameter("command_tau").value)
        a = self.dt / (c_tau + self.dt)
        self.d_roll_f += a * (d_roll - self.d_roll_f)
        self.d_pitch_f += a * (d_pitch - self.d_pitch_f)
        d_roll, d_pitch = self.d_roll_f, self.d_pitch_f

        # ---- reshaping: reactive margin, plus anticipation ----------------
        # Roughness is the RMS body angular RATE, not the attitude error. The
        # error contains the steady tilt of whatever slope the robot is parked
        # on, which is not roughness at all: measured standing still, a 1.1 deg
        # constant offset swamped the 0.3 deg of actual motion and held the
        # stance permanently widened. Rate is inherently zero-mean, so it
        # reports only how hard the terrain is shaking the body.
        r_tau = float(self.get_parameter("rough_tau").value)
        floor = float(self.get_parameter("rough_floor").value)
        rate_sq = max(self.wx * self.wx + self.wy * self.wy - floor * floor, 0.0)
        self.rough_sq += (self.dt / (r_tau + self.dt)) * (rate_sq - self.rough_sq)

        target = float(self.get_parameter("margin_target").value)
        calm = float(self.get_parameter("speed_calm").value)
        wide = float(self.get_parameter("speed_wide").value)
        rough_ref = float(self.get_parameter("rough_ref").value)

        step = float(self.get_parameter("reshape_rate").value) * self.dt
        contact_ok = self.n_contacts >= 3

        if authority > 0.0:
            u_margin = (clamp((target - self.margin) / max(target, 1e-3), 0.0, 1.0)
                        if self.margin_valid else 0.0)
            u_speed = clamp((abs(self.v_cmd) - calm) / max(wide - calm, 1e-3), 0.0, 1.0)
            u_rough = clamp(math.sqrt(self.rough_sq) / max(rough_ref, 1e-3), 0.0, 1.0)
            urgency = max(u_margin, u_speed, u_rough)
        else:
            urgency = 0.0

        # Losing contact is not the same problem as being close to tipping, and
        # it wants the opposite response. Fewer than three loaded feet means the
        # ground has dropped away or the body has grounded out -- either way the
        # legs need to reach DOWN to find the surface, and crouching digs the
        # robot in further. Measured before this existed: driving crouched the
        # robot to 0.30 m, three feet fell to ~5 N against 191 N of weight, and
        # it stayed there after the command stopped, covering 0.3 m in 10 s.
        reach_max = float(self.get_parameter("reach_max").value)
        if authority > 0.0 and not contact_ok:
            urgency = 0.0
            self.reach += clamp(reach_max - self.reach, -step, step)
        else:
            self.reach += clamp(-self.reach, -step, step)

        self.urgency = urgency
        widen_goal = urgency * float(self.get_parameter("widen_max").value)
        crouch_goal = urgency * float(self.get_parameter("crouch_max").value)
        self.widen += clamp(widen_goal - self.widen, -step, step)
        self.crouch += clamp(crouch_goal - self.crouch, -step, step)

        # ---- foot targets and IK ------------------------------------------
        half_track = HALF_TRACK_NOM + self.widen
        ride = clamp(self.height - self.crouch + self.reach, 0.24, 0.47)
        z_nom = -(ride - WHEEL_R)
        dz_max = float(self.get_parameter("dz_max").value)

        command = {}
        for leg in LEGS:
            sx, sy = SIGN[leg]
            x = sx * HIP_X
            y = sy * half_track
            # Third row of R(d_roll, d_pitch)^T applied to the nominal foot:
            # how far the foot moves in body coordinates when the body rotates
            # while the contact stays put.
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

    def _measured_height(self, r_grav):
        """Body height above the contact plane, from the current joint angles."""
        up_b = r_grav.T @ np.array([0.0, 0.0, 1.0])
        drops = []
        for leg in LEGS:
            centre = leg_fk(leg, *self.q[leg])
            contact = centre - WHEEL_R * up_b
            drops.append(-float(np.dot(r_grav @ contact, np.array([0.0, 0.0, 1.0]))))
        return clamp(sum(drops) / len(drops), 0.10, 0.48)

    def _send(self, command):
        point = JointTrajectoryPoint()
        point.positions = [command[leg][i] for leg in LEGS for i in range(3)]
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start = Duration(
            sec=int(self.horizon),
            nanosec=int((self.horizon % 1.0) * 1e9))

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points = [point]
        self.pub_traj.publish(traj)

    # ---- output ----------------------------------------------------------
    def _publish_stability(self, cop, contacts_b, com_b):
        stamp = self.get_clock().now().to_msg()

        self.pub_margin.publish(Float64(data=float(self.margin)))

        poly = PolygonStamped()
        poly.header.stamp = stamp
        poly.header.frame_id = "base"
        # Drawn in the base frame from the actual contact points, so the
        # rendered quad tilts and stretches exactly as the legs move.
        for leg in order_ccw_legs(contacts_b):
            p = contacts_b[leg]
            poly.polygon.points.append(
                Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        self.pub_poly.publish(poly)

        com = PointStamped()
        com.header.stamp = stamp
        com.header.frame_id = "base"
        com.point.x, com.point.y, com.point.z = (float(v) for v in com_b)
        self.pub_com.publish(com)

        if cop is not None:
            # Back out of the gravity frame so it can be drawn alongside the
            # polygon in base coordinates.
            r_grav = ry(self.pitch) @ rx(self.roll)
            mean_z = float(np.mean([(r_grav @ contacts_b[leg])[2] for leg in LEGS]))
            p_b = r_grav.T @ np.array([cop[0], cop[1], mean_z])
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "base"
            msg.point.x, msg.point.y, msg.point.z = (float(v) for v in p_b)
            self.pub_cop.publish(msg)

    def _report(self):
        if self.height is None:
            return
        margin = f"{self.margin:+.3f}" if self.margin_valid else "  n/a"
        state = (f"{self.phase} h={self.height - self.crouch + self.reach:.3f} "
                 f"feet={self.n_contacts} margin={margin} "
                 f"track={2 * (HALF_TRACK_NOM + self.widen):.3f} "
                 f"urgency={self.urgency:.2f} rough={math.sqrt(self.rough_sq):.2f}rad/s "
                 f"rp={math.degrees(self.roll):+.1f}/{math.degrees(self.pitch):+.1f}deg "
                 f"i={math.degrees(self.i_roll):+.1f}/{math.degrees(self.i_pitch):+.1f}deg")
        self.pub_state.publish(String(data=state))


def order_ccw_legs(contacts_b):
    """All four leg names, ordered counter-clockwise about the stance centroid."""
    legs = list(LEGS)
    cx = sum(contacts_b[l][0] for l in legs) / len(legs)
    cy = sum(contacts_b[l][1] for l in legs) / len(legs)
    return sorted(legs, key=lambda l: math.atan2(contacts_b[l][1] - cy,
                                                 contacts_b[l][0] - cx))


def main():
    rclpy.init()
    node = MagiBalance()
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
