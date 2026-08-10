#!/usr/bin/env python3
"""Rigid-body model of the MAGI Go2W leg: geometry, kinematics, inertials.

Shared by magi_balance.py (the original quasi-static balance controller) and
magi_stabilizer.py (the dynamic tipover-prevention controller that supersedes
it), so the physical constants have exactly one definition. Everything here is
read off go2w_body.xacro; if the URDF changes, this file is the only place that
has to follow.

No ROS in this module -- it is plain numpy, so it can be exercised from a unit
test or a REPL without a running graph.
"""

import math

import numpy as np

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
HALF_BASE = HIP_X                     # 0.1934, half the fore/aft wheelbase

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


# Link masses and CoM offsets, straight out of the URDF inertials.
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
TOTAL_MASS = BODY_MASS + 4.0 * sum(m for _, m, _ in LEG_LINKS)   # 19.521 kg


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
# Support polygon helpers
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


# ---------------------------------------------------------------------------
# Force-angle stability measure (Papadopoulos & Rey, 1996).
#
# WHY NOT JUST "IS THE CoP INSIDE THE POLYGON".
#
# The inside/outside test answers a yes/no question about a quasi-static robot.
# It has three failures that matter here:
#
#   * it needs three contacts to mean anything, so it goes blind exactly when a
#     wheel lifts -- which is the moment before a rollover, not a moment to be
#     blind in;
#   * "distance to the edge" is measured in metres on the ground, which mixes
#     badly across the fore/aft and lateral directions and says nothing about
#     how much *acceleration* is left;
#   * it is quasi-static, so it lags a dynamic tipover.
#
# The force-angle measure answers the question actually being asked: how far
# can the net force vector rotate before the robot goes over. For each edge of
# the support pattern it takes the angle between the net force (gravity plus
# every inertial pseudo-force, which is exactly what an accelerometer reports)
# and the perpendicular from the edge axis to the CoM. That angle hits zero at
# the instant of tipover, whatever the robot is doing at the time. It works
# with two contacts, on a slope, in a turn and over a bump, and its units are
# radians of remaining tilt, which is directly comparable across directions.
# ---------------------------------------------------------------------------
def force_angle_margin(contacts, com, f_dir):
    """Smallest tipover angle over all support edges, radians.

    contacts  list of 3-vectors, the contact points, in boundary order (use
              order_contacts_ccw); "boundary order" only has to be a cycle
              around the pattern, either handedness
    com       3-vector, whole-robot centre of mass
    f_dir     3-vector, unit, the direction the net force acts (effective
              gravity: "down" including centrifugal and terrain accelerations)

    Returns (margin, index_of_critical_edge). Positive is upright; the value is
    how far the force vector may still rotate about the worst edge before the
    robot tips over it. Two contacts still give a two-edge pattern (the same
    axis both ways round), which correctly reads ~0 for a robot balanced on one
    diagonal. Fewer than two returns 0.0 -- nothing is holding it up.

    The per-edge angle is unsigned: rotating the force AWAY from an edge is
    safe as far as that edge is concerned, and the robot tips over whichever
    edge the force reaches first, so the minimum is the answer. The overall
    SIGN comes from one inside/outside test of the whole pattern, which is
    independent of how the edges happen to be traversed -- an ordering-derived
    sign per edge is easy to get backwards and silently reports a robot on its
    side as maximally stable.
    """
    n = len(contacts)
    if n < 2:
        return 0.0, -1

    eye = np.eye(3)
    best = float("inf")
    worst = -1
    for i in range(n):
        p1 = contacts[i]
        p2 = contacts[(i + 1) % n]
        axis = p2 - p1
        norm = np.linalg.norm(axis)
        if norm < 1e-6:
            continue
        axis = axis / norm
        perp = eye - np.outer(axis, axis)

        # Perpendicular from the tipover axis to the CoM: the lever the weight
        # acts on. Points from the CoM towards the axis.
        l_vec = perp @ (p1 - com)
        l_norm = np.linalg.norm(l_vec)
        if l_norm < 1e-6:
            # CoM sits on the axis: already balanced on a knife edge.
            return 0.0, i
        l_hat = l_vec / l_norm

        # Only the component of the force perpendicular to the axis can rotate
        # the robot about it.
        f_perp = perp @ f_dir
        f_norm = np.linalg.norm(f_perp)
        if f_norm < 1e-9:
            continue

        theta = math.acos(clamp(float(np.dot(f_perp / f_norm, l_hat)), -1.0, 1.0))
        if theta < best:
            best, worst = theta, i

    if best == float("inf"):
        return 0.0, -1
    return (best if force_line_inside(contacts, com, f_dir) else -best), worst


def force_line_inside(contacts, com, f_dir):
    """Does the net-force line through the CoM pass inside the support pattern?

    The test is done in the plane normal to f_dir, which is exactly the
    projection that decides tipping: gravity (plus every inertial term) acts
    along f_dir, so the robot is upright precisely while that line pierces the
    pattern. Needs three contacts to bound an area at all.
    """
    if len(contacts) < 3:
        return False
    # The pattern has to be BELOW the CoM along the force direction. Without
    # this the test is purely two-dimensional and happily reports an upside
    # down robot -- whose wheels project inside its own footprint just as well
    # as they do the right way up -- as maximally stable.
    centroid = sum(contacts) / len(contacts)
    if float(np.dot(centroid - com, f_dir)) <= 0.0:
        return False
    ex, ey = plane_basis(f_dir)
    pts = [(float(np.dot(p - com, ex)), float(np.dot(p - com, ey)))
           for p in contacts]
    sign = 0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        # Cross product of the edge with the vector to the origin (the CoM,
        # which is the projected force line by construction).
        cross = (x2 - x1) * (0.0 - y1) - (y2 - y1) * (0.0 - x1)
        if abs(cross) < 1e-12:
            continue
        s = 1 if cross > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return sign != 0


def plane_basis(f_dir):
    """Two orthonormal axes spanning the plane normal to f_dir."""
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, f_dir))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    ex = np.cross(f_dir, ref)
    ex = ex / np.linalg.norm(ex)
    ey = np.cross(f_dir, ex)
    return ex, ey


def order_contacts_ccw(contacts, com, f_dir):
    """Order contact points counter-clockwise in the plane normal to f_dir.

    force_angle_margin walks the support pattern edge by edge, so the points
    have to come in boundary order or the "edges" cut across the interior.
    """
    if len(contacts) < 2:
        return list(contacts)
    ex, ey = plane_basis(f_dir)
    return sorted(contacts,
                  key=lambda p: math.atan2(float(np.dot(p - com, ey)),
                                           float(np.dot(p - com, ex))))
