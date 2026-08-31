#!/usr/bin/env python3
"""Leg-kinematics height and vertical velocity for the MAGI Go2W.

Bounds the EKF's vertical channel, which accelerometers alone cannot do. With
only IMU and wheel odometry, z has no measurement at all: feeding linear
acceleration in gives the filter something to integrate but nothing to correct
against, so the accelerometer bias double-integrates. Measured on this robot
while completely stationary, that produced -108 m of z in 40 s, drifting at
0.098 m/s^2 against the 0.1 m/s^2 bias configured in the URDF.

The legs can measure what the IMU cannot. Each wheel in contact is a point of
known position relative to the body (forward kinematics through TF) touching a
surface, so the body's height above that surface is directly observable:

    h = wheel_radius - (R * t_i).z

where t_i is the axle offset in the base frame and R is body orientation. Only
roll and pitch are used to build R -- the z component of R*t is invariant to
yaw -- which conveniently avoids the IMU's drifting, unreferenced heading.

WHAT IS PUBLISHED, AND WHAT IT MEANS

  /magi/terrain_height   Float64, body height above the contact plane. This is
                         height above LOCAL TERRAIN, not height above the odom
                         origin. Useful directly for terrain-adaptive posture.

  /magi/contacts         Which feet are loaded, from the foot force sensors.

  /magi/leg_twist        TwistWithCovarianceStamped carrying vz only, for the
                         EKF to fuse as twist0.

vz is dh/dt, i.e. vertical velocity RELATIVE TO THE TERRAIN. On level ground
that is true vertical velocity. Driving up a slope at constant ride height it
reads zero while the robot genuinely climbs, so it does not let odom track
absolute elevation -- that still needs SLAM. What it does do is pin the
vertical channel to a measured quantity, so z error becomes a slow random walk
instead of an unbounded quadratic divergence.

WHERE Z = 0 IS, AND WHY THIS NODE DECIDES IT

REP-105 leaves the odom origin arbitrary, and robot_localization's default is
the base frame's pose when the filter starts. On a robot whose base_link sits
on its axles that is close enough to the ground for nobody to notice. On this
one `base` rides 0.35 m up, so z = 0 in odom -- and therefore in map -- landed
at the robot's CHEST, not on the floor.

Everything that is intrinsically two-dimensional then floats there. RTAB-Map's
/map occupancy grid is a nav_msgs/OccupancyGrid with origin.position.z = 0, so
RViz drew it as a flat plane 0.35 m above the terrain, cutting through the
robot: the map looked higher than the robot and the wheels hung below it. The
3D cloud was never wrong -- measured against the simulator's own heightmap, its
ground returns sit within 0.03 m of the true surface -- and neither was the
robot. Only the datum was.

So once the robot is standing still and the height reading has settled, this
node calls the EKF's /set_pose once and moves the z datum down by exactly the
height it just measured. After that z = 0 in odom is the ground plane under the
start point, /odometry/filtered reports the robot's real ride height instead of
~0, and the occupancy grid lands on the wheels. x, y and orientation are taken
from the filter's own current estimate, so this is a pure vertical shift and
nothing else about the pose is disturbed.

Set set_ground_datum:=false to keep the old base-height datum.
"""

import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped, Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_localization.srv import SetPose
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener

LEGS = ["FL", "FR", "RL", "RR"]


def quat_to_roll_pitch(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch


def rotate_z_component(t, roll, pitch):
    """z component of R(roll, pitch) * t. Yaw is irrelevant to this."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    # Third row of Rz(yaw)Ry(pitch)Rx(roll) is independent of yaw.
    return -sp * t[0] + cp * sr * t[1] + cp * cr * t[2]


class LegOdometry(Node):
    def __init__(self):
        super().__init__("magi_leg_odometry")

        self.declare_parameter("wheel_radius", 0.086)
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("contact_force_threshold", 5.0)
        self.declare_parameter("rate", 50.0)
        # dh/dt from finite differences is noisy; this is the time constant of
        # the low-pass on vz, in seconds.
        self.declare_parameter("velocity_filter_tau", 0.08)
        self.declare_parameter("vz_stddev", 0.05)
        # ---- ground datum (see the module docstring) ------------------------
        self.declare_parameter("set_ground_datum", True)
        self.declare_parameter("datum_service", "/set_pose")
        # How long the height has to sit still, and how still, before the
        # reading is taken as the ride height rather than part of the stand-up.
        self.declare_parameter("datum_settle", 1.0)
        self.declare_parameter("datum_tolerance", 0.005)
        # Give up rather than shift the datum under a filter that has already
        # been running and moving; a late jump would drag the map with it.
        self.declare_parameter("datum_deadline", 60.0)

        self.radius = self.get_parameter("wheel_radius").value
        self.base = self.get_parameter("base_frame").value
        self.threshold = self.get_parameter("contact_force_threshold").value
        self.tau = self.get_parameter("velocity_filter_tau").value
        vz_var = self.get_parameter("vz_stddev").value ** 2

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.force = {}
        for leg in LEGS:
            self.create_subscription(
                Wrench, f"/foot_force/{leg}",
                lambda m, k=leg: self.force.__setitem__(k, m), sensor_qos)
        self.roll = self.pitch = 0.0
        self.create_subscription(Imu, "/imu/data", self._on_imu, sensor_qos)

        self.pub_h = self.create_publisher(Float64, "/magi/terrain_height", 10)
        self.pub_c = self.create_publisher(String, "/magi/contacts", 10)
        self.pub_t = self.create_publisher(
            TwistWithCovarianceStamped, "/magi/leg_twist", 10)

        self._cov = [0.0] * 36
        # Only vz is meaningful. Everything else is given a huge variance so a
        # consumer that ignores the config still cannot take it seriously.
        for i in (0, 7, 21, 28, 35):
            self._cov[i] = 1.0e6
        self._cov[14] = vz_var          # vz

        self.h = None
        self.vz = 0.0
        self.last_stamp = None

        self.datum_done = not bool(self.get_parameter("set_ground_datum").value)
        self.datum_since = None       # when the height last started sitting still
        self.datum_ref = None         # the height it has been sitting at
        self.datum_future = None
        self.odom = None
        self.started = self.get_clock().now()
        if not self.datum_done:
            self.create_subscription(
                Odometry, "/odometry/filtered",
                lambda m: setattr(self, "odom", m), 10)
            self.datum_client = self.create_client(
                SetPose, self.get_parameter("datum_service").value)

        period = 1.0 / self.get_parameter("rate").value
        self.create_timer(period, self._update)

        self.get_logger().info(
            f"leg odometry up: r={self.radius} m, contact threshold "
            f"{self.threshold} N, vz sigma {math.sqrt(vz_var)} m/s")

    def _on_imu(self, msg):
        self.roll, self.pitch = quat_to_roll_pitch(msg.orientation)

    def _stance(self):
        out = []
        for leg in LEGS:
            w = self.force.get(leg)
            if w is None:
                continue
            mag = math.sqrt(w.force.x ** 2 + w.force.y ** 2 + w.force.z ** 2)
            if mag >= self.threshold:
                out.append(leg)
        return out

    def _update(self):
        stance = self._stance()
        if not stance:
            return

        heights = []
        for leg in stance:
            try:
                tf = self.buffer.lookup_transform(
                    self.base, f"{leg}_foot", rclpy.time.Time())
            except Exception:
                continue
            t = (tf.transform.translation.x,
                 tf.transform.translation.y,
                 tf.transform.translation.z)
            heights.append(self.radius - rotate_z_component(t, self.roll, self.pitch))

        if not heights:
            return

        h = sum(heights) / len(heights)
        now = self.get_clock().now()

        if self.h is not None and self.last_stamp is not None:
            dt = (now - self.last_stamp).nanoseconds * 1e-9
            if dt > 1e-4:
                raw = (h - self.h) / dt
                alpha = dt / (self.tau + dt)
                self.vz += alpha * (raw - self.vz)
        self.h, self.last_stamp = h, now

        self.pub_h.publish(Float64(data=h))
        self.pub_c.publish(String(data=",".join(stance)))

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base
        msg.twist.twist.linear.z = self.vz
        msg.twist.covariance = self._cov
        self.pub_t.publish(msg)

        if not self.datum_done:
            self._ground_datum(h, len(stance), now)

    def _ground_datum(self, h, contacts, now):
        """Move the odom z datum from the body down to the ground, once.

        Waits for the robot to be standing on at least three feet with the
        measured height steady, so the value taken is the settled ride height
        and not a moment during the stand-up.
        """
        age = (now - self.started).nanoseconds * 1e-9
        if age > float(self.get_parameter("datum_deadline").value):
            self.datum_done = True
            self.get_logger().warn(
                "ground datum not set within "
                f"{self.get_parameter('datum_deadline').value:.0f} s; odom z "
                "stays referenced to the body, so a 2D map will sit one ride "
                "height above the ground")
            return

        if self.datum_future is not None:
            if not self.datum_future.done():
                return
            self.datum_done = True
            self.get_logger().info(
                f"ground datum set: odom z = 0 is now the ground plane, "
                f"{self.datum_ref:.3f} m below the body")
            return

        tol = float(self.get_parameter("datum_tolerance").value)
        if contacts < 3 or self.datum_ref is None or abs(h - self.datum_ref) > tol:
            self.datum_ref, self.datum_since = h, now
            return
        if (now - self.datum_since).nanoseconds * 1e-9 < float(
                self.get_parameter("datum_settle").value):
            return
        if self.odom is None or not self.datum_client.service_is_ready():
            return

        # x, y and orientation come from the filter's own estimate, so this
        # only ever moves z. The covariance is the filter's too, with the z
        # term tightened to the accuracy of the leg-kinematics height.
        request = SetPose.Request()
        request.pose.header.stamp = now.to_msg()
        request.pose.header.frame_id = self.odom.header.frame_id or "odom"
        request.pose.pose = self.odom.pose
        request.pose.pose.pose.position.z = float(h)
        cov = list(self.odom.pose.covariance)
        cov[14] = 1.0e-4                      # (1 cm)^2 on z
        request.pose.pose.covariance = cov
        self.datum_ref = h
        self.datum_future = self.datum_client.call_async(request)


def main():
    rclpy.init()
    node = LegOdometry()
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
