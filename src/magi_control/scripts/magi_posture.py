#!/usr/bin/env python3
"""Posture control for the MAGI Go2W legs.

Turns a desired body height into the 12 leg joint angles and sends them to
leg_controller as a smoothly interpolated trajectory.

    ros2 topic pub --once /magi/posture std_msgs/String "{data: stand}"
    ros2 topic pub --once /magi/body_height std_msgs/Float64 "{data: 0.30}"

Leg geometry (from the URDF joint tree and left_wheel.dae):

    hip --L1=0.213--> knee --L2=0.2264--> wheel centre, wheel radius 0.086

With the wheel held directly under the hip, the thigh angle `a` and calf angle
`c` that put the body at height h come from the two-link triangle:

    d      = h - r                                   (hip-to-wheel-centre reach)
    cos c  = (d^2 - L1^2 - L2^2) / (2 L1 L2)
    a      = atan2(-L2 sin c, L1 + L2 cos c)

`c` takes the negative (knee-back) branch, which is the sign convention the
calf joint limits [-2.7227, -0.83776] already enforce.
"""

import math

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

L1 = 0.213      # hip -> knee
L2 = 0.2264     # knee -> wheel centre
R_WHEEL = 0.086

# Joint limits that bound the reachable body height.
CALF_MIN, CALF_MAX = -2.7227, -0.83776

# Order must match leg_controller's `joints` list in magi_controllers.yaml.
LEG_JOINTS = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

NAMED_POSTURES = {
    "stand": 0.40,
    "tall": 0.46,
    "crouch": 0.28,
    "sit": 0.20,
}


def _reach_limits():
    """Body-height range reachable within the calf joint limits."""
    heights = []
    for c in (CALF_MIN, CALF_MAX):
        d2 = L1 * L1 + L2 * L2 + 2.0 * L1 * L2 * math.cos(c)
        heights.append(math.sqrt(max(d2, 0.0)) + R_WHEEL)
    return min(heights), max(heights)


HEIGHT_MIN, HEIGHT_MAX = _reach_limits()


def leg_ik(height):
    """Return (thigh, calf) angles placing the body at `height` metres.

    The height is clamped to what the calf joint limits actually allow.
    """
    height = min(max(height, HEIGHT_MIN), HEIGHT_MAX)
    d = height - R_WHEEL

    cos_c = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_c = min(max(cos_c, -1.0), 1.0)
    calf = -math.acos(cos_c)
    calf = min(max(calf, CALF_MIN), CALF_MAX)

    thigh = math.atan2(-L2 * math.sin(calf), L1 + L2 * math.cos(calf))
    return thigh, calf


class MagiPosture(Node):
    def __init__(self):
        super().__init__("magi_posture")

        self.declare_parameter("initial_height", NAMED_POSTURES["stand"])
        self.declare_parameter("transition_time", 2.0)
        self.declare_parameter("controller", "leg_controller")

        controller = self.get_parameter("controller").value
        self._transition = float(self.get_parameter("transition_time").value)

        self._pub = self.create_publisher(
            JointTrajectory, f"/{controller}/joint_trajectory", 10
        )
        self.create_subscription(String, "/magi/posture", self._on_posture, 10)
        self.create_subscription(Float64, "/magi/body_height", self._on_height, 10)

        self.get_logger().info(
            f"posture control ready; height range "
            f"{HEIGHT_MIN:.3f}-{HEIGHT_MAX:.3f} m, "
            f"postures: {', '.join(sorted(NAMED_POSTURES))}"
        )

        # Give the controller a moment to activate, then assume the initial pose.
        initial = float(self.get_parameter("initial_height").value)
        self._startup = self.create_timer(
            2.0, lambda: self._send_once(initial, self._startup)
        )

    def _send_once(self, height, timer):
        timer.cancel()
        self.send_height(height)

    def _on_posture(self, msg):
        name = msg.data.strip().lower()
        if name not in NAMED_POSTURES:
            self.get_logger().warn(
                f"unknown posture '{msg.data}'; known: {', '.join(sorted(NAMED_POSTURES))}"
            )
            return
        self.send_height(NAMED_POSTURES[name])

    def _on_height(self, msg):
        self.send_height(float(msg.data))

    def send_height(self, height):
        thigh, calf = leg_ik(height)

        point = JointTrajectoryPoint()
        point.positions = [0.0, thigh, calf] * 4
        point.velocities = [0.0] * len(LEG_JOINTS)
        point.time_from_start = Duration(
            sec=int(self._transition),
            nanosec=int((self._transition % 1.0) * 1e9),
        )

        traj = JointTrajectory()
        traj.joint_names = LEG_JOINTS
        traj.points = [point]
        self._pub.publish(traj)

        self.get_logger().info(
            f"height {height:.3f} m -> thigh {thigh:.3f} rad, calf {calf:.3f} rad"
        )


def main():
    rclpy.init()
    node = MagiPosture()
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
